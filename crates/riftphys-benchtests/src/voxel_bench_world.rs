use clap::Parser;
use glam::{Quat, UVec3, Vec3};
use rand::{rngs::StdRng, Rng, SeedableRng};
use std::ffi::{c_void, CString};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

// Voxels
use riftphys_vox::{Ray, RayHit, VoxelChunk};

use riftphys_vox::gpu_wgpu::{GpuRaycaster, HitGpu};

// World + models
use riftphys_world::{world};
use riftphys_core::{iso, quat_identity, vec3, BodyId, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::*;

use riftphys_gravity::GravitySpec;
use riftphys_viz::viz::DebugSettings;
use riftphys_aero::{CombinedAero, FlatPlateDrag, ISA, SimpleWing};
use riftphys_acceleration::{SimplePropulsion, ThrottleCurve};

use riftphys_riftnet::sys;
use riftphys_riftnet::wire::{pack_snapshot, NetEnt};

// -----------------------------------------------------------------------------
// CLI
// -----------------------------------------------------------------------------
#[derive(Parser, Debug)]
#[command(name = "voxel_bench_world")]
struct Args {
    // Voxel + ray bench
    #[arg(long, default_value_t = 256)]
    dim: u32,
    #[arg(long, default_value_t = 1.0)]
    voxel_size: f32,
    #[arg(long, default_value_t = 32768)]
    batch: usize,
    /// If --viz is not set, run this many ticks then exit with a summary.
    /// If --viz is set, we live-loop until Ctrl-C (this value is ignored).
    #[arg(long, default_value_t = 360)]
    iters: usize,
    #[arg(long, default_value_t = 0)]
    seed: u64,
    #[arg(long, default_value_t = 2.0)]
    tmax: f32,
    #[arg(long, default_value_t = false)]
    gpu: bool,

    // World
    #[arg(long, default_value_t = 120)]
    hz: u32,
    #[arg(long, default_value_t = 60)]
    print_every: u32,

    // Capsule params
    #[arg(long, default_value_t = 0.25)]
    cap_r: f32,
    #[arg(long, default_value_t = 0.50)]
    cap_hh: f32,
    #[arg(long, default_value_t = 80.0)]
    cap_mass: f32,
    #[arg(long, default_value_t = 5.0)]
    cap_start_y: f32,

    // Throttle gating
    #[arg(long, default_value_t = 0.0)]
    thr_grounded: f32,
    #[arg(long, default_value_t = 0.5)]
    thr_air: f32,

    // Viz / RiftNet
    /// Start RiftNet server and stream snapshots (run until Ctrl-C).
    #[arg(long, default_value_t = false)]
    viz: bool,
    #[arg(long, default_value_t = 49111)]
    port: u16,
    /// Number of rays per tick to visualize as skinny capsules (kept small).
    #[arg(long, default_value_t = 256)]
    viz_rays: usize,
    /// Optional wall-clock pacer (Hz). Example: --realtime-hz 60
    #[arg(long)]
    realtime_hz: Option<f32>,
}
pub fn paint_slab(
    ch: &mut VoxelChunk,
    z0: u32, z1: u32,
    mat: riftphys_materials::materials::MaterialId
){
    for z in z0..z1 {
        for y in 0..ch.dims.y {
            for x in 0..ch.dims.x {
                if ch.get(x,y,z) { ch.set_mat(x,y,z, mat); }
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Small utilities
// -----------------------------------------------------------------------------
fn percentile(mut xs: Vec<f64>, p: f64) -> f64 {
    xs.sort_by(|a, b| a.total_cmp(b));
    if xs.is_empty() {
        return 0.0;
    }
    let n = xs.len();
    let rank = ((n as f64 - 1.0) * p).round() as usize;
    xs[rank.min(n - 1)]
}

use riftphys_materials::materials::MaterialId as MID;

fn fill_chunk(ch: &mut VoxelChunk) -> usize {
    let c = Vec3::new(ch.dims.x as f32 * 0.5, ch.dims.y as f32 * 0.5, ch.dims.z as f32 * 0.5);
    let r0 = (ch.dims.x.min(ch.dims.y).min(ch.dims.z) as f32) * 0.30;
    let r1 = r0 * 0.55;
    let mut filled = 0usize;

    for z in 0..ch.dims.z { for y in 0..ch.dims.y { for x in 0..ch.dims.x {
        let p = Vec3::new(x as f32 + 0.5, y as f32 + 0.5, z as f32 + 0.5);
        let mut occ = (p - c).length() < r0; // rock sphere
        let bmin = c - Vec3::splat(r1);
        let bmax = c + Vec3::splat(r1);
        occ |= p.x > bmin.x && p.y > bmin.y && p.z > bmin.z &&
            p.x < bmax.x && p.y < bmax.y && p.z < bmax.z; // inner cube
        if occ {
            ch.set(x,y,z,true);
            // simple: lower half rock, upper shell ice, random mud seams
            let mat = if p.y < c.y - 3.0 { MID::Granite }
            else if (p - c).length() > r0 - 1.5 { MID::Ice }
            else if (x + y + z) % 23 == 0 { MID::Mud }
            else { MID::Sandstone };
            ch.set_mat(x,y,z, mat);
            filled += 1;
        }
    }}}
    filled
}


fn gen_rays(
    rng: &mut StdRng,
    dim: u32,
    voxel_size: f32,
    n: usize,
    tmax: f32,
) -> Vec<Ray> {
    let w = dim as f32 * voxel_size;
    let box_min = Vec3::new(-0.25 * w, -0.25 * w, -0.25 * w);
    let box_max = Vec3::new(1.25 * w, 1.25 * w, 1.25 * w);
    let mut rays = Vec::with_capacity(n);
    for _ in 0..n {
        let o = Vec3::new(
            rng.gen::<f32>().mul_add(box_max.x - box_min.x, box_min.x),
            rng.gen::<f32>().mul_add(box_max.y - box_min.y, box_min.y),
            rng.gen::<f32>().mul_add(box_max.z - box_min.z, box_min.z),
        );
        let d = Vec3::new(
            rng.gen::<f32>() * 2.0 - 1.0,
            rng.gen::<f32>() * 2.0 - 1.0,
            rng.gen::<f32>() * 2.0 - 1.0,
        )
            .normalize_or_zero();
        rays.push(Ray { o, d, tmax });
    }
    rays
}

// -----------------------------------------------------------------------------
// World with capsule + aero/prop (grounded hysteresis gating)
// -----------------------------------------------------------------------------
struct Scene {
    world: world::World,
    capsule_id: BodyId,
}

fn build_world(args: &Args) -> Scene {
    let mut w = world::WorldBuilder::new().with_capacity(512, 2048).build();
    w.set_epoch(1);
    w.set_rng_seed(0xC0FFEE);
    w.set_debug(DebugSettings {
        print_every: 0, // we print manually below
        show_bodies: false,
        show_contacts: false,
        show_impulses: false,
        show_energy: false,
        max_lines: 8,
        ..DebugSettings::default()
    });

    // Gravity (uniform)
    w.queue_gravity_swap(GravitySpec::Uniform {
        g: [0.0, -9.81, 0.0],
    });

    // Ground: wide flat box, top at y=0
    let mut mat_ground = material(MaterialId::Default);
    mat_ground.restitution = 0.0;
    mat_ground.mu_s = 1.2;
    mat_ground.mu_k = 1.0;

    let ground = w.add_body(
        iso(vec3(0.0, -0.25, 0.0), quat_identity()), // hy=0.25 ⇒ top at y=0
        Velocity::default(),
        MassProps::infinite(),
        false,
    );
    w.add_collider(
        ground,
        Shape::Box {
            hx: 1000.0,
            hy: 0.25,
            hz: 1000.0,
        },
        mat_ground,
    );

    // Capsule (dynamic)
    let mut mat_dyn = material(MaterialId::Default);
    mat_dyn.restitution = 0.0;
    mat_dyn.mu_s = 1.0;
    mat_dyn.mu_k = 0.9;

    let c_body = w.add_body(
        iso(vec3(0.0, args.cap_start_y, 0.0), quat_identity()),
        Velocity::default(),
        MassProps::from_capsule(args.cap_r, args.cap_hh, MaterialId::Default),
        true,
    );

    w.add_collider(
        c_body,
        Shape::Capsule {
            r: args.cap_r,
            hh: args.cap_hh,
        },
        mat_dyn,
    );

    // Models: modest drag + small wing + capped thrust
    let aero = CombinedAero::new(vec![
        Arc::new(FlatPlateDrag {
            cd: 1.0,
            area_m2: 0.25,
            isa: ISA::default(),
        }),
        Arc::new(SimpleWing {
            cl_per_rad: 5.5,
            stall_rad: 0.8,
            area_m2: 0.5,
            lift_dir_body: Vec3::Y.into(),
            isa: ISA::default(),
            cd0: 0.02,
            k_induced: 0.05,
        }),
    ]);
    let prop = SimplePropulsion {
        thrust_constant_n: 120.0,
        thrust_max_n: 800.0,
        curve: ThrottleCurve::QuadEaseIn,
    };
    let aero_h = w.models_mut().register_aero(Arc::new(aero));
    let prop_h = w.models_mut().register_prop(Arc::new(prop));
    w.set_body_accel(c_body, Some(aero_h), Some(prop_h), 0.5, args.thr_air, None);

    Scene {
        world: w,
        capsule_id: c_body,
    }
}

// -----------------------------------------------------------------------------
// RiftNet: event callback (optional logging hook)
// -----------------------------------------------------------------------------
unsafe extern "C" fn on_server_event(
    _ev: *const riftphys_riftnet::sys::RiftEvent,
    _user: *mut c_void,
) {
    // You can log connect/disconnect here if desired.
}

// -----------------------------------------------------------------------------
// Viewer glyph helpers (reuses your NetEnt path, no viewer changes)
// -----------------------------------------------------------------------------
#[inline]
fn orient_y_to(dir: Vec3) -> Quat {
    let d = dir.normalize_or_zero();
    if d.length_squared() < 1.0e-12 {
        return Quat::IDENTITY;
    }
    let up = Vec3::Y;
    let v = up.cross(d);
    let w = (1.0 + up.dot(d)).max(1.0e-8);
    Quat::from_xyzw(v.x, v.y, v.z, w).normalize()
}

fn encode_shape(world: &world::World, bid: BodyId) -> (f32, f32, f32, u32) {
    use riftphys_geom::Shape;
    match world.primary_shape(bid) {
        Some(Shape::Sphere { r }) => (r, r, r, 1),
        Some(Shape::Box { hx, hy, hz }) => (hx, hy, hz, 2),
        Some(Shape::Capsule { r, hh }) => {
            // viewer expects sy = (hh + r)/2 like your renderer
            (r, (hh + r) / 2.0, r, 3)
        }
        _ => (0.25, 0.25, 0.25, 1), // fallback sphere
    }
}

fn append_vox_debug_glyphs(
    ents: &mut Vec<NetEnt>,
    id_base: u32,
    chunk_min: Vec3,
    chunk_max: Vec3,
    rays: &[Ray],
    hits: &[RayHit],
    max_draw: usize,
) {
    // Chunk bounds: one big box centered at (min+max)/2 with half-extents (max-min)/2
    let c = (chunk_min + chunk_max) * 0.5;
    let hx = (chunk_max.x - chunk_min.x) * 0.5;
    let hy = (chunk_max.y - chunk_min.y) * 0.5;
    let hz = (chunk_max.z - chunk_min.z) * 0.5;
    ents.push(NetEnt {
        id: id_base,
        px: c.x,
        py: c.y,
        pz: c.z,
        qx: 0.0,
        qy: 0.0,
        qz: 0.0,
        qw: 1.0,
        sx: hx,
        sy: hy,
        sz: hz,
        kind: 2, // Box
    });

    // Rays, hits, normals (draw a small sample per tick)
    let n = rays.len().min(hits.len()).min(max_draw);
    let ray_r = 0.01_f32; // visual radius for rays
    let nrm_len = 0.25_f32;
    for i in 0..n {
        let r = rays[i];
        let h = hits[i];

        let d_unit = r.d.normalize_or_zero();
        let t_draw = if h.hit { h.t.min(r.tmax) } else { r.tmax };
        let p0 = r.o;
        let p1 = r.o + d_unit * t_draw;
        let mid = (p0 + p1) * 0.5;
        let dir = (p1 - p0);
        let len = dir.length();
        if len > 1.0e-6 {
            let q = orient_y_to(dir);
            // Capsule expects sy = half-height
            ents.push(NetEnt {
                id: id_base + 1 + (i as u32) * 3,
                px: mid.x,
                py: mid.y,
                pz: mid.z,
                qx: q.x,
                qy: q.y,
                qz: q.z,
                qw: q.w,
                sx: ray_r,
                sy: len * 0.5,
                sz: ray_r,
                kind: 3, // Capsule
            });
        }

        if h.hit {
            // Hit point: small sphere
            let hp = r.o + d_unit * h.t;
            ents.push(NetEnt {
                id: id_base + 1 + (i as u32) * 3 + 1,
                px: hp.x,
                py: hp.y,
                pz: hp.z,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                qw: 1.0,
                sx: 0.03,
                sy: 0.03,
                sz: 0.03,
                kind: 1, // Sphere
            });

            // Normal arrow: tiny capsule
            let qn = orient_y_to(h.normal);
            ents.push(NetEnt {
                id: id_base + 1 + (i as u32) * 3 + 2,
                px: hp.x,
                py: hp.y,
                pz: hp.z,
                qx: qn.x,
                qy: qn.y,
                qz: qn.z,
                qw: qn.w,
                sx: 0.01,
                sy: nrm_len * 0.5,
                sz: 0.01,
                kind: 3, // Capsule
            });
        }
    }
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
fn main() {
    let args = Args::parse();
    let dt = 1.0 / args.hz.max(1) as f32;

    // ---- Build voxel chunk
    let mut chunk = VoxelChunk::new(UVec3::splat(args.dim));
    chunk.voxel_size = args.voxel_size;
    let filled = fill_chunk(&mut chunk);
    let total = (args.dim as u64 * args.dim as u64 * args.dim as u64) as f64;
    let fill_pct = (filled as f64 / total) * 100.0;

    // ---- Build world with model system + capsule
    let Scene {
        mut world,
        capsule_id,
    } = build_world(&args);

    // ---- GPU raycaster (optional)
    let gpu_rc = if args.gpu {
        println!("[GPU] Initializing GpuRaycaster…");
        Some(riftphys_vox::gpu_wgpu::GpuRaycaster::new())
    } else {
        None
    };

    // ---- Intro print
    println!("== Voxel World Bench ==");
    println!(
        "Chunk: {}^3  voxel={:.3}  filled: {} ({:.2}%)",
        args.dim, args.voxel_size, filled, fill_pct
    );
    println!(
        "Rays: batch={}  iters={}  gpu={}",
        args.batch, args.iters, args.gpu
    );
    println!(
        "World: dt={:.6}s (hz={})  ticks={}",
        dt,
        args.hz,
        if args.viz { usize::MAX } else { args.iters }
    );
    println!(
        "Capsule: r={:.3} hh={:.3} mass={:.1} start_y={:.2}",
        args.cap_r, args.cap_hh, args.cap_mass, args.cap_start_y
    );

    // ---- Ctrl-C handling
    let running = Arc::new(AtomicBool::new(true));
    {
        let r = running.clone();
        ctrlc::set_handler(move || {
            r.store(false, Ordering::SeqCst);
        })
            .expect("ctrlc");
    }

    // ---- Optional real-time pacer
    let mut pacer = args
        .realtime_hz
        .map(|hz| (Instant::now(), Duration::from_secs_f32(1.0 / hz)));

    // ---- RiftNet server (optional)
    let mut host_c_opt: Option<CString> = None;
    let server_opt = if args.viz {
        let host = CString::new("0.0.0.0").unwrap();
        let cfg = sys::RiftServerConfig {
            host_address: host.as_ptr(),
            port: args.port,
            event_callback: Some(on_server_event),
            user_data: std::ptr::null_mut(),
        };
        let server = unsafe { sys::rift_server_create(&cfg) };
        assert!(!server.is_null(), "rift_server_create failed");
        let start_res = unsafe { sys::rift_server_start(server) };
        assert!(
            matches!(start_res, sys::RiftResult::RIFT_SUCCESS),
            "rift_server_start failed"
        );
        host_c_opt = Some(host); // keep CString alive
        Some(server)
    } else {
        None
    };

    // ---- Stats accumulators
    let mut rng = StdRng::seed_from_u64(args.seed);
    let mut cpu_lat_us: Vec<f64> = Vec::new();
    let mut gpu_lat_us: Vec<f64> = Vec::new();
    let mut cpu_rays: u64 = 0;
    let mut cpu_secs: f64 = 0.0;
    let mut gpu_rays: u64 = 0;
    let mut gpu_secs: f64 = 0.0;

    let mut total_fp = 0usize;
    let mut total_fn = 0usize;
    let mut delta_t_ms: Vec<f64> = Vec::new();
    let mut delta_deg: Vec<f64> = Vec::new();

    // Grounded hysteresis on capsule
    struct GroundState {
        grounded: bool,
        on_ticks: u32,
        off_ticks: u32,
    }
    let mut gs = GroundState {
        grounded: false,
        on_ticks: 0,
        off_ticks: 0,
    };
    const NF_ON: f32 = 20.0;
    const NF_OFF: f32 = 5.0;
    const ON_DEBOUNCE_T: u32 = 3;
    const OFF_DEBOUNCE_T: u32 = 6;

    // ---- Live loop (until Ctrl-C if --viz, else run `iters`)
    let mut tick: u64 = 0;
    let live = args.viz;
    while running.load(Ordering::SeqCst) && (!live && (tick as usize) < args.iters || live) {
        // === RAYCAST (CPU)
        let rays = gen_rays(&mut rng, args.dim, args.voxel_size, args.batch, args.tmax);
        let mut cpu_hits = vec![RayHit::default(); args.batch];
        let t0 = Instant::now();
        riftphys_vox::cpu::raycast_many(&chunk, &rays, &mut cpu_hits);
        let dt_cpu = t0.elapsed().as_secs_f64();
        let per_us_cpu = (dt_cpu * 1e6) / args.batch as f64;
        cpu_lat_us.push(per_us_cpu);
        cpu_rays += args.batch as u64;
        cpu_secs += dt_cpu;

        // === RAYCAST (GPU)
        let (per_us_gpu, gpu_hits_opt) = if let (true, Some(g)) = (args.gpu, &gpu_rc) {
            let (ms, gpu_hits) = g.raycast_batch(&chunk, &rays);
            let per = (ms * 1e3) / args.batch as f64; // ms -> µs
            gpu_lat_us.push(per);
            gpu_rays += args.batch as u64;
            gpu_secs += ms as f64 * 1e-3;
            (per, Some(gpu_hits))
        } else {
            (0.0, None)
        };


        // === GPU vs CPU error checks
        if args.gpu {
            if let Some(gpu_hits) = gpu_hits_opt {
                for i in 0..args.batch {
                    let ch = &cpu_hits[i];
                    let gh = &gpu_hits[i];
                    let ghit = gh.hit != 0;
                    if ch.hit && !ghit {
                        total_fn += 1;
                    }
                    if !ch.hit && ghit {
                        total_fp += 1;
                    }
                    if ch.hit && ghit {
                        let dt_abs_ms = (ch.t - gh.t).abs() as f64;
                        let nc = ch.normal.normalize_or_zero();
                        let ng = Vec3::new(gh.nx, gh.ny, gh.nz).normalize_or_zero();
                        let dot = (nc.dot(ng)).clamp(-1.0, 1.0);
                        let ddeg = dot.acos().to_degrees() as f64;
                        delta_t_ms.push(dt_abs_ms);
                        delta_deg.push(ddeg);
                    }
                }
            }
        }

        // === WORLD STEP + CAPSULE COLLISION DIAG
        let nf = world.normal_force(capsule_id, dt);
        // hysteresis
        if gs.grounded {
            if nf <= NF_OFF {
                gs.off_ticks += 1;
            } else {
                gs.off_ticks = 0;
            }
            if gs.off_ticks >= OFF_DEBOUNCE_T {
                gs.grounded = false;
                gs.off_ticks = 0;
            }
        } else {
            if nf >= NF_ON {
                gs.on_ticks += 1;
            } else {
                gs.on_ticks = 0;
            }
            if gs.on_ticks >= ON_DEBOUNCE_T {
                gs.grounded = true;
                gs.on_ticks = 0;
            }
        }
        // throttle gating
        world.set_body_throttle(
            capsule_id,
            if gs.grounded {
                args.thr_grounded
            } else {
                args.thr_air
            },
        );

        let stats = world.step(dt);

            // === PRINT cadence
        if args.print_every != 0 && (tick as u32) % args.print_every == 0 {
            let p = world.get_body_pose(capsule_id).pos;
            let v = world.get_body_vel(capsule_id).lin;

            // CPU throughput for this tick window
            let mrays_cpu = if dt_cpu > 0.0 {
                (args.batch as f64 / dt_cpu) / 1.0e6
            } else { 0.0 };

            // Build the optional GPU tail string once
            let gpu_tail = if args.gpu {
                let mrays_gpu = if per_us_gpu > 0.0 { 1.0 / per_us_gpu } else { 0.0 };
                format!("  GPU {:6.3}µs/r ({:6.2} MRays/s)", per_us_gpu, mrays_gpu)
            } else {
                String::new()
            };


            println!(
                "[tick {:5}] capsule: y={:+.3} vy={:+.3} NF={:6.1} grounded={} contacts={:3} \
         | rays: CPU {:6.3}µs/r ({:6.2} MRays/s){}",
                tick, p.y, v.y, nf, gs.grounded, stats.contacts, per_us_cpu, mrays_cpu, gpu_tail
            );
        }


        // === RiftNet snapshot (optional, only if --viz)
        if let Some(server) = server_opt {
            let mut ents: Vec<NetEnt> = Vec::with_capacity(world.num_bodies() as usize + 1_024);

            // world bodies
            for id in 0..(world.num_bodies() as u32) {
                let bid = BodyId(id);
                let pose = world.get_body_pose(bid);
                let (sx, sy, sz, kind) = encode_shape(&world, bid);
                ents.push(NetEnt {
                    id,
                    px: pose.pos.x,
                    py: pose.pos.y,
                    pz: pose.pos.z,
                    qx: pose.rot.x,
                    qy: pose.rot.y,
                    qz: pose.rot.z,
                    qw: pose.rot.w,
                    sx,
                    sy,
                    sz,
                    kind,
                });
            }

            // voxel/raycast debug overlays
            let mn = chunk.aabb_min();
            let mx = chunk.aabb_max();
            append_vox_debug_glyphs(
                &mut ents,
                1_000_000, // debug glyph ID base, away from body IDs
                mn,
                mx,
                &rays,
                &cpu_hits,
                args.viz_rays,
            );

            // broadcast
            let epoch_u64 = world.epoch_id as u64;
            for payload in pack_snapshot(tick, epoch_u64, &ents) {
                unsafe {
                    let _ = sys::rift_server_broadcast(
                        server,
                        payload.as_ptr(),
                        payload.len() as sys::size_t,
                    );
                }
            }
        }

        // real-time pacer
        if let Some((ref mut last, frame)) = pacer {
            let now = Instant::now();
            let next = *last + frame;
            if now < next {
                std::thread::sleep(next - now);
            }
            *last = Instant::now();
        }

        tick = tick.wrapping_add(1);
    }

    // === Final summaries (finite run, or when Ctrl-C)
    println!();
    println!("== Summary ==");
    if !cpu_lat_us.is_empty() {
        println!(
            "CPU per-ray µs: p50={:.3}  p95={:.3}  p99={:.3}",
            percentile(cpu_lat_us.clone(), 0.50),
            percentile(cpu_lat_us.clone(), 0.95),
            percentile(cpu_lat_us.clone(), 0.99)
        );
        let cpu_mrays_s = if cpu_secs > 0.0 {
            (cpu_rays as f64 / cpu_secs) / 1.0e6
        } else {
            0.0
        };
        println!("CPU throughput: {:.2} MRays/s", cpu_mrays_s);
    }

    if args.gpu && !gpu_lat_us.is_empty() {
        println!(
            "GPU per-ray µs: p50={:.3}  p95={:.3}  p99={:.3}",
            percentile(gpu_lat_us.clone(), 0.50),
            percentile(gpu_lat_us.clone(), 0.95),
            percentile(gpu_lat_us.clone(), 0.99)
        );
        let gpu_mrays_s = if gpu_secs > 0.0 {
            (gpu_rays as f64 / gpu_secs) / 1.0e6
        } else {
            0.0
        };
        println!("GPU throughput: {:.2} MRays/s", gpu_mrays_s);

        let n = (tick as f64 * args.batch as f64).max(1.0);
        let fp = (total_fp as f64 / n) * 100.0;
        let fn_ = (total_fn as f64 / n) * 100.0;
        println!("GPU error vs CPU: FP={:.4}%  FN={:.4}%", fp, fn_);
        if !delta_t_ms.is_empty() {
            println!(
                "Δt (hit∧hit) ms: mean={:.6}  p95={:.6}  p99={:.6}",
                delta_t_ms.iter().sum::<f64>() / delta_t_ms.len() as f64,
                percentile(delta_t_ms.clone(), 0.95),
                percentile(delta_t_ms, 0.99)
            );
        }
        if !delta_deg.is_empty() {
            let mean = delta_deg.iter().sum::<f64>() / delta_deg.len() as f64;
            println!(
                "Δnormal deg:     mean={:.6}  p95={:.6}  p99={:.6}",
                mean,
                percentile(delta_deg.clone(), 0.95),
                percentile(delta_deg, 0.99)
            );
        }
    }
}
