// use std::time::Instant;
// // use glam::Vec3 as GVec3;
// use riftphys_world::*;
// use riftphys_core::{vec3, iso, quat_identity, Velocity, BodyId};
// use riftphys_materials::materials::*;
// use riftphys_geom::{Shape, MassProps};
// use riftphys_gravity::GravitySpec;
// use riftphys_viz::viz::DebugSettings;
//
// fn pct(mut xs: Vec<f32>, p: f32) -> f32 {
//     if xs.is_empty() { return 0.0; }
//     xs.sort_by(|a,b| a.partial_cmp(b).unwrap());
//     let k = ((xs.len() as f32 - 1.0) * p).round() as usize;
//     xs[k]
// }
//
// struct Perf {
//     name: &'static str,
//     mean: f32, p50: f32, p95: f32, p99: f32,
//     contacts: u64, errors: u64,
// }
//
// fn build_world_mega(seed: u64, print_every: u32, gspec: GravitySpec, dt: f32) -> (world::World, Vec<BodyId>) {
//     let n_spheres   = 500usize;
//     let n_capsules  = 250usize;
//     let n_boxes     = 250usize;
//     let n_walls     = 12usize;
//     let n_dynplanes = 8usize;
//
//     let cap_bodies = n_spheres + n_capsules + n_boxes + n_walls + n_dynplanes + 32;
//     let cap_cols   = cap_bodies;
//
//     let mut w = world::WorldBuilder::new()
//         .with_capacity(cap_bodies, cap_cols)
//         .build();
//
//     w.set_rng_seed(seed);
//     w.set_epoch(1);
//     w.set_debug(DebugSettings {
//         print_every,
//         show_bodies:   false,
//         show_contacts: false,
//         show_impulses: false,
//         show_energy:   false,
//         max_lines:     16,
//         json_every:    0,
//     });
//     w.queue_gravity_swap(gspec);
//
//     let mut ids: Vec<BodyId> = Vec::new();
//
//     // Ground (static) – grippy "grit" slab
//     let ground = w.add_body(
//         iso(vec3(0.0, 0.0, 0.0), quat_identity()),
//         Velocity::default(),
//         MassProps::infinite(),
//         false,
//     );
//     let gmat = material(MaterialId::Grit);
//     w.add_collider(ground, Shape::Box { hx: 400.0, hy: 0.5, hz: 400.0 }, gmat);
//
//     // Thin vertical walls (concrete)
//     for wi in 0..n_walls {
//         let x = -12.0 + wi as f32 * 2.0;
//         let wall = w.add_body(
//             iso(vec3(x, 2.0, 0.0), quat_identity()),
//             Velocity::default(),
//             MassProps::infinite(),
//             false,
//         );
//         let m = material(MaterialId::Concrete);
//         w.add_collider(wall, Shape::Box { hx: 0.05, hy: 3.0, hz: 25.0 }, m);
//     }
//
//     // Dynamic planes – big moving slabs (concrete)
//     for k in 0..n_dynplanes {
//         let z = -15.0 + (k as f32) * 4.0;
//         let b = w.add_body(
//             iso(vec3(0.0, 1.25, z), quat_identity()),
//             Velocity {
//                 lin: vec3((k as f32 * 0.2).sin() * 0.5, 0.0, 0.0),
//                 ang: vec3(0.0, 0.0, 0.0),
//             },
//             MassProps::from_box(vec3(10.0, 0.25, 5.0), MaterialId::Concrete),
//             true,
//         );
//         let m = material(MaterialId::Concrete);
//         w.add_collider(b, Shape::Box { hx: 10.0, hy: 0.25, hz: 5.0 }, m);
//         ids.push(b);
//     }
//
//     // Spheres – mostly Default, every 5th is Ice
//     for i in 0..n_spheres {
//         let row = i / 50;
//         let col = i % 50;
//         let x = -20.0 + (col as f32) * 0.8;
//         let z = -20.0 + (row as f32) * 0.8;
//         let y = 1.5 + ((i % 7) as f32) * 0.05;
//
//         let mat_id = if i % 5 == 0 { MaterialId::Ice } else { MaterialId::Default };
//         let b = w.add_body(
//             iso(vec3(x, y, z), quat_identity()),
//             Velocity {
//                 lin: vec3(
//                     6.0 + (i % 9) as f32 * 0.25,
//                     -0.4 * ((i % 5) as f32),
//                     0.3 * ((i % 3) as f32 - 1.0),
//                 ),
//                 ang: vec3(0.0, 0.0, 0.0),
//             },
//             MassProps::from_sphere(0.18, mat_id),
//             true,
//         );
//         let m = material(mat_id);
//         w.add_collider(b, Shape::Sphere { r: 0.18 }, m);
//         ids.push(b);
//     }
//
//     // Capsules – rubbery, high-friction movers
//     for j in 0..n_capsules {
//         let x = 5.0 + (j % 25) as f32 * 0.6;
//         let z = -10.0 + (j / 25) as f32 * 0.7;
//         let b = w.add_body(
//             iso(vec3(x, 2.0 + (j % 7) as f32 * 0.05, z), quat_identity()),
//             Velocity {
//                 lin: vec3(-1.8 - (j % 7) as f32 * 0.2, -0.3, 0.2),
//                 ang: vec3(0.0, 0.0, 0.0),
//             },
//             MassProps::from_capsule(0.15, 0.35, MaterialId::RubberSoft),
//             true,
//         );
//         let m = material(MaterialId::RubberSoft);
//         w.add_collider(b, Shape::Capsule { r: 0.15, hh: 0.35 }, m);
//         ids.push(b);
//     }
//
//     // Boxes – generic Default solid
//     for k in 0..n_boxes {
//         let x = -8.0 + (k % 25) as f32 * 0.7;
//         let z =  8.0 + (k / 25) as f32 * 0.7;
//         let b = w.add_body(
//             iso(vec3(x, 1.2 + (k % 5) as f32 * 0.05, z), quat_identity()),
//             Velocity {
//                 lin: vec3(0.6 * ((k % 7) as f32 - 3.0), -0.2, 0.3),
//                 ang: vec3(0.0, 0.0, 0.0),
//             },
//             MassProps::from_box(vec3(0.2, 0.2, 0.2), MaterialId::Default),
//             true,
//         );
//         let m = material(MaterialId::Default);
//         w.add_collider(b, Shape::Box { hx: 0.2, hy: 0.2, hz: 0.2 }, m);
//         ids.push(b);
//     }
//
//     // Warmup for broadphase buckets
//     for _ in 0..60 {
//         let _ = w.step(dt);
//     }
//
//     (w, ids)
// }
//
//
// fn run_one(name: &'static str, mut w: world::World, ticks: usize, dt: f32) -> Perf {
//     let mut step_ms: Vec<f32> = Vec::with_capacity(ticks);
//     let mut total_contacts: u64 = 0;
//     let mut errors: u64 = 0;
//
//     let probe_ids = [BodyId(0u32)]; // sanity probe on ground only
//
//     for _ in 0..ticks {
//         let t0 = Instant::now();
//         let stats = w.step(dt);
//         let ms = (t0.elapsed().as_secs_f64() * 1000.0) as f32;
//         step_ms.push(ms);
//         total_contacts += stats.contacts as u64;
//
//         for id in probe_ids {
//             let p = w.get_body_pose(id).pos;
//             if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) { errors += 1; }
//         }
//     }
//
//     let mean = step_ms.iter().copied().sum::<f32>() / step_ms.len() as f32;
//     let p50 = pct(step_ms.clone(), 0.50);
//     let p95 = pct(step_ms.clone(), 0.95);
//     let p99 = pct(step_ms.clone(), 0.99);
//     Perf { name, mean, p50, p95, p99, contacts: total_contacts, errors }
// }
//
// fn main() {
//     // global knobs
//     let ticks: usize = std::env::var("TICKS").ok().and_then(|s| s.parse().ok()).unwrap_or(600);
//     let dt: f32 = std::env::var("DT").ok().and_then(|s| s.parse().ok()).unwrap_or(1.0/120.0);
//
//     // four gravities (LayeredPlanet centers are far below origin to give “down”)
//     let earth = GravitySpec::LayeredPlanet { surface_g: 9.81,  radius: 6_371_000.0, center: [0.0, -6_371_000.0, 0.0], min_r: 1_000.0 };
//     let mars  = GravitySpec::LayeredPlanet { surface_g: 3.721, radius: 3_396_200.0, center: [0.0, -3_396_200.0, 0.0], min_r: 1_000.0 };
//     let moon  = GravitySpec::LayeredPlanet { surface_g: 1.62,  radius: 1_737_400.0, center: [0.0, -1_737_400.0, 0.0], min_r: 1_000.0 };
//     // Sun is **very** strong; we run it at a smaller dt for stability
//     let sun   = GravitySpec::LayeredPlanet { surface_g: 274.0, radius: 696_340_000.0, center: [0.0, -696_340_000.0, 0.0], min_r: 10_000.0 };
//
//     let (w_earth, _) = build_world_mega(0xE11, 0, earth, dt);
//     let (w_mars , _) = build_world_mega(0xE12, 0, mars , dt);
//     let (w_moon , _) = build_world_mega(0xE13, 0, moon , dt);
//     let dt_sun: f32 = dt.min(1.0_f32 / 240.0_f32); // clamp to faster (smaller) step for Sun
//     let (w_sun  , _) = build_world_mega(0xE14, 0, sun, dt_sun);
//
//     let perf_e = run_one("Earth", w_earth, ticks, dt);
//     let perf_m = run_one("Mars ", w_mars , ticks, dt);
//     let perf_o = run_one("Moon ", w_moon , ticks, dt);
//     let perf_s = run_one("Sun  ", w_sun  , ticks, dt_sun);
//
//     println!("=== PERF SUMMARY (ticks={}) ===", ticks);
//     for p in [perf_e, perf_m, perf_o, perf_s] {
//         println!(
//             "{}  mean={:.3}  p50={:.3}  p95={:.3}  p99={:.3}  contacts={}  errors={}",
//             p.name, p.mean, p.p50, p.p95, p.p99, p.contacts, p.errors
//         );
//     }
// }
