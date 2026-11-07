// SPDX-License-Identifier: RiftForged Proprietary License
//! Diagnostic for `world_robotics_test::test_pvr_energy_conservation_pendulum`: where the
//! hinged pendulum's energy goes. Run with
//!     cargo test -p riftphys-world --release --test pendulum_energy_diag -- --ignored --nocapture
use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::MassProps;
use riftphys_world::world::{JointProjection, World};

const G: f32 = 9.81;
const L: f32 = 5.0;
const HALF: f32 = 1.0;
const DENSITY: f32 = 10.0;

struct Run { world: World, bob: BodyId, mass: f32, i_pivot: f32 }

fn build(iters: (u32, u32), proj: JointProjection) -> Run {
    let mut w = World::with_capacity(4, 4);
    w.set_joint_projection(proj);
    w.set_gravity(Vec3::new(0.0, -G, 0.0));
    w.set_solver_iterations(iters.0, iters.1);
    let pivot = w.add_body(Isometry { pos: Vec3::new(0.0, 20.0, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    let mp = MassProps::from_box_density(Vec3::splat(HALF), DENSITY);
    let mass = 8.0 * HALF * HALF * HALF * DENSITY;
    let i_cm = mp.inertia.z_axis.z;
    let bob = w.add_body(Isometry { pos: Vec3::new(L, 20.0, 0.0), rot: Quat::IDENTITY }, Velocity::default(), mp, true);
    w.add_hinge_joint(pivot, bob, Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY },
                      Isometry { pos: Vec3::new(-L, 0.0, 0.0), rot: Quat::IDENTITY }, 2);
    Run { world: w, bob, mass, i_pivot: i_cm + mass * L * L }
}

/// Swing energy measured from the bottom of the arc: m g (y - y_bottom) + KE (lin + ang).
fn swing_energy(r: &Run) -> f32 {
    let p = r.world.get_body_pose(r.bob);
    let v = r.world.get_body_vel(r.bob);
    let i_cm = r.i_pivot - r.mass * L * L;
    r.mass * G * (p.pos.y - (20.0 - L)) + 0.5 * r.mass * v.lin.length_squared() + 0.5 * i_cm * v.ang.length_squared()
}

/// Total energy as the ledger computes it (potential from y = 0).
fn ledger_energy(r: &Run) -> f32 {
    let p = r.world.get_body_pose(r.bob);
    let v = r.world.get_body_vel(r.bob);
    let i_cm = r.i_pivot - r.mass * L * L;
    r.mass * G * p.pos.y + 0.5 * r.mass * v.lin.length_squared() + 0.5 * i_cm * v.ang.length_squared()
}

/// Reference: the physical pendulum integrated with RK4 at a fine step. Returns angle from the
/// downward vertical (positive toward +x) at time t.
fn reference_angle(r: &Run, t: f32) -> f32 {
    let k = r.mass * G * L / r.i_pivot;
    let (mut th, mut om) = (std::f64::consts::FRAC_PI_2, 0.0f64);
    let h = 1e-5f64;
    let n = (t as f64 / h).round() as usize;
    let f = |th: f64| -(k as f64) * th.sin();
    for _ in 0..n {
        let (k1t, k1o) = (om, f(th));
        let (k2t, k2o) = (om + 0.5 * h * k1o, f(th + 0.5 * h * k1t));
        let (k3t, k3o) = (om + 0.5 * h * k2o, f(th + 0.5 * h * k2t));
        let (k4t, k4o) = (om + h * k3o, f(th + h * k3t));
        th += h / 6.0 * (k1t + 2.0 * k2t + 2.0 * k3t + k4t);
        om += h / 6.0 * (k1o + 2.0 * k2o + 2.0 * k3o + k4o);
    }
    th as f32
}

fn angle(r: &Run) -> f32 {
    let p = r.world.get_body_pose(r.bob).pos;
    (p.x).atan2(20.0 - p.y)
}

/// Runs `secs` at `hz`, mirroring the test (energy reference taken after the first step).
/// Returns (ledger retention, swing retention, worst anchor error, final angle, joint-drift work per tick).
fn run(hz: f32, secs: f32, iters: (u32, u32), trace: bool, proj: JointProjection) -> (f32, f32, f32, f32) {
    let mut r = build(iters, proj);
    let dt = 1.0 / hz;
    r.world.step(dt);
    let (e0_ledger, e0_swing) = (ledger_energy(&r), swing_energy(&r));
    let n = (secs * hz).round() as usize;
    let mut worst = 0.0f32;
    let mut prev = e0_swing;
    for k in 0..n {
        r.world.step(dt);
        let p = r.world.get_body_pose(r.bob).pos;
        worst = worst.max((p.distance(Vec3::new(0.0, 20.0, 0.0)) - L).abs());
        let e = swing_energy(&r);
        if trace && (k % ((hz / 20.0) as usize).max(1) == 0) {
            let v = r.world.get_body_vel(r.bob);
            let radial = v.lin.dot((p - Vec3::new(0.0, 20.0, 0.0)).normalize());
            println!("  t {:5.2}  angle {:7.2} deg  E_swing {:8.1} J  dE/tick {:7.2} J  |v| {:6.3}  v_radial {:8.5}  anchor err {:.5}",
                     (k + 2) as f32 * dt, angle(&r).to_degrees(), e, e - prev, v.lin.length(), radial,
                     (p.distance(Vec3::new(0.0, 20.0, 0.0)) - L).abs());
        }
        prev = e;
    }
    let t_end = (n + 1) as f32 * dt;
    let ang_err = angle(&r) - reference_angle(&r, t_end);
    (ledger_energy(&r) / e0_ledger, swing_energy(&r) / e0_swing, worst, ang_err.to_degrees())
}

#[test]
#[ignore]
fn pendulum_energy_diagnostic() {
    for proj in [JointProjection::Pbd, JointProjection::Shake] {
        println!("
########## {:?}", proj);
        energy_diagnostic(proj);
    }
}

fn energy_diagnostic(proj: JointProjection) {
    let r = build((20, 20), proj);
    let k = r.mass * G * L / r.i_pivot;
    println!("mass {:.1} kg, I_pivot {:.1} kg m^2, small-angle period {:.3} s, swing energy at release m g L = {:.1} J",
             r.mass, r.i_pivot, 2.0 * std::f32::consts::PI / k.sqrt(), r.mass * G * L);

    println!("\n== The test's run (62.5 Hz, 5 s, iters 20/20), trace every ~1/20 s");
    let res = run(62.5, 4.8, (20, 20), std::env::var("TRACE").is_ok(), proj);
    println!("ledger retention {:.3}%  swing retention {:.2}%  worst anchor err {:.5} m  angle vs reference {:+.2} deg",
             res.0 * 100.0, res.1 * 100.0, res.2, res.3);

    println!("\n== Step rate (5 s, iters 20/20)");
    for hz in [62.5f32, 120.0, 240.0, 480.0, 960.0] {
        let res = run(hz, 4.8, (20, 20), false, proj);
        println!("  {:6.1} Hz  ledger {:7.3}%  swing {:6.2}%  anchor {:.5} m  angle err {:+6.2} deg", hz, res.0 * 100.0, res.1 * 100.0, res.2, res.3);
    }

    println!("\n== Iterations (62.5 Hz, 5 s)");
    for it in [(1u32, 1u32), (5, 5), (20, 1), (1, 20), (20, 20), (50, 50)] {
        let res = run(62.5, 4.8, it, false, proj);
        println!("  vel {:2} pos {:2}  ledger {:7.3}%  swing {:6.2}%  anchor {:.5} m  angle err {:+6.2} deg", it.0, it.1, res.0 * 100.0, res.1 * 100.0, res.2, res.3);
    }
}

/// Point-mass pendulum (length `l`, released horizontal) with the engine's update scheme and
/// variants; returns swing-energy retention after `secs`. f64, so precision is not a factor.
///   0: generalized coordinates, symplectic Euler (reference)
///   1: engine: gravity; velocity projected onto the tangent at x_n; drift; position projected
///      along the gradient at the drifted point; velocity += correction / dt
///   2: as 1, but the position correction is along the start-of-step gradient (SHAKE)
///   3: as 1 without the velocity projection (pure PBD)
fn point_pendulum(scheme: u32, hz: f64, secs: f64, l: f64) -> f64 {
    let (g, dt) = (9.81f64, 1.0 / hz);
    let n = (secs * hz).round() as usize;
    let energy = |x: [f64; 2], v: [f64; 2]| g * (x[1] + l) + 0.5 * (v[0] * v[0] + v[1] * v[1]);
    let (mut x, mut v) = ([l, 0.0f64], [0.0f64, 0.0]);
    let (mut th, mut om) = (std::f64::consts::FRAC_PI_2, 0.0f64);
    let mut e0 = None;
    for _ in 0..=n {
        if scheme == 0 {
            om += -(g / l) * th.sin() * dt;
            th += om * dt;
            x = [l * th.sin(), -l * th.cos()];
            v = [l * om * th.cos(), l * om * th.sin()];
        } else {
            v[1] -= g * dt;
            if scheme != 3 {
                let r = [x[0] / l, x[1] / l];
                let vr = v[0] * r[0] + v[1] * r[1];
                v = [v[0] - vr * r[0], v[1] - vr * r[1]];
            }
            let xt = [x[0] + v[0] * dt, x[1] + v[1] * dt];
            let xn = if scheme == 2 {
                // along r_n: |xt + s r_n| = l
                let r = [x[0] / l, x[1] / l];
                let b = xt[0] * r[0] + xt[1] * r[1];
                let c = xt[0] * xt[0] + xt[1] * xt[1] - l * l;
                let s = -b + (b * b - c).sqrt();
                [xt[0] + s * r[0], xt[1] + s * r[1]]
            } else {
                let len = (xt[0] * xt[0] + xt[1] * xt[1]).sqrt();
                [xt[0] * l / len, xt[1] * l / len]
            };
            v = [v[0] + (xn[0] - xt[0]) / dt, v[1] + (xn[1] - xt[1]) / dt];
            x = xn;
        }
        if e0.is_none() { e0 = Some(energy(x, v)); }
    }
    energy(x, v) / e0.unwrap()
}

#[test]
#[ignore]
fn pendulum_scheme_diagnostic() {
    // Equivalent simple-pendulum length of the test's cube: I_pivot / (m L)
    let l = 2053.3 / (80.0 * 5.0);
    println!("point pendulum, l = {:.3} m, released horizontal; swing retention after 4.8 s", l);
    for hz in [62.5f64, 120.0, 240.0, 480.0, 960.0] {
        print!("  {:6.1} Hz", hz);
        for (s, name) in [(0u32, "generalized"), (1, "engine"), (2, "SHAKE dir"), (3, "pure PBD")] {
            print!("  {} {:7.2}%", name, point_pendulum(s, hz, 4.8, l) * 100.0);
        }
        println!();
    }
    // Same at the turning points only (where the symplectic Euler phase offset vanishes):
    println!("over 20 s:");
    for hz in [62.5f64, 240.0] {
        print!("  {:6.1} Hz", hz);
        for (s, name) in [(0u32, "generalized"), (1, "engine"), (2, "SHAKE dir"), (3, "pure PBD")] {
            print!("  {} {:7.2}%", name, point_pendulum(s, hz, 20.0, l) * 100.0);
        }
        println!();
    }
}

/// f32 precision: the same engine pendulum hung at different heights.
#[test]
#[ignore]
fn pendulum_height_diagnostic() {
    for proj in [JointProjection::Pbd, JointProjection::Shake] {
    for hz in [62.5f32, 240.0, 480.0, 960.0] {
        for y in [0.0f32, 20.0, 200.0, 1000.0] {
            let mut w = World::with_capacity(4, 4);
            w.set_joint_projection(proj);
            w.set_gravity(Vec3::new(0.0, -G, 0.0));
            let pivot = w.add_body(Isometry { pos: Vec3::new(0.0, y, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
            let mp = MassProps::from_box_density(Vec3::splat(HALF), DENSITY);
            let mass = 80.0f32;
            let i_cm = mp.inertia.z_axis.z;
            let bob = w.add_body(Isometry { pos: Vec3::new(L, y, 0.0), rot: Quat::IDENTITY }, Velocity::default(), mp, true);
            w.add_hinge_joint(pivot, bob, Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY },
                              Isometry { pos: Vec3::new(-L, 0.0, 0.0), rot: Quat::IDENTITY }, 2);
            let e = |w: &World| { let p = w.get_body_pose(bob); let v = w.get_body_vel(bob);
                mass * G * (p.pos.y - (y - L)) + 0.5 * mass * v.lin.length_squared() + 0.5 * i_cm * v.ang.length_squared() };
            let dt = 1.0 / hz;
            w.step(dt);
            let e0 = e(&w);
            for _ in 0..(4.8 * hz) as usize { w.step(dt); }
            println!("  {:?} {:6.1} Hz  pivot y {:5.0}  swing retention {:7.2}%", proj, hz, y, e(&w) / e0 * 100.0);
        }
    }
    }
}

/// No gravity: the pendulum spinning at a steady rate. Kinetic energy lost per tick against the
/// chord prediction (fraction phi^2 / 3, phi = angle turned per tick).
#[test]
#[ignore]
fn pendulum_spin_diagnostic() {
    for proj in [JointProjection::Pbd, JointProjection::Shake] {
    println!("{:?}", proj);
    for v0 in [0.5f32, 1.0, 2.0, 3.0, 5.0, 9.6] {
        // The bob moving tangentially (-y at +x) with the matching rate about z
        let mut w = World::with_capacity(4, 4);
        w.set_joint_projection(proj);
        w.set_gravity(Vec3::ZERO);
        let pivot = w.add_body(Isometry { pos: Vec3::new(0.0, 20.0, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
        let mp = MassProps::from_box_density(Vec3::splat(HALF), DENSITY);
        let i_cm = mp.inertia.z_axis.z;
        let b = w.add_body(Isometry { pos: Vec3::new(L, 20.0, 0.0), rot: Quat::IDENTITY },
                           Velocity { lin: Vec3::new(0.0, -v0, 0.0), ang: Vec3::new(0.0, 0.0, -v0 / L) }, mp, true);
        w.add_hinge_joint(pivot, b, Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY },
                          Isometry { pos: Vec3::new(-L, 0.0, 0.0), rot: Quat::IDENTITY }, 2);
        let ke = |w: &World| { let v = w.get_body_vel(b); 0.5 * 80.0 * v.lin.length_squared() + 0.5 * i_cm * v.ang.length_squared() };
        let dt = 0.016f32;
        w.step(dt);
        let k0 = ke(&w);
        let n = 100;
        for _ in 0..n { w.step(dt); }
        let per_tick = 1.0 - (ke(&w) / k0).powf(1.0 / n as f32);
        let phi = v0 * dt / L;
        println!("  v {:4.1} m/s  KE lost per tick {:.3e}  chord prediction phi^2/3 {:.3e}", v0, per_tick, phi * phi / 3.0);
    }
    }
}

#[test]
#[ignore]
fn limit_and_double_trace() {
    use riftphys_articulation::bodies::D6Limit;
    for proj in [JointProjection::Pbd, JointProjection::Shake] {
        println!("##### {:?} hinge into limit min -1.8", proj);
        let mut r = build((20, 20), proj);
        // the hinge is joint 0
        r.world.set_d6_limit_r(riftphys_core::JointId(0), 2, Some(D6Limit { min: -1.8, max: 0.1 }));
        let dt = 1.0 / 62.5;
        r.world.step(dt);
        let e0 = swing_energy(&r);
        for k in 0..160 {
            r.world.step(dt);
            let p = r.world.get_body_pose(r.bob);
            let (ax, an) = p.rot.to_axis_angle();
            let ang = an * ax.z.signum();
            if k % 4 == 0 || (ang < -1.9 && ang > -2.6) {
                let v = r.world.get_body_vel(r.bob);
                println!("  t {:5.2} rot {:7.1} deg  meas {:6.3}  E {:6.3}  w {:7.3}  anchor err {:.4}", (k + 2) as f32 * dt, ang.to_degrees(),
                         2.0 * (ang / 2.0).sin(), swing_energy(&r) / e0, v.ang.z, (p.pos.distance(Vec3::new(0.0, 20.0, 0.0)) - L).abs());
            }
        }
    }
}

/// Rods hinged at their ends (robot-limb proportions): 1 m x 0.05 m, 1000 kg/m^3.
fn rod(w: &mut World, centre: Vec3) -> (BodyId, f32, f32) {
    let half = Vec3::new(0.5, 0.025, 0.025);
    let mp = MassProps::from_box_density(half, 1000.0);
    let m = 8.0 * half.x * half.y * half.z * 1000.0;
    let i = mp.inertia.z_axis.z;
    (w.add_body(Isometry { pos: centre, rot: Quat::IDENTITY }, Velocity::default(), mp, true), m, i)
}

#[test]
#[ignore]
fn rod_scenarios() {
    use riftphys_articulation::bodies::D6Limit;
    let e = |w: &World, b: BodyId, m: f32, i: f32| { let p = w.get_body_pose(b); let v = w.get_body_vel(b);
        m * G * p.pos.y + 0.5 * m * v.lin.length_squared() + 0.5 * i * v.ang.length_squared() };
    for proj in [JointProjection::Pbd, JointProjection::Shake] {
        for hz in [62.5f32, 240.0] {
            let dt = 1.0 / hz;
            // Double pendulum of rods, released horizontal
            let mut w = World::with_capacity(8, 8);
            w.set_gravity(Vec3::new(0.0, -G, 0.0));
            w.set_joint_projection(proj);
            let p = w.add_body(Isometry { pos: Vec3::new(0.0, 10.0, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
            let (a, ma, ia) = rod(&mut w, Vec3::new(0.5, 10.0, 0.0));
            let (b, mb, ib) = rod(&mut w, Vec3::new(1.5, 10.0, 0.0));
            w.add_hinge_joint(p, a, Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY }, Isometry { pos: Vec3::new(-0.5, 0.0, 0.0), rot: Quat::IDENTITY }, 2);
            w.add_hinge_joint(a, b, Isometry { pos: Vec3::new(0.5, 0.0, 0.0), rot: Quat::IDENTITY }, Isometry { pos: Vec3::new(-0.5, 0.0, 0.0), rot: Quat::IDENTITY }, 2);
            let floor = ma * G * 9.5 + mb * G * 8.5;
            let tot = |w: &World| e(w, a, ma, ia) + e(w, b, mb, ib) - floor;
            w.step(dt);
            let e0 = tot(&w);
            let (mut lo, mut hi, mut sep) = (1.0f32, 1.0f32, 0.0f32);
            for _ in 0..(10.0 * hz) as usize {
                w.step(dt);
                let r = tot(&w) / e0; lo = lo.min(r); hi = hi.max(r);
                let (pa, pb) = (w.get_body_pose(a), w.get_body_pose(b));
                sep = sep.max(((pa.pos + pa.rot * Vec3::new(0.5, 0.0, 0.0)) - (pb.pos + pb.rot * Vec3::new(-0.5, 0.0, 0.0))).length());
            }
            println!("{:?} {:5.1} Hz double rod pendulum 10 s: energy {:.4}..{:.4}  worst separation {:.4} m", proj, hz, lo, hi, sep);

            // Single rod into a limit past the bottom (hinge measure 2 sin(angle/2) = -1.8)
            let mut w = World::with_capacity(8, 8);
            w.set_gravity(Vec3::new(0.0, -G, 0.0));
            w.set_joint_projection(proj);
            let p = w.add_body(Isometry { pos: Vec3::new(0.0, 10.0, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
            let (a, ma, ia) = rod(&mut w, Vec3::new(0.5, 10.0, 0.0));
            let j = w.add_hinge_joint(p, a, Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY }, Isometry { pos: Vec3::new(-0.5, 0.0, 0.0), rot: Quat::IDENTITY }, 2);
            w.set_d6_limit_r(j, 2, Some(D6Limit { min: -1.8, max: 0.1 }));
            let tot = |w: &World| e(w, a, ma, ia) - ma * G * 9.5;
            w.step(dt);
            let e0 = tot(&w);
            let (mut hi, mut sep) = (1.0f32, 0.0f32);
            for _ in 0..(5.0 * hz) as usize {
                w.step(dt);
                hi = hi.max(tot(&w) / e0);
                let pa = w.get_body_pose(a);
                sep = sep.max(((pa.pos + pa.rot * Vec3::new(-0.5, 0.0, 0.0)) - Vec3::new(0.0, 10.0, 0.0)).length());
            }
            println!("{:?} {:5.1} Hz rod into limit: energy after 5 s {:.4}, peak {:.4}, worst separation {:.4} m", proj, hz, tot(&w) / e0, hi, sep);
        }
    }
}

#[test]
#[ignore]
fn spherical_trace() {
    use riftphys_core::Mat3;
    for proj in [JointProjection::Pbd, JointProjection::Shake] {
        for hz in [62.5f32, 240.0] {
            let mut w = World::with_capacity(8, 8);
            w.set_gravity(Vec3::new(0.0, -G, 0.0));
            w.set_joint_projection(proj);
            let pivot = Vec3::new(0.0, 10.0, 0.0);
            let p = w.add_body(Isometry { pos: pivot, rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
            let d = Vec3::new(60f32.to_radians().sin(), -60f32.to_radians().cos(), 0.0);
            let centre = pivot + d * 0.5;
            let spin = Vec3::new(0.0, 3.0, 0.0);
            let half = Vec3::new(0.5, 0.025, 0.025);
            let mp = MassProps::from_box_density(half, 1000.0);
            let m = 8.0 * half.x * half.y * half.z * 1000.0;
            let b = w.add_body(Isometry { pos: centre, rot: Quat::from_rotation_arc(Vec3::X, d) }, Velocity { lin: spin.cross(centre - pivot), ang: spin }, mp, true);
            w.add_ball_joint(p, b, Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY }, Isometry { pos: Vec3::new(-0.5, 0.0, 0.0), rot: Quat::IDENTITY });
            let e = |w: &World| { let ps = w.get_body_pose(b); let v = w.get_body_vel(b); let r = Mat3::from_quat(ps.rot);
                m * G * (ps.pos.y - 9.5) + 0.5 * m * v.lin.length_squared() + 0.5 * v.ang.dot((r * mp.inertia * r.transpose()) * v.ang) };
            let dt = 1.0 / hz;
            w.step(dt);
            let e0 = e(&w);
            let mut line = String::new();
            let (mut lo, mut hi) = (1.0f32, 1.0f32);
            for k in 0..(10.0 * hz) as usize {
                w.step(dt);
                let r = e(&w) / e0; lo = lo.min(r); hi = hi.max(r);
                if k % (hz as usize) == 0 { line += &format!(" {:.3}", r); }
            }
            println!("{:?} {:5.1} Hz: range {:.4}..{:.4}  per second:{}", proj, hz, lo, hi, line);
        }
    }
}

/// The fidelity report's large swing (1 m, 1 kg rod hinged at its end, released horizontal):
/// energy range over 10 s.
#[test]
#[ignore]
fn large_swing_range() {
    for proj in [JointProjection::Pbd, JointProjection::Shake] {
        for hz in [60.0f32, 240.0] {
            let mut w = World::with_capacity(4, 4);
            w.set_gravity(Vec3::new(0.0, -G, 0.0));
            w.set_joint_projection(proj);
            let h = Vec3::new(0.02, 0.5, 0.02);
            let pivot = Vec3::new(0.0, 3.0, 0.0);
            let a = w.add_body(Isometry { pos: pivot, rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
            let rot = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
            let mp = MassProps::from_box_density(h, 1.0 / (8.0 * h.x * h.y * h.z));
            let rod = w.add_body(Isometry { pos: pivot + rot * Vec3::new(0.0, -h.y, 0.0), rot }, Velocity::default(), mp, true);
            w.add_hinge_joint(a, rod, Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY }, Isometry { pos: Vec3::new(0.0, h.y, 0.0), rot: Quat::IDENTITY }, 0);
            let e = |w: &World| { let p = w.get_body_pose(rod); let v = w.get_body_vel(rod); let wb = p.rot.inverse() * v.ang;
                0.5 * v.lin.length_squared() + 0.5 * (wb * Vec3::new(mp.inertia.x_axis.x, mp.inertia.y_axis.y, mp.inertia.z_axis.z) * wb).dot(Vec3::ONE) + G * (p.pos.y - 2.5) };
            let e0 = e(&w);
            let (mut lo, mut hi) = (f32::MAX, f32::MIN);
            let n = (10.0 * hz) as usize;
            let mut last = 0.0;
            for _ in 0..n { w.step(1.0 / hz); let r = e(&w) / e0; lo = lo.min(r); hi = hi.max(r); last = r; }
            // second half only: no secular trend means the two halves span the same range
            println!("{:?} {:5.1} Hz: energy/initial over 10 s {:.4}..{:.4}, at 10 s {:.4}", proj, hz, lo, hi, last);
        }
    }
}
