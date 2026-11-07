// SPDX-License-Identifier: RiftForged Proprietary License
//! Diagnostics for JOINT_CONDITIONING_SPEC: small bodies on long lever arms, and hinge position
//! drives that stop short of their target. Run with
//!     cargo test -p riftphys-world --release --test joint_conditioning_diag -- --ignored --nocapture
use riftphys_articulation::bodies::{D6Limit, Drive};
use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::MassProps;
use riftphys_world::world::{JointProjection, World};

const G: f32 = 9.81;

fn iso(pos: Vec3) -> Isometry { Isometry { pos, rot: Quat::IDENTITY } }

fn hinge_angle(w: &World, b: BodyId) -> f32 {
    let q = w.get_body_pose(b).rot;
    let q = if q.w < 0.0 { -q } else { q };
    2.0 * q.z.atan2(q.w)
}

/// A 1 m, 2.5 kg rod on a hinge about z at distance `lever` from its centre (0.5 = at its end,
/// 0 = through its centre), lying along +x, with a position drive to `target` radians.
fn driven_rod(proj: JointProjection, iters: (u32, u32), gravity: f32, lever: f32, drive: Drive) -> (World, BodyId) {
    let mut w = World::with_capacity(4, 4);
    w.set_joint_projection(proj);
    w.set_solver_iterations(iters.0, iters.1);
    w.set_gravity(Vec3::new(0.0, -gravity, 0.0));
    let p = w.add_body(iso(Vec3::new(0.0, 10.0, 0.0)), Velocity::default(), MassProps::infinite(), false);
    let r = w.add_body(iso(Vec3::new(lever, 10.0, 0.0)), Velocity::default(), MassProps::from_box_density(Vec3::new(0.5, 0.025, 0.025), 1000.0), true);
    let j = w.add_hinge_joint(p, r, iso(Vec3::ZERO), iso(Vec3::new(-lever, 0.0, 0.0)), 2);
    w.set_d6_drive_r(j, 2, Some(drive));
    (w, r)
}

#[test]
#[ignore]
fn drive_diagnostic() {
    let drive = Drive { target: 1.0, kp: 100.0, kd: 10.0, max_impulse: 1000.0, vel_mode: false };
    for proj in [JointProjection::Pbd, JointProjection::Shake] {
        for (gravity, lever) in [(0.0f32, 0.5f32), (9.81, 0.5), (0.0, 0.0), (9.81, 0.0)] {
            for pos_iters in [1u32, 4, 20, 21, 100] {
                let (mut w, r) = driven_rod(proj, (20, pos_iters), gravity, lever, drive);
                let mut line = String::new();
                let mut peak_w = 0.0f32;
                for k in 0..(5.0 * 240.0) as usize {
                    w.step(1.0 / 240.0);
                    peak_w = peak_w.max(w.get_body_vel(r).ang.length());
                    if [1usize, 24, 240, 1199].contains(&k) { line += &format!(" {:7.2}", hinge_angle(&w, r).to_degrees()); }
                }
                let anchor = w.get_body_pose(r);
                let sep = ((anchor.pos + anchor.rot * Vec3::new(-lever, 0.0, 0.0)) - Vec3::new(0.0, 10.0, 0.0)).length();
                println!("{:?} g {:4.1} lever {:.1} pos iters {:3}: angle (deg) at 1 tick / 0.1 s / 1 s / 5 s:{}  target {:.2}  peak |w| {:7.2}  anchor sep {:.4}",
                         proj, gravity, lever, pos_iters, line, drive.target.to_degrees(), peak_w, sep);
            }
        }
    }
}

/// The hinge position solve alone (no step): rod at 0, target 1 rad, no gravity. How far does
/// one `solve_joints` call with n iterations get?
#[test]
#[ignore]
fn drive_position_solve_only() {
    let drive = Drive { target: 1.0, kp: 100.0, kd: 10.0, max_impulse: 1000.0, vel_mode: false };
    for lever in [0.5f32, 0.0] {
        for n in [1u32, 2, 5, 20, 21, 100, 101, 1000, 1001] {
            let (mut w, r) = driven_rod(JointProjection::Pbd, (20, 20), 0.0, lever, drive);
            w.solve_joints(1.0 / 240.0, n);
            let p = w.get_body_pose(r);
            let sep = ((p.pos + p.rot * Vec3::new(-lever, 0.0, 0.0)) - Vec3::new(0.0, 10.0, 0.0)).length();
            println!("lever {:.1} iterations {:4}: angle {:7.2} deg (target 57.30)  anchor sep {:.5} m", lever, n, hinge_angle(&w, r).to_degrees(), sep);
        }
    }
}

/// Iteration by iteration (one solve_joints call per iteration, the step's multipliers carried
/// by calling Joints directly is not exposed, so each call restarts them; rigid rows have
/// alpha 0, so that changes nothing): angle and anchor separation, and the target sweep.
#[test]
#[ignore]
fn drive_iteration_trace() {
    for target in [0.05f32, 0.2, 0.5, 1.0, 2.0] {
        let drive = Drive { target, kp: 100.0, kd: 10.0, max_impulse: 1000.0, vel_mode: false };
        let (mut w, r) = driven_rod(JointProjection::Pbd, (20, 20), 0.0, 0.5, drive);
        let mut line = String::new();
        for k in 0..400 {
            w.solve_joints(1.0 / 240.0, 1);
            let p = w.get_body_pose(r);
            let sep = ((p.pos + p.rot * Vec3::new(-0.5, 0.0, 0.0)) - Vec3::new(0.0, 10.0, 0.0)).length();
            if k < 8 || k == 199 || k == 399 { line += &format!(" [{} {:.2}deg {:.3}m]", k + 1, hinge_angle(&w, r).to_degrees(), sep); }
        }
        println!("target {:5.2} rad ({:6.2} deg):{}", target, target.to_degrees(), line);
    }
}

/// Energy of a set of bodies: m g y + linear KE + rotational KE (world inertia).
fn energy(w: &World, bodies: &[(BodyId, MassProps)]) -> f32 {
    bodies.iter().map(|(b, mp)| {
        let p = w.get_body_pose(*b);
        let v = w.get_body_vel(*b);
        let r = riftphys_core::Mat3::from_quat(p.rot);
        let i = r * mp.inertia * r.transpose();
        mp.mass * G * p.pos.y + 0.5 * mp.mass * v.lin.length_squared() + 0.5 * v.ang.dot(i * v.ang)
    }).sum()
}

/// The three lever-arm cases of JOINT_ENERGY_SPEC 5.4, at the current head.
#[test]
#[ignore]
fn lever_arm_cases() {
    for proj in [JointProjection::Pbd, JointProjection::Shake] {
        for hz in [62.5f32, 240.0] {
            for pos_iters in [20u32, 100, 500] {
                let dt = 1.0 / hz;
                // (a) 2 m cube (80 kg) on a 5 m arm into a hinge limit at -1.8 rad
                {
                    let mut w = World::with_capacity(4, 4);
                    w.set_joint_projection(proj);
                    w.set_solver_iterations(20, pos_iters);
                    w.set_gravity(Vec3::new(0.0, -G, 0.0));
                    let p = w.add_body(iso(Vec3::new(0.0, 20.0, 0.0)), Velocity::default(), MassProps::infinite(), false);
                    let mp = MassProps::from_box_density(Vec3::splat(1.0), 10.0);
                    let b = w.add_body(iso(Vec3::new(5.0, 20.0, 0.0)), Velocity::default(), mp, true);
                    let j = w.add_hinge_joint(p, b, iso(Vec3::ZERO), iso(Vec3::new(-5.0, 0.0, 0.0)), 2);
                    w.set_d6_limit_r(j, 2, Some(D6Limit { min: -1.8, max: 0.1 }));
                    let floor = mp.mass * G * 15.0;
                    let e0 = energy(&w, &[(b, mp)]) - floor;
                    let mut peak = 0.0f32;
                    let mut sep = 0.0f32;
                    for _ in 0..(5.0 * hz) as usize {
                        w.step(dt);
                        peak = peak.max((energy(&w, &[(b, mp)]) - floor) / e0);
                        let q = w.get_body_pose(b);
                        sep = sep.max(((q.pos + q.rot * Vec3::new(-5.0, 0.0, 0.0)) - Vec3::new(0.0, 20.0, 0.0)).length());
                    }
                    let end = (energy(&w, &[(b, mp)]) - floor) / e0;
                    println!("{:?} {:5.1} Hz pos {:3} | cube on 5 m arm into limit: energy at 5 s {:6.3}, peak {:6.3}, worst sep {:.4} m, rests at {:7.2} deg (stop -103.13)",
                             proj, hz, pos_iters, end, peak, sep, hinge_angle(&w, b).to_degrees());
                }
                // (b) double pendulum of 0.2 m cubes on 1 m massless arms, released horizontal
                {
                    let mut w = World::with_capacity(8, 8);
                    w.set_joint_projection(proj);
                    w.set_solver_iterations(20, pos_iters);
                    w.set_gravity(Vec3::new(0.0, -G, 0.0));
                    let p = w.add_body(iso(Vec3::new(0.0, 10.0, 0.0)), Velocity::default(), MassProps::infinite(), false);
                    let mp = MassProps::from_box_density(Vec3::splat(0.1), 1000.0);
                    let a = w.add_body(iso(Vec3::new(1.0, 10.0, 0.0)), Velocity::default(), mp, true);
                    let b = w.add_body(iso(Vec3::new(2.0, 10.0, 0.0)), Velocity::default(), mp, true);
                    w.add_hinge_joint(p, a, iso(Vec3::ZERO), iso(Vec3::new(-1.0, 0.0, 0.0)), 2);
                    w.add_hinge_joint(a, b, iso(Vec3::ZERO), iso(Vec3::new(-1.0, 0.0, 0.0)), 2);
                    let floor = mp.mass * G * 9.0 + mp.mass * G * 8.0;
                    let set = [(a, mp), (b, mp)];
                    let e0 = energy(&w, &set) - floor;
                    let (mut lo, mut hi, mut sep) = (1.0f32, 1.0f32, 0.0f32);
                    for _ in 0..(10.0 * hz) as usize {
                        w.step(dt);
                        let r = (energy(&w, &set) - floor) / e0;
                        lo = lo.min(r); hi = hi.max(r);
                        let (qa, qb) = (w.get_body_pose(a), w.get_body_pose(b));
                        sep = sep.max(((qa.pos + qa.rot * Vec3::new(-1.0, 0.0, 0.0)) - Vec3::new(0.0, 10.0, 0.0)).length())
                                 .max((qa.pos - (qb.pos + qb.rot * Vec3::new(-1.0, 0.0, 0.0))).length());
                    }
                    println!("{:?} {:5.1} Hz pos {:3} | double pendulum, 0.2 m cubes on 1 m arms: energy 10 s {:6.3}..{:6.3}, worst sep {:.4} m",
                             proj, hz, pos_iters, lo, hi, sep);
                }
                // (c) conical pendulum: 0.1 m cube on a 2 m arm (ball joint), 30 degree cone
                {
                    let mut w = World::with_capacity(4, 4);
                    w.set_joint_projection(proj);
                    w.set_solver_iterations(20, pos_iters);
                    w.set_gravity(Vec3::new(0.0, -G, 0.0));
                    let pivot = Vec3::new(0.0, 10.0, 0.0);
                    let l = 2.0f32;
                    let cone = 30f32.to_radians();
                    let p = w.add_body(iso(pivot), Velocity::default(), MassProps::infinite(), false);
                    let d = Vec3::new(cone.sin(), -cone.cos(), 0.0);
                    let centre = pivot + d * l;
                    let omega = (G / (l * cone.cos())).sqrt();
                    let spin = Vec3::new(0.0, omega, 0.0);
                    let mp = MassProps::from_box_density(Vec3::splat(0.05), 1000.0);
                    // Bob's local +y points at the pivot
                    let rot = Quat::from_rotation_arc(Vec3::Y, -d);
                    let b = w.add_body(Isometry { pos: centre, rot }, Velocity { lin: spin.cross(centre - pivot), ang: spin }, mp, true);
                    w.add_ball_joint(p, b, iso(Vec3::ZERO), iso(Vec3::new(0.0, l, 0.0)));
                    let polar = |w: &World| { let c = w.get_body_pose(b).pos - pivot; (-c.y / c.length()).acos() };
                    let (mut lo, mut hi) = (cone, cone);
                    for _ in 0..(10.0 * hz) as usize {
                        w.step(dt);
                        let a = polar(&w); lo = lo.min(a); hi = hi.max(a);
                    }
                    println!("{:?} {:5.1} Hz pos {:3} | conical pendulum 0.1 m bob on 2 m arm: cone 30.00 deg, range over 10 s {:6.2}..{:6.2} deg",
                             proj, hz, pos_iters, lo.to_degrees(), hi.to_degrees());
                }
            }
        }
    }
}

/// Cost of the joint solve: a chain of 20 hinged 1 m rods, 2400 steps at 240 Hz (SHAKE, 20
/// position iterations). Prints wall time per step; compare builds on the same machine.
#[test]
#[ignore]
fn joint_solve_timing() {
    let mut w = World::with_capacity(32, 32);
    w.set_gravity(Vec3::new(0.0, -G, 0.0));
    let mut prev = w.add_body(iso(Vec3::new(0.0, 30.0, 0.0)), Velocity::default(), MassProps::infinite(), false);
    for k in 0..20 {
        let b = w.add_body(iso(Vec3::new(0.5 + k as f32, 30.0, 0.0)), Velocity::default(), MassProps::from_box_density(Vec3::new(0.5, 0.025, 0.025), 1000.0), true);
        let fa = if k == 0 { iso(Vec3::ZERO) } else { iso(Vec3::new(0.5, 0.0, 0.0)) };
        w.add_hinge_joint(prev, b, fa, iso(Vec3::new(-0.5, 0.0, 0.0)), 2);
        prev = b;
    }
    for _ in 0..240 { w.step(1.0 / 240.0); }
    let mut best = f64::MAX;
    for _ in 0..5 {
        let t = std::time::Instant::now();
        for _ in 0..480 { w.step(1.0 / 240.0); }
        best = best.min(t.elapsed().as_secs_f64() / 480.0);
    }
    println!("20-rod chain: best {:.1} us per step", best * 1e6);
}

/// Static sag of a sprung hinge (1 m rod, spring on z, c in rad per N m), kept awake, started
/// at the analytic sag: the angle it settles at, per mode, rate and iteration count.
#[test]
#[ignore]
fn sprung_hinge_sag_offset() {
    use riftphys_articulation::bodies::{D6Axis, D6Joint};
    let c = 0.15f32;
    let half = Vec3::new(0.5, 0.025, 0.025);
    let m = 8.0 * half.x * half.y * half.z * 1000.0;
    let mut theta = 0.0f32;
    for _ in 0..200 { theta = 0.5 * theta + 0.5 * c * m * G * 0.5 * theta.cos(); }
    for proj in [JointProjection::Pbd, JointProjection::Shake] {
        for hz in [60.0f32, 120.0, 240.0, 480.0] {
            for iters in [20u32, 100] {
                let mut w = World::with_capacity(4, 4);
                w.set_joint_projection(proj);
                w.set_solver_iterations(20, iters);
                w.set_gravity(Vec3::new(0.0, -G, 0.0));
                let p = w.add_body(iso(Vec3::new(0.0, 10.0, 0.0)), Velocity::default(), MassProps::infinite(), false);
                let rot = Quat::from_rotation_z(-theta);
                let rod = w.add_body(Isometry { pos: Vec3::new(0.0, 10.0, 0.0) + rot * Vec3::new(0.5, 0.0, 0.0), rot }, Velocity::default(), MassProps::from_box_density(half, 1000.0), true);
                w.set_body_sleep_allowed(rod, false);
                let mut j = D6Joint { a: p, b: rod, fa: iso(Vec3::ZERO), fb: iso(Vec3::new(-0.5, 0.0, 0.0)), t: Default::default(), r: Default::default(), hinge_axis: Some(2) };
                for i in 0..3 { j.t[i].enabled = true; j.r[i].enabled = true; }
                j.r[2] = D6Axis { enabled: true, compliance: c, ..Default::default() };
                w.add_d6_joint(j);
                let (mut lo, mut hi) = (f32::MAX, f32::MIN);
                let mut line = String::new();
                for k in 0..(10.0 * hz) as usize {
                    w.step(1.0 / hz);
                    let a = -hinge_angle(&w, rod);
                    if k as f32 >= 5.0 * hz { lo = lo.min(a); hi = hi.max(a); }
                    if k == 0 || k == (0.5 * hz) as usize || k == (10.0 * hz) as usize - 1 { line += &format!(" {:.5}", a / theta); }
                }
                println!("{:?} {:5.0} Hz iters {:3}: sag/static at 1 tick, 0.5 s, 10 s:{}   range 5-10 s {:.5}..{:.5}", proj, hz, iters, line, lo / theta, hi / theta);
            }
        }
    }
}
