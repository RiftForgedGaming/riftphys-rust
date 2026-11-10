// SPDX-License-Identifier: RiftForged Proprietary License
//! PARITY_SPEC W4f (3.4h): the geared rotor's row in the position pass. Rigs are coaxial (every
//! body's COM on the z axis through the hinges), so each is an exact reduced-coordinate system:
//! a motor link (I_m) on a hinge with armature J_a, a load link (I_l) on a sprung hinge to it.
use riftphys_articulation::bodies::{D6Axis, D6Joint, D6Limit};
use riftphys_core::{BodyId, Isometry, JointId, Quat, Vec3, Velocity};
use riftphys_geom::MassProps;
use riftphys_world::world::World;

fn iso(rot: Quat) -> Isometry { Isometry { pos: Vec3::ZERO, rot } }

/// A body centred on the axis with inertia `izz` about z (and a stiff 1 kg mass so the base
/// moves little linearly; every COM is on the axis anyway).
fn disc(w: &mut World, izz: f32, rot: Quat) -> BodyId {
    let mut mp = MassProps::from_box_density(Vec3::splat(0.05), 1.0 / 0.001);
    mp.inertia.z_axis.z = izz;
    mp.inertia.x_axis.x = izz.max(1.0e-3);
    mp.inertia.y_axis.y = izz.max(1.0e-3);
    let b = w.add_body(iso(rot), Velocity::default(), mp, true);
    w.set_body_sleep_allowed(b, false);
    b
}

struct Rig { w: World, base: BodyId, motor: BodyId, load: BodyId, mj: JointId }

/// Base (fixed or free), the motor hinge with `j_a`, the sprung hinge (k) to the load, which
/// starts turned by `defl` from the motor. No gravity. Optional limit on the motor hinge.
fn rig(i_m: f32, i_l: f32, j_a: f32, k: f32, defl: f32, base_free: bool, iters: (u32, u32), limit: Option<D6Limit>) -> Rig {
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::ZERO);
    w.set_solver_iterations(iters.0, iters.1);
    let base = if base_free { disc(&mut w, 1.0, Quat::IDENTITY) } else { w.add_body(iso(Quat::IDENTITY), Velocity::default(), MassProps::infinite(), false) };
    let motor = disc(&mut w, i_m, Quat::IDENTITY);
    let load = disc(&mut w, i_l, Quat::from_rotation_z(defl));
    let mj = w.add_hinge_joint(base, motor, iso(Quat::IDENTITY), iso(Quat::IDENTITY), 2);
    if limit.is_some() { w.set_d6_limit_r(mj, 2, limit); }
    if j_a > 0.0 { w.set_d6_armature(mj, 2, j_a); }
    let mut d = D6Joint { a: motor, b: load, fa: iso(Quat::IDENTITY), fb: iso(Quat::IDENTITY), t: Default::default(), r: Default::default(), hinge_axis: Some(2) };
    for i in 0..3 { d.t[i].enabled = true; d.r[i].enabled = true; }
    d.r[2] = D6Axis { enabled: true, compliance: 1.0 / k, ..Default::default() };
    w.add_d6_joint(d);
    Rig { w, base, motor, load, mj }
}

fn yaw(w: &World, b: BodyId) -> f32 { let q = w.get_body_pose(b).rot; 2.0 * q.z.atan2(q.w) }

/// One step's position-pass turns of the motor and the load (final minus predicted).
fn pass_turns(r: &mut Rig, dt: f32) -> (f32, f32) {
    r.w.set_record_predicted(true);
    r.w.step(dt);
    let pred = r.w.predicted_poses().unwrap().to_vec();
    let at = |b: BodyId| { let q = pred[b.0 as usize].rot; 2.0 * q.z.atan2(q.w) };
    (yaw(&r.w, r.motor) - at(r.motor), yaw(&r.w, r.load) - at(r.load))
}

const I_M: f32 = 0.002;
const I_L: f32 = 0.04;
const J_A: f32 = 0.0934;

#[test]
fn a_the_spring_correction_splits_by_i_plus_j_a() {
    let want = -I_L / (I_M + J_A);
    let mut r = rig(I_M, I_L, J_A, 1500.0, -0.06, false, (8, 256), None);
    let (m, l) = pass_turns(&mut r, 1.0 / 240.0);
    assert!(((m / l) - want).abs() < 0.02 * want.abs(), "converged: split {:.4}, want {:.4} (without the rotor row {:.4})", m / l, want, -I_L / I_M);
    let mut r = rig(I_M, I_L, J_A, 1500.0, -0.06, false, (8, 6), None);
    let (m6, l6) = pass_turns(&mut r, 1.0 / 240.0);
    println!("(a) split converged {:.4}, at (8, 6) {:.4}, analytic {:.4}; today's (no rotor row) analytic {:.4}", m / l, m6 / l6, want, -I_L / I_M);
    let mut r = rig(I_M, I_L, J_A, 1500.0, -0.06, false, (8, 256), None);
    r.w.set_rotor_position_row(false);
    let (m0, l0) = pass_turns(&mut r, 1.0 / 240.0);
    println!("(a) without the rotor row (W4b alone), converged: split {:.4}", m0 / l0);
}

#[test]
fn b_the_split_is_the_same_at_every_rate() {
    let want = -I_L / (I_M + J_A);
    for hz in [60.0f32, 240.0, 2000.0] {
        let mut r = rig(I_M, I_L, J_A, 1500.0, -0.06, false, (8, 256), None);
        let (m, l) = pass_turns(&mut r, 1.0 / hz);
        assert!(((m / l) - want).abs() < 0.02 * want.abs(), "{} Hz: split {:.4}, want {:.4}", hz, m / l, want);
    }
}

#[test]
fn c_no_impulse_when_nothing_else_corrects() {
    // A free hinge with armature (no spring, no limit) under a constant torque
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::ZERO);
    let base = w.add_body(iso(Quat::IDENTITY), Velocity::default(), MassProps::infinite(), false);
    let m = disc(&mut w, 0.05, Quat::IDENTITY);
    let j = w.add_hinge_joint(base, m, iso(Quat::IDENTITY), iso(Quat::IDENTITY), 2);
    w.set_d6_armature(j, 2, 0.05);
    let mut worst = 0.0f32;
    for hz in [60.0f32, 240.0] {
        for _ in 0..(hz as usize) {
            w.apply_torque(m, Vec3::new(0.0, 0.0, 0.3 / hz));
            w.step(1.0 / hz);
            worst = worst.max(w.d6_rotor_position_lambda(j, 2).abs());
        }
    }
    assert!(worst < 1.0e-6, "the rotor row moved a free hinge: |lambda| {:e}", worst);
}

fn energy(r: &Rig, k: f32, j_a: f32) -> f32 {
    let e_body = |b: BodyId, i: f32| 0.5 * i * r.w.get_body_vel(b).ang.z.powi(2);
    let rel = yaw(&r.w, r.load) - yaw(&r.w, r.motor);
    let rotor = r.w.d6_rotor_velocity(r.mj, 2).unwrap_or(0.0);
    e_body(r.base, 1.0) + e_body(r.motor, I_M) + e_body(r.load, I_L) + 0.5 * j_a * rotor * rotor + 0.5 * k * rel * rel
}

/// (d)'s torque-free half, rigid coupling (the Reviewer, 2026-11-10): a free 1 kg base, the motor
/// hinge (J_a) and a load locked rigidly to the motor with its COM 0.2 m off the axis, spinning at
/// 5 rad/s about it, no gravity. The hinge's rows correct the off-axis load every step and the base
/// recoils, so the position pass works on the armature joint. Max relative energy drift over 2 s.
fn rigid_spin_drift(j_a: f32, hz: f32) -> f32 {
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::ZERO);
    let base = disc(&mut w, 1.0, Quat::IDENTITY);
    let motor = disc(&mut w, I_M, Quat::IDENTITY);
    let mut mp = MassProps::from_box_density(Vec3::splat(0.05), 1.0 / 0.001);
    mp.inertia.z_axis.z = I_L;
    let load = w.add_body(Isometry { pos: Vec3::new(0.2, 0.0, 0.0), rot: Quat::IDENTITY }, Velocity::default(), mp, true);
    w.set_body_sleep_allowed(load, false);
    let mj = w.add_hinge_joint(base, motor, iso(Quat::IDENTITY), iso(Quat::IDENTITY), 2);
    let mut d = D6Joint { a: motor, b: load, fa: iso(Quat::IDENTITY), fb: Isometry { pos: Vec3::new(-0.2, 0.0, 0.0), rot: Quat::IDENTITY }, t: Default::default(), r: Default::default(), hinge_axis: None };
    for i in 0..3 { d.t[i].enabled = true; d.r[i].enabled = true; }
    w.add_d6_joint(d);
    w.set_body_vel(motor, Velocity { lin: Vec3::ZERO, ang: Vec3::new(0.0, 0.0, 5.0) });
    w.set_body_vel(load, Velocity { lin: Vec3::new(0.0, 1.0, 0.0), ang: Vec3::new(0.0, 0.0, 5.0) });
    if j_a > 0.0 { w.set_d6_armature(mj, 2, j_a); }
    let e = |w: &World| -> f32 {
        let ke = |b: BodyId, m: f32, i: f32| { let v = w.get_body_vel(b); 0.5 * m * v.lin.length_squared() + 0.5 * i * v.ang.z * v.ang.z };
        let rotor = w.d6_rotor_velocity(mj, 2).unwrap_or(0.0);
        ke(base, 1.0, 1.0) + ke(motor, 1.0, I_M) + ke(load, 1.0, I_L) + 0.5 * j_a * rotor * rotor
    };
    let e0 = e(&w);
    let mut worst = 0.0f32;
    for _ in 0..(2.0 * hz) as usize { w.step(1.0 / hz); worst = worst.max((e(&w) - e0).abs() / e0); }
    worst
}

#[test]
fn d_energy_balance() {
    let (k, hz) = (20.0f32, 240.0f32);
    let (with, without) = (rigid_spin_drift(J_A, hz), rigid_spin_drift(0.0, hz));
    println!("(d) torque-free 2 s, rigid coupling: max relative energy drift {:.4} with the rotor, {:.4} without armature", with, without);
    assert!(with <= (1.5 * without).max(0.02), "energy drift {:.4} with the rotor, {:.4} without", with, without);
    // Base fixed, constant motor torque for 0.5 s: energy gained = work
    let tau = 0.5f32;
    let mut r = rig(I_M, I_L, J_A, k, 0.0, false, (8, 6), None);
    let e0 = energy(&r, k, J_A);
    let mut work = 0.0f32;
    for _ in 0..(0.5 * hz) as usize {
        let a0 = yaw(&r.w, r.motor);
        r.w.apply_torque(r.motor, Vec3::new(0.0, 0.0, tau / hz));
        r.w.step(1.0 / hz);
        work += tau * (yaw(&r.w, r.motor) - a0);
    }
    let gained = energy(&r, k, J_A) - e0;
    println!("(d) under torque: energy gained {:.5} J, work {:.5} J", gained, work);
    assert!((gained - work).abs() < 0.05 * work.abs(), "energy gained {:.5} J against work {:.5} J", gained, work);
}

#[test]
fn e_rollback_with_the_rotor_row_replays() {
    let mut r = rig(I_M, I_L, J_A, 1500.0, -0.06, false, (8, 6), None);
    let tq = |r: &mut Rig| r.w.apply_torque(r.motor, Vec3::new(0.0, 0.0, 90.0 / 240.0));
    for _ in 0..20 { tq(&mut r); r.w.step(1.0 / 240.0); }
    let snap: Vec<_> = [r.motor, r.load].iter().map(|&b| (b, r.w.get_body_pose(b), r.w.get_body_vel(b))).collect();
    let carried = r.w.carried_state();
    let h0 = r.w.step_hash();
    let run = |r: &mut Rig| -> Vec<[u8; 32]> { (0..60).map(|_| { tq(r); r.w.step(1.0 / 240.0); r.w.step_hash() }).collect() };
    let first = run(&mut r);
    for (b, p, v) in &snap { r.w.set_body_pose(*b, *p); r.w.set_body_vel(*b, *v); }
    r.w.restore_carried_state(&carried);
    assert_eq!(r.w.step_hash(), h0, "restore hashes differently");
    assert_eq!(first, run(&mut r), "replay with the rotor row diverged");
}

/// The Cassie knee in miniature: base fixed, the motor hinge with J_a, a light motor link, the
/// spring, the load; a constant 20 N m motor torque from rest (the load turns about 0.7 rad in
/// 100 ms; 90 N m wrapped the angle past pi). Load (angle, rate) per 240 Hz tick. The spring is
/// K_G = 36 N m/rad, so with the rotor h omega_n = 0.15 at 240 Hz (omega_n^2 = k (1/I_l +
/// 1/(I_m + J_a))): the gate measures the rotor, not implicit Euler's damping of a stiff spring
/// (the Reviewer, 2026-11-10; at 1500 N m/rad, h omega ~ 1 left a 4% residue converged).
const K_G: f32 = 36.0;
fn knee(hz: f32, rotor_row: bool) -> Vec<(f32, f32)> { knee_at(hz, rotor_row, (8, 6)) }

fn knee_at(hz: f32, rotor_row: bool, iters: (u32, u32)) -> Vec<(f32, f32)> {
    let mut r = rig(I_M, I_L, J_A, K_G, 0.0, false, iters, None);
    r.w.set_rotor_position_row(rotor_row);
    let sub = (hz / 240.0).round() as usize;
    let mut out = Vec::new();
    for _ in 0..24 {
        for _ in 0..sub {
            r.w.apply_torque(r.motor, Vec3::new(0.0, 0.0, 20.0 / hz));
            r.w.step(1.0 / hz);
        }
        out.push((yaw(&r.w, r.load), r.w.get_body_vel(r.load).ang.z));
    }
    out
}

/// (g)'s reference (restated at review, 2026-11-10): the same 2-DoF system stepped with the
/// engine's own scheme, in reduced coordinates, at 240 Hz. q = (motor angle, load angle) about z,
/// M = diag(I_m + J_a, I_l) (the rotor's inertia rides on the motor coordinate), K the spring
/// k [[1, -1], [-1, 1]], tau = (20, 0) N m. Each step: the torque kick v += h M^-1 tau (the velocity
/// solve, where the spring has no row); the drift q* = q + h v; the implicit spring
/// (M + h^2 K) q+ = M q* (the converged XPBD position pass); v+ = (q+ - q) / h. Load (angle, rate).
fn knee_reference(hz: f32) -> Vec<(f32, f32)> {
    let (h, m1, m2, k, tau) = (1.0 / hz as f64, (I_M + J_A) as f64, I_L as f64, K_G as f64, 20.0f64);
    let (mut q, mut v) = ([0.0f64; 2], [0.0f64; 2]);
    let sub = (hz / 240.0).round() as usize;
    let mut out = Vec::new();
    for _ in 0..24 {
        for _ in 0..sub {
            v[0] += h * tau / m1;
            let qs = [q[0] + h * v[0], q[1] + h * v[1]];
            // (M + h^2 K) q+ = M q*
            let (a, b, c, d) = (m1 + h * h * k, -h * h * k, -h * h * k, m2 + h * h * k);
            let (r0, r1) = (m1 * qs[0], m2 * qs[1]);
            let det = a * d - b * c;
            let qn = [(d * r0 - b * r1) / det, (a * r1 - c * r0) / det];
            v = [(qn[0] - q[0]) / h, (qn[1] - q[1]) / h];
            q = qn;
        }
        out.push((q[1] as f32, v[1] as f32));
    }
    out
}

/// The 2-DoF analytic solution's load angle at 240 Hz ticks (report-only).
fn knee_analytic() -> Vec<f32> {
    let (i1, i2, k, tau) = (I_M + J_A, I_L, K_G, 20.0f32);
    let wn = (k * (1.0 / i1 + 1.0 / i2)).sqrt();
    (1..=24).map(|n| {
        let t = n as f32 / 240.0;
        let x = tau / (i1 * wn * wn) * (1.0 - (wn * t).cos());
        0.5 * tau / (i1 + i2) * t * t - (i1 / (i1 + i2)) * x
    }).collect()
}

/// max |error| / max |reference| for the load's angle and rate.
fn rel_err(a: &[(f32, f32)], f: &[(f32, f32)]) -> (f32, f32) {
    let range = |g: &dyn Fn(&(f32, f32)) -> f32| f.iter().map(|x| g(x).abs()).fold(0.0f32, f32::max);
    let err = |g: &dyn Fn(&(f32, f32)) -> f32| a.iter().zip(f).map(|(x, y)| (g(x) - g(y)).abs()).fold(0.0f32, f32::max);
    (err(&|x| x.0) / range(&|x| x.0), err(&|x| x.1) / range(&|x| x.1))
}

#[test]
fn g_the_knee_in_miniature_matches_its_reduced_coordinate_scheme() {
    let reference = knee_reference(240.0);
    let (a0, r0) = rel_err(&knee(240.0, false), &reference);
    let (a1, r1) = rel_err(&knee(240.0, true), &reference);
    let fine = knee(8160.0, true);
    let analytic = knee_analytic();
    println!("(g) 240 Hz against the reduced-coordinate scheme at 240 Hz (max |error| / max |value|): W4b alone angle {:.4} rate {:.4}; with the rotor angle {:.4} rate {:.4}", a0, r0, a1, r1);
    println!("(g) load angle at 100 ms: 240 Hz {:.5}, reduced scheme {:.5}, 8 kHz {:.5}, analytic {:.5}", knee(240.0, true)[23].0, reference[23].0, fine[23].0, analytic[23]);
    assert!(a0 > 0.03 || r0 > 0.03, "the mutation check: W4b alone already passes ({:.4}, {:.4})", a0, r0);
    assert!(a1 < 0.03 && r1 < 0.03, "with the rotor: angle {:.4}, rate {:.4} of range", a1, r1);
}

#[test]
fn h_a_limit_has_the_last_word() {
    let lim = D6Limit { min: -0.5, max: 0.5 };
    let mut r = rig(I_M, I_L, J_A, 1500.0, 0.0, false, (8, 6), Some(lim));
    let mut worst = 0.0f32;
    for k in 0..240 {
        r.w.apply_torque(r.motor, Vec3::new(0.0, 0.0, 90.0 / 240.0));
        r.w.step(1.0 / 240.0);
        if k > 10 { worst = worst.max(yaw(&r.w, r.motor) - 0.5); }
    }
    assert!(worst < 1.0e-3, "past the stop by {:.5} rad", worst);
}

#[test]
#[ignore]
fn probe() {
    let want = -I_L / (I_M + J_A);
    for it in [64u32, 256, 1024] {
        for hz in [60.0f32, 240.0, 2000.0] {
            let mut r = rig(I_M, I_L, J_A, 1500.0, -0.06, false, (8, it), None);
            let (m, l) = pass_turns(&mut r, 1.0 / hz);
            println!("iters {} {} Hz split {:.4} (want {:.4}) load turn {:.5}", it, hz, m / l, want, l);
        }
    }
    for row in [false, true] {
        for iters in [(8u32, 6u32), (8, 64)] {
            let mut r = rig(I_M, I_L, J_A, 1500.0, 0.0, false, iters, Some(D6Limit { min: -0.5, max: 0.5 }));
            r.w.set_rotor_position_row(row);
            let mut worst = 0.0f32;
            for k in 0..240 {
                r.w.apply_torque(r.motor, Vec3::new(0.0, 0.0, 90.0 / 240.0));
                r.w.step(1.0 / 240.0);
                if k > 10 { worst = worst.max(yaw(&r.w, r.motor) - 0.5); }
            }
            println!("limit: rotor row {} iters {:?}: worst past stop {:.5}, final {:.4}, rotor w {:?}", row, iters, worst, yaw(&r.w, r.motor), r.w.d6_rotor_velocity(r.mj, 2));
        }
    }
    let (k, hz) = (20.0f32, 240.0f32);
    let mut r = rig(I_M, I_L, 0.0, k, -0.2, true, (8, 6), None);
    for s in 0..10 { println!("energy step {}: {:.5} motor {:.4} load {:.4} base {:.4}", s, energy(&r, k, 0.0), yaw(&r.w, r.motor), yaw(&r.w, r.load), yaw(&r.w, r.base)); r.w.step(1.0 / hz); }
    for iters in [(8u32, 6u32), (8, 24), (8, 96), (8, 384), (64, 384)] {
        for row in [false, true] {
            let (a, f) = (knee_at(240.0, row, iters), knee_reference(240.0));
            let ra = f.iter().map(|x| x.0.abs()).fold(0.0f32, f32::max);
            let rr = f.iter().map(|x| x.1.abs()).fold(0.0f32, f32::max);
            let ea = a.iter().zip(&f).map(|(x, y)| (x.0 - y.0).abs()).fold(0.0f32, f32::max);
            let er = a.iter().zip(&f).map(|(x, y)| (x.1 - y.1).abs()).fold(0.0f32, f32::max);
            println!("knee 240 Hz at {:?} (8 kHz at (8, 6)), rotor row {}: angle {:.4} rate {:.4}", iters, row, ea / ra, er / rr);
        }
    }
    let f6 = knee_at(8160.0, true, (8, 6)); let f96 = knee_at(8160.0, true, (8, 96));
    println!("8 kHz reference (8,6) vs (8,96): last angle {:.5} / {:.5}, last rate {:.4} / {:.4}", f6[23].0, f96[23].0, f6[23].1, f96[23].1);
}

#[test]
#[ignore]
fn probe_knee_series() {
    let (a, f) = (knee_at(240.0, true, (8, 96)), knee_at(8160.0, true, (8, 6)));
    let w = knee_at(240.0, false, (8, 96));
    // Analytic: motor+rotor I1, load I2, spring K_G, torque 20 on the motor from rest
    let (i1, i2, k, tau) = (I_M + J_A, I_L, K_G, 20.0f32);
    let wn = (k * (1.0 / i1 + 1.0 / i2)).sqrt();
    for n in 0..24 {
        let t = (n + 1) as f32 / 240.0;
        let acc = tau / (i1 + i2);
        let rel_amp = tau / (i1 * wn * wn); // spring windup's steady value; rel = amp (1 - cos wn t)
        let load = 0.5 * acc * t * t - (i1 / (i1 + i2)) * 0.0 - rel_amp * (i1 / (i1 + i2)) * 0.0;
        let _ = load;
        // exact: theta_l = acc t^2/2 - (I1/(I1+I2)) * x where x = motor-load windup = (tau I2 / (I1 (I1+I2) wn^2)) (1 - cos wn t)
        let x = tau / (i1 * wn * wn) * (1.0 - (wn * t).cos());
        let th_l = 0.5 * acc * t * t - (i1 / (i1 + i2)) * x;
        println!("{:5.1} ms | 240 {:.5} | 8k {:.5} | analytic {:.5} | 240 W4b {:.5}", t * 1e3, a[n].0, f[n].0, th_l, w[n].0);
    }
}

/// The (a) rig with the spring damped (a W4c damped compliant row, gamma > 0) and the motor hinge
/// optionally duplicated (a second identical hinge: five redundant rigid rows in the block).
fn rig_block(damping: f32, duplicate: bool, iters: (u32, u32), blocks: bool) -> Rig {
    let mut r = rig(I_M, I_L, J_A, 1500.0, -0.06, false, iters, None);
    r.w.set_armature_blocks(blocks);
    if duplicate { r.w.add_hinge_joint(r.base, r.motor, iso(Quat::IDENTITY), iso(Quat::IDENTITY), 2); }
    if damping > 0.0 {
        // Rebuild the spring joint with damping: a fresh rig with the damped spring
        let mut w = World::with_capacity(4, 4);
        w.set_gravity(Vec3::ZERO);
        w.set_solver_iterations(iters.0, iters.1);
        let base = w.add_body(iso(Quat::IDENTITY), Velocity::default(), MassProps::infinite(), false);
        let motor = disc(&mut w, I_M, Quat::IDENTITY);
        let load = disc(&mut w, I_L, Quat::from_rotation_z(-0.06));
        let mj = w.add_hinge_joint(base, motor, iso(Quat::IDENTITY), iso(Quat::IDENTITY), 2);
        w.set_d6_armature(mj, 2, J_A);
        if duplicate { w.add_hinge_joint(base, motor, iso(Quat::IDENTITY), iso(Quat::IDENTITY), 2); }
        let mut d = D6Joint { a: motor, b: load, fa: iso(Quat::IDENTITY), fb: iso(Quat::IDENTITY), t: Default::default(), r: Default::default(), hinge_axis: Some(2) };
        for i in 0..3 { d.t[i].enabled = true; d.r[i].enabled = true; }
        d.r[2] = D6Axis { enabled: true, compliance: 1.0 / 1500.0, damping, ..Default::default() };
        w.add_d6_joint(d);
        w.set_armature_blocks(blocks);
        r = Rig { w, base, motor, load, mj };
    }
    r
}

/// Motor and load yaw, the load's position, and the rotor's position-pass lambda after `steps`
/// steps under a 90 N m motor torque.
fn run_block(r: &mut Rig, steps: usize) -> (f32, f32, Vec3, f32) {
    for _ in 0..steps {
        r.w.apply_torque(r.motor, Vec3::new(0.0, 0.0, 90.0 / 240.0));
        r.w.step(1.0 / 240.0);
    }
    (yaw(&r.w, r.motor), yaw(&r.w, r.load), r.w.get_body_pose(r.load).pos, r.w.d6_rotor_position_lambda(r.mj, 2))
}

#[test]
fn block_matches_converged_gauss_seidel() {
    // A damped compliant row (gamma ~ 1) and rigid rows on the light link: the block at (8, 6)
    // against Gauss-Seidel alone at (8, 4096), one step
    let damping = 1500.0 * 1.0 / 240.0; // gamma = c d / h = 1
    let (mb, lb, pb, lamb) = run_block(&mut rig_block(damping, false, (8, 6), true), 1);
    let (mg, lg, pg, lamg) = run_block(&mut rig_block(damping, false, (8, 4096), false), 1);
    println!("block (8, 6): motor {:.7} load {:.7} rotor lambda {:.7}; Gauss-Seidel (8, 4096): motor {:.7} load {:.7} rotor lambda {:.7}", mb, lb, lamb, mg, lg, lamg);
    assert!((mb - mg).abs() < 1.0e-6 && (lb - lg).abs() < 1.0e-6 && (pb - pg).length() < 1.0e-6, "poses differ: motor {:e}, load {:e}, position {:e}", (mb - mg).abs(), (lb - lg).abs(), (pb - pg).length());
    assert!((lamb - lamg).abs() < 1.0e-6, "rotor lambda {:e} against {:e}", lamb, lamg);
}

#[test]
fn a_duplicated_row_changes_nothing() {
    let (m1, l1, p1, _) = run_block(&mut rig_block(0.0, false, (8, 6), true), 24);
    let mut dup = rig_block(0.0, true, (8, 6), true);
    let (m2, l2, p2, _) = run_block(&mut dup, 24);
    println!("single hinge: motor {:.6} load {:.6}; duplicated: motor {:.6} load {:.6}; blocks {:?}", m1, l1, m2, l2, dup.w.armature_blocks());
    assert!((m1 - m2).abs() < 1.0e-5 && (l1 - l2).abs() < 1.0e-5 && (p1 - p2).length() < 1.0e-5, "duplicate changed it: motor {:e}, load {:e}", (m1 - m2).abs(), (l1 - l2).abs());
    assert_eq!(dup.w.armature_blocks(), (1, 0));
}
