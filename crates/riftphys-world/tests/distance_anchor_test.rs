// Anchored, damped distance joints (PARITY_SPEC W4c): the rod passes torque through its lever
// arms, stretches by F x compliance, and its damper acts where the oscillator is resolved.
use riftphys_articulation::bodies::{D6Axis, D6Joint};
use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::MassProps;
use riftphys_world::world::World;

fn iso(pos: Vec3, rot: Quat) -> Isometry { Isometry { pos, rot } }

/// A lever of length `l` along direction angle `th` (radians, in the xy plane) from `pivot`.
fn lever(w: &mut World, base: BodyId, pivot: Vec3, l: f32, th: f32, spring: Option<f32>) -> (BodyId, Quat) {
    let rot = Quat::from_rotation_z(th);
    let mp = MassProps::from_box_density(Vec3::new(0.5 * l, 0.02, 0.02), 1000.0);
    let b = w.add_body(iso(pivot + rot * Vec3::new(0.5 * l, 0.0, 0.0), rot), Velocity::default(), mp, true);
    w.set_body_sleep_allowed(b, false);
    let (fa, fb) = (iso(pivot, Quat::IDENTITY), iso(Vec3::new(-0.5 * l, 0.0, 0.0), rot.conjugate()));
    let j = match spring {
        None => w.add_hinge_joint(base, b, fa, fb, 2),
        Some(k) => {
            let mut d = D6Joint { a: base, b, fa, fb, t: Default::default(), r: Default::default(), hinge_axis: Some(2) };
            for i in 0..3 { d.t[i].enabled = true; d.r[i].enabled = true; }
            d.r[2] = D6Axis { enabled: true, compliance: 1.0 / k, ..Default::default() };
            w.add_d6_joint(d)
        }
    };
    // No hinge dampers: a damper solved before the spring's position correction carries part of
    // a static load (3.4b, open). The compliant spring's own numerical damping settles it.
    let _ = j;
    (b, rot)
}

/// Lever A (0.4 m, driven by a constant torque) and lever B (0.25 m, held by a torsional spring
/// of 50 N m/rad), both at angle `th` to the rod joining their tips. At rest the spring holds
/// tau_A x (r_B x n)_z / (r_A x n)_z, with the lever arms and rod direction where they end up.
fn torque_ratio(th: f32, compliance: f32) -> (f32, f32) {
    let mut w = World::with_capacity(8, 8);
    w.set_gravity(Vec3::ZERO);
    let base = w.add_body(iso(Vec3::ZERO, Quat::IDENTITY), Velocity::default(), MassProps::infinite(), false);
    let (la, lb, k, tau) = (0.4f32, 0.25f32, 50.0f32, 0.5f32);
    let pb = Vec3::new(1.0, 0.0, 0.0) + Quat::from_rotation_z(th) * Vec3::new(la - lb, 0.0, 0.0);
    let (a, _) = lever(&mut w, base, Vec3::ZERO, la, th, None);
    let (b, rot_b0) = lever(&mut w, base, pb, lb, th, Some(k));
    // Tips: the rod runs along x, 1 m long
    let rod = w.add_distance_joint_at(a, Vec3::new(0.5 * la, 0.0, 0.0), b, Vec3::new(0.5 * lb, 0.0, 0.0), 1.0, compliance);
    let _ = rod;
    let hz = 240.0f32;
    for _ in 0..(20.0 * hz) as usize {
        w.apply_torque(a, Vec3::new(0.0, 0.0, tau / hz));
        w.step(1.0 / hz);
    }
    let (qa, qb) = (w.get_body_pose(a), w.get_body_pose(b));
    let tip_a = qa.pos + qa.rot * Vec3::new(0.5 * la, 0.0, 0.0);
    let tip_b = qb.pos + qb.rot * Vec3::new(0.5 * lb, 0.0, 0.0);
    let n = (tip_b - tip_a).normalize();
    let arm_a = (tip_a - Vec3::ZERO).cross(n).z;
    let arm_b = (tip_b - pb).cross(n).z;
    let expect = tau * arm_b.abs() / arm_a.abs();
    let dq = qb.rot * rot_b0.conjugate();
    let phi = 2.0 * dq.z.atan2(dq.w);
    (k * phi.abs(), expect)
}

#[test]
fn a_rod_between_levers_passes_torque_at_the_lever_arm_ratio() {
    for th in [90.0f32, 60.0, 120.0] {
        for c in [0.0f32, 1.0e-5] {
            let (got, expect) = torque_ratio(th.to_radians(), c);
            assert!((got / expect - 1.0).abs() < 0.02, "levers at {} deg, compliance {}: spring holds {:.5} N m, statics {:.5}", th, c, got, expect);
        }
    }
}

/// A 2 kg box hanging under gravity from a static point by a compliant anchored rod (anchor
/// 0.1 m above its COM): at rest the stretch is compliance x m g, whatever the rod's damping
/// (XPBD's damping term acts on the change since the step's start, which is 0 at rest; a
/// velocity row before the projection carried c dt / (m + c dt) of the load).
#[test]
fn a_static_load_stretches_it_by_force_times_compliance() {
    for hz in [60.0f32, 240.0] {
        for damping in [0.0f32, 20.0, 200.0] {
            let mut w = World::with_capacity(4, 4);
            let g = 9.81f32;
            w.set_gravity(Vec3::new(0.0, -g, 0.0));
            let top = Vec3::new(0.0, 5.0, 0.0);
            let pivot = w.add_body(iso(top, Quat::IDENTITY), Velocity::default(), MassProps::infinite(), false);
            let half = Vec3::new(0.1, 0.1, 0.1);
            let mp = MassProps::from_box_density(half, 2.0 / (8.0 * half.x * half.y * half.z));
            let (rest, c) = (1.0f32, 1.0e-3f32);
            let b = w.add_body(iso(top - Vec3::new(0.0, rest + 0.1, 0.0), Quat::IDENTITY), Velocity::default(), mp, true);
            w.set_body_sleep_allowed(b, false);
            let j = w.add_distance_joint_at(pivot, Vec3::ZERO, b, Vec3::new(0.0, 0.1, 0.0), rest, c);
            w.set_distance_damping(j, damping);
            for _ in 0..(20.0 * hz) as usize { w.step(1.0 / hz); }
            let p = w.get_body_pose(b);
            let len = (p.pos + p.rot * Vec3::new(0.0, 0.1, 0.0) - top).length();
            let (got, expect) = (len - rest, c * 2.0 * g);
            assert!((got / expect - 1.0).abs() < 0.01, "{} Hz, damping {}: stretch {:.6} m, F c = {:.6} m", hz, damping, got, expect);
        }
    }
}

/// A 1 m, 2.5 kg rod held out by a damped torsional spring (compliance 0.008 rad/N m) about z,
/// released level: at rest it sags to the static angle, theta = c m g (L/2) cos theta, whatever
/// the hinge's damping.
#[test]
fn a_damped_compliant_hinge_sags_to_its_static_angle() {
    for hz in [60.0f32, 240.0] {
        for damping in [0.0f32, 2.0, 20.0] {
            let mut w = World::with_capacity(4, 4);
            let g = 9.81f32;
            w.set_gravity(Vec3::new(0.0, -g, 0.0));
            let pin = w.add_body(iso(Vec3::new(0.0, 10.0, 0.0), Quat::IDENTITY), Velocity::default(), MassProps::infinite(), false);
            let half = Vec3::new(0.5, 0.025, 0.025);
            let m = 2.5f32;
            let mp = MassProps::from_box_density(half, m / (8.0 * half.x * half.y * half.z));
            let rod = w.add_body(iso(Vec3::new(0.5, 10.0, 0.0), Quat::IDENTITY), Velocity::default(), mp, true);
            w.set_body_sleep_allowed(rod, false);
            let c = 0.008f32;
            let mut d = D6Joint { a: pin, b: rod, fa: iso(Vec3::ZERO, Quat::IDENTITY), fb: iso(Vec3::new(-0.5, 0.0, 0.0), Quat::IDENTITY), t: Default::default(), r: Default::default(), hinge_axis: Some(2) };
            for i in 0..3 { d.t[i].enabled = true; d.r[i].enabled = true; }
            d.r[2] = D6Axis { enabled: true, compliance: c, ..Default::default() };
            let j = w.add_d6_joint(d);
            w.set_d6_damping(j, true, 2, damping, 0.0);
            for _ in 0..(30.0 * hz) as usize { w.step(1.0 / hz); }
            let mut theta = 0.0f32;
            for _ in 0..200 { theta = 0.5 * theta + 0.5 * c * m * g * 0.5 * theta.cos(); }
            let q = w.get_body_pose(rod).rot;
            let got = -2.0 * q.z.atan2(q.w);
            assert!((got / theta - 1.0).abs() < 0.01, "{} Hz, damping {}: sag {:.5} rad, static {:.5} rad", hz, damping, got, theta);
        }
    }
}

/// A closed loop: two hinged levers (B's hinge a damped torsional spring) joined at their tips by
/// a compliant, damped anchored rod, released under gravity. Energy, counting both springs,
/// never rises, and the rod's stretch stays bounded.
#[test]
fn a_damped_loop_never_gains_energy() {
    for hz in [60.0f32, 240.0] {
        let mut w = World::with_capacity(8, 8);
        let g = 9.81f32;
        w.set_gravity(Vec3::new(0.0, -g, 0.0));
        let base = w.add_body(iso(Vec3::ZERO, Quat::IDENTITY), Velocity::default(), MassProps::infinite(), false);
        let (la, lb, k_hinge, c_rod) = (0.4f32, 0.25f32, 20.0f32, 1.0e-4f32);
        let th = 0.3f32;
        let pb = Vec3::new(1.0, 0.0, 0.0) + Quat::from_rotation_z(th) * Vec3::new(la - lb, 0.0, 0.0);
        let mut bodies = Vec::new();
        let (a, _) = lever(&mut w, base, Vec3::ZERO, la, th, None);
        bodies.push(a);
        let rot_b0 = Quat::from_rotation_z(th);
        let mpb = MassProps::from_box_density(Vec3::new(0.5 * lb, 0.02, 0.02), 1000.0);
        let b = w.add_body(iso(pb + rot_b0 * Vec3::new(0.5 * lb, 0.0, 0.0), rot_b0), Velocity::default(), mpb, true);
        w.set_body_sleep_allowed(b, false);
        let mut d = D6Joint { a: base, b, fa: iso(pb, Quat::IDENTITY), fb: iso(Vec3::new(-0.5 * lb, 0.0, 0.0), rot_b0.conjugate()), t: Default::default(), r: Default::default(), hinge_axis: Some(2) };
        for i in 0..3 { d.t[i].enabled = true; d.r[i].enabled = true; }
        d.r[2] = D6Axis { enabled: true, compliance: 1.0 / k_hinge, ..Default::default() };
        let hb = w.add_d6_joint(d);
        w.set_d6_damping(hb, true, 2, 0.5, 0.0);
        bodies.push(b);
        let rod = w.add_distance_joint_at(a, Vec3::new(0.5 * la, 0.0, 0.0), b, Vec3::new(0.5 * lb, 0.0, 0.0), 1.0, c_rod);
        w.set_distance_damping(rod, 50.0);
        let props: Vec<MassProps> = vec![MassProps::from_box_density(Vec3::new(0.5 * la, 0.02, 0.02), 1000.0), mpb];
        let energy = |w: &World| {
            let mut e = 0.0f32;
            for (id, mp) in bodies.iter().zip(&props) {
                let (q, v) = (w.get_body_pose(*id), w.get_body_vel(*id));
                let r = riftphys_core::Mat3::from_quat(q.rot);
                let iw = r * mp.inertia * r.transpose();
                e += 0.5 * mp.mass * v.lin.length_squared() + 0.5 * v.ang.dot(iw * v.ang) + mp.mass * g * q.pos.y;
            }
            let (qa, qb) = (w.get_body_pose(a), w.get_body_pose(b));
            let len = ((qb.pos + qb.rot * Vec3::new(0.5 * lb, 0.0, 0.0)) - (qa.pos + qa.rot * Vec3::new(0.5 * la, 0.0, 0.0))).length();
            let dq = qb.rot * rot_b0.conjugate();
            let phi = 2.0 * dq.z.atan2(dq.w);
            (e + 0.5 * (len - 1.0) * (len - 1.0) / c_rod + 0.5 * k_hinge * phi * phi, (len - 1.0).abs())
        };
        let (e0, _) = energy(&w);
        let (mut top, mut worst) = (e0, 0.0f32);
        for _ in 0..(5.0 * hz) as usize {
            w.step(1.0 / hz);
            let (e, err) = energy(&w);
            top = top.max(e);
            worst = worst.max(err);
        }
        let (e_end, _) = energy(&w);
        println!("{} Hz: energy {:.4} -> {:.4} J, highest {:.4}; worst rod stretch {:.2e} m", hz, e0, e_end, top, worst);
        assert!(top <= e0 + 1e-3 * e0.abs().max(1.0), "{} Hz: energy rose from {:.5} to {:.5} J", hz, e0, top);
        assert!(worst < 5e-3, "{} Hz: the rod stretched {:.2e} m", hz, worst);
    }
}

/// A 1 kg mass on a compliant anchored rod from a static point, no gravity, started stretched
/// by `x0`. Returns the stretch every step.
fn oscillator(k: f32, c_d: f32, m: f32, hz: f32, secs: f32, x0: f32) -> Vec<f32> {
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::ZERO);
    let pivot = w.add_body(iso(Vec3::ZERO, Quat::IDENTITY), Velocity::default(), MassProps::infinite(), false);
    let half = Vec3::splat(0.05);
    let mp = MassProps::from_box_density(half, m / (8.0 * half.x * half.y * half.z));
    let rest = 1.0f32;
    // Anchored 0.05 m behind the COM along the rod, so the anchored rows run with no torque
    let b = w.add_body(iso(Vec3::new(rest + 0.05 + x0, 0.0, 0.0), Quat::IDENTITY), Velocity::default(), mp, true);
    w.set_body_sleep_allowed(b, false);
    let j = w.add_distance_joint_at(pivot, Vec3::ZERO, b, Vec3::new(-0.05, 0.0, 0.0), rest, 1.0 / k);
    w.set_distance_damping(j, c_d);
    (0..(secs * hz) as usize).map(|_| { w.step(1.0 / hz); w.get_body_pose(b).pos.x - 0.05 - rest }).collect()
}

/// Times of the downward zero crossings, interpolated.
fn crossings(x: &[f32], dt: f32) -> Vec<f32> {
    (1..x.len()).filter(|&i| x[i - 1] > 0.0 && x[i] <= 0.0).map(|i| (i as f32 - x[i] / (x[i] - x[i - 1])) * dt).collect()
}

/// Peaks of a decaying oscillation (local maxima above zero).
fn peaks(x: &[f32]) -> Vec<f32> {
    (1..x.len() - 1).filter(|&i| x[i] > 0.0 && x[i] >= x[i - 1] && x[i] > x[i + 1]).map(|i| x[i]).collect()
}

/// A resolved oscillator (omega dt = 0.02 at 240 Hz). The compliant row is position-level only,
/// so it carries implicit Euler's own numerical damping (about 2 pi omega dt of energy per
/// period); the damper is measured against the undamped run's decay. Undamped, it rings at
/// omega. At zeta 0.1, the damped period and the decay per period beyond the undamped run's
/// match the closed form. At zeta 1 it doesn't overshoot.
#[test]
fn the_damper_matches_the_closed_form_where_the_oscillator_is_resolved() {
    let (hz, m) = (240.0f32, 1.0f32);
    let omega = 0.02 * hz;
    let k = m * omega * omega;
    let dt = 1.0 / hz;
    let x0 = 0.01f32;
    let secs = 4.0 * 2.0 * std::f32::consts::PI / omega;
    let x = oscillator(k, 0.0, m, hz, secs, x0);
    let t = crossings(&x, dt);
    let period = (t[t.len() - 1] - t[0]) / (t.len() - 1) as f32;
    let closed = 2.0 * std::f32::consts::PI / omega;
    assert!((period / closed - 1.0).abs() < 0.01, "undamped period {:.5} s, closed form {:.5} s", period, closed);
    let p0 = peaks(&x);
    let numerical = p0[2] / p0[1];
    let zeta = 0.1f32;
    let x = oscillator(k, 2.0 * zeta * m * omega, m, hz, secs, x0);
    let t = crossings(&x, dt);
    let td = t[1] - t[0];
    let closed_d = 2.0 * std::f32::consts::PI / (omega * (1.0 - zeta * zeta).sqrt());
    assert!((td / closed_d - 1.0).abs() < 0.02, "zeta 0.1: damped period {:.5} s, closed form {:.5} s", td, closed_d);
    let p = peaks(&x);
    let ratio = p[2] / p[1] / numerical;
    let closed_r = (-zeta * omega * closed_d).exp();
    println!("numerical decay per period {:.4}; damper {:.4}, closed form {:.4}", numerical, ratio, closed_r);
    assert!((ratio / closed_r - 1.0).abs() < 0.05, "zeta 0.1: the damper's decay per period {:.4}, closed form {:.4}", ratio, closed_r);
    let x = oscillator(k, 2.0 * m * omega, m, hz, secs, x0);
    let under = x.iter().cloned().fold(0.0f32, f32::min);
    // Critical damping is the boundary of overshoot: XPBD's damping term undershoots by 1.4% of
    // the start here (the velocity-row damper it replaced, under 1%)
    assert!(under > -0.02 * x0, "zeta 1 overshot to {:.2e} m from {:.2e}", under, x0);
}

/// Drake's achilles spring (1e6 N/m, 2e3 N s/m) on a 0.5 kg mass at 2 kHz (omega dt = 0.71):
/// overdamped, it settles without overshoot. Undamped, it overshoots: at this omega dt the
/// compliant row's numerical damping is strong, so it doesn't ring for long.
#[test]
fn drakes_achilles_spring_settles_at_2_khz() {
    let (hz, m, k, x0) = (2000.0f32, 0.5f32, 1.0e6f32, 1.0e-3f32);
    let x = oscillator(k, 2.0e3, m, hz, 0.05, x0);
    let under = x.iter().cloned().fold(0.0f32, f32::min);
    assert!(under > -0.05 * x0, "damped: overshot to {:.2e} m", under);
    assert!(x[x.len() - 1].abs() < 1e-3 * x0, "damped: still {:.2e} m after 50 ms", x[x.len() - 1]);
    let x = oscillator(k, 0.0, m, hz, 0.05, x0);
    let under = x.iter().cloned().fold(0.0f32, f32::min);
    println!("undamped at 2 kHz: undershoot {:.3} of the start", under / x0);
    assert!(under < -0.1 * x0, "undamped: undershoot only {:.2e} m", under);
}
