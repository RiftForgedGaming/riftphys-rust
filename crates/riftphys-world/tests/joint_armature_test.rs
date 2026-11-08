// SPDX-License-Identifier: RiftForged Proprietary License
//! PARITY_SPEC W4b: axis-only armature, a rotor geared to a hinge (inertia J_a about the axis,
//! no gravity). Checked against reduced-coordinate closed forms: a torque accelerates the hinge
//! at tau / (I + J_a); a pendulum swings with period 2 pi sqrt((I_pivot + J_a) / (m g d)).
use riftphys_core::{BodyId, Isometry, JointId, Quat, Vec3, Velocity};
use riftphys_geom::MassProps;
use riftphys_world::world::World;

fn iso(pos: Vec3) -> Isometry { Isometry { pos, rot: Quat::IDENTITY } }

const HALF: Vec3 = Vec3::new(0.5, 0.1, 0.1);
const MASS: f32 = 2.0;
fn izz() -> f32 { MASS / 3.0 * (HALF.x * HALF.x + HALF.y * HALF.y) }

/// A 2 kg bar hinged about z through its centre to a static anchor, no gravity.
fn spinner() -> (World, BodyId, JointId) {
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::ZERO);
    let p = w.add_body(iso(Vec3::ZERO), Velocity::default(), MassProps::infinite(), false);
    let b = w.add_body(iso(Vec3::ZERO), Velocity::default(), MassProps::from_box_density(HALF, MASS / (8.0 * HALF.x * HALF.y * HALF.z)), true);
    w.set_body_sleep_allowed(b, false);
    let j = w.add_hinge_joint(p, b, iso(Vec3::ZERO), iso(Vec3::ZERO), 2);
    (w, b, j)
}

#[test]
fn torque_accelerates_at_i_plus_armature() {
    let i = izz();
    let tau = 0.3f32;
    for hz in [60.0f32, 240.0] {
        for host_writes_velocity in [false, true] {
            let (mut w, b, j) = spinner();
            w.set_d6_armature(j, 2, i); // doubles the inertia about the hinge
            let t = 1.0;
            for _ in 0..(t * hz) as usize {
                // apply_torque takes an angular impulse: torque x dt
                w.apply_torque(b, Vec3::new(0.0, 0.0, tau / hz));
                if host_writes_velocity { let v = w.get_body_vel(b); w.set_body_vel(b, v); }
                w.step(1.0 / hz);
            }
            let want = tau * t / (2.0 * i);
            let got = w.get_body_vel(b).ang.z;
            assert!((got - want).abs() < 0.01 * want, "{} Hz (host writes velocity: {}): w {:.4}, want {:.4} (without armature {:.4})", hz, host_writes_velocity, got, want, 2.0 * want);
        }
    }
}

/// A 1 m, 2.5 kg rod hinged at its end about z, released from `a0` rad. Returns the mean period
/// over the first 4 swings and the relative energy change (rotor included) after `secs`.
fn pendulum(armature: f32, a0: f32, hz: f32, secs: f32) -> (f32, f32) {
    let mut w = World::with_capacity(4, 4);
    let g = 9.81;
    w.set_gravity(Vec3::new(0.0, -g, 0.0));
    let p = w.add_body(iso(Vec3::new(0.0, 10.0, 0.0)), Velocity::default(), MassProps::infinite(), false);
    let half = Vec3::new(0.5, 0.025, 0.025);
    let m = 2.5f32;
    // Hanging straight down, rotated by a0 about the pivot
    let q = Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2 + a0);
    let pos = Vec3::new(0.0, 10.0, 0.0) + q * Vec3::new(0.5, 0.0, 0.0);
    let r = w.add_body(Isometry { pos, rot: q }, Velocity::default(), MassProps::from_box_density(half, m / (8.0 * half.x * half.y * half.z)), true);
    w.set_body_sleep_allowed(r, false);
    let j = w.add_hinge_joint(p, r, Isometry { pos: Vec3::ZERO, rot: Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2) }, iso(Vec3::new(-0.5, 0.0, 0.0)), 2);
    if armature > 0.0 { w.set_d6_armature(j, 2, armature); }
    let i_pivot = m / 3.0 * (half.x * half.x + half.y * half.y) + m * 0.25;
    let energy = |w: &World| {
        let (pose, v) = (w.get_body_pose(r), w.get_body_vel(r));
        let wz = v.ang.z;
        0.5 * (i_pivot + armature) * wz * wz + m * g * (pose.pos.y - 10.0)
    };
    let e0 = energy(&w);
    let angle = |w: &World| { let d = w.get_body_pose(r).pos - Vec3::new(0.0, 10.0, 0.0); d.x.atan2(-d.y) };
    let (mut last, mut crossings) = (angle(&w), Vec::new());
    for k in 1..=(secs * hz) as usize {
        w.step(1.0 / hz);
        let a = angle(&w);
        if (a > 0.0) != (last > 0.0) {
            // Linear interpolation of the zero crossing
            let f = last / (last - a);
            crossings.push((k as f32 - 1.0 + f) / hz);
        }
        last = a;
    }
    let period = if crossings.len() >= 9 { (crossings[8] - crossings[0]) / 4.0 } else { f32::NAN };
    (period, (energy(&w) - e0) / e0.abs())
}

#[test]
fn pendulum_period_includes_the_armature() {
    let (m, g, d) = (2.5f32, 9.81f32, 0.5f32);
    let i_pivot = m / 3.0 * (0.25 + 0.025 * 0.025) + m * 0.25;
    for hz in [60.0f32, 240.0] {
        for ja in [0.0f32, 0.5] {
            let (period, _) = pendulum(ja, 0.05, hz, 12.0);
            let want = 2.0 * std::f32::consts::PI * ((i_pivot + ja) / (m * g * d)).sqrt();
            assert!((period - want).abs() < 0.01 * want, "{} Hz, J_a {}: period {:.4} s, want {:.4}", hz, ja, period, want);
        }
    }
}

#[test]
fn armature_keeps_energy_as_well_as_without() {
    for hz in [60.0f32, 240.0] {
        let (_, e_plain) = pendulum(0.0, 0.5, hz, 10.0);
        let (_, e_arm) = pendulum(0.5, 0.5, hz, 10.0);
        assert!(e_arm.abs() <= e_plain.abs() + 0.01, "{} Hz: energy change {:.4} with armature, {:.4} without", hz, e_arm, e_plain);
    }
}

#[test]
fn rollback_with_armature_replays() {
    let (mut w, b, j) = spinner();
    w.set_d6_armature(j, 2, 0.1);
    for _ in 0..30 { w.apply_torque(b, Vec3::new(0.0, 0.0, 0.01)); w.step(1.0 / 60.0); }
    let (pose, vel, carried) = (w.get_body_pose(b), w.get_body_vel(b), w.carried_state());
    let h0 = w.step_hash();
    // The host also writes the bar's velocity before each step, as a velocity-driven controller
    // would: after a restore (pose, velocity, carried state) that must not restart the rotor
    let run = |w: &mut World| -> Vec<[u8; 32]> { (0..60).map(|_| {
        let mut v = w.get_body_vel(b);
        v.ang.z += 0.001;
        w.set_body_vel(b, v);
        w.apply_torque(b, Vec3::new(0.0, 0.0, 0.01));
        w.step(1.0 / 60.0);
        w.step_hash()
    }).collect() };
    let first = run(&mut w);
    w.set_body_pose(b, pose);
    w.set_body_vel(b, vel);
    w.restore_carried_state(&carried);
    assert_eq!(w.step_hash(), h0, "restore hashes differently");
    assert_eq!(first, run(&mut w), "replay with armature diverged");
    // The rotor is in the hash: after a torque the links move on and the rotor doesn't, until
    // a reset sets it to the links' speed, which is a different state
    let after_torque = |w: &mut World, reset: bool| {
        w.set_body_pose(b, pose);
        w.set_body_vel(b, vel);
        w.restore_carried_state(&carried);
        w.apply_torque(b, Vec3::new(0.0, 0.0, 0.05));
        if reset { w.reset_d6_rotor(j); }
        w.step_hash()
    };
    assert_ne!(after_torque(&mut w, false), after_torque(&mut w, true), "the rotor isn't in the hash");
}

#[test]
fn pose_then_velocity_reset_stops_the_rotor() {
    // A hinge turning at 3 rad/s is reset home: pose, then velocity 0. The rotor must restart at
    // 0 too, or the first step pulls the bar back up toward 3 rad/s by J_a / (I + J_a)
    for hz in [60.0f32, 240.0] {
        let (mut w, b, j) = spinner();
        w.set_d6_armature(j, 2, izz());
        w.set_body_vel(b, Velocity { lin: Vec3::ZERO, ang: Vec3::new(0.0, 0.0, 3.0) });
        w.reset_d6_rotor(j);
        for _ in 0..30 { w.step(1.0 / hz); }
        assert!((w.get_body_vel(b).ang.z - 3.0).abs() < 1e-3, "{} Hz: not spinning before the reset", hz);
        w.set_body_pose(b, iso(Vec3::ZERO));
        w.set_body_vel(b, Velocity::default());
        for _ in 0..10 { w.step(1.0 / hz); }
        let wz = w.get_body_vel(b).ang.z;
        assert!(wz.abs() < 1e-4, "{} Hz: the bar left its reset at {:.4} rad/s", hz, wz);
    }
}

#[test]
fn a_d6_joint_built_with_armature_starts_its_rotor() {
    // add_d6_joint with armature already set: no lazy first step, so the first torque counts
    let i = izz();
    let tau = 0.3f32;
    for hz in [60.0f32, 240.0] {
        let mut w = World::with_capacity(4, 4);
        w.set_gravity(Vec3::ZERO);
        let p = w.add_body(iso(Vec3::ZERO), Velocity::default(), MassProps::infinite(), false);
        let b = w.add_body(iso(Vec3::ZERO), Velocity::default(), MassProps::from_box_density(HALF, MASS / (8.0 * HALF.x * HALF.y * HALF.z)), true);
        w.set_body_sleep_allowed(b, false);
        let mut d = riftphys_articulation::bodies::Generic6Dof { a: p, b, fa: iso(Vec3::ZERO), fb: iso(Vec3::ZERO), t: Default::default(), r: Default::default(), hinge_axis: Some(2) };
        for ax in 0..3 { d.t[ax].enabled = true; d.r[ax].enabled = ax != 2; }
        d.r[2].armature = i;
        w.add_d6_joint(d);
        for _ in 0..hz as usize { w.apply_torque(b, Vec3::new(0.0, 0.0, tau / hz)); w.step(1.0 / hz); }
        let want = tau / (2.0 * i);
        let got = w.get_body_vel(b).ang.z;
        assert!((got - want).abs() < 0.01 * want, "{} Hz: w {:.4}, want {:.4}", hz, got, want);
    }
}
