// SPDX-License-Identifier: RiftForged Proprietary License
//! Compliant joints (XPBD compliance, inverse stiffness): a joint started at its static sag under
//! gravity stays there. The multiplier is per step; accumulated across steps it made the joint
//! stiffer every step, and without one the stiffness depended on the iteration count.
use riftphys_articulation::bodies::{D6Axis, D6Joint};
use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::MassProps;
use riftphys_world::world::World;

const G: f32 = 9.81;

fn world() -> World {
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::new(0.0, -G, 0.0));
    w
}

fn iso(pos: Vec3, rot: Quat) -> Isometry { Isometry { pos, rot } }

fn pin(w: &mut World) -> BodyId {
    w.add_body(iso(Vec3::new(0.0, 10.0, 0.0), Quat::IDENTITY), Velocity::default(), MassProps::infinite(), false)
}

/// A 1 m, 2.5 kg rod held out horizontally by a torsional spring about z (compliance `c`,
/// rad per N m), started at its static sag. Returns the worst departure from the sag over
/// `secs`, relative to the sag.
fn torsion_spring(c: f32, hz: f32, secs: f32) -> f32 {
    let mut w = world();
    let p = pin(&mut w);
    let half = Vec3::new(0.5, 0.025, 0.025);
    let mp = MassProps::from_box_density(half, 1000.0);
    let m = 8.0 * half.x * half.y * half.z * 1000.0;
    // Static sag: the spring measures 2 sin(theta / 2) about z; gravity's torque is m g 0.5 cos theta
    let mut theta = 0.0f32;
    for _ in 0..50 { theta = 2.0 * (c * m * G * 0.5 * theta.cos() / 2.0).asin(); }
    let rot = Quat::from_rotation_z(-theta);
    let rod = w.add_body(iso(Vec3::new(0.0, 10.0, 0.0) + rot * Vec3::new(0.5, 0.0, 0.0), rot), Velocity::default(), mp, true);
    let mut j = D6Joint { a: p, b: rod, fa: iso(Vec3::ZERO, Quat::IDENTITY), fb: iso(Vec3::new(-0.5, 0.0, 0.0), Quat::IDENTITY),
                          t: Default::default(), r: Default::default() };
    for i in 0..3 { j.t[i].enabled = true; j.r[i].enabled = true; }
    j.r[2] = D6Axis { enabled: true, compliance: c, ..Default::default() };
    w.add_d6_joint(j);
    let mut worst = 0.0f32;
    for _ in 0..(secs * hz) as usize {
        w.step(1.0 / hz);
        let (axis, ang) = w.get_body_pose(rod).rot.to_axis_angle();
        let sag = -ang * axis.z.signum();
        worst = worst.max((sag - theta).abs() / theta);
    }
    worst
}

#[test]
fn torsional_spring_holds_its_static_sag() {
    for hz in [60.0f32, 240.0] {
        let worst = torsion_spring(0.008, hz, 10.0);
        assert!(worst < 0.03, "{} Hz: sag departed {:.1}% from static", hz, worst * 100.0);
    }
}

/// A 2 kg weight on a compliant distance joint (compliance `c`, m per N), started at its static
/// stretch: rest + c m g.
fn hanging_spring(c: f32, hz: f32, iters: u32, secs: f32) -> f32 {
    let mut w = world();
    w.set_solver_iterations(iters, iters);
    let p = pin(&mut w);
    let m = 2.0f32;
    let stretch = c * m * G;
    let mp = MassProps::from_box_density(Vec3::splat(0.05), m / 0.001);
    let bob = w.add_body(iso(Vec3::new(0.0, 9.0 - stretch, 0.0), Quat::IDENTITY), Velocity::default(), mp, true);
    w.add_distance_joint(p, bob, 1.0, c);
    let mut worst = 0.0f32;
    for _ in 0..(secs * hz) as usize {
        w.step(1.0 / hz);
        let s = (10.0 - w.get_body_pose(bob).pos.y) - 1.0;
        worst = worst.max((s - stretch).abs() / stretch);
    }
    worst
}

/// The stretch is set by the compliance, not by the step rate or iteration count.
#[test]
fn compliant_distance_joint_holds_its_static_stretch() {
    for (hz, iters) in [(60.0f32, 20u32), (240.0, 20), (240.0, 4)] {
        let worst = hanging_spring(0.0005, hz, iters, 10.0);
        assert!(worst < 0.03, "{} Hz, {} iterations: stretch departed {:.1}% from static", hz, iters, worst * 100.0);
    }
}

/// Rigid rows are unaffected: compliance 0 still holds the joint shut.
#[test]
fn rigid_distance_joint_stays_rigid() {
    let mut w = world();
    let p = pin(&mut w);
    let bob = w.add_body(iso(Vec3::new(1.0, 10.0, 0.0), Quat::IDENTITY), Velocity::default(), MassProps::from_box_density(Vec3::splat(0.05), 1000.0), true);
    w.add_distance_joint(p, bob, 1.0, 0.0);
    let mut worst = 0.0f32;
    for _ in 0..600 {
        w.step(1.0 / 60.0);
        worst = worst.max(((w.get_body_pose(bob).pos - Vec3::new(0.0, 10.0, 0.0)).length() - 1.0).abs());
    }
    assert!(worst < 0.002, "rigid distance joint stretched {:.4} m", worst);
}
