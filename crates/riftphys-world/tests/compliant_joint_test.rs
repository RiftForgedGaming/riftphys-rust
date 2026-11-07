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
    // Static sag: the spring acts on the hinge angle in radians (a sprung hinge is a hinge), so
    // theta = c x gravity's torque m g 0.5 cos theta
    let mut theta = 0.0f32;
    for _ in 0..200 { theta = 0.5 * theta + 0.5 * c * m * G * 0.5 * theta.cos(); }
    let rot = Quat::from_rotation_z(-theta);
    let rod = w.add_body(iso(Vec3::new(0.0, 10.0, 0.0) + rot * Vec3::new(0.5, 0.0, 0.0), rot), Velocity::default(), mp, true);
    // Kept awake: a sleeping body freezes wherever it is, which hid a 1% offset at 240 Hz
    w.set_body_sleep_allowed(rod, false);
    // hinge_axis left unset: a sprung hinge is recognised from its rows (spring on z, x and y rigid)
    let mut j = D6Joint { a: p, b: rod, fa: iso(Vec3::ZERO, Quat::IDENTITY), fb: iso(Vec3::new(-0.5, 0.0, 0.0), Quat::IDENTITY),
                          t: Default::default(), r: Default::default(), hinge_axis: None };
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

/// A soft spring that sags about 1 rad: here 2 sin(theta / 2) would be 4% short of theta, so
/// the spring must act on the angle itself.
#[test]
fn torsional_spring_large_sag_is_in_radians() {
    // Measured (kept awake): 0.0004% at 60 Hz, 0.87% at 240 Hz. On 2 sin(theta / 2) it was 3.3%
    // and 4.1%. The 240 Hz offset is not the spring law: it doesn't change with the iteration
    // count, the mode or an unrounded multiplier, and the rod freezes once its motion per step
    // falls below the 1e-6 pose grid (JOINT_CONDITIONING_SPEC C6), so 240 Hz gets a looser gate
    // until C6.
    for (hz, gate) in [(60.0f32, 0.0025f32), (240.0, 0.015)] {
        let worst = torsion_spring(0.15, hz, 10.0);
        assert!(worst < gate, "{} Hz: sag departed {:.2}% from static", hz, worst * 100.0);
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
    w.set_body_sleep_allowed(bob, false);
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

/// A sprung hinge twisted past 90 degrees and knocked sideways (no gravity): a 1 m rod hinged at
/// its end about z, a spring on z (c = 0.15), started 2.5 rad round and spinning 0.3 rad/s about
/// x. Its alignment rows must measure only the misalignment of the hinge axis; measuring the
/// quaternion error instead (which mixes in the twist) pushed the wrong way past ~90 degrees.
/// Returns (worst tilt of the hinge axis in degrees, worst anchor gap in m) over 10 s.
fn sprung_hinge_past_90(align_c: f32, hinge_axis: Option<usize>, hz: f32) -> (f32, f32) {
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::ZERO);
    let p = pin(&mut w);
    let half = Vec3::new(0.5, 0.025, 0.025);
    let rot = Quat::from_rotation_z(2.5);
    let rod = w.add_body(iso(Vec3::new(0.0, 10.0, 0.0) + rot * Vec3::new(0.5, 0.0, 0.0), rot),
                         Velocity { lin: Vec3::ZERO, ang: Vec3::new(0.3, 0.0, 0.0) }, MassProps::from_box_density(half, 1000.0), true);
    w.set_body_sleep_allowed(rod, false);
    let mut j = D6Joint { a: p, b: rod, fa: iso(Vec3::ZERO, Quat::IDENTITY), fb: iso(Vec3::new(-0.5, 0.0, 0.0), Quat::IDENTITY),
                          t: Default::default(), r: Default::default(), hinge_axis };
    for i in 0..3 { j.t[i].enabled = true; j.r[i].enabled = true; }
    j.r[0].compliance = align_c;
    j.r[1].compliance = align_c;
    j.r[2] = D6Axis { enabled: true, compliance: 0.15, ..Default::default() };
    w.add_d6_joint(j);
    let (mut tilt, mut gap) = (0.0f32, 0.0f32);
    for _ in 0..(10.0 * hz) as usize {
        w.step(1.0 / hz);
        let q = w.get_body_pose(rod);
        tilt = tilt.max((q.rot * Vec3::Z).dot(Vec3::Z).clamp(-1.0, 1.0).acos().to_degrees());
        gap = gap.max(((q.pos + q.rot * Vec3::new(-0.5, 0.0, 0.0)) - Vec3::new(0.0, 10.0, 0.0)).length());
    }
    (tilt, gap)
}

#[test]
fn sprung_hinge_stays_aligned_past_90_degrees() {
    // (alignment-row compliance, hinge_axis): rigid rows, a compliance that rounds to zero on the
    // solver's grid (both recognised from the rows), and soft rows (needs hinge_axis set)
    for (align_c, axis) in [(0.0f32, None), (1.0e-9, None), (1.0e-3, Some(2usize))] {
        for hz in [60.0f32, 240.0] {
            let (tilt, gap) = sprung_hinge_past_90(align_c, axis, hz);
            assert!(tilt < 1.0 && gap < 0.001, "alignment c {:e}, {} Hz: hinge axis tilted {:.2} deg, joint apart {:.4} m", align_c, hz, tilt, gap);
        }
    }
}
