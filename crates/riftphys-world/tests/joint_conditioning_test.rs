// SPDX-License-Identifier: RiftForged Proprietary License
//! Joint rows are measured afresh before each one is solved (JOINT_CONDITIONING_SPEC C1). Measured
//! once per joint instead, the rows of a joint with a lever arm overcorrected together and cycled
//! between two wrong states: a hinge drive to 57.3 deg alternated between 24 deg (after an even
//! number of iterations) and 98 deg (odd), with the joint 0.34-0.48 m apart; a cube on a long arm
//! gained 23x its energy at a stop; a double pendulum gained 1757x.
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

/// A 1 m, 2.5 kg rod hinged at its end about z, lying along +x, with a rigid position drive
/// (kp = 0: until drives become spring-dampers they ignore their gains) to 1 rad.
fn driven_rod(proj: JointProjection, pos_iters: u32, gravity: f32) -> (World, BodyId) {
    let mut w = World::with_capacity(4, 4);
    w.set_joint_projection(proj);
    w.set_solver_iterations(20, pos_iters);
    w.set_gravity(Vec3::new(0.0, -gravity, 0.0));
    let p = w.add_body(iso(Vec3::new(0.0, 10.0, 0.0)), Velocity::default(), MassProps::infinite(), false);
    let r = w.add_body(iso(Vec3::new(0.5, 10.0, 0.0)), Velocity::default(), MassProps::from_box_density(Vec3::new(0.5, 0.025, 0.025), 1000.0), true);
    let j = w.add_hinge_joint(p, r, iso(Vec3::ZERO), iso(Vec3::new(-0.5, 0.0, 0.0)), 2);
    w.set_d6_drive_r(j, 2, Some(Drive { target: 1.0, kp: 0.0, kd: 0.0, max_impulse: 1000.0, vel_mode: false }));
    (w, r)
}

fn anchor_gap(w: &World, r: BodyId) -> f32 {
    let p = w.get_body_pose(r);
    ((p.pos + p.rot * Vec3::new(-0.5, 0.0, 0.0)) - Vec3::new(0.0, 10.0, 0.0)).length()
}

/// In a world step the drive holds its target within 0.1 s, at an even and an odd iteration count.
#[test]
fn hinge_drive_reaches_its_target_at_any_iteration_count() {
    let target = 1f32.to_degrees();
    for proj in [JointProjection::Pbd, JointProjection::Shake] {
        for hz in [60.0f32, 240.0] {
            for gravity in [0.0f32, G] {
                for pos_iters in [20u32, 21] {
                    let (mut w, r) = driven_rod(proj, pos_iters, gravity);
                    let mut worst_after = 0.0f32;
                    let mut worst_gap = 0.0f32;
                    for k in 0..(1.0 * hz) as usize {
                        w.step(1.0 / hz);
                        if k as f32 / hz >= 0.1 {
                            worst_after = worst_after.max((hinge_angle(&w, r).to_degrees() - target).abs());
                            worst_gap = worst_gap.max(anchor_gap(&w, r));
                        }
                    }
                    assert!(worst_after < 0.1, "{:?} {} Hz g {} iters {}: off target by {:.3} deg after 0.1 s", proj, hz, gravity, pos_iters, worst_after);
                    assert!(worst_gap < 0.001, "{:?} {} Hz g {} iters {}: joint apart by {:.4} m", proj, hz, gravity, pos_iters, worst_gap);
                }
            }
        }
    }
}

/// The position solve alone converges, and doesn't depend on whether it stops on an even or an
/// odd iteration.
#[test]
fn hinge_drive_position_solve_converges() {
    for n in [1000u32, 1001] {
        let (mut w, r) = driven_rod(JointProjection::Pbd, 20, 0.0);
        w.solve_joints(1.0 / 240.0, n);
        let err = (hinge_angle(&w, r).to_degrees() - 1f32.to_degrees()).abs();
        assert!(err < 0.1, "{} iterations: off target by {:.3} deg", n, err);
        assert!(anchor_gap(&w, r) < 0.001, "{} iterations: joint apart by {:.4} m", n, anchor_gap(&w, r));
    }
}

fn energy(w: &World, b: BodyId, mp: &MassProps) -> f32 {
    let p = w.get_body_pose(b);
    let v = w.get_body_vel(b);
    let r = riftphys_core::Mat3::from_quat(p.rot);
    mp.mass * G * p.pos.y + 0.5 * mp.mass * v.lin.length_squared() + 0.5 * v.ang.dot((r * mp.inertia * r.transpose()) * v.ang)
}

/// A 2 m, 80 kg cube on a 5 m arm swings into a hinge stop: the stop takes energy, never adds it.
#[test]
fn cube_on_long_arm_into_a_stop_gains_no_energy() {
    for proj in [JointProjection::Pbd, JointProjection::Shake] {
        for (hz, max_gap) in [(62.5f32, 0.1f32), (240.0, 0.03)] {
            let mut w = World::with_capacity(4, 4);
            w.set_joint_projection(proj);
            w.set_gravity(Vec3::new(0.0, -G, 0.0));
            let p = w.add_body(iso(Vec3::new(0.0, 20.0, 0.0)), Velocity::default(), MassProps::infinite(), false);
            let mp = MassProps::from_box_density(Vec3::splat(1.0), 10.0);
            let b = w.add_body(iso(Vec3::new(5.0, 20.0, 0.0)), Velocity::default(), mp, true);
            let j = w.add_hinge_joint(p, b, iso(Vec3::ZERO), iso(Vec3::new(-5.0, 0.0, 0.0)), 2);
            w.set_d6_limit_r(j, 2, Some(D6Limit { min: -1.8, max: 0.1 }));
            let floor = mp.mass * G * 15.0;
            let e0 = energy(&w, b, &mp) - floor;
            let (mut peak, mut gap) = (0.0f32, 0.0f32);
            for _ in 0..(5.0 * hz) as usize {
                w.step(1.0 / hz);
                peak = peak.max((energy(&w, b, &mp) - floor) / e0);
                let q = w.get_body_pose(b);
                gap = gap.max(((q.pos + q.rot * Vec3::new(-5.0, 0.0, 0.0)) - Vec3::new(0.0, 20.0, 0.0)).length());
            }
            assert!(peak <= 1.01, "{:?} {} Hz: energy peaked at {:.3}x", proj, hz, peak);
            assert!(gap < max_gap, "{:?} {} Hz: joint apart by {:.4} m", proj, hz, gap);
        }
    }
}

/// Double pendulum of 0.2 m cubes on 1 m massless arms (SHAKE, the default, 20 iterations),
/// released horizontal. An interim gate (JOINT_CONDITIONING_SPEC C1), looser than parity's
/// +-0.02: the block solve (C5) tightens it.
#[test]
fn double_pendulum_of_small_cubes_keeps_its_energy() {
    for (hz, lo_ok, hi_ok) in [(62.5f32, 0.93f32, 1.05f32), (240.0, 0.98, 1.04)] {
        let mut w = World::with_capacity(8, 8);
        w.set_gravity(Vec3::new(0.0, -G, 0.0));
        let p = w.add_body(iso(Vec3::new(0.0, 10.0, 0.0)), Velocity::default(), MassProps::infinite(), false);
        let mp = MassProps::from_box_density(Vec3::splat(0.1), 1000.0);
        let a = w.add_body(iso(Vec3::new(1.0, 10.0, 0.0)), Velocity::default(), mp, true);
        let b = w.add_body(iso(Vec3::new(2.0, 10.0, 0.0)), Velocity::default(), mp, true);
        w.add_hinge_joint(p, a, iso(Vec3::ZERO), iso(Vec3::new(-1.0, 0.0, 0.0)), 2);
        w.add_hinge_joint(a, b, iso(Vec3::ZERO), iso(Vec3::new(-1.0, 0.0, 0.0)), 2);
        let floor = mp.mass * G * 17.0;
        let e = |w: &World| energy(w, a, &mp) + energy(w, b, &mp) - floor;
        let e0 = e(&w);
        let (mut lo, mut hi) = (1.0f32, 1.0f32);
        for _ in 0..(10.0 * hz) as usize {
            w.step(1.0 / hz);
            let r = e(&w) / e0;
            lo = lo.min(r);
            hi = hi.max(r);
        }
        assert!(lo >= lo_ok && hi <= hi_ok, "{} Hz: energy ranged {:.3}..{:.3}, gate {}..{}", hz, lo, hi, lo_ok, hi_ok);
    }
}
