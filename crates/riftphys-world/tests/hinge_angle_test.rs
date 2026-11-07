// SPDX-License-Identifier: RiftForged Proprietary License
//! Hinge limits are in radians: a hinge measures its own twist angle, not
//! the small-angle 2 sin(angle / 2) (under which a 2.2 limit was out of reach and -1.5 sat at
//! -97 degrees).
use riftphys_articulation::bodies::D6Limit;
use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::MassProps;
use riftphys_world::world::World;

fn iso(pos: Vec3) -> Isometry { Isometry { pos, rot: Quat::IDENTITY } }

/// A 1 m, 2.5 kg rod hinged at its end about z, lying along +x (hinge angle 0). Gravity swings
/// it toward negative angles.
fn rod(gravity: f32) -> (World, BodyId, riftphys_core::JointId) {
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::new(0.0, -gravity, 0.0));
    let p = w.add_body(iso(Vec3::new(0.0, 10.0, 0.0)), Velocity::default(), MassProps::infinite(), false);
    let r = w.add_body(iso(Vec3::new(0.5, 10.0, 0.0)), Velocity::default(), MassProps::from_box_density(Vec3::new(0.5, 0.025, 0.025), 1000.0), true);
    let j = w.add_hinge_joint(p, r, iso(Vec3::ZERO), iso(Vec3::new(-0.5, 0.0, 0.0)), 2);
    (w, r, j)
}

fn angle(w: &World, r: BodyId) -> f32 {
    let (axis, a) = w.get_body_pose(r).rot.to_axis_angle();
    if a.abs() < 1e-6 { 0.0 } else { a * axis.z.signum() }
}

/// Returns (furthest angle reached, angle after `secs`), both in degrees.
fn swing(limit: D6Limit, secs: f32) -> (f32, f32) {
    let (mut w, r, j) = rod(9.81);
    w.set_d6_limit_r(j, 2, Some(limit));
    let mut furthest = 0.0f32;
    for _ in 0..(secs * 240.0) as usize {
        w.step(1.0 / 240.0);
        let a = angle(&w, r);
        if a.abs() > furthest.abs() { furthest = a; }
    }
    (furthest.to_degrees(), angle(&w, r).to_degrees())
}

/// A stop at -1 rad (-57.3 degrees) above the bottom of the swing: the rod falls onto it and
/// rests there.
#[test]
fn hinge_rests_on_a_limit_in_radians() {
    let (furthest, rest) = swing(D6Limit { min: -1.0, max: 1.0 }, 5.0);
    let stop = -1f32.to_degrees();
    assert!((rest - stop).abs() < 0.5, "rested at {:.2} deg, stop at {:.2}", rest, stop);
    assert!(furthest > stop - 1.0, "overshot to {:.2} deg", furthest);
}

/// A stop at -2.2 rad (-126 degrees), past the bottom of the swing: reachable now (2 sin(a / 2)
/// never exceeds 2, so a 2.2 limit used to be unreachable).
#[test]
fn hinge_limit_beyond_two_radians_holds() {
    let (furthest, _) = swing(D6Limit { min: -2.2, max: 0.1 }, 3.0);
    let stop = -2.2f32.to_degrees();
    assert!(furthest > stop - 1.0 && furthest < stop + 1.0, "furthest {:.2} deg, stop at {:.2}", furthest, stop);
}
