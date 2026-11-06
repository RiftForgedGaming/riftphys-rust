// SPDX-License-Identifier: RiftForged Proprietary License
//! Compliant (Hunt-Crossley) contact, opt in per material pair via `MatPairProps::stiffness`.
use riftphys_core::{Isometry, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::World;

const G: f32 = 9.81;

fn scene(k: f32, d: f32, mu: f32, shape: Shape, pos: Vec3, vel: Vec3) -> (World, riftphys_core::BodyId) {
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::new(0.0, -G, 0.0));
    let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    w.add_collider(g, Shape::Box { hx: 20.0, hy: 0.5, hz: 20.0 }, material(MaterialId::Concrete));
    let mut p = w.pair_props(MaterialId::Steel, MaterialId::Concrete);
    p.stiffness = k;
    p.dissipation = d;
    p.mu_s = mu;
    p.mu_k = mu;
    w.set_pair_props(MaterialId::Steel, MaterialId::Concrete, p);
    let mp = match shape {
        Shape::Sphere { r } => MassProps::from_sphere_density(r, 1.0 / (4.0 / 3.0 * std::f32::consts::PI * r * r * r)),
        Shape::Box { hx, hy, hz } => MassProps::from_box_density(Vec3::new(hx, hy, hz), 1.0 / (8.0 * hx * hy * hz)),
        _ => unreachable!(),
    };
    let b = w.add_body(Isometry { pos, rot: Quat::IDENTITY }, Velocity { lin: vel, ang: Vec3::ZERO }, mp, true);
    w.add_collider(b, shape, material(MaterialId::Steel));
    w.set_body_sleep_allowed(b, false);
    (w, b)
}

/// A 1 kg box resting on its four corner contacts sinks m g / (4 k), and stays there.
#[test]
fn resting_penetration_matches_the_spring() {
    let k = 20000.0;
    let (mut w, b) = scene(k, 1.0, 0.5, Shape::Box { hx: 0.1, hy: 0.1, hz: 0.1 }, Vec3::new(0.0, 0.1, 0.0), Vec3::ZERO);
    for _ in 0..2000 { w.step(1.0 / 1000.0); }
    let sink = 0.1 - w.get_body_pose(b).pos.y;
    let want = G / (4.0 * k);
    assert!((sink - want).abs() < 0.15 * want, "sink {:.6} m, want {:.6}", sink, want);
    assert!(w.get_body_vel(b).lin.length() < 1e-3);
}

/// Without dissipation an impact gives back nearly all its speed; dissipation takes some away,
/// more the faster the impact (Hunt-Crossley).
#[test]
fn dissipation_sets_the_bounce() {
    let bounce = |d: f32, v0: f32| {
        let (mut w, b) = scene(1.0e5, d, 0.0, Shape::Sphere { r: 0.05 }, Vec3::new(0.0, 0.06, 0.0), Vec3::new(0.0, -v0, 0.0));
        let mut out = 0.0f32;
        for _ in 0..400 {
            w.step(1.0 / 4000.0);
            out = out.max(w.get_body_vel(b).lin.y);
        }
        out / v0
    };
    let elastic = bounce(0.0, 2.0);
    let damped = bounce(0.5, 2.0);
    let damped_fast = bounce(0.5, 4.0);
    assert!(elastic > 0.9, "d 0: e {:.3}", elastic);
    assert!(damped < elastic - 0.2, "d 0.5: e {:.3} vs {:.3}", damped, elastic);
    assert!(damped_fast < damped, "faster impacts lose more: {:.3} vs {:.3}", damped_fast, damped);
}

/// Friction on a compliant contact is still Coulomb: a box sliding at v0 stops in v0^2 / (2 mu g).
#[test]
fn friction_uses_the_compliant_normal_force() {
    let (mu, v0) = (0.3, 2.0);
    let (mut w, b) = scene(50000.0, 2.0, mu, Shape::Box { hx: 0.1, hy: 0.1, hz: 0.1 }, Vec3::new(0.0, 0.1 - G / (4.0 * 50000.0), 0.0), Vec3::new(v0, 0.0, 0.0));
    for _ in 0..3000 { w.step(1.0 / 1000.0); }
    let dist = w.get_body_pose(b).pos.x;
    let want = v0 * v0 / (2.0 * mu * G);
    assert!((dist - want).abs() < 0.05 * want, "slid {:.3} m, want {:.3}", dist, want);
}

/// Deterministic like the rigid model: same inputs, same hash.
#[test]
fn compliant_steps_replay_bit_exactly() {
    let run = || {
        let (mut w, _) = scene(10800.0, 0.4, 0.2, Shape::Box { hx: 0.05, hy: 0.05, hz: 0.05 }, Vec3::new(0.0, 0.3, 0.0), Vec3::new(1.0, 0.0, 0.5));
        for _ in 0..1500 { w.step(1.0 / 1480.0); }
        w.step_hash()
    };
    assert_eq!(run(), run());
}
