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

// ── Implicit compliant contact (PARITY_SPEC W4e) ──

/// A 1 kg sphere on one compliant contact rests at F / k within 1%, at 60 Hz, 240 Hz and 2 kHz.
#[test]
fn implicit_contact_rests_at_force_over_stiffness() {
    let (k, r) = (20000.0f32, 0.05f32);
    for hz in [60.0f32, 240.0, 2000.0] {
        let (mut w, b) = scene(k, 1.0, 0.5, Shape::Sphere { r }, Vec3::new(0.0, r, 0.0), Vec3::ZERO);
        for _ in 0..(3.0 * hz) as usize { w.step(1.0 / hz); }
        let sink = r - w.get_body_pose(b).pos.y;
        let want = G / k;
        assert!((sink / want - 1.0).abs() < 0.01, "{} Hz: sank {:.6} m, F / k = {:.6}", hz, sink, want);
    }
}

/// Hunt-Crossley's restitution for a 1 kg body hitting at `v_in`: the contact ODE
/// m x'' = -k x (1 + d x') (force never pulling) integrated by RK4 at 1 us.
fn hunt_crossley_e(k: f64, d: f64, v_in: f64) -> f64 {
    let f = |x: f64, v: f64| -> f64 { if x <= 0.0 { 0.0 } else { -(k * x * (1.0 + d * v)).max(0.0) } };
    let (mut x, mut v, h) = (0.0f64, v_in, 1.0e-6);
    loop {
        let (k1x, k1v) = (v, f(x, v));
        let (k2x, k2v) = (v + 0.5 * h * k1v, f(x + 0.5 * h * k1x, v + 0.5 * h * k1v));
        let (k3x, k3v) = (v + 0.5 * h * k2v, f(x + 0.5 * h * k2x, v + 0.5 * h * k2v));
        let (k4x, k4v) = (v + h * k3v, f(x + h * k3x, v + h * k3v));
        x += h / 6.0 * (k1x + 2.0 * k2x + 2.0 * k3x + k4x);
        v += h / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
        if x < 0.0 { return -v / v_in; }
    }
}

/// The bounce off a compliant contact, gravity off, at 1 m/s, against Hunt-Crossley's own ODE
/// (`hunt_crossley_e`). At 2 kHz with k 1000 N/m on 1 kg (omega dt 0.016) within 5% (the
/// implicit spring's own dissipation is about 2.5% there), with the damping linearised at the
/// predicted mid-step penetration (the default) and at the step-start penetration (reported); at
/// 240 Hz reported only (implicit Euler on the contact spring dissipates on its own: D9).
#[test]
fn implicit_contact_bounces_as_hunt_crossley() {
    let (r, v_in) = (0.05f32, 1.0f32);
    let bounce = |k: f32, d: f32, hz: f32, at_start: bool| {
        let mut w = World::with_capacity(4, 4);
        w.set_gravity(Vec3::ZERO);
        w.set_compliant_damping_at_start(at_start);
        let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
        w.add_collider(g, Shape::Box { hx: 20.0, hy: 0.5, hz: 20.0 }, material(MaterialId::Concrete));
        let mut p = w.pair_props(MaterialId::Steel, MaterialId::Concrete);
        p.stiffness = k;
        p.dissipation = d;
        p.mu_s = 0.0;
        p.mu_k = 0.0;
        w.set_pair_props(MaterialId::Steel, MaterialId::Concrete, p);
        let mp = MassProps::from_sphere_density(r, 1.0 / (4.0 / 3.0 * std::f32::consts::PI * r * r * r));
        let b = w.add_body(Isometry { pos: Vec3::new(0.0, r + 0.002, 0.0), rot: Quat::IDENTITY }, Velocity { lin: Vec3::new(0.0, -v_in, 0.0), ang: Vec3::ZERO }, mp, true);
        w.add_collider(b, Shape::Sphere { r }, material(MaterialId::Steel));
        w.set_body_sleep_allowed(b, false);
        for _ in 0..(0.5 * hz) as usize { w.step(1.0 / hz); }
        w.get_body_vel(b).lin.y / v_in
    };
    for (k, d) in [(1000.0f32, 0.0f32), (1000.0, 0.1), (1000.0, 0.3), (10000.0, 0.15)] {
        let want = hunt_crossley_e(k as f64, d as f64, v_in as f64) as f32;
        let (mid, start) = (bounce(k, d, 2000.0, false), bounce(k, d, 2000.0, true));
        let (mid240, start240) = (bounce(k, d, 240.0, false), bounce(k, d, 240.0, true));
        println!("k {} d {:.2}: HC e {:.3}; 2 kHz e {:.3} (step-start damping {:.3}); 240 Hz e {:.3} (step-start {:.3})", k, d, want, mid, start, mid240, start240);
        if k <= 1000.0 { assert!((mid / want - 1.0).abs() < 0.05, "k {} d {}: e {:.3} at 2 kHz, Hunt-Crossley {:.3}", k, d, mid, want); }
    }
}

/// A 0.15 kg toe link on a hinge whose tip sphere lands at 2 m/s on compliant ground, at 240 Hz:
/// bounded with k at 9400 N/m and at x100. (The explicit form W4e replaced spun this up to
/// 730,399 rad/s at 9400 N/m and 4.5e9 at x100.)
#[test]
fn a_light_hinged_toe_lands_bounded_at_240_hz() {
    for k in [9400.0f32, 940000.0] {
        let mut w = World::with_capacity(4, 4);
        w.set_gravity(Vec3::new(0.0, -G, 0.0));
        let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
        w.add_collider(g, Shape::Box { hx: 20.0, hy: 0.5, hz: 20.0 }, material(MaterialId::Concrete));
        let mut p = w.pair_props(MaterialId::Rubber, MaterialId::Concrete);
        p.stiffness = k;
        p.dissipation = 3.0;
        w.set_pair_props(MaterialId::Rubber, MaterialId::Concrete, p);
        // A 0.1 m link, horizontal, pivoting 3 mm up; its tip a 1 mm sphere 2 mm above the ground,
        // swinging down at 20 rad/s: the tip lands at 2 m/s
        let py = 0.003f32;
        let pivot = w.add_body(Isometry { pos: Vec3::new(0.0, py, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
        let half = Vec3::new(0.05, 0.01, 0.01);
        let mp = MassProps::from_box_density(half, 0.15 / (8.0 * half.x * half.y * half.z));
        let w0 = 20.0f32;
        let com = Vec3::new(0.05, py, 0.0);
        let b = w.add_body(Isometry { pos: com, rot: Quat::IDENTITY }, Velocity { lin: Vec3::new(0.0, -w0 * 0.05, 0.0), ang: Vec3::new(0.0, 0.0, -w0) }, mp, true);
        w.set_body_sleep_allowed(b, false);
        w.add_hinge_joint(pivot, b, Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY }, Isometry { pos: Vec3::new(-0.05, 0.0, 0.0), rot: Quat::IDENTITY }, 2);
        w.add_collider_at(b, Isometry { pos: Vec3::new(0.05, 0.0, 0.0), rot: Quat::IDENTITY }, Shape::Sphere { r: 0.001 }, material(MaterialId::Rubber));
        let (mut top, mut deep) = (0.0f32, 0.0f32);
        for _ in 0..240 {
            w.step(1.0 / 240.0);
            let v = w.get_body_vel(b);
            top = top.max(v.ang.length());
            let q = w.get_body_pose(b);
            deep = deep.min((q.pos + q.rot * Vec3::new(0.05, 0.0, 0.0)).y - 0.001);
        }
        println!("k {}: peak spin {:.1} rad/s (from {}), deepest {:.1} mm", k, top, w0, -deep * 1e3);
        assert!(top.is_finite() && top < 3.0 * w0, "k {}: spun up to {:.1} rad/s", k, top);
        assert!(deep > -0.05, "k {}: {:.1} mm deep", k, -deep * 1e3);
    }
}

/// A box at rest on a 20 degree compliant slope with mu 0.6 (above tan 20 = 0.36) doesn't creep:
/// under 1 mm over 5 s at 60 Hz. Friction reads the soft row's normal impulse.
#[test]
fn a_box_holds_on_a_compliant_slope() {
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::new(0.0, -G, 0.0));
    let tilt = Quat::from_rotation_z(20f32.to_radians());
    let g = w.add_body(Isometry { pos: tilt * Vec3::new(0.0, -0.5, 0.0), rot: tilt }, Velocity::default(), MassProps::infinite(), false);
    w.add_collider(g, Shape::Box { hx: 20.0, hy: 0.5, hz: 20.0 }, material(MaterialId::Concrete));
    let mut p = w.pair_props(MaterialId::Steel, MaterialId::Concrete);
    p.stiffness = 20000.0;
    p.dissipation = 1.0;
    p.mu_s = 0.6;
    p.mu_k = 0.6;
    w.set_pair_props(MaterialId::Steel, MaterialId::Concrete, p);
    let half = Vec3::splat(0.1);
    let mp = MassProps::from_box_density(half, 1.0 / (8.0 * half.x * half.y * half.z));
    let start = tilt * Vec3::new(0.0, 0.1 - G * 20f32.to_radians().cos() / (4.0 * 20000.0), 0.0);
    let b = w.add_body(Isometry { pos: start, rot: tilt }, Velocity::default(), mp, true);
    w.add_collider(b, Shape::Box { hx: 0.1, hy: 0.1, hz: 0.1 }, material(MaterialId::Steel));
    w.set_body_sleep_allowed(b, false);
    for _ in 0..60 { w.step(1.0 / 60.0); }
    let settled = w.get_body_pose(b).pos;
    for _ in 0..300 { w.step(1.0 / 60.0); }
    let creep = (w.get_body_pose(b).pos - settled).length();
    assert!(creep < 1e-3, "crept {:.2} mm in 5 s", creep * 1e3);
}
