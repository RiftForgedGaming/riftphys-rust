// SPDX-License-Identifier: RiftForged Proprietary License
//! Patch friction, opt in per material pair via `MatPairProps::patch_friction`.
use riftphys_core::{Isometry, Mat3, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::World;

const G: f32 = 9.81;
const A: f32 = 0.0524;
const M: f32 = 0.37;
const I: f32 = 8.1e-4;
const MU: f32 = 0.2;

/// A cube resting on its face, spinning about the vertical at `w0` and sliding at `v0`; returns
/// (angle turned, distance slid) once it has stopped.
fn spin(patch: bool, stiffness: f32, w0: f32, v0: f32) -> (f32, f32) {
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::new(0.0, -G, 0.0));
    let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    w.add_collider(g, Shape::Box { hx: 10.0, hy: 0.5, hz: 10.0 }, material(MaterialId::Wood));
    let mut p = w.pair_props(MaterialId::AcrylicPmma, MaterialId::Wood);
    p.mu_s = MU;
    p.mu_k = MU;
    p.restitution = 0.0;
    p.stiffness = stiffness;
    p.dissipation = if stiffness > 0.0 { 3.0 } else { 0.0 };
    p.patch_friction = patch;
    w.set_pair_props(MaterialId::AcrylicPmma, MaterialId::Wood, p);
    let mut mp = MassProps::from_box_density(Vec3::splat(A), M / (8.0 * A * A * A));
    mp.inertia = Mat3::from_diagonal(Vec3::splat(I).into());
    let sink = if stiffness > 0.0 { G * M / (4.0 * stiffness) } else { 0.0 };
    let c = w.add_body(Isometry { pos: Vec3::new(0.0, A - sink, 0.0), rot: Quat::IDENTITY },
                       Velocity { lin: Vec3::new(v0, 0.0, 0.0), ang: Vec3::new(0.0, w0, 0.0) }, mp, true);
    w.add_collider(c, Shape::Box { hx: A, hy: A, hz: A }, material(MaterialId::AcrylicPmma));
    w.set_body_sleep_allowed(c, false);
    let mut angle = 0.0f32;
    for _ in 0..4000 {
        w.step(1.0 / 1480.0);
        angle += w.get_body_vel(c).ang.y / 1480.0;
    }
    (angle, w.get_body_pose(c).pos.x)
}

/// Mean distance from the centre over a square face of half-width A.
fn face_radius() -> f32 { A * (2f32.sqrt() + (1.0 + 2f32.sqrt()).ln()) / 3.0 }

/// Spinning flat, the face stops after I w0^2 / (2 mu m g r): with per-point friction r is the
/// corner distance (sqrt 2 A); with patch friction it's the face's mean radius (0.765 A).
#[test]
fn a_spinning_face_stops_as_a_uniformly_pressed_face() {
    let w0 = 6.0;
    let theta = |r: f32| I * w0 * w0 / (2.0 * MU * M * G * r);
    for k in [0.0f32, 13591.0] {
        let (patch, _) = spin(true, k, w0, 0.0);
        let (corners, _) = spin(false, k, w0, 0.0);
        let (want_patch, want_corners) = (theta(face_radius()), theta(A * 2f32.sqrt()));
        assert!((patch - want_patch).abs() < 0.03 * want_patch, "k {}: patch spun {:.4}, want {:.4}", k, patch, want_patch);
        assert!((corners - want_corners).abs() < 0.03 * want_corners, "k {}: per-point spun {:.4}, want {:.4}", k, corners, want_corners);
    }
}

/// Patch friction keeps sliding friction Coulomb: v0^2 / (2 mu g).
#[test]
fn sliding_distance_is_unchanged() {
    let v0 = 1.5;
    let want = v0 * v0 / (2.0 * MU * G);
    for k in [0.0f32, 13591.0] {
        let (_, slid) = spin(true, k, 0.0, v0);
        assert!((slid - want).abs() < 0.03 * want, "k {}: slid {:.4}, want {:.4}", k, slid, want);
    }
}

#[test]
fn patch_friction_replays_bit_exactly() {
    let run = || spin(true, 0.0, 4.0, 1.0);
    assert_eq!(run(), run());
}
