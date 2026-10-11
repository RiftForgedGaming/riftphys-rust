// Regression tests for the contact solver: bodies must come to rest ON a static body, not in or
// through it. The solver used to apply twice the impulse it accumulated, so every iteration's push
// was cancelled by the next clamp and everything fell through the ground; it also treated the
// narrowphase contact margin as penetration and had no angular terms.

use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::World;

const DT: f32 = 1.0 / 60.0;

fn world_with_ground() -> World {
    let mut w = World::with_capacity(16, 16);
    w.set_gravity(Vec3::new(0.0, -9.81, 0.0));
    let ground = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    w.add_collider(ground, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(MaterialId::Concrete));
    w
}

fn add_dynamic(w: &mut World, pos: Vec3, rot: Quat, half: Vec3, mass: f32, shape: Shape) -> BodyId {
    let mp = MassProps::from_box_density(half, mass / (8.0 * half.x * half.y * half.z));
    let id = w.add_body(Isometry { pos, rot }, Velocity::default(), mp, true);
    w.add_collider(id, shape, material(MaterialId::Rubber));
    w.set_body_sleep_allowed(id, false);
    id
}

#[test]
fn shapes_rest_on_a_static_ground_body() {
    for (name, shape, half) in [
        ("sphere", Shape::Sphere { r: 0.1 }, Vec3::splat(0.1)),
        ("capsule", Shape::Capsule { r: 0.1, hh: 0.1 }, Vec3::new(0.1, 0.2, 0.1)),
        ("box", Shape::Box { hx: 0.1, hy: 0.1, hz: 0.1 }, Vec3::splat(0.1)),
    ] {
        for mass in [1.0, 80.0] {
            let mut w = world_with_ground();
            let id = add_dynamic(&mut w, Vec3::new(0.0, half.y + 0.2, 0.0), Quat::IDENTITY, half, mass, shape);
            for _ in 0..180 { w.step(DT); }
            let y = w.get_body_pose(id).pos.y;
            assert!((y - half.y).abs() < 0.012, "{} {} kg rests at y {:.4}, expected {:.3}", name, mass, y, half.y);
        }
    }
}

#[test]
fn tilted_box_settles_flat() {
    // Dropped on one edge, a cube must topple onto a face: needs torque from off-centre contacts.
    let mut w = world_with_ground();
    let h = Vec3::splat(0.1);
    let id = add_dynamic(&mut w, Vec3::new(0.0, 0.25, 0.0), Quat::from_rotation_z(0.6), h, 2.0, Shape::Box { hx: h.x, hy: h.y, hz: h.z });
    for _ in 0..300 { w.step(DT); }
    let p = w.get_body_pose(id);
    let r = p.rot;
    let face_up = [Vec3::X, Vec3::Y, Vec3::Z].iter().map(|a| (r * *a).y.abs()).fold(0.0f32, f32::max);
    assert!(face_up > 0.99, "cube did not settle on a face: max |axis.y| {:.3}", face_up);
    assert!((p.pos.y - 0.1).abs() < 0.012, "cube centre at y {:.4}", p.pos.y);
}

#[test]
fn stacked_boxes_hold() {
    let mut w = world_with_ground();
    let h = Vec3::new(0.2, 0.1, 0.2);
    let ids: Vec<BodyId> = (0..3)
        .map(|i| add_dynamic(&mut w, Vec3::new(0.0, 0.1 + 0.2 * i as f32 + 0.01 * i as f32, 0.0), Quat::IDENTITY, h, 5.0, Shape::Box { hx: h.x, hy: h.y, hz: h.z }))
        .collect();
    for _ in 0..240 { w.step(DT); }
    for (i, id) in ids.iter().enumerate() {
        let p = w.get_body_pose(*id).pos;
        let want = 0.1 + 0.2 * i as f32;
        assert!((p.y - want).abs() < 0.03 && p.x.abs() < 0.02 && p.z.abs() < 0.02, "box {} at {:?}, expected y {:.2}", i, p, want);
    }
}
