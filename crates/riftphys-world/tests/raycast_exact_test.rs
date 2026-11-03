// SPDX-License-Identifier: RiftForged Proprietary License
//! `World::raycast_exact` tests true shapes; `World::raycast` (RiftForged's FFI ray) keeps
//! testing bounding boxes and is unchanged.
use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::World;

fn static_box(w: &mut World, pos: Vec3, rot: Quat, he: Vec3) -> BodyId {
    let id = w.add_body(Isometry { pos, rot }, Velocity::default(), MassProps::infinite(), false);
    w.add_collider(id, Shape::Box { hx: he.x, hy: he.y, hz: he.z }, material(MaterialId::Steel));
    id
}

#[test]
fn tilted_plate_reads_its_slope_not_its_bounds() {
    let mut w = World::with_capacity(8, 8);
    // A 10 mm plate tilted 30 degrees about z, centred at the origin
    static_box(&mut w, Vec3::ZERO, Quat::from_rotation_z(30f32.to_radians()), Vec3::new(0.1, 0.005, 0.1));
    let down = -Vec3::Y;
    for x in [-0.05f32, 0.0, 0.05] {
        let hit = w.raycast_exact(Vec3::new(x, 0.5, 0.0), down, 1.0, |_| false).unwrap();
        // Top face: y = x tan30 + 0.005 / cos30
        let want = x * 30f32.to_radians().tan() + 0.005 / 30f32.to_radians().cos();
        assert!((hit.point.y - want).abs() < 1e-5, "x {} exact y {} want {}", x, hit.point.y, want);
        // The bounding-box ray reads the top of the box's bounds, the same height for every x
        let aabb_hit = w.raycast(Vec3::new(x, 0.5, 0.0), down, 1.0, None).unwrap();
        assert!(aabb_hit.point.y > want - 1e-6);
    }
}

#[test]
fn ray_passes_through_a_gap_between_plates() {
    let mut w = World::with_capacity(8, 8);
    // Two 10 mm plates with a 2 mm gap at x = 0, on a table 10 mm lower
    static_box(&mut w, Vec3::new(-0.051, -0.005, 0.0), Quat::IDENTITY, Vec3::new(0.05, 0.005, 0.1));
    static_box(&mut w, Vec3::new(0.051, -0.005, 0.0), Quat::IDENTITY, Vec3::new(0.05, 0.005, 0.1));
    let table = static_box(&mut w, Vec3::new(0.0, -0.02, 0.0), Quat::IDENTITY, Vec3::new(0.2, 0.01, 0.2));
    let hit = w.raycast_exact(Vec3::new(0.0, 0.2, 0.0), -Vec3::Y, 1.0, |_| false).unwrap();
    assert_eq!(hit.body, table);
    assert!((hit.point.y + 0.01).abs() < 1e-6);
    let plate = w.raycast_exact(Vec3::new(0.0015, 0.2, 0.0), -Vec3::Y, 1.0, |_| false).unwrap();
    assert!(plate.point.y.abs() < 1e-6);
    // Skipping a body sees through it
    let past = w.raycast_exact(Vec3::new(0.0, -0.015, 0.0), -Vec3::Y, 1.0, |b| b == table);
    assert!(past.is_none());
}

#[test]
fn capsule_and_sphere_hit_their_surface() {
    let mut w = World::with_capacity(8, 8);
    let s = w.add_body(Isometry { pos: Vec3::new(0.0, 0.0, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    w.add_collider(s, Shape::Sphere { r: 0.1 }, material(MaterialId::Steel));
    // Off-centre ray: the sphere's bounding box would be hit at y = 0.1
    let hit = w.raycast_exact(Vec3::new(0.08, 1.0, 0.0), -Vec3::Y, 2.0, |_| false).unwrap();
    assert!((hit.point.y - 0.06).abs() < 1e-5, "y {}", hit.point.y);
    assert!(w.raycast_exact(Vec3::new(0.09, 1.0, 0.09), -Vec3::Y, 2.0, |_| false).is_none());
    assert!(w.raycast(Vec3::new(0.09, 1.0, 0.09), -Vec3::Y, 2.0, None).is_some());
}
