// use riftphys_world::world::WorldBuilder;
// use riftphys_core::{iso, quat_identity, vec3, Velocity, BodyId};
// use riftphys_geom::{MassProps, Shape};
// use riftphys_materials::materials::{material, MaterialId};
//
// fn build_wall_world() -> riftphys_world::world::World {
//     let mut w = WorldBuilder::new().with_capacity(32, 64).build();
//     w.set_rng_seed(0xC0FFEE);
//     w.set_epoch(1);
//     w.set_gravity(vec3(0.0, 0.0, 0.0));
//
//     // Static thin wall at x = 0
//     let wall = w.add_body(
//         iso(vec3(0.0, 1.0, 0.0), quat_identity()),
//         Velocity::default(),
//         MassProps::infinite(),
//         false,
//     );
//     w.add_collider(wall, Shape::Box { hx: 0.05, hy: 5.0, hz: 5.0 }, material(MaterialId::Grit));
//
//     w
// }
//
// #[test]
// fn capsule_ccd_stops_on_wall_single_collider() {
//     let mut w = build_wall_world();
//
//     // Fast capsule from x=-1 toward +X (would tunnel without CCD)
//     let cap = w.add_body(
//         iso(vec3(-1.0, 1.0, 0.0), quat_identity()),
//         Velocity { lin: vec3(200.0, 0.0, 0.0), ang: vec3(0.0, 0.0, 0.0) },
//         MassProps::from_capsule(0.25, 0.5, MaterialId::Steel),
//         true,
//     );
//     w.add_collider(cap, Shape::Capsule { r: 0.25, hh: 0.5 }, material(MaterialId::RubberSoft));
//
//     let dt = 1.0 / 120.0;
//     let _ = w.step(dt);
//
//     let p = w.get_body_pose(cap).pos;
//     // Wall plane at x=0 with hx=0.05. Capsule radius 0.25 => center should not cross ~ -0.30.
//     assert!(p.x <= -0.28, "capsule tunneled: pos.x={}", p.x);
// }
//
// #[test]
// fn capsule_ccd_stops_on_wall_even_if_capsule_is_not_first_collider() {
//     let mut w = build_wall_world();
//
//     let body = w.add_body(
//         iso(vec3(-1.0, 1.0, 0.0), quat_identity()),
//         Velocity { lin: vec3(200.0, 0.0, 0.0), ang: vec3(0.0, 0.0, 0.0) },
//         MassProps::from_capsule(0.25, 0.5, MaterialId::Steel),
//         true,
//     );
//
//     // Add a non-capsule collider first (this is what your old loop broke on)
//     w.add_collider(body, Shape::Box { hx: 0.02, hy: 0.02, hz: 0.02 }, material(MaterialId::RubberSoft));
//     w.add_collider(body, Shape::Capsule { r: 0.25, hh: 0.5 }, material(MaterialId::RubberSoft));
//
//     let dt = 1.0 / 120.0;
//     let _ = w.step(dt);
//
//     let p = w.get_body_pose(body).pos;
//     assert!(p.x <= -0.28, "capsule CCD skipped (capsule not first collider): pos.x={}", p.x);
// }
