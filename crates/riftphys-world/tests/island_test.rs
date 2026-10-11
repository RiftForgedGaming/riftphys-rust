    // // world/tests/island_test.rs
    // use riftphys_world::world::WorldBuilder;
    // use riftphys_core::{iso, quat_identity, vec3, Velocity, BodyId};
    // use riftphys_geom::{MassProps, Shape};
    // use riftphys_materials::materials::{material, MaterialId};
    //
    // /// Helper to build a standard testing environment
    // fn setup_test_world() -> riftphys_world::world::World {
    //     let mut w = WorldBuilder::new().with_capacity(128, 256).build();
    //     w.set_rng_seed(0x1337);
    //     w.set_gravity(vec3(0.0, -9.81, 0.0));
    //     w
    // }
    //
    // #[test]
    // fn test_island_joint_connectivity() {
    //     let mut w = setup_test_world();
    //
    //     // 1. Create two dynamic spheres aligned on the X-axis
    //     let b1 = w.add_body(
    //         iso(vec3(0.0, 5.0, 0.0), quat_identity()),
    //         Velocity::default(),
    //         MassProps::from_sphere(0.5, MaterialId::Steel),
    //         true,
    //     );
    //     let b2 = w.add_body(
    //         iso(vec3(2.0, 5.0, 0.0), quat_identity()),
    //         Velocity::default(),
    //         MassProps::from_sphere(0.5, MaterialId::Steel),
    //         true,
    //     );
    //
    //     // 2. Add a distance joint between them
    //     w.add_distance_joint(b1, b2, 2.0, 0.0);
    //
    //     w.step(1.0 / 60.0);
    //
    //     // 3. Verify connectivity via impulse propagation
    //     // Push b1 horizontally TOWARDS b2 along the constraint axis
    //     w.apply_impulse(b1, vec3(10.0, 0.0, 0.0));
    //     w.step(1.0 / 60.0);
    //
    //     let v2 = w.get_body_vel(b2).lin;
    //
    //     // b2 should now be pushed to the right by the joint
    //     assert!(v2.x > 0.0, "Joint connectivity failed: impulse did not propagate. v2.x = {}", v2.x);
    // }
    //
    // #[test]
    // fn test_island_contact_propagation() {
    //     let mut w = setup_test_world();
    //     let mat = material(MaterialId::Grit);
    //
    //     // 1. Setup Ground (Static)
    //     let ground = w.add_body(
    //         iso(vec3(0.0, -0.5, 0.0), quat_identity()),
    //         Velocity::default(),
    //         MassProps::infinite(),
    //         false,
    //     );
    //     w.add_collider(ground, Shape::Box { hx: 10.0, hy: 0.5, hz: 10.0 }, mat);
    //
    //     // 2. Create a vertical stack of 3 boxes
    //     // Spawn them exactly touching to minimize settling time
    //     let mut stack = Vec::new();
    //     for i in 0..3 {
    //         let b = w.add_body(
    //             iso(vec3(0.0, 0.5 + (i as f32 * 1.0), 0.0), quat_identity()), // 1.0 spacing, no air gap
    //             Velocity::default(),
    //             MassProps::from_box(vec3(0.5, 0.5, 0.5), MaterialId::Steel),
    //             true,
    //         );
    //         w.add_collider(b, Shape::Box { hx: 0.5, hy: 0.5, hz: 0.5 }, mat);
    //         stack.push(b);
    //     }
    //
    //     // 3. Settle the physics engine
    //     // Step 10 times to let the narrowphase build solid, warm manifolds
    //     // and let the Island Manager bind them together.
    //     for _ in 0..10 {
    //         w.step(1.0 / 60.0);
    //     }
    //
    //     // 4. Verify propagation: Kick the bottom-most dynamic box
    //     // We apply an angled/upward impulse so it physically knocks the stack over,
    //     // ensuring momentum propagates through the contacts regardless of friction settings.
    //     w.apply_impulse(stack[0], vec3(10.0, 5.0, 0.0));
    //     w.step(1.0 / 60.0);
    //
    //     // If contact propagation works, the top-most box should have been jolted
    //     let v_top = w.get_body_vel(stack[2]).lin;
    //     assert!(v_top.length() > 0.1, "Contact propagation failed: stack island did not transfer momentum. v_top = {:?}", v_top);
    // }
    //
    // #[test]
    // fn test_island_spatial_independence() {
    //     let mut w = setup_test_world();
    //
    //     // Create two spheres very far apart
    //     let b1 = w.add_body(
    //         iso(vec3(-100.0, 10.0, 0.0), quat_identity()),
    //         Velocity::default(),
    //         MassProps::from_sphere(0.5, MaterialId::Steel),
    //         true,
    //     );
    //     let b2 = w.add_body(
    //         iso(vec3(100.0, 10.0, 0.0), quat_identity()),
    //         Velocity::default(),
    //         MassProps::from_sphere(0.5, MaterialId::Steel),
    //         true,
    //     );
    //
    //     // Step physics
    //     w.step(1.0 / 60.0);
    //
    //     // Apply impulse to sphere 1
    //     w.apply_impulse(b1, vec3(0.0, 10.0, 0.0));
    //     w.step(1.0 / 60.0);
    //
    //     // Sphere 2 should be completely unaffected (independent island)
    //     let v2 = w.get_body_vel(b2).lin;
    //     assert_eq!(v2.x, 0.0);
    //     assert!(v2.y < 0.0, "Sphere 2 should only be affected by gravity, not Sphere 1's impulse");
    // }