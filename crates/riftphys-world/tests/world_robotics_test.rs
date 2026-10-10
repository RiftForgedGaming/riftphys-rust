// SPDX-License-Identifier: RiftForged Proprietary License
use riftphys_world::world::World;
use riftphys_core::{Isometry, Quat, Vec3, Velocity};
use riftphys_geom::MassProps;
use riftphys_articulation::bodies::D6Drive;
use riftphys_viz::viz::{DebugSettings, LedgerEvent};

// Helper to spawn a basic world configured for PVR Telemetry
fn setup_test_world() -> World {
    let mut world = World::with_capacity(32, 32);
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    // Enable continuous PVR Telemetry dumping for validation assertions
    let mut debug = DebugSettings::default();
    debug.dump_state_every = 1;
    world.set_debug(debug);

    // Set a fixed seed to guarantee bit-for-bit determinism across test runs
    world.set_rng_seed(0xDEADBEEF);

    world
}

#[test]
fn test_xpbd_hinge_joint_stability() {
    let mut world = setup_test_world();

    // 1. Spawn a static pivot point (Pelvis/Anchor) at altitude 10.0m
    let pivot_pose = Isometry { pos: Vec3::new(0.0, 10.0, 0.0), rot: Quat::IDENTITY };
    let pivot_id = world.add_body(pivot_pose, Velocity::default(), MassProps::infinite(), false);

    // 2. Spawn a dynamic pendulum (Calf/Leg) at altitude 10.0m, offset by 1.0m on the Z axis
    // It starts perfectly horizontal.
    let leg_pose = Isometry { pos: Vec3::new(0.0, 10.0, 1.0), rot: Quat::IDENTITY };
    let leg_mass = MassProps::from_box_density(Vec3::new(0.1, 0.5, 0.1), 50.0);
    let leg_id = world.add_body(leg_pose, Velocity::default(), leg_mass, true);

    // 3. Bind them with a Hinge on the X-axis (Pitch)
    let _knee_joint = world.add_hinge_joint(
        pivot_id,
        leg_id,
        Isometry { pos: Vec3::new(0.0, 0.0, 0.0), rot: Quat::IDENTITY },
        Isometry { pos: Vec3::new(0.0, 0.0, -1.0), rot: Quat::IDENTITY },
        0 // X-Axis
    );

    let dt = 0.016f32;

    // Simulate for 1 second (60 ticks) using the unified step function
    for _ in 0..60 {
        world.step(dt);
    }

    let final_leg_pose = world.get_body_pose(leg_id);

    // ASSERTIONS:
    // 1. The leg should have swung downward due to gravity (Y < 10.0).
    assert!(final_leg_pose.pos.y < 9.5, "Leg did not swing downward under gravity.");

    // 2. The joint must not have ripped apart.
    // The distance between pivot (0,10,0) and the leg's anchor point should still be exactly 1.0m.
    let dist = final_leg_pose.pos.distance(Vec3::new(0.0, 10.0, 0.0));
    assert!(
        (dist - 1.0).abs() < 0.01,
        "XPBD Constraint failed! Joint ripped apart. Distance is {}", dist
    );
}

#[test]
fn test_xpbd_motor_drive_velocity() {
    let mut world = setup_test_world();

    // 1. Spawn static pivot
    let pivot_pose = Isometry { pos: Vec3::new(0.0, 10.0, 0.0), rot: Quat::IDENTITY };
    let pivot_id = world.add_body(pivot_pose, Velocity::default(), MassProps::infinite(), false);

    // 2. Spawn dynamic leg hanging straight down (0, 9, 0)
    let leg_pose = Isometry { pos: Vec3::new(0.0, 9.0, 0.0), rot: Quat::IDENTITY };
    let leg_mass = MassProps::from_box_density(Vec3::new(0.1, 0.5, 0.1), 50.0);
    let leg_id = world.add_body(leg_pose, Velocity::default(), leg_mass, true);

    // 3. Bind with a Hinge
    let knee_joint = world.add_hinge_joint(
        pivot_id,
        leg_id,
        Isometry { pos: Vec3::new(0.0, 0.0, 0.0), rot: Quat::IDENTITY },
        Isometry { pos: Vec3::new(0.0, 1.0, 0.0), rot: Quat::IDENTITY },
        0 // X-Axis
    );

    // 4. Inject a Motor Drive aiming for +2.0 rad/s on the X-axis
    // Increase KD significantly to overcome the constraint damping
    let motor = D6Drive {
        target: 2.0,
        kp: 0.0,
        kd: 5.0, // Increased from 1.0 to 5.0
        max_impulse: 1000.0, // Increased from 500.0
        vel_mode: true,
    };
    world.set_d6_drive_r(knee_joint, 0, Some(motor));

    let dt = 0.016f32;

    // Simulate for 0.5 seconds (30 ticks)
    for _ in 0..30 {
        world.step(dt);
    }

    let final_vel = world.get_body_vel(leg_id);
    let final_pose = world.get_body_pose(leg_id);

    // ASSERTIONS:
    // 1. The motor should have forced the leg to rotate positively around the X axis.
    assert!(
        final_vel.ang.x > 1.5,
        "Motor failed to reach target velocity. Current ang_vel_x: {}", final_vel.ang.x
    );

    // 2. The physical position of the leg must have swung upward (Z should no longer be 0, Y should rise).
    assert!(
        final_pose.pos.z > 0.1,
        "Leg did not physically swing forward. Pos: {:?}", final_pose.pos
    );
}

// =========================================================================
// NEW PVR TESTS
// =========================================================================

#[test]
fn test_pvr_energy_conservation_pendulum() {
    let mut world = setup_test_world();

    // Setup an isolated pendulum to check thermodynamic integrity
    let pivot_id = world.add_body(
        Isometry { pos: Vec3::new(0.0, 20.0, 0.0), rot: Quat::IDENTITY },
        Velocity::default(), MassProps::infinite(), false
    );
    let leg_id = world.add_body(
        Isometry { pos: Vec3::new(5.0, 20.0, 0.0), rot: Quat::IDENTITY }, // 5 meters out
        Velocity::default(), MassProps::from_box_density(Vec3::new(1.0, 1.0, 1.0), 10.0), true
    );
    world.add_hinge_joint(
        pivot_id, leg_id,
        Isometry { pos: Vec3::new(0.0, 0.0, 0.0), rot: Quat::IDENTITY },
        Isometry { pos: Vec3::new(-5.0, 0.0, 0.0), rot: Quat::IDENTITY },
        2 // Z-Axis hinge
    );

    let dt = 0.016f32;
    let mut initial_energy = 0.0;

    // Capture initial energy state after 1st tick
    world.step(dt);
    for event in world.ledger.iter() {
        if let LedgerEvent::SystemEnergy { total, .. } = event {
            initial_energy = *total;
        }
    }

    // Run for 300 ticks (5 seconds of isolated swinging)
    for _ in 0..300 {
        world.step(dt);
    }

    // Capture final energy
    let mut final_energy = 0.0;
    for event in world.ledger.iter() {
        if let LedgerEvent::SystemEnergy { total, .. } = event {
            final_energy = *total;
        }
    }

    // A rigid body physics engine with an XPBD solver will inevitably bleed *some* energy
    // due to constraint stabilization and positional projection.
    // The goal here is to assure the bleed stays strictly within a bounded architectural limit, preventing explosive jitter.
    let energy_retention = final_energy / initial_energy;
    assert!(
        energy_retention > 0.98 && energy_retention <= 1.001,
        "Thermodynamic integrity failure! Initial E: {}, Final E: {} (Retention: {:.3}%)",
        initial_energy, final_energy, energy_retention * 100.0
    );
}

#[test]
fn test_pvr_high_mass_ratio_stability() {
    let mut world = setup_test_world();

    // Floor
    world.add_body(Isometry { pos: Vec3::new(0.0, 0.0, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);

    // A 1kg box placed on the floor
    let light_box = world.add_body(
        Isometry { pos: Vec3::new(0.0, 0.5, 0.0), rot: Quat::IDENTITY },
        Velocity::default(), MassProps::from_box_density(Vec3::new(0.5, 0.5, 0.5), 8.0), true // 8kg/m3 * 0.125m3 = 1kg
    );

    // A 1000kg box dropped directly on top of the 1kg box
    let heavy_box = world.add_body(
        Isometry { pos: Vec3::new(0.0, 2.0, 0.0), rot: Quat::IDENTITY },
        Velocity::default(), MassProps::from_box_density(Vec3::new(0.5, 0.5, 0.5), 8000.0), true // 8000kg/m3 * 0.125m3 = 1000kg
    );

    let dt = 0.016f32;
    for _ in 0..120 { // 2 seconds to settle
        world.step(dt);
    }

    let light_vel = world.get_body_vel(light_box);
    let heavy_vel = world.get_body_vel(heavy_box);

    // The system should have resolved the penetration and gone to sleep/rest.
    // If the mass ratio constraint blew up, these velocities will be massive or NaN.
    assert!(
        light_vel.lin.length() < 0.1,
        "High mass-ratio stack jittering! Light box velocity: {:?}", light_vel.lin
    );
    assert!(
        heavy_vel.lin.length() < 0.1,
        "High mass-ratio stack jittering! Heavy box velocity: {:?}", heavy_vel.lin
    );
}

#[test]
fn test_pvr_determinism() {
    // We execute the exact same heavy stack scenario twice and ensure bit-for-bit identical final states.

    let run_simulation = || -> Vec3 {
        let mut world = setup_test_world();
        let body = world.add_body(
            Isometry { pos: Vec3::new(10.5, 50.0, -3.3), rot: Quat::from_axis_angle(Vec3::new(1.0, 1.0, 0.0).normalize(), 0.5) },
            Velocity { lin: Vec3::new(1.0, -5.0, 2.0), ang: Vec3::new(0.1, 0.5, -0.2) },
            MassProps::from_box_density(Vec3::new(1.0, 1.0, 1.0), 10.0), true
        );
        for _ in 0..100 { world.step(0.016); }
        world.get_body_pose(body).pos
    };

    let result_a = run_simulation();
    let result_b = run_simulation();

    // Float equality is checked exactly. If the solver operations happen in different orders
    // across threads, this will flag the non-determinism.
    assert_eq!(
        result_a, result_b,
        "PVR Determinism failure! Execution A yielded {:?}, Execution B yielded {:?}", result_a, result_b
    );
}