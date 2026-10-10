use anyhow::*;
use glam::Vec3;
use std::fs;

use riftphys_core::{iso, quat_identity, vec3, BodyId, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_gravity::GravitySpec;
use riftphys_io::rig_physics::{humanoid_from_rig, load_into_world, PhysicsRig, WeaponLink};
use riftphys_io::RigData;
use riftphys_materials::materials::*;
use riftphys_viz::viz::DebugSettings;
use riftphys_world::world::World;

// Import the shared types from your bench-helpers crate
use riftphys_bench_helpers::bench_loco::{loco_tick_with_heading, MeleeWalker, PathKind, Walker};
use riftphys_locomotion::loco_state as loco;

fn main() -> Result<()> {
    let dt = 1.0 / 120.0;
    let ticks = 2400; // 20 seconds of combat

    let mut world = World::with_capacity(256, 1024);

    setup_world(&mut world);

    // Load rig and attach "sticks" to the hands procedurally
    let rig_data_raw = fs::read_to_string("assets/superhuman.rig.json")?;
    let rig_data: RigData = serde_json::from_str(&rig_data_raw)?;
    let mut phys_rig = humanoid_from_rig(&rig_data)?;

    // Procedural weapon attachment: 1 meter long stick in the right hand
    phys_rig = phys_rig.with_weapon(WeaponLink {
        hand_node_name: "r_hand".into(),
        weapon_name: "melee_stick".into(),
        dimensions: [0.03, 0.5], // r, hh
        offset: [0.0, 0.0, 0.2],
    });

    let mut fighters = spawn_combatants(&mut world, &phys_rig)?;

    println!("Simulation Start: Deterministic Melee Test");

    for step in 0..ticks {
        // 1. Update AI and Combat Logic
        for i in 0..fighters.len() {
            let pos_i = world.get_body_pose(fighters[i].base.pelvis).pos;

            // Search for target
            for j in 0..fighters.len() {
                if i == j { continue; }
                let pos_j = world.get_body_pose(fighters[j].base.pelvis).pos;

                // If within 1.5m and not currently swinging, start a strike
                if pos_i.distance(pos_j) < 1.5 && fighters[i].strike_time <= 0.0 {
                    fighters[i].strike_time = 0.6; // Duration of the swing
                }
            }

            // Run locomotion and combat systems
            fighters[i].base.step(&mut world, dt);
            fighters[i].update_combat(&mut world, dt);
        }

        // 2. Physics Step (Includes fixed rotation integration)
        world.step(dt);

        if step % 120 == 0 {
            let weapon_count = world.count_capsules();
            let bp_pairs = world.num_broadphase_pairs();
            println!(
                "Tick {:4} | Weapons: {} | BP Pairs: {} | Impulse Sum: {:.2}",
                step, weapon_count, bp_pairs, world.total_impulse_sum()
            );
        }
    }

    println!("Final State Hash: {:02x?}", world.step_hash());
    Ok(())
}

fn setup_world(w: &mut World) {
    w.set_epoch(1);
    w.set_rng_seed(0xDEADC0DE);
    w.queue_gravity_swap(GravitySpec::Uniform { g: [0.0, -9.81, 0.0] });

    // High friction ground to prevent sliding during impacts
    let mut mat_ground = material(MaterialId::Default);
    mat_ground.mu_s = 1.2;
    mat_ground.mu_k = 1.0;

    let ground = w.add_body(iso(vec3(0.0, -0.25, 0.0), quat_identity()), Velocity::default(), MassProps::infinite(), false);
    w.add_collider(ground, Shape::Box { hx: 50.0, hy: 0.25, hz: 50.0 }, mat_ground);
}

fn spawn_combatants(world: &mut World, phys: &PhysicsRig) -> Result<Vec<MeleeWalker>> {
    let mut fighters = Vec::new();
    let gait = loco::GaitSpec {
        stance_dur: 0.4,
        swing_dur: 0.4,
        step_len: 0.4,
        step_h: 0.15,
    };

    // Spawn two combatants facing each other
    // In spawn_combatants, move them closer
    let configs = [
        (vec3(-0.5, 0.0, 0.0), Vec3::X),  // Fighter A
        (vec3(0.5, 0.0, 0.0), -Vec3::X), // Fighter B
    ];

    for (pos, dir) in configs {
        let map = load_into_world(world, phys)?;

        // Disable sleep to ensure active combat
        for id in map.body.values() {
            world.set_body_sleep_allowed(*id, false);
        }

        let walker = Walker {
            pelvis: map.body("pelvis"),
            left: map.body("l_foot"),
            right: map.body("r_foot"),
            state: loco::LocoState::new(map.body("pelvis").0, map.body("l_foot").0, map.body("r_foot").0, gait),
            clocks: (
                loco::FootClock::new(gait.stance_dur, gait.swing_dur),
                loco::FootClock::new(gait.stance_dur, gait.swing_dur),
            ),            gait,
            path: PathKind::Straight { dir_ws: dir },
        };

        fighters.push(MeleeWalker {
            base: walker,
            weapon_hand: map.body("r_hand"),
            target_enemy: None,
            strike_time: 0.0,
        });
    }

    Ok(fighters)
}