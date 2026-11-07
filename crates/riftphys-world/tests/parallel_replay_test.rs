// SPDX-License-Identifier: RiftForged Proprietary License
//! 200 boxes in two 10 x 10 layers on the ground run the parallel narrowphase and contact-solver
//! paths (64 or more pairs, and a contact batch of 64 or more: each box's first ground contact
//! lands in the same level). The parallel joint path (64+ joints in one level) is not covered. Its step hash must match bit for bit across separate processes with
//! different rayon thread counts (PRECISION_SPEC P1): pairs are sorted, contact buckets are a
//! BTreeMap, rayon's collect keeps order and solver batches write disjoint bodies, so the thread
//! count must not change any result.
use std::process::Command;

use riftphys_core::{Isometry, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::World;

const CHILD: &str = "RIFTPHYS_PARALLEL_REPLAY_CHILD";

/// 200 boxes in two 10 x 10 layers, slightly rotated so they settle unevenly against each other;
/// 240 steps at 120 Hz. Returns the step hash.
fn pile_hash() -> String {
    let mut w = World::with_capacity(256, 256);
    w.set_gravity(Vec3::new(0.0, -9.81, 0.0));
    let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(MaterialId::Concrete));
    let half = Vec3::splat(0.1);
    let mut n = 0;
    for y in 0..2 {
        for x in 0..10 {
            for z in 0..10 {
                let pos = Vec3::new(x as f32 * 0.21 - 1.0, 0.12 + y as f32 * 0.22, z as f32 * 0.21 - 1.0);
                let rot = Quat::from_rotation_y(0.07 * n as f32) * Quat::from_rotation_x(0.03 * (n % 7) as f32);
                let b = w.add_body(Isometry { pos, rot }, Velocity::default(), MassProps::from_box_density(half, 500.0), true);
                w.add_collider(b, Shape::Box { hx: half.x, hy: half.y, hz: half.z }, material(MaterialId::Wood));
                n += 1;
            }
        }
    }
    let (mut most_pairs, mut most_batch) = (0, 0);
    for _ in 0..240 {
        most_pairs = most_pairs.max(w.step(1.0 / 120.0).pairs_tested);
        most_batch = most_batch.max(w.last_max_batch().0);
    }
    assert!(most_pairs >= 64, "only {} pairs: the parallel narrowphase never ran", most_pairs);
    assert!(most_batch >= 64, "largest contact batch {}: the parallel contact solve never ran", most_batch);
    w.step_hash().iter().map(|b| format!("{:02x}", b)).collect()
}

/// Run in a child process: prints the hash for the parent to read.
#[test]
#[ignore]
fn parallel_replay_child() {
    if std::env::var(CHILD).is_ok() {
        println!("HASH {}", pile_hash());
    }
}

#[test]
fn parallel_replay_matches_across_thread_counts() {
    let exe = std::env::current_exe().unwrap();
    let run = |threads: &str| -> String {
        let out = Command::new(&exe)
            .args(["parallel_replay_child", "--exact", "--ignored", "--nocapture", "--test-threads", "1"])
            .env(CHILD, "1")
            .env("RAYON_NUM_THREADS", threads)
            .output()
            .expect("spawn child test process");
        let text = String::from_utf8_lossy(&out.stdout).to_string();
        text.lines().find_map(|l| l.split_once("HASH ").map(|(_, h)| h.trim().to_string()))
            .unwrap_or_else(|| panic!("child ({} threads) printed no hash:\n{}\n{}", threads, text, String::from_utf8_lossy(&out.stderr)))
    };
    let one = run("1");
    let many = run("8");
    assert_eq!(one, many, "step hash differs between 1 and 8 rayon threads");
    // and a third process, with 3 threads
    assert_eq!(one, run("3"));
}
