// Golden bits for scenes with today's distance joints (PARITY_SPEC 3.4b, W4c): recorded at
// f781e2a, before anchors and damping, so the zero-anchor path is checked to stay bit for bit.
// Nothing golden covered a distance joint before (the Reviewer's finding).
use riftphys_controllers::GuardParams;
use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::MassProps;
use riftphys_world::world::World;

fn iso(pos: Vec3, rot: Quat) -> Isometry { Isometry { pos, rot } }

fn bits(w: &World, ids: &[BodyId]) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for &b in ids {
        let (q, v) = (w.get_body_pose(b), w.get_body_vel(b));
        for x in [q.pos.x, q.pos.y, q.pos.z, q.rot.x, q.rot.y, q.rot.z, q.rot.w, v.lin.x, v.lin.y, v.lin.z, v.ang.x, v.ang.y, v.ang.z] {
            h = (h ^ x.to_bits() as u64).wrapping_mul(0x100000001b3);
        }
    }
    h
}

/// Kind 0: a rigid rod (a box swinging on a 1 m distance joint, out of plane). Kind 1: a
/// compliant rod (the same box on a spring, started stretched and sideways). Kind 2: the guard
/// controller's compliant joint on a moving effector.
fn scene(kind: u32, hz: f32) -> u64 {
    let mut w = World::with_capacity(8, 8);
    w.set_gravity(Vec3::new(0.0, -9.81, 0.0));
    let pivot = w.add_body(iso(Vec3::new(0.0, 5.0, 0.0), Quat::IDENTITY), Velocity::default(), MassProps::infinite(), false);
    let mp = MassProps::from_box_density(Vec3::new(0.1, 0.05, 0.08), 500.0);
    let rot = Quat::from_rotation_y(0.3) * Quat::from_rotation_x(0.2);
    let start = Vec3::new(0.7, 4.3, 0.2);
    let vel = Velocity { lin: Vec3::new(0.0, 0.5, 1.5), ang: Vec3::new(0.4, -1.0, 2.0) };
    let b = w.add_body(iso(start, rot), vel, mp, true);
    w.set_body_sleep_allowed(b, false);
    let rest = (start - Vec3::new(0.0, 5.0, 0.0)).length();
    match kind {
        0 => { w.add_distance_joint(pivot, b, rest, 0.0); }
        1 => { w.add_distance_joint(pivot, b, rest * 0.8, 2.0e-3); }
        _ => { w.add_guard_controller(pivot, b, GuardParams { rest_guard: rest * 0.9, rest_brace: rest * 0.7, k_guard: 1.0e-3, k_brace: 1.0e-4, brace_time: 0.5 }); }
    }
    for _ in 0..(3.0 * hz) as usize { w.step(1.0 / hz); }
    let p = w.get_body_pose(b).pos;
    assert!(p.is_finite() && (p - Vec3::new(0.0, 5.0, 0.0)).length() < 3.0, "kind {} {} Hz went to {:?}", kind, hz, p);
    bits(&w, &[b])
}

#[test]
fn distance_joint_scenes_keep_their_bits() {
    // Recorded at f781e2a (release and debug alike; checked both)
    let golden: [(u32, f32, u64); 6] = [
        (0, 60.0, 0x87f81a83f9615530), (0, 240.0, 0xe7411745726c1915),
        (1, 60.0, 0x6f111d612da3e10f), (1, 240.0, 0xed532e5e87913e18),
        (2, 60.0, 0xbb57c95b1772e5eb), (2, 240.0, 0xc1a92484be836750),
    ];
    let mut report = String::new();
    let mut bad = false;
    for (kind, hz, want) in golden {
        let got = scene(kind, hz);
        report += &format!("kind {} {} Hz: {:016x}\n", kind, hz, got);
        if got != want { bad = true; }
    }
    assert!(!bad, "distance-joint bits changed:\n{}", report);
}
