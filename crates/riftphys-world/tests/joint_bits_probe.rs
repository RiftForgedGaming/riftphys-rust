// Probe: final pose and velocity bits of a jointed chain (limit, motor, free hinges), for
// checking that a change leaves rigid rows bit-identical. Run with --ignored --nocapture.
use riftphys_articulation::bodies::{D6Drive, D6Limit};
use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::MassProps;
use riftphys_world::world::World;

fn iso(pos: Vec3) -> Isometry { Isometry { pos, rot: Quat::IDENTITY } }

fn chain() -> (World, Vec<BodyId>) {
    let mut w = World::with_capacity(16, 16);
    w.set_gravity(Vec3::new(0.0, -9.81, 0.0));
    let p = w.add_body(iso(Vec3::new(0.0, 10.0, 0.0)), Velocity::default(), MassProps::infinite(), false);
    let mut prev = p;
    let mut ids = Vec::new();
    for k in 0..4 {
        let r = w.add_body(iso(Vec3::new(0.5 + k as f32, 10.0, 0.0)), Velocity::default(), MassProps::from_box_density(Vec3::new(0.5, 0.025, 0.025), 1000.0), true);
        let fa = if k == 0 { iso(Vec3::ZERO) } else { iso(Vec3::new(0.5, 0.0, 0.0)) };
        let j = w.add_hinge_joint(prev, r, fa, iso(Vec3::new(-0.5, 0.0, 0.0)), 2);
        match k {
            0 => w.set_d6_limit_r(j, 2, Some(D6Limit { min: -1.0, max: 1.0 })),
            1 => w.set_d6_drive_r(j, 2, Some(D6Drive { target: 2.0, kp: 0.0, kd: 0.0, max_impulse: 0.05, vel_mode: true })),
            _ => {}
        }
        ids.push(r);
        prev = r;
    }
    (w, ids)
}

#[test]
#[ignore]
fn joint_bits() {
    for hz in [60.0f32, 240.0] {
        let (mut w, ids) = chain();
        for _ in 0..(5.0 * hz) as usize { w.step(1.0 / hz); }
        let mut bits: u64 = 0xcbf29ce484222325;
        for &b in &ids {
            let (q, v) = (w.get_body_pose(b), w.get_body_vel(b));
            for x in [q.pos.x, q.pos.y, q.pos.z, q.rot.x, q.rot.y, q.rot.z, q.rot.w, v.lin.x, v.lin.y, v.lin.z, v.ang.x, v.ang.y, v.ang.z] {
                bits = (bits ^ x.to_bits() as u64).wrapping_mul(0x100000001b3);
            }
        }
        println!("joint_bits {} Hz: {:016x}", hz, bits);
    }
}
