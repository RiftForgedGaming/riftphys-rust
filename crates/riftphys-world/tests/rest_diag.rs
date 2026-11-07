// SPDX-License-Identifier: RiftForged Proprietary License
//! Diagnostics for stacks that creep sideways and fall while kept awake (PRECISION_SPEC 3.5's
//! stack ticket). Run with
//!     cargo test -p riftphys-world --release --test rest_diag -- --ignored --nocapture
use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, pair_props, MatPairProps, MaterialId};
use riftphys_world::world::World;

const G: f32 = 9.81;

#[derive(Clone, Copy)]
struct Case {
    name: &'static str,
    n: usize,
    gap: f32,
    iters: (u32, u32),
    tweak: fn(MatPairProps) -> MatPairProps,
}

fn same(p: MatPairProps) -> MatPairProps { p }
fn no_bounce(mut p: MatPairProps) -> MatPairProps { p.restitution = 0.0; p }
fn frictionless(mut p: MatPairProps) -> MatPairProps { p.mu_s = 0.0; p.mu_k = 0.0; p }
fn patch(mut p: MatPairProps) -> MatPairProps { p.patch_friction = true; p }
fn grippy(mut p: MatPairProps) -> MatPairProps { p.mu_s = 5.0; p.mu_k = 5.0; p }

/// n rubber boxes 0.4 x 0.2 x 0.4 m (5 kg) on a static concrete ground body, started `gap` apart,
/// kept awake for `secs`. Returns (first time any box is 2 cm off its column or None, worst
/// horizontal offset at the end, worst tilt in degrees at the end, per-box offsets at the end).
fn stack(c: Case, hz: f32, secs: f32) -> (Option<f32>, f32, f32, Vec<Vec3>) {
    let mut w = World::with_capacity(16, 16);
    w.set_gravity(Vec3::new(0.0, -G, 0.0));
    w.set_solver_iterations(c.iters.0, c.iters.1);
    for (a, b) in [(MaterialId::Rubber, MaterialId::Rubber), (MaterialId::Rubber, MaterialId::Concrete)] {
        w.set_pair_props(a, b, (c.tweak)(pair_props(a, b)));
    }
    let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(MaterialId::Concrete));
    let half = Vec3::new(0.2, 0.1, 0.2);
    let ids: Vec<BodyId> = (0..c.n).map(|i| {
        let b = w.add_body(Isometry { pos: Vec3::new(0.0, 0.1 + (0.2 + c.gap) * i as f32, 0.0), rot: Quat::IDENTITY }, Velocity::default(),
                           MassProps::from_box_density(half, 5.0 / (8.0 * half.x * half.y * half.z)), true);
        w.add_collider(b, Shape::Box { hx: half.x, hy: half.y, hz: half.z }, material(MaterialId::Rubber));
        w.set_body_sleep_allowed(b, false);
        b
    }).collect();
    let mut crossed = None;
    for k in 1..=(secs * hz) as usize {
        w.step(1.0 / hz);
        if crossed.is_none() && ids.iter().any(|&b| { let p = w.get_body_pose(b).pos; (p.x * p.x + p.z * p.z).sqrt() > 0.02 }) {
            crossed = Some(k as f32 / hz);
        }
    }
    let offs: Vec<Vec3> = ids.iter().map(|&b| { let p = w.get_body_pose(b).pos; Vec3::new(p.x, p.y, p.z) }).collect();
    let worst = offs.iter().map(|p| (p.x * p.x + p.z * p.z).sqrt()).fold(0.0f32, f32::max);
    let tilt = ids.iter().map(|&b| (w.get_body_pose(b).rot * Vec3::Y).y.clamp(-1.0, 1.0).acos().to_degrees()).fold(0.0f32, f32::max);
    (crossed, worst, tilt, offs)
}

#[test]
#[ignore]
fn stack_creep_variants() {
    let cases = [
        Case { name: "3 boxes, base", n: 3, gap: 0.002, iters: (20, 20), tweak: same },
        Case { name: "2 boxes, base", n: 2, gap: 0.002, iters: (20, 20), tweak: same },
        Case { name: "3, no bounce", n: 3, gap: 0.002, iters: (20, 20), tweak: no_bounce },
        Case { name: "3, frictionless", n: 3, gap: 0.002, iters: (20, 20), tweak: frictionless },
        Case { name: "3, mu 5", n: 3, gap: 0.002, iters: (20, 20), tweak: grippy },
        Case { name: "3, patch friction", n: 3, gap: 0.002, iters: (20, 20), tweak: patch },
        Case { name: "3, iters 60/60", n: 3, gap: 0.002, iters: (60, 60), tweak: same },
        Case { name: "3, iters 20/1", n: 3, gap: 0.002, iters: (20, 1), tweak: same },
        Case { name: "3, iters 60/20", n: 3, gap: 0.002, iters: (60, 20), tweak: same },
        Case { name: "3, iters 20/60", n: 3, gap: 0.002, iters: (20, 60), tweak: same },
        Case { name: "3, iters 120/20", n: 3, gap: 0.002, iters: (120, 20), tweak: same },
        Case { name: "3 frictionless 60/60", n: 3, gap: 0.002, iters: (60, 60), tweak: frictionless },
        Case { name: "3, touching", n: 3, gap: 0.0, iters: (20, 20), tweak: same },
    ];
    for c in cases {
        for hz in [60.0f32, 240.0] {
            let (t, worst, tilt, offs) = stack(c, hz, 30.0);
            let t = t.map_or("never".to_string(), |t| format!("{:.2} s", t));
            let o: Vec<String> = offs.iter().map(|p| format!("({:+.3},{:.3},{:+.3})", p.x, p.y, p.z)).collect();
            println!("{:20} {:4.0} Hz: 2 cm at {:>7} | end worst {:.3} m, tilt {:5.1} deg | {}", c.name, hz, t, worst, tilt, o.join(" "));
        }
    }
}
