// SPDX-License-Identifier: RiftForged Proprietary License
//! AT_REST_SPEC R1 acceptance: stacks stand while kept awake (contact warm start), keep standing
//! through despawns elsewhere and through sleep, a heavy box doesn't crush or launch a light
//! one, and the carried state (warm-start cache, last velocities, sleep) restores a world
//! exactly. The standing biped's gate is in riftphys-uriel (`biped::tests::standing_holds_still`).
use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, pair_props, MatPairProps, MaterialId};
use riftphys_world::world::World;

const G: f32 = 9.81;

fn ground(w: &mut World) {
    let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(MaterialId::Concrete));
}

fn boxed(w: &mut World, pos: Vec3, half: Vec3, mass: f32, mat: MaterialId, sleep: bool) -> BodyId {
    let b = w.add_body(Isometry { pos, rot: Quat::IDENTITY }, Velocity::default(), MassProps::from_box_density(half, mass / (8.0 * half.x * half.y * half.z)), true);
    w.add_collider(b, Shape::Box { hx: half.x, hy: half.y, hz: half.z }, material(mat));
    w.set_body_sleep_allowed(b, sleep);
    b
}

/// n wood boxes 0.4 x 0.2 x 0.4 m (5 kg), started 1 cm apart.
fn wood_stack(w: &mut World, n: usize, x: f32, sleep: bool) -> Vec<BodyId> {
    (0..n).map(|i| boxed(w, Vec3::new(x, 0.1 + 0.21 * i as f32, 0.0), Vec3::new(0.2, 0.1, 0.2), 5.0, MaterialId::Wood, sleep)).collect()
}

fn world() -> World {
    let mut w = World::with_capacity(32, 32);
    w.set_gravity(Vec3::new(0.0, -G, 0.0));
    ground(&mut w);
    w
}

/// Largest distance any of `ids` moves from where it is after `settle` s, over the next `secs` s.
fn worst_move(w: &mut World, ids: &[BodyId], hz: f32, settle: f32, secs: f32, mut each: impl FnMut(&mut World, usize)) -> f32 {
    for k in 0..(settle * hz) as usize { w.step(1.0 / hz); each(w, k); }
    let p0: Vec<Vec3> = ids.iter().map(|&b| w.get_body_pose(b).pos).collect();
    let mut worst = 0.0f32;
    for k in 0..(secs * hz) as usize {
        w.step(1.0 / hz);
        each(w, k + (settle * hz) as usize);
        for (b, p) in ids.iter().zip(&p0) { worst = worst.max((w.get_body_pose(*b).pos - *p).length()); }
    }
    worst
}

#[test]
fn stacks_stand_kept_awake() {
    // (boxes, Hz, gate on the worst box's movement over 60 s)
    for (n, hz, gate) in [(3usize, 60.0f32, 1e-4f32), (3, 240.0, 1e-4), (5, 60.0, 1e-4), (5, 240.0, 1e-4), (10, 60.0, 1e-2), (10, 240.0, 1e-3)] {
        let mut w = world();
        let ids = wood_stack(&mut w, n, 0.0, false);
        let worst = worst_move(&mut w, &ids, hz, 1.0, 60.0, |_, _| {});
        assert!(worst < gate, "{}-box stack at {} Hz: a box moved {:.3e} m in 60 s (gate {:.0e})", n, hz, worst, gate);
    }
}

#[test]
fn boxes_rest_at_their_true_height() {
    for hz in [60.0f32, 240.0] {
        let mut w = world();
        let ids = wood_stack(&mut w, 3, 0.0, false);
        for _ in 0..(2.0 * hz) as usize { w.step(1.0 / hz); }
        for (i, b) in ids.iter().enumerate() {
            let y = w.get_body_pose(*b).pos.y;
            let want = 0.1 + 0.2 * i as f32;
            assert!((y - want).abs() < 1e-3, "box {} at {} Hz rests at y {:.4}, want {:.3}", i, hz, y, want);
        }
    }
}

/// `rest_diag`'s stacks: 3 rubber boxes started 2 mm apart, iterations (20, 20), 30 s. First time
/// any box is 2 cm off its column, or None.
fn rubber_stack(n: usize, hz: f32, tweak: fn(MatPairProps) -> MatPairProps, gap: f32) -> Option<f32> {
    let mut w = World::with_capacity(16, 16);
    w.set_gravity(Vec3::new(0.0, -G, 0.0));
    for (a, b) in [(MaterialId::Rubber, MaterialId::Rubber), (MaterialId::Rubber, MaterialId::Concrete)] {
        w.set_pair_props(a, b, tweak(pair_props(a, b)));
    }
    ground(&mut w);
    let ids: Vec<BodyId> = (0..n).map(|i| boxed(&mut w, Vec3::new(0.0, 0.1 + (0.2 + gap) * i as f32, 0.0), Vec3::new(0.2, 0.1, 0.2), 5.0, MaterialId::Rubber, false)).collect();
    for k in 1..=(30.0 * hz) as usize {
        w.step(1.0 / hz);
        if ids.iter().any(|&b| { let p = w.get_body_pose(b).pos; (p.x * p.x + p.z * p.z).sqrt() > 0.02 }) { return Some(k as f32 / hz); }
    }
    None
}

#[test]
fn rubber_stack_variants_hold() {
    fn same(p: MatPairProps) -> MatPairProps { p }
    fn no_bounce(mut p: MatPairProps) -> MatPairProps { p.restitution = 0.0; p }
    fn frictionless(mut p: MatPairProps) -> MatPairProps { p.mu_s = 0.0; p.mu_k = 0.0; p }
    fn grippy(mut p: MatPairProps) -> MatPairProps { p.mu_s = 5.0; p.mu_k = 5.0; p }
    fn patch(mut p: MatPairProps) -> MatPairProps { p.patch_friction = true; p }
    let cases: [(&str, usize, fn(MatPairProps) -> MatPairProps, f32, &[f32]); 7] = [
        ("3 boxes", 3, same, 0.002, &[60.0, 240.0]),
        ("2 boxes", 2, same, 0.002, &[60.0, 240.0]),
        ("no bounce", 3, no_bounce, 0.002, &[60.0, 240.0]),
        ("mu 5", 3, grippy, 0.002, &[60.0, 240.0]),
        ("patch friction", 3, patch, 0.002, &[60.0, 240.0]),
        ("started touching", 3, same, 0.0, &[60.0, 240.0]),
        // A frictionless stack is only neutrally stable: at 60 Hz it still slides at about 9 s
        // (AT_REST_SPEC D3); 240 Hz holds
        ("frictionless", 3, frictionless, 0.002, &[240.0]),
    ];
    for (name, n, tweak, gap, rates) in cases {
        for &hz in rates {
            let t = rubber_stack(n, hz, tweak, gap);
            assert!(t.is_none(), "{} at {} Hz: a box was 2 cm off its column at {:.2} s", name, hz, t.unwrap());
        }
    }
}

#[test]
fn stack_holds_while_other_bodies_despawn() {
    // Removing a body drops only its own warm-start entries. The debris is added before the
    // stack, so each removal shifts the stack's collider indices (keys use body ids).
    let mut w = world();
    let mut debris: Vec<BodyId> = (0..40).map(|i| boxed(&mut w, Vec3::new(5.0 + 0.3 * i as f32, 0.1, 0.0), Vec3::splat(0.1), 1.0, MaterialId::Wood, false)).collect();
    let ids = wood_stack(&mut w, 5, 0.0, false);
    let mut fewest = usize::MAX;
    let worst = worst_move(&mut w, &ids, 60.0, 1.0, 60.0, |w, k| {
        // The step after each removal: every stack contact (5 pairs x 4) and every remaining
        // debris contact (4 each) started warm
        if k > 60 && k % 10 == 1 { fewest = fewest.min(w.warm_matched().saturating_sub(4 * debris.len())); }
        if k >= 60 && k % 10 == 0 && !debris.is_empty() { assert!(w.remove_body(debris.remove(0))); }
    });
    assert!(fewest >= 20, "after a despawn only {} stack contacts started warm", fewest);
    assert!(worst < 1e-4, "a stack box moved {:.3e} m while debris despawned", worst);
}

#[test]
fn sleeping_stack_wakes_warm() {
    let mut w = world();
    let ids = wood_stack(&mut w, 5, 0.0, true);
    let mut asleep_entries = None;
    for _ in 0..600 {
        w.step(1.0 / 60.0);
        // While the whole stack sleeps its entries stay put
        if w.num_sleeping() >= 5 {
            let n = w.warm_contacts();
            assert_eq!(*asleep_entries.get_or_insert(n), n, "warm entries changed while the stack slept");
        }
    }
    let asleep_entries = asleep_entries.expect("the stack never fell asleep");
    assert!(asleep_entries >= 20, "only {} warm entries for a sleeping 5-box stack", asleep_entries);
    // Nudge the top box awake, keep everything awake, stand 60 s more
    for &b in &ids { w.set_body_sleep_allowed(b, false); }
    w.apply_impulse(*ids.last().unwrap(), Vec3::new(0.05, 0.0, 0.0));
    w.step(1.0 / 60.0);
    assert!(w.warm_matched() >= 20, "the stack woke with only {} contacts warm", w.warm_matched());
    let p0: Vec<Vec3> = ids.iter().map(|&b| w.get_body_pose(b).pos).collect();
    for _ in 0..3600 { w.step(1.0 / 60.0); }
    for (i, (b, p)) in ids.iter().zip(&p0).enumerate() {
        let d = (w.get_body_pose(*b).pos - *p).length();
        assert!(d < 0.01, "box {} moved {:.3e} m after the nudge", i, d);
    }
}

#[test]
fn heavy_box_neither_crushes_nor_launches_a_light_one() {
    let mut w = world();
    let light = boxed(&mut w, Vec3::new(0.0, 0.5, 0.0), Vec3::splat(0.5), 1.0, MaterialId::Wood, false);
    let heavy = boxed(&mut w, Vec3::new(0.0, 2.0, 0.0), Vec3::splat(0.5), 1000.0, MaterialId::Wood, false);
    let mut up = 0.0f32;
    let mut lowest_light = f32::MAX;
    for _ in 0..180 {
        w.step(1.0 / 60.0);
        for b in [light, heavy] { up = up.max(w.get_body_vel(b).lin.y); }
        lowest_light = lowest_light.min(w.get_body_pose(light).pos.y);
    }
    assert!(lowest_light > 0.45, "the light box was crushed: centre down to y {:.3}", lowest_light);
    assert!(up < 0.5, "a box was launched upward at {:.2} m/s", up);
    let (pl, ph) = (w.get_body_pose(light).pos.y, w.get_body_pose(heavy).pos.y);
    assert!((pl - 0.5).abs() < 0.01 && (ph - 1.5).abs() < 0.02, "rest heights light {:.3} heavy {:.3}", pl, ph);
}

/// Poses, velocities and carried state of every body.
fn capture(w: &World, n: u32) -> (Vec<(Isometry, Velocity)>, riftphys_world::world::CarriedState) {
    ((0..n).map(|i| (w.get_body_pose(BodyId(i)), w.get_body_vel(BodyId(i)))).collect(), w.carried_state())
}

fn restore(w: &mut World, s: &(Vec<(Isometry, Velocity)>, riftphys_world::world::CarriedState), carried: bool) {
    for (i, (p, v)) in s.0.iter().enumerate() {
        w.set_body_pose(BodyId(i as u32), *p);
        w.set_body_vel(BodyId(i as u32), *v);
    }
    // Cold: the same restore with the warm-start cache cleared, so only the cache differs
    w.restore_carried_state(&s.1);
    if !carried { w.clear_solver_caches(); }
}

#[test]
fn carried_state_restores_exactly() {
    let mut w = world();
    let stack = wood_stack(&mut w, 3, 0.0, true);
    boxed(&mut w, Vec3::new(3.0, 3.0, 0.0), Vec3::splat(0.2), 2.0, MaterialId::Wood, true);
    // 15 steps: in contact, awake. 400: the stack asleep (the nudge after each restore wakes it,
    // so its carried entries are used)
    for settle in [15usize, 400] {
        for _ in 0..settle { w.step(1.0 / 60.0); }
        assert!(w.warm_contacts() > 0);
        assert_eq!(w.num_sleeping() >= 3, settle == 400, "{} asleep after {} steps", w.num_sleeping(), settle);
        let n = w.num_bodies();
        let snap = capture(&w, n);
        let top = *stack.last().unwrap();
        let run = |w: &mut World| -> Vec<[u8; 32]> {
            w.apply_impulse(top, Vec3::new(0.3, 0.0, 0.0));
            (0..90).map(|_| { w.step(1.0 / 60.0); w.step_hash() }).collect()
        };
        let h0 = w.step_hash();
        let first = run(&mut w);
        restore(&mut w, &snap, true);
        assert_eq!(w.step_hash(), h0, "the hash after a full restore differs (settled {} steps)", settle);
        assert_eq!(first, run(&mut w), "restore with carried state diverged (settled {} steps)", settle);
        // The carried state matters: a cold restore is a different run, and the step hash covers
        // the carried state (AT_REST_SPEC D2), so it differs before any step
        restore(&mut w, &snap, false);
        assert_ne!(w.step_hash(), h0, "a cold restore hashes like the full one (settled {} steps)", settle);
        assert_ne!(first, run(&mut w), "a cold restore matched: the test proves nothing (settled {} steps)", settle);
        restore(&mut w, &snap, true);
    }
}

#[test]
fn teleport_drops_only_that_bodys_warm_entries() {
    let mut w = world();
    let a = wood_stack(&mut w, 2, 0.0, false);
    wood_stack(&mut w, 2, 3.0, false);
    for _ in 0..60 { w.step(1.0 / 60.0); }
    let before = w.warm_contacts();
    // Top box of the first stack: its 4 contacts with the box below go; the other stack's stay
    let p = w.get_body_pose(a[1]);
    w.set_body_pose(a[1], Isometry { pos: p.pos + Vec3::new(0.0, 2.0, 0.0), rot: p.rot });
    assert_eq!(w.warm_contacts(), before - 4, "warm entries {} -> {}", before, w.warm_contacts());
    // Setting a velocity keeps them
    w.set_body_vel(a[0], Velocity::default());
    assert_eq!(w.warm_contacts(), before - 4);
}

// ---- AT_REST_SPEC R3: face clipping (box-box and box-triangle) ----

fn tilt_deg(q: Quat) -> f32 { (q * Vec3::Y).dot(Vec3::Y).clamp(-1.0, 1.0).acos().to_degrees() }

#[test]
fn yawed_stacks_stand() {
    // The middle box of 3 yawed: at 45 degrees no corner of either box is inside the other, so
    // the corner test gave one centre point and the stack toppled in about 2 s.
    // Started touching: landing from a gap is fragile at 45 degrees (it rocks for seconds after
    // some gaps, before W6 too) and is AT_REST_SPEC R3b, gated by w6_bits_probe's
    // w6_yawed_gap_sweep. This test is the resting contact.
    for (deg, hz, gate) in [(3.0f32, 60.0f32, 1e-4f32), (3.0, 240.0, 1e-4), (45.0, 60.0, 1e-4), (45.0, 240.0, 1e-4)] {
        let mut w = world();
        let ids: Vec<BodyId> = (0..3).map(|i| {
            let b = boxed(&mut w, Vec3::new(0.0, 0.1 + 0.2 * i as f32, 0.0), Vec3::new(0.2, 0.1, 0.2), 5.0, MaterialId::Wood, false);
            if i == 1 { w.set_body_pose(b, Isometry { pos: w.get_body_pose(b).pos, rot: Quat::from_rotation_y(deg.to_radians()) }); }
            b
        }).collect();
        // Every contact starts warm while the stack rests: a reference face that flipped between
        // the two boxes would keep a different 4 of the clipped points and start them cold
        let settle = hz as usize;
        let mut cold = 0usize;
        let worst = worst_move(&mut w, &ids, hz, 1.0, 60.0, |w, k| {
            if k > settle && w.warm_matched() != w.last_frame_contacts.len() { cold += 1; }
        });
        assert!(worst < gate, "stack yawed {} deg at {} Hz: a box moved {:.3e} m in 60 s (gate {:.0e})", deg, hz, worst, gate);
        assert_eq!(cold, 0, "stack yawed {} deg at {} Hz: {} steps had contacts start cold", deg, hz, cold);
        for &b in &ids { assert!(tilt_deg(w.get_body_pose(b).rot) < 1.0, "stack yawed {} deg at {} Hz tilted", deg, hz); }
    }
}

#[test]
fn box_on_a_ledge_stays_or_tips_by_its_centre_of_mass() {
    // A slab with its top at y = 0 and its edge at x = 0; a 0.4 m box hangs over the edge with
    // its centre of mass 1 cm inside (holds 60 s) or 1 cm past (tips off within 2 s)
    for hz in [60.0f32, 240.0] {
        for (off, stays) in [(-0.01f32, true), (0.01, false)] {
            let mut w = World::with_capacity(8, 8);
            w.set_gravity(Vec3::new(0.0, -G, 0.0));
            let g = w.add_body(Isometry { pos: Vec3::new(-5.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
            w.add_collider(g, Shape::Box { hx: 5.0, hy: 0.5, hz: 5.0 }, material(MaterialId::Concrete));
            let b = boxed(&mut w, Vec3::new(off, 0.1, 0.0), Vec3::new(0.2, 0.1, 0.2), 5.0, MaterialId::Wood, false);
            let secs = if stays { 60.0 } else { 2.0 };
            for _ in 0..(secs * hz) as usize { w.step(1.0 / hz); }
            let p = w.get_body_pose(b);
            if stays {
                assert!((p.pos - Vec3::new(off, 0.1, 0.0)).length() < 1e-3 && tilt_deg(p.rot) < 0.5, "box 1 cm inside the edge at {} Hz moved to {:?}, tilt {:.1}", hz, p.pos, tilt_deg(p.rot));
            } else {
                assert!(p.pos.y < -0.2 && tilt_deg(p.rot) > 30.0, "box 1 cm past the edge at {} Hz hasn't tipped off in 2 s: y {:.3}, tilt {:.1}", hz, p.pos.y, tilt_deg(p.rot));
            }
        }
    }
}

#[test]
fn box_rests_flat_on_a_heightfield() {
    // 0.25 m cells under a 0.4 m box: its base spans several triangles. One point per triangle,
    // on the triangle's edge, threw it off within 0.2 s
    use glam::{UVec2, Vec2};
    use riftphys_terrain::terrain::HeightField;
    for hz in [60.0f32, 240.0] {
        for (deg, c) in [(0.0f32, 2.0f32), (0.0, 2.1), (30.0, 2.05)] {
            let mut w = World::with_capacity(8, 8);
            w.set_gravity(Vec3::new(0.0, -G, 0.0));
            w.add_environment(Box::new(HeightField::from_heights(UVec2::new(20, 20), Vec2::new(0.25, 0.25), Vec2::ZERO, vec![0.0; 400], MaterialId::Concrete)));
            let b = boxed(&mut w, Vec3::new(c, 0.1, c), Vec3::new(0.2, 0.1, 0.2), 5.0, MaterialId::Wood, false);
            w.set_body_pose(b, Isometry { pos: Vec3::new(c, 0.1, c), rot: Quat::from_rotation_y(deg.to_radians()) });
            let worst = worst_move(&mut w, &[b], hz, 1.0, 60.0, |_, _| {});
            assert!(worst < 1e-4, "box yawed {} deg at x {} on a heightfield at {} Hz moved {:.3e} m", deg, c, hz, worst);
            assert!((w.get_body_pose(b).pos.y - 0.1).abs() < 1e-3);
        }
    }
}
