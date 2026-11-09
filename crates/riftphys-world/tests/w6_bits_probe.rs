// SPDX-License-Identifier: RiftForged Proprietary License
// Probe (PARITY_SPEC W6): final pose and velocity bits of jointed bodies striking the floor
// faster than 1 m/s, for checking that W6 leaves contacts on jointed bodies bit-identical.
// Run with --ignored --nocapture, before and after the change.
use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::World;

/// A 1 m rubber rod hinged at a pivot 0.8 m up, released horizontal: it swings into a rubber
/// floor at about 4 m/s, bounces and rattles. Plus a 2-link hinged chain dropped as a free
/// assembly onto the floor (jointed to each other, not to anything static).
fn scene() -> (World, Vec<BodyId>) {
    let mut w = World::with_capacity(16, 16);
    w.set_gravity(Vec3::new(0.0, -9.81, 0.0));
    let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(MaterialId::Rubber));
    let half = Vec3::new(0.5, 0.04, 0.04);
    let density = 2.0 / (8.0 * half.x * half.y * half.z);
    let p = w.add_body(Isometry { pos: Vec3::new(0.0, 0.8, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    let rod = w.add_body(Isometry { pos: Vec3::new(0.5, 0.8, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::from_box_density(half, density), true);
    w.add_collider(rod, Shape::Box { hx: half.x, hy: half.y, hz: half.z }, material(MaterialId::Rubber));
    w.set_body_sleep_allowed(rod, false);
    w.add_hinge_joint(p, rod, Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY }, Isometry { pos: Vec3::new(-0.5, 0.0, 0.0), rot: Quat::IDENTITY }, 2);
    let mut ids = vec![rod];
    let mut prev: Option<BodyId> = None;
    for k in 0..2 {
        let pos = Vec3::new(3.0 + k as f32, 1.5 + 0.3 * k as f32, 0.0);
        let link = w.add_body(Isometry { pos, rot: Quat::IDENTITY }, Velocity::default(), MassProps::from_box_density(half, density), true);
        w.add_collider(link, Shape::Box { hx: half.x, hy: half.y, hz: half.z }, material(MaterialId::Rubber));
        w.set_body_sleep_allowed(link, false);
        if let Some(a) = prev {
            w.add_hinge_joint(a, link, Isometry { pos: Vec3::new(0.5, 0.0, 0.0), rot: Quat::IDENTITY }, Isometry { pos: Vec3::new(-0.5, 0.0, 0.0), rot: Quat::IDENTITY }, 2);
        }
        prev = Some(link);
        ids.push(link);
    }
    (w, ids)
}

#[test]
#[ignore]
fn w6_jointed_bits() {
    for hz in [60.0f32, 240.0] {
        let (mut w, ids) = scene();
        for _ in 0..(3.0 * hz) as usize { w.step(1.0 / hz); }
        let mut bits: u64 = 0xcbf29ce484222325;
        for &b in &ids {
            let (q, v) = (w.get_body_pose(b), w.get_body_vel(b));
            for x in [q.pos.x, q.pos.y, q.pos.z, q.rot.x, q.rot.y, q.rot.z, q.rot.w, v.lin.x, v.lin.y, v.lin.z, v.ang.x, v.ang.y, v.ang.z] {
                bits = (bits ^ x.to_bits() as u64).wrapping_mul(0x100000001b3);
            }
        }
        println!("w6_jointed_bits {} Hz: {:016x} hash {:02x?}", hz, bits, &w.step_hash()[..8]);
    }
}

/// The box-on-box drop of `restitution_toi_test`: the lower box's height range, and when.
#[test]
#[ignore]
fn w6_box_on_box_probe() {
    use riftphys_materials::materials::MaterialId as M;
    for hz in [60.0f32, 240.0] {
        let mut w = World::with_capacity(16, 16);
        w.set_gravity(Vec3::new(0.0, -9.81, 0.0));
        let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
        w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(M::Concrete));
        let half = Vec3::new(0.2, 0.1, 0.2);
        let mk = |w: &mut World, y: f32| {
            let b = w.add_body(Isometry { pos: Vec3::new(0.0, y, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::from_box_density(half, 2.0 / (8.0 * half.x * half.y * half.z)), true);
            w.add_collider(b, Shape::Box { hx: half.x, hy: half.y, hz: half.z }, material(M::Aluminum));
            w.set_body_sleep_allowed(b, false);
            b
        };
        let low = mk(&mut w, 0.1);
        let up = mk(&mut w, 1.3);
        let mut log = Vec::new();
        for k in 0..(1.5 * hz) as usize {
            w.step(1.0 / hz);
            let (yl, yu, vu) = (w.get_body_pose(low).pos.y, w.get_body_pose(up).pos.y, w.get_body_vel(up).lin.y);
            if (yl - 0.1).abs() > 2e-4 { log.push(format!("  step {} t {:.3}: low y {:.5} vy {:.4}, up y {:.4} vy {:.3}", k, k as f32 / hz, yl, w.get_body_vel(low).lin.y, yu, vu)); }
        }
        println!("box_on_box {} Hz: {} steps with the lower box off 0.1 by > 0.2 mm", hz, log.len());
        for l in log.iter().take(12) { println!("{}", l); }
    }
}

/// at_rest_test's 45-degree yawed wood stack at 60 Hz: steps where a box's contacts all carry
/// zero normal impulse (so W6's H2 would move it ballistically).
#[test]
#[ignore]
fn w6_yawed_stack_probe() {
    use riftphys_materials::materials::MaterialId as M;
    let mut w = World::with_capacity(32, 32);
    w.set_gravity(Vec3::new(0.0, -9.81, 0.0));
    let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(M::Concrete));
    let half = Vec3::new(0.2, 0.1, 0.2);
    let ids: Vec<BodyId> = (0..3).map(|i| {
        let rot = if i == 1 { Quat::from_rotation_y(45f32.to_radians()) } else { Quat::IDENTITY };
        let b = w.add_body(Isometry { pos: Vec3::new(0.0, 0.1 + 0.21 * i as f32, 0.0), rot }, Velocity::default(), MassProps::from_box_density(half, 5.0 / (8.0 * half.x * half.y * half.z)), true);
        w.add_collider(b, Shape::Box { hx: half.x, hy: half.y, hz: half.z }, material(M::Wood));
        w.set_body_sleep_allowed(b, false);
        b
    }).collect();
    let mut shown = 0;
    let mut cold = 0usize;
    for k in 0..(61.0 * 60.0) as usize {
        w.step(1.0 / 60.0);
        if k > 60 && w.warm_matched() != w.last_frame_contacts.len() { cold += 1; if cold < 6 { println!("step {}: cold, {} of {} warm", k, w.warm_matched(), w.last_frame_contacts.len()); } }
        if k < 240 && k % 4 == 0 {
            let t = |b: BodyId| ((w.get_body_pose(b).rot * Vec3::Y).dot(Vec3::Y)).clamp(-1.0, 1.0).acos().to_degrees();
            let imp: Vec<String> = w.last_frame_contacts.iter().map(|c| format!("{}-{}:{:.2}", c.a_collider, c.b_collider, c.impulse)).collect();
            println!("  k {} y {:.5} {:.5} {:.5} tilt {:.3} {:.3} {:.3} w1 {:?} | {}", k, w.get_body_pose(ids[0]).pos.y, w.get_body_pose(ids[1]).pos.y, w.get_body_pose(ids[2]).pos.y, t(ids[0]), t(ids[1]), t(ids[2]), w.get_body_vel(ids[1]).ang, imp.join(" "));
        }
        if k % 600 == 0 { println!("t {:.0} s: top {:?} tilt mid {:.4} deg, contacts {}", k as f32 / 60.0, w.get_body_pose(ids[2]).pos, ((w.get_body_pose(ids[1]).rot * Vec3::Y).dot(Vec3::Y)).clamp(-1.0, 1.0).acos().to_degrees(), w.last_frame_contacts.len()); }
        for (n, &b) in ids.iter().enumerate() {
            let col = n + 1;
            let mine: Vec<&riftphys_world::world::world::Contact> = w.last_frame_contacts.iter().filter(|c| c.a_collider == col || c.b_collider == col).collect();
            if !mine.is_empty() && mine.iter().all(|c| c.impulse == 0.0) && shown < 15 {
                shown += 1;
                let v = w.get_body_vel(b);
                println!("step {} box {}: {} contacts all zero, depths {:?}, v {:?} w {:?}", k, n, mine.len(), mine.iter().map(|c| c.depth - 0.05).collect::<Vec<_>>(), v.lin, v.ang);
            }
        }
    }
    println!("cold steps after 1 s: {}", cold);
    println!("yawed probe: {} reports; final y {:?}", shown, ids.iter().map(|&b| w.get_body_pose(b).pos).collect::<Vec<_>>());
}

/// The 45-degree yawed wood stack at 60 Hz dropped from several gaps: worst movement from 1 s
/// to 21 s, and steps with a cold contact. Is its landing fragile before W6 too?
#[test]
#[ignore]
fn w6_yawed_gap_sweep() {
    use riftphys_materials::materials::MaterialId as M;
    for gap in [0.0f32, 0.004, 0.008, 0.01, 0.012, 0.016, 0.02] {
        let mut w = World::with_capacity(32, 32);
        w.set_gravity(Vec3::new(0.0, -9.81, 0.0));
        let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
        w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(M::Concrete));
        let half = Vec3::new(0.2, 0.1, 0.2);
        let ids: Vec<BodyId> = (0..3).map(|i| {
            let rot = if i == 1 { Quat::from_rotation_y(45f32.to_radians()) } else { Quat::IDENTITY };
            let b = w.add_body(Isometry { pos: Vec3::new(0.0, 0.1 + (0.2 + gap) * i as f32, 0.0), rot }, Velocity::default(), MassProps::from_box_density(half, 5.0 / (8.0 * half.x * half.y * half.z)), true);
            w.add_collider(b, Shape::Box { hx: half.x, hy: half.y, hz: half.z }, material(M::Wood));
            w.set_body_sleep_allowed(b, false);
            b
        }).collect();
        for _ in 0..60 { w.step(1.0 / 60.0); }
        let p0: Vec<Vec3> = ids.iter().map(|&b| w.get_body_pose(b).pos).collect();
        let (mut worst, mut cold) = (0.0f32, 0usize);
        for _ in 0..1200 {
            w.step(1.0 / 60.0);
            if w.warm_matched() != w.last_frame_contacts.len() { cold += 1; }
            for (k, &b) in ids.iter().enumerate() { worst = worst.max((w.get_body_pose(b).pos - p0[k]).length()); }
        }
        println!("yawed 45 gap {:.3} m: worst move {:.3e} m, cold steps {}", gap, worst, cold);
    }
}

/// A box dropped 1 m onto a two-box stack at 60 Hz: B and C's heights and velocities around the hit.
#[test]
#[ignore]
fn w6_stack3_probe() {
    use riftphys_materials::materials::MaterialId as M;
    let mut w = World::with_capacity(16, 16);
    w.set_gravity(Vec3::new(0.0, -9.81, 0.0));
    let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(M::Concrete));
    let half = Vec3::new(0.2, 0.1, 0.2);
    let mk = |w: &mut World, y: f32| {
        let b = w.add_body(Isometry { pos: Vec3::new(0.0, y, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::from_box_density(half, 2.0 / (8.0 * half.x * half.y * half.z)), true);
        w.add_collider(b, Shape::Box { hx: half.x, hy: half.y, hz: half.z }, material(M::Aluminum));
        w.set_body_sleep_allowed(b, false);
        b
    };
    let c = mk(&mut w, 0.1);
    let b = mk(&mut w, 0.3);
    let a = mk(&mut w, 1.5);
    let mut spikes = vec![0usize; 10];
    for k in 0..600usize {
        w.step(1.0 / 60.0);
        if (84..420).contains(&k) {
            let st = w.last_bounce_stats();
            let va = w.get_body_vel(a).lin.y;
            if va.abs() > 0.02 || st.toi_rows > 0 || st.support_rows > 0 || w.last_ballistic(a) || !w.last_contact_loaded(a) {
                println!("  t {:.3} A vy {:+.3} ballistic {} loaded {} | toi {} support {} pseudo {} solves {}/{} residual {:.1e} | max approach {:.3} m/s | A y {:.5} B {:.5} C {:.5}",
                    k as f32 / 60.0, va, w.last_ballistic(a), w.last_contact_loaded(a), st.toi_rows, st.support_rows, st.pseudo_rows, st.solves, st.pseudo_solves, st.residual, st.max_approach,
                    w.get_body_pose(a).pos.y, w.get_body_pose(b).pos.y, w.get_body_pose(c).pos.y);
            }
        }
        if k >= 90 && w.get_body_vel(a).lin.y.abs() > 0.02 { spikes[k / 60] += 1; }
        if k == 599 { println!("A jolts > 0.02 m/s per second (from 1.5 s): {:?}; final A {:.5} B {:.5} C {:.5}", spikes, w.get_body_pose(a).pos.y, w.get_body_pose(b).pos.y, w.get_body_pose(c).pos.y); }
        if false {
            let imp: Vec<String> = w.last_frame_contacts.iter().map(|ct| format!("{}-{}:{:.2}", ct.a_collider, ct.b_collider, ct.impulse)).collect();
            println!("k {} A y {:.4} vy {:.3} | B y {:.5} vy {:.4} | C y {:.5} vy {:.4} | {}", k, w.get_body_pose(a).pos.y, w.get_body_vel(a).lin.y, w.get_body_pose(b).pos.y, w.get_body_vel(b).lin.y, w.get_body_pose(c).pos.y, w.get_body_vel(c).lin.y, imp.join(" "));
        }
    }
}

/// at_rest_test's 10-box wood stack (1 cm gaps) at 60 Hz: which W6 branches fire while it stands
#[test]
#[ignore]
fn w6_stack10_probe() {
    use riftphys_materials::materials::MaterialId as M;
    for hz in [60.0f32, 240.0] {
        let mut w = World::with_capacity(32, 32);
        w.set_gravity(Vec3::new(0.0, -9.81, 0.0));
        let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
        w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(M::Concrete));
        let half = Vec3::new(0.2, 0.1, 0.2);
        let ids: Vec<BodyId> = (0..10).map(|i| {
            let b = w.add_body(Isometry { pos: Vec3::new(0.0, 0.1 + 0.21 * i as f32, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::from_box_density(half, 5.0 / (8.0 * half.x * half.y * half.z)), true);
            w.add_collider(b, Shape::Box { hx: half.x, hy: half.y, hz: half.z }, material(M::Wood));
            w.set_body_sleep_allowed(b, false);
            b
        }).collect();
        let n = (61.0 * hz) as usize;
        let mut p0 = Vec::new();
        // per second: steps with any ballistic body, any unloaded body, toi rows, support rows; max approach; worst drift
        let mut row = (0usize, 0usize, 0usize, 0usize, 0.0f32, 0.0f32, String::new());
        for k in 0..n {
            w.step(1.0 / hz);
            if k + 1 == hz as usize { p0 = ids.iter().map(|&b| w.get_body_pose(b).pos).collect(); }
            let st = w.last_bounce_stats();
            let bal: Vec<usize> = ids.iter().enumerate().filter(|(_, &b)| w.last_ballistic(b)).map(|(i, _)| i).collect();
            if !bal.is_empty() { row.0 += 1; if row.6.len() < 40 { row.6 += &format!("{:?}", bal); } }
            if ids.iter().any(|&b| !w.last_contact_loaded(b)) { row.1 += 1; }
            if st.toi_rows > 0 { row.2 += 1; }
            if st.support_rows > 0 || st.pseudo_rows > 0 { row.3 += 1; }
            row.4 = row.4.max(st.max_approach);
            if hz == 60.0 && k < 22 {
                let (wi, wd) = ids.iter().enumerate().map(|(i, &b)| (i, if p0.is_empty() { 0.0 } else { (w.get_body_pose(b).pos - p0[i]).length() })).fold((0, 0.0f32), |m, x| if x.1 > m.1 { x } else { m });
                let ys: Vec<String> = ids.iter().map(|&b| format!("{:.4}", w.get_body_pose(b).pos.y)).collect();
                let vs: Vec<String> = ids.iter().map(|&b| format!("{:+.2}", w.get_body_vel(b).lin.y)).collect();
                let lat = ids.iter().map(|&b| { let p = w.get_body_pose(b).pos; (p.x * p.x + p.z * p.z).sqrt() }).fold(0.0f32, f32::max);
                let lats: Vec<String> = ids.iter().map(|&b| { let p = w.get_body_pose(b).pos; format!("{:.0e}", (p.x * p.x + p.z * p.z).sqrt()) }).collect();
                let angs: Vec<String> = ids.iter().map(|&b| format!("{:.0e}", w.get_body_vel(b).ang.length())).collect();
                println!("  k {:3} toi {} sup {} pse {} solves {}/{} appr {:.2} bal {:?} | y {} | vy {} | lat {} | ang {}", k, st.toi_rows, st.support_rows, st.pseudo_rows, st.solves, st.pseudo_solves, st.max_approach, bal, ys.join(" "), vs.join(" "), lats.join(" "), angs.join(" "));
                let _ = (lat, wi, wd);
            }
            if !p0.is_empty() { row.5 = row.5.max(ids.iter().zip(&p0).map(|(&b, p)| (w.get_body_pose(b).pos - *p).length()).fold(0.0, f32::max)); }
            if (k + 1) % (hz as usize) == 0 {
                let s = (k + 1) / hz as usize;
                if s <= 5 || s % 5 == 0 || row.0 + row.2 + row.3 > 0 {
                    println!("{} Hz s {:2}: ballistic steps {} unloaded {} toi {} support {} | max approach {:.3} | drift {:.2e} {}", hz, s, row.0, row.1, row.2, row.3, row.4, row.5, row.6);
                }
                row = (0, 0, 0, 0, 0.0, row.5, String::new());
            }
        }
    }
}

/// The 20-box pyramid of restitution_toi_test: per step around the impact, how far the pile has
/// moved and how fast it moves, with the bounce stats
#[test]
#[ignore]
fn w6_pile_probe() {
    use riftphys_materials::materials::MaterialId as M;
    let hz = 60.0f32;
    let mut w = World::with_capacity(32, 32);
    w.set_gravity(Vec3::new(0.0, -9.81, 0.0));
    let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(M::Concrete));
    let half = Vec3::new(0.2, 0.1, 0.2);
    let mk = |w: &mut World, p: Vec3| {
        let b = w.add_body(Isometry { pos: p, rot: Quat::IDENTITY }, Velocity::default(), MassProps::from_box_density(half, 2.0 / (8.0 * half.x * half.y * half.z)), true);
        w.add_collider(b, Shape::Box { hx: half.x, hy: half.y, hz: half.z }, material(M::Aluminum));
        w.set_body_sleep_allowed(b, false);
        b
    };
    let mut pile = Vec::new();
    for (layer, n) in [6usize, 5, 4, 3, 2].iter().enumerate() {
        for i in 0..*n {
            let x = (i as f32 - (*n as f32 - 1.0) * 0.5) * 0.4;
            pile.push(mk(&mut w, Vec3::new(x, 0.1 + 0.2 * layer as f32, 0.0)));
        }
    }
    for _ in 0..hz as usize { w.step(1.0 / hz); }
    let p0: Vec<Vec3> = pile.iter().map(|&b| w.get_body_pose(b).pos).collect();
    let top = mk(&mut w, Vec3::new(-0.2, 2.0, 0.0));
    for k in 0..48usize {
        w.step(1.0 / hz);
        let st = w.last_bounce_stats();
        if k < 24 || (k > 30 && st.toi_rows + st.support_rows == 0) { continue; }
        let (wi, moved) = pile.iter().zip(&p0).enumerate().map(|(i, (&b, p))| (i, (w.get_body_pose(b).pos - *p).length())).fold((0, 0.0f32), |m, x| if x.1 > m.1 { x } else { m });
        let vmax = pile.iter().map(|&b| w.get_body_vel(b).lin.length()).fold(0.0f32, f32::max);
        let dmove = w.get_body_pose(pile[wi]).pos - p0[wi];
        println!("  k {:2} top y {:.4} vy {:+.3} | toi {} sup {} pse {} solves {}/{} sweeps {}/{} | pile moved {:.2e} (box {} by {:+.1e} {:+.1e} {:+.1e}), fastest {:.2e} m/s",
            k, w.get_body_pose(top).pos.y, w.get_body_vel(top).lin.y, st.toi_rows, st.support_rows, st.pseudo_rows, st.solves, st.pseudo_solves, st.sweeps, st.pseudo_sweeps, moved, wi, dmove.x, dmove.y, dmove.z, vmax);
    }
}

/// PARITY_SPEC W6 cost (the Reviewer's item 1): the bounce step's time against the bounce
/// component's rows, on brick pyramids (base n, a box dropped 1 m onto the top brick), the
/// direct limit lifted. Release, single core.
#[test]
#[ignore]
fn w6_cost_probe() {
    use riftphys_materials::materials::MaterialId as M;
    // Each pyramid five times: the bounce passes' minimum (least contended) and median wall time
    for base in [5usize, 6, 7, 8, 9, 10, 11, 12, 13, 14] {
        let hz = 60.0f32;
        let mut times: Vec<f64> = Vec::new();
        let mut last = String::new();
        for _rep in 0..5 {
            let mut w = World::with_capacity(128, 128);
            w.set_gravity(Vec3::new(0.0, -9.81, 0.0));
            w.set_bounce_direct_max(usize::MAX);
            let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
            w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(M::Concrete));
            let half = Vec3::new(0.2, 0.1, 0.2);
            let mk = |w: &mut World, p: Vec3| {
                let b = w.add_body(Isometry { pos: p, rot: Quat::IDENTITY }, Velocity::default(), MassProps::from_box_density(half, 2.0 / (8.0 * half.x * half.y * half.z)), true);
                w.add_collider(b, Shape::Box { hx: half.x, hy: half.y, hz: half.z }, material(M::Aluminum));
                w.set_body_sleep_allowed(b, false);
                b
            };
            let mut bricks = 0;
            for layer in 0..base {
                let n = base - layer;
                for i in 0..n {
                    let x = (i as f32 - (n as f32 - 1.0) * 0.5) * 0.4;
                    mk(&mut w, Vec3::new(x, 0.1 + 0.2 * layer as f32, 0.0));
                    bricks += 1;
                }
            }
            for _ in 0..hz as usize { w.step(1.0 / hz); }
            mk(&mut w, Vec3::new(0.0, 0.1 + 0.2 * base as f32 + 1.0, 0.0));
            for _ in 0..(0.8 * hz) as usize {
                w.step(1.0 / hz);
                let st = w.last_bounce_stats();
                if st.toi_rows > 0 {
                    times.push(st.pass_us as f64 / 1e3);
                    last = format!("base {:2} ({:3} bricks): largest component {:3} rows, {} solves, fallbacks {} | fill: K {:.1}, L {:.1} per row below the diagonal",
                        base, bricks, st.largest, st.solves, st.fallbacks, st.k_nnz as f64 / st.largest.max(1) as f64, st.l_nnz as f64 / st.largest.max(1) as f64);
                    break;
                }
            }
        }
        times.sort_by(|a, b| a.partial_cmp(b).unwrap());
        println!("{} | passes min {:.2} ms, median {:.2} ms", last, times[0], times[times.len() / 2]);
    }
}
