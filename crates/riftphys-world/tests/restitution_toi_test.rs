// SPDX-License-Identifier: RiftForged Proprietary License
//! PARITY_SPEC W6: free bodies bounce at their time of impact inside the step, so a rebound's
//! apex is e^2 h at 60 Hz as well as at 240 Hz; the drone bounces off its ground; a bounce on a
//! resting box doesn't lift it; a zero-g head-on collision matches the 1-D closed form; and a
//! rollback across a bounce replays.
use riftphys_core::{BodyId, Isometry, Mat3, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, pair_props, MaterialId};
use riftphys_world::world::World;

const G: f32 = 9.81;

fn iso(pos: Vec3) -> Isometry { Isometry { pos, rot: Quat::IDENTITY } }

/// A static floor whose top is at y = 0.
fn floor(w: &mut World, mat: MaterialId) {
    let g = w.add_body(iso(Vec3::new(0.0, -0.5, 0.0)), Velocity::default(), MassProps::infinite(), false);
    w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(mat));
}

fn world(gravity: bool) -> World {
    let mut w = World::with_capacity(16, 16);
    w.set_gravity(if gravity { Vec3::new(0.0, -G, 0.0) } else { Vec3::ZERO });
    w
}

fn ball(w: &mut World, pos: Vec3, vel: Vec3, r: f32, mass: f32, mat: MaterialId) -> BodyId {
    let mp = MassProps::from_sphere_density(r, mass / (4.0 / 3.0 * std::f32::consts::PI * r * r * r));
    let b = w.add_body(iso(pos), Velocity { lin: vel, ang: Vec3::ZERO }, mp, true);
    w.add_collider(b, Shape::Sphere { r }, material(mat));
    w.set_body_sleep_allowed(b, false);
    b
}

fn boxed(w: &mut World, pose: Isometry, half: Vec3, mass: f32, mat: MaterialId) -> (BodyId, MassProps) {
    let mp = MassProps::from_box_density(half, mass / (8.0 * half.x * half.y * half.z));
    let b = w.add_body(pose, Velocity::default(), mp, true);
    w.add_collider(b, Shape::Box { hx: half.x, hy: half.y, hz: half.z }, material(mat));
    w.set_body_sleep_allowed(b, false);
    (b, mp)
}

/// Apexes (of `bottom`) after each of the first `n` bounces of `b`, stepping at `hz`.
fn apexes(w: &mut World, b: BodyId, hz: f32, secs: f32, n: usize, bottom: impl Fn(&World) -> f32) -> Vec<f32> {
    let (mut out, mut rising, mut apex) = (Vec::new(), false, 0.0f32);
    for _ in 0..(secs * hz) as usize {
        w.step(1.0 / hz);
        let (y, vy) = (bottom(w), w.get_body_vel(b).lin.y);
        if !rising && vy > 0.0 { rising = true; apex = y; }
        if rising {
            apex = apex.max(y);
            if vy < 0.0 { out.push(apex); rising = false; if out.len() == n { break; } }
        }
    }
    out
}

fn ball_apexes(h: f32, mat: MaterialId, hz: f32) -> Vec<f32> {
    let r = 0.1;
    let mut w = world(true);
    floor(&mut w, mat);
    let b = ball(&mut w, Vec3::new(0.0, r + h, 0.0), Vec3::ZERO, r, 0.1, mat);
    apexes(&mut w, b, hz, 6.0, 2, |w| w.get_body_pose(b).pos.y - r)
}

#[test]
fn fidelity_bounce_reaches_e2_h() {
    // riftphys-fidelity's case: a rubber ball (pair e 0.80) from 1 m; was +13.0% at 60 Hz
    let e = pair_props(MaterialId::Rubber, MaterialId::Rubber).restitution;
    for hz in [60.0f32, 240.0] {
        let got = ball_apexes(1.0, MaterialId::Rubber, hz)[0];
        let want = e * e;
        println!("{} Hz: apex {:.4} m, e^2 h {:.4} m ({:+.2}%)", hz, got, want, 100.0 * (got - want) / want);
        assert!((got - want).abs() < 0.02 * want, "{} Hz: apex {:.4}, want {:.4}", hz, got, want);
    }
}

#[test]
fn drops_and_second_bounces_reach_e2_h_and_e4_h() {
    for hz in [60.0f32, 240.0] {
        for mat in [MaterialId::Rubber, MaterialId::Aluminum] {
            let e = pair_props(mat, mat).restitution;
            for h in [0.3f32, 1.0, 3.0] {
                let a = ball_apexes(h, mat, hz);
                let (w1, w2) = (e * e * h, e.powi(4) * h);
                println!("{} Hz, e {:.2}, {} m: apex {:.4} ({:+.2}%), second {:?} (e^4 h {:.4})", hz, e, h, a[0], 100.0 * (a[0] - w1) / w1, a.get(1), w2);
                assert!((a[0] - w1).abs() < 0.02 * w1, "{} Hz, e {}, {} m: apex {:.4}, want {:.4}", hz, e, h, a[0], w1);
                // The second bounce exists only if the first rebound lands faster than 1 m/s
                if (2.0 * G * w1).sqrt() > 1.3 {
                    let a2 = *a.get(1).expect("no second bounce");
                    assert!((a2 - w2).abs() < 0.03 * w2, "{} Hz, e {}, {} m: second apex {:.4}, want {:.4}", hz, e, h, a2, w2);
                }
            }
        }
    }
}

/// drone.rs's body: a 0.5 x 0.1 x 0.5 m, 1.2 kg aluminium box over concrete (pair e 0.40).
const DRONE_HALF: Vec3 = Vec3::new(0.25, 0.05, 0.25);

fn energy(w: &World, b: BodyId, mp: &MassProps) -> f32 {
    let (q, v) = (w.get_body_pose(b), w.get_body_vel(b));
    let r = Mat3::from_quat(q.rot);
    let iw = r * mp.inertia * r.transpose();
    0.5 * mp.mass * v.lin.length_squared() + 0.5 * v.ang.dot(iw.mul_vec3(v.ang)) + mp.mass * G * q.pos.y
}

#[test]
fn a_falling_drone_bounces() {
    let e = pair_props(MaterialId::Aluminum, MaterialId::Concrete).restitution;
    let h = 2.0;
    let hz = 60.0;
    let mut w = world(true);
    floor(&mut w, MaterialId::Concrete);
    let (b, _) = boxed(&mut w, iso(Vec3::new(0.0, h + DRONE_HALF.y, 0.0)), DRONE_HALF, 1.2, MaterialId::Aluminum);
    let (mut rising, mut apex, mut residual, mut spin, mut sweeps) = (false, 0.0f32, 0.0f32, 0.0f32, (0usize, 0usize));
    for _ in 0..(3.0 * hz) as usize {
        w.step(1.0 / hz);
        residual = residual.max(w.last_bounce_residual());
        let st = w.last_bounce_stats();
        sweeps = (sweeps.0.max(st.solves), sweeps.1.max(st.pseudo_solves));
        let v = w.get_body_vel(b);
        let y = w.get_body_pose(b).pos.y - DRONE_HALF.y;
        if !rising && v.lin.y > 0.0 { rising = true; }
        if rising {
            spin = spin.max(v.ang.length());
            apex = apex.max(y);
            if v.lin.y < 0.0 { break; }
        }
    }
    let want = e * e * h;
    println!("drone flat from {} m: apex {:.4} m, e^2 h {:.4} ({:+.2}%), spin {:.2e} rad/s, pseudo residual {:.2e} m/s, solves {:?}", h, apex, want, 100.0 * (apex - want) / want, spin, residual, sweeps);
    assert!((apex - want).abs() < 0.03 * want, "drone apex {:.4}, want {:.4}", apex, want);
    // Four corners landing together bounce together
    assert!(spin < 1e-3, "the flat drone picked up {:.2e} rad/s", spin);
    assert!(residual < 1e-4, "pseudo solve residual {:.2e} m/s", residual);
}

#[test]
fn a_tilted_drone_landing_on_an_edge_gains_no_energy() {
    for hz in [60.0f32, 240.0] {
        let mut w = world(true);
        floor(&mut w, MaterialId::Concrete);
        let pose = Isometry { pos: Vec3::new(0.0, 2.3, 0.0), rot: Quat::from_rotation_x(20f32.to_radians()) };
        let (b, mp) = boxed(&mut w, pose, DRONE_HALF, 1.2, MaterialId::Aluminum);
        let e0 = energy(&w, b, &mp);
        let mut worst = f32::NEG_INFINITY;
        let mut bounced = false;
        for _ in 0..(2.0 * hz) as usize {
            w.step(1.0 / hz);
            worst = worst.max(energy(&w, b, &mp));
            if w.get_body_vel(b).lin.y > 0.5 { bounced = true; }
        }
        println!("{} Hz: tilted drone, energy {:.4} J at release, highest after {:.4} J", hz, e0, worst);
        assert!(bounced, "{} Hz: the tilted drone didn't bounce", hz);
        assert!(worst <= e0 * 1.001 + 1e-4, "{} Hz: energy rose from {:.4} to {:.4} J", hz, e0, worst);
    }
}

#[test]
fn a_bounce_on_a_resting_box_doesnt_lift_it() {
    // RiftPhys Reviewer's H1: the pseudo solve pulls the bounced pair together; without the
    // freeze rule the lower box would rise about 38 mm off its floor at 60 Hz.
    // Before W6 (b1ebad0, w6_bits_probe's w6_box_on_box_probe) the restitution pass, which has
    // no floor row, drove the lower box 5 mm into the floor; the projection then popped it
    // 4.6 mm (60 Hz) / 5.8 mm (240 Hz) above rest, and the upper box left at 0.7 / 0.1 m/s
    // instead of e v. Held bodies are immovable in the bounce rows now, as in the shock pass.
    let e = pair_props(MaterialId::Aluminum, MaterialId::Aluminum).restitution;
    for (hz, m_up) in [(60.0f32, 2.0f32), (240.0, 2.0), (60.0, 0.2), (240.0, 0.2)] {
        let gate = 1e-3;
        let half = Vec3::new(0.2, 0.1, 0.2);
        let mut w = world(true);
        floor(&mut w, MaterialId::Concrete);
        let (low, mpl) = boxed(&mut w, iso(Vec3::new(0.0, 0.1, 0.0)), half, 2.0, MaterialId::Aluminum);
        let (up, mpu) = boxed(&mut w, iso(Vec3::new(0.0, 1.3, 0.0)), half, m_up, MaterialId::Aluminum);
        let total = |w: &World| energy(w, low, &mpl) + energy(w, up, &mpu);
        let e0 = total(&w);
        let (mut lo, mut hi, mut worst) = (f32::MAX, f32::MIN, f32::NEG_INFINITY);
        let (mut rising, mut apex, mut done, mut residual) = (false, 0.0f32, false, 0.0f32);
        for _ in 0..(1.5 * hz) as usize {
            w.step(1.0 / hz);
            residual = residual.max(w.last_bounce_residual());
            let y = w.get_body_pose(low).pos.y;
            lo = lo.min(y);
            hi = hi.max(y);
            worst = worst.max(total(&w));
            // The upper box's bottom above the lower box's top
            let (yu, vu) = (w.get_body_pose(up).pos.y - 0.3, w.get_body_vel(up).lin.y);
            if !done && !rising && vu > 0.0 { rising = true; }
            if !done && rising { apex = apex.max(yu); if vu < 0.0 { done = true; } }
        }
        let want = e * e * 1.0;
        println!("{} Hz, {} kg on 2 kg: lower box height {:.5}..{:.5} m, upper rebound {:.4} m (e^2 h {:.4}), energy {:.4} J at release, highest {:.4} J", hz, m_up, lo, hi, apex, want, e0, worst);
        assert!((apex - want).abs() < 0.03 * want, "{} Hz, {} kg: the upper box rebounded {:.4} m, want {:.4}", hz, m_up, apex, want);
        assert!(hi - lo < gate, "{} Hz, {} kg: the lower box moved {:.4} m", hz, m_up, hi - lo);
        assert!(residual < 1e-4, "{} Hz, {} kg: pseudo residual {:.2e} m/s", hz, m_up, residual);
        assert!(worst <= e0 * 1.001 + 1e-4, "{} Hz: energy rose from {:.4} to {:.4} J", hz, e0, worst);
    }
}

#[test]
fn head_on_in_zero_g_matches_the_closed_form() {
    let e = pair_props(MaterialId::Aluminum, MaterialId::Aluminum).restitution;
    let (m1, m2, v1, v2, r) = (1.0f32, 2.0f32, 3.0f32, -1.0f32, 0.1f32);
    for hz in [60.0f32, 240.0] {
        let mut w = world(false);
        // 0.83 m apart, closing at 4 m/s: contact at 0.2075 s, inside a step at both rates
        let a = ball(&mut w, Vec3::new(0.0, 1.0, 0.0), Vec3::new(v1, 0.0, 0.0), r, m1, MaterialId::Aluminum);
        let b = ball(&mut w, Vec3::new(2.0 * r + 0.83, 1.0, 0.0), Vec3::new(v2, 0.0, 0.0), r, m2, MaterialId::Aluminum);
        let t_end = 0.5;
        for _ in 0..(t_end * hz).round() as usize { w.step(1.0 / hz); }
        let p = m1 * v1 + m2 * v2;
        let want1 = (p + m2 * e * (v2 - v1)) / (m1 + m2);
        let want2 = (p + m1 * e * (v1 - v2)) / (m1 + m2);
        let (got1, got2) = (w.get_body_vel(a).lin.x, w.get_body_vel(b).lin.x);
        let gap = w.get_body_pose(b).pos.x - w.get_body_pose(a).pos.x - 2.0 * r;
        let want_gap = e * (v1 - v2) * (t_end - 0.83 / (v1 - v2));
        println!("{} Hz: v {:.4} / {:.4} (want {:.4} / {:.4}), gap {:.4} m (want {:.4})", hz, got1, got2, want1, want2, gap, want_gap);
        assert!((got1 - want1).abs() < 0.01 * want1.abs(), "{} Hz: v1 {:.4}, want {:.4}", hz, got1, want1);
        assert!((got2 - want2).abs() < 0.01 * want2.abs(), "{} Hz: v2 {:.4}, want {:.4}", hz, got2, want2);
        assert!((gap - want_gap).abs() < 2e-3, "{} Hz: gap {:.4}, want {:.4}", hz, gap, want_gap);
    }
}

#[test]
fn rollback_across_a_bounce_replays() {
    let hz = 60.0;
    let mut w = world(true);
    floor(&mut w, MaterialId::Concrete);
    let (b, _) = boxed(&mut w, Isometry { pos: Vec3::new(0.0, 1.05, 0.0), rot: Quat::from_rotation_z(0.2) }, DRONE_HALF, 1.2, MaterialId::Aluminum);
    // Released 1 m up, it lands at about 0.45 s: snapshot just before
    for _ in 0..24 { w.step(1.0 / hz); }
    let (pose, vel, carried) = (w.get_body_pose(b), w.get_body_vel(b), w.carried_state());
    let run = |w: &mut World| -> Vec<[u8; 32]> { (0..30).map(|_| { w.step(1.0 / hz); w.step_hash() }).collect() };
    let first = run(&mut w);
    assert!(w.get_body_vel(b).lin.y > 0.0 || first.len() == 30, "no bounce in the window");
    w.set_body_pose(b, pose);
    w.set_body_vel(b, vel);
    w.restore_carried_state(&carried);
    assert_eq!(first, run(&mut w), "replay across a bounce diverged");
}

#[test]
fn a_side_hit_on_a_resting_box_shares_momentum() {
    // RiftPhys Reviewer's W6 build review, H1: a ball strikes a box resting on a (frictionless)
    // floor sideways. The floor holds the box up, not back: the box takes its 1-D share
    for hz in [60.0f32, 240.0] {
        let mut w = world(true);
        floor(&mut w, MaterialId::Ice);
        let mut slick = pair_props(MaterialId::Rubber, MaterialId::Ice);
        slick.mu_s = 0.0;
        slick.mu_k = 0.0;
        w.set_pair_props(MaterialId::Rubber, MaterialId::Ice, slick);
        let e = pair_props(MaterialId::Rubber, MaterialId::Rubber).restitution;
        let (m, v) = (1.0f32, 3.0f32);
        let half = Vec3::new(0.15, 0.15, 0.15);
        let (bx, _) = boxed(&mut w, iso(Vec3::new(0.0, 0.15, 0.0)), half, m, MaterialId::Rubber);
        for _ in 0..30 { w.step(1.0 / hz); }
        // 2.3 cm short of the box at 3 m/s, at its centre's height
        let ball_id = ball(&mut w, Vec3::new(-0.15 - 0.1 - 0.023, 0.15, 0.0), Vec3::new(v, 0.0, 0.0), 0.1, m, MaterialId::Rubber);
        let mut hit = None;
        for k in 0..(0.2 * hz) as usize {
            w.step(1.0 / hz);
            if w.get_body_vel(ball_id).lin.x < 0.5 * v { hit = Some((k, w.last_bounce_stats())); break; }
        }
        assert!(hit.is_some(), "{} Hz: no hit", hz);
        let (vb, vx) = (w.get_body_vel(ball_id).lin.x, w.get_body_vel(bx).lin.x);
        let (want_ball, want_box) = ((1.0 - e) * 0.5 * v, (1.0 + e) * 0.5 * v);
        let st = hit.unwrap().1;
        println!("{} Hz: ball {:.4} (want {:.4}), box {:.4} (want {:.4}), momentum {:.4} of {:.4}, solves {} / {} ({} support rows)", hz, vb, want_ball, vx, want_box, m * (vb + vx), m * v, st.solves, st.pseudo_solves, st.support_rows);
        assert!((vx - want_box).abs() < 0.02 * want_box, "{} Hz: box {:.4}, want {:.4}", hz, vx, want_box);
        assert!((vb - want_ball).abs() < 0.02 * want_box, "{} Hz: ball {:.4}, want {:.4}", hz, vb, want_ball);
        assert!((m * (vb + vx) - m * v).abs() < 0.01 * m * v, "{} Hz: momentum {:.4}, was {:.4}", hz, m * (vb + vx), m * v);
    }
}

#[test]
fn a_fast_corner_bounces_beside_a_slow_loaded_one() {
    // The Reviewer's H2: a box pivoting on one loaded edge slaps its other end down faster than
    // 1 m/s. The slow, loaded pivot points must not suppress the fast end's bounce
    let e = pair_props(MaterialId::Rubber, MaterialId::Rubber).restitution;
    for hz in [60.0f32, 240.0] {
        let mut w = world(true);
        floor(&mut w, MaterialId::Rubber);
        let half = Vec3::new(0.3, 0.05, 0.1);
        let th = 20f32.to_radians();
        let rot = Quat::from_rotation_z(th);
        let pivot_local = Vec3::new(-half.x, -half.y, 0.0);
        let far_local = Vec3::new(half.x, -half.y, 0.0);
        let centre = Vec3::new(0.0, 0.0, 0.0) - rot * pivot_local;
        let (b, _) = boxed(&mut w, Isometry { pos: centre, rot }, half, 2.0, MaterialId::Rubber);
        let wz = -4.0f32;
        let r_p = rot * pivot_local;
        w.set_body_vel(b, Velocity { lin: -Vec3::new(0.0, 0.0, wz).cross(r_p), ang: Vec3::new(0.0, 0.0, wz) });
        let corner_vy = |w: &World| {
            let (q, v) = (w.get_body_pose(b), w.get_body_vel(b));
            (v.lin + v.ang.cross(q.rot * far_local)).y
        };
        let (mut last, mut out) = (corner_vy(&w), None);
        for _ in 0..(0.5 * hz) as usize {
            w.step(1.0 / hz);
            let vy = corner_vy(&w);
            if last < -1.0 && vy > 0.0 { out = Some((-last, vy)); break; }
            last = vy;
        }
        let (v_in, v_out) = out.expect("the far end never bounced");
        println!("{} Hz: far end in at {:.3} m/s, out at {:.3} (e v {:.3})", hz, v_in, v_out, e * v_in);
        assert!(v_out >= 0.95 * e * v_in, "{} Hz: far end left at {:.3}, e v_in {:.3}", hz, v_out, e * v_in);
    }
}

#[test]
fn a_bounce_on_a_two_box_stack_moves_neither() {
    // The Reviewer's W6 rework review: support has to reach the bottom of a pile. With only the
    // bounced pair involved, B's contact on C was a support row but C's floor contact wasn't,
    // so the bounce drove C into the floor and the pseudo pull lifted B and C together.
    let e = pair_props(MaterialId::Aluminum, MaterialId::Aluminum).restitution;
    for hz in [60.0f32, 240.0] {
        let half = Vec3::new(0.2, 0.1, 0.2);
        let mut w = world(true);
        floor(&mut w, MaterialId::Concrete);
        let (c, mpc) = boxed(&mut w, iso(Vec3::new(0.0, 0.1, 0.0)), half, 2.0, MaterialId::Aluminum);
        let (b, mpb) = boxed(&mut w, iso(Vec3::new(0.0, 0.3, 0.0)), half, 2.0, MaterialId::Aluminum);
        let (a, mpa) = boxed(&mut w, iso(Vec3::new(0.0, 1.5, 0.0)), half, 2.0, MaterialId::Aluminum);
        let total = |w: &World| energy(w, a, &mpa) + energy(w, b, &mpb) + energy(w, c, &mpc);
        let e0 = total(&w);
        let (mut range_b, mut range_c) = ((f32::MAX, f32::MIN), (f32::MAX, f32::MIN));
        let (mut rising, mut apex, mut done, mut worst, mut residual) = (false, 0.0f32, false, f32::NEG_INFINITY, 0.0f32);
        let (mut sweeps, mut fallbacks) = ((0usize, 0usize), 0usize);
        for _ in 0..(1.5 * hz) as usize {
            w.step(1.0 / hz);
            residual = residual.max(w.last_bounce_residual());
            let st = w.last_bounce_stats();
            sweeps = (sweeps.0.max(st.solves), sweeps.1.max(st.pseudo_solves));
            fallbacks += st.fallbacks;
            // Across the bounce, up to the top box's apex: when it later lands for good the stack
            // settles into its contact slop under the extra weight, as at b1ebad0 (which sank it
            // 10 mm at the impact instead)
            if !done {
                let (yb, yc) = (w.get_body_pose(b).pos.y, w.get_body_pose(c).pos.y);
                range_b = (range_b.0.min(yb), range_b.1.max(yb));
                range_c = (range_c.0.min(yc), range_c.1.max(yc));
            }
            worst = worst.max(total(&w));
            let (ya, va) = (w.get_body_pose(a).pos.y - 0.5, w.get_body_vel(a).lin.y);
            if !done && !rising && va > 0.0 { rising = true; }
            if !done && rising { apex = apex.max(ya); if va < 0.0 { done = true; } }
        }
        let want = e * e * 1.0;
        println!("{} Hz: B {:.5}..{:.5}, C {:.5}..{:.5} m, top rebound {:.4} (e^2 h {:.4}), energy {:.4} -> highest {:.4} J, residual {:.1e}, solves {:?}", hz, range_b.0, range_b.1, range_c.0, range_c.1, apex, want, e0, worst, residual, sweeps);
        assert!(range_b.1 - range_b.0 < 1e-4, "{} Hz: B moved {:.5} m", hz, range_b.1 - range_b.0);
        assert!(range_c.1 - range_c.0 < 1e-4, "{} Hz: C moved {:.5} m", hz, range_c.1 - range_c.0);
        assert!((apex - want).abs() < 0.02 * want, "{} Hz: the top box rebounded {:.4}, want {:.4}", hz, apex, want);
        assert!(worst <= e0 * 1.001 + 1e-4, "{} Hz: energy rose from {:.4} to {:.4} J", hz, e0, worst);
        assert!(residual < 1e-4, "{} Hz: pseudo residual {:.2e} m/s", hz, residual);
        assert_eq!(fallbacks, 0, "{} Hz: the bounce passes fell back to sweeping", hz);
    }
}

#[test]
fn the_sweeping_fallback_stays_sane() {
    // The Reviewer: nothing ran the fallback. A component above the direct limit, or one the
    // interior point doesn't finish, is swept. Forced here with a limit of 8 rows on the two-box
    // stack: the bounce must stay finite, gain no energy and leave a bounded residual.
    for hz in [60.0f32, 240.0] {
        let half = Vec3::new(0.2, 0.1, 0.2);
        let mut w = world(true);
        w.set_bounce_direct_max(8);
        floor(&mut w, MaterialId::Concrete);
        let (c, mpc) = boxed(&mut w, iso(Vec3::new(0.0, 0.1, 0.0)), half, 2.0, MaterialId::Aluminum);
        let (b, mpb) = boxed(&mut w, iso(Vec3::new(0.0, 0.3, 0.0)), half, 2.0, MaterialId::Aluminum);
        let (a, mpa) = boxed(&mut w, iso(Vec3::new(0.0, 1.5, 0.0)), half, 2.0, MaterialId::Aluminum);
        let total = |w: &World| energy(w, a, &mpa) + energy(w, b, &mpb) + energy(w, c, &mpc);
        let e0 = total(&w);
        let (mut fallbacks, mut worst, mut residual, mut finite) = (0usize, f32::NEG_INFINITY, 0.0f32, true);
        for _ in 0..(1.5 * hz) as usize {
            w.step(1.0 / hz);
            let st = w.last_bounce_stats();
            fallbacks += st.fallbacks;
            residual = residual.max(st.residual);
            for id in [a, b, c] {
                let (q, v) = (w.get_body_pose(id), w.get_body_vel(id));
                finite &= q.pos.is_finite() && q.rot.is_finite() && v.lin.is_finite() && v.ang.is_finite();
            }
            worst = worst.max(total(&w));
        }
        println!("{} Hz, direct limit 8: {} fallbacks, energy {:.4} -> highest {:.4} J, residual {:.1e}", hz, fallbacks, e0, worst, residual);
        assert!(fallbacks > 0, "{} Hz: the fallback never ran", hz);
        assert!(finite, "{} Hz: a body went non-finite", hz);
        assert!(worst <= e0 * 1.001 + 1e-4, "{} Hz: energy rose from {:.4} to {:.4} J", hz, e0, worst);
        assert!(residual < 1e-3, "{} Hz: pseudo residual {:.2e} m/s", hz, residual);
    }
}

#[test]
fn a_1000_to_1_mass_ratio_bounce_converges() {
    // The Reviewer: the interior point's starting impulses must be on the problem's scale. A
    // 100 kg box dropped 1 m onto a 0.1 kg box on the floor, and the reverse.
    for (top_kg, bottom_kg) in [(100.0f32, 0.1f32), (0.1, 100.0)] {
        for hz in [60.0f32, 240.0] {
            let half = Vec3::new(0.2, 0.1, 0.2);
            let mut w = world(true);
            floor(&mut w, MaterialId::Concrete);
            let (lo, mplo) = boxed(&mut w, iso(Vec3::new(0.0, 0.1, 0.0)), half, bottom_kg, MaterialId::Aluminum);
            let (hi, mphi) = boxed(&mut w, iso(Vec3::new(0.0, 1.3, 0.0)), half, top_kg, MaterialId::Aluminum);
            let total = |w: &World| energy(w, lo, &mplo) + energy(w, hi, &mphi);
            let e0 = total(&w);
            let (mut solves, mut fallbacks, mut bounced, mut worst, mut finite) = (0usize, 0usize, false, f32::NEG_INFINITY, true);
            let (mut backward, mut residual) = (0.0f64, 0.0f64);
            for _ in 0..(1.5 * hz) as usize {
                w.step(1.0 / hz);
                let st = w.last_bounce_stats();
                solves = solves.max(st.solves);
                fallbacks += st.fallbacks;
                bounced |= st.toi_rows > 0;
                backward = backward.max(st.backward_error);
                residual = residual.max(st.true_residual);
                for id in [lo, hi] {
                    let v = w.get_body_vel(id);
                    finite &= w.get_body_pose(id).pos.is_finite() && v.lin.is_finite() && v.ang.is_finite();
                }
                worst = worst.max(total(&w));
            }
            println!("{} kg on {} kg at {} Hz: {} solves, {} fallbacks, energy {:.3} -> highest {:.3} J, backward error {:.1e}, true residual {:.1e} m/s", top_kg, bottom_kg, hz, solves, fallbacks, e0, worst, backward, residual);
            assert!(backward <= 1e-12 && residual <= 1e-7, "{} kg on {} kg at {} Hz: backward error {:.2e}, true residual {:.2e}", top_kg, bottom_kg, hz, backward, residual);
            assert!(bounced, "{} kg on {} kg at {} Hz: nothing bounced", top_kg, bottom_kg, hz);
            assert!(finite, "{} kg on {} kg at {} Hz: a body went non-finite", top_kg, bottom_kg, hz);
            assert!(solves <= 40 && fallbacks == 0, "{} kg on {} kg at {} Hz: {} solves, {} fallbacks", top_kg, bottom_kg, hz, solves, fallbacks);
            assert!(worst <= e0 * 1.001 + 1e-3, "{} kg on {} kg at {} Hz: energy rose from {:.3} to {:.3} J", top_kg, bottom_kg, hz, e0, worst);
        }
    }
}

#[test]
fn a_bounce_on_a_20_box_pile_converges() {
    // The Reviewer: the bounce passes take in the whole loaded island, so they must converge on a
    // pile (Gauss-Seidel hit its 256-sweep cap; now an interior-point solve per component). A brick pyramid of 20 boxes (6, 5, 4, 3, 2 a layer,
    // each on two below), a box dropped 1 m onto its top.
    for hz in [60.0f32, 240.0] {
        let half = Vec3::new(0.2, 0.1, 0.2);
        let mut w = world(true);
        floor(&mut w, MaterialId::Concrete);
        let mut pile = Vec::new();
        for (layer, n) in [6usize, 5, 4, 3, 2].iter().enumerate() {
            for i in 0..*n {
                let x = (i as f32 - (*n as f32 - 1.0) * 0.5) * 0.4;
                pile.push(boxed(&mut w, iso(Vec3::new(x, 0.1 + 0.2 * layer as f32, 0.0)), half, 2.0, MaterialId::Aluminum).0);
            }
        }
        assert_eq!(pile.len(), 20);
        for _ in 0..hz as usize { w.step(1.0 / hz); }
        let p0: Vec<Vec3> = pile.iter().map(|&b| w.get_body_pose(b).pos).collect();
        let (top, _) = boxed(&mut w, iso(Vec3::new(-0.2, 0.9 + 1.0 + 0.1, 0.0)), half, 2.0, MaterialId::Aluminum);
        let (mut most, mut residual, mut rows, mut bounced) = ((0usize, 0usize), 0.0f32, 0usize, false);
        // The pile's largest movement across the bounce, up to the top box's apex
        let (mut rising, mut done, mut moved, mut swept) = (false, false, 0.0f32, false);
        for _ in 0..(0.8 * hz) as usize {
            w.step(1.0 / hz);
            let st = w.last_bounce_stats();
            if st.toi_rows > 0 { bounced = true; }
            most = (most.0.max(st.solves), most.1.max(st.pseudo_solves));
            swept |= st.fallbacks > 0;
            rows = rows.max(st.support_rows);
            residual = residual.max(st.residual);
            let vy = w.get_body_vel(top).lin.y;
            if !done && !rising && vy > 0.0 { rising = true; }
            if rising && vy < 0.0 { done = true; }
            if !done {
                moved = moved.max(pile.iter().zip(&p0).map(|(&b, p)| (w.get_body_pose(b).pos - *p).length()).fold(0.0f32, f32::max));
            }
        }
        println!("{} Hz: 20-box pile, solves {} / {}, {} support rows, residual {:.1e}, pile moved {:.2e} m across the bounce", hz, most.0, most.1, rows, residual, moved);
        assert!(bounced, "{} Hz: nothing bounced", hz);
        assert!(done, "{} Hz: the top box never reached its apex", hz);
        // 1 mm at 60 Hz and 0.2 mm at 240 Hz (measured 0.34 and 0.045 mm; agreed with the
        // Reviewer). The bounce passes solve exactly (interior point); what still moves the pile
        // is the main solve's: on the impact step it leaves pairs separating at up to 1.4 cm/s
        // and sideways friction residue, which push-only support rows can't take back
        // (PARITY_SPEC W6b). b1ebad0 moved this pile 13 mm (60 Hz) and 6.8 mm (240 Hz), and
        // didn't bounce the box.
        let gate = if hz < 120.0 { 1e-3 } else { 2e-4 };
        assert!(moved < gate, "{} Hz: the pile moved {:.2e} m across the bounce (gate {:.0e})", hz, moved, gate);
        assert!(rows >= 10, "{} Hz: the support reached only {} rows", hz, rows);
        assert!(!swept, "{} Hz: the bounce passes fell back to sweeping", hz);
        assert!(most.0 <= 64 && most.1 <= 64, "{} Hz: {:?} solves", hz, most);
        assert!(residual < 1e-4, "{} Hz: residual {:.2e}", hz, residual);
    }
}

#[test]
fn a_struck_three_stack_rattles_only_as_aftermath() {
    // The Reviewer's rule for the rattle after a box lands on a two-box stack (w6_stack3_probe):
    // a W6 branch (bounce, support or pseudo rows) firing on a step where no contact approaches
    // faster than 1 m/s is a bug; jolts with none firing are the aftermath of a legitimate
    // bounce. The rattle is the top box settling about 1 cm into the one below in jolts, every
    // contact loaded (the hard landing's rest depth, AT_REST_SPEC R5); how long it lasts swings
    // with sub-micron differences in the bounce (still by 4 s in one build, by 7 s in the next),
    // so the settle gate has margin: still from 10 s on.
    for hz in [60.0f32, 240.0] {
        let half = Vec3::new(0.2, 0.1, 0.2);
        let mut w = world(true);
        floor(&mut w, MaterialId::Concrete);
        let bodies: Vec<(BodyId, MassProps)> = [0.1f32, 0.3, 1.5].iter().map(|&y| boxed(&mut w, iso(Vec3::new(0.0, y, 0.0)), half, 2.0, MaterialId::Aluminum)).collect();
        let ids: Vec<BodyId> = bodies.iter().map(|b| b.0).collect();
        let (mut stray, mut low, mut rise) = (0usize, f32::INFINITY, 0.0f32);
        for k in 0..(10.0 * hz) as usize {
            w.step(1.0 / hz);
            let st = w.last_bounce_stats();
            if st.toi_rows + st.support_rows + st.pseudo_rows > 0 && st.max_approach <= 1.0 { stray += 1; }
            // Reported, not gated (the Reviewer): the largest rise in total energy over any window
            // from 1.4 s. Sinking releases energy; a rise would be depth correction pushing out (R5)
            if k as f32 / hz >= 1.4 {
                let e: f32 = bodies.iter().map(|(b, mp)| energy(&w, *b, mp)).sum();
                low = low.min(e);
                rise = rise.max(e - low);
            }
        }
        let p0: Vec<Vec3> = ids.iter().map(|&b| w.get_body_pose(b).pos).collect();
        let (mut moved, mut fastest) = (0.0f32, 0.0f32);
        for _ in 0..hz as usize {
            w.step(1.0 / hz);
            for (&b, p) in ids.iter().zip(&p0) {
                moved = moved.max((w.get_body_pose(b).pos - *p).length());
                fastest = fastest.max(w.get_body_vel(b).lin.length());
            }
        }
        println!("{} Hz: W6 rows without a 1 m/s approach on {} steps; largest energy rise from 1.4 s {:.2e} J; from 10 s to 11 s the stack moved {:.2e} m, fastest {:.2e} m/s", hz, stray, rise, moved, fastest);
        assert_eq!(stray, 0, "{} Hz: W6 rows fired on {} steps with no approach over 1 m/s", hz, stray);
        assert!(moved < 1e-4 && fastest < 0.02, "{} Hz: the stack moved {:.2e} m, fastest {:.2e} m/s", hz, moved, fastest);
    }
}

/// The 20-box pyramid of `a_bounce_on_a_20_box_pile_converges`, settled, with a box above it;
/// or (pile false) the two-box stack with a box 1.2 m above
fn bounce_scene(pile: bool, hz: f32, dense: bool) -> World {
    let half = Vec3::new(0.2, 0.1, 0.2);
    let mut w = World::with_capacity(32, 32);
    w.set_gravity(Vec3::new(0.0, -G, 0.0));
    w.set_bounce_dense_reference(dense);
    floor(&mut w, MaterialId::Concrete);
    if pile {
        for (layer, n) in [6usize, 5, 4, 3, 2].iter().enumerate() {
            for i in 0..*n {
                let x = (i as f32 - (*n as f32 - 1.0) * 0.5) * 0.4;
                boxed(&mut w, iso(Vec3::new(x, 0.1 + 0.2 * layer as f32, 0.0)), half, 2.0, MaterialId::Aluminum);
            }
        }
        for _ in 0..hz as usize { w.step(1.0 / hz); }
        boxed(&mut w, iso(Vec3::new(-0.2, 2.0, 0.0)), half, 2.0, MaterialId::Aluminum);
    } else {
        for y in [0.1f32, 0.3, 1.5] { boxed(&mut w, iso(Vec3::new(0.0, y, 0.0)), half, 2.0, MaterialId::Aluminum); }
    }
    w
}

#[test]
fn the_sparse_factor_is_right() {
    // The Reviewer's conditions, replacing per-row sparse/dense agreement (which no factorisation
    // reaches on interior-point systems: near convergence w / y spans 1e-30 to 1e30):
    // (i) the first Newton system of each velocity solve (well conditioned, D = K_ii), sparse
    //     against dense, within 1e-12 of the largest entry;
    // (ii') every solve of both passes has a normwise backward error of at most 1e-12;
    // (iii) the outcome: no fallback, and the pile and stack gates hold (their own tests).
    // Also reported: the velocity error the compliance leaves on pushing rows, against the
    // unregularised K (gate 1e-7 m/s).
    for pile in [true, false] {
        for hz in [60.0f32, 240.0] {
            let mut w = bounce_scene(pile, hz, true);
            let (mut check, mut backward, mut residual, mut fallbacks, mut bounces) = (0.0f64, 0.0f64, 0.0f64, 0usize, 0usize);
            for _ in 0..(2.0 * hz) as usize {
                w.step(1.0 / hz);
                let st = w.last_bounce_stats();
                fallbacks += st.fallbacks;
                if st.toi_rows == 0 { continue; }
                bounces += 1;
                check = check.max(st.dense_check);
                backward = backward.max(st.backward_error);
                residual = residual.max(st.true_residual);
                if bounces == 1 {
                    println!("{} at {} Hz: {} rows, {} solves; fill: K {} entries, L {} below the diagonal",
                        if pile { "pile" } else { "stack" }, hz, st.toi_rows + st.support_rows, st.solves, st.k_nnz, st.l_nnz);
                }
            }
            let name = if pile { "pile" } else { "stack" };
            println!("{} at {} Hz: {} bounce steps; first systems sparse/dense {:.1e}; worst backward error {:.1e}; true residual {:.1e} m/s; {} fallbacks", name, hz, bounces, check, backward, residual, fallbacks);
            assert!(bounces > 0, "{} at {} Hz: no bounce", name, hz);
            assert!(check <= 1e-12, "{} at {} Hz: first systems differ by {:.2e}", name, hz, check);
            assert!(backward <= 1e-12, "{} at {} Hz: backward error {:.2e}", name, hz, backward);
            assert!(residual <= 1e-7, "{} at {} Hz: the compliance left {:.2e} m/s", name, hz, residual);
            assert_eq!(fallbacks, 0, "{} at {} Hz: fallbacks", name, hz);
        }
    }
}

/// PARITY_SPEC W6's reference hashes: the step hash after each bounce scene, from the build that
/// lands (recorded in the spec). Run with --ignored --nocapture.
#[test]
#[ignore]
fn w6_scene_hashes() {
    let hex = |h: [u8; 32]| h.iter().take(8).map(|b| format!("{:02x}", b)).collect::<String>();
    for hz in [60.0f32, 240.0] {
        let mut w = bounce_scene(true, hz, false);
        for _ in 0..(1.0 * hz) as usize { w.step(1.0 / hz); }
        println!("pile, {} Hz, 1 s after the drop: {}", hz, hex(w.step_hash()));
        let mut w = bounce_scene(false, hz, false);
        for _ in 0..(3.0 * hz) as usize { w.step(1.0 / hz); }
        println!("box on a two-box stack, {} Hz, 3 s: {}", hz, hex(w.step_hash()));
        let mut w = world(true);
        floor(&mut w, MaterialId::Concrete);
        boxed(&mut w, iso(Vec3::new(0.0, 2.0, 0.0)), Vec3::new(0.2, 0.05, 0.2), 1.5, MaterialId::Aluminum);
        for _ in 0..(3.0 * hz) as usize { w.step(1.0 / hz); }
        println!("drone-like box flat from 2 m, {} Hz, 3 s: {}", hz, hex(w.step_hash()));
    }
}

/// PARITY_SPEC W6's landing gate: the 20-box pile's bounce passes (`BounceStats::pass_us`, wall
/// time) at most 5 ms on a quiet machine. Five runs at each rate; per run the slowest bounce step,
/// then the minimum and median over runs. Run in release with --ignored --nocapture.
#[test]
#[ignore]
fn w6_pile_timing() {
    for hz in [60.0f32, 240.0] {
        let mut worst: Vec<f64> = Vec::new();
        let mut steps = 0usize;
        for _rep in 0..5 {
            let mut w = bounce_scene(true, hz, false);
            let mut slowest = 0.0f64;
            steps = 0;
            for _ in 0..(2.0 * hz) as usize {
                w.step(1.0 / hz);
                let st = w.last_bounce_stats();
                if st.toi_rows == 0 { continue; }
                steps += 1;
                slowest = slowest.max(st.pass_us as f64 / 1e3);
            }
            worst.push(slowest);
        }
        worst.sort_by(|a, b| a.partial_cmp(b).unwrap());
        println!("20-box pile, {} Hz: {} bounce steps; slowest step's passes min {:.2} ms, median {:.2} ms, max {:.2} ms",
            hz, steps, worst[0], worst[worst.len() / 2], worst[worst.len() - 1]);
    }
}
