// SPDX-License-Identifier: RiftForged Proprietary License
//! Diagnostics for PRECISION_SPEC: what the 1e-6 state grid (positions, velocities and
//! quaternion components rounded on every write) and f32 do to slow motion, springs and
//! multipliers. Run with
//!     cargo test -p riftphys-world --release --test precision_diag -- --ignored --nocapture
use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::World;

const G: f32 = 9.81;

fn iso(pos: Vec3) -> Isometry { Isometry { pos, rot: Quat::IDENTITY } }

fn free_box(w: &mut World, pos: Vec3, vel: Velocity) -> BodyId {
    let b = w.add_body(iso(pos), vel, MassProps::from_box_density(Vec3::splat(0.1), 1000.0), true);
    w.set_body_sleep_allowed(b, false);
    b
}

/// A free body (no gravity, no contacts) given a slow spin or drift: how far it actually turns
/// or moves in 10 s, against what it should.
#[test]
#[ignore]
fn slow_motion_dead_zone() {
    for hz in [60.0f32, 240.0, 960.0] {
        let mut line_w = String::new();
        let mut line_v = String::new();
        let mut line_n = String::new();
        for s in [1e-5f32, 1e-4, 3e-4, 1e-3, 3e-3, 1e-2] {
            let mut w = World::with_capacity(4, 4);
            w.set_gravity(Vec3::ZERO);
            let spin = free_box(&mut w, Vec3::new(0.0, 5.0, 0.0), Velocity { lin: Vec3::ZERO, ang: Vec3::new(0.0, 0.0, s) });
            let drift = free_box(&mut w, Vec3::new(3.0, 5.0, 0.0), Velocity { lin: Vec3::new(s, 0.0, 0.0), ang: Vec3::ZERO });
            let near = free_box(&mut w, Vec3::new(0.5, 0.5, 0.0), Velocity { lin: Vec3::new(s, 0.0, 0.0), ang: Vec3::ZERO });
            for _ in 0..(10.0 * hz) as usize { w.step(1.0 / hz); }
            let (axis, ang) = w.get_body_pose(spin).rot.to_axis_angle();
            let turned = ang * axis.z.signum();
            let moved = w.get_body_pose(drift).pos.x - 3.0;
            line_w += &format!("  {:.0e}: {:.3}", s, turned / (s * 10.0));
            line_v += &format!("  {:.0e}: {:.3}", s, moved / (s * 10.0));
            line_n += &format!("  {:.0e}: {:.3}", s, (w.get_body_pose(near).pos.x - 0.5) / (s * 10.0));
        }
        println!("{:4.0} Hz spin  (rad/s: turned / expected):{}", hz, line_w);
        println!("{:4.0} Hz drift at 5.8 m (m/s: moved / expected):{}", hz, line_v);
        println!("{:4.0} Hz drift at 0.7 m (m/s: moved / expected):{}", hz, line_n);
    }
}

/// A weight hanging on a compliant distance joint, started at its static stretch rest + c m g,
/// kept awake: the stretch it holds (ratio to static) after 10 s, and its worst departure.
fn hanging(m: f32, c: f32, hz: f32) -> (f32, f32) {
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::new(0.0, -G, 0.0));
    let p = w.add_body(iso(Vec3::new(0.0, 10.0, 0.0)), Velocity::default(), MassProps::infinite(), false);
    let stretch = c * m * G;
    let side = 0.1f32;
    let bob = w.add_body(iso(Vec3::new(0.0, 9.0 - stretch, 0.0)), Velocity::default(), MassProps::from_box_density(Vec3::splat(side / 2.0), m / (side * side * side)), true);
    w.set_body_sleep_allowed(bob, false);
    w.add_distance_joint(p, bob, 1.0, c);
    let mut worst = 0.0f32;
    for _ in 0..(10.0 * hz) as usize {
        w.step(1.0 / hz);
        let s = (10.0 - w.get_body_pose(bob).pos.y) - 1.0;
        worst = worst.max((s - stretch).abs() / stretch);
    }
    (((10.0 - w.get_body_pose(bob).pos.y) - 1.0) / stretch, worst)
}

/// Springs across masses, rates and stiffnesses: where the multiplier and compliance grids bite.
#[test]
#[ignore]
fn spring_resolution() {
    for (m, c) in [(2.0f32, 5e-4f32), (0.1, 5e-4), (1.0, 5e-4), (1.0, 1e-5), (1.0, 1e-6), (1.0, 1.4e-6), (1.0, 4e-7), (1.0, 1e-7)] {
        let mut line = String::new();
        for hz in [60.0f32, 240.0, 960.0] {
            let (end, worst) = hanging(m, c, hz);
            line += &format!("  {:4.0} Hz: end {:.4}, worst {:5.2}%", hz, end, worst * 100.0);
        }
        println!("m {:4.2} kg, c {:.1e} m/N (static stretch {:.2e} m):{}", m, c, c * m * G, line);
    }
}

/// A body resting still: does the grid make it creep (rounding bias) when nothing acts on it but
/// gravity balanced by a rigid joint? A 1 kg box hanging on a rigid 1 m distance joint, 60 s.
#[test]
#[ignore]
fn rest_creep() {
    for hz in [60.0f32, 240.0, 960.0] {
        let mut w = World::with_capacity(4, 4);
        w.set_gravity(Vec3::new(0.0, -G, 0.0));
        let p = w.add_body(iso(Vec3::new(0.0, 10.0, 0.0)), Velocity::default(), MassProps::infinite(), false);
        let bob = w.add_body(iso(Vec3::new(0.0, 9.0, 0.0)), Velocity::default(), MassProps::from_box_density(Vec3::splat(0.05), 1000.0), true);
        w.set_body_sleep_allowed(bob, false);
        w.add_distance_joint(p, bob, 1.0, 0.0);
        let mut worst = 0.0f32;
        for _ in 0..(60.0 * hz) as usize {
            w.step(1.0 / hz);
            worst = worst.max((w.get_body_pose(bob).pos - Vec3::new(0.0, 9.0, 0.0)).length());
        }
        println!("{:4.0} Hz: rigid hanging weight wandered at most {:.2e} m in 60 s", hz, worst);
    }
}

fn ground(w: &mut World, tilt: f32) {
    let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::from_rotation_z(tilt) }, Velocity::default(), MassProps::infinite(), false);
    w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(MaterialId::Concrete));
}

fn wood_box(w: &mut World, pos: Vec3, rot: Quat, half: Vec3, mass: f32) -> BodyId {
    let b = w.add_body(Isometry { pos, rot }, Velocity::default(), MassProps::from_box_density(half, mass / (8.0 * half.x * half.y * half.z)), true);
    w.add_collider(b, Shape::Box { hx: half.x, hy: half.y, hz: half.z }, material(MaterialId::Wood));
    w.set_body_sleep_allowed(b, false);
    b
}

/// A wood box on a 20 degree concrete slope (mu_s 0.66, above tan 20 = 0.36): how far it creeps
/// over 60 s after settling for 1 s.
pub fn slope_creep(hz: f32) -> f32 {
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::new(0.0, -G, 0.0));
    let tilt = 20f32.to_radians();
    ground(&mut w, tilt);
    let half = Vec3::new(0.1, 0.05, 0.1);
    let rot = Quat::from_rotation_z(tilt);
    let b = wood_box(&mut w, rot * Vec3::new(0.0, half.y, 0.0), rot, half, 1.0);
    for _ in 0..hz as usize { w.step(1.0 / hz); }
    let p0 = w.get_body_pose(b).pos;
    let mut worst = 0.0f32;
    for _ in 0..(60.0 * hz) as usize { w.step(1.0 / hz); worst = worst.max((w.get_body_pose(b).pos - p0).length()); }
    worst
}

/// An n-box stack: whether it stands, and how far the top box moves over 60 s after settling 1 s.
pub fn stack_drift(hz: f32, n: usize) -> (bool, f32) {
    let mut w = World::with_capacity(16, 16);
    w.set_gravity(Vec3::new(0.0, -G, 0.0));
    ground(&mut w, 0.0);
    let half = Vec3::new(0.2, 0.1, 0.2);
    let ids: Vec<BodyId> = (0..n).map(|i| wood_box(&mut w, Vec3::new(0.0, 0.1 + 0.21 * i as f32, 0.0), Quat::IDENTITY, half, 5.0)).collect();
    for _ in 0..hz as usize { w.step(1.0 / hz); }
    let top = *ids.last().unwrap();
    let p0 = w.get_body_pose(top).pos;
    let mut worst = 0.0f32;
    for _ in 0..(60.0 * hz) as usize { w.step(1.0 / hz); worst = worst.max((w.get_body_pose(top).pos - p0).length()); }
    let standing = ids.iter().enumerate().all(|(i, id)| {
        let p = w.get_body_pose(*id).pos;
        (p.y - (0.1 + 0.2 * i as f32)).abs() < 0.03 && p.x.abs() < 0.05 && p.z.abs() < 0.05
    });
    (standing, worst)
}

/// PRECISION_SPEC P1 at-rest cases (slope, stack; `rest_creep` above, the biped in
/// riftphys-uriel's `standing_still_drift`).
#[test]
#[ignore]
fn at_rest() {
    for hz in [60.0f32, 240.0] {
        let slope = slope_creep(hz);
        let mut line = format!("{:4.0} Hz: slope creep {:.3e} m", hz, slope);
        for n in [3usize, 5, 10] {
            let (standing, top) = stack_drift(hz, n);
            line += &format!(" | {}-box stack standing {}, top moved {:.3e} m", n, standing, top);
        }
        println!("{}", line);
    }
}



