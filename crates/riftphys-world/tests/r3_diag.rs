// SPDX-License-Identifier: RiftForged Proprietary License
//! Diagnostics for AT_REST_SPEC R3 (box-box face clipping, terrain reduction). Run with
//!     cargo test -p riftphys-world --release --test r3_diag -- --ignored --nocapture --test-threads 1
use glam::{UVec2, Vec2};
use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_terrain::terrain::HeightField;
use riftphys_world::world::World;

const G: f32 = 9.81;

fn boxed(w: &mut World, pos: Vec3, rot: Quat, half: Vec3, mass: f32) -> BodyId {
    let b = w.add_body(Isometry { pos, rot }, Velocity::default(), MassProps::from_box_density(half, mass / (8.0 * half.x * half.y * half.z)), true);
    w.add_collider(b, Shape::Box { hx: half.x, hy: half.y, hz: half.z }, material(MaterialId::Wood));
    w.set_body_sleep_allowed(b, false);
    b
}

fn tilt_deg(q: Quat) -> f32 { (q * Vec3::Y).dot(Vec3::Y).clamp(-1.0, 1.0).acos().to_degrees() }

/// Steps `secs`; returns (first time any box is 2 cm off its start column or tilted 5 degrees,
/// worst movement after a 1 s settle, worst tilt at the end).
fn run(w: &mut World, ids: &[BodyId], hz: f32, secs: f32) -> (Option<f32>, f32, f32) {
    let start: Vec<Vec3> = ids.iter().map(|&b| w.get_body_pose(b).pos).collect();
    let mut first = None;
    let mut p1: Vec<Vec3> = Vec::new();
    let mut worst = 0.0f32;
    for k in 1..=(secs * hz) as usize {
        w.step(1.0 / hz);
        if k == hz as usize { p1 = ids.iter().map(|&b| w.get_body_pose(b).pos).collect(); }
        for (i, &b) in ids.iter().enumerate() {
            let p = w.get_body_pose(b);
            let d = p.pos - start[i];
            if first.is_none() && ((d.x * d.x + d.z * d.z).sqrt() > 0.02 || tilt_deg(p.rot) > 5.0) { first = Some(k as f32 / hz); }
            if !p1.is_empty() { worst = worst.max((p.pos - p1[i]).length()); }
        }
    }
    let tilt = ids.iter().map(|&b| tilt_deg(w.get_body_pose(b).rot)).fold(0.0, f32::max);
    (first, worst, tilt)
}

fn ground(w: &mut World) {
    let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(MaterialId::Concrete));
}

#[test]
#[ignore]
fn yawed_stacks() {
    for deg in [0.0f32, 3.0, 45.0] {
        for hz in [60.0f32, 240.0] {
            let mut w = World::with_capacity(16, 16);
            w.set_gravity(Vec3::new(0.0, -G, 0.0));
            ground(&mut w);
            let half = Vec3::new(0.2, 0.1, 0.2);
            let ids: Vec<BodyId> = (0..3).map(|i| {
                let rot = if i == 1 { Quat::from_rotation_y(deg.to_radians()) } else { Quat::IDENTITY };
                boxed(&mut w, Vec3::new(0.0, 0.1 + 0.21 * i as f32, 0.0), rot, half, 5.0)
            }).collect();
            let (first, worst, tilt) = run(&mut w, &ids, hz, 60.0);
            println!("yaw {:>4.1} deg {:>3} Hz: off column at {:?}, worst move after 1 s {:.3e} m, tilt at end {:.2} deg", deg, hz, first, worst, tilt);
        }
    }
}

#[test]
#[ignore]
fn ledge() {
    // Static slab: top at y = 0, its +x edge at x = 0. A 0.4 m box with its centre of mass `off`
    // past the edge (negative = inside).
    for off in [-0.05f32, -0.01, 0.01, 0.05] {
        for hz in [60.0f32, 240.0] {
            let mut w = World::with_capacity(16, 16);
            w.set_gravity(Vec3::new(0.0, -G, 0.0));
            let g = w.add_body(Isometry { pos: Vec3::new(-5.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
            w.add_collider(g, Shape::Box { hx: 5.0, hy: 0.5, hz: 5.0 }, material(MaterialId::Concrete));
            let b = boxed(&mut w, Vec3::new(off, 0.1, 0.0), Quat::IDENTITY, Vec3::new(0.2, 0.1, 0.2), 5.0);
            for _ in 0..(5.0 * hz) as usize { w.step(1.0 / hz); }
            let p = w.get_body_pose(b);
            println!("ledge com {:+.2} m past edge, {:>3} Hz: after 5 s y {:+.3}, tilt {:.1} deg", off, hz, p.pos.y, tilt_deg(p.rot));
        }
    }
}

#[test]
#[ignore]
fn heightfield_box() {
    // Flat heightfield, 0.25 m cells: a 0.4 x 0.4 m box base covers several triangles.
    for (deg, cx) in [(0.0f32, 2.0f32), (0.0, 2.1), (30.0, 2.05)] {
        for hz in [60.0f32, 240.0] {
            let mut w = World::with_capacity(16, 16);
            w.set_gravity(Vec3::new(0.0, -G, 0.0));
            w.add_environment(Box::new(HeightField::from_heights(UVec2::new(20, 20), Vec2::new(0.25, 0.25), Vec2::ZERO, vec![0.0; 400], MaterialId::Concrete)));
            let b = boxed(&mut w, Vec3::new(cx, 0.1, cx), Quat::from_rotation_y(deg.to_radians()), Vec3::new(0.2, 0.1, 0.2), 5.0);
            let (first, worst, tilt) = run(&mut w, &[b], hz, 60.0);
            let c = w.last_frame_contacts.len();
            println!("heightfield yaw {:>4.1} at x {:.2}, {:>3} Hz: off at {:?}, worst move {:.3e} m, tilt {:.2} deg, y {:+.4}, {} contacts",
                deg, cx, hz, first, worst, tilt, w.get_body_pose(b).pos.y, c);
        }
    }
}

#[test]
#[ignore]
fn ledge_debug() {
    for gx in [-5.0f32, 0.0] {
        let mut w = World::with_capacity(16, 16);
        w.set_gravity(Vec3::new(0.0, -G, 0.0));
        let g = w.add_body(Isometry { pos: Vec3::new(gx, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
        w.add_collider(g, Shape::Box { hx: 5.0, hy: 0.5, hz: 5.0 }, material(MaterialId::Concrete));
        let b = boxed(&mut w, Vec3::new(-0.05, 0.1, 0.0), Quat::IDENTITY, Vec3::new(0.2, 0.1, 0.2), 5.0);
        for k in 0..5 {
            let s = w.step(1.0 / 60.0);
            println!("ground x {} step {}: pairs {} contacts {} y {:+.4}", gx, k, s.pairs_tested, s.contacts, w.get_body_pose(b).pos.y);
        }
    }
}
