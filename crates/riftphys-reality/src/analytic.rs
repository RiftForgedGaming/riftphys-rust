//! Closed-form physics benchmarks. Each case builds a small world, runs it, and compares one
//! measured quantity with its textbook value. They state what "correct" means before any mocap
//! data exists, and they say which physics the engine is missing.

use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, pair_props, MaterialId};
use riftphys_world::world::World;

pub const G: f32 = 9.81;

#[derive(Clone, Copy, Debug)]
pub enum Tol {
    /// |sim - truth| / |truth| <= x
    Rel(f64),
    /// |sim - truth| <= x
    Abs(f64),
    /// sim >= x (qualitative behaviour, e.g. "flips at least once")
    AtLeast(f64),
}

#[derive(Clone, Debug)]
pub struct Case {
    pub name: &'static str,
    /// What was measured, and anything needed to read it
    pub quantity: String,
    pub unit: &'static str,
    pub sim: f64,
    pub truth: f64,
    pub tol: Tol,
}

impl Case {
    pub fn error(&self) -> f64 {
        match self.tol {
            Tol::Rel(_) => ((self.sim - self.truth) / self.truth.abs().max(1e-12)).abs(),
            Tol::Abs(_) => (self.sim - self.truth).abs(),
            Tol::AtLeast(_) => 0.0,
        }
    }

    pub fn pass(&self) -> bool {
        match self.tol {
            Tol::Rel(x) | Tol::Abs(x) => self.error() <= x,
            Tol::AtLeast(x) => self.sim >= x,
        }
    }

    pub fn tol_text(&self) -> String {
        match self.tol {
            Tol::Rel(x) => format!("{:.1}%", 100.0 * x),
            Tol::Abs(x) => format!("{} {}", x, self.unit),
            Tol::AtLeast(x) => format!(">= {}", x),
        }
    }

    pub fn error_text(&self) -> String {
        match self.tol {
            Tol::Rel(_) => format!("{:.2}%", 100.0 * self.error()),
            Tol::Abs(_) => format!("{:.4} {}", self.error(), self.unit),
            Tol::AtLeast(_) => "-".into(),
        }
    }
}

// ---------------------------------------------------------------- world helpers

fn new_world(gravity: bool) -> World {
    let mut w = World::with_capacity(16, 16);
    w.set_gravity(if gravity { Vec3::new(0.0, -G, 0.0) } else { Vec3::ZERO });
    w
}

/// Static ground whose top surface passes through the origin, tilted by `tilt` rad about +z
/// (so the downhill direction is -x for positive tilt).
fn ground(w: &mut World, mat: MaterialId, tilt: f32) -> BodyId {
    let rot = Quat::from_rotation_z(tilt);
    let g = w.add_body(Isometry { pos: rot * Vec3::new(0.0, -0.5, 0.0), rot }, Velocity::default(), MassProps::infinite(), false);
    w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(mat));
    g
}

fn anchor(w: &mut World, pos: Vec3) -> BodyId {
    w.add_body(Isometry { pos, rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false)
}

fn sphere(w: &mut World, pos: Vec3, vel: Vec3, r: f32, mass: f32, mat: Option<MaterialId>) -> BodyId {
    let mp = MassProps::from_sphere_density(r, mass / (4.0 / 3.0 * std::f32::consts::PI * r * r * r));
    let id = w.add_body(Isometry { pos, rot: Quat::IDENTITY }, Velocity { lin: vel, ang: Vec3::ZERO }, mp, true);
    if let Some(m) = mat { w.add_collider(id, Shape::Sphere { r }, material(m)); }
    w.set_body_sleep_allowed(id, false);
    id
}

fn cuboid(w: &mut World, pose: Isometry, vel: Velocity, half: Vec3, mass: f32, mat: Option<MaterialId>) -> BodyId {
    let mp = MassProps::from_box_density(half, mass / (8.0 * half.x * half.y * half.z));
    let id = w.add_body(pose, vel, mp, true);
    if let Some(m) = mat { w.add_collider(id, Shape::Box { hx: half.x, hy: half.y, hz: half.z }, material(m)); }
    w.set_body_sleep_allowed(id, false);
    id
}

/// Box inertia (local, diagonal) about its centre.
fn box_inertia(m: f32, h: Vec3) -> Vec3 {
    Vec3::new(m / 3.0 * (h.y * h.y + h.z * h.z), m / 3.0 * (h.x * h.x + h.z * h.z), m / 3.0 * (h.x * h.x + h.y * h.y))
}

fn steps(seconds: f32, dt: f32) -> usize { (seconds / dt).round() as usize }

/// Mean period from upward zero crossings (linearly interpolated) of a sampled signal.
fn period(samples: &[f32], dt: f32) -> f64 {
    let mut crossings = Vec::new();
    for i in 1..samples.len() {
        let (a, b) = (samples[i - 1], samples[i]);
        if a < 0.0 && b >= 0.0 {
            crossings.push((i - 1) as f64 + (-a / (b - a)) as f64);
        }
    }
    if crossings.len() < 2 { return f64::NAN; }
    (crossings[crossings.len() - 1] - crossings[0]) / (crossings.len() - 1) as f64 * dt as f64
}

// ---------------------------------------------------------------- cases

fn free_fall(dt: f32) -> Case {
    let mut w = new_world(true);
    let b = sphere(&mut w, Vec3::new(0.0, 100.0, 0.0), Vec3::ZERO, 0.05, 1.0, None);
    for _ in 0..steps(1.0, dt) { w.step(dt); }
    Case { name: "free fall", quantity: "drop after 1 s from rest".into(), unit: "m",
           sim: (100.0 - w.get_body_pose(b).pos.y) as f64, truth: 0.5 * G as f64, tol: Tol::Rel(0.01) }
}

fn simple_pendulum(dt: f32) -> [Case; 2] {
    let (len, th0) = (1.0f32, 0.1f32);
    let mut w = new_world(true);
    let a = anchor(&mut w, Vec3::new(0.0, 3.0, 0.0));
    let bob = sphere(&mut w, Vec3::new(len * th0.sin(), 3.0 - len * th0.cos(), 0.0), Vec3::ZERO, 0.02, 1.0, None);
    w.add_distance_joint(a, bob, len, 0.0);
    let n = steps(30.0, dt);
    let xs: Vec<f32> = (0..n).map(|_| { w.step(dt); w.get_body_pose(bob).pos.x }).collect();
    let t0 = 2.0 * std::f64::consts::PI * (len as f64 / G as f64).sqrt() * (1.0 + (th0 as f64).powi(2) / 16.0);
    let last = steps(2.5, dt).min(n);
    let amp = xs[n - last..].iter().fold(0.0f32, |m, x| m.max(x.abs()));
    [
        Case { name: "simple pendulum", quantity: "period, 1 m, 0.1 rad (distance joint)".into(), unit: "s",
               sim: period(&xs, dt), truth: t0, tol: Tol::Rel(0.01) },
        Case { name: "simple pendulum", quantity: "amplitude after 30 s / initial (energy kept)".into(), unit: "",
               sim: (amp / (len * th0.sin())) as f64, truth: 1.0, tol: Tol::Abs(0.02) },
    ]
}

fn physical_pendulum(dt: f32) -> [Case; 2] {
    let (h, m, th0) = (Vec3::new(0.02, 0.5, 0.02), 1.0f32, 0.1f32);
    let mut w = new_world(true);
    let pivot = Vec3::new(0.0, 3.0, 0.0);
    let a = anchor(&mut w, pivot);
    let rot = Quat::from_rotation_x(th0);
    let rod = cuboid(&mut w, Isometry { pos: pivot + rot * Vec3::new(0.0, -h.y, 0.0), rot }, Velocity::default(), h, m, None);
    w.add_hinge_joint(a, rod, Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY }, Isometry { pos: Vec3::new(0.0, h.y, 0.0), rot: Quat::IDENTITY }, 0);
    let n = steps(30.0, dt);
    let angle = |w: &World| { let up = w.get_body_pose(rod).rot * Vec3::Y; up.z.atan2(up.y) };
    let th: Vec<f32> = (0..n).map(|_| { w.step(dt); angle(&w) }).collect();
    let i_pivot = box_inertia(m, h).x as f64 + (m * h.y * h.y) as f64;
    let t0 = 2.0 * std::f64::consts::PI * (i_pivot / (m as f64 * G as f64 * h.y as f64)).sqrt() * (1.0 + (th0 as f64).powi(2) / 16.0);
    let last = steps(2.0, dt).min(n);
    let amp = th[n - last..].iter().fold(0.0f32, |mx, x| mx.max(x.abs()));
    [
        Case { name: "physical pendulum", quantity: "period, 1 m rod on a hinge (uses real inertia)".into(), unit: "s",
               sim: period(&th, dt), truth: t0, tol: Tol::Rel(0.01) },
        Case { name: "physical pendulum", quantity: "amplitude after 30 s / initial".into(), unit: "",
               sim: (amp / th0) as f64, truth: 1.0, tol: Tol::Abs(0.02) },
    ]
}

/// Large swing: a 1 m rod released horizontal (90 degrees) on a hinge; total mechanical energy
/// after 10 s relative to the start.
fn large_swing(dt: f32) -> Case {
    let (h, m) = (Vec3::new(0.02, 0.5, 0.02), 1.0f32);
    let inertia = box_inertia(m, h);
    let mut w = new_world(true);
    let pivot = Vec3::new(0.0, 3.0, 0.0);
    let a = anchor(&mut w, pivot);
    let rot = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
    let rod = cuboid(&mut w, Isometry { pos: pivot + rot * Vec3::new(0.0, -h.y, 0.0), rot }, Velocity::default(), h, m, None);
    w.add_hinge_joint(a, rod, Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY }, Isometry { pos: Vec3::new(0.0, h.y, 0.0), rot: Quat::IDENTITY }, 0);
    let energy = |w: &World| {
        let p = w.get_body_pose(rod); let v = w.get_body_vel(rod);
        let wb = p.rot.inverse() * v.ang;
        (0.5 * m * v.lin.length_squared() + 0.5 * (wb * inertia * wb).dot(Vec3::ONE) + m * G * (p.pos.y - (pivot.y - h.y))) as f64
    };
    let e0 = energy(&w);
    for _ in 0..steps(10.0, dt) { w.step(dt); }
    Case { name: "large swing", quantity: "energy after 10 s / initial, rod released at 90 degrees".into(), unit: "",
           sim: energy(&w) / e0, truth: 1.0, tol: Tol::Abs(0.02) }
}

/// Block sliding to rest on a level floor: d = v0^2 / (2 mu_k g).
fn sliding(dt: f32) -> Case {
    let (mat, v0) = (MaterialId::Wood, 3.0f32);
    let pair = pair_props(mat, mat);
    let mut w = new_world(true);
    ground(&mut w, mat, 0.0);
    let h = Vec3::new(0.1, 0.05, 0.1);
    let b = cuboid(&mut w, Isometry { pos: Vec3::new(0.0, h.y, 0.0), rot: Quat::IDENTITY }, Velocity { lin: Vec3::new(v0, 0.0, 0.0), ang: Vec3::ZERO }, h, 1.0, Some(mat));
    for _ in 0..steps(3.0, dt) { w.step(dt); }
    let truth = (v0 * v0 / (2.0 * pair.mu_k * G)) as f64;
    Case { name: "sliding friction", quantity: format!("stopping distance from 3 m/s, wood on wood (mu_k {:.2}; with mu_s {:.2} it would be {:.3} m)",
                                                         pair.mu_k, pair.mu_s, v0 * v0 / (2.0 * pair.mu_s * G)),
           unit: "m", sim: w.get_body_pose(b).pos.x as f64, truth, tol: Tol::Rel(0.05) }
}

fn on_incline(dt: f32, tilt: f32, seconds: f32) -> f64 {
    let mat = MaterialId::Wood;
    let mut w = new_world(true);
    ground(&mut w, mat, tilt);
    let h = Vec3::new(0.1, 0.05, 0.1);
    let rot = Quat::from_rotation_z(tilt);
    let start = rot * Vec3::new(0.0, h.y, 0.0);
    let b = cuboid(&mut w, Isometry { pos: start, rot }, Velocity::default(), h, 1.0, Some(mat));
    for _ in 0..steps(seconds, dt) { w.step(dt); }
    // Distance travelled downhill (-x along the slope)
    (-(w.get_body_pose(b).pos - start).dot(rot * Vec3::X)) as f64
}

fn incline(dt: f32) -> [Case; 2] {
    let pair = pair_props(MaterialId::Wood, MaterialId::Wood);
    let hold = 0.8 * pair.mu_s.atan();
    let slide = pair.mu_s.atan() + 0.1;
    let a = G * (slide.sin() - pair.mu_k * slide.cos());
    [
        Case { name: "static friction", quantity: format!("creep in 2 s on a {:.2} rad slope (below atan mu_s)", hold), unit: "m",
               sim: on_incline(dt, hold, 2.0), truth: 0.0, tol: Tol::Abs(0.005) },
        Case { name: "incline slide", quantity: format!("distance in 1 s from rest on {:.2} rad: g(sin - mu_k cos) t^2/2", slide), unit: "m",
               sim: on_incline(dt, slide, 1.0), truth: (0.5 * a) as f64, tol: Tol::Rel(0.05) },
    ]
}

/// Ball dropped from 1 m: first rebound apex = e^2 x drop height.
fn bounce(dt: f32, ball: MaterialId, floor: MaterialId) -> Case {
    let (r, h0) = (0.1f32, 1.0f32);
    let e = pair_props(ball, floor).restitution;
    let mut w = new_world(true);
    ground(&mut w, floor, 0.0);
    let b = sphere(&mut w, Vec3::new(0.0, r + h0, 0.0), Vec3::ZERO, r, 0.1, Some(ball));
    let (mut bounced, mut apex) = (false, 0.0f32);
    for _ in 0..steps(2.0, dt) {
        w.step(dt);
        let (y, vy) = (w.get_body_pose(b).pos.y - r, w.get_body_vel(b).lin.y);
        if !bounced && vy > 0.0 { bounced = true; }
        if bounced {
            apex = apex.max(y);
            if vy < 0.0 { break; }
        }
    }
    let truth = (e * e * h0) as f64;
    Case { name: "restitution", quantity: format!("rebound apex from 1 m, {:?} ball on {:?} (pair e {:.2})", ball, floor, e), unit: "m",
           sim: apex as f64, truth, tol: if truth > 0.0 { Tol::Rel(0.05) } else { Tol::Abs(0.005) } }
}

/// Solid ball rolling down a slope without slipping: a = 5/7 g sin(theta).
fn rolling(dt: f32) -> [Case; 2] {
    let (tilt, r) = (0.3f32, 0.1f32);
    let mut w = new_world(true);
    ground(&mut w, MaterialId::Rubber, tilt);
    let rot = Quat::from_rotation_z(tilt);
    let start = rot * Vec3::new(0.0, r, 0.0);
    let b = sphere(&mut w, start, Vec3::ZERO, r, 1.0, Some(MaterialId::Rubber));
    for _ in 0..steps(1.0, dt) { w.step(dt); }
    let downhill = -(rot * Vec3::X);
    let d = (w.get_body_pose(b).pos - start).dot(downhill);
    let v = w.get_body_vel(b).lin.dot(downhill);
    let spin = w.get_body_vel(b).ang.length();
    let a = 5.0 / 7.0 * G * tilt.sin();
    [
        Case { name: "rolling", quantity: format!("distance in 1 s down {:.1} rad (sliding would give {:.3} m)", tilt, 0.5 * G * tilt.sin()), unit: "m",
               sim: d as f64, truth: (0.5 * a) as f64, tol: Tol::Rel(0.05) },
        Case { name: "rolling", quantity: "spin x radius / speed (1 = no slip)".into(), unit: "",
               sim: (spin * r / v.max(1e-6)) as f64, truth: 1.0, tol: Tol::Abs(0.05) },
    ]
}

/// Torque-free box spun about its intermediate axis: angular momentum is conserved in the world
/// frame, and the spin axis flips over and over (the Dzhanibekov effect).
fn tumbling(dt: f32) -> [Case; 3] {
    let (h, m) = (Vec3::new(0.3, 0.2, 0.1), 1.0f32);
    let inertia = box_inertia(m, h);
    let mut w = new_world(false);
    let w0 = Vec3::new(0.02, 4.0, 0.02);
    let b = cuboid(&mut w, Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY }, Velocity { lin: Vec3::ZERO, ang: w0 }, h, m, None);
    let momentum = |w: &World| {
        let p = w.get_body_pose(b);
        let wb = p.rot.inverse() * w.get_body_vel(b).ang;
        (p.rot * (inertia * wb), 0.5 * (wb * inertia * wb).dot(Vec3::ONE), wb)
    };
    let (l0, e0, _) = momentum(&w);
    let (mut dl, mut de, mut flips, mut last_sign) = (0.0f32, 0.0f32, 0u32, 1.0f32);
    for _ in 0..steps(10.0, dt) {
        w.step(dt);
        let (l, e, wb) = momentum(&w);
        dl = dl.max((l - l0).length() / l0.length());
        de = de.max((e - e0).abs() / e0);
        if wb.y.signum() != last_sign && wb.y.abs() > 0.5 { flips += 1; last_sign = wb.y.signum(); }
    }
    [
        Case { name: "torque-free rotation", quantity: "max change of world angular momentum over 10 s / |L|".into(), unit: "",
               sim: dl as f64, truth: 0.0, tol: Tol::Abs(0.01) },
        Case { name: "torque-free rotation", quantity: "max change of rotational energy over 10 s / E".into(), unit: "",
               sim: de as f64, truth: 0.0, tol: Tol::Abs(0.01) },
        Case { name: "torque-free rotation", quantity: "intermediate-axis flips in 10 s (Dzhanibekov)".into(), unit: "",
               sim: flips as f64, truth: 1.0, tol: Tol::AtLeast(1.0) },
    ]
}

/// Head-on collision of 1 kg and 2 kg steel balls at +-1 m/s, no gravity.
fn collision(dt: f32) -> [Case; 3] {
    let (m1, m2, u1, u2) = (1.0f32, 2.0f32, 1.0f32, -1.0f32);
    let e = pair_props(MaterialId::Steel, MaterialId::Steel).restitution;
    let mut w = new_world(false);
    let a = sphere(&mut w, Vec3::new(-0.5, 0.0, 0.0), Vec3::new(u1, 0.0, 0.0), 0.1, m1, Some(MaterialId::Steel));
    let b = sphere(&mut w, Vec3::new(0.5, 0.0, 0.0), Vec3::new(u2, 0.0, 0.0), 0.1, m2, Some(MaterialId::Steel));
    for _ in 0..steps(1.0, dt) { w.step(dt); }
    let (v1, v2) = (w.get_body_vel(a).lin.x, w.get_body_vel(b).lin.x);
    let p = m1 * u1 + m2 * u2;
    let v1t = (p - m2 * e * (u1 - u2)) / (m1 + m2);
    let v2t = (p + m1 * e * (u1 - u2)) / (m1 + m2);
    [
        Case { name: "collision", quantity: "total momentum after".into(), unit: "kg m/s",
               sim: (m1 * v1 + m2 * v2) as f64, truth: p as f64, tol: Tol::Abs(0.01) },
        Case { name: "collision", quantity: format!("1 kg ball velocity after (e {:.2})", e), unit: "m/s",
               sim: v1 as f64, truth: v1t as f64, tol: Tol::Abs(0.05) },
        Case { name: "collision", quantity: format!("2 kg ball velocity after (e {:.2})", e), unit: "m/s",
               sim: v2 as f64, truth: v2t as f64, tol: Tol::Abs(0.05) },
    ]
}

fn resting(dt: f32) -> Case {
    let mut w = new_world(true);
    ground(&mut w, MaterialId::Wood, 0.0);
    let h = Vec3::new(0.2, 0.1, 0.15);
    let start = Vec3::new(0.0, h.y, 0.0);
    let b = cuboid(&mut w, Isometry { pos: start, rot: Quat::IDENTITY }, Velocity::default(), h, 5.0, Some(MaterialId::Wood));
    for _ in 0..steps(10.0, dt) { w.step(dt); }
    Case { name: "resting contact", quantity: "drift of a box at rest over 10 s".into(), unit: "m",
           sim: (w.get_body_pose(b).pos - start).length() as f64, truth: 0.0, tol: Tol::Abs(0.002) }
}

/// Every case at time step `dt`.
pub fn run_all(dt: f32) -> Vec<Case> {
    let mut out = vec![free_fall(dt)];
    out.extend(simple_pendulum(dt));
    out.extend(physical_pendulum(dt));
    out.push(large_swing(dt));
    out.push(sliding(dt));
    out.extend(incline(dt));
    out.push(bounce(dt, MaterialId::Rubber, MaterialId::Rubber));
    out.push(bounce(dt, MaterialId::Rubber, MaterialId::Concrete));
    out.extend(rolling(dt));
    out.extend(tumbling(dt));
    out.extend(collision(dt));
    out.push(resting(dt));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Open gaps, each still measured by the report. Everything else must stay within tolerance.
    /// - large swing: velocity projection in maximal coordinates removes ~(w dt)^2 of a fast
    ///   joint's kinetic energy per step (docs/PHYSICS_FIDELITY.md).
    const KNOWN_GAPS_240HZ: [&str; 1] = ["large swing"];

    #[test]
    fn benchmarks_hold_at_240_hz() {
        let failing: Vec<String> = run_all(1.0 / 240.0).into_iter()
            .filter(|c| !c.pass() && !KNOWN_GAPS_240HZ.contains(&c.name))
            .map(|c| format!("{}: {} sim {:.4} truth {:.4}", c.name, c.quantity, c.sim, c.truth)).collect();
        assert!(failing.is_empty(), "benchmarks outside tolerance at 240 Hz:
{}", failing.join("
"));
    }
}
