//! Real cube tosses: the ContactNets dataset (570 tosses of a 10.48 cm acrylic cube onto a table,
//! AprilTags at 148 Hz; `tools/contactnets_to_reality.py`) replayed in RiftPhys and scored with
//! the trajectory error Acosta, Yang & Posa used to rank Drake, MuJoCo and Bullet on the same
//! tosses ("Validating Robotics Simulators on Real-World Impacts", RA-L 2022, Eq. 3):
//!
//!   e_q = (1/T) sum_t ( (2/l) |p*_t - p_t|^2 + angle(R*_t, R_t)^2 )
//!
//! exactly as printed (the 2/l multiplies the squared distance in metres; angles in radians),
//! with l the cube's side, every toss started from its measured first state (pose, velocity and
//! angular velocity from the dataset), simulated at 1480 Hz and compared at the 148 Hz frames.
//! Their Table V (fitted contact parameters, all tosses): position error as % of the cube's
//! width, rotation error in degrees and e_q, each mean +- std over tosses.

use crate::cmaes;
use crate::log::RealityLog;
use riftphys_core::{Isometry, Mat3, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::World;

/// Cube: half-width (m), mass (kg) and inertia about each axis (kg m^2), from the dataset and
/// Acosta et al. Table I (0.295 kg half-widths^2, printed there as ".0081" for 0.00081).
pub const HALF: f32 = 0.0524;
pub const MASS: f32 = 0.37;
pub const INERTIA: f32 = 8.1e-4;
pub const SIM_HZ: f32 = 1480.0;
const CUBE: MaterialId = MaterialId::AcrylicPmma;
const TABLE: MaterialId = MaterialId::Wood;

/// The published Table V rows: (simulator, position % width mean, std, rotation deg mean, std,
/// e_q mean, std).
pub const PUBLISHED: [(&str, f32, f32, f32, f32, f32, f32); 3] = [
    ("Drake", 13.5, 8.2, 16.5, 20.0, 0.27, 0.57),
    ("Bullet", 14.9, 8.9, 16.5, 20.2, 0.27, 0.57),
    ("MuJoCo", 25.1, 10.8, 21.7, 21.4, 0.38, 0.63),
];

/// One measured toss: frame poses at 148 Hz from release, and the measured start velocities.
#[derive(Clone, Debug)]
pub struct Toss {
    pub index: u32,
    pub dt: f64,
    pub pos: Vec<Vec3>,
    pub rot: Vec<Quat>,
    pub vel: Vec3,
    pub ang: Vec3,
}

/// Splits a converted log into tosses: each `state` event starts one, running to the next break.
pub fn tosses(log: &RealityLog) -> Result<Vec<Toss>, String> {
    let track = log.tracks.get("cube").ok_or("log has no `cube` body (convert with tools/contactnets_to_reality.py)")?;
    let dt = 1.0 / log.rate_hz as f64;
    let mut out = Vec::new();
    for (t0, e, data) in &log.events {
        if e != "state" { continue; }
        let end = log.breaks.iter().copied().find(|b| b > t0).unwrap_or(f64::INFINITY);
        let frames: Vec<_> = track.iter().filter(|s| s.time >= t0 - 1e-6 && s.time < end).collect();
        if frames.len() < 2 { continue; }
        let v3 = |k: &str| -> Result<Vec3, String> {
            let a = data.get(k).and_then(|v| v.as_array()).ok_or(format!("state at {}: no {}", t0, k))?;
            Ok(Vec3::new(a[0].as_f64().unwrap_or(0.0) as f32, a[1].as_f64().unwrap_or(0.0) as f32, a[2].as_f64().unwrap_or(0.0) as f32))
        };
        out.push(Toss {
            index: data.get("toss").and_then(|v| v.as_u64()).unwrap_or(out.len() as u64) as u32,
            dt,
            pos: frames.iter().map(|s| s.pos).collect(),
            rot: frames.iter().map(|s| s.rot).collect(),
            vel: v3("vel")?,
            ang: v3("ang")?,
        });
    }
    Ok(out)
}

/// Table height: where the cubes come to rest (median final centre height, less a half-width).
pub fn table_height(ts: &[Toss]) -> f32 {
    let mut y: Vec<f32> = ts.iter().map(|t| t.pos.last().unwrap().y - HALF).collect();
    y.sort_by(|a, b| a.partial_cmp(b).unwrap());
    y[y.len() / 2]
}

/// Contact parameters of the cube on the table: restitution, static and kinetic friction.
#[derive(Clone, Copy, Debug)]
pub struct Params { pub restitution: f32, pub mu_s: f32, pub mu_k: f32 }

impl Params {
    /// The cube's measured values (Acosta et al. Table I): restitution 0.125, friction 0.18.
    pub fn measured() -> Self { Self { restitution: 0.125, mu_s: 0.18, mu_k: 0.18 } }
    pub const NAMES: [&'static str; 3] = ["restitution", "mu_s", "mu_k"];
    pub const BOUNDS: [(f32, f32); 3] = [(0.0, 0.9), (0.02, 1.0), (0.02, 1.0)];
    fn from_unit(x: &[f64]) -> Self {
        let m = |i: usize| { let (lo, hi) = Self::BOUNDS[i]; lo + (hi - lo) * x[i].clamp(0.0, 1.0) as f32 };
        Self { restitution: m(0), mu_s: m(1), mu_k: m(2) }
    }
    fn to_unit(&self) -> Vec<f64> {
        [self.restitution, self.mu_s, self.mu_k].iter().zip(Self::BOUNDS).map(|(v, (lo, hi))| ((v - lo) / (hi - lo)) as f64).collect()
    }
}

/// Errors of one replayed toss, averaged over its frames after the first.
#[derive(Clone, Copy, Debug, Default)]
pub struct TossError {
    /// Mean position error as % of the cube's width
    pub pos_pct: f32,
    /// Mean rotation error, degrees
    pub rot_deg: f32,
    /// Acosta et al. Eq. 3
    pub e_q: f32,
}

/// Replays one toss from its measured first state at `SIM_HZ` and scores it at the logged frames.
pub fn replay(t: &Toss, p: &Params, table_y: f32, hz: f32) -> TossError {
    let mut world = World::with_capacity(4, 4);
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));
    let ground = world.add_body(Isometry { pos: Vec3::new(0.0, table_y - 0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    world.add_collider(ground, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(TABLE));
    let mut props = world.pair_props(CUBE, TABLE);
    props.restitution = p.restitution;
    props.mu_k = p.mu_k;
    props.mu_s = p.mu_s.max(p.mu_k);
    world.set_pair_props(CUBE, TABLE, props);
    let mut mp = MassProps::from_box_density(Vec3::splat(HALF), MASS / (8.0 * HALF * HALF * HALF));
    mp.inertia = Mat3::from_diagonal(Vec3::splat(INERTIA).into());
    let cube = world.add_body(Isometry { pos: t.pos[0], rot: t.rot[0] }, Velocity { lin: t.vel, ang: t.ang }, mp, true);
    world.add_collider(cube, Shape::Box { hx: HALF, hy: HALF, hz: HALF }, material(CUBE));
    world.set_body_sleep_allowed(cube, false);

    let sub = ((t.dt * hz as f64).round() as usize).max(1);
    let h = (t.dt / sub as f64) as f32;
    let (mut pos, mut rot, mut eq) = (0.0f32, 0.0f32, 0.0f32);
    let n = t.pos.len() - 1;
    for k in 1..=n {
        for _ in 0..sub { world.step(h); }
        let s = world.get_body_pose(cube);
        let dp = (s.pos - t.pos[k]).length();
        let mut dq = s.rot * t.rot[k].conjugate();
        if dq.w < 0.0 { dq = -dq; }
        let ang = 2.0 * Vec3::new(dq.x, dq.y, dq.z).length().atan2(dq.w);
        pos += dp / (2.0 * HALF);
        rot += ang;
        // Eq. 3 as published: (2/l) times the squared distance in metres, plus the angle squared
        eq += dp * dp / HALF + ang * ang;
    }
    let n = n as f32;
    TossError { pos_pct: 100.0 * pos / n, rot_deg: rot.to_degrees() / n, e_q: eq / n }
}

/// Replays every toss (in parallel; results in toss order, so totals are deterministic).
pub fn replay_all(ts: &[Toss], p: &Params, table_y: f32, hz: f32) -> Vec<TossError> {
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get()).min(16);
    let chunk = ts.len().div_ceil(threads).max(1);
    std::thread::scope(|s| {
        let handles: Vec<_> = ts.chunks(chunk).map(|c| s.spawn(move || c.iter().map(|t| replay(t, p, table_y, hz)).collect::<Vec<_>>())).collect();
        handles.into_iter().flat_map(|h| h.join().unwrap()).collect()
    })
}

/// Mean and standard deviation over tosses: (pos %, std), (rot deg, std), (e_q, std).
pub fn summary(es: &[TossError]) -> [(f32, f32); 3] {
    let stat = |f: &dyn Fn(&TossError) -> f32| {
        let n = es.len().max(1) as f32;
        let m = es.iter().map(|e| f(e)).sum::<f32>() / n;
        let v = es.iter().map(|e| (f(e) - m).powi(2)).sum::<f32>() / n;
        (m, v.sqrt())
    };
    [stat(&|e| e.pos_pct), stat(&|e| e.rot_deg), stat(&|e| e.e_q)]
}

/// Fits the contact parameters by CMA-ES, minimizing mean e_q over `train`.
pub fn fit(train: &[Toss], init: &Params, table_y: f32, hz: f32, max_evals: usize, seed: u64) -> (Params, usize) {
    let r = cmaes::minimize(|x| {
        let es = replay_all(train, &Params::from_unit(x), table_y, hz);
        es.iter().map(|e| e.e_q as f64).sum::<f64>() / es.len() as f64
    }, &init.to_unit(), 0.25, max_evals, seed);
    (Params::from_unit(&r.x), r.evals)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A toss synthesized by RiftPhys itself replays with zero error, and other parameters do not.
    #[test]
    fn self_replay_is_exact_and_parameters_matter() {
        let p = Params::measured();
        let mut t = Toss {
            index: 0, dt: 1.0 / 148.0, pos: vec![Vec3::new(0.0, 0.3, 0.0)],
            rot: vec![Quat::from_rotation_x(0.4) * Quat::from_rotation_z(0.3)],
            vel: Vec3::new(0.8, 0.5, -0.3), ang: Vec3::new(3.0, -1.0, 2.0),
        };
        // Record RiftPhys's own trajectory, with the same scene, as the "measurement"
        let frames = 150;
        let mut w = World::with_capacity(4, 4);
        w.set_gravity(Vec3::new(0.0, -9.81, 0.0));
        let g = w.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
        w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(TABLE));
        let mut props = w.pair_props(CUBE, TABLE);
        props.restitution = p.restitution; props.mu_k = p.mu_k; props.mu_s = p.mu_s;
        w.set_pair_props(CUBE, TABLE, props);
        let mut mp = MassProps::from_box_density(Vec3::splat(HALF), MASS / (8.0 * HALF * HALF * HALF));
        mp.inertia = Mat3::from_diagonal(Vec3::splat(INERTIA).into());
        let c = w.add_body(Isometry { pos: t.pos[0], rot: t.rot[0] }, Velocity { lin: t.vel, ang: t.ang }, mp, true);
        w.add_collider(c, Shape::Box { hx: HALF, hy: HALF, hz: HALF }, material(CUBE));
        w.set_body_sleep_allowed(c, false);
        for _ in 0..frames {
            for _ in 0..10 { w.step(1.0 / 1480.0); }
            let s = w.get_body_pose(c);
            t.pos.push(s.pos);
            t.rot.push(s.rot);
        }
        let e = replay(&t, &p, 0.0, SIM_HZ);
        assert!(e.e_q < 1e-8 && e.pos_pct < 1e-3, "self replay {:?}", e);
        let other = replay(&t, &Params { restitution: 0.6, mu_s: 0.6, mu_k: 0.5 }, 0.0, SIM_HZ);
        assert!(other.e_q > 1e-3, "parameters had no effect: {:?}", other);
        // The cube really hit the table and settled on a face
        assert!((t.pos.last().unwrap().y - HALF).abs() < 0.003, "rest height {}", t.pos.last().unwrap().y);
    }
}
