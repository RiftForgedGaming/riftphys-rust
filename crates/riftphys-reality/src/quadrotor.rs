//! Rotor-driven vehicles replayed from measured rotor speeds. The log carries mocap poses for
//! one body and `cmd` lines with each rotor's speed (rpm, as the Blackbird tachometers report
//! it); the scene says where the rotors are and fits the aerodynamic coefficients.
//!
//! Per rotor: thrust k_thrust * (rpm / 1000)^2 along body up (+y), at the rotor's position, and
//! a reaction torque spin * k_torque * (rpm / 1000)^2 about body up. Plus linear drag on the body.
//! The rotor speeds are measured, not commanded, so no motor model sits in between.
//!
//! Body frame (RiftPhys): x forward, y up, z right. `tools/blackbird_to_reality.py` maps
//! Blackbird's NED mocap frame and FRD body frame onto it.

use crate::fit::{FitConfig, Replayable};
use crate::log::RealityLog;
use crate::scene::{Window, WindowError};
use riftphys_core::{Isometry, Mat3, Quat, Vec3, Velocity};
use riftphys_geom::MassProps;
use riftphys_world::world::World;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RotorSpec {
    /// Rotor hub position in the body frame (m)
    pub pos: [f32; 3],
    /// +1 or -1: sign of the reaction torque about body up
    pub spin: f32,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum QuadParamKind {
    /// Thrust coefficient, N per krpm^2
    KThrust,
    /// Reaction-torque coefficient, N m per krpm^2
    KTorque,
    /// Linear drag, N per m/s
    Drag,
    /// Multiplier on roll and pitch inertia (body x and z)
    InertiaRollPitch,
    /// Multiplier on yaw inertia (body y)
    InertiaYaw,
    /// Seconds the logged rotor speeds lag the real ones (estimated speeds are filtered): replay
    /// uses the log at t + lead
    RpmLead,
    /// Rotational damping, N m per rad/s (rotor drag and blade flapping resist body rotation)
    AngDrag,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct QuadParam {
    pub name: String,
    pub kind: QuadParamKind,
    pub min: f32,
    pub max: f32,
    pub init: f32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct QuadScene {
    pub name: String,
    pub hz: u32,
    /// Log body (mocap pose) and log agent (`cmd` rotor speeds, in `rotors` order)
    pub body: String,
    pub cmd: String,
    pub mass: f32,
    /// Principal inertia in the body frame (kg m^2): x (roll), y (yaw), z (pitch)
    pub inertia: [f32; 3],
    pub rotors: Vec<RotorSpec>,
    /// Values used for anything not listed in `params`
    pub k_thrust: f32,
    pub k_torque: f32,
    #[serde(default)]
    pub drag: f32,
    #[serde(default)]
    pub rpm_lead: f32,
    #[serde(default)]
    pub ang_drag: f32,
    #[serde(default)]
    pub params: Vec<QuadParam>,
    #[serde(default)]
    pub notes: String,
}

/// Physical values for one evaluation.
#[derive(Clone, Copy, Debug)]
pub struct QuadValues { pub k_thrust: f32, pub k_torque: f32, pub drag: f32, pub inertia: Vec3, pub rpm_lead: f32, pub ang_drag: f32 }

impl QuadScene {
    pub fn load(path: &str) -> Result<Self, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {}", path, e))?;
        serde_json::from_str(&text).map_err(|e| format!("{}: {}", path, e))
    }

    pub fn dt(&self) -> f32 { 1.0 / self.hz as f32 }

    pub fn values(&self, x: &[f32]) -> QuadValues {
        let mut v = QuadValues { k_thrust: self.k_thrust, k_torque: self.k_torque, drag: self.drag, inertia: Vec3::from_array(self.inertia), rpm_lead: self.rpm_lead, ang_drag: self.ang_drag };
        for (p, &val) in self.params.iter().zip(x) {
            match p.kind {
                QuadParamKind::KThrust => v.k_thrust = val,
                QuadParamKind::KTorque => v.k_torque = val,
                QuadParamKind::Drag => v.drag = val,
                QuadParamKind::InertiaRollPitch => { v.inertia.x = self.inertia[0] * val; v.inertia.z = self.inertia[2] * val; }
                QuadParamKind::InertiaYaw => v.inertia.y = self.inertia[1] * val,
                QuadParamKind::RpmLead => v.rpm_lead = val,
                QuadParamKind::AngDrag => v.ang_drag = val,
            }
        }
        v
    }

    /// A world with the vehicle (no ground: replay windows are in flight).
    pub fn build(&self, v: &QuadValues, pose: Isometry, vel: Velocity) -> (World, riftphys_core::BodyId) {
        let mut world = World::with_capacity(4, 4);
        world.set_gravity(Vec3::new(0.0, -9.81, 0.0));
        let mp = MassProps { mass: self.mass, inv_mass: 1.0 / self.mass, inertia: Mat3::from_diagonal(v.inertia) };
        let id = world.add_body(pose, vel, mp, true);
        world.set_body_sleep_allowed(id, false);
        (world, id)
    }

    /// Applies one step of rotor forces for speeds `rpm` (rotors order) over `dt`.
    pub fn apply_rotors(&self, world: &mut World, id: riftphys_core::BodyId, v: &QuadValues, rpm: &[f32], dt: f32) {
        let pose = world.get_body_pose(id);
        let up = pose.rot * Vec3::Y;
        let (mut force, mut torque) = (Vec3::ZERO, Vec3::ZERO);
        for (r, &w) in self.rotors.iter().zip(rpm) {
            let k2 = (w / 1000.0).powi(2);
            let f = up * (v.k_thrust * k2);
            force += f;
            torque += (pose.rot * Vec3::from_array(r.pos)).cross(f) + up * (r.spin * v.k_torque * k2);
        }
        force -= world.get_body_vel(id).lin * v.drag;
        torque -= world.get_body_vel(id).ang * v.ang_drag;
        world.apply_impulse(id, force * dt);
        world.apply_torque(id, torque * dt);
    }

    /// Rotor speeds at `t`, linearly interpolated between logged samples.
    pub fn rpm_at(&self, log: &RealityLog, t: f64) -> Option<Vec<f32>> {
        let c = log.cmds.get(&self.cmd)?;
        if c.is_empty() || t < c[0].0 || t > c[c.len() - 1].0 { return None; }
        let i = c.partition_point(|(ct, _)| *ct <= t).max(1) - 1;
        let j = (i + 1).min(c.len() - 1);
        let f = if c[j].0 > c[i].0 { ((t - c[i].0) / (c[j].0 - c[i].0)) as f32 } else { 0.0 };
        Some(c[i].1.iter().zip(&c[j].1).map(|(a, b)| a + (b - a) * f).collect())
    }

    /// Hover-point estimate of k_thrust from the log: average total rotor (rpm/1000)^2 against
    /// mass x vertical (specific) acceleration. A good starting guess for fitting.
    pub fn estimate_k_thrust(&self, log: &RealityLog, half: f64) -> Option<f32> {
        let tr = log.tracks.get(&self.body)?;
        let (mut num, mut den) = (0.0f64, 0.0f64);
        for s in tr.iter().step_by(10) {
            let (Some(rpm), Some((_, v0, _, res0)), Some((_, v1, _, _))) = (self.rpm_at(log, s.time), log.state_at(&self.body, s.time - 0.01, half), log.state_at(&self.body, s.time + 0.01, half)) else { continue };
            if res0 > 0.005 { continue; }
            let accel = (v1 - v0) / 0.02 + Vec3::new(0.0, 9.81, 0.0);
            let up = s.rot * Vec3::Y;
            let sum_k2: f32 = rpm.iter().map(|w| (w / 1000.0).powi(2)).sum();
            num += (self.mass * accel.dot(up)) as f64 * sum_k2 as f64;
            den += (sum_k2 * sum_k2) as f64;
        }
        (den > 0.0).then(|| (num / den) as f32)
    }
}

impl Replayable for QuadScene {
    fn params(&self) -> Vec<(String, f32, f32, f32)> {
        self.params.iter().map(|p| (p.name.clone(), p.min, p.max, p.init)).collect()
    }

    fn windows(&self, log: &RealityLog, cfg: &FitConfig) -> Vec<Window> {
        let Some((lo, hi)) = log.span(&[self.body.clone()]) else { return vec![] };
        let Some(c) = log.cmds.get(&self.cmd) else { return vec![] };
        if c.is_empty() { return vec![]; }
        let (lo, hi) = (lo.max(c[0].0), hi.min(c[c.len() - 1].0));
        let mut out = Vec::new();
        let mut t = lo + cfg.vel_half;
        while t + cfg.window <= hi - cfg.vel_half {
            let crosses = log.breaks.iter().any(|&b| b > t - cfg.vel_half && b < t + cfg.window);
            let smooth = log.state_at(&self.body, t, cfg.vel_half).map_or(false, |s| s.3 <= cfg.max_residual);
            if !crosses && smooth { out.push(Window { start: t, len: cfg.window }); }
            t += cfg.stride;
        }
        out
    }

    fn replay(&self, log: &RealityLog, x: &[f32], w: Window, cfg: &FitConfig) -> Option<WindowError> {
        let v = self.values(x);
        let (_, q) = log.pose_at(&self.body, w.start)?;
        let (p, lin, ang, _) = log.state_at(&self.body, w.start, cfg.vel_half)?;
        let (mut world, id) = self.build(&v, Isometry { pos: p, rot: q }, Velocity { lin, ang });
        let dt = self.dt() as f64;
        let steps = (w.len / dt).ceil() as usize;
        let mut sim = vec![p];
        for k in 0..steps {
            let rpm = self.rpm_at(log, w.start + (k as f64 + 0.5) * dt + v.rpm_lead as f64)?;
            self.apply_rotors(&mut world, id, &v, &rpm, dt as f32);
            world.step(dt as f32);
            sim.push(world.get_body_pose(id).pos);
        }
        let mut err = WindowError::default();
        for s in log.tracks.get(&self.body)?.iter().filter(|s| s.time > w.start && s.time <= w.start + w.len) {
            let f = (s.time - w.start) / dt;
            let i = (f.floor() as usize).min(steps - 1);
            let pos = sim[i].lerp(sim[i + 1], (f - i as f64) as f32);
            err.samples.push(((s.time - w.start) as f32, (pos - s.pos).length()));
        }
        Some(err)
    }
}

// ---------------------------------------------------------------- synthetic flights

/// Per-rotor thrusts producing collective thrust `t` and body torques `tau` (least squares over
/// the rotor layout: rows [T, tau_x, tau_y, tau_z]).
fn mix(scene: &QuadScene, v: &QuadValues, t: f32, tau: Vec3) -> Vec<f32> {
    let n = scene.rotors.len();
    // A (4 x n): per unit rotor thrust
    let ratio = v.k_torque / v.k_thrust;
    let cols: Vec<[f32; 4]> = scene.rotors.iter().map(|r| {
        let tq = Vec3::from_array(r.pos).cross(Vec3::Y) + Vec3::Y * (r.spin * ratio);
        [1.0, tq.x, tq.y, tq.z]
    }).collect();
    let b = [t, tau.x, tau.y, tau.z];
    // Minimum-norm solution f = A^T (A A^T)^-1 b
    let mut m = [[0.0f64; 4]; 4];
    for i in 0..4 { for j in 0..4 { m[i][j] = cols.iter().map(|c| (c[i] * c[j]) as f64).sum(); } }
    let mut aug: Vec<Vec<f64>> = (0..4).map(|i| { let mut r = m[i].to_vec(); r.push(b[i] as f64); r }).collect();
    for c in 0..4 {
        let p = (c..4).max_by(|&x, &y| aug[x][c].abs().partial_cmp(&aug[y][c].abs()).unwrap()).unwrap();
        aug.swap(c, p);
        let d = aug[c][c];
        if d.abs() < 1e-12 { return vec![t / n as f32; n]; }
        for k in c..5 { aug[c][k] /= d; }
        for r in 0..4 { if r != c { let f = aug[r][c]; for k in c..5 { aug[r][k] -= f * aug[c][k]; } } }
    }
    let y: Vec<f64> = (0..4).map(|i| aug[i][4]).collect();
    cols.iter().map(|c| (0..4).map(|i| c[i] as f64 * y[i]).sum::<f64>() as f32).collect()
}

pub struct QuadSynthConfig { pub seconds: f32, pub pose_hz: f32, pub rpm_hz: f32, pub pos_noise: f32, pub rpm_noise: f32, pub seed: u64 }

/// A flight with known coefficients: a PD controller chases random waypoints and the log records
/// mocap-rate poses and tachometer-rate rotor speeds (with noise), like a Blackbird log.
pub fn synthesize(scene: &QuadScene, truth: &[f32], cfg: &QuadSynthConfig) -> RealityLog {
    let v = scene.values(truth);
    let mut rng = cfg.seed.max(1);
    let mut unit = move || { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; ((rng >> 40) as f32 + 0.5) / (1u64 << 24) as f32 };
    let start = Vec3::new(0.0, 1.5, 0.0);
    let (mut world, id) = scene.build(&v, Isometry { pos: start, rot: Quat::IDENTITY }, Velocity::default());
    // Samples are taken at simulation steps, so neither stream can be faster than `hz`
    let pose_hz = cfg.pose_hz.min(scene.hz as f32);
    let rpm_hz = cfg.rpm_hz.min(scene.hz as f32);
    let mut log = RealityLog { source: "synthetic".into(), rate_hz: pose_hz, ..Default::default() };
    log.tracks.insert(scene.body.clone(), Vec::new());
    log.cmds.insert(scene.cmd.clone(), Vec::new());
    let dt = scene.dt();
    let steps = (cfg.seconds / dt) as usize;
    let (mut target, mut next_target) = (start, 0.0f32);
    let (mut next_pose, mut next_rpm) = (0.0f64, 0.0f64);
    let mut gauss = |u: &mut dyn FnMut() -> f32| { let (a, b) = (u(), u()); (-2.0 * a.ln()).sqrt() * (2.0 * std::f32::consts::PI * b).cos() };
    let k_max = 30.0f32; // krpm
    for k in 0..steps {
        let t = k as f32 * dt;
        if t >= next_target {
            target = Vec3::new(unit() * 4.0 - 2.0, 0.8 + unit() * 1.5, unit() * 4.0 - 2.0);
            next_target = t + 1.0 + unit() * 1.5;
        }
        let p = world.get_body_pose(id);
        let vel = world.get_body_vel(id);
        // Position PD -> desired acceleration -> desired thrust direction
        let a_des = ((target - p.pos) * 6.0 - vel.lin * 4.0).clamp_length_max(8.0) + Vec3::new(0.0, 9.81, 0.0);
        let up = p.rot * Vec3::Y;
        let thrust = scene.mass * a_des.dot(up).max(0.0);
        let want_up = a_des.normalize_or_zero();
        let err = up.cross(want_up); // world-frame tilt error
        let fwd = p.rot * Vec3::X;
        let yaw_err = Vec3::new(fwd.z, 0.0, 0.0).length() * -fwd.z.signum(); // hold heading +x
        let w_body = p.rot.inverse() * vel.ang;
        let tilt_body = p.rot.inverse() * err;
        let tau = Vec3::new(
            v.inertia.x * (120.0 * tilt_body.x - 20.0 * w_body.x),
            v.inertia.y * (8.0 * yaw_err - 4.0 * w_body.y),
            v.inertia.z * (120.0 * tilt_body.z - 20.0 * w_body.z),
        );
        let f = mix(scene, &v, thrust, tau);
        let rpm: Vec<f32> = f.iter().map(|fi| (fi.max(0.0) / v.k_thrust).sqrt().min(k_max) * 1000.0).collect();
        let tt = k as f64 * dt as f64;
        if tt >= next_pose {
            let n = Vec3::new(gauss(&mut unit), gauss(&mut unit), gauss(&mut unit)) * cfg.pos_noise;
            log.tracks.get_mut(&scene.body).unwrap().push(crate::log::Sample { time: tt, pos: p.pos + n, rot: p.rot });
            next_pose += 1.0 / pose_hz as f64;
        }
        if tt >= next_rpm {
            let noisy: Vec<f32> = rpm.iter().map(|r| r + gauss(&mut unit) * cfg.rpm_noise).collect();
            log.cmds.get_mut(&scene.cmd).unwrap().push((tt, noisy));
            next_rpm += 1.0 / rpm_hz as f64;
        }
        scene.apply_rotors(&mut world, id, &v, &rpm, dt);
        world.step(dt);
    }
    log
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fit::fit;

    #[test]
    fn recovers_thrust_coefficient_from_a_synthetic_flight() {
        let scene: QuadScene = serde_json::from_str(include_str!("../scenes/quad_x.json")).unwrap();
        let mut scene = scene;
        scene.params.retain(|p| p.name == "k_thrust" || p.name == "inertia_rp");
        let truth = [0.01, 1.3];
        let log = synthesize(&scene, &truth, &QuadSynthConfig { seconds: 20.0, pose_hz: 240.0, rpm_hz: 190.0, pos_noise: 0.0003, rpm_noise: 20.0, seed: 3 });
        let est = scene.estimate_k_thrust(&log, 0.03).unwrap();
        assert!((est - 0.01).abs() < 0.0005, "hover estimate {}", est);
        let cfg = FitConfig { window: 0.5, stride: 0.2, max_residual: 0.003, max_evals: 120, ..FitConfig::default() };
        let r = fit(&scene, &log, &cfg).unwrap();
        assert!((r.fitted[0] - 0.01).abs() < 0.0003, "k_thrust {:?}", r.fitted);
        assert!(r.test_after.rms[2] < 0.5 * r.test_before.rms[2], "held-out error {:?} -> {:?}", r.test_before.rms, r.test_after.rms);
    }
}
