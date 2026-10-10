//! Quadrotor scenario: one drone or a squad holding a formation, on RiftPhys rigid bodies.
//!
//! Plant: each drone is a dynamic box with four rotors in a "+" layout at arm length L on the
//! body x/z axes. A rotor command u in [0, 1] gives thrust u * T_MAX along the body up axis;
//! the mixer turns the four thrusts into total thrust plus roll/pitch/yaw torques, applied as
//! impulses before each world step. The controller only ever outputs rotor commands, as on a
//! real drone.
//!
//! Expert: a cascaded PD controller (position -> desired acceleration -> desired tilt ->
//! torques -> rotor mix). It is the demonstrator for one-shot imitation; the rollout can execute
//! a noise-perturbed copy (DART-style) while recording the expert's clean command as `demo`.

use crate::protocol::{ActuatorSpec, EntState, KindSpec};
use crate::rng::Rng;
use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::World;

pub const KIND: &str = "quadrotor";
pub const ROTORS: usize = 4;

/// Body half extents (m), mass (kg), arm length (m), max thrust per rotor (N), yaw drag coefficient (m).
pub const HALF: Vec3 = Vec3::new(0.25, 0.05, 0.25);
pub const MASS: f32 = 1.2;
pub const ARM: f32 = 0.2;
pub const G: f32 = 9.81;
/// Hover at u = 0.5 on every rotor.
pub const T_MAX: f32 = MASS * G / (ROTORS as f32 * 0.5);
pub const K_YAW: f32 = 0.02;

/// Rotor order: 0 = +x, 1 = -x, 2 = +z, 3 = -z. Spin pairs (0,1) and (2,3) are opposite, so
/// yaw torque = K_YAW * ((F0 + F1) - (F2 + F3)).
pub fn kind_spec() -> KindSpec {
    let rotor = |n: &str| ActuatorSpec { name: n.to_string(), min: 0.0, max: 1.0, unit: "throttle".to_string() };
    KindSpec { kind: KIND.to_string(), actuators: vec![rotor("rotor_px"), rotor("rotor_nx"), rotor("rotor_pz"), rotor("rotor_nz")] }
}

/// Body-frame wrench (thrust along +y, torque) produced by rotor forces.
pub fn wrench(forces: [f32; ROTORS]) -> (f32, Vec3) {
    let [f0, f1, f2, f3] = forces;
    let thrust = f0 + f1 + f2 + f3;
    // r x (0, F, 0) for r = (x, 0, z) is (-z F, 0, x F)
    let tau = Vec3::new(ARM * (f3 - f2), K_YAW * ((f0 + f1) - (f2 + f3)), ARM * (f0 - f1));
    (thrust, tau)
}

/// Inverse of `wrench`, with priority desaturation like a real flight controller: thrust is
/// kept, yaw gets at most half the thrust's differential, and roll/pitch are scaled down
/// together (never re-signed) until every rotor fits [0, T_MAX]. Exact when nothing saturates.
pub fn mix(thrust: f32, tau: Vec3) -> [f32; ROTORS] {
    let t = thrust.clamp(0.0, 0.95 * ROTORS as f32 * T_MAX);
    let yaw = (tau.y / K_YAW).clamp(-0.5 * t, 0.5 * t);
    let a = 0.5 * (t + yaw); // F0 + F1
    let b = 0.5 * (t - yaw); // F2 + F3
    let (z, x) = (tau.z / ARM, tau.x / ARM);
    // Largest s <= 1 keeping a +- s|z| and b +- s|x| inside [0, 2 T_MAX]
    let room = |sum: f32, d: f32| if d.abs() < 1e-9 { f32::INFINITY } else { sum.min(2.0 * T_MAX - sum).max(0.0) / d.abs() };
    let s = room(a, z).min(room(b, x)).min(1.0);
    let f = [
        0.5 * (a + s * z),
        0.5 * (a - s * z),
        0.5 * (b - s * x),
        0.5 * (b + s * x),
    ];
    f.map(|fi| (fi / T_MAX).clamp(0.0, 1.0))
}

pub struct Drone {
    pub body: BodyId,
    pub target: Vec3,
    pub inertia: Vec3,
    /// True mass (kg): MASS plus any payload.
    pub mass: f32,
    /// Steady wind force on this drone (N, world frame).
    pub wind: Vec3,
    pub last_act: [f32; ROTORS],
    pub last_demo: Option<[f32; ROTORS]>,
    pub crashed: bool,
}

pub struct DroneScenario {
    pub drones: Vec<Drone>,
    pub formation_center: Vec3,
    pub spacing: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct DroneConfig {
    pub count: usize,
    pub spacing: f32,
    /// Random start offset from the slot (m) and tilt (rad).
    pub start_offset: f32,
    pub start_tilt: f32,
    pub altitude: f32,
    /// Random payload per drone: up to this fraction of MASS extra (0 = none).
    pub payload: f32,
    /// Random steady horizontal wind per drone: up to this force (N) (0 = none).
    pub wind: f32,
}

impl Default for DroneConfig {
    fn default() -> Self {
        Self { count: 1, spacing: 1.5, start_offset: 1.5, start_tilt: 0.25, altitude: 3.0, payload: 0.0, wind: 0.0 }
    }
}

/// Formation slot for drone `i` of `n`: a line for 2-3, a ring beyond.
pub fn formation_slot(i: usize, n: usize, spacing: f32) -> Vec3 {
    if n <= 1 {
        return Vec3::ZERO;
    }
    if n <= 3 {
        return Vec3::new((i as f32 - (n as f32 - 1.0) * 0.5) * spacing, 0.0, 0.0);
    }
    let a = std::f32::consts::TAU * i as f32 / n as f32;
    let r = spacing * n as f32 / std::f32::consts::TAU;
    Vec3::new(r * a.cos(), 0.0, r * a.sin())
}

impl DroneScenario {
    /// Builds the ground and the squad into a fresh world.
    pub fn build(world: &mut World, cfg: &DroneConfig, rng: &mut Rng) -> Self {
        world.set_gravity(Vec3::new(0.0, -G, 0.0));
        let ground = world.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
        world.add_collider(ground, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(MaterialId::Concrete));

        let center = Vec3::new(rng.range(-2.0, 2.0), cfg.altitude, rng.range(-2.0, 2.0));
        let mut drones = Vec::with_capacity(cfg.count);
        for i in 0..cfg.count {
            let target = center + formation_slot(i, cfg.count, cfg.spacing);
            let start = target + Vec3::new(rng.range(-1.0, 1.0), rng.range(-1.0, 1.0) * 0.5, rng.range(-1.0, 1.0)) * cfg.start_offset;
            let start = Vec3::new(start.x, start.y.max(0.5), start.z);
            let tilt = Quat::from_euler(glam::EulerRot::XYZ, rng.range(-1.0, 1.0) * cfg.start_tilt, rng.range(-3.14, 3.14), rng.range(-1.0, 1.0) * cfg.start_tilt);
            // Disturbances draw random numbers only when enabled, so default episodes are unchanged
            let mass = if cfg.payload > 0.0 { MASS * (1.0 + rng.range(0.0, cfg.payload)) } else { MASS };
            let wind = if cfg.wind > 0.0 {
                let a = rng.range(0.0, std::f32::consts::TAU);
                Vec3::new(a.cos(), 0.0, a.sin()) * rng.range(0.3, 1.0) * cfg.wind
            } else {
                Vec3::ZERO
            };
            let mp = MassProps::from_box_density(HALF, mass / (8.0 * HALF.x * HALF.y * HALF.z));
            let body = world.add_body(Isometry { pos: start, rot: tilt }, Velocity::default(), mp, true);
            world.add_collider(body, Shape::Box { hx: HALF.x, hy: HALF.y, hz: HALF.z }, material(MaterialId::Aluminum));
            world.set_body_sleep_allowed(body, false);
            let inertia = Vec3::new(mp.inertia.x_axis.x, mp.inertia.y_axis.y, mp.inertia.z_axis.z);
            drones.push(Drone { body, target, inertia, mass, wind, last_act: [0.0; ROTORS], last_demo: None, crashed: false });
        }
        Self { drones, formation_center: center, spacing: cfg.spacing }
    }

    /// Applies rotor commands for drone `i` for one tick of length `dt`.
    pub fn apply(&mut self, world: &mut World, i: usize, act: [f32; ROTORS], dt: f32) {
        let d = &mut self.drones[i];
        let u = act.map(|a| a.clamp(0.0, 1.0));
        d.last_act = u;
        if d.crashed {
            return;
        }
        let (thrust, tau_body) = wrench(u.map(|ui| ui * T_MAX));
        let rot = world.get_body_pose(d.body).rot;
        world.apply_impulse(d.body, (rot * Vec3::new(0.0, thrust, 0.0) + d.wind) * dt);
        world.apply_torque(d.body, rot * tau_body * dt);
    }

    pub fn observe(&self, world: &World) -> Vec<EntState> {
        self.drones.iter().map(|d| {
            let pose = world.get_body_pose(d.body);
            let vel = world.get_body_vel(d.body);
            EntState {
                eid: d.body.0,
                kind: KIND.to_string(),
                pos: pose.pos.to_array(),
                rot: pose.rot.to_array(),
                vel: vel.lin.to_array(),
                ang: vel.ang.to_array(),
                target: Some(d.target.to_array()),
                act: d.last_act.to_vec(),
                demo: d.last_demo.map(|a| a.to_vec()),
            }
        }).collect()
    }
}

/// Cascaded PD hover/formation controller.
///
/// `oracle` controllers know each drone's true mass and wind and feed them forward (the
/// demonstrator). Non-oracle controllers assume the nominal airframe in still air, like a
/// controller tuned on the bench and flown with an unknown payload.
#[derive(Clone, Copy, Debug)]
pub struct ExpertPd {
    pub kp: f32,
    pub kd: f32,
    pub max_tilt_accel: f32,
    pub k_att: f32,
    pub k_rate: f32,
    pub k_yaw: f32,
    pub oracle: bool,
}

impl Default for ExpertPd {
    fn default() -> Self {
        Self { kp: 4.0, kd: 3.0, max_tilt_accel: 5.0, k_att: 40.0, k_rate: 10.0, k_yaw: 3.0, oracle: true }
    }
}

impl ExpertPd {
    /// A deliberately weak, nominal-model controller: keeps a drone up but drifts and sags
    /// under payload and wind. The base that residual learning corrects.
    pub fn nominal_base() -> Self {
        Self { kp: 2.0, kd: 2.0, max_tilt_accel: 4.0, k_att: 30.0, k_rate: 8.0, k_yaw: 2.0, oracle: false }
    }
}

impl ExpertPd {
    pub fn command(&self, world: &World, d: &Drone) -> [f32; ROTORS] {
        let pose = world.get_body_pose(d.body);
        let vel = world.get_body_vel(d.body);
        let rot = pose.rot;

        let (mass, wind) = if self.oracle { (d.mass, d.wind) } else { (MASS, Vec3::ZERO) };

        // Position loop -> desired acceleration (gravity and known wind compensated), horizontal limited
        let mut a = (d.target - pose.pos) * self.kp - vel.lin * self.kd - wind / mass;
        let h = Vec3::new(a.x, 0.0, a.z);
        if h.length() > self.max_tilt_accel {
            let s = self.max_tilt_accel / h.length();
            a.x *= s;
            a.z *= s;
        }
        a.y = a.y.max(-0.8 * G) + G;

        let up = rot * Vec3::Y;
        let up_des = a.normalize_or_zero();
        let thrust = mass * a.dot(up).max(0.0);

        // Attitude loop: rotate up toward up_des, damp rates, hold yaw = 0
        let fwd = rot * Vec3::Z;
        let yaw = fwd.x.atan2(fwd.z);
        let err = up.cross(up_des);
        let ang_world = vel.ang;
        let ang_body = rot.inverse() * ang_world;
        let err_body = rot.inverse() * err;
        let alpha = Vec3::new(
            self.k_att * err_body.x - self.k_rate * ang_body.x,
            -self.k_yaw * yaw - self.k_rate * 0.5 * ang_body.y,
            self.k_att * err_body.z - self.k_rate * ang_body.z,
        );
        mix(thrust, alpha * d.inertia)
    }
}

/// Tilt from vertical (rad).
pub fn tilt(world: &World, d: &Drone) -> f32 {
    let up = world.get_body_pose(d.body).rot * Vec3::Y;
    up.y.clamp(-1.0, 1.0).acos()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mixer_inverts_wrench() {
        let thrust = MASS * G;
        let tau = Vec3::new(0.05, 0.01, -0.04);
        let u = mix(thrust, tau);
        let (t2, tau2) = wrench(u.map(|x| x * T_MAX));
        assert!((t2 - thrust).abs() < 1e-3);
        assert!((tau2 - tau).length() < 1e-4, "{:?} vs {:?}", tau2, tau);
    }

    #[test]
    fn hover_command_is_half_throttle() {
        let u = mix(MASS * G, Vec3::ZERO);
        for x in u {
            assert!((x - 0.5).abs() < 1e-5);
        }
    }

    #[test]
    fn formation_slots_are_distinct() {
        for n in 1..8 {
            for i in 0..n {
                for j in (i + 1)..n {
                    assert!((formation_slot(i, n, 1.5) - formation_slot(j, n, 1.5)).length() > 1.0);
                }
            }
        }
    }
}

