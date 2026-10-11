//! Biped (humanoid lower body) scenario, revived from argref `uriel-agent-biped` onto the
//! corrected RiftPhys physics.
//!
//! Morphology (80 kg, y up, forward +z): head, torso, pelvis, and per leg a thigh, shank and a
//! flat foot. The old agent balanced on rounded calf tips with two non-physical assists (a
//! weight-bearing tether and an external stabilising torque on the torso); this version adds
//! ankles and feet, so it can stand the way real humanoids do, with ankle torque against the
//! ground, and uses no assists. All joints are hinges about the body x axis (sagittal plane);
//! side-to-side balance comes from the stance width and the feet.
//!
//! Actuation: six joint torques (hip, knee, ankle per leg), each applied equal and opposite to
//! the parent and child about the joint axis, clamped to a per-joint limit.

use crate::protocol::{EntState, KindSpec, ActuatorSpec};
use crate::rng::Rng;
use riftphys_articulation::bodies::D6Limit;
use riftphys_core::{BodyId, Isometry, JointId, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::World;

pub const KIND: &str = "biped";
pub const JOINTS: usize = 6;
pub const G: f32 = 9.81;

/// Joint order everywhere (commands, observations): left hip/knee/ankle, right hip/knee/ankle.
pub const JOINT_NAMES: [&str; JOINTS] = ["hip_l", "knee_l", "ankle_l", "hip_r", "knee_r", "ankle_r"];
/// Torque limits (N m), same order.
pub const MAX_TORQUE: [f32; JOINTS] = [200.0, 200.0, 120.0, 200.0, 200.0, 120.0];
/// Control rate (Hz): one observation / command per tick.
pub const CONTROL_HZ: u32 = 60;
/// Physics and servo substeps per control tick (240 Hz). Joint torques are recomputed every
/// substep, like a robot's inner servo loop running faster than its policy.
pub const SUBSTEPS: u32 = 4;
/// Reflected actuator (rotor) inertia added to each limb body about its joint axis, kg m^2 -
/// MuJoCo's `armature`. Without it a 1 kg foot (I ~ 0.005) makes any useful ankle PD unstable.
pub const ARMATURE: f32 = 0.08;

pub fn kind_spec() -> KindSpec {
    KindSpec {
        kind: KIND.to_string(),
        actuators: JOINT_NAMES.iter().zip(MAX_TORQUE).map(|(n, m)| ActuatorSpec { name: n.to_string(), min: -m, max: m, unit: "N m".to_string() }).collect(),
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Joint {
    pub id: JointId,
    pub parent: BodyId,
    pub child: BodyId,
}

pub struct Biped {
    pub head: BodyId,
    pub torso: BodyId,
    pub pelvis: BodyId,
    /// [left, right]
    pub thigh: [BodyId; 2],
    pub shank: [BodyId; 2],
    pub foot: [BodyId; 2],
    pub joints: [Joint; JOINTS],
    pub masses: Vec<(BodyId, f32)>,
    pub last_act: [f32; JOINTS],
    pub last_demo: Option<[f32; JOINTS]>,
    pub fallen: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct BipedConfig {
    /// Random initial lean of the whole body (rad, about x).
    pub start_lean: f32,
    /// Probability per tick of a push on the torso, and its size (N s, horizontal).
    pub push_rate: f32,
    pub push_impulse: f32,
    /// World solver iterations (velocity, position) per physics substep. (8, 6) holds 20/20 at
    /// 12 and 20 N s pushes like (20, 20) at 2.75x the speed; (4, 2) is too soft to stand.
    pub solver_iters: (u32, u32),
}

impl Default for BipedConfig {
    fn default() -> Self {
        Self { start_lean: 0.03, push_rate: 0.004, push_impulse: 12.0, solver_iters: (8, 6) }
    }
}

fn box_body(world: &mut World, pos: Vec3, half: Vec3, mass: f32, shape: Shape, mat: MaterialId, armature: f32) -> (BodyId, f32) {
    let mut mp = MassProps::from_box_density(half, mass / (8.0 * half.x * half.y * half.z));
    mp.inertia.x_axis.x += armature;
    let id = world.add_body(Isometry { pos, rot: Quat::IDENTITY }, Velocity::default(), mp, true);
    world.add_collider(id, shape, material(mat));
    world.set_body_sleep_allowed(id, false);
    (id, mass)
}

fn iso(x: f32, y: f32, z: f32) -> Isometry {
    Isometry { pos: Vec3::new(x, y, z), rot: Quat::IDENTITY }
}

impl Biped {
    pub fn build(world: &mut World, cfg: &BipedConfig, rng: &mut Rng) -> Self {
        world.set_gravity(Vec3::new(0.0, -G, 0.0));
        world.set_solver_iterations(cfg.solver_iters.0, cfg.solver_iters.1);
        let ground = world.add_body(iso(0.0, -0.5, 0.0), Velocity::default(), MassProps::infinite(), false);
        world.add_collider(ground, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(MaterialId::Concrete));

        let mut masses = Vec::new();
        let mut add = |w: &mut World, pos: Vec3, half: Vec3, m: f32, shape: Shape, mat: MaterialId, arm: f32| {
            let (id, mass) = box_body(w, pos, half, m, shape, mat, arm);
            masses.push((id, mass));
            id
        };

        // Heights: foot centre 0.03, ankle 0.08, knee 0.48, hip 0.88, pelvis 0.96, torso 1.44, head 1.95
        let pelvis = add(world, Vec3::new(0.0, 0.96, 0.0), Vec3::new(0.18, 0.08, 0.10), 10.0, Shape::Box { hx: 0.18, hy: 0.08, hz: 0.10 }, MaterialId::Aluminum, 0.0);
        let torso = add(world, Vec3::new(0.0, 1.44, 0.0), Vec3::new(0.20, 0.40, 0.12), 40.0, Shape::Capsule { r: 0.2, hh: 0.2 }, MaterialId::Aluminum, 0.0);
        let head = add(world, Vec3::new(0.0, 1.95, 0.0), Vec3::new(0.1, 0.1, 0.1), 4.0, Shape::Box { hx: 0.1, hy: 0.1, hz: 0.1 }, MaterialId::Aluminum, 0.0);
        let mut thigh = [pelvis; 2];
        let mut shank = [pelvis; 2];
        let mut foot = [pelvis; 2];
        for (s, x) in [(0usize, 0.12f32), (1, -0.12)] {
            thigh[s] = add(world, Vec3::new(x, 0.68, 0.0), Vec3::new(0.06, 0.2, 0.06), 8.0, Shape::Capsule { r: 0.06, hh: 0.14 }, MaterialId::Aluminum, ARMATURE);
            shank[s] = add(world, Vec3::new(x, 0.28, 0.0), Vec3::new(0.05, 0.2, 0.05), 4.0, Shape::Capsule { r: 0.05, hh: 0.15 }, MaterialId::Aluminum, ARMATURE);
            foot[s] = add(world, Vec3::new(x, 0.03, 0.04), Vec3::new(0.06, 0.03, 0.12), 1.0, Shape::Box { hx: 0.06, hy: 0.03, hz: 0.12 }, MaterialId::Rubber, ARMATURE);
        }

        // Rigid upper body: spine and neck are hinges locked at 0
        let spine = world.add_hinge_joint(pelvis, torso, iso(0.0, 0.08, 0.0), iso(0.0, -0.40, 0.0), 0);
        world.set_d6_limit_r(spine, 0, Some(D6Limit { min: 0.0, max: 0.0 }));
        let neck = world.add_hinge_joint(torso, head, iso(0.0, 0.40, 0.0), iso(0.0, -0.11, 0.0), 0);
        world.set_d6_limit_r(neck, 0, Some(D6Limit { min: 0.0, max: 0.0 }));

        // Rotation about +x carries -y toward -z: positive knee angle = flexion (foot back)
        let mut joints = Vec::with_capacity(JOINTS);
        for (s, x) in [(0usize, 0.12f32), (1, -0.12)] {
            let hip = world.add_hinge_joint(pelvis, thigh[s], iso(x, -0.08, 0.0), iso(0.0, 0.2, 0.0), 0);
            world.set_d6_limit_r(hip, 0, Some(D6Limit { min: -1.5, max: 0.8 }));
            let knee = world.add_hinge_joint(thigh[s], shank[s], iso(0.0, -0.2, 0.0), iso(0.0, 0.2, 0.0), 0);
            world.set_d6_limit_r(knee, 0, Some(D6Limit { min: 0.0, max: 2.2 }));
            let ankle = world.add_hinge_joint(shank[s], foot[s], iso(0.0, -0.2, 0.0), iso(0.0, 0.05, -0.04), 0);
            world.set_d6_limit_r(ankle, 0, Some(D6Limit { min: -0.7, max: 0.7 }));
            joints.push(Joint { id: hip, parent: pelvis, child: thigh[s] });
            joints.push(Joint { id: knee, parent: thigh[s], child: shank[s] });
            joints.push(Joint { id: ankle, parent: shank[s], child: foot[s] });
        }

        let b = Biped {
            head, torso, pelvis, thigh, shank, foot,
            joints: joints.try_into().map_err(|_| ()).unwrap(),
            masses, last_act: [0.0; JOINTS], last_demo: None, fallen: false,
        };
        // Initial lean: rotate the whole body about the ankle line
        let lean = rng.range(-1.0, 1.0) * cfg.start_lean;
        if lean != 0.0 {
            let pivot = Vec3::new(0.0, 0.08, 0.0);
            let q = Quat::from_rotation_x(lean);
            for &(id, _) in &b.masses {
                let p = world.get_body_pose(id);
                world.set_body_pose(id, Isometry { pos: pivot + q * (p.pos - pivot), rot: q * p.rot });
            }
        }
        b
    }

    pub fn all_bodies(&self) -> impl Iterator<Item = BodyId> + '_ {
        self.masses.iter().map(|&(id, _)| id)
    }

    /// World axis of joint `j` (the parent's x axis).
    pub fn axis(&self, world: &World, j: usize) -> Vec3 {
        world.get_body_pose(self.joints[j].parent).rot * Vec3::X
    }

    /// Joint angle (rad) about its axis.
    pub fn angle(&self, world: &World, j: usize) -> f32 {
        world.get_joint_relative_rot(self.joints[j].id)
            .map(|q| { let q = if q.w < 0.0 { -q } else { q }; 2.0 * q.x.atan2(q.w) })
            .unwrap_or(0.0)
    }

    /// Joint rate (rad/s): child minus parent angular velocity along the axis.
    pub fn rate(&self, world: &World, j: usize) -> f32 {
        let a = self.axis(world, j);
        (world.get_body_vel(self.joints[j].child).ang - world.get_body_vel(self.joints[j].parent).ang).dot(a)
    }

    /// Applies joint torques (clamped) for one tick.
    pub fn apply(&mut self, world: &mut World, torques: [f32; JOINTS], dt: f32) {
        let t: [f32; JOINTS] = std::array::from_fn(|j| torques[j].clamp(-MAX_TORQUE[j], MAX_TORQUE[j]));
        self.last_act = t;
        for j in 0..JOINTS {
            let a = self.axis(world, j) * t[j] * dt;
            world.apply_torque(self.joints[j].child, a);
            world.apply_torque(self.joints[j].parent, -a);
        }
    }

    /// One control tick: `SUBSTEPS` physics steps, with torques from `servo` recomputed (and
    /// clamped) before each. Returns the torques of the last substep.
    pub fn tick(&mut self, world: &mut World, mut servo: impl FnMut(&World, &Biped) -> [f32; JOINTS]) -> [f32; JOINTS] {
        let dt = 1.0 / (CONTROL_HZ * SUBSTEPS) as f32;
        for _ in 0..SUBSTEPS {
            let t = servo(world, self);
            self.apply(world, t, dt);
            world.step(dt);
        }
        self.last_act
    }

    pub fn com(&self, world: &World) -> (Vec3, Vec3) {
        let (mut p, mut v, mut m) = (Vec3::ZERO, Vec3::ZERO, 0.0);
        for &(id, mass) in &self.masses {
            p += world.get_body_pose(id).pos * mass;
            v += world.get_body_vel(id).lin * mass;
            m += mass;
        }
        (p / m, v / m)
    }

    /// Torso pitch (rad, about x; positive leans the head toward -z) and its rate.
    pub fn torso_pitch(&self, world: &World) -> (f32, f32) {
        let up = world.get_body_pose(self.torso).rot * Vec3::Y;
        (up.z.atan2(up.y) * -1.0, world.get_body_vel(self.torso).ang.x)
    }

    pub fn is_fallen(&self, world: &World) -> bool {
        let up = world.get_body_pose(self.torso).rot * Vec3::Y;
        world.get_body_pose(self.pelvis).pos.y < 0.6 || up.y < 0.6
    }

    /// One entity: pelvis pose and velocities, plus joint angles/rates in `joints`.
    pub fn observe(&self, world: &World) -> EntState {
        let pose = world.get_body_pose(self.pelvis);
        let vel = world.get_body_vel(self.pelvis);
        EntState {
            eid: self.pelvis.0,
            kind: KIND.to_string(),
            pos: pose.pos.to_array(),
            rot: pose.rot.to_array(),
            vel: vel.lin.to_array(),
            ang: vel.ang.to_array(),
            target: None,
            act: self.last_act.to_vec(),
            demo: self.last_demo.map(|d| d.to_vec()),
            joints: Some((0..JOINTS).map(|j| [self.angle(world, j), self.rate(world, j)]).collect()),
        }
    }
}

/// PD stand controller: joint PD toward a slightly crouched pose, plus an ankle strategy
/// (move the centre of pressure to bring the centre of mass back over the feet) and a hip
/// strategy (keep the torso upright).
#[derive(Clone, Copy, Debug)]
pub struct StandPd {
    pub kp: [f32; 3],
    pub kd: [f32; 3],
    /// Ankle balance gains on centre-of-mass offset (m) and velocity (m/s), relative to the feet.
    pub k_com: f32,
    pub k_com_v: f32,
    /// Hip gains on torso pitch and pitch rate.
    pub k_torso: f32,
    pub k_torso_v: f32,
    /// Pose targets (hip, knee, ankle).
    pub pose: [f32; 3],
    /// Sign of the balance terms (set by tuning against the joint conventions).
    pub ankle_sign: f32,
    pub hip_sign: f32,
}

impl Default for StandPd {
    fn default() -> Self {
        Self {
            kp: [300.0, 300.0, 150.0], kd: [30.0, 30.0, 15.0],
            k_com: 800.0, k_com_v: 150.0, k_torso: 400.0, k_torso_v: 60.0,
            pose: [-0.1, 0.2, -0.1], ankle_sign: 1.0, hip_sign: -1.0,
        }
    }
}

impl StandPd {
    /// Joint PD toward the stance pose only, no balance strategies: the residual base. Falls
    /// within a couple of seconds; what it lacks is exactly what `StandPd` adds.
    pub fn joint_pd_only() -> Self {
        Self { k_com: 0.0, k_com_v: 0.0, k_torso: 0.0, k_torso_v: 0.0, ..Self::default() }
    }

    pub fn command(&self, world: &World, b: &Biped) -> [f32; JOINTS] {
        let (com, com_v) = b.com(world);
        let feet_z = 0.5 * (world.get_body_pose(b.foot[0]).pos.z + world.get_body_pose(b.foot[1]).pos.z);
        let (pitch, pitch_v) = b.torso_pitch(world);
        let ankle_bal = self.ankle_sign * (self.k_com * (com.z - feet_z) + self.k_com_v * com_v.z);
        let hip_bal = self.hip_sign * (self.k_torso * pitch + self.k_torso_v * pitch_v);
        let mut out = [0.0; JOINTS];
        for s in 0..2 {
            for k in 0..3 {
                let j = s * 3 + k;
                let pd = self.kp[k] * (self.pose[k] - b.angle(world, j)) - self.kd[k] * b.rate(world, j);
                out[j] = pd + match k { 0 => hip_bal, 2 => ankle_bal, _ => 0.0 };
            }
        }
        out
    }
}

/// Learner-facing balance features at the servo rate, in `SERVO_FEATURES` order: centre of mass
/// relative to the feet (fore-aft) and its velocity, torso pitch and rate, then per-leg hip, knee
/// and ankle angles (left then right).
pub const SERVO_FEATURES: [&str; 10] = ["com_dz", "com_vz", "pitch", "pitch_v",
    "hip_l", "knee_l", "ankle_l", "hip_r", "knee_r", "ankle_r"];

impl Biped {
    pub fn servo_obs(&self, world: &World) -> [f32; 10] {
        let (com, com_v) = self.com(world);
        let feet_z = 0.5 * (world.get_body_pose(self.foot[0]).pos.z + world.get_body_pose(self.foot[1]).pos.z);
        let (pitch, pitch_v) = self.torso_pitch(world);
        let mut o = [0.0; 10];
        o[..4].copy_from_slice(&[com.z - feet_z, com_v.z, pitch, pitch_v]);
        for j in 0..JOINTS { o[4 + j] = self.angle(world, j); }
        o
    }
}

/// One stand episode: a biped on flat ground, random pushes on the torso, ending when it falls
/// or time runs out. Same seeding scheme as the drone `DroneEnv`.
pub struct BipedEnv {
    pub world: World,
    pub biped: Biped,
    pub rng: Rng,
    pub cfg: BipedConfig,
    /// Control ticks taken / episode length
    pub tick: u32,
    pub ticks: u32,
    pub fallen: bool,
}

impl BipedEnv {
    pub fn new(cfg: &BipedConfig, seconds: f32, ep_index: u64, seed: u64) -> Self {
        let ep_seed = seed ^ ep_index.wrapping_mul(0xA24B_AED4_963E_E407);
        let mut rng = Rng::new(ep_seed);
        let mut world = World::with_capacity(32, 32);
        world.set_rng_seed(ep_seed);
        let biped = Biped::build(&mut world, cfg, &mut rng);
        Self { world, biped, rng, cfg: *cfg, tick: 0, ticks: (seconds * CONTROL_HZ as f32) as u32, fallen: false }
    }

    pub fn done(&self) -> bool { self.fallen || self.tick >= self.ticks }

    pub fn seconds(&self) -> f32 { self.tick as f32 / CONTROL_HZ as f32 }

    /// One control tick (maybe a push, then `SUBSTEPS` servo steps with torques from `servo`).
    pub fn step(&mut self, servo: impl FnMut(&World, &Biped) -> [f32; JOINTS]) {
        if self.rng.unit() < self.cfg.push_rate {
            let dir = Vec3::new(self.rng.range(-0.3, 0.3), 0.0, self.rng.range(-1.0, 1.0)).normalize_or_zero();
            self.world.apply_impulse(self.biped.torso, dir * self.cfg.push_impulse);
        }
        self.biped.tick(&mut self.world, servo);
        self.tick += 1;
        if self.biped.is_fallen(&self.world) { self.fallen = true; }
    }
}

/// Seconds the biped stays up under `ctl` (or `seconds` if it never falls).
pub fn stand_time(ctl: &StandPd, cfg: &BipedConfig, seed: u64, seconds: f32) -> f32 {
    let mut env = BipedEnv::new(cfg, seconds, 0, seed);
    while !env.done() { env.step(|w, b| ctl.command(w, b)); }
    env.seconds()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore]
    fn tune_stand_signs() {
        let cfg = BipedConfig { push_rate: 0.0, ..Default::default() };
        for ankle_sign in [1.0, -1.0] {
            for hip_sign in [1.0, -1.0] {
                let ctl = StandPd { ankle_sign, hip_sign, ..Default::default() };
                let times: Vec<f32> = (0..5).map(|s| stand_time(&ctl, &cfg, 100 + s, 10.0)).collect();
                println!("ankle {:+} hip {:+}: stand times {:?}", ankle_sign, hip_sign, times);
            }
        }
        let zero = StandPd { kp: [0.0; 3], kd: [0.0; 3], k_com: 0.0, k_com_v: 0.0, k_torso: 0.0, k_torso_v: 0.0, ..Default::default() };
        println!("no control: {:?}", (0..5).map(|s| stand_time(&zero, &cfg, 100 + s, 10.0)).collect::<Vec<_>>());
    }
}


#[cfg(test)]
mod trace {
    use super::*;

    #[test]
    #[ignore]
    fn trace_stand_pd() {
        let cfg = BipedConfig { push_rate: 0.0, ..Default::default() };
        let ctl = StandPd::default();
        let mut rng = Rng::new(100);
        let mut world = World::with_capacity(32, 32);
        let mut b = Biped::build(&mut world, &cfg, &mut rng);
        for t in 0..240 {
            b.tick(&mut world, |w, b| ctl.command(w, b));
            if t % 10 == 0 {
                let (com, comv) = b.com(&world);
                let fz = 0.5 * (world.get_body_pose(b.foot[0]).pos.z + world.get_body_pose(b.foot[1]).pos.z);
                let (pitch, _) = b.torso_pitch(&world);
                let ang: Vec<String> = (0..3).map(|j| format!("{:+.2}", b.angle(&world, j))).collect();
                let tq: Vec<String> = (0..3).map(|j| format!("{:+.0}", b.last_act[j])).collect();
                let fl = world.get_body_pose(b.foot[0]); let fv = world.get_body_vel(b.foot[0]);
                println!("FOOT t{:>3} foot z {:+.4} y {:+.4} vz {:+.4} vy {:+.4} w {:+.3?} | com z {:+.4}", t, fl.pos.z, fl.pos.y, fv.lin.z, fv.lin.y, fv.ang, com.z);
                println!("t{:>3} pelvis y {:.3} com-feet z {:+.3} comv z {:+.2} pitch {:+.2} | L ang {} | L tq {} | foot y {:.3} up.y {:.2}",
                    t, world.get_body_pose(b.pelvis).pos.y, com.z - fz, comv.z, pitch, ang.join(" "), tq.join(" "),
                    world.get_body_pose(b.foot[0]).pos.y, (world.get_body_pose(b.foot[0]).rot * Vec3::Y).y);
            }
            if b.is_fallen(&world) { println!("fallen at t{}", t); break; }
        }
    }

    #[test]
    #[ignore]
    fn base_alone_falls() {
        let cfg = BipedConfig::default();
        let times: Vec<f32> = (0..10).map(|s| stand_time(&StandPd::joint_pd_only(), &cfg, 1000 + s, 10.0)).collect();
        println!("joint PD only: {:?}", times);
    }

    #[test]
    #[ignore]
    fn push_recovery_sweep() {
        for impulse in [0.0f32, 6.0, 12.0, 20.0, 30.0] {
            let cfg = BipedConfig { push_impulse: impulse, ..Default::default() };
            let times: Vec<f32> = (0..20).map(|s| stand_time(&StandPd::default(), &cfg, 1000 + s, 10.0)).collect();
            let ok = times.iter().filter(|t| **t >= 10.0).count();
            println!("push {:>4.0} N s: {}/20 stood 10 s, min {:.2} s", impulse, ok, times.iter().cloned().fold(f32::MAX, f32::min));
        }
    }

    #[test]
    #[ignore]
    fn profile_steps() {
        let cfg = BipedConfig { push_rate: 0.0, ..Default::default() };
        let ctl = StandPd::default();
        let mut rng = Rng::new(100);
        let mut world = World::with_capacity(32, 32);
        let mut b = Biped::build(&mut world, &cfg, &mut rng);
        let ticks = 600;
        let t0 = std::time::Instant::now();
        for _ in 0..ticks { b.tick(&mut world, |w, b| ctl.command(w, b)); }
        let el = t0.elapsed().as_secs_f64();
        println!("{} physics steps in {:.3} s: {:.1} us/step ({:.1}x real time)", ticks * SUBSTEPS, el,
            el * 1e6 / (ticks * SUBSTEPS) as f64, (ticks as f64 / CONTROL_HZ as f64) / el);
    }

    #[test]
    #[ignore]
    fn profile_joint_pass() {
        let cfg = BipedConfig { push_rate: 0.0, ..Default::default() };
        let mut rng = Rng::new(100);
        let mut world = World::with_capacity(32, 32);
        let _b = Biped::build(&mut world, &cfg, &mut rng);
        let n = 20000;
        let t0 = std::time::Instant::now();
        for _ in 0..n { world.solve_joints(1.0 / 240.0, 1); }
        println!("joint pass (8 D6): {:.2} us", t0.elapsed().as_secs_f64() * 1e6 / n as f64);
    }

    #[test]
    #[ignore]
    fn iteration_sweep() {
        for iters in [(20u32, 20u32), (10, 10), (8, 6), (6, 4), (4, 4), (4, 2)] {
            let t0 = std::time::Instant::now();
            let mut row = Vec::new();
            for impulse in [12.0f32, 20.0] {
                let cfg = BipedConfig { push_impulse: impulse, solver_iters: iters, ..Default::default() };
                let ok = (0..20).filter(|s| stand_time(&StandPd::default(), &cfg, 1000 + s, 10.0) >= 10.0).count();
                row.push(format!("{} N s: {}/20", impulse, ok));
            }
            // passive sink
            let cfg = BipedConfig { push_rate: 0.0, start_lean: 0.0, solver_iters: iters, ..Default::default() };
            let mut rng = Rng::new(5);
            let mut world = World::with_capacity(32, 32);
            let mut b = Biped::build(&mut world, &cfg, &mut rng);
            let mut min_y = f32::MAX;
            for _ in 0..120 { b.tick(&mut world, |_, _| [0.0; JOINTS]); min_y = min_y.min(world.get_body_pose(b.foot[0]).pos.y); }
            let steps = 2.0 * 20.0 * 600.0 * SUBSTEPS as f64 + 120.0 * SUBSTEPS as f64;
            println!("iters {:?}: {} | passive foot min y {:+.3} | {:.1} us/step", iters, row.join(", "), min_y, t0.elapsed().as_secs_f64() * 1e6 / steps);
        }
    }

    /// Passive (zero torque) collapse: feet must stay on the ground, not sink into it.
    #[test]
    #[ignore]
    fn passive_feet_stay_on_ground() {
        let cfg = BipedConfig { push_rate: 0.0, start_lean: 0.0, ..Default::default() };
        let mut rng = Rng::new(5);
        let mut world = World::with_capacity(32, 32);
        let mut b = Biped::build(&mut world, &cfg, &mut rng);
        let mut min_y = f32::MAX;
        for _ in 0..120 {
            b.tick(&mut world, |_, _| [0.0; JOINTS]);
            min_y = min_y.min(world.get_body_pose(b.foot[0]).pos.y);
        }
        println!("lowest foot centre y {:+.3} (rest 0.030)", min_y);
    }
}
