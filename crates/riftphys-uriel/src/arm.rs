//! Six-joint welding arm with continuous (unlimited) joints, for the fabrication rungs.
//!
//! Layout (fixed base, y up; at the zero pose the arm reaches along +x): `base_yaw` about y on
//! a static column, `shoulder` and `elbow` about z (upper arm up, forearm forward), then a wrist:
//! `forearm_roll` about the forearm, `wrist_pitch` across it, `tool_roll` about the tool. Six
//! joints because the task needs six: the tool-centre-point (TCP) position, the torch direction
//! and the roll of a line sensor that must stay across the seam. Every joint turns freely through
//! full turns (the Designer's ruling, 2026-11-03); angles are read wrapped to (-pi, pi].
//!
//! Built like `biped`: maximal-coordinate D6 hinges, actuator armature about each joint axis,
//! joint torques applied equal and opposite, 60 Hz control with 4 servo substeps (240 Hz).
//! The `CartesianServo` (damped-least-squares resolved rate on top of a joint PD with gravity
//! compensation) is the inner loop shared by the seam teacher and the nominal-path floor.

use crate::protocol::{ActuatorSpec, EntState, KindSpec};
use riftphys_core::{BodyId, Isometry, JointId, Mat3, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::step::MELEE_CCD_MARGIN;
use riftphys_world::world::world::TERRAIN_CI;
use riftphys_world::world::World;
use std::f32::consts::PI;

pub const KIND: &str = "arm";
pub const JOINTS: usize = 6;
/// Links: 0 is the static base column, link j + 1 is the child of joint j (6 is the tool).
pub const LINKS: usize = JOINTS + 1;
pub const G: f32 = 9.81;
pub const JOINT_NAMES: [&str; JOINTS] = ["base_yaw", "shoulder", "elbow", "forearm_roll", "wrist_pitch", "tool_roll"];
pub const LINK_NAMES: [&str; LINKS] = ["base", "turret", "upper_arm", "forearm", "wrist_1", "wrist_2", "tool"];
/// Torque limits (N m), joint order.
pub const MAX_TORQUE: [f32; JOINTS] = [120.0, 150.0, 80.0, 20.0, 20.0, 10.0];
/// Reflected actuator inertia about each joint axis (kg m^2), added to the child link.
pub const ARMATURE: [f32; JOINTS] = [0.10, 0.10, 0.05, 0.02, 0.02, 0.01];
pub const CONTROL_HZ: u32 = 60;
pub const SUBSTEPS: u32 = 4;
/// TCP in the tool link's frame: the wire tip, 18 mm past the end of the 16 mm nozzle (the tool
/// collider) so the nozzle clears both plates of a 45 degree fillet at 2 mm standoff. The tool's local axes: +x travel ("fwd"), +y along the torch
/// toward the work, +z = x cross y (across the seam).
pub const TCP_LOCAL: Vec3 = Vec3::new(0.0, 0.084, 0.0);
/// Contacts closer than this (m) count as touching.
pub const TOUCH: f32 = 0.001;

/// One link at the zero pose, relative to the base point: centre, collider, mass, and whether
/// its long (local y) axis points along world x instead of y.
struct LinkDef { centre: [f32; 3], shape: Shape, mass: f32, along_x: bool }

const LINK_DEFS: [LinkDef; LINKS] = [
    LinkDef { centre: [0.0, 0.15, 0.0], shape: Shape::Cylinder { r: 0.08, hh: 0.15 }, mass: 0.0, along_x: false },
    LinkDef { centre: [0.0, 0.36, 0.0], shape: Shape::Cylinder { r: 0.07, hh: 0.06 }, mass: 4.0, along_x: false },
    LinkDef { centre: [0.0, 0.62, 0.0], shape: Shape::Capsule { r: 0.05, hh: 0.13 }, mass: 3.5, along_x: false },
    LinkDef { centre: [0.125, 0.82, 0.0], shape: Shape::Capsule { r: 0.04, hh: 0.08 }, mass: 1.6, along_x: true },
    LinkDef { centre: [0.30, 0.82, 0.0], shape: Shape::Capsule { r: 0.035, hh: 0.01 }, mass: 0.6, along_x: true },
    LinkDef { centre: [0.38, 0.82, 0.0], shape: Shape::Sphere { r: 0.03 }, mass: 0.6, along_x: true },
    LinkDef { centre: [0.476, 0.82, 0.0], shape: Shape::Capsule { r: 0.008, hh: 0.058 }, mass: 0.6, along_x: true },
];
/// Joint anchors and axes at the zero pose, relative to the base point.
const JOINT_ANCHOR: [[f32; 3]; JOINTS] = [[0.0, 0.30, 0.0], [0.0, 0.42, 0.0], [0.0, 0.82, 0.0], [0.25, 0.82, 0.0], [0.35, 0.82, 0.0], [0.41, 0.82, 0.0]];
const JOINT_AXIS: [[f32; 3]; JOINTS] = [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]];

pub fn kind_spec() -> KindSpec {
    KindSpec {
        kind: KIND.to_string(),
        actuators: JOINT_NAMES.iter().zip(MAX_TORQUE).map(|(n, m)| ActuatorSpec { name: n.to_string(), min: -m, max: m, unit: "N m".to_string() }).collect(),
        sensors: None,
    }
}

/// Angle wrapped to (-pi, pi].
pub fn wrap(a: f32) -> f32 {
    let mut x = (a + PI).rem_euclid(2.0 * PI) - PI;
    if x <= -PI { x += 2.0 * PI; }
    x
}

fn link_rot(def: &LinkDef) -> Quat {
    if def.along_x { Quat::from_rotation_z(-PI / 2.0) } else { Quat::IDENTITY }
}

/// Zero-pose link poses relative to the base point.
fn zero_pose(i: usize) -> Isometry {
    Isometry { pos: Vec3::from_array(LINK_DEFS[i].centre), rot: link_rot(&LINK_DEFS[i]) }
}

/// Forward kinematics by product of exponentials: link poses (relative to the base point) for
/// joint angles `q`. Matches the built arm exactly at build time; the world is the truth after.
pub fn fk(q: &[f32; JOINTS]) -> [Isometry; LINKS] {
    let mut out = [zero_pose(0); LINKS];
    let (mut r, mut t) = (Quat::IDENTITY, Vec3::ZERO);
    for i in 1..LINKS {
        let j = i - 1;
        let a = Vec3::from_array(JOINT_AXIS[j]);
        let p = Vec3::from_array(JOINT_ANCHOR[j]);
        // Accumulated motion: (r, t) followed by a turn q_j about the zero-pose axis through p
        let rj = Quat::from_axis_angle(a, q[j]);
        let tj = p - rj * p;
        t = r * tj + t;
        r = r * rj;
        let z = zero_pose(i);
        out[i] = Isometry { pos: r * z.pos + t, rot: (r * z.rot).normalize() };
    }
    out
}

/// TCP pose (relative to the base point) for joint angles `q`.
pub fn fk_tcp(q: &[f32; JOINTS]) -> Isometry {
    let tool = fk(q)[LINKS - 1];
    Isometry { pos: tool.pos + tool.rot * TCP_LOCAL, rot: tool.rot }
}

/// Rotation vector (axis * angle) taking `from` to `to`, both world rotations.
pub fn rot_error(to: Quat, from: Quat) -> Vec3 {
    let mut d = to * from.conjugate();
    if d.w < 0.0 { d = -d; }
    let v = Vec3::new(d.x, d.y, d.z);
    let s = v.length();
    if s < 1.0e-7 { return v * 2.0; }
    v * (2.0 * s.atan2(d.w) / s)
}

/// Joint angles that put the TCP at `target` (relative to the base point), by damped least
/// squares on the kinematic model from `q0`. Deterministic; used for the build pose.
pub fn ik(target: &Isometry, q0: [f32; JOINTS], iters: u32) -> [f32; JOINTS] {
    let mut q = q0;
    for _ in 0..iters {
        let cur = fk_tcp(&q);
        let mut e = [0.0f32; 6];
        e[..3].copy_from_slice(&(target.pos - cur.pos).to_array());
        e[3..].copy_from_slice(&(rot_error(target.rot, cur.rot) * ORI_WEIGHT).to_array());
        let mut jac = [[0.0f32; 6]; JOINTS];
        for k in 0..JOINTS {
            let mut qk = q;
            qk[k] += 1.0e-3;
            let p = fk_tcp(&qk);
            let dp = (p.pos - cur.pos) / 1.0e-3;
            let dr = rot_error(p.rot, cur.rot) * (ORI_WEIGHT / 1.0e-3);
            jac[k] = [dp.x, dp.y, dp.z, dr.x, dr.y, dr.z];
        }
        let dq = dls(&jac, &e, 0.01);
        for k in 0..JOINTS { q[k] = wrap(q[k] + dq[k].clamp(-0.3, 0.3)); }
    }
    q
}

/// Orientation rows of the task are scaled by this length (m) so a radian of tool rotation
/// weighs like 0.3 m of travel in the least-squares solve.
pub const ORI_WEIGHT: f32 = 0.3;

/// Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 e, `jac[k]` the column of joint k.
pub fn dls(jac: &[[f32; 6]; JOINTS], e: &[f32; 6], lambda: f32) -> [f32; JOINTS] {
    let mut a = [[0.0f32; 7]; 6];
    for r in 0..6 {
        for c in 0..6 {
            a[r][c] = (0..JOINTS).map(|k| jac[k][r] * jac[k][c]).sum::<f32>();
        }
        a[r][r] += lambda * lambda;
        a[r][6] = e[r];
    }
    // Gaussian elimination, partial pivoting (fixed order: deterministic)
    for col in 0..6 {
        let piv = (col..6).fold(col, |b, r| if a[r][col].abs() > a[b][col].abs() { r } else { b });
        a.swap(col, piv);
        let d = a[col][col];
        if d.abs() < 1.0e-20 { continue; }
        for r in (col + 1)..6 {
            let f = a[r][col] / d;
            for c in col..7 { a[r][c] -= f * a[col][c]; }
        }
    }
    let mut y = [0.0f32; 6];
    for r in (0..6).rev() {
        let s = a[r][6] - ((r + 1)..6).map(|c| a[r][c] * y[c]).sum::<f32>();
        y[r] = if a[r][r].abs() < 1.0e-20 { 0.0 } else { s / a[r][r] };
    }
    std::array::from_fn(|k| (0..6).map(|r| jac[k][r] * y[r]).sum())
}

#[derive(Clone, Copy, Debug)]
pub struct Joint {
    pub id: JointId,
    pub parent: BodyId,
    pub child: BodyId,
    /// Joint frame on the parent (x = axis), for the world axis and anchor.
    pub frame: Isometry,
}

/// What an arm link is touching this step.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ArmContacts {
    /// Two non-adjacent arm links in contact (link indices).
    pub self_pair: Option<(usize, usize)>,
    /// An arm link touching anything else (the workpiece, the table).
    pub env_link: Option<usize>,
}

#[derive(Clone, Copy, Debug)]
pub struct ArmConfig {
    /// World solver iterations (velocity, position) per substep, as the biped.
    pub solver_iters: (u32, u32),
}

impl Default for ArmConfig {
    fn default() -> Self { Self { solver_iters: (8, 6) } }
}

pub struct Arm {
    pub base: Vec3,
    pub links: [BodyId; LINKS],
    pub masses: [f32; LINKS],
    pub joints: [Joint; JOINTS],
    pub last_act: [f32; JOINTS],
    pub last_demo: Option<[f32; JOINTS]>,
}

impl Arm {
    /// Builds the arm with its base point at `base` (the column stands on it) and joint angles
    /// `q`. Joint frames are defined at the zero pose, so angles read 0 there and `q` here.
    pub fn build(world: &mut World, cfg: &ArmConfig, base: Vec3, q: [f32; JOINTS]) -> Self {
        world.set_gravity(Vec3::new(0.0, -G, 0.0));
        world.set_solver_iterations(cfg.solver_iters.0, cfg.solver_iters.1);
        let zero = fk(&[0.0; JOINTS]);
        let posed = fk(&q);
        let mut links = [BodyId(0); LINKS];
        let mut masses = [0.0; LINKS];
        for i in 0..LINKS {
            let def = &LINK_DEFS[i];
            let pose = Isometry { pos: base + posed[i].pos, rot: posed[i].rot };
            let mut mp = if i == 0 { MassProps::infinite() } else { link_mass(&def.shape, def.mass) };
            if i > 0 {
                // Armature: the drive's reflected inertia about the joint axis plus the housing's
                // inertia about the others, added isotropically. Armature about the joint axis
                // alone left the light wrist links nearly massless crosswise, and the chain
                // ill-conditioned: wrist torques leaked into the neighbouring joints.
                mp.inertia += Mat3::IDENTITY * ARMATURE[i - 1];
            }
            let id = world.add_body(pose, Velocity::default(), mp, i > 0);
            world.add_collider(id, def.shape, material(MaterialId::Aluminum));
            world.set_body_sleep_allowed(id, false);
            links[i] = id;
            masses[i] = def.mass;
        }
        let joints: [Joint; JOINTS] = std::array::from_fn(|j| {
            let (pz, cz) = (zero[j], zero[j + 1]);
            let anchor = Vec3::from_array(JOINT_ANCHOR[j]);
            let axis_rot = Quat::from_rotation_arc(Vec3::X, Vec3::from_array(JOINT_AXIS[j]));
            let fa = Isometry { pos: pz.rot.conjugate() * (anchor - pz.pos), rot: (pz.rot.conjugate() * axis_rot).normalize() };
            let fb = Isometry { pos: cz.rot.conjugate() * (anchor - cz.pos), rot: (cz.rot.conjugate() * axis_rot).normalize() };
            let id = world.add_hinge_joint(links[j], links[j + 1], fa, fb, 0);
            Joint { id, parent: links[j], child: links[j + 1], frame: fa }
        });
        Arm { base, links, masses, joints, last_act: [0.0; JOINTS], last_demo: None }
    }

    pub fn link_index(&self, body: BodyId) -> Option<usize> {
        self.links.iter().position(|&b| b == body)
    }

    /// World axis of joint `j`.
    pub fn axis(&self, world: &World, j: usize) -> Vec3 {
        let p = world.get_body_pose(self.joints[j].parent);
        p.rot * (self.joints[j].frame.rot * Vec3::X)
    }

    /// World anchor (a point on the axis) of joint `j`.
    pub fn anchor(&self, world: &World, j: usize) -> Vec3 {
        let p = world.get_body_pose(self.joints[j].parent);
        p.pos + p.rot * self.joints[j].frame.pos
    }

    /// Joint angle, wrapped to (-pi, pi].
    pub fn angle(&self, world: &World, j: usize) -> f32 {
        world.get_joint_relative_rot(self.joints[j].id)
            .map(|q| { let q = if q.w < 0.0 { -q } else { q }; wrap(2.0 * q.x.atan2(q.w)) })
            .unwrap_or(0.0)
    }

    pub fn angles(&self, world: &World) -> [f32; JOINTS] {
        std::array::from_fn(|j| self.angle(world, j))
    }

    /// Joint rate (rad/s): child minus parent angular velocity along the axis.
    pub fn rate(&self, world: &World, j: usize) -> f32 {
        let a = self.axis(world, j);
        (world.get_body_vel(self.joints[j].child).ang - world.get_body_vel(self.joints[j].parent).ang).dot(a)
    }

    pub fn rates(&self, world: &World) -> [f32; JOINTS] {
        std::array::from_fn(|j| self.rate(world, j))
    }

    pub fn tool(&self) -> BodyId { self.links[LINKS - 1] }

    /// TCP pose in the world: position of the torch tip, rotation of the tool link.
    pub fn tcp(&self, world: &World) -> Isometry {
        let t = world.get_body_pose(self.tool());
        Isometry { pos: t.pos + t.rot * TCP_LOCAL, rot: t.rot }
    }

    /// TCP linear and angular velocity.
    pub fn tcp_vel(&self, world: &World) -> (Vec3, Vec3) {
        let t = world.get_body_pose(self.tool());
        let v = world.get_body_vel(self.tool());
        (v.lin + v.ang.cross(t.rot * TCP_LOCAL), v.ang)
    }

    /// Geometric Jacobian at the TCP from the world state: column j = (a x (p_tcp - p_j), a),
    /// orientation rows scaled by `ORI_WEIGHT`.
    pub fn jacobian(&self, world: &World) -> [[f32; 6]; JOINTS] {
        let p = self.tcp(world).pos;
        std::array::from_fn(|j| {
            let a = self.axis(world, j);
            let l = a.cross(p - self.anchor(world, j));
            let w = a * ORI_WEIGHT;
            [l.x, l.y, l.z, w.x, w.y, w.z]
        })
    }

    /// Joint torques that hold the arm against gravity in its current pose.
    pub fn gravity_torques(&self, world: &World) -> [f32; JOINTS] {
        let g = Vec3::new(0.0, -G, 0.0);
        std::array::from_fn(|j| {
            let (a, p) = (self.axis(world, j), self.anchor(world, j));
            -(j + 1..LINKS).map(|i| a.dot((world.get_body_pose(self.links[i]).pos - p).cross(g * self.masses[i]))).sum::<f32>()
        })
    }

    /// Applies joint torques (clamped) for one substep.
    pub fn apply(&mut self, world: &mut World, torques: [f32; JOINTS], dt: f32) {
        let t: [f32; JOINTS] = std::array::from_fn(|j| torques[j].clamp(-MAX_TORQUE[j], MAX_TORQUE[j]));
        self.last_act = t;
        for j in 0..JOINTS {
            let a = self.axis(world, j) * t[j] * dt;
            world.apply_torque(self.joints[j].child, a);
            world.apply_torque(self.joints[j].parent, -a);
        }
    }

    /// One control tick: `SUBSTEPS` physics steps, torques from `servo` recomputed before each.
    pub fn tick(&mut self, world: &mut World, mut servo: impl FnMut(&World, &Arm, f32) -> [f32; JOINTS]) -> [f32; JOINTS] {
        let dt = 1.0 / (CONTROL_HZ * SUBSTEPS) as f32;
        for _ in 0..SUBSTEPS {
            let t = servo(world, self, dt);
            self.apply(world, t, dt);
            world.step(dt);
        }
        self.last_act
    }

    /// Contacts from the last step that actually touch (within `TOUCH`; speculative contacts
    /// further apart are ignored). Links joined by a joint never collide (engine rule).
    pub fn contacts(&self, world: &World) -> ArmContacts {
        let mut out = ArmContacts::default();
        let link_of = |ci: usize| -> Option<usize> {
            if ci == TERRAIN_CI { None } else { world.colliders.get(ci).and_then(|c| self.link_index(c.body)) }
        };
        for c in &world.last_frame_contacts {
            if c.depth - MELEE_CCD_MARGIN < -TOUCH { continue; }
            match (link_of(c.a_collider), link_of(c.b_collider)) {
                (Some(a), Some(b)) => {
                    if out.self_pair.is_none() { out.self_pair = Some((a.min(b), a.max(b))); }
                }
                (Some(l), None) | (None, Some(l)) => {
                    if l > 0 && out.env_link.is_none() { out.env_link = Some(l); }
                }
                _ => {}
            }
        }
        out
    }

    /// One entity: the TCP frame and velocities, joint angles (wrapped) and rates.
    pub fn observe(&self, world: &World) -> EntState {
        let tcp = self.tcp(world);
        let (v, w) = self.tcp_vel(world);
        EntState {
            eid: self.tool().0,
            kind: KIND.to_string(),
            pos: tcp.pos.to_array(),
            rot: tcp.rot.to_array(),
            vel: v.to_array(),
            ang: w.to_array(),
            target: None,
            act: self.last_act.to_vec(),
            demo: self.last_demo.map(|d| d.to_vec()),
            joints: Some((0..JOINTS).map(|j| [self.angle(world, j), self.rate(world, j)]).collect()),
            scan: None,
            err: None,
        }
    }
}

fn link_mass(shape: &Shape, mass: f32) -> MassProps {
    match *shape {
        Shape::Capsule { r, hh } => {
            let vol = PI * r * r * 2.0 * hh + 4.0 / 3.0 * PI * r * r * r;
            MassProps::from_capsule_density(r, hh, mass / vol)
        }
        Shape::Cylinder { r, hh } => MassProps::from_cylinder_density(r, hh, mass / (PI * r * r * 2.0 * hh)),
        Shape::Sphere { r } => MassProps::from_sphere_density(r, mass / (4.0 / 3.0 * PI * r * r * r)),
        _ => unreachable!("arm links are capsules, cylinders or spheres"),
    }
}

/// Where the TCP should be: a pose and its feed-forward velocity.
#[derive(Clone, Copy, Debug)]
pub struct TcpTarget {
    pub pose: Isometry,
    pub vel: Vec3,
}

/// Damped-least-squares resolved-rate servo on a joint PD with gravity compensation. Keeps an
/// unwrapped joint reference as its own state and compares it to the (wrapped) measured angles
/// through the shortest wrapped difference, so crossing +-pi is never a jump. Resolved rate moves
/// from the current configuration, so it never chooses between IK branches.
#[derive(Clone, Debug)]
pub struct CartesianServo {
    pub q_ref: [f32; JOINTS],
    pub kp: [f32; JOINTS],
    pub kd: [f32; JOINTS],
    /// Task-space gain (1/s) on position and orientation error.
    pub k_task: f32,
    pub lambda: f32,
    /// Joint reference speed limit (rad/s).
    pub max_rate: f32,
}

impl CartesianServo {
    pub fn new(arm: &Arm, world: &World) -> Self {
        Self {
            q_ref: arm.angles(world),
            kp: [500.0, 800.0, 400.0, 40.0, 40.0, 20.0],
            kd: [40.0, 60.0, 30.0, 2.5, 2.5, 1.2],
            k_task: 8.0,
            lambda: 0.02,
            max_rate: 2.0,
        }
    }

    /// Joint torques for one substep toward `target`. Advances the joint reference by `dt`.
    pub fn torques(&mut self, world: &World, arm: &Arm, target: &TcpTarget, dt: f32) -> [f32; JOINTS] {
        let tcp = arm.tcp(world);
        let ep = target.pose.pos - tcp.pos;
        let eo = rot_error(target.pose.rot, tcp.rot);
        let v = target.vel + ep * self.k_task;
        let w = eo * (self.k_task * ORI_WEIGHT);
        let qd = dls(&arm.jacobian(world), &[v.x, v.y, v.z, w.x, w.y, w.z], self.lambda);
        let q = arm.angles(world);
        let rates = arm.rates(world);
        let grav = arm.gravity_torques(world);
        std::array::from_fn(|j| {
            let qdj = qd[j].clamp(-self.max_rate, self.max_rate);
            // Keep the reference within reach of the joint (no wind-up while a torque saturates)
            let lead = wrap(self.q_ref[j] + qdj * dt - q[j]).clamp(-0.2, 0.2);
            self.q_ref[j] = q[j] + lead;
            self.kp[j] * lead + self.kd[j] * (qdj - rates[j]) + grav[j]
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_stays_in_half_open_range() {
        for a in [-7.0f32, -PI, -3.0, 0.0, 3.0, PI, 7.0, 100.0] {
            let w = wrap(a);
            assert!(w > -PI - 1e-6 && w <= PI + 1e-6, "{} -> {}", a, w);
            assert!(((a - w) / (2.0 * PI)).fract().abs() < 1e-4 || ((a - w) / (2.0 * PI)).fract().abs() > 1.0 - 1e-4);
        }
    }

    #[test]
    fn built_angles_read_back() {
        let q = [0.4, -0.9, 1.2, -2.5, 1.0, 3.0];
        let mut world = World::with_capacity(16, 16);
        let arm = Arm::build(&mut world, &ArmConfig::default(), Vec3::ZERO, q);
        for j in 0..JOINTS {
            assert!((arm.angle(&world, j) - q[j]).abs() < 1e-4, "joint {}: {} vs {}", j, arm.angle(&world, j), q[j]);
        }
        let tcp = arm.tcp(&world);
        let want = fk_tcp(&q);
        assert!((tcp.pos - want.pos).length() < 1e-5);
    }

    #[test]
    fn ik_reaches_a_welding_pose() {
        // Torch straight down (tool y = -world y), travel along +z, 0.45 m in front at plate height
        let target = Isometry { pos: Vec3::new(0.45, 0.002, 0.0), rot: Quat::from_mat3(&glam::Mat3::from_cols(Vec3::Z, -Vec3::Y, Vec3::X)) };
        let q = ik(&target, [0.0, -0.6, -0.6, 0.0, -1.0, 0.0], 200);
        let got = fk_tcp(&q);
        assert!((got.pos - target.pos).length() < 1e-4, "pos {:?}", got.pos);
        assert!(rot_error(target.rot, got.rot).length() < 1e-3);
    }

    fn hold(q: [f32; JOINTS], ticks: u32, servo: bool, gravity: bool) -> f32 {
        let mut world = World::with_capacity(16, 16);
        let mut arm = Arm::build(&mut world, &ArmConfig::default(), Vec3::ZERO, q);
        let start = arm.tcp(&world);
        let mut s = CartesianServo::new(&arm, &world);
        let target = TcpTarget { pose: start, vel: Vec3::ZERO };
        for _ in 0..ticks {
            arm.tick(&mut world, |w, a, dt| {
                if servo { s.torques(w, a, &target, dt) }
                else if gravity { a.gravity_torques(w) }
                else { [0.0; JOINTS] }
            });
        }
        (arm.tcp(&world).pos - start.pos).length()
    }

    /// Gravity compensation is right (the arm barely moves without feedback), and the servo
    /// holds a welding pose to a fraction of a millimetre.
    #[test]
    fn gravity_compensation_and_servo_hold_the_pose() {
        let q = [0.0, -0.5, -0.8, 0.0, -0.9, 0.0];
        let (limp, comp) = (hold(q, 60, false, false), hold(q, 60, false, true));
        assert!(comp < limp / 20.0, "limp {:.3} m, compensated {:.3} m", limp, comp);
        let held = hold(q, 120, true, true);
        assert!(held < 0.0005, "servo drift {:.5} m", held);
    }

    /// Continuous joints: base yaw, forearm roll and tool roll each make two full turns under a
    /// joint PD on an unwrapped reference. The wrapped readings never jump (consecutive ticks
    /// differ by the motion only), their unwrapped sum is the commanded two turns, and the hinge
    /// keeps its other axes locked through +-pi.
    #[test]
    fn joints_turn_through_full_turns_without_jumps() {
        let q0 = [0.0, -0.5, -0.8, 0.0, -0.9, 0.0];
        let mut world = World::with_capacity(16, 16);
        let mut arm = Arm::build(&mut world, &ArmConfig::default(), Vec3::ZERO, q0);
        let spin = [0usize, 3, 5];
        let turns = 2.0 * 2.0 * PI;
        let ticks = 8 * CONTROL_HZ;
        let mut q_ref = q0;
        let mut prev = arm.angles(&world);
        let mut travelled = [0.0f32; JOINTS];
        let kp = [500.0, 800.0, 400.0, 40.0, 40.0, 20.0];
        let kd = [40.0, 60.0, 30.0, 2.5, 2.5, 1.2];
        let rate = turns / (ticks as f32 / CONTROL_HZ as f32 - 1.0);
        for t in 0..ticks {
            // Spin for all but the last second, then hold
            let moving = t < ticks - CONTROL_HZ;
            arm.tick(&mut world, |w, a, dt| {
                for &j in &spin { if moving { q_ref[j] += rate * dt; } }
                let (q, qd, g) = (a.angles(w), a.rates(w), a.gravity_torques(w));
                std::array::from_fn(|j| {
                    let ff = if moving && spin.contains(&j) { rate } else { 0.0 };
                    kp[j] * wrap(q_ref[j] - q[j]) + kd[j] * (ff - qd[j]) + g[j]
                })
            });
            let now = arm.angles(&world);
            for j in 0..JOINTS {
                let step = wrap(now[j] - prev[j]);
                assert!(step.abs() < 0.2, "joint {} jumped {:.3} rad at tick {}", j, step, t);
                travelled[j] += step;
            }
            prev = now;
        }
        for &j in &spin {
            assert!((travelled[j] - turns).abs() < 0.05, "joint {} turned {:.3} rad, want {:.3}", JOINT_NAMES[j], travelled[j], turns);
        }
        // Joint frames still coincide (hinge axes stayed locked): TCP matches the kinematic model
        let fk_pos = fk_tcp(&arm.angles(&world)).pos;
        assert!((arm.tcp(&world).pos - fk_pos).length() < 0.003, "joint separation {:.4} m", (arm.tcp(&world).pos - fk_pos).length());
    }
}
