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
/// Reflected actuator inertia about each joint axis (kg m^2): added to the child link
/// (`ArmArmature::Isotropic`), or carried by the joint's rotor row (`ArmArmature::Axis`).
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

/// How the drives' reflected inertia (`ARMATURE`) enters the arm (PARITY_SPEC 3.9, decision D6).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ArmArmature {
    /// Added to each child link's inertia about every axis (the arm URIEL trains on; default).
    /// About the joint axis it resists the child's absolute rotation, so a parent joint's motion
    /// drags it.
    #[default]
    Isotropic,
    /// The geared-motor model: about the joint axis, the joint's rotor row (PARITY_SPEC W4b)
    /// resists the joint's relative rotation; crosswise, the child keeps the same added inertia
    /// as `Isotropic`, so the light wrist links stay conditioned. The total about the axis is
    /// unchanged.
    Axis,
}

#[derive(Clone, Copy, Debug)]
pub struct ArmConfig {
    /// World solver iterations (velocity, position) per substep, as the biped.
    pub solver_iters: (u32, u32),
    pub armature: ArmArmature,
}

impl Default for ArmConfig {
    fn default() -> Self { Self { solver_iters: (8, 6), armature: ArmArmature::Isotropic } }
}

impl ArmConfig {
    /// Each armature mode's own config. `Axis` solves 96 velocity iterations, not 8: a soft
    /// rotor row beside the stiff hinge rows converges slowly. At 8, a seam teacher's first
    /// second replayed open loop strayed 25-30% from the converged motion; at 96, 0.6-1.1%
    /// (`Isotropic` at 8: 0.9-1.8%; PARITY_SPEC 3.9). Ordering the rotor rows first or last
    /// didn't help.
    pub fn for_armature(armature: ArmArmature) -> Self {
        match armature {
            ArmArmature::Isotropic => Self::default(),
            ArmArmature::Axis => Self { solver_iters: (96, 6), armature },
        }
    }
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
                // ill-conditioned: wrist torques leaked into the neighbouring joints. `Axis`
                // keeps the crosswise part, A (I - u u^T) with u the joint axis in the child's
                // frame, and moves the part about the axis to the joint's rotor row below.
                mp.inertia += match cfg.armature {
                    ArmArmature::Isotropic => Mat3::IDENTITY * ARMATURE[i - 1],
                    ArmArmature::Axis => {
                        let u = zero[i].rot.conjugate() * Vec3::from_array(JOINT_AXIS[i - 1]);
                        (Mat3::IDENTITY - Mat3::from_cols((u * u.x).into(), (u * u.y).into(), (u * u.z).into())) * ARMATURE[i - 1]
                    }
                };
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
        if cfg.armature == ArmArmature::Axis {
            // The rotor about each hinge's free axis (joint-frame x); it starts at the links'
            // relative velocity (zero: the arm is built at rest)
            for (j, joint) in joints.iter().enumerate() { world.set_d6_armature(joint.id, 0, ARMATURE[j]); }
        }
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

/// How far an arm at its own velocity iterations (`lo`) strays from the same arm at many
/// (`hi`), open loop: both get torques from `torques(tick, substep, world, arm)` for `ticks`
/// control ticks. Returns the worst joint-rate difference as a share of the largest rate on the
/// `hi` arm, over the first tick (a cold start) and over the whole run, and the joint it was on.
#[cfg(test)]
pub(crate) fn iteration_rate_error(
    lo: (&mut World, &mut Arm), hi: (&mut World, &mut Arm), ticks: u32,
    mut torques: impl FnMut(u32, u32, &World, &Arm) -> [f32; JOINTS],
) -> (f32, f32, usize) {
    let dt = 1.0 / (CONTROL_HZ * SUBSTEPS) as f32;
    let (lw, la) = lo;
    let (hw, ha) = hi;
    let (mut scale, mut first, mut worst, mut at) = (0.0f32, 0.0f32, 0.0f32, 0usize);
    for tick in 0..ticks {
        for k in 0..SUBSTEPS {
            let t = torques(tick, k, lw, la);
            la.apply(lw, t, dt);
            lw.step(dt);
            let t = torques(tick, k, hw, ha);
            ha.apply(hw, t, dt);
            hw.step(dt);
            let (rl, rh) = (la.rates(lw), ha.rates(hw));
            for j in 0..JOINTS {
                scale = scale.max(rh[j].abs());
                let d = (rl[j] - rh[j]).abs();
                if d > worst { worst = d; at = j; }
            }
        }
        if tick == 0 { first = worst / scale.max(1e-9); }
    }
    (first, worst / scale.max(1e-9), at)
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

    /// A scripted torque profile: gravity compensation plus a sine per joint, 2 s at 60 Hz
    /// control. Returns the step hash (first 8 bytes, hex) after 1 s and after 2 s.
    fn torque_profile_hashes(cfg: &ArmConfig) -> (String, String) {
        let hex = |h: [u8; 32]| h.iter().take(8).map(|b| format!("{:02x}", b)).collect::<String>();
        let mut world = World::with_capacity(16, 16);
        let mut arm = Arm::build(&mut world, cfg, Vec3::ZERO, [0.3, -0.5, -0.8, 0.4, -0.9, 0.2]);
        let mut t = 0.0f32;
        let mut at_1s = String::new();
        for tick in 0..2 * CONTROL_HZ {
            arm.tick(&mut world, |w, a, dt| {
                t += dt;
                let g = a.gravity_torques(w);
                std::array::from_fn(|j| g[j] + 0.2 * MAX_TORQUE[j] * (t * (1.0 + 0.7 * j as f32)).sin())
            });
            if tick + 1 == CONTROL_HZ { at_1s = hex(world.step_hash()); }
        }
        (at_1s, hex(world.step_hash()))
    }

    /// PARITY_SPEC 3.9: the default arm is bit-identical across the D6 split (hashes recorded
    /// by a probe on the build before it, debug and release alike).
    #[test]
    fn the_default_arm_is_unchanged_by_the_axis_option() {
        let (a, b) = torque_profile_hashes(&ArmConfig::default());
        assert_eq!((a.as_str(), b.as_str()), ("fbec063efe6f8343", "bb1b6c157c0bb23d"));
    }

    fn axis_cfg() -> ArmConfig { ArmConfig::for_armature(ArmArmature::Axis) }

    /// Holds joint `k` still with a velocity motor (strong enough for any test torque here).
    fn lock(world: &mut World, arm: &Arm, k: usize) {
        use riftphys_articulation::bodies::D6Drive;
        world.set_d6_drive_r(arm.joints[k].id, 0, Some(D6Drive { target: 0.0, kp: 0.0, kd: 0.0, max_impulse: 1.0e6, vel_mode: true }));
    }

    /// Link i's inertia in its own frame, with the armature of the joint before it added as
    /// `inertia` says (the arm's build).
    fn local_inertia(i: usize, inertia: ArmArmature) -> Mat3 {
        let def = &LINK_DEFS[i];
        let u = fk(&[0.0; JOINTS])[i].rot.conjugate() * Vec3::from_array(JOINT_AXIS[i - 1]);
        link_mass(&def.shape, def.mass).inertia + match inertia {
            ArmArmature::Isotropic => Mat3::IDENTITY * ARMATURE[i - 1],
            ArmArmature::Axis => (Mat3::IDENTITY - Mat3::from_cols((u * u.x).into(), (u * u.y).into(), (u * u.z).into())) * ARMATURE[i - 1],
        }
    }

    /// The model's mass matrix over joints `js` (every other joint held), at the world's pose:
    /// link inertias as `inertia` builds them, plus each joint's rotor (its armature, on relative
    /// rotation) if `rotors`.
    fn mass_matrix(world: &World, arm: &Arm, js: &[usize], inertia: ArmArmature, rotors: bool) -> Vec<Vec<f32>> {
        let n = js.len();
        let mut m = vec![vec![0.0f32; n]; n];
        for i in 1..LINKS {
            let pose = world.get_body_pose(arm.links[i]);
            let local = local_inertia(i, inertia);
            // Link i's velocity per unit rate of each joint before it: (linear, angular in its frame)
            let col: Vec<Option<(Vec3, Vec3)>> = js.iter().map(|&j| (i > j).then(|| {
                let (a, p) = (arm.axis(world, j), arm.anchor(world, j));
                (a.cross(pose.pos - p), pose.rot.conjugate() * a)
            })).collect();
            for r in 0..n {
                for c in 0..n {
                    if let (Some((vr, wr)), Some((vc, wc))) = (col[r], col[c]) {
                        m[r][c] += LINK_DEFS[i].mass * vr.dot(vc) + wr.dot(local.mul_vec3(wc));
                    }
                }
            }
        }
        if rotors { for (r, &j) in js.iter().enumerate() { m[r][r] += ARMATURE[j]; } }
        m
    }

    /// One substep of torque `tau` on joint `driven`, joints `free` free and the rest held, zero
    /// gravity, from rest at pose `q`: the free joints' rates. `zero_rotors` builds `Axis` and
    /// then takes the rotors off (no armature about the axes).
    fn one_step_rates(q: [f32; JOINTS], mode: ArmArmature, zero_rotors: bool, iters: (u32, u32), driven: usize, tau: f32, free: &[usize]) -> (Vec<f32>, Vec<Vec<f32>>) {
        let dt = 1.0 / (CONTROL_HZ * SUBSTEPS) as f32;
        let mut world = World::with_capacity(16, 16);
        let mut arm = Arm::build(&mut world, &ArmConfig { armature: mode, solver_iters: iters }, Vec3::ZERO, q);
        world.set_gravity(Vec3::ZERO);
        if zero_rotors { for joint in &arm.joints { world.set_d6_armature(joint.id, 0, 0.0); } }
        for k in (0..JOINTS).filter(|k| !free.contains(k)) { lock(&mut world, &arm, k); }
        let m = mass_matrix(&world, &arm, free, mode, mode == ArmArmature::Axis && !zero_rotors);
        let mut t = [0.0; JOINTS];
        t[driven] = tau;
        arm.apply(&mut world, t, dt);
        world.step(dt);
        (free.iter().map(|&j| arm.rate(&world, j)).collect(), m)
    }

    /// PARITY_SPEC 3.9: with every other joint held (the parent fixed, the links beyond moving as
    /// one), a torque on joint j accelerates it as each mode's model predicts, within 1%. The two
    /// modes agree only where no joint beyond j turns about an axis along j's: a held rotor beyond
    /// adds nothing about j's axis in `Axis` (it turns with its housing), its armature in
    /// `Isotropic`. Run at 512 velocity iterations: the model is the subject; what the arm's own
    /// iterations give (8; 96 in `Axis`) is printed (a soft rotor row beside stiff held rows converges slowly).
    #[test]
    fn a_torque_turns_each_joint_as_its_model_predicts() {
        let q = [0.3, -0.5, -0.8, 0.4, -0.9, 0.2];
        let dt = 1.0 / (CONTROL_HZ * SUBSTEPS) as f32;
        for j in 0..JOINTS {
            let mut rate = [0.0f32; 2];
            for (k, mode) in [ArmArmature::Isotropic, ArmArmature::Axis].into_iter().enumerate() {
                let tau = 0.2 * MAX_TORQUE[j];
                let (got, m) = one_step_rates(q, mode, false, (512, 6), j, tau, &[j]);
                let (at8, _) = one_step_rates(q, mode, false, ArmConfig::for_armature(mode).solver_iters, j, tau, &[j]);
                let want = tau * dt / m[0][0];
                rate[k] = got[0];
                println!("{} {:?}: rate {:.5} rad/s, model {:.5} (I {:.4} kg m^2); at the arm's iterations {:+.2}%",
                    JOINT_NAMES[j], mode, got[0], want, m[0][0], 100.0 * (at8[0] / want - 1.0));
                assert!((got[0] - want).abs() <= 0.01 * want.abs(), "{} {:?}: {} vs {}", JOINT_NAMES[j], mode, got[0], want);
            }
            println!("{}: Axis / Isotropic {:.4}", JOINT_NAMES[j], rate[1] / rate[0]);
        }
    }

    /// PARITY_SPEC 3.9, the Reviewer's coupling test: zero gravity, the shoulder driven, the elbow
    /// free (both about z), the rest held. Over one substep from rest both joints move as each
    /// model's 2x2 mass matrix says, within 1%: `Axis` (the elbow's rotor resists only relative
    /// rotation), no armature, `Isotropic` (armature on the links' absolute rotation). The elbow's
    /// lag per unit of shoulder motion, -M21/M22, is printed for the three.
    #[test]
    fn a_free_elbow_follows_a_driven_shoulder_as_each_model_predicts() {
        let q = [0.0, -0.5, -0.8, 0.0, -0.9, 0.0];
        let dt = 1.0 / (CONTROL_HZ * SUBSTEPS) as f32;
        let tau = 30.0;
        for (name, mode, zero) in [("Axis", ArmArmature::Axis, false), ("no armature", ArmArmature::Axis, true), ("Isotropic", ArmArmature::Isotropic, false)] {
            let (got, m) = one_step_rates(q, mode, zero, (512, 6), 1, tau, &[1, 2]);
            let (at8, _) = one_step_rates(q, mode, zero, ArmConfig::for_armature(mode).solver_iters, 1, tau, &[1, 2]);
            let det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
            let want = [m[1][1] * tau * dt / det, -m[1][0] * tau * dt / det];
            println!("{}: shoulder {:.5} (model {:.5}), elbow {:.5} (model {:.5}) rad/s; elbow/shoulder {:.4}, model {:.4}; at the arm's iterations {:.4}",
                name, got[0], want[0], got[1], want[1], got[1] / got[0], -m[1][0] / m[1][1], at8[1] / at8[0]);
            for k in 0..2 {
                assert!((got[k] - want[k]).abs() <= 0.01 * want[k].abs(), "{} joint {}: {} vs {}", name, k + 1, got[k], want[k]);
            }
        }
    }

    /// Solves m x = b (Gaussian elimination, partial pivoting).
    fn solve_dense(mut m: Vec<Vec<f32>>, mut b: Vec<f32>) -> Vec<f32> {
        let n = b.len();
        for c in 0..n {
            let p = (c..n).fold(c, |best, r| if m[r][c].abs() > m[best][c].abs() { r } else { best });
            m.swap(c, p);
            b.swap(c, p);
            for r in c + 1..n {
                let f = m[r][c] / m[c][c];
                for k in c..n { m[r][k] -= f * m[c][k]; }
                b[r] -= f * b[c];
            }
        }
        let mut x = vec![0.0f32; n];
        for r in (0..n).rev() { x[r] = (b[r] - (r + 1..n).map(|k| m[r][k] * x[k]).sum::<f32>()) / m[r][r]; }
        x
    }

    /// PARITY_SPEC 3.9: the free arm (nothing held, zero gravity, from rest), a torque on each joint
    /// in turn for one substep: every joint's rate against the model's 6x6 mass matrix, in both
    /// modes. At 512 velocity iterations within 1% of the largest rate; the error at the arm's own
    /// iterations is printed (how much of the armature the default solve realises).
    #[test]
    fn the_free_arm_moves_as_its_mass_matrix_says() {
        let q = [0.3, -0.5, -0.8, 0.4, -0.9, 0.2];
        let dt = 1.0 / (CONTROL_HZ * SUBSTEPS) as f32;
        let all: Vec<usize> = (0..JOINTS).collect();
        for mode in [ArmArmature::Isotropic, ArmArmature::Axis] {
            for j in 0..JOINTS {
                let tau = 0.2 * MAX_TORQUE[j];
                let (got, m) = one_step_rates(q, mode, false, (512, 6), j, tau, &all);
                let (at8, _) = one_step_rates(q, mode, false, ArmConfig::for_armature(mode).solver_iters, j, tau, &all);
                let mut b = vec![0.0f32; JOINTS];
                b[j] = tau * dt;
                let want = solve_dense(m, b);
                let scale = want.iter().fold(0.0f32, |a, x| a.max(x.abs()));
                let err = |r: &[f32]| r.iter().zip(&want).fold(0.0f32, |a, (x, y)| a.max((x - y).abs())) / scale;
                println!("{:?}, torque on {}: worst joint error {:.4}% of the largest rate; at the arm's iterations {:.2}%",
                    mode, JOINT_NAMES[j], 100.0 * err(&got), 100.0 * err(&at8));
                assert!(err(&got) <= 0.01, "{:?} {}: {:?} vs {:?}", mode, JOINT_NAMES[j], got, want);
            }
        }
    }

    /// PARITY_SPEC 3.9: a small torque held on one joint of the free arm (zero gravity, from rest)
    /// for 0.1 s: every joint's rate against the model's 6x6 mass matrix, within 2% of the largest
    /// at the arm's own iterations. Warm-started joint rows converge across substeps what a cold
    /// first substep doesn't (`the_free_arm_moves_as_its_mass_matrix_says` prints that); the
    /// remainder at 512 iterations is the motion's own nonlinearity.
    #[test]
    fn a_held_torque_moves_the_free_arm_as_its_mass_matrix_says() {
        let q = [0.3, -0.5, -0.8, 0.4, -0.9, 0.2];
        let dt = 1.0 / (CONTROL_HZ * SUBSTEPS) as f32;
        let steps = 24;
        for mode in [ArmArmature::Isotropic, ArmArmature::Axis] {
            for j in 0..JOINTS {
                let tau = 0.01 * MAX_TORQUE[j];
                let mut line = format!("{:?}, torque on {:12}:", mode, JOINT_NAMES[j]);
                for iters in [ArmConfig::for_armature(mode).solver_iters, (512, 6)] {
                    let mut world = World::with_capacity(16, 16);
                    let mut arm = Arm::build(&mut world, &ArmConfig { armature: mode, solver_iters: iters }, Vec3::ZERO, q);
                    world.set_gravity(Vec3::ZERO);
                    let all: Vec<usize> = (0..JOINTS).collect();
                    let m = mass_matrix(&world, &arm, &all, mode, mode == ArmArmature::Axis);
                    let mut b = vec![0.0f32; JOINTS];
                    b[j] = tau * dt * steps as f32;
                    let want = solve_dense(m, b);
                    let mut t = [0.0; JOINTS];
                    t[j] = tau;
                    for _ in 0..steps { arm.apply(&mut world, t, dt); world.step(dt); }
                    let got = arm.rates(&world);
                    let scale = want.iter().fold(0.0f32, |a, x| a.max(x.abs()));
                    let err = got.iter().zip(&want).fold(0.0f32, |a, (x, y)| a.max((x - y).abs())) / scale;
                    line += &format!("  {:?} iters {:6.2}%", iters.0, 100.0 * err);
                    assert!(err <= 0.02, "{:?} {} at {:?} iterations: {:?} vs {:?}", mode, JOINT_NAMES[j], iters, got, want);
                }
                println!("{}", line);
            }
        }
    }

    fn profile(t: f32, w: &World, a: &Arm) -> [f32; JOINTS] {
        let g = a.gravity_torques(w);
        std::array::from_fn(|j| g[j] + 0.2 * MAX_TORQUE[j] * (t * (1.0 + 0.7 * j as f32)).sin())
    }

    /// The arm at its own iterations against 512, built from rest in `mode`, 1 s open loop.
    fn iteration_error(mode: ArmArmature, iters: u32, torques: impl FnMut(u32, u32, &World, &Arm) -> [f32; JOINTS]) -> (f32, f32, usize) {
        let q = [0.3, -0.5, -0.8, 0.4, -0.9, 0.2];
        let (mut lw, mut hw) = (World::with_capacity(16, 16), World::with_capacity(16, 16));
        let mut la = Arm::build(&mut lw, &ArmConfig { armature: mode, solver_iters: (iters, 6) }, Vec3::ZERO, q);
        let mut ha = Arm::build(&mut hw, &ArmConfig { armature: mode, solver_iters: (512, 6) }, Vec3::ZERO, q);
        iteration_rate_error((&mut lw, &mut la), (&mut hw, &mut ha), CONTROL_HZ, torques)
    }

    /// The scripted sine profile (`torque_profile_hashes`'s) and a fresh random torque each
    /// control tick (uniform within ±0.2 of each joint's limit, gravity compensated, seeded).
    fn sine_and_random(mode: ArmArmature, iters: u32) -> [(f32, f32, usize); 2] {
        let dt = 1.0 / (CONTROL_HZ * SUBSTEPS) as f32;
        let sine = iteration_error(mode, iters, |tick, k, w, a| profile((tick * SUBSTEPS + k + 1) as f32 * dt, w, a));
        let mut rng = crate::rng::Rng::new(0xD6);
        let kicks: Vec<[f32; JOINTS]> = (0..CONTROL_HZ).map(|_| std::array::from_fn(|j| rng.range(-0.2, 0.2) * MAX_TORQUE[j])).collect();
        let random = iteration_error(mode, iters, |tick, _, w, a| {
            let g = a.gravity_torques(w);
            std::array::from_fn(|j| g[j] + kicks[tick as usize][j])
        });
        [sine, random]
    }

    /// Largest joint rate over 1 s of the sine profile (finite or not), at `iters`.
    fn sine_top_rate(mode: ArmArmature, iters: u32, zero_rotors: bool) -> f32 {
        let dt = 1.0 / (CONTROL_HZ * SUBSTEPS) as f32;
        let mut w = World::with_capacity(16, 16);
        let mut a = Arm::build(&mut w, &ArmConfig { armature: mode, solver_iters: (iters, 6) }, Vec3::ZERO, [0.3, -0.5, -0.8, 0.4, -0.9, 0.2]);
        if zero_rotors { for joint in &a.joints { w.set_d6_armature(joint.id, 0, 0.0); } }
        let mut top = 0.0f32;
        for n in 0..CONTROL_HZ * SUBSTEPS {
            let t = profile((n + 1) as f32 * dt, &w, &a);
            a.apply(&mut w, t, dt);
            w.step(dt);
            for r in a.rates(&w) { top = if r.is_finite() { top.max(r.abs()) } else { f32::INFINITY }; }
        }
        top
    }

    /// PARITY_SPEC 3.4j cost gate (report-only): the Axis arm at its own iterations on the sine
    /// profile for 1 s, wall clock per step with W4b alone, W4f (no blocks) and W4g.
    #[test]
    #[ignore]
    fn w4g_axis_cost_probe() {
        let dt = 1.0 / (CONTROL_HZ * SUBSTEPS) as f32;
        for (name, rotor_row, blocks) in [("W4b alone", false, false), ("W4f", true, false), ("W4g", true, true)] {
            let mut best = f64::INFINITY;
            for _ in 0..5 {
                let mut w = World::with_capacity(16, 16);
                let mut a = Arm::build(&mut w, &ArmConfig::for_armature(ArmArmature::Axis), Vec3::ZERO, [0.3, -0.5, -0.8, 0.4, -0.9, 0.2]);
                w.set_rotor_position_row(rotor_row);
                w.set_armature_blocks(blocks);
                let t0 = std::time::Instant::now();
                for n in 0..CONTROL_HZ * SUBSTEPS {
                    let t = profile((n + 1) as f32 * dt, &w, &a);
                    a.apply(&mut w, t, dt);
                    w.step(dt);
                }
                best = best.min(t0.elapsed().as_secs_f64() / (CONTROL_HZ * SUBSTEPS) as f64);
                if blocks { println!("blocks {:?}", w.armature_blocks()); }
            }
            println!("Axis arm, {}: {:.1} us per step (best of 5)", name, best * 1e6);
        }
    }

    /// PARITY_SPEC 3.9: the sine profile drives wrist_1 into the tool at tick 23. `Axis` stays
    /// finite and bounded through it at its own iterations and at 512 (it measured 85 rad/s; the
    /// World's undivided gyroscopic turn took the thin tool link to 1e25), and before the contact
    /// it follows the 512-iteration arm within 2%.
    #[test]
    fn an_axis_arm_stays_bounded_through_a_self_contact() {
        let own = ArmConfig::for_armature(ArmArmature::Axis).solver_iters.0;
        for iters in [own, 512] {
            let top = sine_top_rate(ArmArmature::Axis, iters, false);
            println!("Axis at {} iterations: largest joint rate {:.2} rad/s", iters, top);
            assert!(top.is_finite() && top < 200.0, "at {} iterations: {}", iters, top);
        }
        let dt = 1.0 / (CONTROL_HZ * SUBSTEPS) as f32;
        let q = [0.3, -0.5, -0.8, 0.4, -0.9, 0.2];
        let (mut lw, mut hw) = (World::with_capacity(16, 16), World::with_capacity(16, 16));
        let mut la = Arm::build(&mut lw, &ArmConfig::for_armature(ArmArmature::Axis), Vec3::ZERO, q);
        let mut ha = Arm::build(&mut hw, &ArmConfig { armature: ArmArmature::Axis, solver_iters: (512, 6) }, Vec3::ZERO, q);
        let (_, before, j) = iteration_rate_error((&mut lw, &mut la), (&mut hw, &mut ha), 22, |tick, k, w, a| profile((tick * SUBSTEPS + k + 1) as f32 * dt, w, a));
        println!("before the first contact: {:.2}% (worst on {})", 100.0 * before, JOINT_NAMES[j]);
        assert!(before <= 0.02, "{:.2}% on {}", 100.0 * before, JOINT_NAMES[j]);
    }

    /// Probe (PARITY_SPEC 3.9, open item): a chain of thin links with no armature, the sine
    /// profile, largest joint rate. Run with --ignored --nocapture.
    #[test]
    #[ignore]
    fn d6_rotors_zeroed_probe() {
        for iters in [8, 96, 512] {
            println!("rotors zeroed, {} iterations: largest joint rate {:e} rad/s", iters, sine_top_rate(ArmArmature::Axis, iters, true));
        }
    }

    /// Probe (PARITY_SPEC 3.9, the Reviewer's cold-substep check). Run with --ignored --nocapture.
    #[test]
    #[ignore]
    fn d6_iteration_error_probe() {
        for (mode, iters) in [(ArmArmature::Isotropic, 8), (ArmArmature::Axis, 8), (ArmArmature::Axis, 16), (ArmArmature::Axis, 32), (ArmArmature::Axis, 64), (ArmArmature::Axis, 96)] {
            for (name, (first, all, j)) in ["sine", "random"].iter().zip(sine_and_random(mode, iters)) {
                println!("{:?} at {:2} iterations {:6}: first tick {:6.2}%, over 1 s {:6.2}% (worst on {})", mode, iters, name, 100.0 * first, 100.0 * all, JOINT_NAMES[j]);
            }
        }
    }

    /// PARITY_SPEC 3.9: a rollback in `Axis` mode replays hash for hash (the rotors are carried
    /// state, restored with the rest).
    #[test]
    fn an_axis_arm_rolls_back_hash_for_hash() {
        let mut world = World::with_capacity(16, 16);
        let mut arm = Arm::build(&mut world, &axis_cfg(), Vec3::ZERO, [0.3, -0.5, -0.8, 0.4, -0.9, 0.2]);
        let mut t = 0.0f32;
        for _ in 0..30 { arm.tick(&mut world, |w, a, dt| { t += dt; profile(t, w, a) }); }
        let saved: Vec<(Isometry, Velocity)> = arm.links.iter().map(|&b| (world.get_body_pose(b), world.get_body_vel(b))).collect();
        let carried = world.carried_state();
        let run = |world: &mut World, arm: &mut Arm, t0: f32| -> Vec<[u8; 32]> {
            let mut tt = t0;
            (0..30).map(|_| {
                arm.tick(world, |w, a, dt| { tt += dt; profile(tt, w, a) });
                world.step_hash()
            }).collect()
        };
        let first = run(&mut world, &mut arm, t);
        for (&b, &(pose, vel)) in arm.links.iter().zip(&saved) {
            world.set_body_pose(b, pose);
            world.set_body_vel(b, vel);
        }
        world.restore_carried_state(&carried);
        assert_eq!(first, run(&mut world, &mut arm, t), "Axis-mode replay diverged");
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
