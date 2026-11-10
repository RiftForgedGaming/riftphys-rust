//! Cassie jumping impacts (PARITY_SPEC W4, P4): the DAIR Lab's impact dataset replayed in
//! RiftPhys. This module holds the data (the dataset's `.npy` trajectories), the model (Cassie
//! from `cassie_v2.urdf` through [`crate::urdf`], plus dairlib's constants: the achilles rods,
//! the leaf springs' stiffness, the motors' reflected inertia, the toe contact points), and the
//! pre-fit checks of PARITY_SPEC 3.4a (the rod length and toe heights at the first sample, a
//! forward-kinematics round trip, the leaf springs' frequencies on the built model).
//!
//! Conventions, from the dataset's README and the paper's code (3.4a):
//! - a state row is 45 numbers: pelvis quaternion (w, x, y, z), pelvis origin xyz, the 16 joint
//!   angles, then pelvis angular velocity (world frame), pelvis origin velocity (world frame) and
//!   the 16 joint rates, in the position order (columns 41 and 43, the ankle springs' rates, are
//!   unmeasured and 0);
//! - the 10 inputs are joint-side torques, N m, at the same times;
//! - the URDF world is z-up; RiftPhys's is y-up ([`urdf::Z_UP_TO_Y_UP`]).

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use glam::{DQuat, DVec3};
use riftphys_core::{BodyId, Isometry, JointId, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::World;

use crate::urdf::{self, Frame, Urdf};

/// Where the parity data sits on this machine (outside the repository).
pub const DATA_ROOT: &str = "R:/BACKUP REPOS ETC/riftphys-parity-data/cassie";

/// The 16 joints of a state row, in column order (positions 7..23, rates 29..45).
pub const JOINTS: [&str; 16] = [
    "hip_roll_left", "hip_roll_right", "hip_yaw_left", "hip_yaw_right", "hip_pitch_left", "hip_pitch_right",
    "knee_left", "knee_right", "knee_joint_left", "knee_joint_right", "ankle_joint_left", "ankle_joint_right",
    "ankle_spring_joint_left", "toe_left", "ankle_spring_joint_right", "toe_right",
];

/// The 10 actuated joints, in input order, with dairlib's rotor inertias (kg m^2) and gear
/// ratios (`AddCassieMultibody`): reflected inertia = rotor x ratio^2.
pub const ACTUATORS: [&str; 10] = [
    "hip_roll_left", "hip_roll_right", "hip_yaw_left", "hip_yaw_right", "hip_pitch_left", "hip_pitch_right",
    "knee_left", "knee_right", "toe_left", "toe_right",
];
pub const ROTOR_INERTIA: [f64; 10] = [61e-6, 61e-6, 61e-6, 61e-6, 365e-6, 365e-6, 365e-6, 365e-6, 4.9e-6, 4.9e-6];
pub const GEAR_RATIO: [f64; 10] = [25.0, 25.0, 25.0, 25.0, 16.0, 16.0, 16.0, 16.0, 50.0, 50.0];

/// The paper's 22 jumps (raw logs 08-11, 14-17, 20-33) as public trajectory numbers.
pub const PAPER_JUMPS: [usize; 22] = [0, 1, 2, 3, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23];
/// All public trajectories (the 2021-02-26 ones, 24-28, came after the paper).
pub const ALL_JUMPS: usize = 29;

/// The achilles rod (dairlib `cassie_utils.{h,cc}`): rest length, its ends on the heel spring
/// and the thigh (z +-0.045 for left / right), and Drake's spring standing in for the loop.
pub const ACHILLES_LENGTH: f64 = 0.5012;
pub const ROD_ON_HEEL: [f64; 3] = [0.11877, -0.01, 0.0];
pub const ROD_ON_THIGH_Z: f64 = 0.045;
/// The toe's two contact points in the toe frame (front, rear).
pub const TOE_POINTS: [[f64; 3]; 2] = [[-0.0457, 0.112, 0.0], [0.088, 0.0, 0.0]];

/// The model's free parameters: leaf springs (dairlib's code values, not the URDF's), the rods,
/// the toe spheres and the contact.
#[derive(Clone, Copy, Debug)]
pub struct Params {
    /// Knee and ankle leaf springs, N m/rad (dairlib: 1500 and 1250; the URDF says 2300, 2000).
    pub knee_k: f64,
    pub ankle_k: f64,
    /// Achilles rod: compliance, m/N, and damping, N s/m (Drake's 1e6 N/m, 2e3 N s/m).
    pub rod_compliance: f32,
    pub rod_damping: f32,
    /// Toe sphere radius, m.
    pub toe_r: f32,
    /// Contact: stiffness N/m, Hunt-Crossley dissipation s/m, friction.
    pub k: f32,
    pub d: f32,
    pub mu: f32,
    /// Joint damping from the URDF (false zeroes it, to measure natural frequencies).
    pub joint_damping: bool,
    /// Reflected motor inertia (false leaves the rotors out).
    pub armature: bool,
    /// The toe spheres (false: no contact, for the half-step start's trial step).
    pub toes: bool,
    /// Solver iterations (velocity, position): the biped workload's (8, 6), at which P4 is judged
    /// (D9, decided 2026-11-10); the World's own default is (20, 20).
    pub iters: (u32, u32),
    /// Apply the measured motor torques (false: none, a diagnostic).
    pub inputs: bool,
    /// The ankle springs' damping, N m s/rad, overriding the URDF's "made up" 10 (None: the URDF).
    pub ankle_spring_c: Option<f64>,
}

impl Default for Params {
    fn default() -> Self {
        // Contact at Table VI's Drake values until fitted
        Self { knee_k: 1500.0, ankle_k: 1250.0, rod_compliance: 1.0e-6, rod_damping: 2.0e3, toe_r: 0.001,
               k: 9400.0, d: 3.0, mu: 0.45, joint_damping: true, armature: true, toes: true, iters: (8, 6), inputs: true, ankle_spring_c: None }
    }
}

/* ───────────────────────── the data ───────────────────────── */

/// Reads a little-endian float64 `.npy` array (format 1.0 or 2.0, C order): shape and values.
/// The header is parsed as text; nothing is unpickled.
pub fn read_npy_f64(path: &Path) -> Result<(Vec<usize>, Vec<f64>), String> {
    let b = std::fs::read(path).map_err(|e| format!("{}: {}", path.display(), e))?;
    if b.len() < 10 || &b[..6] != b"\x93NUMPY" { return Err(format!("{}: not an .npy file", path.display())); }
    let (hlen, start) = match b[6] {
        1 => (u16::from_le_bytes([b[8], b[9]]) as usize, 10),
        2 | 3 => (u32::from_le_bytes([b[8], b[9], b[10], b[11]]) as usize, 12),
        v => return Err(format!("{}: npy version {}", path.display(), v)),
    };
    let header = std::str::from_utf8(&b[start..start + hlen]).map_err(|_| "npy header isn't text")?;
    if !header.contains("'descr': '<f8'") { return Err(format!("{}: not little-endian float64 ({})", path.display(), header.trim())); }
    if header.contains("'fortran_order': True") { return Err(format!("{}: Fortran order", path.display())); }
    let s = header.split("'shape':").nth(1).and_then(|r| r.split('(').nth(1)).and_then(|r| r.split(')').next()).ok_or("npy: no shape")?;
    let shape: Vec<usize> = s.split(',').map(str::trim).filter(|t| !t.is_empty()).map(|t| t.parse::<usize>().map_err(|e| e.to_string())).collect::<Result<_, _>>()?;
    let n: usize = shape.iter().product();
    let data = &b[start + hlen..];
    if data.len() != 8 * n { return Err(format!("{}: {} bytes of data for shape {:?}", path.display(), data.len(), shape)); }
    Ok((shape, data.chunks_exact(8).map(|c| f64::from_le_bytes(c.try_into().unwrap())).collect()))
}

/// One public trajectory: times (s), states (45) and inputs (10), 100 samples at about 2 kHz.
#[derive(Clone, Debug)]
pub struct Traj {
    pub index: usize,
    pub t: Vec<f64>,
    pub x: Vec<[f64; 45]>,
    pub u: Vec<[f64; 10]>,
}

pub fn data_dir() -> PathBuf { Path::new(DATA_ROOT).join("cassie_impact_dataset-main/jumping_data/data") }

pub fn load_traj(dir: &Path, index: usize) -> Result<Traj, String> {
    let rows = |name: &str, w: usize| -> Result<Vec<Vec<f64>>, String> {
        let (shape, v) = read_npy_f64(&dir.join(format!("{}_{:02}.npy", name, index)))?;
        let cols = if shape.len() == 1 { 1 } else { shape[1] };
        if cols != w { return Err(format!("{}_{:02}: {} columns, expected {}", name, index, cols, w)); }
        Ok(v.chunks_exact(w).map(|c| c.to_vec()).collect())
    };
    let t: Vec<f64> = rows("t", 1)?.into_iter().map(|r| r[0]).collect();
    let x: Vec<[f64; 45]> = rows("x", 45)?.into_iter().map(|r| r.try_into().unwrap()).collect();
    let u: Vec<[f64; 10]> = rows("u", 10)?.into_iter().map(|r| r.try_into().unwrap()).collect();
    if t.len() != x.len() || t.len() != u.len() { return Err(format!("trajectory {}: {} times, {} states, {} inputs", index, t.len(), x.len(), u.len())); }
    Ok(Traj { index, t, x, u })
}

/* ───────────────────────── the model ───────────────────────── */

/// Cassie built in a World.
pub struct Cassie {
    pub world: World,
    /// Whether `apply_inputs` applies the measured torques.
    pub inputs: bool,
    /// The static ground box.
    pub ground: BodyId,
    pub urdf: Urdf,
    pub model: urdf::Model,
    /// Achilles rods, left and right.
    pub rods: [JointId; 2],
    /// Toe spheres: (toe link, body, point in the toe frame).
    pub toes: Vec<(&'static str, BodyId, DVec3)>,
}

pub fn load_urdf() -> Result<Urdf, String> {
    let path = Path::new(DATA_ROOT).join("cassie_v2.urdf");
    Urdf::parse(&std::fs::read_to_string(&path).map_err(|e| format!("{}: {}", path.display(), e))?)
}

/// The URDF build configuration for a state row (pelvis floating, unless `pinned`).
pub fn state_config(row: &[f64; 45], params: &Params, pinned: bool) -> urdf::Config {
    let q = DQuat::from_xyzw(row[1], row[2], row[3], row[0]).normalize();
    let mut cfg = urdf::Config {
        root: Frame::new(DVec3::new(row[4], row[5], row[6]), q),
        root_ang: DVec3::new(row[23], row[24], row[25]),
        root_lin: DVec3::new(row[26], row[27], row[28]),
        floating: !pinned,
        ..Default::default()
    };
    for (k, j) in JOINTS.iter().enumerate() {
        cfg.q.insert(j.to_string(), row[7 + k]);
        cfg.qd.insert(j.to_string(), row[29 + k]);
    }
    cfg.stiffness = HashMap::from([
        ("knee_joint_left".to_string(), params.knee_k), ("knee_joint_right".to_string(), params.knee_k),
        ("ankle_spring_joint_left".to_string(), params.ankle_k), ("ankle_spring_joint_right".to_string(), params.ankle_k),
    ]);
    cfg
}

/// Builds Cassie at a state row on a ground box (top at height 0), gravity on. `pinned` welds
/// the pelvis to the world (for hanging tests).
pub fn build(urdf: &Urdf, row: &[f64; 45], p: &Params, pinned: bool) -> Result<Cassie, String> {
    let mut world = World::with_capacity(32, 32);
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));
    world.set_solver_iterations(p.iters.0, p.iters.1);
    let ground = world.add_body(Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    world.add_collider(ground, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(MaterialId::Concrete));
    let mut pair = world.pair_props(MaterialId::Rubber, MaterialId::Concrete);
    pair.stiffness = p.k;
    pair.dissipation = p.d;
    pair.mu_s = p.mu;
    pair.mu_k = p.mu;
    world.set_pair_props(MaterialId::Rubber, MaterialId::Concrete, pair);

    let model = urdf::build(urdf, &mut world, &state_config(row, p, pinned))?;
    if !p.joint_damping {
        for id in model.joints.values() { world.set_d6_damping(*id, true, 0, 0.0, 0.0); }
    } else if let Some(c) = p.ankle_spring_c {
        for name in ["ankle_spring_joint_left", "ankle_spring_joint_right"] { world.set_d6_damping(model.joints[name], true, 0, c as f32, 0.0); }
    }
    if p.armature {
        for (k, name) in ACTUATORS.iter().enumerate() {
            let id = *model.joints.get(*name).ok_or(format!("no joint {}", name))?;
            world.set_d6_armature(id, 0, (ROTOR_INERTIA[k] * GEAR_RATIO[k] * GEAR_RATIO[k]) as f32);
        }
    }
    let mut rods = Vec::new();
    for (side, z) in [("left", ROD_ON_THIGH_Z), ("right", -ROD_ON_THIGH_Z)] {
        let (hb, hp) = model.body_point(&format!("heel_spring_{}", side), DVec3::from(ROD_ON_HEEL)).ok_or("no heel spring")?;
        let (tb, tp) = model.body_point(&format!("thigh_{}", side), DVec3::new(0.0, 0.0, z)).ok_or("no thigh")?;
        let j = world.add_distance_joint_at(tb, tp, hb, hp, ACHILLES_LENGTH as f32, p.rod_compliance);
        world.set_distance_damping(j, p.rod_damping);
        rods.push(j);
    }
    let mut toes = Vec::new();
    for link in ["toe_left", "toe_right"] {
        if !p.toes { break; }
        for pt in TOE_POINTS {
            let (b, local) = model.body_point(link, DVec3::from(pt)).ok_or("no toe")?;
            world.add_collider_at(b, Isometry { pos: local, rot: Quat::IDENTITY }, Shape::Sphere { r: p.toe_r }, material(MaterialId::Rubber));
            toes.push((link, b, DVec3::from(pt)));
        }
    }
    Ok(Cassie { world, inputs: p.inputs, ground, urdf: urdf.clone(), model, rods: [rods[0], rods[1]], toes })
}

fn d(v: Vec3) -> DVec3 { DVec3::new(v.x as f64, v.y as f64, v.z as f64) }

impl Cassie {
    /// A link's frame in the URDF's (z-up) world.
    pub fn link_frame_urdf(&self, link: &str) -> Frame {
        let f = self.model.link_frame(&self.world, link).expect("link");
        urdf::Z_UP_TO_Y_UP.inverse().mul(&f)
    }

    /// A link's angular velocity and the velocity of a point in it, in the URDF's world.
    fn link_vel_urdf(&self, link: &str, p_link: DVec3) -> (DVec3, DVec3) {
        let (b, _) = self.model.links[link];
        let body = &self.model.bodies[b];
        let v = self.world.get_body_vel(body.id);
        let pose = self.world.get_body_pose(body.id);
        let at = self.model.point(&self.world, link, p_link).unwrap();
        let w = d(v.ang);
        let lin = d(v.lin) + w.cross(at - d(pose.pos));
        let back = urdf::Z_UP_TO_Y_UP.rot.conjugate();
        (back * w, back * lin)
    }

    /// The state row read back from the World (the forward-kinematics round trip).
    pub fn state(&self) -> [f64; 45] {
        let mut x = [0.0; 45];
        let pelvis = self.link_frame_urdf("pelvis");
        let q = if pelvis.rot.w < 0.0 { -pelvis.rot } else { pelvis.rot };
        x[0..4].copy_from_slice(&[q.w, q.x, q.y, q.z]);
        x[4..7].copy_from_slice(&pelvis.pos.to_array());
        let (w, v) = self.link_vel_urdf("pelvis", DVec3::ZERO);
        x[23..26].copy_from_slice(&w.to_array());
        x[26..29].copy_from_slice(&v.to_array());
        for (k, name) in JOINTS.iter().enumerate() {
            x[7 + k] = urdf::joint_angle(&self.world, &self.model, &self.urdf, name).unwrap();
            let j = self.urdf.joint(name).unwrap();
            let (wp, _) = self.link_vel_urdf(&j.parent, DVec3::ZERO);
            let (wc, _) = self.link_vel_urdf(&j.child, DVec3::ZERO);
            let axis = self.link_frame_urdf(&j.child).rot * j.axis;
            x[29 + k] = (wc - wp).dot(axis);
        }
        x
    }

    /// Achilles rod lengths now, left and right (m).
    pub fn rod_lengths(&self) -> [f64; 2] {
        let mut out = [0.0; 2];
        for (i, (side, z)) in [("left", ROD_ON_THIGH_Z), ("right", -ROD_ON_THIGH_Z)].into_iter().enumerate() {
            let h = self.model.point(&self.world, &format!("heel_spring_{}", side), DVec3::from(ROD_ON_HEEL)).unwrap();
            let t = self.model.point(&self.world, &format!("thigh_{}", side), DVec3::new(0.0, 0.0, z)).unwrap();
            out[i] = (h - t).length();
        }
        out
    }

    /// The rods' length rates (m/s), left and right.
    pub fn rod_rates(&self) -> [f64; 2] {
        let mut out = [0.0; 2];
        for (i, (side, z)) in [("left", ROD_ON_THIGH_Z), ("right", -ROD_ON_THIGH_Z)].into_iter().enumerate() {
            let (hl, tl) = (format!("heel_spring_{}", side), format!("thigh_{}", side));
            let (ph, pt) = (DVec3::from(ROD_ON_HEEL), DVec3::new(0.0, 0.0, z));
            let (h, t) = (self.link_frame_urdf(&hl).point(ph), self.link_frame_urdf(&tl).point(pt));
            let (_, vh) = self.link_vel_urdf(&hl, ph);
            let (_, vt) = self.link_vel_urdf(&tl, pt);
            out[i] = (vh - vt).dot((h - t).normalize());
        }
        out
    }

    /// The toe points' vertical velocities (m/s), in the order of `toe_heights`.
    pub fn toe_vz(&self) -> [f64; 4] {
        let mut out = [0.0; 4];
        for (i, (link, _, p)) in self.toes.iter().enumerate() { out[i] = self.link_vel_urdf(link, *p).1.z; }
        out
    }

    /// The toe points' heights above the ground (m): left front, left rear, right front,
    /// right rear. The sphere centres; a sphere touches at r.
    pub fn toe_heights(&self) -> [f64; 4] {
        let mut out = [0.0; 4];
        for (i, (link, _, p)) in self.toes.iter().enumerate() { out[i] = self.link_frame_urdf(link).point(*p).z; }
        out
    }
}

/// The passive joints whose recorded rates lag their positions by about 4.5 ms (the estimator
/// filters them; the Reviewer measured lags of 9-10 samples, against 1 for the motor joints):
/// the knee springs and the ankles, as (position column, rate column).
pub const LAGGED_RATES: [(usize, usize); 4] = [(15, 37), (16, 38), (17, 39), (18, 40)];
/// Samples for the one-sided fit at sample 0 (about 3.5 ms, before every jump's first contact).
pub const START_FIT_SAMPLES: usize = 8;

/// The derivative at sample 0 of a least-squares quadratic through column `col`'s first `n`
/// samples: x = a + b (t - t0) + c (t - t0)^2, returning b.
pub fn start_slope(t: &Traj, col: usize, n: usize) -> f64 {
    let t0 = t.t[0];
    // Normal equations for [a, b, c]
    let mut m = [[0.0f64; 3]; 3];
    let mut r = [0.0f64; 3];
    for k in 0..n.min(t.x.len()) {
        let dt = t.t[k] - t0;
        let basis = [1.0, dt, dt * dt];
        for i in 0..3 { r[i] += basis[i] * t.x[k][col]; for j in 0..3 { m[i][j] += basis[i] * basis[j]; } }
    }
    // Cramer's rule for b
    let det3 = |a: [[f64; 3]; 3]| a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    let mut mb = m;
    for i in 0..3 { mb[i][1] = r[i]; }
    det3(mb) / det3(m)
}

/// Sample 0's state for the replay's start (PARITY_SPEC 3.4a): the recorded row with the lagged
/// passive rates (knee springs, ankles) taken from a quadratic fit to their recorded positions
/// over the first `START_FIT_SAMPLES` samples; the motor joints and the pelvis keep their
/// recorded rates. The ankle springs are then solved by closure ([`loop_consistent`]).
pub fn start_row(urdf: &Urdf, t: &Traj, p: &Params) -> Result<([f64; 45], [f64; 2]), String> {
    let mut row = t.x[0];
    for (pos, rate) in LAGGED_RATES { row[rate] = start_slope(t, pos, START_FIT_SAMPLES); }
    loop_consistent(urdf, &row, p)
}

/// The state row with each ankle spring's rate (columns 41 and 43, unmeasured and 0 in the data)
/// set so its achilles rod's length rate is 0: the loop-consistent start (PARITY_SPEC 3.4a). The
/// positions already close the loop (the estimator's inverse kinematics), and the rod's rate is
/// linear in the ankle spring's, one scalar per leg. Returns the row and the two rates found.
pub fn loop_consistent(urdf: &Urdf, row: &[f64; 45], p: &Params) -> Result<([f64; 45], [f64; 2]), String> {
    let rates = |r: &[f64; 45]| -> Result<[f64; 2], String> { Ok(build(urdf, r, p, false)?.rod_rates()) };
    let r0 = rates(row)?;
    let mut probe = *row;
    probe[41] += 1.0;
    probe[43] += 1.0;
    let r1 = rates(&probe)?;
    let mut out = *row;
    let mut found = [0.0; 2];
    for (k, col) in [41usize, 43].into_iter().enumerate() {
        let slope = r1[k] - r0[k];
        if !(slope.abs() > 1e-9) { return Err(format!("the rod's rate doesn't depend on column {}", col)); }
        found[k] = row[col] - r0[k] / slope;
        out[col] = found[k];
    }
    Ok((out, found))
}

/// Smallest angle between two unit quaternions given as (w, x, y, z).
pub fn quat_angle(a: &[f64], b: &[f64]) -> f64 {
    let dot: f64 = a.iter().zip(b).map(|(x, y)| x * y).sum::<f64>().abs().min(1.0);
    2.0 * dot.acos()
}

/* ───────────────────────── the loss ───────────────────────── */

/// The paper's loss per trajectory, as `cassie_loss_utils.py` computes it (PARITY_SPEC 3.4a): the
/// mean over the samples of the weighted squared state error, in four terms:
/// - positions (pelvis origin xyz and the 16 joints, columns 4-22), weight 10;
/// - velocities (pelvis origin velocity and the 16 joint rates, columns 26-44): pelvis 100; hip
///   roll, knee spring and toe 0.01; ankle spring 0; the rest 1;
/// - pelvis angular velocity (columns 23-25), weight 5;
/// - orientation, (2 x the quaternion's geodesic distance)^2, the rotation angle squared, weight 1.
pub fn loss_terms(sim: &[[f64; 45]], data: &[[f64; 45]]) -> [f64; 4] {
    const VEL_W: [f64; 19] = [100.0, 100.0, 100.0, 0.01, 0.01, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.01, 0.01, 1.0, 1.0, 0.0, 0.01, 0.0, 0.01];
    let n = sim.len().min(data.len());
    let mut t = [0.0f64; 4];
    for k in 0..n {
        let (a, b) = (&sim[k], &data[k]);
        for i in 4..23 { t[0] += 10.0 * (a[i] - b[i]).powi(2); }
        for (j, i) in (26..45).enumerate() { t[1] += VEL_W[j] * (a[i] - b[i]).powi(2); }
        for i in 23..26 { t[2] += 5.0 * (a[i] - b[i]).powi(2); }
        let qa = DQuat::from_xyzw(a[1], a[2], a[3], a[0]).normalize();
        let qb = DQuat::from_xyzw(b[1], b[2], b[3], b[0]).normalize();
        t[3] += quat_angle(&[qa.w, qa.x, qa.y, qa.z], &[qb.w, qb.x, qb.y, qb.z]).powi(2);
    }
    t.map(|v| v / n as f64)
}

pub fn loss(sim: &[[f64; 45]], data: &[[f64; 45]]) -> f64 { loss_terms(sim, data).iter().sum() }

/// The recorded states with the lagged passive rates (knee springs, ankles) replaced by their
/// recorded positions' derivative: a least-squares quadratic over 9 samples on the real times,
/// centred where it can be, one-sided at the edges. For the secondary loss line, a diagnostic
/// (the 4.5 ms window also smooths the impact transient).
pub fn delag(t: &Traj) -> Vec<[f64; 45]> {
    let n = t.x.len();
    let w = 9usize.min(n);
    let mut out = t.x.clone();
    for k in 0..n {
        let lo = k.saturating_sub(w / 2).min(n - w);
        for (pos, rate) in LAGGED_RATES {
            out[k][rate] = quad_slope(&t.t[lo..lo + w], &t.x[lo..lo + w], pos, t.t[k]);
        }
    }
    out
}

/// The derivative at time `at` of a least-squares quadratic through column `col` of `x`.
fn quad_slope(ts: &[f64], x: &[[f64; 45]], col: usize, at: f64) -> f64 {
    let mut m = [[0.0f64; 3]; 3];
    let mut r = [0.0f64; 3];
    for (tk, xk) in ts.iter().zip(x) {
        let d = tk - at;
        let basis = [1.0, d, d * d];
        for i in 0..3 { r[i] += basis[i] * xk[col]; for j in 0..3 { m[i][j] += basis[i] * basis[j]; } }
    }
    let det3 = |a: [[f64; 3]; 3]| a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    let mut mb = m;
    for i in 0..3 { mb[i][1] = r[i]; }
    det3(mb) / det3(m)
}

/* ───────────────────────── the replay ───────────────────────── */

/// Which start a replay uses (PARITY_SPEC 3.4a).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Line {
    /// The loop-consistent start (lagged passive rates refit), then the half-step correction.
    Main,
    /// The main line's corrected start with the ankle springs' rates set to 0.
    Sensitivity,
}

/// How the half-step start's correction is taken (PARITY_SPEC 3.4a).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Correction {
    /// None: the start is the loop-consistent state itself.
    Off,
    /// From one trial step at the replay's own step h: v0 - (v1 - v0) / 2.
    Trial,
    /// From one trial step at a finer step h_f, scaled to h: v0 - (h / h_f) (v1 - v0) / 2.
    Fine(f64),
    /// The rule decided before the fit (PARITY_SPEC 3.4a): a coordinate takes its own trial
    /// step's correction only if that step's change, over a 2 kHz trial step's, is within 25% of
    /// the step ratio h / h_2k (smooth dynamics); the others take none. At 2 kHz every
    /// coordinate passes, so it is the plain trial correction there.
    Gated,
}

/// What a replay gives back.
#[derive(Clone, Debug)]
pub struct Replay {
    /// The simulated state at each recorded sample time.
    pub states: Vec<[f64; 45]>,
    /// Time of the first toe touch (a sphere's bottom at or below the ground), s after sample 0.
    pub first_contact: Option<f64>,
    /// The half-step trial step: the largest change of an ankle spring's rate and of any other
    /// joint's rate over it (rad/s), and the rods' length rate after it (m/s).
    pub trial: (f64, f64, f64),
}

/// The angular impulse (N m s) of the inputs' first-order hold over [a, b] for actuator `j`, on
/// the real sample times (held at the ends).
fn input_impulse(t: &Traj, j: usize, a: f64, b: f64) -> f64 {
    let n = t.t.len();
    let u = |time: f64| -> f64 {
        if time <= t.t[0] { return t.u[0][j]; }
        if time >= t.t[n - 1] { return t.u[n - 1][j]; }
        let k = t.t.partition_point(|&x| x <= time) - 1;
        let s = (time - t.t[k]) / (t.t[k + 1] - t.t[k]);
        t.u[k][j] * (1.0 - s) + t.u[k + 1][j] * s
    };
    // Exact for a piecewise-linear input: trapezoids between the knots inside [a, b]
    let mut pts = vec![a];
    pts.extend(t.t.iter().cloned().filter(|&x| x > a && x < b));
    pts.push(b);
    pts.windows(2).map(|w| 0.5 * (u(w[0]) + u(w[1])) * (w[1] - w[0])).sum()
}

impl Cassie {
    /// Applies each actuator's angular impulse over [a, b] as an equal and opposite pair on the
    /// joint's child and parent, about the joint's axis.
    pub fn apply_inputs(&mut self, t: &Traj, a: f64, b: f64) {
        if !self.inputs { return; }
        for (k, name) in ACTUATORS.iter().enumerate() {
            let imp = input_impulse(t, k, a, b);
            let j = self.urdf.joint(name).unwrap().clone();
            let axis = self.model.link_frame(&self.world, &j.child).unwrap().rot * j.axis * imp;
            let v = Vec3::new(axis.x as f32, axis.y as f32, axis.z as f32);
            let (cb, _) = self.model.links[&j.child];
            let (pb, _) = self.model.links[&j.parent];
            let (c, pa) = (self.model.bodies[cb].id, self.model.bodies[pb].id);
            self.world.apply_torque(c, v);
            self.world.apply_torque(pa, -v);
        }
    }
}

/// The 22 velocity coordinates of a row (pelvis angular, pelvis origin linear, 16 joint rates).
const VEL: std::ops::Range<usize> = 23..45;

/// Replays trajectory `t` at `hz` from the start of `line`, recording at the data's sample times.
pub fn replay(urdf: &Urdf, t: &Traj, p: &Params, hz: f64, line: Line) -> Result<Replay, String> {
    replay_with(urdf, t, p, hz, line, Correction::Gated)
}

/// One trial step of length `h` from `lc`, contact off: the state after it.
pub fn trial_step(urdf: &Urdf, t: &Traj, p: &Params, lc: &[f64; 45], h: f64) -> Result<(Cassie, [f64; 45]), String> {
    let mut trial = build(urdf, lc, &Params { toes: false, ..*p }, false)?;
    trial.apply_inputs(t, t.t[0], t.t[0] + h);
    trial.world.step(h as f32);
    let after = trial.state();
    Ok((trial, after))
}

pub fn replay_with(urdf: &Urdf, t: &Traj, p: &Params, hz: f64, line: Line, corr: Correction) -> Result<Replay, String> {
    let h = 1.0 / hz;
    let (lc, _) = start_row(urdf, t, p)?;
    // The half-step correction from a trial step on the loop-consistent state, contact off
    let (trial, after) = trial_step(urdf, t, p, &lc, h)?;
    let mut start = lc;
    match corr {
        Correction::Off => {}
        Correction::Trial => { for i in VEL { start[i] = lc[i] - 0.5 * (after[i] - lc[i]); } }
        Correction::Fine(hf) => {
            let (_, fine) = trial_step(urdf, t, p, &lc, hf)?;
            for i in VEL { start[i] = lc[i] - 0.5 * (h / hf) * (fine[i] - lc[i]); }
        }
        Correction::Gated => {
            let hf = 1.0 / 2000.0;
            let (_, fine) = trial_step(urdf, t, p, &lc, hf)?;
            let want = h / hf;
            for i in VEL {
                let (dc, df) = (after[i] - lc[i], fine[i] - lc[i]);
                let smooth = df.abs() > 1e-12 && ((dc / df) / want - 1.0).abs() <= 0.25;
                if smooth { start[i] = lc[i] - 0.5 * dc; }
            }
        }
    }
    let d_ankle = (after[41] - lc[41]).abs().max((after[43] - lc[43]).abs());
    let d_other = (29..45).filter(|&i| i != 41 && i != 43).map(|i| (after[i] - lc[i]).abs()).fold(0.0, f64::max);
    let rods = trial.rod_rates();
    let trial_info = (d_ankle, d_other, rods[0].abs().max(rods[1].abs()));
    if line == Line::Sensitivity { start[41] = 0.0; start[43] = 0.0; }
    // Positions stay the data's; only the rates are corrected
    let mut c = build(urdf, &start, p, false)?;
    let t0 = t.t[0];
    let end = t.t[t.t.len() - 1] - t0;
    let steps = (end / h).ceil() as usize + 1;
    let mut times = vec![0.0];
    let mut states = vec![c.state()];
    let mut first_contact = None;
    for k in 0..steps {
        let (a, b) = (t0 + k as f64 * h, t0 + (k + 1) as f64 * h);
        c.apply_inputs(t, a, b);
        c.world.step(h as f32);
        times.push(b - t0);
        states.push(c.state());
        if first_contact.is_none() && c.toe_heights().iter().any(|&z| z <= p.toe_r as f64) { first_contact = Some(b - t0); }
    }
    // Linear interpolation to the sample times (the quaternion normalised, its sign aligned)
    let out = t.t.iter().map(|&tk| {
        let x = tk - t0;
        let i = times.partition_point(|&s| s <= x).clamp(1, times.len() - 1);
        let s = ((x - times[i - 1]) / (times[i] - times[i - 1])).clamp(0.0, 1.0);
        let (a, b) = (&states[i - 1], &states[i]);
        let mut r = [0.0; 45];
        let sign = if (0..4).map(|j| a[j] * b[j]).sum::<f64>() < 0.0 { -1.0 } else { 1.0 };
        for j in 0..45 { r[j] = a[j] * (1.0 - s) + if j < 4 { sign } else { 1.0 } * b[j] * s; }
        let n = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]).sqrt();
        for j in 0..4 { r[j] /= n; }
        r
    }).collect();
    Ok(Replay { states: out, first_contact, trial: trial_info })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The loss reproduces cassie_loss_utils on synthetic cases: equal trajectories give 0, and
    /// known offsets give their known values (per term, averaged over the samples).
    #[test]
    fn the_loss_matches_the_papers_on_synthetic_cases() {
        let mut row = [0.0f64; 45];
        row[0] = 1.0;
        let data = vec![row; 100];
        assert_eq!(loss(&data, &data), 0.0);
        let shifted = |f: &dyn Fn(&mut [f64; 45])| { let mut r = row; f(&mut r); vec![r; 100] };
        // Pelvis x 1 cm off: 10 x 1e-4
        assert!((loss(&shifted(&|r| r[4] = 0.01), &data) - 1e-3).abs() < 1e-12);
        // Pelvis vz 0.1 m/s off: 100 x 0.01; knee spring rate 1 rad/s off: 0.01; ankle spring: 0
        assert!((loss(&shifted(&|r| r[28] = 0.1), &data) - 1.0).abs() < 1e-12);
        assert!((loss(&shifted(&|r| r[37] = 1.0), &data) - 0.01).abs() < 1e-12);
        assert_eq!(loss(&shifted(&|r| r[41] = 1.0), &data), 0.0);
        // Pelvis omega 0.1 rad/s off: 5 x 0.01
        assert!((loss(&shifted(&|r| r[24] = 0.1), &data) - 0.05).abs() < 1e-12);
        // Orientation turned 0.1 rad: 0.01
        let q = DQuat::from_rotation_z(0.1);
        assert!((loss(&shifted(&|r| { r[0] = q.w; r[3] = q.z; }), &data) - 0.01).abs() < 1e-9);
        // Half the samples off: half the value (a mean, not a sum)
        let mut half = data.clone();
        for r in half.iter_mut().take(50) { r[4] = 0.01; }
        assert!((loss(&half, &data) - 5e-4).abs() < 1e-12);
    }

    fn here() -> bool { data_dir().join("x_00.npy").exists() && Path::new(DATA_ROOT).join("cassie_v2.urdf").exists() }

    #[test]
    fn reads_the_trajectories() {
        if !here() { eprintln!("skipped: the Cassie data isn't on this machine"); return; }
        for i in 0..ALL_JUMPS {
            let t = load_traj(&data_dir(), i).unwrap();
            assert_eq!(t.x.len(), 100, "trajectory {}", i);
            assert!(t.t.windows(2).all(|w| w[1] > w[0]), "trajectory {}: times not increasing", i);
            // The unmeasured ankle-spring rates are 0
            assert!(t.x.iter().all(|r| r[41] == 0.0 && r[43] == 0.0), "trajectory {}", i);
        }
    }

    /// The loop-consistent start: each rod's length rate is under 1 mm/s, from up to 0.51 m/s
    /// with the ankle springs' rate at 0.
    #[test]
    fn the_loop_consistent_start_closes_the_rods_rate() {
        if !here() { eprintln!("skipped: the Cassie data isn't on this machine"); return; }
        let urdf = load_urdf().unwrap();
        let p = Params::default();
        let (mut worst, mut biggest) = (0.0f64, 0.0f64);
        for i in 0..ALL_JUMPS {
            let t = load_traj(&data_dir(), i).unwrap();
            let (row, found) = loop_consistent(&urdf, &t.x[0], &p).unwrap();
            let r = build(&urdf, &row, &p, false).unwrap().rod_rates();
            worst = worst.max(r[0].abs()).max(r[1].abs());
            biggest = biggest.max(found[0].abs()).max(found[1].abs());
        }
        println!("loop-consistent start: worst rod rate {:.2e} m/s; ankle-spring rates up to {:.2} rad/s", worst, biggest);
        assert!(worst < 1e-3, "rod rate {:.2e} m/s", worst);
    }

    /// A dataset row built into bodies and read back gives the row: joint angles and rates, the
    /// pelvis origin's pose and velocity (catches the column order, z-up to y-up, and COM
    /// against origin). The tolerances are f32 storage of positions near 1 m and velocities
    /// near a few m/s.
    #[test]
    fn the_kinematics_round_trip() {
        if !here() { eprintln!("skipped: the Cassie data isn't on this machine"); return; }
        let urdf = load_urdf().unwrap();
        let p = Params::default();
        let mut worst = [0.0f64; 6];
        for i in 0..ALL_JUMPS {
            let t = load_traj(&data_dir(), i).unwrap();
            for s in [0usize, 50, 99] {
                let row = &t.x[s];
                let c = build(&urdf, row, &p, false).unwrap();
                let back = c.state();
                let mut qn = row[0..4].to_vec();
                let n = qn.iter().map(|v| v * v).sum::<f64>().sqrt();
                qn.iter_mut().for_each(|v| *v /= n);
                let e = [
                    quat_angle(&qn, &back[0..4]),
                    (0..3).map(|k| (row[4 + k] - back[4 + k]).abs()).fold(0.0, f64::max),
                    (7..23).map(|k| (row[k] - back[k]).abs()).fold(0.0, f64::max),
                    (23..26).map(|k| (row[k] - back[k]).abs()).fold(0.0, f64::max),
                    (26..29).map(|k| (row[k] - back[k]).abs()).fold(0.0, f64::max),
                    (29..45).map(|k| (row[k] - back[k]).abs()).fold(0.0, f64::max),
                ];
                for k in 0..6 { worst[k] = worst[k].max(e[k]); }
            }
        }
        println!("round trip, worst: orientation {:.2e} rad, pelvis {:.2e} m, joints {:.2e} rad, pelvis w {:.2e} rad/s, v {:.2e} m/s, joint rates {:.2e} rad/s",
                 worst[0], worst[1], worst[2], worst[3], worst[4], worst[5]);
        assert!(worst[0] < 1e-5 && worst[1] < 1e-5 && worst[2] < 1e-4, "poses: {:?}", worst);
        assert!(worst[3] < 1e-4 && worst[4] < 1e-4 && worst[5] < 1e-3, "rates: {:?}", worst);
    }
}
