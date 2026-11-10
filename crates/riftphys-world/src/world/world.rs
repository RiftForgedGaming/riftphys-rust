// SPDX-License-Identifier: RiftForged Proprietary License
use std::collections::BTreeMap;

use riftphys_controllers::{BalanceCtrl, BalanceParams, GuardCtrl};
use riftphys_core::{
    epoch_id, BodyId, ColliderId, EpochDescriptor, EpochId, Isometry,
    JointId, Quat, Scalar, StepCtx, Vec3, Velocity, XorShift64,
};
use riftphys_core::models::{AeroHandle, AeroQuery, ModelRegistry, PropHandle, PropQuery};

use riftphys_articulation::bodies::{D6Joint, Joints, D6Drive, D6Limit, JointKind};
use riftphys_dynamics::dynamics::{Bodies, BodyDesc};
use riftphys_geom::{aabb_of, Aabb, Shape};
use riftphys_geom::MassProps;
use riftphys_gravity::{eval as grav_eval, spec_id as gravity_epoch_id, GravitySpec};
use riftphys_viz::viz::{DebugSettings, Ledger, LedgerEvent, ScheduleRecorder};

use riftphys_collision::environment::SpatialEnvironment;
use riftphys_collision::broadphase::GridBroadphase;

use crate::world::sleep::SleepManager;
use super::islands::IslandManager;
use super::voxel_env::VoxelEnvironment;
use riftphys_vox::VoxelChunk;

pub const TERRAIN_CI: usize = u32::MAX as usize;
pub(super) const CCD_MAX_MARGIN: f32 = 1.0e-0;

/* ---------------- Controllers ---------------- */

pub(super) struct GuardInstance {
    pub(super) joint: JointId,
    pub(super) eff: BodyId,
    pub(super) ctrl: GuardCtrl,
}

pub(super) struct BalanceInstance {
    pub(super) pelvis: BodyId,
    pub(super) left: BodyId,
    pub(super) right: BodyId,
    pub(super) ctrl: BalanceCtrl,
}

#[derive(Clone, Debug)]
pub struct PlayerController {
    pub body: BodyId,
    pub radius: f32,
    pub height: f32,
    pub speed: f32,
    pub jump_impulse: f32,
    pub grounded: bool,
    pub vertical_vel: f32,
    pub input_dir: Vec3,
}

impl PlayerController {
    pub fn new(body: BodyId) -> Self {
        Self {
            body,
            radius: 0.4,
            height: 1.8,
            speed: 5.0,
            jump_impulse: 5.0,
            grounded: false,
            vertical_vel: 0.0,
            input_dir: Vec3::ZERO,
        }
    }
}

/* ---------------- Collider & Contact ---------------- */

#[derive(Copy, Clone, Debug)]
pub struct Collider {
    pub body: BodyId,
    pub shape: Shape,
    pub aabb: Aabb,
    pub material: riftphys_materials::materials::Material,
    /// The shape's pose in its body's frame (PARITY_SPEC W4d); `None` is the body's own pose.
    pub local: Option<Isometry>,
}

impl Collider {
    /// The shape's pose in the world, given its body's: every read of a collider's world pose
    /// goes through here. With no offset it is `body` itself, bit for bit.
    #[inline]
    pub fn world_pose(&self, body: Isometry) -> Isometry {
        match self.local {
            None => body,
            Some(l) => Isometry { pos: body.pos + body.rot * l.pos, rot: (body.rot * l.rot).normalize() },
        }
    }

    /// How far the shape reaches from its body's COM, at most: its offset plus its own AABB's
    /// half-diagonal (the swept broadphase turns this into |omega| x reach x dt).
    #[inline]
    pub fn reach(&self) -> f32 {
        let half = 0.5 * (self.aabb.max - self.aabb.min).length();
        match self.local { None => half, Some(l) => l.pos.length() + half }
    }
}

/// One step's bounce passes (PARITY_SPEC W6), for tests and traces.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct BounceStats {
    /// Contacts bounced at their time of impact, support rows beside them, pseudo rows
    pub toi_rows: usize,
    pub support_rows: usize,
    pub pseudo_rows: usize,
    /// Direct solves the velocity pass used (the most any component took) and the pseudo pass's
    /// (1); 0 above the direct limit
    pub solves: usize,
    pub pseudo_solves: usize,
    /// Gauss-Seidel sweeps (each at most 256), only above the direct limit or if the velocity
    /// pass's direct solve didn't converge; else 0
    pub sweeps: usize,
    pub pseudo_sweeps: usize,
    /// Connected components the velocity pass solved separately
    pub components: usize,
    /// Components that fell back to sweeping, in either pass (above the direct limit, or not
    /// converged): 0 in every suite
    pub fallbacks: usize,
    /// The velocity pass's largest component: its rows, and the entries of K and of L below the
    /// diagonal (the sparse factor's fill)
    pub largest: usize,
    pub k_nnz: usize,
    pub l_nnz: usize,
    /// Wall time of both bounce passes on a bounce step, in microseconds (a diagnostic: it never
    /// feeds the simulation)
    pub pass_us: u64,
    /// With the dense reference on (tests only): the first Newton system of each velocity solve
    /// (well conditioned), sparse against dense, the worst difference relative to the largest
    pub dense_check: f64,
    /// The worst normwise backward error of any bounce-pass solve this step (infinity norms):
    /// a backward-stable factor gives about 1e-15, however ill-conditioned the system
    pub backward_error: f64,
    /// The largest velocity error left on a pushing row, against the unregularised K (m/s): the
    /// compliance's cost, about 1e-9 of the velocity change
    pub true_residual: f64,
    /// The pseudo solve's largest residual (m/s)
    pub residual: f32,
    /// The fastest mid-step approach of any rigid contact (m/s, positive approaching); bounces
    /// need more than 1
    pub max_approach: f32,
}

#[derive(Copy, Clone, Debug)]
pub struct Contact {
    pub a_collider: usize,
    pub b_collider: usize,
    pub point: Vec3,
    pub normal: Vec3,
    pub depth: Scalar,
    pub env_material: Option<riftphys_materials::materials::MaterialId>,
    pub impulse: f32,
}

#[derive(Copy, Clone, Debug)]
pub struct RayHit {
    pub body: BodyId,
    pub toi: f32,
    pub point: Vec3,
    pub normal: Vec3,
}

#[derive(Copy, Clone, Debug)]
pub struct SweepHit {
    pub body: BodyId,
    pub toi: f32,
    pub started_overlapping: bool,
    pub point: Vec3,
    pub normal: Vec3,
}

/// How the position-level joint solve chooses the directions it corrects along. Errors are
/// always measured at the current iterate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum JointProjection {
    /// Directions at the current iterate (standard PBD/XPBD). Dissipates at first order in dt:
    /// a hinged pendulum keeps about 90% of its swing energy per period at 62.5 Hz.
    Pbd,
    /// Directions at the start of the step, the configuration the velocity solve used (SHAKE).
    /// Symplectic: free joints keep their energy (no secular drift). The default.
    #[default]
    Shake,
}

/* ---------------- Builder ---------------- */

pub struct WorldBuilder {
    pub bodies: usize,
    pub colliders: usize,
}

impl WorldBuilder {
    pub fn new() -> Self { Self { bodies: 128, colliders: 128 } }
    pub fn with_capacity(mut self, bodies: usize, colliders: usize) -> Self {
        self.bodies = bodies;
        self.colliders = colliders;
        self
    }
    pub fn build(self) -> World { World::with_capacity(self.bodies, self.colliders) }
}

#[derive(Copy, Clone, Debug)]
pub(super) struct AccelComp {
    pub(super) aero: Option<AeroHandle>,
    pub(super) prop: Option<PropHandle>,
    pub(super) ref_area_m2: f32,
    pub(super) throttle01: f32,
    pub(super) forward_dir_world: Option<Vec3>,
}

/// Key of a warm-start entry (AT_REST_SPEC R1): (body A, body B, collider ordinal within A,
/// collider ordinal within B). Terrain is B = `u32::MAX`, ordinal 0. Body ids, not collider
/// indices: `remove_body` shifts every later collider index, and body ids are never reused.
pub(super) type WarmKey = (u32, u32, u32, u32);

/// One contact's impulses at the end of last step's iterations (before the shock pass).
#[derive(Copy, Clone, Debug, PartialEq)]
pub(super) struct WarmContact {
    /// Contact point in body A's frame, at the pose the solve ran at (matching tolerance applies here)
    pub(super) la: Vec3,
    pub(super) jn: f32,
    /// Friction impulse as a world vector
    pub(super) jt: Vec3,
}

/// One friction patch's accumulators: sliding impulse (world vector) and torsion about the normal.
#[derive(Copy, Clone, Debug, PartialEq)]
pub(super) struct WarmPatch {
    pub(super) jt: Vec3,
    pub(super) tw: f32,
}

/// One D6 joint row's impulse at the end of last step's iterations, with its row tag.
#[derive(Copy, Clone, Debug, PartialEq)]
pub(super) struct WarmRow {
    pub(super) tag: u8,
    pub(super) lam: f32,
}

/// State a step reads that body poses and velocities don't hold: the contact warm-start cache,
/// the ballistic update's last velocities and the sleep state. A rollback snapshot must carry it
/// (AT_REST_SPEC D2); restore it after setting poses and velocities, which overwrite parts of it.
#[derive(Clone, Debug, Default)]
pub struct CarriedState {
    warm: BTreeMap<WarmKey, Vec<WarmContact>>,
    warm_patches: BTreeMap<WarmKey, WarmPatch>,
    warm_joints: BTreeMap<u32, Vec<WarmRow>>,
    last_lin_vel: Vec<Vec3>,
    awake: Vec<u8>,
    sleep_ticks: Vec<u16>,
    rotor: BTreeMap<(u32, u8), f32>,
}

/* ---------------- World ---------------- */

pub struct World {
    pub gravity: Vec3,
    pub epoch_id: u64,
    pub rng: XorShift64,

    pub(super) schedule: ScheduleRecorder,
    pub bodies: Bodies,
    pub colliders: Vec<Collider>,
    pub(super) island_manager: IslandManager,
    pub(super) broadphase: GridBroadphase,
    pub(super) sleep: SleepManager,
    pub(super) pending_epoch: Option<EpochDescriptor>,
    pub(super) pending_gravity: Option<GravitySpec>,
    pub(super) gravity_proc: GravitySpec,

    pub(super) joints: Joints,
    /// (velocity, position) solver iterations per step; see `set_solver_iterations`
    /// Diagnostic (PARITY_SPEC W4e): damp implicit compliant contacts at the step-start
    /// penetration instead of the predicted mid-step one.
    pub(super) compliant_damping_at_start: bool,
    pub(super) solver_iters: (u32, u32),
    /// How the position-level joint solve chooses its correction directions
    pub(super) joint_projection: JointProjection,
    /// Velocities at the start of the current step, before forces (restitution uses the
    /// approach speed, not one that already includes this step's gravity)
    pub(super) pre_step_vel: Vec<Velocity>,
    /// Per-step outputs of the contact solve (PARITY_SPEC W6), never carried or hashed:
    /// bodies whose contacts carried load this step, the split-impulse pseudo-velocity
    /// (linear, angular) that places a bounced free body where its time of impact puts it,
    /// and the pseudo solve's last residual (m/s).
    pub(super) contact_loaded: Vec<bool>,
    pub(super) bounce_pseudo: Vec<(Vec3, Vec3)>,
    /// A diagnostic record (PARITY_SPEC 3.4g): when Some, each step stores every body's pose after
    /// integration and before the position pass. Read-only; it never changes the step.
    pub(super) diag_predicted: Option<Vec<Isometry>>,
    /// W4f's rotor row in the position pass (on by default; off reproduces W4b alone, for the
    /// mutation checks and before/after reports, PARITY_SPEC 3.4h).
    pub(super) rotor_position_row: bool,
    pub(super) last_bounce_residual: f32,
    pub(super) last_bounce_stats: BounceStats,
    /// The W6 bounce passes' direct-solve limit, in rows per component (`DIRECT_MAX`)
    pub(super) bounce_direct_max: usize,
    /// Tests only: also solve the first Newton system of each velocity solve densely, and report
    /// the difference from the sparse solve (`BounceStats::dense_check`)
    pub(super) bounce_dense_reference: bool,
    /// The last step's velocity-pass impulse changes, per row, components in order (tests only)
    pub(super) last_bounce_x: Vec<f64>,
    /// Which bodies moved ballistically last step (W6 H2), for tracing
    pub(super) last_ballistic: Vec<bool>,
    /// Measured contact properties per material pair, replacing the built-in table (set from
    /// fitting real data; see riftphys-reality). Keys are (min, max) by MaterialId order.
    /// Linear velocity at the end of each body's last step (or as last set): the start of the
    /// exact ballistic update. Impulses applied between steps are forces over the next step.
    pub(super) last_lin_vel: Vec<Vec3>,
    pub(super) pair_overrides: Vec<((riftphys_materials::materials::MaterialId, riftphys_materials::materials::MaterialId), riftphys_materials::materials::MatPairProps)>,
    pub(super) tick: u64,
    pub(super) debug: DebugSettings,
    pub ledger: Ledger,

    pub environments: Vec<Box<dyn SpatialEnvironment>>,

    pub(super) guards: Vec<GuardInstance>,
    pub(super) models: ModelRegistry,
    pub(super) accel_comps: Vec<Option<AccelComp>>,
    pub(super) balances: Vec<BalanceInstance>,

    pub players: Vec<PlayerController>,

    pub last_frame_contacts: Vec<Contact>,
    /// Largest batch (contact level, joint level) the last solve ran; batches of `PAR_MIN` (64)
    /// or more run in parallel. Lets tests prove they reach the parallel solver paths.
    pub(super) last_max_batch: (usize, usize),

    /// Contact warm start (AT_REST_SPEC R1): last step's contacts per key, in contact order
    pub(super) warm: BTreeMap<WarmKey, Vec<WarmContact>>,
    /// Armature rotors' velocities, per (joint, rotational axis), carried between steps
    /// (PARITY_SPEC W4b). Physical state, not a cache: `clear_warm` leaves it.
    pub(super) rotor: BTreeMap<(u32, u8), f32>,
    /// Joints whose rotors `set_body_pose` restarted since the last step: a `set_body_vel` on
    /// their bodies before the next step restarts them again, so pose-then-velocity resets
    /// don't leave the rotor at the old speed. Cleared each step; not carried state.
    pub(super) rotor_pending: std::collections::BTreeSet<u32>,
    pub(super) warm_patches: BTreeMap<WarmKey, WarmPatch>,
    /// D6 joint rows' impulses per joint index, for warm starting
    pub(super) warm_joints: BTreeMap<u32, Vec<WarmRow>>,
    /// Contacts the last solve matched to a warm entry
    pub(super) last_warm_matched: usize,
    pub(super) last_normal_impulse: Vec<f32>,

    pub(super) prov_impulses_sum: f32,
    pub(super) prov_ccd_hits: u32,
    pub(super) prov_aero_sum: f32,
    pub(super) prov_prop_sum: f32,
    pub(super) alive: Vec<u8>,
}

impl World {
    #[inline]
    pub(super) fn record(&mut self, e: LedgerEvent) {
        use LedgerEvent::*;
        match e {
            CCDHit { .. } => self.prov_ccd_hits = self.prov_ccd_hits.wrapping_add(1),
            ImpulseN { jn, .. } => self.prov_impulses_sum += jn.abs(),
            ImpulseT { jt1, jt2, .. } => self.prov_impulses_sum += (jt1 * jt1 + jt2 * jt2).sqrt(),
            AeroProp { t_n, d_n, .. } => { self.prov_prop_sum += t_n; self.prov_aero_sum += d_n; }
            BalanceAccel { ax, az, .. } => self.prov_prop_sum += (ax * ax + az * az).sqrt(),
            _ => {}
        }
        self.ledger.push(e);
    }

    pub fn total_impulse_sum(&self) -> f32 { self.prov_impulses_sum }
    pub fn num_colliders(&self) -> Vec<Collider> { self.colliders.clone() }
    pub fn num_broadphase_pairs(&self) -> usize { self.broadphase.get_pairs() }
    pub fn count_capsules(&self) -> usize {
        self.colliders.iter().filter(|c| matches!(c.shape, Shape::Capsule { .. })).count()
    }

    #[inline]
    pub(super) fn reset_provenance(&mut self) {
        self.prov_impulses_sum = 0.0;
        self.prov_ccd_hits = 0;
        self.prov_aero_sum = 0.0;
        self.prov_prop_sum = 0.0;
    }

    #[inline]
    pub(super) fn gather_box_aabbs_with_map(&self) -> (Vec<Aabb>, Vec<usize>) {
        let mut aabbs: Vec<Aabb> = Vec::new();
        let mut map: Vec<usize> = Vec::new();
        for (ci, c) in self.colliders.iter().enumerate() {
            if let Shape::Box { .. } = c.shape {
                aabbs.push(c.aabb);
                map.push(ci);
            }
        }
        (aabbs, map)
    }

    pub fn num_bodies(&self) -> u32 { self.bodies.len() as u32 }
    pub fn inv_mass_of(&self, id: BodyId) -> f32 { self.bodies.inv_mass_of(id.0) }
    pub fn primary_shape(&self, body: BodyId) -> Option<Shape> { self.colliders.iter().find(|c| c.body == body).map(|c| c.shape) }
    #[inline]
    pub fn tick_index(&self) -> u64 { self.tick }
    pub fn for_each_collider<F: FnMut(u32, BodyId, &Shape, &Aabb)>(&self, mut f: F) {
        for (i, c) in self.colliders.iter().enumerate() { f(i as u32, c.body, &c.shape, &c.aabb); }
    }
    pub fn body_pose(&self, id: BodyId) -> Isometry { self.bodies.pose(id.0) }

    pub fn with_capacity(bodies: usize, colliders: usize) -> Self {
        let g = Vec3::new(0.0, -9.81, 0.0);
        Self {
            gravity: g,
            epoch_id: 0,
            rng: XorShift64::new(0xC0FFEE),
            schedule: ScheduleRecorder::new(),
            bodies: Bodies::with_capacity(bodies),
            colliders: Vec::with_capacity(colliders),
            island_manager: IslandManager::new(bodies),
            sleep: SleepManager::new(bodies),
            broadphase: GridBroadphase::new(2.5),
            pending_epoch: None,
            pending_gravity: None,
            gravity_proc: GravitySpec::Uniform { g: [g.x, g.y, g.z] },
            joints: Joints::new(),
            compliant_damping_at_start: false,
            solver_iters: (20, 20),
            joint_projection: JointProjection::default(),
            pre_step_vel: Vec::new(),
            contact_loaded: Vec::new(),
            bounce_pseudo: Vec::new(),
            diag_predicted: None,
            rotor_position_row: true,
            last_bounce_residual: 0.0,
            last_bounce_stats: BounceStats::default(),
            bounce_direct_max: super::solver::DIRECT_MAX,
            bounce_dense_reference: false,
            last_bounce_x: Vec::new(),
            last_ballistic: Vec::new(),
            pair_overrides: Vec::new(),
            last_lin_vel: Vec::new(),
            tick: 0,
            debug: DebugSettings::default(),
            ledger: Ledger::new(4096),
            environments: Vec::new(),
            guards: Vec::new(),
            models: ModelRegistry::new(),
            accel_comps: vec![None; bodies],
            balances: Vec::new(),
            players: Vec::new(),
            last_frame_contacts: Vec::with_capacity(colliders * 4),
            last_max_batch: (0, 0),
            warm: BTreeMap::new(),
            rotor: BTreeMap::new(),
            rotor_pending: std::collections::BTreeSet::new(),
            warm_patches: BTreeMap::new(),
            warm_joints: BTreeMap::new(),
            last_warm_matched: 0,
            last_normal_impulse: vec![0.0; bodies],
            prov_impulses_sum: 0.0,
            prov_ccd_hits: 0,
            prov_aero_sum: 0.0,
            prov_prop_sum: 0.0,
            alive: vec![0; bodies],
        }
    }

    pub fn add_player(&mut self, pose: Isometry) -> usize {
        let id = self.add_body(pose, Velocity::default(), MassProps::infinite(), false);
        let pc = PlayerController::new(id);
        let hh = pc.height * 0.5;
        let r = pc.radius;
        self.add_collider(id, Shape::Capsule { r, hh }, riftphys_materials::materials::material(riftphys_materials::materials::MaterialId::Default));
        let idx = self.players.len();
        self.players.push(pc);
        idx
    }

    pub fn add_environment(&mut self, env: Box<dyn SpatialEnvironment>) { self.environments.push(env); }
    pub fn clear_environments(&mut self) { self.environments.clear(); }
    pub fn add_voxel_chunk(&mut self, chunk: VoxelChunk, origin: Vec3) {
        let env = Box::new(VoxelEnvironment::new(chunk, origin));
        self.environments.push(env);
    }
    pub fn add_balance_controller(&mut self, pelvis: BodyId, left: BodyId, right: BodyId, params: BalanceParams) {
        self.balances.push(BalanceInstance { pelvis, left, right, ctrl: BalanceCtrl::new(params) });
    }

    pub fn models_mut(&mut self) -> &mut ModelRegistry { &mut self.models }
    pub fn models(&self) -> &ModelRegistry { &self.models }

    pub fn set_body_accel(
        &mut self, body: BodyId, aero: Option<AeroHandle>, prop: Option<PropHandle>,
        ref_area_m2: f32, throttle01: f32, forward_dir_world: Option<Vec3>,
    ) {
        let i = body.0 as usize;
        if self.accel_comps.len() <= i { self.accel_comps.resize(i + 1, None); }
        self.accel_comps[i] = Some(AccelComp { aero, prop, ref_area_m2, throttle01, forward_dir_world });
    }

    /// Moves a body. Its warm-start entries are dropped: anchors are matched in body A's frame,
    /// so a teleported body would otherwise be warm-started with its old place's load.
    pub fn set_body_pose(&mut self, id: BodyId, pose: Isometry) {
        self.drop_warm_of(id.0);
        self.bodies.set_pose(id.0, pose);
        for c in &mut self.colliders {
            if c.body == id { c.aabb = aabb_of(&c.shape, &c.world_pose(pose)); }
        }
        // Armature rotors on this body's joints restart from the links (PARITY_SPEC W4b)
        let with_rotor: Vec<u32> = self.joints.kinds().iter().enumerate().filter_map(|(k, j)| match j {
            JointKind::D6(d) if (d.a == id || d.b == id) && d.r.iter().any(|a| a.armature > 0.0) => Some(k as u32),
            _ => None,
        }).collect();
        for k in with_rotor { self.rotor_init(k); self.rotor_pending.insert(k); }
    }

    pub fn yaw_body(&mut self, id: BodyId, dy: f32) {
        if dy == 0.0 { return; }
        let mut p = self.bodies.pose(id.0);
        let dq = Quat::from_rotation_y(dy);
        p.rot = (dq * p.rot).normalize();
        self.set_body_pose(id, p);
    }

    pub fn set_body_throttle(&mut self, body: BodyId, t: f32) {
        if let Some(Some(ac)) = self.accel_comps.get_mut(body.0 as usize) { ac.throttle01 = t.clamp(0.0, 1.0); }
    }

    pub fn add_guard_controller(&mut self, pivot: BodyId, eff: BodyId, params: riftphys_controllers::GuardParams) -> JointId {
        let j = self.joints.add_distance_joint(pivot, eff, params.rest_guard, params.k_guard);
        self.guards.push(GuardInstance { joint: j, eff, ctrl: GuardCtrl::new(params) });
        j
    }

    pub fn set_debug(&mut self, cfg: DebugSettings) { self.debug = cfg; }
    pub fn get_body_pose(&self, id: BodyId) -> Isometry { self.bodies.pose(id.0) }
    pub fn get_body_vel(&self, id: BodyId) -> Velocity { self.bodies.vel(id.0) }

    pub fn add_distance_joint(&mut self, a: BodyId, b: BodyId, rest: Scalar, compliance: Scalar) -> JointId {
        self.joints.add_distance_joint(a, b, rest, compliance)
    }

    /// A distance joint between anchor `la` on `a` and `lb` on `b`, each in its body's frame
    /// (PARITY_SPEC W4c): the rod passes torque through its lever arms. `compliance` in m/N,
    /// 0 for rigid.
    pub fn add_distance_joint_at(&mut self, a: BodyId, la: Vec3, b: BodyId, lb: Vec3, rest: Scalar, compliance: Scalar) -> JointId {
        self.joints.add_distance_joint_at(a, la, b, lb, rest, compliance)
    }

    /// Damping along a compliant distance joint, N s/m: an implicit velocity row, stable for
    /// any value. A rigid joint ignores it (it already holds the rod's rate at 0).
    pub fn set_distance_damping(&mut self, id: JointId, damping: Scalar) {
        self.joints.set_distance_damping(id, damping);
    }

    pub fn set_gravity(&mut self, g: Vec3) {
        self.gravity = g;
        self.gravity_proc = GravitySpec::Uniform { g: [g.x, g.y, g.z] };
    }

    pub fn set_body_sleep_allowed(&mut self, id: BodyId, allowed: bool) {
        let idx = id.0 as usize;
        self.sleep.ensure_capacity(idx);
        self.sleep.sleep_allowed[idx] = allowed;
        if !allowed { self.sleep.wake(idx); }
    }

    pub fn set_epoch(&mut self, epoch: u64) { self.epoch_id = epoch; }
    pub fn set_rng_seed(&mut self, seed: u64) { self.rng = XorShift64::new(seed); }
    pub fn queue_epoch_swap(&mut self, desc: EpochDescriptor) { self.pending_epoch = Some(desc); }
    pub fn queue_gravity_swap(&mut self, spec: GravitySpec) { self.pending_gravity = Some(spec); }

    pub(super) fn apply_pending_epoch_if_any(&mut self) {
        if let Some(spec) = self.pending_gravity.take() {
            self.epoch_id = gravity_epoch_id(&spec);
            self.gravity_proc = spec;
            if let GravitySpec::Uniform { g } = spec { self.gravity = Vec3::new(g[0], g[1], g[2]); }
            self.clear_warm();
        }
        if let Some(desc) = self.pending_epoch.take() {
            self.epoch_id = epoch_id(&desc);
            self.set_gravity(Vec3::new(desc.gravity_g[0], desc.gravity_g[1], desc.gravity_g[2]));
            self.clear_warm();
        }
    }

    pub fn num_sleeping(&self) -> usize {
        self.sleep.awake.iter().filter(|a| a.load(std::sync::atomic::Ordering::Relaxed) == 0).count()
    }

    pub fn add_body(&mut self, pose: Isometry, vel: Velocity, mass: MassProps, dynamic: bool) -> BodyId {
        let inv_mass = if dynamic { mass.inv_mass } else { 0.0 };
        // Keep the shape's real inertia (MassProps.inertia is body-frame); the old path dropped
        // it and used diag(inv_mass), so torques ignored a body's size and shape.
        let det = mass.inertia.determinant();
        let inv_inertia_local = (dynamic && det.is_finite() && det > 0.0).then(|| mass.inertia.inverse());
        let id = self.bodies.add(BodyDesc { pose, vel, inv_mass, dynamic, inv_inertia_local });
        if self.last_lin_vel.len() <= id as usize { self.last_lin_vel.resize(id as usize + 1, Vec3::ZERO); }
        self.last_lin_vel[id as usize] = vel.lin;
        let idx = id as usize;
        self.sleep.ensure_capacity(idx);
        self.sleep.wake(idx);
        if self.alive.len() <= idx { self.alive.resize(idx + 1, 0); }
        self.alive[idx] = 1;

        if self.accel_comps.len() <= idx { self.accel_comps.resize(idx + 1, None); }
        if self.last_normal_impulse.len() <= idx { self.last_normal_impulse.resize(idx + 1, 0.0); }

        BodyId(id)
    }

    pub fn add_collider(&mut self, body: BodyId, shape: Shape, material: riftphys_materials::materials::Material) -> ColliderId {
        let pose = self.bodies.pose(body.0);
        let aabb = aabb_of(&shape, &pose);
        let id = self.colliders.len() as u32;
        self.colliders.push(Collider { body, shape, aabb, material, local: None });
        ColliderId(id)
    }

    /// A collider posed at `local` in its body's frame (PARITY_SPEC W4d): a contact point off
    /// the body's COM, such as a foot's toe. Like `add_collider` it never touches mass.
    pub fn add_collider_at(&mut self, body: BodyId, local: Isometry, shape: Shape, material: riftphys_materials::materials::Material) -> ColliderId {
        let id = self.colliders.len() as u32;
        let mut c = Collider { body, shape, aabb: Aabb { min: Vec3::ZERO, max: Vec3::ZERO }, material, local: Some(local) };
        c.aabb = aabb_of(&shape, &c.world_pose(self.bodies.pose(body.0)));
        self.colliders.push(c);
        ColliderId(id)
    }

    #[inline]
    pub(super) fn eval_extra_accel(
        models: &ModelRegistry, bodies: &Bodies, accel_comps: &Vec<Option<AccelComp>>,
        epoch_id: u64, tick: u64, i: u32, pose: Isometry, vel: Velocity, gravity_proc: &GravitySpec, dt: f32,
    ) -> (Vec3, Vec3) {
        let a_grav = grav_eval(gravity_proc, pose.pos);
        let mut a_extra = Vec3::ZERO;

        if let Some(Some(ac)) = accel_comps.get(i as usize) {
            let inv_m = bodies.inv_mass_of(i);
            if inv_m > 0.0 {
                let mass = 1.0 / inv_m;
                let fwd_w = ac.forward_dir_world.unwrap_or_else(|| pose.rot * Vec3::new(1.0, 0.0, 0.0));
                let v_w = vel.lin;
                let speed = v_w.length();
                let vhat = if speed > 1e-6 { v_w / speed } else { fwd_w };
                let cos_a = fwd_w.dot(vhat).clamp(-1.0, 1.0);

                let aq = AeroQuery {
                    vel_world: [v_w.x, v_w.y, v_w.z],
                    ang_vel_world: [vel.ang.x, vel.ang.y, vel.ang.z],
                    orientation: pose.rot,
                    area: ac.ref_area_m2, mass, altitude: pose.pos.y, alpha_rad: cos_a.acos(),
                };

                let pq = PropQuery { throttle01: ac.throttle01, forward_dir_world: [fwd_w.x, fwd_w.y, fwd_w.z], mass };

                if let Some(h) = ac.aero {
                    let a = models.aero(h).accel_contrib(&StepCtx { dt, tick, epoch: EpochId(epoch_id) }, aq);
                    a_extra += Vec3::new(a[0], a[1], a[2]);
                }
                if let Some(h) = ac.prop {
                    let a = models.prop(h).accel_contrib(&StepCtx { dt, tick, epoch: EpochId(epoch_id) }, pq);
                    a_extra += Vec3::new(a[0], a[1], a[2]);
                }
            }
        }
        (a_grav, a_extra)
    }

    pub fn set_body_vel(&mut self, id: BodyId, vel: Velocity) {
        self.bodies.set_vel(id.0, vel);
        if let Some(v) = self.last_lin_vel.get_mut(id.0 as usize) { *v = vel.lin; }
        // A reset (pose, then velocity) restarts the rotors the pose restarted; other velocity
        // writes keep them (PARITY_SPEC W4b)
        if !self.rotor_pending.is_empty() {
            let kinds = self.joints.kinds();
            let again: Vec<u32> = self.rotor_pending.iter().copied().filter(|&k| match kinds.get(k as usize) {
                Some(JointKind::D6(d)) => d.a == id || d.b == id,
                _ => false,
            }).collect();
            for k in again { self.rotor_init(k); }
        }
    }

    pub fn apply_impulse(&mut self, id: BodyId, impulse: Vec3) {
        self.sleep.wake(id.0 as usize);
        self.bodies.apply_impulse(id.0, impulse);
    }

    pub fn body_alive(&self, id: BodyId) -> bool { self.alive.get(id.0 as usize).copied().unwrap_or(0) != 0 }

    pub fn remove_body(&mut self, id: BodyId) -> bool {
        let i = id.0 as usize;
        if i >= self.bodies.len() || !self.body_alive(id) { return false; }

        self.alive[i] = 0;
        let _ = self.bodies.deactivate(id.0);
        self.colliders.retain(|c| c.body != id);

        if i < self.accel_comps.len() { self.accel_comps[i] = None; }
        if i < self.last_normal_impulse.len() { self.last_normal_impulse[i] = 0.0; }

        self.guards.retain(|g| g.eff != id);
        self.balances.retain(|b| b.pelvis != id && b.left != id && b.right != id);
        // Only this body's entries: clearing everything would cold-start every stack in the
        // world whenever debris despawns
        self.drop_warm_of(id.0);
        true
    }

    #[inline]
    pub fn normal_force(&self, body: BodyId, dt: f32) -> f32 { self.last_normal_impulse.get(body.0 as usize).copied().unwrap_or(0.0) / dt }

    pub fn add_ball_joint(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry) -> JointId { self.joints.add_ball(a, b, fa, fb) }
    pub fn add_hinge_joint(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry, hinge_axis: usize) -> JointId { self.joints.add_hinge(a, b, fa, fb, hinge_axis) }
    pub fn add_d6_joint(&mut self, j: D6Joint) -> JointId {
        let id = self.joints.add_d6(j);
        // Armature set on the joint as built: start its rotors now, as set_d6_armature does
        if j.r.iter().any(|a| a.armature > 0.0) { self.rotor_init(id.0); }
        id
    }

    pub fn integrate_forces(&mut self, dt: f32) {
        let count = self.bodies.len() as u32;
        for i in 0..count {
            if !self.body_alive(BodyId(i)) || !self.bodies.is_dynamic(i) { continue; }
            let pose = self.bodies.pose(i);
            let mut vel = self.bodies.vel(i);

            let (a_grav, a_extra) = Self::eval_extra_accel(&self.models, &self.bodies, &self.accel_comps, self.epoch_id, self.tick, i, pose, vel, &self.gravity_proc, dt);
            vel.lin += (a_grav + a_extra) * dt;
            self.bodies.set_vel(i, vel);
        }
    }

    pub fn integrate_positions(&mut self, dt: f32) {
        let count = self.bodies.len() as u32;
        for i in 0..count {
            if !self.body_alive(BodyId(i)) || !self.bodies.is_dynamic(i) { continue; }
            let mut pose = self.bodies.pose(i);
            let mut vel = self.bodies.vel(i);

            // --- THE FIX: Universal Shape Clamping ---
            // Iterate ALL attached shapes to find the absolute minimum
            // geometric thickness for this composite body.
            let mut overall_min_thickness = f32::MAX;
            let mut has_shapes = false;

            for c in &self.colliders {
                if c.body.0 == i {
                    has_shapes = true;
                    let thickness = match c.shape {
                        Shape::Sphere { r } => r,
                        Shape::Capsule { r, .. } => r,
                        Shape::Box { hx, hy, hz } => hx.min(hy).min(hz),
                        _ => 0.1,
                    };
                    if thickness < overall_min_thickness {
                        overall_min_thickness = thickness;
                    }
                }
            }

            if has_shapes {
                let max_dist = overall_min_thickness * 0.9;
                let dist = vel.lin.length() * dt;

                if dist > max_dist {
                    vel.lin = vel.lin * (max_dist / dist);
                    self.prov_ccd_hits = self.prov_ccd_hits.wrapping_add(1);
                }
            }

            pose.pos += vel.lin * dt;
            let omega = vel.ang;
            let dq = Quat::from_xyzw(omega.x, omega.y, omega.z, 0.0) * pose.rot;

            pose.rot.x += 0.5 * dt * dq.x;
            pose.rot.y += 0.5 * dt * dq.y;
            pose.rot.z += 0.5 * dt * dq.z;
            pose.rot.w += 0.5 * dt * dq.w;

            pose.rot = pose.rot.normalize();
            self.bodies.set_pose(i, pose);
            self.bodies.set_vel(i, vel);
        }
    }

    pub fn update_colliders_aabb(&mut self) {
        for collider in &mut self.colliders {
            let pose = collider.world_pose(self.bodies.pose(collider.body.0));
            collider.aabb = aabb_of(&collider.shape, &pose);
        }
    }

    pub fn apply_torque(&mut self, id: BodyId, torque: Vec3) {
        self.sleep.wake(id.0 as usize);
        let mut vel = self.bodies.vel(id.0);
        let inv_inertia = self.bodies.inv_inertia_world(id.0);
        vel.ang += inv_inertia * torque;
        self.bodies.set_vel(id.0, vel);
    }

    pub fn set_tick(&mut self, tick: u64) { self.tick = tick; }
    /// Turns the predicted-pose record on or off (a diagnostic; it doesn't change the step).
    pub fn set_record_predicted(&mut self, on: bool) { self.diag_predicted = if on { Some(Vec::new()) } else { None }; }
    /// Each body's pose after the last step's integration, before its position pass (if recorded).
    pub fn predicted_poses(&self) -> Option<&[Isometry]> { self.diag_predicted.as_deref() }
    pub fn clear_solver_caches(&mut self) { self.clear_warm(); self.last_normal_impulse.fill(0.0); }

    pub(super) fn clear_warm(&mut self) { self.warm.clear(); self.warm_patches.clear(); self.warm_joints.clear(); }

    /// Drops every warm-start entry whose key holds body `b`.
    pub(super) fn drop_warm_of(&mut self, b: u32) {
        self.warm.retain(|k, _| k.0 != b && k.1 != b);
        self.warm_patches.retain(|k, _| k.0 != b && k.1 != b);
        let kinds = self.joints.kinds();
        self.warm_joints.retain(|&j, _| match kinds.get(j as usize) {
            Some(JointKind::D6(d)) => d.a.0 != b && d.b.0 != b,
            Some(JointKind::Distance(d)) => d.a.0 != b && d.b.0 != b,
            None => false,
        });
        // A teleported or removed body's joints restart their rotors from the links (set_body_vel
        // keeps them: hosts that write velocities every step would otherwise lose armature)
        self.rotor.retain(|&(j, _), _| match kinds.get(j as usize) {
            Some(JointKind::D6(d)) => d.a.0 != b && d.b.0 != b,
            _ => false,
        });
    }

    /// Sets a joint's armature rotors to its links' relative velocity now. Done when armature
    /// is set, on `reset_d6_rotor` and on `set_body_pose`: started at the next step instead, the
    /// rotor would take the links' speed after that step's host torques, and the first torque
    /// would bypass the armature.
    fn rotor_init(&mut self, j_idx: u32) {
        self.rotor.retain(|&(j, _), _| j != j_idx);
        let Some(JointKind::D6(d)) = self.joints.kinds().get(j_idx as usize).copied() else { return };
        let (pa, va, vb) = (self.bodies.pose(d.a.0), self.bodies.vel(d.a.0), self.bodies.vel(d.b.0));
        let rot_a = pa.rot * d.fa.rot;
        for i in 0..3 {
            if d.r[i].armature > 0.0 {
                let u = rot_a * [Vec3::X, Vec3::Y, Vec3::Z][i];
                self.rotor.insert((j_idx, i as u8), (vb.ang - va.ang).dot(u));
            }
        }
    }

    /// Restarts a joint's armature rotors from its links' relative velocity.
    pub fn reset_d6_rotor(&mut self, id: JointId) { self.rotor_init(id.0); }
    /// An armature rotor's velocity (rad/s), as carried between steps.
    pub fn d6_rotor_velocity(&self, id: JointId, axis: usize) -> Option<f32> { self.rotor.get(&(id.0, axis as u8)).copied() }
    /// The angular impulse x dt the rotor row gave the links in the last step's position pass.
    pub fn d6_rotor_position_lambda(&self, id: JointId, axis: usize) -> f32 {
        match self.joints.kinds().get(id.0 as usize) { Some(JointKind::D6(d)) if axis < 3 => d.r[axis].rotor_lambda, _ => 0.0 }
    }
    /// Turns W4f's rotor row in the position pass on (the default) or off (W4b alone).
    pub fn set_rotor_position_row(&mut self, on: bool) { self.rotor_position_row = on; }

    /// Armature (kg m^2) about rotational axis `axis` of a D6 joint (PARITY_SPEC W4b). Its rotor
    /// starts at the links' current relative velocity.
    pub fn set_d6_armature(&mut self, id: JointId, axis: usize, armature: f32) {
        self.joints.set_d6_armature(id, axis, armature);
        self.rotor_init(id.0);
    }

    /// Number of contacts in the warm-start cache (diagnostics and tests).
    pub fn warm_contacts(&self) -> usize { self.warm.values().map(Vec::len).sum() }

    /// Contacts the last step's solve started warm (diagnostics and tests).
    pub fn warm_matched(&self) -> usize { self.last_warm_matched }

    /// Hashes the carried state (AT_REST_SPEC D2): the warm-start entries, last velocities and
    /// sleep state, in map order, each collection prefixed with its length. Part of `step_hash`.
    pub(super) fn hash_carried(&self, h: &mut riftphys_core::StepHasher) {
        use riftphys_core::hash::canon_f32_bits;
        use riftphys_core::hash_vec3;
        use std::sync::atomic::Ordering::Relaxed;
        let key = |h: &mut riftphys_core::StepHasher, k: &WarmKey| { h.update_u32(k.0); h.update_u32(k.1); h.update_u32(k.2); h.update_u32(k.3); };
        h.update_u64(self.warm.len() as u64);
        for (k, v) in &self.warm {
            key(h, k);
            h.update_u32(v.len() as u32);
            for c in v { hash_vec3(h, &c.la); h.update_f32_bits(canon_f32_bits(c.jn)); hash_vec3(h, &c.jt); }
        }
        h.update_u64(self.warm_patches.len() as u64);
        for (k, p) in &self.warm_patches { key(h, k); hash_vec3(h, &p.jt); h.update_f32_bits(canon_f32_bits(p.tw)); }
        h.update_u64(self.warm_joints.len() as u64);
        for (j, rows) in &self.warm_joints {
            h.update_u32(*j);
            h.update_u32(rows.len() as u32);
            for r in rows { h.update_u8(r.tag); h.update_f32_bits(canon_f32_bits(r.lam)); }
        }
        h.update_u64(self.rotor.len() as u64);
        for (&(j, ax), w) in &self.rotor { h.update_u32(j); h.update_u8(ax); h.update_f32_bits(canon_f32_bits(*w)); }
        h.update_u64(self.last_lin_vel.len() as u64);
        for v in &self.last_lin_vel { hash_vec3(h, v); }
        h.update_u64(self.sleep.awake.len() as u64);
        for a in self.sleep.awake.iter() { h.update_u8(a.load(Relaxed)); }
        h.update_u64(self.sleep.sleep_ticks.len() as u64);
        for t in self.sleep.sleep_ticks.iter() { h.update_bytes(&t.load(Relaxed).to_le_bytes()); }
    }

    /// The state a step carries besides body poses and velocities (see `CarriedState`).
    pub fn carried_state(&self) -> CarriedState {
        use std::sync::atomic::Ordering::Relaxed;
        CarriedState {
            warm: self.warm.clone(),
            warm_patches: self.warm_patches.clone(),
            warm_joints: self.warm_joints.clone(),
            last_lin_vel: self.last_lin_vel.clone(),
            awake: self.sleep.awake.iter().map(|a| a.load(Relaxed)).collect(),
            sleep_ticks: self.sleep.sleep_ticks.iter().map(|t| t.load(Relaxed)).collect(),
            rotor: self.rotor.clone(),
        }
    }

    /// Restores `carried_state()`. Call it after setting poses and velocities: `set_body_pose`
    /// drops warm entries and `set_body_vel` writes the last velocity.
    pub fn restore_carried_state(&mut self, s: &CarriedState) {
        use std::sync::atomic::Ordering::Relaxed;
        self.warm = s.warm.clone();
        self.warm_patches = s.warm_patches.clone();
        self.warm_joints = s.warm_joints.clone();
        self.rotor = s.rotor.clone();
        // The restored rotors are authoritative: the restore's own set_body_pose calls marked
        // them pending, and a set_body_vel before the next step would re-init them, which the
        // original run (nothing pending) didn't do
        self.rotor_pending.clear();
        for (i, v) in s.last_lin_vel.iter().enumerate() {
            if let Some(x) = self.last_lin_vel.get_mut(i) { *x = *v; }
        }
        for (i, (a, t)) in s.awake.iter().zip(&s.sleep_ticks).enumerate() {
            if i < self.sleep.awake.len() {
                self.sleep.awake[i].store(*a, Relaxed);
                self.sleep.sleep_ticks[i].store(*t, Relaxed);
            }
        }
        // A body removed since the snapshot can't come back: drop its entries, or they would be
        // kept as an idle pair's every step from here on. One pass over the cache.
        let alive = |b: u32| b == u32::MAX || self.alive.get(b as usize).copied().unwrap_or(0) != 0;
        self.warm.retain(|k, _| alive(k.0) && alive(k.1));
        self.warm_patches.retain(|k, _| alive(k.0) && alive(k.1));
        let kinds = self.joints.kinds();
        self.warm_joints.retain(|&j, _| match kinds.get(j as usize) {
            Some(JointKind::D6(d)) => alive(d.a.0) && alive(d.b.0),
            Some(JointKind::Distance(d)) => alive(d.a.0) && alive(d.b.0),
            None => false,
        });
        self.rotor.retain(|&(j, _), _| match kinds.get(j as usize) {
            Some(JointKind::D6(d)) => alive(d.a.0) && alive(d.b.0),
            _ => false,
        });
    }
    pub fn wake_all_bodies(&mut self) {
        for i in 0..self.bodies.len() { if self.bodies.is_dynamic(i as u32) { self.sleep.wake(i); } }
    }

    pub fn set_d6_drive_r(&mut self, id: JointId, axis: usize, drive: Option<D6Drive>) { self.joints.set_d6_drive_r(id, axis, drive); }
    pub fn set_d6_drive_t(&mut self, id: JointId, axis: usize, drive: Option<D6Drive>) { self.joints.set_d6_drive_t(id, axis, drive); }
    pub fn set_d6_limit_r(&mut self, id: JointId, axis: usize, limit: Option<D6Limit>) { self.joints.set_d6_limit_r(id, axis, limit); }
    /// Viscous damping (N m s/rad or N s/m) and Coulomb friction (N m or N) on one axis of a D6
    /// joint, rotational if `angular` (PARITY_SPEC W3a).
    pub fn set_d6_damping(&mut self, id: JointId, angular: bool, axis: usize, damping: f32, friction: f32) { self.joints.set_d6_damping(id, angular, axis, damping, friction); }

    // --- XPBD CONSTRAINT SOLVER ---
    /// Use measured contact properties for a material pair (either order) instead of the table.
    pub fn set_pair_props(&mut self, a: riftphys_materials::materials::MaterialId, b: riftphys_materials::materials::MaterialId, props: riftphys_materials::materials::MatPairProps) {
        let key = if (a as u8) <= (b as u8) { (a, b) } else { (b, a) };
        self.pair_overrides.retain(|(k, _)| *k != key);
        self.pair_overrides.push((key, props));
    }

    /// Contact properties the solver uses for a pair: a measured override, else the table.
    pub fn pair_props(&self, a: riftphys_materials::materials::MaterialId, b: riftphys_materials::materials::MaterialId) -> riftphys_materials::materials::MatPairProps {
        let key = if (a as u8) <= (b as u8) { (a, b) } else { (b, a) };
        self.pair_overrides.iter().find(|(k, _)| *k == key).map(|(_, p)| *p)
            .unwrap_or_else(|| riftphys_materials::materials::pair_props(a, b))
    }

    /// Velocity-level (contacts, distance joints) and position-level (D6 joints + contact
    /// projection) iterations per step; default (20, 20). Scenes that substep can use fewer.
    /// Diagnostic (PARITY_SPEC W4e): damp implicit compliant contacts at the step-start
    /// penetration (true) instead of the predicted mid-step one (false, the default).
    pub fn set_compliant_damping_at_start(&mut self, on: bool) { self.compliant_damping_at_start = on; }

    pub fn set_solver_iterations(&mut self, velocity: u32, position: u32) {
        self.solver_iters = (velocity.max(1), position.max(1));
    }

    /// How the position-level joint solve in `step` chooses its correction directions.
    pub fn set_joint_projection(&mut self, p: JointProjection) { self.joint_projection = p; }
    pub fn joint_projection(&self) -> JointProjection { self.joint_projection }

    /// Largest (contact, joint) batch of the last step's solve: 64 or more ran in parallel.
    pub fn last_max_batch(&self) -> (usize, usize) { self.last_max_batch }

    /// Largest normal-velocity residual (m/s) the last step's bounce pseudo solve left
    /// (PARITY_SPEC W6); 0 when nothing bounced.
    pub fn last_bounce_residual(&self) -> f32 { self.last_bounce_residual }

    /// The last step's bounce passes (PARITY_SPEC W6): rows, sweeps used, residual, and the
    /// fastest mid-step approach of any rigid contact.
    pub fn last_bounce_stats(&self) -> BounceStats { self.last_bounce_stats }

    /// Tests only: the bounce passes' direct-solve limit in rows per component, so the sweeping
    /// fallback can be exercised. Not part of the simulation's configuration.
    #[doc(hidden)]
    pub fn set_bounce_direct_max(&mut self, rows: usize) { self.bounce_direct_max = rows; }

    /// Tests only: also solve the first Newton system of each velocity solve densely (the
    /// reference for the sparse factor); `BounceStats::dense_check` reports the difference. Like
    /// `set_bounce_direct_max`, not saved in `CarriedState` or hashed; a new World starts off.
    #[doc(hidden)]
    pub fn set_bounce_dense_reference(&mut self, on: bool) { self.bounce_dense_reference = on; }

    /// Tests only: the last step's velocity-pass impulse changes per row (components in order)
    #[doc(hidden)]
    pub fn last_bounce_impulses(&self) -> &[f64] { &self.last_bounce_x }

    /// Whether `body` moved ballistically last step (free flight, W6 H2).
    pub fn last_ballistic(&self, body: BodyId) -> bool { self.last_ballistic.get(body.0 as usize).copied().unwrap_or(false) }

    /// Whether any of `body`'s contacts carried load last step.
    pub fn last_contact_loaded(&self, body: BodyId) -> bool { self.contact_loaded.get(body.0 as usize).copied().unwrap_or(false) }

    /// The joint solve inside `step`: SHAKE directions from the start-of-step poses `start`
    /// (indexed by body), or PBD.
    /// W4f: before the position pass, each armature axis's rotor target, theta_r* = the axis
    /// angle at the step's start + dt x omega_rotor (as the velocity solve left it). The gear is
    /// rigid, so the rotor's angle equals the axis's at every step start and isn't carried.
    pub(super) fn rotor_targets(&mut self, start: &[Isometry], dt: f32) {
        if self.rotor.is_empty() || !self.rotor_position_row { return; }
        let rotor = &self.rotor;
        let pose = |id: u32| start.get(id as usize).copied().unwrap_or(Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY });
        for (k, kind) in self.joints.kinds_mut().iter_mut().enumerate() {
            let JointKind::D6(d) = kind else { continue; };
            for i in 0..3 {
                d.r[i].rotor_target = None;
                if !(d.r[i].armature > 0.0) { continue; }
                let Some(&w) = rotor.get(&(k as u32, i as u8)) else { continue; };
                let (ra, rb) = (pose(d.a.0).rot * d.fa.rot, pose(d.b.0).rot * d.fb.rot);
                let a0 = riftphys_articulation::bodies::d6_row_angle(d, ra, rb, i);
                d.r[i].rotor_target = Some(a0 + dt * w);
            }
        }
    }

    /// W4f: after the position pass, each rotor keeps the impulse it gave the links there,
    /// omega_rotor -= lambda_pos / (J_a dt), and the targets are cleared.
    pub(super) fn rotor_carry(&mut self, dt: f32) {
        if self.rotor.is_empty() { return; }
        for (k, kind) in self.joints.kinds_mut().iter_mut().enumerate() {
            let JointKind::D6(d) = kind else { continue; };
            for i in 0..3 {
                if d.r[i].rotor_target.take().is_none() { continue; }
                let lam = d.r[i].rotor_lambda;
                if lam != 0.0 {
                    if let Some(w) = self.rotor.get_mut(&(k as u32, i as u8)) { *w -= lam / (d.r[i].armature * dt); }
                }
            }
        }
    }

    pub(super) fn solve_joints_in_step(&mut self, dt: f32, start: &[Isometry], frozen: &[bool]) {
        // Damped compliant rows measure their change from the step's start under PBD too
        let shake = (self.joint_projection == JointProjection::Shake).then_some(start);
        self.joints.solve_step_with(&mut self.bodies, dt, 1, shake, Some(start), frozen);
    }

    /// Position-level joint solve from the current poses (PBD directions; used outside `step`,
    /// e.g. to settle a pose).
    pub fn solve_joints(&mut self, dt: f32, iterations: u32) {
        // 1. Standard XPBD Solve (each call is one step: multipliers start from zero)
        self.joints.begin_step();
        self.joints.solve(&mut self.bodies, dt, iterations);

        // The center-to-center "HARD PROJECTION" block has been completely removed.
        // XPBD naturally enforces positional coincidence at the localized anchor
        // offsets defined by the Isometry parameters in the Generic6Dof constraint.
        // Forcing an arbitrary 1.0m global distance overrides the solver and
        // introduces infinite energy into underconstrained/shorter assemblies.
    }
    pub fn is_motor_driven(&self, body_id: BodyId) -> bool {
        for kind in self.joints.kinds().iter() {
            if let JointKind::D6(j) = kind {
                if j.a == body_id || j.b == body_id {
                    for axis in 0..3 {
                        if let Some(drive) = j.r[axis].drive {
                            if drive.vel_mode { return true; }
                        }
                    }
                }
            }
        }
        false
    }
    pub fn get_joint_relative_rot(&self, id: JointId) -> Option<Quat> {
        let kinds = self.joints.kinds();
        if id.0 as usize >= kinds.len() { return None; }
        match &kinds[id.0 as usize] {
            JointKind::D6(j) => {
                let pa = self.bodies.pose(j.a.0);
                let pb = self.bodies.pose(j.b.0);
                let ra = pa.rot * j.fa.rot;
                let rb = pb.rot * j.fb.rot;
                Some(ra.conjugate() * rb)
            },
            _ => None
        }
    }
}