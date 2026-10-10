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

#[derive(Copy, Clone, Default)]
pub(super) struct WarmImp {
    pub(super) jn: f32,
    pub(super) jt1: f32,
    pub(super) jt2: f32,
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

    pub(super) warm_cache: BTreeMap<(u32, u32), WarmImp>,
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
            warm_cache: BTreeMap::new(),
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

    pub fn set_body_pose(&mut self, id: BodyId, pose: Isometry) {
        self.bodies.set_pose(id.0, pose);
        for c in &mut self.colliders {
            if c.body == id { c.aabb = aabb_of(&c.shape, &pose); }
        }
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
            self.warm_cache.clear();
        }
        if let Some(desc) = self.pending_epoch.take() {
            self.epoch_id = epoch_id(&desc);
            self.set_gravity(Vec3::new(desc.gravity_g[0], desc.gravity_g[1], desc.gravity_g[2]));
            self.warm_cache.clear();
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
        self.colliders.push(Collider { body, shape, aabb, material });
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

    pub fn set_body_vel(&mut self, id: BodyId, vel: Velocity) { self.bodies.set_vel(id.0, vel); }

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
        self.warm_cache.clear();
        true
    }

    #[inline]
    pub fn normal_force(&self, body: BodyId, dt: f32) -> f32 { self.last_normal_impulse.get(body.0 as usize).copied().unwrap_or(0.0) / dt }

    pub fn add_ball_joint(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry) -> JointId { self.joints.add_ball(a, b, fa, fb) }
    pub fn add_hinge_joint(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry, hinge_axis: usize) -> JointId { self.joints.add_hinge(a, b, fa, fb, hinge_axis) }
    pub fn add_d6_joint(&mut self, j: D6Joint) -> JointId { self.joints.add_d6(j) }

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
            let pose = self.bodies.pose(collider.body.0);
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
    pub fn clear_solver_caches(&mut self) { self.warm_cache.clear(); self.last_normal_impulse.fill(0.0); }
    pub fn wake_all_bodies(&mut self) {
        for i in 0..self.bodies.len() { if self.bodies.is_dynamic(i as u32) { self.sleep.wake(i); } }
    }

    pub fn set_d6_drive_r(&mut self, id: JointId, axis: usize, drive: Option<D6Drive>) { self.joints.set_d6_drive_r(id, axis, drive); }
    pub fn set_d6_drive_t(&mut self, id: JointId, axis: usize, drive: Option<D6Drive>) { self.joints.set_d6_drive_t(id, axis, drive); }
    pub fn set_d6_limit_r(&mut self, id: JointId, axis: usize, limit: Option<D6Limit>) { self.joints.set_d6_limit_r(id, axis, limit); }

    // --- XPBD CONSTRAINT SOLVER ---
    pub fn solve_joints(&mut self, dt: f32, iterations: u32) {
        // 1. Standard XPBD Solve
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