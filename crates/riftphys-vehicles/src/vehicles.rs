//! Simple wheeled-vehicle helpers (MVP, deterministic-friendly).
//!
//! - World-agnostic: caller supplies callbacks for pose/vel and writing vel.
//! - Ground is sampled via `GroundSampler` (HeightField adapter included).
//! - Quantizes dt/inputs/forces to reduce drift and keep cross-run stability.

use glam::{Affine3A, Quat, Vec3 as GVec3};
use riftphys_core::{BodyId, Scalar, Vec3, Velocity}; // Vec3 == glam::Vec3A

use riftphys_terrain::HeightField;

/// Quantize to 1e-6 (determinism-friendly).
#[inline]
fn q6(x: f32) -> f32 {
    if x.is_finite() {
        (x * 1.0e6_f32).round() * 1.0e-6_f32
    } else {
        0.0
    }
}

/// Per-step quantities (ground samples, forces, velocities, dt) are not rounded (PRECISION_SPEC
/// P1); non-finite values become 0. Inputs and parameters still go through `q6`.
#[inline]
fn fin(x: f32) -> f32 {
    if x.is_finite() { x } else { 0.0 }
}

#[inline]
fn fin_v(v: Vec3) -> Vec3 {
    Vec3::new(fin(v.x), fin(v.y), fin(v.z))
}

#[inline]
#[allow(dead_code)]
fn q6v(v: Vec3) -> Vec3 {
    Vec3::new(q6(v.x), q6(v.y), q6(v.z))
}

#[inline]
fn v3a(v: GVec3) -> Vec3 {
    v.into()
}

/// Ground sampling abstraction (world-space xz → (height_y, normal_ws)).
pub trait GroundSampler {
    /// Returns `(height_y, normal_ws)` for the given world-space XZ.
    fn sample(&self, wx: f32, wz: f32) -> (f32, Vec3);
}

/// Direct adapter: assumes HeightField’s local origin corresponds to world origin.
/// If your HF is in a tile with an origin offset, use `HeightFieldSampler`.
impl GroundSampler for HeightField {
    #[inline]
    fn sample(&self, wx: f32, wz: f32) -> (f32, Vec3) {
        let y = self.sample_height(wx, wz);
        let mut n: Vec3 = self.sample_normal(wx, wz).into();
        n = n.normalize_or_zero();
        if n.length_squared() < 1.0e-12 {
            n = Vec3::Y;
        }
        (fin(y), fin_v(n))
    }
}

/// HeightField wrapper that applies a world→local offset for sampling.
#[derive(Copy, Clone, Debug)]
pub struct HeightFieldSampler<'a> {
    /// HeightField reference.
    pub hf: &'a HeightField,
    /// World-space origin of the heightfield’s (0,0) sample in XZ.
    pub origin_xz: [f32; 2],
}

impl<'a> HeightFieldSampler<'a> {
    /// Create a sampler over an existing [`HeightField`].
    ///
    /// `origin_xz` is the world-space XZ position that corresponds to the heightfield’s local (0,0).
    /// The sampler converts world coordinates `(wx, wz)` into heightfield-local coordinates by
    /// subtracting this origin before sampling height and normal.
    #[inline]
    pub fn new(hf: &'a HeightField, origin_xz: [f32; 2]) -> Self {
        Self { hf, origin_xz }
    }
}


impl<'a> GroundSampler for HeightFieldSampler<'a> {
    #[inline]
    fn sample(&self, wx: f32, wz: f32) -> (f32, Vec3) {
        let lx = wx - self.origin_xz[0];
        let lz = wz - self.origin_xz[1];
        let y = self.hf.sample_height(lx, lz);
        let mut n: Vec3 = self.hf.sample_normal(lx, lz).into();
        n = n.normalize_or_zero();
        if n.length_squared() < 1.0e-12 {
            n = Vec3::Y;
        }
        (fin(y), fin_v(n))
    }
}

/// Wheel geometry and suspension / tire parameters.
#[derive(Copy, Clone, Debug)]
pub struct WheelParams {
    /// Wheel attach point in chassis space.
    pub local_pos: Vec3,
    /// Suspension axis in chassis space (typically -Y).
    pub susp_dir: Vec3,
    /// Suspension rest length (m).
    pub rest_len: f32,
    /// Spring rate (N/m) along `susp_dir`.
    pub k_spring: f32,
    /// Damping (N·s/m) along `susp_dir`.
    pub k_damp: f32,
    /// Wheel radius (m).
    pub radius: f32,
    /// Longitudinal friction coefficient.
    pub mu_long: f32,
    /// Lateral friction coefficient.
    pub mu_lat: f32,
}

/// Driver input for a vehicle tick.
#[derive(Copy, Clone, Debug)]
pub struct AxleInput {
    /// Throttle in [-1, 1] (negative ≈ brake/reverse in this MVP).
    pub throttle01: f32,
    /// Steering angle (radians, + = steer left around up axis).
    pub steer_rad: f32,
}

/// Static vehicle description: chassis body and wheel layout.
#[derive(Clone, Debug)]
pub struct VehicleParams {
    /// Chassis body id in the host world.
    pub body: BodyId,
    /// Per-wheel parameters (index 0..N).
    pub wheels: Vec<WheelParams>,
    /// Indices into `wheels` that receive drive torque.
    pub drive_map: Vec<usize>,
    /// Indices into `wheels` that steer.
    pub steer_map: Vec<usize>,
    /// Chassis mass hint (kg) used to convert forces → acceleration.
    pub mass_hint: f32,
}

/// Runtime container you update every tick.
#[derive(Clone, Debug)]
pub struct VehicleInstance {
    /// Static setup.
    pub p: VehicleParams,
    /// Current driver input.
    pub input: AxleInput,
}

impl VehicleInstance {
    /// Advance one physics tick using world–agnostic callbacks.
    ///
    /// * `pose_of`  – given a body id, return `(position, rotation)` in world space.
    /// * `vel_of`   – given a body id, return current `Velocity`.
    /// * `set_vel`  – write a new `Velocity` for the body.
    /// * `ground`   – optional ground sampler (heightfield, world raycast, etc).
    /// * `dt`       – timestep (seconds).
    pub fn step_with_host<P, V, S>(
        &mut self,
        mut pose_of: P,
        mut vel_of: V,
        mut set_vel: S,
        ground: Option<&dyn GroundSampler>,
        dt: Scalar,
    ) where
        P: FnMut(BodyId) -> (GVec3, Quat),
        V: FnMut(BodyId) -> Velocity,
        S: FnMut(BodyId, Velocity),
    {
        let dt = fin(dt as f32) as Scalar;
        if dt <= 0.0 {
            return;
        }

        // Quantize inputs so host-side float noise doesn’t desync forces.
        let throttle = q6(self.input.throttle01.clamp(-1.0, 1.0));
        let steer_in = q6(self.input.steer_rad);

        let mass = q6(self.p.mass_hint.max(1.0));
        let g_mag = 9.81_f32; // keep stable constant here; world can override later if needed.

        let (pos_w, rot_w) = pose_of(self.p.body);
        let pose_aff = Affine3A::from_rotation_translation(rot_w, pos_w);
        let vel = vel_of(self.p.body);

        let mut force_sum_w = GVec3::ZERO;

        for wi in 0..self.p.wheels.len() {
            let is_drive = self.p.drive_map.iter().any(|&k| k == wi);
            let is_steer = self.p.steer_map.iter().any(|&k| k == wi);
            let steer = if is_steer { steer_in } else { 0.0 };

            let (hit, n_w, f_susp, f_long, f_lat) =
                self.eval_wheel(wi, pose_aff, vel, ground, throttle, steer, mass, g_mag);

            if !hit {
                continue;
            }

            // Build wheel frame in world:
            // - chassis forward/right from pose
            // - optional steering around chassis up axis
            let up_w = pose_aff.transform_vector3(GVec3::Y).normalize_or_zero();
            let chassis_fwd = pose_aff.transform_vector3(GVec3::X).normalize_or_zero();
            let chassis_lat = pose_aff.transform_vector3(GVec3::Z).normalize_or_zero();

            let steer_q = if up_w.length_squared() > 1.0e-12 {
                Quat::from_axis_angle(up_w, steer)
            } else {
                Quat::IDENTITY
            };

            let fwd_w = (steer_q * chassis_fwd).normalize_or_zero();
            let lat_w = (steer_q * chassis_lat).normalize_or_zero();

            let n_w: GVec3 = n_w.into();
            let f_wheel = n_w * f_susp + fwd_w * f_long + lat_w * f_lat;

            // Quantize contribution before summing.
            force_sum_w += GVec3::new(fin(f_wheel.x), fin(f_wheel.y), fin(f_wheel.z));

            // If not a drive wheel, kill longitudinal drive force (still allows suspension + lateral).
            if !is_drive {
                // remove the drive portion we just added
                let f_drive = fwd_w * f_long;
                force_sum_w -= GVec3::new(fin(f_drive.x), fin(f_drive.y), fin(f_drive.z));
            }
        }

        if force_sum_w.length_squared() <= 0.0 {
            return;
        }

        let mut v = vel;
        let a = force_sum_w / mass;

        // Integrate linear velocity (MVP). Quantize output.
        v.lin = fin_v(v.lin + v3a(a) * dt);
        set_vel(self.p.body, v);
    }

    /// Internal wheel query vs ground sampler.
    /// Returns `(contact?, normal_ws, f_susp, f_long, f_lat)`.
    fn eval_wheel(
        &self,
        idx: usize,
        pose_world: Affine3A,
        vel_world: Velocity,
        ground: Option<&dyn GroundSampler>,
        throttle: f32,
        steer_rad: f32,
        mass: f32,
        g_mag: f32,
    ) -> (bool, Vec3, f32, f32, f32) {
        let wp = self.p.wheels[idx];

        let Some(ground) = ground else {
            return (false, Vec3::Y, 0.0, 0.0, 0.0);
        };

        // Anchor in world.
        let p0_w = pose_world.transform_point3(wp.local_pos.into());

        // Suspension axis in world.
        let mut dir_w = pose_world.transform_vector3(wp.susp_dir.into()).normalize_or_zero();
        if dir_w.length_squared() < 1.0e-12 {
            dir_w = -GVec3::Y; // deterministic fallback
        }

        // Cast length along suspension.
        let cast_len = q6(wp.rest_len.max(0.0) + wp.radius.max(0.0));

        // Sample ground beneath wheel (vertical height query).
        let wx = fin(p0_w.x);
        let wz = fin(p0_w.z);
        let (ground_y, mut n_g) = ground.sample(wx, wz);
        n_g = n_g.normalize_or_zero();
        if n_g.length_squared() < 1.0e-12 {
            n_g = Vec3::Y;
        }

        // Bottom point at full extension (along suspension axis).
        let p_bottom = p0_w + dir_w * cast_len;
        let y_bottom = fin(p_bottom.y);

        // No contact if bottom is above ground.
        if y_bottom > ground_y {
            return (false, n_g, 0.0, 0.0, 0.0);
        }

        // Compression amount (scalar). Quantize.
        let x = fin((ground_y - y_bottom).clamp(0.0, cast_len));

        // Relative velocity along suspension axis (MVP: chassis linear only).
        let v_rel = fin(vel_world.lin.dot(v3a(dir_w)));

        let f_spring = fin(wp.k_spring.max(0.0) * x);
        let f_damp = fin(wp.k_damp.max(0.0) * v_rel);
        let f_susp = fin(f_spring - f_damp); // along +dir_w

        // Tire frame (world): chassis axes with steering around up axis.
        let up_w = pose_world.transform_vector3(GVec3::Y).normalize_or_zero();
        let chassis_fwd = pose_world.transform_vector3(GVec3::X).normalize_or_zero();
        let chassis_lat = pose_world.transform_vector3(GVec3::Z).normalize_or_zero();

        let steer_q = if up_w.length_squared() > 1.0e-12 {
            Quat::from_axis_angle(up_w, steer_rad)
        } else {
            Quat::IDENTITY
        };

        let fwd_w: Vec3 = v3a((steer_q * chassis_fwd).normalize_or_zero());
        let lat_w: Vec3 = v3a((steer_q * chassis_lat).normalize_or_zero());

        // Project chassis linear velocity into tire axes.
        let v = vel_world.lin;
        let _v_long = fin(v.dot(fwd_w));
        let v_lat = fin(v.dot(lat_w));

        // Simple friction caps (normal load proxy = m*g).
        let fmax_long = fin(wp.mu_long.max(0.0) * mass * g_mag);
        let fmax_lat = fin(wp.mu_lat.max(0.0) * mass * g_mag);

        // Longitudinal: throttle * cap (drive map applied by caller).
        let f_long = fin((throttle * fmax_long).clamp(-fmax_long, fmax_long));

        // Lateral: super-MVP cornering spring; oppose lateral velocity.
        let f_lat = fin((-v_lat * mass).clamp(-fmax_lat, fmax_lat));

        (true, n_g, f_susp, f_long, f_lat)
    }
}
