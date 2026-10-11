
use riftphys_core::types::{Isometry, Mat3, Velocity, Vec3};
use riftphys_core::{Quat, Scalar};
use glam::Vec3 as GVec3;

const EPS_ANG2: f32 = 1.0e-24;

/* ─────────────────────────  Quantization + sanitation  ───────────────────────── */

#[inline] fn q6(x: f32) -> f32 { if !x.is_finite() { return 0.0; } (x * 1.0e6_f32).round() * 1.0e-6_f32 }
#[inline] fn q6v(v: Vec3) -> Vec3 { Vec3::new(q6(v.x), q6(v.y), q6(v.z)) }
#[inline] fn quat_canon(mut q: Quat) -> Quat { if q.w < 0.0 { q = -q; } q }
#[inline] fn q6q(q: Quat) -> Quat {
    let mut qq = Quat::from_xyzw(q6(q.x), q6(q.y), q6(q.z), q6(q.w));
    if !qq.is_finite() { return Quat::IDENTITY; }
    qq = qq.normalize();
    quat_canon(qq)
}
#[inline] fn safe_pos(v: Vec3) -> Vec3 { if v.is_finite() { q6v(v) } else { Vec3::ZERO } }
#[inline] fn safe_vec(v: Vec3) -> Vec3 { if v.is_finite() { q6v(v) } else { Vec3::ZERO } }
#[inline] fn safe_quat(q: Quat) -> Quat { if !q.is_finite() { Quat::IDENTITY } else { q6q(q) } }

/* ─────────────────────────  Bodies  ───────────────────────── */

#[derive(Copy, Clone, Debug)]
pub struct BodyDesc {
    pub pose: Isometry,
    pub vel: Velocity,
    pub inv_mass: Scalar,
    pub dynamic: bool,
    /// Body-frame inverse inertia tensor. None falls back to the old isotropic
    /// diag(inv_mass), which treats every body like a unit sphere of radius ~1.2 m.
    pub inv_inertia_local: Option<Mat3>,
}

pub struct Bodies {
    pos: Vec<Vec3>,
    rot: Vec<Quat>,
    linvel: Vec<Vec3>,
    angvel: Vec<Vec3>,
    inv_mass: Vec<Scalar>,
    dynamic: Vec<bool>,
    inv_inertia_local: Vec<Mat3>,
}

impl Bodies {
    pub fn with_capacity(cap: usize) -> Self {
        Self {
            pos: Vec::with_capacity(cap),
            rot: Vec::with_capacity(cap),
            linvel: Vec::with_capacity(cap),
            angvel: Vec::with_capacity(cap),
            inv_mass: Vec::with_capacity(cap),
            dynamic: Vec::with_capacity(cap),
            inv_inertia_local: Vec::with_capacity(cap),
        }
    }

    #[inline] pub fn len(&self) -> usize { self.pos.len() }

    // Required for hashing
    /// Iterator for hashing or batch processing in stable order.
    pub fn indices(&self) -> impl ExactSizeIterator<Item = u32> {
        0..(self.len() as u32)
    }

    /// Small-angle orientation correction (world space).
    /// Used by D6 joints for positional/angular correction.
    pub fn apply_orientation_delta(&mut self, id: u32, dtheta_world: Vec3) {
        let i = self.idx(id);
        if self.inv_mass[i] <= 0.0 || !dtheta_world.is_finite() { return; }

        let d = q6v(dtheta_world);
        let ang2 = d.length_squared();
        if ang2 <= EPS_ANG2 { return; }

        // dq = [sin(theta/2) * axis, cos(theta/2)]
        // For small angles: sin(x) approx x, cos(x) approx 1
        let dq = Quat::from_xyzw(
            d.x * 0.5,
            d.y * 0.5,
            d.z * 0.5,
            1.0,
        ).normalize();

        self.rot[i] = q6q((dq * self.rot[i]).normalize());
    }

    #[inline]
    fn idx(&self, id: u32) -> usize {
        let i = id as usize;
        debug_assert!(i < self.len());
        i
    }

    pub fn add(&mut self, desc: BodyDesc) -> u32 {
        let is_dyn = desc.dynamic && desc.inv_mass.is_finite() && desc.inv_mass > 0.0;
        let inv_m = if is_dyn { q6(desc.inv_mass) } else { 0.0 };

        self.pos.push(safe_pos(desc.pose.pos));
        self.rot.push(safe_quat(desc.pose.rot));
        self.linvel.push(safe_vec(desc.vel.lin));
        self.angvel.push(safe_vec(desc.vel.ang));
        self.inv_mass.push(inv_m);
        self.dynamic.push(is_dyn);

        let inv_i = match (is_dyn, desc.inv_inertia_local) {
            (false, _) => Mat3::ZERO,
            (true, Some(m)) if m.is_finite() => m,
            (true, _) => Mat3::from_diagonal(GVec3::splat(inv_m)),
        };
        self.inv_inertia_local.push(inv_i);

        (self.pos.len() as u32) - 1
    }

    pub fn add_legacy(&mut self, pose: Isometry, vel: Velocity, inv_mass: Scalar, dynamic: bool) -> u32 {
        self.add(BodyDesc { pose, vel, inv_mass, dynamic, inv_inertia_local: None })
    }

    // -------- Accessors --------

    #[inline] pub fn pose(&self, id: u32) -> Isometry {
        let i = self.idx(id);
        Isometry { pos: self.pos[i], rot: self.rot[i] }
    }

    #[inline] pub fn set_pose(&mut self, id: u32, iso: Isometry) {
        let i = self.idx(id);
        if iso.pos.is_finite() { self.pos[i] = q6v(iso.pos); }
        if iso.rot.is_finite() { self.rot[i] = safe_quat(iso.rot); }
    }

    #[inline] pub fn vel(&self, id: u32) -> Velocity {
        let i = self.idx(id);
        Velocity { lin: self.linvel[i], ang: self.angvel[i] }
    }

    #[inline] pub fn set_vel(&mut self, id: u32, v: Velocity) {
        let i = self.idx(id);
        if v.lin.is_finite() { self.linvel[i] = q6v(v.lin); }
        if v.ang.is_finite() { self.angvel[i] = q6v(v.ang); }
    }

    #[inline] pub fn inv_mass_of(&self, id: u32) -> Scalar { self.inv_mass[self.idx(id)] }
    #[inline] pub fn is_dynamic(&self, id: u32) -> bool { self.dynamic[self.idx(id)] }

    /// World-frame inverse inertia: R * I_local^-1 * R^T (glam matrices are column-major,
    /// so `from_quat` is the body-to-world rotation and its transpose is the inverse).
    /// Body-frame inverse inertia (zero for static bodies).
    #[inline] pub fn inv_inertia_local_of(&self, id: u32) -> Mat3 {
        let i = self.idx(id);
        if self.inv_mass[i] == 0.0 { Mat3::ZERO } else { self.inv_inertia_local[i] }
    }

    #[inline] pub fn inv_inertia_world(&self, id: u32) -> Mat3 {
        let i = self.idx(id);
        if self.inv_mass[i] == 0.0 { return Mat3::ZERO; }
        let r = Mat3::from_quat(self.rot[i]);
        r * self.inv_inertia_local[i] * r.transpose()
    }
    // -------- Integration (Semi-Implicit) --------

    pub fn integrate_all(&mut self, gravity: Vec3, dt: Scalar) {
        let dt_q = q6(dt);
        if dt_q <= 0.0 { return; }
        let g = if gravity.is_finite() { gravity } else { Vec3::ZERO };

        for i in 0..self.len() {
            if !self.dynamic[i] || self.inv_mass[i] <= 0.0 { continue; }

            self.linvel[i] = q6v(self.linvel[i] + g * dt_q);
            self.pos[i] = q6v(self.pos[i] + self.linvel[i] * dt_q);
            let w = self.angvel[i];
            self.rot[i] = Self::integrate_rot(self.rot[i], w, dt_q);
        }
    }

    #[inline]
    fn integrate_rot(q: Quat, w_world: Vec3, dt: Scalar) -> Quat {
        let w2 = w_world.length_squared();
        if w2 <= EPS_ANG2 || !w2.is_finite() { return q; }
        let qdot = Quat::from_xyzw(w_world.x, w_world.y, w_world.z, 0.0) * q;
        let s = 0.5 * dt;
        let qn = Quat::from_xyzw(q.x + qdot.x*s, q.y + qdot.y*s, q.z + qdot.z*s, q.w + qdot.w*s);
        q6q(qn.normalize())
    }

    // -------- Impulses / Corrections --------

    #[inline] pub fn apply_impulse(&mut self, id: u32, j: Vec3) {
        let i = self.idx(id);
        let im = self.inv_mass[i];
        if im > 0.0 && j.is_finite() { self.linvel[i] = q6v(self.linvel[i] + j * im); }
    }

    #[inline] pub fn apply_angular_impulse(&mut self, id: u32, tau: Vec3) {
        let i = self.idx(id);
        if self.inv_mass[i] > 0.0 && tau.is_finite() {
            let dw = self.inv_inertia_world(id) * tau;
            self.angvel[i] = q6v(self.angvel[i] + dw);
        }
    }

    #[inline] pub fn apply_position_delta(&mut self, id: u32, dp: Vec3) {
        let i = self.idx(id);
        if self.inv_mass[i] > 0.0 && dp.is_finite() { self.pos[i] = q6v(self.pos[i] + dp); }
    }

    // Deactivation
    pub fn deactivate(&mut self, id: u32) -> bool {
        let i = id as usize;
        if i >= self.len() { return false; }
        self.linvel[i] = Vec3::ZERO; self.angvel[i] = Vec3::ZERO;
        self.inv_mass[i] = 0.0; self.dynamic[i] = false;
        self.inv_inertia_local[i] = Mat3::ZERO;
        true
    }

}