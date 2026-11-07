use riftphys_core::{Quat, Vec3, StepCtx};
use riftphys_core::models::{AeroModel, AeroQuery};

use crate::ISA;

// Not rounded (PRECISION_SPEC P1); non-finite values become 0.
#[inline]
fn fin(x: f32) -> f32 {
    if x.is_finite() { x } else { 0.0 }
}

#[inline]
fn fin_v(v: Vec3) -> Vec3 {
    Vec3::new(fin(v.x), fin(v.y), fin(v.z))
}

#[inline]
fn normalize_fin(v: Vec3) -> Vec3 {
    let v = fin_v(v);
    let len2 = v.length_squared();
    if len2 > 1.0e-12 {
        v * (1.0 / len2.sqrt())
    } else {
        Vec3::ZERO
    }
}

#[derive(Copy, Clone, Debug)]
pub struct FlatPlateDrag {
    pub cd: f32,
    pub area_m2: f32,
    pub isa: ISA,
}

impl AeroModel for FlatPlateDrag {
    fn accel_contrib(&self, _ctx: &StepCtx, q: AeroQuery) -> [f32; 3] {
        let mass = fin(q.mass);
        if !(mass > 0.0) { return [0.0; 3]; }

        let rho = fin(self.isa.density(q.altitude));
        if rho <= 0.0 { return [0.0; 3]; }

        let v = Vec3::from_array(q.vel_world);
        let v2 = v.length_squared();
        if !v2.is_finite() || v2 <= 1.0e-20 { return [0.0; 3]; }

        let speed = v2.sqrt();
        if speed <= 1.0e-6 { return [0.0; 3]; }

        let vhat = normalize_fin(v / speed);
        if vhat == Vec3::ZERO { return [0.0; 3]; }

        let cd = fin(self.cd).max(0.0);
        let area = fin(self.area_m2).max(0.0);
        if area <= 0.0 { return [0.0; 3]; }

        let f_mag = fin(0.5 * rho * fin(speed * speed) * cd * area);
        let a = fin_v((-vhat) * fin(f_mag / mass));
        [a.x, a.y, a.z]
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SimpleWing {
    pub cl_per_rad: f32,
    pub stall_rad: f32,
    pub area_m2: f32,
    pub lift_dir_body: Vec3,
    pub isa: ISA,
    pub cd0: f32,
    pub k_induced: f32,
}

impl AeroModel for SimpleWing {
    fn accel_contrib(&self, _ctx: &StepCtx, q: AeroQuery) -> [f32; 3] {
        let mass = fin(q.mass);
        if !(mass > 0.0) { return [0.0; 3]; }

        let rho = fin(self.isa.density(q.altitude));
        if rho <= 0.0 { return [0.0; 3]; }

        let v = Vec3::from_array(q.vel_world);
        let v2 = v.length_squared();
        if !v2.is_finite() || v2 <= 1.0e-20 { return [0.0; 3]; }

        let speed = v2.sqrt();
        if speed <= 1.0e-6 { return [0.0; 3]; }

        let vhat = normalize_fin(v / speed);
        if vhat == Vec3::ZERO { return [0.0; 3]; }

        let stall = fin(self.stall_rad).abs().max(1.0e-6);
        let alpha = fin(q.alpha_rad).clamp(-stall, stall);

        let cl_per = fin(self.cl_per_rad);
        let cl = fin(cl_per * alpha);

        let cd0 = fin(self.cd0).max(0.0);
        let k   = fin(self.k_induced).max(0.0);
        let cd  = fin(cd0 + fin(k * cl * cl));

        let area = fin(self.area_m2).max(0.0);
        if area <= 0.0 { return [0.0; 3]; }

        let ori: Quat = q.orientation;

        // Replaced mul_vec3a with standard multiplication
        let lift_world_nominal = fin_v(ori * self.lift_dir_body);

        let proj = fin(lift_world_nominal.dot(vhat));
        let mut lift_dir = fin_v(lift_world_nominal - vhat * proj);

        let lift_len2 = lift_dir.length_squared();
        if !lift_len2.is_finite() || lift_len2 <= 1.0e-20 { return [0.0; 3]; }

        lift_dir = normalize_fin(lift_dir);
        if lift_dir == Vec3::ZERO { return [0.0; 3]; }

        let qdyn = fin(0.5 * rho * fin(speed * speed));
        let l = fin(qdyn * cl * area);
        let d = fin(qdyn * cd * area);

        let a_lift = fin_v(lift_dir * fin(l / mass));
        let a_drag = fin_v((-vhat)   * fin(d / mass));

        let a = fin_v(a_lift + a_drag);
        [a.x, a.y, a.z]
    }
}