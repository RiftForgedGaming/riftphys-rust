// riftphys-acceleration/src/propulsion.rs

use riftphys_core::StepCtx;
use riftphys_core::Vec3;
use riftphys_core::models::{PropQuery, PropulsionModel};

/// Non-finite values become 0. Not rounded (PRECISION_SPEC P1).
#[inline]
fn fin(x: f32) -> f32 {
    if x.is_finite() { x } else { 0.0 }
}

#[inline]
fn fin_v(v: Vec3) -> Vec3 {
    Vec3::new(fin(v.x), fin(v.y), fin(v.z))
}

/// Normalize, guarding tiny/NaN input.
#[inline]
fn normalize_fin(v: Vec3) -> Vec3 {
    let v = fin_v(v);
    let len2 = v.length_squared();
    if !len2.is_finite() || len2 <= 1.0e-20 {
        return Vec3::ZERO;
    }
    let inv_len = 1.0 / len2.sqrt();
    v * inv_len
}

#[derive(Copy, Clone, Debug)]
pub enum ThrottleCurve {
    Linear,
    QuadEaseIn,
    /// Polynomial coefficients p0..p4 (up to 4th order), evaluated with Horner.
    Custom([f32; 5]),
}

#[inline]
fn eval_curve(c: &ThrottleCurve, t_in: f32) -> f32 {
    // Treat non-finite throttle as 0.
    let t = if t_in.is_finite() { t_in } else { 0.0 };
    let t = fin(t.clamp(0.0, 1.0));

    match c {
        ThrottleCurve::Linear => t,
        ThrottleCurve::QuadEaseIn => fin(t * t),
        ThrottleCurve::Custom(p) => {
            // Horner with quantization each step.
            let t = fin(t);
            let mut y = fin(p[4]);
            y = fin(y * t + fin(p[3]));
            y = fin(y * t + fin(p[2]));
            y = fin(y * t + fin(p[1]));
            y = fin(y * t + fin(p[0]));
            y
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SimplePropulsion {
    /// Baseline thrust added even at throttle=0.
    pub thrust_constant_n: f32,
    /// Thrust scale driven by the curve gain.
    pub thrust_max_n: f32,
    pub curve: ThrottleCurve,
}

impl PropulsionModel for SimplePropulsion {
    fn accel_contrib(&self, _ctx: &StepCtx, q: PropQuery) -> [f32; 3] {
        // Mass and throttle sanity.
        let mass = fin(q.mass);
        if !(mass > 0.0) {
            return [0.0; 3];
        }

        // Curve gain in [0..1], NaN-safe.
        let gain = eval_curve(&self.curve, q.throttle01).max(0.0);
        let gain = fin(gain);

        // Thrust and accel magnitude (quantized).
        let thrust_c = fin(self.thrust_constant_n);
        let thrust_m = fin(self.thrust_max_n);
        let thrust = fin(thrust_c + fin(thrust_m * gain));
        let a_mag = fin(thrust / mass);

        // Forward dir (world). Quantize + deterministic normalize.
        let dir_in = Vec3::from_array(q.forward_dir_world);
        let dir = normalize_fin(dir_in);

        // Final accel (quantized output).
        let a = fin_v(dir * a_mag);
        [a.x, a.y, a.z]
    }
}
