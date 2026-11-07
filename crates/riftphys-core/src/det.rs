// riftphys-core/src/det.rs
use crate::types::{Quat, Vec3};
use crate::Scalar;

pub const Q6: Scalar = 1.0e-6;

#[inline]
pub fn q6(x: Scalar) -> Scalar {
    if !x.is_finite() { return 0.0; } // Deterministic fallback
    // Use trunc/fractional split if worried about extreme precision,
    // but round is generally stable for simulation ranges.
    (x / Q6).round() * Q6
}

/// Sort key for a float: its value in whole 1e-6 units, so values that differ only by ulp noise
/// tie, and the caller's next key (or a stable sort's original order) decides. Without it, a sort
/// over values that are equal on paper (the corners of a box lying flat) is decided by rounding
/// noise, which is deterministic but chaotic (PRECISION_SPEC P1). -inf sorts first; +inf and NaN
/// last.
#[inline]
pub fn sort_key6(x: Scalar) -> i64 {
    if x.is_nan() { return i64::MAX; }
    let k = (x as f64 * 1.0e6).round();
    if k >= i64::MAX as f64 { i64::MAX } else if k <= i64::MIN as f64 { i64::MIN } else { k as i64 }
}

#[inline]
pub fn q6_vec3(v: Vec3) -> Vec3 {
    Vec3::new(q6(v.x), q6(v.y), q6(v.z))
}

#[inline]
pub fn q6_unit_vec3(v: Vec3) -> Vec3 {
    // quantize -> normalize -> quantize, so the unit vector is stable across platforms
    let n = q6_vec3(v).normalize_or_zero();
    q6_vec3(n)
}

#[inline]
pub fn safe_contact_dist(cd: Scalar) -> Scalar {
    if cd.is_finite() { cd.max(0.0) } else { 0.0 }
}

#[inline]
pub fn quat_canonical(q: Quat) -> Quat {
    // Standardizing on the "W is positive" or "First non-zero is positive"
    if q.w < 0.0 || (q.w == 0.0 && (q.z < 0.0 || (q.z == 0.0 && (q.y < 0.0 || (q.y == 0.0 && q.x < 0.0))))) {
        -q
    } else {
        q
    }
}

#[inline]
pub fn q6_quat(q: Quat) -> Quat {
    let mut c = quat_canonical(q);
    c.x = q6(c.x);
    c.y = q6(c.y);
    c.z = q6(c.z);
    c.w = q6(c.w);
    // Re-normalize and re-canonicalize to ensure it stays on the hypersphere
    quat_canonical(c.normalize())
}

#[inline]
pub fn vec3_is_finite(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

#[inline]
pub fn quat_is_finite(q: Quat) -> bool {
    q.x.is_finite() && q.y.is_finite() && q.z.is_finite() && q.w.is_finite()
}
