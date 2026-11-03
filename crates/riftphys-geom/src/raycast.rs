// riftphys-geom/src/raycast.rs
//! Exact ray casts against every `Shape` at a pose (as opposed to its bounding box).

use riftphys_core::types::{Isometry, Vec3};
use crate::ray::Ray;
use crate::shape::Shape;

const EPS: f32 = 1.0e-9;

/// First hit of the ray `origin + t dir` (`dir` unit length), 0 <= t <= `max_t`, against `shape`
/// placed at `xf`. Returns (t, world-space surface normal facing the ray). A ray starting inside
/// a solid hits at t = 0 with normal -dir.
pub fn ray_shape(shape: &Shape, xf: &Isometry, origin: Vec3, dir: Vec3, max_t: f32) -> Option<(f32, Vec3)> {
    if !(max_t >= 0.0) || !origin.is_finite() || !dir.is_finite() { return None; }
    let inv = xf.rot.conjugate();
    let o = inv * (origin - xf.pos);
    let d = inv * dir;
    let local = match *shape {
        Shape::Sphere { r } => ray_sphere(o, d, Vec3::ZERO, r.abs()),
        Shape::Box { hx, hy, hz } => ray_box(o, d, Vec3::new(hx.abs(), hy.abs(), hz.abs())),
        Shape::Capsule { r, hh } => ray_capsule(o, d, r.abs(), hh.abs()),
        Shape::Cylinder { r, hh } => ray_cylinder(o, d, r.abs(), hh.abs()),
        Shape::Triangle(tri) => {
            let ray = Ray::new(o, d);
            tri.intersect_ray(&ray, 0.0, max_t).map(|(t, _)| {
                let n = tri.normal();
                (t, if n.dot(d) > 0.0 { -n } else { n })
            })
        }
        Shape::Plane(pl) => {
            let dn = pl.n.dot(d);
            if dn.abs() < EPS { None } else {
                let t = -(pl.n.dot(o) + pl.d) / dn;
                if t >= 0.0 { Some((t, if dn > 0.0 { -pl.n } else { pl.n })) } else { None }
            }
        }
    }?;
    let (t, n) = local;
    if t <= max_t && t.is_finite() { Some((t, xf.rot * n)) } else { None }
}

fn ray_sphere(o: Vec3, d: Vec3, c: Vec3, r: f32) -> Option<(f32, Vec3)> {
    let m = o - c;
    let b = m.dot(d);
    let cc = m.length_squared() - r * r;
    if cc <= 0.0 { return Some((0.0, -d)); }
    if b > 0.0 { return None; }
    let disc = b * b - cc;
    if disc < 0.0 { return None; }
    let t = -b - disc.sqrt();
    let t = t.max(0.0);
    Some((t, (m + d * t).normalize_or_zero()))
}

fn ray_box(o: Vec3, d: Vec3, he: Vec3) -> Option<(f32, Vec3)> {
    let (mut tmin, mut tmax) = (0.0f32, f32::INFINITY);
    let mut normal = -d;
    let oa = o.to_array();
    let da = d.to_array();
    let ha = he.to_array();
    for ax in 0..3 {
        if da[ax].abs() < EPS {
            if oa[ax] < -ha[ax] || oa[ax] > ha[ax] { return None; }
            continue;
        }
        let inv = 1.0 / da[ax];
        let (mut t1, mut t2) = ((-ha[ax] - oa[ax]) * inv, (ha[ax] - oa[ax]) * inv);
        // Entering face: -axis when moving +, +axis when moving -
        let mut sign = -1.0;
        if t1 > t2 { std::mem::swap(&mut t1, &mut t2); sign = 1.0; }
        if t1 > tmin {
            tmin = t1;
            let mut n = [0.0; 3];
            n[ax] = sign;
            normal = Vec3::from_array(n);
        }
        tmax = tmax.min(t2);
        if tmin > tmax { return None; }
    }
    Some((tmin, normal))
}

/// Ray against the side of an infinite cylinder about local y, kept where |y| <= hh.
fn ray_tube(o: Vec3, d: Vec3, r: f32, hh: f32) -> Option<(f32, Vec3)> {
    let a = d.x * d.x + d.z * d.z;
    if a < EPS { return None; }
    let b = o.x * d.x + o.z * d.z;
    let c = o.x * o.x + o.z * o.z - r * r;
    let disc = b * b - a * c;
    if disc < 0.0 { return None; }
    let t = (-b - disc.sqrt()) / a;
    if t < 0.0 { return None; }
    let p = o + d * t;
    if p.y.abs() > hh { return None; }
    Some((t, Vec3::new(p.x, 0.0, p.z).normalize_or_zero()))
}

fn ray_capsule(o: Vec3, d: Vec3, r: f32, hh: f32) -> Option<(f32, Vec3)> {
    // Inside: distance from the core segment
    let core = Vec3::new(0.0, o.y.clamp(-hh, hh), 0.0);
    if (o - core).length_squared() <= r * r { return Some((0.0, -d)); }
    let mut best: Option<(f32, Vec3)> = ray_tube(o, d, r, hh);
    for cy in [hh, -hh] {
        if let Some(h) = ray_sphere(o, d, Vec3::new(0.0, cy, 0.0), r) {
            if best.map_or(true, |b| h.0 < b.0) { best = Some(h); }
        }
    }
    best
}

fn ray_cylinder(o: Vec3, d: Vec3, r: f32, hh: f32) -> Option<(f32, Vec3)> {
    if o.y.abs() <= hh && o.x * o.x + o.z * o.z <= r * r { return Some((0.0, -d)); }
    let mut best = ray_tube(o, d, r, hh);
    if d.y.abs() >= EPS {
        for (cy, ny) in [(hh, 1.0f32), (-hh, -1.0)] {
            let t = (cy - o.y) / d.y;
            if t < 0.0 { continue; }
            let p = o + d * t;
            if p.x * p.x + p.z * p.z <= r * r && best.map_or(true, |b| t < b.0) {
                best = Some((t, Vec3::new(0.0, ny, 0.0)));
            }
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use riftphys_core::types::Quat;

    fn at(x: f32, y: f32, z: f32) -> Isometry { Isometry { pos: Vec3::new(x, y, z), rot: Quat::IDENTITY } }

    #[test]
    fn rotated_box_is_hit_on_its_face_not_its_bounds() {
        // A unit cube turned 45 degrees about y: its footprint is the square |x| + |z| <= 0.707,
        // its bounding box the square |x|, |z| <= 0.707. A ray down at (0.6, 0) is inside the
        // footprint and hits the top face at y = 0.5.
        let xf = Isometry { pos: Vec3::ZERO, rot: Quat::from_rotation_y(std::f32::consts::FRAC_PI_4) };
        let b = Shape::Box { hx: 0.5, hy: 0.5, hz: 0.5 };
        let (t, n) = ray_shape(&b, &xf, Vec3::new(0.6, 2.0, 0.0), -Vec3::Y, 10.0).unwrap();
        assert!((t - 1.5).abs() < 1e-5 && (n - Vec3::Y).length() < 1e-5);
        // Down at (0.6, 0.6): inside the bounding box, outside the footprint
        assert!(ray_shape(&b, &xf, Vec3::new(0.6, 2.0, 0.6), -Vec3::Y, 10.0).is_none());
        // Tilted box: a ray down onto a 30 degree slope hits the slope, normal tilted
        let tilt = Isometry { pos: Vec3::ZERO, rot: Quat::from_rotation_z(30f32.to_radians()) };
        let slab = Shape::Box { hx: 1.0, hy: 0.1, hz: 1.0 };
        let (t, n) = ray_shape(&slab, &tilt, Vec3::new(0.0, 1.0, 0.0), -Vec3::Y, 10.0).unwrap();
        assert!((t - (1.0 - 0.1 / 30f32.to_radians().cos())).abs() < 1e-5, "t {}", t);
        assert!((n.y - 30f32.to_radians().cos()).abs() < 1e-5);
    }

    #[test]
    fn sphere_capsule_cylinder() {
        let s = Shape::Sphere { r: 0.5 };
        let (t, n) = ray_shape(&s, &at(0.0, 0.0, 0.0), Vec3::new(-3.0, 0.0, 0.0), Vec3::X, 10.0).unwrap();
        assert!((t - 2.5).abs() < 1e-5 && (n + Vec3::X).length() < 1e-5);
        assert!(ray_shape(&s, &at(0.0, 0.0, 0.0), Vec3::new(-3.0, 0.6, 0.0), Vec3::X, 10.0).is_none());
        assert!(ray_shape(&s, &at(0.0, 0.0, 0.0), Vec3::new(-3.0, 0.0, 0.0), Vec3::X, 2.0).is_none());

        let c = Shape::Capsule { r: 0.2, hh: 0.5 };
        let (t, _) = ray_shape(&c, &at(0.0, 0.0, 0.0), Vec3::new(-1.0, 0.3, 0.0), Vec3::X, 10.0).unwrap();
        assert!((t - 0.8).abs() < 1e-5);
        let (t, n) = ray_shape(&c, &at(0.0, 0.0, 0.0), Vec3::new(0.0, 2.0, 0.0), -Vec3::Y, 10.0).unwrap();
        assert!((t - 1.3).abs() < 1e-5 && (n - Vec3::Y).length() < 1e-5);
        // Past the end cap's side: misses
        assert!(ray_shape(&c, &at(0.0, 0.0, 0.0), Vec3::new(-1.0, 0.71, 0.0), Vec3::X, 10.0).is_none());

        let y = Shape::Cylinder { r: 0.2, hh: 0.5 };
        let (t, n) = ray_shape(&y, &at(0.0, 0.0, 0.0), Vec3::new(0.15, 2.0, 0.0), -Vec3::Y, 10.0).unwrap();
        assert!((t - 1.5).abs() < 1e-5 && (n - Vec3::Y).length() < 1e-5);
        assert!(ray_shape(&y, &at(0.0, 0.0, 0.0), Vec3::new(0.25, 2.0, 0.0), -Vec3::Y, 10.0).is_none());
    }

    #[test]
    fn start_inside_hits_at_zero() {
        let b = Shape::Box { hx: 1.0, hy: 1.0, hz: 1.0 };
        let (t, _) = ray_shape(&b, &at(0.0, 0.0, 0.0), Vec3::ZERO, Vec3::X, 5.0).unwrap();
        assert_eq!(t, 0.0);
    }
}
