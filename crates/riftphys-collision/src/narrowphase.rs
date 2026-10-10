// File: crates/riftphys-collision/src/narrowphase.rs

use riftphys_core::types::{Isometry, Vec3};
use riftphys_geom::{Shape, Triangle};

#[derive(Copy, Clone, Debug, Default)]
pub struct ContactPoint {
    pub position:    Vec3,
    pub normal:      Vec3, // points from A → B
    pub penetration: f32,  // includes contact slop
}

/// Zero-allocation manifold. Max 4 points (sufficient for Box-Box).
#[derive(Clone, Debug, Default)]
pub struct ContactManifold {
    pub normal: Vec3,           // shared normal for manifold
    pub points: [ContactPoint; 4],
    pub len: usize,
}

impl ContactManifold {
    #[inline]
    pub fn push(&mut self, pt: ContactPoint) {
        if self.len < 4 {
            self.points[self.len] = pt;
            self.len += 1;
        }
    }
}

#[inline]
fn q6(x: f32) -> f32 {
    if x.is_finite() {
        (x * 1.0e6_f32).round() * 1.0e-6_f32
    } else {
        0.0
    }
}

#[inline]
fn q6_vec3(v: Vec3) -> Vec3 {
    Vec3::new(q6(v.x), q6(v.y), q6(v.z))
}

#[inline]
fn q6_norm_vec3(v: Vec3, fallback: Vec3) -> Vec3 {
    let qv = q6_vec3(v);
    let len2 = qv.length_squared();

    if len2 > 1.0e-12 {
        let inv = 1.0 / len2.sqrt();
        q6_vec3(qv * inv)
    } else {
        fallback
    }
}

#[inline]
fn finite_vec3(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

#[inline]
pub fn flip_manifold(mut m: ContactManifold) -> ContactManifold {
    m.normal = -m.normal;
    for i in 0..m.len {
        m.points[i].normal = m.normal;
    }
    m
}

/// Canonical public entry for dynamic collider pairs.
/// - `normal` points from A → B.
/// - `contact_dist` is a slop / speculative distance (meters).
pub fn collide_shapes(
    shape_a: &Shape,
    xf_a: &Isometry,
    shape_b: &Shape,
    xf_b: &Isometry,
    contact_dist: f32,
) -> Option<ContactManifold> {
    use Shape::*;

    if !finite_vec3(xf_a.pos) || !finite_vec3(xf_b.pos) {
        return None;
    }

    let contact_dist = q6(contact_dist.max(0.0));

    match (shape_a, shape_b) {
        (Sphere { r: ra }, Sphere { r: rb }) => {
            collide_sphere_sphere(xf_a.pos, *ra, xf_b.pos, *rb, contact_dist)
        }
        (Capsule { r: ra, hh: ha }, Sphere { r: rb }) => {
            collide_capsule_sphere(xf_a, *ra, *ha, xf_b.pos, *rb, contact_dist)
        }
        (Sphere { r: ra }, Capsule { r: rb, hh: hb }) => {
            collide_capsule_sphere(xf_b, *rb, *hb, xf_a.pos, *ra, contact_dist)
                .map(flip_manifold)
        }
        (Capsule { r: ra, hh: ha }, Capsule { r: rb, hh: hb }) => {
            collide_capsule_capsule(xf_a, *ra, *ha, xf_b, *rb, *hb, contact_dist)
        }
        (Box { hx, hy, hz }, Sphere { r }) => {
            collide_box_sphere(xf_a, Vec3::new(*hx, *hy, *hz), xf_b.pos, *r, contact_dist)
        }
        (Sphere { r }, Box { hx, hy, hz }) => {
            collide_box_sphere(xf_b, Vec3::new(*hx, *hy, *hz), xf_a.pos, *r, contact_dist)
                .map(flip_manifold)
        }
        (Box { hx, hy, hz }, Capsule { r, hh }) => {
            collide_box_capsule(xf_a, Vec3::new(*hx, *hy, *hz), xf_b, *r, *hh, contact_dist)
        }
        (Capsule { r, hh }, Box { hx, hy, hz }) => {
            collide_box_capsule(xf_b, Vec3::new(*hx, *hy, *hz), xf_a, *r, *hh, contact_dist)
                .map(flip_manifold)
        }

        // --- NEW: TRIANGLE MATH HANDLERS ---
        (Sphere { r }, Triangle(tri)) => {
            collide_sphere_triangle(xf_a.pos, *r, tri, xf_b, contact_dist)
        }
        (Triangle(tri), Sphere { r }) => {
            collide_sphere_triangle(xf_b.pos, *r, tri, xf_a, contact_dist)
                .map(flip_manifold)
        }
        (Capsule { r, hh }, Triangle(tri)) => {
            collide_capsule_triangle(xf_a, *r, *hh, tri, xf_b, contact_dist)
        }
        (Triangle(tri), Capsule { r, hh }) => {
            collide_capsule_triangle(xf_b, *r, *hh, tri, xf_a, contact_dist)
                .map(flip_manifold)
        }
        // --- ROBUST BOX-TRIANGLE ROUTING ---
        (Shape::Box { .. }, Shape::Triangle(tri)) => {
            // Re-extract extents safely
            if let Shape::Box { hx, hy, hz } = shape_a {
                collide_box_triangle(xf_a, Vec3::new(*hx, *hy, *hz), tri, xf_b, contact_dist)
            } else {
                None
            }
        }
        (Shape::Triangle(tri), Shape::Box { .. }) => {
            if let Shape::Box { hx, hy, hz } = shape_b {
                collide_box_triangle(xf_b, Vec3::new(*hx, *hy, *hz), tri, xf_a, contact_dist)
                    .map(flip_manifold)
            } else {
                None
            }
        }

        // --- CATCH-ALL DEBUGGER ---
        (a, b) => {
            // This will tell us EXACTLY what the two shapes are that failed
            // println!("Narrowphase Dispatch Miss: {:?} vs {:?}", a, b);
            None
        }
        _ => None,
    }
}

/* ───────────────────────── Sphere / Capsule Logic ───────────────────────── */
fn collide_box_triangle(
    box_xf: &Isometry,
    box_extents: Vec3,
    tri: &Triangle,
    tri_xf: &Isometry,
    contact_dist: f32,
) -> Option<ContactManifold> {
    // 1. Transform triangle to world space
    let ta = tri_xf.pos + tri_xf.rot * tri.a;
    let tb = tri_xf.pos + tri_xf.rot * tri.b;
    let tc = tri_xf.pos + tri_xf.rot * tri.c;
    let normal = (tb - ta).cross(tc - ta).normalize_or_zero();

    // 2. Sample the 8 corners of the box
    let mut deepest_pt = Vec3::ZERO;
    let mut max_pen = -f32::MAX;
    let mut hit = false;

    // Local corners of the box
    for x in [-1.0, 1.0] {
        for y in [-1.0, 1.0] {
            for z in [-1.0, 1.0] {
                let corner_l = Vec3::new(x, y, z) * box_extents;
                let corner_w = box_xf.pos + box_xf.rot * corner_l;

                // Find closest point on triangle for this corner
                let cp = closest_point_triangle(corner_w, ta, tb, tc);
                let delta = corner_w - cp;
                let dist = delta.dot(normal); // Distance along triangle normal

                // If the corner is "behind" the triangle normal within contact distance
                if dist <= contact_dist {
                    let pen = contact_dist - dist;
                    if pen > max_pen {
                        max_pen = pen;
                        deepest_pt = cp;
                        hit = true;
                    }
                }
            }
        }
    }

    // 3. Sample Triangle vertices against Box (Reverse check)
    for tri_v in [ta, tb, tc] {
        let p_local = box_xf.rot.conjugate() * (tri_v - box_xf.pos);
        let closest_l = p_local.clamp(-box_extents, box_extents);
        let closest_w = box_xf.pos + box_xf.rot * closest_l;
        let dist = (tri_v - closest_w).length();

        if dist <= contact_dist {
            let pen = contact_dist - dist;
            if pen > max_pen {
                max_pen = pen;
                deepest_pt = closest_w;
                hit = true;
            }
        }
    }

    if !hit { return None; }

    let mut manifold = ContactManifold {
        normal: q6_norm_vec3(-normal, -normal), // Normal points A -> B (Box -> Terrain)
        ..Default::default()
    };

    manifold.push(ContactPoint {
        position: q6_vec3(deepest_pt),
        normal: manifold.normal,
        penetration: q6(max_pen),
    });

    Some(manifold)
}
fn collide_sphere_sphere(
    center_a: Vec3,
    ra: f32,
    center_b: Vec3,
    rb: f32,
    contact_dist: f32,
) -> Option<ContactManifold> {
    let delta = center_b - center_a;
    let dist2 = delta.length_squared();

    let ra = q6(ra.max(0.0));
    let rb = q6(rb.max(0.0));
    let allowed = q6(ra + rb + contact_dist);

    if !dist2.is_finite() || dist2 > allowed * allowed {
        return None;
    }

    let dist = dist2.sqrt();
    let raw_n = if dist > 1.0e-6 { delta / dist } else { Vec3::new(1.0, 0.0, 0.0) };
    let normal = q6_norm_vec3(raw_n, Vec3::new(1.0, 0.0, 0.0));

    let penetration = q6((allowed - dist).max(0.0));
    let pa = center_a + normal * ra;
    let pb = center_b - normal * rb;
    let position = q6_vec3((pa + pb) * 0.5);

    let mut manifold = ContactManifold { normal, ..Default::default() };
    manifold.push(ContactPoint { position, normal, penetration });
    Some(manifold)
}

#[inline]
fn capsule_segment_world(xf: &Isometry, half: f32) -> (Vec3, Vec3) {
    let axis = xf.rot * Vec3::Y;
    let h = half.abs();
    (xf.pos - axis * h, xf.pos + axis * h)
}

#[inline]
fn closest_point_segment(p: Vec3, a: Vec3, b: Vec3) -> (Vec3, f32) {
    let ab = b - a;
    let ab_len2 = ab.length_squared();
    if ab_len2 <= 1.0e-12 {
        return (a, 0.0);
    }
    let t = ((p - a).dot(ab) / ab_len2).clamp(0.0, 1.0);
    (a + ab * t, t)
}

fn closest_points_segment_segment(p0: Vec3, p1: Vec3, q0: Vec3, q1: Vec3) -> (Vec3, Vec3) {
    let u = p1 - p0;
    let v = q1 - q0;
    let w = p0 - q0;
    let a = u.dot(u);
    let b = u.dot(v);
    let c = v.dot(v);
    let d = u.dot(w);
    let e = v.dot(w);
    let d0 = a * c - b * b;

    let mut s_n;
    let mut t_n;
    let mut s_d = d0;
    let mut t_d = d0;

    const EPS: f32 = 1.0e-6;

    if d0 < EPS {
        s_n = 0.0; s_d = 1.0;
        t_n = e;   t_d = c;
    } else {
        s_n = b * e - c * d;
        t_n = a * e - b * d;
        if s_n < 0.0 {
            s_n = 0.0; t_n = e; t_d = c;
        } else if s_n > s_d {
            s_n = s_d; t_n = e + b; t_d = c;
        }
    }

    if t_n < 0.0 {
        t_n = 0.0;
        if -d < 0.0 { s_n = 0.0; }
        else if -d > a { s_n = s_d; }
        else { s_n = -d; s_d = a; }
    } else if t_n > t_d {
        t_n = t_d;
        if (-d + b) < 0.0 { s_n = 0.0; }
        else if (-d + b) > a { s_n = s_d; }
        else { s_n = -d + b; s_d = a; }
    }

    let sc = if s_n.abs() < EPS { 0.0 } else { s_n / s_d };
    let tc = if t_n.abs() < EPS { 0.0 } else { t_n / t_d };

    (p0 + u * sc, q0 + v * tc)
}

fn collide_capsule_sphere(
    cap_xf: &Isometry,
    cap_r: f32,
    cap_half: f32,
    sphere_center: Vec3,
    sphere_r: f32,
    contact_dist: f32,
) -> Option<ContactManifold> {
    let (a, b) = capsule_segment_world(cap_xf, cap_half);
    let (q, _t) = closest_point_segment(sphere_center, a, b);
    collide_sphere_sphere(q, cap_r, sphere_center, sphere_r, contact_dist)
}

fn collide_capsule_capsule(
    xf_a: &Isometry,
    ra: f32,
    ha: f32,
    xf_b: &Isometry,
    rb: f32,
    hb: f32,
    contact_dist: f32,
) -> Option<ContactManifold> {
    let (a0, a1) = capsule_segment_world(xf_a, ha);
    let (b0, b1) = capsule_segment_world(xf_b, hb);
    let (pa, pb) = closest_points_segment_segment(a0, a1, b0, b1);
    collide_sphere_sphere(pa, ra, pb, rb, contact_dist)
}

/* ───────────────────────── Box Logic (Voxel Ready) ───────────────────────── */
fn collide_box_sphere(
    box_xf: &Isometry,
    box_extents: Vec3,
    sphere_center: Vec3,
    sphere_r: f32,
    contact_dist: f32,
) -> Option<ContactManifold> {
    let p_local = q6_vec3(box_xf.rot.conjugate() * (sphere_center - box_xf.pos));
    let mut closest_local = p_local.clamp(-box_extents, box_extents);
    let mut inside = false;

    if closest_local == p_local {
        inside = true;
        let abs_p = p_local.abs();
        let dist_to_face = box_extents - abs_p;

        if dist_to_face.x < dist_to_face.y && dist_to_face.x < dist_to_face.z {
            closest_local.x = if p_local.x > 0.0 { box_extents.x } else { -box_extents.x };
        } else if dist_to_face.y < dist_to_face.z {
            closest_local.y = if p_local.y > 0.0 { box_extents.y } else { -box_extents.y };
        } else {
            closest_local.z = if p_local.z > 0.0 { box_extents.z } else { -box_extents.z };
        }
    }

    let closest_world = q6_vec3(box_xf.pos + box_xf.rot * closest_local);
    let delta = sphere_center - closest_world;
    let dist2 = delta.length_squared();
    let dist = dist2.sqrt();

    let r = q6(sphere_r.max(0.0));
    let allowed = q6(r + contact_dist);

    if dist > allowed {
        return None;
    }

    let raw_n = if dist > 1.0e-6 { delta / dist } else { box_xf.rot * Vec3::Y };
    let normal = q6_norm_vec3(raw_n, box_xf.rot * Vec3::Y);
    let penetration = q6(allowed - dist);

    let position = if inside {
        q6_vec3(closest_world)
    } else {
        q6_vec3((closest_world + (sphere_center - normal * r)) * 0.5)
    };

    let mut manifold = ContactManifold { normal, ..Default::default() };
    manifold.push(ContactPoint { position, normal, penetration });
    Some(manifold)
}

fn collide_box_capsule(
    box_xf: &Isometry,
    box_extents: Vec3,
    cap_xf: &Isometry,
    cap_r: f32,
    cap_hh: f32,
    contact_dist: f32,
) -> Option<ContactManifold> {
    let (a_w, b_w) = capsule_segment_world(cap_xf, cap_hh);
    let inv_rot = box_xf.rot.conjugate();
    let a_l = q6_vec3(inv_rot * (a_w - box_xf.pos));
    let b_l = q6_vec3(inv_rot * (b_w - box_xf.pos));

    let (mid_l, _) = closest_point_segment(Vec3::ZERO, a_l, b_l);

    let test_points_l = [a_l, b_l, mid_l];
    let mut best_m: Option<ContactManifold> = None;

    for p_l in test_points_l {
        let p_w = box_xf.pos + box_xf.rot * p_l;
        if let Some(m) = collide_box_sphere(box_xf, box_extents, p_w, cap_r, contact_dist) {
            if best_m.is_none() || m.points[0].penetration > best_m.as_ref().unwrap().points[0].penetration {
                best_m = Some(m);
            }
        }
    }
    best_m
}

/* ───────────────────────── TRIANGLE LOGIC (NEW) ───────────────────────── */

/// Analytic closest point on a 3D triangle to a point P
fn closest_point_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;

    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);

    if d1 <= 0.0 && d2 <= 0.0 { return a; }

    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 { return b; }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return a + v * ab;
    }

    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 { return c; }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return a + w * ac;
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    a + ab * v + ac * w
}

fn collide_sphere_triangle(
    sphere_center: Vec3,
    sphere_r: f32,
    tri: &Triangle,
    tri_xf: &Isometry,
    contact_dist: f32,
) -> Option<ContactManifold> {
    // Transform triangle to world space
    let a = tri_xf.pos + tri_xf.rot * tri.a;
    let b = tri_xf.pos + tri_xf.rot * tri.b;
    let c = tri_xf.pos + tri_xf.rot * tri.c;

    let closest = closest_point_triangle(sphere_center, a, b, c);
    let delta = sphere_center - closest;
    let dist2 = delta.length_squared();

    let r = q6(sphere_r.max(0.0));
    let allowed = q6(r + contact_dist);

    if dist2 > allowed * allowed {
        return None;
    }

    let dist = dist2.sqrt();

    // Triangle normal (fallback if center is exactly on the triangle)
    let tri_n = (b - a).cross(c - a).normalize_or_zero();
    let raw_n = if dist > 1.0e-6 { delta / dist } else { tri_n };
    let normal = q6_norm_vec3(raw_n, tri_n);

    let penetration = q6(allowed - dist);
    let position = q6_vec3(closest);

    let mut manifold = ContactManifold { normal, ..Default::default() };
    manifold.push(ContactPoint { position, normal, penetration });
    Some(manifold)
}

fn collide_capsule_triangle(
    cap_xf: &Isometry,
    cap_r: f32,
    cap_hh: f32,
    tri: &Triangle,
    tri_xf: &Isometry,
    contact_dist: f32,
) -> Option<ContactManifold> {
    let (a_w, b_w) = capsule_segment_world(cap_xf, cap_hh);

    // Evaluate 3 points along the capsule (Top, Bottom, Middle) to find the deepest penetration
    let mid_w = (a_w + b_w) * 0.5;
    let test_points = [a_w, b_w, mid_w];

    let mut best_m: Option<ContactManifold> = None;

    for p in test_points {
        if let Some(m) = collide_sphere_triangle(p, cap_r, tri, tri_xf, contact_dist) {
            if best_m.is_none() || m.points[0].penetration > best_m.as_ref().unwrap().points[0].penetration {
                best_m = Some(m);
            }
        }
    }
    best_m
}