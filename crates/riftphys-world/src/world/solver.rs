// SPDX-License-Identifier: RiftForged Proprietary License
use rayon::prelude::*;
use super::geom::orthonormal_basis;
use riftphys_core::{Mat3, Vec3};
use super::world::{Contact, World, TERRAIN_CI, WarmContact, WarmKey, WarmPatch, WarmRow};
use super::islands::Island;
use riftphys_materials::materials as mats;
use std::collections::BTreeMap;

/// A new contact takes last step's entry of the same key whose anchor (body A's frame) is
/// nearest, within this distance (AT_REST_SPEC R1). Box anchors are corners, so 1 cm is far
/// below any feature spacing.
const WARM_MATCH_TOL: f32 = 0.01;

/// Below this many items a batch runs on the calling thread: rayon's fork/join costs more than
/// the work for small scenes (it made a 9-body biped ~10x slower). Level scheduling makes the
/// result identical either way.
const PAR_MIN: usize = 64;

#[inline]
fn run_batch<F: Fn(&usize) + Sync + Send>(batch: &[usize], f: F) {
    if batch.len() >= PAR_MIN { batch.par_iter().for_each(f) } else { batch.iter().for_each(f) }
}

#[derive(Copy, Clone)]
struct UnsafeSyncBodies(*mut riftphys_dynamics::dynamics::Bodies);
unsafe impl Send for UnsafeSyncBodies {}
unsafe impl Sync for UnsafeSyncBodies {}

impl UnsafeSyncBodies {
    #[inline(always)]
    unsafe fn get_mut(&self) -> &mut riftphys_dynamics::dynamics::Bodies { &mut *self.0 }
}

#[derive(Copy, Clone)]
struct UnsafeAcc(*mut f32);
unsafe impl Send for UnsafeAcc {}
unsafe impl Sync for UnsafeAcc {}

impl UnsafeAcc {
    #[inline(always)]
    unsafe fn get_mut_slice(&self, len: usize) -> &mut [f32] {
        std::slice::from_raw_parts_mut(self.0, len)
    }
}

/// One contact point. `normal` points from A to B; `ra`/`rb` run from each body's centre to the
/// contact point, so impulses carry torque as well as force.
#[derive(Clone, Copy)]
struct ContactConstraint {
    ai: u32, bi: u32,
    b_is_terrain: bool,
    inv_a: f32,
    inv_b: f32,
    ii_a: Mat3,
    ii_b: Mat3,
    ra: Vec3, rb: Vec3,
    /// Effective masses along the normal and the two tangents
    m_n: f32, m_t1: f32, m_t2: f32,
    normal: Vec3, t1: Vec3, t2: Vec3,
    /// Friction: Stribeck curve from static mu_s at rest toward kinetic mu_k when sliding
    mu_s: f32, mu_k: f32, stribeck_v: f32,
    /// Target normal separating speed: positive pushes a penetrating pair apart, negative lets a
    /// pair still `gap` apart approach by up to gap/dt (speculative contact)
    depth_bias: f32,
    restitution_bias: f32,
    /// Compliant pair (`MatPairProps::stiffness` > 0): the normal impulse is the Hunt-Crossley
    /// force over the step, `soft_jn`, applied once; no rigid normal row, shock or restitution
    soft: bool,
    soft_jn: f32,
    /// Friction is solved by this point's patch (see `FrictionPatch`), not here
    patch: bool,
}

/// One manifold's friction under patch friction (`MatPairProps::patch_friction`): sliding at
/// the centroid of its points and torsion about the normal, both limited by the manifold's total
/// normal impulse. `at` is a constraint located at the centroid (lever arms, effective masses).
struct FrictionPatch {
    points: Vec<usize>,
    at: ContactConstraint,
    /// Mean distance from the centroid over the contact patch (m), the torsional lever arm
    radius: f32,
    /// Effective mass for relative rotation about the normal
    m_tw: f32,
}

/// Mean distance from the origin over the convex polygon `pts` (plane coordinates, around the
/// origin). Two points: a uniformly loaded segment, a quarter of its length.
fn patch_radius(pts: &[(f32, f32)]) -> f32 {
    if pts.len() < 2 { return 0.0; }
    let mut p: Vec<(f32, f32)> = pts.to_vec();
    p.sort_by_key(|a| riftphys_core::det::sort_key6(a.1.atan2(a.0)));
    let (mut area, mut moment) = (0.0f32, 0.0f32);
    const N: usize = 6;
    for i in 0..p.len() {
        let (a, b) = (p[i], p[(i + 1) % p.len()]);
        let tri = 0.5 * (a.0 * b.1 - a.1 * b.0).abs();
        if tri <= 0.0 { continue; }
        // Integrate |x| over the triangle (origin, a, b): N x N subdivision, centroid rule
        let sub = tri / (N * N) as f32;
        for r in 0..N {
            for c in 0..(N - r) {
                for up in [true, false] {
                    if !up && c + r + 1 >= N { continue; }
                    let (u, v) = if up { (r as f32 + 1.0 / 3.0, c as f32 + 1.0 / 3.0) } else { (r as f32 + 2.0 / 3.0, c as f32 + 2.0 / 3.0) };
                    let (u, v) = (u / N as f32, v / N as f32);
                    let x = (a.0 * u + b.0 * v, a.1 * u + b.1 * v);
                    moment += (x.0 * x.0 + x.1 * x.1).sqrt() * sub;
                }
            }
        }
        area += tri;
    }
    if area > 1e-12 { return moment / area; }
    // Degenerate (collinear points): a uniformly loaded segment
    let mut len = 0.0f32;
    for a in &p { for b in &p { len = len.max(((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()); } }
    0.25 * len
}

/// One velocity-level row of a D6 joint. Linear rows constrain the anchors' relative velocity
/// along `u` (lever arms `ra`, `rb`); angular rows the relative angular velocity along `u`.
/// The row keeps the relative velocity inside [vmin, vmax] with its accumulated impulse in
/// [lam_min, lam_max]: equality rows (locked axes) are [0, 0] / unbounded, a lower limit is
/// [speculative bound, inf) / [0, inf), an upper limit the mirror image. A soft row (`gamma` > 0,
/// a damper) solves v + gamma lam = target instead of v = target; rigid rows have gamma 0.
#[derive(Clone, Copy)]
struct JointRow {
    a: u32, b: u32,
    angular: bool,
    u: Vec3, ra: Vec3, rb: Vec3,
    /// Inverse masses and world inverse inertias (zero for static bodies)
    wa: f32, wb: f32, ia: Mat3, ib: Mat3,
    /// 1 / (k + gamma), k the row's inverse effective mass
    m_eff: f32,
    target: f32,
    lam_min: f32, lam_max: f32,
    /// Softness: 1 / (damping dt) for a damper row, 0 for rigid rows
    gamma: f32,
    /// Which row of its joint this is: angular (bit 5), axis (bits 3-4), kind (bits 0-2:
    /// equality, motor, lower limit, upper limit, damping, friction). Warm starting matches rows
    /// by it.
    tag: u8,
}

#[inline(always)]
fn joint_row_vel(bodies: &riftphys_dynamics::dynamics::Bodies, r: &JointRow) -> f32 {
    let (a, b) = (bodies.vel(r.a), bodies.vel(r.b));
    if r.angular { (b.ang - a.ang).dot(r.u) }
    else { ((b.lin + b.ang.cross(r.rb)) - (a.lin + a.ang.cross(r.ra))).dot(r.u) }
}

#[inline(always)]
fn joint_row_apply(bodies: &mut riftphys_dynamics::dynamics::Bodies, r: &JointRow, lam: f32) {
    let p = r.u * lam;
    if r.angular {
        if r.wa > 0.0 { bodies.apply_angular_impulse(r.a, -p); }
        if r.wb > 0.0 { bodies.apply_angular_impulse(r.b, p); }
    } else {
        if r.wa > 0.0 { bodies.apply_impulse(r.a, -p); bodies.apply_angular_impulse(r.a, r.ra.cross(-p)); }
        if r.wb > 0.0 { bodies.apply_impulse(r.b, p); bodies.apply_angular_impulse(r.b, r.rb.cross(p)); }
    }
}

/// Velocity rows of a D6 joint: locked axes as equalities, limits as speculative inequalities.
/// Drives and soft (compliant) rows stay at position level.
fn d6_rows(bodies: &riftphys_dynamics::dynamics::Bodies, j: &riftphys_articulation::bodies::D6Joint, dt: f32, j_idx: u32, rotor: &BTreeMap<(u32, u8), f32>, out: &mut Vec<JointRow>) {
    let (a, b) = (j.a.0, j.b.0);
    let (pa, pb) = (bodies.pose(a), bodies.pose(b));
    let rot_a = pa.rot * j.fa.rot;
    let rot_b = pb.rot * j.fb.rot;
    let xa = pa.pos + pa.rot * j.fa.pos;
    let xb = pb.pos + pb.rot * j.fb.pos;
    let axes = [rot_a * Vec3::X, rot_a * Vec3::Y, rot_a * Vec3::Z];
    let (wa, wb) = (bodies.inv_mass_of(a), bodies.inv_mass_of(b));
    if wa == 0.0 && wb == 0.0 { return; }
    let (ia, ib) = (bodies.inv_inertia_world(a), bodies.inv_inertia_world(b));
    let (ra, rb) = (xa - pa.pos, xb - pb.pos);
    let d = xb - xa;
    let mut q_err = rot_a.conjugate() * rot_b;
    if q_err.w < 0.0 { q_err = -q_err; }
    let e_world = rot_a * (Vec3::new(q_err.x, q_err.y, q_err.z) * 2.0);

    for angular in [false, true] {
        for i in 0..3 {
            let row = if angular { &j.r[i] } else { &j.t[i] };
            let u = axes[i];
            let k = |u: Vec3| if angular {
                u.dot(ia.mul_vec3(u)) + u.dot(ib.mul_vec3(u))
            } else {
                let (ca, cb) = (ra.cross(u), rb.cross(u));
                wa + wb + ca.dot(ia.mul_vec3(ca)) + cb.dot(ib.mul_vec3(cb))
            };
            let tag = ((angular as u8) << 5) | ((i as u8) << 3);
            // Joint damping and Coulomb friction (PARITY_SPEC W3a), on any axis, free ones too.
            // The damper is implicit: v + gamma lam = 0 with gamma = 1 / (c dt) gives the impulse
            // -c dt v_end, stable for any c (a rigid lock as c grows).
            if row.damping > 0.0 || row.friction > 0.0 {
                let kk = k(u);
                if kk > 1e-9 {
                    let base = JointRow { a, b, angular, u, ra, rb, wa, wb, ia, ib, m_eff: 1.0 / kk, target: 0.0, lam_min: f32::NEG_INFINITY, lam_max: f32::INFINITY, gamma: 0.0, tag };
                    if row.damping > 0.0 {
                        let gamma = 1.0 / (row.damping * dt);
                        out.push(JointRow { gamma, m_eff: 1.0 / (kk + gamma), tag: tag | 4, ..base });
                    }
                    if row.friction > 0.0 {
                        let cap = row.friction * dt;
                        out.push(JointRow { lam_min: -cap, lam_max: cap, tag: tag | 5, ..base });
                    }
                }
            }
            // Armature (PARITY_SPEC W4b): the geared rotor's momentum balance, a soft row
            // w_rel + lam / J_a = w_rotor (the links take lam, the rotor -lam). Under a torque
            // across the joint w_rel then changes by tau dt / (I + J_a), the reduced-coordinate
            // answer. w_rotor is carried state; the first step starts it from the links.
            if angular && row.armature > 0.0 {
                let kk = k(u);
                if kk > 1e-9 {
                    let gamma = 1.0 / row.armature;
                    let w_rel = (bodies.vel(b).ang - bodies.vel(a).ang).dot(u);
                    let target = rotor.get(&(j_idx, i as u8)).copied().unwrap_or(w_rel);
                    out.push(JointRow { a, b, angular, u, ra, rb, wa, wb, ia, ib, m_eff: 1.0 / (kk + gamma), target, lam_min: f32::NEG_INFINITY, lam_max: f32::INFINITY, gamma, tag: tag | 6 });
                }
            }
            if !row.enabled || row.compliance > 0.0 { continue; }
            // Position drives stay at position level; velocity drives become motor rows below
            if row.drive.map_or(false, |d| !d.vel_mode) { continue; }
            let k = k(u);
            if k <= 1e-9 { continue; }
            let base = JointRow { a, b, angular, u, ra, rb, wa, wb, ia, ib, m_eff: 1.0 / k, target: 0.0, lam_min: f32::NEG_INFINITY, lam_max: f32::INFINITY, gamma: 0.0, tag };
            if let Some(drive) = row.drive {
                // Motor: drive the relative velocity to the target with at most max_impulse per
                // step (a constraint motor; adding kd x error straight to the velocity diverged
                // for kd > 2). Limits still apply through their own rows.
                let cap = drive.max_impulse.abs();
                out.push(JointRow { target: drive.target, lam_min: -cap, lam_max: cap, tag: tag | 1, ..base });
            }
            match row.limit {
                None if row.drive.is_some() => {}
                None => out.push(base),
                Some(lim) => {
                    // Same measure as the position solve (radians on a hinge's own axis)
                    let x = if angular { riftphys_articulation::bodies::d6_row_angle(j, rot_a, rot_b, i) } else { d.dot(u) };
                    // Approach the stop by at most what is left this step; never push out here
                    // (the position solve removes any violation)
                    out.push(JointRow { target: ((lim.min - x) / dt).min(0.0), lam_min: 0.0, tag: tag | 2, ..base });
                    out.push(JointRow { target: ((lim.max - x) / dt).max(0.0), lam_max: 0.0, lam_min: f32::NEG_INFINITY, tag: tag | 3, ..base });
                }
            }
        }
    }
}

/// Friction coefficient at tangential slip speed `slip`: Stribeck curve from mu_s at rest toward
/// mu_k when sliding.
#[inline(always)]
fn mu_dynamic(cs: &ContactConstraint, slip: f32) -> f32 {
    cs.mu_k + (cs.mu_s - cs.mu_k) * (-(slip / cs.stribeck_v).powi(2)).exp()
}

/// Tangential slip speed at the contact.
#[inline(always)]
fn slip_speed(bodies: &riftphys_dynamics::dynamics::Bodies, cs: &ContactConstraint) -> f32 {
    let v = contact_rel_vel(bodies, cs);
    let (vt1, vt2) = (v.dot(cs.t1), v.dot(cs.t2));
    (vt1 * vt1 + vt2 * vt2).sqrt()
}

/// Relative velocity of B with respect to A at the contact point.
#[inline(always)]
fn contact_rel_vel(bodies: &riftphys_dynamics::dynamics::Bodies, cs: &ContactConstraint) -> Vec3 {
    let a = bodies.vel(cs.ai);
    let va = a.lin + a.ang.cross(cs.ra);
    let vb = if cs.b_is_terrain { Vec3::ZERO } else {
        let b = bodies.vel(cs.bi);
        b.lin + b.ang.cross(cs.rb)
    };
    vb - va
}

/// Applies impulse `j` on B (and -j on A) at the contact point.
#[inline(always)]
fn apply_contact_impulse(bodies: &mut riftphys_dynamics::dynamics::Bodies, cs: &ContactConstraint, j: Vec3) {
    if cs.inv_a > 0.0 {
        bodies.apply_impulse(cs.ai, -j);
        bodies.apply_angular_impulse(cs.ai, cs.ra.cross(-j));
    }
    if cs.inv_b > 0.0 {
        bodies.apply_impulse(cs.bi, j);
        bodies.apply_angular_impulse(cs.bi, cs.rb.cross(j));
    }
}

impl World {
    /// The pair properties a contact is solved with (terrain contacts use the environment's material).
    pub(super) fn contact_pair_props(&self, c: &Contact) -> mats::MatPairProps {
        self.pair_props(self.colliders[c.a_collider].material.id,
                        if c.b_collider == TERRAIN_CI { c.env_material.unwrap_or(mats::MaterialId::Default) }
                        else { self.colliders[c.b_collider].material.id })
    }

    /// Sequential-impulse contact solve. `margin` is the contact distance the narrowphase added to
    /// every reported depth: true penetration is `depth - margin`, and pairs still apart are
    /// solved speculatively instead of being pushed apart.
    pub fn solve_islands_parallel(&mut self, islands: &[Island], contacts: &mut [Contact], dt: f32, margin: f32) {
        let iterations = self.solver_iters.0;
        let beta = 0.05;     // Lowered for stability
        let slop = 0.005;
        let num_bodies = self.bodies.len();

        // 1. Constraint Pre-processing
        let constraints: Vec<ContactConstraint> = contacts.iter().map(|c| {
            let ai = self.colliders[c.a_collider].body.0;
            let b_is_terrain = c.b_collider == TERRAIN_CI;
            let bi = if b_is_terrain { 0 } else { self.colliders[c.b_collider].body.0 };

            let a_awake = self.sleep.is_awake(ai as usize);
            let b_awake = if b_is_terrain { false } else { self.sleep.is_awake(bi as usize) };

            let inv_a = if a_awake { self.bodies.inv_mass_of(ai) } else { 0.0 };
            let inv_b = if b_awake { self.bodies.inv_mass_of(bi) } else { 0.0 };
            let ii_a = if inv_a > 0.0 { self.bodies.inv_inertia_world(ai) } else { Mat3::ZERO };
            let ii_b = if inv_b > 0.0 { self.bodies.inv_inertia_world(bi) } else { Mat3::ZERO };

            let pair = self.contact_pair_props(c);

            // Narrowphase normals point A -> B for every pair, terrain included
            let normal = c.normal;
            let (t1, t2) = orthonormal_basis(normal);
            let ra = c.point - self.bodies.pose(ai).pos;
            let rb = if b_is_terrain { Vec3::ZERO } else { c.point - self.bodies.pose(bi).pos };
            let eff = |d: Vec3| -> f32 {
                let ca = ra.cross(d);
                let cb = rb.cross(d);
                let k = inv_a + inv_b + ca.dot(ii_a.mul_vec3(ca)) + cb.dot(ii_b.mul_vec3(cb));
                if k > 1e-9 { 1.0 / k } else { 0.0 }
            };

            let mut cs = ContactConstraint {
                ai, bi, b_is_terrain,
                inv_a, inv_b, ii_a, ii_b, ra, rb,
                m_n: eff(normal), m_t1: eff(t1), m_t2: eff(t2),
                normal, t1, t2, mu_s: pair.mu_s, mu_k: pair.mu_k, stribeck_v: pair.stribeck_v.max(1e-4),
                depth_bias: 0.0,
                restitution_bias: 0.0,
                soft: pair.stiffness > 0.0,
                soft_jn: 0.0,
                patch: false,
            };
            let pen = c.depth - margin;
            cs.depth_bias = if pen > 0.0 { ((pen - slop).max(0.0) * (beta / dt)).min(2.0) } else { pen / dt };
            // Approach speed from the start of the step, before this step's forces
            let pre = |id: u32| self.pre_step_vel.get(id as usize).copied().unwrap_or_else(|| self.bodies.vel(id));
            let (va, vb) = (pre(ai), if b_is_terrain { Default::default() } else { pre(bi) });
            let v_pre = ((vb.lin + vb.ang.cross(rb)) - (va.lin + va.ang.cross(ra))).dot(normal);
            // Approach speed at mid-step (between the start-of-step and after-gravity velocities)
            let v_rel_n = 0.5 * (v_pre + contact_rel_vel(&self.bodies, &cs).dot(normal));
            if cs.soft {
                // Hunt-Crossley: f = k x (1 + d x'), x the true penetration, x' its rate (the
                // mid-step approach speed); never pulls. No rigid row, so nothing to bias.
                cs.depth_bias = 0.0;
                if pen > 0.0 {
                    let f = pair.stiffness * pen * (1.0 + pair.dissipation * (-v_rel_n));
                    cs.soft_jn = f.max(0.0) * dt;
                }
                return cs;
            }
            // Applied after the solve, only to contacts that carried load (see below)
            if v_rel_n < -1.0 { cs.restitution_bias = -pair.restitution * v_rel_n; }
            cs
        }).collect();

        // 1b. Friction patches: manifolds of two or more points on patch-friction pairs
        let mut constraints = constraints;
        let mut groups: std::collections::BTreeMap<(usize, usize), Vec<usize>> = std::collections::BTreeMap::new();
        for (i, c) in contacts.iter().enumerate() {
            if self.contact_pair_props(c).patch_friction { groups.entry((c.a_collider, c.b_collider)).or_default().push(i); }
        }
        let mut patches: Vec<FrictionPatch> = Vec::new();
        for (_, pts) in groups {
            if pts.len() < 2 { continue; }
            let first = constraints[pts[0]];
            let centroid = pts.iter().fold(Vec3::ZERO, |s, &i| s + contacts[i].point) / pts.len() as f32;
            let normal = pts.iter().fold(Vec3::ZERO, |s, &i| s + contacts[i].normal).normalize_or_zero();
            if normal == Vec3::ZERO { continue; }
            let (t1, t2) = orthonormal_basis(normal);
            let plane: Vec<(f32, f32)> = pts.iter().map(|&i| { let d = contacts[i].point - centroid; (d.dot(t1), d.dot(t2)) }).collect();
            let ra = centroid - self.bodies.pose(first.ai).pos;
            let rb = if first.b_is_terrain { Vec3::ZERO } else { centroid - self.bodies.pose(first.bi).pos };
            let eff = |d: Vec3| -> f32 {
                let (ca, cb) = (ra.cross(d), rb.cross(d));
                let k = first.inv_a + first.inv_b + ca.dot(first.ii_a.mul_vec3(ca)) + cb.dot(first.ii_b.mul_vec3(cb));
                if k > 1e-9 { 1.0 / k } else { 0.0 }
            };
            let kt = normal.dot(first.ii_a.mul_vec3(normal)) + normal.dot(first.ii_b.mul_vec3(normal));
            let at = ContactConstraint { ra, rb, normal, t1, t2, m_n: eff(normal), m_t1: eff(t1), m_t2: eff(t2), ..first };
            for &i in &pts { constraints[i].patch = true; }
            patches.push(FrictionPatch { points: pts, at, radius: patch_radius(&plane), m_tw: if kt > 1e-9 { 1.0 / kt } else { 0.0 } });
        }
        let constraints = constraints;

        // 2a. Graph Coloring (Level Scheduling) for Contacts
        let mut contact_batches: Vec<Vec<usize>> = Vec::with_capacity(16);
        let mut body_level = vec![0; num_bodies];

        for c_idx in 0..constraints.len() {
            let cs = &constraints[c_idx];
            // Only bodies that receive impulses need ordering; static bodies are read-only
            let write_a = cs.inv_a > 0.0;
            let write_b = !cs.b_is_terrain && cs.inv_b > 0.0;
            if !write_a && !write_b { continue; }
            let level_a = if write_a { body_level[cs.ai as usize] } else { 0 };
            let level_b = if write_b { body_level[cs.bi as usize] } else { 0 };

            let target_level = level_a.max(level_b);
            if target_level >= contact_batches.len() {
                contact_batches.push(Vec::new());
            }
            contact_batches[target_level].push(c_idx);

            if write_a { body_level[cs.ai as usize] = target_level + 1; }
            if write_b { body_level[cs.bi as usize] = target_level + 1; }
        }

        // 2b. Graph Coloring for Joints
        let mut joint_batches: Vec<Vec<usize>> = Vec::with_capacity(8);
        let mut joint_body_level = vec![0; num_bodies];
        let mut active_joints = Vec::new();
        for island in islands { active_joints.extend(&island.joint_indices); }

        // D6 joints at velocity level: solving them only by position projection made the
        // restoring torque of a hinged body implicit, losing ~(omega dt)^2 of its energy per step
        // (a 1 m rod pendulum lost a third of its energy per swing at 60 Hz).
        let mut joint_rows: Vec<JointRow> = Vec::new();
        let mut row_range = vec![(0usize, 0usize); self.joints.kinds().len()];
        for &j_idx in &active_joints {
            if let riftphys_articulation::bodies::JointKind::D6(j) = &self.joints.kinds()[j_idx] {
                let start = joint_rows.len();
                d6_rows(&self.bodies, j, dt, j_idx as u32, &self.rotor, &mut joint_rows);
                row_range[j_idx] = (start, joint_rows.len());
            }
        }
        let mut row_acc = vec![0.0f32; joint_rows.len()];
        let num_rows = joint_rows.len();
        let joint_rows_ref = joint_rows.as_slice();
        let row_range_ref = row_range.as_slice();

        for &j_idx in &active_joints {
            let (id_a, id_b) = match &self.joints.kinds()[j_idx] {
                riftphys_articulation::bodies::JointKind::Distance(j) => (j.a.0, j.b.0),
                riftphys_articulation::bodies::JointKind::D6(j) => (j.a.0, j.b.0),
            };
            let level_a = joint_body_level[id_a as usize];
            let level_b = joint_body_level[id_b as usize];
            let target = level_a.max(level_b);

            if target >= joint_batches.len() { joint_batches.push(Vec::new()); }
            joint_batches[target].push(j_idx);

            joint_body_level[id_a as usize] = target + 1;
            joint_body_level[id_b as usize] = target + 1;
        }
        self.last_max_batch = (
            contact_batches.iter().map(Vec::len).max().unwrap_or(0),
            joint_batches.iter().map(Vec::len).max().unwrap_or(0),
        );

        // 3. The Global Parallel Solve
        let mut acc_jn = vec![0.0f32; constraints.len()];
        // Accumulated friction impulse per contact, [t1, t2]
        let mut acc_jt = vec![0.0f32; constraints.len() * 2];
        let joints_ref = &self.joints;
        let num_constraints = constraints.len();
        let constraints_slice = constraints.as_slice();
        // Accumulated patch friction: [t1, t2, torsion] per patch
        let mut patch_acc = vec![0.0f32; patches.len() * 3];
        // Compliant contacts: their whole normal impulse, once, before the iterations
        for (c_idx, cs) in constraints.iter().enumerate() {
            if cs.soft && cs.soft_jn > 0.0 {
                acc_jn[c_idx] = cs.soft_jn;
                apply_contact_impulse(&mut self.bodies, cs, cs.normal * cs.soft_jn);
            }
        }

        // Warm start, joints: each D6 row starts from last step's impulse if the joint still has
        // the same rows, clamped to the row's bounds. Serial, in joint order. Starting cold let
        // gravity's load on a standing biped go unconverged every step, and the position solve
        // corrected the remainder by dragging its light feet along the ground.
        for (j_idx, &(s, e)) in row_range.iter().enumerate() {
            if e == s { continue; }
            let Some(prev) = self.warm_joints.get(&(j_idx as u32)) else { continue };
            if prev.len() != e - s || prev.iter().zip(&joint_rows[s..e]).any(|(p, r)| p.tag != r.tag) { continue; }
            for (k, p) in (s..e).zip(prev) {
                let r = &joint_rows[k];
                let lam = p.lam.clamp(r.lam_min, r.lam_max);
                row_acc[k] = lam;
                if lam != 0.0 { joint_row_apply(&mut self.bodies, r, lam); }
            }
        }

        // Warm start (AT_REST_SPEC R1): each rigid contact starts from last step's impulses
        // at the nearest matching anchor. Matched and applied serially, in contact order (float
        // sums depend on order), never through `run_batch`, so replay stays exact.
        let ord = collider_ordinals(&self.colliders, num_bodies);
        let keys: Vec<WarmKey> = contacts.iter().map(|c| {
            let a = &self.colliders[c.a_collider];
            if c.b_collider == TERRAIN_CI { return (a.body.0, u32::MAX, ord[c.a_collider], 0); }
            debug_assert!(c.a_collider < c.b_collider, "contact pair out of order: {} {}", c.a_collider, c.b_collider);
            (a.body.0, self.colliders[c.b_collider].body.0, ord[c.a_collider], ord[c.b_collider])
        }).collect();
        // Contact points in body A's frame, at the poses this solve runs at
        let anchors: Vec<Vec3> = contacts.iter().zip(&constraints).map(|(c, cs)| {
            let pa = self.bodies.pose(cs.ai);
            pa.rot.conjugate() * (c.point - pa.pos)
        }).collect();
        self.last_warm_matched = 0;
        {
            let mut used: BTreeMap<WarmKey, Vec<bool>> = BTreeMap::new();
            for (c_idx, cs) in constraints.iter().enumerate() {
                if cs.soft { continue; }
                let Some(entries) = self.warm.get(&keys[c_idx]) else { continue };
                let taken = used.entry(keys[c_idx]).or_insert_with(|| vec![false; entries.len()]);
                // Nearest unused entry within the tolerance; ties go to the lowest index
                let mut best: Option<(f32, usize)> = None;
                for (k, e) in entries.iter().enumerate() {
                    if taken[k] { continue; }
                    let d = (e.la - anchors[c_idx]).length_squared();
                    if d <= WARM_MATCH_TOL * WARM_MATCH_TOL && best.map_or(true, |(bd, _)| d < bd) { best = Some((d, k)); }
                }
                let Some((_, k)) = best else { continue };
                taken[k] = true;
                self.last_warm_matched += 1;
                let e = entries[k];
                acc_jn[c_idx] = e.jn;
                let mut imp = cs.normal * e.jn;
                if !cs.patch {
                    // Projected on the new tangents, clamped as the iterations clamp it
                    let mu = mu_dynamic(cs, slip_speed(&self.bodies, cs));
                    let t = Vec3::new(e.jt.dot(cs.t1), e.jt.dot(cs.t2), 0.0).clamp_length_max(mu * e.jn);
                    acc_jt[2 * c_idx] = t.x;
                    acc_jt[2 * c_idx + 1] = t.y;
                    imp += cs.t1 * t.x + cs.t2 * t.y;
                }
                apply_contact_impulse(&mut self.bodies, cs, imp);
            }
        }
        // Patches: their sliding and torsion accumulators, limited by the warm normal load
        // (compliant pairs are left cold, like their points)
        for (k, p) in patches.iter().enumerate() {
            if p.at.soft { continue; }
            let Some(wp) = self.warm_patches.get(&keys[p.points[0]]).copied() else { continue };
            let cs = &p.at;
            let jn: f32 = p.points.iter().map(|&i| acc_jn[i]).sum();
            let mu = mu_dynamic(cs, slip_speed(&self.bodies, cs));
            let t = Vec3::new(wp.jt.dot(cs.t1), wp.jt.dot(cs.t2), 0.0).clamp_length_max(mu * jn);
            let lim = mu * jn * p.radius;
            let tw = wp.tw.clamp(-lim, lim);
            patch_acc[3 * k] = t.x;
            patch_acc[3 * k + 1] = t.y;
            patch_acc[3 * k + 2] = tw;
            apply_contact_impulse(&mut self.bodies, cs, cs.t1 * t.x + cs.t2 * t.y);
            if tw != 0.0 {
                if cs.inv_a > 0.0 { self.bodies.apply_angular_impulse(cs.ai, -cs.normal * tw); }
                if !cs.b_is_terrain && cs.inv_b > 0.0 { self.bodies.apply_angular_impulse(cs.bi, cs.normal * tw); }
            }
        }
        let acc_jn_ptr = UnsafeAcc(acc_jn.as_mut_ptr());
        let acc_jt_ptr = UnsafeAcc(acc_jt.as_mut_ptr());
        let row_acc_ptr = UnsafeAcc(row_acc.as_mut_ptr());
        let bodies_ptr = UnsafeSyncBodies(&mut self.bodies as *mut _);

        for _it in 0..iterations {
            // A. Solve Joints (Batched Parallel)
            for batch in &joint_batches {
                run_batch(batch, |&j_idx| {
                    let bodies = unsafe { bodies_ptr.get_mut() };
                    if let riftphys_articulation::bodies::JointKind::Distance(j) = &joints_ref.kinds()[j_idx] {
                        // Compliant (spring) joints act at position level only, like compliant D6
                        // rows: a rigid velocity row here held them at their rest length.
                        if j.compliance > 0.0 { return; }
                        let pa = bodies.pose(j.a.0).pos;
                        let pb = bodies.pose(j.b.0).pos;
                        let dir = pb - pa;
                        let len = dir.length();
                        if len < 1e-6 { return; }
                        let n = dir / len;

                        let inv_a = bodies.inv_mass_of(j.a.0);
                        let inv_b = bodies.inv_mass_of(j.b.0);
                        let inv_sum = inv_a + inv_b;
                        if inv_sum < 1e-9 { return; }

                        let v_rel = (bodies.vel(j.b.0).lin - bodies.vel(j.a.0).lin).dot(n);
                        let bias = (len - j.rest) * (0.1 / dt);
                        let j_mag = -(v_rel + bias) / inv_sum;

                        bodies.apply_impulse(j.a.0, -n * j_mag);
                        bodies.apply_impulse(j.b.0,  n * j_mag);
                    } else {
                        let acc = unsafe { row_acc_ptr.get_mut_slice(num_rows) };
                        let (start, end) = row_range_ref[j_idx];
                        for k in start..end {
                            let r = &joint_rows_ref[k];
                            let v = joint_row_vel(bodies, r);
                            let old = acc[k];
                            // Rigid rows run exactly as before; soft rows (dampers) add gamma lam
                            let new = if r.gamma == 0.0 { old + (r.target - v) * r.m_eff } else { old + (r.target - v - r.gamma * old) * r.m_eff };
                            let new = new.clamp(r.lam_min, r.lam_max);
                            acc[k] = new;
                            if new != old { joint_row_apply(bodies, r, new - old); }
                        }
                    }
                });
            }

            // B. Solve Contacts (Batched Parallel)
            for batch in &contact_batches {
                run_batch(batch, move |&c_idx| {
                    let cs = &constraints_slice[c_idx];
                    let bodies = unsafe { bodies_ptr.get_mut() };
                    let acc_jn_slice = unsafe { acc_jn_ptr.get_mut_slice(num_constraints) };

                    let acc_jt_slice = unsafe { acc_jt_ptr.get_mut_slice(num_constraints * 2) };

                    // Normal: accumulated impulse clamped >= 0 (compliant: fixed, applied above)
                    let new_jn = if cs.soft { acc_jn_slice[c_idx] } else {
                        let v_rel_n = contact_rel_vel(bodies, cs).dot(cs.normal);
                        let jn_raw = (cs.depth_bias - v_rel_n) * cs.m_n;
                        let old_jn = acc_jn_slice[c_idx];
                        let new_jn = (old_jn + jn_raw).max(0.0);
                        acc_jn_slice[c_idx] = new_jn;
                        apply_contact_impulse(bodies, cs, cs.normal * (new_jn - old_jn));
                        new_jn
                    };
                    if cs.patch { return; }

                    // Friction: accumulated tangent impulse kept inside the Coulomb disc mu * jn
                    let v_f = contact_rel_vel(bodies, cs);
                    let (vt1, vt2) = (v_f.dot(cs.t1), v_f.dot(cs.t2));
                    let slip = (vt1 * vt1 + vt2 * vt2).sqrt();
                    let mu = mu_dynamic(cs, slip);
                    let old_t = Vec3::new(acc_jt_slice[2 * c_idx], acc_jt_slice[2 * c_idx + 1], 0.0);
                    let new_t = (old_t - Vec3::new(vt1 * cs.m_t1, vt2 * cs.m_t2, 0.0))
                        .clamp_length_max(mu * new_jn);
                    acc_jt_slice[2 * c_idx] = new_t.x;
                    acc_jt_slice[2 * c_idx + 1] = new_t.y;
                    let d = new_t - old_t;
                    apply_contact_impulse(bodies, cs, cs.t1 * d.x + cs.t2 * d.y);
                });
            }

            // C. Friction patches (sequential, in patch order): sliding at the centroid and
            // torsion about the normal, limited by the patch's current total normal impulse
            if !patches.is_empty() {
                let bodies = unsafe { bodies_ptr.get_mut() };
                let acc_jn_slice = unsafe { acc_jn_ptr.get_mut_slice(num_constraints) };
                for (k, p) in patches.iter().enumerate() {
                    let jn: f32 = p.points.iter().map(|&i| acc_jn_slice[i]).sum();
                    let cs = &p.at;
                    let v = contact_rel_vel(bodies, cs);
                    let (vt1, vt2) = (v.dot(cs.t1), v.dot(cs.t2));
                    let slip = (vt1 * vt1 + vt2 * vt2).sqrt();
                    let mu = mu_dynamic(cs, slip);
                    let old_t = Vec3::new(patch_acc[3 * k], patch_acc[3 * k + 1], 0.0);
                    let new_t = (old_t - Vec3::new(vt1 * cs.m_t1, vt2 * cs.m_t2, 0.0)).clamp_length_max(mu * jn);
                    patch_acc[3 * k] = new_t.x;
                    patch_acc[3 * k + 1] = new_t.y;
                    let d = new_t - old_t;
                    apply_contact_impulse(bodies, cs, cs.t1 * d.x + cs.t2 * d.y);
                    // Torsion: relative spin of B about the normal, against mu x load x radius
                    let wa = if cs.inv_a > 0.0 { bodies.vel(cs.ai).ang } else { Vec3::ZERO };
                    let wb = if cs.b_is_terrain || cs.inv_b <= 0.0 { Vec3::ZERO } else { bodies.vel(cs.bi).ang };
                    let w_rel = (wb - wa).dot(cs.normal);
                    let old_tw = patch_acc[3 * k + 2];
                    let lim = mu * jn * p.radius;
                    let new_tw = (old_tw - w_rel * p.m_tw).clamp(-lim, lim);
                    patch_acc[3 * k + 2] = new_tw;
                    let dt_w = new_tw - old_tw;
                    if dt_w != 0.0 {
                        if cs.inv_a > 0.0 { bodies.apply_angular_impulse(cs.ai, -cs.normal * dt_w); }
                        if !cs.b_is_terrain && cs.inv_b > 0.0 { bodies.apply_angular_impulse(cs.bi, cs.normal * dt_w); }
                    }
                }
            }
        }

        // Warm-start cache for next step: the impulses as the iterations left them. Stored before
        // the shock pass, whose frozen-body jn on a light body under a heavy one is far larger
        // than a solve with real masses gives, and before restitution, so a bounce isn't stored.
        let mut warm: BTreeMap<WarmKey, Vec<WarmContact>> = BTreeMap::new();
        for (c_idx, cs) in constraints.iter().enumerate() {
            if cs.soft { continue; }
            let jt = cs.t1 * acc_jt[2 * c_idx] + cs.t2 * acc_jt[2 * c_idx + 1];
            if acc_jn[c_idx] == 0.0 && jt == Vec3::ZERO { continue; }
            warm.entry(keys[c_idx]).or_default().push(WarmContact { la: anchors[c_idx], jn: acc_jn[c_idx], jt });
        }
        let mut warm_patches: BTreeMap<WarmKey, WarmPatch> = BTreeMap::new();
        for (k, p) in patches.iter().enumerate() {
            if p.at.soft { continue; }
            let jt = p.at.t1 * patch_acc[3 * k] + p.at.t2 * patch_acc[3 * k + 1];
            warm_patches.insert(keys[p.points[0]], WarmPatch { jt, tw: patch_acc[3 * k + 2] });
        }
        // Pairs where neither body is awake keep their entries even when not solved this step,
        // so a sleeping stack wakes warm. A removed body isn't idle: its entries go.
        let idle = |b: u32| b == u32::MAX || (self.alive.get(b as usize).copied().unwrap_or(0) != 0
            && (!self.bodies.is_dynamic(b) || !self.sleep.is_awake(b as usize)));
        for (k, v) in &self.warm {
            if idle(k.0) && idle(k.1) && !warm.contains_key(k) { warm.insert(*k, v.clone()); }
        }
        for (k, v) in &self.warm_patches {
            if idle(k.0) && idle(k.1) && !warm_patches.contains_key(k) { warm_patches.insert(*k, *v); }
        }
        self.warm = warm;
        self.warm_patches = warm_patches;
        // Joints solved this step replace their rows; the rest (asleep) keep theirs
        for (j_idx, &(s, e)) in row_range.iter().enumerate() {
            if e == s { continue; }
            let rows = (s..e).map(|k| WarmRow { tag: joint_rows[k].tag, lam: row_acc[k] }).collect();
            self.warm_joints.insert(j_idx as u32, rows);
            // Armature rotors keep their own balance, w_rotor - lam / J_a: equal to the links'
            // w_rel when the iterations converged, and untouched by the shock and restitution
            // passes below, which apply contact impulses with no joint rows (storing w_rel would
            // hand their change to the rotor with no impulse, making or losing energy at an
            // impact)
            for k in s..e {
                let r = &joint_rows[k];
                if r.tag & 7 == 6 {
                    self.rotor.insert((j_idx as u32, (r.tag >> 3) & 3), r.target - row_acc[k] * r.gamma);
                }
            }
        }

        // 3a. Shock propagation (Guendelman et al. 2003): one more normal pass, bottom-up along
        // gravity, with the lower body of each pair treated as immovable, so support reaches
        // the top of a stack in one sweep. Plain iterations can't push enough impulse through a
        // light body: a 1000 kg box dropped on a 1 kg box crushed straight through it.
        let up = if self.gravity.length_squared() > 0.0 { -self.gravity.normalize() } else { Vec3::Y };
        let height = |cs: &ContactConstraint| {
            let ha = self.bodies.pose(cs.ai).pos.dot(up);
            if cs.b_is_terrain { f32::NEG_INFINITY } else { ha.min(self.bodies.pose(cs.bi).pos.dot(up)) }
        };
        let mut order: Vec<usize> = (0..constraints.len()).filter(|&i| !constraints[i].soft && (constraints[i].inv_a > 0.0 || constraints[i].inv_b > 0.0)).collect();
        order.sort_by_key(|&x| (riftphys_core::det::sort_key6(height(&constraints[x])), x));
        for &c_idx in &order {
            let mut cs = constraints[c_idx];
            if cs.inv_a > 0.0 && cs.inv_b > 0.0 {
                // Freeze the lower body for this pass
                if self.bodies.pose(cs.ai).pos.dot(up) < self.bodies.pose(cs.bi).pos.dot(up) {
                    cs.inv_a = 0.0; cs.ii_a = Mat3::ZERO;
                } else {
                    cs.inv_b = 0.0; cs.ii_b = Mat3::ZERO;
                }
                let (ca, cb) = (cs.ra.cross(cs.normal), cs.rb.cross(cs.normal));
                let k = cs.inv_a + cs.inv_b + ca.dot(cs.ii_a.mul_vec3(ca)) + cb.dot(cs.ii_b.mul_vec3(cb));
                cs.m_n = if k > 1e-9 { 1.0 / k } else { 0.0 };
            }
            let v = contact_rel_vel(&self.bodies, &cs).dot(cs.normal);
            let old = acc_jn[c_idx];
            let new = (old + (cs.depth_bias - v) * cs.m_n).max(0.0);
            acc_jn[c_idx] = new;
            apply_contact_impulse(&mut self.bodies, &cs, cs.normal * (new - old));
        }

        // 3b. Restitution (as Box2D v3's relax pass): once the solve has settled which contacts
        // actually stop an approach (normal impulse > 0), bounce those at e x approach speed.
        // In the iterations, a speculative contact whose gap absorbs the approach must not bounce.
        for (c_idx, cs) in constraints.iter().enumerate() {
            if cs.restitution_bias <= 0.0 || acc_jn[c_idx] <= 0.0 { continue; }
            let v = contact_rel_vel(&self.bodies, cs).dot(cs.normal);
            let old = acc_jn[c_idx];
            let new = (old + (cs.restitution_bias - v) * cs.m_n).max(0.0);
            acc_jn[c_idx] = new;
            apply_contact_impulse(&mut self.bodies, cs, cs.normal * (new - old));
        }

        // 4. Write back the final normal impulses to the contact array
        for (i, contact) in contacts.iter_mut().enumerate() {
            contact.impulse = acc_jn[i];
        }
    }
}

/// Each collider's ordinal among its body's colliders, in collider order.
fn collider_ordinals(colliders: &[super::world::Collider], num_bodies: usize) -> Vec<u32> {
    let mut next = vec![0u32; num_bodies];
    colliders.iter().map(|c| {
        let b = c.body.0 as usize;
        if b >= next.len() { next.resize(b + 1, 0); }
        let o = next[b];
        next[b] += 1;
        o
    }).collect()
}