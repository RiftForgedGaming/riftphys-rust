// SPDX-License-Identifier: RiftForged Proprietary License
use rayon::prelude::*;
use super::geom::orthonormal_basis;
use riftphys_core::{Mat3, Vec3};
use super::world::{Contact, World, TERRAIN_CI};
use super::islands::Island;
use riftphys_materials::materials as mats;

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
}

/// One velocity-level row of a D6 joint. Linear rows constrain the anchors' relative velocity
/// along `u` (lever arms `ra`, `rb`); angular rows the relative angular velocity along `u`.
/// The row keeps the relative velocity inside [vmin, vmax] with its accumulated impulse in
/// [lam_min, lam_max]: equality rows (locked axes) are [0, 0] / unbounded, a lower limit is
/// [speculative bound, inf) / [0, inf), an upper limit the mirror image.
#[derive(Clone, Copy)]
struct JointRow {
    a: u32, b: u32,
    angular: bool,
    u: Vec3, ra: Vec3, rb: Vec3,
    /// Inverse masses and world inverse inertias (zero for static bodies)
    wa: f32, wb: f32, ia: Mat3, ib: Mat3,
    m_eff: f32,
    target: f32,
    lam_min: f32, lam_max: f32,
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
fn d6_rows(bodies: &riftphys_dynamics::dynamics::Bodies, j: &riftphys_articulation::bodies::D6Joint, dt: f32, out: &mut Vec<JointRow>) {
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
            if !row.enabled || row.compliance > 0.0 { continue; }
            // Position drives stay at position level; velocity drives become motor rows below
            if row.drive.map_or(false, |d| !d.vel_mode) { continue; }
            let u = axes[i];
            let k = if angular {
                u.dot(ia.mul_vec3(u)) + u.dot(ib.mul_vec3(u))
            } else {
                let (ca, cb) = (ra.cross(u), rb.cross(u));
                wa + wb + ca.dot(ia.mul_vec3(ca)) + cb.dot(ib.mul_vec3(cb))
            };
            if k <= 1e-9 { continue; }
            let base = JointRow { a, b, angular, u, ra, rb, wa, wb, ia, ib, m_eff: 1.0 / k, target: 0.0, lam_min: f32::NEG_INFINITY, lam_max: f32::INFINITY };
            if let Some(drive) = row.drive {
                // Motor: drive the relative velocity to the target with at most max_impulse per
                // step (a constraint motor; adding kd x error straight to the velocity diverged
                // for kd > 2). Limits still apply through their own rows.
                let cap = drive.max_impulse.abs();
                out.push(JointRow { target: drive.target, lam_min: -cap, lam_max: cap, ..base });
            }
            match row.limit {
                None if row.drive.is_some() => {}
                None => out.push(base),
                Some(lim) => {
                    let x = if angular { e_world.dot(u) } else { d.dot(u) };
                    // Approach the stop by at most what is left this step; never push out here
                    // (the position solve removes any violation)
                    out.push(JointRow { target: ((lim.min - x) / dt).min(0.0), lam_min: 0.0, ..base });
                    out.push(JointRow { target: ((lim.max - x) / dt).max(0.0), lam_max: 0.0, lam_min: f32::NEG_INFINITY, ..base });
                }
            }
        }
    }
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

            let pair = self.pair_props(self.colliders[c.a_collider].material.id,
                                        if b_is_terrain { c.env_material.unwrap_or(mats::MaterialId::Default) }
                                        else { self.colliders[c.b_collider].material.id });

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
            };
            let pen = c.depth - margin;
            cs.depth_bias = if pen > 0.0 { ((pen - slop).max(0.0) * (beta / dt)).min(2.0) } else { pen / dt };
            // Approach speed from the start of the step, before this step's forces
            let pre = |id: u32| self.pre_step_vel.get(id as usize).copied().unwrap_or_else(|| self.bodies.vel(id));
            let (va, vb) = (pre(ai), if b_is_terrain { Default::default() } else { pre(bi) });
            let v_pre = ((vb.lin + vb.ang.cross(rb)) - (va.lin + va.ang.cross(ra))).dot(normal);
            // Approach speed at mid-step (between the start-of-step and after-gravity velocities)
            let v_rel_n = 0.5 * (v_pre + contact_rel_vel(&self.bodies, &cs).dot(normal));
            // Applied after the solve, only to contacts that carried load (see below)
            if v_rel_n < -1.0 { cs.restitution_bias = -pair.restitution * v_rel_n; }
            cs
        }).collect();

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
                d6_rows(&self.bodies, j, dt, &mut joint_rows);
                row_range[j_idx] = (start, joint_rows.len());
            }
        }
        let mut row_acc = vec![0.0f32; joint_rows.len()];
        let row_acc_ptr = UnsafeAcc(row_acc.as_mut_ptr());
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

        // 3. The Global Parallel Solve
        let mut acc_jn = vec![0.0f32; constraints.len()];
        let acc_jn_ptr = UnsafeAcc(acc_jn.as_mut_ptr());
        // Accumulated friction impulse per contact, [t1, t2]
        let mut acc_jt = vec![0.0f32; constraints.len() * 2];
        let acc_jt_ptr = UnsafeAcc(acc_jt.as_mut_ptr());
        let bodies_ptr = UnsafeSyncBodies(&mut self.bodies as *mut _);
        let joints_ref = &self.joints;
        let num_constraints = constraints.len();
        let constraints_slice = constraints.as_slice();

        for _it in 0..iterations {
            // A. Solve Joints (Batched Parallel)
            for batch in &joint_batches {
                run_batch(batch, |&j_idx| {
                    let bodies = unsafe { bodies_ptr.get_mut() };
                    if let riftphys_articulation::bodies::JointKind::Distance(j) = &joints_ref.kinds()[j_idx] {
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
                            let new = (old + (r.target - v) * r.m_eff).clamp(r.lam_min, r.lam_max);
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

                    // Normal: accumulated impulse clamped >= 0
                    let v_rel_n = contact_rel_vel(bodies, cs).dot(cs.normal);
                    let jn_raw = (cs.depth_bias - v_rel_n) * cs.m_n;
                    let old_jn = acc_jn_slice[c_idx];
                    let new_jn = (old_jn + jn_raw).max(0.0);
                    acc_jn_slice[c_idx] = new_jn;
                    apply_contact_impulse(bodies, cs, cs.normal * (new_jn - old_jn));

                    // Friction: accumulated tangent impulse kept inside the Coulomb disc mu * jn
                    let v_f = contact_rel_vel(bodies, cs);
                    let (vt1, vt2) = (v_f.dot(cs.t1), v_f.dot(cs.t2));
                    let slip = (vt1 * vt1 + vt2 * vt2).sqrt();
                    let mu = cs.mu_k + (cs.mu_s - cs.mu_k) * (-(slip / cs.stribeck_v).powi(2)).exp();
                    let old_t = Vec3::new(acc_jt_slice[2 * c_idx], acc_jt_slice[2 * c_idx + 1], 0.0);
                    let new_t = (old_t - Vec3::new(vt1 * cs.m_t1, vt2 * cs.m_t2, 0.0))
                        .clamp_length_max(mu * new_jn);
                    acc_jt_slice[2 * c_idx] = new_t.x;
                    acc_jt_slice[2 * c_idx + 1] = new_t.y;
                    let d = new_t - old_t;
                    apply_contact_impulse(bodies, cs, cs.t1 * d.x + cs.t2 * d.y);
                });
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
        let mut order: Vec<usize> = (0..constraints.len()).filter(|&i| constraints[i].inv_a > 0.0 || constraints[i].inv_b > 0.0).collect();
        order.sort_by(|&x, &y| height(&constraints[x]).partial_cmp(&height(&constraints[y])).unwrap_or(std::cmp::Ordering::Equal).then(x.cmp(&y)));
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