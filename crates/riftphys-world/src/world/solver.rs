// SPDX-License-Identifier: RiftForged Proprietary License
use rayon::prelude::*;
use super::geom::orthonormal_basis;
use riftphys_core::Vec3;
use super::world::{Contact, World, TERRAIN_CI};
use super::islands::Island;
use riftphys_materials::materials as mats;

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

struct ContactConstraint {
    ai: u32, bi: u32,
    b_is_terrain: bool,
    inv_a: f32,
    inv_b: f32,
    inv_mass_sum: f32,
    normal: Vec3, t1: Vec3, t2: Vec3,
    mu_s: f32,
    depth_bias: f32,
    restitution_bias: f32,
}

impl World {
    pub fn solve_islands_parallel(&mut self, islands: &[Island], contacts: &mut [Contact], dt: f32) {
        let iterations = 20; // Increased for energy conservation
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
            let inv_sum = inv_a + inv_b;

            let pair = mats::pair_props(self.colliders[c.a_collider].material.id,
                                        if b_is_terrain { c.env_material.unwrap_or(mats::MaterialId::Default) }
                                        else { self.colliders[c.b_collider].material.id });

            let mut normal = c.normal;
            if b_is_terrain {
                normal = -normal;
            }

            let (t1, t2) = orthonormal_basis(normal);
            let v_rel = if b_is_terrain { -self.bodies.vel(ai).lin }
            else { self.bodies.vel(bi).lin - self.bodies.vel(ai).lin };

            let v_rel_n = v_rel.dot(normal);
            let restitution_bias = if v_rel_n < -1.0 { -pair.restitution * v_rel_n } else { 0.0 };

            ContactConstraint {
                ai, bi, b_is_terrain,
                inv_a, inv_b,
                inv_mass_sum: if inv_sum > 0.0 { 1.0 / inv_sum } else { 0.0 },
                normal, t1, t2, mu_s: pair.mu_s,
                depth_bias: ((c.depth - slop).max(0.0) * (beta / dt)).min(2.0),
                restitution_bias,
            }
        }).collect();

        // 2a. Graph Coloring (Level Scheduling) for Contacts
        let mut contact_batches: Vec<Vec<usize>> = Vec::with_capacity(16);
        let mut body_level = vec![0; num_bodies];

        for c_idx in 0..constraints.len() {
            let cs = &constraints[c_idx];
            let level_a = body_level[cs.ai as usize];
            let level_b = if cs.b_is_terrain { 0 } else { body_level[cs.bi as usize] };

            let target_level = level_a.max(level_b);
            if target_level >= contact_batches.len() {
                contact_batches.push(Vec::new());
            }
            contact_batches[target_level].push(c_idx);

            body_level[cs.ai as usize] = target_level + 1;
            if !cs.b_is_terrain { body_level[cs.bi as usize] = target_level + 1; }
        }

        // 2b. Graph Coloring for Joints
        let mut joint_batches: Vec<Vec<usize>> = Vec::with_capacity(8);
        let mut joint_body_level = vec![0; num_bodies];
        let mut active_joints = Vec::new();
        for island in islands { active_joints.extend(&island.joint_indices); }

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
        let bodies_ptr = UnsafeSyncBodies(&mut self.bodies as *mut _);
        let joints_ref = &self.joints;
        let num_constraints = constraints.len();
        let constraints_slice = constraints.as_slice();

        for _it in 0..iterations {
            // A. Solve Joints (Batched Parallel)
            for batch in &joint_batches {
                batch.par_iter().for_each(|&j_idx| {
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
                    }
                });
            }

            // B. Solve Contacts (Batched Parallel)
            for batch in &contact_batches {
                batch.par_iter().for_each(move |&c_idx| {
                    let cs = &constraints_slice[c_idx];
                    let bodies = unsafe { bodies_ptr.get_mut() };
                    let acc_jn_slice = unsafe { acc_jn_ptr.get_mut_slice(num_constraints) };

                    let va = bodies.vel(cs.ai).lin;
                    let vb = if cs.b_is_terrain { Vec3::ZERO } else { bodies.vel(cs.bi).lin };
                    let v_rel_n = (vb - va).dot(cs.normal);

                    let jn_raw = (cs.restitution_bias + cs.depth_bias - v_rel_n) * cs.inv_mass_sum;
                    let old_jn = acc_jn_slice[c_idx];
                    let new_jn = (old_jn + jn_raw).max(0.0);
                    acc_jn_slice[c_idx] = new_jn;
                    let jn = new_jn - old_jn;

                    // Mass-Weighted Impulse Distribution (The Fix for Jitter)
                    let total_inv_mass = cs.inv_a + cs.inv_b;
                    let weight_a = if total_inv_mass > 0.0 { cs.inv_a / total_inv_mass } else { 0.5 };
                    let weight_b = if total_inv_mass > 0.0 { cs.inv_b / total_inv_mass } else { 0.5 };

                    let impulse = cs.normal * jn;
                    if cs.inv_a > 0.0 { bodies.apply_impulse(cs.ai, -impulse * (weight_a * 2.0)); }
                    if cs.inv_b > 0.0 { bodies.apply_impulse(cs.bi, impulse * (weight_b * 2.0)); }

                    let v_f = if cs.b_is_terrain { -bodies.vel(cs.ai).lin }
                    else { bodies.vel(cs.bi).lin - bodies.vel(cs.ai).lin };

                    let tangent_v = Vec3::new(v_f.dot(cs.t1), v_f.dot(cs.t2), 0.0);
                    let max_fric = cs.mu_s * acc_jn_slice[c_idx];

                    if tangent_v.length() > 1e-6 {
                        let j_fric = -(tangent_v.truncate() * cs.inv_mass_sum);
                        let impulse_f = (cs.t1 * j_fric.x + cs.t2 * j_fric.y).clamp_length_max(max_fric);

                        if cs.inv_a > 0.0 { bodies.apply_impulse(cs.ai, -impulse_f); }
                        if cs.inv_b > 0.0 { bodies.apply_impulse(cs.bi, impulse_f); }
                    }
                });
            }
        }

        // 4. Write back the final normal impulses to the contact array
        for (i, contact) in contacts.iter_mut().enumerate() {
            contact.impulse = acc_jn[i];
        }
    }
}