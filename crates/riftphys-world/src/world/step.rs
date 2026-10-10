// SPDX-License-Identifier: RiftForged Proprietary License
use rayon::prelude::*;
use super::world::{Contact, World, CCD_MAX_MARGIN, TERRAIN_CI};
use riftphys_core::{StepStage, StepStats, Scalar, Quat, Isometry, Vec3, BodyId};
use riftphys_geom::{aabb_of, Shape, Aabb};
use riftphys_collision::narrowphase::collide_shapes;
use riftphys_gravity::eval as grav_eval;
use riftphys_viz::viz::LedgerEvent;

// We use a safer, tighter margin for melee to prevent ghosting
// and false speculative contacts from ruining the narrowphase solver.
const MELEE_CCD_MARGIN: f32 = 0.05;

impl World {
    pub fn step(&mut self, dt: Scalar) -> StepStats {
        self.schedule.clear();
        self.tick = self.tick.wrapping_add(1);
        self.ledger.clear();
        self.reset_provenance();
        self.apply_pending_epoch_if_any();
        self.update_players(dt);

        let count = self.bodies.len() as u32;

        // --- STAGE 1: Force Integration ---
        for i in 0..count {
            if !self.bodies.is_dynamic(i) || !self.sleep.is_awake(i as usize) { continue; }
            let mut vel = self.bodies.vel(i);
            let pose = self.bodies.pose(i);
            let mut a_total = grav_eval(&self.gravity_proc, pose.pos);
            if let Some(Some(ac)) = self.accel_comps.get(i as usize) {
                let (a_grav, a_extra) = Self::eval_extra_accel(&self.models, &self.bodies, &self.accel_comps, self.epoch_id, self.tick, i, pose, vel, &self.gravity_proc, dt);
                a_total = a_grav + a_extra;
            }
            vel.lin += a_total * dt;
            self.bodies.set_vel(i, vel);
        }

        // --- STAGE 2: Broadphase & Narrowphase ---
        self.update_colliders_aabb();
        let pairs = self.broadphase.run(&self.colliders.iter().map(|c| c.aabb).collect::<Vec<_>>());
        let pairs_tested_count = pairs.len() as u32;

        let colliders = &self.colliders;
        let bodies = &self.bodies;
        let mut raw_contacts: Vec<Contact> = pairs.par_iter().flat_map(|&(i, j)| {
            let ca = &colliders[i];
            let cb = &colliders[j];
            let mut local = Vec::new();
            if let Some(manifold) = collide_shapes(&ca.shape, &bodies.pose(ca.body.0), &cb.shape, &bodies.pose(cb.body.0), MELEE_CCD_MARGIN) {
                for pt_idx in 0..manifold.len {
                    let pt = &manifold.points[pt_idx as usize];
                    local.push(Contact { a_collider: i, b_collider: j, point: pt.position, normal: pt.normal, depth: pt.penetration, env_material: None, impulse: 0.0 });
                }
            }
            local
        }).collect();

        let env_contacts: Vec<Contact> = colliders.par_iter().enumerate().flat_map(|(ci, c)| {
            let mut local = Vec::new();
            let mut env_buffer = Vec::new();
            for env in &self.environments { env.query_aabb(&c.aabb, &mut env_buffer); }
            for env_col in env_buffer {
                if let Some(manifold) = collide_shapes(&c.shape, &bodies.pose(c.body.0), &env_col.shape, &env_col.transform, MELEE_CCD_MARGIN) {
                    for pt_idx in 0..manifold.len {
                        let pt = &manifold.points[pt_idx as usize];
                        local.push(Contact { a_collider: ci, b_collider: TERRAIN_CI, point: pt.position, normal: pt.normal, depth: pt.penetration, env_material: Some(env_col.material), impulse: 0.0 });
                    }
                }
            }
            local
        }).collect();
        raw_contacts.extend(env_contacts);

        // --- STAGE 3: Solve Contacts (Velocity Level) ---
        let mut contacts = self.cull_and_orient_contacts(raw_contacts);
        let islands = self.island_manager.build_islands(self.bodies.len(), &self.colliders, &self.bodies, &self.joints, &contacts);

        if !islands.is_empty() {
            self.solve_islands_parallel(&islands, &mut contacts, dt);
        }

        // --- STAGE 3.5: Motor Injection ---
        for kind in self.joints.kinds().iter() {
            if let riftphys_articulation::bodies::JointKind::D6(j) = kind {
                for axis in 0..3 {
                    if let Some(drive) = j.r[axis].drive {
                        if drive.vel_mode {
                            let axis_vec = match axis {
                                0 => Vec3::new(1.0, 0.0, 0.0),
                                1 => Vec3::new(0.0, 1.0, 0.0),
                                _ => Vec3::new(0.0, 0.0, 1.0),
                            };
                            let vel_a = self.bodies.vel(j.a.0).ang;
                            let vel_b = self.bodies.vel(j.b.0).ang;
                            let v_rel = (vel_b - vel_a).dot(axis_vec);
                            let err = drive.target - v_rel;

                            // Motor directly drives velocity. Removed `dt` multiplication to restore its bite.
                            let correction = (err * drive.kd).clamp(-drive.max_impulse, drive.max_impulse);

                            if self.bodies.is_dynamic(j.a.0) {
                                let mut va = self.bodies.vel(j.a.0);
                                va.ang -= axis_vec * correction;
                                self.bodies.set_vel(j.a.0, va);
                            }
                            if self.bodies.is_dynamic(j.b.0) {
                                let mut vb = self.bodies.vel(j.b.0);
                                vb.ang += axis_vec * correction;
                                self.bodies.set_vel(j.b.0, vb);
                            }
                        }
                    }
                }
            }
        }

        // --- STAGE 4: Position Integration ---
        let mut original_poses = vec![Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY }; count as usize];
        for i in 0..count {
            if !self.bodies.is_dynamic(i) || !self.sleep.is_awake(i as usize) { continue; }
            let mut pose = self.bodies.pose(i);
            original_poses[i as usize] = pose;

            let vel = self.bodies.vel(i);
            pose.pos += vel.lin * dt;
            if vel.ang.length_squared() > 1e-9 {
                let axis = vel.ang.normalize();
                let angle = vel.ang.length() * dt;
                pose.rot = (Quat::from_axis_angle(axis, angle) * pose.rot).normalize();
            }
            self.bodies.set_pose(i, pose);
        }

        // --- STAGE 5: Solve Joints (Position Level) ---
        if !self.joints.kinds().is_empty() {
            self.solve_joints(dt, 20);
        }

        // --- STAGE 6: Velocity Derivation (THE FINAL FIX) ---
        for i in 0..count {
            if !self.bodies.is_dynamic(i) || !self.sleep.is_awake(i as usize) { continue; }
            let mut vel = self.bodies.vel(i);
            let old_pose = original_poses[i as usize];
            let mut new_pose = self.bodies.pose(i);

            vel.lin = (new_pose.pos - old_pose.pos) / dt;

            // FIX 1: MASS RATIO CRUSH (Floor grounding guarantee)
            let mut touching_ground = false;
            for c in &contacts {
                if c.b_collider == TERRAIN_CI && self.colliders[c.a_collider].body.0 == i {
                    touching_ground = true; break;
                }
            }

            // If the heavy stack pushed us down into the floor, kill the derived downward velocity
            // and snap position back to prevent sinking.
            if touching_ground && vel.lin.y < 0.0 {
                vel.lin.y = 0.0;
                new_pose.pos.y = old_pose.pos.y;
                self.bodies.set_pose(i, new_pose);
            }

            // FIX 2: MOTOR OVERWRITE
            let mut is_driven = false;
            for kind in self.joints.kinds().iter() {
                if let riftphys_articulation::bodies::JointKind::D6(j) = kind {
                    if j.a.0 == i || j.b.0 == i {
                        for axis in 0..3 {
                            if let Some(drive) = j.r[axis].drive {
                                if drive.vel_mode { is_driven = true; break; }
                            }
                        }
                    }
                }
            }

            // Only derive angular velocity if a motor isn't actively setting it
            if !is_driven {
                let dq = new_pose.rot * old_pose.rot.conjugate();
                let mut ang_vel = Vec3::new(dq.x, dq.y, dq.z) * (2.0 / dt);
                if dq.w < 0.0 { ang_vel = -ang_vel; }
                vel.ang = ang_vel;
            }

            self.bodies.set_vel(i, vel);
        }

        // --- STAGE 7: Sleep & Telemetry ---
        self.sleep.evaluate(count, &self.bodies);
        self.update_colliders_aabb();
        self.last_frame_contacts = contacts;

        if self.debug.dump_state_every > 0 && self.tick % self.debug.dump_state_every as u64 == 0 {
            let mut ke_lin_total = 0.0;
            let mut ke_ang_total = 0.0;
            let mut pe_grav_total = 0.0;
            let g_mag = self.gravity.length();

            for i in 0..count {
                if !self.body_alive(BodyId(i)) { continue; }
                let pose = self.bodies.pose(i);
                let vel = self.bodies.vel(i);

                self.record(LedgerEvent::BodyState { id: i, pos: pose.pos, rot: pose.rot, v_lin: vel.lin, v_ang: vel.ang });

                if self.bodies.is_dynamic(i) {
                    let inv_mass = self.bodies.inv_mass_of(i);
                    if inv_mass > 0.0 {
                        let mass = 1.0 / inv_mass;
                        ke_lin_total += 0.5 * mass * vel.lin.length_squared();

                        let inv_i = self.bodies.inv_inertia_world(i);
                        if inv_i.x_axis.x > 0.0 && inv_i.y_axis.y > 0.0 && inv_i.z_axis.z > 0.0 {
                            ke_ang_total += 0.5 * ((vel.ang.x * vel.ang.x / inv_i.x_axis.x) + (vel.ang.y * vel.ang.y / inv_i.y_axis.y) + (vel.ang.z * vel.ang.z / inv_i.z_axis.z));
                        }
                        pe_grav_total += mass * g_mag * pose.pos.y;
                    }
                }
            }
            self.record(LedgerEvent::SystemEnergy { ke_lin: ke_lin_total, ke_ang: ke_ang_total, pe_grav: pe_grav_total, total: ke_lin_total + ke_ang_total + pe_grav_total });
        }

        StepStats { pairs_tested: pairs_tested_count, contacts: self.last_frame_contacts.len() as u32, islands: islands.len() as u32 }
    }

    fn cull_and_orient_contacts(&self, mut contacts: Vec<Contact>) -> Vec<Contact> {
        let mut buckets: std::collections::BTreeMap<(u32,u32), Vec<Contact>> = std::collections::BTreeMap::new();
        for c in contacts.drain(..) {
            let key = if c.a_collider <= c.b_collider { (c.a_collider as u32, c.b_collider as u32) }
            else { (c.b_collider as u32, c.a_collider as u32) };
            buckets.entry(key).or_default().push(c);
        }

        let mut culled = Vec::new();
        for (_, mut v) in buckets {
            v.sort_by(|a, b| b.depth.partial_cmp(&a.depth).unwrap());
            v.truncate(4);
            culled.extend(v);
        }

        for c in &mut culled {
            if c.b_collider == TERRAIN_CI { continue; }
            let pa = self.bodies.pose(self.colliders[c.a_collider].body.0).pos;
            let pb = self.bodies.pose(self.colliders[c.b_collider].body.0).pos;
            if c.normal.dot(pb - pa) < 0.0 { c.normal = -c.normal; }
        }
        culled
    }

    pub fn update_players(&mut self, dt: f32) {
        for p_idx in 0..self.players.len() {
            let mut player = self.players[p_idx].clone();
            let mut pos = self.bodies.pose(player.body.0).pos;

            // 1. Gravity & Sticky Floor Logic
            if !player.grounded {
                player.vertical_vel -= 9.81 * dt; // Fall
            } else {
                // Apply a continuous sticky downward velocity to hug slopes
                if player.vertical_vel <= 0.0 { player.vertical_vel = -0.5; }
            }

            // 2. Decode the Input Vector from the FFI
            let input_mag_sq = player.input_dir.length_squared();
            let move_dir = if input_mag_sq > 1e-6 {
                player.input_dir / input_mag_sq.sqrt()
            } else {
                Vec3::ZERO
            };

            let mut desired_move = (move_dir * player.speed * dt)
                + Vec3::new(0.0, player.vertical_vel * dt, 0.0);

            player.grounded = false; // Assume falling until we verify floor hit

            // 3. Iterative Sweep/Slide
            for _ in 0..4 {
                if desired_move.length_squared() < 1e-6 { break; }

                let player_xf = Isometry { pos: pos + desired_move, rot: Quat::IDENTITY };
                let player_shape = Shape::Capsule { r: player.radius, hh: player.height * 0.5 };
                let player_aabb = aabb_of(&player_shape, &player_xf);

                let mut hit_normal = None;
                let mut hit_penetration = 0.0;
                let mut hit_body = None;

                // A. Check Terrain with 5cm Slop Margin
                let mut env_buffer = Vec::new();
                for env in &self.environments { env.query_aabb(&player_aabb, &mut env_buffer); }
                for env_col in env_buffer {
                    // 0.05 margin ensures continuous floor detection
                    if let Some(m) = collide_shapes(&player_shape, &player_xf, &env_col.shape, &env_col.transform, 0.05) {
                        hit_normal = Some(m.points[0].normal);
                        hit_penetration = m.points[0].penetration;
                        break;
                    }
                }

                // B. Check Spheres with 5cm Slop Margin
                if hit_normal.is_none() {
                    // Addressed warning: prefixed `ci` with an underscore
                    for (_ci, c) in self.colliders.iter().enumerate() {
                        if c.body != player.body && player_aabb.intersects(&c.aabb) {
                            if let Some(m) = collide_shapes(&player_shape, &player_xf, &c.shape, &self.bodies.pose(c.body.0), 0.05) {
                                hit_normal = Some(m.points[0].normal);
                                hit_penetration = m.points[0].penetration;
                                hit_body = Some(c.body);
                                break;
                            }
                        }
                    }
                }

                // C. Geometrically Stable Resolution
                if let Some(normal) = hit_normal {
                    if normal.y > 0.6 { player.grounded = true; player.vertical_vel = 0.0; }

                    // Apply the penetration pushout directly to the target movement vector
                    desired_move += normal * (hit_penetration + 0.001);

                    // Kill only the velocity going directly into the wall/floor
                    let dot = desired_move.dot(normal);
                    if dot < 0.0 {
                        desired_move -= normal * dot;
                    }

                    // Kick dynamic spheres
                    if let Some(b_id) = hit_body {
                        if self.bodies.is_dynamic (b_id.0) {
                            self.sleep.wake(b_id.0 as usize);
                            self.bodies.apply_impulse(b_id.0, -normal * 10.0);
                        }
                    }
                } else {
                    break; // Path is clear, break early
                }
            }

            // Finally, apply the guaranteed-safe movement vector
            pos += desired_move;
            self.bodies.set_pose(player.body.0, Isometry { pos, rot: Quat::IDENTITY });
            self.players[p_idx] = player;
        }
    }
}