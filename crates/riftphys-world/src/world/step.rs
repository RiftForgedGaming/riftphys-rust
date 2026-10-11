// SPDX-License-Identifier: RiftForged Proprietary License
use rayon::prelude::*;
use super::world::{Collider, Contact, World, CCD_MAX_MARGIN, TERRAIN_CI};
use riftphys_core::{StepStage, StepStats, Scalar, Quat, Isometry, Vec3, BodyId};
use riftphys_geom::{aabb_of, Shape, Aabb};
use riftphys_collision::narrowphase::collide_shapes;
use riftphys_gravity::eval as grav_eval;
use riftphys_viz::viz::LedgerEvent;

// We use a safer, tighter margin for melee to prevent ghosting
// and false speculative contacts from ruining the narrowphase solver.
const MELEE_CCD_MARGIN: f32 = 0.05;
/// Below this many pairs/colliders, contact generation runs on the calling thread.
const PAR_MIN: usize = 64;
/// Penetration left alone by the position-level contact projection (matches the velocity solver's slop).
const CONTACT_POS_SLOP: f32 = 0.005;

/// A contact tracked through the position-level solve: one anchor on each body (body-local), the
/// normal (A -> B) and the true penetration at the start of the step. Penetration after bodies
/// move is the start value minus how far the anchors separated along the normal.
struct PosContact {
    a: u32,
    b: Option<u32>,
    la: Vec3,
    lb: Vec3,
    /// World anchor on B when B is terrain (static)
    pb_world: Vec3,
    n: Vec3,
    pen0: f32,
}

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
        self.pre_step_vel.clear();
        self.pre_step_vel.extend((0..count).map(|i| self.bodies.vel(i)));
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
        // Swept, margin-inflated boxes: a body moving |v| dt this step must find its contacts
        // before it arrives (a ball 7 mm above the floor at 4.4 m/s otherwise sank 7 cm in one
        // 60 Hz step before any contact existed).
        let sweep: Vec<f32> = self.colliders.iter().map(|c| {
            let v = self.bodies.vel(c.body.0);
            let reach = 0.5 * (c.aabb.max - c.aabb.min).length();
            MELEE_CCD_MARGIN + (v.lin.length() + v.ang.length() * reach) * dt
        }).collect();
        let swept: Vec<Aabb> = self.colliders.iter().zip(&sweep)
            .map(|(c, s)| Aabb { min: c.aabb.min - Vec3::splat(*s), max: c.aabb.max + Vec3::splat(*s) }).collect();
        let pairs = self.broadphase.run(&swept);
        let pairs_tested_count = pairs.len() as u32;

        // Bodies joined directly by a joint do not collide with each other (as in PhysX/Bullet):
        // limb segments overlap at their joints, and contacts there would fight the joint.
        let jointed: std::collections::HashSet<(u32, u32)> = self.joints.kinds().iter().map(|k| {
            let (a, b) = match k {
                riftphys_articulation::bodies::JointKind::Distance(j) => (j.a.0, j.b.0),
                riftphys_articulation::bodies::JointKind::D6(j) => (j.a.0, j.b.0),
            };
            (a.min(b), a.max(b))
        }).collect();

        let colliders = &self.colliders;
        let bodies = &self.bodies;
        let narrow = |&(i, j): &(usize, usize)| -> Vec<Contact> {
            let ca = &colliders[i];
            let cb = &colliders[j];
            let mut local = Vec::new();
            let (ba, bb) = (ca.body.0, cb.body.0);
            if ba == bb || jointed.contains(&(ba.min(bb), ba.max(bb))) { return local; }
            // Contact distance covers both bodies' motion this step; depths are reported with the
            // fixed margin only, so the solver reads true penetration as depth - MELEE_CCD_MARGIN
            let reach = sweep[i] + sweep[j] - MELEE_CCD_MARGIN;
            if let Some(manifold) = collide_shapes(&ca.shape, &bodies.pose(ca.body.0), &cb.shape, &bodies.pose(cb.body.0), reach) {
                for pt_idx in 0..manifold.len {
                    let pt = &manifold.points[pt_idx as usize];
                    local.push(Contact { a_collider: i, b_collider: j, point: pt.position, normal: pt.normal, depth: pt.penetration - reach + MELEE_CCD_MARGIN, env_material: None, impulse: 0.0 });
                }
            }
            local
        };
        // Small scenes run on this thread (rayon overhead dominates); output order is the same
        let mut raw_contacts: Vec<Contact> = if pairs.len() >= PAR_MIN {
            pairs.par_iter().flat_map(narrow).collect()
        } else {
            pairs.iter().flat_map(narrow).collect()
        };

        let env_one = |(ci, c): (usize, &Collider)| -> Vec<Contact> {
            let mut local = Vec::new();
            let mut env_buffer = Vec::new();
            for env in &self.environments { env.query_aabb(&c.aabb, &mut env_buffer); }
            for env_col in env_buffer {
                if let Some(manifold) = collide_shapes(&c.shape, &bodies.pose(c.body.0), &env_col.shape, &env_col.transform, sweep[ci]) {
                    for pt_idx in 0..manifold.len {
                        let pt = &manifold.points[pt_idx as usize];
                        local.push(Contact { a_collider: ci, b_collider: TERRAIN_CI, point: pt.position, normal: pt.normal, depth: pt.penetration - sweep[ci] + MELEE_CCD_MARGIN, env_material: Some(env_col.material), impulse: 0.0 });
                    }
                }
            }
            local
        };
        let env_contacts: Vec<Contact> = if self.environments.is_empty() {
            Vec::new()
        } else if colliders.len() >= PAR_MIN {
            colliders.par_iter().enumerate().flat_map(env_one).collect()
        } else {
            colliders.iter().enumerate().flat_map(env_one).collect()
        };
        raw_contacts.extend(env_contacts);

        // --- STAGE 3: Solve Contacts (Velocity Level) ---
        let mut contacts = self.cull_and_orient_contacts(raw_contacts);
        let islands = self.island_manager.build_islands(self.bodies.len(), &self.colliders, &self.bodies, &self.joints, &contacts);

        if !islands.is_empty() {
            self.solve_islands_parallel(&islands, &mut contacts, dt, MELEE_CCD_MARGIN);
        }

        // (Velocity motors are constraint rows in the velocity solve; see `d6_rows`.)

        // --- STAGE 4: Position Integration ---
        let mut original_poses = vec![Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY }; count as usize];
        let mut predicted_poses = original_poses.clone();
        // Bodies touched by a contact or joint this step; the rest fly ballistically
        let mut constrained = vec![false; count as usize];
        let mut jointed_body = vec![false; count as usize];
        for c in &contacts {
            constrained[self.colliders[c.a_collider].body.0 as usize] = true;
            if c.b_collider != TERRAIN_CI { constrained[self.colliders[c.b_collider].body.0 as usize] = true; }
        }
        for k in self.joints.kinds().iter() {
            let (a, b) = match k {
                riftphys_articulation::bodies::JointKind::Distance(j) => (j.a.0, j.b.0),
                riftphys_articulation::bodies::JointKind::D6(j) => (j.a.0, j.b.0),
            };
            constrained[a as usize] = true;
            constrained[b as usize] = true;
            jointed_body[a as usize] = true;
            jointed_body[b as usize] = true;
        }
        for i in 0..count {
            if !self.bodies.is_dynamic(i) || !self.sleep.is_awake(i as usize) { continue; }
            let mut pose = self.bodies.pose(i);
            original_poses[i as usize] = pose;

            let mut vel = self.bodies.vel(i);
            pose.pos += vel.lin * dt;
            // Ballistic bodies: move with the average of the last step's end velocity and this
            // step's (exact for constant forces, gravity and thrust alike; semi-implicit Euler
            // overshoots a fall by a dt t / 2, 1.7% in 1 s at 60 Hz). Constrained bodies keep
            // the semi-implicit update the contact and joint solves were built against.
            if !constrained[i as usize] {
                let v0 = self.last_lin_vel.get(i as usize).copied().unwrap_or(vel.lin);
                pose.pos += (v0 - vel.lin) * (0.5 * dt);
            }
            if vel.ang.length_squared() > 1e-9 {
                // Torque-free rotation: world angular momentum L = I_world w is constant through
                // the rotation, so w changes as the body turns (gyroscopic effect, intermediate-
                // axis flips). Keeping w fixed instead loses L and energy.
                let l = self.bodies.inv_inertia_world(i);
                let det = l.determinant();
                let momentum = (det.is_finite() && det.abs() > 1e-20).then(|| l.inverse().mul_vec3(vel.ang));
                let turn = |q: Quat, w: Vec3| -> Quat {
                    if w.length_squared() <= 1e-18 { return q; }
                    (Quat::from_axis_angle(w.normalize(), w.length() * dt) * q).normalize()
                };
                let start = pose.rot;
                pose.rot = turn(start, vel.ang);
                if let Some(m) = momentum {
                    let inv_local = self.bodies.inv_inertia_local_of(i);
                    let w_at = |q: Quat| { let r = riftphys_core::Mat3::from_quat(q); (r * inv_local * r.transpose()).mul_vec3(m) };
                    // Midpoint: turn by the average of the start and end rates (second order;
                    // turning by the start rate alone drifted the energy 34% in 10 s at 60 Hz)
                    let mut w_end = w_at(pose.rot);
                    for _ in 0..2 {
                        pose.rot = turn(start, (vel.ang + w_end) * 0.5);
                        w_end = w_at(pose.rot);
                    }
                    vel.ang = w_end;
                    self.bodies.set_vel(i, vel);
                }
            }
            self.bodies.set_pose(i, pose);
            predicted_poses[i as usize] = self.bodies.pose(i);
        }

        // --- STAGE 5: Solve Joints + Contacts (Position Level) ---
        // Interleaved so joint corrections cannot drive bodies (feet under a heavy load) into the
        // ground: each joint pass is followed by a contact projection.
        let pos_contacts = self.pos_contacts(&contacts, &original_poses);
        let has_joints = !self.joints.kinds().is_empty();
        // Displacement from contact projection per body (linear, angular): kept out of the
        // derived velocity (split impulse), so pushing a body out of penetration doesn't launch it.
        let mut split = vec![(Vec3::ZERO, Vec3::ZERO); count as usize];
        if has_joints || !pos_contacts.is_empty() {
            for _ in 0..self.solver_iters.1 {
                if has_joints { self.solve_joints(dt, 1); }
                self.project_contacts(&pos_contacts, &mut split, false);
            }
            // Shock propagation, as in the velocity solve: bottom-up, lower body immovable
            self.project_contacts(&pos_contacts, &mut split, true);
        }

        // --- STAGE 6: Velocity update from position corrections ---
        // XPBD: what the joint solve moved a body (final pose - predicted pose) becomes velocity.
        // Only the correction is added: re-deriving the whole velocity from position differences
        // loses ~ulp(position)/dt per step (1.4% of free-fall speed at 240 Hz, 100 m up).
        for i in 0..count {
            if !self.bodies.is_dynamic(i) || !self.sleep.is_awake(i as usize) { continue; }
            let mut vel = self.bodies.vel(i);
            let old_pose = original_poses[i as usize];
            let pred = predicted_poses[i as usize];
            let mut new_pose = self.bodies.pose(i);
            // Split impulse only for bodies outside articulations: on a jointed body (a loaded
            // foot) contact projection and the joint solve push against each other every
            // iteration, so the summed contact push says nothing about the net motion and
            // subtracting it injected metres per second of false velocity. (Splitting the joint
            // corrections as well was tried: the biped teacher then fell 20/20.)
            let (split_lin, split_ang) = if jointed_body[i as usize] { (Vec3::ZERO, Vec3::ZERO) } else { split[i as usize] };

            let d_lin = new_pose.pos - pred.pos - split_lin;
            if d_lin != Vec3::ZERO { vel.lin += d_lin / dt; }

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
            if !is_driven && new_pose.rot != pred.rot {
                let dq = new_pose.rot * pred.rot.conjugate();
                let mut d_ang = Vec3::new(dq.x, dq.y, dq.z) * 2.0;
                if dq.w < 0.0 { d_ang = -d_ang; }
                vel.ang += (d_ang - split_ang) / dt;
            }

            self.bodies.set_vel(i, vel);
        }

        if self.last_lin_vel.len() < count as usize { self.last_lin_vel.resize(count as usize, Vec3::ZERO); }
        for i in 0..count { self.last_lin_vel[i as usize] = self.bodies.vel(i).lin; }

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

    /// Builds the position-level view of this step's contacts from the poses they were found at.
    fn pos_contacts(&self, contacts: &[Contact], start: &[Isometry]) -> Vec<PosContact> {
        let pose_at = |id: u32| -> Isometry {
            if self.bodies.is_dynamic(id) && self.sleep.is_awake(id as usize) { start[id as usize] } else { self.bodies.pose(id) }
        };
        let movable = |id: u32| self.bodies.inv_mass_of(id) > 0.0 && self.sleep.is_awake(id as usize);
        contacts.iter().filter_map(|c| {
            let a = self.colliders[c.a_collider].body.0;
            let b = if c.b_collider == TERRAIN_CI { None } else { Some(self.colliders[c.b_collider].body.0) };
            if !movable(a) && !b.map_or(false, movable) { return None; }
            let pen0 = c.depth - MELEE_CCD_MARGIN;
            let pa = pose_at(a);
            let la = pa.rot.conjugate() * (c.point - pa.pos);
            let (lb, pb_world) = match b {
                Some(b) => { let pb = pose_at(b); (pb.rot.conjugate() * (c.point - pb.pos), c.point) }
                None => (Vec3::ZERO, c.point),
            };
            Some(PosContact { a, b, la, lb, pb_world, n: c.normal, pen0 })
        }).collect()
    }

    /// One pass of position-level contact projection: pushes each penetrating pair apart (with
    /// rotation, through each body's inverse inertia) until only the slop remains.
    fn project_contacts(&mut self, pcs: &[PosContact], split: &mut [(Vec3, Vec3)], shock: bool) {
        let up = if self.gravity.length_squared() > 0.0 { -self.gravity.normalize() } else { Vec3::Y };
        let mut order: Vec<usize> = (0..pcs.len()).collect();
        if shock {
            let height = |pc: &PosContact| match pc.b {
                Some(b) => self.bodies.pose(pc.a).pos.dot(up).min(self.bodies.pose(b).pos.dot(up)),
                None => f32::NEG_INFINITY,
            };
            order.sort_by(|&x, &y| height(&pcs[x]).partial_cmp(&height(&pcs[y])).unwrap_or(std::cmp::Ordering::Equal).then(x.cmp(&y)));
        }
        for &k in &order {
            let pc = &pcs[k];
            let mut a_mov = self.bodies.inv_mass_of(pc.a) > 0.0 && self.sleep.is_awake(pc.a as usize);
            let mut b_mov = pc.b.map_or(false, |b| self.bodies.inv_mass_of(b) > 0.0 && self.sleep.is_awake(b as usize));
            if shock && a_mov && b_mov {
                let (ha, hb) = (self.bodies.pose(pc.a).pos.dot(up), self.bodies.pose(pc.b.unwrap()).pos.dot(up));
                if ha < hb { a_mov = false; } else { b_mov = false; }
            }
            let pa = self.bodies.pose(pc.a);
            let wa = pa.pos + pa.rot * pc.la;
            let (wb, pb) = match pc.b {
                Some(b) => { let pb = self.bodies.pose(b); (pb.pos + pb.rot * pc.lb, Some(pb)) }
                None => (pc.pb_world, None),
            };
            let pen = pc.pen0 - (wb - wa).dot(pc.n); // both anchors start at the contact point
            let err = pen - CONTACT_POS_SLOP;
            if err <= 0.0 { continue; }

            let ra = wa - pa.pos;
            let rb = pb.map_or(Vec3::ZERO, |p| wb - p.pos);
            let (ima, iia) = if a_mov { (self.bodies.inv_mass_of(pc.a), self.bodies.inv_inertia_world(pc.a)) } else { (0.0, riftphys_core::Mat3::ZERO) };
            let (imb, iib) = match pc.b { Some(b) if b_mov => (self.bodies.inv_mass_of(b), self.bodies.inv_inertia_world(b)), _ => (0.0, riftphys_core::Mat3::ZERO) };
            let ca = ra.cross(pc.n);
            let cb = rb.cross(pc.n);
            let w = ima + imb + ca.dot(iia.mul_vec3(ca)) + cb.dot(iib.mul_vec3(cb));
            if w <= 1e-9 { continue; }
            let lambda = err / w;
            let move_body = |bodies: &mut riftphys_dynamics::dynamics::Bodies, split: &mut [(Vec3, Vec3)], id: u32, pose: Isometry, im: f32, ii: riftphys_core::Mat3, r: Vec3, j: Vec3| {
                let dth = ii.mul_vec3(r.cross(j));
                split[id as usize].0 += j * im;
                split[id as usize].1 += dth;
                let dq = Quat::from_xyzw(dth.x, dth.y, dth.z, 0.0) * pose.rot * 0.5;
                let rot = Quat::from_xyzw(pose.rot.x + dq.x, pose.rot.y + dq.y, pose.rot.z + dq.z, pose.rot.w + dq.w).normalize();
                bodies.set_pose(id, Isometry { pos: pose.pos + j * im, rot });
            };
            if a_mov { move_body(&mut self.bodies, split, pc.a, pa, ima, iia, ra, -pc.n * lambda); }
            if let (Some(b), Some(p)) = (pc.b, pb) {
                if b_mov { move_body(&mut self.bodies, split, b, p, imb, iib, rb, pc.n * lambda); }
            }
        }
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
