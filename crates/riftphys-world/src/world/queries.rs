// crates/riftphys-world/src/world/queries.rs

use super::geom::ray_aabb;
use super::world::{RayHit, SweepHit, World};
use riftphys_core::{BodyId, Vec3, iso, Quat};
use riftphys_geom::{aabb_of, ray_shape, Aabb, Shape};
use riftphys_query as query;
use riftphys_collision::environment::EnvCollider;

const TERRAIN_HIT_BODY: BodyId = BodyId(u32::MAX);

impl World {
    /// Ray cast against the true shapes of colliders and environment primitives (rotated boxes,
    /// spheres, capsules, cylinders, triangles), where `raycast` tests collider bounding boxes.
    /// Bodies for which `skip` returns true are ignored (e.g. a sensor's own robot). Read-only:
    /// the result is a pure function of the world state. Ties go to the lowest collider index,
    /// then to environments.
    pub fn raycast_exact(&self, origin: Vec3, dir: Vec3, max_dist: f32, skip: impl Fn(BodyId) -> bool) -> Option<RayHit> {
        let max_d = max_dist.max(0.0);
        if max_d <= 0.0 { return None; }
        let len = dir.length();
        if !len.is_finite() || len < 1.0e-12 { return None; }
        let dir = dir / len;

        let mut best: Option<(f32, usize, Vec3)> = None;
        for (ci, c) in self.colliders.iter().enumerate() {
            if skip(c.body) { continue; }
            let pose = c.world_pose(self.bodies.pose(c.body.0));
            if ray_aabb(origin, dir, &aabb_of(&c.shape, &pose), max_d).is_none() { continue; }
            if let Some((t, n)) = ray_shape(&c.shape, &pose, origin, dir, max_d) {
                if best.map_or(true, |(bt, _, _)| t < bt) { best = Some((t, ci, n)); }
            }
        }

        let ray_end = origin + dir * max_d;
        let query = Aabb::new(origin.min(ray_end), origin.max(ray_end));
        let mut env_buffer = Vec::with_capacity(64);
        for env in &self.environments { env.query_aabb(&query, &mut env_buffer); }
        for env_col in env_buffer {
            if let Some((t, n)) = ray_shape(&env_col.shape, &env_col.transform, origin, dir, max_d) {
                if best.map_or(true, |(bt, _, _)| t < bt) { best = Some((t, usize::MAX, n)); }
            }
        }

        let (t, ci, normal) = best?;
        let body = if ci == usize::MAX { TERRAIN_HIT_BODY } else { self.colliders[ci].body };
        Some(RayHit { body, toi: (t / max_d).clamp(0.0, 1.0), point: origin + dir * t, normal })
    }

    pub fn raycast(&self, origin: Vec3, dir: Vec3, max_dist: f32, ignore: Option<BodyId>) -> Option<RayHit> {
        let max_d = max_dist.max(0.0);
        if max_d <= 0.0 { return None; }

        let len = dir.length();
        if !len.is_finite() || len < 1.0e-12 { return None; }
        let dir = dir / len;

        let mut best: Option<(f32, usize, Vec3, Vec3)> = None;

        // 1. Check Dynamic Colliders
        for (ci, c) in self.colliders.iter().enumerate() {
            if let Some(ig) = ignore {
                if c.body == ig { continue; }
            }
            if let Some((t, p, n)) = ray_aabb(origin, dir, &c.aabb, max_d) {
                match best {
                    None => best = Some((t, ci, p, n)),
                    Some((bt, bi, _, _)) => {
                        if (t < bt - 1.0e-9) || ((t - bt).abs() <= 1.0e-9 && ci < bi) {
                            best = Some((t, ci, p, n));
                        }
                    }
                }
            }
        }

        // 2. Check Generic Environments (Voxel grids, Heightfields)
        let ray_end = origin + dir * max_d;
        let query_aabb = Aabb::new(origin.min(ray_end), origin.max(ray_end));
        let mut env_buffer = Vec::with_capacity(64);

        for env in &self.environments {
            env.query_aabb(&query_aabb, &mut env_buffer);
        }

        let ray_obj = riftphys_geom::Ray::new(origin, dir);
        for env_col in env_buffer {
            let hit = match env_col.shape {
                Shape::Triangle(tri) => {
                    // Triangle raycast returns parameter 't' and barycentric coords
                    tri.intersect_ray(&ray_obj, 0.0, max_d).map(|(t, _bary)| {
                        (t, ray_obj.point_at(t), tri.normal())
                    })
                },
                Shape::Box { .. } => {
                    // Fallback to bounding-box testing for voxels
                    let aabb = riftphys_geom::aabb_of(&env_col.shape, &env_col.transform);
                    ray_aabb(origin, dir, &aabb, max_d)
                },
                _ => None,
            };

            if let Some((tt, tp, tn)) = hit {
                best = match best {
                    None => Some((tt, usize::MAX, tp, tn)),
                    Some((bt, bi, bp, bn)) => {
                        if tt < bt - 1.0e-9 { Some((tt, usize::MAX, tp, tn)) } else { Some((bt, bi, bp, bn)) }
                    }
                };
            }
        }

        let (t, ci, point, normal) = best?;
        let body = if ci == usize::MAX { TERRAIN_HIT_BODY } else { self.colliders[ci].body };
        Some(RayHit { body, toi: (t / max_d).clamp(0.0, 1.0), point, normal })
    }

    pub fn sweep_capsule(
        &self,
        from: Vec3,
        to: Vec3,
        radius: f32,
        half_height: f32,
        ignore: Option<BodyId>,
    ) -> Option<SweepHit> {
        let v = to - from;
        let dist_sq = v.length_squared();
        let cap_shape = Shape::Capsule { r: radius, hh: half_height };

        // FIX: Use the 'iso' helper from riftphys_core
        let cap_xf = iso(from, Quat::IDENTITY);

        // 1. DISCRETE PENETRATION CHECK (T=0)
        for (ci, c) in self.colliders.iter().enumerate() {
            if let Some(ig) = ignore {
                if c.body == ig { continue; }
            }

            let body_xf = c.world_pose(self.bodies.pose(c.body.0));
            if let Some(manifold) = riftphys_collision::narrowphase::collide_shapes(
                &cap_shape, &cap_xf, &c.shape, &body_xf, 0.0
            ) {
                if manifold.len > 0 {
                    return Some(SweepHit {
                        body: c.body,
                        toi: 0.0,
                        started_overlapping: true,
                        point: manifold.points[0].position,
                        normal: manifold.normal,
                    });
                }
            }
        }

        // Check environment at T=0
        let start_aabb = riftphys_geom::aabb_of(&cap_shape, &cap_xf);
        let mut env_buffer = Vec::with_capacity(32);
        for env in &self.environments {
            env.query_aabb(&start_aabb, &mut env_buffer);
        }

        for env_col in &env_buffer {
            if let Some(manifold) = riftphys_collision::narrowphase::collide_shapes(
                &cap_shape, &cap_xf, &env_col.shape, &env_col.transform, 0.0
            ) {
                if manifold.len > 0 {
                    return Some(SweepHit {
                        body: TERRAIN_HIT_BODY,
                        toi: 0.0,
                        started_overlapping: true,
                        point: manifold.points[0].position,
                        normal: manifold.normal,
                    });
                }
            }
        }

        // 2. STATIONARY SHORT-CIRCUIT
        if dist_sq < 1.0e-12 { return None; }

        // 3. CONTINUOUS SWEEP (T > 0)
        let dt = 1.0_f32;
        let tip0 = from + Vec3::new(0.0, half_height, 0.0);
        let base0 = from + Vec3::new(0.0, -half_height, 0.0);
        let eps = 1.0e-9;
        let mut best: Option<(f32, usize, u8, Vec3, Vec3)> = None;

        for (ci, c) in self.colliders.iter().enumerate() {
            if let Some(ig) = ignore { if c.body == ig { continue; } }

            if let Some(hit) = query::sweep_two_spheres_aabbs(
                tip0, v, radius, base0, v, radius, std::slice::from_ref(&c.aabb), dt,
            ) {
                let t = hit.toi.clamp(0.0, 1.0);
                let n = hit.normal;
                let center0 = if hit.sample_kind == 0 { tip0 } else { base0 };
                let point = (center0 + v * t) - n * radius;

                let cand = (t, ci, hit.sample_kind, point, n);
                best = match best {
                    None => Some(cand),
                    Some((bt, bi, bk, _, _)) => {
                        if (t < bt - eps) || ((t - bt).abs() <= eps && (ci < bi)) {
                            Some(cand)
                        } else { best }
                    }
                };
            }
        }

        // Sweep against Environment AABBs
        let min_bound = from.min(to) - Vec3::splat(radius + half_height);
        let max_bound = from.max(to) + Vec3::splat(radius + half_height);
        let sweep_aabb = Aabb::new(min_bound, max_bound);

        env_buffer.clear();
        for env in &self.environments { env.query_aabb(&sweep_aabb, &mut env_buffer); }

        for env_col in env_buffer {
            let env_aabb = riftphys_geom::aabb_of(&env_col.shape, &env_col.transform);
            if let Some(hit) = query::sweep_two_spheres_aabbs(
                tip0, v, radius, base0, v, radius, std::slice::from_ref(&env_aabb), dt
            ) {
                let t = hit.toi.clamp(0.0, 1.0);
                let n = hit.normal;
                let center0 = if hit.sample_kind == 0 { tip0 } else { base0 };
                let point = (center0 + v * t) - n * radius;

                best = match best {
                    None => Some((t, usize::MAX, hit.sample_kind, point, n)),
                    Some((bt, bi, bk, bp, bn)) => {
                        if t < bt - eps { Some((t, usize::MAX, hit.sample_kind, point, n)) }
                        else { Some((bt, bi, bk, bp, bn)) }
                    }
                };
            }
        }

        let (toi, ci, _sk, point, normal) = best?;
        let body = if ci == usize::MAX { TERRAIN_HIT_BODY } else { self.colliders[ci].body };

        Some(SweepHit {
            body,
            toi,
            started_overlapping: false,
            point,
            normal
        })
    }
    pub fn sweep_blade_against_boxes(
        &self,
        tip_p0: Vec3,
        tip_v: Vec3,
        tip_r: f32,
        mid_p0: Vec3,
        mid_v: Vec3,
        mid_r: f32,
        dt: f32,
    ) -> Option<(usize, f32, Vec3)> {
        let mut boxes: Vec<Aabb> = Vec::new();
        for c in &self.colliders {
            if let Shape::Box { .. } = c.shape {
                boxes.push(c.aabb);
            }
        }
        if boxes.is_empty() {
            return None;
        }

        let hit = query::sweep_two_spheres_aabbs(
            tip_p0, tip_v, tip_r,
            mid_p0, mid_v, mid_r,
            &boxes,
            dt,
        )?;

        Some((hit.target_index, hit.toi, hit.normal))
    }}