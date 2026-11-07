#![cfg(test)]

use super::*;
use std::ffi::CStr;

struct W(*mut RPhysWorld);
impl Drop for W {
    fn drop(&mut self) {
        unsafe {
            if !self.0.is_null() {
                rphys_world_destroy(self.0);
            }
        }
    }
}

fn mk_world() -> W {
    unsafe {
        let w = rphys_world_create(1024, 2048);
        assert!(!w.is_null(), "rphys_world_create returned null");

        assert_eq!(rphys_world_set_epoch(w, 1), RPHYS_OK, "err={}", last_err());
        assert_eq!(rphys_world_set_rng_seed(w, 0xBADC0FFEu64), RPHYS_OK, "err={}", last_err());

        W(w)
    }
}

fn last_err() -> String {
    unsafe {
        let p = rphys_last_error_message();
        if p.is_null() {
            "<null>".to_string()
        } else {
            CStr::from_ptr(p).to_string_lossy().into_owned()
        }
    }
}

fn mk_identity_iso(x: f32, y: f32, z: f32) -> RPhysIsometry {
    RPhysIsometry {
        pos: RPhysVec3 { x, y, z },
        rot: RPhysQuat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
    }
}

fn mk_zero_vel() -> RPhysVelocity {
    RPhysVelocity {
        lin: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
        ang: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
    }
}

fn mk_default_mat() -> RPhysMaterial {
    RPhysMaterial { mu_static: 0.9, mu_dynamic: 0.7, restitution: 0.0 }
}

/* ===================== BASIC ABI ===================== */

#[test]
fn abi_version_matches_constant() {
    assert_eq!(rphys_get_abi_version(), RPHYS_ABI_VERSION);
}

#[test]
fn build_id_is_non_null() {
    unsafe {
        let p = rphys_get_build_id();
        assert!(!p.is_null());
        let s = CStr::from_ptr(p).to_string_lossy();
        assert!(s.contains("riftphys_ffi-"), "build id was: {s}");
    }
}

#[test]
fn world_create_destroy_smoke() {
    let _w = mk_world();
}

/* ===================== STEPPING ===================== */

#[test]
fn step_ex_rejects_small_size_bytes() {
    unsafe {
        let w = mk_world();

        let mut p = RPhysStepParams {
            size_bytes: 0,
            flags: 0,
            substeps: 1,
            solver_iters: 0,
            dt: 1.0 / 60.0,
        };

        p.size_bytes = (std::mem::size_of::<RPhysStepParams>() as u32) - 1;
        let r = rphys_world_step_ex(w.0, &p);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "expected BAD_ARG, got {r}, err={}", last_err());
    }
}

#[test]
fn step_ex_rejects_substeps_zero() {
    unsafe {
        let w = mk_world();

        let p = RPhysStepParams {
            size_bytes: std::mem::size_of::<RPhysStepParams>() as u32,
            flags: 0,
            substeps: 0,
            solver_iters: 0,
            dt: 1.0 / 60.0,
        };

        let r = rphys_world_step_ex(w.0, &p);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "expected BAD_ARG, got {r}, err={}", last_err());
    }
}

#[test]
fn step_ex_rejects_non_finite_dt() {
    unsafe {
        let w = mk_world();

        let mut p = RPhysStepParams {
            size_bytes: std::mem::size_of::<RPhysStepParams>() as u32,
            flags: 0,
            substeps: 1,
            solver_iters: 0,
            dt: f32::NAN,
        };

        let r = rphys_world_step_ex(w.0, &p);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "NaN dt: got {r}, err={}", last_err());

        p.dt = f32::INFINITY;
        let r = rphys_world_step_ex(w.0, &p);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "Inf dt: got {r}, err={}", last_err());
    }
}

/* ===================== INTEGRATION ===================== */

#[test]
fn sphere_integrates_downward_velocity() {
    unsafe {
        let w = mk_world();

        let desc = RPhysSphereBodyDesc {
            pose: mk_identity_iso(0.0, 10.0, 0.0),
            vel: RPhysVelocity {
                lin: RPhysVec3 { x: 0.0, y: -1.0, z: 0.0 },
                ang: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            },
            radius: 0.5,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: RPhysMaterial { mu_static: 0.5, mu_dynamic: 0.4, restitution: 0.1 },
            user_tag: 0,
        };

        let mut id: u32 = RPHYS_INVALID_BODY_ID;
        assert_eq!(rphys_add_sphere_body(w.0, &desc, &mut id), RPHYS_OK, "err={}", last_err());
        assert_ne!(id, RPHYS_INVALID_BODY_ID);

        for _ in 0..60 {
            assert_eq!(rphys_world_step(w.0, 1.0 / 60.0), RPHYS_OK, "err={}", last_err());
        }

        let mut pose_out = mk_identity_iso(0.0, 0.0, 0.0);
        assert_eq!(rphys_body_get_pose(w.0 as *const RPhysWorld, id, &mut pose_out), RPHYS_OK, "err={}", last_err());
        assert!(pose_out.pos.y < 10.0);
    }
}

/* ===================== QUERIES ===================== */

#[test]
fn raycast_hits_ground_box_and_ignore_works() {
    unsafe {
        let w = mk_world();

        let ground = RPhysBoxBodyDesc {
            pose: mk_identity_iso(0.0, -0.5, 0.0),
            vel: mk_zero_vel(),
            hx: 50.0,
            hy: 0.5,
            hz: 50.0,
            mass: 0.0,
            body_type: RPHYS_BODY_STATIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        let mut ground_id: u32 = RPHYS_INVALID_BODY_ID;
        assert_eq!(rphys_add_box_body(w.0, &ground, &mut ground_id), RPHYS_OK, "err={}", last_err());
        assert_ne!(ground_id, RPHYS_INVALID_BODY_ID);

        let q = RPhysRayQuery {
            origin: RPhysVec3 { x: 0.0, y: 5.0, z: 0.0 },
            dir:    RPhysVec3 { x: 0.0, y: -1.0, z: 0.0 },
            max_distance: 20.0,
            ignore_body: 0,
        };

        let mut hit = RPhysRayHit {
            hit: RPHYS_FALSE,
            body_id: RPHYS_INVALID_BODY_ID,
            fraction: 0.0,
            point: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            normal: RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 },
        };

        assert_eq!(rphys_world_raycast(w.0 as *const RPhysWorld, &q, &mut hit), RPHYS_OK, "err={}", last_err());
        assert_eq!(hit.hit, RPHYS_TRUE);
        assert_eq!(hit.body_id, ground_id);

        let q2 = RPhysRayQuery { ignore_body: ground_id, ..q };
        let mut hit2 = hit;
        assert_eq!(rphys_world_raycast(w.0 as *const RPhysWorld, &q2, &mut hit2), RPHYS_OK, "err={}", last_err());
        assert_eq!(hit2.hit, RPHYS_FALSE);
        assert_eq!(hit2.body_id, RPHYS_INVALID_BODY_ID);
        assert_eq!(hit2.fraction, 1.0);
    }
}

#[test]
fn raycast_zero_direction_is_safe_and_misses() {
    unsafe {
        let w = mk_world();

        let q = RPhysRayQuery {
            origin: RPhysVec3 { x: 0.0, y: 5.0, z: 0.0 },
            dir:    RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            max_distance: 20.0,
            ignore_body: 0,
        };

        let mut hit = RPhysRayHit {
            hit: RPHYS_TRUE,
            body_id: 123,
            fraction: 0.0,
            point: RPhysVec3 { x: 9.0, y: 9.0, z: 9.0 },
            normal: RPhysVec3 { x: 9.0, y: 9.0, z: 9.0 },
        };

        let r = rphys_world_raycast(w.0 as *const RPhysWorld, &q, &mut hit);
        assert_eq!(r, RPHYS_OK, "err={}", last_err());
        assert_eq!(hit.hit, RPHYS_FALSE);
        assert_eq!(hit.body_id, RPHYS_INVALID_BODY_ID);
        assert_eq!(hit.fraction, 1.0);
    }
}

#[test]
fn raycast_invalid_ignore_returns_invalid_id_and_writes_miss() {
    unsafe {
        let w = mk_world();

        let q = RPhysRayQuery {
            origin: RPhysVec3 { x: 0.0, y: 5.0, z: 0.0 },
            dir:    RPhysVec3 { x: 0.0, y: -1.0, z: 0.0 },
            max_distance: 20.0,
            ignore_body: 999999, // out-of-range
        };

        let mut hit = RPhysRayHit {
            hit: RPHYS_TRUE,
            body_id: 123,
            fraction: 0.0,
            point: RPhysVec3 { x: 9.0, y: 9.0, z: 9.0 },
            normal: RPhysVec3 { x: 9.0, y: 9.0, z: 9.0 },
        };

        let r = rphys_world_raycast(w.0 as *const RPhysWorld, &q, &mut hit);
        assert_eq!(r, RPHYS_ERR_INVALID_ID, "err={}", last_err());
        assert_eq!(hit.hit, RPHYS_FALSE);
        assert_eq!(hit.body_id, RPHYS_INVALID_BODY_ID);
        assert_eq!(hit.fraction, 1.0);
    }
}

#[test]
fn capsule_sweep_hits_wall_and_ignore_works() {
    unsafe {
        let w = mk_world();

        let wall = RPhysBoxBodyDesc {
            pose: mk_identity_iso(3.0, 1.0, 0.0),
            vel: mk_zero_vel(),
            hx: 0.125,
            hy: 2.0,
            hz: 2.0,
            mass: 0.0,
            body_type: RPHYS_BODY_STATIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        let mut wall_id: u32 = RPHYS_INVALID_BODY_ID;
        assert_eq!(rphys_add_box_body(w.0, &wall, &mut wall_id), RPHYS_OK, "err={}", last_err());
        assert_ne!(wall_id, RPHYS_INVALID_BODY_ID);

        let q = RPhysCapsuleSweepQuery {
            from: RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 },
            to:   RPhysVec3 { x: 6.0, y: 1.0, z: 0.0 },
            radius: 0.5,
            half_height: 0.5,
            ignore_body: 0,
        };

        let mut hit = RPhysCapsuleSweepHit {
            hit: RPHYS_FALSE,
            body_id: RPHYS_INVALID_BODY_ID,
            fraction: 0.0,
            started_overlapping: RPHYS_FALSE,
            point: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            normal: RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 },
        };

        assert_eq!(rphys_world_capsule_sweep(w.0 as *const RPhysWorld, &q, &mut hit), RPHYS_OK, "err={}", last_err());
        assert_eq!(hit.hit, RPHYS_TRUE);
        assert_eq!(hit.body_id, wall_id);
        assert!((0.0..=1.0).contains(&hit.fraction));

        let q2 = RPhysCapsuleSweepQuery { ignore_body: wall_id, ..q };
        let mut hit2 = hit;
        assert_eq!(rphys_world_capsule_sweep(w.0 as *const RPhysWorld, &q2, &mut hit2), RPHYS_OK, "err={}", last_err());
        assert_eq!(hit2.hit, RPHYS_FALSE);
        assert_eq!(hit2.body_id, RPHYS_INVALID_BODY_ID);
        assert_eq!(hit2.fraction, 1.0);
    }
}

/* ===================== DETERMINISM ===================== */

#[test]
fn determinism_hash_matches_between_two_worlds() {
    unsafe {
        let a = mk_world();
        let b = mk_world();

        let sphere = RPhysSphereBodyDesc {
            pose: mk_identity_iso(2.0, 3.0, 4.0),
            vel: RPhysVelocity {
                lin: RPhysVec3 { x: 1.0, y: -2.0, z: 0.5 },
                ang: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            },
            radius: 0.25,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: RPhysMaterial { mu_static: 0.5, mu_dynamic: 0.4, restitution: 0.1 },
            user_tag: 0,
        };

        let mut ida: u32 = 0;
        let mut idb: u32 = 0;
        assert_eq!(rphys_add_sphere_body(a.0, &sphere, &mut ida), RPHYS_OK, "err={}", last_err());
        assert_eq!(rphys_add_sphere_body(b.0, &sphere, &mut idb), RPHYS_OK, "err={}", last_err());
        assert_ne!(ida, 0);
        assert_ne!(idb, 0);

        let mut ha = RPhysHash32 { bytes: [0u8; RPHYS_STEP_HASH_BYTES] };
        let mut hb = RPhysHash32 { bytes: [0u8; RPHYS_STEP_HASH_BYTES] };

        for _ in 0..120 {
            assert_eq!(rphys_world_step_and_hash(a.0, 1.0 / 120.0, &mut ha), RPHYS_OK, "err={}", last_err());
            assert_eq!(rphys_world_step_and_hash(b.0, 1.0 / 120.0, &mut hb), RPHYS_OK, "err={}", last_err());
            assert_eq!(ha.bytes, hb.bytes, "hash diverged between identical runs");
        }
    }
}

/* ===================== HEIGHTFIELD ===================== */

#[test]
fn heightfield_i16_rejects_bad_inputs() {
    unsafe {
        let w = mk_world();

        let r = rphys_world_set_heightfield_i16(w.0, std::ptr::null(), 0.0, 0.0, 0.0);
        assert_eq!(r, RPHYS_ERR_NULL);

        let heights: [i16; 4] = [0, 0, 0, 0];
        let mut d = RPhysHeightfieldDesc {
            size_bytes: 0,
            flags: 0,
            width: 2,
            height: 2,
            row_stride: 0,
            cell_size_x: 1.0,
            cell_size_z: 1.0,
            height_scale: 1.0,
            height_offset: 0.0,
            heights: heights.as_ptr(),
        };
        d.size_bytes = (std::mem::size_of::<RPhysHeightfieldDesc>() as u32) - 1;
        let r = rphys_world_set_heightfield_i16(w.0, &d, 0.0, 0.0, 0.0);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "err={}", last_err());

        let mut d2 = d;
        d2.size_bytes = std::mem::size_of::<RPhysHeightfieldDesc>() as u32;
        d2.width = 0;
        let r = rphys_world_set_heightfield_i16(w.0, &d2, 0.0, 0.0, 0.0);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "err={}", last_err());

        let mut d3 = d;
        d3.size_bytes = std::mem::size_of::<RPhysHeightfieldDesc>() as u32;
        d3.heights = std::ptr::null();
        let r = rphys_world_set_heightfield_i16(w.0, &d3, 0.0, 0.0, 0.0);
        assert_eq!(r, RPHYS_ERR_NULL, "err={}", last_err());

        let mut d4 = d;
        d4.size_bytes = std::mem::size_of::<RPhysHeightfieldDesc>() as u32;
        d4.row_stride = 1;
        let r = rphys_world_set_heightfield_i16(w.0, &d4, 0.0, 0.0, 0.0);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "err={}", last_err());

        let mut d5 = d;
        d5.size_bytes = std::mem::size_of::<RPhysHeightfieldDesc>() as u32;
        d5.cell_size_x = 0.0;
        let r = rphys_world_set_heightfield_i16(w.0, &d5, 0.0, 0.0, 0.0);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "err={}", last_err());
    }
}

#[test]
fn heightfield_raw32_rejects_bad_inputs() {
    unsafe {
        let w = mk_world();

        let r = rphys_world_set_heightfield_raw32_square(
            w.0,
            std::ptr::null(),
            0,
            10.0,
            10.0,
            0.0,
            0.0,
            0.0,
        );
        assert_eq!(r, RPHYS_ERR_NULL);

        let bytes = [0u8; 3];
        let r = rphys_world_set_heightfield_raw32_square(
            w.0,
            bytes.as_ptr(),
            bytes.len() as u32,
            10.0,
            10.0,
            0.0,
            0.0,
            0.0,
        );
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "err={}", last_err());
    }
}

#[test]
fn heightfield_set_and_clear_smoke() {
    unsafe {
        let w = mk_world();

        let heights: [i16; 4] = [0, 0, 0, 0];
        let d = RPhysHeightfieldDesc {
            size_bytes: std::mem::size_of::<RPhysHeightfieldDesc>() as u32,
            flags: 0,
            width: 2,
            height: 2,
            row_stride: 0,
            cell_size_x: 1.0,
            cell_size_z: 1.0,
            height_scale: 1.0,
            height_offset: 0.0,
            heights: heights.as_ptr(),
        };

        assert_eq!(rphys_world_set_heightfield_i16(w.0, &d, 0.0, 0.0, 0.0), RPHYS_OK, "err={}", last_err());

        for _ in 0..10 {
            assert_eq!(rphys_world_step(w.0, 1.0 / 60.0), RPHYS_OK, "err={}", last_err());
        }

        assert_eq!(rphys_world_clear_heightfield(w.0), RPHYS_OK, "err={}", last_err());
    }
}

#[test]
fn raycast_hits_terrain_when_heightfield_enabled() {
    unsafe {
        let w = mk_world();

        let heights: [i16; 4] = [0, 0, 0, 0];
        let d = RPhysHeightfieldDesc {
            size_bytes: std::mem::size_of::<RPhysHeightfieldDesc>() as u32,
            flags: 0,
            width: 2,
            height: 2,
            row_stride: 0,
            cell_size_x: 10.0,
            cell_size_z: 10.0,
            height_scale: 1.0,
            height_offset: 0.0,
            heights: heights.as_ptr(),
        };

        assert_eq!(rphys_world_set_heightfield_i16(w.0, &d, 0.0, 0.0, 0.0), RPHYS_OK, "err={}", last_err());

        let q = RPhysRayQuery {
            origin: RPhysVec3 { x: 1.0, y: 5.0, z: 1.0 },
            dir:    RPhysVec3 { x: 0.0, y: -1.0, z: 0.0 },
            max_distance: 50.0,
            ignore_body: 0,
        };

        let mut hit = RPhysRayHit {
            hit: RPHYS_FALSE,
            body_id: RPHYS_INVALID_BODY_ID,
            fraction: 0.0,
            point: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            normal: RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 },
        };

        assert_eq!(rphys_world_raycast(w.0 as *const RPhysWorld, &q, &mut hit), RPHYS_OK, "err={}", last_err());

        assert_eq!(hit.hit, RPHYS_TRUE, "expected terrain hit; got miss");
        assert_eq!(hit.body_id, RPHYS_TERRAIN_BODY_ID, "expected terrain body id");
        assert!(hit.point.y.abs() < 1.0e-3, "expected y≈0, got {}", hit.point.y);
        assert!(hit.normal.y > 0.9, "expected upward normal, got {}", hit.normal.y);
    }
}

/* ===================== TERRAIN SENTINEL CONTRACT ===================== */

#[test]
fn terrain_body_id_must_be_rejected_by_all_body_apis() {
    unsafe {
        let w = mk_world();

        let mut pose_out = mk_identity_iso(0.0, 0.0, 0.0);
        assert_eq!(
            rphys_body_get_pose(w.0 as *const RPhysWorld, RPHYS_TERRAIN_BODY_ID, &mut pose_out),
            RPHYS_ERR_INVALID_ID,
            "get_pose: err={}",
            last_err()
        );

        let pose_in = mk_identity_iso(1.0, 2.0, 3.0);
        assert_eq!(
            rphys_body_set_pose(w.0, RPHYS_TERRAIN_BODY_ID, &pose_in),
            RPHYS_ERR_INVALID_ID,
            "set_pose: err={}",
            last_err()
        );

        let mut vel_out = mk_zero_vel();
        assert_eq!(
            rphys_body_get_velocity(w.0 as *const RPhysWorld, RPHYS_TERRAIN_BODY_ID, &mut vel_out),
            RPHYS_ERR_INVALID_ID,
            "get_velocity: err={}",
            last_err()
        );

        let vel_in = RPhysVelocity {
            lin: RPhysVec3 { x: 1.0, y: 2.0, z: 3.0 },
            ang: RPhysVec3 { x: 0.1, y: 0.2, z: 0.3 },
        };
        assert_eq!(
            rphys_body_set_velocity(w.0, RPHYS_TERRAIN_BODY_ID, &vel_in),
            RPHYS_ERR_INVALID_ID,
            "set_velocity: err={}",
            last_err()
        );

        assert_eq!(
            rphys_body_apply_impulse(w.0, RPHYS_TERRAIN_BODY_ID, RPhysVec3 { x: 1.0, y: 0.0, z: 0.0 }),
            RPHYS_ERR_INVALID_ID,
            "apply_impulse: err={}",
            last_err()
        );

        assert_eq!(
            rphys_body_remove(w.0, RPHYS_TERRAIN_BODY_ID),
            RPHYS_ERR_INVALID_ID,
            "remove: err={}",
            last_err()
        );
    }
}

#[test]
fn terrain_body_id_must_be_rejected_in_ignore_fields() {
    unsafe {
        let w = mk_world();

        let ground = RPhysBoxBodyDesc {
            pose: mk_identity_iso(0.0, -0.5, 0.0),
            vel: mk_zero_vel(),
            hx: 50.0,
            hy: 0.5,
            hz: 50.0,
            mass: 0.0,
            body_type: RPHYS_BODY_STATIC,
            material: mk_default_mat(),
            user_tag: 0,
        };
        let mut ground_id: u32 = RPHYS_INVALID_BODY_ID;
        assert_eq!(rphys_add_box_body(w.0, &ground, &mut ground_id), RPHYS_OK, "err={}", last_err());

        let q = RPhysRayQuery {
            origin: RPhysVec3 { x: 0.0, y: 5.0, z: 0.0 },
            dir:    RPhysVec3 { x: 0.0, y: -1.0, z: 0.0 },
            max_distance: 20.0,
            ignore_body: RPHYS_TERRAIN_BODY_ID,
        };
        let mut hit = RPhysRayHit {
            hit: RPHYS_FALSE,
            body_id: RPHYS_INVALID_BODY_ID,
            fraction: 0.0,
            point: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            normal: RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 },
        };
        let r = rphys_world_raycast(w.0 as *const RPhysWorld, &q, &mut hit);
        assert_eq!(r, RPHYS_ERR_INVALID_ID, "raycast(ignore=terrain): got {r}, err={}", last_err());

        let sq = RPhysCapsuleSweepQuery {
            from: RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 },
            to:   RPhysVec3 { x: 6.0, y: 1.0, z: 0.0 },
            radius: 0.5,
            half_height: 0.5,
            ignore_body: RPHYS_TERRAIN_BODY_ID,
        };
        let mut shit = RPhysCapsuleSweepHit {
            hit: RPHYS_FALSE,
            body_id: RPHYS_INVALID_BODY_ID,
            fraction: 0.0,
            started_overlapping: RPHYS_FALSE,
            point: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            normal: RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 },
        };
        let r = rphys_world_capsule_sweep(w.0 as *const RPhysWorld, &sq, &mut shit);
        assert_eq!(r, RPHYS_ERR_INVALID_ID, "sweep(ignore=terrain): got {r}, err={}", last_err());
    }
}

/* ===================== REMOVE / LIVENESS ===================== */

#[test]
fn remove_body_makes_id_invalid_and_double_remove_fails() {
    unsafe {
        let w = mk_world();

        let desc = RPhysSphereBodyDesc {
            pose: mk_identity_iso(0.0, 1.0, 0.0),
            vel: mk_zero_vel(),
            radius: 0.5,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        let mut id: u32 = 0;
        assert_eq!(rphys_add_sphere_body(w.0, &desc, &mut id), RPHYS_OK, "err={}", last_err());
        assert!(id != 0);

        assert_eq!(rphys_body_remove(w.0, id), RPHYS_OK, "err={}", last_err());

        let mut pose_out = mk_identity_iso(0.0, 0.0, 0.0);
        assert_eq!(
            rphys_body_get_pose(w.0 as *const RPhysWorld, id, &mut pose_out),
            RPHYS_ERR_INVALID_ID,
            "get_pose on removed id: err={}",
            last_err()
        );

        let vel_in = mk_zero_vel();
        assert_eq!(
            rphys_body_set_velocity(w.0, id, &vel_in),
            RPHYS_ERR_INVALID_ID,
            "set_velocity on removed id: err={}",
            last_err()
        );

        assert_eq!(
            rphys_body_apply_impulse(w.0, id, RPhysVec3 { x: 1.0, y: 0.0, z: 0.0 }),
            RPHYS_ERR_INVALID_ID,
            "apply_impulse on removed id: err={}",
            last_err()
        );

        assert_eq!(
            rphys_body_remove(w.0, id),
            RPHYS_ERR_INVALID_ID,
            "double remove should fail: err={}",
            last_err()
        );

        let q = RPhysRayQuery {
            origin: RPhysVec3 { x: 0.0, y: 5.0, z: 0.0 },
            dir:    RPhysVec3 { x: 0.0, y: -1.0, z: 0.0 },
            max_distance: 20.0,
            ignore_body: id,
        };
        let mut hit = RPhysRayHit {
            hit: RPHYS_FALSE,
            body_id: RPHYS_INVALID_BODY_ID,
            fraction: 0.0,
            point: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            normal: RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 },
        };
        let r = rphys_world_raycast(w.0 as *const RPhysWorld, &q, &mut hit);
        assert_eq!(r, RPHYS_ERR_INVALID_ID, "ignore removed id must be invalid: err={}", last_err());
        assert_eq!(hit.hit, RPHYS_FALSE);
        assert_eq!(hit.body_id, RPHYS_INVALID_BODY_ID);
        assert_eq!(hit.fraction, 1.0);
    }
}

#[test]
fn removed_body_id_must_not_be_reused_in_v1() {
    unsafe {
        let w = mk_world();

        let s0 = RPhysSphereBodyDesc {
            pose: mk_identity_iso(0.0, 1.0, 0.0),
            vel: mk_zero_vel(),
            radius: 0.25,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        let mut id0: u32 = 0;
        assert_eq!(rphys_add_sphere_body(w.0, &s0, &mut id0), RPHYS_OK, "err={}", last_err());
        assert!(id0 != 0);

        assert_eq!(rphys_body_remove(w.0, id0), RPHYS_OK, "err={}", last_err());

        let s1 = RPhysSphereBodyDesc { pose: mk_identity_iso(1.0, 1.0, 0.0), ..s0 };
        let mut id1: u32 = 0;
        assert_eq!(rphys_add_sphere_body(w.0, &s1, &mut id1), RPHYS_OK, "err={}", last_err());
        assert!(id1 != 0);

        assert_ne!(
            id1, id0,
            "ID reuse creates stale-handle hazards in the game; keep tombstones in V1"
        );

        let mut pose_out = mk_identity_iso(0.0, 0.0, 0.0);
        assert_eq!(
            rphys_body_get_pose(w.0 as *const RPhysWorld, id0, &mut pose_out),
            RPHYS_ERR_INVALID_ID,
            "old id must stay invalid even after more adds: err={}",
            last_err()
        );
    }
}

#[test]
fn add_body_respects_capacity_and_returns_err_capacity() {
    unsafe {
        // tiny capacity so we can hit it quickly
        let w = { rphys_world_create(2, 4) };
        assert!(!w.is_null(), "rphys_world_create returned null");

        assert_eq!(rphys_world_set_epoch(w, 1), RPHYS_OK, "err={}", last_err());
        assert_eq!(rphys_world_set_rng_seed(w, 0xBADC0FFEu64), RPHYS_OK, "err={}", last_err());

        let desc = RPhysSphereBodyDesc {
            pose: mk_identity_iso(0.0, 1.0, 0.0),
            vel: mk_zero_vel(),
            radius: 0.25,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        let mut id0 = 0u32;
        let mut id1 = 0u32;
        let mut id2 = 0u32;

        assert_eq!(rphys_add_sphere_body(w, &desc, &mut id0), RPHYS_OK, "err={}", last_err());
        assert_eq!(rphys_add_sphere_body(w, &desc, &mut id1), RPHYS_OK, "err={}", last_err());
        assert_ne!(id0, 0);
        assert_ne!(id1, 0);

        let r = rphys_add_sphere_body(w, &desc, &mut id2);
        assert_eq!(r, RPHYS_ERR_CAPACITY, "expected ERR_CAPACITY, got {r}, err={}", last_err());
        assert_eq!(id2, 0, "out id must remain 0 on failure");

        rphys_world_destroy(w);
    }
}
#[test]
fn add_body_null_args_return_err_null_and_zero_out_id() {
    unsafe {
        let w = mk_world();

        let desc = RPhysSphereBodyDesc {
            pose: mk_identity_iso(0.0, 1.0, 0.0),
            vel: mk_zero_vel(),
            radius: 0.25,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        // null world
        let mut out = 123u32;
        let r = rphys_add_sphere_body(std::ptr::null_mut(), &desc, &mut out);
        assert_eq!(r, RPHYS_ERR_NULL);
        assert_eq!(out, 0, "out id must be zeroed on failure");

        // null desc
        let mut out = 123u32;
        let r = rphys_add_sphere_body(w.0, std::ptr::null(), &mut out);
        assert_eq!(r, RPHYS_ERR_NULL);
        assert_eq!(out, 0, "out id must be zeroed on failure");

        // null out
        let r = rphys_add_sphere_body(w.0, &desc, std::ptr::null_mut());
        assert_eq!(r, RPHYS_ERR_NULL);
    }
}
#[test]
fn capsule_sweep_invalid_ignore_returns_invalid_id_and_writes_miss() {
    unsafe {
        let w = mk_world();

        let q = RPhysCapsuleSweepQuery {
            from: RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 },
            to:   RPhysVec3 { x: 1.0, y: 1.0, z: 0.0 },
            radius: 0.5,
            half_height: 0.5,
            ignore_body: 999_999, // out of range
        };

        let mut hit = RPhysCapsuleSweepHit {
            hit: RPHYS_TRUE, // deliberately "dirty"
            body_id: 123,
            fraction: 0.123,
            started_overlapping: RPHYS_TRUE,
            point: RPhysVec3 { x: 9.0, y: 9.0, z: 9.0 },
            normal: RPhysVec3 { x: 9.0, y: 9.0, z: 9.0 },
        };

        let r = rphys_world_capsule_sweep(w.0 as *const RPhysWorld, &q, &mut hit);
        assert_eq!(r, RPHYS_ERR_INVALID_ID, "got {r}, err={}", last_err());

        // deterministic miss must be written
        assert_eq!(hit.hit, RPHYS_FALSE);
        assert_eq!(hit.body_id, RPHYS_INVALID_BODY_ID);
        assert_eq!(hit.fraction, 1.0);
        assert_eq!(hit.started_overlapping, RPHYS_FALSE);
        assert_eq!(hit.point.x, 0.0);
        assert_eq!(hit.normal.y, 1.0);
    }
}
#[test]
fn tick_index_increments_with_step_and_step_ex_substeps() {
    unsafe {
        let w = mk_world();

        let mut t0: u64 = 0;
        assert_eq!(rphys_world_tick_index(w.0 as *const RPhysWorld, &mut t0), RPHYS_OK);

        assert_eq!(rphys_world_step(w.0, 1.0 / 60.0), RPHYS_OK);

        let mut t1: u64 = 0;
        assert_eq!(rphys_world_tick_index(w.0 as *const RPhysWorld, &mut t1), RPHYS_OK);
        assert_eq!(t1, t0 + 1, "tick must advance by 1 on rphys_world_step");

        let p = RPhysStepParams {
            size_bytes: std::mem::size_of::<RPhysStepParams>() as u32,
            flags: 0,
            substeps: 4,
            solver_iters: 0,
            dt: 1.0 / 60.0,
        };
        assert_eq!(rphys_world_step_ex(w.0, &p), RPHYS_OK);

        let mut t2: u64 = 0;
        assert_eq!(rphys_world_tick_index(w.0 as *const RPhysWorld, &mut t2), RPHYS_OK);
        assert_eq!(t2, t1 + 4, "tick must advance by substeps on step_ex");
    }
}
#[test]
fn apply_impulse_changes_linear_velocity() {
    unsafe {
        let w = mk_world();

        let desc = RPhysSphereBodyDesc {
            pose: mk_identity_iso(0.0, 1.0, 0.0),
            vel: mk_zero_vel(),
            radius: 0.5,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        let mut id = 0u32;
        assert_eq!(rphys_add_sphere_body(w.0, &desc, &mut id), RPHYS_OK);
        assert_ne!(id, 0);

        let mut v0 = mk_zero_vel();
        assert_eq!(rphys_body_get_velocity(w.0 as *const RPhysWorld, id, &mut v0), RPHYS_OK);

        assert_eq!(rphys_body_apply_impulse(w.0, id, RPhysVec3 { x: 10.0, y: 0.0, z: 0.0 }), RPHYS_OK);

        let mut v1 = mk_zero_vel();
        assert_eq!(rphys_body_get_velocity(w.0 as *const RPhysWorld, id, &mut v1), RPHYS_OK);

        assert!(v1.lin.x > v0.lin.x, "impulse should increase x velocity (v0={}, v1={})", v0.lin.x, v1.lin.x);
    }
}
#[test]
fn heightfield_clear_removes_terrain_hits() {
    unsafe {
        let w = mk_world();

        let heights: [i16; 4] = [0, 0, 0, 0];
        let d = RPhysHeightfieldDesc {
            size_bytes: std::mem::size_of::<RPhysHeightfieldDesc>() as u32,
            flags: 0,
            width: 2,
            height: 2,
            row_stride: 0,
            cell_size_x: 10.0,
            cell_size_z: 10.0,
            height_scale: 1.0,
            height_offset: 0.0,
            heights: heights.as_ptr(),
        };

        assert_eq!(rphys_world_set_heightfield_i16(w.0, &d, 0.0, 0.0, 0.0), RPHYS_OK, "err={}", last_err());

        let q = RPhysRayQuery {
            origin: RPhysVec3 { x: 1.0, y: 5.0, z: 1.0 },
            dir:    RPhysVec3 { x: 0.0, y: -1.0, z: 0.0 },
            max_distance: 50.0,
            ignore_body: 0,
        };

        let mut hit = RPhysRayHit {
            hit: RPHYS_FALSE,
            body_id: 0,
            fraction: 0.0,
            point: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            normal: RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 },
        };

        assert_eq!(rphys_world_raycast(w.0 as *const RPhysWorld, &q, &mut hit), RPHYS_OK, "err={}", last_err());
        assert_eq!(hit.hit, RPHYS_TRUE);
        assert_eq!(hit.body_id, RPHYS_TERRAIN_BODY_ID);

        assert_eq!(rphys_world_clear_heightfield(w.0), RPHYS_OK, "err={}", last_err());

        let mut hit2 = hit;
        assert_eq!(rphys_world_raycast(w.0 as *const RPhysWorld, &q, &mut hit2), RPHYS_OK, "err={}", last_err());
        assert_eq!(hit2.hit, RPHYS_FALSE, "after clear, should miss terrain");
        assert_eq!(hit2.body_id, 0);
    }
}
#[test]
// #[ignore]
fn step_ex_rejects_negative_dt() {
    unsafe {
        let w = mk_world();
        let p = RPhysStepParams {
            size_bytes: std::mem::size_of::<RPhysStepParams>() as u32,
            flags: 0,
            substeps: 1,
            solver_iters: 0,
            dt: -1.0 / 60.0,
        };
        let r = rphys_world_step_ex(w.0, &p);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "expected BAD_ARG, got {r}, err={}", last_err());
    }
}
#[test]
// #[ignore]
fn set_pose_rejects_nan() {
    unsafe {
        let w = mk_world();

        let desc = RPhysSphereBodyDesc {
            pose: mk_identity_iso(0.0, 1.0, 0.0),
            vel: mk_zero_vel(),
            radius: 0.5,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: mk_default_mat(),
            user_tag: 0,
        };
        let mut id = 0u32;
        assert_eq!(rphys_add_sphere_body(w.0, &desc, &mut id), RPHYS_OK);

        let bad = RPhysIsometry {
            pos: RPhysVec3 { x: f32::NAN, y: 0.0, z: 0.0 },
            rot: RPhysQuat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        };
        let r = rphys_body_set_pose(w.0, id, &bad);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "expected BAD_ARG, got {r}, err={}", last_err());
    }
}
#[test]
// #[ignore]
fn add_body_respects_collider_capacity_err_capacity() {
    unsafe {
        // allow lots of bodies, but only 1 collider
        let w = rphys_world_create(128, 1);
        assert!(!w.is_null());
        assert_eq!(rphys_world_set_epoch(w, 1), RPHYS_OK);
        assert_eq!(rphys_world_set_rng_seed(w, 0xBADC0FFEu64), RPHYS_OK);

        let desc = RPhysSphereBodyDesc {
            pose: mk_identity_iso(0.0, 1.0, 0.0),
            vel: mk_zero_vel(),
            radius: 0.25,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        let mut id0 = 0u32;
        let mut id1 = 0u32;
        assert_eq!(rphys_add_sphere_body(w, &desc, &mut id0), RPHYS_OK);

        let r = rphys_add_sphere_body(w, &desc, &mut id1);
        assert_eq!(r, RPHYS_ERR_CAPACITY, "expected ERR_CAPACITY, got {r}, err={}", last_err());
        assert_eq!(id1, 0);

        rphys_world_destroy(w);
    }
}

#[test]
fn add_sphere_rejects_bad_radius() {
    unsafe {
        let w = mk_world();

        let mut desc = RPhysSphereBodyDesc {
            pose: mk_identity_iso(0.0, 1.0, 0.0),
            vel: mk_zero_vel(),
            radius: 0.5,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        let mut id = 777u32;

        desc.radius = 0.0;
        let r = rphys_add_sphere_body(w.0, &desc, &mut id);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "err={}", last_err());
        assert_eq!(id, 0);

        desc.radius = -1.0;
        let r = rphys_add_sphere_body(w.0, &desc, &mut id);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "err={}", last_err());
        assert_eq!(id, 0);

        desc.radius = f32::NAN;
        let r = rphys_add_sphere_body(w.0, &desc, &mut id);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "err={}", last_err());
        assert_eq!(id, 0);
    }
}

#[test]
fn add_box_rejects_bad_extents() {
    unsafe {
        let w = mk_world();

        let mut desc = RPhysBoxBodyDesc {
            pose: mk_identity_iso(0.0, 1.0, 0.0),
            vel: mk_zero_vel(),
            hx: 1.0,
            hy: 1.0,
            hz: 1.0,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        let mut id = 777u32;

        desc.hx = 0.0;
        let r = rphys_add_box_body(w.0, &desc, &mut id);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "err={}", last_err());
        assert_eq!(id, 0);

        desc.hx = 1.0;
        desc.hy = -0.1;
        let r = rphys_add_box_body(w.0, &desc, &mut id);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "err={}", last_err());
        assert_eq!(id, 0);

        desc.hy = 1.0;
        desc.hz = f32::INFINITY;
        let r = rphys_add_box_body(w.0, &desc, &mut id);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "err={}", last_err());
        assert_eq!(id, 0);
    }
}

#[test]
fn add_capsule_rejects_bad_dims() {
    unsafe {
        let w = mk_world();

        let mut desc = RPhysCapsuleBodyDesc {
            pose: mk_identity_iso(0.0, 1.0, 0.0),
            vel: mk_zero_vel(),
            radius: 0.5,
            half_height: 0.5,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        let mut id = 777u32;

        desc.radius = 0.0;
        let r = rphys_add_capsule_body(w.0, &desc, &mut id);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "err={}", last_err());
        assert_eq!(id, 0);

        desc.radius = 0.5;
        desc.half_height = -0.25;
        let r = rphys_add_capsule_body(w.0, &desc, &mut id);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "err={}", last_err());
        assert_eq!(id, 0);

        desc.half_height = 0.5;
        desc.radius = f32::NAN;
        let r = rphys_add_capsule_body(w.0, &desc, &mut id);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "err={}", last_err());
        assert_eq!(id, 0);
    }
}
#[test]
fn set_velocity_rejects_nan() {
    unsafe {
        let w = mk_world();

        let desc = RPhysSphereBodyDesc {
            pose: mk_identity_iso(0.0, 1.0, 0.0),
            vel: mk_zero_vel(),
            radius: 0.5,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        let mut id = 0u32;
        assert_eq!(rphys_add_sphere_body(w.0, &desc, &mut id), RPHYS_OK);

        let bad = RPhysVelocity {
            lin: RPhysVec3 { x: f32::NAN, y: 0.0, z: 0.0 },
            ang: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
        };

        let r = rphys_body_set_velocity(w.0, id, &bad);
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "err={}", last_err());
    }
}

#[test]
fn apply_impulse_rejects_nan() {
    unsafe {
        let w = mk_world();

        let desc = RPhysSphereBodyDesc {
            pose: mk_identity_iso(0.0, 1.0, 0.0),
            vel: mk_zero_vel(),
            radius: 0.5,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        let mut id = 0u32;
        assert_eq!(rphys_add_sphere_body(w.0, &desc, &mut id), RPHYS_OK);

        let r = rphys_body_apply_impulse(w.0, id, RPhysVec3 { x: f32::INFINITY, y: 0.0, z: 0.0 });
        assert_eq!(r, RPHYS_ERR_BAD_ARG, "err={}", last_err());
    }
}
#[test]
fn raycast_fraction_is_in_0_1_on_hit_and_1_on_miss() {
    unsafe {
        let w = mk_world();

        let ground = RPhysBoxBodyDesc {
            pose: mk_identity_iso(0.0, -0.5, 0.0),
            vel: mk_zero_vel(),
            hx: 50.0,
            hy: 0.5,
            hz: 50.0,
            mass: 0.0,
            body_type: RPHYS_BODY_STATIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        let mut ground_id = 0u32;
        assert_eq!(rphys_add_box_body(w.0, &ground, &mut ground_id), RPHYS_OK);

        let q_hit = RPhysRayQuery {
            origin: RPhysVec3 { x: 0.0, y: 5.0, z: 0.0 },
            dir:    RPhysVec3 { x: 0.0, y: -1.0, z: 0.0 },
            max_distance: 20.0,
            ignore_body: 0,
        };

        let mut hit = RPhysRayHit { hit: 0, body_id: 0, fraction: 0.0, point: RPhysVec3{x:0.0,y:0.0,z:0.0}, normal: RPhysVec3{x:0.0,y:1.0,z:0.0} };
        assert_eq!(rphys_world_raycast(w.0 as *const RPhysWorld, &q_hit, &mut hit), RPHYS_OK);
        assert_eq!(hit.hit, RPHYS_TRUE);
        assert_eq!(hit.body_id, ground_id);
        assert!((0.0..=1.0).contains(&hit.fraction), "fraction out of range: {}", hit.fraction);

        let q_miss = RPhysRayQuery { dir: RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 }, ..q_hit };
        let mut miss = hit;
        assert_eq!(rphys_world_raycast(w.0 as *const RPhysWorld, &q_miss, &mut miss), RPHYS_OK);
        assert_eq!(miss.hit, RPHYS_FALSE);
        assert_eq!(miss.fraction, 1.0);
        assert_eq!(miss.body_id, 0);
    }
}
#[test]
fn determinism_hash_matches_with_contacts_drop_on_ground() {
    unsafe {
        let a = mk_world();
        let b = mk_world();

        let ground = RPhysBoxBodyDesc {
            pose: mk_identity_iso(0.0, -0.5, 0.0),
            vel: mk_zero_vel(),
            hx: 50.0, hy: 0.5, hz: 50.0,
            mass: 0.0,
            body_type: RPHYS_BODY_STATIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        let mut ga=0u32; let mut gb=0u32;
        assert_eq!(rphys_add_box_body(a.0, &ground, &mut ga), RPHYS_OK);
        assert_eq!(rphys_add_box_body(b.0, &ground, &mut gb), RPHYS_OK);

        let sphere = RPhysSphereBodyDesc {
            pose: mk_identity_iso(0.0, 5.0, 0.0),
            vel: mk_zero_vel(),
            radius: 0.5,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: RPhysMaterial { mu_static: 0.9, mu_dynamic: 0.7, restitution: 0.0 },
            user_tag: 0,
        };

        let mut sa=0u32; let mut sb=0u32;
        assert_eq!(rphys_add_sphere_body(a.0, &sphere, &mut sa), RPHYS_OK);
        assert_eq!(rphys_add_sphere_body(b.0, &sphere, &mut sb), RPHYS_OK);

        let mut ha = RPhysHash32 { bytes: [0u8; RPHYS_STEP_HASH_BYTES] };
        let mut hb = RPhysHash32 { bytes: [0u8; RPHYS_STEP_HASH_BYTES] };

        for _ in 0..240 {
            assert_eq!(rphys_world_step_and_hash(a.0, 1.0 / 120.0, &mut ha), RPHYS_OK);
            assert_eq!(rphys_world_step_and_hash(b.0, 1.0 / 120.0, &mut hb), RPHYS_OK);
            assert_eq!(ha.bytes, hb.bytes, "hash diverged during contact sim");
        }
    }
}
#[test]
fn removed_ids_stay_invalid_across_many_adds() {
    unsafe {
        let w = mk_world();

        let s = RPhysSphereBodyDesc {
            pose: mk_identity_iso(0.0, 1.0, 0.0),
            vel: mk_zero_vel(),
            radius: 0.25,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        let mut id0 = 0u32;
        assert_eq!(rphys_add_sphere_body(w.0, &s, &mut id0), RPHYS_OK);
        assert_eq!(rphys_body_remove(w.0, id0), RPHYS_OK);

        // add a bunch more
        for i in 0..128 {
            let mut id = 0u32;
            let s2 = RPhysSphereBodyDesc { pose: mk_identity_iso(i as f32, 1.0, 0.0), ..s };
            assert_eq!(rphys_add_sphere_body(w.0, &s2, &mut id), RPHYS_OK);
            assert_ne!(id, id0);
        }

        let mut pose_out = mk_identity_iso(0.0, 0.0, 0.0);
        assert_eq!(rphys_body_get_pose(w.0 as *const RPhysWorld, id0, &mut pose_out), RPHYS_ERR_INVALID_ID);
    }
}
#[test]
fn raycast_negative_max_distance_is_safe_and_misses() {
    unsafe {
        let w = mk_world();

        let q = RPhysRayQuery {
            origin: RPhysVec3 { x: 0.0, y: 5.0, z: 0.0 },
            dir:    RPhysVec3 { x: 0.0, y: -1.0, z: 0.0 },
            max_distance: -10.0,
            ignore_body: 0,
        };

        let mut hit = RPhysRayHit {
            hit: RPHYS_TRUE,
            body_id: 123,
            fraction: 0.0,
            point: RPhysVec3 { x: 9.0, y: 9.0, z: 9.0 },
            normal: RPhysVec3 { x: 9.0, y: 9.0, z: 9.0 },
        };

        let r = rphys_world_raycast(w.0 as *const RPhysWorld, &q, &mut hit);
        assert_eq!(r, RPHYS_OK, "err={}", last_err());
        assert_eq!(hit.hit, RPHYS_FALSE);
        assert_eq!(hit.body_id, 0);
        assert_eq!(hit.fraction, 1.0);
    }
}
#[test]
fn capsule_sweep_started_overlapping_sets_flag() {
    unsafe {
        let w = mk_world();

        let wall = RPhysBoxBodyDesc {
            pose: mk_identity_iso(0.0, 1.0, 0.0),
            vel: mk_zero_vel(),
            hx: 1.0,
            hy: 1.0,
            hz: 1.0,
            mass: 0.0,
            body_type: RPHYS_BODY_STATIC,
            material: mk_default_mat(),
            user_tag: 0,
        };

        let mut wall_id = 0u32;
        assert_eq!(rphys_add_box_body(w.0, &wall, &mut wall_id), RPHYS_OK, "err={}", last_err());

        // start *inside* / intersecting
        let q = RPhysCapsuleSweepQuery {
            from: RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 },
            to:   RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 },
            radius: 0.75,
            half_height: 0.5,
            ignore_body: 0,
        };

        let mut hit = RPhysCapsuleSweepHit {
            hit: RPHYS_FALSE,
            body_id: 0,
            fraction: 0.0,
            started_overlapping: RPHYS_FALSE,
            point: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 },
            normal: RPhysVec3 { x: 0.0, y: 1.0, z: 0.0 },
        };

        let r = rphys_world_capsule_sweep(w.0 as *const RPhysWorld, &q, &mut hit);
        assert_eq!(r, RPHYS_OK, "err={}", last_err());
        assert_eq!(hit.hit, RPHYS_TRUE);
        assert_eq!(hit.body_id, wall_id);
        assert_eq!(hit.started_overlapping, RPHYS_TRUE);
        assert!((0.0..=1.0).contains(&hit.fraction));
    }
}
#[test]
fn last_error_is_thread_local() {
    use std::thread;

    let t = thread::spawn(|| {
        unsafe {
            let w = mk_world();
            let q = RPhysRayQuery {
                origin: RPhysVec3 { x: 0.0, y: 5.0, z: 0.0 },
                dir:    RPhysVec3 { x: 0.0, y: -1.0, z: 0.0 },
                max_distance: 20.0,
                ignore_body: 999_999, // invalid => sets last error on this thread
            };
            let mut hit = RPhysRayHit { hit: 1, body_id: 123, fraction: 0.0, point: RPhysVec3{x:9.0,y:9.0,z:9.0}, normal: RPhysVec3{x:9.0,y:9.0,z:9.0} };
            let r = rphys_world_raycast(w.0 as *const RPhysWorld, &q, &mut hit);
            assert_eq!(r, RPHYS_ERR_INVALID_ID);
            last_err()
        }
    });

    let msg = t.join().unwrap();
    assert!(msg != "<null>", "worker thread should have an error message");

    // main thread should remain clean
    assert_eq!(last_err(), "<null>");
}
#[test]
fn abi_layout_sizes_are_stable() {
    use std::mem::{size_of, align_of};

    assert_eq!(size_of::<RPhysVec3>(), 12);
    assert_eq!(align_of::<RPhysVec3>(), 4);

    assert_eq!(size_of::<RPhysQuat>(), 16);
    assert_eq!(align_of::<RPhysQuat>(), 4);

    assert_eq!(size_of::<RPhysIsometry>(), 28); // 12 + 16
    assert_eq!(align_of::<RPhysIsometry>(), 4);

    assert_eq!(size_of::<RPhysVelocity>(), 24);
    assert_eq!(align_of::<RPhysVelocity>(), 4);

    assert_eq!(size_of::<RPhysHash32>(), RPHYS_STEP_HASH_BYTES);
}
#[test]
fn get_pose_invalid_id_writes_identity_on_failure() {
    unsafe {
        let w = mk_world();
        let mut out = mk_identity_iso(9.0, 9.0, 9.0);

        let r = rphys_body_get_pose(w.0 as *const RPhysWorld, 999_999, &mut out);
        assert_eq!(r, RPHYS_ERR_INVALID_ID);

        // if you choose “deterministic out on failure”:
        assert_eq!(out.pos.x, 0.0);
        assert_eq!(out.pos.y, 0.0);
        assert_eq!(out.pos.z, 0.0);
        assert_eq!(out.rot.w, 1.0);
    }
}
#[test]
fn world_create_clears_last_error() {
    unsafe {
        // Force an error first.
        let _ = rphys_world_step(core::ptr::null_mut(), 0.016);
        assert!(!rphys_last_error_message().is_null());

        // Successful create must clear it.
        let w = rphys_world_create(16, 16);
        assert!(!w.is_null());
        assert!(rphys_last_error_message().is_null());

        rphys_world_destroy(w);
    }
}
#[test]
fn body_ids_are_monotonic_even_after_remove() {
    unsafe {
        let w = rphys_world_create(16, 16);
        assert!(!w.is_null());

        let mut id1: u32 = 0;
        // build a minimal valid sphere desc...
        let d = RPhysSphereBodyDesc {
            pose: RPhysIsometry { pos: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 }, rot: RPhysQuat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } },
            vel: RPhysVelocity { lin: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 }, ang: RPhysVec3 { x: 0.0, y: 0.0, z: 0.0 } },
            radius: 0.5,
            mass: 1.0,
            body_type: RPHYS_BODY_DYNAMIC,
            material: RPhysMaterial { mu_static: 0.9, mu_dynamic: 0.7, restitution: 0.05 },
            user_tag: 0,
        };

        assert_eq!(rphys_add_sphere_body(w, &d, &mut id1), RPHYS_OK);
        assert_eq!(id1, 1);

        assert_eq!(rphys_body_remove(w, id1), RPHYS_OK);

        let mut id2: u32 = 0;
        let r = rphys_add_sphere_body(w, &d, &mut id2);

        // Under ABI v1, the second allocation must be a new id (2).
        assert_eq!(r, RPHYS_OK);
        assert_eq!(id2, 2);

        rphys_world_destroy(w);
    }
}
#[test]
fn last_error_copy_truncates_safely_and_reports_full_len() {
    unsafe {
        let _ = rphys_world_step(core::ptr::null_mut(), 0.016);

        let mut buf = [0i8; 8];
        let mut full_len: u32 = 0;

        assert_eq!(rphys_last_error_copy(buf.as_mut_ptr(), buf.len() as u32, &mut full_len), RPHYS_OK);
        assert!(full_len > 0);

        // must be NUL terminated
        assert_eq!(buf[buf.len() - 1], 0);
    }
}
/* ===================== PLAYER CONTROLLER ===================== */

#[test]
fn player_controller_basic_movement_and_gravity() {
    unsafe {
        let w = mk_world();

        let desc = RPhysPlayerDesc {
            start_pose: mk_identity_iso(0.0, 10.0, 0.0), // Start floating in the air
            radius: 0.4,
            height: 1.8,
            speed: 5.0,
        };

        let mut p_idx = u32::MAX;
        assert_eq!(rphys_add_player(w.0, &desc, &mut p_idx), RPHYS_OK, "err={}", last_err());
        assert_ne!(p_idx, u32::MAX);

        // Feed an input vector pushing the player along the +X axis
        let move_dir = RPhysVec3 { x: 1.0, y: 0.0, z: 0.0 };
        assert_eq!(rphys_player_set_input(w.0, p_idx, move_dir), RPHYS_OK, "err={}", last_err());

        // Step the world. The player should fall (gravity) AND move +X (input).
        for _ in 0..10 {
            assert_eq!(rphys_world_step(w.0, 1.0 / 60.0), RPHYS_OK, "err={}", last_err());
        }

        let mut pose_out = mk_identity_iso(0.0, 0.0, 0.0);
        let mut grounded = RPHYS_TRUE; // Preset to true so we can ensure it gets written to false

        assert_eq!(
            rphys_player_get_pose(w.0 as *const RPhysWorld, p_idx, &mut pose_out, &mut grounded),
            RPHYS_OK,
            "err={}",
            last_err()
        );

        // Verify gravity pulled them down
        assert!(pose_out.pos.y < 10.0, "Player did not fall. Y: {}", pose_out.pos.y);

        // Verify input moved them forward
        assert!(pose_out.pos.x > 0.0, "Player did not move forward. X: {}", pose_out.pos.x);

        // Verify they know they are in the air
        assert_eq!(grounded, RPHYS_FALSE, "Player should not be grounded in the air.");
    }
}

#[test]
fn player_controller_hits_heightfield_and_grounds() {
    unsafe {
        let w = mk_world();

        // 1. Create a flat ground at Y=0
        let heights: [i16; 4] = [0, 0, 0, 0];
        let d = RPhysHeightfieldDesc {
            size_bytes: std::mem::size_of::<RPhysHeightfieldDesc>() as u32,
            flags: 0,
            width: 2,
            height: 2,
            row_stride: 0,
            cell_size_x: 10.0,
            cell_size_z: 10.0,
            height_scale: 1.0,
            height_offset: 0.0,
            heights: heights.as_ptr(),
        };
        assert_eq!(rphys_world_set_heightfield_i16(w.0, &d, 0.0, 0.0, 0.0), RPHYS_OK, "err={}", last_err());

        // 2. Drop the player right above the mesh
        let desc = RPhysPlayerDesc {
            start_pose: mk_identity_iso(5.0, 3.0, 5.0),
            radius: 0.5,
            height: 2.0, // Extents: 1.0 half-height + 0.5 radius = 1.5 total collision distance to center
            speed: 5.0,
        };

        let mut p_idx = u32::MAX;
        assert_eq!(rphys_add_player(w.0, &desc, &mut p_idx), RPHYS_OK, "err={}", last_err());

        // 3. Step enough frames for gravity to slam the player into the heightfield
        for _ in 0..60 {
            assert_eq!(rphys_world_step(w.0, 1.0 / 60.0), RPHYS_OK, "err={}", last_err());
        }

        // 4. Retrieve pose and ground state
        let mut pose_out = mk_identity_iso(0.0, 0.0, 0.0);
        let mut grounded = RPHYS_FALSE;
        assert_eq!(
            rphys_player_get_pose(w.0 as *const RPhysWorld, p_idx, &mut pose_out, &mut grounded),
            RPHYS_OK,
            "err={}",
            last_err()
        );

        // The math: Y should settle right around 1.5 (half-height 1.0 + radius 0.5)
        assert_eq!(grounded, RPHYS_TRUE, "Player failed to ground on the heightfield!");
        assert!(pose_out.pos.y > 1.4 && pose_out.pos.y < 1.6, "Player sank into or hovered over ground. Y: {}", pose_out.pos.y);
    }
}
/* ===================== ROLLBACK (AT_REST_SPEC D2) ===================== */

/// A static ground, a 3-box stack and a box dropped beside it. Returns the world and the id of
/// the stack's top box.
unsafe fn rollback_scene() -> (W, u32) {
    let w = mk_world();
    let ground = RPhysBoxBodyDesc {
        pose: mk_identity_iso(0.0, -0.5, 0.0), vel: mk_zero_vel(), hx: 50.0, hy: 0.5, hz: 50.0,
        mass: 0.0, body_type: RPHYS_BODY_STATIC, material: mk_default_mat(), user_tag: 0,
    };
    let mut id: u32 = 0;
    assert_eq!(rphys_add_box_body(w.0, &ground, &mut id), RPHYS_OK, "err={}", last_err());
    for i in 0..3 {
        let b = RPhysBoxBodyDesc {
            pose: mk_identity_iso(0.0, 0.1 + 0.205 * i as f32, 0.0), vel: mk_zero_vel(), hx: 0.2, hy: 0.1, hz: 0.2,
            mass: 5.0, body_type: RPHYS_BODY_DYNAMIC, material: mk_default_mat(), user_tag: 0,
        };
        assert_eq!(rphys_add_box_body(w.0, &b, &mut id), RPHYS_OK, "err={}", last_err());
    }
    let top = id;
    let drop = RPhysBoxBodyDesc {
        pose: mk_identity_iso(3.0, 4.0, 0.0), vel: mk_zero_vel(), hx: 0.2, hy: 0.2, hz: 0.2,
        mass: 2.0, body_type: RPHYS_BODY_DYNAMIC, material: mk_default_mat(), user_tag: 0,
    };
    assert_eq!(rphys_add_box_body(w.0, &drop, &mut id), RPHYS_OK, "err={}", last_err());
    (w, top)
}

unsafe fn hashes(w: &W, n: usize) -> Vec<[u8; RPHYS_STEP_HASH_BYTES]> {
    (0..n).map(|_| {
        let mut h = RPhysHash32 { bytes: [0u8; RPHYS_STEP_HASH_BYTES] };
        assert_eq!(rphys_world_step_and_hash(w.0, 1.0 / 60.0, &mut h), RPHYS_OK, "err={}", last_err());
        h.bytes
    }).collect()
}

/// Snapshot after `settle` steps, nudge the top box, step 90, restore, nudge it the same way,
/// step 90 again: every hash must match. The snapshot carries the warm-start cache, last
/// velocities and sleep state; without them the first step after the restore starts every
/// contact cold. The nudge wakes a sleeping stack, so its carried entries are used.
unsafe fn rollback_replays(settle: usize, expect_asleep: bool) {
    let (w, top) = rollback_scene();
    let nudge = || assert_eq!(rphys_body_apply_impulse(w.0, top, RPhysVec3 { x: 0.3, y: 0.0, z: 0.0 }), RPHYS_OK, "err={}", last_err());
    hashes(&w, settle);
    {
        let st = w.0.as_ref().unwrap().state.lock().unwrap();
        assert!(st.world.warm_contacts() > 0, "nothing in the warm-start cache to carry");
        assert_eq!(st.world.num_sleeping() > 0, expect_asleep, "{} bodies asleep", st.world.num_sleeping());
    }
    let snap = rphys_world_snapshot_create(w.0 as *const RPhysWorld);
    assert!(!snap.is_null(), "err={}", last_err());
    nudge();
    let first = hashes(&w, 90);
    assert_eq!(rphys_world_snapshot_restore(w.0, snap), RPHYS_OK, "err={}", last_err());
    nudge();
    let again = hashes(&w, 90);
    rphys_world_snapshot_free(snap);
    for (k, (a, b)) in first.iter().zip(&again).enumerate() {
        assert_eq!(a, b, "step {} after the restore differs from the original run", k + 1);
    }
}

#[test]
fn rollback_replays_a_warm_resting_stack() {
    // 15 steps: the stack is in contact (warm), the dropped box still falling, nothing asleep
    unsafe { rollback_replays(15, false) }
}

#[test]
fn rollback_replays_with_part_of_the_scene_asleep() {
    // 400 steps: the stack has fallen asleep
    unsafe { rollback_replays(400, true) }
}
