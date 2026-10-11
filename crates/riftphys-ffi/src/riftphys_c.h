// File: riftphys_c.h
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== ABI VERSION ===================== */

#define RPHYS_ABI_VERSION 1u
uint32_t    rphys_get_abi_version(void);
const char* rphys_get_build_id(void);

/* ===================== ABI TYPES ===================== */

typedef uint32_t RPhysBool;
#define RPHYS_FALSE 0u
#define RPHYS_TRUE  1u

typedef uint32_t RPhysResult;
#define RPHYS_OK             0u
#define RPHYS_ERR_NULL       1u
#define RPHYS_ERR_INVALID_ID 2u
#define RPHYS_ERR_CAPACITY   3u
#define RPHYS_ERR_BAD_ARG    4u
#define RPHYS_ERR_INTERNAL   0x80000000u

// 0 is reserved invalid / none.
#define RPHYS_INVALID_BODY_ID 0u

// Reserved hit object id for terrain in query results.
// This is NOT a body id and must be rejected by body APIs.
#define RPHYS_TERRAIN_BODY_ID 0xFFFFFFFFu

// Optional: TLS error message (valid until next API call that sets it on the same thread)
const char* rphys_last_error_message(void);

/* ===================== OPAQUE HANDLE ===================== */

typedef struct RPhysWorld RPhysWorld;

/* ===================== POD TYPES ===================== */

typedef struct RPhysVec3 { float x, y, z; } RPhysVec3;
typedef struct RPhysQuat { float x, y, z, w; } RPhysQuat;

typedef struct RPhysIsometry {
    RPhysVec3 pos;
    RPhysQuat rot;
} RPhysIsometry;

typedef struct RPhysVelocity {
    RPhysVec3 lin;
    RPhysVec3 ang;
} RPhysVelocity;

typedef struct RPhysMaterial {
    float mu_static;
    float mu_dynamic;
    float restitution;
} RPhysMaterial;

// Keep this as u32 to avoid C enum-size ambiguity.
typedef uint32_t RPhysBodyType;
#define RPHYS_BODY_STATIC  0u
#define RPHYS_BODY_DYNAMIC 1u

typedef struct RPhysCapsuleBodyDesc {
    RPhysIsometry pose;
    RPhysVelocity vel;
    float radius;
    float half_height;
    float mass;            // ABI kept; may be ignored by impl
    RPhysBodyType body_type;
    RPhysMaterial material;
    uint32_t user_tag;     // ABI kept; may be unused by impl
} RPhysCapsuleBodyDesc;

typedef struct RPhysBoxBodyDesc {
    RPhysIsometry pose;
    RPhysVelocity vel;
    float hx, hy, hz;
    float mass;            // ABI kept; may be ignored by impl
    RPhysBodyType body_type;
    RPhysMaterial material;
    uint32_t user_tag;
} RPhysBoxBodyDesc;

typedef struct RPhysSphereBodyDesc {
    RPhysIsometry pose;
    RPhysVelocity vel;
    float radius;
    float mass;            // ABI kept; may be ignored by impl
    RPhysBodyType body_type;
    RPhysMaterial material;
    uint32_t user_tag;
} RPhysSphereBodyDesc;

/* ===================== HASH OUTPUT ===================== */

#define RPHYS_STEP_HASH_BYTES 32

typedef struct RPhysHash32 {
    uint8_t bytes[RPHYS_STEP_HASH_BYTES];
} RPhysHash32;

/* ===================== STEPPING PARAMS ===================== */

typedef struct RPhysStepParams {
    uint32_t size_bytes;   // must be set by caller = sizeof(RPhysStepParams)
    uint32_t flags;        // reserved
    uint32_t substeps;     // must be > 0
    uint32_t solver_iters; // 0 = engine default (may be ignored by impl)
    float    dt;           // must be finite (and ideally >= 0)
} RPhysStepParams;

/* ===================== QUERIES ===================== */

typedef struct RPhysRayQuery {
    RPhysVec3 origin;
    RPhysVec3 dir;          // does not need to be normalized; zero is allowed (miss)
    float max_distance;     // impl may clamp <0 to 0
    uint32_t ignore_body;   // 1-based; 0 = none; terrain sentinel is invalid here
} RPhysRayQuery;

typedef struct RPhysRayHit {
    RPhysBool hit;
    uint32_t  body_id;      // 1-based; 0 = none; may be RPHYS_TERRAIN_BODY_ID for terrain
    float     fraction;
    RPhysVec3 point;
    RPhysVec3 normal;
} RPhysRayHit;

typedef struct RPhysCapsuleSweepQuery {
    RPhysVec3 from;
    RPhysVec3 to;
    float radius;
    float half_height;
    uint32_t ignore_body;   // 1-based; 0 = none; terrain sentinel is invalid here
} RPhysCapsuleSweepQuery;

typedef struct RPhysCapsuleSweepHit {
    RPhysBool hit;
    uint32_t  body_id;      // 1-based; 0 = none; may be RPHYS_TERRAIN_BODY_ID for terrain
    float     fraction;
    RPhysBool started_overlapping;
    RPhysVec3 point;
    RPhysVec3 normal;
} RPhysCapsuleSweepHit;

/* ===================== HEIGHTFIELD ===================== */

typedef struct RPhysHeightfieldDesc {
    uint32_t size_bytes;      // must be set by caller = sizeof(RPhysHeightfieldDesc)
    uint32_t flags;           // reserved (0)
    uint32_t width;           // samples in X
    uint32_t height;          // samples in Z
    uint32_t row_stride;      // 0 = width
    float    cell_size_x;     // world units between samples (X)
    float    cell_size_z;     // world units between samples (Z)
    float    height_scale;    // world units per i16 unit
    float    height_offset;   // world units bias
    const int16_t* heights;   // row-major, length >= row_stride*height
} RPhysHeightfieldDesc;

RPhysResult rphys_world_set_heightfield_i16(
    RPhysWorld* world,
    const RPhysHeightfieldDesc* hf,
    float origin_x, float origin_z,
    float y_offset);

RPhysResult rphys_world_set_heightfield_raw32_square(
    RPhysWorld* world,
    const uint8_t* bytes,
    uint32_t bytes_len,
    float world_size_x,
    float world_size_z,
    float origin_x,
    float origin_z,
    float y_offset);

RPhysResult rphys_world_clear_heightfield(RPhysWorld* world);

/* ===================== API ===================== */

// lifecycle
RPhysWorld* rphys_world_create(uint32_t max_bodies, uint32_t max_colliders);
void         rphys_world_destroy(RPhysWorld* world);
RPhysResult  rphys_world_set_epoch(RPhysWorld* world, uint64_t epoch);
RPhysResult  rphys_world_set_rng_seed(RPhysWorld* world, uint64_t seed);

// bodies (NOTE: out_body_id receives 1-based id; 0 on failure)
RPhysResult  rphys_add_capsule_body(RPhysWorld* world, const RPhysCapsuleBodyDesc* desc, uint32_t* out_body_id);
RPhysResult  rphys_add_box_body    (RPhysWorld* world, const RPhysBoxBodyDesc* desc, uint32_t* out_body_id);
RPhysResult  rphys_add_sphere_body (RPhysWorld* world, const RPhysSphereBodyDesc* desc, uint32_t* out_body_id);
RPhysResult  rphys_body_remove     (RPhysWorld* world, uint32_t body_id);

// state
RPhysResult  rphys_body_get_pose      (const RPhysWorld* world, uint32_t body_id, RPhysIsometry* out_pose);
RPhysResult  rphys_body_set_pose      (RPhysWorld* world, uint32_t body_id, const RPhysIsometry* pose);
RPhysResult  rphys_body_get_velocity  (const RPhysWorld* world, uint32_t body_id, RPhysVelocity* out_vel);
RPhysResult  rphys_body_set_velocity  (RPhysWorld* world, uint32_t body_id, const RPhysVelocity* vel);
RPhysResult  rphys_body_apply_impulse (RPhysWorld* world, uint32_t body_id, RPhysVec3 impulse);

// stepping
RPhysResult  rphys_world_step_ex(RPhysWorld* world, const RPhysStepParams* p);
RPhysResult  rphys_world_step   (RPhysWorld* world, float dt);

// determinism / diagnostics
RPhysResult  rphys_world_tick_index   (const RPhysWorld* world, uint64_t* out_tick);
RPhysResult  rphys_world_num_bodies   (const RPhysWorld* world, uint32_t* out_num);
RPhysResult  rphys_world_get_step_hash(const RPhysWorld* world, RPhysHash32* out_hash);
RPhysResult  rphys_world_step_and_hash(RPhysWorld* world, float dt, RPhysHash32* out_hash);

// queries (always write a deterministic miss to out_* when possible)
RPhysResult  rphys_world_raycast(const RPhysWorld* world, const RPhysRayQuery* query, RPhysRayHit* out_hit);
RPhysResult  rphys_world_capsule_sweep(const RPhysWorld* world, const RPhysCapsuleSweepQuery* query, RPhysCapsuleSweepHit* out_hit);

// Last Error Copy
RPhysResult rphys_last_error_copy(char* dst, uint32_t cap, uint32_t* out_len);

/* ===================== KINEMATIC PLAYER CONTROLLER ===================== */

typedef struct RPhysPlayerDesc {
    RPhysIsometry start_pose;
    float radius;
    float height;
    float speed;
} RPhysPlayerDesc;

// Returns a 0-based player_idx (not a standard body_id)
RPhysResult rphys_add_player(RPhysWorld* world, const RPhysPlayerDesc* desc, uint32_t* out_player_idx);

// Feeds WASD input direction to the player for the next frame
RPhysResult rphys_player_set_input(RPhysWorld* world, uint32_t player_idx, RPhysVec3 move_dir);

// Retrieves the resulting pose after physics step, and whether the player is touching the ground
RPhysResult rphys_player_get_pose(const RPhysWorld* world, uint32_t player_idx, RPhysIsometry* out_pose, RPhysBool* out_grounded);

// Retrieves contact events for the current tick.
// If `out_buffer` is NULL, it only returns the number of contacts in `out_count`.
// If `out_buffer` is provided, it copies up to `buffer_capacity` events into it.
RPhysResult rphys_world_get_contacts(
    const RPhysWorld* world,
    RPhysContactEvent* out_buffer,
    uint32_t buffer_capacity,
    uint32_t* out_count
);
/* ===================== ROLLBACK / SNAPSHOTS ===================== */

typedef struct RPhysWorldSnapshot RPhysWorldSnapshot;

RPhysWorldSnapshot* rphys_world_snapshot_create(const RPhysWorld* world);
RPhysResult         rphys_world_snapshot_restore(RPhysWorld* world, const RPhysWorldSnapshot* snapshot);
void                rphys_world_snapshot_free(RPhysWorldSnapshot* snapshot);

#ifdef __cplusplus
}
#endif