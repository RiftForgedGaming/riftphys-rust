# RiftPhys C FFI / C-ABI surface

This document sketches a C-friendly ABI for embedding the Rust RiftPhys core inside the existing RiftForged/Bullet host shown in the prompt. It mirrors the data-oriented Rust API in `riftphys-core`, `riftphys-geom`, and `riftphys-world`, but uses opaque handles and POD structs so that a C or C++ host can drive the simulation without depending on Rust symbols or layout quirks.

## Goals
- Keep the public surface purely C (no name mangling, no Rust-specific types) while mapping 1:1 to the Rust types used by `World`.
- Preserve determinism expectations: fixed time step, explicit gravity/epoch swaps, and stable IDs.
- Allow a host to build worlds, add bodies/colliders, set velocities/poses, and step the simulation, mirroring the Bullet adapter methods in the reference C++ layer (e.g., `create_body`, `set_velocity`, `set_pose_kinematic`, `step`).

## Core value types
These mirror the Rust engine's math and identifiers so that the host can construct compatible shapes and poses before calling into Rust:

```c
typedef struct RPhysVec3 { float x, y, z; } RPhysVec3;             // riftphys_core::types::Vec3
typedef struct RPhysQuat { float x, y, z, w; } RPhysQuat;          // glam::Quat wrapper

typedef struct RPhysIsometry {                                    // riftphys_core::types::Isometry
    RPhysVec3 pos;
    RPhysQuat rot;
} RPhysIsometry;

typedef struct RPhysVelocity {                                    // riftphys_core::types::Velocity
    RPhysVec3 lin;
    RPhysVec3 ang;
} RPhysVelocity;

typedef struct RPhysMaterial {                                    // riftphys_geom::mass::Material
    float density;
    float mu_static;
    float mu_dynamic;
    float restitution;
    uint32_t material_id;  // engine-specific lookup (mirrors riftphys_materials::MaterialId)
} RPhysMaterial;

typedef enum RPhysShapeKind {                                     // riftphys_geom::shape::Shape
    RPHYS_SHAPE_SPHERE,
    RPHYS_SHAPE_BOX,
    RPHYS_SHAPE_CAPSULE
} RPhysShapeKind;

typedef struct RPhysShape {
    RPhysShapeKind kind;
    union {
        struct { float radius; } sphere;
        struct { float hx, hy, hz; } box;      // half-extents
        struct { float radius, half_height; } capsule; // Y-up half-height like Shape::Capsule { r, hh }
    } data;
} RPhysShape;
```

Rust-side IDs (e.g., `BodyId`, `ColliderId`) are simple `u32` wrappers, so the C ABI exposes them as plain integers to avoid layout mismatches.

## World/handle types
Opaque pointers keep host code decoupled from Rust layouts:

```c
typedef struct RPhysWorld RPhysWorld; // opaque; allocated/freed by the FFI

typedef struct {
    uint32_t body_id;    // maps to riftphys_core::BodyId
} RPhysBodyHandle;

typedef struct {
    uint32_t collider_id; // maps to riftphys_core::ColliderId
} RPhysColliderHandle;
```

## World construction and teardown
The FFI mirrors `World::with_capacity` and `WorldBuilder` defaults to pre-allocate SoA storage, preventing reallocations that could disturb determinism:

```c
// Create a world with explicit body/collider capacities (defaults match WorldBuilder::new()).
RPhysWorld* rphys_world_create(uint32_t body_capacity, uint32_t collider_capacity);

// Destroy the world and free all owned memory. Safe to pass NULL.
void rphys_world_destroy(RPhysWorld* world);
```

## Composition: bodies and colliders
The Rust world separates a body's inertial state from its colliders. The C ABI follows the same split so callers can create dynamic or static bodies and attach shapes/materials explicitly:

```c
typedef struct {
    RPhysIsometry pose;
    RPhysVelocity vel;
    float mass;            // if dynamic; ignored for static/kinematic
    bool dynamic;          // maps to BodyDesc::dynamic
} RPhysBodyDesc;

// Adds a body to the world; dynamic bodies get inv_mass from mass/inertia, statics set inv_mass=0.
RPhysBodyHandle rphys_body_add(RPhysWorld* world, const RPhysBodyDesc* desc);

// Attach a collider/shape to an existing body with the provided material.
RPhysColliderHandle rphys_collider_add(RPhysWorld* world, RPhysBodyHandle body, const RPhysShape* shape, const RPhysMaterial* mat);
```

### Optional mass helpers
If the host wants Rust to compute inertial tensors identical to the engine's implementations (`MassProps::from_sphere/box/capsule`), the FFI can expose helper constructors that wrap those routines and feed `mass` back into `RPhysBodyDesc`:

```c
float rphys_mass_sphere(float radius, float density);
float rphys_mass_box(RPhysVec3 half_extents, float density);
float rphys_mass_capsule(float radius, float half_height, float density);
```

## Simulation control
The stepper must match the fixed time-step determinism of `World::step` while respecting pending epoch/gravity swaps:

```c
// Apply a gravity swap at the next tick boundary (wraps World::queue_gravity_swap).
typedef struct {
    RPhysVec3 g;   // uniform gravity vector
} RPhysGravityUniform;
void rphys_world_set_gravity_uniform(RPhysWorld* world, RPhysGravityUniform g);

// Advance one fixed step; returns aggregated step stats if desired.
typedef struct {
    uint32_t num_bodies;
    uint32_t num_contacts;
    uint32_t tick_index;
} RPhysStepStats;
void rphys_world_step(RPhysWorld* world, float dt, RPhysStepStats* out_stats);
```

## State accessors and kinematic control
These align with the Bullet shim’s `get_pose`, `set_pose_kinematic`, and `set_velocity` entry points:

```c
bool rphys_body_get_pose(const RPhysWorld* world, RPhysBodyHandle body, RPhysIsometry* out_pose);
void rphys_body_set_pose(const RPhysWorld* world, RPhysBodyHandle body, const RPhysIsometry* pose); // for kinematic bodies
void rphys_body_set_velocity(RPhysWorld* world, RPhysBodyHandle body, const RPhysVelocity* vel);
```

## Simple queries
The Rust world exposes deterministic helper queries (e.g., `sweep_blade_against_boxes`) and terrain sampling. A minimal C surface can include the generic collider/terrain helpers so the host can author controller-style sweeps similar to the Bullet shim:

```c
typedef struct {
    RPhysVec3 tip_p0;
    RPhysVec3 tip_v;
    float tip_radius;
    RPhysVec3 mid_p0;
    RPhysVec3 mid_v;
    float mid_radius;
    float dt;
} RPhysBladeSweep;

typedef struct {
    bool hit;
    uint32_t collider_index;
    float toi;
    RPhysVec3 normal;
} RPhysBladeHit;

bool rphys_sweep_blade(const RPhysWorld* world, const RPhysBladeSweep* sweep, RPhysBladeHit* out_hit);

bool rphys_sample_heightfield(RPhysWorld* world, float world_x, float world_z, float* out_height, RPhysVec3* out_normal);
```

## Error handling and safety
- All functions return explicit success/failure booleans where a precondition could be violated (e.g., invalid handles).
- Ownership: the Rust side owns all allocations; the host never frees handles directly.
- Threading: match Rust’s single-threaded determinism assumptions unless a future MT backend is exposed behind a compile-time feature.

## Why this maps cleanly to RiftPhys internals
- The value types and shapes mirror `riftphys-core` and `riftphys-geom` (`Vec3`, `Isometry`, `Velocity`, `Shape`, `Material`), ensuring layout-compatible PODs without Rust-specific features.【F:crates/riftphys-core/src/types.rs†L1-L19】【F:crates/riftphys-core/src/ids.rs†L1-L13】【F:crates/riftphys-geom/src/shape.rs†L5-L29】【F:crates/riftphys-geom/src/mass.rs†L4-L52】
- World composition and stepping functions (`add_body`, `add_collider`, `queue_gravity_swap`, `step`) are directly reflected in the C API surface so the host can drive the same lifecycle as the Rust engine expects.【F:crates/riftphys-world/src/lib.rs†L320-L357】【F:crates/riftphys-world/src/lib.rs†L513-L599】
- The optional helper queries mirror existing deterministic utilities like terrain sampling and blade sweeps, keeping host-visible behavior consistent with the Rust implementation.【F:crates/riftphys-world/src/lib.rs†L359-L400】
