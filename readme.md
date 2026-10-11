# RiftPhys v2.0

**RiftPhys** is a high-performance, strictly deterministic 3D physics engine written in Rust, featuring a robust C/C++ Foreign Function Interface (FFI). It is designed from the ground up for authoritative server architectures and rollback networking.

Version 2.0 marks the transition from the experimental V1.0 (MIT) to a proprietary, production-ready architecture powering the RiftForged MMO ARPG.

## Key Features

* **Strict Determinism:** Guaranteed cross-platform deterministic execution. Hashes computed at the end of every tick will match bit-for-bit across all clients and the server, enabling reliable client-side prediction and server reconciliation.
* **O(1) Heightfield Terrain:** Highly optimized spatial grid lookups for heightfield terrain. V2.0 introduces robust local-to-world origin transformations, allowing for off-center map boundaries and infinite negative/positive axis traversal without OOB memory panics.
* **Snapshot & Rollback:** Zero-allocation state saving and restoration. The engine can instantly capture the exact state of all rigid bodies, colliders, and kinematic controllers, and restore them for lag compensation and rollback.
* **Kinematic Player Controllers:** Built-in player controllers with collision geometry (capsules), continuous collision detection, and slope-handling parameters, exposed directly to the FFI.
* **Bulletproof FFI Boundary:** Strict ABI contract utilizing 1-based indexing to prevent zero-index null pointer errors, with explicit terrain sentinel handling.


## Architecture Guidelines

RiftPhys operates on a "build architecture before building code" philosophy. When interfacing with the engine via C/C++:

1. **Memory Ownership:** The Rust FFI owns the FFI memory. Host applications must handle structural layouts accurately. Never pass dangling pointers (e.g., stack-allocated `std::vector` data) into persistent functions like `rphys_world_set_heightfield`.
2. **Zero-Initialization:** Always zero-initialize structs passed to the FFI (e.g., `RPhysStepParams params{};`). Uninitialized stack memory containing garbage data will instantly break determinism.
3. **Deterministic Inputs:** Input forces and player intents must be applied at the exact same tick index on both the client and the authoritative server.

## Building and Linking (Windows/MSVC)

When integrating the RiftPhys static library (`riftphys.lib`) into a C++ Windows environment, you must link the underlying Rust dependencies. Add the following pragmas to your host application:

```cpp
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "userenv.lib")
#pragma comment(lib, "bcrypt.lib")
#pragma comment(lib, "advapi32.lib")
#pragma comment(lib, "ntdll.lib")

