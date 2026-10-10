//! RiftPhys <-> URIEL: deterministic robotics scenarios that emit URIEL telemetry.
//!
//! - `protocol`: the line format (shared shape with the RiftForged server telemetry contract).
//! - `drone`: quadrotor plant, rotor mixer, expert PD controller, squad formations.
//! - `rollout`: episode runner (seeded, bit-exact replays, per-sample world hash).

pub mod drone;
pub mod protocol;
pub mod rng;
pub mod rollout;
