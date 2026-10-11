//! RiftPhys <-> URIEL: deterministic robotics scenarios that emit URIEL telemetry.
//!
//! - `protocol`: the line format (shared shape with the RiftForged server telemetry contract).
//! - `drone`: quadrotor plant, rotor mixer, expert PD controller, squad formations.
//! - `biped`: humanoid lower body with ankles and feet, joint-torque actuation, PD stand expert.
//! - `rollout`: episode runner (seeded, bit-exact replays, per-sample world hash).
//! - `features`: learner inputs shared by data export and live control.

pub mod biped;
pub mod drone;
pub mod features;
pub mod protocol;
pub mod rng;
pub mod rollout;
