//! Physics fidelity for RiftPhys: how close the engine is to reality, and tools to close the gap.
//!
//! - [`analytic`]: closed-form benchmarks (free fall, pendulums, friction, restitution, rolling,
//!   torque-free rotation, collisions, resting contact), each with a stated tolerance. No
//!   hardware needed; `riftphys-fidelity` prints the report.
//! - [`log`]: reality logs (mocap poses + commands) in RiftPhys conventions.
//! - [`scene`]: rebuild a captured scene, replay windows of a log, measure position error.
//! - [`fit`] + [`cmaes`]: fit unknown parameters (restitution, friction, drag, mass) on training
//!   takes, report held-out error before and after (`riftphys-fit`).
//! - [`synth`]: synthetic logs from known parameters, to check the pipeline (`riftphys-synth`).
//!
//! See docs/PHYSICS_FIDELITY.md.

pub mod analytic;
pub mod cmaes;
pub mod fit;
pub mod log;
pub mod scene;
pub mod synth;
