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
//! - [`quadrotor`]: vehicles replayed from measured rotor speeds (Blackbird-style logs).
//! - [`cube_toss`]: real cube tosses (ContactNets) scored as Drake, MuJoCo and Bullet were
//!   (`riftphys-cube-toss`).
//! - [`double_pendulum`]: a real double pendulum (Myers et al.), PARITY_SPEC W3
//!   (`riftphys-double-pendulum`).
//! - [`cassie`]: Cassie jumping impacts (PARITY_SPEC W4): the data, the model, the pre-fit checks.
//! - [`urdf`]: a minimal URDF loader that builds bodies and hinges in a World (PARITY_SPEC W4a).
//!
//! See docs/PHYSICS_FIDELITY.md.

pub mod analytic;
pub mod cassie;
pub mod cmaes;
pub mod cube_toss;
pub mod double_pendulum;
pub mod fit;
pub mod log;
pub mod quadrotor;
pub mod scene;
pub mod synth;
pub mod urdf;
