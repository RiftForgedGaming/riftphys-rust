// riftphys-geom/src/lib.rs
#![forbid(unsafe_code)]

// If this ever trips, you enabled the determinism killer somewhere in the workspace.
// Keep this guard here AND in core once you lock workspace deps.
#[cfg(feature = "glam-fast-math")]
compile_error!("glam fast-math enabled. This breaks determinism. Turn it off.");


pub mod aabb;
pub mod shape;
pub mod mass;
pub mod triangle;
pub mod ray;
pub mod plane;
pub mod raycast;

pub use aabb::*;
pub use mass::*;
pub use plane::*;
pub use ray::*;
pub use shape::*;
pub use triangle::*;
pub use raycast::ray_shape;
