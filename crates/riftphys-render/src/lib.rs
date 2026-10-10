pub mod palette;
pub mod camera;
pub mod core_types;
pub mod mesh_gen;
pub mod render;

// Re-export the primary tools for the visualizer client
pub use camera::Camera;
pub use core_types::{Instance, ShapeKind, trs};
pub use render::Renderer;