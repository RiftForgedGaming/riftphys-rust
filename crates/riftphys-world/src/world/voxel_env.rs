use riftphys_collision::environment::{EnvCollider, SpatialEnvironment};
use riftphys_core::{Isometry, Quat, Vec3};
use riftphys_geom::{Aabb, Shape};
use riftphys_vox::VoxelChunk;

pub struct VoxelEnvironment {
    pub chunk: VoxelChunk,
    pub origin: Vec3, // The world-space origin of the chunk's (0,0,0) coordinate
}

impl VoxelEnvironment {
    pub fn new(chunk: VoxelChunk, origin: Vec3) -> Self {
        Self { chunk, origin }
    }
}

impl SpatialEnvironment for VoxelEnvironment {
    fn query_aabb(&self, aabb: &Aabb, buffer: &mut Vec<EnvCollider>) {
        let s = self.chunk.voxel_size;
        let half_s = s * 0.5;

        // 1. Shift query AABB into the chunk's local coordinate space
        let local_min = aabb.min - self.origin;
        let local_max = aabb.max - self.origin;

        // 2. Map continuous bounds to discrete grid indices, clamping to chunk dimensions.
        let start_x = (local_min.x / s).floor().max(0.0) as u32;
        let start_y = (local_min.y / s).floor().max(0.0) as u32;
        let start_z = (local_min.z / s).floor().max(0.0) as u32;

        let end_x = ((local_max.x / s).ceil() as u32).min(self.chunk.dims.x.saturating_sub(1));
        let end_y = ((local_max.y / s).ceil() as u32).min(self.chunk.dims.y.saturating_sub(1));
        let end_z = ((local_max.z / s).ceil() as u32).min(self.chunk.dims.z.saturating_sub(1));

        // Short-circuit if completely outside bounds
        if start_x > end_x || start_y > end_y || start_z > end_z {
            return;
        }

        // 3. Extract the occupied voxels strictly within the intersection area
        for z in start_z..=end_z {
            for y in start_y..=end_y {
                for x in start_x..=end_x {
                    if self.chunk.get(x, y, z) {
                        let center_local = Vec3::new(
                            x as f32 * s + half_s,
                            y as f32 * s + half_s,
                            z as f32 * s + half_s,
                        );

                        buffer.push(EnvCollider {
                            shape: Shape::Box {
                                hx: half_s,
                                hy: half_s,
                                hz: half_s,
                            },
                            transform: Isometry {
                                pos: self.origin + center_local,
                                rot: Quat::IDENTITY,
                            },
                            material: self.chunk.mat(x, y, z),
                            user_data: 0, // <-- The missing field
                        });
                    }
                }
            }
        }
    }
}