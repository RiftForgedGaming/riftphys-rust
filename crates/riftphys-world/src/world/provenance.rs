use super::world::World;

use riftphys_core::{hash_quat, hash_vec3, StepHasher};

impl World {
    pub fn step_hash(&self) -> [u8; 32] {
        let mut h = StepHasher::new();
        h.update_bytes(&self.epoch_id.to_le_bytes());
        h.update_bytes(&self.rng.state().to_le_bytes());
        h.update_bytes(&self.schedule.digest());
        for i in self.bodies.indices() {
            let pose = self.bodies.pose(i);
            let vel  = self.bodies.vel(i);
            h.update_bytes(&i.to_le_bytes());
            hash_vec3(&mut h, &pose.pos);
            hash_quat(&mut h, &pose.rot);
            hash_vec3(&mut h, &vel.lin);
            hash_vec3(&mut h, &vel.ang);
        }
        h.finalize()
    }

    pub(super) fn provenance_sums(&self) -> (f32, u32, f32, f32) {
        (self.prov_impulses_sum, self.prov_ccd_hits, self.prov_aero_sum, self.prov_prop_sum)
    }
}
