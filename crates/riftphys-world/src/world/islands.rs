// SPDX-License-Identifier: RiftForged Proprietary License
use riftphys_articulation::bodies::Joints;
use riftphys_dynamics::dynamics::Bodies;
use super::world::{Contact, Collider, TERRAIN_CI};

pub(super) struct Island {
    pub(super) contact_indices: Vec<usize>,
    pub(super) joint_indices: Vec<usize>,
    pub(super) min_body_id: u32,
}

pub(super) struct IslandManager {
    parent: Vec<u32>,
    rank: Vec<u32>,
    island_lookup: Vec<u32>,
}

impl IslandManager {
    pub(super) fn new(capacity: usize) -> Self {
        Self {
            parent: Vec::with_capacity(capacity),
            rank: Vec::with_capacity(capacity),
            island_lookup: Vec::with_capacity(capacity),
        }
    }

    pub(super) fn build_islands(
        &mut self,
        num_bodies: usize,
        colliders: &[Collider],
        bodies: &Bodies,
        joints: &Joints,
        contacts: &[Contact]
    ) -> Vec<Island> {
        self.parent.clear();
        self.rank.clear();
        self.island_lookup.clear();

        for i in 0..num_bodies {
            self.parent.push(i as u32);
            self.rank.push(0);
            self.island_lookup.push(u32::MAX);
        }

        let is_dynamic = |id: u32| -> bool { bodies.inv_mass_of(id) > 0.0 };

        // --- 1. UNION PHASE ---
        for joint_kind in joints.kinds().iter() {
            let (id_a, id_b) = match joint_kind {
                riftphys_articulation::bodies::JointKind::Distance(j) => (j.a.0, j.b.0),
                riftphys_articulation::bodies::JointKind::D6(j) => (j.a.0, j.b.0),
            };
            if is_dynamic(id_a) && is_dynamic(id_b) {
                self.union(id_a, id_b);
            }
        }

        for c in contacts {
            let id_a = colliders[c.a_collider].body.0;
            if c.b_collider != TERRAIN_CI {
                let id_b = colliders[c.b_collider].body.0;
                if is_dynamic(id_a) && is_dynamic(id_b) {
                    self.union(id_a, id_b);
                }
            }
        }

        // --- 2. COLLECTION PHASE ---
        let mut islands: Vec<Island> = Vec::with_capacity(num_bodies / 4);

        for (idx, joint_kind) in joints.kinds().iter().enumerate() {
            let (id_a, id_b) = match joint_kind {
                riftphys_articulation::bodies::JointKind::Distance(j) => (j.a.0, j.b.0),
                riftphys_articulation::bodies::JointKind::D6(j) => (j.a.0, j.b.0),
            };
            self.route_to_island(id_a, id_b, is_dynamic(id_a), is_dynamic(id_b), true, idx, &mut islands);
        }

        for (idx, c) in contacts.iter().enumerate() {
            let id_a = colliders[c.a_collider].body.0;
            let mut id_b = 0;
            let mut dyn_b = false;

            if c.b_collider != TERRAIN_CI {
                id_b = colliders[c.b_collider].body.0;
                dyn_b = is_dynamic(id_b);
            }
            self.route_to_island(id_a, id_b, is_dynamic(id_a), dyn_b, false, idx, &mut islands);
        }

        islands.sort_unstable_by_key(|is| is.min_body_id);
        islands
    }

    #[inline]
    fn route_to_island(
        &mut self,
        body_a: u32, body_b: u32, dyn_a: bool, dyn_b: bool,
        is_joint: bool, idx: usize, islands: &mut Vec<Island>
    ) {
        if !dyn_a && !dyn_b { return; }

        let root = if dyn_a { self.find(body_a) } else { self.find(body_b) };
        let island_idx = self.island_lookup[root as usize];

        let target_island = if island_idx == u32::MAX {
            let new_idx = islands.len() as u32;
            self.island_lookup[root as usize] = new_idx;
            islands.push(Island {
                contact_indices: Vec::new(),
                joint_indices: Vec::new(),
                min_body_id: root,
            });
            &mut islands[new_idx as usize]
        } else {
            &mut islands[island_idx as usize]
        };

        if is_joint {
            target_island.joint_indices.push(idx);
        } else {
            target_island.contact_indices.push(idx);
        }
    }

    fn find(&mut self, i: u32) -> u32 {
        let mut root = i;
        while self.parent[root as usize] != root {
            self.parent[root as usize] = self.parent[self.parent[root as usize] as usize];
            root = self.parent[root as usize];
        }
        root
    }

    fn union(&mut self, i: u32, j: u32) {
        let root_i = self.find(i);
        let root_j = self.find(j);

        if root_i != root_j {
            if self.rank[root_i as usize] < self.rank[root_j as usize] {
                self.parent[root_i as usize] = root_j;
            } else if self.rank[root_i as usize] > self.rank[root_j as usize] {
                self.parent[root_j as usize] = root_i;
            } else {
                self.parent[root_i as usize] = root_j;
                self.rank[root_j as usize] += 1;
            }
        }
    }
}