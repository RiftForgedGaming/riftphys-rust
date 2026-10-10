//! URIEL robotics protocol: what a RiftPhys scenario writes for URIEL (and, later, reads back as
//! commands). Same shape as the RiftForged server telemetry contract (one JSON object per line,
//! tagged by `t`, raw named state, binning left to the consumer), so one URIEL pipeline serves
//! game and robotics data. See docs/URIEL_PROTOCOL.md.

use serde::{Deserialize, Serialize};

pub const PROTOCOL_VERSION: &str = "0.1";

/// One actuator of an agent kind, in command order.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct ActuatorSpec {
    pub name: String,
    pub min: f32,
    pub max: f32,
    pub unit: String,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct KindSpec {
    pub kind: String,
    pub actuators: Vec<ActuatorSpec>,
}

/// Per-entity state in a sample. Positions in metres, world frame, y up; `rot` is a unit
/// quaternion [x, y, z, w]; `ang` is world angular velocity in rad/s.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct EntState {
    pub eid: u32,
    pub kind: String,
    pub pos: [f32; 3],
    pub rot: [f32; 4],
    pub vel: [f32; 3],
    pub ang: [f32; 3],
    /// Where the agent is trying to be (formation slot, waypoint), if it has a goal.
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub target: Option<[f32; 3]>,
    /// Actuator commands applied this tick, in `KindSpec.actuators` order.
    pub act: Vec<f32>,
    /// A teacher's command for this tick, when one exists (the label for imitation). With
    /// exploration noise, `act` (what ran) differs from `demo` (what the expert wanted).
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub demo: Option<Vec<f32>>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "t", rename_all = "snake_case")]
pub enum Line {
    /// First line of every file.
    Meta {
        tick: u64,
        v: String,
        source: String,
        scenario: String,
        sim_hz: u32,
        sample_every: u32,
        seed: u64,
        kinds: Vec<KindSpec>,
    },
    EpisodeStart {
        tick: u64,
        ep: String,
        scenario: String,
        members: Vec<u32>,
        seed: u64,
    },
    Sample {
        tick: u64,
        ep: String,
        ents: Vec<EntState>,
        /// World step hash (first 16 hex chars of blake3) after this tick: replaying the
        /// episode from its seed must reproduce it exactly.
        hash: String,
    },
    Event {
        tick: u64,
        ep: String,
        /// `crash`, `gust`, `reached`, ...
        e: String,
        who: u32,
        #[serde(default)]
        data: serde_json::Value,
    },
    EpisodeEnd {
        tick: u64,
        ep: String,
        /// `stable` (goal held), `crash`, or `timeout`
        result: String,
        duration_s: f32,
    },
    /// Closed loop (consumer -> RiftPhys): actuator commands for one entity at a tick.
    Command {
        tick: u64,
        who: u32,
        act: Vec<f32>,
    },
}

pub fn hash_hex(hash: &[u8; 32]) -> String {
    hash[..8].iter().map(|b| format!("{:02x}", b)).collect()
}
