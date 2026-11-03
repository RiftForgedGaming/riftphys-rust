//! URIEL robotics protocol: what a RiftPhys scenario writes for URIEL (and, later, reads back as
//! commands). Same shape as the RiftForged server telemetry contract (one JSON object per line,
//! tagged by `t`, raw named state, binning left to the consumer), so one URIEL pipeline serves
//! game and robotics data. See docs/URIEL_PROTOCOL.md.

use serde::{Deserialize, Serialize};

pub const PROTOCOL_VERSION: &str = "0.2";

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
    /// v0.2: sensors this kind carries; their readings appear in each entity's sample.
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub sensors: Option<Vec<SensorSpec>>,
}

/// v0.2: a sensor of an agent kind. `line_scan`: `n` rays fanned across `fan_m` at the work,
/// ranges up to `max_range_m`, reported in `EntState::scan` in ray order.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct SensorSpec {
    pub name: String,
    #[serde(rename = "type")]
    pub kind: String,
    pub n: u32,
    pub fan_m: f32,
    pub max_range_m: f32,
    pub unit: String,
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
    /// Articulated agents: [angle rad, rate rad/s] per joint, in `KindSpec.actuators` order.
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub joints: Option<Vec<[f32; 2]>>,
    /// v0.2, line-scan sensors: range per ray (m), `null` where nothing was hit.
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub scan: Option<Vec<Option<f32>>>,
    /// v0.2, the outcome metric when the scenario has a ground truth: tracking error (m) of the
    /// entity against `target` in the task's own frame (seam tracking: [lateral, height]).
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub err: Option<Vec<f32>>,
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
        /// v0.2: the episode's ground truth and seeded disturbances (scenario-specific object).
        #[serde(skip_serializing_if = "Option::is_none", default)]
        scene: Option<serde_json::Value>,
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
    /// v0.2, human in the loop: an operator input at its exact tick (`pause`, `resume`,
    /// `override` with `data`, `stop`). Scenarios that support an operator record every input
    /// they act on as this line; serving them live (from a socket) is milestone M5.
    Operator {
        tick: u64,
        ep: String,
        who: u32,
        cmd: String,
        #[serde(default)]
        data: serde_json::Value,
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
