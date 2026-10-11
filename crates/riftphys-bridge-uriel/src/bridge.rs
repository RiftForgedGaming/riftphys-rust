
// riftphys-bridge-uriel/src/lib.rs
use anyhow::{Result, anyhow};
use serde::{Serialize, Deserialize};
use serde_json::Value;

use riftphys_core::BodyId;
use riftphys_core::models::{AeroHandle, PropHandle};
use riftphys_gravity::GravitySpec;
use riftphys_world::world::World;

// -----------------------------------------------------------------------------
// Scenario description (static / slow-changing)
// -----------------------------------------------------------------------------

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SimulationDescriptor {
    /// Target fixed timestep the host should use for world.step(dt).
    pub dt: f32,
    /// Gravity model + parameters.
    pub gravity: GravityBlock,
    /// Per-body accelerator bindings (aero/prop) + base ref area + initial throttle.
    pub accel: Vec<AccelBlock>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct GravityBlock {
    /// "Uniform" | "LayeredPlanet" | future variants
    pub model: String,
    pub params: Value,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct AccelBlock {
    /// World body index (BodyId.0). URIEL reasons over these.
    pub body_index: u32,
    /// Reference area for aero in m^2 (used by your models).
    pub ref_area_m2: f32,
    /// Initial throttle [0,1] for this body.
    pub throttle01: f32,
    /// Opaque handles: host populates these from world.models_mut().register_*.
    pub aero: Option<usize>,
    pub prop: Option<usize>,
}

/// Validate and apply a SimulationDescriptor to an existing World.
/// This is called when URIEL (or the host) pushes a new scenario.
pub fn validate_and_queue(world: &mut World, sd: &SimulationDescriptor) -> Result<()> {
    if !(sd.dt > 0.0 && sd.dt <= 1.0) {
        return Err(anyhow!("dt out of range"));
    }

    // --- Gravity ---
    let gspec = match sd.gravity.model.as_str() {
        "Uniform" => {
            let arr = sd
                .gravity
                .params
                .get("g")
                .and_then(|v| v.as_array())
                .ok_or_else(|| anyhow!("uniform.g missing"))?;
            if arr.len() != 3 {
                return Err(anyhow!("uniform.g must be len=3"));
            }
            let gx = arr[0].as_f64().ok_or_else(|| anyhow!("gx"))? as f32;
            let gy = arr[1].as_f64().ok_or_else(|| anyhow!("gy"))? as f32;
            let gz = arr[2].as_f64().ok_or_else(|| anyhow!("gz"))? as f32;
            GravitySpec::Uniform { g: [gx, gy, gz] }
        }
        "LayeredPlanet" => {
            let sg = sd
                .gravity
                .params
                .get("surface_g")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| anyhow!("surface_g"))? as f32;
            let r = sd
                .gravity
                .params
                .get("radius")
                .and_then(|v| v.as_f64())
                .ok_or_else(|| anyhow!("radius"))? as f32;
            let c = sd
                .gravity
                .params
                .get("center")
                .and_then(|v| v.as_array())
                .ok_or_else(|| anyhow!("center"))?;
            if c.len() != 3 {
                return Err(anyhow!("center len!=3"));
            }
            let cx = c[0].as_f64().unwrap_or(0.0) as f32;
            let cy = c[1].as_f64().unwrap_or(0.0) as f32;
            let cz = c[2].as_f64().unwrap_or(0.0) as f32;
            let min_r = sd
                .gravity
                .params
                .get("min_r")
                .and_then(|v| v.as_f64())
                .unwrap_or(1000.0) as f32;
            GravitySpec::LayeredPlanet {
                surface_g: sg,
                radius: r,
                center: [cx, cy, cz],
                min_r,
            }
        }
        other => return Err(anyhow!("unsupported gravity model: {}", other)),
    };
    world.queue_gravity_swap(gspec);

    // --- Accel bindings ---
    for a in &sd.accel {
        let id = BodyId(a.body_index);

        // World-side model registries are already populated by the host.
        // Here we just wrap the integer IDs into handle types.
        let aero_h = a.aero.map(|i| AeroHandle(i as u32));
        let prop_h = a.prop.map(|i| PropHandle(i as u32));

        world.set_body_accel(id, aero_h, prop_h, a.ref_area_m2, a.throttle01, None);
    }

    Ok(())
}

/// Convenience: parse JSON → SimulationDescriptor → apply.
/// Returns the fully-typed descriptor so the caller can keep dt, etc.
pub fn apply_sim_descriptor_json(world: &mut World, json: &str) -> Result<SimulationDescriptor> {
    let sd: SimulationDescriptor = serde_json::from_str(json)?;
    validate_and_queue(world, &sd)?;
    Ok(sd)
}

// -----------------------------------------------------------------------------
// Control path (fast / per-tick)
// -----------------------------------------------------------------------------

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ControlPacket {
    /// Logical tick URIEL thinks this command applies to.
    pub tick: u64,
    /// dt URIEL assumes the host will advance by when applying this control.
    /// (You can cross-check this if you want.)
    pub dt: f32,
    /// Per-body throttle commands.
    pub bodies: Vec<ThrottleCmd>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ThrottleCmd {
    pub body_index: u32,
    pub throttle01: f32,
}

/// Apply a control packet: updates only throttle, not bindings.
pub fn apply_control(world: &mut World, cp: &ControlPacket) -> Result<()> {
    if !(cp.dt > 0.0 && cp.dt <= 1.0) {
        return Err(anyhow!("control dt out of range"));
    }

    for cmd in &cp.bodies {
        let bid = BodyId(cmd.body_index);
        // Host already bound aero/prop via SimulationDescriptor; here we just modulate throttle.
        world.set_body_throttle(bid, cmd.throttle01);
    }

    Ok(())
}

pub fn apply_control_json(world: &mut World, json: &str) -> Result<ControlPacket> {
    let cp: ControlPacket = serde_json::from_str(json)?;
    apply_control(world, &cp)?;
    Ok(cp)
}
