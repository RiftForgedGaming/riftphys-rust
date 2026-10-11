//! A scene spec says how to rebuild a captured scene in RiftPhys (ground, objects matched to
//! log bodies by name) and which physical parameters are unknown. Replay starts every object
//! from its logged pose and velocity at a window's start, steps the world, and compares the
//! simulated poses with the logged ones.
//!
//! ```json
//! {"name": "ball_drop", "hz": 240,
//!  "ground": {"material": "Concrete", "height": 0.0},
//!  "objects": [{"body": "ball", "shape": {"sphere": {"r": 0.033}}, "mass": 0.058, "material": "Rubber"}],
//!  "params": [{"name": "e", "kind": "restitution", "pair": ["Rubber", "Concrete"], "min": 0.0, "max": 1.0, "init": 0.5},
//!             {"name": "drag", "kind": "drag", "body": "ball", "min": 0.0, "max": 0.01, "init": 0.0}]}
//! ```

use crate::log::RealityLog;
use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::World;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ShapeSpec {
    Sphere { r: f32 },
    Box { hx: f32, hy: f32, hz: f32 },
    Capsule { r: f32, hh: f32 },
}

impl ShapeSpec {
    fn shape(&self) -> Shape {
        match *self {
            ShapeSpec::Sphere { r } => Shape::Sphere { r },
            ShapeSpec::Box { hx, hy, hz } => Shape::Box { hx, hy, hz },
            ShapeSpec::Capsule { r, hh } => Shape::Capsule { r, hh },
        }
    }

    fn mass_props(&self, mass: f32) -> MassProps {
        match *self {
            ShapeSpec::Sphere { r } => MassProps::from_sphere_density(r, mass / (4.0 / 3.0 * std::f32::consts::PI * r * r * r)),
            ShapeSpec::Box { hx, hy, hz } => MassProps::from_box_density(Vec3::new(hx, hy, hz), mass / (8.0 * hx * hy * hz)),
            ShapeSpec::Capsule { r, hh } => {
                let vol = std::f32::consts::PI * r * r * (2.0 * hh) + 4.0 / 3.0 * std::f32::consts::PI * r * r * r;
                MassProps::from_capsule_density(r, hh, mass / vol)
            }
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GroundSpec {
    pub material: MaterialId,
    /// Height of the ground surface (m)
    #[serde(default)]
    pub height: f32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ObjectSpec {
    /// Log body name
    pub body: String,
    pub shape: ShapeSpec,
    pub mass: f32,
    pub material: MaterialId,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ParamKind {
    /// Contact properties of a material pair
    Restitution,
    MuS,
    MuK,
    /// Quadratic air drag coefficient k (N per (m/s)^2): F = -k |v| v, on one body
    Drag,
    /// Mass multiplier on one body (inertia scales with it)
    MassScale,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ParamSpec {
    pub name: String,
    pub kind: ParamKind,
    #[serde(default)]
    pub pair: Option<(MaterialId, MaterialId)>,
    #[serde(default)]
    pub body: Option<String>,
    pub min: f32,
    pub max: f32,
    pub init: f32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Scene {
    pub name: String,
    /// Simulation rate for replay
    pub hz: u32,
    #[serde(default)]
    pub ground: Option<GroundSpec>,
    pub objects: Vec<ObjectSpec>,
    #[serde(default)]
    pub params: Vec<ParamSpec>,
}

/// A built world and its objects, in `Scene::objects` order.
pub struct Built {
    pub world: World,
    pub ids: Vec<BodyId>,
    /// Drag coefficient per object
    pub drag: Vec<f32>,
    pub masses: Vec<f32>,
}

impl Scene {
    pub fn load(path: &str) -> Result<Self, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {}", path, e))?;
        let s: Scene = serde_json::from_str(&text).map_err(|e| format!("{}: {}", path, e))?;
        s.validate()?;
        Ok(s)
    }

    pub fn validate(&self) -> Result<(), String> {
        for p in &self.params {
            if !(p.min <= p.init && p.init <= p.max) { return Err(format!("param {}: init outside [min, max]", p.name)); }
            match p.kind {
                ParamKind::Restitution | ParamKind::MuS | ParamKind::MuK if p.pair.is_none() => return Err(format!("param {} needs a pair", p.name)),
                ParamKind::Drag | ParamKind::MassScale => {
                    let b = p.body.as_ref().ok_or(format!("param {} needs a body", p.name))?;
                    if !self.objects.iter().any(|o| &o.body == b) { return Err(format!("param {}: no object {}", p.name, b)); }
                }
                _ => {}
            }
        }
        Ok(())
    }

    pub fn dt(&self) -> f32 { 1.0 / self.hz as f32 }

    pub fn init_params(&self) -> Vec<f32> { self.params.iter().map(|p| p.init).collect() }

    pub fn bodies(&self) -> Vec<String> { self.objects.iter().map(|o| o.body.clone()).collect() }

    /// Build the world with parameter values `x` (in `params` order), every object at rest at
    /// the origin; `place` positions them.
    pub fn build(&self, x: &[f32]) -> Built {
        let mut world = World::with_capacity(8 + self.objects.len(), 8 + self.objects.len());
        world.set_gravity(Vec3::new(0.0, -9.81, 0.0));
        if let Some(g) = &self.ground {
            let id = world.add_body(Isometry { pos: Vec3::new(0.0, g.height - 0.5, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
            world.add_collider(id, Shape::Box { hx: 100.0, hy: 0.5, hz: 100.0 }, material(g.material));
        }
        // Pair properties: start from the table, apply fitted values
        let mut pairs: Vec<((MaterialId, MaterialId), riftphys_materials::materials::MatPairProps)> = Vec::new();
        let mut mass_scale = vec![1.0f32; self.objects.len()];
        let mut drag = vec![0.0f32; self.objects.len()];
        for (p, &v) in self.params.iter().zip(x) {
            let obj = |b: &Option<String>| self.objects.iter().position(|o| Some(&o.body) == b.as_ref()).unwrap();
            match p.kind {
                ParamKind::Restitution | ParamKind::MuS | ParamKind::MuK => {
                    let (a, b) = p.pair.unwrap();
                    let i = match pairs.iter().position(|(k, _)| *k == (a, b)) {
                        Some(i) => i,
                        None => { pairs.push(((a, b), world.pair_props(a, b))); pairs.len() - 1 }
                    };
                    let props = &mut pairs[i].1;
                    match p.kind {
                        ParamKind::Restitution => props.restitution = v,
                        ParamKind::MuS => props.mu_s = v,
                        _ => props.mu_k = v,
                    }
                }
                ParamKind::Drag => drag[obj(&p.body)] = v,
                ParamKind::MassScale => mass_scale[obj(&p.body)] = v,
            }
        }
        for ((a, b), props) in pairs {
            let mut props = props;
            props.mu_s = props.mu_s.max(props.mu_k); // static friction is never below kinetic
            world.set_pair_props(a, b, props);
        }
        let mut ids = Vec::new();
        let mut masses = Vec::new();
        for (o, s) in self.objects.iter().zip(&mass_scale) {
            let m = o.mass * s;
            let id = world.add_body(Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY }, Velocity::default(), o.shape.mass_props(m), true);
            world.add_collider(id, o.shape.shape(), material(o.material));
            world.set_body_sleep_allowed(id, false);
            ids.push(id);
            masses.push(m);
        }
        Built { world, ids, drag, masses }
    }
}

/// Position error between replay and log for one window, sampled at every logged pose.
#[derive(Clone, Debug, Default)]
pub struct WindowError {
    /// (seconds since window start, position error m) per logged sample, all bodies
    pub samples: Vec<(f32, f32)>,
}

impl WindowError {
    /// RMS position error over samples up to `horizon` seconds into the window.
    pub fn rms_until(&self, horizon: f32) -> Option<f32> {
        let v: Vec<f32> = self.samples.iter().filter(|(t, _)| *t <= horizon).map(|(_, e)| *e).collect();
        (!v.is_empty()).then(|| (v.iter().map(|e| e * e).sum::<f32>() / v.len() as f32).sqrt())
    }
}

/// One replay window: start time and length.
#[derive(Clone, Copy, Debug)]
pub struct Window { pub start: f64, pub len: f64 }

/// Windows of `len` seconds every `stride`, inside the time covered by every scene body, never
/// across a `break`, and only where every body's start state is smooth (fit residual at most
/// `max_residual` m: no impact inside the span used to estimate it).
pub fn windows(scene: &Scene, log: &RealityLog, len: f64, stride: f64, vel_half: f64, max_residual: f32) -> Vec<Window> {
    let Some((lo, hi)) = log.span(&scene.bodies()) else { return vec![] };
    let mut out = Vec::new();
    let mut t = lo + vel_half;
    while t + len <= hi - vel_half {
        let crosses = log.breaks.iter().any(|&b| b > t - vel_half && b < t + len);
        let smooth = scene.objects.iter().all(|o| log.state_at(&o.body, t, vel_half).map_or(false, |s| s.3 <= max_residual));
        if !crosses && smooth { out.push(Window { start: t, len }); }
        t += stride;
    }
    out
}

/// Replay one window with parameters `x`.
pub fn replay(scene: &Scene, log: &RealityLog, x: &[f32], w: Window, vel_half: f64) -> Option<WindowError> {
    let mut b = scene.build(x);
    for (o, id) in scene.objects.iter().zip(&b.ids) {
        let (_, q) = log.pose_at(&o.body, w.start)?;
        let (p, v, wv, _) = log.state_at(&o.body, w.start, vel_half)?;
        b.world.set_body_pose(*id, Isometry { pos: p, rot: q });
        b.world.set_body_vel(*id, Velocity { lin: v, ang: wv });
    }
    let dt = scene.dt() as f64;
    let steps = (w.len / dt).ceil() as usize;
    // Simulated positions at every step (step 0 = window start)
    let mut sim: Vec<Vec<Vec3>> = vec![b.ids.iter().map(|id| b.world.get_body_pose(*id).pos).collect()];
    for _ in 0..steps {
        for (k, id) in b.ids.iter().enumerate() {
            if b.drag[k] > 0.0 {
                let v = b.world.get_body_vel(*id).lin;
                b.world.apply_impulse(*id, -v * (b.drag[k] * v.length() * dt as f32));
            }
        }
        b.world.step(dt as f32);
        sim.push(b.ids.iter().map(|id| b.world.get_body_pose(*id).pos).collect());
    }
    let mut err = WindowError::default();
    for (k, o) in scene.objects.iter().enumerate() {
        for s in log.tracks.get(&o.body)?.iter().filter(|s| s.time > w.start && s.time <= w.start + w.len) {
            let f = (s.time - w.start) / dt;
            let i = (f.floor() as usize).min(steps - 1);
            let a = (f - i as f64) as f32;
            let p = sim[i][k].lerp(sim[i + 1][k], a);
            err.samples.push(((s.time - w.start) as f32, (p - s.pos).length()));
        }
    }
    Some(err)
}

impl crate::fit::Replayable for Scene {
    fn params(&self) -> Vec<(String, f32, f32, f32)> {
        self.params.iter().map(|p| (p.name.clone(), p.min, p.max, p.init)).collect()
    }
    fn windows(&self, log: &RealityLog, cfg: &crate::fit::FitConfig) -> Vec<Window> {
        windows(self, log, cfg.window, cfg.stride, cfg.vel_half, cfg.max_residual)
    }
    fn replay(&self, log: &RealityLog, x: &[f32], w: Window, cfg: &crate::fit::FitConfig) -> Option<WindowError> {
        replay(self, log, x, w, cfg.vel_half)
    }
}
