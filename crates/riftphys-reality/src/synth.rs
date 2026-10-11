//! Synthetic reality logs: simulate a scene with known "true" parameters, sample poses like a
//! mocap system (rate, noise), and write a reality log. Fitting one back from a wrong guess
//! checks the whole pipeline before real data exists.

use crate::log::{RealityLog, Sample};
use crate::scene::Scene;
use riftphys_core::{Isometry, Quat, Vec3, Velocity};

pub struct SynthConfig {
    pub takes: usize,
    pub take_seconds: f32,
    pub rate_hz: f32,
    /// Position noise (m, standard deviation)
    pub noise: f32,
    pub seed: u64,
}

struct Rng(u64);
impl Rng {
    fn unit(&mut self) -> f32 { self.0 ^= self.0 << 13; self.0 ^= self.0 >> 7; self.0 ^= self.0 << 17; ((self.0 >> 40) as f32 + 0.5) / (1u64 << 24) as f32 }
    fn range(&mut self, lo: f32, hi: f32) -> f32 { lo + (hi - lo) * self.unit() }
    fn gauss(&mut self) -> f32 { let (u, v) = (self.unit(), self.unit()); (-2.0 * u.ln()).sqrt() * (2.0 * std::f32::consts::PI * v).cos() }
}

/// Throws every scene object from a random height with random velocity and spin, `takes` times.
pub fn synthesize(scene: &Scene, truth: &[f32], cfg: &SynthConfig) -> RealityLog {
    let mut rng = Rng(cfg.seed.max(1));
    let mut log = RealityLog { source: "synthetic".into(), rate_hz: cfg.rate_hz, ..Default::default() };
    for o in &scene.objects { log.tracks.insert(o.body.clone(), Vec::new()); }
    let dt = scene.dt();
    let steps = (cfg.take_seconds / dt).round() as usize;
    let every = (1.0 / (cfg.rate_hz * dt)).round().max(1.0) as usize;
    let mut t0 = 0.0f64;
    for take in 0..cfg.takes {
        let mut b = scene.build(truth);
        for (k, id) in b.ids.iter().enumerate() {
            let pos = Vec3::new(rng.range(-0.5, 0.5) + 1.5 * k as f32, rng.range(0.5, 1.5), rng.range(-0.5, 0.5));
            let rot = Quat::from_euler(glam::EulerRot::XYZ, rng.range(-3.0, 3.0), rng.range(-3.0, 3.0), rng.range(-3.0, 3.0));
            let lin = Vec3::new(rng.range(-2.0, 2.0), rng.range(0.0, 2.0), rng.range(-2.0, 2.0));
            let ang = Vec3::new(rng.range(-5.0, 5.0), rng.range(-5.0, 5.0), rng.range(-5.0, 5.0));
            b.world.set_body_pose(*id, Isometry { pos, rot });
            b.world.set_body_vel(*id, Velocity { lin, ang });
        }
        for s in 0..=steps {
            if s % every == 0 {
                let t = t0 + s as f64 * dt as f64;
                for (o, id) in scene.objects.iter().zip(&b.ids) {
                    let p = b.world.get_body_pose(*id);
                    let n = Vec3::new(rng.gauss(), rng.gauss(), rng.gauss()) * cfg.noise;
                    log.tracks.get_mut(&o.body).unwrap().push(Sample { time: t, pos: p.pos + n, rot: p.rot });
                }
            }
            if s < steps {
                for (k, id) in b.ids.iter().enumerate() {
                    if b.drag[k] > 0.0 {
                        let v = b.world.get_body_vel(*id).lin;
                        b.world.apply_impulse(*id, -v * (b.drag[k] * v.length() * dt));
                    }
                }
                b.world.step(dt);
            }
        }
        t0 += cfg.take_seconds as f64 + 0.5;
        if take + 1 < cfg.takes { log.breaks.push(t0 - 0.25); }
    }
    log
}
