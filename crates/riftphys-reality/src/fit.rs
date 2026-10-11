//! Fit a scene's unknown parameters to a reality log: replay windows, score the position error,
//! minimize it with CMA-ES on training windows, and report held-out error before and after.
//! Held-out data is whole takes (between `break` events), so overlapping windows of one throw
//! never land on both sides; a log without breaks holds out its last part in time.

use crate::cmaes;
use crate::log::RealityLog;
use crate::scene::{replay, windows, Scene, Window};

#[derive(Clone, Debug)]
pub struct FitConfig {
    /// Replay window length and spacing (s)
    pub window: f64,
    pub stride: f64,
    /// Every k-th take is held out (or the last 1/k of the log when there are no breaks)
    pub holdout_every: usize,
    /// Half-width of the span whose constant-acceleration fit gives a window's start state (s)
    pub vel_half: f64,
    /// Skip windows whose start-state fit residual exceeds this (m): an impact in the span
    pub max_residual: f32,
    pub max_evals: usize,
    pub seed: u64,
}

impl Default for FitConfig {
    fn default() -> Self { Self { window: 1.0, stride: 0.25, holdout_every: 4, vel_half: 0.03, max_residual: 0.002, max_evals: 300, seed: 1 } }
}

pub const HORIZONS: [f32; 4] = [0.1, 0.25, 0.5, 1.0];

#[derive(Clone, Debug)]
pub struct Scores {
    pub windows: usize,
    /// Mean over windows of the RMS position error up to each of `HORIZONS` (m)
    pub rms: Vec<f32>,
}

#[derive(Clone, Debug)]
pub struct FitReport {
    pub names: Vec<String>,
    pub initial: Vec<f32>,
    pub fitted: Vec<f32>,
    pub train_before: Scores,
    pub train_after: Scores,
    pub test_before: Scores,
    pub test_after: Scores,
    pub evals: usize,
}

pub fn split(log: &RealityLog, ws: &[Window], every: usize) -> (Vec<Window>, Vec<Window>) {
    let every = every.max(2);
    let (mut train, mut test) = (Vec::new(), Vec::new());
    if !log.breaks.is_empty() {
        for w in ws {
            let take = log.breaks.iter().filter(|&&b| b <= w.start).count();
            if take % every == every - 1 { test.push(*w) } else { train.push(*w) }
        }
    } else if let (Some(first), Some(last)) = (ws.first(), ws.last()) {
        let cut = last.start - (last.start - first.start) / every as f64;
        for w in ws { if w.start >= cut { test.push(*w) } else { train.push(*w) } }
    }
    (train, test)
}

/// Mean RMS position error over windows at each horizon, for physical parameters `x`.
pub fn score(scene: &Scene, log: &RealityLog, x: &[f32], ws: &[Window], vel_half: f64) -> Scores {
    let errs: Vec<_> = ws.iter().filter_map(|w| replay(scene, log, x, *w, vel_half)).collect();
    let rms = HORIZONS.iter().map(|&h| {
        let v: Vec<f32> = errs.iter().filter_map(|e| e.rms_until(h)).collect();
        if v.is_empty() { f32::NAN } else { v.iter().sum::<f32>() / v.len() as f32 }
    }).collect();
    Scores { windows: errs.len(), rms }
}

fn to_phys(scene: &Scene, u: &[f64]) -> Vec<f32> {
    scene.params.iter().zip(u).map(|(p, &v)| p.min + v as f32 * (p.max - p.min)).collect()
}

pub fn fit(scene: &Scene, log: &RealityLog, cfg: &FitConfig) -> Result<FitReport, String> {
    if scene.params.is_empty() { return Err("scene has no params to fit".into()); }
    let ws = windows(scene, log, cfg.window, cfg.stride, cfg.vel_half, cfg.max_residual);
    let (train, test) = split(log, &ws, cfg.holdout_every);
    if train.is_empty() || test.is_empty() {
        return Err(format!("need training and held-out windows (got {} / {} of {})", train.len(), test.len(), ws.len()));
    }
    let x0 = scene.init_params();
    let u0: Vec<f64> = scene.params.iter().map(|p| ((p.init - p.min) / (p.max - p.min).max(1e-12)) as f64).collect();
    // Objective: mean RMS over the whole window, training windows only
    let full = HORIZONS.len() - 1;
    let objective = |u: &[f64]| -> f64 {
        let s = score(scene, log, &to_phys(scene, u), &train, cfg.vel_half);
        let v = s.rms[full] as f64;
        if v.is_finite() { v } else { 1e9 }
    };
    let r = cmaes::minimize(objective, &u0, 0.3, cfg.max_evals, cfg.seed);
    let x = to_phys(scene, &r.x);
    Ok(FitReport {
        names: scene.params.iter().map(|p| p.name.clone()).collect(),
        initial: x0.clone(),
        fitted: x.clone(),
        train_before: score(scene, log, &x0, &train, cfg.vel_half),
        train_after: score(scene, log, &x, &train, cfg.vel_half),
        test_before: score(scene, log, &x0, &test, cfg.vel_half),
        test_after: score(scene, log, &x, &test, cfg.vel_half),
        evals: r.evals,
    })
}
