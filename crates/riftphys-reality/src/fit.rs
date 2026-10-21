//! Fit a scene's unknown parameters to a reality log: replay windows, score the position error,
//! minimize it with CMA-ES on training windows, and report held-out error before and after.
//! Held-out data is whole takes (between `break` events), so overlapping windows of one throw
//! never land on both sides; a log without breaks holds out its last part in time.

use crate::cmaes;
use crate::log::RealityLog;
use crate::scene::{Window, WindowError};

/// Anything whose parameters can be fitted by replaying windows of a reality log: object scenes
/// (`scene::Scene`) and rotor-driven vehicles (`quadrotor::QuadScene`).
pub trait Replayable {
    /// (name, min, max, init) per fitted parameter
    fn params(&self) -> Vec<(String, f32, f32, f32)>;
    fn windows(&self, log: &RealityLog, cfg: &FitConfig) -> Vec<Window>;
    fn replay(&self, log: &RealityLog, x: &[f32], w: Window, cfg: &FitConfig) -> Option<WindowError>;
}

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
pub fn score(scene: &dyn Replayable, log: &RealityLog, x: &[f32], ws: &[Window], cfg: &FitConfig) -> Scores {
    let errs: Vec<_> = ws.iter().filter_map(|w| scene.replay(log, x, *w, cfg)).collect();
    let rms = HORIZONS.iter().map(|&h| {
        let v: Vec<f32> = errs.iter().filter_map(|e| e.rms_until(h)).collect();
        if v.is_empty() { f32::NAN } else { v.iter().sum::<f32>() / v.len() as f32 }
    }).collect();
    Scores { windows: errs.len(), rms }
}

fn to_phys(params: &[(String, f32, f32, f32)], u: &[f64]) -> Vec<f32> {
    params.iter().zip(u).map(|((_, lo, hi, _), &v)| lo + v as f32 * (hi - lo)).collect()
}

pub fn fit(scene: &dyn Replayable, log: &RealityLog, cfg: &FitConfig) -> Result<FitReport, String> {
    let ws = scene.windows(log, cfg);
    let (train, test) = split(log, &ws, cfg.holdout_every);
    if train.is_empty() || test.is_empty() {
        return Err(format!("need training and held-out windows (got {} / {} of {})", train.len(), test.len(), ws.len()));
    }
    fit_windows(scene, log, &train, log, &test, cfg)
}

/// Fit on every window of `train_log`, score held-out on every window of `test_log` (a benchmark's
/// own split, e.g. unseen trajectory types).
pub fn fit_logs(scene: &dyn Replayable, train_log: &RealityLog, test_log: &RealityLog, cfg: &FitConfig) -> Result<FitReport, String> {
    let (train, test) = (scene.windows(train_log, cfg), scene.windows(test_log, cfg));
    if train.is_empty() || test.is_empty() {
        return Err(format!("need training and held-out windows (got {} / {})", train.len(), test.len()));
    }
    fit_windows(scene, train_log, &train, test_log, &test, cfg)
}

fn fit_windows(scene: &dyn Replayable, log: &RealityLog, train: &[Window], test_log: &RealityLog, test: &[Window], cfg: &FitConfig) -> Result<FitReport, String> {
    let params = scene.params();
    if params.is_empty() { return Err("scene has no params to fit".into()); }
    let x0: Vec<f32> = params.iter().map(|p| p.3).collect();
    let u0: Vec<f64> = params.iter().map(|(_, lo, hi, init)| ((init - lo) / (hi - lo).max(1e-12)) as f64).collect();
    // Objective: mean RMS over the whole window, training windows only
    let full = HORIZONS.len() - 1;
    let objective = |u: &[f64]| -> f64 {
        let s = score(scene, log, &to_phys(&params, u), train, cfg);
        let v = s.rms[full] as f64;
        if v.is_finite() { v } else { 1e9 }
    };
    let r = cmaes::minimize(objective, &u0, 0.3, cfg.max_evals, cfg.seed);
    let x = to_phys(&params, &r.x);
    Ok(FitReport {
        names: params.iter().map(|p| p.0.clone()).collect(),
        initial: x0.clone(),
        fitted: x.clone(),
        train_before: score(scene, log, &x0, train, cfg),
        train_after: score(scene, log, &x, train, cfg),
        test_before: score(scene, test_log, &x0, test, cfg),
        test_after: score(scene, test_log, &x, test, cfg),
        evals: r.evals,
    })
}
