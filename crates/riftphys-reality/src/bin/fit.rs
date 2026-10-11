//! Fit a scene's parameters to a reality log and report held-out error before and after.
//!
//!     riftphys-fit --log ball.jsonl --scene scenes/ball_drop.json [--window 1.0] [--stride 0.25]
//!                  [--holdout-every 4] [--evals 300] [--seed 1] [--out fitted.json]

use riftphys_reality::fit::{fit, FitConfig, Scores, HORIZONS};
use riftphys_reality::log::RealityLog;
use riftphys_reality::scene::Scene;

fn row(name: &str, s: &Scores) -> String {
    let cells: Vec<String> = s.rms.iter().map(|v| format!("{:>9.2}", v * 1000.0)).collect();
    format!("{:<24}{} | {} windows", name, cells.join(""), s.windows)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let get = |n: &str| args.windows(2).find(|w| w[0] == n).map(|w| w[1].clone());
    let log = RealityLog::load(&get("--log").expect("--log")).unwrap_or_else(|e| panic!("{}", e));
    let scene = Scene::load(&get("--scene").expect("--scene")).unwrap_or_else(|e| panic!("{}", e));
    let d = FitConfig::default();
    let cfg = FitConfig {
        window: get("--window").map(|v| v.parse().unwrap()).unwrap_or(d.window),
        stride: get("--stride").map(|v| v.parse().unwrap()).unwrap_or(d.stride),
        holdout_every: get("--holdout-every").map(|v| v.parse().unwrap()).unwrap_or(d.holdout_every),
        vel_half: get("--vel-half").map(|v| v.parse().unwrap()).unwrap_or(d.vel_half),
        max_residual: get("--max-residual").map(|v| v.parse().unwrap()).unwrap_or(d.max_residual),
        max_evals: get("--evals").map(|v| v.parse().unwrap()).unwrap_or(d.max_evals),
        seed: get("--seed").map(|v| v.parse().unwrap()).unwrap_or(d.seed),
    };
    let t0 = std::time::Instant::now();
    let r = fit(&scene, &log, &cfg).unwrap_or_else(|e| panic!("{}", e));
    println!("scene {} | log {} ({} Hz) | {} evaluations in {:.1} s", scene.name, log.source, log.rate_hz, r.evals, t0.elapsed().as_secs_f32());
    println!("\n{:<16}{:>10}{:>10}", "parameter", "initial", "fitted");
    for ((n, a), b) in r.names.iter().zip(&r.initial).zip(&r.fitted) { println!("{:<16}{:>10.4}{:>10.4}", n, a, b); }
    let hs: Vec<String> = HORIZONS.iter().map(|h| format!("{:>8.2}s", h)).collect();
    println!("\nRMS position error (mm) up to horizon:\n{:<24}{}", "", hs.join(" "));
    println!("{}", row("train, initial", &r.train_before));
    println!("{}", row("train, fitted", &r.train_after));
    println!("{}", row("held-out, initial", &r.test_before));
    println!("{}", row("held-out, fitted", &r.test_after));
    if let Some(out) = get("--out") {
        let v: serde_json::Map<String, serde_json::Value> = r.names.iter().zip(&r.fitted).map(|(n, x)| (n.clone(), serde_json::json!(x))).collect();
        std::fs::write(&out, serde_json::to_string_pretty(&v).unwrap()).unwrap();
        println!("\nwrote {}", out);
    }
}
