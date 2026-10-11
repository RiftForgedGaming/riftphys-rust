//! Synthetic reality log from a scene with known parameter values.
//!
//!     riftphys-synth --scene scenes/ball_drop.json --truth 0.72,0.45,0.0015 --out ball.jsonl
//!                    [--takes 16] [--seconds 2] [--rate 240] [--noise 0.0005] [--seed 1]

use riftphys_reality::quadrotor::{self, QuadScene, QuadSynthConfig};
use riftphys_reality::scene::Scene;
use riftphys_reality::synth::{synthesize, SynthConfig};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let get = |n: &str| args.windows(2).find(|w| w[0] == n).map(|w| w[1].clone());
    let path = get("--scene").expect("--scene");
    if std::fs::read_to_string(&path).map(|t| t.contains("\"rotors\"")).unwrap_or(false) {
        // Rotor-driven vehicle: one PD-controlled flight logged like Blackbird (poses + rotor rpm)
        let q = QuadScene::load(&path).unwrap_or_else(|e| panic!("{}", e));
        let truth: Vec<f32> = match get("--truth") {
            Some(t) => t.split(',').map(|v| v.trim().parse().expect("--truth values")).collect(),
            None => q.params.iter().map(|p| p.init).collect(),
        };
        assert_eq!(truth.len(), q.params.len(), "--truth needs one value per scene param");
        let cfg = QuadSynthConfig {
            seconds: get("--seconds").map(|v| v.parse().unwrap()).unwrap_or(60.0),
            pose_hz: get("--rate").map(|v| v.parse().unwrap()).unwrap_or(360.0),
            rpm_hz: get("--rpm-rate").map(|v| v.parse().unwrap()).unwrap_or(190.0),
            pos_noise: get("--noise").map(|v| v.parse().unwrap()).unwrap_or(0.0003),
            rpm_noise: get("--rpm-noise").map(|v| v.parse().unwrap()).unwrap_or(20.0),
            seed: get("--seed").map(|v| v.parse().unwrap()).unwrap_or(1),
        };
        let out = get("--out").expect("--out");
        let log = quadrotor::synthesize(&q, &truth, &cfg);
        let notes = format!("synthetic {} truth {:?}", q.name, q.params.iter().map(|p| &p.name).zip(&truth).collect::<Vec<_>>());
        log.save(&out, &notes).unwrap_or_else(|e| panic!("{}", e));
        println!("wrote {} ({} s flight, poses {} Hz, rpm {} Hz; capped at the scene's {} Hz step): {}", out, cfg.seconds, cfg.pose_hz.min(q.hz as f32), cfg.rpm_hz.min(q.hz as f32), q.hz, notes);
        return;
    }
    let scene = Scene::load(&path).unwrap_or_else(|e| panic!("{}", e));
    let truth: Vec<f32> = match get("--truth") {
        Some(t) => t.split(',').map(|v| v.trim().parse().expect("--truth values")).collect(),
        None => scene.init_params(),
    };
    assert_eq!(truth.len(), scene.params.len(), "--truth needs one value per scene param ({:?})", scene.params.iter().map(|p| &p.name).collect::<Vec<_>>());
    let cfg = SynthConfig {
        takes: get("--takes").map(|v| v.parse().unwrap()).unwrap_or(16),
        take_seconds: get("--seconds").map(|v| v.parse().unwrap()).unwrap_or(2.0),
        rate_hz: get("--rate").map(|v| v.parse().unwrap()).unwrap_or(240.0),
        noise: get("--noise").map(|v| v.parse().unwrap()).unwrap_or(0.0005),
        seed: get("--seed").map(|v| v.parse().unwrap()).unwrap_or(1),
    };
    let out = get("--out").expect("--out");
    let log = synthesize(&scene, &truth, &cfg);
    let notes = format!("synthetic {} truth {:?}", scene.name, scene.params.iter().map(|p| &p.name).zip(&truth).collect::<Vec<_>>());
    log.save(&out, &notes).unwrap_or_else(|e| panic!("{}", e));
    println!("wrote {} ({} takes x {} s at {} Hz, noise {} m): {}", out, cfg.takes, cfg.take_seconds, cfg.rate_hz, cfg.noise, notes);
}
