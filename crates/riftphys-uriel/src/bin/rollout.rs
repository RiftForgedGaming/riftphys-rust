//! Writes expert drone demonstrations as URIEL protocol JSONL.
//!
//!     riftphys-uriel-rollout --out drone_hover.jsonl [--episodes 50] [--drones 1] [--seed 1]
//!                            [--seconds 10] [--noise 0.1] [--gust-rate 0.004]

use riftphys_uriel::rollout::{meta_line, run_episode, RolloutConfig};
use std::io::{BufWriter, Write};

fn arg<T: std::str::FromStr>(args: &[String], name: &str, default: T) -> T {
    args.windows(2).find(|w| w[0] == name).map(|w| w[1].parse().ok().expect("bad argument value")).unwrap_or(default)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let out: String = arg(&args, "--out", String::new());
    if out.is_empty() {
        eprintln!("usage: riftphys-uriel-rollout --out FILE.jsonl [--episodes N] [--drones N] [--seed S] [--seconds S] [--noise X] [--gust-rate P]");
        std::process::exit(1);
    }
    let episodes: u64 = arg(&args, "--episodes", 50);
    let seed: u64 = arg(&args, "--seed", 1);
    let mut cfg = RolloutConfig::default();
    cfg.drones.count = arg(&args, "--drones", 1);
    cfg.seconds = arg(&args, "--seconds", cfg.seconds);
    cfg.noise = arg(&args, "--noise", cfg.noise);
    cfg.gust_rate = arg(&args, "--gust-rate", cfg.gust_rate);

    let mut w = BufWriter::new(std::fs::File::create(&out).expect("cannot create output"));
    writeln!(w, "{}", serde_json::to_string(&meta_line(&cfg, seed)).unwrap()).unwrap();
    let mut results = std::collections::BTreeMap::<String, u32>::new();
    for e in 0..episodes {
        for line in run_episode(&cfg, e, seed) {
            if let riftphys_uriel::protocol::Line::EpisodeEnd { result, .. } = &line {
                *results.entry(result.clone()).or_default() += 1;
            }
            writeln!(w, "{}", serde_json::to_string(&line).unwrap()).unwrap();
        }
    }
    w.flush().unwrap();
    println!("{} episodes of {} drone(s) -> {} | results {:?}", episodes, cfg.drones.count, out, results);
}
