//! Seam tracking (fabrication F1): writes teacher demonstrations as URIEL protocol v0.2 JSONL, or
//! compares the teacher with the nominal-path floor on held-out episodes.
//!
//!     riftphys-uriel-seam --out seam.jsonl [--episodes 20] [--seed 1] [--seam line|ring]
//!                         [--joint butt|v|fillet] [--noise 0.05] [--sample-every 3] [--arc-deg 360]
//!                         [--operator "pause@6,resume@8,override@10:2:5,resume@12,stop@20"]
//!     riftphys-uriel-seam --eval [--episodes 10] [--seed 1000] [--seam line|ring] [--joint all]
//!
//! Both take `--armature isotropic|axis` (PARITY_SPEC 3.9): `isotropic`, the default, is the arm
//! URIEL trains on; `axis` carries the drives' reflected inertia on the joints' rotor rows (the
//! geared-motor model, at 96 velocity iterations), and its scenarios end in `_axis`.

use riftphys_uriel::protocol::Line;
use riftphys_uriel::seam::{parse_operator, track, JointType, SeamConfig, SeamRollout, SeamShape, TrackStats};
use std::io::{BufWriter, Write};

fn arg<T: std::str::FromStr>(args: &[String], name: &str, default: T) -> T {
    args.windows(2).find(|w| w[0] == name).map(|w| w[1].parse().ok().expect("bad argument value")).unwrap_or(default)
}

fn summary(stats: &[TrackStats]) -> String {
    let scored: Vec<&TrackStats> = stats.iter().filter(|s| s.lat_mean_mm.is_finite()).collect();
    let n = scored.len().max(1) as f32;
    let mean = |f: &dyn Fn(&TrackStats) -> f32| scored.iter().map(|s| f(s)).sum::<f32>() / n;
    let done = stats.iter().filter(|s| s.result == "completed").count();
    format!("completed {}/{} | lateral mean {:.3} mm, p95 {:.3} mm | height mean {:.3} mm, p95 {:.3} mm",
        done, stats.len(), mean(&|s| s.lat_mean_mm), mean(&|s| s.lat_p95_mm), mean(&|s| s.h_mean_mm), mean(&|s| s.h_p95_mm))
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let eval = args.iter().any(|a| a == "--eval");
    let out: String = arg(&args, "--out", String::new());
    if out.is_empty() && !eval {
        eprintln!("usage: riftphys-uriel-seam --out FILE.jsonl [--episodes N] [--seed S] [--seam line|ring] [--joint butt|v|fillet] [--noise X] [--sample-every K] [--arc-deg D] [--operator SCRIPT] [--armature isotropic|axis]\n       riftphys-uriel-seam --eval [--episodes N] [--seed S] [--seam line|ring] [--joint butt|v|fillet|all] [--armature isotropic|axis]");
        std::process::exit(1);
    }
    let shape = SeamShape::parse(&arg(&args, "--seam", "line".to_string())).expect("--seam line|ring");
    let joint_arg: String = arg(&args, "--joint", if eval { "all".to_string() } else { "v".to_string() });
    let joints: Vec<JointType> = if joint_arg == "all" { vec![JointType::Butt, JointType::V, JointType::Fillet] }
        else { vec![JointType::parse(&joint_arg).expect("--joint butt|v|fillet|all")] };
    let mut base = SeamConfig { shape, ..Default::default() };
    base.arm = riftphys_uriel::arm::ArmConfig::for_armature(match arg(&args, "--armature", "isotropic".to_string()).as_str() {
        "isotropic" => riftphys_uriel::arm::ArmArmature::Isotropic,
        "axis" => riftphys_uriel::arm::ArmArmature::Axis,
        other => panic!("--armature isotropic|axis, not {}", other),
    });
    base.ring_arc = arg(&args, "--arc-deg", 360.0f32).to_radians();

    if eval {
        let episodes: u64 = arg(&args, "--episodes", 10);
        let seed: u64 = arg(&args, "--seed", 1000);
        for joint in joints {
            let cfg = SeamConfig { joint, ..base };
            let teacher: Vec<TrackStats> = (0..episodes).map(|e| track(&cfg, e, seed, true)).collect();
            let floor: Vec<TrackStats> = (0..episodes).map(|e| track(&cfg, e, seed, false)).collect();
            println!("{} | teacher: {}", cfg.scenario(), summary(&teacher));
            println!("{} | floor:   {}", cfg.scenario(), summary(&floor));
        }
        println!("W6 bounce fallbacks to sweeping: {}, largest component {} rows", riftphys_world::world::solver::bounce_fallbacks_total(), riftphys_world::world::solver::bounce_largest_component());
        return;
    }

    let episodes: u64 = arg(&args, "--episodes", 20);
    let seed: u64 = arg(&args, "--seed", 1);
    let mut r = SeamRollout::new(SeamConfig { joint: joints[0], ..base });
    r.noise = arg(&args, "--noise", r.noise);
    r.sample_every = arg(&args, "--sample-every", r.sample_every);
    r.operator = parse_operator(&arg(&args, "--operator", String::new())).unwrap_or_else(|e| {
        eprintln!("--operator: {}", e);
        std::process::exit(1)
    });

    let mut w = BufWriter::new(std::fs::File::create(&out).expect("cannot create output"));
    writeln!(w, "{}", serde_json::to_string(&r.meta_line(seed)).unwrap()).unwrap();
    let mut stats = Vec::new();
    let mut results = std::collections::BTreeMap::<String, u32>::new();
    for e in 0..episodes {
        let (lines, st) = r.run_episode(e, seed);
        for line in &lines {
            if let Line::EpisodeEnd { result, .. } = line { *results.entry(result.clone()).or_default() += 1; }
            writeln!(w, "{}", serde_json::to_string(line).unwrap()).unwrap();
        }
        stats.push(st);
    }
    w.flush().unwrap();
    println!("{} episodes of {} -> {} | results {:?}", episodes, r.cfg.scenario(), out, results);
    println!("{}", summary(&stats));
}
