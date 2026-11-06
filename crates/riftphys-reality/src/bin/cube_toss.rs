//! RiftPhys on real cube tosses (ContactNets), scored as Acosta, Yang & Posa scored Drake, MuJoCo
//! and Bullet (RA-L 2022, Table V).
//!
//!     riftphys-cube-toss --log cube_tosses.jsonl [--hz 1480] [--fit] [--evals 120] [--seed 1]
//!                        [--out results.json]
//!
//! Without --fit: the cube's measured parameters (restitution 0.125, friction 0.18), all tosses.
//! With --fit: CMA-ES on the even-numbered tosses, scored on the odd ones (held out) and on all.

use riftphys_reality::cube_toss::{fit, replay_all, summary, table_height, tosses, Params, TossError, PUBLISHED};
use riftphys_reality::log::RealityLog;

fn row(name: &str, s: &[(f32, f32); 3]) -> String {
    format!("{:<34}{:>7.1} +-{:>5.1}{:>8.1} +-{:>5.1}{:>7.2} +-{:>5.2}", name, s[0].0, s[0].1, s[1].0, s[1].1, s[2].0, s[2].1)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let get = |n: &str| args.windows(2).find(|w| w[0] == n).map(|w| w[1].clone());
    let log = RealityLog::load(&get("--log").expect("--log FILE.jsonl")).unwrap_or_else(|e| panic!("{}", e));
    let hz: f32 = get("--hz").map(|v| v.parse().unwrap()).unwrap_or(riftphys_reality::cube_toss::SIM_HZ);
    let ts = tosses(&log).unwrap_or_else(|e| panic!("{}", e));
    let table_y = table_height(&ts);
    println!("{} tosses, {} frames, table at y = {:.4} m (median rest height less a half-width), sim {} Hz",
        ts.len(), ts.iter().map(|t| t.pos.len()).sum::<usize>(), table_y, hz);

    let measured = Params::measured();
    let t0 = std::time::Instant::now();
    let base = replay_all(&ts, &measured, table_y, hz);
    let mut out = serde_json::json!({ "tosses": ts.len(), "table_y": table_y, "sim_hz": hz,
        "measured": { "params": [measured.restitution, measured.mu_s, measured.mu_k], "all": summary(&base) } });

    println!("\n{:<34}{:>16}{:>16}{:>15}", "", "position % width", "rotation deg", "e_q");
    for (n, pm, ps, rm, rs, em, es) in PUBLISHED {
        println!("{}", row(&format!("{} (published, fitted)", n), &[(pm, ps), (rm, rs), (em, es)]));
    }
    println!("{}", row("RiftPhys, measured params, all", &summary(&base)));
    println!("  ({:.1} s for {} tosses)", t0.elapsed().as_secs_f32(), ts.len());

    if args.iter().any(|a| a == "--fit") {
        let evals: usize = get("--evals").map(|v| v.parse().unwrap()).unwrap_or(120);
        let seed: u64 = get("--seed").map(|v| v.parse().unwrap()).unwrap_or(1);
        let train: Vec<_> = ts.iter().filter(|t| t.index % 2 == 0).cloned().collect();
        let test: Vec<_> = ts.iter().filter(|t| t.index % 2 == 1).cloned().collect();
        let t1 = std::time::Instant::now();
        let (p, used) = fit(&train, &measured, table_y, hz, evals, seed);
        let pick = |es: &[TossError], set: &[riftphys_reality::cube_toss::Toss]| -> Vec<TossError> {
            let ids: std::collections::BTreeSet<u32> = set.iter().map(|t| t.index).collect();
            ts.iter().zip(es).filter(|(t, _)| ids.contains(&t.index)).map(|(_, e)| *e).collect()
        };
        let fitted = replay_all(&ts, &p, table_y, hz);
        println!("\nfit on {} even tosses: {} evaluations in {:.0} s -> restitution {:.3}, mu_s {:.3}, mu_k {:.3}",
            train.len(), used, t1.elapsed().as_secs_f32(), p.restitution, p.mu_s, p.mu_k);
        println!("{}", row("RiftPhys, measured, held-out", &summary(&pick(&base, &test))));
        println!("{}", row("RiftPhys, fitted, held-out", &summary(&pick(&fitted, &test))));
        println!("{}", row("RiftPhys, fitted, all", &summary(&fitted)));
        out["fitted"] = serde_json::json!({ "params": [p.restitution, p.mu_s, p.mu_k], "evals": used,
            "held_out": summary(&pick(&fitted, &test)), "all": summary(&fitted),
            "measured_held_out": summary(&pick(&base, &test)) });
    }
    if let Some(path) = get("--out") {
        std::fs::write(&path, serde_json::to_string_pretty(&out).unwrap()).unwrap();
        println!("\nwrote {}", path);
    }
}
