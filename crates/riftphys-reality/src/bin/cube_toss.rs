//! RiftPhys on real cube tosses (ContactNets), scored as Acosta, Yang & Posa scored Drake, MuJoCo
//! and Bullet (RA-L 2022, Table V).
//!
//!     riftphys-cube-toss --log cube_tosses.jsonl [--hz 1480] [--fit [--compliant] [--patch]] [--evals 120]
//!                        [--seed 1] [--out results.json]
//!                        [--diagnose [--params e,mu_s,mu_k[,k,d]] [--show TOSS]]
//!
//! Without --fit: the cube's measured parameters (restitution 0.125, friction 0.18), all tosses.
//! With --fit: CMA-ES on the even-numbered tosses, scored on the odd ones (held out) and on all;
//! --compliant fits Hunt-Crossley stiffness and dissipation with friction (rigid: restitution and
//! friction); --patch turns on patch friction (also for --diagnose).

use riftphys_reality::cube_toss::{fit, replay_all, summary, table_height, tosses, Model, Params, Toss, TossError, PUBLISHED};
use riftphys_reality::log::RealityLog;

fn row(name: &str, s: &[(f32, f32); 3]) -> String {
    format!("{:<34}{:>7.1} +-{:>5.1}{:>8.1} +-{:>5.1}{:>7.2} +-{:>5.2}", name, s[0].0, s[0].1, s[1].0, s[1].1, s[2].0, s[2].1)
}

/// Where replays go wrong: flight before the first contact, the impact, sliding after it.
fn diagnose(ts: &[Toss], p: &Params, table_y: f32, hz: f32) {
    use riftphys_reality::cube_toss::{clearance, pose_error, replay, trace, HALF};
    let first_contact = |poses: &[(riftphys_core::Vec3, riftphys_core::Quat)]| poses.iter().position(|(q, r)| clearance(*q, *r, table_y) < 0.002);
    let mean = |v: &[f32]| if v.is_empty() { f32::NAN } else { v.iter().sum::<f32>() / v.len() as f32 };
    let (mut pre, mut dk, mut slide_r, mut slide_s, mut slide_err, mut rest_p, mut rest_r) = (vec![], vec![], vec![], vec![], vec![], vec![], vec![]);
    let buckets = [0.05f64, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.7];
    let mut by_t: Vec<Vec<f32>> = vec![vec![]; buckets.len()];
    let mut worst: Vec<(f32, u32, f32, f32)> = vec![];
    let mut rot_t: Vec<Vec<f32>> = vec![vec![]; buckets.len()];
    // At rest: which face is up (real vs sim), and for matching faces the yaw left over
    let (mut same_face, mut yaw_same, mut rot_diff) = (0usize, vec![], vec![]);
    let face_up = |q: riftphys_core::Quat| {
        let up = q.conjugate() * riftphys_core::Vec3::Y;
        let a = up.abs();
        if a.x >= a.y && a.x >= a.z { if up.x > 0.0 { 0 } else { 1 } } else if a.y >= a.z { if up.y > 0.0 { 2 } else { 3 } } else if up.z > 0.0 { 4 } else { 5 }
    };
    for t in ts {
        let sim = trace(t, p, table_y, hz);
        let real: Vec<_> = t.pos.iter().zip(&t.rot).map(|(a, b)| (*a, *b)).collect();
        for (i, b) in buckets.iter().enumerate() {
            let k = (b / t.dt).round() as usize;
            if k < sim.len() {
                let (ep, er) = pose_error(sim[k].0, sim[k].1, real[k].0, real[k].1);
                by_t[i].push(1000.0 * ep);
                rot_t[i].push(er.to_degrees());
            }
        }
        let (qs, qr) = (sim[sim.len() - 1].1, real[real.len() - 1].1);
        let ang = pose_error(riftphys_core::Vec3::ZERO, qs, riftphys_core::Vec3::ZERO, qr).1.to_degrees();
        if face_up(qs) == face_up(qr) { same_face += 1; yaw_same.push(ang); } else { rot_diff.push(ang); }
        let (Some(kr), Some(ks)) = (first_contact(&real), first_contact(&sim)) else { continue };
        if kr > 0 { pre.push(1000.0 * pose_error(sim[kr - 1].0, sim[kr - 1].1, real[kr - 1].0, real[kr - 1].1).0); }
        dk.push(ks as f32 - kr as f32);
        let flat = |v: riftphys_core::Vec3| riftphys_core::Vec3::new(v.x, 0.0, v.z);
        let (last_r, last_s) = (real.len() - 1, sim.len() - 1);
        let sr = flat(real[last_r].0 - real[kr].0);
        let ss = flat(sim[last_s].0 - sim[ks].0);
        slide_r.push(1000.0 * sr.length());
        slide_s.push(1000.0 * ss.length());
        slide_err.push(1000.0 * (ss - sr).length());
        let (ep, er) = pose_error(sim[last_s].0, sim[last_s].1, real[last_r].0, real[last_r].1);
        rest_p.push(1000.0 * ep);
        rest_r.push(er.to_degrees());
        let e = replay(t, p, table_y, hz);
        worst.push((e.e_q, t.index, 1000.0 * flat(real[kr].0 - real[0].0).length(), (real.len() as f64 * t.dt) as f32));
    }
    println!("\ndiagnosis with restitution {:.3}, mu_s {:.3}, mu_k {:.3}, stiffness {:.0}, dissipation {:.3}:", p.restitution, p.mu_s, p.mu_k, p.stiffness, p.dissipation);
    println!("  position error just before the real first contact: mean {:.1} mm (flight: start state, gravity)", mean(&pre));
    let early = dk.iter().filter(|d| **d < -1.0).count();
    let late = dk.iter().filter(|d| **d > 1.0).count();
    println!("  first contact, sim minus real: mean {:+.2} frames; {} early, {} late by more than one frame (of {})", mean(&dk), early, late, dk.len());
    println!("  horizontal travel from first contact to rest: real {:.1} mm, sim {:.1} mm; difference {:.1} mm", mean(&slide_r), mean(&slide_s), mean(&slide_err));
    println!("  final pose error: {:.1} mm, {:.1} deg (cube width {:.1} mm)", mean(&rest_p), mean(&rest_r), 2000.0 * HALF);
    let cells: Vec<String> = buckets.iter().zip(&by_t).map(|(b, v)| format!("{:.2}s {:>5.1}", b, mean(v))).collect();
    println!("  mean position error (mm) at time since release: {}", cells.join(" | "));
    let cells: Vec<String> = buckets.iter().zip(&rot_t).map(|(b, v)| format!("{:.2}s {:>5.1}", b, mean(v))).collect();
    println!("  mean rotation error (deg) at time since release: {}", cells.join(" | "));
    let n = ts.len();
    let mut ys = yaw_same.clone();
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let med = |v: &[f32]| if v.is_empty() { f32::NAN } else { v[v.len() / 2] };
    println!("  at rest: same face up in {}/{} ({:.0}%); their leftover rotation (yaw) mean {:.1} deg, median {:.1}; other face: {} tosses, mean {:.1} deg",
        same_face, n, 100.0 * same_face as f32 / n as f32, mean(&yaw_same), med(&ys), rot_diff.len(), mean(&rot_diff));
    worst.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
    println!("  worst tosses (e_q, toss, horizontal flight mm, length s): {}",
        worst.iter().take(8).map(|w| format!("{:.2} #{} {:.0}mm {:.2}s", w.0, w.1, w.2, w.3)).collect::<Vec<_>>().join("; "));
}

/// One toss frame by frame: real and simulated centre height, lowest-corner clearance, horizontal
/// speed and spin (finite differences), and the error.
fn show(t: &Toss, p: &Params, table_y: f32, hz: f32) {
    use riftphys_reality::cube_toss::{clearance, pose_error, trace};
    let sim = trace(t, p, table_y, hz);
    let rate = |a: &[(riftphys_core::Vec3, riftphys_core::Quat)], k: usize| -> (f32, f32) {
        if k == 0 { return (0.0, 0.0); }
        let d = a[k].0 - a[k - 1].0;
        let mut q = a[k].1 * a[k - 1].1.conjugate();
        if q.w < 0.0 { q = -q; }
        (((d.x * d.x + d.z * d.z).sqrt() / t.dt as f32), 2.0 * riftphys_core::Vec3::new(q.x, q.y, q.z).length().min(1.0).asin() / t.dt as f32)
    };
    let real: Vec<_> = t.pos.iter().zip(&t.rot).map(|(a, b)| (*a, *b)).collect();
    println!("\ntoss #{}: frame  | real: clear mm  vxz m/s  spin rad/s | sim: clear mm  vxz m/s  spin rad/s | err mm  deg", t.index);
    for k in (0..real.len()).step_by(3) {
        let (rv, rw) = rate(&real, k);
        let (sv, sw) = rate(&sim, k);
        let (ep, er) = pose_error(sim[k].0, sim[k].1, real[k].0, real[k].1);
        println!("  {:>4} {:>5.3}s | {:>8.1} {:>8.2} {:>8.1} | {:>8.1} {:>8.2} {:>8.1} | {:>6.1} {:>5.1}", k, k as f64 * t.dt,
            1000.0 * clearance(real[k].0, real[k].1, table_y), rv, rw, 1000.0 * clearance(sim[k].0, sim[k].1, table_y), sv, sw, 1000.0 * ep, er.to_degrees());
    }
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
        let model = if args.iter().any(|a| a == "--compliant") { Model::Compliant } else { Model::Rigid };
        let start = if model == Model::Compliant { Params::compliant_start() } else { measured };
        let start = Params { patch_friction: args.iter().any(|a| a == "--patch"), ..start };
        let (p, used) = fit(&train, model, &start, table_y, hz, evals, seed);
        let pick = |es: &[TossError], set: &[riftphys_reality::cube_toss::Toss]| -> Vec<TossError> {
            let ids: std::collections::BTreeSet<u32> = set.iter().map(|t| t.index).collect();
            ts.iter().zip(es).filter(|(t, _)| ids.contains(&t.index)).map(|(_, e)| *e).collect()
        };
        let fitted = replay_all(&ts, &p, table_y, hz);
        println!("\n{:?} fit{} on {} even tosses: {} evaluations in {:.0} s -> restitution {:.3}, mu_s {:.3}, mu_k {:.3}, stiffness {:.0} N/m, dissipation {:.3} s/m",
            model, if p.patch_friction { " (patch friction)" } else { "" }, train.len(), used, t1.elapsed().as_secs_f32(), p.restitution, p.mu_s, p.mu_k, p.stiffness, p.dissipation);
        println!("{}", row("RiftPhys, measured, held-out", &summary(&pick(&base, &test))));
        println!("{}", row("RiftPhys, fitted, held-out", &summary(&pick(&fitted, &test))));
        println!("{}", row("RiftPhys, fitted, all", &summary(&fitted)));
        out["fitted"] = serde_json::json!({ "model": format!("{:?}", model), "patch_friction": p.patch_friction, "params": [p.restitution, p.mu_s, p.mu_k, p.stiffness, p.dissipation], "evals": used,
            "held_out": summary(&pick(&fitted, &test)), "all": summary(&fitted),
            "measured_held_out": summary(&pick(&base, &test)) });
    }
    if args.iter().any(|a| a == "--diagnose") {
        let p = match get("--params") {
            Some(s) => {
                let v: Vec<f32> = s.split(',').map(|x| x.parse().unwrap()).collect();
                Params { restitution: v[0], mu_s: v[1], mu_k: v[2], stiffness: *v.get(3).unwrap_or(&0.0), dissipation: *v.get(4).unwrap_or(&0.0),
                         patch_friction: args.iter().any(|a| a == "--patch") }
            }
            None => measured,
        };
        diagnose(&ts, &p, table_y, hz);
        if let Some(n) = get("--show") {
            let n: u32 = n.parse().unwrap();
            let t = ts.iter().find(|t| t.index == n).expect("no such toss");
            show(t, &p, table_y, hz);
        }
    }
    if let Some(path) = get("--out") {
        std::fs::write(&path, serde_json::to_string_pretty(&out).unwrap()).unwrap();
        println!("\nwrote {}", path);
    }
}
