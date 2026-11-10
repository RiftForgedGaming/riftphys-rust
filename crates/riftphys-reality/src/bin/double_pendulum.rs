//! RiftPhys on a real double pendulum (PARITY_SPEC W3, P3).
//!
//!     riftphys-double-pendulum --log double_pendulum.jsonl [--hz 240] [--iters 8,6]
//!                              [--params c1,f1,c2,f2] [--fit [--evals 200] [--seed 1] [--fit-horizons 4]]
//!                              [--dump errors.csv] [--energy energy.csv] [--half-step-start]
//!
//! Without --fit: the given damping (default the fit's start), all windows. With --fit: CMA-ES on
//! trial 2, scored on trials 1 and 3 (held out) and on all. --dump writes each window's error per
//! horizon (radians) for the final parameters; --energy a 60 s free run from each trial's first
//! window; tools/parity/double_pendulum_compare.py reads both.

use riftphys_reality::double_pendulum::{energy_run, fit, load, summary, window_error, Params, Rig, Sim, Window, FIT_TRIAL, HORIZONS};
use riftphys_reality::log::RealityLog;

fn arg(args: &[String], name: &str) -> Option<String> {
    args.windows(2).find(|w| w[0] == name).map(|w| w[1].clone())
}

fn nums(s: &str) -> Vec<f32> {
    s.split(',').map(|x| x.trim().parse().expect("bad number")).collect()
}

fn report(name: &str, rig: &Rig, p: &Params, ws: &[Window], sim: &Sim) {
    let errs: Vec<[f32; 4]> = ws.iter().map(|w| window_error(rig, p, w, sim)).collect();
    let s = summary(&errs);
    let cells: Vec<String> = HORIZONS.iter().zip(&s).map(|(h, (m, se))| format!("{:.2} s {:6.2} +-{:5.2}", h, m, se)).collect();
    println!("{:<26} {} windows | deg: {}", name, ws.len(), cells.join(" | "));
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = arg(&args, "--log").unwrap_or_else(|| {
        eprintln!("usage: riftphys-double-pendulum --log FILE.jsonl [--hz 240] [--iters 8,6] [--params c1,f1,c2,f2] [--fit [--evals N] [--seed S]]");
        std::process::exit(1)
    });
    let log = RealityLog::parse(&std::fs::read_to_string(&path).expect("cannot read log")).expect("bad log");
    let (rig, windows) = load(&log).expect("bad log");
    let hz: f32 = arg(&args, "--hz").map_or(riftphys_reality::double_pendulum::SIM_HZ, |x| x.parse().expect("--hz"));
    let it = arg(&args, "--iters").map_or(vec![8.0, 6.0], |x| nums(&x));
    let iters = (it[0] as u32, it[1] as u32);
    let mut p = arg(&args, "--params").map_or(Params::start(), |x| {
        let v = nums(&x);
        Params { c: [v[0], v[2]], f: [v[1], v[3]] }
    });
    println!("rig {:?}", rig);
    let train: Vec<Window> = windows.iter().filter(|w| w.trial == FIT_TRIAL).cloned().collect();
    let held: Vec<Window> = windows.iter().filter(|w| w.trial != FIT_TRIAL).cloned().collect();
    let sim = Sim { hz, iters, half_step_start: args.iter().any(|a| a == "--half-step-start") };
    println!("{} Hz, iterations {:?}, half-step start {}", hz, iters, sim.half_step_start);
    report("start: all", &rig, &p, &windows, &sim);
    if args.iter().any(|a| a == "--fit") {
        let evals: usize = arg(&args, "--evals").map_or(200, |x| x.parse().expect("--evals"));
        let seed: u64 = arg(&args, "--seed").map_or(1, |x| x.parse().expect("--seed"));
        let horizons: usize = arg(&args, "--fit-horizons").map_or(4, |x| x.parse().expect("--fit-horizons"));
        let (fitted, n, objective) = fit(&rig, &train, &p, &sim, horizons, evals, seed);
        p = fitted;
        println!("fitted after {} evaluations: c {:?}, f {:?}; training objective {:.6}", n, p.c, p.f, objective);
        report("fitted: trial 2 (train)", &rig, &p, &train, &sim);
        report("fitted: trials 1, 3 (held)", &rig, &p, &held, &sim);
        report("fitted: all", &rig, &p, &windows, &sim);
    }
    println!("params {},{},{},{}", p.c[0], p.f[0], p.c[1], p.f[1]);
    if let Some(path) = arg(&args, "--dump") {
        let mut text = String::from("trial,time,e0,e1,e2,e3\n");
        for w in &windows {
            let e = window_error(&rig, &p, w, &sim);
            text += &format!("{},{},{},{},{},{}\n", w.trial, w.time, e[0], e[1], e[2], e[3]);
        }
        std::fs::write(path, text).expect("cannot write --dump");
    }
    if let Some(path) = arg(&args, "--energy") {
        let mut text = String::from("trial,t,energy\n");
        for trial in 1..=3u32 {
            let Some(w) = windows.iter().find(|w| w.trial == trial) else { continue };
            for (t, e) in energy_run(&rig, &p, w, 60.0, &sim) { text += &format!("{},{},{}\n", trial, t, e); }
        }
        std::fs::write(path, text).expect("cannot write --energy");
    }
}
