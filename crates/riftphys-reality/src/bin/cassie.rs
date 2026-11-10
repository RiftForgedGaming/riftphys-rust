//! Cassie's pre-fit checks (PARITY_SPEC 3.4a):
//!   riftphys-cassie --check     rod length and rate, toe heights, at each trajectory's first sample
//!   riftphys-cassie --pluck     the leaf springs' frequencies on the built model, hung at 2 kHz
use riftphys_articulation::bodies::{D6Joint, D6Limit};
use riftphys_core::{Isometry, Quat, Vec3};
use riftphys_reality::cassie::{self, Params, ACHILLES_LENGTH, ACTUATORS, ALL_JUMPS, PAPER_JUMPS};

fn check() -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    let p = Params::default();
    println!("traj paper | rod L, R - 0.5012 (mm) | rod rate L, R (m/s) | toe heights - r (mm): LF LR RF RR | pelvis vz (m/s) | first toe contact, ballistic (ms)");
    let (mut worst_rod, mut worst_rate) = (0.0f64, 0.0f64);
    let mut first = Vec::new();
    for i in 0..ALL_JUMPS {
        let t = cassie::load_traj(&cassie::data_dir(), i)?;
        let c = cassie::build(&urdf, &t.x[0], &p, false)?;
        let rod = c.rod_lengths();
        let rate = c.rod_rates();
        let toes = c.toe_heights();
        let vz = c.toe_vz();
        // Free fall from each toe's height and vertical speed: the earliest arrival
        let arrive = (0..4).map(|k| {
            let (h, v, g) = (toes[k] - p.toe_r as f64, -vz[k], 9.81);
            (-v + (v * v + 2.0 * g * h).sqrt()) / g
        }).fold(f64::MAX, f64::min);
        first.push(arrive);
        worst_rod = worst_rod.max((rod[0] - ACHILLES_LENGTH).abs()).max((rod[1] - ACHILLES_LENGTH).abs());
        worst_rate = worst_rate.max(rate[0].abs()).max(rate[1].abs());
        println!("{:4} {:5} | {:+7.2} {:+7.2} | {:+7.3} {:+7.3} | {:+6.1} {:+6.1} {:+6.1} {:+6.1} | {:+.3} | {:5.1}",
                 i, if PAPER_JUMPS.contains(&i) { "yes" } else { "" },
                 (rod[0] - ACHILLES_LENGTH) * 1e3, (rod[1] - ACHILLES_LENGTH) * 1e3, rate[0], rate[1],
                 (toes[0] - p.toe_r as f64) * 1e3, (toes[1] - p.toe_r as f64) * 1e3, (toes[2] - p.toe_r as f64) * 1e3, (toes[3] - p.toe_r as f64) * 1e3,
                 t.x[0][28], first.last().unwrap() * 1e3);
    }
    println!("worst rod error {:.2} mm, worst rod rate {:.3} m/s", worst_rod * 1e3, worst_rate);
    let paper: Vec<f64> = PAPER_JUMPS.iter().map(|&i| first[i] * 1e3).collect();
    println!("first toe contact, paper's 22: {:.1} to {:.1} ms", paper.iter().cloned().fold(f64::MAX, f64::min), paper.iter().cloned().fold(0.0, f64::max));
    Ok(())
}

/// Dominant frequency (Hz) of a signal sampled at `hz`, its mean removed, by a direct DFT over
/// 1-400 Hz in 0.5 Hz steps.
fn dominant(x: &[f64], hz: f64) -> f64 { dominant_from(x, hz, 3.0) }

fn dominant_from(x: &[f64], hz: f64, fmin: f64) -> f64 {
    // A linear trend removed (a slow drift otherwise wins the lowest bin)
    let n = x.len() as f64;
    let (mx, mk) = (x.iter().sum::<f64>() / n, (n - 1.0) / 2.0);
    let slope = x.iter().enumerate().map(|(k, v)| (k as f64 - mk) * (v - mx)).sum::<f64>() / x.iter().enumerate().map(|(k, _)| (k as f64 - mk).powi(2)).sum::<f64>();
    let x: Vec<f64> = x.iter().enumerate().map(|(k, v)| v - mx - slope * (k as f64 - mk)).collect();
    let mean = 0.0;
    let mut best = (0.0, 0.0);
    let mut f = fmin;
    while f <= 400.0 {
        let (mut re, mut im) = (0.0, 0.0);
        for (k, v) in x.iter().enumerate() {
            let a = 2.0 * std::f64::consts::PI * f * k as f64 / hz;
            re += (v - mean) * a.cos();
            im += (v - mean) * a.sin();
        }
        let pw = re * re + im * im;
        if pw > best.1 { best = (f, pw); }
        f += 0.5;
    }
    best.0
}

/// Locks the 10 actuated joints at their current angles: a limit with min = max (position and
/// velocity rows). Velocity drives at 0 held the rate, not the angle, and the hung leg crept
/// (the left knee 0.08 rad in 4 s), which a spectrum read as a 3 Hz mode.
fn lock_motors(c: &mut cassie::Cassie) {
    for name in ACTUATORS {
        let id = c.model.joints[name];
        let q = riftphys_reality::urdf::joint_angle(&c.world, &c.model, &c.urdf, name).unwrap() as f32;
        c.world.set_d6_limit_r(id, 0, Some(D6Limit { min: q, max: q }));
    }
}

fn pluck() -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    // Undamped (no joint damping, no rod damping), so the frequencies are the springs' own
    let p = Params { joint_damping: false, rod_damping: 0.0, ..Params::default() };
    let hz = 2000.0f64;
    for i in [0usize, 5, 10, 15, 20, 23] {
        let t = cassie::load_traj(&cassie::data_dir(), i)?;
        // Hung: the pelvis welded, at rest, from the trajectory's first pose
        let mut row = t.x[0];
        for v in row[23..45].iter_mut() { *v = 0.0; }
        let mut c = cassie::build(&urdf, &row, &p, true)?;
        // The motors locked in position, so only the springs and the loop move
        lock_motors(&mut c);
        let names = ["knee_joint_left", "ankle_spring_joint_left", "knee_joint_right", "ankle_spring_joint_right"];
        let mut series: Vec<Vec<f64>> = vec![Vec::new(); names.len()];
        let q0: Vec<f64> = ACTUATORS.iter().map(|n| riftphys_reality::urdf::joint_angle(&c.world, &c.model, &c.urdf, n).unwrap()).collect();
        for _ in 0..(0.5 * hz) as usize {
            c.world.step((1.0 / hz) as f32);
            for (s, n) in series.iter_mut().zip(names) { s.push(riftphys_reality::urdf::joint_angle(&c.world, &c.model, &c.urdf, n).unwrap()); }
        }
        let creep = ACTUATORS.iter().zip(&q0).map(|(n, a)| (riftphys_reality::urdf::joint_angle(&c.world, &c.model, &c.urdf, n).unwrap() - a).abs()).fold(0.0, f64::max);
        println!("traj {:2}: the locked motors crept at most {:.1e} rad", i, creep);
        print!("traj {:2}:", i);
        for (s, n) in series.iter().zip(names) {
            let f = dominant(s, hz);
            let w = 2.0 * std::f64::consts::PI * f;
            print!("  {} {:.1} Hz (omega dt {:.2} at 240 Hz, {:.3} at 2 kHz)", n, f, w / 240.0, w / 2000.0);
        }
        println!();
    }
    Ok(())
}

/// Trajectory `i`, hung as in `--pluck`, for `secs`: every joint's angle range (the mode shape),
/// its distance from its URDF limits, and the dominant frequency of each spring.
fn probe(i: usize, secs: f64) -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    let p = Params { joint_damping: false, rod_damping: 0.0, ..Params::default() };
    let hz = 2000.0f64;
    let t = cassie::load_traj(&cassie::data_dir(), i)?;
    let mut row = t.x[0];
    for v in row[23..45].iter_mut() { *v = 0.0; }
    let mut c = cassie::build(&urdf, &row, &p, true)?;
    lock_motors(&mut c);
    let names = riftphys_reality::cassie::JOINTS;
    let mut series: Vec<Vec<f64>> = vec![Vec::new(); names.len()];
    for _ in 0..(secs * hz) as usize {
        c.world.step((1.0 / hz) as f32);
        for (s, n) in series.iter_mut().zip(names) { s.push(riftphys_reality::urdf::joint_angle(&c.world, &c.model, &c.urdf, n).unwrap()); }
    }
    let half = series[0].len() / 2;
    for (s, n) in series.iter().zip(names) {
        let tail = &s[half..];
        let (lo, hi) = tail.iter().fold((f64::MAX, f64::MIN), |(a, b), v| (a.min(*v), b.max(*v)));
        let lim = c.urdf.joint(n).and_then(|j| j.limit);
        let margin = lim.map(|(l, h)| (tail.iter().cloned().fold(f64::MAX, f64::min) - l).min(h - tail.iter().cloned().fold(f64::MIN, f64::max)));
        println!("{:26} start {:+.4} last {:+.4} swing (2nd half) {:.2e} rad  limit margin {:?}  f {:.1} Hz",
                 n, s[0], s[s.len() - 1], hi - lo, margin.map(|m| (m * 1e3).round() / 1e3), dominant(tail, hz));
    }
    Ok(())
}

/// The loop-consistent ankle-spring rates beside an independent estimate: the slope of a line
/// through the recorded ankle-spring angles' first 6 samples (about 2.5 ms; the estimator's IK).
/// Reported, not gated.
fn rates() -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    let p = Params::default();
    println!("traj paper | ankle spring rate, loop-consistent / from the recorded angles (rad/s): left | right");
    let (mut sign, mut far) = (0, 0);
    for i in 0..ALL_JUMPS {
        let t = cassie::load_traj(&cassie::data_dir(), i)?;
        let (_, found) = cassie::loop_consistent(&urdf, &t.x[0], &p)?;
        let mut slope = [0.0f64; 2];
        for (k, col) in [19usize, 21].into_iter().enumerate() {
            let n = 6;
            let (tm, xm) = ((0..n).map(|j| t.t[j]).sum::<f64>() / n as f64, (0..n).map(|j| t.x[j][col]).sum::<f64>() / n as f64);
            slope[k] = (0..n).map(|j| (t.t[j] - tm) * (t.x[j][col] - xm)).sum::<f64>() / (0..n).map(|j| (t.t[j] - tm).powi(2)).sum::<f64>();
            if found[k].signum() != slope[k].signum() && found[k].abs().max(slope[k].abs()) > 0.2 { sign += 1; }
            if (found[k] - slope[k]).abs() > 1.0 { far += 1; }
        }
        println!("{:4} {:5} | {:+6.2} / {:+6.2} | {:+6.2} / {:+6.2}", i, if PAPER_JUMPS.contains(&i) { "yes" } else { "" }, found[0], slope[0], found[1], slope[1]);
    }
    println!("of 58 legs: {} differ in sign (either above 0.2 rad/s), {} by more than 1 rad/s", sign, far);
    Ok(())
}

/// The leaf springs standing: Cassie at a trajectory's last sample (feet down), at rest, motors
/// locked, on the compliant ground, undamped; the springs ring as it settles into its load.
fn stance(secs: f64) -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    let p = Params { joint_damping: false, rod_damping: 0.0, ..Params::default() };
    let hz = 2000.0f64;
    for i in [0usize, 5, 10, 15, 20, 23] {
        let t = cassie::load_traj(&cassie::data_dir(), i)?;
        let mut row = t.x[t.x.len() - 1];
        for v in row[23..45].iter_mut() { *v = 0.0; }
        let mut c = cassie::build(&urdf, &row, &p, false)?;
        lock_motors(&mut c);
        // A vertical guide: the pelvis may only move up and down (a D6 joint to the ground, all
        // rows locked but world-vertical translation). With the motors locked it can't balance on
        // four points, and unguided it toppled within 2 s, its pelvis falling through the floor
        let (pb, _) = c.model.links["pelvis"];
        let pid = c.model.bodies[pb].id;
        let pose = c.world.get_body_pose(pid);
        let mut d = D6Joint { a: c.ground, b: pid,
            fa: Isometry { pos: pose.pos - c.world.get_body_pose(c.ground).pos, rot: Quat::IDENTITY },
            fb: Isometry { pos: Vec3::ZERO, rot: pose.rot.conjugate() }, t: Default::default(), r: Default::default(), hinge_axis: None };
        for k in 0..3 { d.t[k].enabled = k != 1; d.r[k].enabled = true; }
        c.world.add_d6_joint(d);
        let names = ["knee_joint_left", "ankle_spring_joint_left", "knee_joint_right", "ankle_spring_joint_right"];
        let mut series: Vec<Vec<f64>> = vec![Vec::new(); names.len()];
        let mut low = f64::MAX;
        for _ in 0..(secs * hz) as usize {
            c.world.step((1.0 / hz) as f32);
            for (s, n) in series.iter_mut().zip(names) { s.push(riftphys_reality::urdf::joint_angle(&c.world, &c.model, &c.urdf, n).unwrap()); }
            low = low.min(c.toe_heights().iter().cloned().fold(f64::MAX, f64::min));
        }
        let pelvis = c.link_frame_urdf("pelvis").pos.z;
        print!("traj {:2} (guided; pelvis {:.3} m at the end, from {:.3}; deepest toe centre {:+.1} mm):", i, pelvis, row[6], low * 1e3);
        for (s, n) in series.iter().zip(names) {
            // The first 0.3 s (the landing into load) left out
            let ring = &s[(0.3 * hz) as usize..];
            let f = dominant_from(ring, hz, 1.5);
            let w = 2.0 * std::f64::consts::PI * f;
            let tail = &s[s.len() / 2..];
            print!("  {} {:.1} Hz (omega dt {:.2} at 240 Hz; settles at {:+.3} rad)", n, f, w / 240.0, tail.iter().sum::<f64>() / tail.len() as f64);
        }
        println!();
    }
    Ok(())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let r = match args.get(1).map(String::as_str) {
        Some("--check") => check(),
        Some("--pluck") => pluck(),
        Some("--rates") => rates(),
        Some("--stance") => stance(2.0),
        Some("--probe") => probe(args.get(2).and_then(|a| a.parse().ok()).unwrap_or(20), 4.0),
        _ => Err("usage: riftphys-cassie --check | --pluck | --stance | --rates | --probe [traj]".into()),
    };
    if let Err(e) = r { eprintln!("{}", e); std::process::exit(1); }
}
