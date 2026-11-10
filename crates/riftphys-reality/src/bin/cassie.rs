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

/// The loop-consistent ankle-spring rates beside an independent estimate, the slope at sample 0
/// of a quadratic through the recorded ankle-spring angles' first 8 samples (the estimator's IK),
/// before and after the lagged passive rates (knee springs, ankles) are taken from their
/// positions. Reported, not gated.
fn rates() -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    let p = Params::default();
    println!("traj paper | ankle spring rate (rad/s): IK slope | closure from recorded rates | closure from refit passive rates   (left; right)");
    let (mut before, mut after) = ([0usize; 2], [0usize; 2]);
    let (mut err_b, mut err_a) = (Vec::new(), Vec::new());
    for i in 0..ALL_JUMPS {
        let t = cassie::load_traj(&cassie::data_dir(), i)?;
        let (_, rec) = cassie::loop_consistent(&urdf, &t.x[0], &p)?;
        let (_, fit) = cassie::start_row(&urdf, &t, &p)?;
        let ik = [cassie::start_slope(&t, 19, cassie::START_FIT_SAMPLES), cassie::start_slope(&t, 21, cassie::START_FIT_SAMPLES)];
        for k in 0..2 {
            let tally = |c: &mut [usize; 2], v: f64, errs: &mut Vec<f64>| {
                if v.signum() != ik[k].signum() && v.abs().max(ik[k].abs()) > 0.2 { c[0] += 1; }
                if (v - ik[k]).abs() > 1.0 { c[1] += 1; }
                errs.push((v - ik[k]).abs());
            };
            tally(&mut before, rec[k], &mut err_b);
            tally(&mut after, fit[k], &mut err_a);
        }
        println!("{:4} {:5} | {:+6.2} {:+6.2} {:+6.2} ; {:+6.2} {:+6.2} {:+6.2}", i, if PAPER_JUMPS.contains(&i) { "yes" } else { "" },
                 ik[0], rec[0], fit[0], ik[1], rec[1], fit[1]);
    }
    let median = |v: &mut Vec<f64>| { v.sort_by(|a, b| a.partial_cmp(b).unwrap()); v[v.len() / 2] };
    println!("of 58 legs, against the IK slope: from recorded rates {} differ in sign, {} by > 1 rad/s, median |diff| {:.2}; from refit rates {} in sign, {} by > 1 rad/s, median |diff| {:.2}",
             before[0], before[1], median(&mut err_b), after[0], after[1], median(&mut err_a));
    Ok(())
}

/// The leaf springs standing: Cassie at a trajectory's last sample (feet down), at rest, motors
/// locked, on the compliant ground, undamped; the springs ring as it settles into its load.
fn stance(secs: f64, k: f32) -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    let p = Params { joint_damping: false, rod_damping: 0.0, k, ..Params::default() };
    println!("ground stiffness {} N/m per toe", k);
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
            // The leaf springs' own modes with the foot down, above the body's bounce
            let f5 = peak_above(ring, hz, 5.0).unwrap_or(f64::NAN);
            let (w, w5) = (2.0 * std::f64::consts::PI * f, 2.0 * std::f64::consts::PI * f5);
            let tail = &s[s.len() / 2..];
            print!("  {} {:.1} Hz (omega dt {:.2}), above 5 Hz {:.1} Hz (omega dt {:.2} at 240 Hz); settles at {:+.3} rad", n, f, w / 240.0, f5, w5 / 240.0, tail.iter().sum::<f64>() / tail.len() as f64);
        }
        println!();
    }
    Ok(())
}

/// The largest local peak of a Hann-windowed spectrum strictly above `fmin` (Hz), in 0.25 Hz
/// steps up to 200 Hz, with a linear trend removed: the leakage of a strong lower mode then ends
/// at the edge rather than winning it. None if no interior peak.
fn peak_above(x: &[f64], hz: f64, fmin: f64) -> Option<f64> {
    let n = x.len() as f64;
    let (mx, mk) = (x.iter().sum::<f64>() / n, (n - 1.0) / 2.0);
    let slope = x.iter().enumerate().map(|(k, v)| (k as f64 - mk) * (v - mx)).sum::<f64>() / x.iter().enumerate().map(|(k, _)| (k as f64 - mk).powi(2)).sum::<f64>();
    let y: Vec<f64> = x.iter().enumerate().map(|(k, v)| {
        let hann = 0.5 - 0.5 * (2.0 * std::f64::consts::PI * k as f64 / (n - 1.0)).cos();
        (v - mx - slope * (k as f64 - mk)) * hann
    }).collect();
    let power = |f: f64| {
        let (mut re, mut im) = (0.0, 0.0);
        for (k, v) in y.iter().enumerate() { let a = 2.0 * std::f64::consts::PI * f * k as f64 / hz; re += v * a.cos(); im += v * a.sin(); }
        re * re + im * im
    };
    let fs: Vec<f64> = (0..).map(|k| fmin + 0.25 * k as f64).take_while(|f| *f <= 200.0).collect();
    let p: Vec<f64> = fs.iter().map(|f| power(*f)).collect();
    (1..p.len() - 1).filter(|&k| p[k] > p[k - 1] && p[k] >= p[k + 1]).max_by(|&a, &b| p[a].partial_cmp(&p[b]).unwrap()).map(|k| fs[k])
}

/// The paper's 22 jumps replayed at `hz` on `line`, contact at Table VI's Drake values (not fitted):
/// each jump's loss and terms, the secondary (de-lagged) loss, first contact, and the trial step.
fn replay_all(hz: f64, line: cassie::Line) -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    let ankle_c: Option<f64> = std::env::var("CASSIE_ANKLE_C").ok().and_then(|v| v.parse().ok());
    // P4's iterations are the biped workload's (8, 6) (D9); CASSIE_ITERS[_POS] override them
    let iters: u32 = std::env::var("CASSIE_ITERS").ok().and_then(|v| v.parse().ok()).unwrap_or(8);
    let iters_pos: u32 = std::env::var("CASSIE_ITERS_POS").ok().and_then(|v| v.parse().ok()).unwrap_or(if std::env::var("CASSIE_ITERS").is_ok() { iters } else { 6 });
    let p = Params { ankle_spring_c: ankle_c, iters: (iters, iters_pos), ..Params::default() };
    println!("iterations ({}, {})", iters, iters_pos);
    println!("ankle-spring damping: {}", ankle_c.map_or("the URDF's 10".to_string(), |c| c.to_string()));
    let (mut t1, mut t2) = ([0.0f64; 4], [0.0f64; 4]);
    println!("{} Hz, {:?} line, unfitted (k {}, d {}, mu {})", hz, line, p.k, p.d, p.mu);
    println!("traj | loss (pos, vel, omega, quat) | de-lagged loss | first contact (ms) | trial: d ankle spring, d other max (rad/s), rod rate (m/s)");
    let (mut sum, mut sum2) = (0.0, 0.0);
    for &i in PAPER_JUMPS.iter() {
        let t = cassie::load_traj(&cassie::data_dir(), i)?;
        let r = cassie::replay(&urdf, &t, &p, hz, line)?;
        let terms = cassie::loss_terms(&r.states, &t.x);
        let l: f64 = terms.iter().sum();
        let terms2 = cassie::loss_terms(&r.states, &cassie::delag(&t));
        let l2: f64 = terms2.iter().sum();
        for k in 0..4 { t1[k] += terms[k] / 22.0; t2[k] += terms2[k] / 22.0; }
        sum += l;
        sum2 += l2;
        println!("{:4} | {:7.2} ({:6.2} {:6.2} {:5.2} {:5.2}) | {:7.2} | {:>6} | {:.3} {:.3} {:.4}", i, l, terms[0], terms[1], terms[2], terms[3], l2,
                 r.first_contact.map_or("none".to_string(), |f| format!("{:.1}", f * 1e3)), r.trial.0, r.trial.1, r.trial.2);
    }
    println!("mean over the 22: {:.2} (de-lagged {:.2})", sum / 22.0, sum2 / 22.0);
    println!("mean terms (pos, vel, omega, quat): paper {:.2} {:.2} {:.2} {:.2}; de-lagged {:.2} {:.2} {:.2} {:.2}", t1[0], t1[1], t1[2], t1[3], t2[0], t2[1], t2[2], t2[3]);
    Ok(())
}

/// One jump's velocity-term contributions by column (the largest first), at `hz`, main line.
fn terms(i: usize, hz: f64) -> Result<(), String> {
    const VEL_W: [f64; 19] = [100.0, 100.0, 100.0, 0.01, 0.01, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.01, 0.01, 1.0, 1.0, 0.0, 0.01, 0.0, 0.01];
    let names = ["pelvis vx", "pelvis vy", "pelvis vz"];
    let urdf = cassie::load_urdf()?;
    let iters: u32 = std::env::var("CASSIE_ITERS").ok().and_then(|v| v.parse().ok()).unwrap_or(20);
    let p = Params { iters: (iters, iters), ..Params::default() };
    let t = cassie::load_traj(&cassie::data_dir(), i)?;
    let r = cassie::replay(&urdf, &t, &p, hz, cassie::Line::Main)?;
    println!("{} iterations: loss {:.2}", iters, cassie::loss(&r.states, &t.x));
    let n = t.x.len() as f64;
    let mut c: Vec<(String, f64, f64)> = (26..45).enumerate().map(|(j, col)| {
        let name = if j < 3 { names[j].to_string() } else { cassie::JOINTS[j - 3].to_string() };
        let e = r.states.iter().zip(&t.x).map(|(a, b)| (a[col] - b[col]).powi(2)).sum::<f64>() / n;
        (name, VEL_W[j] * e, e.sqrt())
    }).collect();
    c.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    println!("traj {} at {} Hz, velocity term by column (weighted, RMS error):", i, hz);
    for (name, w, rms) in c.iter().take(8) { println!("  {:26} {:8.2}  rms {:.3}", name, w, rms); }
    // The pelvis vertical velocity, sampled
    for k in (0..t.x.len()).step_by(10) { println!("  t {:5.1} ms  vz sim {:+.3} data {:+.3}", (t.t[k] - t.t[0]) * 1e3, r.states[k][28], t.x[k][28]); }
    Ok(())
}

/// The loss floor of sampling alone: each jump's recorded states held only on a grid at `hz` and
/// interpolated linearly back to the sample times, scored against themselves.
fn floor(hz: f64) -> Result<(), String> {
    let mut sum = 0.0;
    for &i in PAPER_JUMPS.iter() {
        let t = cassie::load_traj(&cassie::data_dir(), i)?;
        let h = 1.0 / hz;
        let at = |x: f64| -> [f64; 45] {
            // The recorded state at time x (linear between samples)
            let k = t.t.partition_point(|&s| s <= x).clamp(1, t.t.len() - 1);
            let s = ((x - t.t[k - 1]) / (t.t[k] - t.t[k - 1])).clamp(0.0, 1.0);
            let mut r = [0.0; 45];
            for j in 0..45 { r[j] = t.x[k - 1][j] * (1.0 - s) + t.x[k][j] * s; }
            r
        };
        let held: Vec<[f64; 45]> = t.t.iter().map(|&tk| {
            let g = ((tk - t.t[0]) / h).floor();
            let (a, b) = (t.t[0] + g * h, t.t[0] + (g + 1.0) * h);
            let (xa, xb) = (at(a), at(b));
            let s = (tk - a) / h;
            let mut r = [0.0; 45];
            for j in 0..45 { r[j] = xa[j] * (1.0 - s) + xb[j] * s; }
            r
        }).collect();
        sum += cassie::loss(&held, &t.x);
    }
    println!("sampling floor at {} Hz: mean loss {:.2} over the 22", hz, sum / 22.0);
    Ok(())
}

/// Trajectory 0's loss at `hz` with the half-step correction off, from the replay's own trial
/// step, and from a 2 kHz trial scaled to the step; and the trial steps' rate changes, 240 Hz
/// over 2 kHz, per coordinate (smooth dynamics give h ratio 8.33).
fn correction(i: usize, hz: f64) -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    let p = Params::default();
    let t = cassie::load_traj(&cassie::data_dir(), i)?;
    for (name, c) in [("off", cassie::Correction::Off), ("own trial", cassie::Correction::Trial), ("2 kHz trial, scaled", cassie::Correction::Fine(1.0 / 2000.0))] {
        let r = cassie::replay_with(&urdf, &t, &p, hz, cassie::Line::Main, c)?;
        println!("traj {} at {} Hz, correction {:22}: loss {:.2}", i, hz, name, cassie::loss(&r.states, &t.x));
    }
    let (lc, _) = cassie::start_row(&urdf, &t, &p)?;
    let (_, coarse) = cassie::trial_step(&urdf, &t, &p, &lc, 1.0 / 240.0)?;
    let (_, fine) = cassie::trial_step(&urdf, &t, &p, &lc, 1.0 / 2000.0)?;
    println!("trial-step rate change, 240 Hz / 2 kHz (smooth: 8.33):");
    let names: Vec<String> = ["pelvis wx", "pelvis wy", "pelvis wz", "pelvis vx", "pelvis vy", "pelvis vz"].iter().map(|s| s.to_string()).chain(cassie::JOINTS.iter().map(|s| s.to_string())).collect();
    for (k, col) in (23..45).enumerate() {
        let (dc, df) = (coarse[col] - lc[col], fine[col] - lc[col]);
        let ratio = if df.abs() > 1e-9 { dc / df } else { f64::NAN };
        let flag = if (ratio / 8.333 - 1.0).abs() > 0.25 { "  <- outside +-25%" } else { "" };
        println!("  {:26} 240 Hz {:+8.4}  2 kHz {:+8.4}  ratio {:6.2}{}", names[k], dc, df, ratio, flag);
    }
    Ok(())
}

/// Trajectory `i` at 240 Hz scored against RiftPhys's own 2 kHz run (same start, correction off,
/// same parameters), for the base model and one ingredient toggled at a time; and the base
/// case's error over time with first contact marked.
fn vs2k(i: usize) -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    let t = cassie::load_traj(&cassie::data_dir(), i)?;
    let base = Params::default();
    let cases: Vec<(&str, Params)> = vec![
        ("base", base),
        ("no torques", Params { inputs: false, ..base }),
        ("no armature", Params { armature: false, ..base }),
        ("leaf springs x100 stiffer", Params { knee_k: base.knee_k * 100.0, ankle_k: base.ankle_k * 100.0, ..base }),
        ("no joint damping", Params { joint_damping: false, ..base }),
        ("contact k x100", Params { k: base.k * 100.0, ..base }),
        ("80 iterations", Params { iters: (80, 80), ..base }),
    ];
    for (name, p) in &cases {
        let a = cassie::replay_with(&urdf, &t, p, 240.0, cassie::Line::Main, cassie::Correction::Off)?;
        let b = cassie::replay_with(&urdf, &t, p, 2000.0, cassie::Line::Main, cassie::Correction::Off)?;
        println!("traj {} {:28}: 240 Hz against 2 kHz {:8.2}; against the data: 240 Hz {:8.2}, 2 kHz {:7.2}; first contact {:?} / {:?} ms",
                 i, name, cassie::loss(&a.states, &b.states), cassie::loss(&a.states, &t.x), cassie::loss(&b.states, &t.x),
                 a.first_contact.map(|f| (f * 1e4).round() / 10.0), b.first_contact.map(|f| (f * 1e4).round() / 10.0));
        if *name == "base" {
            for k in (0..t.x.len()).step_by(5) {
                let e = cassie::loss(&a.states[k..k + 1], &b.states[k..k + 1]);
                let knee = (a.states[k][35] - b.states[k][35], a.states[k][37] - b.states[k][37]);
                println!("   t {:5.1} ms  per-sample loss {:8.2}  knee L rate diff {:+7.2}  knee spring L rate diff {:+7.2}", (t.t[k] - t.t[0]) * 1e3, e, knee.0, knee.1);
            }
        }
    }
    Ok(())
}

/// The de-lagged rates against the recorded rates shifted back by `lag` samples (and unshifted),
/// over the paper's 22, on samples 4..(n - lag - 4): RMS per lagged column.
fn delag_check(lag: usize) -> Result<(), String> {
    let mut acc = [[0.0f64; 2]; 4];
    let mut n = 0usize;
    for &i in PAPER_JUMPS.iter() {
        let t = cassie::load_traj(&cassie::data_dir(), i)?;
        let d = cassie::delag(&t);
        for k in 4..t.x.len() - lag - 4 {
            for (c, (_, rate)) in cassie::LAGGED_RATES.iter().enumerate() {
                acc[c][0] += (d[k][*rate] - t.x[k + lag][*rate]).powi(2);
                acc[c][1] += (d[k][*rate] - t.x[k][*rate]).powi(2);
            }
            n += 1;
        }
    }
    let names = ["knee spring L", "knee spring R", "ankle L", "ankle R"];
    for c in 0..4 {
        println!("{:14} de-lagged against recorded shifted {} samples: RMS {:.3} rad/s; unshifted: {:.3}", names[c], lag, (acc[c][0] / n as f64).sqrt(), (acc[c][1] / n as f64).sqrt());
    }
    Ok(())
}

/// The paper's 22 at 240 Hz against RiftPhys's own 2 kHz run (same start, the decided gated
/// correction at each rate, same parameters) at several iteration counts, with each run's loss
/// against the data and the mean energy change over 10, 25 and 50 ms (J).
fn iters_check() -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    println!("iterations | 240 Hz vs own 2 kHz | 240 Hz vs data | 2 kHz vs data | 240 Hz energy change at 10, 25, 50 ms | 2 kHz energy change");
    for it in [(8u32, 6u32), (20, 20), (80, 80), (320, 320)] {
        let p = Params { iters: it, ..Params::default() };
        let (mut own, mut d240, mut d2k) = (0.0, 0.0, 0.0);
        let (mut e240, mut e2k) = ([0.0f64; 3], [0.0f64; 3]);
        for &i in PAPER_JUMPS.iter() {
            let t = cassie::load_traj(&cassie::data_dir(), i)?;
            let a = cassie::replay(&urdf, &t, &p, 240.0, cassie::Line::Main)?;
            let b = cassie::replay(&urdf, &t, &p, 2000.0, cassie::Line::Main)?;
            own += cassie::loss(&a.states, &b.states) / 22.0;
            d240 += cassie::loss(&a.states, &t.x) / 22.0;
            d2k += cassie::loss(&b.states, &t.x) / 22.0;
            for k in 0..3 { e240[k] += (a.energy[k + 1] - a.energy[0]) / 22.0; e2k[k] += (b.energy[k + 1] - b.energy[0]) / 22.0; }
        }
        println!("{:?} | {:7.2} | {:7.2} | {:6.2} | {:+7.2} {:+7.2} {:+7.2} | {:+7.2} {:+7.2} {:+7.2}", it, own, d240, d2k, e240[0], e240[1], e240[2], e2k[0], e2k[1], e2k[2]);
    }
    Ok(())
}

/// The fit as stated before it ran (PARITY_SPEC 3.4a): per fold, k, d and mu fitted on the other
/// four folds at `hz` and (8, 6); each held-out jump's loss on the main and sensitivity lines; at
/// 240 Hz also the held-out loss at 20, 80 and 320 iterations with the fitted parameters held
/// fixed; and an all-22 in-sample fit beside it. Writes the per-jump results as JSON to `out`.
fn fit_all(hz: f64, evals: usize, out: &str) -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    let trajs: Vec<cassie::Traj> = PAPER_JUMPS.iter().map(|&i| cassie::load_traj(&cassie::data_dir(), i)).collect::<Result<_, _>>()?;
    let start = Params::default();
    let folds = cassie::folds(1);
    let curve: Vec<(u32, u32)> = if hz < 1000.0 { vec![(8, 6), (20, 20), (80, 80), (320, 320)] } else { vec![(8, 6)] };
    let mut rows = Vec::new();
    let mut fold_params = Vec::new();
    for (f, held) in folds.iter().enumerate() {
        let train: Vec<usize> = PAPER_JUMPS.iter().cloned().filter(|j| !held.contains(j)).collect();
        let (p, obj) = cassie::fit(&urdf, &trajs, &train, &start, hz, evals, 1);
        println!("fold {} (held out {:?}): k {:.0} N/m, d {:.3} s/m, mu {:.3}; training loss {:.2}", f + 1, held, p.k, p.d, p.mu, obj);
        fold_params.push(serde_json::json!({ "fold": f + 1, "held_out": held, "k": p.k, "d": p.d, "mu": p.mu, "train_loss": obj }));
        for &j in held {
            let t = trajs.iter().find(|t| t.index == j).unwrap();
            let main = cassie::replay(&urdf, t, &p, hz, cassie::Line::Main)?;
            let sens = cassie::replay(&urdf, t, &p, hz, cassie::Line::Sensitivity)?;
            let at: Vec<f64> = curve.iter().map(|&it| {
                let q = Params { iters: it, ..p };
                cassie::replay(&urdf, t, &q, hz, cassie::Line::Main).map(|r| cassie::loss(&r.states, &t.x)).unwrap_or(f64::NAN)
            }).collect();
            rows.push(serde_json::json!({ "jump": j, "fold": f + 1,
                "main": cassie::loss(&main.states, &t.x), "sensitivity": cassie::loss(&sens.states, &t.x),
                "main_delagged": cassie::loss(&main.states, &cassie::delag(t)), "terms": cassie::loss_terms(&main.states, &t.x),
                "first_contact_ms": main.first_contact.map(|x| x * 1e3), "iterations": curve.iter().map(|c| format!("{:?}", c)).collect::<Vec<_>>(), "by_iterations": at }));
        }
    }
    let mean = |key: &str| rows.iter().map(|r| r[key].as_f64().unwrap()).sum::<f64>() / rows.len() as f64;
    println!("{} Hz, held out, mean over the 22: main {:.2}, sensitivity {:.2}, de-lagged {:.2}", hz, mean("main"), mean("sensitivity"), mean("main_delagged"));
    if curve.len() > 1 {
        for (k, it) in curve.iter().enumerate() {
            let m = rows.iter().map(|r| r["by_iterations"][k].as_f64().unwrap()).sum::<f64>() / rows.len() as f64;
            println!("  held out at {:?} iterations (the 240 Hz parameters fixed): {:.2}", it, m);
        }
    }
    let (all, all_obj) = cassie::fit(&urdf, &trajs, &PAPER_JUMPS, &start, hz, evals, 1);
    println!("in-sample, all 22: k {:.0}, d {:.3}, mu {:.3}; loss {:.2}", all.k, all.d, all.mu, all_obj);
    let doc = serde_json::json!({ "hz": hz, "iterations": [8, 6], "evals": evals, "seed": 1, "folds": fold_params,
        "jumps": rows, "in_sample": { "k": all.k, "d": all.d, "mu": all.mu, "loss": all_obj } });
    std::fs::write(out, serde_json::to_string_pretty(&doc).unwrap()).map_err(|e| e.to_string())?;
    println!("wrote {}", out);
    Ok(())
}

/// The data states both engines start from (PARITY_SPEC 3.4e), for every public trajectory: the
/// loop-consistent start row (`cassie::start_row`), plus each trajectory's times, recorded states
/// and inputs; and RiftPhys's own main-line replay of trajectory 0 at 2 kHz, to check the other
/// engine's loss port against this one. Written as JSON to `out`.
fn export_starts(out: &str) -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    let p = Params::default();
    let mut trajs = Vec::new();
    for i in 0..ALL_JUMPS {
        let t = cassie::load_traj(&cassie::data_dir(), i)?;
        let (start, ankle) = cassie::start_row(&urdf, &t, &p)?;
        trajs.push(serde_json::json!({ "index": i, "paper": PAPER_JUMPS.contains(&i), "start": start.to_vec(), "ankle_spring_rates": ankle,
            "t": t.t, "x": t.x.iter().map(|r| r.to_vec()).collect::<Vec<_>>(), "u": t.u.iter().map(|r| r.to_vec()).collect::<Vec<_>>() }));
    }
    let t0 = cassie::load_traj(&cassie::data_dir(), 0)?;
    let r = cassie::replay(&urdf, &t0, &p, 2000.0, cassie::Line::Main)?;
    let check = serde_json::json!({ "trajectory": 0, "hz": 2000.0, "states": r.states.iter().map(|s| s.to_vec()).collect::<Vec<_>>(),
        "loss": cassie::loss(&r.states, &t0.x), "terms": cassie::loss_terms(&r.states, &t0.x) });
    let doc = serde_json::json!({ "trajectories": trajs, "loss_check": check, "folds": cassie::folds(1) });
    std::fs::write(out, serde_json::to_string(&doc).unwrap()).map_err(|e| e.to_string())?;
    println!("wrote {} ({} trajectories)", out, ALL_JUMPS);
    Ok(())
}

/// D9 rule (b)'s check: the held-out 240 Hz replays with each fold's fitted parameters (from
/// `fits`, the 240 Hz fit's JSON) and the leaf springs made rigid (k 1e9 N m/rad); written in the
/// fit's JSON form to `out`, to pair against MuJoCo as the pass was.
fn rigid_check(fits: &str, out: &str) -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    let doc: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(fits).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
    let mut rows = Vec::new();
    for f in doc["folds"].as_array().ok_or("no folds")? {
        let p = Params { k: f["k"].as_f64().unwrap() as f32, d: f["d"].as_f64().unwrap() as f32, mu: f["mu"].as_f64().unwrap() as f32,
                         rigid_springs: true, ..Params::default() };
        for j in f["held_out"].as_array().unwrap() {
            let j = j.as_u64().unwrap() as usize;
            let t = cassie::load_traj(&cassie::data_dir(), j)?;
            let main = cassie::replay(&urdf, &t, &p, 240.0, cassie::Line::Main)?;
            let sens = cassie::replay(&urdf, &t, &p, 240.0, cassie::Line::Sensitivity)?;
            rows.push(serde_json::json!({ "jump": j, "fold": f["fold"], "main": cassie::loss(&main.states, &t.x), "sensitivity": cassie::loss(&sens.states, &t.x) }));
        }
    }
    let mean = rows.iter().map(|r| r["main"].as_f64().unwrap()).sum::<f64>() / rows.len() as f64;
    println!("240 Hz, (8, 6), leaf springs rigid, the 240 Hz folds' parameters: held-out mean {:.2}", mean);
    std::fs::write(out, serde_json::to_string_pretty(&serde_json::json!({ "hz": 240.0, "leaf_springs": "rigid, held at their starting angles", "jumps": rows })).unwrap()).map_err(|e| e.to_string())?;
    Ok(())
}

/// The 240 Hz step error, isolated (PARITY_SPEC 3.4g): for each configuration, the paper's 22 at
/// 240 Hz scored against RiftPhys's own 2 kHz run of the same configuration (the same start and
/// parameters), with the error before the first contact and after it split out. Report only.
fn diagnose() -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    let b = Params::default();
    let cases: Vec<(&str, Params)> = vec![
        ("base (W4e, (8, 6), Table VI contact)", b),
        ("no contact (the toes left out)", Params { toes: false, ..b }),
        ("no contact, no torques", Params { toes: false, inputs: false, ..b }),
        ("no contact, no rotors", Params { toes: false, armature: false, ..b }),
        ("no contact, leaf springs rigid", Params { toes: false, rigid_springs: true, ..b }),
        ("leaf springs rigid", Params { rigid_springs: true, ..b }),
        ("no contact, achilles rod rigid", Params { toes: false, rod_compliance: 0.0, rod_damping: 0.0, ..b }),
        ("no contact, no joint damping", Params { toes: false, joint_damping: false, ..b }),
        ("no contact, 80 iterations", Params { toes: false, iters: (80, 80), ..b }),
        ("no rotors", Params { armature: false, ..b }),
        ("achilles rod rigid", Params { rod_compliance: 0.0, rod_damping: 0.0, ..b }),
        ("contact k x0.1", Params { k: b.k * 0.1, ..b }),
        ("contact k x10", Params { k: b.k * 10.0, ..b }),
        ("contact d 0", Params { d: 0.0, ..b }),
        ("80 iterations", Params { iters: (80, 80), ..b }),
        ("no contact, rotors on the links", Params { toes: false, armature_on_link: 0x3ff, ..b }),
        ("rotors on the links", Params { armature_on_link: 0x3ff, ..b }),
        ("no contact, rotors on the links, springs rigid", Params { toes: false, armature_on_link: 0x3ff, rigid_springs: true, ..b }),
        ("no contact, knee rotors on the links", Params { toes: false, armature_on_link: 0xc0, ..b }),
        ("knee rotors on the links", Params { armature_on_link: 0xc0, ..b }),
        ("rotors on the links but the toes'", Params { armature_on_link: 0xff, ..b }),
    ];
    println!("configuration | 240 Hz against its own 2 kHz: all samples, before first contact, after | mean first contact 240 / 2k (ms)");
    let only = std::env::var("CASSIE_DIAG").ok();
    for (name, p) in cases.iter().filter(|(n, _)| only.as_deref().map_or(true, |o| n.contains(o))) {
        let (mut all, mut pre, mut post, mut npre, mut npost) = (0.0, 0.0, 0.0, 0usize, 0usize);
        let (mut fc240, mut fc2k, mut nfc) = (0.0, 0.0, 0usize);
        for &i in PAPER_JUMPS.iter() {
            let t = cassie::load_traj(&cassie::data_dir(), i)?;
            let a = cassie::replay(&urdf, &t, p, 240.0, cassie::Line::Main)?;
            let c = cassie::replay(&urdf, &t, p, 2000.0, cassie::Line::Main)?;
            all += cassie::loss(&a.states, &c.states) / 22.0;
            let touch = c.first_contact.unwrap_or(f64::INFINITY);
            for k in 0..t.t.len() {
                let e = cassie::loss(&a.states[k..k + 1], &c.states[k..k + 1]);
                if t.t[k] - t.t[0] < touch { pre += e; npre += 1; } else { post += e; npost += 1; }
            }
            if let (Some(x), Some(y)) = (a.first_contact, c.first_contact) { fc240 += x; fc2k += y; nfc += 1; }
        }
        let m = |s: f64, n: usize| if n > 0 { s / n as f64 } else { f64::NAN };
        println!("{:40} | {:7.2} | {:7.2} | {:7.2} | {}", name, all, m(pre, npre), m(post, npost),
                 if nfc > 0 { format!("{:.1} / {:.1}", fc240 / nfc as f64 * 1e3, fc2k / nfc as f64 * 1e3) } else { "-".into() });
    }
    Ok(())
}

/// Flight only (no contact): which actuators carry the 240 Hz error, and whether 2 kHz is itself
/// converged (against 8 kHz). Report-only (PARITY_SPEC 3.4g).
fn diag_inputs() -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    let b = Params { toes: false, ..Params::default() };
    let run = |p: &Params, lo: f64, hi: f64| -> Result<f64, String> {
        let mut all = 0.0;
        for &i in PAPER_JUMPS.iter() {
            let t = cassie::load_traj(&cassie::data_dir(), i)?;
            let a = cassie::replay(&urdf, &t, p, lo, cassie::Line::Main)?;
            let c = cassie::replay(&urdf, &t, p, hi, cassie::Line::Main)?;
            all += cassie::loss(&a.states, &c.states) / 22.0;
        }
        Ok(all)
    };
    println!("flight, 240 Hz against 2 kHz, one actuator pair at a time:");
    for (name, mask) in [("all ten", 0x3ffu16), ("none", 0), ("hip roll", 0x3), ("hip yaw", 0xc), ("hip pitch", 0x30), ("knee", 0xc0), ("toe", 0x300), ("all but toe", 0xff)] {
        println!("  {:12} {:8.2}", name, run(&Params { input_mask: mask, ..b }, 240.0, 2000.0)?);
    }
    println!("flight, all torques, each rate against 8 kHz:");
    for hz in [240.0, 480.0, 960.0, 2000.0, 4000.0] {
        println!("  {:6} Hz {:8.2}", hz, run(&b, hz, 8000.0)?);
    }
    Ok(())
}

/// One jump, flight, knee torques only: the left knee, knee spring and ankle over time at three rates.
fn trace_knee(i: usize) -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    let p = Params { toes: false, input_mask: 0xc0, ..Params::default() };
    let t = cassie::load_traj(&cassie::data_dir(), i)?;
    let r: Vec<_> = [240.0, 2000.0, 8000.0].iter().map(|&hz| cassie::replay(&urdf, &t, &p, hz, cassie::Line::Main)).collect::<Result<_, _>>()?;
    let col = |name: &str| cassie::JOINTS.iter().position(|j| *j == name).unwrap();
    let (kq, sq, aq) = (7 + col("knee_left"), 7 + col("knee_joint_left"), 7 + col("ankle_joint_left"));
    let kv = 29 + col("knee_left");
    println!("t ms | tau knee L | knee q 240/2k/8k | knee rate 240/2k/8k | spring q 240/2k/8k | ankle q 240/2k/8k");
    for k in (0..t.t.len()).step_by(2) {
        let f = |c: usize| format!("{:8.4} {:8.4} {:8.4}", r[0].states[k][c], r[1].states[k][c], r[2].states[k][c]);
        println!("{:5.1} | {:7.1} | {} | {} | {} | {}", (t.t[k] - t.t[0]) * 1e3, t.u[k][6], f(kq), f(kv), f(sq), f(aq));
    }
    Ok(())
}

/// Wall clock per simulated second of the Cassie replay (the 22 jumps, (8, 6), contact on), by rate.
fn cost() -> Result<(), String> {
    let urdf = cassie::load_urdf()?;
    let trajs: Vec<_> = PAPER_JUMPS.iter().map(|&i| cassie::load_traj(&cassie::data_dir(), i)).collect::<Result<_, _>>()?;
    let sim: f64 = trajs.iter().map(|t| t.t[t.t.len() - 1] - t.t[0]).sum();
    for hz in [240.0, 480.0, 960.0, 2000.0] {
        let t0 = std::time::Instant::now();
        for t in &trajs { cassie::replay(&urdf, t, &Params::default(), hz, cassie::Line::Main)?; }
        println!("{:6} Hz: {:.1} ms per simulated second ({:.2} s simulated; includes the start solve and trial steps)", hz, t0.elapsed().as_secs_f64() / sim * 1e3, sim);
    }
    Ok(())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let r = match args.get(1).map(String::as_str) {
        Some("--check") => check(),
        Some("--pluck") => pluck(),
        Some("--rates") => rates(),
        Some("--iters") => iters_check(),
        Some("--diagnose") => diagnose(),
        Some("--diag-inputs") => diag_inputs(),
        Some("--cost") => cost(),
        Some("--trace-knee") => trace_knee(args.get(2).and_then(|a| a.parse().ok()).unwrap_or(0)),
        Some("--rigid-check") => match (args.get(2), args.get(3)) { (Some(f), Some(o)) => rigid_check(f, o), _ => Err("usage: --rigid-check fits.json out.json".into()) },
        Some("--export-starts") => export_starts(&args.get(2).cloned().unwrap_or_else(|| "cassie_starts.json".to_string())),
        Some("--fit") => {
            let hz = args.get(2).and_then(|a| a.parse().ok()).unwrap_or(240.0);
            let evals = args.get(3).and_then(|a| a.parse().ok()).unwrap_or(200);
            let out = args.get(4).cloned().unwrap_or_else(|| format!("riftphys_cassie_fit_{}hz.json", hz));
            fit_all(hz, evals, &out)
        }
        Some("--folds") => { for (k, f) in cassie::folds(1).iter().enumerate() { println!("fold {}: {:?}", k + 1, f); } Ok(()) }
        Some("--delag") => delag_check(args.get(2).and_then(|a| a.parse().ok()).unwrap_or(9)),
        Some("--vs2k") => vs2k(args.get(2).and_then(|a| a.parse().ok()).unwrap_or(0)),
        Some("--correction") => correction(args.get(2).and_then(|a| a.parse().ok()).unwrap_or(0), args.get(3).and_then(|a| a.parse().ok()).unwrap_or(240.0)),
        Some("--floor") => floor(args.get(2).and_then(|a| a.parse().ok()).unwrap_or(240.0)),
        Some("--terms") => terms(args.get(2).and_then(|a| a.parse().ok()).unwrap_or(0), args.get(3).and_then(|a| a.parse().ok()).unwrap_or(240.0)),
        Some("--replay") => {
            let hz = args.get(2).and_then(|a| a.parse().ok()).unwrap_or(240.0);
            let line = if args.get(3).map(String::as_str) == Some("sensitivity") { cassie::Line::Sensitivity } else { cassie::Line::Main };
            replay_all(hz, line)
        }
        Some("--stance") => stance(2.0, args.get(2).and_then(|a| a.parse().ok()).unwrap_or(9400.0)),
        Some("--probe") => probe(args.get(2).and_then(|a| a.parse().ok()).unwrap_or(20), 4.0),
        _ => Err("usage: riftphys-cassie --check | --pluck | --stance | --rates | --probe [traj]".into()),
    };
    if let Err(e) = r { eprintln!("{}", e); std::process::exit(1); }
}
