//! A real double pendulum (PARITY_SPEC W3, P3): the Myers et al. rig (HardwareX 2020), converted by
//! `tools/double_pendulum_to_reality.py`, replayed in RiftPhys from measured states.
//!
//! Each window starts from a measured state (both links' angles and rates) and runs free for 1 s;
//! the error is the two links' RMS angle error at 0.1, 0.25, 0.5 and 1 s. The motion is chaotic,
//! so the short horizons carry the comparison. Each joint has viscous damping and Coulomb
//! friction (PARITY_SPEC W3a), fitted by CMA-ES on trial 2 and held out on trials 1 and 3.

use crate::cmaes;
use crate::log::RealityLog;
use riftphys_core::{Isometry, Mat3, Quat, Vec3, Velocity};
use riftphys_geom::MassProps;
use riftphys_world::world::World;
use std::f32::consts::PI;

pub const G: f32 = 9.81;
pub const SIM_HZ: f32 = 240.0;
/// Scored horizons, seconds.
pub const HORIZONS: [f32; 4] = [0.1, 0.25, 0.5, 1.0];
/// The trial damping is fitted on; the others are held out.
pub const FIT_TRIAL: u32 = 2;

/// How a replay is integrated.
#[derive(Clone, Copy, Debug)]
pub struct Sim {
    pub hz: f32,
    pub iters: (u32, u32),
    /// Start from the measured rate less half a step's change (PARITY_SPEC 3.3, second method)
    pub half_step_start: bool,
}

impl Sim {
    pub fn new(hz: f32, iters: (u32, u32)) -> Self { Self { hz, iters, half_step_start: false } }
}

/// One link: mass (kg), centre's distance from its pivot (m), inertia about the centre (kg m^2).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Link { pub m: f32, pub c: f32, pub i: f32 }

/// The rig, summed from the paper's Table 6 by the converter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rig { pub upper: Link, pub lower: Link, pub l: f32 }

/// Each joint's viscous damping (N m s/rad) and Coulomb friction (N m): upper, then lower.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params { pub c: [f32; 2], pub f: [f32; 2] }

impl Params {
    /// Search box in log10: c in [1e-6, 1e-2], f in [1e-5, 1e-1].
    fn from_unit(x: &[f64]) -> Self {
        let lg = |i: usize, lo: f32, hi: f32| 10f32.powf(lo + (hi - lo) * x[i].clamp(0.0, 1.0) as f32);
        Self { c: [lg(0, -6.0, -2.0), lg(2, -6.0, -2.0)], f: [lg(1, -5.0, -1.0), lg(3, -5.0, -1.0)] }
    }
    fn to_unit(self) -> Vec<f64> {
        let n = |v: f32, lo: f32, hi: f32| ((v.max(1e-12).log10() - lo) / (hi - lo)).clamp(0.0, 1.0) as f64;
        vec![n(self.c[0], -6.0, -2.0), n(self.f[0], -5.0, -1.0), n(self.c[1], -6.0, -2.0), n(self.f[1], -5.0, -1.0)]
    }
    /// The fit's starting point: the order of the paper's Table 3 (its Coulomb term scales with the
    /// joint's normal force; here it is one constant torque per joint).
    pub fn start() -> Self { Self { c: [2e-5, 5e-6], f: [1e-3, 3e-4] } }
}

/// One measured window.
#[derive(Clone, Debug)]
pub struct Window {
    pub trial: u32,
    pub time: f64,
    pub phi: [f32; 2],
    pub rate: [f32; 2],
    /// Measured angles at each of `HORIZONS`
    pub at: [[f32; 2]; 4],
}

/// Wraps to (-pi, pi].
pub fn wrap(a: f32) -> f32 {
    let w = (a + PI).rem_euclid(2.0 * PI) - PI;
    if w <= -PI { w + 2.0 * PI } else { w }
}

/// The angle of a rotation about +z.
pub fn angle_z(q: Quat) -> f32 {
    wrap(2.0 * q.z.atan2(q.w))
}

fn link_props(l: &Link) -> MassProps {
    MassProps { mass: l.m, inv_mass: 1.0 / l.m, inertia: Mat3::from_diagonal(Vec3::splat(l.i).into()) }
}

/// Reads the rig and the windows from a converted log.
pub fn load(log: &RealityLog) -> Result<(Rig, Vec<Window>), String> {
    let num = |v: &serde_json::Value, k: &str| -> Result<f32, String> {
        v.get(k).and_then(|x| x.as_f64()).map(|x| x as f32).ok_or(format!("rig: no {}", k))
    };
    let (mut rig, mut out) = (None, Vec::new());
    let (upper, lower) = (log.tracks.get("upper").ok_or("no `upper` body")?, log.tracks.get("lower").ok_or("no `lower` body")?);
    let rate = log.rate_hz as f64;
    let pair = |v: &serde_json::Value, k: &str| -> Result<[f32; 2], String> {
        let a = v.get(k).and_then(|x| x.as_array()).ok_or(format!("state: no {}", k))?;
        Ok([a[0].as_f64().unwrap_or(f64::NAN) as f32, a[1].as_f64().unwrap_or(f64::NAN) as f32])
    };
    for (t, e, data) in &log.events {
        if e == "rig" {
            let link = |k: &str| -> Result<Link, String> {
                let v = data.get(k).ok_or(format!("rig: no {}", k))?;
                Ok(Link { m: num(v, "m")?, c: num(v, "c")?, i: num(v, "i")? })
            };
            rig = Some(Rig { upper: link("upper")?, lower: link("lower")?, l: num(data, "l")? });
        } else if e == "state" {
            let k0 = upper.partition_point(|s| s.time < t - 1e-6);
            let mut at = [[0.0f32; 2]; 4];
            let mut whole = true;
            for (h, a) in HORIZONS.iter().zip(at.iter_mut()) {
                let k = k0 + (*h as f64 * rate).round() as usize;
                if k >= upper.len() || k >= lower.len() { whole = false; break; }
                *a = [angle_z(upper[k].rot), angle_z(lower[k].rot)];
            }
            if !whole { continue; }
            out.push(Window {
                trial: data.get("trial").and_then(|x| x.as_u64()).ok_or("state: no trial")? as u32,
                time: *t, phi: pair(data, "phi")?, rate: pair(data, "rate")?, at,
            });
        }
    }
    Ok((rig.ok_or("log has no `rig` event")?, out))
}

/// The two links' poses and velocities at angles `phi` and rates `rate`.
struct Built { world: World, bodies: [riftphys_core::BodyId; 2] }

fn build(rig: &Rig, p: &Params, phi: [f32; 2], rate: [f32; 2], iters: (u32, u32)) -> Built {
    let down = |a: f32| Vec3::new(a.sin(), -a.cos(), 0.0);
    let spin = |a: f32| Quat::from_rotation_z(a);
    let axis = Quat::from_rotation_arc(Vec3::X, Vec3::Z);
    let mut world = World::with_capacity(4, 4);
    world.set_gravity(Vec3::new(0.0, -G, 0.0));
    world.set_solver_iterations(iters.0, iters.1);
    let base = world.add_body(Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
    let (w1, w2) = (Vec3::Z * rate[0], Vec3::Z * rate[1]);
    let knee = down(phi[0]) * rig.l;
    let (x1, x2) = (down(phi[0]) * rig.upper.c, knee + down(phi[1]) * rig.lower.c);
    let upper = world.add_body(Isometry { pos: x1, rot: spin(phi[0]) }, Velocity { lin: w1.cross(x1), ang: w1 }, link_props(&rig.upper), true);
    let lower = world.add_body(Isometry { pos: x2, rot: spin(phi[1]) }, Velocity { lin: w1.cross(knee) + w2.cross(x2 - knee), ang: w2 }, link_props(&rig.lower), true);
    for b in [upper, lower] { world.set_body_sleep_allowed(b, false); }
    let frame = |rot: f32, pos: Vec3| Isometry { pos, rot: (spin(rot).conjugate() * axis).normalize() };
    let shoulder = world.add_hinge_joint(base, upper, Isometry { pos: Vec3::ZERO, rot: axis }, frame(phi[0], Vec3::new(0.0, rig.upper.c, 0.0)), 0);
    let elbow = world.add_hinge_joint(upper, lower, frame(phi[0], Vec3::new(0.0, rig.upper.c - rig.l, 0.0)), frame(phi[1], Vec3::new(0.0, rig.lower.c, 0.0)), 0);
    world.set_d6_damping(shoulder, true, 0, p.c[0], p.f[0]);
    world.set_d6_damping(elbow, true, 0, p.c[1], p.f[1]);
    Built { world, bodies: [upper, lower] }
}

/// The rig at a measured start. The World's step is leapfrog (v += h a(x), then x += h v), so its
/// velocity sits half a step before its position. Handing it a rate measured at the position's
/// time starts it half a step late; with `half_step_start` the start rate is the measured one less
/// half of the first step's change.
fn start(rig: &Rig, p: &Params, phi: [f32; 2], rate: [f32; 2], sim: &Sim) -> Built {
    if !sim.half_step_start { return build(rig, p, phi, rate, sim.iters); }
    let mut first = build(rig, p, phi, rate, sim.iters);
    first.world.step(1.0 / sim.hz);
    let after = [first.world.get_body_vel(first.bodies[0]).ang.z, first.world.get_body_vel(first.bodies[1]).ang.z];
    build(rig, p, phi, [rate[0] - 0.5 * (after[0] - rate[0]), rate[1] - 0.5 * (after[1] - rate[1])], sim.iters)
}

/// The simulated angles at each of `HORIZONS`, from a window's measured start.
pub fn replay(rig: &Rig, p: &Params, w: &Window, sim: &Sim) -> [[f32; 2]; 4] {
    let hz = sim.hz;
    let mut b = start(rig, p, w.phi, w.rate, sim);
    let h = 1.0 / hz;
    let mut out = [[0.0f32; 2]; 4];
    let mut step = 0usize;
    for (k, t) in HORIZONS.iter().enumerate() {
        let target = (t * hz).round() as usize;
        while step < target { b.world.step(h); step += 1; }
        out[k] = [angle_z(b.world.get_body_pose(b.bodies[0]).rot), angle_z(b.world.get_body_pose(b.bodies[1]).rot)];
    }
    out
}

/// One window's error per horizon: the two links' RMS angle error, radians.
pub fn window_error(rig: &Rig, p: &Params, w: &Window, sim: &Sim) -> [f32; 4] {
    let got = replay(rig, p, w, sim);
    std::array::from_fn(|k| {
        let (a, b) = (wrap(got[k][0] - w.at[k][0]), wrap(got[k][1] - w.at[k][1]));
        ((a * a + b * b) / 2.0).sqrt()
    })
}

/// Mean and standard error of the windows' errors, degrees, per horizon.
pub fn summary(errs: &[[f32; 4]]) -> [(f32, f32); 4] {
    std::array::from_fn(|k| {
        let n = errs.len().max(1) as f32;
        let m = errs.iter().map(|e| e[k]).sum::<f32>() / n;
        let v = errs.iter().map(|e| (e[k] - m) * (e[k] - m)).sum::<f32>() / n;
        (m.to_degrees(), (v / n).sqrt().to_degrees())
    })
}

/// Fits the damping on `train`: minimises the mean error summed over the first `horizons` of
/// `HORIZONS`. Returns the parameters, the evaluations and the training objective.
pub fn fit(rig: &Rig, train: &[Window], init: &Params, sim: &Sim, horizons: usize, max_evals: usize, seed: u64) -> (Params, usize, f32) {
    let objective = |p: &Params| -> f32 {
        let total: f32 = train.iter().map(|w| window_error(rig, p, w, sim)[..horizons].iter().sum::<f32>()).sum();
        total / train.len().max(1) as f32
    };
    let r = cmaes::minimize(|x| objective(&Params::from_unit(x)) as f64, &init.to_unit(), 0.2, max_evals, seed);
    let p = Params::from_unit(&r.x);
    (p, r.evals, objective(&p))
}

/// Energy above the hanging rest pose at angles `phi` and rates `rate`.
pub fn energy(r: &Rig, phi: [f32; 2], rate: [f32; 2]) -> f32 {
    let (s1, c1, s2, c2) = (phi[0].sin(), phi[0].cos(), phi[1].sin(), phi[1].cos());
    let (w1, w2) = (rate[0], rate[1]);
    let (uc, lc, l) = (r.upper.c, r.lower.c, r.l);
    let (vx, vy) = (l * c1 * w1 + lc * c2 * w2, l * s1 * w1 + lc * s2 * w2);
    let kinetic = 0.5 * r.upper.m * uc * uc * w1 * w1 + 0.5 * r.upper.i * w1 * w1 + 0.5 * r.lower.m * (vx * vx + vy * vy) + 0.5 * r.lower.i * w2 * w2;
    kinetic + r.upper.m * G * uc * (1.0 - c1) + r.lower.m * G * (l * (1.0 - c1) + lc * (1.0 - c2))
}

/// A free run from a window's start for `seconds`: (time, energy above rest), ten times a second.
pub fn energy_run(rig: &Rig, p: &Params, w: &Window, seconds: f32, sim: &Sim) -> Vec<(f32, f32)> {
    let hz = sim.hz;
    let mut b = start(rig, p, w.phi, w.rate, sim);
    let h = 1.0 / hz;
    let every = (hz / 10.0).round() as usize;
    let n = (seconds * hz).round() as usize;
    let mut out = vec![(0.0, energy(rig, w.phi, w.rate))];
    for i in 1..=n {
        b.world.step(h);
        if i % every == 0 {
            let (u, l) = (b.bodies[0], b.bodies[1]);
            let phi = [angle_z(b.world.get_body_pose(u).rot), angle_z(b.world.get_body_pose(l).rot)];
            let rate = [b.world.get_body_vel(u).ang.z, b.world.get_body_vel(l).ang.z];
            out.push((i as f32 * h, energy(rig, phi, rate)));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rig() -> Rig {
        Rig { upper: Link { m: 0.3406, c: 0.07642, i: 1.4543e-3 }, lower: Link { m: 0.1169, c: 0.0715, i: 2.4685e-4 }, l: 0.1719 }
    }

    /// The linearised rig's normal modes, in rad/s, and each mode's lower/upper amplitude ratio.
    fn modes(r: &Rig) -> [(f32, f32); 2] {
        let a = r.upper.i + r.upper.m * r.upper.c * r.upper.c + r.lower.m * r.l * r.l;
        let b = r.lower.m * r.l * r.lower.c;
        let c = r.lower.i + r.lower.m * r.lower.c * r.lower.c;
        let (k1, k2) = (G * (r.upper.m * r.upper.c + r.lower.m * r.l), G * r.lower.m * r.lower.c);
        let (qa, qb, qc) = (a * c - b * b, -(a * k2 + c * k1), k1 * k2);
        let d = (qb * qb - 4.0 * qa * qc).sqrt();
        [(-qb - d) / (2.0 * qa), (-qb + d) / (2.0 * qa)].map(|w2| (w2.sqrt(), (k1 - w2 * a) / (w2 * b)))
    }

    /// PARITY_SPEC W3: a small swing started along a normal mode returns to its start after one
    /// period of the linearised rig, within 1%, for both modes.
    #[test]
    fn a_small_swing_follows_the_rigs_normal_modes() {
        let r = rig();
        let p = Params { c: [0.0; 2], f: [0.0; 2] };
        for (w, ratio) in modes(&r) {
            let amp = 0.02 / ratio.abs().max(1.0);
            let phi0 = [amp, amp * ratio];
            let period = 2.0 * PI / w;
            let h = 1.0 / 960.0;
            let mut b = build(&r, &p, phi0, [0.0; 2], (32, 12));
            let n = (period / h).round() as usize;
            let (mut min_phi, mut at_half) = (f32::MAX, 0usize);
            for i in 0..n {
                b.world.step(h);
                let a = angle_z(b.world.get_body_pose(b.bodies[0]).rot);
                if a < min_phi { min_phi = a; at_half = i + 1; }
            }
            let half = at_half as f32 * h;
            assert!((half - period / 2.0).abs() < 0.01 * period, "mode {:.3} rad/s: half period {:.4} s, expected {:.4}", w, half, period / 2.0);
            let end = [angle_z(b.world.get_body_pose(b.bodies[0]).rot), angle_z(b.world.get_body_pose(b.bodies[1]).rot)];
            for k in 0..2 { assert!((end[k] - phi0[k]).abs() < 0.03 * phi0[k].abs().max(amp), "mode {:.3}: link {} ends at {} from {}", w, k, end[k], phi0[k]); }
        }
    }

    /// With no damping, a large swing keeps its energy within 2% over 2 s at the benchmark's rate.
    #[test]
    fn an_undamped_swing_keeps_its_energy() {
        let r = rig();
        let p = Params { c: [0.0; 2], f: [0.0; 2] };
        let mut b = build(&r, &p, [2.0, 2.6], [0.0; 2], (8, 6));
        let energy = |w: &World, b: &[riftphys_core::BodyId; 2]| -> f32 {
            [(r.upper, b[0]), (r.lower, b[1])].iter().map(|(l, id)| {
                let (x, v) = (w.get_body_pose(*id).pos, w.get_body_vel(*id));
                0.5 * l.m * v.lin.length_squared() + 0.5 * l.i * v.ang.z * v.ang.z + l.m * G * x.y
            }).sum()
        };
        let reference = -(r.upper.m * r.upper.c + r.lower.m * (r.l + r.lower.c)) * G;
        let e0 = energy(&b.world, &b.bodies);
        for _ in 0..480 { b.world.step(1.0 / SIM_HZ); }
        let e1 = energy(&b.world, &b.bodies);
        assert!((e1 - e0).abs() < 0.02 * (e0 - reference), "energy {} -> {} (above rest {})", e0, e1, e0 - reference);
    }

    #[test]
    fn a_window_from_its_own_replay_has_no_error() {
        let r = rig();
        let p = Params::start();
        let mut w = Window { trial: 1, time: 0.0, phi: [1.0, -0.5], rate: [2.0, -3.0], at: [[0.0; 2]; 4] };
        w.at = replay(&r, &p, &w, &Sim::new(SIM_HZ, (8, 6)));
        assert!(window_error(&r, &p, &w, &Sim::new(SIM_HZ, (8, 6))).iter().all(|e| *e < 1e-5));
        let off = Params { c: [1e-3, 1e-3], f: [1e-2, 1e-2] };
        assert!(window_error(&r, &off, &w, &Sim::new(SIM_HZ, (8, 6)))[3] > 1e-3, "damping had no effect");
    }

    /// The half-step start removes the first-order error of a free swing: at a coarse step, the
    /// replay against a fine-step reference is several times closer with it than without.
    #[test]
    fn the_half_step_start_removes_the_first_order_error() {
        let r = rig();
        let p = Params { c: [0.0; 2], f: [0.0; 2] };
        let w = Window { trial: 1, time: 0.0, phi: [1.2, 0.4], rate: [0.0, 0.0], at: [[0.0; 2]; 4] };
        let fine = replay(&r, &p, &w, &Sim { hz: 7680.0, iters: (32, 12), half_step_start: true });
        let at = |half_step_start: bool| -> f32 {
            let got = replay(&r, &p, &w, &Sim { hz: SIM_HZ, iters: (32, 12), half_step_start });
            (0..2).map(|k| wrap(got[0][k] - fine[0][k]).abs()).sum()
        };
        let (plain, shifted) = (at(false), at(true));
        println!("error at 0.1 s: plain start {:.2e} rad, half-step start {:.2e} rad", plain, shifted);
        assert!(shifted * 3.0 < plain, "plain {:.2e}, half-step {:.2e}", plain, shifted);
    }

    #[test]
    fn unit_box_round_trips() {
        let p = Params { c: [3e-5, 7e-6], f: [2e-3, 4e-4] };
        let q = Params::from_unit(&p.to_unit());
        for k in 0..2 {
            assert!((q.c[k] / p.c[k] - 1.0).abs() < 1e-3 && (q.f[k] / p.f[k] - 1.0).abs() < 1e-3);
        }
    }
}
