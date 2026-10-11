//! Reality logs: what a motion-capture session recorded, in RiftPhys conventions (metres,
//! seconds, y up, quaternions [x, y, z, w] body to world). One JSON object per line, tagged by
//! `t`, like the URIEL protocol:
//!
//! ```text
//! {"t":"meta","v":"0.1","source":"optitrack","rate_hz":240,"bodies":["ball"],"notes":"..."}
//! {"t":"pose","time":0.0042,"body":"ball","pos":[0.1,1.2,0.0],"rot":[0,0,0,1]}
//! {"t":"cmd","time":0.0042,"who":"cf1","act":[0.5,0.5,0.5,0.5]}
//! {"t":"event","time":2.0,"e":"break"}
//! ```
//!
//! A `break` event separates takes (a ball picked up and thrown again): replay windows never
//! straddle one. `tools/mocap_to_reality.py` converts OptiTrack Motive CSV exports and generic
//! CSV (libsurvive, Crazyflie logs) into this format.

use riftphys_core::{Quat, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub const REALITY_VERSION: &str = "0.1";

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "t", rename_all = "snake_case")]
pub enum RealityLine {
    Meta {
        v: String,
        /// Capture system: optitrack, vicon, libsurvive, crazyflie, synthetic, ...
        source: String,
        rate_hz: f32,
        bodies: Vec<String>,
        #[serde(default)]
        notes: String,
    },
    Pose { time: f64, body: String, pos: [f32; 3], rot: [f32; 4] },
    Cmd { time: f64, who: String, act: Vec<f32> },
    Event {
        time: f64,
        e: String,
        #[serde(default)]
        data: serde_json::Value,
    },
}

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub time: f64,
    pub pos: Vec3,
    pub rot: Quat,
}

#[derive(Clone, Debug, Default)]
pub struct RealityLog {
    pub source: String,
    pub rate_hz: f32,
    /// Per body, samples in time order
    pub tracks: BTreeMap<String, Vec<Sample>>,
    /// Per actuated agent, (time, command) in time order
    pub cmds: BTreeMap<String, Vec<(f64, Vec<f32>)>>,
    /// Times of `break` events, sorted
    pub breaks: Vec<f64>,
}

impl RealityLog {
    pub fn parse(text: &str) -> Result<Self, String> {
        let mut log = RealityLog::default();
        for (n, line) in text.lines().enumerate() {
            let line = line.trim();
            if line.is_empty() { continue; }
            let parsed: RealityLine = serde_json::from_str(line).map_err(|e| format!("line {}: {}", n + 1, e))?;
            match parsed {
                RealityLine::Meta { v, source, rate_hz, bodies, .. } => {
                    if v != REALITY_VERSION { return Err(format!("unsupported reality log version {}", v)); }
                    log.source = source;
                    log.rate_hz = rate_hz;
                    for b in bodies { log.tracks.entry(b).or_default(); }
                }
                RealityLine::Pose { time, body, pos, rot } => {
                    let q = Quat::from_array(rot);
                    if !(q.length_squared() > 0.5) { return Err(format!("line {}: bad quaternion", n + 1)); }
                    log.tracks.entry(body).or_default().push(Sample { time, pos: Vec3::from_array(pos), rot: q.normalize() });
                }
                RealityLine::Cmd { time, who, act } => log.cmds.entry(who).or_default().push((time, act)),
                RealityLine::Event { time, e, .. } => if e == "break" { log.breaks.push(time); },
            }
        }
        for t in log.tracks.values_mut() { t.sort_by(|a, b| a.time.partial_cmp(&b.time).unwrap()); }
        for c in log.cmds.values_mut() { c.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap()); }
        log.breaks.sort_by(|a, b| a.partial_cmp(b).unwrap());
        Ok(log)
    }

    pub fn load(path: &str) -> Result<Self, String> {
        Self::parse(&std::fs::read_to_string(path).map_err(|e| format!("{}: {}", path, e))?)
    }

    /// Lines for a log (meta first), ready to write one per line.
    pub fn to_lines(&self, notes: &str) -> Vec<RealityLine> {
        let mut out = vec![RealityLine::Meta { v: REALITY_VERSION.into(), source: self.source.clone(), rate_hz: self.rate_hz,
                                               bodies: self.tracks.keys().cloned().collect(), notes: notes.into() }];
        let mut timed: Vec<(f64, RealityLine)> = Vec::new();
        for (body, track) in &self.tracks {
            for s in track {
                timed.push((s.time, RealityLine::Pose { time: s.time, body: body.clone(), pos: s.pos.to_array(), rot: s.rot.to_array() }));
            }
        }
        for (who, cmds) in &self.cmds {
            for (t, act) in cmds { timed.push((*t, RealityLine::Cmd { time: *t, who: who.clone(), act: act.clone() })); }
        }
        for &t in &self.breaks { timed.push((t, RealityLine::Event { time: t, e: "break".into(), data: serde_json::Value::Null })); }
        timed.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        out.extend(timed.into_iter().map(|(_, l)| l));
        out
    }

    pub fn save(&self, path: &str, notes: &str) -> Result<(), String> {
        let text: String = self.to_lines(notes).iter().map(|l| serde_json::to_string(l).unwrap() + "\n").collect();
        std::fs::write(path, text).map_err(|e| format!("{}: {}", path, e))
    }

    /// Pose of `body` at time `t`, linearly interpolated (slerp for rotation); None outside the track.
    pub fn pose_at(&self, body: &str, t: f64) -> Option<(Vec3, Quat)> {
        let tr = self.tracks.get(body)?;
        if tr.is_empty() || t < tr[0].time || t > tr[tr.len() - 1].time { return None; }
        let i = tr.partition_point(|s| s.time <= t).max(1) - 1;
        let j = (i + 1).min(tr.len() - 1);
        let (a, b) = (tr[i], tr[j]);
        let f = if b.time > a.time { ((t - a.time) / (b.time - a.time)) as f32 } else { 0.0 };
        Some((a.pos.lerp(b.pos, f), a.rot.slerp(b.rot, f)))
    }

    /// Velocity of `body` at `t` (linear, angular), from a central difference over `half` seconds
    /// either side. Mocap gives poses only; this is how a replay starts a body moving.
    pub fn velocity_at(&self, body: &str, t: f64, half: f64) -> Option<(Vec3, Vec3)> {
        let (p0, q0) = self.pose_at(body, t - half)?;
        let (p1, q1) = self.pose_at(body, t + half)?;
        let span = (2.0 * half) as f32;
        let mut dq = q1 * q0.conjugate();
        if dq.w < 0.0 { dq = -dq; }
        let (axis, angle) = dq.to_axis_angle();
        Some(((p1 - p0) / span, axis * (angle / span)))
    }

    /// State of `body` at `t` from a least-squares constant-acceleration fit to every sample in
    /// [t - half, t + half]: (position, velocity, angular velocity, fit residual RMS in m).
    /// Averages out mocap noise (a two-point difference of 0.5 mm noise over 20 ms is ~3.5 cm/s
    /// off) and the residual exposes an impact inside the span, where no smooth state exists.
    pub fn state_at(&self, body: &str, t: f64, half: f64) -> Option<(Vec3, Vec3, Vec3, f32)> {
        let tr = self.tracks.get(body)?;
        let pts: Vec<&Sample> = tr.iter().filter(|s| (s.time - t).abs() <= half).collect();
        if pts.len() < 5 { return None; }
        // Normal equations for p(tau) = c0 + c1 tau + c2 tau^2 / 2, per axis
        let mut m = [[0.0f64; 3]; 3];
        let mut rhs = [[0.0f64; 3]; 3];
        for s in &pts {
            let tau = s.time - t;
            let basis = [1.0, tau, 0.5 * tau * tau];
            for i in 0..3 {
                for j in 0..3 { m[i][j] += basis[i] * basis[j]; }
                for (k, v) in s.pos.to_array().iter().enumerate() { rhs[k][i] += basis[i] * *v as f64; }
            }
        }
        let solve = |b: [f64; 3]| -> Option<[f64; 3]> {
            let det = |a: &[[f64; 3]; 3]| a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
            let d = det(&m);
            if d.abs() < 1e-30 { return None; }
            let mut out = [0.0; 3];
            for c in 0..3 {
                let mut a = m;
                for r in 0..3 { a[r][c] = b[r]; }
                out[c] = det(&a) / d;
            }
            Some(out)
        };
        let (cx, cy, cz) = (solve(rhs[0])?, solve(rhs[1])?, solve(rhs[2])?);
        let pos = Vec3::new(cx[0] as f32, cy[0] as f32, cz[0] as f32);
        let vel = Vec3::new(cx[1] as f32, cy[1] as f32, cz[1] as f32);
        let mut sq = 0.0f64;
        for s in &pts {
            let tau = s.time - t;
            let p = [cx[0] + cx[1] * tau + 0.5 * cx[2] * tau * tau, cy[0] + cy[1] * tau + 0.5 * cy[2] * tau * tau, cz[0] + cz[1] * tau + 0.5 * cz[2] * tau * tau];
            let e = Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32) - s.pos;
            sq += e.length_squared() as f64;
        }
        let (_, ang) = self.velocity_at(body, t, half)?;
        Some((pos, vel, ang, (sq / pts.len() as f64).sqrt() as f32))
    }

    /// Time span covered by every listed body.
    pub fn span(&self, bodies: &[String]) -> Option<(f64, f64)> {
        let mut lo = f64::NEG_INFINITY;
        let mut hi = f64::INFINITY;
        for b in bodies {
            let tr = self.tracks.get(b)?;
            if tr.is_empty() { return None; }
            lo = lo.max(tr[0].time);
            hi = hi.min(tr[tr.len() - 1].time);
        }
        (hi > lo).then_some((lo, hi))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_and_interpolation() {
        let mut log = RealityLog { source: "synthetic".into(), rate_hz: 100.0, ..Default::default() };
        log.tracks.insert("ball".into(), (0..11).map(|i| Sample { time: i as f64 * 0.01, pos: Vec3::new(i as f32 * 0.01, 0.0, 0.0), rot: Quat::IDENTITY }).collect());
        log.breaks.push(0.05);
        let text: String = log.to_lines("test").iter().map(|l| serde_json::to_string(l).unwrap() + "\n").collect();
        let back = RealityLog::parse(&text).unwrap();
        assert_eq!(back.tracks["ball"].len(), 11);
        assert_eq!(back.breaks, vec![0.05]);
        let (p, _) = back.pose_at("ball", 0.025).unwrap();
        assert!((p.x - 0.025).abs() < 1e-6);
        let (v, w) = back.velocity_at("ball", 0.05, 0.02).unwrap();
        assert!((v.x - 1.0).abs() < 1e-4 && w.length() < 1e-6, "v {:?} w {:?}", v, w);
    }
}
