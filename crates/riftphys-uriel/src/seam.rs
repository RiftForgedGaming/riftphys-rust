//! Seam tracking (fabrication rung F1): seam workpieces, a laser line profiler on the arm's
//! tool, the classical seam-tracking teacher, the nominal-path floor, and the episode.
//!
//! - Workpiece: the seam's cross-section (butt, V or fillet joint) extruded along its path as a
//!   static triangle ribbon (`SeamWorkpiece`, a world environment). Triangles cast exactly and
//!   only the few segments near a query are touched, so contacts stay cheap at 1 mm detail.
//! - Paths: a straight `line` in front of the arm, or a `ring` around its base (one pass with
//!   continuous joints). Seeded disturbances: fixture offset and yaw, curvature (bow or radial
//!   waviness), gap variation, travel speed, profiler noise.
//! - Profiler: `n` rays from an emitter ahead of and above the TCP, fanned across the seam, cast
//!   with `World::raycast_exact` (the arm itself skipped). Noise comes from the episode `Rng`.
//! - Teacher (`SeamTracker`): profile -> seam point -> offset from the CAD seam -> Cartesian
//!   target -> `CartesianServo`. The floor is the same tracker with sensing off (CAD seam only).
//! - Operator (human in the loop): pause / resume / override (jog) / stop, recorded as
//!   protocol `operator` lines at their tick.
//!
//! See docs/URIEL_Fabrication_F1.md.

use crate::arm::{self, Arm, ArmConfig, CartesianServo, TcpTarget, CONTROL_HZ, JOINTS, LINK_NAMES, MAX_TORQUE, SUBSTEPS};
use crate::protocol::{hash_hex, EntState, KindSpec, Line, SensorSpec};
use crate::rng::Rng;
use riftphys_collision::environment::{EnvCollider, SpatialEnvironment};
use riftphys_core::{Isometry, Quat, Vec3, Velocity};
use riftphys_geom::{Aabb, MassProps, Shape, Triangle};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::World;
use std::f32::consts::PI;

/// Plate thickness, V root face, web height (fillet) and the ribbon's half width (m).
pub const PLATE_T: f32 = 0.010;
pub const ROOT_FACE: f32 = 0.002;
pub const WEB_H: f32 = 0.040;
pub const HALF_WIDTH: f32 = 0.10;
/// Half of the V's 60 degree included angle.
pub const BEVEL: f32 = PI / 6.0;
/// Search window of the seam finder around the CAD seam (m): detections further off are
/// rejected, as a real tracker's window would.
pub const SEARCH_WINDOW: f32 = 0.015;
/// Workpiece segment length along the seam (m).
pub const SEG: f32 = 0.005;
/// Tracking error (m) beyond which, held for `LOST_TICKS`, the seam counts as lost.
pub const LOST_ERR: f32 = 0.005;
pub const LOST_TICKS: u32 = 30;
/// Ticks of braking after an emergency stop before the episode ends.
pub const STOP_TICKS: u32 = 30;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointType { Butt, V, Fillet }

impl JointType {
    pub fn name(self) -> &'static str { match self { JointType::Butt => "butt", JointType::V => "v", JointType::Fillet => "fillet" } }
    pub fn parse(s: &str) -> Option<Self> {
        match s { "butt" => Some(JointType::Butt), "v" => Some(JointType::V), "fillet" => Some(JointType::Fillet), _ => None }
    }
    /// Depth of the seam point below the plate top (the V's root is at the top of its root face).
    pub fn root_depth(self) -> f32 { if self == JointType::V { PLATE_T - ROOT_FACE } else { 0.0 } }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SeamShape { Line, Ring }

impl SeamShape {
    pub fn name(self) -> &'static str { match self { SeamShape::Line => "line", SeamShape::Ring => "ring" } }
    pub fn parse(s: &str) -> Option<Self> { match s { "line" => Some(SeamShape::Line), "ring" => Some(SeamShape::Ring), _ => None } }
}

#[derive(Clone, Copy, Debug)]
pub struct SeamConfig {
    pub shape: SeamShape,
    pub joint: JointType,
    /// Line: length and distance in front of the base (m). Ring: radius (m) and arc (rad).
    pub line_len: f32,
    pub line_dist: f32,
    pub ring_radius: f32,
    pub ring_arc: f32,
    /// Disturbance bounds: fixture offset (lateral and height, m), fixture yaw (rad, line),
    /// curvature amplitude (m), gap range (m), travel speed range (m/s), profiler noise (m).
    pub offset_max: f32,
    pub yaw_max: f32,
    pub bow_max: f32,
    pub gap: (f32, f32),
    pub speed: (f32, f32),
    pub noise: f32,
    /// Torch: standoff of the TCP from the seam point (m), travel (push) angle and fillet work
    /// angle (rad).
    pub standoff: f32,
    pub travel_angle: f32,
    pub work_angle: f32,
    pub profiler: Profiler,
    /// Phases (s): approach from the ready pose to the seam start, then a settle before travel.
    pub approach_s: f32,
    pub settle_s: f32,
    /// Random spread of the ready pose's joint angles (rad).
    pub start_spread: f32,
    pub arm: ArmConfig,
}

impl Default for SeamConfig {
    fn default() -> Self {
        Self {
            shape: SeamShape::Line, joint: JointType::V,
            line_len: 0.40, line_dist: 0.45, ring_radius: 0.45, ring_arc: 2.0 * PI,
            offset_max: 0.003, yaw_max: 1f32.to_radians(), bow_max: 0.006,
            gap: (0.0005, 0.0025), speed: (0.015, 0.025), noise: 0.0001,
            standoff: 0.002, travel_angle: 10f32.to_radians(), work_angle: 45f32.to_radians(),
            profiler: Profiler::default(),
            approach_s: 2.0, settle_s: 0.5, start_spread: 0.05,
            arm: ArmConfig::default(),
        }
    }
}

impl SeamConfig {
    /// `seam_<shape>_<joint>`, with `_axis` for the axis-armature arm (PARITY_SPEC 3.9)
    pub fn scenario(&self) -> String {
        let arm = if self.arm.armature == crate::arm::ArmArmature::Axis { "_axis" } else { "" };
        format!("seam_{}_{}{}", self.shape.name(), self.joint.name(), arm)
    }
}

// ---------------------------------------------------------------------------------------------
// Paths and seams
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub enum PathKind {
    Line { start: Vec3, dir: Vec3, len: f32 },
    Ring { centre: Vec3, radius: f32, a0: f32, arc: f32 },
}

/// A seam centreline at the plate top: the CAD path plus curvature and a fixture transform.
#[derive(Clone, Copy, Debug)]
pub struct Path {
    pub kind: PathKind,
    /// Line: lateral half-sine bow amplitude. Ring: radial waviness amplitude, `waves` per turn.
    pub bow: f32,
    pub waves: f32,
    pub phase: f32,
    /// Fixture: yaw about `pivot`, then `shift`.
    pub yaw: f32,
    pub pivot: Vec3,
    pub shift: Vec3,
}

impl Path {
    pub fn nominal(kind: PathKind) -> Self {
        Self { kind, bow: 0.0, waves: 0.0, phase: 0.0, yaw: 0.0, pivot: Vec3::ZERO, shift: Vec3::ZERO }
    }

    pub fn len(&self) -> f32 {
        match self.kind { PathKind::Line { len, .. } => len, PathKind::Ring { radius, arc, .. } => radius * arc }
    }

    pub fn closed(&self) -> bool {
        matches!(self.kind, PathKind::Ring { arc, .. } if arc >= 2.0 * PI - 1.0e-4)
    }

    fn raw(&self, s: f32) -> Vec3 {
        match self.kind {
            PathKind::Line { start, dir, len } => {
                let lat = dir.cross(Vec3::Y).normalize();
                start + dir * s + lat * (self.bow * (PI * s / len).sin())
            }
            PathKind::Ring { centre, radius, a0, .. } => {
                let th = a0 + s / radius;
                let rho = radius + self.bow * (self.waves * th + self.phase).sin();
                centre + Vec3::new(th.cos(), 0.0, th.sin()) * rho
            }
        }
    }

    pub fn point(&self, s: f32) -> Vec3 {
        self.pivot + Quat::from_rotation_y(self.yaw) * (self.raw(s) - self.pivot) + self.shift
    }

    pub fn tangent(&self, s: f32) -> Vec3 {
        (self.point(s + 1.0e-3) - self.point(s - 1.0e-3)).normalize()
    }

    /// Horizontal unit vector across the seam (tangent x up).
    pub fn lateral(&self, s: f32) -> Vec3 {
        self.tangent(s).cross(Vec3::Y).normalize()
    }

    /// Parameter of the point of the path in [lo, hi] closest to `p` (horizontal distance):
    /// a 2 mm scan, then a golden-section refinement.
    pub fn project(&self, p: Vec3, lo: f32, hi: f32) -> f32 {
        let d2 = |s: f32| { let q = self.point(s) - p; q.x * q.x + q.z * q.z };
        let n = (((hi - lo) / 0.002).ceil() as u32).max(1);
        let mut best = (lo, d2(lo));
        for i in 1..=n {
            let s = lo + (hi - lo) * i as f32 / n as f32;
            let d = d2(s);
            if d < best.1 { best = (s, d); }
        }
        let (mut a, mut b) = (best.0 - 0.002, best.0 + 0.002);
        let r = 0.618_034f32;
        for _ in 0..24 {
            let (c, d) = (b - r * (b - a), a + r * (b - a));
            if d2(c) < d2(d) { b = d; } else { a = c; }
        }
        0.5 * (a + b)
    }
}

/// Root gap along the seam: g0 + amp sin(2 pi s / wavelength + phase), at least 0.3 mm.
#[derive(Clone, Copy, Debug)]
pub struct Gap { pub g0: f32, pub amp: f32, pub wavelength: f32, pub phase: f32 }

impl Gap {
    pub fn at(&self, s: f32) -> f32 {
        (self.g0 + self.amp * (2.0 * PI * s / self.wavelength + self.phase).sin()).max(0.0003)
    }
}

/// One episode's seam: the CAD path the floor follows, and the real one the plates form.
#[derive(Clone, Copy, Debug)]
pub struct Seam {
    pub joint: JointType,
    pub nominal: Path,
    pub actual: Path,
    pub gap: Gap,
    /// Seeded disturbance values, for the episode record.
    pub offset: Vec3,
    pub speed: f32,
}

impl Seam {
    pub fn generate(cfg: &SeamConfig, rng: &mut Rng) -> Self {
        let kind = match cfg.shape {
            SeamShape::Line => PathKind::Line { start: Vec3::new(cfg.line_dist, 0.0, -0.5 * cfg.line_len), dir: Vec3::Z, len: cfg.line_len },
            SeamShape::Ring => PathKind::Ring { centre: Vec3::ZERO, radius: cfg.ring_radius, a0: 0.0, arc: cfg.ring_arc },
        };
        let nominal = Path::nominal(kind);
        let len = nominal.len();
        let mid = nominal.point(0.5 * len);
        let lat = nominal.lateral(0.5 * len);
        let offset = match cfg.shape {
            // Line: across the seam; ring: the centre moves in any horizontal direction
            SeamShape::Line => lat * rng.range(-1.0, 1.0) * cfg.offset_max,
            SeamShape::Ring => { let a = rng.range(0.0, 2.0 * PI); Vec3::new(a.cos(), 0.0, a.sin()) * rng.range(0.0, cfg.offset_max) }
        } + Vec3::Y * rng.range(-1.0, 1.0) * (0.66 * cfg.offset_max);
        let mut actual = nominal;
        actual.bow = rng.range(-1.0, 1.0) * cfg.bow_max;
        actual.waves = 3.0;
        actual.phase = rng.range(0.0, 2.0 * PI);
        if cfg.shape == SeamShape::Line { actual.yaw = rng.range(-1.0, 1.0) * cfg.yaw_max; }
        actual.pivot = mid;
        actual.shift = offset;
        let (glo, ghi) = cfg.gap;
        let mut wavelength = rng.range(0.1, 0.3);
        if actual.closed() { wavelength = len / (len / wavelength).round().max(1.0); }
        let gap = Gap { g0: 0.5 * (glo + ghi), amp: 0.5 * (ghi - glo) * rng.range(0.3, 1.0), wavelength, phase: rng.range(0.0, 2.0 * PI) };
        let speed = rng.range(cfg.speed.0, cfg.speed.1);
        Seam { joint: cfg.joint, nominal, actual, gap, offset, speed }
    }

    pub fn len(&self) -> f32 { self.nominal.len() }

    /// The range of s that projections search: past both ends of an open seam; for a closed ring
    /// one turn starting just before the run-on, so the run-on (negative s) never aliases to the
    /// end of the turn.
    pub fn s_window(&self) -> (f32, f32) {
        let len = self.len();
        if self.actual.closed() { (-0.05, len - 0.05) } else { (-0.05, len + 0.05) }
    }

    /// Ground-truth seam point at `s` on the real seam.
    pub fn true_root(&self, s: f32) -> Vec3 { self.actual.point(s) - Vec3::Y * self.joint.root_depth() }

    /// The CAD seam point at `s`.
    pub fn nominal_root(&self, s: f32) -> Vec3 { self.nominal.point(s) - Vec3::Y * self.joint.root_depth() }

    /// Cross-section polyline (lateral d, height y) at gap `g`, left (-lateral) to right.
    pub fn section(&self, g: f32) -> Vec<(f32, f32)> {
        let w = HALF_WIDTH;
        match self.joint {
            JointType::Butt => vec![(-w, 0.0), (-0.5 * g, 0.0), (-0.5 * g, -PLATE_T), (0.5 * g, -PLATE_T), (0.5 * g, 0.0), (w, 0.0)],
            JointType::V => {
                let top = 0.5 * g + (PLATE_T - ROOT_FACE) * BEVEL.tan();
                let rf = -(PLATE_T - ROOT_FACE);
                vec![(-w, 0.0), (-top, 0.0), (-0.5 * g, rf), (-0.5 * g, -PLATE_T), (0.5 * g, -PLATE_T), (0.5 * g, rf), (top, 0.0), (w, 0.0)]
            }
            // A web of plate thickness standing on the -lateral side of the seam (T joint)
            JointType::Fillet => vec![(-w, 0.0), (-PLATE_T, 0.0), (-PLATE_T, WEB_H), (0.0, WEB_H), (0.0, 0.0), (w, 0.0)],
        }
    }

    /// The tool rotation for welding at a point with seam tangent `t`: torch down (tilted by the
    /// work angle toward a fillet's web), pushed forward by the travel angle; tool x along the
    /// travel, y along the torch, z = x cross y (the profiler's line, across the seam).
    pub fn torch_rot(&self, cfg: &SeamConfig, t: Vec3) -> Quat {
        let lat = t.cross(Vec3::Y).normalize();
        let mut torch = -Vec3::Y;
        if self.joint == JointType::Fillet {
            torch = -Vec3::Y * cfg.work_angle.cos() - lat * cfg.work_angle.sin();
        }
        let torch = (torch + t * cfg.travel_angle.tan()).normalize();
        let fwd = (t - torch * t.dot(torch)).normalize();
        Quat::from_mat3(&glam::Mat3::from_cols(fwd, torch, fwd.cross(torch))).normalize()
    }
}

// ---------------------------------------------------------------------------------------------
// Workpiece
// ---------------------------------------------------------------------------------------------

/// The real seam as a static triangle ribbon: its cross-section swept along the path at `SEG`
/// spacing, with the gap varying along it. Segments are culled by their bounds per query, in
/// segment order (deterministic).
pub struct SeamWorkpiece {
    segs: Vec<(Aabb, std::ops::Range<usize>)>,
    tris: Vec<Triangle>,
}

impl SeamWorkpiece {
    pub fn build(seam: &Seam) -> Self {
        let len = seam.len();
        let closed = seam.actual.closed();
        // Open seams run 60 mm past each end so the look-ahead always sees plate
        let (s0, s1) = if closed { (0.0, len) } else { (-0.06, len + 0.06) };
        let n = ((s1 - s0) / SEG).ceil() as usize;
        let section = |k: usize| -> Vec<Vec3> {
            let s = s0 + (s1 - s0) * k as f32 / n as f32;
            let s = if closed && k == n { 0.0 } else { s };
            let (c, lat) = (seam.actual.point(s), seam.actual.lateral(s));
            seam.section(seam.gap.at(s)).into_iter().map(|(d, y)| c + lat * d + Vec3::Y * y).collect()
        };
        let mut segs = Vec::with_capacity(n);
        let mut tris = Vec::new();
        let mut prev = section(0);
        for k in 1..=n {
            let next = section(k);
            let first = tris.len();
            let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
            for i in 0..prev.len() - 1 {
                let (a, b, c, d) = (prev[i], prev[i + 1], next[i + 1], next[i]);
                for p in [a, b, c, d] { lo = lo.min(p); hi = hi.max(p); }
                tris.push(Triangle::new(a, b, c));
                tris.push(Triangle::new(a, c, d));
            }
            segs.push((Aabb::new(lo - Vec3::splat(1.0e-4), hi + Vec3::splat(1.0e-4)), first..tris.len()));
            prev = next;
        }
        Self { segs, tris }
    }

    pub fn triangles(&self) -> usize { self.tris.len() }
}

impl SpatialEnvironment for SeamWorkpiece {
    fn query_aabb(&self, aabb: &Aabb, out_buffer: &mut Vec<EnvCollider>) {
        let ident = Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY };
        for (k, (b, range)) in self.segs.iter().enumerate() {
            if !b.overlaps(aabb) { continue; }
            for t in &self.tris[range.clone()] {
                out_buffer.push(EnvCollider { shape: Shape::Triangle(*t), transform: ident, material: MaterialId::Steel, user_data: k as u64 });
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Profiler
// ---------------------------------------------------------------------------------------------

/// Laser line profiler fixed to the tool: `n` rays from an emitter `height` back along the torch
/// and `lookahead` ahead of the TCP, fanned across the seam so they span `fan` at the TCP.
#[derive(Clone, Copy, Debug)]
pub struct Profiler {
    pub n: u32,
    pub fan: f32,
    pub height: f32,
    pub lookahead: f32,
    pub max_range: f32,
}

impl Default for Profiler {
    fn default() -> Self { Self { n: 64, fan: 0.04, height: 0.10, lookahead: 0.025, max_range: 0.25 } }
}

impl Profiler {
    pub fn spec(&self) -> SensorSpec {
        SensorSpec { name: "profiler".into(), kind: "line_scan".into(), n: self.n, fan_m: self.fan, max_range_m: self.max_range, unit: "m".into() }
    }

    /// Emitter position, and the torch and across-seam axes, for a TCP pose.
    pub fn frame(&self, tcp: &Isometry) -> (Vec3, Vec3, Vec3) {
        let (fwd, torch, across) = (tcp.rot * Vec3::X, tcp.rot * Vec3::Y, tcp.rot * Vec3::Z);
        (tcp.pos - torch * self.height + fwd * self.lookahead, torch, across)
    }

    pub fn ray(&self, i: u32, torch: Vec3, across: Vec3) -> Vec3 {
        let f = if self.n > 1 { 2.0 * i as f32 / (self.n - 1) as f32 - 1.0 } else { 0.0 };
        (torch + across * (f * 0.5 * self.fan / self.height)).normalize()
    }

    /// Ranges along each ray (m), `None` where nothing is hit; the arm's own links are skipped.
    pub fn scan(&self, world: &World, arm: &Arm) -> Vec<Option<f32>> {
        let (o, torch, across) = self.frame(&arm.tcp(world));
        (0..self.n).map(|i| {
            world.raycast_exact(o, self.ray(i, torch, across), self.max_range, |b| arm.link_index(b).is_some())
                .map(|h| h.toi * self.max_range)
        }).collect()
    }

    /// Hit points in the world for ranges taken at `tcp`.
    pub fn points(&self, tcp: &Isometry, ranges: &[Option<f32>]) -> Vec<Option<Vec3>> {
        let (o, torch, across) = self.frame(tcp);
        ranges.iter().enumerate().map(|(i, r)| r.map(|r| o + self.ray(i as u32, torch, across) * r)).collect()
    }
}

/// The classical seam finder: the seam point (world) in one profile, or `None` if the joint's
/// shape isn't in it. Butt and V: the two walls cross half the groove depth; the seam is midway
/// between the crossings, at the root depth below the plate top. Fillet: the web's vertical
/// face stands above the seam, so its lowest hit, dropped to the plate, is the seam.
pub fn find_seam(joint: JointType, pts: &[Option<Vec3>]) -> Option<Vec3> {
    let valid: Vec<Vec3> = pts.iter().flatten().copied().collect();
    if valid.len() < pts.len() / 4 { return None; }
    let (lo, hi) = valid.iter().fold((f32::MAX, f32::MIN), |(l, h), p| (l.min(p.y), h.max(p.y)));
    let cross = |a: Vec3, b: Vec3, thr: f32| a + (b - a) * ((thr - a.y) / (b.y - a.y));
    match joint {
        JointType::Butt | JointType::V => {
            let depth = if joint == JointType::Butt { PLATE_T } else { PLATE_T - ROOT_FACE };
            if hi - lo < 0.5 * depth { return None; }
            let thr = hi - 0.5 * depth;
            let top: Vec<f32> = valid.iter().filter(|p| p.y > hi - 0.25 * depth).map(|p| p.y).collect();
            let top = top.iter().sum::<f32>() / top.len() as f32;
            let down = valid.windows(2).find(|w| w[0].y >= thr && w[1].y < thr).map(|w| cross(w[0], w[1], thr))?;
            let up = valid.windows(2).rev().find(|w| w[0].y < thr && w[1].y >= thr).map(|w| cross(w[0], w[1], thr))?;
            let mid = 0.5 * (down + up);
            Some(Vec3::new(mid.x, top - joint.root_depth(), mid.z))
        }
        JointType::Fillet => {
            // A few millimetres of the face locate the corner; the fan need not reach the web's top
            if hi - lo < 0.005 { return None; }
            // Plate: the lowest hits. Web face: hits clearly between the plate and the web top.
            // The face is vertical, so its lowest hit stands right above the corner.
            let plate: Vec<f32> = valid.iter().filter(|p| p.y < lo + 0.002).map(|p| p.y).collect();
            let top = plate.iter().sum::<f32>() / plate.len() as f32;
            let face = valid.iter().filter(|p| p.y > top + 0.002 && p.y < top + WEB_H - 0.002)
                .fold(None, |b: Option<Vec3>, p| if b.map_or(true, |b| p.y < b.y) { Some(*p) } else { b })?;
            Some(Vec3::new(face.x, top, face.z))
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Teacher and floor
// ---------------------------------------------------------------------------------------------

/// Classical seam tracker. With sensing on it is the teacher: each profile gives a seam point
/// ahead of the torch, kept as an offset from the CAD seam at that point; the torch follows the
/// CAD seam plus the offset interpolated at its own position. With sensing off it is the floor:
/// the CAD seam alone. Both drive the arm through the same `CartesianServo`.
#[derive(Clone, Debug)]
pub struct SeamTracker {
    pub sensing: bool,
    pub servo: CartesianServo,
    /// (s, offset from the CAD seam point) from each profile, in order of s.
    pub seen: Vec<(f32, Vec3)>,
    /// Start of the approach and the target of this tick.
    start: Option<Isometry>,
    pub target: Option<TcpTarget>,
}

impl SeamTracker {
    pub fn teacher(env: &SeamEnv) -> Self {
        Self { sensing: true, servo: CartesianServo::new(&env.arm, &env.world), seen: Vec::new(), start: None, target: None }
    }

    pub fn nominal(env: &SeamEnv) -> Self {
        Self { sensing: false, ..Self::teacher(env) }
    }

    /// CAD-seam offset at `s` from what has been seen: interpolated between profiles, held flat
    /// after the last, and before the first (the torch's first 25 mm, which the look-ahead never
    /// saw) extrapolated along the slope of the first 20 mm of profiles (fixture yaw and bow).
    pub fn offset_at(&self, s: f32) -> Vec3 {
        match self.seen.iter().position(|&(si, _)| si >= s) {
            None => self.seen.last().map_or(Vec3::ZERO, |e| e.1),
            Some(0) => {
                let (s0, o0) = self.seen[0];
                let head: Vec<&(f32, Vec3)> = self.seen.iter().take_while(|e| e.0 < s0 + 0.02).collect();
                let span = head.last().map_or(0.0, |e| e.0 - s0);
                if span < 0.005 { return o0; }
                // Least-squares slope of the offset over s, capped to what yaw and bow can give
                let n = head.len() as f32;
                let sm = head.iter().map(|e| e.0).sum::<f32>() / n;
                let om = head.iter().fold(Vec3::ZERO, |a, e| a + e.1) / n;
                let sxx = head.iter().map(|e| (e.0 - sm) * (e.0 - sm)).sum::<f32>();
                let sxo = head.iter().fold(Vec3::ZERO, |a, e| a + (e.1 - om) * (e.0 - sm));
                let slope = (sxo / sxx).clamp_length_max(0.06);
                om + slope * (s - sm)
            }
            Some(i) => {
                let ((s0, o0), (s1, o1)) = (self.seen[i - 1], self.seen[i]);
                let f = if s1 > s0 { (s - s0) / (s1 - s0) } else { 0.0 };
                o0 + (o1 - o0) * f
            }
        }
    }

    /// Updates from this tick's profile (taken at the current TCP) and sets the tick's target.
    pub fn observe(&mut self, env: &SeamEnv, scan: &[Option<f32>]) {
        let tcp = env.arm.tcp(&env.world);
        if self.start.is_none() { self.start = Some(tcp); }
        if self.sensing {
            let pts = env.cfg.profiler.points(&tcp, scan);
            if let Some(p) = find_seam(env.seam.joint, &pts) {
                let (lo, hi) = env.seam.s_window();
                let s = env.seam.nominal.project(p, lo, hi);
                let off = p - env.seam.nominal_root(s);
                if off.length() < SEARCH_WINDOW && self.seen.last().map_or(true, |&(last, _)| s > last + 1.0e-4) {
                    self.seen.push((s, off));
                }
            }
        }
        let s = env.s_torch();
        let t = env.seam.nominal.tangent(s);
        let rot = env.seam.torch_rot(&env.cfg, t);
        let root = env.seam.nominal_root(s) + self.offset_at(s);
        let goal = Isometry { pos: root - rot * Vec3::Y * env.cfg.standoff, rot };
        let a = env.approach();
        let pose = if a < 1.0 {
            let st = self.start.unwrap();
            Isometry { pos: st.pos.lerp(goal.pos, a), rot: st.rot.slerp(goal.rot, a).normalize() }
        } else { goal };
        let vel = if env.travelling() { t * env.seam.speed } else { Vec3::ZERO };
        self.target = Some(TcpTarget { pose, vel });
    }

    /// Joint torques for one substep (`k` substeps into the tick).
    pub fn torques(&mut self, world: &World, arm: &Arm, k: u32, dt: f32) -> [f32; JOINTS] {
        let mut t = self.target.expect("observe before torques");
        t.pose.pos += t.vel * (k as f32 * dt);
        self.servo.torques(world, arm, &t, dt)
    }
}

// ---------------------------------------------------------------------------------------------
// Episode
// ---------------------------------------------------------------------------------------------

/// Operator (human in the loop) inputs. `Override` takes the arm: the operator's jog moves the
/// target by (lateral, height) metres in the seam frame from where the TCP was, and the
/// controller's torques are ignored until `Resume`, after which the controller has a second to
/// bring the torch back before the seam can count as lost. `Stop` brakes the arm and holds it
/// where the stop came (joint PD on that pose, gravity held), then ends the episode.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OperatorCmd { Pause, Resume, Override { lateral: f32, height: f32 }, Stop }

pub struct SeamEnv {
    pub world: World,
    pub arm: Arm,
    pub seam: Seam,
    pub rng: Rng,
    pub cfg: SeamConfig,
    pub ep: String,
    pub ep_seed: u64,
    pub tick: u64,
    pub max_ticks: u64,
    /// Travel clock (s): stops while paused or overridden.
    pub travel_t: f32,
    pub paused: bool,
    pub override_target: Option<(TcpTarget, CartesianServo)>,
    /// Emergency stop: the tick it came and the joint angles it holds.
    pub stopped: Option<(u64, [f32; JOINTS])>,
    pub result: Option<&'static str>,
    pub off_ticks: u32,
    /// No seam-lost check before this tick (recovery after an operator override).
    pub grace_until: u64,
    /// Last projection of the TCP onto the real seam (warm start).
    s_tcp: f32,
    pub scan: Vec<Option<f32>>,
    pub workpiece_tris: usize,
}

impl SeamEnv {
    pub fn ep_seed(seed: u64, ep_index: u64) -> u64 { seed ^ ep_index.wrapping_mul(0xA24B_AED4_963E_E407) }

    pub fn new(cfg: &SeamConfig, ep_index: u64, seed: u64) -> Self {
        let ep_seed = Self::ep_seed(seed, ep_index);
        let mut rng = Rng::new(ep_seed);
        let seam = Seam::generate(cfg, &mut rng);
        let mut world = World::with_capacity(16, 16);
        world.set_rng_seed(ep_seed);
        // Fixture table under the plates
        let table = world.add_body(Isometry { pos: Vec3::new(0.0, -PLATE_T - 0.051, 0.0), rot: Quat::IDENTITY }, Velocity::default(), MassProps::infinite(), false);
        world.add_collider(table, Shape::Box { hx: 1.2, hy: 0.05, hz: 1.2 }, material(MaterialId::Steel));
        let wp = SeamWorkpiece::build(&seam);
        let workpiece_tris = wp.triangles();
        world.add_environment(Box::new(wp));
        // Ready pose: 80 mm back along the torch from the CAD run-on start, then a spread
        let s0 = -cfg.profiler.lookahead;
        let rot = seam.torch_rot(cfg, seam.nominal.tangent(s0));
        let start = Isometry { pos: seam.nominal_root(s0) - rot * Vec3::Y * 0.08, rot };
        let p = start.pos;
        let q0 = [(-p.z).atan2(p.x), -0.6, -0.6, 0.0, -1.0, 0.0];
        let mut q = arm::ik(&start, q0, 300);
        for x in q.iter_mut() { *x = arm::wrap(*x + rng.range(-1.0, 1.0) * cfg.start_spread); }
        let arm = Arm::build(&mut world, &cfg.arm, Vec3::ZERO, q);
        let travel_s = (seam.len() + cfg.profiler.lookahead) / seam.speed;
        let max_ticks = ((cfg.approach_s + cfg.settle_s + travel_s + 5.0) * CONTROL_HZ as f32) as u64;
        let mut env = Self {
            world, arm, seam, rng, cfg: *cfg, ep: format!("{}-{}", cfg.scenario(), ep_index), ep_seed,
            tick: 0, max_ticks, travel_t: 0.0, paused: false, override_target: None, stopped: None,
            result: None, off_ticks: 0, grace_until: 0, s_tcp: 0.0, scan: Vec::new(), workpiece_tris,
        };
        env.scan = env.read_profiler();
        env
    }

    pub fn seconds(&self) -> f32 { self.tick as f32 / CONTROL_HZ as f32 }
    pub fn done(&self) -> bool { self.result.is_some() }

    /// Approach progress in [0, 1] (min-jerk).
    pub fn approach(&self) -> f32 {
        let x = (self.seconds() / self.cfg.approach_s).clamp(0.0, 1.0);
        x * x * x * (10.0 - 15.0 * x + 6.0 * x * x)
    }

    pub fn travelling(&self) -> bool {
        self.seconds() >= self.cfg.approach_s + self.cfg.settle_s && !self.paused && self.override_target.is_none()
            && self.stopped.is_none() && self.s_torch() < self.seam.len()
    }

    /// Where along the seam the torch should be now (m). Travel starts one profiler look-ahead
    /// before the seam (a run-on over the plates), so the profile has covered the seam's start
    /// by the time the torch gets there; tracking is scored from s = 0.
    pub fn s_torch(&self) -> f32 { (self.travel_t * self.seam.speed - self.cfg.profiler.lookahead).min(self.seam.len()) }

    /// Whether the torch is on the seam proper (scored).
    pub fn on_seam(&self) -> bool { self.travelling() && self.s_torch() >= 0.0 }

    /// The profiler as read this tick: exact ranges plus seeded noise. One draw per ray, hit or
    /// not (a miss discards it), so the noise stream does not depend on which rays hit: two
    /// episodes from the same seed see the same noise whatever their controllers do.
    fn read_profiler(&mut self) -> Vec<Option<f32>> {
        let clean = self.cfg.profiler.scan(&self.world, &self.arm);
        let noise = self.cfg.noise;
        clean.into_iter().map(|r| { let n = self.rng.range(-noise, noise); r.map(|r| r + n) }).collect()
    }

    /// The ideal TCP at the TCP's own position along the real seam, and the seam frame there.
    pub fn ideal(&mut self) -> (Vec3, Vec3, f32) {
        let tcp = self.arm.tcp(&self.world).pos;
        let (lo, hi) = if self.tick == 0 { self.seam.s_window() } else { (self.s_tcp - 0.03, self.s_tcp + 0.03) };
        self.s_tcp = self.seam.actual.project(tcp, lo, hi);
        let s = self.s_tcp;
        let rot = self.seam.torch_rot(&self.cfg, self.seam.actual.tangent(s));
        (self.seam.true_root(s) - rot * Vec3::Y * self.cfg.standoff, self.seam.actual.lateral(s), s)
    }

    /// Tracking error of the TCP against the real seam: (lateral, height) in metres.
    pub fn error(&mut self) -> [f32; 2] {
        let (ideal, lat, _) = self.ideal();
        let e = self.arm.tcp(&self.world).pos - ideal;
        [e.dot(lat), e.y]
    }

    /// Applies an operator input now; returns its protocol line.
    pub fn operator(&mut self, cmd: OperatorCmd) -> Line {
        let who = self.arm.tool().0;
        let (name, data) = match cmd {
            OperatorCmd::Pause => { self.paused = true; ("pause", serde_json::Value::Null) }
            OperatorCmd::Resume => {
                if self.override_target.take().is_some() { self.grace_until = self.tick + CONTROL_HZ as u64; }
                self.paused = false;
                ("resume", serde_json::Value::Null)
            }
            OperatorCmd::Override { lateral, height } => {
                let tcp = self.arm.tcp(&self.world);
                let lat = self.seam.actual.lateral(self.s_tcp);
                let pose = Isometry { pos: tcp.pos + lat * lateral + Vec3::Y * height, rot: tcp.rot };
                let servo = CartesianServo::new(&self.arm, &self.world);
                self.override_target = Some((TcpTarget { pose, vel: Vec3::ZERO }, servo));
                ("override", serde_json::json!({ "lateral_m": lateral, "height_m": height }))
            }
            OperatorCmd::Stop => { self.stopped = Some((self.tick, self.arm.angles(&self.world))); ("stop", serde_json::Value::Null) }
        };
        Line::Operator { tick: self.tick, ep: self.ep.clone(), who, cmd: name.into(), data }
    }

    /// One control tick: `SUBSTEPS` physics steps with the controller's torques (unless the
    /// operator holds the arm), then contacts, progress and tracking checks. Returns events.
    pub fn step(&mut self, mut servo: impl FnMut(&World, &Arm, u32, f32) -> [f32; JOINTS]) -> Vec<Line> {
        let dt = 1.0 / (CONTROL_HZ * SUBSTEPS) as f32;
        let travelling = self.travelling();
        let mut op = self.override_target.take();
        let stopped = self.stopped;
        let hold = CartesianServo::new(&self.arm, &self.world);
        for k in 0..SUBSTEPS {
            let t = if let Some((_, q_stop)) = stopped {
                // Brake and hold where the stop came (joint PD on the stop pose, gravity held)
                let (q, r, g) = (self.arm.angles(&self.world), self.arm.rates(&self.world), self.arm.gravity_torques(&self.world));
                std::array::from_fn(|j| hold.kp[j] * arm::wrap(q_stop[j] - q[j]) - hold.kd[j] * r[j] + g[j])
            } else if let Some((target, s)) = op.as_mut() {
                s.torques(&self.world, &self.arm, target, dt)
            } else {
                servo(&self.world, &self.arm, k, dt)
            };
            self.arm.apply(&mut self.world, t, dt);
            self.world.step(dt);
        }
        self.override_target = op;
        self.tick += 1;
        if travelling { self.travel_t += 1.0 / CONTROL_HZ as f32; }

        let (tick, ep, who) = (self.tick, self.ep.clone(), self.arm.tool().0);
        let mut events = Vec::new();
        let c = self.arm.contacts(&self.world);
        if let Some((a, b)) = c.self_pair {
            events.push(Line::Event { tick, ep: ep.clone(), e: "self_contact".into(), who, data: serde_json::json!({ "links": [LINK_NAMES[a], LINK_NAMES[b]] }) });
            self.result = Some("collided");
        } else if let Some(l) = c.env_link {
            events.push(Line::Event { tick, ep: ep.clone(), e: "tool_contact".into(), who, data: serde_json::json!({ "link": LINK_NAMES[l] }) });
            self.result = Some("collided");
        }
        if self.result.is_none() {
            if let Some((at, _)) = self.stopped {
                if tick >= at + STOP_TICKS as u64 { self.result = Some("stopped"); }
            } else if self.seconds() >= self.cfg.approach_s + self.cfg.settle_s && self.override_target.is_none() && tick >= self.grace_until {
                let e = self.error();
                if e[0].abs() > LOST_ERR || e[1].abs() > LOST_ERR { self.off_ticks += 1; } else { self.off_ticks = 0; }
                if self.off_ticks == LOST_TICKS {
                    events.push(Line::Event { tick, ep: ep.clone(), e: "seam_lost".into(), who, data: serde_json::json!({ "err_m": e }) });
                    self.result = Some("lost");
                } else if self.s_torch() >= self.seam.len() {
                    self.result = Some("completed");
                }
            }
            if self.result.is_none() && tick >= self.max_ticks { self.result = Some("timeout"); }
        }
        self.scan = self.read_profiler();
        events
    }

    pub fn kind_spec(&self) -> KindSpec {
        KindSpec { sensors: Some(vec![self.cfg.profiler.spec()]), ..arm::kind_spec() }
    }

    pub fn start_line(&self) -> Line {
        let len = self.seam.len();
        let n = (len / 0.001).ceil() as usize;
        let truth: Vec<[f32; 3]> = (0..=n).map(|i| self.seam.true_root(len * i as f32 / n as f32).to_array()).collect();
        let scene = serde_json::json!({
            "seam": self.cfg.shape.name(), "joint": self.seam.joint.name(), "length_m": len,
            "plate_t_m": PLATE_T, "standoff_m": self.cfg.standoff,
            "offset_m": self.seam.offset.to_array(), "yaw_rad": self.seam.actual.yaw, "bow_m": self.seam.actual.bow,
            "gap": { "g0_m": self.seam.gap.g0, "amp_m": self.seam.gap.amp, "wavelength_m": self.seam.gap.wavelength },
            "speed_mps": self.seam.speed, "noise_m": self.cfg.noise,
            "approach_s": self.cfg.approach_s, "settle_s": self.cfg.settle_s,
            "truth": truth,
        });
        Line::EpisodeStart { tick: 0, ep: self.ep.clone(), scenario: self.cfg.scenario(), members: vec![self.arm.tool().0], seed: self.ep_seed, scene: Some(scene) }
    }

    /// The entity with its profiler scan, the ideal TCP point (`target`) and the error (`err`).
    pub fn observe(&mut self) -> EntState {
        let mut e = self.arm.observe(&self.world);
        let (ideal, lat, _) = self.ideal();
        let d = self.arm.tcp(&self.world).pos - ideal;
        e.target = Some(ideal.to_array());
        e.err = Some(vec![d.dot(lat), d.y]);
        e.scan = Some(self.scan.clone());
        e
    }

    pub fn sample_line(&mut self, act: [f32; JOINTS], demo: Option<[f32; JOINTS]>) -> Line {
        let mut e = self.observe();
        e.act = act.to_vec();
        e.demo = demo.map(|d| d.to_vec());
        Line::Sample { tick: self.tick, ep: self.ep.clone(), ents: vec![e], hash: hash_hex(&self.world.step_hash()) }
    }

    pub fn end_line(&self) -> Line {
        Line::EpisodeEnd { tick: self.tick, ep: self.ep.clone(), result: self.result.unwrap_or("timeout").into(), duration_s: self.seconds() }
    }
}

/// Learner-facing features at the control rate, in `SEAM_FEATURES` order: the seam point the
/// profile shows, relative to the TCP in the tool frame (travel, torch, across; zero when not
/// found) and whether it was found; then per joint the sine and cosine of the wrapped angle and
/// the rate.
pub const SEAM_FEATURES: [&str; 22] = ["seam_fwd", "seam_torch", "seam_across", "seam_found",
    "sin_q1", "cos_q1", "sin_q2", "cos_q2", "sin_q3", "cos_q3", "sin_q4", "cos_q4", "sin_q5", "cos_q5", "sin_q6", "cos_q6",
    "qd1", "qd2", "qd3", "qd4", "qd5", "qd6"];

pub fn seam_obs(env: &SeamEnv) -> [f32; 22] {
    let tcp = env.arm.tcp(&env.world);
    let mut o = [0.0; 22];
    if let Some(p) = find_seam(env.seam.joint, &env.cfg.profiler.points(&tcp, &env.scan)) {
        let d = tcp.rot.conjugate() * (p - tcp.pos);
        o[..4].copy_from_slice(&[d.x, d.y, d.z, 1.0]);
    }
    let (q, qd) = (env.arm.angles(&env.world), env.arm.rates(&env.world));
    for j in 0..JOINTS {
        o[4 + 2 * j] = q[j].sin();
        o[5 + 2 * j] = q[j].cos();
        o[16 + j] = qd[j];
    }
    o
}

/// Tracking result of one episode: errors (mm) over the travel phase, and the outcome.
#[derive(Clone, Debug)]
pub struct TrackStats {
    pub lat_mean_mm: f32,
    pub lat_p95_mm: f32,
    pub h_mean_mm: f32,
    pub h_p95_mm: f32,
    pub result: &'static str,
}

impl TrackStats {
    pub fn from_errors(errs: &[[f32; 2]], result: &'static str) -> Self {
        let stat = |k: usize| -> (f32, f32) {
            if errs.is_empty() { return (f32::NAN, f32::NAN); }
            let mut v: Vec<f32> = errs.iter().map(|e| e[k].abs() * 1000.0).collect();
            v.sort_by(|a, b| a.partial_cmp(b).unwrap());
            (v.iter().sum::<f32>() / v.len() as f32, v[((v.len() as f32 * 0.95) as usize).min(v.len() - 1)])
        };
        let ((lm, lp), (hm, hp)) = (stat(0), stat(1));
        Self { lat_mean_mm: lm, lat_p95_mm: lp, h_mean_mm: hm, h_p95_mm: hp, result }
    }
}

/// Runs one episode under the teacher (`sensing`) or the floor, with no exploration noise and
/// no samples; returns its tracking statistics over the seam proper.
pub fn track(cfg: &SeamConfig, ep_index: u64, seed: u64, sensing: bool) -> TrackStats {
    let mut env = SeamEnv::new(cfg, ep_index, seed);
    let mut ctl = if sensing { SeamTracker::teacher(&env) } else { SeamTracker::nominal(&env) };
    let mut errs = Vec::new();
    while !env.done() {
        let scan = env.scan.clone();
        ctl.observe(&env, &scan);
        env.step(|w, a, k, dt| ctl.torques(w, a, k, dt));
        if env.on_seam() || env.result == Some("completed") { errs.push(env.error()); }
    }
    TrackStats::from_errors(&errs, env.result.unwrap_or("timeout"))
}

/// A rollout: the scene, DART exploration noise on the teacher's torques (a fraction of each
/// joint's limit, uniform, held for a control tick), the sample cadence and a scripted operator
/// (inputs at episode seconds).
#[derive(Clone, Debug)]
pub struct SeamRollout {
    pub cfg: SeamConfig,
    pub noise: f32,
    pub sample_every: u32,
    pub operator: Vec<(f32, OperatorCmd)>,
}

impl SeamRollout {
    pub fn new(cfg: SeamConfig) -> Self { Self { cfg, noise: 0.05, sample_every: 3, operator: Vec::new() } }

    pub fn meta_line(&self, seed: u64) -> Line {
        Line::Meta {
            tick: 0, v: crate::protocol::PROTOCOL_VERSION.into(), source: "riftphys".into(), scenario: self.cfg.scenario(),
            sim_hz: CONTROL_HZ, sample_every: self.sample_every, seed,
            kinds: vec![KindSpec { sensors: Some(vec![self.cfg.profiler.spec()]), ..arm::kind_spec() }],
        }
    }

    /// One demonstration episode under the noisy teacher: its lines (no meta line) and its
    /// tracking statistics. Samples pair the state with the torques chosen from it: `demo` the
    /// teacher's (first servo substep), `act` what ran (demo plus the tick's noise). While the
    /// operator overrides or stops the arm, the controller's choice is still recorded and the
    /// `operator` lines say who held the arm.
    pub fn run_episode(&self, ep_index: u64, seed: u64) -> (Vec<Line>, TrackStats) {
        let mut env = SeamEnv::new(&self.cfg, ep_index, seed);
        let mut ctl = SeamTracker::teacher(&env);
        let mut lines = vec![env.start_line()];
        let mut ops = self.operator.clone();
        ops.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        let mut ops = ops.into_iter().peekable();
        let mut errs = Vec::new();
        let dt = 1.0 / (CONTROL_HZ * SUBSTEPS) as f32;
        while !env.done() {
            while ops.peek().map_or(false, |o| o.0 <= env.seconds()) {
                let (_, cmd) = ops.next().unwrap();
                lines.push(env.operator(cmd));
            }
            let scan = env.scan.clone();
            ctl.observe(&env, &scan);
            let demo = ctl.torques(&env.world, &env.arm, 0, dt);
            let n: [f32; JOINTS] = std::array::from_fn(|j| env.rng.range(-1.0, 1.0) * self.noise * MAX_TORQUE[j]);
            let act: [f32; JOINTS] = std::array::from_fn(|j| (demo[j] + n[j]).clamp(-MAX_TORQUE[j], MAX_TORQUE[j]));
            if (env.tick + 1) % self.sample_every as u64 == 0 {
                lines.push(env.sample_line(act, Some(demo)));
            }
            lines.extend(env.step(|w, a, k, dt| if k == 0 { act } else {
                let t = ctl.torques(w, a, k, dt);
                std::array::from_fn(|j| t[j] + n[j])
            }));
            if env.on_seam() || env.result == Some("completed") { errs.push(env.error()); }
        }
        lines.push(env.end_line());
        (lines, TrackStats::from_errors(&errs, env.result.unwrap_or("timeout")))
    }
}

/// Parses an operator script: comma-separated `cmd@seconds`, with `override@seconds:lat_mm:height_mm`.
pub fn parse_operator(script: &str) -> Result<Vec<(f32, OperatorCmd)>, String> {
    script.split(',').filter(|x| !x.trim().is_empty()).map(|item| {
        let (cmd, rest) = item.trim().split_once('@').ok_or(format!("{}: expected cmd@seconds", item))?;
        let mut parts = rest.split(':');
        let t: f32 = parts.next().unwrap().parse().map_err(|_| format!("{}: bad time", item))?;
        let mm = |p: Option<&str>| -> Result<f32, String> { p.unwrap_or("0").parse::<f32>().map(|x| x / 1000.0).map_err(|_| format!("{}: bad mm", item)) };
        let c = match cmd {
            "pause" => OperatorCmd::Pause,
            "resume" => OperatorCmd::Resume,
            "stop" => OperatorCmd::Stop,
            "override" => OperatorCmd::Override { lateral: mm(parts.next())?, height: mm(parts.next())? },
            _ => return Err(format!("{}: unknown command (pause, resume, override, stop)", item)),
        };
        Ok((t, c))
    }).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn short(shape: SeamShape, joint: JointType) -> SeamConfig {
        let mut c = SeamConfig { shape, joint, ..Default::default() };
        c.line_len = 0.12;
        c.ring_arc = 0.3;
        c.speed = (0.03, 0.04);
        c
    }

    /// PARITY_SPEC 3.9: a 10 s seam rollout (0.15 m of line at 20 mm/s after the 2.5 s approach
    /// and settle) completes with the arm's axis armature, its tracking within a loose bound and
    /// reported beside the default arm's.
    #[test]
    fn an_axis_armature_arm_tracks_a_seam() {
        for armature in [crate::arm::ArmArmature::Isotropic, crate::arm::ArmArmature::Axis] {
            let mut c = SeamConfig { arm: ArmConfig::for_armature(armature), ..Default::default() };
            c.line_len = 0.15;
            c.speed = (0.02, 0.02);
            let (lines, st) = SeamRollout::new(c).run_episode(0, 7);
            println!("{:?}: {} | lateral mean {:.3} mm, p95 {:.3} mm | height mean {:.3} mm, p95 {:.3} mm | {} lines",
                armature, st.result, st.lat_mean_mm, st.lat_p95_mm, st.h_mean_mm, st.h_p95_mm, lines.len());
            // The rollout carries DART exploration noise (5% of each joint's limit), so its p95
            // runs near 2.5 mm, not the noiseless eval's 0.25: the bound catches an abort or a
            // drift, not a tuning change
            assert_eq!(st.result, "completed", "{:?}", armature);
            assert!(st.lat_p95_mm < 4.0 && st.h_p95_mm < 4.0, "{:?}: lateral p95 {:.3} mm, height p95 {:.3} mm", armature, st.lat_p95_mm, st.h_p95_mm);
        }
    }

    /// A seam teacher's torques, recorded on the arm at its own iterations over its first second
    /// (episode `ep`, seed 1000, a v-groove line), replayed open loop on that arm and on the same
    /// arm at 512 iterations: `arm::iteration_rate_error`'s (first tick, over 1 s, joint).
    fn teacher_replay_error(armature: crate::arm::ArmArmature, iters: u32, ep: u64) -> (f32, f32, usize) {
        let cfg = SeamConfig { joint: JointType::V, arm: ArmConfig::for_armature(armature), ..Default::default() };
        let mut env = SeamEnv::new(&cfg, ep, 1000);
        let mut ctl = SeamTracker::teacher(&env);
        let mut rec: Vec<[f32; JOINTS]> = Vec::new();
        for _ in 0..CONTROL_HZ {
            let scan = env.scan.clone();
            ctl.observe(&env, &scan);
            env.step(|w, a, k, dt| { let t = ctl.torques(w, a, k, dt); rec.push(t); t });
        }
        let mut lo = SeamEnv::new(&SeamConfig { arm: ArmConfig { solver_iters: (iters, 6), ..cfg.arm }, ..cfg }, ep, 1000);
        let mut hi = SeamEnv::new(&SeamConfig { arm: ArmConfig { solver_iters: (512, 6), ..cfg.arm }, ..cfg }, ep, 1000);
        crate::arm::iteration_rate_error((&mut lo.world, &mut lo.arm), (&mut hi.world, &mut hi.arm), CONTROL_HZ,
            |tick, k, _, _| rec[(tick * SUBSTEPS + k) as usize])
    }

    /// PARITY_SPEC 3.9, the Reviewer's cold-substep check: a seam teacher's first second, replayed
    /// open loop from rest, moves the arm at its own iterations (8; 96 in `Axis`) within 2% of the
    /// largest joint rate of the same arm at 512, in both modes.
    #[test]
    fn a_teacher_replay_converges_at_the_arms_own_iterations() {
        for armature in [crate::arm::ArmArmature::Isotropic, crate::arm::ArmArmature::Axis] {
            let iters = ArmConfig::for_armature(armature).solver_iters.0;
            let (_, all, j) = teacher_replay_error(armature, iters, 0);
            println!("{:?} at {} iterations: {:.2}% over 1 s (worst on {})", armature, iters, 100.0 * all, crate::arm::JOINT_NAMES[j]);
            assert!(all <= 0.02, "{:?}: {:.2}% on {}", armature, 100.0 * all, crate::arm::JOINT_NAMES[j]);
        }
    }

    /// Probe (PARITY_SPEC 3.9, the Reviewer's cold-substep check, input (c)). Run with
    /// --ignored --nocapture.
    #[test]
    #[ignore]
    fn d6_teacher_replay_probe() {
        use crate::arm::ArmArmature::{Axis, Isotropic};
        for (armature, iters) in [(Isotropic, 8), (Axis, 8), (Axis, 16), (Axis, 32), (Axis, 64), (Axis, 96), (Axis, 128)] {
            for ep in 0..3 {
                let (first, all, j) = teacher_replay_error(armature, iters, ep);
                println!("{:?} at {:2} iterations, teacher replay ep {}: first tick {:6.2}%, over 1 s {:6.2}% (worst on {})",
                    armature, iters, ep, 100.0 * first, 100.0 * all, crate::arm::JOINT_NAMES[j]);
            }
        }
    }

    #[test]
    fn profiler_finds_the_true_seam_from_above() {
        for joint in [JointType::Butt, JointType::V, JointType::Fillet] {
            let cfg = SeamConfig { joint, ..Default::default() };
            let env = SeamEnv::new(&cfg, 0, 7);
            // Scan from an ideal pose over the true seam at s = 0.2, without noise
            let s = 0.2;
            let rot = env.seam.torch_rot(&cfg, env.seam.actual.tangent(s));
            let tcp = Isometry { pos: env.seam.true_root(s) - rot * Vec3::Y * cfg.standoff, rot };
            let (o, torch, across) = cfg.profiler.frame(&tcp);
            let ranges: Vec<Option<f32>> = (0..cfg.profiler.n).map(|i| {
                env.world.raycast_exact(o, cfg.profiler.ray(i, torch, across), cfg.profiler.max_range, |b| env.arm.link_index(b).is_some())
                    .map(|h| h.toi * cfg.profiler.max_range)
            }).collect();
            assert!(ranges.iter().filter(|r| r.is_some()).count() > 50, "{:?}: too few hits", joint);
            let p = find_seam(joint, &cfg.profiler.points(&tcp, &ranges)).expect("seam found");
            // The profile is 25 mm ahead: compare with the true seam there
            let s_ahead = env.seam.actual.project(p, s, s + 0.05);
            let err = p - env.seam.true_root(s_ahead);
            assert!(err.length() < 0.0005, "{:?}: found seam off by {:.2} mm", joint, err.length() * 1000.0);
            assert!((s_ahead - s - cfg.profiler.lookahead).abs() < 0.005);
        }
    }

    #[test]
    fn teacher_tracks_and_floor_does_not() {
        for joint in [JointType::Butt, JointType::V, JointType::Fillet] {
            let cfg = short(SeamShape::Line, joint);
            let t = track(&cfg, 0, 11, true);
            let f = track(&cfg, 0, 11, false);
            println!("{:?}: teacher {:?}, floor {:?}", joint, t, f);
            assert_eq!(t.result, "completed", "{:?}: teacher {:?}", joint, t);
            assert!(t.lat_p95_mm < 0.6 && t.h_p95_mm < 0.6, "{:?}: teacher {:?}", joint, t);
            assert!(f.lat_mean_mm > 2.0 * t.lat_mean_mm, "{:?}: floor {:?} vs teacher {:?}", joint, f, t);
        }
    }

    #[test]
    fn ring_seam_turns_the_base() {
        let cfg = short(SeamShape::Ring, JointType::V);
        let t = track(&cfg, 0, 3, true);
        assert_eq!(t.result, "completed", "{:?}", t);
        assert!(t.lat_p95_mm < 0.8, "{:?}", t);
    }

    #[test]
    fn rollouts_replay_bit_exactly() {
        let mut r = SeamRollout::new(short(SeamShape::Line, JointType::Butt));
        r.operator = parse_operator("pause@3,resume@3.5,override@4:1:3,resume@4.5").unwrap();
        let (a, sa) = r.run_episode(2, 9);
        let (b, _) = r.run_episode(2, 9);
        let ja: Vec<String> = a.iter().map(|l| serde_json::to_string(l).unwrap()).collect();
        let jb: Vec<String> = b.iter().map(|l| serde_json::to_string(l).unwrap()).collect();
        assert_eq!(ja, jb);
        assert_eq!(sa.result, "completed", "{:?}", sa);
        assert_eq!(a.iter().filter(|l| matches!(l, Line::Operator { .. })).count(), 4);
        let samples: Vec<&Line> = a.iter().filter(|l| matches!(l, Line::Sample { .. })).collect();
        assert!(samples.len() > 100);
        if let Line::Sample { ents, .. } = samples[samples.len() * 3 / 4] {
            let e = &ents[0];
            assert_eq!(e.scan.as_ref().unwrap().len(), 64);
            assert!(e.err.as_ref().unwrap().iter().all(|x| x.abs() < 0.002), "err {:?}", e.err);
            assert!(e.demo.is_some() && e.joints.as_ref().unwrap().len() == JOINTS);
        }
        // Another episode differs
        let (c, _) = r.run_episode(3, 9);
        assert_ne!(serde_json::to_string(&a[1]).unwrap(), serde_json::to_string(&c[1]).unwrap());
    }

    #[test]
    fn operator_pause_override_and_stop() {
        let cfg = short(SeamShape::Line, JointType::V);
        let mut env = SeamEnv::new(&cfg, 0, 5);
        let mut ctl = SeamTracker::teacher(&env);
        let mut lines = Vec::new();
        let run = |env: &mut SeamEnv, ctl: &mut SeamTracker, ticks: u32| {
            for _ in 0..ticks {
                let scan = env.scan.clone();
                ctl.observe(env, &scan);
                env.step(|w, a, k, dt| ctl.torques(w, a, k, dt));
            }
        };
        run(&mut env, &mut ctl, 180);
        lines.push(env.operator(OperatorCmd::Pause));
        let s0 = env.s_torch();
        run(&mut env, &mut ctl, 60);
        assert_eq!(env.s_torch(), s0, "travel must stop while paused");
        let before = env.arm.tcp(&env.world).pos;
        lines.push(env.operator(OperatorCmd::Override { lateral: 0.004, height: 0.010 }));
        run(&mut env, &mut ctl, 90);
        let moved = env.arm.tcp(&env.world).pos - before;
        assert!((moved.y - 0.010).abs() < 0.001, "jog height {:.4}", moved.y);
        lines.push(env.operator(OperatorCmd::Resume));
        run(&mut env, &mut ctl, 60);
        assert!(env.s_torch() > s0);
        lines.push(env.operator(OperatorCmd::Stop));
        run(&mut env, &mut ctl, STOP_TICKS + 1);
        assert_eq!(env.result, Some("stopped"));
        let speed = env.arm.tcp_vel(&env.world).0.length();
        assert!(speed < 0.01, "still moving at {:.3} m/s after the stop", speed);
        assert!(matches!(&lines[1], Line::Operator { cmd, .. } if cmd == "override"));
    }
}
