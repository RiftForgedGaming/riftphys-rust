
use riftphys_core::{BodyId, Isometry, JointId, Mat3, Quat, Scalar, Vec3};
use riftphys_dynamics::dynamics::Bodies;

/* ─────────────────────────  Quantization ───────────────────────── */

#[inline]
fn q6(x: f32) -> f32 {
    if !x.is_finite() { return 0.0; }
    (x * 1.0e6_f32).round() * 1.0e-6_f32
}

#[inline]
fn q6v(v: Vec3) -> Vec3 {
    Vec3::new(q6(v.x), q6(v.y), q6(v.z))
}

#[inline]
fn normalize_q6(v: Vec3) -> Vec3 {
    let v = q6v(v);
    let len2 = v.length_squared();
    if len2 > 1.0e-12 {
        q6v(v * (1.0 / len2.sqrt()))
    } else {
        Vec3::ZERO
    }
}

/* ─────────────────────────  Distance Joint ─────────────────── */

#[derive(Copy, Clone, Debug)]
pub struct DistanceJoint {
    pub a: BodyId,
    pub b: BodyId,
    pub rest: Scalar,
    pub compliance: Scalar,
    /// XPBD multiplier accumulated over the current step's iterations (zeroed by `begin_step`)
    pub lambda_acc: Scalar,
}

/* ─────────────────────────  D6 types ──────────── */

#[derive(Copy, Clone, Debug, Default)]
pub struct Drive {
    pub target: Scalar,
    pub kp: Scalar,
    pub kd: Scalar,
    pub max_impulse: Scalar,
    pub vel_mode: bool,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Limit {
    pub min: Scalar,
    pub max: Scalar,
}

#[derive(Copy, Clone, Debug)]
pub struct D6Axis {
    pub enabled: bool,
    pub compliance: Scalar,
    pub drive: Option<Drive>,
    pub limit: Option<Limit>,
    pub lambda_acc: Scalar,
}

impl Default for D6Axis {
    fn default() -> Self {
        Self {
            enabled: false,
            compliance: 0.0,
            drive: None,
            limit: None,
            lambda_acc: 0.0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Generic6Dof {
    pub a: BodyId,
    pub b: BodyId,
    pub fa: Isometry,
    pub fb: Isometry,
    pub t: [D6Axis; 3],
    pub r: [D6Axis; 3],
}

/* ─────────────────────────  Joint union + container ─────────────────────────── */

#[derive(Copy, Clone, Debug)]
pub enum JointKind {
    Distance(DistanceJoint),
    D6(Generic6Dof),
}

#[derive(Default)]
pub struct Joints {
    kinds: Vec<JointKind>,
}

impl Joints {
    pub fn new() -> Self {
        Self { kinds: Vec::new() }
    }
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.kinds.is_empty()
    }
    #[inline]
    pub fn kinds(&self) -> &[JointKind] {
        &self.kinds
    }
    #[inline]
    pub fn len(&self) -> usize {
        self.kinds.len()
    }
    pub fn set_distance_params(&mut self, id: JointId, rest: f32, compliance: f32) {
        if let Some(JointKind::Distance(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.rest = q6(rest);
            j.compliance = q6(compliance).max(0.0);
        }
    }

    pub fn add_distance_joint(&mut self, a: BodyId, b: BodyId, rest: Scalar, compliance: Scalar) -> JointId {
        self.kinds.push(JointKind::Distance(DistanceJoint {
            a,
            b,
            rest: q6(rest),
            compliance: q6(compliance).max(0.0),
            lambda_acc: 0.0,
        }));
        JointId((self.kinds.len() as u32) - 1)
    }

    /// Starts a step's position solve: zeroes every XPBD multiplier. The multiplier belongs to
    /// one step; carried over, a compliant row grew stiffer every step (a torsional spring sagged
    /// 21x too little at 60 Hz).
    pub fn begin_step(&mut self) {
        for k in &mut self.kinds {
            match k {
                JointKind::Distance(j) => j.lambda_acc = 0.0,
                JointKind::D6(j) => {
                    for r in j.t.iter_mut().chain(j.r.iter_mut()) { r.lambda_acc = 0.0; }
                }
            }
        }
    }

    pub fn add_d6(&mut self, j: Generic6Dof) -> JointId {
        self.kinds.push(JointKind::D6(j));
        JointId((self.kinds.len() as u32) - 1)
    }

    pub fn add_ball(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry) -> JointId {
        let mut j = Generic6Dof { a, b, fa, fb, t: Default::default(), r: Default::default() };
        for ax in 0..3 { j.t[ax].enabled = true; }
        for ax in 0..3 { j.r[ax].enabled = false; }
        self.add_d6(j)
    }

    pub fn add_hinge(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry, hinge_axis: usize) -> JointId {
        let mut j = Generic6Dof { a, b, fa, fb, t: Default::default(), r: Default::default() };
        for ax in 0..3 { j.t[ax].enabled = true; }
        for ax in 0..3 { j.r[ax].enabled = ax != hinge_axis; }
        self.add_d6(j)
    }

    pub fn add_universal(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry, ax0: usize, ax1: usize) -> JointId {
        let mut j = Generic6Dof { a, b, fa, fb, t: Default::default(), r: Default::default() };
        for ax in 0..3 { j.t[ax].enabled = true; }
        for ax in 0..3 { j.r[ax].enabled = !(ax == ax0 || ax == ax1); }
        self.add_d6(j)
    }

    pub fn set_d6_drive_t(&mut self, id: JointId, axis: usize, d: Option<Drive>) {
        if let Some(JointKind::D6(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.t[axis].drive = d;
            if j.t[axis].drive.is_some() { j.t[axis].enabled = true; }
        }
    }

    pub fn set_d6_drive_r(&mut self, id: JointId, axis: usize, d: Option<Drive>) {
        if let Some(JointKind::D6(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.r[axis].drive = d;
            if j.r[axis].drive.is_some() { j.r[axis].enabled = true; }
        }
    }

    pub fn set_d6_limit_t(&mut self, id: JointId, axis: usize, lim: Option<Limit>) {
        if let Some(JointKind::D6(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.t[axis].limit = lim;
            if j.t[axis].limit.is_some() { j.t[axis].enabled = true; }
        }
    }

    pub fn set_d6_limit_r(&mut self, id: JointId, axis: usize, lim: Option<Limit>) {
        if let Some(JointKind::D6(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.r[axis].limit = lim;
            if j.r[axis].limit.is_some() { j.r[axis].enabled = true; }
        }
    }

    /// Position-level joint solve (PBD/XPBD): corrections along the constraint directions at the
    /// current iterate.
    pub fn solve(&mut self, bodies: &mut Bodies, dt: Scalar, iterations: u32) {
        self.solve_step(bodies, dt, iterations, None, &[]);
    }

    /// Position-level joint solve with SHAKE projection: the error is measured at the current
    /// iterate, but corrections run along the directions (axes, lever arms, inertias) of `start`,
    /// the poses at the start of the step that the velocity solve used. Together with that
    /// velocity solve this is one impulse along the start-of-step constraint gradient, chosen so
    /// the bodies land on the constraint: symplectic, so a free joint keeps its energy instead of
    /// bleeding it at first order in dt (a hinged pendulum kept 90% of its swing energy per
    /// period at 62.5 Hz with `solve`). `start` is indexed by body id.
    pub fn solve_shake(&mut self, bodies: &mut Bodies, dt: Scalar, iterations: u32, start: &[Isometry]) {
        self.solve_step(bodies, dt, iterations, Some(start), &[]);
    }

    /// The position solve inside a world step: SHAKE directions from `start` if given (else
    /// PBD), and joints skipped when both bodies are `frozen` (static or asleep, indexed by body
    /// id; missing entries count as not frozen). A sleeping body gets no gravity, so a compliant
    /// row solved on it relaxed to zero sag while it slept.
    pub fn solve_step(&mut self, bodies: &mut Bodies, dt: Scalar, iterations: u32, start: Option<&[Isometry]>, frozen: &[bool]) {
        if self.kinds.is_empty() { return; }
        let is_frozen = |id: u32| frozen.get(id as usize).copied().unwrap_or(false);

        let dt = q6(dt);
        if !(dt > 0.0) { return; }
        // Not quantized: on the 1e-6 grid dt^2 itself was off by 2% at 240 Hz and 8% at 960 Hz
        // (1.736e-5 -> 1.7e-5, 1.085e-6 -> 1e-6), and every compliance with it.
        let dt2 = (dt * dt).max(1.0e-12);

        #[inline]
        fn alpha_dt2(c: Scalar, dt2: Scalar) -> Scalar {
            let c = q6(c).max(0.0);
            if c <= 0.0 { 0.0 } else { q6(c / dt2) }
        }

        for _ in 0..iterations.max(1) {
            for k in &mut self.kinds {
                let (a, b) = match k { JointKind::Distance(j) => (j.a.0, j.b.0), JointKind::D6(j) => (j.a.0, j.b.0) };
                if is_frozen(a) && is_frozen(b) { continue; }
                match k {
                    JointKind::Distance(j) => {
                        let s = start.map(|s| (frame(s, bodies, j.a.0), frame(s, bodies, j.b.0)));
                        solve_distance_row(bodies, j, dt, dt2, s)
                    }
                    JointKind::D6(j) => {
                        let s = start.map(|s| (frame(s, bodies, j.a.0), frame(s, bodies, j.b.0)));
                        solve_d6(bodies, j, dt, dt2, alpha_dt2, s)
                    }
                }
            }
        }
    }
}

/* ─────────────────────────  Solvers  ───────────────────────── */

/// Largest rotation (radians) a body may make within one step for SHAKE directions to be used.
/// Beyond it the start-of-step directions are too far from the current ones for the iteration
/// to converge well, so that joint falls back to current-iterate directions.
const SHAKE_MAX_TURN: f32 = 0.5;

/// The start-of-step pose of body `id` (its current pose if `start` doesn't cover it).
#[inline]
fn frame(start: &[Isometry], bodies: &Bodies, id: u32) -> Isometry {
    start.get(id as usize).copied().unwrap_or_else(|| bodies.pose(id))
}

/// Rotation between two orientations, in radians.
#[inline]
fn turned(a: Quat, b: Quat) -> f32 {
    let d = (a * b.conjugate()).w.abs().min(1.0);
    2.0 * d.acos()
}

/// World inverse inertia of body `id` at orientation `rot`.
#[inline]
fn inv_inertia_at(bodies: &Bodies, id: u32, rot: Quat) -> Mat3 {
    let r = Mat3::from_quat(rot);
    r * bodies.inv_inertia_local_of(id) * r.transpose()
}

fn solve_distance_row(bodies: &mut Bodies, j: &mut DistanceJoint, dt: Scalar, dt2: Scalar, start: Option<(Isometry, Isometry)>) {
    let ia = j.a.0;
    let ib = j.b.0;

    let wa = bodies.inv_mass_of(ia);
    let wb = bodies.inv_mass_of(ib);
    let w_sum = q6(wa + wb);
    if w_sum <= 0.0 { return; }

    let pa = bodies.pose(ia).pos;
    let pb = bodies.pose(ib).pos;

    let d = pb - pa;
    let len2 = d.length_squared();
    if !len2.is_finite() || len2 <= 1.0e-20 { return; }

    let len = len2.sqrt();
    if len <= 1.0e-6 { return; }

    let n_cur = normalize_q6(d / len);
    if n_cur == Vec3::ZERO { return; }
    // SHAKE: correct along the start-of-step direction
    let mut n = n_cur;
    if let Some((sa, sb)) = start {
        let n0 = normalize_q6(sb.pos - sa.pos);
        if n0 != Vec3::ZERO && n0.dot(n_cur) > SHAKE_MAX_TURN.cos() { n = n0; }
    }

    let c_val = q6(len - q6(j.rest));

    let alpha = if j.compliance <= 0.0 { 0.0 } else { q6(q6(j.compliance).max(0.0) / dt2) };
    let denom = q6(w_sum + alpha);
    if denom <= 0.0 { return; }

    // XPBD: the multiplier accumulates over the step's iterations, so a compliant joint settles
    // at C = -alpha lambda (stretch = compliance x force) whatever the iteration count; without it
    // the stiffness depended on the iteration count and compliance was nearly ignored. The error
    // grows along n_cur; a correction along n changes it at the rate n . n_cur.
    let lambda_old = q6(j.lambda_acc);
    let lambda = q6(-(c_val + q6(alpha * lambda_old)) / (denom * n.dot(n_cur)));
    if lambda.abs() <= 1.0e-9 { return; }
    j.lambda_acc = q6(lambda_old + lambda);

    bodies.apply_position_delta(ia, q6v(-n * q6(lambda * wa)));
    bodies.apply_position_delta(ib, q6v( n * q6(lambda * wb)));

    let _ = dt;
}

fn solve_d6(
    bodies: &mut Bodies,
    j: &mut Generic6Dof,
    dt: Scalar,
    dt2: Scalar,
    alpha_dt2: fn(Scalar, Scalar) -> Scalar,
    start: Option<(Isometry, Isometry)>,
) {
    let pa = bodies.pose(j.a.0);
    let pb = bodies.pose(j.b.0);

    let ra: Quat = pa.rot * j.fa.rot;
    let rb: Quat = pb.rot * j.fb.rot;

    let xa = q6v(pa.pos + pa.rot * j.fa.pos);
    let xb = q6v(pb.pos + pb.rot * j.fb.pos);

    let ax = normalize_q6(ra * Vec3::X);
    let ay = normalize_q6(ra * Vec3::Y);
    let az = normalize_q6(ra * Vec3::Z);
    let axes = [ax, ay, az];

    let wa = q6(bodies.inv_mass_of(j.a.0));
    let wb = q6(bodies.inv_mass_of(j.b.0));

    // Correction directions: the current iterate's (PBD), or the start of the step's (SHAKE).
    // Errors are always measured at the current iterate.
    let start = start.filter(|(sa, sb)| turned(sa.rot, pa.rot) < SHAKE_MAX_TURN && turned(sb.rot, pb.rot) < SHAKE_MAX_TURN);
    let (dir_axes, lever_a, lever_b, iaw, ibw): ([Vec3; 3], Vec3, Vec3, Mat3, Mat3) = match start {
        Some((sa, sb)) => {
            let ra0 = sa.rot * j.fa.rot;
            ([normalize_q6(ra0 * Vec3::X), normalize_q6(ra0 * Vec3::Y), normalize_q6(ra0 * Vec3::Z)],
             q6v(sa.rot * j.fa.pos), q6v(sb.rot * j.fb.pos),
             inv_inertia_at(bodies, j.a.0, sa.rot), inv_inertia_at(bodies, j.b.0, sb.rot))
        }
        None => (axes, xa - pa.pos, xb - pb.pos, bodies.inv_inertia_world(j.a.0), bodies.inv_inertia_world(j.b.0)),
    };

    let d = q6v(xb - xa);

    for i in 0..3 {
        let row = &mut j.t[i];
        if !row.enabled { continue; }

        let u = axes[i];
        if u == Vec3::ZERO { continue; }
        let un = dir_axes[i];

        let coord = q6(d.dot(u));
        let mut target_pos = 0.0f32;
        let mut c_err: Option<f32> = Some(q6(coord - 0.0));

        if let Some(lim) = row.limit {
            let lo = q6(lim.min);
            let hi = q6(lim.max);
            if coord < lo {
                c_err = Some(q6(coord - lo));
                target_pos = lo;
            } else if coord > hi {
                c_err = Some(q6(coord - hi));
                target_pos = hi;
            } else {
                c_err = None;
                target_pos = coord;
            }
        }

        if let Some(drive) = row.drive {
            if !drive.vel_mode {
                target_pos = q6(drive.target);
                if let Some(lim) = row.limit {
                    target_pos = target_pos.clamp(q6(lim.min), q6(lim.max));
                }
                c_err = Some(q6(coord - target_pos));
            } else if row.limit.is_none() {
                // Velocity drives act at velocity level (motor rows in the World's velocity
                // solve); at position level the axis is free apart from its limits. Leaving it
                // locked at 0 here (as before) meant a velocity-driven joint could never turn.
                c_err = None;
            }
        }

        let Some(mut c) = c_err else { continue; };

        let bias_vel = 0.0f32;

        let ra_u = q6v(lever_a.cross(un));
        let rb_u = q6v(lever_b.cross(un));

        let w_rot_a = q6(ra_u.dot(iaw * ra_u));
        let w_rot_b = q6(rb_u.dot(ibw * rb_u));
        let w = q6(wa + wb + w_rot_a + w_rot_b);

        let alpha = alpha_dt2(row.compliance, dt2);
        let denom = q6(w + alpha);
        if denom <= 0.0 { continue; }

        c = q6(c + q6(bias_vel * dt));
        let lambda_old = q6(row.lambda_acc);
        let dl = q6(-(c + q6(alpha * lambda_old)) / denom);
        row.lambda_acc = q6(lambda_old + dl);

        bodies.apply_position_delta(j.a.0, q6v(-un * q6(dl * wa)));
        bodies.apply_position_delta(j.b.0, q6v( un * q6(dl * wb)));

        bodies.apply_orientation_delta(j.a.0, q6v(-(iaw * ra_u) * dl));
        bodies.apply_orientation_delta(j.b.0, q6v( (ibw * rb_u) * dl));

        let _ = target_pos;
    }

    let mut q_err = ra.conjugate() * rb;
    if q_err.w < 0.0 { q_err = -q_err; }

    // 'e_local' is generated in the local coordinate space of Anchor A
    let e_local = q6v(2.0 * Vec3::new(q_err.x, q_err.y, q_err.z));

    // Rotate the local error vector into world space to match 'axes'
    let e_world = q6v(ra * e_local);

    // With one rotational axis not rigidly locked (a hinge, limited or not), the other two rows
    // only keep that axis aligned. Measure the misalignment as a_f x b_f, which does not depend
    // on the twist about it. The quaternion error above mixes the twist into its other
    // components (by half the twist angle), so past ~90 degrees of twist a locked row pushed
    // the wrong way and a hinge diverged.
    let rigid = |r: &D6Axis| r.enabled && r.limit.is_none() && r.drive.is_none();
    let hinge = d6_hinge_axis(j);
    let e_lock = match hinge {
        Some(f) => {
            let unit = [Vec3::X, Vec3::Y, Vec3::Z][f];
            q6v(normalize_q6(ra * unit).cross(normalize_q6(rb * unit)))
        }
        None => e_world,
    };
    // A hinge's own angle, in radians (limits and position drives are given in radians; the
    // small-angle measure 2 sin(angle / 2) put a 2.2 limit out of reach and -1.5 at -97 deg)
    let hinge_angle = hinge.map(|f| { let v = Vec3::new(q_err.x, q_err.y, q_err.z); q6(2.0 * v[f].atan2(q_err.w)) });

    for i in 0..3 {
        let row = &mut j.r[i];
        if !row.enabled { continue; }

        let u = axes[i];
        if u == Vec3::ZERO { continue; }
        let un = dir_axes[i];

        // Both vectors are now correctly operating in world space
        let angle = if rigid(row) { q6(e_lock.dot(u)) }
                    else if hinge == Some(i) { hinge_angle.unwrap_or(0.0) }
                    else { q6(e_world.dot(u)) };
        let mut target_ang = 0.0f32;
        let mut c_err: Option<f32> = Some(q6(angle - 0.0));

        if let Some(lim) = row.limit {
            let lo = q6(lim.min);
            let hi = q6(lim.max);
            if angle < lo {
                c_err = Some(q6(angle - lo));
                target_ang = lo;
            } else if angle > hi {
                c_err = Some(q6(angle - hi));
                target_ang = hi;
            } else {
                c_err = None;
                target_ang = angle;
            }
        }

        if let Some(drive) = row.drive {
            if !drive.vel_mode {
                target_ang = q6(drive.target);
                if let Some(lim) = row.limit {
                    target_ang = target_ang.clamp(q6(lim.min), q6(lim.max));
                }
                c_err = Some(q6(angle - target_ang));
            } else if row.limit.is_none() {
                // Velocity drives: velocity level only (see the translational rows above)
                c_err = None;
            }
        }

        let Some(mut c) = c_err else { continue; };

        let bias_vel = 0.0f32;

        let w = q6(un.dot(iaw * un) + un.dot(ibw * un));
        let alpha = alpha_dt2(row.compliance, dt2);
        let denom = q6(w + alpha);
        if denom <= 0.0 { continue; }

        c = q6(c + q6(bias_vel * dt));
        let lambda_old = q6(row.lambda_acc);
        let dl = q6(-(c + q6(alpha * lambda_old)) / denom);
        row.lambda_acc = q6(lambda_old + dl);

        bodies.apply_orientation_delta(j.a.0, q6v(-(iaw * un) * dl));
        bodies.apply_orientation_delta(j.b.0, q6v( (ibw * un) * dl));

        let _ = target_ang;
    }
}

/// The hinge axis of a D6 joint: the one rotational axis that is not rigidly locked (free,
/// limited or driven), if exactly one is. `None` for ball, universal and fully locked joints.
pub fn d6_hinge_axis(j: &Generic6Dof) -> Option<usize> {
    let rigid = |r: &D6Axis| r.enabled && r.limit.is_none() && r.drive.is_none();
    let mut free = None;
    let mut n = 0;
    for i in 0..3 { if !rigid(&j.r[i]) { free = Some(i); n += 1; } }
    if n == 1 { free } else { None }
}

/// The angle a D6 joint's rotational row `i` measures, from the joint frames' world rotations
/// `rot_a` (body A's rotation x frame A) and `rot_b`. On a hinge's own axis it is the twist
/// angle in radians, wrapped to (-pi, pi], so limits and position drives mean what they say.
/// Elsewhere it is the small-angle measure 2 sin(angle / 2) along the axis (ball and universal
/// joints have no single twist; their limits stay approximate past small angles).
pub fn d6_row_angle(j: &Generic6Dof, rot_a: Quat, rot_b: Quat, i: usize) -> f32 {
    let mut q = rot_a.conjugate() * rot_b;
    if q.w < 0.0 { q = -q; }
    let v = Vec3::new(q.x, q.y, q.z);
    if d6_hinge_axis(j) == Some(i) {
        2.0 * v[i].atan2(q.w)
    } else {
        (rot_a * (v * 2.0)).dot(rot_a * [Vec3::X, Vec3::Y, Vec3::Z][i])
    }
}

pub use D6Axis as D6AxisRow;
pub use Generic6Dof as D6Joint;
pub use Drive as D6Drive;
pub use Limit as D6Limit;