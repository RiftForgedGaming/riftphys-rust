
use riftphys_core::{BodyId, Isometry, JointId, Mat3, Quat, Scalar, Vec3};
use riftphys_dynamics::dynamics::Bodies;

/* ─────────────────────────  Sanitation ───────────────────────── */
// Nothing here is rounded (PRECISION_SPEC P1); non-finite values become 0.

#[inline]
fn fin(x: f32) -> f32 {
    if x.is_finite() { x } else { 0.0 }
}

#[inline]
fn fin_v(v: Vec3) -> Vec3 {
    Vec3::new(fin(v.x), fin(v.y), fin(v.z))
}

#[inline]
fn normalize_fin(v: Vec3) -> Vec3 {
    let v = fin_v(v);
    let len2 = v.length_squared();
    if len2 > 1.0e-12 {
        v * (1.0 / len2.sqrt())
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
    /// Anchors in each body's frame (PARITY_SPEC W4c). Both zero: the COMs, solved by the
    /// original rows, bit for bit.
    pub la: Vec3,
    pub lb: Vec3,
    /// Damping along the rod, N s/m, compliant rods only (a velocity row; 0 = none).
    pub damping: Scalar,
}

impl DistanceJoint {
    /// Solved by the anchored rows (anchors or damping set), not the original COM-to-COM ones.
    #[inline]
    pub fn anchored(&self) -> bool { self.la != Vec3::ZERO || self.lb != Vec3::ZERO || self.damping > 0.0 }
}

/* ─────────────────────────  D6 types ──────────── */

/// Whether a D6 row's damping is XPBD's damping term in its position row (PARITY_SPEC W4c): an
/// enabled compliant row whose position part always acts (no limit, no velocity drive). Any
/// other row with damping keeps W3a's velocity damper (a limited row's position part acts only
/// outside its range, so moving its damper there left it undamped inside).
#[inline]
pub fn position_damped(a: &D6Axis) -> bool {
    a.enabled && a.compliance > 0.0 && a.damping > 0.0 && a.limit.is_none() && a.drive.map_or(true, |d| !d.vel_mode)
}

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
    /// Viscous damping on this axis, N m s/rad (rotational) or N s/m (linear): a velocity row
    /// whose impulse is -damping x relative velocity x dt, solved implicitly (stable for any
    /// value). Works on free axes too. 0 = none. (PARITY_SPEC W3a)
    pub damping: Scalar,
    /// Coulomb friction on this axis, N m or N: a velocity row holding the relative velocity at
    /// 0 with at most friction x dt of impulse per step. 0 = none. (PARITY_SPEC W3a)
    pub friction: Scalar,
    /// Armature, kg m^2 (rotational axes): a rotor geared rigidly to this axis (rotor inertia x
    /// gear ratio^2), feeling no gravity or contact. 0 = none. (PARITY_SPEC W4b)
    pub armature: Scalar,
    /// The geared rotor's predicted angle for this step's position pass (PARITY_SPEC W4f): set
    /// by the World before the pass on armature axes, cleared after. None: no rotor row (and
    /// always outside a step). Transient, not carried state.
    pub rotor_target: Option<Scalar>,
    /// The rotor row's multiplier accumulated over this step's position pass (zeroed by
    /// `begin_step`): the angular impulse x dt the links took from the rotor.
    pub rotor_lambda: Scalar,
}

impl Default for D6Axis {
    fn default() -> Self {
        Self {
            enabled: false,
            compliance: 0.0,
            drive: None,
            limit: None,
            lambda_acc: 0.0,
            damping: 0.0,
            friction: 0.0,
            armature: 0.0,
            rotor_target: None,
            rotor_lambda: 0.0,
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
    /// The hinge's rotational axis (0, 1, 2 = x, y, z of frame A), if this joint is a hinge:
    /// set by `add_hinge`. Its row measures the twist angle in radians (for limits, drives and
    /// springs), and the other two rows only keep it aligned, whatever their compliance. `None`
    /// derives it from the rows (`d6_hinge_axis`). `set_d6_limit_r` / `set_d6_drive_r` on another
    /// rotational axis clear it; edit `r` directly and you must keep it consistent yourself.
    pub hinge_axis: Option<usize>,
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
    /// W4g's armature blocks off (PARITY_SPEC 3.4j): the rows are solved by Gauss-Seidel alone,
    /// as W4f did (for the before/after reports and the block tests). Default: blocks on.
    blocks_off: bool,
    /// The last position pass's armature blocks formed, and blocks over the row cap that fell
    /// back to Gauss-Seidel.
    block_counts: (u32, u32),
}

impl Joints {
    pub fn new() -> Self {
        Self { kinds: Vec::new(), blocks_off: false, block_counts: (0, 0) }
    }
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.kinds.is_empty()
    }
    #[inline]
    pub fn kinds(&self) -> &[JointKind] {
        &self.kinds
    }
    /// Turns W4g's armature blocks on (the default) or off (Gauss-Seidel alone, as W4f).
    pub fn set_armature_blocks(&mut self, on: bool) { self.blocks_off = !on; }
    /// The last position pass's armature blocks: (formed, fallen back to Gauss-Seidel).
    pub fn armature_block_counts(&self) -> (u32, u32) { self.block_counts }

    /// W4g's blocks for this step: every D6 joint with a rotor target (an armature axis inside a
    /// step), joined with every D6 joint that has its child link as a body; blocks sharing a
    /// joint merge. Each block's joint indices ascend. Blocks whose rows could exceed
    /// `BLOCK_MAX_ROWS` are left to Gauss-Seidel and counted.
    fn armature_blocks(&self, bodies: &Bodies) -> (Vec<Vec<usize>>, u32) {
        let n = self.kinds.len();
        let d6 = |k: usize| match &self.kinds[k] { JointKind::D6(j) => Some(j), _ => None };
        let motor = |k: usize| d6(k).map_or(false, |j| j.r.iter().any(|a| a.armature > 0.0 && a.rotor_target.is_some()));
        if !(0..n).any(motor) { return (Vec::new(), 0); }
        let mut parent: Vec<usize> = (0..n).collect();
        fn root(p: &mut Vec<usize>, mut x: usize) -> usize { while p[x] != x { p[x] = p[p[x]]; x = p[x]; } x }
        let mut member = vec![false; n];
        for k in 0..n {
            if !motor(k) { continue; }
            member[k] = true;
            let child = d6(k).unwrap().b;
            for m in 0..n {
                if m == k { continue; }
                if let Some(o) = d6(m) {
                    if o.a == child || o.b == child {
                        member[m] = true;
                        let (rk, rm) = (root(&mut parent, k), root(&mut parent, m));
                        if rk != rm { let (lo, hi) = (rk.min(rm), rk.max(rm)); parent[hi] = lo; }
                    }
                }
            }
        }
        // W4h (PARITY_SPEC 3.4k): every joint on a closed loop through a block joins it. The joint
        // graph's non-bridge edges (bodies as nodes; a static body is a separate leaf per joint, so
        // no loop runs through the ground) fall into 2-edge-connected components; a component
        // holding a member brings all its joints in.
        let ends = |k: usize| -> (u32, u32) { match &self.kinds[k] { JointKind::Distance(j) => (j.a.0, j.b.0), JointKind::D6(j) => (j.a.0, j.b.0) } };
        let node = |id: u32, k: usize, side: u32| -> u64 {
            if bodies.inv_mass_of(id) > 0.0 { id as u64 } else { (1u64 << 40) | ((k as u64) << 1) | side as u64 }
        };
        let edges: Vec<(u64, u64)> = (0..n).map(|k| { let (a, b) = ends(k); (node(a, k, 0), node(b, k, 1)) }).collect();
        // Connected without edge `skip`? (graphs here are tens of joints: a search per edge is cheap)
        let connected_without = |skip: usize, from: u64, to: u64| -> bool {
            let mut seen = std::collections::BTreeSet::new();
            let mut todo = vec![from];
            seen.insert(from);
            while let Some(x) = todo.pop() {
                if x == to { return true; }
                for (e, &(u, v)) in edges.iter().enumerate() {
                    if e == skip { continue; }
                    let y = if u == x { v } else if v == x { u } else { continue };
                    if seen.insert(y) { todo.push(y); }
                }
            }
            false
        };
        let on_loop: Vec<bool> = (0..n).map(|k| edges[k].0 != edges[k].1 && connected_without(k, edges[k].0, edges[k].1)).collect();
        // 2-edge-connected components: bodies joined through loop edges
        let mut comp: std::collections::BTreeMap<u64, u64> = std::collections::BTreeMap::new();
        fn croot(c: &mut std::collections::BTreeMap<u64, u64>, mut x: u64) -> u64 {
            loop { let p = *c.entry(x).or_insert(x); if p == x { return x; } let gp = *c.entry(p).or_insert(p); c.insert(x, gp); x = gp; }
        }
        for k in 0..n {
            if !on_loop[k] { continue; }
            let (ra, rb) = (croot(&mut comp, edges[k].0), croot(&mut comp, edges[k].1));
            if ra != rb { let (lo, hi) = (ra.min(rb), ra.max(rb)); comp.insert(hi, lo); }
        }
        let loop_comp: Vec<Option<u64>> = (0..n).map(|k| if on_loop[k] { Some(croot(&mut comp, edges[k].0)) } else { None }).collect();
        let held: std::collections::BTreeSet<u64> = (0..n).filter(|&k| member[k]).filter_map(|k| loop_comp[k]).collect();
        for c in held {
            let first = (0..n).find(|&k| member[k] && loop_comp[k] == Some(c)).unwrap();
            for k in 0..n {
                if loop_comp[k] != Some(c) { continue; }
                member[k] = true;
                let (rk, rf) = (root(&mut parent, k), root(&mut parent, first));
                if rk != rf { let (lo, hi) = (rk.min(rf), rk.max(rf)); parent[hi] = lo; }
            }
        }
        let mut groups: std::collections::BTreeMap<usize, Vec<usize>> = std::collections::BTreeMap::new();
        for k in 0..n { if member[k] { let r = root(&mut parent, k); groups.entry(r).or_default().push(k); } }
        let (mut out, mut fallbacks) = (Vec::new(), 0u32);
        for (_, g) in groups {
            let rows: usize = g.iter().map(|&k| match d6(k) { Some(j) => 6 + j.r.iter().filter(|a| a.armature > 0.0).count(), None => 1 }).sum();
            if rows > BLOCK_MAX_ROWS { fallbacks += 1; } else { out.push(g); }
        }
        (out, fallbacks)
    }

    /// The joints, mutably (the World sets W4f's per-step rotor targets through this).
    #[inline]
    pub fn kinds_mut(&mut self) -> &mut [JointKind] {
        &mut self.kinds
    }
    #[inline]
    pub fn len(&self) -> usize {
        self.kinds.len()
    }
    pub fn set_distance_params(&mut self, id: JointId, rest: f32, compliance: f32) {
        if let Some(JointKind::Distance(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.rest = fin(rest);
            j.compliance = fin(compliance).max(0.0);
        }
    }

    pub fn add_distance_joint(&mut self, a: BodyId, b: BodyId, rest: Scalar, compliance: Scalar) -> JointId {
        self.kinds.push(JointKind::Distance(DistanceJoint {
            a,
            b,
            rest: fin(rest),
            compliance: fin(compliance).max(0.0),
            lambda_acc: 0.0,
            la: Vec3::ZERO,
            lb: Vec3::ZERO,
            damping: 0.0,
        }));
        JointId((self.kinds.len() as u32) - 1)
    }

    /// A distance joint between anchor `la` on `a` and `lb` on `b` (each in its body's frame),
    /// held at `rest` with `compliance` (m/N; 0 = rigid). Anchors give the rod a lever arm, so
    /// it passes torque (PARITY_SPEC W4c).
    pub fn add_distance_joint_at(&mut self, a: BodyId, la: Vec3, b: BodyId, lb: Vec3, rest: Scalar, compliance: Scalar) -> JointId {
        let id = self.add_distance_joint(a, b, rest, compliance);
        if let Some(JointKind::Distance(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.la = fin_v(la);
            j.lb = fin_v(lb);
        }
        id
    }

    /// Damping along a compliant distance joint, N s/m (negative or non-finite: 0). It only acts
    /// on compliant joints: a rigid one already holds the rod's rate at 0.
    pub fn set_distance_damping(&mut self, id: JointId, damping: Scalar) {
        if let Some(JointKind::Distance(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.damping = fin(damping).max(0.0);
        }
    }

    /// Starts a step's position solve: zeroes every XPBD multiplier. The multiplier belongs to
    /// one step; carried over, a compliant row grew stiffer every step (a torsional spring sagged
    /// 21x too little at 60 Hz).
    pub fn begin_step(&mut self) {
        for k in &mut self.kinds {
            match k {
                JointKind::Distance(j) => j.lambda_acc = 0.0,
                JointKind::D6(j) => {
                    for r in j.t.iter_mut().chain(j.r.iter_mut()) { r.lambda_acc = 0.0; r.rotor_lambda = 0.0; }
                }
            }
        }
    }

    pub fn add_d6(&mut self, j: Generic6Dof) -> JointId {
        self.kinds.push(JointKind::D6(j));
        JointId((self.kinds.len() as u32) - 1)
    }

    pub fn add_ball(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry) -> JointId {
        let mut j = Generic6Dof { a, b, fa, fb, t: Default::default(), r: Default::default(), hinge_axis: None };
        for ax in 0..3 { j.t[ax].enabled = true; }
        for ax in 0..3 { j.r[ax].enabled = false; }
        self.add_d6(j)
    }

    pub fn add_hinge(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry, hinge_axis: usize) -> JointId {
        let mut j = Generic6Dof { a, b, fa, fb, t: Default::default(), r: Default::default(), hinge_axis: Some(hinge_axis) };
        for ax in 0..3 { j.t[ax].enabled = true; }
        for ax in 0..3 { j.r[ax].enabled = ax != hinge_axis; }
        self.add_d6(j)
    }

    pub fn add_universal(&mut self, a: BodyId, b: BodyId, fa: Isometry, fb: Isometry, ax0: usize, ax1: usize) -> JointId {
        let mut j = Generic6Dof { a, b, fa, fb, t: Default::default(), r: Default::default(), hinge_axis: None };
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
            if j.r[axis].drive.is_some() { j.r[axis].enabled = true; leave_hinge(j, axis); }
        }
    }

    pub fn set_d6_limit_t(&mut self, id: JointId, axis: usize, lim: Option<Limit>) {
        if let Some(JointKind::D6(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.t[axis].limit = lim;
            if j.t[axis].limit.is_some() { j.t[axis].enabled = true; }
        }
    }

    /// Viscous damping and Coulomb friction on one axis of a D6 joint (rotational if `angular`).
    /// Negative or non-finite values become 0. Neither changes which axes are constrained.
    pub fn set_d6_damping(&mut self, id: JointId, angular: bool, axis: usize, damping: Scalar, friction: Scalar) {
        if let Some(JointKind::D6(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            let a = if angular { &mut j.r[axis] } else { &mut j.t[axis] };
            a.damping = fin(damping).max(0.0);
            a.friction = fin(friction).max(0.0);
        }
    }

    /// Armature (kg m^2) about rotational axis `axis` of a D6 joint. Negative or non-finite
    /// values become 0.
    pub fn set_d6_armature(&mut self, id: JointId, axis: usize, armature: Scalar) {
        if let Some(JointKind::D6(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.r[axis].armature = fin(armature).max(0.0);
        }
    }

    pub fn set_d6_limit_r(&mut self, id: JointId, axis: usize, lim: Option<Limit>) {
        if let Some(JointKind::D6(ref mut j)) = self.kinds.get_mut(id.0 as usize) {
            j.r[axis].limit = lim;
            if j.r[axis].limit.is_some() { j.r[axis].enabled = true; leave_hinge(j, axis); }
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
        self.solve_step_with(bodies, dt, iterations, start, start, frozen);
    }

    /// As `solve_step`, with SHAKE's directions from `shake` (None: PBD) kept apart from
    /// `step_start`, the step's start poses that a damped compliant row measures its change
    /// against (PARITY_SPEC W4c). With no `step_start` (a projection outside a step, as
    /// `solve`), that change is taken as 0: such rows act as springs only.
    pub fn solve_step_with(&mut self, bodies: &mut Bodies, dt: Scalar, iterations: u32, shake: Option<&[Isometry]>, step_start: Option<&[Isometry]>, frozen: &[bool]) {
        let start = shake;
        if self.kinds.is_empty() { return; }
        let is_frozen = |id: u32| frozen.get(id as usize).copied().unwrap_or(false);

        let dt = fin(dt);
        if !(dt > 0.0) { return; }
        // Not quantized: on the 1e-6 grid dt^2 itself was off by 2% at 240 Hz and 8% at 960 Hz
        // (1.736e-5 -> 1.7e-5, 1.085e-6 -> 1e-6), and every compliance with it.
        let dt2 = (dt * dt).max(1.0e-12);

        #[inline]
        fn alpha_dt2(c: Scalar, dt2: Scalar) -> Scalar {
            let c = fin(c).max(0.0);
            if c <= 0.0 { 0.0 } else { fin(c / dt2) }
        }

        // W4g (PARITY_SPEC 3.4j): inside a step, each armature axis's rotor row, its joint's rows
        // and the rows of the D6 joints sharing its child link are solved as one block per sweep,
        // in place of those joints' Gauss-Seidel rows, at the block's first joint in order
        let blocks = if step_start.is_some() && !self.blocks_off { self.armature_blocks(bodies) } else { (Vec::new(), 0) };
        self.block_counts = (blocks.0.len() as u32, blocks.1);
        let mut in_block = vec![usize::MAX; self.kinds.len()];
        for (bi, members) in blocks.0.iter().enumerate() { for &m in members { in_block[m] = bi; } }

        for _ in 0..iterations.max(1) {
            for idx in 0..self.kinds.len() {
                if in_block[idx] != usize::MAX {
                    let members = &blocks.0[in_block[idx]];
                    if members[0] == idx { solve_armature_block(bodies, &mut self.kinds, members, dt, dt2, start, step_start, frozen); }
                    continue;
                }
                let k = &mut self.kinds[idx];
                let (a, b) = match k { JointKind::Distance(j) => (j.a.0, j.b.0), JointKind::D6(j) => (j.a.0, j.b.0) };
                if is_frozen(a) && is_frozen(b) { continue; }
                match k {
                    JointKind::Distance(j) => {
                        let s = start.map(|s| (frame(s, bodies, j.a.0), frame(s, bodies, j.b.0)));
                        let s0 = step_start.map(|s| (frame(s, bodies, j.a.0), frame(s, bodies, j.b.0)));
                        if j.anchored() { solve_distance_anchored(bodies, j, dt, dt2, s, s0) } else { solve_distance_row(bodies, j, dt, dt2, s) }
                    }
                    JointKind::D6(j) => {
                        let s = start.map(|s| (frame(s, bodies, j.a.0), frame(s, bodies, j.b.0)));
                        let s0 = step_start.map(|s| (frame(s, bodies, j.a.0), frame(s, bodies, j.b.0)));
                        solve_d6(bodies, j, dt, dt2, alpha_dt2, s, s0)
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
    let w_sum = fin(wa + wb);
    if w_sum <= 0.0 { return; }

    let pa = bodies.pose(ia).pos;
    let pb = bodies.pose(ib).pos;

    let d = pb - pa;
    let len2 = d.length_squared();
    if !len2.is_finite() || len2 <= 1.0e-20 { return; }

    let len = len2.sqrt();
    if len <= 1.0e-6 { return; }

    let n_cur = normalize_fin(d / len);
    if n_cur == Vec3::ZERO { return; }
    // SHAKE: correct along the start-of-step direction
    let mut n = n_cur;
    if let Some((sa, sb)) = start {
        let n0 = normalize_fin(sb.pos - sa.pos);
        if n0 != Vec3::ZERO && n0.dot(n_cur) > SHAKE_MAX_TURN.cos() { n = n0; }
    }

    let c_val = fin(len - fin(j.rest));

    let alpha = if j.compliance <= 0.0 { 0.0 } else { fin(fin(j.compliance).max(0.0) / dt2) };
    let denom = fin(w_sum + alpha);
    if denom <= 0.0 { return; }

    // XPBD: the multiplier accumulates over the step's iterations, so a compliant joint settles
    // at C = -alpha lambda (stretch = compliance x force) whatever the iteration count; without it
    // the stiffness depended on the iteration count and compliance was nearly ignored. The error
    // grows along n_cur; a correction along n changes it at the rate n . n_cur.
    let lambda_old = fin(j.lambda_acc);
    let lambda = fin(-(c_val + fin(alpha * lambda_old)) / (denom * n.dot(n_cur)));
    if lambda.abs() <= 1.0e-9 { return; }
    j.lambda_acc = fin(lambda_old + lambda);

    bodies.apply_position_delta(ia, fin_v(-n * fin(lambda * wa)));
    bodies.apply_position_delta(ib, fin_v( n * fin(lambda * wb)));

    let _ = dt;
}

/// The anchored distance row (PARITY_SPEC W4c): C = |x_b - x_a| - rest on the current anchors,
/// corrected along the start-of-step direction n0 with start-of-step lever arms and inertias
/// (SHAKE), over the mixed Jacobian product
/// w_mix = (w_a + w_b)(n_cur . n0) + (r_a x n_cur) . I_a0^-1 (r_a0 x n0) + (r_b x n_cur) . I_b0^-1 (r_b0 x n0),
/// so dlambda = -(C + alpha lambda) / (w_mix + alpha) is SHAKE's Newton step and the compliance
/// stays exact. Current-iterate (PBD) directions when either body turned too far this step or
/// w_mix falls below half the start-of-step generalised mass.
fn solve_distance_anchored(bodies: &mut Bodies, j: &mut DistanceJoint, dt: Scalar, dt2: Scalar, start: Option<(Isometry, Isometry)>, step_start: Option<(Isometry, Isometry)>) {
    let (ia, ib) = (j.a.0, j.b.0);
    let (wa, wb) = (fin(bodies.inv_mass_of(ia)), fin(bodies.inv_mass_of(ib)));
    if wa + wb <= 0.0 { return; }
    let (pa, pb) = (bodies.pose(ia), bodies.pose(ib));
    let (ra, rb) = (fin_v(pa.rot * j.la), fin_v(pb.rot * j.lb));
    let d = fin_v((pb.pos + rb) - (pa.pos + ra));
    let len = d.length();
    if !len.is_finite() || len <= 1.0e-6 { return; }
    let n_cur = normalize_fin(d / len);
    if n_cur == Vec3::ZERO { return; }
    let c_val = fin(len - fin(j.rest));
    let alpha = if j.compliance <= 0.0 { 0.0 } else { fin(fin(j.compliance).max(0.0) / dt2) };

    // Correction direction, lever arms and inverse inertias, and the denominator
    let pbd = || {
        let (iaw, ibw) = (bodies.inv_inertia_world(ia), bodies.inv_inertia_world(ib));
        let (ca, cb) = (ra.cross(n_cur), rb.cross(n_cur));
        let w = wa + wb + ca.dot(iaw * ca) + cb.dot(ibw * cb);
        (n_cur, iaw * ca, ibw * cb, w)
    };
    let shake = start.and_then(|(sa, sb)| {
        if turned(sa.rot, pa.rot) >= SHAKE_MAX_TURN || turned(sb.rot, pb.rot) >= SHAKE_MAX_TURN { return None; }
        let (ra0, rb0) = (sa.rot * j.la, sb.rot * j.lb);
        let n0 = normalize_fin((sb.pos + rb0) - (sa.pos + ra0));
        if n0 == Vec3::ZERO { return None; }
        let (ia0, ib0) = (inv_inertia_at(bodies, ia, sa.rot), inv_inertia_at(bodies, ib, sb.rot));
        let (ga, gb) = (ia0 * ra0.cross(n0), ib0 * rb0.cross(n0));
        let w_sym = wa + wb + ra0.cross(n0).dot(ga) + rb0.cross(n0).dot(gb);
        let w_mix = (wa + wb) * n_cur.dot(n0) + ra.cross(n_cur).dot(ga) + rb.cross(n_cur).dot(gb);
        (w_mix.is_finite() && w_mix >= 0.5 * w_sym).then_some((n0, ga, gb, w_mix))
    });
    let (n, ga, gb, w) = shake.unwrap_or_else(pbd);
    let denom = fin(w + alpha);
    if !(denom > 0.0) { return; }

    let lambda_old = fin(j.lambda_acc);
    // Damping (compliant rods): XPBD's damping term, the spring and damper one implicit
    // Kelvin-Voigt row over the same generalised mass; dC the length's change since the step's
    // start (the Reviewer's design; a velocity row before the projection carried part of a static
    // load, c dt / (m + c dt) of it)
    // No start of step (a projection outside one): no change to damp, a spring only
    let damped = match (j.compliance > 0.0 && j.damping > 0.0, step_start) {
        (true, Some((sa, sb))) => {
            let len0 = ((sb.pos + sb.rot * j.lb) - (sa.pos + sa.rot * j.la)).length();
            let gamma = fin(j.compliance * j.damping / dt);
            Some(fin(-(c_val + fin(alpha * lambda_old) + fin(gamma * fin(len - len0))) / fin((1.0 + gamma) * w + alpha)))
        }
        _ => None,
    };
    let dl = damped.unwrap_or_else(|| fin(-(c_val + fin(alpha * lambda_old)) / denom));
    if dl.abs() <= 1.0e-12 { return; }
    j.lambda_acc = fin(lambda_old + dl);
    bodies.apply_position_delta(ia, fin_v(-n * fin(dl * wa)));
    bodies.apply_position_delta(ib, fin_v(n * fin(dl * wb)));
    bodies.apply_orientation_delta(ia, fin_v(-ga * dl));
    bodies.apply_orientation_delta(ib, fin_v(gb * dl));
}

fn solve_d6(
    bodies: &mut Bodies,
    j: &mut Generic6Dof,
    dt: Scalar,
    dt2: Scalar,
    alpha_dt2: fn(Scalar, Scalar) -> Scalar,
    start: Option<(Isometry, Isometry)>,
    step_start: Option<(Isometry, Isometry)>,
) {
    let units = [Vec3::X, Vec3::Y, Vec3::Z];
    let wa = fin(bodies.inv_mass_of(j.a.0));
    let wb = fin(bodies.inv_mass_of(j.b.0));

    // Correction directions: the current iterate's (PBD), or the start of the step's (SHAKE).
    // The SHAKE fallback test (a body that turned too far this step uses PBD directions) runs
    // once per call, on the poses the call starts from: simpler, and deterministic.
    let (pa0, pb0) = (bodies.pose(j.a.0), bodies.pose(j.b.0));
    // `step_start`: the step's start poses, for a damped compliant row's change since the start
    // (PARITY_SPEC W4c; None outside a step, where that change is 0). SHAKE's directions use
    // `start`, filtered below.
    let start = start.filter(|(sa, sb)| turned(sa.rot, pa0.rot) < SHAKE_MAX_TURN && turned(sb.rot, pb0.rot) < SHAKE_MAX_TURN);
    let shake: Option<([Vec3; 3], Vec3, Vec3, Mat3, Mat3)> = start.map(|(sa, sb)| {
        let ra0 = sa.rot * j.fa.rot;
        ([normalize_fin(ra0 * Vec3::X), normalize_fin(ra0 * Vec3::Y), normalize_fin(ra0 * Vec3::Z)],
         fin_v(sa.rot * j.fa.pos), fin_v(sb.rot * j.fb.pos),
         inv_inertia_at(bodies, j.a.0, sa.rot), inv_inertia_at(bodies, j.b.0, sb.rot))
    });

    // Every row is measured afresh from the bodies' current poses (axes, anchor gap, angles,
    // and in PBD mode its correction directions, lever arms and inertias), so it sees what the
    // rows before it moved. Measured once per call instead, the rows of a joint with a lever arm
    // overcorrected together: a Jacobi sweep with an eigenvalue at -1, which cycled between two
    // wrong states with the joint pulled apart (a hinge drive to 57.3 deg alternated between
    // 24 and 98 deg, the anchor 0.34 / 0.48 m off; JOINT_CONDITIONING_SPEC 2.1).
    for i in 0..3 {
        if !j.t[i].enabled { continue; }

        let (pa, pb) = (bodies.pose(j.a.0), bodies.pose(j.b.0));
        let u = normalize_fin((pa.rot * j.fa.rot) * units[i]);
        if u == Vec3::ZERO { continue; }
        let xa = fin_v(pa.pos + pa.rot * j.fa.pos);
        let xb = fin_v(pb.pos + pb.rot * j.fb.pos);
        let d = fin_v(xb - xa);
        let (un, lever_a, lever_b, iaw, ibw) = match &shake {
            Some((ax0, la, lb, ia, ib)) => (ax0[i], *la, *lb, *ia, *ib),
            None => (u, xa - pa.pos, xb - pb.pos, bodies.inv_inertia_world(j.a.0), bodies.inv_inertia_world(j.b.0)),
        };

        let row = &mut j.t[i];
        let coord = fin(d.dot(u));
        let mut target_pos = 0.0f32;
        let mut c_err: Option<f32> = Some(fin(coord - 0.0));

        if let Some(lim) = row.limit {
            let lo = fin(lim.min);
            let hi = fin(lim.max);
            if coord < lo {
                c_err = Some(fin(coord - lo));
                target_pos = lo;
            } else if coord > hi {
                c_err = Some(fin(coord - hi));
                target_pos = hi;
            } else {
                c_err = None;
                target_pos = coord;
            }
        }

        if let Some(drive) = row.drive {
            if !drive.vel_mode {
                target_pos = fin(drive.target);
                if let Some(lim) = row.limit {
                    target_pos = target_pos.clamp(fin(lim.min), fin(lim.max));
                }
                c_err = Some(fin(coord - target_pos));
            } else if row.limit.is_none() {
                // Velocity drives act at velocity level (motor rows in the World's velocity
                // solve); at position level the axis is free apart from its limits. Leaving it
                // locked at 0 here (as before) meant a velocity-driven joint could never turn.
                c_err = None;
            }
        }

        let Some(mut c) = c_err else { continue; };

        let bias_vel = 0.0f32;

        let ra_u = fin_v(lever_a.cross(un));
        let rb_u = fin_v(lever_b.cross(un));

        let w_rot_a = fin(ra_u.dot(iaw * ra_u));
        let w_rot_b = fin(rb_u.dot(ibw * rb_u));
        let w = fin(wa + wb + w_rot_a + w_rot_b);

        let alpha = alpha_dt2(row.compliance, dt2);
        let denom = fin(w + alpha);
        if denom <= 0.0 { continue; }

        c = fin(c + fin(bias_vel * dt));
        let lambda_old = fin(row.lambda_acc);
        let (compliance, damping, damped) = (row.compliance, row.damping, position_damped(row));
        let dl = match (damped, step_start) {
            // Damped compliant row (PARITY_SPEC W4c): XPBD's damping term, the spring and damper
            // one implicit Kelvin-Voigt row; dC its change since the step's start
            (true, Some((sa, sb))) => {
                let u0 = normalize_fin((sa.rot * j.fa.rot) * units[i]);
                let d0 = fin_v((sb.pos + sb.rot * j.fb.pos) - (sa.pos + sa.rot * j.fa.pos));
                let gamma = fin(compliance * damping / dt);
                let dc = fin(coord - d0.dot(u0));
                fin(-(c + fin(alpha * lambda_old) + fin(gamma * dc)) / fin((1.0 + gamma) * w + alpha))
            }
            _ => fin(-(c + fin(alpha * lambda_old)) / denom),
        };
        let row = &mut j.t[i];
        row.lambda_acc = fin(lambda_old + dl);

        bodies.apply_position_delta(j.a.0, fin_v(-un * fin(dl * wa)));
        bodies.apply_position_delta(j.b.0, fin_v( un * fin(dl * wb)));

        bodies.apply_orientation_delta(j.a.0, fin_v(-(iaw * ra_u) * dl));
        bodies.apply_orientation_delta(j.b.0, fin_v( (ibw * rb_u) * dl));

        let _ = target_pos;
    }

    // With a hinge (one rotational axis not rigidly locked: free, limited, driven or sprung),
    // the other two rows only keep that axis aligned. They measure the misalignment as
    // a_f x b_f, which does not depend on the twist about it; the quaternion error mixes the
    // twist into its other components (by half the twist angle), so past ~90 degrees of twist
    // a locked row pushed the wrong way and a hinge diverged. The hinge's own row measures its
    // angle in radians (limits, drives and springs are given in radians; the small-angle
    // measure 2 sin(angle / 2) put a 2.2 limit out of reach and -1.5 at -97 deg).
    let hinge = d6_hinge_axis(j);

    // The geared rotor's position row (PARITY_SPEC W4f): the rotor as a virtual 1-DoF body on the
    // axis, C = angle - theta_r with theta_r = theta_r* - lambda / J_a, an XPBD row with
    // alpha~ = 1 / J_a (no dt^2: the rotor's inertia is a mass, not a compliance). Run before the
    // axis's own rows so a limit on the same axis has the last word in every sweep.
    for i in 0..3 {
        let (j_a, Some(target)) = (j.r[i].armature, j.r[i].rotor_target) else { continue; };
        if !(j_a > 0.0) { continue; }
        let (pa, pb) = (bodies.pose(j.a.0), bodies.pose(j.b.0));
        let (ra, rb) = (pa.rot * j.fa.rot, pb.rot * j.fb.rot);
        let u = normalize_fin(ra * units[i]);
        if u == Vec3::ZERO { continue; }
        let angle = d6_row_angle(j, ra, rb, i);
        let (un, iaw, ibw) = match &shake {
            Some((ax0, _, _, ia, ib)) => (ax0[i], *ia, *ib),
            None => (u, bodies.inv_inertia_world(j.a.0), bodies.inv_inertia_world(j.b.0)),
        };
        let w = fin(un.dot(iaw * un) + un.dot(ibw * un));
        let alpha = fin(1.0 / j_a);
        let mut c = fin(angle - target);
        // Wrapped to (-pi, pi]: the twist measure wraps, the predicted angle doesn't
        let tau = std::f32::consts::TAU;
        if c > std::f32::consts::PI { c -= tau * ((c + std::f32::consts::PI) / tau).floor(); }
        if c <= -std::f32::consts::PI { c += tau * ((std::f32::consts::PI - c) / tau).floor(); }
        let lambda_old = fin(j.r[i].rotor_lambda);
        let dl = fin(-(c + fin(alpha * lambda_old)) / fin(w + alpha));
        j.r[i].rotor_lambda = fin(lambda_old + dl);
        bodies.apply_orientation_delta(j.a.0, fin_v(-(iaw * un) * dl));
        bodies.apply_orientation_delta(j.b.0, fin_v( (ibw * un) * dl));
    }

    for i in 0..3 {
        if !j.r[i].enabled { continue; }

        let (pa, pb) = (bodies.pose(j.a.0), bodies.pose(j.b.0));
        let ra: Quat = pa.rot * j.fa.rot;
        let rb: Quat = pb.rot * j.fb.rot;
        let u = normalize_fin(ra * units[i]);
        if u == Vec3::ZERO { continue; }
        let mut q_err = ra.conjugate() * rb;
        if q_err.w < 0.0 { q_err = -q_err; }
        let v = Vec3::new(q_err.x, q_err.y, q_err.z);
        let angle = match hinge {
            Some(f) if f == i => fin(2.0 * v[f].atan2(q_err.w)),
            Some(f) => fin(fin_v(normalize_fin(ra * units[f]).cross(normalize_fin(rb * units[f]))).dot(u)),
            // The error vector 2 q_err.xyz, from anchor A's frame into world space
            None => fin(fin_v(ra * fin_v(2.0 * v)).dot(u)),
        };
        let (un, iaw, ibw) = match &shake {
            Some((ax0, _, _, ia, ib)) => (ax0[i], *ia, *ib),
            None => (u, bodies.inv_inertia_world(j.a.0), bodies.inv_inertia_world(j.b.0)),
        };

        let row = &mut j.r[i];
        let mut target_ang = 0.0f32;
        let mut c_err: Option<f32> = Some(fin(angle - 0.0));

        if let Some(lim) = row.limit {
            let lo = fin(lim.min);
            let hi = fin(lim.max);
            if angle < lo {
                c_err = Some(fin(angle - lo));
                target_ang = lo;
            } else if angle > hi {
                c_err = Some(fin(angle - hi));
                target_ang = hi;
            } else {
                c_err = None;
                target_ang = angle;
            }
        }

        if let Some(drive) = row.drive {
            if !drive.vel_mode {
                target_ang = fin(drive.target);
                if let Some(lim) = row.limit {
                    target_ang = target_ang.clamp(fin(lim.min), fin(lim.max));
                }
                c_err = Some(fin(angle - target_ang));
            } else if row.limit.is_none() {
                // Velocity drives: velocity level only (see the translational rows above)
                c_err = None;
            }
        }

        let Some(mut c) = c_err else { continue; };

        let bias_vel = 0.0f32;

        let w = fin(un.dot(iaw * un) + un.dot(ibw * un));
        let alpha = alpha_dt2(row.compliance, dt2);
        let denom = fin(w + alpha);
        if denom <= 0.0 { continue; }

        c = fin(c + fin(bias_vel * dt));
        let lambda_old = fin(row.lambda_acc);
        let (compliance, damping, damped) = (row.compliance, row.damping, position_damped(row));
        let dl = match (damped, step_start) {
            // Damped compliant row (PARITY_SPEC W4c), as the translational rows
            (true, Some((sa, sb))) => {
                let (ra0, rb0) = (sa.rot * j.fa.rot, sb.rot * j.fb.rot);
                let mut q0 = ra0.conjugate() * rb0;
                if q0.w < 0.0 { q0 = -q0; }
                let v0 = Vec3::new(q0.x, q0.y, q0.z);
                let u0 = normalize_fin(ra0 * units[i]);
                let angle0 = match hinge {
                    Some(f) if f == i => fin(2.0 * v0[f].atan2(q0.w)),
                    Some(f) => fin(fin_v(normalize_fin(ra0 * units[f]).cross(normalize_fin(rb0 * units[f]))).dot(u0)),
                    None => fin(fin_v(ra0 * fin_v(2.0 * v0)).dot(u0)),
                };
                let gamma = fin(compliance * damping / dt);
                let dc = fin(angle - angle0);
                fin(-(c + fin(alpha * lambda_old) + fin(gamma * dc)) / fin((1.0 + gamma) * w + alpha))
            }
            _ => fin(-(c + fin(alpha * lambda_old)) / denom),
        };
        let row = &mut j.r[i];
        row.lambda_acc = fin(lambda_old + dl);

        bodies.apply_orientation_delta(j.a.0, fin_v(-(iaw * un) * dl));
        bodies.apply_orientation_delta(j.b.0, fin_v( (ibw * un) * dl));

        let _ = target_ang;
    }
}

/// A limit or drive on a rotational row other than a stored hinge axis makes the joint
/// something else (that row would otherwise be measured as the hinge's alignment, not its own
/// angle): forget the stored axis and derive it from the rows again.
fn leave_hinge(j: &mut Generic6Dof, axis: usize) {
    if j.hinge_axis.map_or(false, |f| f != axis) { j.hinge_axis = None; }
}

/// The hinge axis of a D6 joint, if it has one: `hinge_axis` when set (`add_hinge` sets it).
/// Otherwise it is derived from the rows (PRECISION_SPEC P1):
/// - the one rotational axis that is free (disabled), limited or driven; enabled rows with none
///   of those count as locked, compliant or not, so a hinge whose alignment rows are soft is
///   still a hinge about its free axis;
/// - failing that, the one enabled row whose compliance is positive and at least 100x each other
///   enabled rotational row's: a sprung hinge (0.15 against alignment rows at 0, 1e-9 or 1e-3),
///   whose spring then acts on the angle in radians. A universal joint with a spring on its
///   twist counts too (its other rows are disabled, so only the twist's measure changes);
/// - otherwise none: ball, universal and welded joints, and soft bushings.
///
/// A bushing whose compliance ratio sits near 100x switches between measures as the ratio
/// crosses it; the measures differ only at large angles. Set `hinge_axis` to pin it.
pub fn d6_hinge_axis(j: &Generic6Dof) -> Option<usize> {
    if let Some(f) = j.hinge_axis { return (f < 3).then_some(f); }
    let mut free = None;
    let mut n = 0;
    for i in 0..3 {
        let r = &j.r[i];
        if !(r.enabled && r.limit.is_none() && r.drive.is_none()) { free = Some(i); n += 1; }
    }
    if n == 1 { return free; }
    let c = |i: usize| if j.r[i].enabled { fin(j.r[i].compliance).max(0.0) } else { 0.0 };
    (0..3).find(|&i| j.r[i].enabled && c(i) > 0.0 && (0..3).all(|k| k == i || c(i) >= 100.0 * c(k)))
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

/* ─────────────────────────  W4g: the armature block solve  ───────────────────────── */

/// The largest block W4g solves (rows); larger ones fall back to Gauss-Seidel (PARITY_SPEC 3.4j).
pub const BLOCK_MAX_ROWS: usize = 64;

#[inline]
fn alpha_over_dt2(c: Scalar, dt2: Scalar) -> Scalar {
    let c = fin(c).max(0.0);
    if c <= 0.0 { 0.0 } else { fin(c / dt2) }
}

/// Which multiplier a block row accumulates into: (joint, axis).
#[derive(Clone, Copy)]
enum Slot { T(usize, usize), R(usize, usize), Rotor(usize, usize), Dist(usize) }

/// One position row of a block: its two bodies' linear and angular Jacobians, the residual,
/// alpha~, gamma~, its change since the step's start (damped rows) and its accumulated lambda.
struct BlockRow { a: u32, b: u32, la: Vec3, aa: Vec3, lb: Vec3, ab: Vec3, c: f32, alpha: f32, gamma: f32, dc: f32, lam: f32, slot: Slot }

impl BlockRow {
    /// The row's Jacobian on body `id` (zero if it doesn't touch it).
    fn jac(&self, id: u32) -> (Vec3, Vec3) {
        let (mut l, mut a) = (Vec3::ZERO, Vec3::ZERO);
        if self.a == id { l += self.la; a += self.aa; }
        if self.b == id { l += self.lb; a += self.ab; }
        (l, a)
    }
    fn touches(&self, id: u32) -> bool { self.a == id || self.b == id }
}

/// A row's error as `solve_d6` takes it: locked at 0, a limit only while violated, a position
/// drive's target (clamped into the limit), a velocity drive free at position level.
fn row_error(r: &D6Axis, coord: f32) -> Option<f32> {
    let mut c_err = Some(fin(coord));
    if let Some(lim) = r.limit {
        let (lo, hi) = (fin(lim.min), fin(lim.max));
        c_err = if coord < lo { Some(fin(coord - lo)) } else if coord > hi { Some(fin(coord - hi)) } else { None };
    }
    if let Some(drive) = r.drive {
        if !drive.vel_mode {
            let mut t = fin(drive.target);
            if let Some(lim) = r.limit { t = t.clamp(fin(lim.min), fin(lim.max)); }
            c_err = Some(fin(coord - t));
        } else if r.limit.is_none() {
            c_err = None;
        }
    }
    c_err
}

/// The rows `solve_d6` would solve for joint `m`, measured once from the current poses, with the
/// same directions, lever arms and residuals, and W4f's rotor rows first.
fn block_rows_d6(bodies: &Bodies, m: usize, j: &Generic6Dof, dt: Scalar, dt2: Scalar, start: Option<(Isometry, Isometry)>, step_start: Option<(Isometry, Isometry)>, out: &mut Vec<BlockRow>) {
    let units = [Vec3::X, Vec3::Y, Vec3::Z];
    let (pa, pb) = (bodies.pose(j.a.0), bodies.pose(j.b.0));
    let start = start.filter(|(sa, sb)| turned(sa.rot, pa.rot) < SHAKE_MAX_TURN && turned(sb.rot, pb.rot) < SHAKE_MAX_TURN);
    let shake: Option<([Vec3; 3], Vec3, Vec3)> = start.map(|(sa, sb)| {
        let ra0 = sa.rot * j.fa.rot;
        ([normalize_fin(ra0 * Vec3::X), normalize_fin(ra0 * Vec3::Y), normalize_fin(ra0 * Vec3::Z)],
         fin_v(sa.rot * j.fa.pos), fin_v(sb.rot * j.fb.pos))
    });
    let (ra, rb) = (pa.rot * j.fa.rot, pb.rot * j.fb.rot);
    let hinge = d6_hinge_axis(j);
    let (ia, ib) = (j.a.0, j.b.0);
    let row = |la: Vec3, aa: Vec3, lb: Vec3, ab: Vec3, c: f32, alpha: f32, gamma: f32, dc: f32, lam: f32, slot: Slot| BlockRow { a: ia, b: ib, la, aa, lb, ab, c, alpha, gamma, dc, lam, slot };

    // W4f's rotor rows
    for i in 0..3 {
        let (j_a, Some(target)) = (j.r[i].armature, j.r[i].rotor_target) else { continue; };
        if !(j_a > 0.0) { continue; }
        let u = normalize_fin(ra * units[i]);
        if u == Vec3::ZERO { continue; }
        let un = shake.map_or(u, |s| s.0[i]);
        let mut c = fin(d6_row_angle(j, ra, rb, i) - target);
        let tau = std::f32::consts::TAU;
        if c > std::f32::consts::PI { c -= tau * ((c + std::f32::consts::PI) / tau).floor(); }
        if c <= -std::f32::consts::PI { c += tau * ((std::f32::consts::PI - c) / tau).floor(); }
        out.push(row(Vec3::ZERO, -un, Vec3::ZERO, un, c, fin(1.0 / j_a), 0.0, 0.0, fin(j.r[i].rotor_lambda), Slot::Rotor(m, i)));
    }
    // Translational rows
    for i in 0..3 {
        let r = &j.t[i];
        if !r.enabled { continue; }
        let u = normalize_fin(ra * units[i]);
        if u == Vec3::ZERO { continue; }
        let xa = fin_v(pa.pos + pa.rot * j.fa.pos);
        let xb = fin_v(pb.pos + pb.rot * j.fb.pos);
        let coord = fin(fin_v(xb - xa).dot(u));
        let (un, lever_a, lever_b) = match &shake { Some((ax0, la, lb)) => (ax0[i], *la, *lb), None => (u, xa - pa.pos, xb - pb.pos) };
        let Some(c) = row_error(r, coord) else { continue; };
        let (gamma, dc) = match (position_damped(r), step_start) {
            (true, Some((sa, sb))) => {
                let u0 = normalize_fin((sa.rot * j.fa.rot) * units[i]);
                let d0 = fin_v((sb.pos + sb.rot * j.fb.pos) - (sa.pos + sa.rot * j.fa.pos));
                (fin(r.compliance * r.damping / dt), fin(coord - d0.dot(u0)))
            }
            _ => (0.0, 0.0),
        };
        out.push(row(-un, -fin_v(lever_a.cross(un)), un, fin_v(lever_b.cross(un)), c, alpha_over_dt2(r.compliance, dt2), gamma, dc, fin(r.lambda_acc), Slot::T(m, i)));
    }
    // Rotational rows
    for i in 0..3 {
        let r = &j.r[i];
        if !r.enabled { continue; }
        let u = normalize_fin(ra * units[i]);
        if u == Vec3::ZERO { continue; }
        let measure = |ra: Quat, rb: Quat| -> f32 {
            let mut q = ra.conjugate() * rb;
            if q.w < 0.0 { q = -q; }
            let v = Vec3::new(q.x, q.y, q.z);
            let u = normalize_fin(ra * units[i]);
            match hinge {
                Some(f) if f == i => fin(2.0 * v[f].atan2(q.w)),
                Some(f) => fin(fin_v(normalize_fin(ra * units[f]).cross(normalize_fin(rb * units[f]))).dot(u)),
                None => fin(fin_v(ra * fin_v(2.0 * v)).dot(u)),
            }
        };
        let angle = measure(ra, rb);
        let un = shake.map_or(u, |s| s.0[i]);
        let Some(c) = row_error(r, angle) else { continue; };
        let (gamma, dc) = match (position_damped(r), step_start) {
            (true, Some((sa, sb))) => (fin(r.compliance * r.damping / dt), fin(angle - measure(sa.rot * j.fa.rot, sb.rot * j.fb.rot))),
            _ => (0.0, 0.0),
        };
        out.push(row(Vec3::ZERO, -un, Vec3::ZERO, un, c, alpha_over_dt2(r.compliance, dt2), gamma, dc, fin(r.lambda_acc), Slot::R(m, i)));
    }
}

/// A distance joint's row in a block (W4h, PARITY_SPEC 3.4k): C = |anchor gap| - rest, in the
/// symmetric form at the start pose under SHAKE (the start direction and lever arms on both sides,
/// as the D6 rows enter the block; no w_mix), else the current ones; its compliance and damping kept.
fn block_row_distance(bodies: &Bodies, m: usize, j: &DistanceJoint, dt: Scalar, dt2: Scalar, start: Option<(Isometry, Isometry)>, step_start: Option<(Isometry, Isometry)>, out: &mut Vec<BlockRow>) {
    let (pa, pb) = (bodies.pose(j.a.0), bodies.pose(j.b.0));
    let (ra, rb) = (fin_v(pa.rot * j.la), fin_v(pb.rot * j.lb));
    let d = fin_v((pb.pos + rb) - (pa.pos + ra));
    let len = d.length();
    if !len.is_finite() || len <= 1.0e-6 { return; }
    let (n, la, lb) = match start {
        Some((sa, sb)) => {
            let (ra0, rb0) = (fin_v(sa.rot * j.la), fin_v(sb.rot * j.lb));
            (normalize_fin((sb.pos + rb0) - (sa.pos + ra0)), ra0, rb0)
        }
        None => (normalize_fin(d / len), ra, rb),
    };
    if n == Vec3::ZERO { return; }
    let c = fin(len - fin(j.rest));
    let (gamma, dc) = match (j.compliance > 0.0 && j.damping > 0.0, step_start) {
        (true, Some((sa, sb))) => {
            let len0 = ((sb.pos + sb.rot * j.lb) - (sa.pos + sa.rot * j.la)).length();
            (fin(j.compliance * j.damping / dt), fin(len - len0))
        }
        _ => (0.0, 0.0),
    };
    out.push(BlockRow { a: j.a.0, b: j.b.0, la: -n, aa: -fin_v(la.cross(n)), lb: n, ab: fin_v(lb.cross(n)), c, alpha: alpha_over_dt2(j.compliance, dt2), gamma, dc, lam: fin(j.lambda_acc), slot: Slot::Dist(m) });
}

/// One sweep of a W4g block (PARITY_SPEC 3.4j):
/// [J W J^T + (I + G)^-1 A] dl = -(I + G)^-1 (C + A l + G dC), factored by LDL^T; a pivot under
/// 1e-9 of the largest diagonal drops its row and column, and that row's dl is 0.
fn solve_armature_block(bodies: &mut Bodies, kinds: &mut [JointKind], members: &[usize], dt: Scalar, dt2: Scalar, start: Option<&[Isometry]>, step_start: Option<&[Isometry]>, frozen: &[bool]) {
    let is_frozen = |id: u32| frozen.get(id as usize).copied().unwrap_or(false);
    let ends = |k: &JointKind| match k { JointKind::Distance(j) => (j.a.0, j.b.0), JointKind::D6(j) => (j.a.0, j.b.0) };
    // One direction choice for the whole block (W4h): SHAKE's start-of-step directions only if
    // no body in it turned past SHAKE_MAX_TURN, so rows aren't mixed within one factorisation
    let mut ids: Vec<u32> = members.iter().flat_map(|&m| { let (a, b) = ends(&kinds[m]); [a, b] }).collect();
    ids.sort_unstable();
    ids.dedup();
    let start = start.filter(|s| ids.iter().all(|&id| turned(frame(s, bodies, id).rot, bodies.pose(id).rot) < SHAKE_MAX_TURN));
    let mut rows: Vec<BlockRow> = Vec::new();
    for &m in members {
        let (a, b) = ends(&kinds[m]);
        if is_frozen(a) && is_frozen(b) { continue; }
        let s = start.map(|s| (frame(s, bodies, a), frame(s, bodies, b)));
        let s0 = step_start.map(|s| (frame(s, bodies, a), frame(s, bodies, b)));
        match &kinds[m] {
            JointKind::D6(j) => block_rows_d6(bodies, m, j, dt, dt2, s, s0, &mut rows),
            JointKind::Distance(j) => block_row_distance(bodies, m, j, dt, dt2, s, s0, &mut rows),
        }
    }
    let n = rows.len();
    if n == 0 { return; }
    // One W per body: its start-of-step inverse inertia under SHAKE, else its current one
    let w_of: Vec<(Scalar, Mat3)> = ids.iter().map(|&id| {
        let inv_i = match start.map(|s| frame(s, bodies, id)) {
            Some(s) => inv_inertia_at(bodies, id, s.rot),
            None => bodies.inv_inertia_world(id),
        };
        (fin(bodies.inv_mass_of(id)), inv_i)
    }).collect();

    let mut mat = vec![0.0f64; n * n];
    let mut rhs = vec![0.0f64; n];
    for p in 0..n {
        for q in p..n {
            let mut v = 0.0f64;
            for (k, &id) in ids.iter().enumerate() {
                if !rows[p].touches(id) || !rows[q].touches(id) { continue; }
                let ((lp, ap), (lq, aq)) = (rows[p].jac(id), rows[q].jac(id));
                let (w, ii) = w_of[k];
                v += (lp.dot(lq) * w) as f64 + ap.dot(ii * aq) as f64;
            }
            mat[p * n + q] = v;
            mat[q * n + p] = v;
        }
        let r = &rows[p];
        let g = 1.0 + r.gamma as f64;
        mat[p * n + p] += r.alpha as f64 / g;
        rhs[p] = -(r.c as f64 + r.alpha as f64 * r.lam as f64 + r.gamma as f64 * r.dc as f64) / g;
    }
    // LDL^T with the pivot rule: a dropped row's column is never used and its dl is 0
    let max_diag = (0..n).map(|p| mat[p * n + p]).fold(0.0f64, f64::max);
    let tiny = 1.0e-9 * max_diag;
    let mut l = vec![0.0f64; n * n];
    let mut d = vec![0.0f64; n];
    let mut dropped = vec![false; n];
    for k in 0..n {
        let mut dk = mat[k * n + k];
        for s in 0..k { if !dropped[s] { dk -= l[k * n + s] * l[k * n + s] * d[s]; } }
        if !(dk > tiny) { dropped[k] = true; continue; }
        d[k] = dk;
        for i in (k + 1)..n {
            let mut v = mat[i * n + k];
            for s in 0..k { if !dropped[s] { v -= l[i * n + s] * l[k * n + s] * d[s]; } }
            l[i * n + k] = v / dk;
        }
    }
    let mut y = vec![0.0f64; n];
    for i in 0..n {
        if dropped[i] { continue; }
        let mut v = rhs[i];
        for s in 0..i { if !dropped[s] { v -= l[i * n + s] * y[s]; } }
        y[i] = v;
    }
    let mut x = vec![0.0f64; n];
    for i in (0..n).rev() {
        if dropped[i] { continue; }
        let mut v = y[i] / d[i];
        for s in (i + 1)..n { if !dropped[s] { v -= l[s * n + i] * x[s]; } }
        x[i] = v;
    }
    // Apply J^T W dl per body, then add dl into each row's multiplier
    for (k, &id) in ids.iter().enumerate() {
        let (w, ii) = w_of[k];
        let (mut dl_lin, mut dl_ang) = (Vec3::ZERO, Vec3::ZERO);
        for (p, r) in rows.iter().enumerate() {
            if !r.touches(id) { continue; }
            let (lp, ap) = r.jac(id);
            let dl = x[p] as f32;
            dl_lin += lp * dl;
            dl_ang += ap * dl;
        }
        if dl_lin != Vec3::ZERO { bodies.apply_position_delta(id, fin_v(dl_lin * w)); }
        if dl_ang != Vec3::ZERO { bodies.apply_orientation_delta(id, fin_v(ii * dl_ang)); }
    }
    for (p, r) in rows.iter().enumerate() {
        let dl = fin(x[p] as f32);
        let m = match r.slot { Slot::T(m, _) | Slot::R(m, _) | Slot::Rotor(m, _) | Slot::Dist(m) => m };
        if let (Slot::Dist(_), JointKind::Distance(j)) = (r.slot, &mut kinds[m]) { j.lambda_acc = fin(j.lambda_acc + dl); continue; }
        let JointKind::D6(j) = &mut kinds[m] else { continue; };
        match r.slot {
            Slot::Dist(_) => {}
            Slot::T(_, i) => j.t[i].lambda_acc = fin(j.t[i].lambda_acc + dl),
            Slot::R(_, i) => j.r[i].lambda_acc = fin(j.r[i].lambda_acc + dl),
            Slot::Rotor(_, i) => j.r[i].rotor_lambda = fin(j.r[i].rotor_lambda + dl),
        }
    }
}
