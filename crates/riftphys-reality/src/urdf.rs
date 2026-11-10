//! A minimal URDF loader (PARITY_SPEC W4a): links, inertials, revolute, continuous and fixed
//! joints, limits, `<dynamics damping friction>`, and Drake's `<torsional_spring>` force
//! elements. It builds bodies and hinges in a [`World`] at a given configuration.
//!
//! - **Bodies:** one per link that a fixed joint doesn't attach to its parent. Links on fixed
//!   joints merge into the nearest movable ancestor (mass, COM, inertia by the parallel-axis
//!   theorem). A body's frame sits at its COM, with the link frame's axes, so its inertia can be
//!   a full matrix.
//! - **Hinges:** a revolute joint becomes a hinge about the URDF axis, measuring the URDF angle
//!   q in radians (frame A is the joint origin on the parent; frame B is the child link frame,
//!   both turned so the hinge's row is x).
//! - **Springs:** a hinge with a torsional spring becomes a compliant row (compliance 1/k)
//!   about rest angle 0. The solver doesn't combine a spring with a limit (a limited row only
//!   acts outside its range), so a sprung joint's limits are dropped and listed in
//!   [`Model::dropped_limits`]. Callers check the range instead.
//! - **Not read:** meshes, visuals, collisions, transmissions, mimic joints, and prismatic,
//!   planar or floating joints (these are errors). Contact points are the caller's.
//!
//! Kinematics run in f64 and are rounded to the World's f32 once per body. URDF worlds are
//! usually z-up; pass [`Z_UP_TO_Y_UP`] as `world_from_urdf` for RiftPhys's y-up.

use std::collections::HashMap;

use glam::{DMat3, DQuat, DVec3};
use riftphys_articulation::bodies::{D6Axis, D6Joint, D6Limit};
use riftphys_core::{BodyId, Isometry, JointId, Mat3, Quat, Vec3, Velocity};
use riftphys_geom::MassProps;
use riftphys_world::world::World;

/// Maps a z-up world (URDF, Drake, MuJoCo) onto RiftPhys's y-up: x stays, z becomes y, y
/// becomes −z.
pub const Z_UP_TO_Y_UP: Frame = Frame { pos: DVec3::ZERO, rot: DQuat::from_xyzw(-std::f64::consts::FRAC_1_SQRT_2, 0.0, 0.0, std::f64::consts::FRAC_1_SQRT_2) };

/* ───────────────────────── a rigid transform in f64 ───────────────────────── */

/// A rigid transform in f64: maps a point p in its own frame to rot · p + pos.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    pub pos: DVec3,
    pub rot: DQuat,
}

impl Frame {
    pub const IDENTITY: Frame = Frame { pos: DVec3::ZERO, rot: DQuat::IDENTITY };
    pub fn new(pos: DVec3, rot: DQuat) -> Self { Self { pos, rot } }
    pub fn mul(&self, o: &Frame) -> Frame { Frame { pos: self.pos + self.rot * o.pos, rot: (self.rot * o.rot).normalize() } }
    pub fn inverse(&self) -> Frame { let r = self.rot.conjugate(); Frame { pos: -(r * self.pos), rot: r } }
    pub fn point(&self, p: DVec3) -> DVec3 { self.pos + self.rot * p }
    pub fn to_iso(&self) -> Isometry { Isometry { pos: v32(self.pos), rot: Quat::from_xyzw(self.rot.x as f32, self.rot.y as f32, self.rot.z as f32, self.rot.w as f32).normalize() } }
}

fn v32(v: DVec3) -> Vec3 { Vec3::new(v.x as f32, v.y as f32, v.z as f32) }

/// URDF's fixed-axis roll, pitch, yaw: R = Rz(yaw) Ry(pitch) Rx(roll).
pub fn rpy(r: DVec3) -> DQuat { (DQuat::from_rotation_z(r.z) * DQuat::from_rotation_y(r.y) * DQuat::from_rotation_x(r.x)).normalize() }

/* ───────────────────────── minimal XML ───────────────────────── */

/// An XML element: name, attributes in order, child elements. Text is not kept.
#[derive(Clone, Debug, Default)]
pub struct Element {
    pub name: String,
    pub attrs: Vec<(String, String)>,
    pub children: Vec<Element>,
}

impl Element {
    pub fn attr(&self, k: &str) -> Option<&str> { self.attrs.iter().find(|(n, _)| n == k).map(|(_, v)| v.as_str()) }
    pub fn child(&self, name: &str) -> Option<&Element> { self.children.iter().find(|c| c.name == name) }
    pub fn children_named<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a Element> + 'a { self.children.iter().filter(move |c| c.name == name) }
}

fn unescape(s: &str) -> String {
    s.replace("&lt;", "<").replace("&gt;", ">").replace("&quot;", "\"").replace("&apos;", "'").replace("&amp;", "&")
}

/// Parses the elements and attributes of an XML document: comments, processing instructions
/// and declarations are skipped, and text is ignored. Returns the root element.
pub fn parse_xml(text: &str) -> Result<Element, String> {
    let b = text.as_bytes();
    let mut i = 0usize;
    let mut stack: Vec<Element> = Vec::new();
    let mut root: Option<Element> = None;
    let find = |from: usize, pat: &str| text[from..].find(pat).map(|k| from + k);
    while i < b.len() {
        if b[i] != b'<' { i += 1; continue; }
        if text[i..].starts_with("<!--") {
            i = find(i + 4, "-->").ok_or("unclosed comment")? + 3;
        } else if text[i..].starts_with("<?") {
            i = find(i + 2, "?>").ok_or("unclosed processing instruction")? + 2;
        } else if text[i..].starts_with("<!") {
            i = find(i + 2, ">").ok_or("unclosed declaration")? + 1;
        } else if text[i..].starts_with("</") {
            let end = find(i + 2, ">").ok_or("unclosed end tag")?;
            let name = text[i + 2..end].trim();
            let el = stack.pop().ok_or_else(|| format!("</{}> without an open element", name))?;
            if el.name != name { return Err(format!("</{}> closes <{}>", name, el.name)); }
            match stack.last_mut() { Some(p) => p.children.push(el), None => { if root.is_none() { root = Some(el); } } }
            i = end + 1;
        } else {
            // A start tag: name, then attributes up to '>' or '/>'
            let mut j = i + 1;
            while j < b.len() && !b[j].is_ascii_whitespace() && b[j] != b'>' && b[j] != b'/' { j += 1; }
            let mut el = Element { name: text[i + 1..j].to_string(), ..Default::default() };
            if el.name.is_empty() { return Err(format!("empty tag name at byte {}", i)); }
            let closed;
            loop {
                while j < b.len() && b[j].is_ascii_whitespace() { j += 1; }
                if j >= b.len() { return Err(format!("unclosed <{}>", el.name)); }
                if b[j] == b'>' { closed = false; j += 1; break; }
                if b[j] == b'/' {
                    if b.get(j + 1) != Some(&b'>') { return Err(format!("stray '/' in <{}>", el.name)); }
                    closed = true; j += 2; break;
                }
                let k0 = j;
                while j < b.len() && b[j] != b'=' && !b[j].is_ascii_whitespace() && b[j] != b'>' { j += 1; }
                let key = text[k0..j].to_string();
                while j < b.len() && b[j].is_ascii_whitespace() { j += 1; }
                if b.get(j) != Some(&b'=') { return Err(format!("attribute {} in <{}> has no value", key, el.name)); }
                j += 1;
                while j < b.len() && b[j].is_ascii_whitespace() { j += 1; }
                let q = *b.get(j).ok_or("attribute value missing")?;
                if q != b'"' && q != b'\'' { return Err(format!("attribute {} in <{}> isn't quoted", key, el.name)); }
                let end = text[j + 1..].find(q as char).map(|k| j + 1 + k).ok_or("unclosed attribute value")?;
                el.attrs.push((key, unescape(&text[j + 1..end])));
                j = end + 1;
            }
            if closed {
                match stack.last_mut() { Some(p) => p.children.push(el), None => { if root.is_none() { root = Some(el); } } }
            } else {
                stack.push(el);
            }
            i = j;
        }
    }
    if let Some(open) = stack.last() { return Err(format!("<{}> isn't closed", open.name)); }
    root.ok_or_else(|| "no root element".to_string())
}

/* ───────────────────────── the URDF model ───────────────────────── */

/// A link's mass properties: mass, COM in the link frame, and the inertia about the COM in the
/// link frame's axes (the URDF's inertial frame already rotated in).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Inertial {
    pub mass: f64,
    pub com: DVec3,
    pub inertia: DMat3,
}

#[derive(Clone, Debug)]
pub struct Link {
    pub name: String,
    pub inertial: Option<Inertial>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointType { Revolute, Continuous, Fixed }

#[derive(Clone, Debug)]
pub struct Joint {
    pub name: String,
    pub kind: JointType,
    pub parent: String,
    pub child: String,
    /// The joint frame in the parent link frame (the child link frame at q = 0).
    pub origin: Frame,
    /// Unit axis in the joint frame.
    pub axis: DVec3,
    /// Lower and upper limits, radians (revolute only).
    pub limit: Option<(f64, f64)>,
    pub effort: Option<f64>,
    /// Viscous damping, N m s/rad, and Coulomb friction, N m.
    pub damping: f64,
    pub friction: f64,
}

/// A torsional spring on a joint (Drake's `<force_element><torsional_spring>`), N m/rad.
#[derive(Clone, Debug, PartialEq)]
pub struct Spring {
    pub joint: String,
    pub stiffness: f64,
    pub rest: f64,
}

#[derive(Clone, Debug)]
pub struct Urdf {
    pub name: String,
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
    pub springs: Vec<Spring>,
}

fn nums<const N: usize>(s: Option<&str>, what: &str) -> Result<[f64; N], String> {
    let mut out = [0.0; N];
    let Some(s) = s else { return Ok(out) };
    let parts: Vec<&str> = s.split_whitespace().collect();
    if parts.len() != N { return Err(format!("{}: expected {} numbers, got '{}'", what, N, s)); }
    for (o, p) in out.iter_mut().zip(parts) { *o = p.parse::<f64>().map_err(|e| format!("{}: '{}': {}", what, p, e))?; }
    if out.iter().any(|x| !x.is_finite()) { return Err(format!("{}: non-finite value in '{}'", what, s)); }
    Ok(out)
}

fn num(e: &Element, k: &str, what: &str) -> Result<Option<f64>, String> {
    match e.attr(k) {
        None => Ok(None),
        Some(s) => {
            let v = s.trim().parse::<f64>().map_err(|err| format!("{} {}: '{}': {}", what, k, s, err))?;
            if !v.is_finite() { return Err(format!("{} {}: non-finite", what, k)); }
            Ok(Some(v))
        }
    }
}

fn origin(e: &Element, what: &str) -> Result<Frame, String> {
    match e.child("origin") {
        None => Ok(Frame::IDENTITY),
        Some(o) => {
            let xyz = nums::<3>(o.attr("xyz"), what)?;
            let r = nums::<3>(o.attr("rpy"), what)?;
            Ok(Frame::new(DVec3::from(xyz), rpy(DVec3::from(r))))
        }
    }
}

impl Urdf {
    pub fn parse(text: &str) -> Result<Urdf, String> {
        let root = parse_xml(text)?;
        if root.name != "robot" { return Err(format!("root element is <{}>, not <robot>", root.name)); }
        let mut links = Vec::new();
        for l in root.children_named("link") {
            let name = l.attr("name").ok_or("link without a name")?.to_string();
            let inertial = match l.child("inertial") {
                None => None,
                Some(inr) => {
                    let what = format!("link {} inertial", name);
                    let o = origin(inr, &what)?;
                    let mass = inr.child("mass").map(|m| num(m, "value", &what)).transpose()?.flatten().unwrap_or(0.0);
                    if mass < 0.0 { return Err(format!("{}: negative mass", what)); }
                    let i = inr.child("inertia");
                    let g = |k: &str| -> Result<f64, String> { Ok(match i { Some(e) => num(e, k, &what)?.unwrap_or(0.0), None => 0.0 }) };
                    let (xx, xy, xz, yy, yz, zz) = (g("ixx")?, g("ixy")?, g("ixz")?, g("iyy")?, g("iyz")?, g("izz")?);
                    let local = DMat3::from_cols(DVec3::new(xx, xy, xz), DVec3::new(xy, yy, yz), DVec3::new(xz, yz, zz));
                    let r = DMat3::from_quat(o.rot);
                    Some(Inertial { mass, com: o.pos, inertia: r * local * r.transpose() })
                }
            };
            links.push(Link { name, inertial });
        }
        let mut joints = Vec::new();
        for j in root.children_named("joint") {
            let name = j.attr("name").ok_or("joint without a name")?.to_string();
            let what = format!("joint {}", name);
            let kind = match j.attr("type") {
                Some("revolute") => JointType::Revolute,
                Some("continuous") => JointType::Continuous,
                Some("fixed") => JointType::Fixed,
                other => return Err(format!("{}: type {:?} isn't supported", what, other)),
            };
            if j.child("mimic").is_some() { return Err(format!("{}: mimic joints aren't supported", what)); }
            let parent = j.child("parent").and_then(|p| p.attr("link")).ok_or(format!("{}: no parent", what))?.to_string();
            let child = j.child("child").and_then(|p| p.attr("link")).ok_or(format!("{}: no child", what))?.to_string();
            let axis = match j.child("axis") { Some(a) => DVec3::from(nums::<3>(a.attr("xyz"), &what)?), None => DVec3::X };
            if kind != JointType::Fixed && axis.length() < 1e-9 { return Err(format!("{}: zero axis", what)); }
            let axis = if axis.length() < 1e-9 { DVec3::X } else { axis.normalize() };
            let (mut limit, mut effort) = (None, None);
            if let Some(l) = j.child("limit") {
                effort = num(l, "effort", &what)?;
                if kind == JointType::Revolute {
                    let lo = num(l, "lower", &what)?.unwrap_or(0.0);
                    let hi = num(l, "upper", &what)?.unwrap_or(0.0);
                    if lo > hi { return Err(format!("{}: lower {} above upper {}", what, lo, hi)); }
                    limit = Some((lo, hi));
                }
            } else if kind == JointType::Revolute {
                return Err(format!("{}: a revolute joint needs <limit>", what));
            }
            let (mut damping, mut friction) = (0.0, 0.0);
            if let Some(d) = j.child("dynamics") {
                damping = num(d, "damping", &what)?.unwrap_or(0.0).max(0.0);
                friction = num(d, "friction", &what)?.unwrap_or(0.0).max(0.0);
            }
            joints.push(Joint { name, kind, parent, child, origin: origin(j, &what)?, axis, limit, effort, damping, friction });
        }
        let mut springs = Vec::new();
        for f in root.children_named("force_element") {
            for s in f.children_named("torsional_spring") {
                let what = "torsional_spring";
                let joint = s.child("joint").and_then(|j| j.attr("name")).ok_or("torsional_spring without a joint")?.to_string();
                let stiffness = num(s, "stiffness", what)?.ok_or("torsional_spring without stiffness")?;
                if stiffness <= 0.0 { return Err(format!("spring on {}: stiffness {} isn't positive", joint, stiffness)); }
                springs.push(Spring { joint, stiffness, rest: num(s, "rest_angle", what)?.unwrap_or(0.0) });
            }
        }
        let urdf = Urdf { name: root.attr("name").unwrap_or("").to_string(), links, joints, springs };
        urdf.check()?;
        Ok(urdf)
    }

    fn link(&self, name: &str) -> Option<&Link> { self.links.iter().find(|l| l.name == name) }
    pub fn joint(&self, name: &str) -> Option<&Joint> { self.joints.iter().find(|j| j.name == name) }

    fn check(&self) -> Result<(), String> {
        let mut seen = HashMap::new();
        for l in &self.links { if seen.insert(l.name.as_str(), ()).is_some() { return Err(format!("link {} twice", l.name)); } }
        let mut parent_of = HashMap::new();
        for j in &self.joints {
            for l in [&j.parent, &j.child] { if self.link(l).is_none() { return Err(format!("joint {}: no link {}", j.name, l)); } }
            if parent_of.insert(j.child.as_str(), j.name.as_str()).is_some() { return Err(format!("link {} has two parent joints", j.child)); }
        }
        for s in &self.springs {
            let j = self.joint(&s.joint).ok_or(format!("spring on unknown joint {}", s.joint))?;
            if j.kind == JointType::Fixed { return Err(format!("spring on fixed joint {}", s.joint)); }
            if s.rest != 0.0 { return Err(format!("spring on {}: rest angle {} isn't supported (only 0)", s.joint, s.rest)); }
        }
        self.root()?;
        // Every link reachable from the root, so no cycles
        let mut n = 0;
        let mut todo = vec![self.root()?.to_string()];
        while let Some(l) = todo.pop() {
            n += 1;
            if n > self.links.len() { return Err("the joints form a cycle".into()); }
            todo.extend(self.joints.iter().filter(|j| j.parent == l).map(|j| j.child.clone()));
        }
        if n != self.links.len() { return Err(format!("{} of {} links are reachable from the root", n, self.links.len())); }
        Ok(())
    }

    /// The one link that is no joint's child.
    pub fn root(&self) -> Result<&str, String> {
        let roots: Vec<&str> = self.links.iter().map(|l| l.name.as_str()).filter(|n| !self.joints.iter().any(|j| j.child == *n)).collect();
        match roots.as_slice() { [r] => Ok(r), _ => Err(format!("expected one root link, found {:?}", roots)) }
    }

    pub fn total_mass(&self) -> f64 { self.links.iter().filter_map(|l| l.inertial.map(|i| i.mass)).sum() }

    /// Link frames at configuration q (joints not in `q` are at 0), relative to the root link.
    pub fn link_frames(&self, q: &HashMap<String, f64>) -> HashMap<String, Frame> {
        let mut out = HashMap::new();
        let root = self.root().expect("checked in parse").to_string();
        out.insert(root.clone(), Frame::IDENTITY);
        let mut todo = vec![root];
        while let Some(p) = todo.pop() {
            let fp = out[&p];
            for j in self.joints.iter().filter(|j| j.parent == p) {
                let a = if j.kind == JointType::Fixed { 0.0 } else { q.get(&j.name).copied().unwrap_or(0.0) };
                let f = fp.mul(&j.origin).mul(&Frame::new(DVec3::ZERO, DQuat::from_axis_angle(j.axis, a)));
                out.insert(j.child.clone(), f);
                todo.push(j.child.clone());
            }
        }
        out
    }
}

/// Adds a mass at `com` with `inertia` (about its own COM, same axes) into an accumulated set.
fn merge(into: &mut Option<Inertial>, add: Inertial) {
    let Some(a) = into else { *into = Some(add); return; };
    let m = a.mass + add.mass;
    if m <= 0.0 { return; }
    let c = (a.com * a.mass + add.com * add.mass) / m;
    let shift = |i: &Inertial| {
        let d = i.com - c;
        i.inertia + (DMat3::from_diagonal(DVec3::splat(d.dot(d))) - DMat3::from_cols(d * d.x, d * d.y, d * d.z)) * i.mass
    };
    *into = Some(Inertial { mass: m, com: c, inertia: shift(a) + shift(&add) });
}

/* ───────────────────────── building it in a World ───────────────────────── */

/// Where and how to build a URDF.
#[derive(Clone, Debug)]
pub struct Config {
    /// Maps the URDF's world onto the World's ([`Z_UP_TO_Y_UP`] for a z-up URDF).
    pub world_from_urdf: Frame,
    /// The root link frame in the URDF's world.
    pub root: Frame,
    /// The root link's angular velocity and the velocity of its frame origin, in the URDF's
    /// world (Drake's floating-base convention).
    pub root_ang: DVec3,
    pub root_lin: DVec3,
    /// Joint angles (rad) and rates (rad/s) by joint name; missing joints are 0.
    pub q: HashMap<String, f64>,
    pub qd: HashMap<String, f64>,
    /// A floating base (true) or a root welded to the world (false, the root body is static).
    pub floating: bool,
    /// Spring stiffness overrides by joint name, N m/rad (the URDF's otherwise).
    pub stiffness: HashMap<String, f64>,
}

impl Default for Config {
    fn default() -> Self {
        Self { world_from_urdf: Z_UP_TO_Y_UP, root: Frame::IDENTITY, root_ang: DVec3::ZERO, root_lin: DVec3::ZERO,
               q: HashMap::new(), qd: HashMap::new(), floating: true, stiffness: HashMap::new() }
    }
}

/// One body: the link it was made for, its mass properties, its COM in that link's frame.
#[derive(Clone, Debug)]
pub struct ModelBody {
    pub link: String,
    pub id: BodyId,
    pub mass: f64,
    pub com: DVec3,
}

/// What was built.
#[derive(Clone, Debug)]
pub struct Model {
    pub bodies: Vec<ModelBody>,
    /// For every link: its body (an index into `bodies`) and the link frame in that body's frame.
    pub links: HashMap<String, (usize, Frame)>,
    /// Movable joints by name.
    pub joints: HashMap<String, JointId>,
    /// Sprung joints whose URDF limits were dropped (the solver can't hold both).
    pub dropped_limits: Vec<String>,
}

impl Model {
    /// A link's frame in the World.
    pub fn link_frame(&self, world: &World, link: &str) -> Option<Frame> {
        let (b, f) = self.links.get(link)?;
        let p = world.get_body_pose(self.bodies[*b].id);
        let body = Frame::new(DVec3::new(p.pos.x as f64, p.pos.y as f64, p.pos.z as f64), DQuat::from_xyzw(p.rot.x as f64, p.rot.y as f64, p.rot.z as f64, p.rot.w as f64));
        Some(body.mul(f))
    }

    /// A point given in a link's frame, in World coordinates.
    pub fn point(&self, world: &World, link: &str, p: DVec3) -> Option<DVec3> { Some(self.link_frame(world, link)?.point(p)) }

    /// The body a link belongs to, and the link-frame point `p` in that body's frame.
    pub fn body_point(&self, link: &str, p: DVec3) -> Option<(BodyId, Vec3)> {
        let (b, f) = self.links.get(link)?;
        Some((self.bodies[*b].id, v32(f.point(p))))
    }
}

/// Builds `urdf` in `world` at `cfg`. Gravity, contact and solver settings are the caller's.
pub fn build(urdf: &Urdf, world: &mut World, cfg: &Config) -> Result<Model, String> {
    let root = urdf.root()?.to_string();
    // Owner of each link: itself, or its parent's owner across a fixed joint; and the link's
    // frame in its owner's link frame.
    let mut owner: HashMap<String, (String, Frame)> = HashMap::new();
    owner.insert(root.clone(), (root.clone(), Frame::IDENTITY));
    let mut order = vec![root.clone()];
    let mut k = 0;
    while k < order.len() {
        let p = order[k].clone();
        k += 1;
        for j in urdf.joints.iter().filter(|j| j.parent == p) {
            let o = if j.kind == JointType::Fixed {
                let (po, pf) = owner[&p].clone();
                (po, pf.mul(&j.origin))
            } else {
                (j.child.clone(), Frame::IDENTITY)
            };
            owner.insert(j.child.clone(), o);
            order.push(j.child.clone());
        }
    }
    // Mass properties per owner, in the owner link's frame
    let mut mass: HashMap<String, Option<Inertial>> = HashMap::new();
    for l in &order {
        let (o, f) = &owner[l];
        let e = mass.entry(o.clone()).or_insert(None);
        if let Some(i) = urdf.link(l).and_then(|l| l.inertial) {
            let r = DMat3::from_quat(f.rot);
            merge(e, Inertial { mass: i.mass, com: f.point(i.com), inertia: r * i.inertia * r.transpose() });
        }
    }
    // Link frames and velocities in the URDF world
    let rel = urdf.link_frames(&cfg.q);
    let w = cfg.world_from_urdf;
    let mut ang: HashMap<String, DVec3> = HashMap::new();
    let mut lin: HashMap<String, DVec3> = HashMap::new();
    ang.insert(root.clone(), cfg.root_ang);
    lin.insert(root.clone(), cfg.root_lin);
    let at = |l: &str| cfg.root.mul(&rel[l]);
    for l in order.iter().skip(1) {
        let j = urdf.joints.iter().find(|j| &j.child == l).expect("every non-root link has a parent joint");
        let (fp, fc) = (at(&j.parent), at(l));
        let rate = if j.kind == JointType::Fixed { 0.0 } else { cfg.qd.get(&j.name).copied().unwrap_or(0.0) };
        let wp = ang[&j.parent];
        // The child's frame origin is on the joint axis, so the joint rate adds no velocity there
        lin.insert(l.clone(), lin[&j.parent] + wp.cross(fc.pos - fp.pos));
        ang.insert(l.clone(), wp + fc.rot * j.axis * rate);
    }
    // Bodies, in tree order
    let mut model = Model { bodies: Vec::new(), links: HashMap::new(), joints: HashMap::new(), dropped_limits: Vec::new() };
    let mut body_of: HashMap<String, usize> = HashMap::new();
    for l in &order {
        if owner[l].0 != *l { continue; }
        let dynamic = cfg.floating || *l != root;
        let inr = mass[l].unwrap_or(Inertial { mass: 0.0, com: DVec3::ZERO, inertia: DMat3::ZERO });
        if dynamic && (inr.mass <= 0.0 || inr.inertia.determinant() <= 0.0) {
            return Err(format!("link {} (with what's fixed to it) has no positive mass and inertia", l));
        }
        let link_w = at(l);
        let com_w = link_w.point(inr.com);
        let pose = w.mul(&Frame::new(com_w, link_w.rot));
        let v = w.rot * (lin[l] + ang[l].cross(com_w - link_w.pos));
        let props = if dynamic {
            let i = inr.inertia;
            let m3 = Mat3::from_cols(v32(i.x_axis).into(), v32(i.y_axis).into(), v32(i.z_axis).into());
            MassProps { mass: inr.mass as f32, inv_mass: 1.0 / inr.mass as f32, inertia: m3 }
        } else {
            MassProps::infinite()
        };
        let id = world.add_body(pose.to_iso(), Velocity { lin: v32(v), ang: v32(w.rot * ang[l]) }, props, dynamic);
        world.set_body_sleep_allowed(id, false);
        body_of.insert(l.clone(), model.bodies.len());
        model.bodies.push(ModelBody { link: l.clone(), id, mass: inr.mass, com: inr.com });
    }
    for l in &order {
        let (o, f) = &owner[l];
        let b = body_of[o];
        let body_from_owner = Frame::new(-model.bodies[b].com, DQuat::IDENTITY);
        model.links.insert(l.clone(), (b, body_from_owner.mul(f)));
    }
    // Hinges
    for j in urdf.joints.iter().filter(|j| j.kind != JointType::Fixed) {
        let (pb, pf) = model.links[&j.parent];
        let (cb, cf) = model.links[&j.child];
        let turn = Frame::new(DVec3::ZERO, DQuat::from_rotation_arc(DVec3::X, j.axis));
        let fa = pf.mul(&j.origin).mul(&turn).to_iso();
        let fb = cf.mul(&turn).to_iso();
        let (a, b) = (model.bodies[pb].id, model.bodies[cb].id);
        let spring = urdf.springs.iter().find(|s| s.joint == j.name);
        let id = match spring {
            None => {
                let id = world.add_hinge_joint(a, b, fa, fb, 0);
                if let Some((lo, hi)) = j.limit { world.set_d6_limit_r(id, 0, Some(D6Limit { min: lo as f32, max: hi as f32 })); }
                id
            }
            Some(s) => {
                let k = cfg.stiffness.get(&j.name).copied().unwrap_or(s.stiffness);
                if !(k > 0.0 && k.is_finite()) { return Err(format!("spring on {}: stiffness {} isn't positive", j.name, k)); }
                let mut d = D6Joint { a, b, fa, fb, t: Default::default(), r: Default::default(), hinge_axis: Some(0) };
                for i in 0..3 { d.t[i].enabled = true; d.r[i].enabled = true; }
                d.r[0] = D6Axis { enabled: true, compliance: (1.0 / k) as f32, ..Default::default() };
                if j.limit.is_some() { model.dropped_limits.push(j.name.clone()); }
                world.add_d6_joint(d)
            }
        };
        if j.damping > 0.0 || j.friction > 0.0 { world.set_d6_damping(id, true, 0, j.damping as f32, j.friction as f32); }
        model.joints.insert(j.name.clone(), id);
    }
    Ok(model)
}

/// A hinge's angle in the World, radians: the URDF's q.
pub fn joint_angle(world: &World, model: &Model, urdf: &Urdf, joint: &str) -> Option<f64> {
    let j = urdf.joint(joint)?;
    let fp = model.link_frame(world, &j.parent)?.mul(&j.origin);
    let fc = model.link_frame(world, &j.child)?;
    let r = fp.rot.conjugate() * fc.rot;
    let r = if r.w < 0.0 { -r } else { r };
    Some(2.0 * DVec3::new(r.x, r.y, r.z).dot(j.axis).atan2(r.w))
}

/* ───────────────────────── tests ───────────────────────── */

#[cfg(test)]
mod tests {
    use super::*;

    const PENDULUM: &str = r#"<?xml version="1.0"?>
<!-- a pendulum: a 2 kg rod on a hinge, its inertial frame turned -->
<robot name='pendulum'>
  <link name="base"/>
  <link name="rod">
    <inertial>
      <origin xyz="0 0 -0.4" rpy="0 1.5707963267948966 0"/>
      <mass value="2.0"/>
      <!-- about the rod's own axis (x in the inertial frame, z in the link) it's thin -->
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <link name="bob"><inertial><mass value="0.5"/><inertia ixx="1e-4" iyy="1e-4" izz="1e-4"/></inertial></link>
  <joint name="pivot" type="revolute">
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <parent link="base"/><child link="rod"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="10"/>
  </joint>
  <joint name="weld" type="fixed">
    <origin xyz="0 0 -0.8"/>
    <parent link="rod"/><child link="bob"/>
  </joint>
</robot>"#;

    fn pend() -> Urdf { Urdf::parse(PENDULUM).unwrap() }

    #[test]
    fn parses_links_joints_and_rotated_inertia() {
        let u = pend();
        assert_eq!(u.name, "pendulum");
        assert_eq!((u.links.len(), u.joints.len()), (3, 2));
        assert_eq!(u.root().unwrap(), "base");
        let rod = u.link("rod").unwrap().inertial.unwrap();
        // Turned 90 degrees about y: the thin axis is now the link's z
        assert!((rod.inertia.z_axis.z - 0.001).abs() < 1e-12 && (rod.inertia.x_axis.x - 0.1).abs() < 1e-12, "{:?}", rod.inertia);
        let j = u.joint("pivot").unwrap();
        assert_eq!(j.limit, Some((-3.0, 3.0)));
        assert_eq!(j.effort, Some(10.0));
        assert!((u.total_mass() - 2.5).abs() < 1e-12);
    }

    #[test]
    fn rejects_what_it_cant_build() {
        let bad = PENDULUM.replace("type=\"revolute\"", "type=\"prismatic\"");
        assert!(Urdf::parse(&bad).is_err());
        let cycle = PENDULUM.replace("<parent link=\"rod\"/><child link=\"bob\"/>", "<parent link=\"bob\"/><child link=\"rod\"/>");
        assert!(Urdf::parse(&cycle).is_err());
        assert!(parse_xml("<robot><link name='a'></robot>").is_err());
    }

    #[test]
    fn a_fixed_joint_merges_mass_com_and_inertia() {
        let u = pend();
        let mut w = World::with_capacity(4, 4);
        let m = build(&u, &mut w, &Config { floating: false, ..Default::default() }).unwrap();
        assert_eq!(m.bodies.len(), 2, "base and rod; the bob merges into the rod");
        let rod = m.bodies.iter().find(|b| b.link == "rod").unwrap();
        assert!((rod.mass - 2.5).abs() < 1e-12);
        // COM: (2 x -0.4 + 0.5 x -0.8) / 2.5 = -0.48 along the link's z
        assert!((rod.com - DVec3::new(0.0, 0.0, -0.48)).length() < 1e-12, "{:?}", rod.com);
        assert_eq!(m.links["bob"].0, m.links["rod"].0);
    }

    /// The URDF pendulum swings with the period of its closed form, 2 pi sqrt(I_pivot / (m g d)),
    /// and its hinge reads the URDF angle.
    #[test]
    fn the_pendulum_swings_at_its_closed_form_period() {
        let u = pend();
        let g = 9.81;
        // About the pivot: rod 0.1 + 2 x 0.4^2, bob 1e-4 + 0.5 x 0.8^2
        let i_pivot: f64 = 0.1 + 2.0 * 0.16 + 1e-4 + 0.5 * 0.64;
        let (m, d): (f64, f64) = (2.5, 0.48);
        let period = 2.0 * std::f64::consts::PI * (i_pivot / (m * g * d)).sqrt();
        for hz in [240.0f32, 1000.0] {
            let mut w = World::with_capacity(4, 4);
            w.set_gravity(Vec3::new(0.0, -g as f32, 0.0));
            let q0 = 0.05;
            let cfg = Config { floating: false, q: HashMap::from([("pivot".to_string(), q0)]), ..Default::default() };
            let model = build(&u, &mut w, &cfg).unwrap();
            let a0 = joint_angle(&w, &model, &u, "pivot").unwrap();
            assert!((a0 - q0).abs() < 1e-6, "built at {} reads {}", q0, a0);
            // Time between the first two downward zero crossings
            let (mut last, mut crossings, mut t) = (a0, Vec::new(), 0.0f64);
            while crossings.len() < 3 && t < 10.0 {
                w.step(1.0 / hz);
                t += 1.0 / hz as f64;
                let a = joint_angle(&w, &model, &u, "pivot").unwrap();
                if last > 0.0 && a <= 0.0 { crossings.push(t - (1.0 / hz as f64) * a / (a - last)); }
                last = a;
            }
            let measured = crossings[2] - crossings[1];
            assert!((measured / period - 1.0).abs() < 0.01, "{} Hz: period {:.4} s, closed form {:.4} s", hz, measured, period);
        }
    }

    /// A body's velocity at build matches the finite difference of its position over a small
    /// change of q, with a moving floating base.
    #[test]
    fn velocities_match_a_finite_difference_of_the_kinematics() {
        let u = pend();
        let (q0, qd, root_ang, root_lin) = (0.7, 1.3, DVec3::new(0.2, -0.5, 0.9), DVec3::new(0.3, 0.1, -0.4));
        let h = 1e-6;
        let pos_at = |t: f64| {
            let mut w = World::with_capacity(4, 4);
            let root = Frame::new(DVec3::new(0.1, 0.2, 1.0) + root_lin * t, DQuat::from_scaled_axis(root_ang * t));
            // The base is massless, so it is welded; its velocity still carries into the rod
            let cfg = Config { root, root_ang, root_lin, floating: false, q: HashMap::from([("pivot".to_string(), q0 + qd * t)]),
                               qd: HashMap::from([("pivot".to_string(), qd)]), ..Default::default() };
            let m = build(&u, &mut w, &cfg).unwrap();
            let id = m.bodies.iter().find(|b| b.link == "rod").unwrap().id;
            let p = w.get_body_pose(id).pos;
            (DVec3::new(p.x as f64, p.y as f64, p.z as f64), w.get_body_vel(id))
        };
        let (p0, v) = pos_at(0.0);
        let (p1, _) = pos_at(h);
        let fd = (p1 - p0) / h;
        let v = DVec3::new(v.lin.x as f64, v.lin.y as f64, v.lin.z as f64);
        // f32 positions: the difference quotient carries about 1e-7 / 1e-6 of noise
        assert!((fd - v).length() < 0.2, "finite difference {:?}, built {:?}", fd, v);
        // The exact check, in f64: v = v_root + w_root x (c - o) + w_joint x (c - o_joint)
        let rel = u.link_frames(&HashMap::from([("pivot".to_string(), q0)]));
        let root = Frame::new(DVec3::new(0.1, 0.2, 1.0), DQuat::IDENTITY);
        let rod = root.mul(&rel["rod"]);
        let c = rod.point(DVec3::new(0.0, 0.0, -0.48));
        let joint_w = rod.rot * DVec3::Y * qd;
        let expect = Z_UP_TO_Y_UP.rot * (root_lin + root_ang.cross(c - root.pos) + joint_w.cross(c - rod.pos));
        assert!((expect - v).length() < 1e-5, "expected {:?}, built {:?}", expect, v);
    }

    #[test]
    fn a_sprung_joint_is_compliant_and_drops_its_limit() {
        let sprung = PENDULUM.replace("</robot>", "<force_element name='s'><torsional_spring stiffness='50' rest_angle='0'><joint name='pivot'/></torsional_spring></force_element></robot>");
        let u = Urdf::parse(&sprung).unwrap();
        assert_eq!(u.springs, vec![Spring { joint: "pivot".into(), stiffness: 50.0, rest: 0.0 }]);
        let mut w = World::with_capacity(4, 4);
        let m = build(&u, &mut w, &Config { floating: false, ..Default::default() }).unwrap();
        assert_eq!(m.dropped_limits, vec!["pivot".to_string()]);
        // Gravity off, turned 0.2 rad: the spring pulls it back through 0 with omega^2 = k / I
        let u2 = &u;
        let mut w = World::with_capacity(4, 4);
        w.set_gravity(Vec3::ZERO);
        let cfg = Config { floating: false, q: HashMap::from([("pivot".to_string(), 0.2)]), ..Default::default() };
        let m = build(u2, &mut w, &cfg).unwrap();
        let i_pivot: f64 = 0.1 + 2.0 * 0.16 + 1e-4 + 0.5 * 0.64;
        let quarter = 0.5 * std::f64::consts::PI * (i_pivot / 50.0).sqrt();
        let hz = 1000.0f32;
        let mut t = 0.0;
        let mut last = joint_angle(&w, &m, u2, "pivot").unwrap();
        loop {
            w.step(1.0 / hz);
            t += 1.0 / hz as f64;
            let a = joint_angle(&w, &m, u2, "pivot").unwrap();
            if a <= 0.0 { t -= (1.0 / hz as f64) * a / (a - last); break; }
            last = a;
            assert!(t < 2.0, "never came back");
        }
        assert!((t / quarter - 1.0).abs() < 0.02, "quarter period {:.4} s, closed form {:.4} s", t, quarter);
    }

    /// Cassie's URDF from the parity data, when it is on this machine.
    #[test]
    fn cassie_loads_with_its_full_mass() {
        let path = "R:/BACKUP REPOS ETC/riftphys-parity-data/cassie/cassie_v2.urdf";
        let Ok(text) = std::fs::read_to_string(path) else { eprintln!("skipped: {} isn't here", path); return; };
        let u = Urdf::parse(&text).unwrap();
        let mut w = World::with_capacity(32, 32);
        let m = build(&u, &mut w, &Config::default()).unwrap();
        let built: f64 = m.bodies.iter().map(|b| b.mass).sum();
        assert!((built - u.total_mass()).abs() < 1e-9, "built {} of {} kg", built, u.total_mass());
        // 23 links less the six on fixed joints (two knee springs, four lumped rod masses); the
        // 16 joints of the dataset's state
        assert_eq!(m.bodies.len(), 23 - 6);
        assert_eq!(m.joints.len(), 16);
        let mut dropped = m.dropped_limits.clone();
        dropped.sort();
        assert_eq!(dropped, ["ankle_spring_joint_left", "ankle_spring_joint_right", "knee_joint_left", "knee_joint_right"]);
    }
}
