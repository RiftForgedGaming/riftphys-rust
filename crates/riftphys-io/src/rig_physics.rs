#![deny(missing_docs)]

//! Build coarse physics rigs from imported `RigData` and optionally spawn them into `World`.

use anyhow::{anyhow, Result};
use glam::{Mat4, Quat as GQuat, Vec3 as GVec3};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use crate::RigData;

// Core types (Vec3 = Vec3A alias).
use riftphys_core::{vec3, Isometry, Quat, Velocity};
use riftphys_geom::Shape;
use riftphys_world::world::World;

use riftphys_articulation::bodies::D6Joint;
use riftphys_materials::materials::{material, MaterialId};

/// Quantize to 1e-6 for deterministic JSON emission.
#[inline]
fn q6(x: f32) -> f32 {
    (x * 1.0e6_f32).round() * 1.0e-6_f32
}

#[inline]
fn pack_iso_q(p: GVec3, q: GQuat) -> [f32; 7] {
    let qn = q.normalize();
    [q6(p.x), q6(p.y), q6(p.z), q6(qn.x), q6(qn.y), q6(qn.z), q6(qn.w)]
}

#[inline]
fn unpack_iso(a: [f32; 7]) -> Isometry {
    Isometry {
        pos: vec3(a[0], a[1], a[2]),
        rot: Quat::from_xyzw(a[3], a[4], a[5], a[6]).normalize(),
    }
}

/// A single physics link definition (body + one collider).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LinkDef {
    /// Stable link name.
    pub name: String,
    /// Optional UI mass hint (not used when spawning; `MassProps` drives actual mass).
    pub mass: f32,
    /// Collider shape.
    pub shape: ColliderShape,
    /// World pose `[px,py,pz,qx,qy,qz,qw]`.
    pub pose_ws: [f32; 7],
    /// Material id for mass/friction selection.
    pub material: MaterialId,
}

/// Collider shape options.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ColliderShape {
    /// Capsule radius + half-height (cylindrical part).
    Capsule {
        /// Radius.
        r: f32,
        /// Half-height of the cylindrical section (not including hemispheres).
        hh: f32,
    },

    /// Box half-extents.
    Box {
        /// Half-extent on X.
        hx: f32,
        /// Half-extent on Y.
        hy: f32,
        /// Half-extent on Z.
        hz: f32,
    },

    /// Sphere radius.
    Sphere {
        /// Radius.
        r: f32,
    },
}


/// Joint definitions (coarse presets).
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum JointDef {
    /// Ball joint (currently spawns as a ball preset).
    Ball {
        /// Joint name (stable identifier).
        name: String,
        /// Parent link name.
        parent: String,
        /// Child link name.
        child: String,
        /// Parent-side joint frame `[px,py,pz,qx,qy,qz,qw]` in parent local space.
        frame_a: [f32; 7],
        /// Child-side joint frame `[px,py,pz,qx,qy,qz,qw]` in child local space.
        frame_b: [f32; 7],
        /// Optional angular limits.
        limits: Option<BallLimits>,
        /// Optional angular drives (per-axis).
        drives: Option<Drive6>,
    },
    /// Hinge joint preset.
    Hinge {
        /// Joint name (stable identifier).
        name: String,
        /// Parent link name.
        parent: String,
        /// Child link name.
        child: String,
        /// Which axis index (0=X,1=Y,2=Z) represents the hinge axis in this preset.
        hinge_axis: usize,
        /// Parent-side joint frame `[px,py,pz,qx,qy,qz,qw]` in parent local space.
        frame_a: [f32; 7],
        /// Child-side joint frame `[px,py,pz,qx,qy,qz,qw]` in child local space.
        frame_b: [f32; 7],
        /// Optional hinge limits `[min,max]` (radians).
        limit: Option<[f32; 2]>,
        /// Optional hinge drive.
        drive: Option<Drive1>,
    },
    /// D6 joint with enable flags (limits/drives can be wired later).
    D6 {
        /// Joint name (stable identifier).
        name: String,
        /// Parent link name.
        parent: String,
        /// Child link name.
        child: String,
        /// Parent-side joint frame `[px,py,pz,qx,qy,qz,qw]` in parent local space.
        frame_a: [f32; 7],
        /// Child-side joint frame `[px,py,pz,qx,qy,qz,qw]` in child local space.
        frame_b: [f32; 7],
        /// Translation DOF enabled flags `[x,y,z]`.
        t_enabled: [bool; 3],
        /// Rotation DOF enabled flags `[x,y,z]`.
        r_enabled: [bool; 3],
        /// Optional translation limits per axis `[[min,max];3]`.
        t_limit: Option<[[f32; 2]; 3]>,
        /// Optional rotation limits per axis `[[min,max];3]`.
        r_limit: Option<[[f32; 2]; 3]>,
        /// Optional translation drives per axis.
        t_drive: Option<Drive3>,
        /// Optional rotation drives per axis.
        r_drive: Option<Drive3>,
    },
}

/// Ball joint limit ranges (radians).
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct BallLimits {
    /// Swing about local Y `[min,max]`.
    pub swing_y: [f32; 2],
    /// Swing about local Z `[min,max]`.
    pub swing_z: [f32; 2],
    /// Twist about local X `[min,max]`.
    pub twist: [f32; 2],
}

/// Single-axis drive parameters.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Drive1 {
    /// Target angle/position (units depend on joint type; typically radians).
    pub target: f32,
    /// Proportional gain.
    pub kp: f32,
    /// Derivative gain.
    pub kd: f32,
}

/// 3-axis drive parameters.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Drive3 {
    /// Target per axis `[x,y,z]`.
    pub target: [f32; 3],
    /// Proportional gain per axis `[x,y,z]`.
    pub kp: [f32; 3],
    /// Derivative gain per axis `[x,y,z]`.
    pub kd: [f32; 3],
}

/// 6-axis drive bundle (Rx/Ry/Rz) for ball preset.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Drive6 {
    /// Drive about local X.
    pub rx: Drive1,
    /// Drive about local Y.
    pub ry: Drive1,
    /// Drive about local Z.
    pub rz: Drive1,
}

/// Physics rig definition (links + joints).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicsRig {
    /// Bodies/colliders to spawn.
    pub links: Vec<LinkDef>,
    /// Constraints to spawn after bodies.
    pub joints: Vec<JointDef>,
}

/// Build a coarse humanoid-ish physics rig from a `RigData` skeleton.
///
/// Re-opens the GLB/GLTF referenced in `rig.source_file` to read joint transforms.
/// Deterministic output (quantized poses).
pub fn humanoid_from_rig(rig: &RigData) -> Result<PhysicsRig> {
    fn canon(s: &str) -> String {
        s.to_ascii_lowercase()
            .chars()
            .filter(|c| c.is_ascii_alphanumeric() || *c == '_' || *c == '.' || *c == ':')
            .collect()
    }

    fn find_idx(names: &[String], side: Option<char>, tokens: &[&str]) -> Option<usize> {
        for (i, n) in names.iter().enumerate() {
            let c = canon(n);
            let side_ok = match side {
                Some('l') => c.contains("_l") || c.ends_with('l') || c.contains(".l") || c.contains("left"),
                Some('r') => c.contains("_r") || c.ends_with('r') || c.contains(".r") || c.contains("right"),
                _ => true,
            };
            if !side_ok {
                continue;
            }
            if tokens.iter().all(|t| c.contains(&t.to_ascii_lowercase())) {
                return Some(i);
            }
        }
        None
    }

    let gltf = gltf::Gltf::open(&rig.source_file)?;
    let skins: Vec<gltf::Skin<'_>> = gltf.document.skins().collect();
    let skin = skins
        .get(rig.skin_index)
        .ok_or_else(|| anyhow!("skin {} not found in {}", rig.skin_index, rig.source_file))?;

    let joint_nodes: Vec<gltf::Node<'_>> = skin.joints().collect();
    if joint_nodes.is_empty() {
        return Err(anyhow!("skin {} has zero joints", rig.skin_index));
    }

    let all_nodes: Vec<gltf::Node<'_>> = gltf.document.nodes().collect();
    let mut parent_of: Vec<Option<usize>> = vec![None; all_nodes.len()];
    for n in &all_nodes {
        for ch in n.children() {
            parent_of[ch.index()] = Some(n.index());
        }
    }

    let mut global: Vec<Mat4> = vec![Mat4::IDENTITY; joint_nodes.len()];
    for (i, n) in joint_nodes.iter().enumerate() {
        let mut m = Mat4::from_cols_array_2d(&n.transform().matrix());
        let mut cur = parent_of[n.index()];
        while let Some(pi) = cur {
            let pn = &all_nodes[pi];
            m = Mat4::from_cols_array_2d(&pn.transform().matrix()) * m;
            cur = parent_of[pi];
        }
        global[i] = m;
    }

    let names: Vec<String> = joint_nodes
        .iter()
        .map(|n| n.name().unwrap_or("").to_string())
        .collect();

    // --- Identification ---
    let pelvis = names.iter().position(|n| {
        let c = canon(n);
        c.contains("pelvis") || c.contains("hips")
    }).unwrap_or_else(|| 1.min(names.len().saturating_sub(1)));

    let l_thigh = find_idx(&names, Some('l'), &["thigh"]).or_else(|| find_idx(&names, Some('l'), &["upleg"])).ok_or_else(|| anyhow!("l_thigh missing"))?;
    let l_shank = find_idx(&names, Some('l'), &["calf"]).or_else(|| find_idx(&names, Some('l'), &["shin"])).ok_or_else(|| anyhow!("l_shank missing"))?;
    let l_foot  = find_idx(&names, Some('l'), &["foot"]).ok_or_else(|| anyhow!("l_foot missing"))?;

    let r_thigh = find_idx(&names, Some('r'), &["thigh"]).or_else(|| find_idx(&names, Some('r'), &["upleg"])).ok_or_else(|| anyhow!("r_thigh missing"))?;
    let r_shank = find_idx(&names, Some('r'), &["calf"]).or_else(|| find_idx(&names, Some('r'), &["shin"])).ok_or_else(|| anyhow!("r_shank missing"))?;
    let r_foot  = find_idx(&names, Some('r'), &["foot"]).ok_or_else(|| anyhow!("r_foot missing"))?;

    // Hand nodes for Melee
    let l_hand  = find_idx(&names, Some('l'), &["hand"]).or_else(|| find_idx(&names, Some('l'), &["wrist"])).ok_or_else(|| anyhow!("l_hand missing"))?;
    let r_hand  = find_idx(&names, Some('r'), &["hand"]).or_else(|| find_idx(&names, Some('r'), &["wrist"])).ok_or_else(|| anyhow!("r_hand missing"))?;

    // --- Transforms ---
    let p = |i: usize| -> GVec3 {
        let m = global[i];
        GVec3::new(m.w_axis.x, m.w_axis.y, m.w_axis.z)
    };
    let seg_len = |a: usize, b: usize| -> f32 { (p(b) - p(a)).length().max(0.15) };

    let target_ws = GVec3::new(0.0, 1.20, 0.50);
    let delta = target_ws - p(pelvis);
    let pw = |i: usize| -> GVec3 { p(i) + delta };

    // --- Build PhysicsRig ---
    let links = vec![
        LinkDef { name: "pelvis".into(), mass: 12.0, shape: ColliderShape::Capsule { r: 0.15, hh: 0.35 }, pose_ws: pack_iso_q(pw(pelvis), GQuat::IDENTITY), material: MaterialId::Default },

        LinkDef { name: "l_thigh".into(), mass: 7.0, shape: ColliderShape::Capsule { r: 0.09, hh: seg_len(l_thigh, l_shank)*0.5 }, pose_ws: pack_iso_q(pw(l_thigh), GQuat::IDENTITY), material: MaterialId::Default },
        LinkDef { name: "l_shank".into(), mass: 4.5, shape: ColliderShape::Capsule { r: 0.08, hh: seg_len(l_shank, l_foot)*0.5 }, pose_ws: pack_iso_q(pw(l_shank), GQuat::IDENTITY), material: MaterialId::Default },
        LinkDef { name: "l_foot".into(),  mass: 2.0, shape: ColliderShape::Capsule { r: 0.05, hh: 0.12 }, pose_ws: pack_iso_q(pw(l_foot), GQuat::from_rotation_z(-std::f32::consts::FRAC_PI_2)), material: MaterialId::Default },

        LinkDef { name: "r_thigh".into(), mass: 7.0, shape: ColliderShape::Capsule { r: 0.09, hh: seg_len(r_thigh, r_shank)*0.5 }, pose_ws: pack_iso_q(pw(r_thigh), GQuat::IDENTITY), material: MaterialId::Default },
        LinkDef { name: "r_shank".into(), mass: 4.5, shape: ColliderShape::Capsule { r: 0.08, hh: seg_len(r_shank, r_foot)*0.5 }, pose_ws: pack_iso_q(pw(r_shank), GQuat::IDENTITY), material: MaterialId::Default },
        LinkDef { name: "r_foot".into(),  mass: 2.0, shape: ColliderShape::Capsule { r: 0.05, hh: 0.12 }, pose_ws: pack_iso_q(pw(r_foot), GQuat::from_rotation_z(-std::f32::consts::FRAC_PI_2)), material: MaterialId::Default },

        // Melee Anchor Points
        LinkDef { name: "l_hand".into(),  mass: 1.5, shape: ColliderShape::Sphere { r: 0.06 }, pose_ws: pack_iso_q(pw(l_hand), GQuat::IDENTITY), material: MaterialId::Default },
        LinkDef { name: "r_hand".into(),  mass: 1.5, shape: ColliderShape::Sphere { r: 0.06 }, pose_ws: pack_iso_q(pw(r_hand), GQuat::IDENTITY), material: MaterialId::Default },
    ];

    let ziso = pack_iso_q(GVec3::ZERO, GQuat::IDENTITY);

    // Changed to `mut` so we can dynamically add the shoulders
    let mut joints = vec![
        JointDef::Ball { name: "l_hip".into(), parent: "pelvis".into(), child: "l_thigh".into(), frame_a: ziso, frame_b: ziso, limits: Some(BallLimits { swing_y: [-0.6, 0.8], swing_z: [-0.4, 0.6], twist: [-0.8, 0.8] }), drives: None },
        JointDef::Hinge { name: "l_knee".into(), parent: "l_thigh".into(), child: "l_shank".into(), hinge_axis: 2, frame_a: ziso, frame_b: ziso, limit: Some([0.0, 2.2]), drive: None },
        JointDef::Ball { name: "l_ankle".into(), parent: "l_shank".into(), child: "l_foot".into(), frame_a: ziso, frame_b: ziso, limits: Some(BallLimits { swing_y: [-0.6, 0.6], swing_z: [-0.4, 0.4], twist: [-0.5, 0.5] }), drives: None },

        JointDef::Ball { name: "r_hip".into(), parent: "pelvis".into(), child: "r_thigh".into(), frame_a: ziso, frame_b: ziso, limits: Some(BallLimits { swing_y: [-0.6, 0.8], swing_z: [-0.4, 0.6], twist: [-0.8, 0.8] }), drives: None },
        JointDef::Hinge { name: "r_knee".into(), parent: "r_thigh".into(), child: "r_shank".into(), hinge_axis: 2, frame_a: ziso, frame_b: ziso, limit: Some([0.0, 2.2]), drive: None },
        JointDef::Ball { name: "r_ankle".into(), parent: "r_shank".into(), child: "r_foot".into(), frame_a: ziso, frame_b: ziso, limits: Some(BallLimits { swing_y: [-0.6, 0.6], swing_z: [-0.4, 0.4], twist: [-0.5, 0.5] }), drives: None },
    ];

    // FIX: Attach the hands to the pelvis so they don't fall to the floor
    joints.push(JointDef::Ball {
        name: "l_shoulder".into(),
        parent: "pelvis".into(),
        child: "l_hand".into(),
        frame_a: pack_iso_q(pw(l_hand) - pw(pelvis), GQuat::IDENTITY),
        frame_b: ziso,
        limits: None,
        drives: None,
    });

    joints.push(JointDef::Ball {
        name: "r_shoulder".into(),
        parent: "pelvis".into(),
        child: "r_hand".into(),
        frame_a: pack_iso_q(pw(r_hand) - pw(pelvis), GQuat::IDENTITY),
        frame_b: ziso,
        limits: None,
        drives: None,
    });

    Ok(PhysicsRig { links, joints })
}

/// Name→IDs map returned by `load_into_world`.
pub struct RigMap {
    /// Body ids keyed by `LinkDef.name`.
    pub body: HashMap<String, riftphys_core::BodyId>,
    /// Joint ids keyed by `JointDef` name.
    pub joint: HashMap<String, riftphys_core::JointId>,
}

impl RigMap {
    /// Get a body id by link name.
    pub fn body(&self, name: &str) -> riftphys_core::BodyId {
        *self.body.get(name).expect("body name")
    }

    /// Get a joint id by joint name.
    pub fn joint(&self, name: &str) -> riftphys_core::JointId {
        *self.joint.get(name).expect("joint name")
    }
}

/// Create bodies, colliders, and joints in `world`.
/// Deterministic creation order (links and joints sorted by name).
pub fn load_into_world(world: &mut World, rig: &PhysicsRig) -> Result<RigMap> {
    use riftphys_geom::MassProps;

    let mut body_ids: HashMap<String, riftphys_core::BodyId> = HashMap::new();

    // sort links by name for stable ids
    let mut links = rig.links.clone();
    links.sort_by(|a, b| a.name.cmp(&b.name));

    for l in &links {
        let iso = unpack_iso(l.pose_ws);

        let mass = match l.shape {
            ColliderShape::Capsule { r, hh } => MassProps::from_capsule(r, hh, l.material),
            ColliderShape::Box { hx, hy, hz } => MassProps::from_box(vec3(hx, hy, hz), l.material),
            ColliderShape::Sphere { r } => MassProps::from_sphere(r, l.material),
        };

        let id = world.add_body(iso, Velocity::default(), mass, true);

        let mat = material(l.material);
        match l.shape {
            ColliderShape::Capsule { r, hh } => world.add_collider(id, Shape::Capsule { r, hh }, mat),
            ColliderShape::Box { hx, hy, hz } => world.add_collider(id, Shape::Box { hx, hy, hz }, mat),
            ColliderShape::Sphere { r } => world.add_collider(id, Shape::Sphere { r }, mat),
        };

        body_ids.insert(l.name.clone(), id);
    }

    // joints (also sort by name)
    let mut joints_v = rig.joints.clone();
    joints_v.sort_by(|a, b| name_of(a).cmp(name_of(b)));

    let mut joint_ids: HashMap<String, riftphys_core::JointId> = HashMap::new();

    for j in &joints_v {
        match j {
            JointDef::Ball { name, parent, child, frame_a, frame_b, limits: _, drives: _ } => {
                let a = *body_ids.get(parent).ok_or_else(|| anyhow!("missing body {}", parent))?;
                let b = *body_ids.get(child).ok_or_else(|| anyhow!("missing body {}", child))?;
                let fa = unpack_iso(*frame_a);
                let fb = unpack_iso(*frame_b);
                let jid = world.add_ball_joint(a, b, fa, fb);
                joint_ids.insert(name.clone(), jid);
            }
            JointDef::Hinge { name, parent, child, hinge_axis, frame_a, frame_b, .. } => {
                let a = *body_ids.get(parent).ok_or_else(|| anyhow!("missing body {}", parent))?;
                let b = *body_ids.get(child).ok_or_else(|| anyhow!("missing body {}", child))?;
                let fa = unpack_iso(*frame_a);
                let fb = unpack_iso(*frame_b);
                let jid = world.add_hinge_joint(a, b, fa, fb, *hinge_axis);
                joint_ids.insert(name.clone(), jid);
            }
            JointDef::D6 { name, parent, child, frame_a, frame_b, t_enabled, r_enabled, .. } => {
                let a = *body_ids.get(parent).ok_or_else(|| anyhow!("missing body {}", parent))?;
                let b = *body_ids.get(child).ok_or_else(|| anyhow!("missing body {}", child))?;
                let fa = unpack_iso(*frame_a);
                let fb = unpack_iso(*frame_b);

                let mut d6 = D6Joint {
                    a,
                    b,
                    fa,
                    fb,
                    t: [Default::default(), Default::default(), Default::default()],
                    r: [Default::default(), Default::default(), Default::default()],
                };

                for i in 0..3 {
                    d6.t[i].enabled = t_enabled[i];
                    d6.r[i].enabled = r_enabled[i];
                }

                let jid = world.add_d6_joint(d6);
                joint_ids.insert(name.clone(), jid);
            }
        }
    }

    Ok(RigMap { body: body_ids, joint: joint_ids })
}

fn name_of(j: &JointDef) -> &String {
    match j {
        JointDef::Ball { name, .. } => name,
        JointDef::Hinge { name, .. } => name,
        JointDef::D6 { name, .. } => name,
    }
}

/// Procedural weapon attachment specification.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WeaponLink {
    /// Hand to attach to ("l_hand" or "r_hand").
    pub hand_node_name: String,
    /// Weapon name (e.g., "stick_l").
    pub weapon_name: String,
    /// Dimensions: [radius, half_height].
    pub dimensions: [f32; 2],
    /// Local offset from hand center.
    pub offset: [f32; 3],
}

impl PhysicsRig {
    /// Procedurally adds a weapon "stick" to the rig by finding the appropriate hand nodes.
    pub fn with_weapon(mut self, weapon: WeaponLink) -> Self {
        // 1. Define the Weapon Link
        let weapon_link = LinkDef {
            name: weapon.weapon_name.clone(),
            mass: 2.0, // Solid 2kg hitting force
            shape: ColliderShape::Capsule {
                r: weapon.dimensions[0],
                hh: weapon.dimensions[1]
            },
            // Initial pose is relative; load_into_world handles final WS
            pose_ws: pack_iso_q(GVec3::from_slice(&weapon.offset), GQuat::IDENTITY),
            material: MaterialId::Default,
        };

        // 2. Define the Weld (Fixed D6 Joint)
        let weld = JointDef::D6 {
            name: format!("{}_weld", weapon.weapon_name),
            parent: weapon.hand_node_name,
            child: weapon.weapon_name,
            frame_a: pack_iso_q(GVec3::ZERO, GQuat::IDENTITY),
            frame_b: pack_iso_q(GVec3::ZERO, GQuat::IDENTITY),
            t_enabled: [true, true, true], // Locked
            r_enabled: [true, true, true], // Locked
            t_limit: None,
            r_limit: None,
            t_drive: None,
            r_drive: None,
        };

        self.links.push(weapon_link);
        self.joints.push(weld);
        self
    }
}