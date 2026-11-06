// File: riftphys-materials/src/materials.rs
use std::cmp::Ordering;
use serde::{Serialize, Deserialize};

/// Material catalog is fixed and deterministic.
/// Add more IDs here; order matters only for display (pair mixing is symmetric).
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub enum MaterialId {
    // Core / previously present
    Default, Ice, Rubber, Steel, Grit, Wood, Concrete, Asphalt, Leather, Skin, Carpet, Glass,
    Titanium, Aluminum, Marble, Diamond,

    // Common wet variants
    ConcreteWet, AsphaltWet,

    // Metals
    StainlessSteel, CastIron, WroughtIron, CarbonSteel,
    Copper, Brass, Bronze, TitaniumAlloy, Magnesium, Nickel, Chromium,
    Zinc, Tin, Lead, Gold, Silver, Platinum, AnodizedAluminum,

    // Stones / Minerals
    Granite, Limestone, Sandstone, Slate, Basalt, Obsidian, Travertine, Quartz,

    // Gemstones
    Ruby, Sapphire, Emerald, Topaz, Amethyst, Citrine, Aquamarine, Garnet,
    Opal, Jade, Onyx, Turquoise, Peridot, Moonstone,

    // Ceramics & Glass
    TemperedGlass, BorosilicateGlass, Porcelain,
    CeramicGlazed, CeramicUnglazed, Earthenware, Stoneware,
    TileGlazed, TileUnglazed,

    // Polymers
    PTFE, UHMWPE, HDPE, LDPE, Polypropylene, PomDelrin, Nylon,
    ABS, PvcRigid, Polycarbonate, AcrylicPmma, PEEK, PET, Polyurethane,

    // Rubbers
    RubberSoft, RubberHard, SiliconeRubber, NeopreneRubber, NitrileRubber,

    // Woods
    Oak, Maple, Pine, Birch, Bamboo, Teak, Plywood, MDF,

    // Fabrics / soft
    Cotton, Wool, Denim, Silk, Polyester, Felt, Canvas,

    // Granular / Soils
    SandDry, SandWet, ClayDry, ClayWet, SnowFresh, SnowPacked, Gravel, Mud,

    // Paper / Card
    Paper, Cardboard, CorrugatedCardboard,

    // Composites / Misc
    CarbonFiberEpoxy, Fiberglass, GripTape, PaintedSteel, EvaFoam, PuFoam,
}

/// Canonical single-material properties.
/// Units:
/// - friction coefficients are dimensionless
/// - restitution in [0,1]
/// - stribeck_v (m/s) sets transition speed from static→kinetic
#[derive(Copy,Clone,Debug)]
pub struct MatProps {
    pub mu_s: f32,         // static
    pub mu_k: f32,         // kinetic (high-speed plateau)
    pub restitution: f32,  // bounciness
    pub stribeck_v: f32,   // characteristic slip speed (m/s)
}

/// Pair properties after mixing + overrides; what the solver should use.
#[derive(Copy,Clone,Debug)]
pub struct MatPairProps {
    pub mu_s: f32,
    pub mu_k: f32,        // kinetic plateau (used when v is high)
    pub restitution: f32,
    pub stribeck_v: f32,  // blended; used to compute mu_k(v)
    /// Compliant (Hunt-Crossley) contact, opt in per pair: normal force per contact point
    /// k x (1 + d x') for penetration x (m) and penetration rate x' (m/s), with stiffness k (N/m)
    /// and dissipation d (s/m), as Drake's point contact. 0 (the default) keeps the rigid model:
    /// non-penetration, `restitution` for bounce. A compliant pair ignores `restitution`.
    pub stiffness: f32,
    pub dissipation: f32,
    /// Patch friction, opt in per pair: a manifold of two or more points gets its friction as one
    /// sliding row at the patch centre (limit mu x total load) and one torsional row about the
    /// normal (limit mu x total load x the patch's mean radius), as PhysX's patch friction.
    /// Per-point friction at a face's corners resists spin about the normal as if all the load
    /// sat at the corners: a cube spinning flat stops in 0.54 of the turn a uniformly pressed face
    /// takes. false (the default) keeps per-point friction.
    pub patch_friction: bool,
}

/// Base single-material table.
pub fn props(id: MaterialId) -> MatProps {
    use MaterialId::*;
    match id {
        // Core
        Default  => MatProps { mu_s: 0.60, mu_k: 0.50, restitution: 0.10, stribeck_v: 0.08 },
        Ice      => MatProps { mu_s: 0.05, mu_k: 0.03, restitution: 0.02, stribeck_v: 0.02 },
        Rubber   => MatProps { mu_s: 1.10, mu_k: 0.95, restitution: 0.80, stribeck_v: 0.15 },
        Steel    => MatProps { mu_s: 0.70, mu_k: 0.55, restitution: 0.50, stribeck_v: 0.05 },
        Grit     => MatProps { mu_s: 1.30, mu_k: 1.05, restitution: 0.00, stribeck_v: 0.12 },
        Wood     => MatProps { mu_s: 0.55, mu_k: 0.45, restitution: 0.08, stribeck_v: 0.10 },
        Concrete => MatProps { mu_s: 0.80, mu_k: 0.70, restitution: 0.00, stribeck_v: 0.10 },
        Asphalt  => MatProps { mu_s: 0.90, mu_k: 0.80, restitution: 0.00, stribeck_v: 0.12 },
        Leather  => MatProps { mu_s: 0.80, mu_k: 0.60, restitution: 0.20, stribeck_v: 0.10 },
        Skin     => MatProps { mu_s: 0.55, mu_k: 0.45, restitution: 0.10, stribeck_v: 0.06 },
        Carpet   => MatProps { mu_s: 1.10, mu_k: 0.95, restitution: 0.00, stribeck_v: 0.18 },
        Glass    => MatProps { mu_s: 0.40, mu_k: 0.30, restitution: 0.65, stribeck_v: 0.04 },
        Titanium => MatProps { mu_s: 0.90, mu_k: 0.60, restitution: 0.35, stribeck_v: 0.06 },
        Aluminum => MatProps { mu_s: 0.61, mu_k: 0.47, restitution: 0.40, stribeck_v: 0.05 },
        Marble   => MatProps { mu_s: 0.50, mu_k: 0.40, restitution: 0.12, stribeck_v: 0.08 },
        Diamond  => MatProps { mu_s: 0.20, mu_k: 0.15, restitution: 0.85, stribeck_v: 0.04 },

        // Common wet variants
        ConcreteWet => MatProps { mu_s: 0.60, mu_k: 0.50, restitution: 0.00, stribeck_v: 0.10 },
        AsphaltWet  => MatProps { mu_s: 0.60, mu_k: 0.50, restitution: 0.00, stribeck_v: 0.12 },

        // Metals
        StainlessSteel   => MatProps { mu_s: 0.74, mu_k: 0.57, restitution: 0.50, stribeck_v: 0.05 },
        CastIron         => MatProps { mu_s: 0.40, mu_k: 0.29, restitution: 0.20, stribeck_v: 0.05 },
        WroughtIron      => MatProps { mu_s: 0.55, mu_k: 0.40, restitution: 0.30, stribeck_v: 0.05 },
        CarbonSteel      => MatProps { mu_s: 0.70, mu_k: 0.55, restitution: 0.50, stribeck_v: 0.05 },
        Copper           => MatProps { mu_s: 0.53, mu_k: 0.36, restitution: 0.40, stribeck_v: 0.05 },
        Brass            => MatProps { mu_s: 0.44, mu_k: 0.35, restitution: 0.40, stribeck_v: 0.05 },
        Bronze           => MatProps { mu_s: 0.42, mu_k: 0.34, restitution: 0.40, stribeck_v: 0.05 },
        TitaniumAlloy    => MatProps { mu_s: 0.85, mu_k: 0.58, restitution: 0.35, stribeck_v: 0.06 },
        Magnesium        => MatProps { mu_s: 0.50, mu_k: 0.35, restitution: 0.40, stribeck_v: 0.05 },
        Nickel           => MatProps { mu_s: 0.40, mu_k: 0.33, restitution: 0.45, stribeck_v: 0.05 },
        Chromium         => MatProps { mu_s: 0.43, mu_k: 0.30, restitution: 0.55, stribeck_v: 0.05 },
        Zinc             => MatProps { mu_s: 0.50, mu_k: 0.40, restitution: 0.40, stribeck_v: 0.05 },
        Tin              => MatProps { mu_s: 0.50, mu_k: 0.40, restitution: 0.40, stribeck_v: 0.05 },
        Lead             => MatProps { mu_s: 1.00, mu_k: 0.80, restitution: 0.20, stribeck_v: 0.06 },
        Gold             => MatProps { mu_s: 0.55, mu_k: 0.45, restitution: 0.40, stribeck_v: 0.05 },
        Silver           => MatProps { mu_s: 0.55, mu_k: 0.40, restitution: 0.60, stribeck_v: 0.05 },
        Platinum         => MatProps { mu_s: 0.55, mu_k: 0.45, restitution: 0.50, stribeck_v: 0.05 },
        AnodizedAluminum => MatProps { mu_s: 0.65, mu_k: 0.50, restitution: 0.40, stribeck_v: 0.05 },

        // Stones / Minerals
        Granite    => MatProps { mu_s: 0.65, mu_k: 0.55, restitution: 0.30, stribeck_v: 0.08 },
        Limestone  => MatProps { mu_s: 0.60, mu_k: 0.50, restitution: 0.20, stribeck_v: 0.08 },
        Sandstone  => MatProps { mu_s: 0.60, mu_k: 0.55, restitution: 0.20, stribeck_v: 0.10 },
        Slate      => MatProps { mu_s: 0.65, mu_k: 0.58, restitution: 0.15, stribeck_v: 0.10 },
        Basalt     => MatProps { mu_s: 0.80, mu_k: 0.70, restitution: 0.20, stribeck_v: 0.10 },
        Obsidian   => MatProps { mu_s: 0.40, mu_k: 0.30, restitution: 0.60, stribeck_v: 0.04 },
        Travertine => MatProps { mu_s: 0.55, mu_k: 0.45, restitution: 0.15, stribeck_v: 0.08 },
        Quartz     => MatProps { mu_s: 0.45, mu_k: 0.35, restitution: 0.70, stribeck_v: 0.04 },

        // Gemstones (polished)
        Ruby       => MatProps { mu_s: 0.35, mu_k: 0.28, restitution: 0.80, stribeck_v: 0.04 },
        Sapphire   => MatProps { mu_s: 0.35, mu_k: 0.28, restitution: 0.80, stribeck_v: 0.04 },
        Emerald    => MatProps { mu_s: 0.40, mu_k: 0.32, restitution: 0.70, stribeck_v: 0.05 },
        Topaz      => MatProps { mu_s: 0.40, mu_k: 0.32, restitution: 0.75, stribeck_v: 0.05 },
        Amethyst   => MatProps { mu_s: 0.45, mu_k: 0.36, restitution: 0.70, stribeck_v: 0.05 },
        Citrine    => MatProps { mu_s: 0.45, mu_k: 0.36, restitution: 0.70, stribeck_v: 0.05 },
        Aquamarine => MatProps { mu_s: 0.40, mu_k: 0.32, restitution: 0.72, stribeck_v: 0.05 },
        Garnet     => MatProps { mu_s: 0.45, mu_k: 0.37, restitution: 0.68, stribeck_v: 0.05 },
        Opal       => MatProps { mu_s: 0.50, mu_k: 0.40, restitution: 0.40, stribeck_v: 0.05 },
        Jade       => MatProps { mu_s: 0.45, mu_k: 0.38, restitution: 0.60, stribeck_v: 0.05 },
        Onyx       => MatProps { mu_s: 0.40, mu_k: 0.32, restitution: 0.60, stribeck_v: 0.05 },
        Turquoise  => MatProps { mu_s: 0.50, mu_k: 0.42, restitution: 0.55, stribeck_v: 0.05 },
        Peridot    => MatProps { mu_s: 0.40, mu_k: 0.32, restitution: 0.65, stribeck_v: 0.05 },
        Moonstone  => MatProps { mu_s: 0.45, mu_k: 0.35, restitution: 0.55, stribeck_v: 0.05 },

        // Ceramics & Glass
        TemperedGlass     => MatProps { mu_s: 0.40, mu_k: 0.30, restitution: 0.78, stribeck_v: 0.04 },
        BorosilicateGlass => MatProps { mu_s: 0.38, mu_k: 0.28, restitution: 0.75, stribeck_v: 0.04 },
        Porcelain         => MatProps { mu_s: 0.35, mu_k: 0.30, restitution: 0.60, stribeck_v: 0.05 },
        CeramicGlazed     => MatProps { mu_s: 0.30, mu_k: 0.25, restitution: 0.55, stribeck_v: 0.05 },
        CeramicUnglazed   => MatProps { mu_s: 0.60, mu_k: 0.55, restitution: 0.20, stribeck_v: 0.08 },
        Earthenware       => MatProps { mu_s: 0.65, mu_k: 0.55, restitution: 0.15, stribeck_v: 0.08 },
        Stoneware         => MatProps { mu_s: 0.60, mu_k: 0.52, restitution: 0.20, stribeck_v: 0.08 },
        TileGlazed        => MatProps { mu_s: 0.30, mu_k: 0.25, restitution: 0.55, stribeck_v: 0.05 },
        TileUnglazed      => MatProps { mu_s: 0.60, mu_k: 0.55, restitution: 0.20, stribeck_v: 0.08 },

        // Polymers
        PTFE          => MatProps { mu_s: 0.05, mu_k: 0.04, restitution: 0.30, stribeck_v: 0.02 },
        UHMWPE        => MatProps { mu_s: 0.12, mu_k: 0.10, restitution: 0.35, stribeck_v: 0.03 },
        HDPE          => MatProps { mu_s: 0.20, mu_k: 0.18, restitution: 0.30, stribeck_v: 0.04 },
        LDPE          => MatProps { mu_s: 0.22, mu_k: 0.20, restitution: 0.30, stribeck_v: 0.04 },
        Polypropylene => MatProps { mu_s: 0.30, mu_k: 0.25, restitution: 0.30, stribeck_v: 0.05 },
        PomDelrin    => MatProps { mu_s: 0.20, mu_k: 0.18, restitution: 0.50, stribeck_v: 0.04 },
        Nylon         => MatProps { mu_s: 0.30, mu_k: 0.25, restitution: 0.40, stribeck_v: 0.05 },
        ABS           => MatProps { mu_s: 0.50, mu_k: 0.40, restitution: 0.50, stribeck_v: 0.06 },
        PvcRigid     => MatProps { mu_s: 0.40, mu_k: 0.34, restitution: 0.40, stribeck_v: 0.05 },
        Polycarbonate => MatProps { mu_s: 0.38, mu_k: 0.30, restitution: 0.60, stribeck_v: 0.05 },
        AcrylicPmma  => MatProps { mu_s: 0.35, mu_k: 0.30, restitution: 0.60, stribeck_v: 0.05 },
        PEEK          => MatProps { mu_s: 0.30, mu_k: 0.25, restitution: 0.45, stribeck_v: 0.05 },
        PET           => MatProps { mu_s: 0.32, mu_k: 0.27, restitution: 0.50, stribeck_v: 0.05 },
        Polyurethane  => MatProps { mu_s: 0.70, mu_k: 0.60, restitution: 0.55, stribeck_v: 0.08 },

        // Rubbers
        RubberSoft     => MatProps { mu_s: 1.30, mu_k: 1.05, restitution: 0.85, stribeck_v: 0.18 },
        RubberHard     => MatProps { mu_s: 1.00, mu_k: 0.80, restitution: 0.75, stribeck_v: 0.12 },
        SiliconeRubber => MatProps { mu_s: 1.10, mu_k: 0.90, restitution: 0.75, stribeck_v: 0.18 },
        NeopreneRubber => MatProps { mu_s: 1.05, mu_k: 0.90, restitution: 0.75, stribeck_v: 0.16 },
        NitrileRubber  => MatProps { mu_s: 1.00, mu_k: 0.85, restitution: 0.70, stribeck_v: 0.16 },

        // Woods
        Oak     => MatProps { mu_s: 0.62, mu_k: 0.47, restitution: 0.08, stribeck_v: 0.10 },
        Maple   => MatProps { mu_s: 0.60, mu_k: 0.45, restitution: 0.08, stribeck_v: 0.10 },
        Pine    => MatProps { mu_s: 0.55, mu_k: 0.42, restitution: 0.08, stribeck_v: 0.10 },
        Birch   => MatProps { mu_s: 0.58, mu_k: 0.44, restitution: 0.08, stribeck_v: 0.10 },
        Bamboo  => MatProps { mu_s: 0.50, mu_k: 0.40, restitution: 0.06, stribeck_v: 0.10 },
        Teak    => MatProps { mu_s: 0.55, mu_k: 0.42, restitution: 0.06, stribeck_v: 0.10 },
        Plywood => MatProps { mu_s: 0.50, mu_k: 0.40, restitution: 0.06, stribeck_v: 0.10 },
        MDF     => MatProps { mu_s: 0.45, mu_k: 0.35, restitution: 0.05, stribeck_v: 0.10 },

        // Fabrics / soft
        Cotton     => MatProps { mu_s: 0.60, mu_k: 0.50, restitution: 0.02, stribeck_v: 0.20 },
        Wool       => MatProps { mu_s: 0.70, mu_k: 0.60, restitution: 0.02, stribeck_v: 0.22 },
        Denim      => MatProps { mu_s: 0.80, mu_k: 0.70, restitution: 0.01, stribeck_v: 0.22 },
        Silk       => MatProps { mu_s: 0.45, mu_k: 0.38, restitution: 0.01, stribeck_v: 0.18 },
        Polyester  => MatProps { mu_s: 0.60, mu_k: 0.50, restitution: 0.01, stribeck_v: 0.20 },
        Felt       => MatProps { mu_s: 0.95, mu_k: 0.85, restitution: 0.00, stribeck_v: 0.24 },
        Canvas     => MatProps { mu_s: 1.05, mu_k: 0.90, restitution: 0.00, stribeck_v: 0.22 },

        // Granular / Soils
        SandDry    => MatProps { mu_s: 0.60, mu_k: 0.50, restitution: 0.00, stribeck_v: 0.10 },
        SandWet    => MatProps { mu_s: 0.50, mu_k: 0.40, restitution: 0.00, stribeck_v: 0.10 },
        ClayDry    => MatProps { mu_s: 0.55, mu_k: 0.45, restitution: 0.00, stribeck_v: 0.10 },
        ClayWet    => MatProps { mu_s: 0.40, mu_k: 0.30, restitution: 0.00, stribeck_v: 0.10 },
        SnowFresh  => MatProps { mu_s: 0.20, mu_k: 0.15, restitution: 0.00, stribeck_v: 0.08 },
        SnowPacked => MatProps { mu_s: 0.12, mu_k: 0.08, restitution: 0.00, stribeck_v: 0.06 },
        Gravel     => MatProps { mu_s: 0.65, mu_k: 0.55, restitution: 0.00, stribeck_v: 0.12 },
        Mud        => MatProps { mu_s: 0.30, mu_k: 0.25, restitution: 0.00, stribeck_v: 0.12 },

        // Paper / Card
        Paper               => MatProps { mu_s: 0.50, mu_k: 0.45, restitution: 0.02, stribeck_v: 0.16 },
        Cardboard           => MatProps { mu_s: 0.70, mu_k: 0.60, restitution: 0.01, stribeck_v: 0.18 },
        CorrugatedCardboard => MatProps { mu_s: 0.80, mu_k: 0.70, restitution: 0.01, stribeck_v: 0.18 },

        // Composites / Misc
        CarbonFiberEpoxy => MatProps { mu_s: 0.55, mu_k: 0.50, restitution: 0.30, stribeck_v: 0.06 },
        Fiberglass       => MatProps { mu_s: 0.60, mu_k: 0.55, restitution: 0.30, stribeck_v: 0.08 },
        GripTape         => MatProps { mu_s: 1.50, mu_k: 1.20, restitution: 0.00, stribeck_v: 0.20 },
        PaintedSteel     => MatProps { mu_s: 0.55, mu_k: 0.45, restitution: 0.45, stribeck_v: 0.05 },
        EvaFoam         => MatProps { mu_s: 1.00, mu_k: 0.90, restitution: 0.50, stribeck_v: 0.20 },
        PuFoam          => MatProps { mu_s: 1.10, mu_k: 1.00, restitution: 0.45, stribeck_v: 0.22 },
    }

}


/// Optional explicit pair overrides (deterministic and symmetric).
/// If not present, we use the default mixing law.
#[derive(Copy,Clone,Debug)]
pub struct PairOverride {
    pub mu_s: Option<f32>,
    pub mu_k: Option<f32>,
    pub restitution: Option<f32>,
    pub stribeck_v: Option<f32>,
}

/// A small, curated override table for well-known combinations.
fn override_for(a: MaterialId, b: MaterialId) -> Option<PairOverride> {
    use MaterialId::*;
    // Order-invariant: put (min,max)
    let (x,y) = ordered_pair(a,b);
    match (x,y) {
        (Ice, Rubber) => Some(PairOverride {
            // Rubber on ice often shows strong speed dependence and lower restitution.
            mu_s: Some(0.20), mu_k: Some(0.10), restitution: Some(0.05), stribeck_v: Some(0.08)
        }),
        (Rubber, Asphalt) => Some(PairOverride {
            mu_s: Some(1.10), mu_k: Some(0.95), restitution: Some(0.00), stribeck_v: Some(0.18)
        }),
        (Rubber, Concrete) => Some(PairOverride {
            mu_s: Some(1.00), mu_k: Some(0.85), restitution: Some(0.00), stribeck_v: Some(0.16)
        }),
        (Steel, Steel) => Some(PairOverride {
            mu_s: Some(0.58), mu_k: Some(0.50), restitution: Some(0.08), stribeck_v: Some(0.04)
        }),
        _ => None
    }
}

/// Symmetric (order-independent) pair mix with geometric-mean friction and max restitution.
fn mix_pair(a: MatProps, b: MatProps) -> MatPairProps {
    let mu_s = q((a.mu_s * b.mu_s).abs().sqrt());
    let mu_k = q((a.mu_k * b.mu_k).abs().sqrt());
    let restitution = q(a.restitution.max(b.restitution));
    let stribeck_v  = q(0.5 * (a.stribeck_v + b.stribeck_v));

    MatPairProps { mu_s, mu_k, restitution, stribeck_v, stiffness: 0.0, dissipation: 0.0, patch_friction: false }
}


/// Public entry: compute pair properties with overrides applied.
pub fn pair_props(a: MaterialId, b: MaterialId) -> MatPairProps {
    let base = mix_pair(props(a), props(b));
    if let Some(ov) = override_for(a,b) {
        MatPairProps {
            mu_s:        q(ov.mu_s.unwrap_or(base.mu_s)),
            mu_k:        q(ov.mu_k.unwrap_or(base.mu_k)),
            restitution: q(ov.restitution.unwrap_or(base.restitution)),
            stribeck_v:  q(ov.stribeck_v.unwrap_or(base.stribeck_v)),
            stiffness: 0.0,
            dissipation: 0.0,
            patch_friction: false,
        }
    } else {
        base
    }
}
#[inline]
fn q(x: f32) -> f32 { quantize(x) }

/// Deterministic Stribeck curve for dynamic friction coefficient at slip speed `v`.
/// mu(v) = mu_k + (mu_s - mu_k) * exp(-(v / v_s)^2)
pub fn mu_dynamic(pair: &MatPairProps, v_slip: f32) -> f32 {
    let vs = pair.stribeck_v.max(1.0e-6);
    let t = -(v_slip / vs).powi(2);
    let mu = pair.mu_k + (pair.mu_s - pair.mu_k) * t.exp();
    q(mu)
}

/// Simple 1e-6 quantization for perfect determinism across platforms.
#[inline]
pub fn quantize(x: f32) -> f32 {
    (x * 1.0e6_f32).round() * 1.0e-6_f32
}

#[inline]
fn ordered_pair(a: MaterialId, b: MaterialId) -> (MaterialId, MaterialId) {
    match (a as u32).cmp(&(b as u32)) {
        Ordering::Less | Ordering::Equal => (a,b),
        Ordering::Greater                 => (b,a),
    }
}
/// Physics-oriented properties used by damage, fracture, buoyancy/fluids.
/// Units in SI.
#[derive(Copy, Clone, Debug)]
pub struct PhysProps {
    pub density: f32,                 // kg/m^3 (mass/inertia)
    pub tensile_mpa: f32,             // MPa
    pub compressive_mpa: f32,         // MPa
    pub shear_mpa: f32,               // MPa
    pub fracture_energy_j_per_m2: f32,// Griffith-type energy release rate
    pub porosity: f32,                // 0..1
    pub permeability_m2: f32,         // m^2 (Darcy)
}
/// Energy to remove 1 m^3 of this material as "digging" (J/m^3).
/// Lets you gate voxel edits with one number instead of full stress analysis.
/// Heuristic: base on compressive strength with a safety factor.
/// 1 MPa ≈ 1e6 N/m^2; treat removal cost as k * sigma_c [J/m^3]

#[inline] pub fn contact_props(id: MaterialId) -> MatProps { props(id) }

#[inline] pub fn phys_props(id: MaterialId) -> PhysProps {
    use MaterialId::*;
    match id {
        // numbers are placeholders; tune over time
        Granite => PhysProps { density: 2700.0, tensile_mpa: 7.0, compressive_mpa: 130.0,
            shear_mpa: 20.0, fracture_energy_j_per_m2: 220.0,
            porosity: 0.01, permeability_m2: 1.0e-16 },
        Steel   => PhysProps { density: 7850.0, tensile_mpa: 400.0, compressive_mpa: 250.0,
            shear_mpa: 150.0, fracture_energy_j_per_m2: 800.0,
            porosity: 0.0,  permeability_m2: 0.0 },
        Diamond => PhysProps { density: 3500.0, tensile_mpa: 2800.0, compressive_mpa: 8800.0,
            shear_mpa: 5000.0, fracture_energy_j_per_m2: 5_000.0,
            porosity: 0.0, permeability_m2: 0.0 },
        _ => /* sensible defaults */ PhysProps { density: 1500.0, tensile_mpa: 5.0, compressive_mpa: 30.0,
            shear_mpa: 10.0, fracture_energy_j_per_m2: 50.0,
            porosity: 0.05, permeability_m2: 1.0e-14 },
    }
}

#[inline] pub fn gate(id: MaterialId) -> Gate {
    use MaterialId::*;
    match id {
        Diamond => Gate { tier_lock: 4, kinetic: KineticRequirement::MinSpecificJPerM2(2_500.0) },
        TemperedGlass => Gate { tier_lock: 0, kinetic: KineticRequirement::MinImpulseNS(15.0) },
        _ => Gate { tier_lock: 0, kinetic: KineticRequirement::None },
    }
}


#[inline]
pub fn dig_energy_cost_j_per_m3(mat: MaterialId) -> f32 {
    let p = phys_props(mat);
    let sigma_c_pa = (p.compressive_mpa.max(0.01)) * 1.0e6; // MPa->Pa
    const K: f32 = 15.0; // tune globally; higher = harder to dig
    q(sigma_c_pa * K)
}
/// Minimal legality gate the solver enforces before performing a destructive edit.
/// Keep it tiny so determinism stays pure.
#[derive(Copy, Clone, Debug)]
pub enum KineticRequirement {
    None,
    /// Absolute energy (J) delivered at the contact/region this tick
    MinEnergyJ(f32),
    /// Specific energy per affected area (J/m^2) — good for puncture/tearing
    MinSpecificJPerM2(f32),
    /// Impulse threshold (N·s) — handy for brittle percussive breaks
    MinImpulseNS(f32),
}

#[derive(Copy, Clone, Debug)]
pub struct Gate {
    pub tier_lock: u8,                // progression lock
    pub kinetic: KineticRequirement,  // numeric check
}
#[derive(Copy, Clone, Debug)]
pub struct Material {
    pub id:      MaterialId,
    pub contact: MatProps,
    pub phys:    PhysProps,
    pub gate:    Gate,
    pub loco:    LocoSingle,
    pub mu_s: f32,
    pub mu_k: f32,
    pub restitution: f32,
}

#[derive(Copy, Clone, Debug)]
pub struct LocoSingle {
    pub walkable: bool,        // can a standard biped treat this as “ground”
    pub max_slope_deg: f32,    // comfortable walkable slope (controller can push past)
    pub foothold_quality: f32, // [0..1], 1 = “great traction”, 0 = “avoid if you can”
    pub expected_sink_m: f32,  // typical sink depth under foot load
}
#[inline]
pub fn loco_single(id: MaterialId) -> LocoSingle {
    use MaterialId::*;
    match id {
        // Obviously hostile for gait
        Ice | PTFE | Obsidian => LocoSingle {
            walkable: true,
            max_slope_deg: 5.0,
            foothold_quality: 0.1,
            expected_sink_m: 0.0,
        },

        Mud | ClayWet => LocoSingle {
            walkable: true,
            max_slope_deg: 15.0,
            foothold_quality: 0.4,
            expected_sink_m: 0.05,
        },

        SandDry | SandWet | Gravel => LocoSingle {
            walkable: true,
            max_slope_deg: 20.0,
            foothold_quality: 0.6,
            expected_sink_m: 0.03,
        },

        SnowFresh | SnowPacked => LocoSingle {
            walkable: true,
            max_slope_deg: 15.0,
            foothold_quality: 0.5,
            expected_sink_m: 0.04,
        },

        Carpet | Felt | Canvas | EvaFoam | PuFoam => LocoSingle {
            walkable: true,
            max_slope_deg: 45.0,
            foothold_quality: 0.95,
            expected_sink_m: 0.0,
        },

        // Example “this is not floor” set if you ever want it
        Glass | TemperedGlass | BorosilicateGlass => LocoSingle {
            walkable: true,
            max_slope_deg: 10.0,
            foothold_quality: 0.3,
            expected_sink_m: 0.0,
        },

        // Default: generic solid-ish terrain
        _ => LocoSingle {
            walkable: true,
            max_slope_deg: 45.0,
            foothold_quality: 0.7,
            expected_sink_m: 0.0,
        },
    }
}

/// Single source of truth in this crate:
#[inline]
pub fn material(id: MaterialId) -> Material {
    let c = contact_props(id);
    Material {
        id,
        contact: c,
        phys:    phys_props(id),
        gate:    gate(id),
        loco:    loco_single(id),
        mu_s: c.mu_s,
        mu_k: c.mu_k,
        restitution: c.restitution,
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    #[test] fn symmetry() {
        let p1 = pair_props(MaterialId::Rubber, MaterialId::Ice);
        let p2 = pair_props(MaterialId::Ice, MaterialId::Rubber);
        assert!((p1.mu_s - p2.mu_s).abs() < 1e-12);
        assert!((p1.mu_k - p2.mu_k).abs() < 1e-12);
    }
    #[test] fn stribeck_limits() {
        let p = pair_props(MaterialId::Rubber, MaterialId::Concrete);
        let m0 = mu_dynamic(&p, 0.0);
        let m1 = mu_dynamic(&p, 10.0);
        assert!(m0 <= p.mu_s + 1e-6 && m0 >= p.mu_k - 1e-6);
        assert!((m1 - p.mu_k).abs() < 1e-3);
    }
}
#[inline]
pub fn passes_gate(g: &Gate, energy_j: f32, area_m2: f32, impulse_ns: f32, player_tier: u8) -> bool {
    if player_tier < g.tier_lock { return false; }
    match g.kinetic {
        KineticRequirement::None => true,
        KineticRequirement::MinEnergyJ(t)        => energy_j >= t,
        KineticRequirement::MinSpecificJPerM2(t) => area_m2 > 0.0 && (energy_j / area_m2) >= t,
        KineticRequirement::MinImpulseNS(t)      => impulse_ns >= t,
    }
}
#[derive(Copy, Clone, Debug)]
pub struct LocoPairProps {
    pub walkable: bool,
    pub max_slope_deg: f32,
    pub foothold_quality: f32,
    pub expected_sink_m: f32,
}

#[inline]
pub fn loco_pair(a: MaterialId, b: MaterialId) -> LocoPairProps {
    let la = loco_single(a);
    let lb = loco_single(b);

    // Symmetric, order-independent:
    let walkable = la.walkable && lb.walkable;
    let max_slope_deg = q(la.max_slope_deg.min(lb.max_slope_deg));
    let foothold_quality = q(0.5 * (la.foothold_quality + lb.foothold_quality));
    let expected_sink_m = q(la.expected_sink_m.max(lb.expected_sink_m));

    LocoPairProps {
        walkable,
        max_slope_deg,
        foothold_quality,
        expected_sink_m,
    }
}

