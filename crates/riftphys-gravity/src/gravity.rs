use riftphys_core::{Vec3, StepHasher};

/// A single gravitating body with standard gravitational parameter μ = G*M
#[derive(Copy, Clone, Debug)]
pub struct PointMass {
    pub mu: f32,          // world units^3 / s^2
    pub center: [f32; 3], // world-space position
}

#[inline]
pub const fn point_mass(mu: f32, center: [f32; 3]) -> PointMass {
    PointMass { mu, center }
}

#[derive(Copy, Clone, Debug)]
pub enum GravitySpec {
    /// Constant acceleration (e.g., Earth: [0,-9.81,0])
    Uniform { g: [f32; 3] },

    /// Point-mass field: a = -mu * r / |r|^3
    InverseSquare {
        mu: f32,
        center: [f32; 3],
        min_r: f32,
    },

    /// Simple planet: inverse-square from a center & radius, clamped inside.
    LayeredPlanet {
        surface_g: f32,
        radius: f32,
        center: [f32; 3],
        min_r: f32,
    },

    /// Sum of inverse-square fields from multiple bodies.
    MultiBody {
        bodies: &'static [PointMass],
        min_r: f32,
    },
}

#[inline]
fn q6(x: f32) -> f32 {
    if x.is_finite() {
        (x * 1.0e6_f32).round() * 1.0e-6_f32
    } else {
        0.0
    }
}

/// The field's acceleration is not rounded (PRECISION_SPEC P1); non-finite components become 0.
/// Its parameters still are (`q6`), as inputs, matching `spec_id`.
#[inline]
fn fin_v(a: Vec3) -> Vec3 {
    let f = |x: f32| if x.is_finite() { x } else { 0.0 };
    Vec3::new(f(a.x), f(a.y), f(a.z))
}

#[inline]
fn q6_i64(x: f32) -> i64 {
    if x.is_finite() {
        (x * 1.0e6_f32).round() as i64
    } else {
        0
    }
}

#[inline]
fn finite_vec3(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

#[inline]
fn v3(center: [f32; 3]) -> Vec3 {
    Vec3::new(q6(center[0]), q6(center[1]), q6(center[2]))
}

#[inline]
pub fn eval(spec: &GravitySpec, p: Vec3) -> Vec3 {
    if !finite_vec3(p) {
        return Vec3::ZERO;
    }

    match *spec {
        GravitySpec::Uniform { g } => {
            Vec3::new(q6(g[0]), q6(g[1]), q6(g[2]))
        }

        GravitySpec::InverseSquare { mu, center, min_r } => {
            let mu = q6(mu);
            let c = v3(center);
            let r = p - c;

            let mut min_r = q6(min_r);
            if min_r < 1.0e-6 { min_r = 1.0e-6; }

            let min_r2 = min_r * min_r;
            let r2 = r.length_squared();
            if !r2.is_finite() {
                return Vec3::ZERO;
            }

            let r2 = r2.max(min_r2);
            let inv_r3 = 1.0 / (r2 * r2.sqrt());
            let a = -r * (mu * inv_r3);

            fin_v(a)
        }

        GravitySpec::LayeredPlanet { surface_g, radius, center, min_r } => {
            let surface_g = q6(surface_g);
            let radius = q6(radius.max(0.0));
            let c = v3(center);

            let rv = p - c;
            let dist2 = rv.length_squared();
            if !dist2.is_finite() {
                return Vec3::ZERO;
            }

            let mut min_r = q6(min_r);
            if min_r < 1.0e-6 { min_r = 1.0e-6; }

            let dist = dist2.sqrt().max(min_r);
            let dir = rv / dist;

            // g = g_surface * (R^2 / r^2)
            let g_mag = surface_g * (radius * radius) / (dist * dist);
            let a = -dir * g_mag;

            fin_v(a)
        }

        GravitySpec::MultiBody { bodies, min_r } => {
            let mut min_r = q6(min_r);
            if min_r < 1.0e-6 { min_r = 1.0e-6; }

            let min_r2 = min_r * min_r;

            let mut a = Vec3::ZERO;
            for b in bodies {
                let mu = q6(b.mu);
                let c = v3(b.center);

                let r = p - c;
                let r2 = r.length_squared();
                if !r2.is_finite() {
                    continue;
                }

                let r2 = r2.max(min_r2);
                let inv_r3 = 1.0 / (r2 * r2.sqrt());
                a += -r * (mu * inv_r3);
            }

            fin_v(a)
        }
    }
}

/// Deterministic 64-bit ID for a gravity spec (used as EpochID when swapping).
/// Important: hashes **q6-quantized** scalars, not raw float bits.
#[inline]
pub fn spec_id(spec: &GravitySpec) -> u64 {
    let mut h = StepHasher::new();

    match *spec {
        GravitySpec::Uniform { g } => {
            h.update_bytes(&[0u8]);
            for f in g {
                h.update_bytes(&q6_i64(f).to_le_bytes());
            }
        }

        GravitySpec::InverseSquare { mu, center, min_r } => {
            h.update_bytes(&[1u8]);
            h.update_bytes(&q6_i64(mu).to_le_bytes());
            for f in center {
                h.update_bytes(&q6_i64(f).to_le_bytes());
            }
            h.update_bytes(&q6_i64(min_r).to_le_bytes());
        }

        GravitySpec::LayeredPlanet { surface_g, radius, center, min_r } => {
            h.update_bytes(&[2u8]);
            h.update_bytes(&q6_i64(surface_g).to_le_bytes());
            h.update_bytes(&q6_i64(radius).to_le_bytes());
            for f in center {
                h.update_bytes(&q6_i64(f).to_le_bytes());
            }
            h.update_bytes(&q6_i64(min_r).to_le_bytes());
        }

        GravitySpec::MultiBody { bodies, min_r } => {
            h.update_bytes(&[3u8]);
            h.update_bytes(&q6_i64(min_r).to_le_bytes());

            let len = bodies.len() as u32;
            h.update_bytes(&len.to_le_bytes());

            // Order matters; slice must be stable (you already require &'static).
            for b in bodies {
                h.update_bytes(&q6_i64(b.mu).to_le_bytes());
                for f in b.center {
                    h.update_bytes(&q6_i64(f).to_le_bytes());
                }
            }
        }
    }

    let b = h.finalize();
    u64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]])
}
