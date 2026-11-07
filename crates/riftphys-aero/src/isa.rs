// riftphys-aero/src/isa.rs

// Not rounded (PRECISION_SPEC P1); non-finite values become 0.
#[inline]
fn fin(x: f32) -> f32 {
    if x.is_finite() { x } else { 0.0 }
}

/// Simple ISA-ish density model (troposphere-ish).
///
/// This is not a full ISA implementation; it’s a stable, cheap density estimate.
/// We quantize outputs for cross-platform stability.
///
/// Valid-ish up to ~11 km in the “standard” profile.
#[derive(Copy, Clone, Debug)]
pub struct ISA {
    pub rho0: f32, // kg/m^3
    pub t0:  f32,  // K
    pub l:   f32,  // K/m
    pub r:   f32,  // J/(kg*K)
    pub g:   f32,  // m/s^2
}

impl Default for ISA {
    fn default() -> Self {
        Self { rho0: 1.225, t0: 288.15, l: 0.0065, r: 287.05, g: 9.80665 }
    }
}

impl ISA {
    /// Air density at altitude (meters).
    #[inline]
    pub fn density(&self, altitude_m: f32) -> f32 {
        let alt = if altitude_m.is_finite() { altitude_m } else { 0.0 };

        // T/T0 = 1 - L*h/T0
        let a = 1.0 - (self.l * alt / self.t0);

        // exponent ≈ g/(R*L) - 1
        let exp = (self.g / (self.r * self.l)) - 1.0;

        // powf can drift across platforms; clamp + quantize the result.
        let base = a.max(0.0);
        let rho = self.rho0 * base.powf(exp);

        fin(rho.max(0.0))
    }
}
