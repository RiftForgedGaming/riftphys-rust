use std::sync::Arc;

use riftphys_core::StepCtx;
use riftphys_core::models::{AeroModel, AeroQuery};

// Not rounded (PRECISION_SPEC P1); non-finite values become 0.
#[inline]
fn fin(x: f32) -> f32 {
    if x.is_finite() { x } else { 0.0 }
}

/// Deterministic composition wrapper.
///
/// Determinism note:
/// - The *order* of `parts` is the order of accumulation. Keep it stable.
/// - We quantize each part contribution and the running sum to reduce
///   cross-platform float drift.
pub struct CombinedAero {
    parts: Vec<Arc<dyn AeroModel>>,
}

impl CombinedAero {
    /// Create a combined aero model from a stable-ordered list of parts.
    pub fn new(parts: Vec<Arc<dyn AeroModel>>) -> Self {
        Self { parts }
    }

    /// Append a part (keeps deterministic order as pushed).
    pub fn push(&mut self, part: Arc<dyn AeroModel>) {
        self.parts.push(part);
    }

    /// Immutable view of parts (debug/inspection).
    pub fn parts(&self) -> &[Arc<dyn AeroModel>] {
        &self.parts
    }
}

impl AeroModel for CombinedAero {
    fn accel_contrib(&self, ctx: &StepCtx, q: AeroQuery) -> [f32; 3] {
        let mut ax = 0.0f32;
        let mut ay = 0.0f32;
        let mut az = 0.0f32;

        for p in &self.parts {
            let a = p.accel_contrib(ctx, q);
            // quantize per-part contribution
            let px = fin(a[0]);
            let py = fin(a[1]);
            let pz = fin(a[2]);

            // quantize running sum each step
            ax = fin(ax + px);
            ay = fin(ay + py);
            az = fin(az + pz);
        }

        [fin(ax), fin(ay), fin(az)]
    }
}
