use glam;

#[derive(Copy, Clone, Debug)]
pub struct BalanceParams {
    pub k_accel: f32,       // how strong to pull pelvis back over support
    pub max_accel: f32,     // hard clamp (m/s^2)
    pub com_height: f32,    // nominal pelvis height (for support proj)
    pub quantize: f32,      // e.g., 1e-6 to kill ulp jitter
}
impl Default for BalanceParams {
    fn default() -> Self {
        Self { k_accel: 20.0, max_accel: 30.0, com_height: 1.0, quantize: 1e-6 }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct BalanceCtrl {
    pub params: BalanceParams,
    pub target_world: glam::Vec2, // desired XZ under pelvis (m)
}
impl BalanceCtrl {
    pub fn new(p: BalanceParams) -> Self {
        Self { params: p, target_world: glam::Vec2::ZERO }
    }

    /// Given pelvis position & current support “foot center”, return a horizontal corrective acceleration.
    pub fn step(&mut self, pelvis_pos: glam::Vec3, support_xz: glam::Vec2) -> glam::Vec2 {
        // error in XZ toward (support_xz + target_world)
        let goal = support_xz + self.target_world;
        let err  = goal - glam::Vec2::new(pelvis_pos.x, pelvis_pos.z);

        // proportional accel (no velocity term yet; deterministic & ordered)
        let mut a = err * self.params.k_accel;

        // clamp & quantize (deterministic)
        let len = a.length();
        if len > self.params.max_accel && len > 0.0 {
            a *= self.params.max_accel / len;
        }
        // quantize 0 (or less, or non-finite) means off; dividing by it made NaN
        let q = self.params.quantize;
        if q > 0.0 && q.is_finite() {
            a.x = (a.x / q).round() * q;
            a.y = (a.y / q).round() * q;
        }
        a
    }
}
