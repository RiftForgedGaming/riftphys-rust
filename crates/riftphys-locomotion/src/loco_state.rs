// riftphys-locomotion/src/loco_state.rs
use glam::{Vec3, Quat};

const Q6: f32 = 1.0e-6;
const US_PER_S: f32 = 1.0e6;

#[inline]
pub fn q6(x: f32) -> f32 {
    if x.is_finite() { (x * US_PER_S).round() * Q6 } else { 0.0 }
}

#[inline]
pub fn q6_vec3(v: Vec3) -> Vec3 {
    Vec3::new(q6(v.x), q6(v.y), q6(v.z))
}

#[inline]
pub fn sec_to_us(x: f32) -> u32 {
    if !x.is_finite() { return 0; }
    let v = (x.max(0.0) * US_PER_S).round();
    v.clamp(0.0, u32::MAX as f32) as u32
}

#[inline]
pub fn us_to_q01(us: u32, denom: u32) -> f32 {
    if denom == 0 { return 0.0; }
    q6((us as f32) / (denom as f32))
}

#[derive(Copy, Clone)]
pub struct GaitSpec {
    pub stance_dur: f32,
    pub swing_dur:  f32,
    pub step_len:   f32,
    pub step_h:     f32,
}

impl GaitSpec {
    #[inline]
    pub fn sanitized(mut self) -> Self {
        // No NaNs/negatives leaking into clocks or arc math.
        self.stance_dur = q6(self.stance_dur.max(0.0));
        self.swing_dur  = q6(self.swing_dur.max(0.0));
        self.step_len   = q6(self.step_len.max(0.0));
        self.step_h     = q6(self.step_h.max(0.0));
        self
    }
}

#[derive(Copy, Clone)]
pub struct FootClock {
    pub t: f32, // kept for compatibility/readability (mirrors t_us)
    pub stance_dur: f32,
    pub swing_dur:  f32,

    // hardened internals
    t_us:      u32,
    stance_us: u32,
    swing_us:  u32,
}

impl FootClock {
      pub fn new(stance: f32, swing: f32) -> Self {
        let stance = q6(stance.max(0.0));
        let swing  = q6(swing.max(0.0));

        let stance_us = sec_to_us(stance);
        let swing_us  = sec_to_us(swing);

        Self {
            t: 0.0,
            stance_dur: stance,
            swing_dur:  swing,
            t_us: 0,
            stance_us,
            swing_us,
        }
    }

    #[inline]
    fn cycle_us(&self) -> u32 {
        self.stance_us.saturating_add(self.swing_us)
    }

    /// Step the clock and return (phase, entered_stance_gate).
    /// Gate is deterministic: transitions computed from integer time.
    pub fn step_gate(&mut self, dt: f32) -> (Phase, bool) {
        // Quantize dt to microseconds deterministically.
        let dt_us = sec_to_us(q6(dt.max(0.0)));

        let cycle = self.cycle_us();
        if cycle == 0 {
            // Degenerate spec: treat as permanent stance.
            self.t_us = 0;
            self.t = 0.0;
            return (Phase::Stance(0.0), false);
        }

        let prev_us = self.t_us;
        self.t_us = self.t_us.saturating_add(dt_us);

        // wrap
        if self.t_us >= cycle {
            self.t_us %= cycle;
        }

        // mirror for external visibility/debug
        self.t = q6((self.t_us as f32) / US_PER_S);

        // Enter stance if we wrapped or crossed from swing -> stance
        // (i.e., prev in swing range, now in stance range)
        let prev_in_stance = prev_us < self.stance_us;
        let now_in_stance  = self.t_us < self.stance_us;
        let entered_stance = (!prev_in_stance && now_in_stance) || (prev_us > self.t_us); // wrapped

        let phase = if now_in_stance {
            Phase::Stance(us_to_q01(self.t_us, self.stance_us.max(1)))
        } else {
            let swing_t = self.t_us.saturating_sub(self.stance_us);
            Phase::Swing(us_to_q01(swing_t, self.swing_us.max(1)))
        };

        (phase, entered_stance)
    }

    /// Legacy signature if you still call it from elsewhere.
    pub fn step(&mut self, dt: f32) -> Phase {
        self.step_gate(dt).0
    }
}

#[derive(Copy, Clone)]
pub enum Phase {
    Stance(f32),
    Swing(f32),
}

#[inline]
fn hermite(p0: Vec3, v0: Vec3, p1: Vec3, v1: Vec3, s: f32) -> Vec3 {
    let s = q6(s.clamp(0.0, 1.0));
    let s2 = s * s;
    let s3 = s2 * s;
    let h00 =  2.0 * s3 - 3.0 * s2 + 1.0;
    let h10 =        s3 - 2.0 * s2 + s;
    let h01 = -2.0 * s3 + 3.0 * s2;
    let h11 =        s3 -       s2;
    q6_vec3(h00 * p0 + h10 * v0 + h01 * p1 + h11 * v1)
}

// Legacy X+ swing helper
pub fn swing_target(p_stance: Vec3, step_len: f32, lift: f32, s: f32) -> Vec3 {
    let step_len = q6(step_len.max(0.0));
    let lift     = q6(lift.max(0.0));

    let p0  = p_stance;
    let p1  = p_stance + Vec3::new(step_len, 0.0, 0.0);
    let mid = p_stance + Vec3::new(step_len * 0.5, lift, 0.0);

    if s <= 0.5 {
        hermite(p0, Vec3::ZERO, mid, Vec3::ZERO, s * 2.0)
    } else {
        hermite(mid, Vec3::ZERO, p1, Vec3::ZERO, (s - 0.5) * 2.0)
    }
}

#[derive(Clone)]
pub struct LocoState {
    pub pelvis: u32,
    pub left:   u32,
    pub right:  u32,

    pub left_clk:  FootClock,
    pub right_clk: FootClock,
    pub spec:      GaitSpec,

    pub left_stance_anchor_ws:  Vec3,
    pub right_stance_anchor_ws: Vec3,

    pub heading_yaw_rad: f32,
    pub pending:         Option<TransitionPlan>,

    prev_l_stance: bool,
    prev_r_stance: bool,
}

impl LocoState {
    pub fn new(pelvis: u32, left: u32, right: u32, spec: GaitSpec) -> Self {
        let spec = spec.sanitized();

        let mut left_clk = FootClock::new(spec.stance_dur, spec.swing_dur);
        let mut right_clk = FootClock::new(spec.stance_dur, spec.swing_dur);

        // Offset the right leg by exactly half the cycle to establish an alternating bipedal gait.
        // This ensures one foot is always grounded while the other executes its swing hermite curve.
        let half_cycle = (spec.stance_dur + spec.swing_dur) * 0.5;
        right_clk.step_gate(half_cycle);

        Self {
            pelvis,
            left,
            right,
            left_clk,
            right_clk,
            spec,
            left_stance_anchor_ws:  Vec3::ZERO,
            right_stance_anchor_ws: Vec3::ZERO,
            heading_yaw_rad: 0.0,
            pending: None,
            prev_l_stance: false,
            prev_r_stance: false,
        }
    }
    pub fn enqueue(&mut self, plan: TransitionPlan) {
        self.pending = Some(plan);
    }

    pub fn step_and_plan(&mut self, dt: f32) -> StepDirective {
        let (lp, l_enter) = self.left_clk.step_gate(dt);
        let (rp, r_enter) = self.right_clk.step_gate(dt);

        let l_stance_now = matches!(lp, Phase::Stance(_));
        let r_stance_now = matches!(rp, Phase::Stance(_));

        // Your old “s <= 0.05” gate was float-sensitive.
        // This gate is already integer-based, so it’s stable.
        let gate = l_enter || r_enter;

        let mut out = StepDirective::default();

        if gate {
            if let Some(plan) = self.pending {
                match plan {
                    TransitionPlan::Turn(TurnPlan { mut yaw_total_rad, mut steps }) => {
                        steps = steps.max(1);
                        yaw_total_rad = q6(yaw_total_rad);
                        let per_step = q6(yaw_total_rad / steps as f32);

                        out.yaw_delta_rad = per_step;
                        self.heading_yaw_rad = q6(self.heading_yaw_rad + per_step);

                        let remaining = steps - 1;
                        if remaining > 0 {
                            self.pending = Some(TransitionPlan::Turn(TurnPlan {
                                yaw_total_rad: q6(yaw_total_rad - per_step),
                                steps: remaining,
                            }));
                        } else {
                            self.pending = None;
                        }

                        out.left_step_len  = self.spec.step_len;
                        out.right_step_len = self.spec.step_len;
                    }
                    TransitionPlan::Start(StartPlan { first_step_len }) => {
                        out.left_step_len  = q6(first_step_len.max(0.0));
                        out.right_step_len = q6(first_step_len.max(0.0));
                        self.pending = None;
                    }
                    TransitionPlan::Stop(_) => {
                        out.left_step_len  = 0.0;
                        out.right_step_len = 0.0;
                        out.stop_after = true;
                        self.pending = None;
                    }
                }
            } else {
                out.left_step_len  = self.spec.step_len;
                out.right_step_len = self.spec.step_len;
            }
        } else {
            out.left_step_len  = self.spec.step_len;
            out.right_step_len = self.spec.step_len;
        }

        out.yaw_delta_rad  = q6(out.yaw_delta_rad);
        out.left_step_len  = q6(out.left_step_len);
        out.right_step_len = q6(out.right_step_len);

        self.prev_l_stance = l_stance_now;
        self.prev_r_stance = r_stance_now;

        out
    }

    pub fn heading_dir(&self) -> Vec3 {
        // Still uses trig under the hood; quantize output to keep it from drifting across long runs.
        let yaw = q6(self.heading_yaw_rad);
        let q = Quat::from_rotation_y(yaw);
        q6_vec3(q * Vec3::X)
    }
}

#[derive(Copy, Clone)]
pub struct TurnPlan {
    pub yaw_total_rad: f32,
    pub steps:         u32,
}

#[derive(Copy, Clone)]
pub struct StartPlan {
    pub first_step_len: f32,
}

#[derive(Copy, Clone)]
pub struct StopPlan;

#[derive(Copy, Clone)]
pub enum TransitionPlan {
    Turn(TurnPlan),
    Start(StartPlan),
    Stop(StopPlan),
}

#[derive(Copy, Clone, Default)]
pub struct StepDirective {
    pub yaw_delta_rad:  f32,
    pub left_step_len:  f32,
    pub right_step_len: f32,
    pub stop_after:     bool,
}

pub fn swing_target_dir(
    p_stance: Vec3,
    dir_ws:   Vec3,
    step_len: f32,
    lift:     f32,
    s:        f32,
) -> Vec3 {
    let step_len = q6(step_len.max(0.0));
    let lift     = q6(lift.max(0.0));

    let dir = dir_ws.normalize_or_zero();
    if dir.length_squared() < 1.0e-12 {
        return q6_vec3(p_stance);
    }

    let p0  = p_stance;
    let p1  = p_stance + dir * step_len;
    let mid = p_stance + dir * (step_len * 0.5) + Vec3::new(0.0, lift, 0.0);

    if s <= 0.5 {
        hermite(p0, Vec3::ZERO, mid, Vec3::ZERO, s * 2.0)
    } else {
        hermite(mid, Vec3::ZERO, p1, Vec3::ZERO, (s - 0.5) * 2.0)
    }
}
