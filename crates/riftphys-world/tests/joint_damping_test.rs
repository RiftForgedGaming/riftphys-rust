// SPDX-License-Identifier: RiftForged Proprietary License
//! PARITY_SPEC W3a: viscous damping and Coulomb friction on D6 axes, as velocity rows. A bar
//! hinged through its centre of mass, no gravity, spun about the hinge, checked against the
//! closed forms: damping decays w as exp(-c t / I); friction stops it after I w0^2 / (2 tau_f).
use riftphys_core::{BodyId, Isometry, JointId, Quat, Vec3, Velocity};
use riftphys_geom::MassProps;
use riftphys_world::world::World;

fn iso(pos: Vec3) -> Isometry { Isometry { pos, rot: Quat::IDENTITY } }

const HALF: Vec3 = Vec3::new(0.5, 0.1, 0.1);
const MASS: f32 = 2.0;

/// The bar's moment of inertia about z.
fn izz() -> f32 { MASS / 3.0 * (HALF.x * HALF.x + HALF.y * HALF.y) }

/// A 2 kg bar hinged about z through its centre to a static anchor, spinning at w0 about z.
fn spinner(w0: f32) -> (World, BodyId, JointId) {
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::ZERO);
    let p = w.add_body(iso(Vec3::ZERO), Velocity::default(), MassProps::infinite(), false);
    let b = w.add_body(iso(Vec3::ZERO), Velocity { lin: Vec3::ZERO, ang: Vec3::new(0.0, 0.0, w0) }, MassProps::from_box_density(HALF, MASS / (8.0 * HALF.x * HALF.y * HALF.z)), true);
    w.set_body_sleep_allowed(b, false);
    let j = w.add_hinge_joint(p, b, iso(Vec3::ZERO), iso(Vec3::ZERO), 2);
    (w, b, j)
}

/// Twist about z, unwrapped step to step.
struct Unwrap { last: f32, total: f32 }
impl Unwrap {
    fn new() -> Self { Unwrap { last: 0.0, total: 0.0 } }
    fn update(&mut self, q: Quat) -> f32 {
        let a = 2.0 * q.z.atan2(q.w);
        let mut d = a - self.last;
        while d > std::f32::consts::PI { d -= 2.0 * std::f32::consts::PI; }
        while d < -std::f32::consts::PI { d += 2.0 * std::f32::consts::PI; }
        self.last = a;
        self.total += d;
        self.total
    }
}

#[test]
fn damped_hinge_decays_at_the_closed_form_rate() {
    let i = izz();
    for hz in [60.0f32, 240.0] {
        for rate in [0.5f32, 2.0] {
            let (mut w, b, j) = spinner(3.0);
            w.set_d6_damping(j, true, 2, rate * i, 0.0);
            let t = 2.0;
            for _ in 0..(t * hz) as usize { w.step(1.0 / hz); }
            let wz = w.get_body_vel(b).ang.z;
            let measured = (3.0 / wz).ln() / t;
            // The implicit damper's per-step factor is 1 / (1 + c dt / I): its rate is
            // ln(1 + c dt / I) / dt, which tends to c / I as c dt / I -> 0 (here at most 0.033,
            // 1.6% below c / I). Checked against the discrete rate to 0.5%, and the continuous
            // one to 2%.
            let h = rate / hz;
            let discrete = (1.0 + h).ln() * hz;
            assert!((measured - discrete).abs() < 0.005 * discrete, "{} Hz, c/I {}: decay rate {:.4}/s, discrete closed form {:.4} (w {:.4})", hz, rate, measured, discrete, wz);
            assert!((measured - rate).abs() < 0.02 * rate, "{} Hz, c/I {}: decay rate {:.4}/s, want {:.4}", hz, rate, measured, rate);
            // Only the hinge axis is damped: the bar stays on its axis
            let v = w.get_body_vel(b);
            assert!(v.ang.x.abs() < 1e-4 && v.ang.y.abs() < 1e-4 && v.lin.length() < 1e-4, "off-axis motion {:?}", v);
        }
    }
}

#[test]
fn friction_only_with_zero_damping_is_finite() {
    // set_d6_damping(c = 0, tau_f > 0): no damper row, so no 1 / (c dt)
    let (mut w, b, j) = spinner(3.0);
    w.set_d6_damping(j, true, 2, 0.0, 0.2);
    for _ in 0..60 { w.step(1.0 / 60.0); }
    let v = w.get_body_vel(b);
    assert!(v.ang.z.is_finite() && v.ang.z < 3.0, "w {:?}", v.ang);
    // Negative and non-finite coefficients are rejected (become 0)
    w.set_d6_damping(j, true, 2, f32::NAN, -1.0);
    let before = w.get_body_vel(b).ang.z;
    w.step(1.0 / 60.0);
    assert!((w.get_body_vel(b).ang.z - before).abs() < 1e-5, "a NaN / negative coefficient acted");
}

#[test]
fn friction_hinge_stops_at_the_closed_form_angle() {
    let i = izz();
    let (w0, tau) = (3.0f32, 0.2f32);
    let want = i * w0 * w0 / (2.0 * tau);
    let t_stop = i * w0 / tau;
    for hz in [60.0f32, 240.0] {
        let (mut w, b, j) = spinner(w0);
        w.set_d6_damping(j, true, 2, 0.0, tau);
        let mut u = Unwrap::new();
        let mut angle = 0.0;
        for _ in 0..((t_stop + 2.0) * hz) as usize { w.step(1.0 / hz); angle = u.update(w.get_body_pose(b).rot); }
        assert!((angle - want).abs() < 0.02 * want, "{} Hz: stopped after {:.4} rad, want {:.4}", hz, angle, want);
        // And stays stopped: friction holds at zero velocity
        assert!(w.get_body_vel(b).ang.z.abs() < 1e-4, "{} Hz: still turning at {:.2e} rad/s", hz, w.get_body_vel(b).ang.z);
    }
}

#[test]
fn very_stiff_damping_is_stable() {
    // c dt / I far above 1: an explicit damper would reverse the spin and blow up
    let i = izz();
    for hz in [60.0f32, 240.0] {
        let (mut w, b, j) = spinner(3.0);
        w.set_d6_damping(j, true, 2, 1.0e4 * i, 0.0);
        let mut last = 3.0f32;
        for _ in 0..hz as usize {
            w.step(1.0 / hz);
            let wz = w.get_body_vel(b).ang.z;
            assert!(wz.is_finite() && wz >= -1e-6 && wz <= last + 1e-6, "{} Hz: w went {} -> {}", hz, last, wz);
            last = wz;
        }
        assert!(last < 1e-3, "{} Hz: w still {}", hz, last);
    }
}

#[test]
fn damping_on_a_limited_hinge_still_stops_at_the_limit() {
    // Damping and a limit on the same axis: the limit's rows still hold
    let (mut w, b, j) = spinner(3.0);
    w.set_d6_damping(j, true, 2, 0.05, 0.0);
    w.set_d6_limit_r(j, 2, Some(riftphys_articulation::bodies::D6Limit { min: -0.5, max: 0.5 }));
    let mut u = Unwrap::new();
    let mut furthest = 0.0f32;
    for _ in 0..480 { w.step(1.0 / 240.0); furthest = furthest.max(u.update(w.get_body_pose(b).rot)); }
    assert!(furthest < 0.5 + 0.02, "passed the limit: {:.4} rad", furthest);
}
