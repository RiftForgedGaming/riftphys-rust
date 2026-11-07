// SPDX-License-Identifier: RiftForged Proprietary License
//! Joint energy behaviour with SHAKE projection (`JointProjection::Shake`, the default): free
//! joints keep their energy (no secular drift), joint limits still dissipate, replay is
//! bit-exact. See docs/JOINT_ENERGY_SPEC.md.
use riftphys_articulation::bodies::D6Limit;
use riftphys_core::{BodyId, Isometry, Mat3, Quat, Vec3, Velocity};
use riftphys_geom::MassProps;
use riftphys_world::world::{JointProjection, World};

const G: f32 = 9.81;
const ID: Quat = Quat::IDENTITY;

fn world() -> World {
    let mut w = World::with_capacity(8, 8);
    w.set_gravity(Vec3::new(0.0, -G, 0.0));
    w.set_joint_projection(JointProjection::Shake);
    w
}

fn iso(pos: Vec3) -> Isometry { Isometry { pos, rot: ID } }

fn pin(w: &mut World, at: Vec3) -> BodyId {
    w.add_body(iso(at), Velocity::default(), MassProps::infinite(), false)
}

/// A body with its mass, and its full body-frame inertia.
struct Link { id: BodyId, m: f32, inertia: Mat3 }

/// A 1 m x 0.05 m x 0.05 m rod (2.5 kg), long axis along its local x: robot-limb proportions.
fn rod(w: &mut World, pose: Isometry, vel: Velocity) -> Link {
    let half = Vec3::new(0.5, 0.025, 0.025);
    let mp = MassProps::from_box_density(half, 1000.0);
    Link { id: w.add_body(pose, vel, mp, true), m: 8.0 * half.x * half.y * half.z * 1000.0, inertia: mp.inertia }
}

/// Energy above height `y0`: potential plus linear and rotational kinetic energy.
fn energy(w: &World, l: &Link, y0: f32) -> f32 {
    let p = w.get_body_pose(l.id);
    let v = w.get_body_vel(l.id);
    let r = Mat3::from_quat(p.rot);
    let i_world = r * l.inertia * r.transpose();
    l.m * G * (p.pos.y - y0) + 0.5 * l.m * v.lin.length_squared() + 0.5 * v.ang.dot(i_world * v.ang)
}

/// The world_robotics pendulum: an 80 kg, 2 m cube on a 5 m arm about z, released horizontal.
fn cube_pendulum() -> (World, Link) {
    let mut w = world();
    let p = pin(&mut w, Vec3::new(0.0, 20.0, 0.0));
    let mp = MassProps::from_box_density(Vec3::splat(1.0), 10.0);
    let bob = Link { id: w.add_body(iso(Vec3::new(5.0, 20.0, 0.0)), Velocity::default(), mp, true), m: 80.0, inertia: mp.inertia };
    w.add_hinge_joint(p, bob.id, iso(Vec3::ZERO), iso(Vec3::new(-5.0, 0.0, 0.0)), 2);
    (w, bob)
}

/// Swing energy relative to the first step's at every step over `secs`: (lowest, highest).
fn cube_swing_range(hz: f32, secs: f32) -> (f32, f32) {
    let (mut w, bob) = cube_pendulum();
    let dt = 1.0 / hz;
    w.step(dt);
    let e0 = energy(&w, &bob, 15.0);
    let (mut lo, mut hi) = (1.0f32, 1.0f32);
    for _ in 0..(secs * hz) as usize {
        w.step(dt);
        let r = energy(&w, &bob, 15.0) / e0;
        lo = lo.min(r);
        hi = hi.max(r);
    }
    (lo, hi)
}

/// Symplectic Euler's energy error is bounded (about 0.8% here at 62.5 Hz) and doesn't grow.
/// PBD directions kept 90% after 5 s and 70% after 20 s.
#[test]
fn hinge_pendulum_keeps_its_energy() {
    for (hz, secs, tol) in [(62.5f32, 5.0f32, 0.02f32), (240.0, 5.0, 0.02), (62.5, 20.0, 0.03)] {
        let (lo, hi) = cube_swing_range(hz, secs);
        assert!(lo > 1.0 - tol && hi < 1.0 + tol, "{} Hz, {} s: swing energy ranged {:.4}..{:.4}", hz, secs, lo, hi);
    }
}

/// The large-swing period matches the physical pendulum (RK4 reference) within 1% at 62.5 Hz.
#[test]
fn hinge_pendulum_period() {
    let (mut w, bob) = cube_pendulum();
    let i_pivot = bob.inertia.z_axis.z + bob.m * 25.0;
    let k = (bob.m * G * 5.0 / i_pivot) as f64;
    // Reference: released at +90 degrees from vertical; the first turning point on the far side
    // is half a period
    let (mut th, mut om, mut t) = (std::f64::consts::FRAC_PI_2, 0.0f64, 0.0f64);
    let h = 1e-5;
    let f = |th: f64| -k * th.sin();
    loop {
        let (k1t, k1o) = (om, f(th));
        let (k2t, k2o) = (om + 0.5 * h * k1o, f(th + 0.5 * h * k1t));
        let (k3t, k3o) = (om + 0.5 * h * k2o, f(th + 0.5 * h * k2t));
        let (k4t, k4o) = (om + h * k3o, f(th + h * k3t));
        let om_new = om + h / 6.0 * (k1o + 2.0 * k2o + 2.0 * k3o + k4o);
        th += h / 6.0 * (k1t + 2.0 * k2t + 2.0 * k3t + k4t);
        t += h;
        if t > 1.0 && om < 0.0 && om_new >= 0.0 { break; }
        om = om_new;
    }
    let period = 2.0 * t as f32;
    // Engine: the second sign change of the bob's angular velocity after release is one period
    let dt = 1.0 / 62.5;
    let (mut prev, mut turns, mut t_engine) = (0.0f32, 0, 0.0f32);
    for n in 1..2000 {
        w.step(dt);
        let wz = w.get_body_vel(bob.id).ang.z;
        if n > 5 && prev != 0.0 && (prev > 0.0) != (wz > 0.0) { turns += 1; }
        if turns == 2 { t_engine = n as f32 * dt; break; }
        prev = wz;
    }
    assert!(((t_engine - period) / period).abs() < 0.01, "period {:.3} s, reference {:.3} s", t_engine, period);
}

/// Two rods hinged end to end, released horizontal. Chaotic, so check energy and joint
/// integrity, not the trajectory. The fast outer link makes symplectic Euler's bounded offset
/// larger at 62.5 Hz. PBD directions lost 59% (62.5 Hz) and 45% (240 Hz) here.
#[test]
fn double_rod_pendulum_keeps_its_energy() {
    for (hz, lo_ok, hi_ok) in [(62.5f32, 0.97f32, 1.05f32), (240.0, 0.98, 1.02)] {
        let mut w = world();
        let p = pin(&mut w, Vec3::new(0.0, 10.0, 0.0));
        let a = rod(&mut w, iso(Vec3::new(0.5, 10.0, 0.0)), Velocity::default());
        let b = rod(&mut w, iso(Vec3::new(1.5, 10.0, 0.0)), Velocity::default());
        w.add_hinge_joint(p, a.id, iso(Vec3::ZERO), iso(Vec3::new(-0.5, 0.0, 0.0)), 2);
        w.add_hinge_joint(a.id, b.id, iso(Vec3::new(0.5, 0.0, 0.0)), iso(Vec3::new(-0.5, 0.0, 0.0)), 2);
        // Above the lowest configuration (both hanging straight down: centres at 9.5 and 8.5)
        let e = |w: &World| energy(w, &a, 9.5) + energy(w, &b, 8.5);
        let dt = 1.0 / hz;
        w.step(dt);
        let e0 = e(&w);
        let (mut lo, mut hi, mut sep) = (1.0f32, 1.0f32, 0.0f32);
        for _ in 0..(10.0 * hz) as usize {
            w.step(dt);
            let r = e(&w) / e0;
            lo = lo.min(r);
            hi = hi.max(r);
            let (pa, pb) = (w.get_body_pose(a.id), w.get_body_pose(b.id));
            sep = sep.max(((pa.pos + pa.rot * Vec3::new(0.5, 0.0, 0.0)) - (pb.pos + pb.rot * Vec3::new(-0.5, 0.0, 0.0))).length())
                     .max(((pa.pos + pa.rot * Vec3::new(-0.5, 0.0, 0.0)) - Vec3::new(0.0, 10.0, 0.0)).length());
        }
        assert!(lo > lo_ok && hi < hi_ok, "{} Hz: double pendulum energy ranged {:.4}..{:.4}", hz, lo, hi);
        assert!(sep < 0.002, "{} Hz: joint separation {:.4} m", hz, sep);
    }
}

/// A rod on a ball joint swinging round in 3D (a spherical pendulum). Gravity has no torque
/// about the vertical through the pivot, so the vertical angular momentum is conserved as well
/// as the energy. The energy error is bounded but rides on the slow precession (0.99..1.04 at
/// 62.5 Hz, 0.99..1.01 at 240 Hz); PBD directions lost 43% and 18% over the same 10 s.
#[test]
fn ball_joint_spherical_pendulum_conserves_energy_and_momentum() {
    for (hz, e_tol) in [(62.5f32, 0.05f32), (240.0, 0.02)] {
        let mut w = world();
        let pivot = Vec3::new(0.0, 10.0, 0.0);
        let p = pin(&mut w, pivot);
        let d = Vec3::new(60f32.to_radians().sin(), -60f32.to_radians().cos(), 0.0);
        let centre = pivot + d * 0.5;
        let spin = Vec3::new(0.0, 3.0, 0.0);
        let pose = Isometry { pos: centre, rot: Quat::from_rotation_arc(Vec3::X, d) };
        let r = rod(&mut w, pose, Velocity { lin: spin.cross(centre - pivot), ang: spin });
        w.add_ball_joint(p, r.id, iso(Vec3::ZERO), iso(Vec3::new(-0.5, 0.0, 0.0)));
        let l_y = |w: &World| {
            let (ps, v) = (w.get_body_pose(r.id), w.get_body_vel(r.id));
            let rm = Mat3::from_quat(ps.rot);
            ((ps.pos - pivot).cross(v.lin * r.m) + (rm * r.inertia * rm.transpose()) * v.ang).y
        };
        let dt = 1.0 / hz;
        w.step(dt);
        let (e0, l0) = (energy(&w, &r, 9.5), l_y(&w));
        let (mut e_err, mut l_err) = (0.0f32, 0.0f32);
        for _ in 0..(10.0 * hz) as usize {
            w.step(dt);
            e_err = e_err.max((energy(&w, &r, 9.5) / e0 - 1.0).abs());
            l_err = l_err.max((l_y(&w) / l0 - 1.0).abs());
        }
        assert!(e_err < e_tol, "{} Hz: spherical pendulum energy drifted {:.2}%", hz, e_err * 100.0);
        assert!(l_err < 0.02, "{} Hz: vertical angular momentum drifted {:.2}%", hz, l_err * 100.0);
    }
}

/// Limits are inequalities and stay inelastic: a rod swinging into its stop loses energy there
/// (SHAKE must not make the stop bounce).
#[test]
fn hinge_limit_still_absorbs_energy() {
    for hz in [62.5f32, 240.0] {
        let mut w = world();
        let p = pin(&mut w, Vec3::new(0.0, 10.0, 0.0));
        let a = rod(&mut w, iso(Vec3::new(0.5, 10.0, 0.0)), Velocity::default());
        let j = w.add_hinge_joint(p, a.id, iso(Vec3::ZERO), iso(Vec3::new(-0.5, 0.0, 0.0)), 2);
        // Released at 0; the free swing would reach -180 degrees. The stop sits past the bottom,
        // at -1.8 rad (-103 degrees).
        w.set_d6_limit_r(j, 2, Some(D6Limit { min: -1.8, max: 0.1 }));
        let dt = 1.0 / hz;
        w.step(dt);
        let e0 = energy(&w, &a, 9.5);
        let mut peak = 1.0f32;
        for _ in 0..(5.0 * hz) as usize {
            w.step(dt);
            peak = peak.max(energy(&w, &a, 9.5) / e0);
        }
        let kept = energy(&w, &a, 9.5) / e0;
        assert!(kept < 0.5 && peak < 1.05, "{} Hz: kept {:.3} of the swing energy after the stop (peak {:.3})", hz, kept, peak);
    }
}

#[test]
fn shake_replays_bit_exactly() {
    let run = || {
        let (mut w, bob) = cube_pendulum();
        for _ in 0..300 { w.step(1.0 / 62.5); }
        (w.get_body_pose(bob.id), w.get_body_vel(bob.id))
    };
    let (a, b) = (run(), run());
    assert_eq!(a.0.pos, b.0.pos);
    assert_eq!(a.0.rot, b.0.rot);
    assert_eq!(a.1.lin, b.1.lin);
    assert_eq!(a.1.ang, b.1.ang);
}

/// SHAKE is the default.
#[test]
fn shake_is_the_default() {
    assert_eq!(World::with_capacity(1, 1).joint_projection(), JointProjection::Shake);
}
