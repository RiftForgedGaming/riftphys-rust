//! Learner-facing features, shared by training-data export and live (closed-loop) control so the
//! two can never drift apart.
//!
//! Quadrotor features are in the drone's BODY frame: rotor commands depend on errors relative to
//! the airframe, so body-frame inputs make one mapping valid at every heading. argref's
//! `tools/riftphys_to_jsonl.py` computes the same values from protocol files.

use crate::protocol::EntState;
use riftphys_core::{Quat, Vec3};

/// Names in output order; schemas list features in this order.
pub const QUADROTOR_FEATURES: [&str; 12] = ["ex", "ey", "ez", "vx", "vy", "vz", "gx", "gz", "wx", "wy", "wz", "yaw"];

/// [ex, ey, ez] position error, [vx, vy, vz] velocity, [gx, gz] world up seen by the airframe
/// (tilt), [wx, wy, wz] body rates, yaw (rad) - all body frame except yaw.
pub fn quadrotor_obs(e: &EntState) -> [f32; 12] {
    let q = Quat::from_array(e.rot);
    let inv = q.inverse();
    let pos = Vec3::from_array(e.pos);
    let target = e.target.map(Vec3::from_array).unwrap_or(pos);
    let err = inv * (target - pos);
    let vel = inv * Vec3::from_array(e.vel);
    let ang = inv * Vec3::from_array(e.ang);
    let up = inv * Vec3::Y;
    let fwd = q * Vec3::Z;
    let yaw = fwd.x.atan2(fwd.z);
    [err.x, err.y, err.z, vel.x, vel.y, vel.z, up.x, up.z, ang.x, ang.y, ang.z, yaw]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ent(rot: Quat, pos: Vec3, target: Vec3) -> EntState {
        EntState { eid: 1, kind: "quadrotor".into(), pos: pos.to_array(), rot: rot.to_array(), vel: [0.0; 3], ang: [0.0; 3],
                   target: Some(target.to_array()), act: vec![], demo: None }
    }

    #[test]
    fn error_is_expressed_in_the_body_frame() {
        // Yawed 90 degrees about +y: body z points along world x, so a target 1 m along world x
        // is 1 m ahead on body z.
        let q = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
        let o = quadrotor_obs(&ent(q, Vec3::ZERO, Vec3::X));
        assert!((o[2] - 1.0).abs() < 1e-5 && o[0].abs() < 1e-5, "{:?}", o);
        assert!((o[11] - std::f32::consts::FRAC_PI_2).abs() < 1e-5, "yaw {:?}", o[11]);
    }

    #[test]
    fn level_drone_sees_up_as_up() {
        let o = quadrotor_obs(&ent(Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO));
        assert!(o[6].abs() < 1e-6 && o[7].abs() < 1e-6);
    }
}
