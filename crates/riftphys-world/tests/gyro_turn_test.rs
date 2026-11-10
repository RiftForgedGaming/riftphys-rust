//! The gyroscopic turn on a thin link (PARITY_SPEC 3.9): world angular momentum is conserved
//! through the rotation and the rate is read back through the turned inertia, a fixed-point
//! iteration that only contracts while dt * |deviator of I^-1| * |L| is small. On a needle (a thin
//! link's inertia about its axis far below the crosswise inertia) the turn subdivides.

use riftphys_core::{Isometry, Mat3, Quat, Vec3, Velocity};
use riftphys_geom::MassProps;
use riftphys_world::world::World;

const AXIS: f32 = 1.9e-5;
const CROSS: f32 = 0.01;

fn needle(w: Vec3) -> (World, riftphys_core::BodyId) {
    let mut world = World::with_capacity(4, 4);
    world.set_gravity(Vec3::ZERO);
    let mass = MassProps { mass: 0.6, inv_mass: 1.0 / 0.6, inertia: Mat3::from_diagonal(Vec3::new(AXIS, CROSS, CROSS).into()) };
    let id = world.add_body(Isometry { pos: Vec3::ZERO, rot: Quat::IDENTITY }, Velocity { lin: Vec3::ZERO, ang: w }, mass, true);
    world.set_body_sleep_allowed(id, false);
    (world, id)
}

/// (world angular momentum, energy, rate about the needle's own axis)
fn state(world: &World, id: riftphys_core::BodyId) -> (Vec3, f32, f32) {
    let (r, w) = (world.get_body_pose(id).rot, world.get_body_vel(id).ang);
    let b = r.conjugate() * w;
    let inertia = Vec3::new(AXIS, CROSS, CROSS);
    let l = r * (b * inertia);
    (l, 0.5 * b.dot(b * inertia), b.x)
}

/// A free needle spinning crosswise, the case that amplified: its momentum holds, it gains no
/// energy, and its rate about its own axis, a constant of an axisymmetric body, stays within
/// floating-point reach of where it started.
#[test]
fn a_free_needle_keeps_its_momentum_energy_and_roll() {
    for (hz, w) in [(240.0f32, Vec3::new(1.0, 10.0, 0.0)), (60.0, Vec3::new(1.0, 10.0, 0.0)), (240.0, Vec3::new(5.0, 3.0, 40.0)), (240.0, Vec3::new(0.001, 10.0, 0.0))] {
        let (mut world, id) = needle(w);
        let (l0, e0, roll0) = state(&world, id);
        let (mut peak, mut top) = (0.0f32, e0);
        let mut worst_l = 0.0f32;
        for _ in 0..(10.0 * hz) as usize {
            world.step(1.0 / hz);
            let (l, e, roll) = state(&world, id);
            assert!(l.is_finite() && e.is_finite(), "non-finite at {} Hz", hz);
            peak = peak.max((roll - roll0).abs());
            top = top.max(e);
            worst_l = worst_l.max((l - l0).length() / l0.length());
        }
        println!("{} Hz, w {:?}: roll moved by at most {:.4} rad/s (of {}); energy up {:.4}%; L off by {:.2e}", hz, w, peak, roll0, 100.0 * (top / e0 - 1.0), worst_l);
        assert!(worst_l < 1e-3, "{} Hz: L drifted {:.2e}", hz, worst_l);
        assert!(top <= e0 * 1.01, "{} Hz: energy {} -> {}", hz, e0, top);
        // f32 sets the floor: the axial rate is L read through a rotation good to about 1e-7, so a
        // crosswise L of 0.1 carries roundoff of 5e-4 rad/s per read on a 1.9e-5 axis, and more
        // pieces mean more reads. Undivided, this needle's roll runs away to tens of rad/s.
        let bound = 5e-4 * l0.length() / AXIS;
        assert!(peak < bound, "{} Hz: roll moved {} from {} (bound {})", hz, peak, roll0, bound);
        // A nearly clean spin, where a torque-based gate would run undivided until the error grew
        if roll0 < 0.01 { assert!(peak < 1.0, "almost no roll: moved {}", peak); }
    }
}
