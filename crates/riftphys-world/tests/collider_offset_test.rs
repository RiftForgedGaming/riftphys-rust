// Colliders posed off their body's COM (PARITY_SPEC W4d): every path that reads a collider's
// world pose (AABB and broadphase, the narrowphase, raycasts, the capsule sweep) sees the offset,
// the swept reach covers it, and the contact's lever arm is the offset's.
use riftphys_core::{BodyId, Isometry, Quat, Vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::World;

fn iso(pos: Vec3, rot: Quat) -> Isometry { Isometry { pos, rot } }

/// A static floor whose top is at y = 0.
fn floor(w: &mut World, mat: MaterialId) {
    let g = w.add_body(iso(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY), Velocity::default(), MassProps::infinite(), false);
    w.add_collider(g, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(mat));
}

/// A box body with no collider of its own.
fn body(w: &mut World, pose: Isometry, vel: Velocity, half: Vec3, mass: f32) -> (BodyId, MassProps) {
    let mp = MassProps::from_box_density(half, mass / (8.0 * half.x * half.y * half.z));
    let b = w.add_body(pose, vel, mp, true);
    w.set_body_sleep_allowed(b, false);
    (b, mp)
}

#[test]
fn the_aabb_follows_the_offset() {
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::ZERO);
    let rot = Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
    let (b, _) = body(&mut w, iso(Vec3::new(1.0, 2.0, 3.0), rot), Velocity { lin: Vec3::ZERO, ang: Vec3::new(0.0, 0.0, 1.0) }, Vec3::splat(0.05), 1.0);
    w.add_collider_at(b, iso(Vec3::new(0.2, 0.0, 0.0), Quat::IDENTITY), Shape::Sphere { r: 0.01 }, material(MaterialId::Rubber));
    let centre = |w: &World| { let mut c = Vec3::ZERO; w.for_each_collider(|_, _, _, a| c = 0.5 * (a.min + a.max)); c };
    // Turned 90 degrees about z: the offset's x is world y
    assert!((centre(&w) - Vec3::new(1.0, 2.2, 3.0)).length() < 1e-6, "{:?}", centre(&w));
    w.step(1.0 / 60.0);
    let p = w.get_body_pose(b);
    let expect = p.pos + p.rot * Vec3::new(0.2, 0.0, 0.0);
    assert!((centre(&w) - expect).length() < 1e-6, "after a step {:?}, expected {:?}", centre(&w), expect);
}

#[test]
fn raycasts_and_the_capsule_sweep_see_the_offset() {
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::ZERO);
    let (b, _) = body(&mut w, iso(Vec3::new(0.0, 1.0, 0.0), Quat::IDENTITY), Velocity::default(), Vec3::splat(0.05), 1.0);
    w.add_collider_at(b, iso(Vec3::new(0.5, 0.0, 0.0), Quat::IDENTITY), Shape::Sphere { r: 0.1 }, material(MaterialId::Rubber));
    // Down through the offset sphere: hits its top at y = 1.1
    let hit = w.raycast_exact(Vec3::new(0.5, 3.0, 0.0), Vec3::new(0.0, -1.0, 0.0), 5.0, |_| false).expect("exact ray missed the offset sphere");
    assert_eq!(hit.body, b);
    assert!((hit.point.y - 1.1).abs() < 1e-4, "exact ray hit at {:?}", hit.point);
    let hit = w.raycast(Vec3::new(0.5, 3.0, 0.0), Vec3::new(0.0, -1.0, 0.0), 5.0, None).expect("ray missed the offset sphere");
    assert_eq!(hit.body, b);
    // Down through the COM, where no shape is: a miss
    assert!(w.raycast_exact(Vec3::new(0.0, 3.0, 0.0), Vec3::new(0.0, -1.0, 0.0), 5.0, |_| false).is_none(), "exact ray hit the COM, where there's no shape");
    // A capsule resting on the offset sphere overlaps it; one at the COM doesn't
    let at = Vec3::new(0.5, 1.0, 0.0);
    let s = w.sweep_capsule(at, at, 0.05, 0.1, None).expect("the capsule sweep missed the offset sphere");
    assert_eq!(s.body, b);
    assert!(s.started_overlapping);
    let com = Vec3::new(0.0, 1.0, 0.0);
    assert!(w.sweep_capsule(com, com, 0.05, 0.1, None).is_none(), "the capsule sweep hit the COM");
}

/// A free body spinning at 50 rad/s about its COM, a 1 mm sphere 0.1 m off it, coming down on
/// the floor. The sphere moves up to omega r dt a step (83 mm at 60 Hz, 21 mm at 240 Hz).
/// - The sweep: whenever the sphere could reach the floor within the step, its contact exists
///   (with the collider's own half-diagonal as reach, 1.7 mm, the sweep covered 60 mm at 60 Hz
///   and missed it).
/// - At 240 Hz the first impact acts before the sphere is 1 mm deep. At 60 Hz (0.83 rad a
///   step) the speculative contact linearises the arc about the current point and the sphere
///   goes about a centimetre deep: a limit of linear speculative contacts under fast rotation,
///   offset or not (printed, not gated).
#[test]
fn a_spinning_offset_sphere_finds_the_floor_in_time() {
    for hz in [60.0f32, 240.0] {
        let spin = 50.0f32;
        let mut w = World::with_capacity(4, 4);
        w.set_gravity(Vec3::new(0.0, -9.81, 0.0));
        floor(&mut w, MaterialId::Concrete);
        let vel = Velocity { lin: Vec3::new(0.0, -0.5, 0.0), ang: Vec3::new(0.0, 0.0, spin) };
        let (b, _) = body(&mut w, iso(Vec3::new(0.0, 0.25, 0.0), Quat::IDENTITY), vel, Vec3::splat(0.02), 0.3);
        let r = 0.001f32;
        w.add_collider_at(b, iso(Vec3::new(0.1, 0.0, 0.0), Quat::IDENTITY), Shape::Sphere { r }, material(MaterialId::Rubber));
        let travel = spin * 0.1 / hz;
        // Up to the first impact (the spin changes) and one step after: the sphere is the body's
        // only collider, so after its first impact the scene says nothing more about the sweep
        let (mut deepest, mut after, mut gap) = (0.0f32, None, f32::MAX);
        for k in 0..(1.0 * hz) as usize {
            let reachable = gap < travel + 0.5 / hz;
            w.step(1.0 / hz);
            if reachable && after.is_none() {
                assert!(!w.last_frame_contacts.is_empty(), "{} Hz: the sphere was {:.1} mm up, within a step's reach, with no contact", hz, gap * 1e3);
            }
            let p = w.get_body_pose(b);
            let y = (p.pos + p.rot * Vec3::new(0.1, 0.0, 0.0)).y - r;
            gap = y.max(0.0);
            deepest = deepest.min(y);
            if after.is_none() && (w.get_body_vel(b).ang.z - spin).abs() > 1e-3 { after = Some(k); }
            if after.map_or(false, |a| k > a) { break; }
        }
        assert!(after.is_some(), "{} Hz: never touched", hz);
        println!("{} Hz ({:.2} rad a step): the first impact acted at most {:.2} mm deep", hz, spin / hz, -deepest * 1e3);
        if hz >= 240.0 { assert!(deepest > -0.001, "{} Hz: the sphere went {:.2} mm into the floor before its contact acted", hz, -deepest * 1e3); }
    }
}

/// A bar falling flat at 1 m/s onto one sphere offset 0.3 m from its COM (1 mm, at COM height,
/// so the lever arm is along x). Gravity off, no bounce (rubber on concrete, e 0). Right after the
/// impact its spin is the rigid-body point-impact closed form: J = v / (1/m + r^2 / I_zz),
/// omega_z = r J / I_zz.
#[test]
fn the_contact_lever_arm_is_the_offset() {
    let hz = 240.0f32;
    let mut w = World::with_capacity(4, 4);
    w.set_gravity(Vec3::ZERO);
    floor(&mut w, MaterialId::Concrete);
    let (half, m, rx, sr) = (Vec3::new(0.35, 0.02, 0.02), 1.0f32, 0.3f32, 0.001f32);
    let v0 = 1.0f32;
    let (b, mp) = body(&mut w, iso(Vec3::new(0.0, sr + 0.002, 0.0), Quat::IDENTITY), Velocity { lin: Vec3::new(0.0, -v0, 0.0), ang: Vec3::ZERO }, half, m);
    w.add_collider_at(b, iso(Vec3::new(rx, 0.0, 0.0), Quat::IDENTITY), Shape::Sphere { r: sr }, material(MaterialId::Rubber));
    let izz = mp.inertia.z_axis.z;
    let j = v0 / (1.0 / m + rx * rx / izz);
    let expect = rx * j / izz;
    // The impact can take two steps (a speculative contact, then the arrival): read the spin
    // once the contact point's normal velocity is 0
    let mut got = 0.0f32;
    for _ in 0..6 {
        w.step(1.0 / hz);
        let v = w.get_body_vel(b);
        got = v.ang.z;
        if (v.lin.y + v.ang.z * rx).abs() < 1e-3 && got != 0.0 { break; }
    }
    assert!((got / expect - 1.0).abs() < 0.02, "spin after the impact {:.4} rad/s, closed form {:.4}", got, expect);
}

/// A bar resting on two spheres offset 0.5 m either side of its COM stays level, with its COM
/// at the spheres' height above the floor.
#[test]
fn a_bar_rests_level_on_two_offset_spheres() {
    for hz in [60.0f32, 240.0] {
        let mut w = World::with_capacity(4, 4);
        w.set_gravity(Vec3::new(0.0, -9.81, 0.0));
        floor(&mut w, MaterialId::Concrete);
        let (sr, dy) = (0.01f32, 0.03f32);
        let (b, _) = body(&mut w, iso(Vec3::new(0.0, sr + dy + 0.005, 0.0), Quat::IDENTITY), Velocity::default(), Vec3::new(0.5, 0.02, 0.02), 2.0);
        for x in [-0.5f32, 0.5] {
            w.add_collider_at(b, iso(Vec3::new(x, -dy, 0.0), Quat::IDENTITY), Shape::Sphere { r: sr }, material(MaterialId::Rubber));
        }
        for _ in 0..(3.0 * hz) as usize { w.step(1.0 / hz); }
        let p = w.get_body_pose(b);
        let tilt = 2.0 * p.rot.z.atan2(p.rot.w);
        assert!(tilt.abs() < 1e-3, "{} Hz: tilted {:.2e} rad", hz, tilt);
        assert!((p.pos.y - (sr + dy)).abs() < 1e-3, "{} Hz: COM at {:.4} m, expected {:.4}", hz, p.pos.y, sr + dy);
    }
}

