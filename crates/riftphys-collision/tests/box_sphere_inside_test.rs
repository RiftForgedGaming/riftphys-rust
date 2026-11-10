//! A sphere whose centre is inside a box is pushed out through the nearest face at any depth
//! (PARITY_SPEC W4, the RiftPhys Reviewer's finding): before, the normal reversed once the
//! centre crossed the face, the penetration shrank with depth, and the contact vanished past
//! r + contact_dist.
use riftphys_collision::narrowphase::collide_shapes;
use riftphys_core::{Isometry, Quat, Vec3};
use riftphys_geom::Shape;

#[test]
fn a_sphere_centre_inside_a_box_is_pushed_out_through_the_nearest_face() {
    // A floor: half-height 0.5, top at y = 0, turned about y so the face isn't axis-trivial
    let floor = Isometry { pos: Vec3::new(0.0, -0.5, 0.0), rot: Quat::from_rotation_y(0.4) };
    let slab = Shape::Box { hx: 5.0, hy: 0.5, hz: 5.0 };
    let (r, cd) = (0.001f32, 0.003f32);
    let ball = Shape::Sphere { r };
    // Centre above the face (outside), then at depths r/2, r, 2r and 20r below it
    for depth in [-0.5 * r, 0.5 * r, r, 2.0 * r, 20.0 * r] {
        let at = Isometry { pos: Vec3::new(0.3, -depth, -0.2), rot: Quat::IDENTITY };
        let m = collide_shapes(&slab, &floor, &ball, &at, cd).unwrap_or_else(|| panic!("no contact at depth {}", depth));
        let p = m.points[0];
        assert!((m.normal - Vec3::Y).length() < 1e-5, "depth {}: normal {:?}, want +y (box to sphere)", depth, m.normal);
        let want = r + depth + cd;
        assert!((p.penetration - want).abs() < 1e-5, "depth {}: penetration {:.6}, want {:.6}", depth, p.penetration, want);
        // The other order flips the normal and keeps the depth
        let f = collide_shapes(&ball, &at, &slab, &floor, cd).expect("no contact, sphere first");
        assert!((f.normal + Vec3::Y).length() < 1e-5 && (f.points[0].penetration - want).abs() < 1e-5);
    }
}
