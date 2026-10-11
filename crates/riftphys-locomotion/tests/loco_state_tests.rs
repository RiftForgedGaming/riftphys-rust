use riftphys_locomotion::loco_state::{FootClock, Phase, q6, sec_to_us};

#[test]
fn test_q6_quantization() {
    let val1 = 1.123456789;
    assert_eq!(q6(val1), 1.123457);

    let val2 = -0.0000001;
    assert_eq!(q6(val2), 0.0);

    let val3 = 5.500000;
    assert_eq!(q6(val3), 5.500000);

    assert_eq!(q6(f32::INFINITY), 0.0);
    assert_eq!(q6(f32::NAN), 0.0);
}

#[test]
fn test_sec_to_us_conversion() {
    assert_eq!(sec_to_us(1.0), 1_000_000);
    assert_eq!(sec_to_us(0.000001), 1);
    assert_eq!(sec_to_us(0.5), 500_000);
    assert_eq!(sec_to_us(-5.0), 0);
}

#[test]
fn test_footclock_phase_transitions() {
    let mut clock = FootClock::new(0.6, 0.4);

    // Step 1: Start of stance (0.0s to 0.5s)
    let (phase1, entered_stance1) = clock.step_gate(0.5);
    // FIX: The clock spawns at t=0, which is already inside the stance phase.
    // Moving to 0.5 stays inside stance; it does not cross the boundary.
    assert!(!entered_stance1, "Should not 'enter' stance; it was already in stance at t=0");
    match phase1 {
        Phase::Stance(q) => assert_eq!(q, q6(0.5 / 0.6)),
        _ => panic!("Expected Stance phase"),
    }

    // Step 2: Cross into swing (0.5s + 0.2s = 0.7s)
    let (phase2, entered_stance2) = clock.step_gate(0.2);
    assert!(!entered_stance2, "Should not enter stance, moved to swing");
    match phase2 {
        Phase::Swing(q) => assert_eq!(q, q6(0.1 / 0.4)),
        _ => panic!("Expected Swing phase"),
    }

    // Step 3: Wrap around back to stance (0.7s + 0.4s = 1.1s -> wrapped to 0.1s)
    let (phase3, entered_stance3) = clock.step_gate(0.4);
    assert!(entered_stance3, "Should enter stance upon wrapping the cycle");
    match phase3 {
        Phase::Stance(q) => assert_eq!(q, q6(0.1 / 0.6)),
        _ => panic!("Expected Stance phase after wrap"),
    }
}
#[test]
fn test_degenerate_gait_spec() {
    let mut clock = FootClock::new(0.0, 0.0);
    let (phase, entered) = clock.step_gate(1.0);

    assert!(!entered);
    match phase {
        Phase::Stance(q) => assert_eq!(q, 0.0),
        _ => panic!("Degenerate clock must return Stance(0.0)"),
    }
}