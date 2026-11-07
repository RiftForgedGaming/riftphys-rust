# Precision: the 1e-6 state grid, f32 positions, and slow motion

Status: proposed 2026-11-07. Diagnosis done, including a probe with the grid off across every
suite (section 2.6). The RiftPhys Reviewer reviewed it the same day; this revision folds that in.
P1 and P2 built the same afternoon (section 3.5); P3 and P4 not started. This replaces JOINT_CONDITIONING_SPEC
C6, which covered only the multiplier and compliance; the 240 Hz sprung-hinge offset left by C2
traced to the grid on body state, which C6 didn't cover.

Diagnostics: `crates/riftphys-world/tests/precision_diag.rs` (`slow_motion_dead_zone`,
`spring_resolution`, `rest_creep`) and `joint_conditioning_diag.rs`'s `sprung_hinge_sag_offset`.
All are ignored tests; run them with
`cargo test -p riftphys-world --release --test precision_diag -- --ignored --nocapture`.

## 1. Summary

- **RiftPhys rounds body state to a 1e-6 grid on every write.**
  - The grid covers positions (1 µm), velocities (1e-6 m/s and rad/s) and quaternion
    components (1e-6).
  - It is applied in `Bodies::set_pose`, `set_vel`, `apply_position_delta`,
    `apply_orientation_delta`, `apply_impulse`, `apply_angular_impulse` and `integrate_all`.
  - The joint solver rounds every intermediate value the same way: λ, its increments, errors,
    alpha and compliance.
- **The grid makes a dead zone.**
  - A body whose motion per step is under half a grid step doesn't move at all, even with a
    velocity. Motion near that size snaps to whole grid steps.
  - The dead zone scales with the step rate: about 1e-6 × rate in m/s, and 2e-6 × rate in rad/s.
- **The dead zone is the main precision floor near the origin.**
  - It causes the 240 Hz sprung-hinge offset (0.87%).
  - It makes light springs at high rates wrong (40% at 960 Hz for 0.1 kg).
  - It turns compliance below 5e-7 rigid, and rounds 1.4e-6 to 1e-6.
- **Turning the grid off in body state and joints (a probe) removes most of it.**
  - The probe removes the rotation dead zone entirely, and fixes light springs at 240 Hz.
  - What's left is f32 itself: absolute positions at 3–10 m from the origin resolve only about
    0.24–0.95 µm, which is the same order as the grid.
  - A further threshold in `step.rs` skips any spin under 3.2e-5 rad/s.
- **The grid doesn't make RiftPhys deterministic across platforms on its own.**
  - Rust never fuses multiply-adds implicitly.
  - But 70 calls to the platform's math library (`atan2`, `acos`, `sin`, …) can differ by an ulp
    between Windows and Linux. The grid only delays the divergence that causes.
  - RiftForged's determinism harness measured exactly that on its quantised tier (section 2.5).
  - Same-machine replay doesn't need the grid; floating-point arithmetic is deterministic for one
    binary.

## 2. Diagnosis

### 2.1 Where the grid is applied

- **Ten crates each define their own `q6`:** dynamics, articulation, collision, core (`det.rs`),
  aero, acceleration, gravity, io, locomotion and vehicles. Terrain uses core's.
- **Body state, in `riftphys-dynamics/src/dynamics.rs`:** every write rounds position, velocity
  and quaternion components, including the joint solver's corrections and the integrator.
- **The joint solver, in `riftphys-articulation/src/bodies.rs`:** rounds every intermediate value:
  - axes and lever arms;
  - errors;
  - λ and Δλ;
  - alpha = q6(q6(c) / dt²);
  - the deltas it applies.
- **The step hash** (`provenance.rs`) hashes raw f32 bits. Because state sits on the grid, those
  bits are grid values today.

### 2.2 The dead zone (at HEAD)

A free 0.2 m box, no gravity, no contacts, kept awake, given a slow spin or drift. The table
gives the turn or distance after 10 s, as a ratio to the expected value; 0 means it never moved.

| rate | quantity | 1e-5 | 1e-4 | 3e-4 | 1e-3 | 3e-3 | 1e-2 |
|---|---|---|---|---|---|---|---|
| 60 Hz | spin (rad/s) | 0 | 1.20 | 0.98 | 0.96 | 1.00 | 1.00 |
| 60 Hz | drift (m/s) | 0 | 1.20 | 1.00 | 1.02 | 1.00 | 1.00 |
| 240 Hz | spin | 0 | 0 | 1.60 | 0.96 | 0.96 | 1.01 |
| 240 Hz | drift | 0 | 0.01 | 0.80 | 0.96 | 1.00 | 1.01 |
| 960 Hz | spin | 0 | 0 | 0 | 1.92 | 1.28 | 0.96 |
| 960 Hz | drift | 0 | 0 | 0 | 0.96 | 0.96 | 1.05 |

- **Below the dead zone, nothing moves.**
- **Near it, motion snaps to whole grid steps per tick:** 1.2×, 1.6× and 1.9× too fast, or 4%
  slow.
- **The workloads that run at high rates feel it most:**
  - URIEL's arm and biped at 240 Hz;
  - Cassie's dataset at 2000 Hz (PARITY_SPEC W4), where the dead zone would be 2e-3 m/s and
    4e-3 rad/s.
- **A rigid joint at rest doesn't creep** (`rest_creep`: 0 m in 60 s at 60–960 Hz). The grid adds
  no drift of its own.

### 2.3 Springs (at HEAD)

A weight on a compliant distance joint, started at its static stretch c·m·g and kept awake. The
table gives the stretch after 10 s as a ratio to static, with the worst departure in brackets.

| mass, compliance | 60 Hz | 240 Hz | 960 Hz |
|---|---|---|---|
| 2 kg, 5e-4 | 1.0000 | 0.9999 | 0.9624 (6.0%) |
| 0.1 kg, 5e-4 | 0.9994 (1.0%) | 0.9819 (4.0%) | 1.3999 (41.7%) |
| 1 kg, 5e-4 | 1.0001 | 1.0011 | 0.9859 (2.1%) |
| 1 kg, 1e-5 | 1.0013 | 1.0013 | 0.9916 (1.8%) |
| 1 kg, 1e-6 | 1.0694 (6.9%) | 1.0694 | 1.0694 |
| 1 kg, 1.4e-6 | 0.7638 (23.6%) | 0.7638 | 0.7638 |
| 1 kg, 4e-7 | 0 (rigid) | 0 | 0 |
| 1 kg, 1e-7 | 0 (rigid) | 0 | 0 |

- **Compliance below 5e-7 rounds to 0, so the joint is rigid.** That is stiffer than 2e6 N/m:
  cables, steel, Cassie's leaf springs.
- **1.4e-6 rounds to 1e-6 (40% stiffer),** and a 10 µm stretch sits on the 1 µm grid (7%).
- **Light bodies and high rates fail,** because the per-step corrections fall below the grid.

The sprung hinge (C2's 1 rad sag, kept awake) drifts in the first 0.5 s, then freezes exactly:
- 240 Hz: 1.0081 (SHAKE) or 1.0009 (PBD) of static, frozen from 5 s to 10 s;
- 480 Hz: 1.0011 frozen;
- 60 and 120 Hz: exact.

### 2.4 The probe: grid off in body state and joints

`q6` turned into the identity (keeping its non-finite → 0 sanitation) in `dynamics.rs` and
`bodies.rs` only. The probe is a throwaway, not committed.

| | HEAD | probe |
|---|---|---|
| spin, 240 Hz, 1e-4 rad/s | 0 | 1.000 |
| spin, 960 Hz, 3e-4 rad/s | 0 | 1.000 |
| spin, any rate, 1e-5 rad/s | 0 | 0 (the `step.rs` threshold, below) |
| drift, 240 Hz, 1e-4 m/s | 0.01 | 1.14 |
| drift, 960 Hz, 3e-4 m/s | 0 | 0.76 |
| spring 0.1 kg, 5e-4, 240 Hz | 0.9819 | 0.9994 |
| spring 0.1 kg, 5e-4, 960 Hz | 1.3999 | 1.0305 |
| spring 1 kg, 1.4e-6, 60 Hz | 0.7638 | 0.9721 |
| spring 1 kg, 1e-7, 60 Hz | 0 (rigid) | 0.9721 |
| sprung hinge 1 rad, SHAKE 240 Hz, 5–10 s | 1.0081 (frozen) | 0.9965–1.0011 (moving) |

What's left after the probe:
- **Rotation is clean** above 3.2e-5 rad/s.
- **`step.rs:177` skips rotating a body whose |ω|² ≤ 1e-9,** so spins under 3.2e-5 rad/s never
  turn, with or without the grid.
- **Slow drift is limited by f32 positions.**
  - The test bodies sit 3–5 m from the origin, where f32 spacing is 0.24–0.48 µm. At 960 Hz,
    1e-4 m/s is 0.1 µm per step, below that spacing, so it's lost.
  - The grid was 1 µm; f32 there is 2–4× finer. Removing the grid gains that factor, not more.
- **Stiff springs settle 2.8% off,** whatever their compliance from 1e-7 to 1.4e-6.
  - Their 1–14 µm stretch is measured at 9 m altitude, where f32 spacing is 0.95 µm.
  - The ratio no longer depends on the compliance, so the spring law is right; the position
    can't represent it.
- **960 Hz springs sit 3% off at every mass.** Per-step corrections there are below the f32
  spacing at 9 m.
- **Far from the origin** (JOINT_ENERGY_SPEC 2.4): at 200 m, f32 spacing is 15 µm, coarser than
  the grid. The probe doesn't help there, and it isn't meant to.

### 2.5 What the grid does for determinism

- **Same machine, same binary:** f32 arithmetic is deterministic, and Rust never fuses
  multiply-adds implicitly. So bit-exact replay (`shake_replays_bit_exactly`, URIEL's
  `rollouts_replay_bit_exactly`) doesn't need the grid. The probe's suite run checks this
  (section 2.6).
- **Across platforms:**
  - `glam` is built with its `libm` feature, so glam's own transcendental functions are portable.
  - The 70 direct `f32::atan2/acos/asin/sin/cos/exp/powf/ln/tan` calls in non-test code use each
    platform's math library, which can differ by an ulp between the MSVC runtime and glibc.
  - A grid only re-synchronises differences that land on the same side of a grid boundary; one
    that crosses a boundary diverges anyway, later.
  - RiftForged's determinism harness (`Specs/DeterminismHarness_Spec.md`, 2026-10-20) measured
    this in its C++ server. Its quantised gameplay tier held until tick 1750 under stock flags,
    then diverged. Raw physics diverged at tick 130. FMA contraction was the cause there; Rust
    has no such contraction, but the math library differences remain.
- **CPU architecture matters as well as the OS.**
  - Windows and Linux on x86_64 take the same SSE2 paths.
  - x86 vs ARM (NEON) can sum dot products in a different order in glam's SIMD code.
- **Some of this already exists:**
  - `riftphys-core` has a `deterministic` feature (glam `scalar-math`, blake3 `pure`), which
    removes the SIMD ordering difference;
  - there are `glam-fast-math` and `glam-core-simd` tripwire features.
- **The direct calls:**
  - The Reviewer counts 70, including `hypot` and `powi` (4 in `solver.rs`).
  - `powi` is an LLVM intrinsic, not a libm call; it goes through `powf` or explicit multiplies.
- **What cross-platform determinism actually needs, whether or not the grid stays:**
  - route those 70 calls through a portable implementation (the `libm` crate, which `glam`
    already uses);
  - turn on the `deterministic` feature across x86 and ARM.

  That is decision D2.

### 2.6 The probe across the regression suites

The same probe across the regression suites (2026-11-07, 09:55–10:25 EST), against f329733:

| suite | f329733 | probe (grid off) |
|---|---|---|
| workspace tests | 134 pass | 133 pass, 1 fails (below) |
| bit-exact replay (`shake_replays_bit_exactly`, `rollouts_replay_bit_exactly`) | pass | pass |
| fidelity: simple pendulum amplitude after 30 s, 60 Hz | 1.0122 | 1.0001 |
| fidelity: physical pendulum amplitude after 30 s, 240 Hz | 1.0016 | 1.0000 |
| fidelity: torque-free rotation, energy and momentum change, 240 Hz | 0.0003 / 0.0001 | 0.0000 / 0.0000 |
| fidelity: large swing, 60 Hz | 0.9754 (FAIL) | 0.9749 (FAIL) |
| fidelity: restitution, 60 Hz | 0.7234 (FAIL) | 0.7234 (FAIL) |
| fidelity: everything else | ok | ok, within 0.2% of before |
| biped push recovery at 20 N·s | 12/20 | 17/20 |
| other pushes (0, 6, 12, 30 N·s) | 20, 20, 20, 7 of 20 | 20, 20, 20, 7 of 20 |
| F1 seams, line: teacher's mean height error (butt, v, fillet) | 0.007 / 0.074 / 0.060 mm | 0.004 / 0.074 / 0.059 mm |
| F1 seams, ring: teacher's mean height error (butt) | 0.006 mm | 0.002 mm |
| F1 seams: completion counts | | identical |

How to read it:
- **Nothing got worse, and several things got better.**
  - The biped stands 17/20 instead of 12/20 at 20 N·s. This is unverified: so large a jump from
    ulp-level changes suggests the 20 N·s case sits near a chaotic threshold. Before it's cited as
    a precision gain, rerun both arms with the push shifted one tick and scaled ±1% (P1's
    acceptance).
  - Pendulums keep their amplitude.
  - Torque-free rotation becomes exact to four places.
  - Seam height errors fall by up to two thirds.
- **The two fidelity failures are unchanged.** They come from the integration scheme (R5) and
  restitution timing (W6), not from precision.
- **Determinism:** the bit-exact replay tests pass without the grid, so same-machine replay
  doesn't depend on it.
  - But both tests are small: 1–2 bodies, and 3 drones. Neither reaches `PAR_MIN` = 64, so
    neither runs the parallel paths. P1 adds a test that does.
- **The probe isn't P1.**
  - It took the grid off `dynamics.rs` and `bodies.rs` only.
  - Contact geometry (`narrowphase.rs`) kept its 1 µm rounding, which also acts as a deadband
    for bodies at rest.
  - P1 reruns every suite with the grid off everywhere it goes.
- **The one failure is C2's rule for inferring the hinge axis from the rows,** in
  `sprung_hinge_stays_aligned_past_90_degrees`: alignment rows at c = 1e-9 tilt 33° at 60 Hz.
  - The rule counted "compliance that rounds to zero on the grid" as rigid. With no grid, 1e-9
    is soft, the rule finds no hinge, and the past-90° divergence returns.
  - P1 has to replace that rule (below).

## 3. Work items

### P1. Take the grid off body state and the joint solver

- **Remove the rounding** where it rounds a quantity: body state, joints, contact geometry,
  forces.
- **Keep the non-finite sanitation** (NaN/∞ → 0 or identity) in every crate whose `q6` goes, not
  only `dynamics.rs` and `bodies.rs`, and keep quaternion normalisation.
  - Collision's `q6` (`narrowphase.rs:32-37`) is also its NaN guard.
  - `step.rs:431` sorts contacts with `partial_cmp(...).unwrap()`, which panics on a NaN depth.
  - So keep the guard there, and switch float sorts to `total_cmp`.
- **Distance and D6 compliance** are used as given. alpha = c / dt² isn't rounded, so any
  positive compliance is soft.
- **Replace C2's rule for inferring the hinge axis from the rows.** It treats a compliance that
  rounds to zero on the grid as rigid; without the grid, a hinge's alignment rows at c = 1e-9
  are no longer rigid, and the joint has no hinge (the probe's one failure).
  - Proposed rule, when `hinge_axis` is unset:
    - **First clause:** the hinge is the one rotational axis that is free, limited or driven.
    - **If that doesn't find exactly one axis:** the hinge is the one enabled row with compliance
      c > 0 that is at least 100× the compliance of each other enabled rotational row. A sprung
      hinge with stiff alignment rows qualifies: 0.15 against 1e-3 or 1e-9. Requiring c > 0 stops
      a joint with three rigid rows from qualifying on all three.
    - **Otherwise** there is no hinge.
  - **What it covers:** the RiftPhys Reviewer's case of soft alignment rows (1e-3) without
    `hinge_axis`.
  - **What it changes:** a universal joint with a spring on its twist becomes a hinge about that
    axis. Its disabled rows are skipped and the twist is in radians, so this is harmless.
  - **What it leaves:**
    - A bushing whose compliance ratio sits near 100× switches between measures. That only shows
      at large angles; document it.
    - An isotropic soft bushing still has no hinge and diverges past 90°. That's the separate
      ticket on soft rows' missing velocity rows.
- **The other crates' `q6`** are audited one by one: collision, aero, acceleration
  (`propulsion.rs`, 22 uses), gravity, vehicles, locomotion, io, terrain and core. There are
  three kinds:
  - **Rounding a quantity** (contact geometry, forces, state): it goes.
    - The Reviewer classified collision, aero, propulsion and locomotion site by site.
    - It checked gravity (26 uses) and vehicles (27) by count only. The audit classifies those
      site by site too, so a sort key or load-time rounding in them isn't removed by default.
  - **Rounding inputs on load:** it stays. Harmless, and it makes loading stable. Examples are
    `rig_physics.rs:29` (poses) and `terrain.rs:31-47` (heights and cell sizes).
  - **Float sort keys,** which sort values that sit on the grid today. With the grid off, ulp
    noise decides them. That is still deterministic on one machine, but chaotic: golden hashes
    churn, and any cross-platform ulp difference is amplified.
    - **The sites:**
      - `narrowphase.rs:251` sorts contact candidates by depth, then x, and keeps the first 4;
      - `step.rs:431` sorts by depth, then truncates to 4;
      - `step.rs:377` and `solver.rs:547` order by height;
      - `solver.rs:86` orders by an `atan2` angle.
    - Today a box lying flat has its corners at exactly equal depths, so the sort's secondary
      order decides which 4 contacts survive.
    - **The fix:** quantise only the key (sort by q6(depth), store the raw depth), or compare
      within a tolerance and fall back to an index.
- **The step hash:** keep hashing raw bits (same-machine replay), or hash rounded values
  (tolerant of ulp differences). That is decision D3.
- **A parallel replay test:**
  - 100 or more boxes piled up, which runs the parallel paths above `PAR_MIN`;
  - run in two separate processes with different `RAYON_NUM_THREADS`;
  - the hashes must match bit for bit.
  - Structurally this should already hold: pairs are sorted (`spatial_hash.rs:45`), contact
    buckets are a `BTreeMap` (`step.rs:422`), rayon's collect keeps order, and solver batches
    write disjoint bodies (`solver.rs:341-399`). But no test proves it.
- **Golden hashes change for every scene.** URIEL re-records at its argref bump.

Acceptance:
- **Dead zone:** spin and drift at 1e-4 to 1e-2 (rad/s or m/s), at 60, 240 and 960 Hz, each
  within 1% of expected.
  - For drift, the bodies sit within 1 m of the origin.
  - Drift whose motion per step is under about 50 ulp of the body's position is excepted until
    P3. f32 rounds each position update by up to 0.5 ulp, the same way every step at constant
    velocity, so the error is up to 0.5 ulp / (v·dt); 1% needs 50 ulp per step. (Amended when
    built: the first draft said 4 ulp, which allows 12.5%.) At 3–5 m, the spacing is
    2.4–4.8e-7 m, so 1e-4 m/s at 240 Hz (4.2e-7 m per step) snaps; the probe read 1.14.
- **Springs:**
  - 0.1–2 kg at compliance 1e-5 to 5e-4 hold static stretch within 1% at 60 and 240 Hz, and
    within 4% at 960 Hz (P3 tightens that).
  - A compliance of 1e-7 is soft, not rigid.
- **Sprung hinge 1 rad:** within 0.5% at 240 Hz, kept awake. Tighten C2's 1.5% gate to match.
- **`sprung_hinge_stays_aligned_past_90_degrees`** passes with alignment rows at 0, 1e-9 and
  1e-3, all three with `hinge_axis` unset. That replaces today's 1e-3 case, which sets it.
- **At rest:** with the grid off everywhere and bodies kept awake for 60 s, all at 60 and 240 Hz:
  - a box held by friction on a 20° slope (μ above tan 20°) creeps less than 0.1 mm;
  - a 10-box stack stays standing, its top box moving less than 1 mm;
  - the biped standing still with no push doesn't fall, and its pelvis drifts less than 1 mm;
  - `rest_creep` reads less than 1 µm.
  - **Each case is also measured at HEAD, with the grid on, and both are reported.** If a case
    already creeps under the grid (from the friction model, say), the gate measures that rather
    than P1. A case that fails at HEAD too is reported as an existing gap, not a P1 regression.
- **Determinism:**
  - `shake_replays_bit_exactly` and `rollouts_replay_bit_exactly` pass;
  - the new parallel replay matches across two processes with different `RAYON_NUM_THREADS`.
- **Regression:** workspace tests, fidelity, biped push recovery and F1 seams, rerun on the full
  P1, not the probe, with any change explained.
- **Biped:** the 20 N·s result is also run with the push shifted one tick and scaled ±1%, with the
  grid and without it. If the result swings by more than a few seeds either way, report it as
  sensitivity, not a gain.

### P2. The spin threshold in `step.rs`

- `step.rs:177` rotates a body only when |ω|² > 1e-9.
- Lower it to a true zero test (`length_squared() > 0.0`, or `EPS_ANG2` = 1e-24 as
  `Bodies::integrate_rot` already uses).
- Acceptance: a spin of 1e-5 rad/s turns within 1% of expected at 60–960 Hz.

### P3. Position precision beyond f32 (only if a workload needs it)

f32 positions resolve 0.24 µm at 2 m, 0.95 µm at 10 m and 15 µm at 200 m. Options:
- **(a) Compensated positions.** Store each body's position as a pair of f32s: value plus
  residual, accumulated with an error-free sum.
  - Slow motion then accumulates instead of being lost.
  - Joint and contact solvers take position differences as (a.hi − b.hi) + (a.lo − b.lo), which
    is exact for nearby bodies even far from the origin.
  - Costs: two more floats per body, a helper for differences, and the residual in the hash.
- **(b) f64 positions** for body state and joint anchors, f32 for everything else.
  - Simplest to reason about.
  - The largest change: `Isometry` is f32 throughout, and every reader of `pose.pos` is
    affected.
- **(c) Island-local origins.** Each island is solved relative to its first body's position.
  This fixes far from the origin but not the 0.24–0.95 µm floor near it.

Recommendation: decide after P1 and W4 (Cassie at 2000 Hz) show whether the f32 floor matters for
a real workload. URIEL's arm and biped work within 2 m of the origin at 240 Hz, where f32 resolves
0.24 µm and 1e-4 m/s moves 0.42 µm per step.

### P4. Portable math (only if decision D2 wants cross-platform determinism)

- Route every `f32` transcendental call in simulation code through the `libm` crate, as `glam`
  already does.
- Add a lint or test that fails on new direct calls.
- Acceptance: the same scene hashes identically on Windows (MSVC) and Linux (GNU) for 10,000
  steps. That needs a Linux runner; the RiftForged determinism harness has one.

### 3.5 Results: P1 and P2 as built (2026-11-07)

**What was built:**
- **Rounding removed** from body state (`dynamics.rs`), the joint solver (`bodies.rs`), contact
  geometry (`narrowphase.rs`), aero, propulsion, the gravity field's result, vehicles' per-step
  quantities (ground samples, forces, velocities, dt), locomotion's outputs (foot targets, heading,
  step lengths) and terrain sampling. Each crate's `q6` became `fin` (non-finite → 0) or was kept.
- **Rounding kept** where it is an input or a hash:
  - gravity field parameters, also hashed by `spec_id`;
  - vehicle throttle, steer, mass hint and cast length;
  - locomotion's integer-microsecond clock and plan inputs;
  - terrain heights and cell sizes on load;
  - `rig_physics.rs` poses on load;
  - material pair coefficients (`materials.rs`, not in the audit list; 1e-6 on a μ of 0.5);
  - the balance controller's own configurable `quantize`.
- **Sort keys:** `riftphys_core::det::sort_key6` (value in whole 1e-6 units, NaN last) at
  `step.rs:377` and `:431`, `solver.rs:86` and `:547`. `step.rs:431` no longer panics on a NaN
  depth.
- **Contact reduction (not in the spec, found by the regression run).**
  - `narrowphase.rs` kept the 4 candidates that were deepest, ties broken by lowest x.
  - On a box lying flat, its 8 corner candidates tie on depth, so all 4 contacts landed on its −x
    edge.
  - With the grid, sub-µm depth differences mostly decided the order instead, and the deeper (sinking)
    side won. With `sort_key6` the tie decided every step, the bias was systematic, and
    `stacked_boxes_hold` failed: box 1 drifted 2.03 cm in 4 s against a 2 cm gate.
  - Now `reduce_to_4` keeps the deepest point, the one farthest from it, the one farthest from
    that line, and the one farthest on the other side: 4 points spanning the patch. Ties go to the
    lowest index.
- **Hinge rule:** as proposed in P1 above (`d6_hinge_axis`).
- **P2:** `step.rs:177` tests `> 0.0`.
- **New tests:**
  - `parallel_replay_test.rs`: 200 boxes in two 10 x 10 layers, hashed in child processes at 1, 8
    and 3 rayon threads.
    - It asserts at least 64 pairs (parallel narrowphase) and a contact batch of at least 64
      (parallel contact solve), read from `World::last_max_batch`.
    - The parallel joint path (64 or more joints in one level) is not covered.
  - `precision_diag.rs` `at_rest` (slope, 3/5/10-box stacks);
  - riftphys-uriel `standing_still_drift` and `push_sensitivity_20` (ignored).

**Acceptance:**

| item | gate | result |
|---|---|---|
| spin 1e-5 to 1e-2 rad/s, 60/240/960 Hz | within 1% | all 1.000 (P2: 1e-5 now turns) |
| drift at 0.7 m, 60 and 240 Hz, 1e-4 to 1e-2 m/s | within 1% | 1.000–1.001 |
| drift at 0.7 m, 960 Hz | within 1%, under 50 ulp/step excepted | 1e-2: 1.001; 3e-3: 0.992; excepted: 1e-3 (17.5 ulp/step): 0.973, 3e-4 (5.2): 0.954, 1e-4 (1.7): 1.144 |
| springs 0.1–2 kg, c 1e-5 to 5e-4, 60/240 Hz | within 1% | worst 0.13% |
| same at 960 Hz | within 4% | worst 3.48% |
| c = 1e-7 | soft | 0.9721 of static (was rigid); the 2.8% is f32 at 9 m |
| sprung hinge 1 rad, 240 Hz | within 0.5% | 0.35% (SHAKE, 5–10 s range 0.9965–1.0011); test gate tightened to 0.5% |
| past 90° with alignment rows 0, 1e-9, 1e-3, `hinge_axis` unset | tilt < 1°, gap < 1 mm | pass, 60 and 240 Hz |
| determinism | replay tests + parallel replay | pass (parallel narrowphase and contact solve; not the joint path) |
| workspace tests | pass | 135 pass (134 + parallel replay) |

**Drift at 960 Hz is off by 2.7% at 1e-3 m/s and 4.6% at 3e-4 m/s, inside the amended exception.**
- That is f32 rounding each position update the same way every step: up to half an ulp per
  step, systematic for a constant velocity.
- 1% needs about 50 ulp of motion per step. The "4 ulp" exception above was too loose by an
  order of magnitude; at 0.5–1 m (ulp 6e-8 m), 1e-3 m/s at 960 Hz is 17 ulp per step and
  reads 2.7% slow.
- Only P3 (a) or (b) fixes this. It matters for W4 (Cassie at 2000 Hz) if slow drift does: see D1.

**At rest (60 s, kept awake; stack boxes 0.4 x 0.2 x 0.4 m, 5 kg, started 1 cm apart):**

| case | HEAD, grid on | P1 |
|---|---|---|
| box on a 20° slope (wood on concrete, μs 0.66) | 0 | 3.6e-9 m (60 Hz), 0 (240 Hz) |
| `rest_creep` | 0 | 0 |
| 3-box stack, 60 Hz / 240 Hz | falls / falls | falls / stands, top moved 3.4 cm |
| 5-box and 10-box stacks | fall | fall |
| biped standing, no push | stands; pelvis drifts 2.04 m | stands; pelvis drifts 2.46 m |

- **Stacks and the standing biped fail with the grid on too.** They are existing gaps, not P1
  regressions.
  - A 3-box stack kept awake creeps sideways and falls after about 15–25 s at HEAD.
    `stacked_boxes_hold` runs only 4 s.
  - The standing biped never falls but wanders about 2 m in a minute under `StandPd`.
- **The RiftPhys Reviewer's stack probe** (3 equal boxes started 2 mm apart, 30 s, time to cross
  the 2 cm gate):
  - P1: 5.03 s at 60 Hz, where the top box moves 7 cm, tilts 0.9° and doesn't topple; 7.25 s at
    240 Hz.
  - HEAD: 1.78 s at 60 Hz, then topples; 8.37 s at 240 Hz.
  - One box on the ground, or on an equal static box, holds within 3e-8 m. A 2-stack drifts under
    1 cm in 30 s either way.
  - The drift direction varies, so it isn't a fixed tie-break bias.
- **Both want their own tickets:** stack creep in the contact solver, and the biped's drift in
  its stand controller.
  - The stack ticket should gate over 30–60 s; `stacked_boxes_hold` runs 4 s and passes with about
    1 s to spare at 60 Hz.
  - **Box-box narrowphase gaps for that ticket (at HEAD too):**
    - There is no face clipping. Two boxes yawed so that no corner of either lies inside the
      other (grown by the contact distance) get a single edge-edge point at the centre. A 3-stack
      whose middle box is yawed 45° topples in 1.6–2.6 s; even at 3° yaw it topples at 60 Hz.
    - The grow (contact distance + 1e-4, about 5 cm with the CCD margin) accepts corners up to
      that far outside the other box's face, so contacts can sit over empty space and hold an
      overhanging box up. `reduce_to_4` picks the outermost candidates, so it prefers those.
    - The fix: clip the incident face against the reference face (Sutherland–Hodgman), then run
      `reduce_to_4` on the clipped polygon.
- **Also fixed:** the balance controller's `quantize` = 0 (the natural "off") divided by zero and
  produced NaN; it now skips quantizing when `quantize` isn't positive.
- **Left alone:** `materials.rs` still rounds μ per contact (`mu_dynamic`), about 2e-6 relative,
  with nothing accumulating. Its `quantize` has no NaN guard; μ inputs are finite constants.

**Regression:**
- **Fidelity:** 38 ok and 2 FAIL, as before; both failures are unchanged (large swing 60 Hz
  0.9749, restitution).
  - Torque-free rotation becomes exact to four places.
  - The 60 Hz simple pendulum's amplitude goes from 1.0122 to 1.0001.
  - At 240 Hz the simple pendulum's period error goes from 0.05% to 0.23% and its amplitude from
    1.0003 to 0.9986, as in the probe. Both are within their gates.
- **Biped push recovery:** 20, 20, 20, 17, 7 of 20 at 0, 6, 12, 20 and 30 N·s (HEAD 12/20 at 20).
- **Sensitivity at 20 N·s,** as drawn, one tick late, ×0.99 and ×1.01:
  - HEAD: 12, 12, 12, 11 of 20;
  - P1: 17, 17, 17, 17.

  So the gain is not a chaotic threshold; it holds for each variation.
- **F1 seams:** completion counts identical. Teacher height errors match the probe:
  - line butt 0.007 → 0.004 mm;
  - ring butt 0.006 → 0.002 mm;
  - the others within 0.001 mm.

## 4. Order and reporting

1. **P1 and P2 together:** about 1–2 days, mostly auditing ten crates' `q6`, plus the regression
   suites. Heavy runs finish before 11 PM EST.
2. **P3:** only if W4 or another workload needs it. (a) is about 2 days; (b) about a week.
3. **P4:** only on D2 = yes. About 1 day, plus the Linux run.

Each commit is named to URIEL Orchestration for the argref bump, and every spec and commit goes to
the RiftPhys Reviewer.

## 5. Decisions for the Designer

- **D1: does the parity report need high step rates?**
  - The dead zone and the f32 floor matter at 240 Hz and up, and most at Cassie's 2000 Hz.
  - After P1, slow drift at 960 Hz is still 2.7% slow at 1e-3 m/s near the origin (section 3.5);
    at 2000 Hz it would be worse.
  - If W4 is in the report, P1 and P2 are needed and P3 may be.
- **D2: is cross-platform determinism a RiftPhys requirement,** across OSes (Windows, Linux) and
  CPU architectures (x86, ARM)?
  - If yes: P4. The grid can still go, because it never guaranteed this.
  - If no: same-machine replay is all that's promised; say so in the docs.
- **D3: what the step hash covers.**
  - (a) Raw bits: exact, for same-machine replay. Recommended with D2 = no, or with P4.
    - Raw bits also change on any refactor that reorders an addition, which the grid mostly
      absorbed. URIEL would re-record golden hashes more often.
  - (b) Values rounded to 1e-6 before hashing: tolerant of ulp differences, but it can still
    differ at grid boundaries.
- **D4: far from the origin.** Will any RiftPhys scene run more than about 100 m from the origin
  (RiftForged-size worlds)? If yes, P3 (a) or (c).
