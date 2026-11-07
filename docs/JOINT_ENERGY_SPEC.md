# Joint energy loss: diagnosis and spec

Status: built 2026-11-06. SHAKE projection is the default (`JointProjection::Shake`); results in
section 5.

Failing test: `world_robotics_test::test_pvr_energy_conservation_pendulum` (97.57% retention,
needs over 98%). It also failed before the recent contact and D6 work.

Diagnostic: `crates/riftphys-world/tests/pendulum_energy_diag.rs` (ignored tests; run with
`cargo test -p riftphys-world --release --test pendulum_energy_diag -- --ignored --nocapture`).

## 1. Summary

- **The loss is real, and larger than the test shows.** A hinged 80 kg cube on a 5 m arm,
  released horizontal at 62.5 Hz, loses 9.7% of its swing energy in 4.8 s (about one period). The
  test reads 2.4% only because it measures potential energy from y = 0 with the pendulum 20 m up.
- **The cause is the joint position solve.**
  - Every step, the velocity solve makes the anchor velocity zero along the joint's rows at the
    start-of-step pose.
  - The position solve then pulls the drifted body back along rows and lever arms evaluated at
    the drifted pose, which is rotated by the angle the body turned this step.
  - These two impulses point in different directions, so the scheme is not symplectic. It bleeds
    energy at first order in dt, all the time, and never gives it back.
- **The fix is to correct along the start-of-step directions (SHAKE).** A point-mass model of the
  engine's scheme reproduces the engine's loss. Changing only the direction of the position
  correction makes it conserve energy as well as an exact integrator, with no secular drift.
- **Substepping is not the fix.** It shrinks the loss linearly, but at 960 Hz the result
  depends on where the pendulum hangs, 97.5% to 101.7%. Per-step corrections there are about
  10 µm, close to the 1 µm quantization grid and to f32 resolution 20 m up.
- **The test's metric needs fixing too:** measure swing energy, not energy above y = 0.

## 2. Measurements

### 2.1 The test's pendulum, traced

At 62.5 Hz, 20 velocity and 20 position iterations:

- **Steady loss:** swing energy falls by 0.95 J per tick at the turning points and by 1.6–1.9 J
  per tick through the bottom. It never rises.
- **Over one period:** 3922 J at release, 3708 J at the first turning point, 3543 J at 4.8 s.
- **Shorter swing:** the first swing reaches 86.8 degrees instead of 90.
- **Joint error is not the cause:** the anchor error peaks at 1.2 mm.
- **A visible symptom:** the bob's radial velocity reaches 0.149 m/s at the bottom. That
  matches v² dt / 2L, the outward component of the chord velocity
  (x_new − x_old) / dt on a circle. The next velocity solve removes it inelastically.

### 2.2 Step rate and iterations

Swing retention after 4.8 s; "ledger" is what the test measures.

| step rate | ledger | swing | worst anchor error |
|---|---|---|---|
| 62.5 Hz | 97.57% | 90.28% | 1.20 mm |
| 120 Hz | 98.67% | 94.69% | 0.33 mm |
| 240 Hz | 99.44% | 97.75% | 0.08 mm |
| 480 Hz | 99.51% | 98.04% | 0.04 mm |
| 960 Hz | 100.42% | 101.70% | 0.02 mm |

| iterations (velocity / position) | swing |
|---|---|
| 1 / 1 | 77.4% |
| 5 / 5 | 84.9% |
| 20 / 1 | 89.6% |
| 1 / 20 | 78.9% |
| 20 / 20 (default) | 90.3% |
| 50 / 50 | 91.2% |

- **Step rate:** the loss roughly halves with each halving of dt up to 240 Hz, the signature of
  a first-order scheme. At 960 Hz energy is gained instead (section 2.4).
- **Iterations:** past the default they buy about 1 point, so solver convergence is a minor
  part of the loss.

### 2.3 Point-mass model of the scheme

The model is a point mass in f64, with no quantization, released horizontal. Its length is
5.133 m, the cube's equivalent simple-pendulum length I_pivot / (m L). Schemes:

- **generalized:** angle coordinates, symplectic Euler. This is the reference.
- **engine:** each step does the following:
  - apply gravity;
  - project the velocity onto the tangent at x_n;
  - drift;
  - project the position back along the gradient at the drifted point;
  - add the correction / dt to the velocity.
- **SHAKE direction:** the same, but the position correction is along the gradient at x_n.
- **pure PBD:** engine without the velocity projection.

| step rate | generalized | engine | SHAKE direction | pure PBD |
|---|---|---|---|---|
| 62.5 Hz, 4.8 s | 100.82% | 90.92% | 100.83% | 74.63% |
| 120 Hz, 4.8 s | 100.43% | 95.04% | 100.43% | 85.36% |
| 240 Hz, 4.8 s | 100.21% | 97.45% | 100.21% | 92.23% |
| 62.5 Hz, 20 s | 99.74% | 70.61% | 99.76% | 36.39% |
| 240 Hz, 20 s | 99.91% | 90.32% | 99.92% | 73.63% |

- **The model reproduces the engine:** 90.9% against 90.3% at 62.5 Hz, and 97.5% against 97.8%
  at 240 Hz. The engine's remaining difference is solver convergence.
- **SHAKE-direction correction matches the exact reference** at every rate and over 20 s. Its
  departures from 100% are the bounded phase offset of symplectic Euler; they don't accumulate.
- **Why:** with the correction along the start-of-step gradient, the velocity impulse and the
  position correction combine into one impulse along G(x_n). That impulse is chosen so the body
  lands on the constraint, which is SHAKE, a symplectic integrator for constrained systems
  (Ryckaert, Ciccotti & Berendsen 1977; Andersen 1983 for RATTLE). The engine instead corrects
  along G(x̃), the gradient at the drifted point. That is standard PBD/XPBD, and its known
  dissipation is what we measure.

### 2.4 Precision floor at high step rates

Swing retention after 4.8 s, by the pivot's height:

| step rate | pivot at y = 0 | y = 20 | y = 200 |
|---|---|---|---|
| 62.5 Hz | 90.29% | 90.28% | 90.50% |
| 960 Hz | 97.50% | 101.70% | 98.67% |

- **At 62.5 Hz the loss is pure scheme:** height makes no difference.
- **At 960 Hz the result depends on height**, which means rounding.
  - The joint solve's per-step correction is about L·Δθ²/2, roughly 10 µm.
  - Positions are rounded to a 1 µm grid (`q6v` in `Bodies::apply_position_delta` and
    `set_pose`), and quaternion components to 1e-6.
  - At y = 200 the f32 spacing, 15 µm, is coarser than the grid.
- **This argues against substepping as the fix:** substeps push every correction toward this
  floor.

### 2.5 Steady spin without gravity (secondary)

A pendulum spinning at a steady rate with gravity off loses kinetic energy per tick at about 3
times the chord-geometry prediction at 9.6 m/s (9.2e-4 against φ²/3 = 3.1e-4). At 2 m/s it loses
17 times the prediction (2.3e-4 against 1.4e-5).

After the fix (SHAKE), the loss per tick at 5 and 9.6 m/s is zero within noise. At 0.5–3 m/s it
is 4e-5 to 2e-4 and does not rise with speed (1 m/s loses more than 3 m/s). That is rounding:
each step's correction there is a few µm, on the 1 µm grid (section 2.4), not the scheme.

## 3. Fix spec

### 3.1 Start-of-step joint Jacobians (SHAKE projection)

In `riftphys_articulation::bodies::solve_d6` (and `solve_distance_row`), evaluate the constraint
error at the current iterate, as now, but take the correction directions from the start-of-step
poses. Those are the poses the velocity solve used (`original_poses` in `World::step`, stage 4).

- **Linear rows:**
  - The axes `u` come from anchor A's frame at the start of the step.
  - The lever arms `xa - pa.pos` and `xb - pb.pos` come from the start-of-step poses, rotated
    anchors included.
  - The world inverse inertias `iaw` and `ibw` also come from the start-of-step rotation.
- **Angular rows:** same treatment, axes from the start-of-step frame. The error (`e_lock`,
  `e_world`) stays at the current iterate.
- **Effective mass:** keep k = J_n M⁻¹ J_nᵀ. This is the SHAKE iteration; it converges for steps
  in which the body turns well under 1 rad. Guard against a vanishing denominator.
- **Plumbing:**
  - `Joints::solve` takes a snapshot of start-of-step poses, one per body, or a closure. World
    already has `original_poses`.
  - The snapshot is built once per step, so it costs no more per iteration.
  - As built: `Joints::solve_shake(bodies, dt, iterations, start)`. `Joints::solve` and the
    public `World::solve_joints` keep PBD directions, because they also run outside `step` (the
    URIEL biped settles its pose with `solve_joints`).
- **Drives and limits:** position drives and limits use the same directions.
  - Limits are inequalities. Their speculative velocity rows already use start-of-step geometry,
    so this makes them consistent.
- **Contacts are out of scope.** Contact projection stays as it is: it is meant to dissipate.

### 3.2 Opt-in first, default after the suites

- **The flag:** `World::set_joint_projection(JointProjection::Shake | Pbd)`, default `Pbd`
  until the suites below pass. It takes a C ABI entry, like the other world settings.
  - As built: the C ABI exposes no joints at all, so it has no entry; it can be added with the
    first joint functions.
- **What it changes:** `step_hash` changes for every jointed scene once the default flips.
  There are no golden hashes in the tests (only `compliant_contact_test.rs` compares
  `step_hash` between two runs of the same scene), so nothing pinned breaks.
- **Make it the default when:**
  - the pendulum, chain and F1 arm tests pass;
  - the biped teacher keeps 20/20 at 12 N·s, and does no worse than 10/20 at 20 N·s;
  - the fidelity report is unchanged.

### 3.3 Test metric

Rewrite `test_pvr_energy_conservation_pendulum`:

- **Measure swing energy:** potential from the lowest point of the arc plus kinetic energy,
  computed from body state, not from the ledger's datum-dependent total.
- **Bound the whole run:**
  - over 5 s at 62.5 Hz, retention stays within [98%, 102%] at every tick;
  - the 2% allows for symplectic Euler's bounded phase offset, up to 0.8% here.
- **Add a 20 s run** that must stay within [97%, 103%]. It catches secular drift: today's scheme
  scores 70% there.
- **Pin the high-rate floor as a known limit:** at 960 Hz, retention within ±3% at any
  height, recorded as a precision bound rather than a target.

Also fix the ledger's rotational energy:

- **The bug:** `SystemEnergy.ke_ang` (step.rs, stage 7) divides by the diagonal of the world
  inverse inertia. That is wrong for a rotated body with unequal inertia.
- **The fix:** use ½ ωᵀ I_world ω, where I_world = inverse of `inv_inertia_world`, or
  R I_local Rᵀ.

### 3.4 New tests

In `crates/riftphys-world/tests/joint_energy_test.rs`:

1. **Single hinge:** swing retention bounds as in 3.3, at 62.5 and 240 Hz.
2. **Double pendulum (two hinged boxes):**
   - energy within ±3% over 10 s at 62.5 Hz;
   - joint separation under 2 mm.
   This is chaotic, so check energy, not the trajectory.
3. **Ball joint (D6 with free rotation):** a conical pendulum holds its cone angle within 1
   degree over 10 s.
4. **Limits still dissipate:** a hinge swinging into a limit loses energy at the stop, so
   inequality rows are not made elastic by accident.
5. **Determinism:** bit-exact replay with SHAKE on.
6. **Rate convergence:** the swing period at 62.5 Hz is within 1% of the RK4 reference
   (`reference_angle` in the diagnostic).

### 3.5 Risks

- **Tuned motion changes:** the biped teacher, the F1 arm's CartesianServo and the locomotion
  controllers were tuned with today's joint damping. Less numerical damping can expose
  oscillation. Run their suites with the flag on before flipping the default.
- **Large rotations per step:** SHAKE iteration converges slowly when a body turns a large
  fraction of a radian in one step, such as fast spinning wheels.
  - Fall back to current-iterate directions when the start-of-step and current lever arms differ
    by more than about 30 degrees.
  - Log the fallback count in `StepStats`.
- **Quantization:** at high step rates and far from the origin, 1 µm and f32 rounding still
  dominate (section 2.4). Out of scope here.
  - **Follow-up:** solve joints relative to the island's anchor, or keep sub-grid residuals.

## 4. Plan

1. Plumb start-of-step poses into `Joints::solve`. Add `JointProjection` with `Pbd` as the
   default (no behaviour change). Expect about half a day.
2. Implement SHAKE directions in `solve_d6` and `solve_distance_row`, behind the flag. Then run
   the diagnostic: the engine should match the point-mass SHAKE column within solver
   convergence.
3. Add the tests in 3.4, and rewrite the pendulum test and the ledger `ke_ang` as in 3.3.
4. Run the full workspace, the biped teacher at 12 and 20 N·s, the F1 seam suite and the
   fidelity report with the flag on. Flip the default if all hold.
5. Record the results in `PHYSICS_FIDELITY.md`.

## 5. Results (as built)

Code:
- `riftphys_articulation::bodies::{Joints::solve_shake, solve_d6, solve_distance_row}`
- `World::set_joint_projection`, with `JointProjection::Shake` the default
- in `World::step`, the start-of-step poses go to the joint solve

The large-rotation fallback (section 3.5) is in, at 0.5 rad per step. There's no fallback counter
in `StepStats` yet.

### 5.1 The test's pendulum

Swing energy after 4.8 s; "ledger" is the old metric.

| | PBD | SHAKE | point model (SHAKE) |
|---|---|---|---|
| 62.5 Hz, swing | 90.28% | 100.90% | 100.83% |
| 62.5 Hz, ledger | 97.57% | 100.23% | |
| 120 Hz, swing | 94.69% | 100.50% | 100.43% |
| 240 Hz, swing | 97.75% | 100.63% | 100.21% |
| 62.5 Hz, angle error at 4.8 s | -1.99° | +0.49° | |
| 62.5 Hz, 1/1 iterations, swing | 77.4% | 98.2% | |

The engine with SHAKE matches the point-mass SHAKE model. The period of the 90-degree swing is
within 0.2% of the RK4 reference.

### 5.2 Realistic links

These are 1 m × 0.05 m rods (2.5 kg) hinged at their ends, as in robot and ragdoll limbs.

| scenario | PBD | SHAKE |
|---|---|---|
| double rod pendulum, 10 s, energy range at 62.5 Hz | 0.41–1.00 | 0.979–1.044 |
| double rod pendulum, 10 s, energy range at 240 Hz | 0.55–1.00 | 0.990–1.014 |
| rod on a ball joint (spherical pendulum), 10 s at 62.5 Hz | 0.57–1.00 | 0.990–1.043 |
| rod on a ball joint, 10 s at 240 Hz | 0.82–1.00 | 0.991–1.012 |
| rod into a hinge limit past the bottom, energy after 5 s | 0.19 | 0.21 (peak 1.024) |

- **No drift:** the SHAKE ranges are bounded and don't trend; they rise and fall with the motion.
- **Vertical angular momentum** of the spherical pendulum, which gravity can't change, holds
  within 2%.
- **Limits still absorb the impact.**

### 5.3 Regression suites (PBD baseline → SHAKE default)

- **Workspace tests:** the only PBD failure (`test_pvr_energy_conservation_pendulum`) now passes,
  and nothing else changed. That test was rewritten to measure swing energy at every tick,
  within ±2%.
- **Fidelity report:** only the pendulum cases change.

  | case | PBD | SHAKE | tolerance |
  |---|---|---|---|
  | large swing, energy after 10 s, 60 Hz | 0.43 | 0.975 | ±0.02 |
  | large swing, energy after 10 s, 240 Hz | 0.77 | 0.993 | ±0.02 |
  | simple pendulum amplitude after 30 s, 60 Hz | 0.983 | 1.011 | ±0.02 |
  | hinged rod amplitude after 30 s, 60 Hz | 0.991 | 1.002 | ±0.02 |
  | hinged rod amplitude after 30 s, 240 Hz | 0.995 | 1.003 | ±0.02 |

  - **Large swing at 60 Hz:** still 0.5 points outside tolerance. It is a single instantaneous
    reading: over the 10 s the energy stays within 0.9725–1.029, with no trend. That is symplectic
    Euler's bounded offset, which is first order in dt; PBD declines steadily to 0.43.
  - **The 240 Hz guard:** `benchmarks_hold_at_240_hz` no longer lists large swing as a known gap.
- **Biped push recovery** (20 seeds, stood 10 s):

  | push | PBD | SHAKE |
  |---|---|---|
  | 0 N·s | 20/20 | 20/20 |
  | 6 N·s | 20/20 | 20/20 |
  | 12 N·s | 20/20 | 20/20 |
  | 20 N·s | 10/20 | 12/20 |
  | 30 N·s | 7/20 | 7/20 |

- **F1 seam teacher** (10 held-out episodes each): every case completed 10/10 in both modes. The
  lateral means:

  | seam | PBD | SHAKE |
  |---|---|---|
  | line butt | 0.110 mm | 0.106 mm |
  | line V | 0.053 mm | 0.050 mm |
  | line fillet | 0.055 mm | 0.054 mm |

  | ring butt | 0.100 mm | 0.101 mm |
  | ring V | 0.033 mm | 0.033 mm |
  | ring fillet | 0.034 mm | 0.034 mm |

### 5.4 New findings, not fixed here

- **Small bodies on long lever arms converge badly in Gauss-Seidel, in both modes.** The
  examples are a 0.1 m bob on a 1–2 m arm, or the 2 m cube on a 5 m arm hitting a stop.
  - **Why:** the bob's own rotational inertia is hundreds of times smaller than its inertia about
    the pivot. Angular rows (limits) and the anchor rows then fight across iterations.
  - **Examples:**
    - the cube on its 5 m arm, hitting a hinge limit at 9.6 m/s, gains energy: 1.3× under PBD,
      6× under SHAKE after the later impacts;
    - a double pendulum of 0.2 m cubes on 1 m arms lost 64% (PBD) or 65% (SHAKE);
    - a conical pendulum of a 0.1 m bob on a 2 m arm drifted 9.4° (PBD) or 3.1° (SHAKE).
  - **What it affects:** these are point-mass-on-massless-rod setups. Real links (rods, limbs)
    don't hit it.
  - **Fix options:**
    - solve each joint's rows as one block, using the 6×6 (or 3×3 plus 3×3) effective mass, so
      a limit feels the inertia about the pivot;
    - or more velocity iterations for such joints.
- **The hinge limit measure is not an angle.** Limits are compared with 2·sin(φ/2), not φ. The
  velocity-level speculative bound therefore allows 1 / cos(φ/2) times the remaining approach
  (2.3× at 128°). A limit set in radians also lands at the wrong angle (−1.8 is about −128°, not
  −103°). This is a units fix, separate from energy.
- **`D6Axis::lambda_acc` is never reset between steps.** For compliant rows (compliance > 0) the
  XPBD multiplier keeps growing across steps. Rigid rows (compliance 0) are unaffected, because
  alpha is 0.
- **Precision floor (section 2.4) applies to both modes.** Far from the origin at high step rates,
  energy is erratic either way. At 240 Hz, 200 m up: PBD 113%, SHAKE 115%; at 1000 m: PBD 84%,
  SHAKE 111%. At 62.5 Hz, 1000 m up, SHAKE reads 105%; PBD's 10% loss hides the same error.
