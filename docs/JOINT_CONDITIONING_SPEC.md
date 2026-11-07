# Joint conditioning: stale joint errors, drives and lever arms

Status: proposed 2026-11-07. C1 and C2 built 2026-11-07 (section 3.7); C3 to C6 not started. The RiftPhys Reviewer reviewed it on
2026-11-07 and approved the direction; this revision folds in that review. It covers:
- the third open joint issue (PARITY_SPEC W5, JOINT_ENERGY_SPEC 5.4);
- the hinge position drive that stops short of its target;
- the RiftPhys Reviewer's findings on 1aa0aeb and c0ef751.

Diagnostics: `crates/riftphys-world/tests/joint_conditioning_diag.rs`. All its tests are
ignored; run them with
`cargo test -p riftphys-world --release --test joint_conditioning_diag -- --ignored --nocapture`.

## 1. Summary

- **The root cause is a measuring bug, not conditioning.**
  - `solve_d6` measures the anchor gap `d`, the lever arms and the world inertias once per call.
  - It then solves the three linear rows against those stale values: row y doesn't see what
    row x just moved.
  - The measurement axes are stale too: they come from body A's rotation at call start.
  - Within a joint, this is a Jacobi sweep posing as Gauss-Seidel.
  - With a lever arm, the rows overcorrect together. The iteration then falls into a two-step
    cycle between two wrong states, with the joint pulled apart in both.
- **This one bug explains both open problems:**
  - the hinge position drive stalls at 24° with its anchor 34 cm apart;
  - most of the lever-arm failures, including every SHAKE blow-up.
- **Re-measuring after each row fixes it.** In a throwaway prototype:
  - the drive reaches its target to 0.01°;
  - SHAKE's double pendulum went from a 1,757× energy gain to 1.03×;
  - all joint test suites still pass.
- **What's left after that fix:**
  - slow convergence for small bodies on long arms (the real W5 conditioning problem);
  - drives that ignore kp, kd and max_impulse;
  - the Reviewer's findings (section 2.5).

## 2. Diagnosis

### 2.1 The stale joint error

Where it is, in `crates/riftphys-articulation/src/bodies.rs`, `solve_d6`:
- `let d = q6v(xb - xa)` is computed once, before the linear-row loop.
- `axes` (the measurement axes `u`, from body A's rotation) is computed once, at call start.
  In PBD mode the correction directions `dir_axes` are those same stale axes.
- `lever_a`, `lever_b`, `iaw` and `ibw` (PBD path) come from the poses at call start.
- The angular rows share one `q_err`, `e_world` and `e_lock`, computed after the linear loop.
  So angular row y doesn't see row x's correction either.

Each row's correction both translates and rotates the body. With the pivot at a rod's end,
about 75% of a linear row's correction is rotation. That rotation moves the anchor along the
other rows' axes, but those rows act on the gap measured before it, so they correct again.

**The evidence:** the 1 m, 2.5 kg rod with a hinge drive to 1 rad, no gravity. In the table,
`solve_joints` runs alone with n iterations, starting at 0°. "Apart" is the anchor separation.

| hinge at | n = 1 | 2 | 5 | 20 | 21 | 100 | 101 | 1000 | 1001 |
|---|---|---|---|---|---|---|---|---|---|
| rod's end (lever 0.5 m) | 53.1° | 0.5° | 85.1° | 23.7° | 98.1° | 24.0°, apart 0.338 m | 98.2°, apart 0.477 m | 24.0° | 98.2° |
| rod's centre (lever 0) | 53.1° | 57.29° | 57.30° | 57.30° | | 57.30° | | 57.30° | |
| end, re-measured per row (prototype) | 53.1° | 12.7° | 56.6° | 49.2° | | 57.27° | | 57.29°, apart 0.00002 m | |

What the table shows:
- **It is a two-step cycle, not a resting state.**
  - After an even number of iterations the rod sits at 24.0° with the joint 0.338 m apart; after
    an odd number, at 98.2° with it 0.477 m apart.
  - That is the sign of a Jacobi sweep with an eigenvalue at −1.
  - The result doesn't depend on gravity or on SHAKE vs PBD, only on whether the count is even
    or odd. The RiftPhys Reviewer caught that; every count first tested was even.
- **Inside a world step it depends on the count too.**
  - With 20 position iterations (even, the default), both modes and both gravities sit at
    23.99°, apart 0.3375 m.
  - With 21 iterations a step lands at 98.2° at 0.1 s and 1 s, back at 24.0° at 5 s.
- **Each iteration alternates between overshoot and undershoot.** The tracked sequence for a
  1 rad target is 53° → 0.5° → 69° → 1.3° → 85° → 14° → 94° → 19° → … → 24°.
- **Where it ends up after an even count depends on the target:**

  | target | after an even count | joint apart by |
  |---|---|---|
  | 0.05 rad | 2.86° (correct) | 0 |
  | 0.2 rad | 1.5° | 9.5 cm |
  | 0.5 rad | −10.7° | 44 cm |
  | 2 rad | 78.7° | 38 cm |

- **Without a lever arm the rows don't couple,** so the hinge at the centre is exact.

### 2.2 The lever-arm cases, before and after re-measuring

These are the cases from JOINT_ENERGY_SPEC 5.4, rerun at HEAD with the prototype. Energy is
the min–max of the ratio to the starting energy over the run; "apart" is the worst anchor
separation.

| case | mode, rate, position iterations | HEAD | re-measured |
|---|---|---|---|
| 2 m cube on 5 m arm into a −1.8 rad stop: peak energy | SHAKE 240 Hz, 20 | 23.4×, apart 0.90 m | 1.001×, apart 0.022 m |
| same | SHAKE 62.5 Hz, 20 | 1.39×, apart 0.96 m | 1.003×, apart 0.082 m |
| same | PBD 240 Hz, 20 | 2.77×, apart 0.37 m | 1.000×, apart 0.021 m |
| double pendulum, 0.2 m cubes on 1 m arms, 10 s | SHAKE 62.5 Hz, 20 | 0.13–6.2, apart 1.78 m | 0.94–1.04, apart 0.046 m |
| same | SHAKE 240 Hz, 20 | 0.95–1.01 | 1.00–1.03 |
| same | SHAKE 240 Hz, 100 | 0.62–1,757 | 0.95–1.46 |
| same | PBD 62.5 Hz, 20 | 0.34–1.00 | 0.35–1.00 (PBD's own loss) |
| conical pendulum, 0.1 m bob on 2 m arm, 30° cone, 10 s | SHAKE 62.5 Hz, 20 | 27.0–30.0° | 18.7–30.0° |
| same | SHAKE 62.5 Hz, 500 | 19.9–34.9° | 29.9–31.0° |
| same | SHAKE 240 Hz, 20 | 21.6–41.7° | 29.3–31.0° |

How to read it:
- **The blow-ups were the stale error, not conditioning.** These are the 23× limit impact, the
  double pendulum's 6× to 1,757×, and the 1.8 m anchor separations. All of them are gone.
- **PBD's double pendulum still loses 65% at 62.5 Hz.** That's PBD's first-order loss, which
  SHAKE (the default) removes.
- **The cube into the stop ends at 3% of its energy.** A rigid stop is a perfectly inelastic
  impact, as in PhysX and Bullet, so that is expected.

### 2.3 What re-measuring leaves (the real W5)

- **Slow convergence for small bodies on long arms.**
  - A row's correction rotates the bob about its own centre, using its tiny inertia.
  - The anchor rows then hand most of that rotation back.
  - Examples:
    - the drive needs about 100 iterations to land within 0.03° when solved alone. In a world
      step it gets there within 0.1 s because each step starts from the last;
    - the conical pendulum at 62.5 Hz with 20 iterations narrows to 18.7°, and needs 500 to
      hold 30° within 1°.
  - This is the block solve PARITY_SPEC W5 describes.
- **More iterations make SHAKE's double pendulum worse at 240 Hz.**
  - With 20 iterations it stays in 1.00–1.03; with 100, 0.95–1.46; with 500, 0.91–1.40. PBD
    shows the same trend (1.01 → 1.16 → 1.31).
  - **The cause is not found.** The likeliest suspect, from the Reviewer, is the same mechanism
    in a weaker form: some mode of the in-step iteration doesn't converge.
    - In SHAKE, the error is measured at the current pose while corrections run along fixed
      start-of-step directions. That is a quasi-Newton iteration.
    - Its iteration matrix, I − J_cur M⁻¹ J₀ᵀ / (J₀ M⁻¹ J₀ᵀ), can have eigenvalues below −1 when
      the chain turns within the step, and lever arms make that worse (R7).
    - A mode near −1 makes the end state depend on where the iteration stops. A mode slightly
      larger than 1 in size grows with the count.
  - **Checks, in order** (C5 does them before building anything):
    1. Run 20 vs 21 and 100 vs 101 iterations. A split between even and odd counts means an
       oscillating mode.
    2. Log the worst anchor gap and angle error after each iteration, for a few steps at 100
       iterations. Does it fall steadily, cycle, or fall then grow?
    3. Split each step's energy change into the velocity-solve part and the position-solve
       plus stage-6 part.
    4. Rebuild the double pendulum as point masses on distance joints (no lever arms, no
       rotation rows). If the trend vanishes, it's D6 row coupling. If it stays, it's how
       velocity is derived from the position correction.
    5. Rule out rounding with C6 option A (no grid) in one run.
  - **Two second-order effects in stage 6** (`step.rs`) are worth keeping in mind. They make
    "SHAKE is symplectic" only approximately true, though they are unlikely to depend on the
    iteration count:
    - ω is derived as 2·dq.xyz, i.e. 2·sin(θ/2);
    - `apply_orientation_delta` applies a small-angle quaternion.

### 2.4 Drives ignore kp, kd and max_impulse

- **A drive and a limit on the same row share one error and one multiplier.** The target is
  clamped into the limit. That is harmless while the drive is rigid, but see C3.
- **Stage 6 drops angular corrections for velocity-driven bodies.**
  - The "FIX 2: MOTOR OVERWRITE" block in `step.rs` skips the whole orientation correction for
    any body on a joint with a `vel_mode` drive. That includes locked rows, limits and contact
    pushes, so the body's ω never sees its position corrections.
  - The block also loops over every joint for every body each step (O(bodies × joints)).
- **The position solve treats a position drive as a rigid constraint to its target.**
  - It doesn't read `kp`, `kd` or `max_impulse`; only the row's `compliance` makes it soft.
  - With re-measuring, a 1 rad target is reached in one tick: peak |ω| 230 rad/s.
  - That isn't a drive in the PhysX or MuJoCo sense (a spring-damper with a force cap).
- **The velocity solve skips position-drive rows,** so they have no velocity-level part.
- **Velocity drives (`vel_mode`) do use `max_impulse`, but not `kd`** (a cap on a motor row).
- **Nothing in URIEL uses position drives today;** the arm and biped apply torques.
  - The Designer's continuous 360° arm joints would.
  - So would any servo-style control, such as Cassie's or a gripper's.

### 2.5 The RiftPhys Reviewer's findings (2026-11-07)

On 1aa0aeb:
- **R1 (medium, before the parity report): a sprung hinge isn't a hinge.**
  - `rigid()` ignores compliance, so a hinge whose axis has a torsional spring has no free axis,
    and `d6_hinge_axis` returns `None`.
  - Two consequences:
    - the spring acts on 2·sin(θ/2), not θ: 4% soft at 1 rad and 16% at 2 rad;
    - the other two rows fall back to the twist-mixed error, which diverges past about 90° of
      twist.
  - `compliant_joint_test` sags only 0.1 rad, and its expected value builds in the old law, so
    it can't see this.
- **R2 (low–medium): q6 rounding of λ and of compliance.**
  - λ = F·dt² on the 1e-6 grid resolves a 0.1 kg body at 240 Hz to about 6%, and a 1 kg body at
    960 Hz to about 10%.
  - `q6(compliance)` turns anything under 5e-7 (stiffer than 2e6 N/m) rigid, and makes 1.4e-6
    40% stiffer.
- **R3 (low): angles wrap at ±π, drive and limit errors don't.**
  - A drive to 3.1 rad from −3.1 rad turns the long way, and targets beyond π can't be reached.
  - A hinge thrown past π on a [−2.2, 0.1] limit wraps and is pushed to the wrong stop.
- **R4 (low, separate ticket): sleeping partners.**
  - Bodies sleep one at a time, and nothing wakes a jointed partner.
  - An awake body's joint moves a sleeping one without velocity.
  - Sleeping or waking a whole island (islands are already built from joints) closes it.

On c0ef751:
- **R5 (medium, before the parity report): the 60 Hz large swing (0.975, range 0.973–1.029) is
  outside its own ±0.02 tolerance,** and no test checks it at 60 Hz. See decision D4.
- **R6 (low–medium): the SHAKE fallback checks the whole body's rotation,** including free spin
  about the hinge axis.
  - A hinge spinning faster than 0.5 rad per step falls back to PBD and loses energy silently:
    30 rad/s at 60 Hz, 120 rad/s at 240 Hz. That covers a spindle, a fast wrist or a flywheel.
  - Spin about the hinge axis doesn't turn frame A's axes or a lever arm along the axis.
- **R7 (low): D6 rows measure along `u` and correct along `un`,** without the 1/(un·u) rate
  factor that the distance row uses. Near the 0.5 rad cutoff they under-correct by up to 12%
  per iteration; the fixed point is unchanged.
- **R8 (nit, separate ticket): the ledger drops a body's whole rotational KE** when one axis
  has zero inverse inertia.

## 3. Work items

### C1. Re-measure after every row (the root-cause fix)

What to refresh, in `solve_d6`:
- **Before each linear row:**
  - the measurement axes `axes` (u), from body A's current rotation;
  - `xa`, `xb` and `d`;
  - in PBD mode, also `dir_axes`, `lever_a`, `lever_b`, `iaw` and `ibw`.
- **Before each angular row:**
  - `axes` (u);
  - `q_err`, `e_world`, `e_lock` and `hinge_angle`;
  - in PBD mode, also `dir_axes`, `iaw` and `ibw`.
- **SHAKE:** the correction directions (`dir_axes` = un, lever arms, inertias) stay at the start
  of the step. Only what is measured (u and the errors) is refreshed, which is what SHAKE calls
  for anyway.

What not to change:
- **The SHAKE fallback filter** (`turned(...) < SHAKE_MAX_TURN`) stays evaluated once per call.
  That's simpler and deterministic, and C4 changes what it measures.
- **`solve_distance_row`:** it is one row and already reads the poses fresh on each call.
- **The velocity rows** (`d6_rows` in `solver.rs`) are built once per velocity solve. That is
  correct for projected Gauss-Seidel on velocity, which is linear in v, so leave them.

Acceptance:
- **Drive:** this test pins the drive to the rigid meaning (kp = 0 under D1a). Until C3 lands,
  drives ignore their gains, so it would pass with any kp. Once C3 lands, kp = 100 is a real
  spring (peak at 0.34 s, settled to 2% by 0.67 s), so C3 replaces this test.
  - The end-hinged rod with a 1 rad drive reaches 57.3° ±0.1° within 0.1 s in a world step, with
    the anchor apart less than 1 mm.
  - Check it at 60 and 240 Hz, both modes, with and without gravity, with 20 and 21 position
    iterations.
  - Solved alone, 1000 and 1001 iterations both give 57.3° ±0.1°.
- **Cube on a 5 m arm into a −1.8 rad stop:** peak energy at most 1.01 at 62.5 and 240 Hz,
  anchor apart less than 0.1 m at 62.5 Hz and 0.03 m at 240 Hz.
- **Double pendulum of 0.2 m cubes, SHAKE, 20 iterations:** energy in 0.93–1.05 at 62.5 Hz and
  0.98–1.04 at 240 Hz. This is an interim gate, looser than parity's ±0.02; don't quote it as
  parity. C5 tightens it.
- **All existing suites:** workspace tests, the fidelity checks, F1 seam tracking (line and
  ring), and biped push recovery. Report any change from 11/20 at 20 N·s.
- **Cost:** measure per-joint solve time before and after. Re-measuring adds a pose read and a
  quaternion product per row.

This changes every joint scene's step hash, so URIEL re-records its golden hashes at the
argref bump it already plans.

### C2. A sprung hinge is a hinge (R1)

- `d6_hinge_axis` and the `rigid` closure count a row with `compliance > 0` as not rigid.
- `compliant_joint_test` derives its expected sag from c·m·g·(L/2)·cos θ = θ, in radians.
- Add a large deflection: c ≈ 0.08, sag about 1 rad, within 3%. Also add the same test at 60 Hz.

Do this before the parity report: it claims compliant joints are right.

### C3. Drives as spring-dampers (2.4, R3)

A position drive on a row becomes an XPBD spring-damper:
- **Spring:** compliance 1/kp, so alpha = 1/(kp·dt²).
- **Damper:** kd as XPBD damping (Macklin et al. 2016, eq. 26), on the row's rate of change
  since the start of the step.
  - In `World::solve_joints` (outside `step`) there is no start of step, so damping is off there.
- **A pure damper (kp = 0, kd > 0) can't be a compliance;** alpha would be infinite.
  - MuJoCo and PhysX allow damping-only drives, used for velocity control.
  - Make it a velocity row with target 0 and an effective mass from kd.
  - D1 settles what kp = 0 means.
- **Force cap:** `max_impulse` today is per step, so the force it allows is max_impulse/dt: 4×
  more at 240 Hz than at 60 Hz. Velocity motors already behave this way.
  - Give the cap a meaning that doesn't depend on the step rate: a `max_force`, with the position
    multiplier capped at |λ_acc| ≤ max_force·dt² and the motor row at max_force·dt.
  - Or document that the cap is per step. D1 includes this choice.
- **A drive and a limit on the same row get separate multipliers.**
  - Today one error and one `lambda_acc` serve both, with the target clamped into the limit.
  - Once the drive is soft and capped, a capped drive pushing into a stop would cap the stop too,
    and the joint would pass through it.
  - The limit keeps its own rigid multiplier.
- **Wrapping and the unwrapped angle:**
  - Keep an unwrapped running angle per row, so continuous joints work. Compute limits on it
    too, so a hinge thrown past π stays on the right side.
  - **Each position drive has a mode** (decision D3):
    - **shortest path:** the error is wrapped into (−π, π] and the target is taken mod 2π.
      Default for a row with no limit.
    - **absolute:** the error is on the unwrapped angle, so multi-turn targets (7 rad) are
      reached. Default for a row with a limit.
  - One rule can't do both: with a wrapped error, a 7 rad target from 0 stops at 7 − 2π ≈ 0.72
    rad.
  - **When it updates:** once per step, after the position solve, as
    prev + wrap(θ_now − wrap(prev)), not once per iteration.
  - **How fast it can turn:** that update needs |Δθ| < π per step, about 188 rad/s at 60 Hz.
    Document the limit, or integrate the angle from ω for faster spindles.
  - **Where it is set:** from the measured pose in `add_joint`; re-sync it on `set_body_pose` and
    on teleports.
  - **What carries it:** it is part of snapshots and the step hash.
  - **Who reads it:** `d6_row_angle` (the speculative limit row in the velocity solve) reads the
    unwrapped angle too. It then needs the joint's state passed in, or the position and velocity
    solves' measures diverge again.
- **Velocity drives:**
  - narrow or remove stage 6's "MOTOR OVERWRITE" (2.4), so a velocity-driven body's ω sees its
    locked-row, limit and contact corrections;
  - then either give velocity drives kd (target velocity with a damping gain), or document that
    `vel_mode` is a pure constraint motor and kd is unused there.

Acceptance:
- **Step response:** the end-hinged test rod (I = mL²/3 ≈ 0.833 kg·m²), gravity off, kp = 100,
  kd = 10, driven from 0 to 1 rad. That gives ω₀ = 10.95 rad/s, ζ = 0.548 and a damped frequency
  ω_d = 9.16 rad/s. At 60 and 240 Hz, each of these is within 5% of its analytic value:
  - peak at t = π/ω_d = 0.343 s;
  - overshoot exp(−ζπ/√(1 − ζ²)) = 12.7%;
  - settling to 2% at 4/(ζω₀) = 0.67 s.
- **Gravity off:** gravity adds a torque of m·g·(L/2)·cos θ, which shifts the steady state.
- **Cap:** under the chosen meaning, the same drive delivers the same peak torque at 60 and
  240 Hz.
- **Drive into a limit:** a capped drive pushing into a stop never passes it by more than the
  limit's own tolerance.
- **Wrapping:**
  - In shortest-path mode, a drive from 3.0 to −3.0 rad turns 0.28 rad, not 6 rad.
  - In absolute mode, a target of 7 rad is reached from 0.
- **Replay:** the unwrapped angle survives snapshot → restore and replays bit-exactly.
- **Velocity-driven bodies:** a velocity-driven hinge against a limit stops at the limit with no
  leftover spin.
- **Diagnostics:** `drive_iteration_trace` in `joint_conditioning_diag.rs` calls `solve_joints`
  once per iteration, and each call resets the multipliers. After C3 the drive row is compliant,
  so the trace needs access to `Joints`, or should be dropped.

### C4. SHAKE fallback by correction directions (R6, R7)

- Decide the fallback by how far the correction directions turned (frame-A axes, `lever_a`,
  `lever_b`), not the whole body's rotation.
- Count fallbacks in `StepStats`.
- Apply the 1/(un·u) factor in D6 rows as the distance row does, or note why not.

Acceptance:
- A flywheel on a hinge at 60 rad/s, 60 Hz, keeps its energy within 1% over 10 s with no
  fallbacks.
- The existing SHAKE tests are unchanged.

### C5. W5 block solve (only what C1 leaves)

- **Solve each D6 joint's rows as one block.** In order of preference:
  - the 6×6 effective mass J M⁻¹ Jᵀ, by Cholesky factorization;
  - 3×3 linear and 3×3 angular blocks coupled through the lever arm.
- **Why it helps:** a correction then rotates a bob about the pivot rather than its own centre.
- **Investigate first** why more iterations raise the double pendulum's energy (2.3), because a
  block solve converges further per iteration and could make that worse.

Acceptance:
- **Conical pendulum:** 0.1 m bob on a 2 m arm holds a 30° cone within 1° for 10 s at 60 Hz with
  20 iterations.
- **Double pendulum:** 0.2 m cubes, energy 0.98–1.02 at 62.5 and 240 Hz. No worse with 100
  iterations than with 20, and no split between even and odd counts (20 vs 21).
- **Drive alone:** converges within 0.1° in 10 iterations.
- **Cost:** per-joint solve time no more than twice C1's.

### C6. Precision of λ and compliance (R2)

**Superseded by docs/PRECISION_SPEC.md (2026-11-07).** The grid on body state turned out to be the
larger floor; that spec covers this item and more.

Option A: keep λ and alpha off the 1e-6 grid.
- They are internal to the step. Determinism needs identical operations, not a grid: f32 is
  bit-reproducible on one target as long as there's no fast-math or FMA contraction.
- Whether the grid exists for cross-platform reasons is decision D2.

The grid doesn't give cross-platform determinism by itself:
- `atan2`, `acos` and `sin` come from each platform's libm and can differ by an ulp between
  Windows and Linux.
- A value near a grid boundary then rounds differently on each.
- The "Determinism harness: Windows vs Linux" session on this machine measures exactly this, and
  its result should decide D2.

Option B: if the grid must stay, quantize relative to scale.
- Round λ relative to m·g·dt²-sized values, and compliance on a log grid.

Acceptance:
- A 0.1 kg spring at 240 Hz and a 1 kg spring at 960 Hz hold their static stretch within 1%.
- A compliance of 1e-7 is not rigid.
- The replay is bit-exact.

### Separate tickets (not in this spec's work)

- **Rig files can't set `hinge_axis`.**
  - `riftphys-io`'s D6 builder always sets `None`; only `JointDef::Hinge` stores the axis.
  - So a rig can't express a sprung hinge whose alignment rows are soft. It falls back to the
    derived rule, which finds no hinge, and the past-90° divergence returns.
  - Fix: an optional `hinge_axis` on the rig's D6 definition, when rig files need it.
  - Found by the Reviewer, 2026-11-07.

- **Soft rows have no velocity rows** (`solver.rs` skips `compliance > 0`).
  - Found by the Reviewer on 2026-11-07, and present at HEAD too.
  - A free hinge with soft alignment rows (c = 1e-3) holds at rest and from a twisted start. But
    spinning at 20–45 rad/s with a sideways knock, it explodes at 60 Hz in both modes. At
    240 Hz and 30 rad/s it holds but tilts 21°.
  - A soft weld (all three rotational rows at 1e-3) started 2.5 rad round explodes at both rates.
  - Suspected cause: soft rows act only at position level. Give them compliant velocity rows,
    or treat them like PhysX's soft limits.

- R4 island sleep.
- R8 ledger rotational KE.
- Make `Joints::solve` and `solve_shake` call `begin_step` themselves, or document that the
  caller must. The Reviewer flagged both as public and unguarded.

### 3.7 Results: C1 and C2 (built 2026-11-07)

**C1.** `solve_d6` re-measures every row from the bodies' current poses before solving it.
- **Both kinds of row:** the axes u and the errors.
- **Linear rows:** the anchor gap.
- **Angular rows:** the angle (q_err, the a_f × b_f alignment, the hinge twist).
- **PBD mode:** also the correction directions, lever arms and inertias.
- **SHAKE:** keeps its correction directions from the start of the step; the fallback test still
  runs once per call.

Re-measuring the axes as well as the gap makes convergence much faster than in the prototype of
2.1:

| drive to 1 rad, solved alone | n = 2 | 5 | 20 | 21 | 100 | 1000 | 1001 |
|---|---|---|---|---|---|---|---|
| HEAD | 0.5° | 85.1° | 23.7° | 98.1° | 24.0° | 24.0° | 98.2° |
| prototype (gap only) | 12.7° | 56.6° | 49.2° | | 57.27° | 57.29° | |
| C1 | 54.6° | 57.10° | 57.30°, apart 0.26 mm | 57.30° | 57.30° | 57.30° | 57.30° |

Inside a world step the drive is at 57.30° from the first tick, at 20 and 21 position
iterations, in both modes, with and without gravity. The joint stays together to 0.1 mm.

**C1 acceptance** (`crates/riftphys-world/tests/joint_conditioning_test.rs`, 4 tests, all pass):

| test | gate | measured |
|---|---|---|
| hinge drive in a world step: 60/240 Hz, PBD/SHAKE, g on/off, 20/21 iterations | 57.3° ±0.1° after 0.1 s, joint apart < 1 mm | 57.30°, 0.0000 m |
| drive solved alone, 1000 and 1001 iterations | 57.3° ±0.1° | 57.30° both |
| cube on 5 m arm into a −1.8 rad stop: peak energy | ≤ 1.01 | PBD 1.000; SHAKE 1.003 at 62.5 Hz, 1.001 at 240 Hz |
| same: worst joint gap | < 0.1 m at 62.5 Hz, < 0.03 m at 240 Hz | 0.052 m / 0.014 m (SHAKE) |
| double pendulum of 0.2 m cubes, SHAKE, 20 iterations (interim gate) | 0.93–1.05 at 62.5 Hz, 0.98–1.04 at 240 Hz | 0.938–1.040, 0.998–1.027 |

The other lever-arm numbers match section 2.2's prototype column:
- The conical pendulum at 62.5 Hz with 20 iterations still narrows to 18.7°.
- The double pendulum still gains energy with more iterations at 240 Hz (100 iterations:
  0.95–1.46).
- Both stay with C5 and the checks in 2.3.

**C2.** A sprung hinge is a hinge, so its spring acts on the twist angle in radians, and its
alignment rows measure only the hinge axis's misalignment.
- `Generic6Dof` has a new field, `hinge_axis: Option<usize>`.
  - `add_hinge` sets it. `add_ball`, `add_universal` and rig files built through
    `riftphys-io` leave it `None`.
  - With it set, the hinge is that axis whatever the other rows' compliance.
  - `set_d6_limit_r` or `set_d6_drive_r` on another rotational axis clears it, and the axis is
    derived from the rows again.
- Unset, `d6_hinge_axis` derives the axis from the rows:
  - **First rule:** a row is rigid only with no limit, no drive and a compliance that is zero on
    the solver's 1e-6 grid (so c = 1e-9 counts as rigid). Exactly one non-rigid axis is the hinge.
  - **Failing that:** compliant rows count as locked, so a hinge whose two alignment rows are
    soft is still a hinge about its free axis.
- **What can't be derived:** a sprung hinge whose alignment rows are also compliant looks like a
  soft weld from its rows, and needs `hinge_axis` set. The RiftPhys Reviewer found that case:
  with only the derived rule, x at c = 1e-3 tilted 34° at 60 Hz, and c = 1e-9 blew up.
- **Effect on existing scenes:** every one built with `add_hinge` gets the axis the old rule
  derived, so they behave the same. The biped's push-recovery sweep is identical; 134 workspace
  tests pass.

`compliant_joint_test` changes:
- **Expected sag:** now from θ = c·m·g·(L/2)·cos θ, in radians.
- **Sleep off:** the spring tests keep their body awake. A sleeping body freezes where it is,
  which hid a 240 Hz offset; the Reviewer caught it.
- **`torsional_spring_large_sag_is_in_radians`** (new): c = 0.15 gives a sag of about 1 rad.
  - Measured with C2: 0.0004% at 60 Hz, 0.87% at 240 Hz. Without C2: 3.3% and 4.1%.
  - Gates: 0.25% at 60 Hz, 1.5% at 240 Hz.
  - The 240 Hz offset isn't the spring law, so it gets the looser gate until C6:
    - it doesn't change with the iteration count (20 or 100), the mode, or an unrounded
      multiplier;
    - the rod drifts in the first 0.5 s, then freezes exactly (identical from 5 s to 10 s) once
      its motion per step falls below the 1e-6 pose grid;
    - unrounding the joint's orientation corrections fixes 480 Hz but not 240 Hz, so the
      position and velocity grids are involved too. This belongs to C6.
- **`sprung_hinge_stays_aligned_past_90_degrees`** (new): the Reviewer's probe. A sprung hinge
  starts at 2.5 rad of twist and is knocked 0.3 rad/s about x, with no gravity.
  - Three alignment-row cases: rigid, c = 1e-9 (both derived), and c = 1e-3 with `hinge_axis`
    set.
  - Gate: tilt < 1° and gap < 1 mm at 60 and 240 Hz. All pass.
  - Without C2 it comes apart.

**Regression** (2026-11-07, 08:43–09:13 EST), against the 1aa0aeb baseline:

| suite | 1aa0aeb | C1 + C2 |
|---|---|---|
| workspace tests | 128 pass | 133 pass (5 new) |
| fidelity: physical pendulum amplitude after 30 s, 60 / 240 Hz | 1.0017 / 1.0025 | 1.0003 / 1.0016 |
| fidelity: large swing, 60 / 240 Hz | 0.9746 / 0.9930 | 0.9754 / 0.9952 |
| biped push recovery at 20 N·s | 11/20 | 12/20 |
| other pushes (0, 6, 12, 30 N·s) | 20, 20, 20, 7 of 20 | unchanged |
| F1 seam tracking, line and ring | | completion counts identical; errors within ±0.006 mm |

Notes on these results:
- The 60 Hz large swing still fails its own ±0.02 tolerance (R5, decision D4).
- **The double pendulum's interim gate has little margin at 62.5 Hz:** 0.938 against 0.93. C3 or
  C4 may move it; recheck it when they land.

**Cost:** a chain of 20 hinged rods at 240 Hz (SHAKE, 20 position iterations) takes 453 µs per
step against 393 µs at HEAD: 15% more for the whole step. This is the extra pose reads and
quaternion products per row (`joint_solve_timing` in the diagnostics).

**Step hashes change for every scene with a D6 joint.** URIEL re-records its golden hashes at the
argref bump.

## 4. Order and reporting

1. **C1 and C2** together: about 1 day with the regression suites.
   - They are what the URIEL parity report waits on: C1 is the third open joint issue, C2 is R1.
   - R5 also needs a decision (D4) before the report.
2. **C4:** half a day.
3. **C3:** 2–3 days. It needs D1.
4. **C5:** 2–4 days. Do it only if C1 doesn't meet the parity acceptance for the workloads
   (P3/P4 in PARITY_SPEC).
5. **C6:** 1 day. It needs D2.

Constraints and reporting:
- Heavy runs finish before 11 PM EST; URIEL tests after that.
- Each commit is named to URIEL Orchestration for the argref bump.
- Specs and commits go to the RiftPhys Reviewer.

## 5. Decisions for the Designer

- **D1: what a drive's gains and cap mean.**
  - **kp = 0, kd = 0:**
    - (a) a rigid lock to the target: today's behaviour, and the default;
    - (b) the drive is off, so a stiff lock must use a large kp.
  - **kp = 0, kd > 0:** a pure damper, as a velocity row with target 0 (recommended, as in
    MuJoCo and PhysX). Otherwise it is an error.
  - **The cap:**
    - (a) a force, `max_force`, the same at every step rate (recommended);
    - (b) per step, as today, so the force allowed scales with the rate.
- **D2: is the q6 grid there for cross-platform determinism** (Windows vs Linux, SSE vs NEON)?
  - If yes, C6 option B; if no, option A.
  - The grid alone doesn't guarantee it (C6), so the Windows vs Linux determinism harness should
    answer this.
- **D3: continuous joints.**
  - Are continuous 360° arm joints planned soon? If yes, C3's unwrapped angle moves ahead of C4.
  - How does a position drive pick shortest path or absolute (C3)?
    - (a) A mode on each drive, defaulting to absolute with a limit and shortest path without
      one (recommended).
    - (b) The target decides: |target| ≤ π means shortest path, otherwise absolute. This is
      simpler but surprising near ±π.
- **D4: the 60 Hz large swing (R5).**
  - (a) Define the target for a symplectic scheme as bounded oscillation with no trend. Widen
    60 Hz to ±0.03 with that reason written down, and add a 60 Hz gate.
  - (b) List it as an open gap at 60 Hz in the parity report.
