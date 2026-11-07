# Real-world parity: definition and remaining work

Status: proposed 2026-11-06. The definition in section 2 needs the Designer's sign-off. W1 and
W2 are built (section 3.7); W3 to W6 are not started.

**Why.** The Designer's rule, relayed by URIEL Orchestration on 2026-11-06: URIEL physics
training resumes only once RiftPhys matches real-world reference values. argref then bumps its
RiftPhys pin in one step and re-records its jointed-scene golden hashes. Until now "parity" has
not been defined, so it couldn't be reported. This spec defines it and lists what's missing.

## 1. Where things stand

| Evidence | Status |
|---|---|
| Closed-form benchmarks, 240 Hz | 20/20 within tolerance (since c0ef751, SHAKE joints) |
| Closed-form benchmarks, 60 Hz | 18/20. Restitution overshoots 13%. Large swing reads 0.975 at 10 s against ±0.02, bounded within 0.973–1.029 |
| Real contact: 570 ContactNets cube tosses (Acosta, Yang & Posa, RA-L 2022) | Level with Drake and Bullet, ahead of MuJoCo (table below) |
| Real jointed mechanism | **None yet** |
| Joint correctness | Three open issues (docs/JOINT_ENERGY_SPEC.md 5.4) |

Cube tosses, held out:

| | position | rotation | e_q |
|---|---|---|---|
| RiftPhys | 13.2% | 17.1° | 0.29 |
| Drake | 13.5% | 16.5° | 0.27 |
| Bullet | 14.9% | 16.5° | 0.27 |
| MuJoCo | 25.1% | 21.7° | 0.38 |

**URIEL workloads** (crate `riftphys-uriel`):

| workload | physics rate | joints | limits | contact |
|---|---|---|---|---|
| F1 seam arm | 240 Hz (60 Hz control × 4 substeps) | 6 continuous hinges, with armature | none | tool and workpiece |
| Biped | 240 Hz | hinges | hip, knee, ankle, spine, neck | feet on the ground |
| Drone rollouts (M1, M2) | 60 Hz | none | none | none in flight |

**The biped's limits don't mean what they say.**
- **The cause:** hinge limits are compared against 2·sin(φ/2), not φ.
- **Knee:** its upper limit of 2.2 is beyond the largest value 2·sin(φ/2) can reach (2.0), so the
  knee has no working upper limit.
- **Hip:** its lower limit of −1.5 sits at −97° instead of −86°.
- **Ankle:** ±0.7 sits at ±41° instead of ±40°.

## 2. Proposed definition

RiftPhys is at parity for a workload when every criterion that workload needs, in the table
below, holds.

The comparison rule, used wherever another engine is the reference:
- RiftPhys's held-out error is no worse than the best published or reproduced engine's;
- or it is worse by less than 2 standard errors of the mean (SE = std / √n over trajectories).
- **Why that rule:** these datasets have per-trajectory spreads as large as the means (cube
  tosses: rotation 16.5 ± 20°). A bare "lower is better" would let noise decide.

| | criterion | arm (F1) | biped | drone |
|---|---|---|---|---|
| P1 | Closed-form benchmarks within tolerance at the workload's physics rate | 240 Hz | 240 Hz | 60 Hz, cases it uses |
| P2 | Real contact: cube tosses (ContactNets), comparison rule against Drake, Bullet and MuJoCo | | ✓ | |
| P3 | Real joints: a real double pendulum, comparison rule against MuJoCo run on the same data (section 3.3) | ✓ | ✓ | |
| P4 | Real articulated impacts: Cassie jumping (Acosta et al.), loss no worse than Drake's published 38.0 under the paper's metric (section 3.4) | | ✓ | |
| P5 | Joint correctness: limits in radians, compliant rows reset per step, plus the lever-arm fix if P3 or P4 shows it | (lever arm only if shown) | ✓ | |

Status against the definition:
- **Arm:** P1 is met. **P3 is missing.**
- **Biped:** P1 and P2 are met. **P3, P4 and P5 are missing.**
- **Drone:** P1 for its cases is met. Free fall, rotation, collision and resting all pass at 60 Hz;
  restitution is excluded while drone scenes have no bouncing contacts (decision D3).

**Per-workload parity** lets arm training resume once P3 holds, without waiting for the
biped's larger P4 (decision D1).

## 3. Work items

### 3.1 W1: hinge limits in radians (P5, biped)

- **The measure:** when exactly one rotational axis is free or limited (a hinge), measure the
  hinge angle as the twist about that axis, φ = 2·atan2(q_err · u, q_err.w), wrapped to (−π, π].
  Use it in:
  - the position solve (`solve_d6`, angular limit rows);
  - the velocity solve's speculative bound (`d6_rows`).
- **Everything else** (several limited rotational axes, as in a ball or universal joint) keeps
  the current small-angle measure. Document it, and add swing-twist limits later if needed.
- **Tests:**
  - a hinge with limits [−1.0, 1.0] rad stops at ±57.3° within 0.5°;
  - a limit of 2.2 rad stops at 126°;
  - limits still absorb energy (`hinge_limit_still_absorbs_energy`);
  - motors and position drives on hinges read the same angle.
- **Biped follow-up:** its limits become real angles; the knee gets a working upper limit at
  126°. Re-run `push_recovery_sweep` and the M3 suite. Re-tune only if they drop, and report the
  numbers to URIEL.
- **Estimate:** half a day, plus the biped re-check.

### 3.2 W2: reset the XPBD multiplier each step (P5)

- **The bug:** `D6Axis::lambda_acc` accumulates forever. It must be zeroed at the start of each
  step's position solve.
- **Effect:** compliant rows only (compliance > 0); rigid rows are unaffected. Cassie's springs
  (W4) need it.
- **Test:** a compliant hinge holding a weight sags to m·g·L·compliance within 1%, and stays
  there over 10 s; today it drifts.
- **Estimate:** one hour.

### 3.3 W3: real double pendulum (P3)

**Data.** Myers, Tempelman, Petrushenko & Khasawneh, "Low-cost double pendulum for
high-quality data collection with open-source video tracking and analysis", HardwareX 8 (2020)
e00138.
- **Source:** Mendeley Data, DOI 10.17632/7yd2ntbh3w.1, CC BY 4.0.
- **Contents:** 1000 fps tracked video, encoder data, and a SolidWorks CAD model.
- **Physical parameters:** the paper identifies damping with error bounds.
- **Download:** encoder data and CAD only, not the video. It needs the user's go-ahead, with the
  file sizes stated once the file list is read.
- **To confirm on download:** the number of runs, the encoder sample rate, and the link masses
  and lengths.
- **Fallback:** the IBM "Double Pendulum Chaotic" set (Asseman et al. 2018; CDLA-Sharing 1.0,
  21 runs of marker positions). Its stated 400 Hz rate is reported to be really 500 Hz, its host
  needs re-finding, and it has no joint friction data.

**Model.**
- Links and bearings come from the CAD (masses and inertias exported from SolidWorks) or the
  paper's tables.
- Each joint gets viscous damping c (N·m·s/rad) and Coulomb friction τ_f (N·m), both fitted.
- **Engine feature needed (W3a):** joint damping and friction on D6 rotational axes.
  - RiftPhys has neither: velocity drives cap impulse but ignore kd.
  - Add `D6Axis::{damping, friction}` as velocity-level rows:
    - friction: target relative velocity 0, impulse capped at τ_f·dt (like a motor);
    - damping: a soft row with impulse −c·ω_rel·dt, made implicit by solving it with effective
      mass k / (1 + k·c·dt), so it stays stable for large c.
  - MuJoCo has both natively (`damping`, `frictionloss`). Tests: a damped hinge's decay rate
    within 2% of the closed form; a friction-only hinge stops in θ = I·ω0² / (2τ_f) within 2%.

**Benchmark.** `riftphys-reality`:
- a converter `tools/double_pendulum_to_reality.py`;
- a scene `scenes/double_pendulum.json`;
- `riftphys-double-pendulum`, a binary like `riftphys-cube-toss`.

**Method:**
- **Windows:** start each window from the measured state. Angles come from the encoders;
  velocities come from a ±10 ms polynomial fit, as the cube-toss start states do.
- **Metric:** RMS angle error of both links at 0.1, 0.25, 0.5 and 1 s horizons. The motion is
  chaotic, so short horizons carry the comparison.
- **Fit:** fit c and τ_f per joint by CMA-ES on the even runs, holding out the odd ones; that is
  4 parameters.
- **Rate:** 240 Hz.
- **MuJoCo:** extend `tools/parity/mujoco_cube_toss.py` into `mujoco_double_pendulum.py`, with the
  same model, windows, fit and metric. Its result is the reference.

**Pass (P3):** at each horizon, RiftPhys's held-out RMS error is no worse than MuJoCo's by more
than 2 SE. Also report the energy decay rate per run against the measured one.

**Estimate:** 2–3 days, plus W3a (half a day).

### 3.4 W4: Cassie jumping impacts (P4)

**Data.**
- **Source:** github.com/DAIRLab/cassie_impact_dataset, BSD-3, under 1 MB, recorded on the
  robot in 2021.
- **Contents:**
  - 22 jumps, recorded at 2000 Hz around the first impact;
  - floating-base state (from a contact-aided invariant EKF), joint positions and velocities,
    and measured motor torques;
  - the ankle spring has no encoder, so its velocity is recorded as 0.
- **Download:** needs the user's go-ahead.

**Reference (Acosta et al., Table VI).**
- **Metric:** loss Σ_t x̃ᵀ W x̃ over a 50 ms window (100 samples) around the first impact.
- **Weights:**
  - positions 10;
  - velocities 100 for floating-base translation, 5 for floating-base rotation;
  - 0.01 for hip roll, knee spring and toe;
  - 1 for the other joints;
  - ankle spring velocity omitted.
- **Results:** Drake 38.0, MuJoCo 30.0, each with fitted contact stiffness, dissipation and
  friction. Bullet was excluded: it has no reflected motor inertia, which matters for Cassie.

**Model**, the largest piece of this spec:
- **Geometry:** Cassie v2 from `cassie_v2.urdf` (dairlib, BSD-3). RiftPhys has no URDF loader.
  Either add a minimal one (links, inertias, revolute joints, fixed joints; W4a, about 1 day) or
  build the model in code from the URDF values.
- **Closed loops:** the achilles rods (heel to hip) as distance joints. Drake lumps the rods'
  mass at the anchors; do the same.
- **Springs:** the knee and ankle leaf springs as compliant hinge rows. Needs W2.
- **Reflected motor inertia** (rotor inertia × gear ratio²) about each actuated joint axis (W4b).
  - **Today:** the arm adds armature isotropically to the child link. That approximation is
    wrong for Cassie, whose rotors are large.
  - **Proper version:** a 1-DoF rotor coupling, or an axis-only armature row in the velocity
    solve.
- **Feet:** toe contact as the paper models it (line or point contacts). Compliant contact
  (22be79e) is fitted the same way as for cubes.
- **Torques:** applied from the measured motor torques through the gear ratios. There's no
  controller, so the replay is open loop over 50 ms.

**Pass (P4):** RiftPhys's loss with fitted contact parameters is ≤ 38.0 (Drake) on the same
22 jumps and metric. If per-jump losses can be reproduced, use the comparison rule against
MuJoCo's 30.0 instead.

**Estimate:** about 1–2 weeks: model and loader 3–4 days, armature 1 day, replay and fit
2–3 days, debugging the closed loops and springs the rest. The biggest risk is model mismatch,
which the paper itself names as the limit for Drake and MuJoCo.

### 3.5 W5: lever-arm conditioning (P5, only if shown)

- **The problem:** small bodies on long lever arms converge badly in Gauss-Seidel
  (JOINT_ENERGY_SPEC 5.4).
- **The fix:** solve each D6 joint's rows as a block, 3×3 linear and 3×3 angular coupled
  through the lever arm, so a limit feels the inertia about the pivot.
- **When:** only if W3 or W4 shows the problem. Neither the arm nor the biped hits it today.

### 3.6 W6: restitution at 60 Hz (only if drone scenes bounce)

- **The problem:** a bounce reverses the velocity for the whole step, losing gravity's effect
  over the part after impact (13% at 60 Hz).
- **The fix:** a sub-step time of impact for restitution, splitting the step at the contact
  time.
- **When:** only if decision D3 says drone scenes need it.

### 3.7 Results: W1 and W2 (built 2026-11-06)

**W2, compliant joints.** Fixing the multiplier exposed four bugs, all fixed:

1. **The D6 multiplier carried over between steps.** `Joints::begin_step` now zeroes it at the
   start of each step's position solve, and `World::solve_joints` does the same per call.
2. **Distance joints had no XPBD multiplier at all.** Compliance was nearly ignored, and the
   stiffness depended on the iteration count. They now accumulate `lambda_acc` like D6 rows.
3. **Distance joints also had a rigid velocity row,** with a Baumgarte pull to the rest length,
   that ignored compliance. Compliant distance joints now act at position level only, as
   compliant D6 rows already did.
4. **Sleeping bodies relaxed their springs.**
   - **The cause:** a sleeping body gets no gravity, but the joint solve kept moving it, so a
     torsional spring at rest unwound to zero sag after 0.5 s.
   - **The fix:** a joint is now skipped when both its bodies are static or asleep
     (`Joints::solve_step`'s `frozen` mask).

Also fixed: dt² was rounded to the 1e-6 grid before computing alpha = compliance / dt². That made
every compliance 2% stiff at 240 Hz and 8% at 960 Hz.

Results (`compliant_joint_test.rs`): a joint started at its static sag or stretch stays within 3%
over 10 s.

| case | before | after |
|---|---|---|
| torsional spring, 60 Hz | 2100% off | within 3% |
| torsional spring, 240 Hz | | within 3% |
| hanging spring, 60 Hz | 100% off (rigid) | within 3% |
| hanging spring, 240 Hz, 20 iterations | | 99.99% of static |
| hanging spring, 240 Hz, 4 iterations | | 99.99% of static |
| hanging spring, 960 Hz | | 96.3% of static |

The 960 Hz figure is the λ quantization floor: λ is about 2e-5 on the 1e-6 grid.

**W1, hinge limits in radians.** `d6_hinge_axis` and `d6_row_angle` (riftphys-articulation) give
a hinge's own twist angle, 2·atan2(q_err along the axis, q_err.w). The position solve and the
velocity solve's speculative bound both use it. Tests (`hinge_angle_test.rs`):

| limit | before | after |
|---|---|---|
| −1 rad | would rest at −60° | rests at −57.3° (within 0.5°) |
| −2.2 rad | unreachable | holds within 1° |

Ball and universal joints keep the small-angle measure.

**New findings, not fixed:**
- **Hinge position drives don't reach their target.**
  - **Before W1:** target 1.0 settled at 88° under PBD, where the old measure meant 60°. Under
    SHAKE it oscillated and ended spinning.
  - **Now:** it settles at 24° instead of 57.3°, stable but wrong.
  - **Who's affected:** nothing in URIEL; the arm and biped apply torques. Investigate with W5;
    a likely cause is the angular row fighting the anchor rows.
- **Compliance below 5e-7 rounds to zero** (`q6(compliance)`), so very stiff springs become rigid.
  Cassie's leaf springs (W4) need checking against this.

**Regression check** (2026-11-06, finished 21:28, before URIEL's 23:00 testing):
- **Workspace tests:** all pass.
- **Fidelity report:** unchanged except the simple pendulum, a rigid distance joint now solved
  with a multiplier.
  - 240 Hz amplitude: 0.999 → 1.0003.
  - 60 Hz amplitude: 1.011 → 1.012.
  - Both within ±0.02.
- **F1 seam teacher:** identical, 10/10 on all six seams. The arm has no limits or compliant
  joints.
- **Biped `push_recovery_sweep`:** same at 0, 6, 12 and 30 N·s; 20 N·s went 12/20 → 11/20, one
  seed of 20.
  - **Why it changed:** the biped's limits now mean what they say. The knee has a working upper
    limit at 126°, which it lacked; the hip stops at −86° instead of −97°.
  - **Not re-tuned.**
  - **Not run:** the M3 DAgger suite, which is a training run, not a quick check.

## 4. Order and reporting

**Order:**
1. W2.
2. W1, then the biped re-check.
3. W3a, then W3. This gives arm parity: report to URIEL with the commits.
4. W4b, W4a, W4. This gives biped parity: report to URIEL with the commits.
5. W5 and W6 only if triggered.

**Each parity report:**
- is a new section in `docs/PHYSICS_FIDELITY.md`;
- is sent as a message to URIEL Orchestration;
- names every commit since argref's pin (2b98bb3) that the bump should include, at least
  22be79e (compliant contact), c0ef751 (SHAKE joints), and the W1, W2, W3a and W4 commits;
- includes any golden-hash changes.

## 5. Decisions for the Designer

- **D1:** per-workload parity, so arm training resumes after P3 without waiting for P4. Or one
  bar for everything.
- **D2:** the comparison rule: no worse than the best reference engine by more than 2 SE.
  Alternatives: strictly better on means; or an absolute tolerance per metric.
- **D3:** the drone at 60 Hz. Is excluding restitution acceptable while drone scenes have no
  bouncing contact? If not, W6 becomes required.
- **D4:** P4 against Drake's 38.0 or MuJoCo's 30.0. The plan uses Drake: it's the engine with the
  closest contact model, and MuJoCo's soft constraints are tuned differently.
- **D5:** downloads: the double pendulum (encoder data and CAD) and the Cassie dataset. Each needs
  the user's go-ahead with its size, given when asked.
