# Real-world parity: definition and remaining work

Status: proposed 2026-11-06. The definition in section 2 needs the Designer's sign-off. W1 and
W2 are built (section 3.7). W3a and W4b are built and committed 2026-11-08 (section 3.8).
2026-11-08: the Designer answered D1–D3 and D6 (section 5). W6 is now required and next
(section 3.6), then the D6 split (3.9). W3 and W4 wait on their data downloads (D5, sizes in
section 5); D4 is open. W5 only if triggered.

**Why.** The Designer's rule, relayed by URIEL Orchestration on 2026-11-06: URIEL physics
training resumes only once RiftPhys matches real-world reference values. argref then bumps its
RiftPhys pin in one step and re-records its jointed-scene golden hashes. Until now "parity" has
not been defined, so it couldn't be reported. This spec defines it and lists what's missing.

## 1. Where things stand

| Evidence | Status |
|---|---|
| Closed-form benchmarks, 240 Hz | 20/20 within tolerance (since c0ef751, SHAKE joints) |
| Closed-form benchmarks, 60 Hz | 19/20 since W6 (restitution 0.15%, was 13%). Large swing reads 0.975 at 10 s against ±0.02, bounded within 0.973–1.029 |
| Real contact: 570 ContactNets cube tosses (Acosta, Yang & Posa, RA-L 2022) | Level with Drake and Bullet, ahead of MuJoCo (table below) |
| Real jointed mechanism | A real double pendulum (Myers et al.): held-out error no worse than MuJoCo RK4's at 0.1–1 s, with the half-step start (section 3.3, decision D8) |
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
- **Arm:** P1 and P3 are met (P3 under decision D8, 2026-11-09). This is arm parity for the
  `Isotropic` default at (8, 6).
- **Biped:** P1, P2 and P3 are met (P3 as for the arm). **P4 and P5 are missing.**
- **Drone:** P1 is met, restitution included (W6, eee2e57; decision D3 made it required). Free
  fall, rotation, collision, resting and restitution all pass at 60 Hz (2026-11-10, on f2b2004).
  This is drone parity.
- **Reported 2026-11-10:** arm and drone parity, in PHYSICS_FIDELITY.md section 4 and to URIEL.

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
  - **Confirmed 2026-11-09** (the paper's open text, PMC9041261): three free-drop trials from
    about [180°, 0, 180°, 0]; encoders at 500 Hz, video tracking at 1000 Hz (both downloaded).
  - **Parameters without SolidWorks:** Table 6 lists every part (encoder, coupling, bearing
    races and balls, shafts, arms, dowel pin, trackers, tape): mass (balance or calculation),
    inertia about its centre (g·mm²) and pivot distances (ℓu 171.9 ± 0.3 mm, r̄5 86.0 ± 0.2,
    r̄9 71.5 ± 0.4), with uncertainties. The arms dominate: upper 132 ± 2 g, 470,000 ± 9,000
    g·mm²; lower 116 ± 2 g, 245,000 ± 5,000 g·mm². Table 4 gives the simplified two-link
    values (m1 0.311 kg, m2 0.111 kg, d1 0.079 m, d2 0.071 m, ℓ1 0.172 m, ℓ2 0.143 m). The model
    sums Table 6 per link; the CAD stays a cross-check, not an input.
  - **The paper's damping fit** (Table 3; viscous, quadratic and Coulomb per joint, about 69% of
    the loss Coulomb, mostly the encoder) is a second reference beside MuJoCo's for W3's fit.
  - **Licence:** the paper gives the files as GNU GPL v3; Mendeley lists the dataset as CC BY
    4.0. Using the measurements in tests is fine under either; nothing from the archives is
    copied into this repository.
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

**Built (2026-11-09), committed as f2b2004:** `tools/double_pendulum_to_reality.py`,
`riftphys-double-pendulum`, `tools/parity/mujoco_double_pendulum.py`. P3 is met with the
half-step start (decision D8).

What the data turned out to be, and what changed from the plan above:
- **Angles come from the video, not the encoders.** The encoder measures only the upper link. The
  video tracks both (500 Hz, degrees, unwrapped). The encoder is a cross-check: the clocks agree
  to 2 parts in 10⁴, offset equal to the encoder's trigger, and the two disagree by 1.3–1.7° RMS,
  which is the measurement floor.
- **The recordings start before the drop.** The pendulum hangs at 0; a hand lifts it to inverted
  in about 2 s, holds it, and lets go at t ≈ 5 s. The converter finds the hold and starts where a
  link has left it by 20°.
- **Table 6's upper arm is per plate.** The rig has two upper plates. Measured small-swing modes
  are 1.098 and 2.175 Hz; two plates (264 g) give 1.108 and 2.271 Hz, one plate 1.068 and 2.488.
  The model uses two. Bearing balls: 7 assumed. Tracker and tape offsets aren't tabulated (under
  1 g each; placed at the arm's centre).
- **No JSON scene:** the converter writes the rig into the log (a `rig` event), so RiftPhys and
  MuJoCo read the same numbers.
- **Windows:** one per second from the release while the lower link still moves over 1 rad/s,
  212 in all. With three trials, the fit uses trial 2 (70 windows) and trials 1 and 3 are held
  out (142).

Held-out error, degrees, mean ± SE over windows (fit on trial 2, 200 evaluations, 240 Hz):

| | 0.1 s | 0.25 s | 0.5 s | 1 s |
|---|---|---|---|---|
| RiftPhys (8, 6 iterations) | 1.60 ± 0.10 | 4.95 ± 0.42 | 9.94 ± 1.31 | 23.40 ± 2.68 |
| MuJoCo 3.17 Euler (default; implicitfast is identical) | 1.87 ± 0.14 | 5.90 ± 0.53 | 13.63 ± 1.63 | 30.26 ± 3.39 |
| MuJoCo 3.17 RK4 | 1.25 ± 0.08 | 4.33 ± 0.35 | 9.47 ± 1.30 | 22.47 ± 2.75 |

- Fitted damping: RiftPhys c [3.0e-6, 1.0e-5], τ_f [2.85e-3, 1e-5]; MuJoCo RK4 c [1.03e-4, 5.1e-6],
  τ_f [2.98e-3, 1.05e-5]. The elbow's friction sits at the bottom of the search box in both.
- RiftPhys's gap to RK4 at 0.1 s is step size, not the solver: 8, 64 and 512 velocity iterations
  all give 1.58° over all windows at 240 Hz; 480 Hz gives 1.38, 960 Hz 1.30, 3840 Hz 1.26, with
  the 240 Hz fit's damping (MuJoCo RK4, all windows: 1.22).
- **The reference is MuJoCo RK4 (decision D7).** On this first pass RiftPhys passes against Euler
  at every horizon; against RK4 at 240 Hz it misses at 0.1 s (0.35° worse, 2 SE = 0.20°) and
  passes at 0.25, 0.5 and 1 s. The final numbers follow the method below.
- **Not done:** the energy decay rate per run, a refit at higher rate.

**Method for the final numbers (written 2026-11-09, before the runs below; from the Reviewer's W3
review).** The table above is a first pass: its 2 SE is one engine's unpaired SE over windows that
aren't independent, one seed, and a window gate that measured tracker noise.
- **Windows:** gated on the ±10 ms smoothed rate of the lower link (peak over the window at least
  1 rad/s), not the raw sample-to-sample difference.
- **Fit:** each engine (RiftPhys; MuJoCo Euler; MuJoCo RK4), 240 Hz, trial 2 only, seeds 1–5 at 200
  evaluations. The primary objective is the one already run, the error summed over all four
  horizons. A second objective (0.1–0.5 s only) and 400 evaluations (seeds 1–3) are reported as
  sensitivity. Each engine's headline parameters are its seed with the lowest *training* objective;
  held-out spread over seeds is reported.
- **Comparison:** per horizon, the mean paired difference RiftPhys − MuJoCo over the same held-out
  windows, with a 95% block-bootstrap interval (blocks of 10 consecutive windows within a trial,
  10,000 resamples, fixed seed), and trials 1 and 3 separately. RiftPhys is **no worse** at a
  horizon when the interval's lower end is at or below 0 (the difference isn't shown to be
  positive; this takes the place of section 2's 2 SE, with the block interval as the SE).
- **Cross-scoring:** each engine with the other's parameters and with the paper's Table 3 (viscous
  as printed, quadratic dropped; Coulomb as μc·N with N the static load on the joint, and as
  μc·N·r with r = 8 mm, since the printed definition and the reported 69% share disagree).
- **Energy decay per run:** from each trial's first window, run free for 60 s; energy above rest
  at 5, 10, 20, 30, 45 and 60 s and the time to reach 50% and 10% of the start, each engine against
  the measurement (1 s moving mean of the measured energy).
- **The rig's residual:** the plate count was chosen using all three trials; it is one discrete
  choice shared by both engines. Table 4's m1 of 0.311 kg sits nearer two plates (0.341) than one
  (0.209). The upper mode measures 2.11 Hz at about 50° lower-link amplitude, 2.17 at 27° and 2.19
  at 18° against the model's 2.271, so about 3% remains after amplitude (about 6% in
  stiffness/inertia). Both engines share it; it probably sets part of the absolute error beyond
  0.25 s.

**Final numbers (2026-11-09, run by that method). P3 is not met at 240 Hz against RK4 (D7): RiftPhys
is worse at 0.1 s and 0.25 s, and no worse at 0.5 s and 1 s.** 197 windows, 131 held out.

Held-out mean error, degrees, each engine's lowest-training-objective seed; then RiftPhys minus
MuJoCo, paired, with the 95% block-bootstrap interval:

| | 0.1 s | 0.25 s | 0.5 s | 1 s |
|---|---|---|---|---|
| RiftPhys | 1.69 | 5.31 | 10.80 | 25.72 |
| MuJoCo RK4 (the reference) | 1.31 | 4.64 | 10.24 | 24.33 |
| MuJoCo Euler | 1.98 | 6.32 | 14.74 | 32.74 |
| **minus RK4** | **+0.38 [+0.28, +0.48] worse** | **+0.67 [+0.26, +1.17] worse** | +0.56 [−0.51, +1.69] | +1.39 [−1.61, +4.54] |
| minus RK4, trial 1 / 3 | +0.29 / +0.47 | +0.44 / +0.92 | −0.29 / +1.46 | +0.55 / +2.26 |
| minus Euler | −0.29 [−0.50, −0.11] | −1.01 [−1.93, −0.27] | −3.93 [−7.23, −1.12] | −7.03 [−15.77, +0.14] |

- **The gap is first order in the step, and the first order comes from the start.** RiftPhys
  against RK4 at 240 Hz, fixed damping: at 480 Hz +0.17 and +0.34 (0.1 and 0.25 s); at 960 Hz +0.09
  and +0.19. It halves with each halving of the step. The World's step is leapfrog (v += h a(x),
  then x += h v), so its velocity sits half a step before its position; positions are otherwise
  second order. A measured velocity handed in at the position's time starts it half a step late,
  an error of (h/2) a that carries through the window (the Reviewer's finding, 2026-11-09).
- **The fit doesn't explain it.** Held-out spread over 5 seeds: RiftPhys 1.69 / 5.28–5.31 /
  10.65–10.80 / 25.16–25.72; RK4 1.31–1.32 / 4.63–4.66 / 10.24–10.30 / 24.17–24.36. 400 evaluations
  and the 0.1–0.5 s objective give the same verdicts at every horizon, against both integrators.
- **Cross-scoring: damping barely moves the windows.** Held-out error at each horizon changes by
  under 0.1° at 0.1 s and under 0.25° at 0.25 s across all five parameter sets (each engine's fit,
  and both readings of Table 3), for all engines. The shoulder's 34× viscous difference isn't
  identified by 1 s windows, so it doesn't matter to this metric. RiftPhys's gap to RK4 persists
  with RK4's own parameters (1.69 / 5.28 / 10.62 / 25.14).
- **Energy decay per run** (mJ above rest, 5, 10, 20, 30, 45, 60 s; trial 1, the others alike):
  measured 859, 645, 381, 215, 78, 12 (t50 13.8 s, t10 41.1 s); RiftPhys 913, 769, 583, 398, 195,
  70 (21.8, 54.3); RK4 913, 773, 504, 310, 122, 23 (18.8, 46.7). Across the trials t50 is 12.9–13.8
  measured, 20.9–21.8 RiftPhys, 18.8–19.2 RK4, each with its own fit (with shared damping they
  agree: see the second method). The 0.1–0.5 s objective swings the other way (t50 about 4 s
  RiftPhys, 5.5 s RK4). An energy term in the fit would be needed to match decay.
- MuJoCo Euler at 240 Hz is fragile here: on some fits its 60 s run gains energy or goes unstable
  (seen with 400-evaluation parameters).

**Second method: the half-step start (written 2026-11-09, after the first result).** This was added
*after* the table above, once the Reviewer traced the first-order error to the start. The table
above stands as the result of the method written first. Whether the second method counts for P3 is
the Designer's decision (D8). It changes the initial condition, with no free parameter, and no
engine code:
- RiftPhys and MuJoCo Euler (both semi-implicit) start from the measured rate less half of the
  first step's change. MuJoCo RK4 takes the measured state as it is.
- Everything else is the first method: same windows and split, 5 seeds refitted, 0.1–0.5 s and
  400-evaluation sensitivity, the same paired bootstrap and verdict rule.
- **Energy with shared damping:** both 60 s decays, RiftPhys and RK4, each run with RK4's parameters
  and with Table 3's, to separate numerical dissipation from the fit.
- **Anywhere a jointed scene starts from a measured velocity needs the same treatment:** W4's
  Cassie replay is next. Free bodies already have the ballistic average. Any velocity set from
  outside the World, or read out for a policy, is half a step offset (2 ms at 240 Hz); no API
  change for now.

Results of the second method, held out, degrees (131 windows; each engine's lowest-training-objective
seed; RK4 as in the first table). **With the half-step start RiftPhys is no worse than RK4 at every
horizon, and better than Euler at 0.1, 0.25 and 0.5 s (1 s not separated):**

| | 0.1 s | 0.25 s | 0.5 s | 1 s |
|---|---|---|---|---|
| RiftPhys, half-step start | 1.23 | 4.49 | 10.64 | 26.61 |
| MuJoCo RK4 (reference, measured state) | 1.31 | 4.64 | 10.24 | 24.33 |
| MuJoCo Euler, half-step start | 1.46 | 5.62 | 13.84 | 31.80 |
| **minus RK4** | −0.08 [−0.17, −0.00] | −0.15 [−0.38, +0.09] | +0.40 [−0.89, +2.45] | +2.28 [−1.85, +6.15] |
| minus RK4, trial 1 / 3 | +0.00 / −0.17 | +0.02 / −0.33 | +1.29 / −0.53 | −0.29 / +4.96 |
| minus Euler | −0.23 [−0.43, −0.07] | −1.13 [−2.13, −0.31] | −3.20 [−6.72, −0.06] | −5.20 [−13.72, +1.67] |

- **Verdicts hold across the sensitivity runs:** the 0.1–0.5 s objective and 400 evaluations are no
  worse than RK4 at every horizon as well.
- **The fit has two basins with this start.** Seeds 1–2 land on shoulder Coulomb friction (τ_f 3.6e-3,
  c 1.7–3.2e-6) and seeds 3–5 on viscous damping (c 1.1e-3, τ_f 1.6–5.7e-4), with near-equal window error:
  held-out 1 s error is 23.6–23.7° in the first basin and 26.2–26.8° in the second. The rule picks the
  lowest training objective, seed 4, in the second. RK4 (first method) stays in the first.
  **Both basins are no worse than RK4 at every horizon, so the bimodality doesn't touch the
  verdict.** RiftPhys − RK4 for the Coulomb basin's lowest-objective seed (2): −0.07 [−0.14, −0.01],
  −0.06 [−0.20, +0.09], +0.43 [−0.76, +2.30], −0.63 [−3.28, +1.60] at 0.1, 0.25, 0.5, 1 s; trial
  1 / 3: −0.02 / −0.13, +0.04 / −0.16, +1.04 / −0.20, −0.94 / −0.30. The viscous basin is the table.
- **Energy with shared damping: no numerical dissipation shows.** With RK4's parameters, t50 and t10
  are 18.1–19.8 and 43.1–48.8 s for RiftPhys, 18.8–19.2 and 45.6–47.5 s for RK4. So the first
  table's bullet that RiftPhys loses energy more slowly was the fit, not the engine. Both engines
  lose energy too slowly against the measured t50 of 12.9–13.8 s and t10 of 39.3–41.1 s with those
  parameters. Table 3 (quadratic dropped, Coulomb as μc·N) is far too light in both: t50 43–49 s.
- With the second method's own fit (viscous basin) RiftPhys decays too fast (t50 6.6–7.0 s): 1 s
  windows don't identify damping. An energy term in the fit is the way to match decay; it wasn't in
  the method.

**Estimate:** 2–3 days, plus W3a (half a day).

### 3.4 W4: Cassie jumping impacts (P4)

**Data.**
- **Source:** github.com/DAIRLab/cassie_impact_dataset, BSD-3, 1.32 MB (96 files), recorded on the
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
- **Torques:** ~~applied from the measured motor torques through the gear ratios~~ (wrong: they
  are joint-side, 3.4a). Applied as measured, open loop over 50 ms; there's no controller.

**W4b design: axis-only armature** (drafted 2026-11-07 overnight, for review before building).
- **The model:** a rotor geared rigidly to the joint, inertia J_a (rotor inertia × gear
  ratio²) about the joint axis u, feeling no gravity or contact. Its velocity is the joint's
  relative angular velocity along u.
- **As a velocity row:** a soft row on u with gamma = 1 / J_a and target ω_rotor, the rotor's
  velocity carried from the last step. It solves ω_rel + λ / J_a = ω_rotor, the rotor's momentum
  balance: the links take λ and the rotor −λ, so J_a (ω_end − ω_rotor) = −λ.
  - Under a torque τ across the joint (or gravity on the links), ω_rel changes by τ dt / (I + J_a)
    per step, the reduced-coordinate answer. Derivation: the links' impulse gives
    ω0 + k τ dt; the row gives λ = −k τ dt / (k + 1/J_a); so ω_end = ω0 + τ dt / (1/k + J_a).
  - It uses the same soft-row machinery as W3a's damper (`gamma`), so it adds no new solver code.
- **ω_rotor is state.** It goes in `CarriedState` and the step hash (AT_REST_SPEC D2).
  - **Why it must be carried** (the RiftPhys Reviewer): `apply_torque` writes I⁻¹τ straight into
    the velocity between steps. Read from ω_rel at the start of a step, the rotor would never see
    a host torque (the biped's PD, Cassie's measured motor torques), and J_a would do nothing.
  - **What is stored:** the rotor's own balance after the solve, ω_rotor − λ / J_a, using the
    row's accumulated λ. It equals ω_rel when nothing runs after the iterations. The shock and
    restitution passes apply contact impulses with no joint rows; storing ω_rel would hand
    their change to the rotor with no impulse, creating or destroying ½ J_a ω² at an impact,
    which is W4's window. Stored this way, the rotor changes only by impulses it received, and
    the next step's row reconciles the two inelastically.
  - The first step after a joint gets armature starts from its current ω_rel.
  - `set_body_pose` and `remove_body` drop the entries of joints on that body. `set_body_vel`
    keeps them: hosts that write velocities every step would otherwise turn armature off
    silently. A velocity reset then exchanges momentum with the geared rotor in one inelastic
    step, which is physical. `reset_d6_rotor(joint)` is the explicit reset.
- **The position solve doesn't see J_a.** SHAKE projects the locked rows only, and the free
  axis has none, so nothing there needs it.
- **API:** `set_d6_armature(joint, axis, j_a)`, on rotational axes.
- **The arm keeps its isotropic armature.** Switching it to the axis row changes F1's dynamics
  and the teacher, so it's decision D6, not part of W4b.
- **Zero values:** the row exists only for J_a > 0 (gamma = 1 / J_a); the setter rejects
  negative and non-finite values, as W3a's does.
- **Torques in the W4 replay:** `World::apply_torque` takes an angular impulse (N m s), not a
  torque; measured motor torques need × dt.
- **Tests:**
  - a hinge with armature under a constant torque accelerates at τ / (I + J_a) within 1%, at
    60 and 240 Hz, and also when the host writes the links' velocities every step;
  - a pendulum's small-swing period is 2π √((I_pivot + J_a) / (m g d)) within 1%;
  - energy, counting ½ J_a ω², is kept over a 10 s swing as well as without armature;
  - a rollback across a step with armature replays hash for hash;
  - across an impact on a jointed limb, the rotor's momentum change equals the row impulses.

**Pass (P4), superseded by 3.4a (review 2026-11-10):** RiftPhys's loss with fitted contact parameters is ≤ 38.0 (Drake) on the same
22 jumps and metric. If per-jump losses can be reproduced, use the comparison rule against
MuJoCo's 30.0 instead.

**Estimate:** about 1–2 weeks: model and loader 3–4 days, armature 1 day, replay and fit
2–3 days, debugging the closed loops and springs the rest. The biggest risk is model mismatch,
which the paper itself names as the limit for Drake and MuJoCo.

### 3.4a W4 build plan, from the paper's code (drafted 2026-11-10, for review before building)

**Sources.** I read these as text; nothing was downloaded:
- the paper (arXiv 2110.00541, the corrected version);
- dairlib `examples/Cassie/cassie_utils.{h,cc}` (master);
- the paper's evaluation code, on dairlib's `contact_model_verification` branch
  (`examples/contact_parameter_learning/`, MIT, last commit 2022-02-24).

**What they show, and where 3.4 changes:**
- **The 22 jumps** are raw logs 08–11, 14–17 and 20–33 (`log_nums_real`). In the public
  dataset (its legend maps trajectory to log) these are trajectories 00–03, 06–09 and 10–23.
  Trajectories 04 and 05 (logs 12 and 13) were left out. Trajectories 24–28 (2021-02-26) came
  later.
- **The windows differ from the public ones.**
  - The paper simulated each jump from a fixed raw-log time, 30.60–30.66 s, and lowered the
    pelvis 0.015 m on 11 logs.
  - The public trajectories start about 5 ms before impact, at 30.637–30.668 s, with their own
    offsets (−0.008 to +0.020 m). For log 08 the two windows share only 12 ms.
  - So Drake's 38.0, and the branch's per-jump Drake and MuJoCo runs (`drake_optimal`,
    `mujoco_optimal`), can't be paired with runs on the public data. The raw logs are only
    available from the authors.
- **The loss as computed** (`cassie_loss_utils.py`) is a per-jump **mean** over the samples,
  not the sum that equation 4 shows. Only the mean gives values near 38. The terms, averaged
  over jumps:
  - positions (pelvis xyz and the 16 joints), weight 10 (the paper's text; the code reads its
    weights from a pickle);
  - velocities (pelvis translation and the 16 joints), from `print_loss_weights`, which matches
    the text: pelvis translation 100; hip roll, knee spring and toe 0.01; ankle spring 0; the
    rest 1;
  - pelvis angular velocity, 5;
  - orientation, (2 × quaternion angle distance)², weight 1 by default.
- **The torques are joint-side**, applied with a first-order hold between the 2 kHz samples.
  Drake's actuation port takes joint effort, and hip roll reaches 42 N·m in the data, far above
  what its motor could give at the motor side. **3.4's "through the gear ratios" is wrong:** the
  replay applies u directly, as u·dt of angular impulse.
- **Reflected inertia:** rotor × ratio² from `AddCassieMultibody`, with rotor inertias (61, 61,
  365, 365, 4.9)e-6 kg m² and ratios (25, 25, 16, 16, 50) for hip roll, hip yaw, hip pitch, knee
  and toe. That gives J_a = 0.0381, 0.0381, 0.0934, 0.0934 and 0.0123 kg m² on each side.
- **Springs:** the knee is 1500 and the ankle 1250 N m/rad in dairlib's code ("2300 in URDF,
  1500 from gazebo"). The URDF says 2300 and 2000. Drake used the code's values.
- **Loops:**
  - Achilles rod: a spring of k 1e6 and damping 2e3 with rest length 0.5012 m, between
    heel_spring (0.11877, −0.01, 0) and thigh (0, 0, ±0.045).
  - Plantar rod: none. The toe torque goes straight to the toe joint.
  - The URDF lumps the rod masses at the anchors.
- **Contact:**
  - four point contacts, two per foot, at (−0.0457, 0.112, 0) and (0.088, 0, 0) in the toe frame;
  - Drake's compliant point contact, with Table VI's identified stiffness 9400 N/m, dissipation
    3.0 s/m and μ 0.45;
  - Drake ran at dt 5e-5 to 8e-5 s.
- **Fit:** nevergrad NGOpt over stiffness [1e3, 1e6] (log), dissipation [0, 3] and μ [0, 1], on
  a random 80/20 train/test split. Table VI's "average loss" doesn't say which set it covers.

**Plan:**
- **W4a: URDF loader** (`riftphys-reality/src/urdf.rs`):
  - reads links, inertials (with the origin's rpy, rotated into the link frame), revolute and
    fixed joints, limits and `<dynamics damping>`;
  - fixed joints merge into the parent: mass, COM, and inertia by the parallel-axis theorem;
  - builds bodies and hinges at a configuration q by forward kinematics, with a world rotation
    from z-up to RiftPhys's y-up;
  - no meshes. Contact points are named by the caller.
  - **Tests:** a two-link URDF matches the same model built by hand (pose, inertia, the hinge's
    period); Cassie's total mass equals the URDF's sum; fixed joints conserve mass and COM.
- **W4: the Cassie model:**
  - the URDF plus the dairlib constants above;
  - the achilles rod as a distance joint with compliance 1e-6 (Drake's spring);
  - leaf springs as compliant hinge rows (1/k);
  - armature through `set_d6_armature`;
  - URDF damping through `set_d6_damping`;
  - URDF limits in radians;
  - the four contact points as 1 mm spheres with compliant contact against the ground.
    Under compliant contact a loaded sphere rests F / k deep (about 1 cm or more), with no slop
    added. Any line that runs rigid feet instead rests each sphere 4 mm with its centre inside
    the ground (`CONTACT_POS_SLOP` is 5 mm, more than r), which the inside-box fix (b4c3c52)
    handles; such a line says so. The ground is a box of half-height 0.5 m, well beyond any
    depth.
- **The state:** dataset rows go through forward kinematics to body poses and velocities. The
  pelvis angular velocity is world-frame (Drake's quaternion floating joint). Ankle-spring
  velocity is unrecorded (0).
  - **Check first:** the toe heights at the first sample. Small and positive means the public
    offsets are already applied.
- **Half-step start,** as in W3, per body: v0 − ½(v1 − v0) from one trial step. The 5 ms
  before impact has no contact.
- **Metric:** the loss above, on the paper's 22 jumps, and on all 29 as a second line.
- **Fit:** k, dissipation and μ by W3's CMA-ES with 5-fold cross-validation, so each jump's loss
  comes from a fit that didn't see it. The in-sample fit is reported too, since 38.0's split is
  unknown.
- **References and pass (rewritten after review):**
  - **MuJoCo rerun on the public windows is the only like-for-like bar.** Drake's 38.0 and
    MuJoCo's 30.0 are on the paper's windows (from 30.60 s, about 40 ms of flight before
    impact, against about 5 ms here) and an unknown split, so a loss on these windows isn't
    comparable to them. They become context lines.
  - MuJoCo runs Agility's MJCF from cassie-mujoco-sim (the model the paper used; D10), refit
    in its own parameters as `learn_cassie_parameters.py` does (stiffness [0, 1e6], damping
    [0, 1000], μ [0.01, 1]), on the same folds as RiftPhys, not at Table VI's values.
  - **Pass (P4):** per jump, held out, RiftPhys's loss is no worse than MuJoCo's under D2's rule
    (2 SE, a bootstrap over the 22 jumps, not over samples), at 240 Hz for both (D9). If
    Agility's model isn't stable at 240 Hz, report that and pair at 2 kHz, saying so.
  - Without D10 there is no valid P4 bar.
- **Folds:** 5 folds over the 22 jumps, fixed before any run with seed 1, stratified by day
  (Jan 27: 00–03 and 06–09; Feb 12: 10–23), the same for both engines. The fold list is written
  here before the first fit. The 29-jump line adds Feb 26 (24–28), a third day, and is labelled
  so.
- **Tests beyond the loss:**
  - the achilles rod's stretch equals compliance × its row force (not a fixed 1 mm: at 1e-6 m/N
    a landing can reach 1000 N per mm), and it doesn't ring (or has damping; below);
  - the rod length from forward kinematics at sample 0 against 0.5012 m, per jump, before any
    run: 1 mm of violation is a 1000 N kick on the first step;
  - spring sag with the pelvis fixed, hanging: deflection = the static moment about the spring
    from the model's own mass at that pose, over k;
  - free-floating Cassie in zero g with random joint torques keeps total linear and angular
    momentum (catches one-sided torques);
  - a forward-kinematics round trip: a dataset row → bodies → joint q, q̇ and the pelvis
    origin's pose and velocity read back equal the row (catches the index order, z-up to y-up,
    and COM against origin);
  - the loss reproduces `cassie_loss_utils.py` on synthetic cases (simulation equal to the data
    gives 0; a known offset gives its known value);
  - the impact check W4b deferred: across the impact, each rotor's momentum change equals its
    row's impulses;
  - a rollback across the impact replays hash for hash.

**Review (RiftPhys Reviewer, 2026-11-10), folded in:**
- **Blocker: the distance joint has no anchors** (bodies.rs: `DistanceJoint` and its row use
  each body's COM and linear inverse mass only). Built as planned, the achilles rod would tie
  the heel-spring COM to the thigh COM and pass no torque, which is the rod's whole job. **W4c,
  below, before the model.**
- **Mine, the same kind: colliders have no offset** (`Collider` is body, shape, AABB,
  material; a shape sits at the body's COM). The toe points aren't at the toe's COM. **W4d,
  below.**
- **Torques** are confirmed joint-side, more strongly than 42 N·m shows: the largest |u| over
  the 29 trajectories exceeds the URDF's effort on three actuators (hip roll right 117.7 against
  112.5; knee 230.3 and 225.6 against 195.2; toe left 64.4 against 45), where motor-side peaks
  would be 4.5, 12.2 and 0.9 N·m. So:
  - nothing clamps to effort (the loader reads it and doesn't apply it);
  - each torque acts as an equal and opposite pair on parent and child (`apply_torque` acts on
    one body, so the replay calls it twice);
  - the first-order hold is integrated over each step on the real sample times (0.27–0.78 ms
    apart, not a uniform 0.5; a 240 Hz step spans about 8 samples).
- **The half-step start's trial step** is 4.17 ms at 240 Hz against about 5 ms before impact,
  with offsets up to +0.020 m. The trial step runs with contact off, and each jump's first
  contact time is reported. The per-body form stands: v0 and v1 both satisfy the joint rows.
- **The achilles row's damping:** Drake's spring had 2e3 N s/m. Undamped, a rod of effective
  mass near 0.5 kg rings with a period of about 4 ms at 2 kHz, inside the window. W4c adds
  damping to the distance row, or a test shows it doesn't ring.
- **Ankle-spring velocity 0 isn't loop-consistent.** The rod length rate it implies at sample 0
  is reported. Drake used 0, so 0 stays the main line. If the rate isn't small, a
  loop-consistent start is a sensitivity line.
- **Frames:** the dataset's and the loss's pelvis xyz and linear velocity are the pelvis frame
  origin, not the COM: v_origin = v_com + ω × (o − c), reading the state in and computing the
  loss. ω is world-frame: on t_00 the finite-differenced quaternion matches world-frame ω to
  0.096 rad/s and body-frame to 0.918 (the Reviewer's check).
- **Velocity indices:** the README's table lists ankle_spring_right before toe_left, but the
  columns are 41 ankle_spring_left, 42 toe_left, 43 ankle_spring_right, 44 toe_right, the
  position order. The replay uses the indices. Columns 41 and 43 are 0 throughout.
- **The paper's z offsets** were hard-coded, not fitted: `drake_cassie_sim.py` (v1) loads
  `optimized_z_offsets.pkl` but then sets 0, and −0.015 m for the 11 logs in
  `log_nums_w_offset`. Velocity offsets are 0.
- **Which learner gave Table VI:** v1 (`learn_cassie_parameters.py`, budget 5000), whose
  parameters `cassie_impact_data.py` loads as `drake_2021_09_01_10_training_5000`. The v2 files
  use the curated trajectories and are dated 2022.
- Smaller: μ's lower bound is 0.01 for both engines; the URDF marks the ankle spring's damping
  of 10 as "made up"; RiftPhys's compliant contact is Drake's Hunt–Crossley form
  (materials.rs), so k and d in s/m fit like for like.

### 3.4b W4c and W4d: anchored distance joints, collider offsets (drafted 2026-11-10, reviewed the same day)

The RiftPhys Reviewer approved both designs on 2026-11-10, with the changes folded in below:
an accumulated damper, the SHAKE form written out, new goldens, the swept reach, and one pose
accessor.

**W4c: a distance joint with anchors and damping.**
- **Today:** `DistanceJoint { a, b, rest, compliance, lambda_acc }` holds the two COMs at
  `rest`, with inverse masses 1/m only. Its velocity row applies a fresh impulse every
  iteration (no accumulator) with a Baumgarte bias.
- **Change:** local anchors `la`, `lb` (in each body's frame) and a damping coefficient `c`
  (N s/m). They are set by `World::add_distance_joint_at(a, la, b, lb, rest, compliance)` and
  `set_distance_damping(id, c)`.
- **Zero anchors and no damping run today's two functions untouched** (a branch, not
  arithmetic). At la = lb = 0 the mixed form below would give (wa + wb)(n·n_cur) + α against
  today's (wa + wb + α)(n·n_cur), and a zero angular delta through a quaternion update can
  change bits. Checked by new goldens, `distance_bits_test.rs`, recorded at f781e2a before the
  change (debug and release agree):

  | scene | 60 Hz | 240 Hz |
  |---|---|---|
  | rigid rod | 87f81a83f9615530 | e7411745726c1915 |
  | compliant rod | 6f111d612da3e10f | ed532e5e87913e18 |
  | guard controller | bb57c95b1772e5eb | c1a92484be836750 |

  Nothing golden covered a distance joint before.
- **Position row, SHAKE form:**
  - C = |x_b − x_a| − rest, measured on the current anchors; n_cur is its direction, with
    r_a, r_b the current lever arms.
  - Correct along n0, the start-of-step anchors' direction. The lever arms are r_a0 = R_a,start
    l_a and r_b0, and the inverse inertias I_a0⁻¹, I_b0⁻¹ are taken at the start-of-step
    orientations, as D6 does.
  - The denominator is the mixed Jacobian product w_mix = (w_a + w_b)(n_cur·n0)
    + (r_a × n_cur)ᵀ I_a0⁻¹ (r_a0 × n0) + (r_b × n_cur)ᵀ I_b0⁻¹ (r_b0 × n0).
  - Δλ = −(C + α λ_acc) / (w_mix + α): SHAKE's Newton step, with the compliance exact.
  - Position deltas ∓ w n0 Δλ; orientation deltas ∓ I0⁻¹ (r0 × n0) Δλ.
  - **Fallback** to current-iterate (PBD) directions, lever arms and inertias when either body
    turned more than `SHAKE_MAX_TURN` this step, or w_mix < ½ w_sym, where w_sym is the
    start-of-step generalised mass.
- **Velocity rows** are `JointRow`s in the same list the D6 rows use, so each gets an
  accumulator, the generalised mass and the solve branch:
  - a rigid rod: an equality row along the current n with the current lever arms, target 0, as
    D6's locked rows (the position solve removes drift; the old Baumgarte bias isn't kept on
    the anchored path);
  - a compliant rod has no velocity row.
  - **Not warm-started.** The rows start cold each step, so they add nothing to the carried
    state or the hash.
- **Damping (revised after the build, the Reviewer's design):** in the compliant row's own
  position step, XPBD's damping term (Macklin, Müller and Chentanez 2016):
  Δλ = −(C + α̃ λ_acc + γ̃ ΔC) / ((1 + γ̃) w + α̃), α̃ = compliance / dt², γ̃ = compliance c / dt,
  where ΔC is the row's change since the step's start and w is w_mix (with its fallback).
  - **Why:** built first as a velocity row (above, as drafted), the damper ran before the
    spring's projection, resisted each step's fresh gravity increment, and so carried
    c dt / (m + c dt) of a static load. A 2 kg box on a compliant rod with c = 20 at 60 Hz
    stretched 0.857 of F × compliance, exactly that. The spring and damper are now one implicit
    Kelvin-Voigt row; at rest ΔC = 0, so the spring carries the whole load.
  - **The same for D6:** a compliant D6 row with damping (`set_d6_damping` on an enabled row with
    compliance > 0) takes the same term, with ΔC from the row's start-of-step coordinate (its
    hinge angle, alignment measure or anchor gap), and no longer emits a damper velocity row.
    W3a's free-axis and limited dampers stay velocity rows, bit for bit. W3a's compliant-row
    case had this flaw and is fixed here (3.8).
  - **Hashes:** no golden scene has damping on a compliant row (the only non-test users of
    `set_d6_damping` are the double pendulum's free hinges and the URDF loader), so none moves.
- **The integrator's own damping of compliant rows (found building W4c):** a compliant row is
  corrected at position level only, so it behaves like implicit Euler and loses energy by
  itself. Measured with no damper: the amplitude falls to 0.94 per period at ω dt = 0.02 (and
  0.73 at ω dt = 0.1); Drake's achilles spring on 0.5 kg at 2 kHz (ω dt = 0.71) undershoots
  only to −0.36 of its start. For W4 this is Cassie's leaf springs at 240 Hz (D9). Their
  frequencies are to be measured on the built model (hung, each spring plucked at 2 kHz), not
  estimated, before any fit.
- **Tests** (`distance_anchor_test.rs`, 6, and `distance_bits_test.rs`), all passing:
  - the goldens above, bit for bit;
  - two hinged levers joined by an anchored rod pass torque at the lever-arm ratio within 2%,
    with the levers at 90°, 60° and 120°, rigid and at compliance 1e-5;
  - a static load stretches the rod by F × compliance within 1% with damping 0, 20 and 200 N s/m,
    at 60 and 240 Hz; a damped compliant hinge sags to its static angle within 1% with damping
    0, 2 and 20 N m s/rad, at 60 and 240 Hz;
  - the damper, where the oscillator is resolved (ω dt = 0.02 at 240 Hz; at the drafted 0.1 the
    integrator's own decay, 0.73 per period, swamped it), measured against the undamped run's
    decay: undamped period within 1%; at ζ = 0.1 the damped period within 2% and the damper's
    decay per period 0.5262 against e^(−ζωT) = 0.5318; at ζ = 1 an undershoot of 1.4% of the
    start (critical damping is the boundary; gate 2%);
  - Drake's 1e6 and 2e3 at 2 kHz: settles with no overshoot; at c = 0 it undershoots to −0.36
    (it doesn't ring long: see the integrator's own damping above);
  - a closed loop (two levers, a compliant damped rod, a damped sprung hinge) released under
    gravity: energy, counting both springs, never rises above its start (0.6900 J, to 0.6445 at
    60 Hz and 0.6471 at 240 Hz over 5 s), and the rod stretches at most 1.9 mm.

**W4d: collider offsets.**
- **Change:** `Collider` gains `local: Option<Isometry>` (the shape's pose in the body frame),
  set by `World::add_collider_at(body, local, shape, material)`. `add_collider_at` never touches
  mass, as `add_collider` doesn't: the URDF's mass stays authoritative.
- **One accessor,** `Collider::world_pose(body_pose)`, which returns `body_pose` itself when
  `local` is `None` and `body_pose * local` otherwise. Every read of a collider's world pose
  goes through it.
  - So identity is bit-identical by construction, not by arithmetic (q·IDENTITY turns −0.0
    into +0.0, which atan2, copysign and 1/x see).
  - Missing a site is impossible.
- **The swept reach:** step.rs inflates each collider by (|v_lin| + |ω| reach) dt, with reach
  the collider's own AABB half-diagonal and v_lin the COM's speed. For an offset collider,
  reach becomes |local.pos| + the half-diagonal (0 + half for no offset, so bit-identical).
  - **Why:** a toe turning at 50 rad/s at 60 Hz moves a sphere 0.1 m from the toe's COM 83 mm
    per step, while today's sweep adds 1.4 mm. The contact would be found late, or not at all,
    in the impact step.
- Contacts store world points, and the solver's lever arm is point − COM, so the solver needs no
  change. Only the toe spheres collide, and directly jointed pairs are filtered (step.rs), so
  self-collision isn't a concern.
- **Tests:**
  - one offset-collider test per path: AABB and broadphase, the sphere-against-ground
    narrowphase pair the toes use, CCD/TOI, raycast, shape cast and overlap;
  - a free body spinning about its COM with a 1 mm sphere 0.1 m off it, approaching the floor
    at 60 Hz, finds the contact before penetrating more than the margin (fails on today's
    reach);
  - the lever arm, directly: a body with one offset sphere, dropped flat so the sphere touches
    first, has the angular velocity of the rigid-body point-impact closed form (r × J) right
    after;
  - a sphere offset 0.5 m from a box's COM rests with the box tilted at the closed-form angle;
  - the at-rest suite and the golden hashes unchanged.

**W4d, built (2026-11-10):** `Collider::local`, `world_pose`, `reach`; `add_collider_at`. The
broadphase sweep, both narrowphase paths, the AABB updates, `raycast_exact` and the capsule
sweep's discrete check read through `world_pose`; `raycast` and the capsule sweep's continuous
part read the AABB, which follows the offset. The CCD integrators in ccd.rs and
`World::integrate_positions` are never called (dead code), so they aren't touched.
- **Tests** (`collider_offset_test.rs`, 5), all passing:
  - the AABB follows the offset, turned and after a step;
  - `raycast_exact`, `raycast` and the capsule sweep hit the offset sphere and miss the COM;
  - a body spinning at 50 rad/s about its COM with a 1 mm sphere 0.1 m off it, falling on the
    floor: whenever the sphere could reach the floor within the step, its contact exists
    (mutation-checked: with the old reach, the sphere was 87.8 mm up, within reach, with no
    contact at 60 Hz). At 240 Hz the first impact acts at 0.17 mm deep. At 60 Hz (0.83 rad a
    step) it acts at 5 mm deep: a linear speculative contact can't follow so coarse an arc,
    offset or not, so that depth is printed, not gated;
  - the lever arm, directly: a bar falling flat at 1 m/s onto one sphere 0.3 m from its COM
    spins at the point-impact closed form within 2% (2.2906 against 2.2907 rad/s) once the
    contact point's normal velocity is 0 (the impact takes two steps: speculative, then
    arrival);
  - a bar resting on two spheres 0.5 m either side of its COM stays level (under 1e-3 rad) at
    the spheres' height (within 1 mm), at 60 and 240 Hz. This replaces the drafted tilted-box
    test, which was the same check made indirect.

**Review of the built W4c and W4d (RiftPhys Reviewer, 2026-11-10), and the fixes:**
- **Blocker, fixed: box–sphere contact with the sphere's centre inside the box.** Measured from
  the nearest face point as outside, the normal pointed into the box and the penetration shrank
  with depth (at 1 mm, 2 mm and 5 mm under a floor with r = 1 mm: 3.0, 2.0 and no contact against
  true 5.0, 6.0 and 9.0 mm), so a contact past r + contact_dist vanished and the sphere fell
  through. Centred spheres were masked by the COM-direction normal flip, but lost the contact all
  the same. Cassie's 1 mm toe spheres would sink about 24 mm at every landing (about 225 N each
  at 9400 N/m).
  - **The fix (narrowphase.rs, `collide_box_sphere`):** inside, the normal is the nearest face's
    outward normal, the penetration r + contact_dist + the centre's depth below that face, at any
    depth; the point is halfway between the face and the sphere's deepest point, as outside.
    Box–capsule tests points with box–sphere, so it inherits the fix.
  - **Tests:** `box_sphere_inside_test.rs` (centre at depths r/2, r, 2r and 20r under a turned
    floor: outward normal, penetration r + d + contact_dist, both argument orders);
    `collider_offset_test.rs`: a 10 kg body on one 1 mm sphere on compliant ground (9400 N/m)
    rests at F / k = 10.4 mm within 2%; the lever scene with gravity, 2 s past impact, keeps its
    offset sphere within 1 mm of the rigid contact's 5 mm position slop. Without the fix the
    first sank 44 m and the second 20 m.
  - **Sphere–triangle, checked and left for its own item:** a triangle has no inside, so past
    its contact range there's nothing to keep. Within range, a centre behind the triangle's
    plane gets a reversed normal, as box–sphere did, and its normal also points from the
    triangle to the sphere, the opposite of box–triangle's A → B. Triangles are the terrain's
    and the F1 seam's workpiece (golden seam SHA-256), and Cassie stands on a box, so this goes
    to a separate review rather than into W4.
- **Damping with no start poses, fixed:** under the PBD projection the step captured no start
  poses, so a damped compliant row measured no change and wasn't damped at all (measured: 0.9997
  per period against 0.5318). The step now captures them under either projection, and
  `Joints::solve_step_with` takes SHAKE's poses and the step's start poses apart. Outside a step
  (`World::solve_joints`) there is no start, and such rows act as springs only (documented).
  Test: `damping_acts_under_the_pbd_projection_too`.
- **A soft limit keeps its damper, fixed:** which rows move their damping into the position row
  is one rule, `position_damped`: enabled, compliant, damped, no limit and no velocity drive.
  Any other damped row keeps W3a's velocity damper (a limited row's position part acts only
  outside its range, so moving its damper there left it undamped inside). Test:
  `a_soft_limited_hinge_keeps_its_damper_inside_its_range` (spin within 2% of
  (1 + c dt / I)^−n).
- **The contact normal flip** (step.rs) compares the colliders' world positions, not the bodies'
  COMs (the same for colliders with no offset).
- Accepted as built: the ζ = 1 gate at 2%; the 60 Hz first-impact depth printed, not gated.
- **Regression (2026-11-10, after all four fixes):** every suite passes (collision, world, URIEL,
  reality). No golden moved: `joint_bits`, `w6_jointed_bits`, W6's scene hashes, the distance
  goldens, the seam and the cube tosses are unchanged, so no golden scene had a sphere's centre
  inside a box.

**Order:** W4c, then W4d, then the W4 model.

**Pre-fit checks (built 2026-11-10, `riftphys-reality/src/cassie.rs`, `riftphys-cassie --check | --pluck`):**
- **The model:** `cassie_v2.urdf` through the W4a loader (17 bodies, 16 joints), with dairlib's
  leaf springs (1500 and 1250 N m/rad), reflected inertia on the 10 actuated joints, the achilles
  rods as anchored distance joints (W4c; 1e-6 m/N, 2e3 N s/m), four 1 mm toe spheres on
  collider offsets (W4d), and a ground box, contact at Table VI's Drake values until fitted.
- **The forward-kinematics round trip** (a test, all 29 trajectories at samples 0, 50 and 99):
  orientation 7.3e-8 rad, pelvis origin 3.6e-8 m, joints 1.1e-7 rad, pelvis ω 2.1e-7 rad/s,
  pelvis origin velocity 7.9e-8 m/s, joint rates 5.0e-7 rad/s, at worst. The column order, z-up
  to y-up and origin against COM hold.
- **Rod length at sample 0:** 0.5012 m on every trajectory, both sides, to under 0.01 mm (the
  state estimator's inverse kinematics closes the loop). No first-step kick.
- **Rod rate at sample 0, with the ankle springs' unmeasured rate as 0:** up to 0.51 m/s
  (trajectory 1, left); 0.1–0.4 m/s is common. The 0 is an estimator artifact on an unweighted
  coordinate: the positions close the loop, so the robot's real state is loop-consistent.
- **The start, decided before any fit (the Reviewer's review of 10af5a4):**
  - **main line: the loop-consistent start,** for both engines. `cassie::loop_consistent` solves
    each ankle spring's rate (one scalar per leg; the rod's rate is linear in it) so its rod's
    length rate is 0: under 7e-7 m/s on all 29 trajectories (a test), with ankle-spring rates of
    up to 4.46 rad/s. MuJoCo gets exactly the same starting velocities, so its equality
    constraint doesn't resolve the inconsistency its own way.
  - **sensitivity line: the ankle springs' rate at 0** for both engines, as Drake had it. The
    context comparison with Drake's 38.0 sits on this line. It differs from the main line in that
    one thing only.
  - **The data state is the same for both engines; each applies its own start correction.** The
    half-step correction belongs to an integrator, not to the data: RiftPhys's is its leapfrog
    offset, taken from its own trial step, as in W3. MuJoCo Euler takes a half-step from its own
    trial step; MuJoCo RK4 takes none. Handing MuJoCo RiftPhys's corrected velocities would build
    RiftPhys's correction into the reference.
  - **RiftPhys's start, main line:** the loop-consistent state, then the half-step correction from
    a trial step on it, which runs with contact off and must show a rod impulse at gravity and
    torque scale, not closure scale. In joint coordinates, q̇_start = q̇_lc − ½(q̇_1 − q̇_lc).
  - **RiftPhys's start, sensitivity line:** the main line's corrected start with the two ankle
    springs' rates then set to 0. (A trial step taken on the 0 state would remove the closure as
    a velocity jump and apply −½ of it backwards, up to about 0.25 m/s of extra rod rate; leaving
    out the correction would drop the half-step start too. Either way the line would differ in
    more than the ankle springs' rate.)
  - **MuJoCo's integrator:** Agility's `cassie.xml` sets PGS with 50 iterations and MuJoCo's
    default Euler integrator at a 0.5 ms step, and the paper's runs used it. As in W3, both
    Euler (the model's own option) and RK4 run, each at 240 Hz for the pairing (D9) and at 2 kHz
    beside it, with the model's solver settings otherwise unchanged; the reference for the pass is
    the stricter of the two on the held-out loss (D7's rule for P3). If Agility's model isn't
    stable at 240 Hz under an integrator, that is reported and that integrator pairs at 2 kHz.
- **The recorded rates against the recorded angles (a cross-check, reported, not gated;
  `riftphys-cassie --rates`):** each loop-consistent ankle-spring rate beside the slope of a line
  through that spring's recorded angle over the first 6 samples (about 2.5 ms; the estimator's
  inverse kinematics). They agree poorly: of 58 legs, 20 differ in sign (where either is above
  0.2 rad/s) and 30 differ by more than 1 rad/s (largest: trajectory 3 left, −0.82 against
  −6.96; trajectory 10, −0.29 / −1.52 against −3.58 / −4.44). The loop-consistent rate is built
  from the other joints' recorded rates, so the recorded rates and positions aren't mutually
  consistent at the first samples. That bounds how well any engine can match the window's first
  milliseconds, for RiftPhys and MuJoCo alike.
- **Toe heights at sample 0:** 11–88 mm above the ground (sphere bottoms). The earliest ballistic
  arrival of any toe is 4.8–11.6 ms on the paper's 22, consistent with "about 5 ms before
  impact" and the public offsets already applied. So the half-step start's trial step (4.17 ms
  at 240 Hz) ends before any contact on the 22; the replay asserts it and prints each jump's
  first-contact time. Trajectory 24 (Feb 26) wouldn't arrive until 82.8 ms, outside its 50 ms
  window. **Decided before any fit:** it stays in the 29-jump line, labelled "no impact in
  window"; it is never in the paper's 22, which carries the pass.
- **Leaf-spring frequencies, measured** (hung: pelvis welded, the 10 motors locked in position by
  a limit with min = max, no joint or rod damping, 2 kHz, the first 0.5 s, a linear trend
  removed; trajectories 0, 5, 10, 15, 20, 23): each leg's knee spring, ankle, ankle spring and
  rod form a 2-DoF loop with two modes, at 10.5–15 Hz and 18–23.5 Hz (8.5–23.5 Hz overall). The
  locked motors creep under 1e-5 rad. So ω dt at 240 Hz is 0.22–0.62, where implicit Euler
  keeps 0.19–0.51 of the amplitude per period; at 2 kHz it is 0.03–0.07. The swing-leg periods,
  43–118 ms, are as long as or longer than the 50 ms window.
  - **Corrected the same day:** the first measurement held the motors with velocity drives at 0,
    which hold a rate, not an angle. The hung leg crept (the left knee 0.08 rad in 4 s) and that
    contaminated the spectra: it gave 12–17.5 Hz, and a 3.5 Hz "mode" on trajectory 20's right
    leg that was the creep. The Reviewer asked for the check rather than an interpretation;
    `riftphys-cassie --probe` prints each joint's motion, limit margin and frequency.
  - These are the swing leg's modes.
- **Leaf springs standing** (`riftphys-cassie --stance`: a trajectory's last sample, at rest,
  motors locked, undamped, on the compliant ground at 9400 N/m, the pelvis on a vertical guide,
  2 kHz for 2 s, the first 0.3 s left out): the dominant mode is the body's bounce on its legs and
  the ground, 2–2.5 Hz on all six trajectories, with the springs and the ground in series: ω dt
  0.05–0.07 at 240 Hz, where implicit Euler keeps 0.82–0.85 per period. Its period (400–500 ms)
  is far longer than the window. Unguided, with the motors locked, the robot toppled within 2 s
  and its pelvis fell through the floor (only the toes collide), so the guide is needed.
  - **So for D9:** standing, the integrator's damping is small; it bites on the swing-leg modes
    (8.5–23.5 Hz, 0.19–0.51 per period at 240 Hz), the legs' motion relative to the body during
    the impact.

### 3.4c Sphere–triangle contact behind the plane (open; owner RiftPhys Orchestration; after W4)

Opened 2026-11-10 at the Reviewer's request, from the inside-box fix's review. Built after W4
unless a workload hits it first. It doesn't block W4 (Cassie stands on a box), and the seam's
golden shows the seam never takes a sphere centre behind a triangle.
- **The failure:** the terrain is triangles. A sphere or capsule pushed, or landing hard enough,
  that its centre crosses a triangle's plane is measured from the closest point as if in front:
  the normal reverses and it is pulled through, as box–sphere did before b4c3c52.
- **A convention mismatch:** `collide_sphere_triangle` returns the normal from the triangle to
  the sphere (B → A for the sphere-first call), while `collide_box_triangle` returns A → B
  (box → terrain), the convention `collide_shapes` documents. Terrain contacts aren't flipped by
  the step, so which one the solver expects decides which is wrong; settle it first.
- **The design point:** a single triangle has no inside, so "behind the plane" needs a rule.
  Proposed: one-sided, by the winding's normal (as box–triangle already is); a centre behind the
  plane within r + the contact margin, over the triangle's face, counts as penetrating, with the
  face normal and penetration r + margin + depth. Farther behind is no contact, or thin terrain
  gets pushed through from below.
- **Tests:** the box_sphere_inside_test depth series (r/2, r, 2r, 20r, and just in front)
  against a single triangle and a two-triangle quad, in both argument orders; the same for a
  capsule; terrain and seam goldens.

### 3.5 W5: lever-arm conditioning (P5, only if shown)

**Update 2026-11-07:** see JOINT_CONDITIONING_SPEC.
- Most of this was a stale-measurement bug, fixed as its item C1.
- Sprung hinges are fixed as C2.
- The block solve below is now that spec's item C5, still only if needed.

- **The problem:** small bodies on long lever arms converge badly in Gauss-Seidel
  (JOINT_ENERGY_SPEC 5.4).
- **The fix:** solve each D6 joint's rows as a block, 3×3 linear and 3×3 angular coupled
  through the lever arm, so a limit feels the inertia about the pivot.
- **When:** only if W3 or W4 shows the problem. Neither the arm nor the biped hits it today.

### 3.6 W6: restitution at 60 Hz (required: D3, 2026-11-08)

**Required, and first** (D3: "if a drone falls it should bounce"). Drafted 2026-11-08, for review
before building.

**Measured at b1ebad0** (`riftphys-fidelity`): a rubber ball (e 0.80) dropped from 1 m rebounds
to 0.7234 m at 60 Hz against e² h = 0.640 (+13.0%), and 0.6626 m at 240 Hz (+3.5%).

**The cause is the bounce step's position, not its velocity.** Worked through at 60 Hz:
- The ball starts the impact step 43.5 mm above the floor at 4.25 m/s. Contacts are speculative,
  so the step's solve closes the gap, and restitution then sets the end velocity to
  e × (mid-step approach speed) = 3.47 m/s.
- A body with contacts moves semi-implicitly, x += v_end dt, so the ball ends the step 58 mm
  *higher* than it started, at 101 mm. It never went down the 43.5 mm to the floor.
- In truth the ball falls 10.0 ms to the floor, bounces at e × 4.35 = 3.48 m/s and rises for the
  remaining 6.6 ms, ending at 23 mm moving at 3.41 m/s.
- The extra 78 mm is the overshoot: 12% of 0.64 m. The velocity is only 1.6% off. At 240 Hz the
  same error is a quarter as large.
- A free-flying body already moves exactly under constant gravity (the ballistic average in
  step.rs); only the bounce step is wrong.

**The fix: a time of impact inside the step, for the bounce's velocity and its position.**
- **Per contact, when it is built** (rigid contacts with a restitution bias, as today: approach
  faster than 1 m/s, mid-step), store along the normal:
  - s0, the gap at the start of the step (−pen, from the start-of-step narrowphase);
  - v0, the relative velocity at the start of the step (already computed, `v_pre`);
  - a, the relative acceleration from this step's forces: (velocity after forces − v0) / dt.
- **Time of impact:** the smallest t_c in [0, dt] with s0 + v0 t + ½ a t² = 0. t_c = 0 when
  s0 ≤ 0 (already touching), dt when there's no root (it can't arrive this step).
- **Velocity:** v_imp = v0 + a t_c, v_out = −e v_imp, τ = dt − t_c. The restitution pass's target
  becomes v_end = v_out + a τ (today: −e × the mid-step approach). The pass is otherwise as today:
  only contacts that carried load, accumulated impulse ≥ 0.
- **Position:** the gap at the end should be s_end = max(0, v_out τ + ½ a τ²). The body's
  integration gives s0 + v_end dt. The difference goes in as a pseudo-velocity along the normal
  (as Bullet's split impulse): target u = (s_end − s0) / dt − v_end, solved with the contact's
  normal effective mass on per-body pseudo-velocities (linear and angular), a few Gauss-Seidel
  sweeps over the bounced contacts, unclamped. Only when s0 > 0: a contact already penetrating
  is the position projection's job.
  - The integration adds the pseudo-velocity to the pose (x += (v + u_lin) dt, and a turn by
    u_ang dt) and discards it. It never enters the stored velocity, the warm start, the carried
    state or the hash, and since `predicted_poses` are taken after it, the XPBD velocity update
    doesn't turn it into velocity.
  - In the worked case u = −4.65 m/s: the ball ends the step at 23 mm, moving up at 3.41 m/s.
- **Scope: bodies in no joint.** A contact on a jointed body (the biped's feet, the arm's links)
  keeps today's restitution exactly, so the biped and the arm stay bit-identical and URIEL's
  numbers don't move. Extending it to jointed bodies needs the joint rows in the pseudo solve;
  that waits until a workload needs it (the biped's rubber feet land slower than 1 m/s when
  standing; a fall would be the case).
- **Unchanged:** compliant (Hunt-Crossley) contacts; the 1 m/s threshold; which contacts bounce;
  everything at rest (no contact approaching faster than 1 m/s gets a bias).

**Revised after the RiftPhys Reviewer's review (2026-11-08: 2 high, 1 medium, 4 low):**
- **H1, the pseudo solve freezes bodies held elsewhere.** u is always negative (it pulls the pair
  together), so a box bouncing on a resting box would lift the lower box about 38 mm with no
  velocity, off its own floor contact. A body is immovable in the pseudo solve (inverse mass and
  inertia 0) when it has a contact outside the bounced set that carried load (normal, friction
  or soft impulse), or any contact with a jointed body: the shock pass's freeze rule. Test: a box
  dropped onto a resting box; the lower box moves < 1 mm across the bounce, and the total energy
  doesn't rise.
- **H2, free bodies in speculative range move ballistically.** `constrained` (step.rs) counts
  every contact, speculative ones with zero impulse included, and the reach is 5 cm + |v| dt.
  So a slow rebound flies its whole arc semi-implicitly and loses about ½ v_out dt at the apex.
  In the Reviewer's 1-D model (an ideal impact step, range as in the code), the second bounce
  comes out 17% low for the drone case and 23% low for 1 m with e 0.4. Fix: a body in no joint
  whose contacts all ended the solve with zero impulse (normal, friction, patch, soft) uses the
  ballistic average (step.rs:174). That brings every case in the model within 0.1%.
  - **Guard (added in the build):** the average starts from last step's velocity, so with a host
    impulse decelerating the body (a drone landing under thrust) it can approach faster than
    the end velocity the speculative bound accepted. A body goes ballistic only if, for each of
    its contacts, the gap at the step's start covers the averaged approach: s0 + v_avg·n dt ≥ 0.
    Otherwise it stays semi-implicit, as today.
  - Free bodies hovering within reach of the ground (drone takeoff and landing, a box tossed
    near a table) change, and so do their golden hashes. Jointed bodies don't.
- **L1, the motion model is the integrator's:** v0's linear part is `last_lin_vel` (as the
  ballistic average), the angular part the start-of-step rate, and a = (velocity after this
  step's forces and host impulses − v0) / dt. A host impulse is force over the step, as the
  integrator treats it, so a drone landing under hover thrust gets the right impact speed.
- **L2, the root:** t_c = 2 s0 / (−v0 + √(v0² − 2 a s0)): the smallest root for any sign of a
  and v0, no division by a, s0 / (−v0) at a = 0. No real root, or a denominator ≤ 0, or t_c > dt:
  t_c = dt.
- **L3, binding with no root:** with a toward the contact, the contact can bind
  (s0 + v_end dt < 0) though the exact path doesn't arrive within dt (by up to ½ |a| dt²). Then
  t_c = dt: the body bounces at the step's end with s_end = 0. That's a bounce, not "no bounce".
  The reverse (a root but no binding) needs a pointing away from the contact; such a contact
  carries no load and keeps today's behaviour.
- **L4, sweeps:** the restitution pass is one sequential sweep, so a box landing flat spins off
  its first corner. Contacts on the time-of-impact path get 4 sweeps (the first as today, with
  the accumulated clamp); the pseudo solve 8, and its residual is exposed for the tests
  (`World::last_bounce_residual`). Jointed contacts keep the single sweep, bit for bit.
- **v_end ≥ 0:** a bounce too weak to leave within the step (v_out τ + ½ a τ² < 0) ends at rest on
  the contact, v_end = 0 and s_end = 0, instead of approaching again.
- **Report (Q2):** the fidelity report says that contacts on jointed bodies still bounce the old
  way at 60 Hz, including a free ball striking an arm link (the contact keeps today's path).
- **Cube tosses (Q3):** re-measured, fitted and held out. The time of impact follows each contact
  point in a straight line, ignoring rotation (a corner at 5 cm spinning 20 rad/s curves about
  2.8 mm per 60 Hz step, the order of s0), so spin-heavy tosses are reported separately if they
  move.

**Build status (2026-11-08, uncommitted, single core):**
- Built as revised, with three changes the build found:
  - **Support rows, not a freeze** (the Reviewer's build review, 2 high: freezing a body held
    elsewhere made it immovable in every direction, so a ball striking a resting crate from the
    side gave it no share of the bounce, and a slow loaded corner of the same box suppressed its
    fast corner's bounce). Before W6, a box dropped on a box resting on the floor barely bounced
    (0.7 m/s at 60 Hz, 0.1 m/s at 240 Hz, against e v = 1.8): the restitution pass has no floor
    row, so the bounce drove the lower box 5 mm into the floor, and the projection then popped
    it 4.6–5.8 mm above rest. Now:
    - **restitution:** the bounced bodies' other loaded rigid contacts join the 16 sweeps as
      push-only rows (fresh accumulator ≥ 0, no more approach than before the sweeps), along
      their own normals;
    - **pseudo solve:** they join as rows holding their gap (pseudo approach rate 0, both ways;
      a push-only row can't stop the pull lifting a resting box off the floor). Mutation check:
      without them the lower box lifts 15.6 mm;
    - **the whole pile** (the Reviewer's rework review): the involved bodies grow to a fixed
      point through rigid loaded contacts, stopping at static, sleeping and jointed bodies, so a
      box bouncing on a two-box stack is held up by the floor under the bottom box, not just by
      the box below it. Compliant contacts are supports in neither pass (their Hunt-Crossley
      force answers on later steps);
    - jointed bodies are immovable in both (their joint rows aren't there);
    - **solved directly, not swept** (see "Solver" below): up to 512 rows, the velocity pass is
      an interior-point solve and the pseudo pass one least-squares solve; above that, both
      sweep alternately to 1e-6 x (1 + |target|) m/s, at most 256 sweeps;
    - **the support target** is the main solve's own, min(depth_bias, 0): no approach beyond the
      gap. Holding only the approach the 8 main iterations left let a struck stack keep sinking
      (8 mm), and the projection popped it back;
    - **test:** a box dropped 1 m onto a two-box stack. Across the bounce the lower boxes move
      under 0.04 mm and the top box rebounds to e² h within 0.15%; with support stopping one box
      down (mutation) the middle box moves 8.9 mm and the rebound is 0.065 m. When the top box
      lands for good, the stack settles into its contact slop under the extra weight, to about
      where b1ebad0 puts it (b1ebad0 sinks it 10 mm at the first impact instead). New with W6:
      the top box rattles while the stack settles (jolts above 0.02 m/s: 5, 11, 14 in seconds
      1–4, none after), where b1ebad0 is still by 1 s. **Traced** (`w6_stack3_probe`, per step
      1.4–7 s), under the Reviewer's rule: no W6 branch fires while it rattles (no bounce,
      support or pseudo rows, no ballistic step, every contact loaded) and no contact approaches
      faster than 0.36 m/s, so it is the aftermath of a legitimate bounce, accepted. Test
      `a_struck_three_stack_is_still_by_5_s`: from 5 s to 6 s the stack moves 3e-9 m (60 Hz) and
      7e-11 m (240 Hz). With unconverged sweeps (the 64-solve cap hit, below) it still rattled at
      5 s. The stack rests about 5 mm deep per contact after the hard landing, as at b1ebad0
      (C at 0.0951): an at-rest item, not W6's;
    - the gap rows can contradict each other (a ball bouncing off one wall while loaded against
      the opposite one, both normals along the pull): Gauss-Seidel then doesn't converge, and
      `World::last_bounce_residual` reports it. An unconverged residual means the bounce lands
      slightly short of its exact position; the velocities are unaffected. The tests assert
      < 1e-4 m/s;
    - friction stays out of the restitution sweeps, as before W6: a box struck from the side
      leaves at its full share before floor friction acts in that step;
    - **impact model:** a side resting on something other than its partner in the impact is
      struck from its start velocity with its forces balanced (a box on the floor is met at g by
      a box falling onto it, and is still when struck from the side).
    - The upper box rebounds to e² h and the lower box doesn't move; a side hit splits as the
      1-D closed form (ball 0.3000, box 2.7000 m/s at v 3, e 0.8; momentum to 4 digits); a fast
      corner beside a slow loaded pivot leaves at 0.97–1.00 e v.
  - **H2 narrowed.** All-ballistic flight within reach made the 45° yawed stack of
    `yawed_stacks_stand` rock for about 3 s after landing from its 1 cm gap. The last step before
    arriving (a contact the end velocity reaches within two steps) stays semi-implicit.
  - **v_end ≥ 0 and s_end clamped**, as revised.
- **Results:** every `restitution_toi_test` case passes (12). First apexes within 0.7% at 60 Hz
  (rubber from 1 m −0.15%, was +13.0%) and 0.03% at 240 Hz; second apexes within 3%; the drone
  −0.13%, no spin, pseudo residual 0; the tilted drone gains no energy; the zero-g head-on matches
  the closed form to 4 digits, gap included. Jointed scenes bit-identical
  (`w6_bits_probe.rs`, `joint_bits_probe.rs`). H1 mutation-checked: without the pseudo freeze
  the lower box lifts 18 mm and the test fails.
- **Open: the 45° yawed stack's landing is fragile, before W6 too.** Dropped from gaps of 0 to
  2 cm (`w6_bits_probe.rs`, `w6_yawed_gap_sweep`), b1ebad0 rocks after 0.8, 1.2 and 2 cm (up to
  7.2 mm moved, cold contacts while rocking) and holds after 0, 0.4, 1.0 and 1.6 cm; the test's
  1 cm was a good draw. With W6 it rocks after 0.4, 0.8, 1.0 and 2 cm and holds after 1.2 and
  1.6 cm. Once still, it stays still (1e-8 m over 50 s). Decided with the Reviewer:
  `yawed_stacks_stand` starts touching, and the landing is AT_REST_SPEC R3b, gated by the sweep.
- **First regression (suites_w6, swept build):**
  - fidelity: rubber restitution at 60 Hz +13.03% (fail) → −0.15%; at 240 Hz +3.53% → +0.01%;
    one case outside tolerance, was two;
  - seams (line, ring) bit-identical; biped push sweep unchanged;
  - cube tosses, held out, fitted: 21.0 → 15.8 (measured params 25.6 → 26.3); 570 tosses 1.6 →
    2.6 s, the fit 100 → 306 s (the sweeps);
  - **one failure: `stacks_stand_kept_awake`, the 10-box stack at 60 Hz moved 24.2 mm after the
    settle second (gate 10 mm; b1ebad0 6.0 mm).**
- **Solver (2026-11-08, after that regression).** The 10-box stack collapses through its 1 cm
  gaps and the top box arrives at 1.48 m/s, a legitimate bounce. Its bounce pass, 39 rows, hit
  the 256-sweep cap; with the cap lifted it converged after 2,301 sweeps and the stack moved
  4.05 mm. Even a two-box stack needed 116–175 sweeps. Support runs down a pile as a chain, and
  a tall stack's rocking modes are weak: Gauss-Seidel is the wrong tool. Tried in order:
  - a plain direct solve: corners of a face give K = J M⁻¹ Jᵀ a null space (4 rows, 3
    freedoms), so a regularised factorisation turns slightly inconsistent targets into huge
    opposing impulses that cancel exactly but not in f32 (a pile thrown at 100 m/s). Every solve
    is now ridge least squares, (K² + εI) x = K r, ε 1e-14 of the largest diagonal: exact when
    rows agree, the smallest impulses when they can't;
  - active set with one-row pivots, then block principal pivoting: they cycle (pivoting is
    guaranteed only for positive definite K);
  - primal active set, then projected Newton: monotone, but one bound per step, or a zigzag on
    the pile (error 4 → 0.04 m/s in 60 solves);
  - **built: primal-dual interior point with Mehrotra's predictor-corrector** (`bounce_qp`,
    `world/bounce_lcp.rs`): the rows are a monotone LCP (total impulse y = acc0 + x ≥ 0, velocity
    error w = K y + q ≥ 0, y·w = 0). Final form below;
  - the 10-box stack moves 4.05 mm at 60 Hz (gate 10 mm, b1ebad0 6.0) and 0.07 mm at 240 Hz; the
    at-rest suite passes (11/11).
- **The Reviewer's reviews (2026-11-08), and the solver as it lands:**
  - **components:** the rows split into connected components (union-find over the bodies that
    move; an immovable side links nothing), each solved alone in a fixed order, in both passes.
    Two unrelated bounces don't share one system, its size, or its solve count;
  - **sparse:** K is held sparse (rows couple only through a shared body) and factored by a
    left-looking Cholesky in a minimum-degree order computed from the row graph alone, lowest
    index on ties (an ordered set keyed by degree, then index). Fill is small: L holds 17–28
    entries per row below the diagonal against 24–29 in K, from 122 to 742 rows (66 bricks:
    K 25.9, L 24.4 per row). A dense factor is kept as the test reference;
  - **compliance, not a floor:** the solve uses K + diag(1e-12 K_ii) (constraint force mixing),
    positive definite, so the solution and every Newton step are unique. Tried first and dropped:
    a floor on w/y (1e-10 of the largest, then per row) with iterative refinement against the
    unfloored matrix. Refinement against a singular K + D builds up null-space components (a
    1000:1 case fell back), and refining only where the floor binds missed rows just above it.
    1e-9 compliance was too much at 1000:1 (the light box's rows reach K_ii × impulse = 1e4 m/s,
    leaving 1e-5 m/s); 1e-12 leaves 1e-8;
  - **the stopping test** judges, against the unregularised K, that each row meets its target or
    carries no impulse and none approaches beyond it, to 1e-7 m/s (an f32 velocity's floor).
    An absolute 1e-8 on the regularised residual had stalled at f64 rounding at 1000:1 while y
    drifted along K's null space (51 solves instead of 8);
  - **K symmetrised:** each body's world inverse inertia is symmetrised in f64 when the rows are
    built. R I⁻¹ Rᵀ in f32 is symmetric only to about 1e-7, K came out unsymmetric, and
    Cholesky (which reads one triangle) solved a slightly different matrix: backward error 4e-10
    on a 5-row system, now 1e-16;
  - **the start** is scaled per row (y0 = eta / K_ii, w0 = eta). A Mehrotra-style start (a damped
    solve, shifted positive and centred) took the 1000:1 case from 18 to 8–9 solves and the side
    hit from 20 to 18, but not the piles (25–26 either way): W6c;
  - **checks (`the_sparse_factor_is_right`, restitution_toi 15/15):**
    (i) the first Newton system of each velocity solve, sparse against dense: 8e-16 to 9e-16 on
    the pile, 1e-16 on the stack (gate 1e-12);
    (ii') every solve's normwise backward error, infinity norms: at most 7.9e-17 on the pile,
    1.1e-16 on the stack, 2.4e-16 at 1000:1 (gate 1e-12);
    (iii) the outcome: no fallback, and the pile, stack and rebound gates;
    true residual on pushing rows against the unregularised K: at most 9.6e-8 m/s (the stopping
    tolerance), gate 1e-7.
    Per-row sparse/dense agreement at 1e-9, asked for first, can't be had: near convergence w/y
    spans 1e-30 to 1e30, and two backward-stable factorisations differ by cond × 1e-16;
  - **the fallback, visible and tested:** `BounceStats` counts components and fallbacks;
    process-wide `solver::bounce_fallbacks_total()` and `bounce_largest_component()` (diagnostics
    outside the hash and rollback) are printed by `riftphys-fidelity`, `riftphys-cube-toss` and
    `riftphys-uriel-seam --eval`, expected 0. `the_sweeping_fallback_stays_sane` forces it with
    a direct limit of 8 rows (`World::set_bounce_direct_max`, tests only, not in `CarriedState`
    or the hash): finite, no energy gained, residual ≤ 2.4e-6 m/s;
  - **no `powi`** in the solve (its precision is unspecified, and the hash depends on it); no
    process-environment knob anywhere in the solve.
- **Cost** (`w6_cost_probe`: a box dropped 1 m onto brick pyramids; release, single core; the
  bounce passes' own wall time, `BounceStats::pass_us`):

  | rows (pyramid base) | dense factor (first build), bounce step | final sparse build, passes (min–median, two runs) | solves | L per row |
  |---|---|---|---|---|
  | 115–122 (5) | 5.3 ms | 1.6 ms | 22 | 15.4 |
  | 155–171 (6) | 14–19 ms | 2.6–2.8 ms | 27 | 16.9 |
  | 219–228 (7) | 42–49 ms | 4.0–4.2 ms | 27 | 19.2 |
  | 270–293 (8) | 73–82 ms | 5.5–5.9 ms | 25 | 21.1 |
  | 319–365 (9) | 117–134 ms | 7.5–7.9 ms | 25 | 23.2 |
  | 374–412 (10) | 169–195 ms | 8.0–8.8 ms | 24 | 22.4 |
  | 498 (11) | 447 ms | 11.7–12.7 ms | 25 | 24.4 |
  | 595 (12) | — | 15.2–16.3 ms | 25 | 26.0 |
  | 662 (13) | — | 17.3–18.4 ms | 26 | 26.2 |
  | 742 (14) | — | 21.2–22.8 ms | 27 | 27.6 |

  Final build timed 2026-11-09 on a quiet machine (CPU load 4–9%), release, single core, the
  probe forcing the direct solve at every size; no fallbacks. **The 20-box pile** (`w6_pile_timing`,
  the slowest bounce step of each of five runs): 3.3–3.9 ms at 60 Hz, 3.2–3.7 ms at 240 Hz.
  **Landing gate (the Reviewer), met:** the pile at most 5 ms, and **`DIRECT_MAX` = 250**, the
  5 ms point (between 220 rows at 4.0–4.2 ms and 279 at 5.5–5.9). Cost grows about as rows^1.4.
  **A component above `DIRECT_MAX` falls back to Gauss-Seidel sweeping, which may not converge**
  (2,301 sweeps on 39 rows; the path by which the 10-box stack moved 24 mm). Each fallback is
  counted in `BounceStats::fallbacks` and `bounce_fallbacks_total()`. suites_w6final's largest
  component was 4 rows, so its results stand. The 2 ms target is W6c.
- **W6c (follow-up): the bounce passes under 2 ms on the 20-box pile,** whose real purpose is
  raising `DIRECT_MAX`: the case to aim for is the largest probe component, 742 rows (the
  105-brick pyramid; the 66-brick one is 498 rows), now 21–23 ms, within 5 ms, so large piles never reach the sweep. One ordering shared by
  both passes where their rows coincide; buffers reused across iterations; fewer iterations
  (25–27 on piles; the Mehrotra-style start above didn't cut them there). A bounce pass runs only
  on a step where something approaches faster than 1 m/s, so its cost is a one-off.
- **The 20-box pile** (the Reviewer's convergence test: a 6-5-4-3-2 brick pyramid, a box dropped
  1 m onto its top): it moves 0.34 mm across the bounce at 60 Hz and 0.045 mm at 240 Hz. b1ebad0
  moved it 13 mm and 6.8 mm, and didn't bounce the box (apex 1.086 m, rest 1.1). The Reviewer's
  bound was 0.1 mm. The remainder isn't the bounce passes: Gauss-Seidel, the active set and the
  interior point all give 0.34 mm, the exact answer of these rows. On the impact step the main
  solve (8 iterations) leaves brick pairs separating at up to 1.4 cm/s and sideways friction
  residue, and push-only support rows with fresh accumulators can't take a push back. Letting
  the support rows continue the main solve's accumulator (only the total ≥ 0) gives 0.27 mm:
  not enough to justify leaving the reviewed design. **Decided with the Reviewer:** the gate is
  1 mm at 60 Hz and 0.2 mm at 240 Hz, and the main solve's impact handling is W6b (3.6b); W6 can
  land before it.
- **Final regression (suites_w6final, 2026-11-08, the final solver, `DIRECT_MAX` 512; single
  core, `--no-fail-fast`):**
  - workspace: 178 passed, 0 failed, 44 ignored (suites_w6 had stopped at its one failure);
  - **bounce fallbacks to sweeping: 0** in every suite; the largest bounce component was 4 rows
    (cube tosses), 1 (fidelity), 0 (seams), so these results hold for any `DIRECT_MAX` ≥ 4;
  - fidelity: rubber restitution −0.15% at 60 Hz, −0.01% at 240 Hz; one case outside tolerance,
    the large swing (energy 0.9748 after 10 s, tolerance 0.02), owned by JOINT_CONDITIONING R5;
  - seams (line, ring), biped push sweep, cube tosses (measured params 26.4, fitted held out
    15.8): identical to suites_w6; 570 tosses in 1.0 s;
  - jointed scenes bit-identical to b1ebad0: 60 Hz d60114b0bb454180, 240 Hz a9ae3e4bacb3c45d
    (until the World fix, section 3.9: now eec23adca1f947fd and 5e213f5f9b6fe2c6);
  - **golden hashes** (`w6_scene_hashes`, the step hash's first 8 bytes):

    | scene | 60 Hz | 240 Hz |
    |---|---|---|
    | 20-box pile, 1 s after the drop | b6be817421a5936a | 1a15ce00411301fa |
    | box on a two-box stack, 3 s | 56a889bb6e73d42c | 90eac03965c85039 |
    | drone-like box flat from 2 m, 3 s | 75f5412b43fa9cc0 | a88414ba7682a6ed |

**Tests (`restitution_toi_test.rs`):**
- the fidelity bounce, rubber on rubber: apex within 1% of e² h at 60 and 240 Hz (gate 2%);
- drops from 0.3, 1 and 3 m with e 0.4 and 0.8: apex within 2% at 60 Hz, the second bounce's apex
  within 3% of e⁴ h;
- **the drone** (`drone.rs`'s aluminium box on its concrete ground, pair e 0.40) dropped flat from
  2 m at 60 Hz: rebounds to e² h within 3%; dropped tilted 20°, landing on an edge: no energy
  gained (kinetic plus potential after the bounce ≤ before);
- a box landing flat (4 contacts): all four bounce together, no spin picked up (|ω| < 1e-3);
- two free bodies colliding head-on in zero gravity (a = 0): the exchange matches the
  1-D closed form for e within 1%, at 60 Hz;
- jointed bodies are bit-identical: a hinged pendulum striking the floor faster than 1 m/s gives
  the same pose and velocity bits before and after (as `joint_bits_probe.rs`);
- a rollback across a bounce replays hash for hash;
- a box dropped 1 m onto a 20-box pile: the passes converge directly (no fallback, ≤ 64 solves),
  the pile moves under 1 mm at 60 Hz and 0.2 mm at 240 Hz across the bounce, residual
  < 1e-4 m/s;
- a box dropped onto a two-box stack: no bounce, support or pseudo row runs on a step without a
  contact approaching faster than 1 m/s (the Reviewer's rule), and the stack is still from 10 s
  on (the rattle's length swings from 4 s to 7 s with sub-micron changes; AT_REST_SPEC R5);
- the fallback forced (a direct limit of 8 rows): finite, no energy gained, bounded residual;
- a 1000:1 mass ratio, both ways: converges in under 64 solves, no fallback, no energy gained;
- the at-rest suite passes unchanged.

**Regression:** fidelity at 60 and 240 Hz (restitution should now pass at 60 Hz: 19/20 or 20/20,
the large swing being the other); the cube tosses, which bounce at e 0.125 and change (report
measured and fitted, held out); seams and the biped, expected bit-identical. Golden step hashes
change for scenes with bouncing free bodies.

**Estimate:** half a day, plus the cube-toss re-measure.

### 3.6b W6b: the main solve on impact steps (open; found 2026-11-08 building W6)

- **Found:** a box dropped 1 m onto a 20-box brick pyramid (`restitution_toi_test.rs`,
  `a_bounce_on_a_20_box_pile_converges`; `w6_bits_probe.rs`, `w6_pile_probe`). W6's bounce
  passes solve exactly, yet the pile moves 0.34 mm across the bounce at 60 Hz and 0.045 mm at
  240 Hz (gates 1 mm and 0.2 mm). b1ebad0 moved it 13 mm and 6.8 mm.
- **The cause is the main solve's, on the impact step:** its 8 iterations leave brick pairs
  separating at up to 1.4 cm/s, and sideways friction residue; box 19, beside the struck box,
  is lifted 0.27 mm. W6's push-only support rows (fresh accumulators) can't take a push back.
  Gauss-Seidel, an active set and the interior point all give the same 0.34 mm.
- **Tried:** support rows continuing the main solve's accumulator (only the total ≥ 0): 0.27 mm.
  Not taken; it leaves the reviewed design for little.
- **To do:** the main solve's handling of a hard impact on a pile (more iterations on impact
  steps, or friction in an exact solve of the struck component), gated by the pile test's bound
  back at 0.1 mm.

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

### 3.8 Results: W3a and W4b (built overnight 2026-11-07, committed 2026-11-08)

Both were reviewed by the RiftPhys Reviewer before the build, and their findings are folded in.
Builds and tests ran single-core while URIEL's holonomy run was going.

**W3a, joint damping and Coulomb friction** (`D6Axis::{damping, friction}`,
`World::set_d6_damping(joint, angular, axis, c, tau_f)`), velocity rows on any axis, free ones
included:
- **Damper:** a soft row, v + γλ = 0 with γ = 1 / (c dt), so the impulse is −c dt v_end:
  implicit, stable for any c. (On a compliant row this carried part of a static load; since
  W4c a compliant row's damping is XPBD's damping term in its position row, 3.4b.) Its per-step factor is 1 / (1 + c dt / I), a rate of
  ln(1 + c dt / I) / dt, which tends to c / I. A fitted c absorbs the difference; report
  c dt / I with any comparison to a paper's identified c.
- **Friction:** target 0, impulse within ±τ_f dt.
- A row exists only for a coefficient above 0; the setter turns negative and non-finite values
  into 0.
- **Rigid rows are bit-identical:** they take the old code path (γ = 0 branches it out). A hinge
  chain with a limit and a motor gives the same pose and velocity bits before and after, at 60 and
  240 Hz (`tests/joint_bits_probe.rs`).
- **Tests** (`joint_damping_test.rs`, 5):
  - decay rate within 0.5% of the discrete closed form and 2% of c / I, at c/I = 0.5 and 2 /s,
    60 and 240 Hz;
  - a friction-only hinge stops at I ω0² / (2 τ_f) within 2% and stays stopped;
  - friction only with c = 0, and NaN or negative coefficients, stay finite and inert;
  - c dt / I = 1e4 is monotone and stable;
  - damping plus a limit still stops at the limit.

**W4b, axis-only armature** (`D6Axis::armature`, `World::set_d6_armature(joint, axis, J_a)`,
`World::reset_d6_rotor`), as designed in 3.4:
- The rotor's speed is carried state (`CarriedState`, the step hash), stored as its own balance,
  ω_rotor − λ / J_a.
- **Found in the build:** a rotor started at the next step took the links' speed after that
  step's host torque, so the first torque bypassed the armature (1.7% at 60 Hz over 1 s). Rotors
  are now set from the links' current speed when the armature is set (`set_d6_armature`, or
  `add_d6_joint` with armature already on the joint), on `reset_d6_rotor`, and on
  `set_body_pose`. `remove_body` drops them.
- **Velocity writes keep the rotor, except in a reset** (the Reviewer's finding): pose then
  velocity is the natural reset order, so `set_body_pose` marks its joints' rotors pending, a
  `set_body_vel` on the same body before the next step restarts them again, and each step clears
  the marks. A host writing velocities every step keeps its armature; `apply_torque` never
  restarts a rotor. `restore_carried_state` clears the marks too: the restore's own
  `set_body_pose` calls set them, and a host velocity write before the next step would otherwise
  overwrite the restored rotor and diverge from the original run (the Reviewer's recheck; the
  rollback test now writes a changed velocity before each step, and fails without the clear).
- **Tests** (`joint_armature_test.rs`, 6):
  - a torque accelerates the hinge at τ / (I + J_a) within 1%, at 60 and 240 Hz, also when the
    host writes the links' velocity every step;
  - a pendulum's small-swing period is 2π √((I_pivot + J_a) / (m g d)) within 1%, with and
    without armature, at 60 and 240 Hz;
  - energy, rotor included, is kept over 10 s at 0.5 rad as well as without armature;
  - a restore replays hash for hash, and the rotor is in the hash;
  - a bar spinning at 3 rad/s, reset with `set_body_pose` then `set_body_vel(0)`, stays at rest;
  - a joint built with armature (`add_d6_joint`) counts the first torque.
- **Not built:** the impact check (rotor momentum against row impulses across an impact). It
  needs W4's Cassie model, so it goes with W4.
- The arm keeps its isotropic armature (decision D6).

**What's left for parity:** W6, the D6 split, the World fix and W3 are built; the arm's P1 and P3
are met (D8), and the drone's P1 with restitution. Both were reported on 2026-11-10. Next, W4
(Cassie; its data and URDF are in hand, a URDF loader isn't built, and its replay needs the
half-step start). D4 is still open.

### 3.9 D6 split: the arm's axis armature, as an option (drafted 2026-11-08)

**Decided (D6): keep today's arm as the default and add the axis armature beside it.** URIEL
trains on the default without anything changing, and can switch when it's ready.

- **`ArmConfig` gets `armature: ArmArmature`**, `Isotropic` (default: today's arm, bit-identical)
  or `Axis`.
- **`Axis`:**
  - the reflected inertia ARMATURE[j] moves to joint j's W4b rotor row,
    `set_d6_armature(joint, 0, ARMATURE[j])` (the arm's hinges are about joint-frame x);
  - the child link keeps today's inertia crosswise to the joint axis:
    `inertia += ARMATURE[j] (I − u uᵀ)`, with u the axis in the child's frame (fb.rot × x).
    `arm.rs:230` records why: armature about the joint axis alone left the light wrist links
    nearly massless crosswise, and wrist torques leaked into the neighbouring joints. So the
    crosswise inertia, and the conditioning, stay as they are;
  - the joint's total inertia about its axis is unchanged too. What changes is what that inertia
    resists: today it resists the child's absolute rotation about the axis (a parent joint's
    motion drags it); on the rotor row it resists the joint's relative rotation, which is how a
    geared motor behaves.
- **Teacher, observations and actions are unchanged;** nothing else in `riftphys-uriel` reads
  `ARMATURE`.
- **Tests (`arm.rs` tests):**
  - the default arm's step hashes over 2 s of a scripted torque profile are the same before and
    after (recorded by a probe before the change);
  - with the parent link held, a torque on joint j accelerates it at the same rate in both modes,
    within 1% (same inertia about the axis);
  - zero gravity, joint 1 driven, joint 2 free (both about z; joint 0 is about y, so it can't
    show this): joint 2's relative drift orders as the models predict,
    |Axis| < |no armature| < |Isotropic| (the Reviewer's replacement for a "within 1e-3 rad"
    test, which can't pass: the child's own inertia still resists absolute rotation);
  - `Axis` mode: a 10 s seam rollout runs, finite, and its tracking is reported beside the
    default's;
  - a rollback in `Axis` mode replays hash for hash (the rotors are carried state).
- **Order:** after W6.

**Built (2026-11-09), `arm.rs`, `seam.rs`, `riftphys-uriel-seam --armature isotropic|axis`:**

> **`Axis` is usable at its own iterations, (96, 6) (`ArmConfig::for_armature(Axis)`); the Reviewer
> approved D6 on 2026-11-09.** At 8 iterations it is still wrong by 18–30% on the teacher replay, and
> it hasn't been compared with a real arm.

- `ArmArmature::{Isotropic (default), Axis}` in `ArmConfig`. `Axis` adds `ARMATURE[j] (I − u uᵀ)` to
  the child and `set_d6_armature(joint, 0, ARMATURE[j])` on the hinge's free axis (the rotor row
  is built before the hinge's enabled check, so it acts there). Axis-mode seam scenarios end in
  `_axis`; the default's names, lines and hashes are unchanged.
- **Default unchanged:** the arm's step hashes over 2 s of a scripted torque profile
  (gravity compensation plus a sine per joint) are fbec063efe6f8343 at 1 s and bb1b6c157c0bb23d
  at 2 s, recorded by a probe before the change, debug and release alike; asserted after.
- **Inertia about each axis, two tests in place of the drafted "same rate in both modes":**
  - every other joint held (velocity motors), a torque on joint j for one substep: the rate
    matches each mode's own model (the links beyond moving as one; `Axis` adds the rotor of j
    only) within 0.03% for all six joints in both modes (gate 1%). The modes agree only where no
    joint beyond j turns about an axis along j's: Axis/Isotropic is 1.0000 at the wrist pitch
    and tool roll, 1.048 at the base yaw, 1.115 at the shoulder (the elbow, about z like the
    shoulder, and in part the wrist pitch (rolled 0.4 rad off z) add their armature about
    the shoulder's axis in `Isotropic`, nothing in `Axis`: a held rotor turns with its housing). So the draft's "within 1% in both modes" holds only for the last joints. This is physics, not
    error: a policy trained on `Isotropic` finds the shoulder 11.5% quicker to a torque in `Axis`
    (about 10% less inertia) and the base yaw 4.8% quicker;
  - the Reviewer's coupling test, as a model check: zero gravity, shoulder driven, elbow free,
    the rest held; one substep from rest; both joints within 0.05% of each model's 2×2 mass
    matrix (gate 1%). The elbow's lag per unit of shoulder motion orders as the Reviewer said:
    `Axis` 0.125, no armature 0.145, `Isotropic` 0.302;
  - the free arm, nothing held, a torque on each joint in turn: every rate within 0.07% of the
    model's 6×6 mass matrix (gate 1%) at 512 velocity iterations.
- **Convergence at 8 velocity iterations (before `Axis` got its own):** one substep from rest (joint rows cold),
  `Isotropic` lands within 1–4% of the model (18.5% for a torque on the base yaw) and `Axis`
  22–112% off: a soft rotor row beside the stiff hinge rows converges slowly. Held for 0.1 s
  (warm-started rows), both are within 1.3% (`a_held_torque_moves_the_free_arm_as_its_mass_matrix_says`,
  gate 2%). A cold start under a sudden large torque is the case to watch in `Axis` mode.
- **Seam rollout:** `an_axis_armature_arm_tracks_a_seam` (0.15 m of line, with DART noise): at 8
  iterations, default lateral mean 0.955 mm, p95 2.48; `Axis` 1.072, p95 2.66. It now asserts
  each run completes with lateral and height p95 under 4 mm (noise-free eval: 0.25).
- **The Reviewer's cold-substep check** (`d6_iteration_error_probe`, `d6_teacher_replay_probe`,
  `d6_sine_contact_probe`, all ignored): open loop, the arm at its own iterations against the
  same arm at 512, worst joint-rate error over 1 s as a share of the largest rate.

  | input | `Isotropic` at 8 | `Axis` at 8 | at 32 | at 64 | at 96 |
  |---|---|---|---|---|---|
  | (b) random torque each tick, ±0.2 limit | 6.06% | 10.10% | 4.18% | 1.50% | 0.52% |
  | (c) seam teacher's first second, 3 episodes | 0.87–1.80% | 18.6–30.1% | 2.4–3.5% | 1.6–3.8% | 0.53–0.87% |
  | (a) sine profile, before the first self-contact (22 ticks) | 0.01% | | | | 0.27% |
  | (a) sine profile, the full 1 s, before the World fix | 55.73% | diverges | diverges | diverges | diverges |
  | (a) sine profile, the full 1 s, with the World fix | 55.73% | 78.34% | 77.22% | 71.07% | 57.12% |

  - **Ordering didn't help** (rotor rows after the hinge rows: the same as built; before them:
    worse, 100–146% on (c)), so `Axis` gets more iterations, per the Reviewer's rule:
    `ArmConfig::for_armature(Axis)` is (96, 6). The default stays (8, 6) and its hashes hold.
    Error roughly halves per doubling up to 96, then flattens at about 1–2% (128: 1.0–1.7%), the
    open-loop measure's own floor.
  - **`Axis` fails on (a): it diverges.** The sine profile drives the arm into a self-contact
    (wrist_1 on the tool) at tick 23. Within 10 ticks the tool roll rate is about 1e25 rad/s, at
    96 and at 512 iterations alike, so it isn't convergence. `Isotropic` passes through the same
    contact and stays at 93 rad/s, though its trajectory then departs from the 512-iteration one
    (hence 55.73%). The tool link's own inertia about its axis is 1.9e-5 kg m² against 0.01
    crosswise, a 500:1 needle.
  - Gated: `a_teacher_replay_converges_at_the_arms_own_iterations` (input (c), 2%, both modes).
  - **Cause, found by the Reviewer (2026-11-09):** the World's gyroscopic turn on a thin link
    (`step.rs:229-239`), not `Axis`, the rotor, the contact or convergence. The turn conserves
    world L and reads w = I(q)⁻¹L, so a turn error projects crosswise L onto the 1.9e-5 axis,
    about 500× amplified, and the joint rows feed it back every substep. Largest joint rate over
    1 s, in the Reviewer's scratch copy:

    | variant | `Axis` at 96 | `Axis` at 512 | first contact |
    |---|---|---|---|
    | as built | 2.8e25 | 1.5e31 | tick 23 |
    | gyroscopic turn off | 86 | 86 | tick 23 |
    | turn subdivided | 84.6 | 85.8 | tick 23 |
    | rotors zeroed, as built | 1.7e29 | 5.9e27 | none (blew up by substep 30) |
    | rotors zeroed, turn off | 3,092 | 3,497 | tick 14 / 11 |
    | rotors zeroed, subdivided | 3.8e30 | 10,681 | tick 19 / 17 |

  - **World fix (built 2026-11-09, `step.rs`, its own change):** the turn subdivides into n pieces,
    n = ceil(gain / 0.5), at most 256, each the existing midpoint at dt / n. The gain is
    dt |dev(I⁻¹)| |L|, with dev the deviator of I⁻¹, so it's zero on an isotropic body. A spin
    principal to roundoff (|L × ω| below 1e-5 |L||ω|, as a rod swinging flat) keeps n = 1, which is
    the old midpoint bit for bit. Tests: `a_free_needle_keeps_its_momentum_energy_and_roll`
    (undivided, roll moves by 66 rad/s and energy rises 8.6%; subdivided, 0.53 rad/s and 0.02%, and
    0.009 rad/s from a nearly clean spin of 0.001 rad/s of roll),
    `an_axis_arm_stays_bounded_through_a_self_contact` (`Axis`: 84.6 rad/s at 96 iterations, 85.8
    at 512; gate 200; 0.34% before the contact, gate 2%).
  - **What moved:** identical before and after: the default arm's hashes (fbec063e / bb1b6c15),
    `joint_bits`, W6's six scene hashes, seam JSONL (line and ring, SHA-256), the 570 cube tosses.
    `w6_jointed_bits` moved (60 Hz d60114b0bb454180 → eec23adca1f947fd; 240 Hz a9ae3e4bacb3c45d →
    5e213f5f9b6fe2c6). That scene is 1 m rods (156:1) rattling on a floor in 3D, where the undivided
    turn had left its contraction range (its gyroscopic-torque gain reached 10.6, above 1 on 22
    steps), so the old bits weren't converged. argref re-records at the bump.
  - **Golden hashes are release-build values.** The 240 Hz pile hash in the W6 table (1a15ce00411301fa)
    is what release builds print; debug prints d3bdc0fa3f0257cf, at 1 and 4 threads, on eee2e57 and
    with the World fix alike (the Reviewer's check). The other five agree. **Open, separate:** the
    same source giving different bits by profile needs explaining. Look first at anything gated on
    `debug_assertions` and the unsafe shared-mutable access in `solver.rs`; find the first step
    whose hash differs and which body it is. Owner: RiftPhys Orchestration.
  - **Re-measured on the fixed build:** (b) and (c) don't change (table above). (a), the full 1 s:
    `Axis` no longer diverges; it reads 57–79%, like `Isotropic`'s 55.73%, because the motion after
    the contact is chaotic; before the contact, 0.34% at 96 iterations. **(96, 6) stands:** 64
    misses 2% on (c) (up to 3.78%), 96 gives 0.53–0.87%, 128 no better.
  - **Open, separate from D6:** the rotors-zeroed arm (a chain of thin links with no armature)
    diverges with the fix too: 5.5e22 rad/s at 8 iterations, 7.7e28 at 96, 2.0e30 at 512
    (`d6_rotors_zeroed_probe`). Any imported model could be that. No root cause yet. Owner:
    RiftPhys Orchestration.
- **Seam eval at 96 iterations** (`riftphys-uriel-seam --eval --seam line|ring --armature
  isotropic|axis`, 10 held-out episodes per joint type, seed 1000): every `_axis` scenario is
  within 0.007 mm of the default's on lateral and height, mean and p95, teacher and floor alike;
  all teachers complete 10/10, the floors 6/10 (line) and 8/10 (ring) in both modes. Feedback
  hides plant error, so this shows the teacher's gains carry over, not that the plant is right.

  | scenario | teacher lateral mean / p95, default | `_axis` |
  |---|---|---|
  | line butt | 0.108 / 0.263 mm | 0.108 / 0.256 mm |
  | line v | 0.050 / 0.084 | 0.051 / 0.085 |
  | line fillet | 0.054 / 0.199 | 0.054 / 0.199 |
  | ring butt | 0.102 / 0.239 | 0.101 / 0.238 |
  | ring v | 0.032 / 0.053 | 0.032 / 0.052 |
  | ring fillet | 0.033 / 0.058 | 0.033 / 0.058 |

- **Rollback:** `an_axis_arm_rolls_back_hash_for_hash` (30 ticks, 120 step hashes).

## 4. Order and reporting

**Order:**
1. W2.
2. W1, then the biped re-check.
3. W6 (required by D3, 2026-11-08), then the D6 split (section 3.9).
4. W3a, then W3. This gives arm parity: report to URIEL with the commits. (W3 built, 3.3; P3 met under D8.)
5. W4b, W4a, W4. This gives biped parity: report to URIEL with the commits.
6. W5 only if triggered.

W1, W2, W3a, W4b, W6, the D6 split and W3 are built. Arm and drone parity were reported on
2026-11-10 (PHYSICS_FIDELITY.md section 4). The Designer ruled on the argref bump the same day: "Wait for
biped". The pin stays at 2b98bb3 and arm and drone training wait; the bump goes to the Designer
with the full report once the biped reaches P4 and P5. Biped parity needs W4a and W4 (P4); P5's fixes
(W1, W2) are built, and its lever-arm part waits on whether P4 shows it.

**Each parity report:**
- is a new section in `docs/PHYSICS_FIDELITY.md`;
- is sent as a message to URIEL Orchestration;
- names every commit since argref's pin (2b98bb3) that the bump should include, at least
  22be79e (compliant contact), c0ef751 (SHAKE joints), and the W1, W2, W3a and W4 commits;
- includes any golden-hash changes.

## 5. Decisions for the Designer

**Answered 2026-11-08 (Designer), in short:** D1 per workload; D2 as proposed, aiming to beat
the references; D3 W6 is required and goes first; D4 open (the Designer asked which Cassie this
is: it is Agility Robotics' bipedal robot, see D4); D5 sizes below, go-ahead pending; D6 split
(section 3.9).

- **D1:** per-workload parity, so arm training resumes after P3 without waiting for P4. Or one
  bar for everything.
  - **Decided 2026-11-08: per workload.**
- **D2:** the comparison rule: no worse than the best reference engine by more than 2 SE.
  Alternatives: strictly better on means; or an absolute tolerance per metric.
  - **Decided 2026-11-08: the 2 SE rule is the pass; beating the best reference is the aim.**
    Each report says, per metric, whether RiftPhys beats, ties (within 2 SE) or trails it.
- **D3:** the drone at 60 Hz. Is excluding restitution acceptable while drone scenes have no
  bouncing contact? If not, W6 becomes required.
  - **Decided 2026-11-08: not acceptable, "if a drone falls it should bounce".** W6 is required
    and comes first (section 3.6, section 4).
- **D4:** P4 against Drake's 38.0 or MuJoCo's 30.0. The plan uses Drake: it's the engine with the
  closest contact model, and MuJoCo's soft constraints are tuned differently.
  - **Open.** "Cassie" here is Agility Robotics' Cassie, the bipedal robot, in the DAIR Lab's
    jumping-impact dataset (Acosta et al., "Validating Robotics Simulators on Real-World
    Impacts", RA-L 2022), not the CASSI spectral-imaging benchmark. Proposed with D2's aim: pass
    at Drake's 38.0, aim for MuJoCo's 30.0.
  - **Reframed 2026-11-10 (3.4a, the Reviewer's finding):** neither number can be the bar. Both
    are on the paper's windows, which aren't public; the public windows start up to about 40 ms later.
    The proposal is now: P4 passes when RiftPhys is no worse than MuJoCo rerun on the public
    windows, per jump under D2's rule, both fit on the same folds (needs D10). 38.0 and 30.0 are
    reported as context.
  - **Decided 2026-11-10 (Designer: "Yes, accept"):** as reframed. P4's bar is MuJoCo rerun on
    the public windows, paired per jump under D2, judged at 240 Hz (D9).
- **D9:** the physics rate for P4.
  - **The options:** 240 Hz, the biped's workload rate (12 steps over the window; the 2 kHz
    samples are compared against states interpolated between steps), or 2 kHz, the data's rate.
    Drake ran at 12.5–20 kHz.
  - **The handicap, named:** compliant rows damp themselves (3.4b). Cassie's leaf springs (knee
    k 1500, ankle 1250 N m/rad) were estimated at ω dt ≈ 0.7 at 240 Hz. Measured on the built
    model, hung (3.4a), they ring in two modes per leg at 8.5–23.5 Hz: ω dt 0.22–0.62 at 240 Hz,
    where implicit Euler keeps (1 + ω²dt²)^(−π/(ω dt)) ≈ 0.19–0.51 of the amplitude per period;
    0.03–0.07 at 2 kHz. The swing-leg periods (43–118 ms) are as long as or longer than the
    window. Standing (3.4a, on a vertical guide), the dominant mode is the body's bounce at
    2–2.5 Hz, ω dt 0.05–0.07 at 240 Hz, keeping 0.82–0.85 per period. MuJoCo's springs aren't damped
    numerically there, so the paired 240 Hz comparison shows it directly. The URIEL biped has no
    compliant rows, dampers or armature, and 240 Hz is already its rate: the handicap comes
    from a feature the biped workload doesn't use today.
  - **Proposed (the Reviewer's wording):** pass at 240 Hz, the workload rate (D1), with 2 kHz
    reported beside it, and what each outcome means fixed now, before any fit:
    - (a) passes at 240 Hz: a pass;
    - (b) fails at 240 Hz, passes at 2 kHz: a fail at the workload rate. It's put down to the
      compliant rows' integrator damping only if a 240 Hz run with the leaf springs made rigid
      (or their ω dt shown irrelevant) closes the gap. A follow-up item for a less dissipative
      compliant row is logged, and it isn't reported as parity;
    - (c) fails at both: model mismatch or the engine, investigated as usual.
  - **Decided 2026-11-10 (Designer: "Yes, accept"):** as proposed: the pass at 240 Hz, 2 kHz
    reported beside it, and outcomes (a)–(c) as written above, fixed before any fit.
- **D10:** downloads for W4.
  - **Needed:** Agility's MuJoCo model, as the paper used it: github.com/osudrl/cassie-mujoco-sim
    `model/cassie.xml` (22,344 bytes) and `model/cassie-stl-meshes/` (11 STL files, 2,049,524
    bytes), 2.07 MB in all.
    - The fork has no licence file. MuJoCo Menagerie states that Agility released the original
      under MIT.
    - Menagerie's own `agility_cassie` (MIT, 7.8 MB) changes the collision geometry and the
      solver, so it isn't the paper's model.
  - **Optional:** the paper's six loss-weight pickles from the dairlib branch (6 files, 37,263
    bytes, MIT). They would be read with `pickletools` only, never unpickled, to confirm the
    position and orientation weights.
  - **Without it there is no valid P4 bar** (D4, reframed).
  - **Downloaded 2026-11-10 (Designer: "Yes, download"):** `model/cassie.xml` and
    `model/cassie-stl-meshes/` from osudrl/cassie-mujoco-sim at commit 188d3d7, into
    `riftphys-parity-data/cassie-mujoco-sim/` (outside this repository; SHA-256 in its
    SOURCE.txt). It came to 14 files, 2,242,036 bytes, not the 2.07 MB quoted: the first listing
    was cut short and missed two meshes that `cassie.xml` references (`achilles-rod.stl`,
    `foot-crank.stl`; 170,168 bytes). Every file matches the host's listed size. The loss-weight
    pickles aren't downloaded.
  - Nothing else is needed: the per-jump `drake_optimal` and `mujoco_optimal` runs are on the
    raw windows (above).
- **D5:** downloads: the double pendulum (encoder data and CAD) and the Cassie dataset. Each needs
  the user's go-ahead with its size, given when asked.
  - **Sizes (read from the hosts' file listings 2026-11-08, nothing downloaded):**
    - double pendulum, Mendeley Data 7yd2ntbh3w v1 (CC BY 4.0): `Encoder_Data.zip` 1.63 MB,
      `Double_Pendulum_CAD.zip` 23.4 MB; also listed and not planned: `Video_Tracking_Data.zip`
      7.36 MB (tracked positions, a cross-check for the encoders) and three small scripts (61 KB);
    - Cassie, github.com/DAIRLab/cassie_impact_dataset (BSD-3): 96 files, 1.32 MB (87 `.npy`
      arrays, legends, a loader), not "under 1 MB" as section 3.4 says;
    - Cassie's model, `cassie_v2.urdf` from github.com/DAIRLab/dairlib (BSD-3): 24 KB. Its
      meshes (42 files, 3.1 MB) aren't needed: the feet are modelled as the paper's contacts.
  - **Cassie downloaded 2026-11-09 (Designer: "Download Cassie"):** the dataset (main branch,
    96 files, 1.3 MB) and `cassie_v2.urdf` (dairlib master, 24,071 bytes), in
    `riftphys-parity-data/cassie/`, outside this repository.
  - **Double pendulum downloaded 2026-11-09 (Designer: any download in the spec):**
    `Encoder_Data.zip` (1,629,108 bytes) and `Double_Pendulum_CAD.zip` (23,404,393 bytes), both
    matching Mendeley's SHA-256, extracted in `riftphys-parity-data/double_pendulum/`. Then
    (Designer: "go ahead on video data and the 3.1 MB of meshes"): `Video_Tracking_Data.zip`
    (7,362,421 bytes, SHA-256 matches) and Cassie's meshes from dairlib
    `examples/Cassie/urdf/meshes/` into `riftphys-parity-data/cassie/meshes/`. The meshes came
    to 92 files, 15.7 MB, not 3.1: the 3.1 MB counted only the top folder, and its `agility/`
    subfolder (which `cassie_v2.urdf` references for ten links) holds the rest. The three
    scripts aren't downloaded.
- **D6:** the F1 arm's armature. Today it is added isotropically to each child link's inertia
  (`arm.rs:230`). Once W4b exists, should the arm use the axis-only row (the physical model)?
  That changes the arm's dynamics and the teacher's behaviour, so argref would re-record. Or
  keep the arm as is. Note: position drives and compliant rows act at position level, after the
  rotor is stored, so they feel J_a only through the next step's catch-up: a step late, with
  some energy loss. The arm uses torques today, but a position-driven arm would see this.
  Cassie's actuated joints are torque-driven, so W4 isn't affected.
  - **Decided 2026-11-08: split.** Keep today's arm as the default and add the axis armature
    as an option, so URIEL can train on the current model without breaking what works now.
    Design in section 3.9.
- **D7:** which MuJoCo is P3's reference? The plan extended the cube-toss script (MuJoCo's
  default, Euler), while the comparison rule says "best published or reproduced engine's".
  - **Decided 2026-11-09: RK4, the stricter.** P3's reference is MuJoCo RK4 at 240 Hz, the same
    rate as RiftPhys. Euler is still reported beside it.
- **D8:** does the half-step start count for P3 (section 3.3, second method)? It was added after the
  first result, so the first table stands either way. With it, RiftPhys is no worse than RK4 at
  every horizon, in both damping basins; without it, worse at 0.1 and 0.25 s. It corrects the
  initial condition (no free parameter, no engine change); the same correction would apply to
  every replay from measured velocities.
  - **What a yes would mean:** trajectory error up to 1 s from a measured state, on two held-out
    runs of one rig, is no worse than MuJoCo RK4's. The sign differs between the two runs at
    0.5 s in both basins, and at 1 s in the viscous one.
  - **What it wouldn't mean:** damping isn't identified by this benchmark. RiftPhys's own
    second-method fit decays with t50 6.6–7.0 s against 12.9–13.8 measured, so these fitted values
    shouldn't be reused elsewhere. Energy decay is reported and neither engine matches it.
  - **Decided 2026-11-09: yes, count it.** P3 is met with the half-step start; the first table
    stays beside it.
  - **So:** section 4 makes this arm parity for the Isotropic default at (8, 6), the arm URIEL
    trains on and the iteration count P3 used; `Axis` stays a separate option. The report to URIEL
    carries these limits and the open debug/release hash item (section 3.9).
