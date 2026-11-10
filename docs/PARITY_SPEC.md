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
- **Torques:** applied from the measured motor torques through the gear ratios. There's no
  controller, so the replay is open loop over 50 ms.

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

**Pass (P4):** RiftPhys's loss with fitted contact parameters is ≤ 38.0 (Drake) on the same
22 jumps and metric. If per-jump losses can be reproduced, use the comparison rule against
MuJoCo's 30.0 instead.

**Estimate:** about 1–2 weeks: model and loader 3–4 days, armature 1 day, replay and fit
2–3 days, debugging the closed loops and springs the rest. The biggest risk is model mismatch,
which the paper itself names as the limit for Drake and MuJoCo.

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
  implicit, stable for any c. Its per-step factor is 1 / (1 + c dt / I), a rate of
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

**What's left for parity:** W6 first (D3), then W3 (double pendulum) and W4 (Cassie), which wait
on the downloads (D5); D4 is still open.

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
4. W3a, then W3. This gives arm parity: report to URIEL with the commits.
5. W4b, W4a, W4. This gives biped parity: report to URIEL with the commits.
6. W5 only if triggered.

W1, W2, W3a and W4b are built. Drone parity needs W6 (P1 at 60 Hz, restitution included).

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
