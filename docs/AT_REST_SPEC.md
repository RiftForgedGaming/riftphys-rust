# At rest: stacks that creep, feet that slide

Status: proposed 2026-11-07, reviewed the same day (4 medium, 6 low findings, folded in). Built
2026-11-07, uncommitted: R1 for contacts **and joint rows**, and D2's snapshot carry. R2 was built,
measured and set aside: see section 2.4, which changes the diagnosis in 2.2. This covers the two
tickets PRECISION_SPEC 3.5 left: stack creep in the contact solver, and the standing biped's drift.
Both fail at HEAD with the grid on too; neither comes from P1.

Diagnostics: `crates/riftphys-world/tests/rest_diag.rs` (`stack_creep_variants`),
`precision_diag.rs` (`at_rest`) and riftphys-uriel's `standing_still_drift` and
`push_sensitivity_20`. All are ignored tests; run them with `--ignored --nocapture`.

## 1. Summary

- **Stacks creep because the contact solver starts cold every step.**
  - `solve_islands_parallel` sets every contact's normal and friction impulse to zero before its
    iterations.
  - `World::warm_cache` exists but nothing fills or reads it: it is dead code from an early commit.
  - With 20 velocity iterations, a 3-box stack is not converged at the end of a step. What's left
    shows up as tilt and sideways slip, so the stack walks off and falls in 2–9 s.
  - A warm-start prototype keeps 3-, 5- and 10-box stacks standing for 60 s at 60 and 240 Hz.
- **The biped's feet slide because the position pass has no friction.**
  - The joint solve corrects each joint by moving both bodies, and the 1 kg foot moves most.
  - Contact projection pushes only along the normal, so nothing holds the foot sideways. On a
    jointed body, step 6 then turns the whole correction into velocity.
  - Over 60 s standing still, the position pass moved a foot 2.82 m; the velocity solve's friction
    pushed back 0.36 m.
- **They have to land together.** Warm starting alone makes the biped worse: push recovery at
  20 N·s falls from 17/20 to 9–10/20, and the standing drift grows from 2.5 m to 3.5 m.
- **The box-box narrowphase gaps the RiftPhys Reviewer found** (no face clipping, a 5 cm grow) are
  the third item. They make yawed stacks topple whatever the solver does.

## 2. Diagnosis

### 2.1 Stacks (`stack_creep_variants`)

Three rubber boxes 0.4 x 0.2 x 0.4 m, 5 kg each, on a static concrete ground body, started 2 mm
apart and kept awake for 30 s. The table gives the time at which any box is first 2 cm off its
column, at 60 / 240 Hz.

| variant | at HEAD (15c12cf) | with warm start (prototype) |
|---|---|---|
| base, iterations (20, 20) | 5.03 s / 7.25 s | never / never; worst 1 mm / 0 |
| 2 boxes | never / never; worst 7 mm / 6 mm | never / never; worst 0 |
| restitution 0 | 5.03 / 7.25 | never / never |
| frictionless | 1.68 / 2.55; slides tens of metres | 9.12 / never |
| μ = 5 | 2.37 / 7.23 | never / never |
| patch friction | 8.57 / 11.23 | never / never |
| started touching | 2.70 / 8.58 | never / never |
| iterations (60, 20) | never / never | never / never |
| iterations (20, 60) | 5.02 / 7.25 | never / never |
| iterations (120, 20) | never; worst 2 mm / 0 | never / never |

How to read it:
- **Velocity iterations decide it, not position iterations.** (60, 20) holds; (20, 60) doesn't.
- **Restitution, friction and patch friction don't matter.**
- **Frictionless stacks slide tens of metres.** A level stack has no lateral load, so all of that
  is unconverged normal impulse: the 4 corners of a face carry unequal impulses, the box tilts,
  and the next step's normals have a sideways part.
- **At HEAD the boxes rest 5 mm deep** (y 0.095 for a box whose centre belongs at 0.100), the
  whole slop. With warm starting they rest at 0.100.

`at_rest` (wood boxes, 1 cm start gaps, 60 s, top box's movement after settling 1 s):

| | HEAD | warm start |
|---|---|---|
| 3 boxes, 60 / 240 Hz | falls / falls | 1.3e-5 m / 8.7e-9 m |
| 5 boxes | falls / falls | 2.2e-6 m / 1.4e-8 m |
| 10 boxes | falls / falls | 5.9e-3 m / 2.6e-5 m |
| slope, 20° | 3.6e-9 m / 0 | 2.0e-8 m / 0 |

**The prototype** (not committed) worked as follows:
- After the solve, it stored each rigid contact's collider pair, world point, normal impulse and
  friction impulse as a world vector.
- Next step, each new contact took the stored contact of the same pair whose point was nearest
  and within 1 cm. It started from that normal impulse, and from the friction vector projected on
  its new tangents, clamped to μs·jn. Both were applied before the iterations.
- Patch-friction contacts warm-started only the normal.

### 2.2 The standing biped

`standing_still_drift`: the biped with no push and no lean, under `StandPd`, 60 s.
- **It never falls, but its pelvis wanders** 2.04 m at f329733 and 2.46 m at 15c12cf.
- **The feet slide at 4–5 cm/s in −z, steadily, both together.** The torso holds a fixed lean of
  −0.039 rad, and its pitch rate is zero.
- **Rubber on concrete (μs about 0.95) would need several hundred newtons** of sideways force to
  slide a loaded foot. The controller isn't producing that.

**Neither iterations nor projection change it** (pelvis drift in 60 s):
- (8, 6), the biped's default: 2.40 m;
- PBD projection instead of SHAKE: 2.39 m;
- (8, 20): 2.26 m;
- (40, 6): 1.62 m;
- (40, 6) with warm start: 0.50 m. Better, but still sliding.

**Where the slide comes from.** A throwaway probe in `step.rs` summed foot 0's z displacement
over 60 s:
- the integration step (velocity × dt, after the velocity solve and its friction): +0.36 m;
- the position pass (joint solve and contact projection): −2.82 m.

So the velocity solve's friction resists, and the position pass beats it.
- **The joint solve's corrections are mass-weighted,** so the light foot takes most of them.
- **`project_contacts` corrects only along the normal.**
- **Step 6 turns the position change into velocity.** For jointed bodies it doesn't subtract the
  split (the note there explains why), so the foot carries that velocity into the next step.

### 2.3 Box-box narrowphase (the RiftPhys Reviewer, 15c12cf review)

- **There is no face clipping.** Candidates are each box's corners that lie inside the other box,
  grown by the contact distance.
  - If two boxes are yawed so that no corner of either lies inside the other, the pair gets a
    single edge-edge point at the centre.
  - A 3-stack whose middle box is yawed 45° topples in 1.6–2.6 s; at 3° it topples at 60 Hz
    (2.1 s at 15c12cf, 6.7 s at HEAD).
- **The grow** (contact distance + 1e-4, about 5 cm with the CCD margin) accepts corners up to
  that far outside the other box's face. Contacts can sit over empty space, and a box hanging off
  a ledge is held up past the edge.
- `reduce_to_4` picks the outermost candidates, so it prefers exactly those points.

### 2.4 What building R1 and R2 found

**R2 as specified makes the biped fall.** Standing still for 60 s (`standing_still_drift`), pelvis
drift:

| build | pelvis | outcome |
|---|---|---|
| R1 (contacts) | 3.60 m | stands |
| R1 + R2, budget μs·(jn_vel + jn_pos) | 2.9 m | falls at about 23 s |
| R1 + R2, budget μs·jn_vel only | 0.41 m | stands |

**Why.** A probe summed each step's impulses over the 8 foot contacts. Every step:
- the velocity solve's friction was 0.6–1.0 N s against a normal impulse of 1.8 N s, at 240 Hz;
- R2's tangential corrections added about as much again;
- so the feet sat near their friction limit.

A biped standing still shouldn't load its feet sideways like that. The load came from the joint
solve, not the contacts.

**The joint rows start cold too.** `solve_islands_parallel` zeroes every D6 row's accumulator
before its iterations, as it did the contacts.
- With 8 velocity iterations the joint rows don't converge under gravity's load.
- The position solve corrects what's left every step. Its corrections are mass-weighted, so the
  1 kg feet take most of them.
- Holding the feet harder (R2) only moves that correction into the legs and pelvis.

Warm-starting the joint rows as well (now part of R1, below), 60 s standing:

| build | pelvis (2–60 s) | pelvis (10–70 s) | feet (2–62 s) |
|---|---|---|---|
| R1, contacts and joints | 4.9 mm | under 1 mm | 0.044 mm |
| + R2 | 4.9 mm | | 1.0 mm |
| + R2, slip deadband 1e-7 / 1e-6 m | 4.9 / 5.0 mm | | 5.4 / 3.2 mm |

- **The pelvis's 4.9 mm is posture settling.** It's done by about 10 s; from 10 s to 60 s the
  pelvis moves about 2e-6 m.
- **R2 now makes resting worse.** Its corrections on a resting contact are at float precision,
  1e-8 to 6e-8 m. Each one renormalizes the body's quaternion, and the next pass sees the rounding
  as fresh slip. A wood box on a 20° slope creeps 6.4e-5 m in 60 s with R2, against 1.9e-8 m
  without it. Deadbands moved the error around without removing it.
- **R2 is set aside, not built.** The patch is kept; it can return if a case shows sliding the
  velocity solve can't hold.

**Push recovery at 20 N s** (`push_sensitivity_20`, 20 seeds, as drawn / one tick late / x0.99 /
x1.01):

| build | stood 10 s |
|---|---|
| HEAD (15c12cf) | 17 / 17 / 17 / 17 |
| R1, contacts | 9 / 9 / 10 / 9 |
| R1, contacts and joints (built) | 11 / 11 / 11 / 10 |
| + R2 | 13 / 14 / 15 / 11 |

At 30 N s the built R1 stands 4/20, HEAD 7/20. HEAD's feet slid at 4–5 cm/s, so it absorbed pushes
by sliding. That is D1, settled by retuning `StandPd`'s balance gains: 19/20 at 20 N s, 8/20 at
30 N s (section 5).

**The 10-box stack at 240 Hz** moves 0.40 mm in 60 s, against R1's 0.1 mm gate.
- The prototype's 2.6e-5 m came from storing jn after the shock pass. Stored that way, the 10-box
  stack reaches 2.6e-5 m, but the 5-box stack at 60 Hz rises to 0.26 mm.
- The build keeps the spec's rule, before the shock pass, and gates the 10-box stack at 1 mm.

## 3. Work items

### R1. Contact warm starting

- **What to store.** After each solve, for each rigid contact:
  - its key, (body A, body B, collider ordinal within A, collider ordinal within B);
  - its anchor in body A's frame (and B's);
  - its normal impulse;
  - its friction impulse, as a world vector.

  This replaces the dead `warm_cache` / `WarmImp`.
  - **Keys use body ids, not collider indices.** `remove_body` drops colliders with `retain`,
    which shifts every later collider index. Body ids are never reused: `Bodies::add` appends, and
    `remove_body` only deactivates.
  - **The pair's order is fixed.** Body pairs always come as `a_collider < b_collider` (the pairs
    are sorted), and terrain is always b; `cull_and_orient_contacts` flips normals, not colliders.
    Assert that order rather than matching either way.
- **When to store jn: after the iterations, before the shock pass.**
  - The shock pass (`solver.rs` 3a) recomputes m_n with the lower body frozen.
  - In the impact it exists for, such as 1000 kg on 1 kg, its jn on the light body's contacts is
    far larger than a solve with real masses gives. Warm-starting that would kick the bodies next
    step.
  - Restitution comes after the shock pass, so a bounce isn't stored either.
- **Matching.** Each new contact takes the stored contact with the same key whose A-frame anchor
  is nearest, within a tolerance. Start at 1 cm; for boxes the anchors are corners, so 1 cm is far
  below any feature spacing.
  - Each stored contact is used at most once.
  - Ties go to the lowest index.
- **Warm start before the iterations:**
  - the normal at the stored jn;
  - friction at the stored vector projected onto the new tangents, clamped as the solver clamps it:
    the μ from `mu_dynamic` at the contact's current slip, times jn;
  - patch friction warm-starts its patch accumulators from the patch's stored values;
  - compliant contacts are untouched; they already apply their force once.
- **Determinism.** Matching runs serially, in contact order. Warm impulses are applied serially, in
  constraint order, before the batched iterations, never through `run_batch`, because float sums
  depend on order. With that, replay is exact.
- **Removing a body** drops only the entries whose key holds that body.
  - Today `remove_body` clears the whole cache (`world.rs:504`).
  - Under warm starting, a scene that despawns debris every few steps would keep cold-starting
    every stack in the world.
- **Clearing the whole cache** stays on epoch and gravity swaps and `clear_solver_caches`, but not
  snapshot restore (D2).
- **`set_body_pose` drops that body's entries.** Anchors are matched in A's frame, so a teleported
  box's corners would still match its old entries and be warm-started with the old place's load.
- **`set_body_vel` doesn't drop entries.**
  - Changing velocity doesn't change contact geometry.
  - Several callers set velocity every step: the harness's `SetVelocity` input, `bench_loco`'s
    feet, and FFI hosts. Dropping entries there would switch warm starting off for exactly the
    driven bodies.
- **Sleep.** Keep entries for pairs where both bodies are asleep; don't rebuild the cache only
  from contacts solved this step. Otherwise a sleeping stack loses its entries and wakes cold.
  - As built, sleeping pairs still get contacts every step (the narrowphase doesn't skip them),
    and those re-store their matched entries. So the retention only matters if that changes;
    removing it fails no test today.
- **Joint rows (added in the build, section 2.4).**
  - Each D6 velocity row's accumulated impulse is stored per joint index after the iterations.
  - Next step, if the joint has the same rows, each row starts from its stored impulse, clamped
    to its bounds, and the impulse is applied before the iterations.
  - "The same rows" means the same tags in order. A tag is angular or linear, the axis, and the
    kind: equality, motor, lower limit or upper limit. Changing a joint's limits or drives
    changes its tags, so it starts cold.
  - Applied serially, in joint order, before the contacts' warm start.
  - `World::warm_matched()` counts the contacts each step started warm, for tests.
  - Joints not solved this step keep their entries.
  - `set_body_pose` and `remove_body` drop the entries of joints on that body.
  - Distance joints have no accumulator in the velocity solve; they're unchanged.

Built: `World::warm`, `warm_patches` and `warm_joints` (`world.rs`), and the warm start and store in
`solve_islands_parallel` (`solver.rs`). The dead `warm_cache` / `WarmImp` are gone.

Acceptance (60 and 240 Hz, kept awake, 60 s):
- **Stacks:** 3 and 5 boxes move under 0.1 mm. 10 boxes stand, the top moving under 1 cm at
  60 Hz and under 0.1 mm at 240 Hz. The prototype read 5.9 mm and 2.6e-5 m.
- **`stack_creep_variants`:** no variant at (20, 20) crosses 2 cm in 30 s, except frictionless
  at 60 Hz, which is reported.
- **Boxes rest at their true height,** within 1 mm, not one slop deep.
- **`stacked_boxes_hold` runs for 30 s** with its 2 cm gate. The Reviewer's 30–60 s advice.
- **Despawns elsewhere:** a 5-box stack holds 60 s while an unrelated body is added and removed
  every 10 steps.
- **Sleep:** a 5-box stack falls asleep, is nudged awake, and stands 60 s more.
- **Heavy on light:** a 1000 kg box dropped on a 1 kg box doesn't crush through it (as today), and
  doesn't launch either box in the steps after impact.
- **Fidelity:** unchanged within its gates. Watch restitution, sliding friction, incline slide,
  and the cube tosses against Drake and Bullet (PARITY_SPEC).
- **Replay:** `shake_replays_bit_exactly`, `rollouts_replay_bit_exactly` and the parallel replay
  all pass.

**Results (built).** Tests: `at_rest_test.rs` (new), `stacked_boxes_hold` (now 30 s),
`biped::tests::standing_holds_still` (new) and the FFI rollback tests.
- **Stacks** (`at_rest`, top box's movement over 60 s after settling 1 s, 60 / 240 Hz):
  - 3 boxes: 1.2e-5 / 1.7e-5 m;
  - 5 boxes: 4.3e-5 / 2.7e-8 m;
  - 10 boxes: 3.7e-3 / 4.0e-4 m. That misses the 0.1 mm gate at 240 Hz (section 2.4); the test
    gates it at 1 mm.
  - The slope: 1.9e-8 / 0 m.
- **`stack_creep_variants`:** every variant holds 30 s at both rates, except frictionless at
  60 Hz, which slides at 8.92 s (D3). Iterations (20, 1) and (20, 60) hold too; at HEAD they fell
  in 5–7 s. No box ends more than 1 mm off its column or tilts.
- **The rest:** all pass.
  - Rest height: within 1 mm.
  - `stacked_boxes_hold`: 30 s.
  - Despawns: 40 debris bodies added before the stack, one removed every 10 steps, so each
    removal shifts the stack's collider indices. All 20 stack contacts start warm the step after
    each removal, and the stack holds under 0.1 mm. Clearing the whole cache on removal fails it.
  - Sleep then nudge: the warm entries don't change while the stack sleeps; the step after the
    nudge starts all 20 stack contacts warm; it then stands 60 s, under 1 cm.
  - Heavy on light: not crushed; nothing launched faster than 0.5 m/s upward.
- **The standing biped:** after its posture settles (10 s), the pelvis moves under 1 mm and the
  feet 0.044 mm in 60 s (0.028 mm with the retuned `StandPd`). Push recovery: section 5, D1.
- **Fidelity:** 38 ok / 2 FAIL, the same as at 15c12cf. The large swing and pendulum amplitudes
  move in the 4th digit.
- **Cube tosses, measured parameters:** position 26.0% (HEAD 25.8%), rotation 23.1° (23.0°),
  e_q 0.48 (0.48). Rigid fit (`--fit`, seed 1, 120 evaluations): the same parameters (e 0,
  mu_s 0.471, mu_k 0.202); held out, position 21.0% (HEAD 20.7%), rotation 17.6° (17.5°), e_q 0.32
  (0.31). Compliant contacts aren't warm-started, so the compliant fit is unchanged.
- **F1 seams:** line and ring, all three joint types: the teacher completes 10/10 and the
  floor 6/10 (line) and 8/10 (ring), as at 15c12cf. Teacher lateral and height errors move by at
  most 0.005 mm.
- **Replay:** the workspace passes (146 tests, 0 failed), including `shake_replays_bit_exactly`,
  `rollouts_replay_bit_exactly` and the parallel replay across 1, 3 and 8 threads.

### R2. Static friction in the position pass

**Set aside (section 2.4).** Built as below and measured. With the budget as specified, the biped
falls. With the joint rows warm-started, the biped's feet hold without R2, and R2 adds creep at
rest. The patch is kept. Kept below as specified, for the record.

`project_contacts` gets a tangential correction (Macklin et al. 2020, *Detailed Rigid Body
Simulation with Extended Position Based Dynamics*, section 3.5):
- **Measure the slip:** for each contact, the relative tangential displacement of its two anchors
  since the start of the step (the anchors start at the same point; `PosContact` has both).
- **Correct it** with the same machinery as the normal: lever arms, inverse inertia, the
  split-impulse bookkeeping.
- **The budget comes from both solves.**
  - The static budget is μs·(jn_vel + jn_pos) − |jt|.
    - jn_vel and jt are the velocity solve's normal and friction impulses for the contact.
    - jn_pos is the position pass's own normal impulse: m_n·Δx_n/dt, summed over every correction
      the projection made to that contact this step.
  - Macklin's bound is μs × the normal position correction alone. That is zero for a box resting
    inside the slop, which is why jn_vel is in it.
  - It isn't zero for the biped. Each joint pass pushes the foot into the ground and projection
    pushes it back out. On a jointed body step 6 turns that push into velocity, so jn_pos is real
    load.
- **Spend the budget once per step, not once per pass.**
  - `project_contacts` runs `solver_iters.1` times, interleaved with the joint solve, then once
    more as the shock pass. Each joint pass pulls the foot again, so each projection sees fresh
    slip.
  - Checked per pass, six passes would get six budgets. So keep a running total of each contact's
    tangential impulse this step, and stop correcting when the total reaches the budget.
- **Stick, or slide at the kinetic limit (as Macklin).**
  - Within the static budget the contact sticks, and the full correction is applied.
  - Past it, the correction is scaled down to the kinetic budget
    max(0, μk·(jn_vel + jn_pos) − |jt|). The velocity solve may already have spent μk·jn_vel on a
    sliding contact, and a negative budget must not turn the correction around.
  - "Past it, nothing" would make friction jump from full to zero at the limit. With joint and
    projection passes alternating, a contact near the limit could flip every pass.
  - For a sliding box this is close to a no-op. It only matters below about (μs − μk)·g·dt,
    roughly 3 cm/s at 60 Hz, and then by about 0.1 mm of stopping distance.
- **The rule is scalar,** |jt| + |Δjt_total| ≤ budget. That is stricter than the vector cone
  |jt + Δjt| ≤ μ·jn, and it keeps the bookkeeping per contact simple.
- **The joint solve** is unchanged. What R2 changes is the contact projection that follows each
  joint pass.

Acceptance:
- **The standing biped** (`standing_still_drift`, 60 s): doesn't fall; the pelvis drifts under
  1 mm and the feet under 0.1 mm.
- **Biped push recovery and `push_sensitivity_20`:** reported. 20 N·s should be no worse than
  17/20.
- **Fidelity, sliding friction:** the gate is unchanged. With the kinetic scaling, R2 does touch a
  sliding box, slightly: only below about 3 cm/s at 60 Hz, changing the stopping distance by at
  most about 0.1 mm.
- **Fidelity, static friction and incline slide:** unchanged.
- **F1 seams:** completion counts and teacher errors are reported. The welding torch rests on
  terrain contacts.

### R3. Box-box face clipping

- **Replace** the corner-inside-plus-grow candidates with the standard reference/incident face
  clip:
  - the reference face is the face of the SAT axis;
  - the incident face is the other box's face most anti-parallel to it;
  - clip the incident face's polygon against the reference face's side planes (Sutherland–Hodgman);
  - keep the clipped points within the contact distance below the reference face.
- **Then run `reduce_to_4`** on the clipped points.
- **The edge-edge case keeps its single point.**
- **Terrain contacts get the same reduction.**
  - `step.rs:431` keeps the 4 deepest points per (collider, terrain) bucket, ties broken in
    narrowphase order. That is the bias `reduce_to_4` removed for box-box.
  - A box lying flat across several heightfield triangles may get all its points on one side.
    Not yet measured.
  - Run `reduce_to_4` on that bucket.

Acceptance:
- a 3-stack with its middle box yawed 3° and 45° holds 60 s at 60 and 240 Hz (with R1);
- a box resting with its centre of mass 1 cm inside a ledge's edge stays; one 1 cm past it tips
  off;
- a box resting flat on a heightfield, across several triangles, holds 60 s (with R1);
- the cube tosses and the rest of fidelity stay within their gates.

### R4. Tests

- Gate `at_rest` and `stack_creep_variants` (R1's numbers) in a non-ignored test. Built:
  `at_rest_test.rs`.
- Gate the standing biped on R1's numbers in a non-ignored test. Built:
  `biped::tests::standing_holds_still` (pelvis under 1 mm, feet under 0.1 mm, over 60 s after a
  10 s settle).
- Add a yawed stack and a ledge test (R3).

## 4. Order

1. **R1, contacts and joint rows, with D2's snapshot carry:** built (section 2.4), uncommitted.
   - It lands alone; R2 is set aside.
   - D1 is settled: `StandPd` is retuned in the same change (section 5, D1), so push recovery is
     back above HEAD. argref's `biped_stand` uses `StandPd::default()` as its teacher, so name the
     retune to URIEL Orchestration with the commit.
   - D4 (the 10-box gate) can be settled with it.
2. **R3:** about 1 day. It's independent of R1, but its stacking acceptance needs R1.
3. **R4** with each.

Golden step hashes change with R1 and R3. Each commit is named to URIEL Orchestration for the
argref bump, and every spec and commit goes to the RiftPhys Reviewer. Heavy runs finish before
11 PM EST.

## 5. Decisions for the Designer

- **D1: R1 leaves biped push recovery at 11/20 at 20 N s (HEAD 17/20): retune `StandPd`, or
  accept it?** At 30 N s it's 4/20 (HEAD 7/20). R2 is set aside (section 2.4).
  - The controller's gains were tuned against a solver that started cold every step and let the
    feet slide.
  - **Decided 2026-11-07: retune.** Done in the R1 change.
    - Only the balance gains moved, so `joint_pd_only` (argref's base) is unchanged:
      - k_com 800 to 1400;
      - k_com_v 150 to 280;
      - k_torso 400 to 2250;
      - k_torso_v 60 to 75.
    - How: a coordinate search, 3 rounds of x0.5 to x2 per gain, scored on seeds 1000–1019 at
      20 and 30 N s, then rounded.

    | | 20 N s, seeds 1000–1019 | 30 N s | 20 N s, held out (2000–2039) | 30 N s, held out |
    |---|---|---|---|---|
    | HEAD (15c12cf) | 17/20 | 7/20 | 29/40 | 10/40 |
    | R1, old gains | 11/20 | 4/20 | 21/40 | 4/40 |
    | R1, retuned | 19/20 | 8/20 | 35/40 | 13/40 |

    - The 20 N s result holds across timing and scale: as drawn / one tick late / x0.99 / x1.01
      gives 19 / 18 / 19 / 18.
    - 0, 6 and 12 N s still stand 20/20.
    - Standing still: the pelvis moves 0.64 mm from 2 s to 62 s and the feet 0.028 mm;
      `standing_holds_still` passes.
    - So the retune beats HEAD on the held-out seeds too, not only on the seeds the search scored.
    - New gate: `biped::tests::push_recovery_20_holds`, on the held-out seeds: at least 32/40 at
      20 N s (measured 35; HEAD 29).
    - For argref (the RiftPhys Reviewer's probe, seeds 2000–2019 against `joint_pd_only`):
      - at 20 N s the teacher's ankle saturation falls from 5.1% to 0.5%, hips 0%;
      - the residual's ankle p1–p99 range narrows from [-66, 726] to [-75, 123];
      - `Space::fit` refits from new demonstrations, but earlier `biped_stand` results aren't
        comparable.
- **D2: the warm cache in rollback snapshots and the step hash.**
  - **The FFI has a rollback snapshot.**
    - `rphys_world_snapshot_create` and `rphys_world_snapshot_restore` are at
      `riftphys-ffi/src/lib.rs:1696-1785`. `RPhysWorldSnapshot` holds the tick, players, body
      poses and body velocities.
    - Restore calls `clear_solver_caches()`, then `wake_all_bodies()`, then sets each body's pose
      and velocity. Waking every body already loses the original run's sleep state.
  - **Under R1 that restore would desync.** The restored world would start every contact cold
    while the original run was warm, so the first step after a rollback differs. For netcode
    rollback that is a desync, so leaving the cache out of the snapshot isn't an option.
  - **The cache is also carried state** like `last_lin_vel` (used at `step.rs:174`) and the sleep
    counters, which the step hash leaves out today.
  - **Proposed, as part of R1:**
    - put the warm cache (and the sleep state, and `last_lin_vel`) in `RPhysWorldSnapshot`;
    - have restore stop clearing the cache, and set each piece of carried state after the calls
      that overwrite it:
      - the cache after the poses, because R1 drops a body's entries on `set_body_pose`;
      - `last_lin_vel` after the velocities, because `set_body_vel` writes it (`world.rs:481`);
      - the sleep state after `wake_all_bodies`, or drop that call;
    - hash all carried state, the cache included.
  - **Despawns:** restore skips bodies that aren't alive, so a body removed after the snapshot
    can't come back, and rollback across a despawn isn't exact (true today too).
    - Built: restore drops the carried warm entries of bodies that aren't alive, and a removed
      body no longer counts as idle, so its entries don't linger.
    - The mirror case: a body spawned after the snapshot survives the restore at its current
      pose, with its warm entries gone. If URIEL or the
    game rolls back across despawns, that needs its own ticket.
  - **Acceptance:** snapshot a resting stack, step N, restore, step N again; the hashes must be
    identical. Also the same with part of the scene asleep.
  - **The decision:** this widens the snapshot and the hash, and URIEL's golden hashes change with
    it. Approve it, or keep the hash to body state and prove rollback by the snapshot test alone.
  - **Built: the snapshot half.**
    - `World::carried_state()` / `restore_carried_state()` hold the contact, patch and joint warm
      entries, `last_lin_vel` and the sleep state (`CarriedState`).
    - `RPhysWorldSnapshot` carries it.
    - Restore no longer clears the caches or wakes every body. It sets poses and velocities, then
      the carried state.
    - Tests: `rollback_replays_a_warm_resting_stack` and
      `rollback_replays_with_part_of_the_scene_asleep` (FFI) nudge the top box right after the
      snapshot and right after the restore, so a sleeping stack wakes and uses its carried
      entries. Then they step 90 and match the original run hash for hash.
      `carried_state_restores_exactly` (riftphys-world) does the same, and checks that a restore
      with only the cache cleared differs.
    - With the carried cache not restored, all three fail, the sleeping cases included.
  - **Not built: the hash.** `step_hash` still covers body state only, pending this decision.
- **D3: the frictionless stack at 60 Hz.**
  - With warm starting it still slides at 9 s; a frictionless stack is only neutrally stable.
  - Accept it, or raise the default velocity iterations for scenes like it?
- **D4: the 10-box stack at 240 Hz.**
  - R1's acceptance asked for under 0.1 mm in 60 s; as built it moves 0.40 mm, and its test gates
    1 mm.
  - Storing jn after the shock pass gives 2.6e-5 m there, but the 5-box stack at 60 Hz rises from
    4.3e-5 to 2.6e-4 m, past its gate. The shock pass's jn is also the heavy-on-light hazard that
    put the store before it.
  - Accept 1 mm, or store after the shock pass?
