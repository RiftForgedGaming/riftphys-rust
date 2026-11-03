# URIEL fabrication F1: seam tracking in RiftPhys

RiftPhys's side of rung F1 of the fabrication roadmap: an arm, a laser line profiler and seam
workpieces, emitted as URIEL protocol v0.2.

Status: **v0.1 (2026-11-03).** Requested by argref `Specs/Fabrication_Roadmap_Spec.md` (verified by
the Designer on 2026-11-03), requests R1-R3. R4-R6 (tool force, material removal, weld heat) wait
until F1 is met. Producer: `crates/riftphys-uriel` (`arm`, `seam`), CLI `riftphys-uriel-seam`.
Engine additions and fixes: `World::raycast_exact`, `ray_shape`, hinge stability past 90 degrees,
terrain grounding (section 7).

## 1. The task

An arm carries a welding torch with a laser line profiler mounted ahead of it. The torch follows a
seam between plates at a fixed travel speed. The teacher sees the seam only through the profiler.
It must keep the torch on the seam despite fixture offset, seam curvature, gap variation and
sensor noise. The floor follows the nominal (CAD) seam without sensing.

| | what |
|---|---|
| teacher (baseline to beat) | `SeamTracker::teacher`: profile, then seam point, then offset from the CAD seam, then Cartesian target, then damped-least-squares resolved rate, then joint PD with gravity compensation |
| floor | the same inner loop on the CAD seam, no sensing (`SeamTracker::nominal`) |
| metric | lateral and height error of the TCP to the ideal TCP on the true seam, mm, mean and 95th percentile over the seam (ISO 9283 style path accuracy); written per sample as `err` |

The Designer's ruling (2026-11-03, roadmap §1a) is that URIEL should beat human design, not match
it. So the teacher is a baseline, and every sample carries the outcome (`err` against the ground
truth) for learning past it.

## 2. The arm (R1)

The Designer's ruling (2026-11-03): a fixed base, no linear track, and every joint continuous
(360 degrees, no limits), so a seam that runs around the arm is one pass with no repositioning.

Six revolute joints, all continuous, in the usual anthropomorphic layout with a wrist:

| # | name | axis (at zero) | link after it | mass | torque limit | armature |
|---|---|---|---|---|---|---|
| 1 | `base_yaw` | world y, top of the 0.30 m base column | turret | 4.0 kg | 120 N m | 0.10 kg m^2 |
| 2 | `shoulder` | horizontal, across the arm | upper arm, 0.40 m | 3.5 kg | 150 N m | 0.10 |
| 3 | `elbow` | parallel to shoulder | forearm, 0.35 m to the wrist | 1.6 kg | 80 N m | 0.05 |
| 4 | `forearm_roll` | along the forearm | wrist link 1 | 0.6 kg | 20 N m | 0.02 |
| 5 | `wrist_pitch` | across the forearm | wrist link 2 | 0.6 kg | 20 N m | 0.02 |
| 6 | `tool_roll` | along the tool | torch and profiler | 0.6 kg | 10 N m | 0.01 |

Six joints because the task needs six: the TCP position (3), the torch direction (2, the work and
travel angles) and the roll of the profiler's line, which must stay across the seam (1). Joints 4-6
orient the tool; joints 1-3 place the wrist. The torch is a 16 mm nozzle with the wire tip (TCP)
18 mm past it, so the nozzle clears both plates of a 45 degree fillet at 2 mm standoff.

Built as the biped is: maximal-coordinate D6 hinges with no limits, joint torques applied equal and
opposite to parent and child, 60 Hz control with 4 physics and servo substeps (240 Hz), solver
iterations (8, 6). The base column is a static body. Armature is added to each child link
isotropically (the drive's reflected inertia plus its housing). Armature about the joint axis alone
left the 0.6 kg wrist links nearly massless crosswise, and wrist torques leaked into neighbouring
joints.

**Wrapped angles.** Joint angles are read from the joint's relative rotation and are always in
(-pi, pi]. They are never unbounded. The protocol's `joints` field carries the wrapped angle. The
learner features (`SEAM_FEATURES`) carry sine and cosine. Controllers that need a continuous
reference keep it in their own state and compare through the shortest wrapped difference, so
crossing +-pi is never a jump (test: two full turns each on base yaw, forearm roll and tool roll).

**Teacher inner loop (`CartesianServo`).** Each substep:
1. The geometric Jacobian (6x6) from the world state: for joint j with world axis a and anchor p,
   column (a x (p_tcp - p), a), orientation rows weighted by 0.3 m.
2. Task-space error: position, and orientation as the rotation vector of R_target R^T.
3. Damped least squares: qdot = J^T (J J^T + lambda^2 I)^-1 (v_ff + K e), lambda = 0.02. Resolved
   rate works from the current configuration, so it never picks between equivalent IK branches and
   never jumps: with continuous joints the arm takes the closest way round.
4. The joint reference integrates qdot, kept within 0.2 rad of the measured angle. Torque = Kp
   wrap(q_ref - q) + Kd (qdot_ref - qdot) + gravity compensation, clamped to the limits.

**Self-collision.** Continuous joints let the arm fold into itself. Bodies joined by a joint do not
collide (engine rule). All other link pairs collide physically. Any contact between two arm links
ends the episode as `collided` with a `self_contact` event. Contact between a link and the workpiece
or table is `tool_contact`, and also ends it.

## 3. The laser line profiler (R2)

A sensor fixed to the tool: N rays (default 64) fanned in the plane of the torch axis and the
across-seam axis. The emitter sits 25 mm ahead of the TCP along the travel direction and 100 mm
back along the torch. The fan covers 40 mm at the TCP. The maximum range is 0.25 m.

Each ray uses **`World::raycast_exact`**, a new engine query. It tests the real shape (rotated box,
sphere, capsule, cylinder, triangle, plane) where `World::raycast` tests only the collider's bounding
box. `World::raycast` is unchanged, because RiftForged calls it through `rphys_world_raycast`. The
arm's own links are skipped.

Output per ray: the range in metres, or `null` when nothing is hit within range. Readings are a pure
function of the world state, so they never change `step_hash`. Sensor noise (default 0.1 mm,
uniform) is drawn from the episode's seeded `Rng`, never the world's.

**Seam finder (`find_seam`).** Butt and V: the two groove walls cross half the groove depth; the
seam is midway between the crossings, at the root depth below the measured plate top. Fillet: the
web's face is vertical, so its lowest hit, dropped to the measured plate, is the seam. Detections
more than 15 mm from the CAD seam are rejected (the tracker's search window).

## 4. Seam workpieces (R3)

The real seam's cross-section, swept along its path at 5 mm segments as a static triangle ribbon
(`SeamWorkpiece`, a world environment), 100 mm of plate each side. Triangles cast exactly, and a
query touches only the few segments near it, so contacts stay cheap at 1 mm detail. (A 1 mm
heightfield would put tens of thousands of triangles under the tool's bounds every substep.) The
plate top is at y = 0. A fixture table sits under everything.

| seam | path | why |
|---|---|---|
| `line` | 0.40 m straight seam 0.45 m in front of the arm, with a seeded lateral half-sine bow | the basic tracking case |
| `ring` | a circle of radius 0.45 m around the arm's base, full 360 degrees by default (`--arc-deg`) | the Designer's continuous-joint case: one pass, the base turns a full turn |

Joint types (profiles across the seam):

| joint | profile | seam point |
|---|---|---|
| `butt` | square groove: the gap floors at the fixture table, 10 mm (plate thickness) below the top | centre of the gap, at the top surface |
| `v` | 60 degree included V, root face 2 mm, root gap, plate 10 mm | centre of the root, at the top of the root face |
| `fillet` | a 40 mm web, plate-thick, standing on the plate on one side of the seam (T joint) | the corner |

Seeded disturbances per episode: fixture offset (lateral +-3 mm and height +-2 mm, or the ring's
centre moved up to 3 mm), fixture yaw +-1 degree (line), curvature (line bow up to 6 mm; ring radial
waviness up to 6 mm, 3 waves per turn), root gap 0.5-2.5 mm varying along the seam (butt and V),
travel speed 15-25 mm/s and profiler noise. The ground-truth seam (1 mm polyline) and the
disturbance values are written to `episode_start.scene`.

Episode phases: approach (2 s, min-jerk, from a ready pose 80 mm back along the torch), settle
(0.5 s), then travel. Travel starts one profiler look-ahead (25 mm) before the seam, a run-on over
the plates, so the profile has covered the seam's start by the time the torch reaches it. Tracking
is scored from the seam start. Results: `completed` (reached the seam's end), `lost` (tracking error
over 5 mm for 0.5 s), `collided` (`self_contact` or `tool_contact`), `stopped` (operator emergency
stop), `timeout`.

## 5. Human in the loop

Roadmap §1a: an outside operator must be able to control the arm. In simulation, `SeamEnv::operator`
takes `pause` (travel stops, the arm holds), `resume`, `override` (the operator takes the arm and
jogs the target by lateral and height millimetres; the controller's torques are ignored until
`resume`, then the controller has 1 s to bring the torch back before the seam can count as lost)
and `stop` (the arm brakes and holds its pose; the episode ends `stopped` after 0.5 s). Each input
is recorded as a protocol `operator` line at its tick. The CLI takes a script
(`--operator "pause@6,resume@8,override@10:2:5,resume@12,stop@20"`, seconds and millimetres).
Serving inputs live from a socket comes with closed-loop `command` lines (M5).

## 6. Protocol v0.2 (additive; v0.1 readers keep working)

In `docs/URIEL_PROTOCOL.md`:
- `meta.v` = `"0.2"`. Every v0.1 line is still valid.
- `meta.kinds[].sensors` (optional): `[{"name":"profiler","type":"line_scan","n":64,
  "fan_m":0.04,"max_range_m":0.25,"unit":"m"}]`.
- `sample.ents[].scan` (optional): the profiler's ranges in ray order, `null` for no hit.
- `sample.ents[].err` (optional): the outcome metric, `[lateral, height]` in metres.
- Kind `arm`: `pos`, `rot`, `vel` and `ang` are the TCP frame, as the biped's are its pelvis.
  `joints` holds the wrapped angle and rate per joint. `target` is the ideal TCP on the true seam at
  the TCP's position along it.
- `episode_start.scene` (optional): seam and joint type, ground-truth polyline, disturbances.
- `operator` lines: `pause`, `resume`, `override` (`data.lateral_m`, `data.height_m`), `stop`.
- Events: `seam_lost`, `tool_contact` (`data.link`), `self_contact` (`data.links`).
- `episode_end.result`: adds `completed`, `lost`, `collided` and `stopped`.

Samples pair the state with the torques chosen from it: `demo` is the teacher's torque at the first
servo substep, `act` is what ran (demo plus DART noise, default 5% of each joint's limit, held for
the tick).

## 7. Engine changes found on the way

- **`World::raycast_exact` and `riftphys_geom::ray_shape`** (new): exact rays against every shape.
- **Hinges past 90 degrees diverged** (`riftphys-articulation`, position-level D6 solve). The locked
  rows of a hinge measured their error from the relative quaternion, `2 q_err.xyz`, which mixes in
  the twist about the hinge by half its angle. Past about 90 degrees of twist the locked rows pushed
  the wrong way, and an arm swinging freely blew up within seconds. With one free (or limited)
  rotational axis, the locked rows now use `a_free x b_free`, which does not depend on the twist.
  Biped knees and hips use limited hinges and pick this up too.
- **Bodies froze within 5 cm of terrain** (`riftphys-world`, step stage 6). The "mass ratio crush"
  guard killed downward velocity and snapped back the position of any body with a terrain contact.
  That included speculative contacts, which reach 5 cm out, so a torch could not descend to its
  seam. Only contacts that touch (true penetration within the 5 mm slop) count now. This changes
  RiftForged behaviour too: bodies hovering just above heightfield or voxel terrain now fall instead
  of floating.

## 8. Results

`riftphys-uriel-seam --eval`, no exploration noise, held-out seeds (line: 10 episodes from seed
1000, 0.40 m; ring: 3 episodes from seed 2000, a full 360 degree turn, 2.83 m). Errors are means
over episodes of each episode's mean and 95th percentile over the seam, mm.

| scene | controller | completed | lateral mean | lateral p95 | height mean | height p95 |
|---|---|---|---|---|---|---|
| line butt | teacher | 10/10 | 0.110 | 0.264 | 0.007 | 0.017 |
| line butt | floor | 6/10 | 2.373 | 4.035 | 0.951 | 0.962 |
| line V | teacher | 10/10 | 0.053 | 0.091 | 0.074 | 0.097 |
| line V | floor | 6/10 | 2.376 | 4.041 | 0.950 | 0.962 |
| line fillet | teacher | 10/10 | 0.055 | 0.202 | 0.060 | 0.083 |
| line fillet | floor | 6/10 | 2.373 | 4.027 | 0.951 | 0.963 |
| ring butt | teacher | 3/3 | 0.114 | 0.253 | 0.006 | 0.015 |
| ring butt | floor | 1/3 | 2.163 | 3.877 | 0.676 | 0.686 |
| ring V | teacher | 3/3 | 0.045 | 0.078 | 0.071 | 0.095 |
| ring V | floor | 1/3 | 2.162 | 3.876 | 0.676 | 0.686 |
| ring fillet | teacher | 3/3 | 0.044 | 0.081 | 0.062 | 0.087 |
| ring fillet | floor | 1/3 | 2.162 | 3.870 | 0.676 | 0.689 |

The floor fails (`lost`) whenever fixture offset plus curvature carry the seam more than 5 mm from
its CAD path; its errors are over the part of the seam it tracked before that. The floor's
statistics are the same across joint types because it never looks at the joint. On the ring, the
base turns a full revolution in one pass with no repositioning.

## 9. Delivery

- `World::raycast_exact`, `ray_shape`, with tests against rotated boxes, spheres, capsules,
  cylinders and gaps.
- `riftphys_uriel::arm` (`Arm`, `ArmConfig`, `CartesianServo`, `JOINTS`, `MAX_TORQUE`,
  `CONTROL_HZ`, `SUBSTEPS`, `kind_spec`, `fk`, `ik`), `riftphys_uriel::seam` (`SeamConfig`,
  `SeamEnv`, `SeamTracker`, `SeamWorkpiece`, `Profiler`, `find_seam`, `SeamRollout`,
  `OperatorCmd`, `SEAM_FEATURES`, `seam_obs`, `track`), the CLI `riftphys-uriel-seam`.
- Tests: bit-exact replay with operator inputs; full turns on the continuous joints with no jumps;
  the profiler finds the true seam to 0.5 mm; the teacher completes and beats the floor on every
  joint type; the ring turns the base; pause, override and stop behave as specified.
- The URIEL side (a `seam_track` binary, like `drone_fly`) is specified by URIEL Orchestration
  against this.
