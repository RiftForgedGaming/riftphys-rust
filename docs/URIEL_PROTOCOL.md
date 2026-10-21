# URIEL protocol: what RiftPhys writes for URIEL

The contract between RiftPhys scenarios (producer) and argref / URIEL (consumer), for robotics
training: drones, drone squads and, next, humanoids. It deliberately has the same shape as the
RiftForged server telemetry contract (`R:/RiftForged/Specs/URIEL_Telemetry.MD`): JSON lines tagged by
`t`, raw named state at a fixed sample rate, events at their exact tick, binning left to the
consumer. One URIEL pipeline serves game and robotics data.

Status: **v0.1 (2026-10-10).** Producer: `crates/riftphys-uriel` (`riftphys-uriel-rollout`).
Consumer: argref `tools/riftphys_to_jsonl.py`.

## Lines

Every line has `t` and `tick` (60 Hz simulation tick). Types: `meta` (first line), `episode_start`,
`sample` (every `sample_every` ticks), `event`, `episode_end`, and `command` (consumer to
RiftPhys, closed loop; defined, not yet served).

```json
{"t":"meta","tick":0,"v":"0.1","source":"riftphys","scenario":"drone_hover","sim_hz":60,"sample_every":3,"seed":1,
 "kinds":[{"kind":"quadrotor","actuators":[{"name":"rotor_px","min":0.0,"max":1.0,"unit":"throttle"}, "..."]}]}
{"t":"episode_start","tick":0,"ep":"drone_hover-0","scenario":"drone_hover","members":[1],"seed":1}
{"t":"sample","tick":3,"ep":"drone_hover-0","hash":"8108c9d6016c9430","ents":[
  {"eid":1,"kind":"quadrotor","pos":[1.68,2.92,0.82],"rot":[0.016,0.924,0.064,0.378],"vel":[0.0,0.003,0.068],
   "ang":[-0.24,-0.21,0.71],"target":[0.27,3.0,0.98],"act":[0.19,0.44,0.77,0.61],"demo":[0.15,0.36,0.86,0.69]}]}
{"t":"event","tick":412,"ep":"drone_hover-0","e":"gust","who":1,"data":{"impulse":[0.4,0.0,-0.45]}}
{"t":"episode_end","tick":600,"ep":"drone_hover-0","result":"stable","duration_s":10.0}
{"t":"command","tick":601,"who":1,"act":[0.5,0.5,0.5,0.5]}
```

### Entity fields

| Field | Units | Notes |
|---|---|---|
| `eid` | | body id |
| `kind` | | agent kind, e.g. `quadrotor`; `meta.kinds` lists its actuators in command order |
| `pos`, `vel` | m, m/s | world frame, y up |
| `rot` | | unit quaternion `[x, y, z, w]`, body to world |
| `ang` | rad/s | world-frame angular velocity |
| `target` | m | the agent's goal (formation slot, waypoint), when it has one |
| `act` | actuator units | commands chosen from this sample's state (applied by the step that follows) |
| `joints` | rad, rad/s | articulated kinds only: `[angle, rate]` per joint, in actuator order |
| `demo` | actuator units | the expert's command for this sample's state, when a teacher exists: the imitation label. Rollouts execute a noise-perturbed copy (`act`) while recording the clean one (DART), so data covers recovering from mistakes |

A sample is the state *before* the tick's step, together with the commands chosen from it, so
(state, `demo`) pairs line up for imitation. `tick` is the number of steps already taken.

### Determinism

`sample.hash` is the first 16 hex characters of the world's `step_hash()` for the sampled state. Every
episode records its `seed` in `episode_start`; re-running the scenario with that seed must
reproduce every line, hashes included (tested in `riftphys-uriel`, `rollouts_replay_bit_exactly`).

### Events

| `e` | `data` | Meaning |
|---|---|---|
| `gust` | `impulse` [x,y,z] N s | a random lateral disturbance hit the agent |
| `crash` | | body below 0.12 m or tilted past 80 degrees; the agent stops flying |
| `reached` | `err_m` | held its target (within 0.15 m, under 0.2 m/s) for 1 s |

`episode_end.result`: `stable` (every agent on target at the end), `crash` (any agent crashed),
`timeout`.

## Scenarios

### Quadrotor (`drone_hover`, `drone_squad_N`)

A 1.2 kg, 0.5 x 0.1 x 0.5 m box with four rotors in a "+" layout at 0.2 m on the body x/z axes.
Rotor command u in [0, 1] gives u x T_MAX along body up (T_MAX = 5.886 N, so hover is u = 0.5 on every rotor).
Rotor order: `rotor_px` (+x), `rotor_nx` (-x), `rotor_pz` (+z), `rotor_nz` (-z); pairs (px, nx)
and (pz, nz) spin opposite ways for yaw. The mixer desaturates like a flight controller (thrust
first, then roll/pitch scaled together, yaw last). The expert is a cascaded PD controller. Squads
of N drones hold a line (N <= 3) or ring formation.

```
riftphys-uriel-rollout --out hover.jsonl --episodes 60 [--drones 4] [--seed 1] [--noise 0.1] [--gust-rate 0.004]
```

### Biped (`biped`, scenario code in `riftphys-uriel::biped`; no rollout CLI yet)

80 kg: pelvis 10, torso 40 (capsule), head 4, thighs 8, shanks 4, feet 1 kg (box 0.12 x 0.06 x
0.24 m, rubber). Spine and neck are locked hinges; six actuated hinges about the pelvis x axis,
in order `hip_l, knee_l, ankle_l, hip_r, knee_r, ankle_r`, torque limits 200/200/120 N m. Angle
signs: positive knee = flexion. Entity state is the pelvis plus `joints`. Control runs at 60 Hz
with 4 physics/servo substeps (240 Hz) per tick; limb bodies carry 0.08 kg m^2 of actuator
armature about the joint axis. Fallen: pelvis below 0.6 m or torso tilted past ~53 degrees.
Teacher: `StandPd`.

## Engine note: contacts

The contact solver applied twice the impulse it accumulated, so each iteration's push was
cancelled by the next clamp and nothing could rest on a static body; it also treated the
narrowphase's 5 cm contact margin as penetration and had no angular terms. Now: sequential
impulses at the contact point (lever arms through the world inverse inertia), accumulated
Coulomb friction, the margin solved as a speculative gap, a position-level contact projection
interleaved with the joint solve (so joints can't press loaded feet into the ground), and no
collisions between bodies joined by a joint. Box-box narrowphase (SAT, up to 4 points) was also
missing. Tests: `contact_resting_test`, `box_box_tests`.

## Engine note: inertia

`riftphys-uriel` surfaced that `Bodies::add` ignored each body's inertia tensor and used
`diag(inv_mass)` for every body, so torques ignored shape and size (46x too weak on the drone).
`World::add_body` now keeps `MassProps.inertia` and the world inverse inertia is R I^-1 R^T
(`test_torque_uses_real_inertia`). Rotational behaviour of every scenario changed accordingly;
`test_xpbd_hinge_joint_stability` was rewritten to check the whole swing, because its
single-instant height check only held for the old (wrong) pendulum period.

## Not yet

- `command` lines are defined but not served: closed-loop control (URIEL flying the drone) needs
  a host that reads commands, over a local socket or `riftnet-rust`.
- Biped rollout CLI and demo export; a learned (URIEL) stand policy.
