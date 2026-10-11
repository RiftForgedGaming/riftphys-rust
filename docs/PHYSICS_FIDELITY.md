# RiftPhys physics fidelity

Goal: RiftPhys trajectories that match measured reality within the measurement's own noise,
benchmark by benchmark. The ladder: closed-form cases (no hardware) -> public datasets -> own
mocap captures -> robots. Code: `crates/riftphys-reality`. Status as of 2026-10-11.

## 1. Closed-form benchmarks

```
cargo run --release -p riftphys-reality --bin riftphys-fidelity [-- --hz 60,240 --json report.json]
```

Each case compares one measured quantity with its textbook value under a stated tolerance.
`benchmarks_hold_at_240_hz` (a test) keeps them there; the only accepted gap is listed below.

| Case | 60 Hz | 240 Hz | Tolerance |
|---|---|---|---|
| Free fall, 1 s drop | 0.00% | 0.01% | 1% |
| Simple pendulum period (distance joint) / amplitude after 30 s | 0.03% / 0.983 | 0.04% / 0.999 | 1% / +-0.02 |
| Hinged rod period (real inertia) / amplitude after 30 s | 0.01% / 0.991 | 0.06% / 0.995 | 1% / +-0.02 |
| **Large swing** (rod released at 90 degrees), energy after 10 s | **0.43** | **0.77** | +-0.02 (open) |
| Sliding to rest from 3 m/s (kinetic friction) | 2.5% | 0.6% | 5% |
| Static friction holds on a slope below atan(mu_s) | 0 mm | 0 mm | 5 mm |
| Incline slide, g(sin - mu_k cos) | 2.9% | 4.8% | 5% |
| Restitution, e = 0.8 rebound apex | **13%** | 3.5% | 5% |
| Restitution, e = 0 (no bounce) | 0 mm | 0 mm | 5 mm |
| Rolling ball, a = 5/7 g sin, and no slip | 1.7%, exact | 0.4%, exact | 5% |
| Torque-free box: angular momentum, energy, intermediate-axis flip | 0.00%, 0.02%, flips | 0.01%, 0.03%, flips | 1% |
| Head-on collision: momentum, both velocities | exact | exact | 0.01 / 0.05 |
| Resting box drift over 10 s | 0 | 0 | 2 mm |

The first run (before this work) passed 9/18 at 60 Hz and 11/18 at 240 Hz.

### What the benchmarks found and what changed

| Finding | Fix |
|---|---|
| Nothing could rest on a static body: the contact solver applied 2x the impulse it accumulated, so the next clamp cancelled it | Sequential impulses at the contact point with lever arms (earlier, with the humanoid work) |
| Only static friction existed: sliding used mu_s, 20% short | Stribeck curve from the material table: mu_s at rest, toward mu_k when sliding |
| Velocity was re-derived from position differences every step: f32 positions lose ~ulp/dt per step (1.4% of a fall at 240 Hz, 100 m up) | Velocity changes only by the position *correction* (final - predicted pose) |
| Pushing a body out of penetration launched it (a ball with e = 0 rebounded 10 cm) | Split impulse: contact projection displacement kept out of the velocity (for bodies outside articulations) |
| No gyroscopic term: torque-free spin lost angular momentum, no Dzhanibekov flip | World angular momentum carried through the rotation update, midpoint rotation (2nd order) |
| Hinged bodies lost a third of their energy per swing: joints were only position-projected, which makes the restoring torque implicit (~(w dt)^2 per step) | D6 joints also solved at velocity level (locked axes as equalities, limits as speculative inequalities) |
| Fast bodies tunnelled: broadphase boxes had no margin or sweep (a ball 7 mm above the floor sank 7 cm before any contact existed) | Swept, margin-inflated broadphase; per-pair contact distance covers both bodies' motion |
| Restitution used a velocity that already included the step's gravity, and fired on speculative contacts | Restitution pass after the solve (as Box2D v3) on contacts that carried load, at mid-step approach speed |
| Semi-implicit Euler: free fall 1.7% long at 60 Hz | Ballistic bodies move with the average of the last step's end velocity and this step's: exact for constant forces, thrust included |
| A 1000 kg box dropped on a 1 kg box crushed through it | Shock propagation (bottom-up pass, lower body immovable) in the velocity and position solves |
| Velocity motors added kd x error straight to the velocity (diverges for kd > 2) and the axis was also pinned to 0 at position level, so a motor could not turn a joint | Motor = velocity-level constraint row with impulse cap; velocity-driven axes free at position level |

`world_robotics_test`: 3 failing tests before -> 1 (the large-swing energy test, below). The
mass-ratio test never had colliders (it measured free fall) and the motor test expected the
wrong sign (+x rotation carries -y toward -z); both fixed.

### Open gaps

- **Fast joint motion loses energy.** A hinged rod released horizontal keeps 43% of its energy
  after 10 s at 60 Hz, 77% at 240 Hz, 87% at 480 Hz; small swings are fine (99%). Per-stage
  measurement puts the loss in velocity projection of maximal-coordinate joints: each step the
  joint's velocity solve removes the part of the velocity that has turned away from the tangent.
  Tried without effect: solving locked point rows along the error direction, splitting joint
  drift out of the velocity (it also broke the biped), half-step gravity for constrained bodies
  (pendulums then gain energy). Candidate fixes: reduced-coordinate articulations (Featherstone /
  MuJoCo-style generalized coordinates) or a RATTLE-consistent rigid-body integrator. Matters for
  robot limbs: fast joints are where fidelity counts.
- **Restitution at 60 Hz overshoots 13%:** a bounce reverses the velocity for the whole step,
  losing the gravity of the part after impact. Use >= 240 Hz for fidelity runs (3.5%).
- **Static friction is a velocity-level approximation:** no position-level friction, so creep is
  possible under persistent tangential load on articulated bodies.

Downstream (same seeds as before): drone M1 unchanged (expert 0.043 m, URIEL DAgger 0.073 m);
M2 squad DAgger 0.199 m; biped M3 still met (DAgger 92.5%; the stand teacher holds 20/20 at
12 N s but 12/20 at 20 N s, from 20/20: the old solver's numerical damping helped it recover).
Physics cost 50 -> 69 us per biped step (velocity-level joint rows).

## 2. Reality logs, replay and fitting

**Format** (`log.rs`): JSON lines, metres / seconds / y up / quaternions [x, y, z, w]:
`meta`, `pose` (time, body, pos, rot), `cmd` (time, who, act), `event` (`break` between takes).

**Converter** (`tools/mocap_to_reality.py`): OptiTrack Motive CSV exports (rigid bodies,
quaternions; units read from the header) and generic CSV with column mapping (libsurvive,
Crazyflie cfclient logs: `--time-scale`, `--units`, `--up z`, `--cmd`). Breaks at tracking gaps
and between files. `python tools/mocap_to_reality.py selftest`. The Motive parser follows the
documented export layout; check the first real export with `--verbose`.

**Scenes** (`scene.rs`, examples in `crates/riftphys-reality/scenes/`): ground, objects matched
to log bodies, and unknown parameters with ranges: pair restitution / mu_s / mu_k, quadratic air
drag, mass scale. Measured pair values override the material table through
`World::set_pair_props`.

**Replay**: windows (default 1 s every 0.25 s) start each object from a least-squares
constant-acceleration fit to +-30 ms of samples (two-point differences of 0.5 mm noise are
~3.5 cm/s off; the fit residual also rejects windows that start across an impact), then compare
simulated and logged positions at every sample.

**Fitting** (`riftphys-fit`): CMA-ES (self-contained, deterministic) on training takes; every
4th take held out. Reports RMS position error at 0.1 / 0.25 / 0.5 / 1 s horizons before and
after, on train and held-out.

**Pipeline check** (synthetic ball: 16 throws, 240 Hz, 0.5 mm noise, truth e 0.72, drag 0.0015):

```
cargo run --release -p riftphys-reality --bin riftphys-synth -- --scene crates/riftphys-reality/scenes/ball_drop.json --truth 0.72,0.45,0.0015 --out ball.jsonl
cargo run --release -p riftphys-reality --bin riftphys-fit -- --log ball.jsonl --scene crates/riftphys-reality/scenes/ball_drop.json --evals 200
```

| | 0.1 s | 0.25 s | 0.5 s | 1 s |
|---|---|---|---|---|
| held-out, initial guess (e 0.3, drag 0) | 1.6 mm | 20.8 | 91.6 | 147.9 |
| held-out, fitted (e 0.713, drag 0.0007) | 1.3 | 3.7 | 6.3 | 11.2 |
| held-out, true parameters (floor) | 1.3 | 3.7 | 7.9 | 14.8 |

Restitution is recovered; friction is not identifiable from bounces alone (the ball barely
slides; capture sliding or spinning throws to fit it). The floor is bounce chaos: impact timing
differs by up to one step and compounds over several bounces.

### Rotor-driven vehicles (quadrotor scenes)

`quadrotor.rs`: a body driven by **measured** rotor speeds from the log's `cmd` lines (as
Blackbird's tachometers report them, so no motor model): thrust k_thrust (rpm/1000)^2 along body
up at each rotor, reaction torque spin k_torque (rpm/1000)^2, linear drag. Rotor layout, mass
and inertia come from the scene; k_thrust, k_torque, drag and inertia scales can be fitted.
`riftphys-fit` detects a quadrotor scene (it has `rotors`), uses 0.5 s windows (open-loop
flight diverges quickly), and prints a hover-regression estimate of k_thrust from the log.

Synthetic check (60 s PD-controlled flight, poses 240 Hz with 0.3 mm noise, rotor rpm 190 Hz with
20 rpm noise; truth k_thrust 0.01, k_torque 0.00016, drag 0.15, roll/pitch inertia x1.3):

| | k_thrust | inertia_rp | drag | k_torque | held-out RMS at 0.5 s |
|---|---|---|---|---|---|
| initial | 0.008 | 1.0 | 0 | 0.0003 | 121 mm |
| fitted | **0.0100** | 1.37 | 0.114 | 0.0003 | 5.4 mm |

Yaw torque is not identifiable from a heading-hold flight (positions barely depend on it);
real flights with yaw motion constrain it. Test: `recovers_thrust_coefficient_from_a_synthetic_flight`.

### Blackbird logs

`tools/blackbird_to_reality.py` reads a flight without ROS installed: the ROS1 bag (v2.0;
plain or bz2 chunks, lz4 with the `lz4` module), topics `/blackbird/state`
(geometry_msgs/PoseStamped, mocap) and `/blackbird/rotor_rpm` (blackbird/MotorRPM: Header,
time[] sample_stamp, float32[] rpm), or the flight's CSVs (`groundTruthPoses.csv`,
`csv/blackbird_slash_rotor_rpm.csv`, columns detected from headers; check with `--verbose`).
Blackbird's mocap frame is NED and its body frame FRD; both map to RiftPhys by +90 degrees
about x: (x, y, z) -> (x, -z, y). Message layouts and topics are from the dataset's own repo
(github.com/mit-aera/Blackbird-Dataset: `msg/MotorRPM.msg`, `logConversionUtilities`).

End-to-end check: the synthetic flight above written out as a NED Blackbird-style bag, converted
back, and fitted gives identical results (hover estimate 0.00999). `python
tools/blackbird_to_reality.py selftest` covers the bag reader.

`scenes/blackbird.json` geometry (mass, inertia, arm, rotor order and spins) is a
**placeholder**: set it from the Blackbird paper before fitting real flights, and check the
rotor order against the rpm channels.

## 3. Next

- Run on real Blackbird flights: its data host (blackbird-dataset.mit.edu) had no DNS record on
  2026-10-11 from local, Google or Cloudflare resolvers. When it is back: one calibration flight
  ("for dynamics, no camera data") first; downloads only with the user's go-ahead.
- Own captures once mocap is set up: drops, tosses, sliding blocks, pendulums, then a Crazyflie.
- Close the fast-joint energy gap (above) before trusting fast limb motion.
