//! Episode runner: builds a scenario, steps it at a fixed rate with a controller, and writes
//! protocol lines. Deterministic: the same seed reproduces every line, hashes included.
//!
//! `DroneEnv` is the reusable tick loop (any controller: the expert for demonstrations, a
//! learner for closed-loop flight); `run_episode` drives it with the noisy expert.

use crate::drone::{self, DroneConfig, DroneScenario, ExpertPd, ROTORS};
use crate::protocol::{hash_hex, EntState, Line, PROTOCOL_VERSION};
use crate::rng::Rng;
use riftphys_core::Vec3;
use riftphys_world::world::World;

pub const SIM_HZ: u32 = 60;

#[derive(Clone, Debug)]
pub struct RolloutConfig {
    pub drones: DroneConfig,
    pub seconds: f32,
    pub sample_every: u32,
    /// Exploration: amplitude of uniform noise added to executed rotor commands.
    pub noise: f32,
    /// Probability per tick per drone of a gust (a random lateral impulse).
    pub gust_rate: f32,
    pub gust_impulse: f32,
    pub expert: ExpertPd,
}

impl Default for RolloutConfig {
    fn default() -> Self {
        Self {
            drones: DroneConfig::default(),
            seconds: 10.0,
            sample_every: 3,
            noise: 0.1,
            gust_rate: 0.004,
            gust_impulse: 0.6,
            expert: ExpertPd::default(),
        }
    }
}

pub fn meta_line(cfg: &RolloutConfig, seed: u64) -> Line {
    Line::Meta {
        tick: 0,
        v: PROTOCOL_VERSION.to_string(),
        source: "riftphys".to_string(),
        scenario: scenario_name(cfg),
        sim_hz: SIM_HZ,
        sample_every: cfg.sample_every,
        seed,
        kinds: vec![drone::kind_spec()],
    }
}

pub fn scenario_name(cfg: &RolloutConfig) -> String {
    if cfg.drones.count == 1 { "drone_hover".to_string() } else { format!("drone_squad_{}", cfg.drones.count) }
}

/// A crash is the body center below this height (the body's half height is 0.05 m).
const CRASH_HEIGHT: f32 = 0.12;
const CRASH_TILT: f32 = 1.4;
/// "On target": within this distance and below this speed.
pub const HOLD_RADIUS: f32 = 0.15;
pub const HOLD_SPEED: f32 = 0.2;

/// One drone episode, stepped one tick at a time by any controller.
pub struct DroneEnv {
    pub world: World,
    pub scenario: DroneScenario,
    pub rng: Rng,
    pub tick: u64,
    pub ticks: u64,
    pub ep: String,
    pub cfg: RolloutConfig,
    stable_ticks: Vec<u32>,
}

impl DroneEnv {
    pub fn new(cfg: &RolloutConfig, ep_index: u64, seed: u64) -> Self {
        let ep_seed = seed ^ ep_index.wrapping_mul(0xA24B_AED4_963E_E407);
        let mut rng = Rng::new(ep_seed);
        let mut world = World::with_capacity(16 + cfg.drones.count, 16 + cfg.drones.count);
        world.set_rng_seed(ep_seed);
        let scenario = DroneScenario::build(&mut world, &cfg.drones, &mut rng);
        let n = scenario.drones.len();
        Self {
            world, scenario, rng, tick: 0,
            ticks: (cfg.seconds * SIM_HZ as f32) as u64,
            ep: format!("{}-{}", scenario_name(cfg), ep_index),
            cfg: cfg.clone(),
            stable_ticks: vec![0; n],
        }
    }

    pub fn ep_seed(cfg_seed: u64, ep_index: u64) -> u64 {
        cfg_seed ^ ep_index.wrapping_mul(0xA24B_AED4_963E_E407)
    }

    pub fn dt(&self) -> f32 { 1.0 / SIM_HZ as f32 }
    pub fn done(&self) -> bool { self.tick >= self.ticks }
    pub fn len(&self) -> usize { self.scenario.drones.len() }

    pub fn start_line(&self, seed: u64) -> Line {
        Line::EpisodeStart {
            tick: 0, ep: self.ep.clone(), scenario: scenario_name(&self.cfg),
            members: self.scenario.drones.iter().map(|d| d.body.0).collect(), seed,
        }
    }

    /// The expert's command for drone `i` in the current state.
    pub fn expert(&self, i: usize) -> [f32; ROTORS] {
        self.cfg.expert.command(&self.world, &self.scenario.drones[i])
    }

    pub fn observe(&self) -> Vec<EntState> {
        self.scenario.observe(&self.world)
    }

    /// Applies one command per drone (and its teacher label, if any), gusts, then one world
    /// step; returns the events this tick produced.
    pub fn step(&mut self, acts: &[[f32; ROTORS]], demos: Option<&[[f32; ROTORS]]>) -> Vec<Line> {
        assert_eq!(acts.len(), self.len());
        self.tick += 1;
        let (tick, dt) = (self.tick, self.dt());
        let mut events = Vec::new();
        for i in 0..self.len() {
            self.scenario.drones[i].last_demo = demos.map(|d| d[i]);
            self.scenario.apply(&mut self.world, i, acts[i], dt);
            if self.rng.unit() < self.cfg.gust_rate && !self.scenario.drones[i].crashed {
                let dir = Vec3::new(self.rng.range(-1.0, 1.0), 0.0, self.rng.range(-1.0, 1.0)).normalize_or_zero();
                let imp = dir * self.cfg.gust_impulse;
                self.world.apply_impulse(self.scenario.drones[i].body, imp);
                events.push(Line::Event { tick, ep: self.ep.clone(), e: "gust".into(), who: self.scenario.drones[i].body.0,
                    data: serde_json::json!({ "impulse": imp.to_array() }) });
            }
        }
        self.world.step(dt);

        for i in 0..self.len() {
            let d = &mut self.scenario.drones[i];
            if d.crashed {
                continue;
            }
            let pose = self.world.get_body_pose(d.body);
            if pose.pos.y < CRASH_HEIGHT || drone::tilt(&self.world, d) > CRASH_TILT {
                d.crashed = true;
                events.push(Line::Event { tick, ep: self.ep.clone(), e: "crash".into(), who: d.body.0, data: serde_json::Value::Null });
                continue;
            }
            let err = (d.target - pose.pos).length();
            let speed = self.world.get_body_vel(d.body).lin.length();
            if err < HOLD_RADIUS && speed < HOLD_SPEED {
                self.stable_ticks[i] += 1;
                if self.stable_ticks[i] == SIM_HZ {
                    events.push(Line::Event { tick, ep: self.ep.clone(), e: "reached".into(), who: d.body.0,
                        data: serde_json::json!({ "err_m": err }) });
                }
            } else {
                self.stable_ticks[i] = 0;
            }
        }
        events
    }

    /// A sample of the current state together with the commands chosen FROM it (applied by the
    /// next `step`): the (state, action) pairing imitation needs. The hash is the world as observed.
    pub fn sample_line(&self, acts: &[[f32; ROTORS]], demos: Option<&[[f32; ROTORS]]>) -> Line {
        let mut ents = self.observe();
        for (i, e) in ents.iter_mut().enumerate() {
            e.act = acts[i].to_vec();
            e.demo = demos.map(|d| d[i].to_vec());
        }
        Line::Sample { tick: self.tick, ep: self.ep.clone(), ents, hash: hash_hex(&self.world.step_hash()) }
    }

    /// Distance from each drone to its target (m).
    pub fn errors(&self) -> Vec<f32> {
        self.scenario.drones.iter().map(|d| (d.target - self.world.get_body_pose(d.body).pos).length()).collect()
    }

    pub fn crashed(&self) -> Vec<bool> {
        self.scenario.drones.iter().map(|d| d.crashed).collect()
    }

    pub fn result(&self) -> &'static str {
        if self.scenario.drones.iter().any(|d| d.crashed) {
            "crash"
        } else if self.stable_ticks.iter().all(|&s| s > 0) {
            "stable"
        } else {
            "timeout"
        }
    }

    pub fn end_line(&self) -> Line {
        Line::EpisodeEnd { tick: self.tick, ep: self.ep.clone(), result: self.result().into(), duration_s: self.cfg.seconds }
    }
}

/// Runs one demonstration episode with the noisy expert and returns its lines (no meta line).
pub fn run_episode(cfg: &RolloutConfig, ep_index: u64, seed: u64) -> Vec<Line> {
    let mut env = DroneEnv::new(cfg, ep_index, seed);
    let mut out = vec![env.start_line(DroneEnv::ep_seed(seed, ep_index))];
    while !env.done() {
        let demos: Vec<[f32; ROTORS]> = (0..env.len()).map(|i| env.expert(i)).collect();
        let acts: Vec<[f32; ROTORS]> = demos.iter().map(|d| {
            let mut a = *d;
            for x in a.iter_mut() { *x += env.rng.range(-cfg.noise, cfg.noise); }
            a
        }).collect();
        // Sample the state the commands were chosen from, then advance
        if (env.tick + 1) % cfg.sample_every as u64 == 0 {
            out.push(env.sample_line(&acts, Some(&demos)));
        }
        out.extend(env.step(&acts, Some(&demos)));
    }
    out.push(env.end_line());
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn expert_hovers_without_noise() {
        let cfg = RolloutConfig { noise: 0.0, gust_rate: 0.0, seconds: 8.0, ..Default::default() };
        let lines = run_episode(&cfg, 0, 7);
        match lines.last().unwrap() {
            Line::EpisodeEnd { result, .. } => assert_eq!(result, "stable", "expert should settle on its target"),
            other => panic!("unexpected last line {:?}", other),
        }
    }

    #[test]
    fn expert_holds_a_squad_formation_through_noise() {
        let cfg = RolloutConfig { drones: DroneConfig { count: 4, ..Default::default() }, seconds: 10.0, ..Default::default() };
        let lines = run_episode(&cfg, 3, 11);
        let crashes = lines.iter().filter(|l| matches!(l, Line::Event { e, .. } if e == "crash")).count();
        assert_eq!(crashes, 0, "noisy expert squad should not crash");
    }

    #[test]
    fn rollouts_replay_bit_exactly() {
        let cfg = RolloutConfig { drones: DroneConfig { count: 3, ..Default::default() }, seconds: 3.0, ..Default::default() };
        let a = serde_json::to_string(&run_episode(&cfg, 5, 42)).unwrap();
        let b = serde_json::to_string(&run_episode(&cfg, 5, 42)).unwrap();
        assert_eq!(a, b, "same seed must reproduce every line, hashes included");
        let c = serde_json::to_string(&run_episode(&cfg, 6, 42)).unwrap();
        assert_ne!(a, c, "different episodes must differ");
    }

    #[test]
    fn oracle_handles_payload_and_wind_that_the_nominal_base_cannot() {
        let mut cfg = RolloutConfig { noise: 0.0, gust_rate: 0.0, seconds: 10.0, ..Default::default() };
        cfg.drones.payload = 0.25;
        cfg.drones.wind = 1.0;
        let final_err = |ctl: ExpertPd| {
            let mut errs = Vec::new();
            for e in 0..6 {
                let mut env = DroneEnv::new(&cfg, e, 3);
                while !env.done() {
                    let acts: Vec<_> = (0..env.len()).map(|i| ctl.command(&env.world, &env.scenario.drones[i])).collect();
                    env.step(&acts, None);
                }
                assert!(!env.crashed()[0], "neither controller should crash in this test");
                errs.push(env.errors()[0]);
            }
            errs.iter().sum::<f32>() / errs.len() as f32
        };
        let (oracle, base) = (final_err(ExpertPd::default()), final_err(ExpertPd::nominal_base()));
        assert!(oracle < 0.1, "oracle final error {}", oracle);
        assert!(base > 3.0 * oracle && base > 0.15, "nominal base should sag/drift: base {} oracle {}", base, oracle);
    }

    #[test]
    fn constant_hover_throttle_drifts_off_target() {
        // The floor any learner must beat: open-loop 0.5 on every rotor cannot correct the
        // random start, so it never holds the target.
        let cfg = RolloutConfig { noise: 0.0, gust_rate: 0.0, seconds: 6.0, ..Default::default() };
        let mut env = DroneEnv::new(&cfg, 0, 7);
        while !env.done() {
            env.step(&[[0.5; ROTORS]], None);
        }
        assert_ne!(env.result(), "stable");
    }
}
