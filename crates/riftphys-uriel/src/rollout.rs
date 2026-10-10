//! Episode runner: builds a scenario, steps it at a fixed rate with a controller, and writes
//! protocol lines. Deterministic: the same seed reproduces every line, hashes included.

use crate::drone::{self, DroneConfig, DroneScenario, ExpertPd, ROTORS};
use crate::protocol::{hash_hex, Line, PROTOCOL_VERSION};
use crate::rng::Rng;
use riftphys_core::Vec3;
use riftphys_world::world::World;

pub const SIM_HZ: u32 = 60;

#[derive(Clone, Debug)]
pub struct RolloutConfig {
    pub drones: DroneConfig,
    pub seconds: f32,
    pub sample_every: u32,
    /// Exploration: std-dev-like amplitude of uniform noise added to executed rotor commands.
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

/// Runs one episode and returns its lines (without the meta line).
pub fn run_episode(cfg: &RolloutConfig, ep_index: u64, seed: u64) -> Vec<Line> {
    let ep_seed = seed ^ ep_index.wrapping_mul(0xA24B_AED4_963E_E407);
    let mut rng = Rng::new(ep_seed);
    let mut world = World::with_capacity(16 + cfg.drones.count, 16 + cfg.drones.count);
    world.set_rng_seed(ep_seed);
    let mut sc = DroneScenario::build(&mut world, &cfg.drones, &mut rng);

    let ep = format!("{}-{}", scenario_name(cfg), ep_index);
    let dt = 1.0 / SIM_HZ as f32;
    let ticks = (cfg.seconds * SIM_HZ as f32) as u64;
    let mut out = vec![Line::EpisodeStart {
        tick: 0,
        ep: ep.clone(),
        scenario: scenario_name(cfg),
        members: sc.drones.iter().map(|d| d.body.0).collect(),
        seed: ep_seed,
    }];

    let mut stable_ticks = vec![0u32; sc.drones.len()];
    for tick in 1..=ticks {
        for i in 0..sc.drones.len() {
            let demo = cfg.expert.command(&world, &sc.drones[i]);
            let mut act = demo;
            for a in act.iter_mut() {
                *a += rng.range(-cfg.noise, cfg.noise);
            }
            sc.drones[i].last_demo = Some(demo);
            sc.apply(&mut world, i, act, dt);

            if rng.unit() < cfg.gust_rate && !sc.drones[i].crashed {
                let dir = Vec3::new(rng.range(-1.0, 1.0), 0.0, rng.range(-1.0, 1.0)).normalize_or_zero();
                world.apply_impulse(sc.drones[i].body, dir * cfg.gust_impulse);
                out.push(Line::Event { tick, ep: ep.clone(), e: "gust".into(), who: sc.drones[i].body.0,
                    data: serde_json::json!({ "impulse": (dir * cfg.gust_impulse).to_array() }) });
            }
        }
        world.step(dt);

        for (i, d) in sc.drones.iter_mut().enumerate() {
            if d.crashed {
                continue;
            }
            let pose = world.get_body_pose(d.body);
            if pose.pos.y < HALF_Y_CRASH || drone::tilt(&world, d) > 1.4 {
                d.crashed = true;
                out.push(Line::Event { tick, ep: ep.clone(), e: "crash".into(), who: d.body.0, data: serde_json::Value::Null });
                continue;
            }
            let err = (d.target - pose.pos).length();
            let speed = world.get_body_vel(d.body).lin.length();
            if err < 0.15 && speed < 0.2 {
                stable_ticks[i] += 1;
                if stable_ticks[i] == SIM_HZ {
                    out.push(Line::Event { tick, ep: ep.clone(), e: "reached".into(), who: d.body.0,
                        data: serde_json::json!({ "err_m": err }) });
                }
            } else {
                stable_ticks[i] = 0;
            }
        }

        if tick % cfg.sample_every as u64 == 0 {
            out.push(Line::Sample { tick, ep: ep.clone(), ents: sc.observe(&world), hash: hash_hex(&world.step_hash()) });
        }
    }

    let result = if sc.drones.iter().any(|d| d.crashed) {
        "crash"
    } else if stable_ticks.iter().all(|&s| s > 0) {
        "stable"
    } else {
        "timeout"
    };
    out.push(Line::EpisodeEnd { tick: ticks, ep, result: result.into(), duration_s: cfg.seconds });
    out
}

/// A crash is the body center below this height (the body's half height is 0.05 m).
const HALF_Y_CRASH: f32 = 0.12;

pub fn act_of(cmd: [f32; ROTORS]) -> Vec<f32> {
    cmd.to_vec()
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
}

