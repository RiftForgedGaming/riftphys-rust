use anyhow::{Result, ensure};
use std::sync::{atomic::{AtomicBool, Ordering}, Arc};
use std::time::{Duration, Instant};
use std::io::{self, Write, Read};
use std::net::{TcpStream, UdpSocket};
use std::env;

use crossterm::event::{poll, read, Event, KeyCode, KeyModifiers, KeyEventKind};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};

use riftphys_core::{iso, quat_identity, vec3, Velocity};
use riftphys_geom::{MassProps, Shape};
use riftphys_materials::materials::{material, MaterialId};
use riftphys_world::world::{World, WorldBuilder};
use glam::{EulerRot};

pub struct PhysicsQuantizer {
    pub num_bins: u32,
    pub id_offset: u32,
}

impl PhysicsQuantizer {
    pub fn new(num_bins: u32, id_offset: u32) -> Self {
        Self { num_bins, id_offset }
    }

    pub fn float_to_node(&self, value: f32) -> u32 {
        let clamped = value.clamp(-1.0, 1.0);
        let normalized = (clamped + 1.0) / 2.0;
        let bin = (normalized * (self.num_bins as f32 - 1.0)).round() as u32;
        self.id_offset + bin
    }

    pub fn node_to_float(&self, node_id: u32) -> f32 {
        if node_id < self.id_offset || node_id > self.id_offset + self.num_bins {
            return 0.0;
        }
        let bin = node_id - self.id_offset;
        let normalized = bin as f32 / (self.num_bins as f32 - 1.0);
        (normalized * 2.0) - 1.0
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct NetEnt {
    pub id: u32, pub kind: u32,
    pub px: f32, pub py: f32, pub pz: f32,
    pub qx: f32, pub qy: f32, pub qz: f32, pub qw: f32,
    pub sx: f32, pub sy: f32, pub sz: f32,
}

struct TerminalGuard;
impl TerminalGuard {
    fn new() -> Result<Self> { enable_raw_mode()?; Ok(Self) }
}
impl Drop for TerminalGuard {
    fn drop(&mut self) {
        let _ = disable_raw_mode();
        println!("\r\n[System] Terminal restored. Simulation ended.");
    }
}

struct PerfMetrics {
    ema_step_time_ms: f32,
    tick_count: u64,
}

impl PerfMetrics {
    fn new() -> Self { Self { ema_step_time_ms: 0.0, tick_count: 0 } }
    fn update(&mut self, ms: f32) {
        if self.tick_count == 0 {
            self.ema_step_time_ms = ms;
        } else {
            self.ema_step_time_ms = (self.ema_step_time_ms * 0.90) + (ms * 0.10);
        }
        self.tick_count += 1;
    }
}

fn handle_input(running: &Arc<AtomicBool>, paused: &Arc<AtomicBool>) -> Result<()> {
    if poll(Duration::from_millis(0))? {
        if let Event::Key(key) = read()? {
            if key.kind == KeyEventKind::Press {
                match key.code {
                    KeyCode::Char(' ') => {
                        let state = paused.load(Ordering::Relaxed);
                        paused.store(!state, Ordering::Relaxed);
                        println!("\r[Control] Simulation {}", if !state { "PAUSED" } else { "RESUMED" });
                    }
                    KeyCode::Esc => { running.store(false, Ordering::SeqCst); }
                    KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                        running.store(false, Ordering::SeqCst);
                    }
                    _ => {}
                }
            }
        }
    }
    Ok(())
}

fn main() -> Result<()> {
    let args: Vec<String> = env::args().collect();
    let headless = args.iter().any(|arg| arg == "--headless");

    let _guard = TerminalGuard::new()?;
    let running = Arc::new(AtomicBool::new(true));
    let paused = Arc::new(AtomicBool::new(false));

    let r = running.clone();
    ctrlc::set_handler(move || { r.store(false, Ordering::SeqCst); })?;

    let target_hz = 60.0;
    let dt = 1.0 / target_hz;

    let mut w = WorldBuilder::new().with_capacity(10, 10).build();
    let mut metrics = PerfMetrics::new();

    let floor_id = w.add_body(
        iso(vec3(10.0, -0.5, 10.0), quat_identity()),
        Velocity::default(),
        MassProps::infinite(),
        false
    );
    w.add_collider(floor_id, Shape::Box { hx: 50.0, hy: 0.5, hz: 50.0 }, material(MaterialId::Grit));

    println!("\r\n>>> Assembling Bipedal Chassis on the Ground...");
    let spawn_offset = 0.0;

    let torso_start = iso(vec3(10.0, 0.6 + spawn_offset, 10.0), quat_identity());
    let torso_id = w.add_body(torso_start, Velocity::default(), MassProps::from_box(vec3(0.4, 0.35, 0.2), MaterialId::Skin), true);
    w.add_collider(torso_id, Shape::Box { hx: 0.4, hy: 0.35, hz: 0.2 }, material(MaterialId::Skin));

    let foot_l_start = iso(vec3(9.75, 0.1 + spawn_offset, 10.0), quat_identity());
    let foot_l_id = w.add_body(foot_l_start, Velocity::default(), MassProps::from_box(vec3(0.15, 0.1, 0.4), MaterialId::GripTape), true);
    w.add_collider(foot_l_id, Shape::Box { hx: 0.15, hy: 0.1, hz: 0.4 }, material(MaterialId::GripTape));

    let foot_r_start = iso(vec3(10.25, 0.1 + spawn_offset, 10.0), quat_identity());
    let foot_r_id = w.add_body(foot_r_start, Velocity::default(), MassProps::from_box(vec3(0.15, 0.1, 0.4), MaterialId::GripTape), true);
    w.add_collider(foot_r_id, Shape::Box { hx: 0.15, hy: 0.1, hz: 0.4 }, material(MaterialId::GripTape));

    let anchor_torso_l = iso(vec3(-0.25, -0.4, 0.0), quat_identity());
    let anchor_foot_l = iso(vec3(0.0, 0.1, 0.0), quat_identity());
    w.add_hinge_joint(torso_id, foot_l_id, anchor_torso_l, anchor_foot_l, 0);

    let anchor_torso_r = iso(vec3(0.25, -0.4, 0.0), quat_identity());
    let anchor_foot_r = iso(vec3(0.0, 0.1, 0.0), quat_identity());
    w.add_hinge_joint(torso_id, foot_r_id, anchor_torso_r, anchor_foot_r, 0);

    let uriel_socket = UdpSocket::bind("127.0.0.1:0")?;
    uriel_socket.set_read_timeout(Some(Duration::from_millis(50)))?;
    let uriel_addr = "127.0.0.1:49113";
    let mut pending_reward = 1.0_f32;

    let udp_socket = UdpSocket::bind("127.0.0.1:0")?;
    println!(">>> [Network] UDP NEURAL LINK BOUND. Target: {}", uriel_addr);

    let mut smoothed_torque = 0.0_f32;
    let torque_alpha = 0.15_f32;
    let mut fall_count = 0;
    let mut needs_reset = false;

    if headless {
        println!(">>> RUNNING IN HEADLESS TRAINING MODE. Visualizer UDP disabled.");
    } else {
        println!(">>> RUNNING IN VISUALIZER MODE (65Hz Lock).");
    }

    println!("Starting Robotics Training. [SPACE] Pause | [ESC/CTRL+C] Quit");

    let mut target_time = Instant::now();
    let frame_duration = Duration::from_secs_f32(dt);

    while running.load(Ordering::SeqCst) {
        handle_input(&running, &paused)?;

        if paused.load(Ordering::SeqCst) {
            std::thread::sleep(Duration::from_millis(16));
            continue;
        }

        // 1. Observe Current State
        let pose = w.body_pose(torso_id);
        let vel = w.get_body_vel(torso_id);
        let (pitch, _, _) = pose.rot.to_euler(EulerRot::XYZ);

        let foot_pose = w.body_pose(foot_l_id);
        let (foot_pitch, _, _) = foot_pose.rot.to_euler(EulerRot::XYZ);
        let relative_ankle_pitch = pitch - foot_pitch;

        // 2. Send Telemetry
        let mut telemetry = [0u8; 20];
        telemetry[0..4].copy_from_slice(&pending_reward.to_le_bytes());
        telemetry[4..8].copy_from_slice(&pose.pos.y.to_le_bytes());
        telemetry[8..12].copy_from_slice(&vel.lin.y.to_le_bytes());
        telemetry[12..16].copy_from_slice(&pitch.to_le_bytes());
        telemetry[16..20].copy_from_slice(&relative_ankle_pitch.to_le_bytes());

        if uriel_socket.send_to(&telemetry, uriel_addr).is_ok() {
            let mut action = [0u8; 4];
            if let Ok((4, _)) = uriel_socket.recv_from(&mut action) {
                let torque = f32::from_le_bytes(action);
                smoothed_torque = smoothed_torque * (1.0 - torque_alpha) + torque * torque_alpha;
            }
        }

        // 3. Execute Delayed Reset
        if needs_reset {
            w.set_body_pose(torso_id, torso_start);
            w.set_body_vel(torso_id, Velocity::default());
            w.set_body_pose(foot_l_id, foot_l_start);
            w.set_body_vel(foot_l_id, Velocity::default());
            w.set_body_pose(foot_r_id, foot_r_start);
            w.set_body_vel(foot_r_id, Velocity::default());

            smoothed_torque = 0.0;
            needs_reset = false;
        }

        // 4. Apply Action (Enforcing 0 torque for initial diagnostic window)
        let fresh_pose = w.body_pose(torso_id);
        let mut applied_torque = smoothed_torque;

        if metrics.tick_count < 10 {
            applied_torque = 0.0;
        }

        if fresh_pose.pos.y < 0.65 {
            w.apply_torque(torso_id, vec3(applied_torque * 5.0, 0.0, 0.0));
            w.apply_torque(foot_l_id, vec3(-applied_torque * 2.5, 0.0, 0.0));
            w.apply_torque(foot_r_id, vec3(-applied_torque * 2.5, 0.0, 0.0));
        }

        // 5. Step Physics
        let step_start = Instant::now();
        w.step(dt);
        let actual_ms = step_start.elapsed().as_secs_f32() * 1000.0;

        // 5.1 Diagnostic Output
        if metrics.tick_count < 10 {
            let torso_diag = w.body_pose(torso_id);
            let foot_diag = w.body_pose(foot_l_id);
            print!("\r\x1B[K[DIAGNOSTIC TICK {:02}] Torso Y: {:.6} | Foot_L Y: {:.6}\n",
                   metrics.tick_count, torso_diag.pos.y, foot_diag.pos.y);
            io::stdout().flush()?;
        }

        metrics.update(actual_ms);

        // 6. Evaluate State & Calculate Rewards
        let torso_p = w.body_pose(torso_id);
        let (current_pitch, _, _) = torso_p.rot.to_euler(EulerRot::XYZ);
        let foot_p = w.body_pose(foot_l_id);
        let (foot_l_pitch, _, _) = foot_p.rot.to_euler(EulerRot::XYZ);
        let current_rel_pitch = current_pitch - foot_l_pitch;

        let is_upright = torso_p.pos.y > 0.35 && torso_p.pos.y < 1.5
            && current_pitch.abs() < 0.5
            && current_rel_pitch.abs() < 0.6;

        if !is_upright {
            pending_reward = 0.0;
            fall_count += 1;
            needs_reset = true;
        } else {
            pending_reward = 1.0 - (current_pitch.abs() * 0.98);
        }

        if !headless {
            let foot_l_p = w.body_pose(foot_l_id);
            let foot_r_p = w.body_pose(foot_r_id);
            let floor_p = w.body_pose(floor_id);

            let ents = vec![
                NetEnt { id: 0, kind: 2, px: floor_p.pos.x, py: floor_p.pos.y, pz: floor_p.pos.z,
                    qx: floor_p.rot.x, qy: floor_p.rot.y, qz: floor_p.rot.z, qw: floor_p.rot.w,
                    sx: 50.0, sy: 0.5, sz: 50.0 },
                NetEnt { id: 1, kind: 2, px: torso_p.pos.x, py: torso_p.pos.y, pz: torso_p.pos.z,
                    qx: torso_p.rot.x, qy: torso_p.rot.y, qz: torso_p.rot.z, qw: torso_p.rot.w,
                    sx: 0.4, sy: 0.35, sz: 0.2 },
                NetEnt { id: 2, kind: 2, px: foot_l_p.pos.x, py: foot_l_p.pos.y, pz: foot_l_p.pos.z,
                    qx: foot_l_p.rot.x, qy: foot_l_p.rot.y, qz: foot_l_p.rot.z, qw: foot_l_p.rot.w,
                    sx: 0.15, sy: 0.1, sz: 0.4 },
                NetEnt { id: 3, kind: 2, px: foot_r_p.pos.x, py: foot_r_p.pos.y, pz: foot_r_p.pos.z,
                    qx: foot_r_p.rot.x, qy: foot_r_p.rot.y, qz: foot_r_p.rot.z, qw: foot_r_p.rot.w,
                    sx: 0.15, sy: 0.1, sz: 0.4 },
            ];

            let bytes: &[u8] = unsafe { std::slice::from_raw_parts(ents.as_ptr() as *const u8, ents.len() * std::mem::size_of::<NetEnt>()) };
            let _ = udp_socket.send_to(bytes, "127.0.0.1:49111");
        }

        if metrics.tick_count >= 10 && metrics.tick_count % 65 == 0 {
            print!("\r\x1B[K");
            let estimated_epsilon = (1.0_f32 * 0.995_f32.powi(fall_count)).max(0.05);
            let mode = if estimated_epsilon > 0.10 { "BABBLING (EXPLORING)" } else { "EXPLOITING" };

            print!(
                "Tick: {:05} | Falls: {} | Pitch: {:.2} | Torque: {:.2} | Phase: {} ({:.1}%)",
                metrics.tick_count, fall_count, current_pitch, smoothed_torque, mode, (estimated_epsilon * 100.0)
            );
            io::stdout().flush()?;
        }

        if !headless {
            target_time += frame_duration;
            let now = Instant::now();
            if target_time > now {
                let remaining = target_time - now;
                if remaining > Duration::from_millis(2) {
                    std::thread::sleep(remaining - Duration::from_millis(1));
                }
                while Instant::now() < target_time { std::hint::spin_loop(); }
            } else if now - target_time > Duration::from_millis(100) {
                target_time = now;
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_physics_quantizer_bounds() {
        let q = PhysicsQuantizer::new(256, 0);

        // Node to Float limits
        assert_eq!(q.node_to_float(0), -1.0);
        assert_eq!(q.node_to_float(255), 1.0);

        // Float to Node mappings
        assert_eq!(q.float_to_node(-1.0), 0);
        assert_eq!(q.float_to_node(1.0), 255);
        assert_eq!(q.float_to_node(0.0), 128);

        // Out of bounds safety
        assert_eq!(q.float_to_node(1.5), 255);
        assert_eq!(q.float_to_node(-5.0), 0);
    }

    #[test]
    fn test_reward_shaping_thresholds() {
        // Evaluate the continuous reward logic
        let calculate_reward = |pitch: f32| -> f32 {
            1.0 - (pitch.abs() * 0.98)
        };

        // Perfectly upright
        assert_eq!(calculate_reward(0.0), 1.0);

        // Just inside success envelope (0.49 rad)
        let edge_reward = calculate_reward(0.49);
        assert!(edge_reward > 0.5, "Reward {:.3} breached the 0.5 URIEL success threshold", edge_reward);

        // Farthest valid envelope point
        let max_envelope_reward = calculate_reward(0.5);
        assert!(max_envelope_reward > 0.5, "Reward {:.3} breached the 0.5 URIEL success threshold", max_envelope_reward);
    }
}