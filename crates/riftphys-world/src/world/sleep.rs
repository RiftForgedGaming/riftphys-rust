// crates/riftphys-world/src/world/sleep.rs
use std::sync::atomic::{AtomicU8, AtomicU16, Ordering};
use riftphys_dynamics::dynamics::Bodies;

pub struct SleepManager {
    pub awake: Vec<AtomicU8>,
    pub sleep_ticks: Vec<AtomicU16>,
    pub sleep_allowed: Vec<bool>, // THE FIX: Flag to disable sleep per-body
    pub lin_threshold_sq: f32,
    pub ang_threshold_sq: f32,
    pub ticks_to_sleep: u16,
}

impl SleepManager {
    pub fn new(capacity: usize) -> Self {
        let mut awake = Vec::with_capacity(capacity);
        let mut sleep_ticks = Vec::with_capacity(capacity);
        let mut sleep_allowed = Vec::with_capacity(capacity);

        for _ in 0..capacity {
            awake.push(AtomicU8::new(1));
            sleep_ticks.push(AtomicU16::new(0));
            sleep_allowed.push(true); // Default to allowing sleep
        }

        Self {
            awake,
            sleep_ticks,
            sleep_allowed,
            // Lowered to 0.01 (0.1 m/s) so normal walking arcs don't trigger sleep
            lin_threshold_sq: 0.01,
            // Lowered to 0.01 (0.1 rad/s)
            ang_threshold_sq: 0.01,
            ticks_to_sleep: 30,
        }
    }

    pub fn ensure_capacity(&mut self, id: usize) {
        if id >= self.awake.len() {
            self.awake.resize_with(id + 1, || AtomicU8::new(1));
            self.sleep_ticks.resize_with(id + 1, || AtomicU16::new(0));
            self.sleep_allowed.resize(id + 1, true);
        }
    }

    #[inline(always)]
    pub fn is_awake(&self, id: usize) -> bool {
        self.awake[id].load(Ordering::Relaxed) == 1
    }

    #[inline(always)]
    pub fn wake(&self, id: usize) {
        self.awake[id].store(1, Ordering::Relaxed);
        // You must clear the tick counter when forcing a body awake!
        self.sleep_ticks[id].store(0, Ordering::Relaxed);
    }

    /// Evaluates kinetic energy. Freezes bodies that have stopped moving.
    pub fn evaluate(&mut self, num_bodies: u32, bodies: &Bodies) {
        for i in 0..num_bodies {
            let idx = i as usize;

            // THE FIX: Skip evaluation entirely if sleep is not allowed for this body
            if !self.sleep_allowed[idx] { continue; }

            if !self.is_awake(idx) || !bodies.is_dynamic(i) { continue; }

            let vel = bodies.vel(i);
            if vel.lin.length_squared() < self.lin_threshold_sq && vel.ang.length_squared() < self.ang_threshold_sq {
                // Safely increment the atomic counter
                let current_ticks = self.sleep_ticks[idx].fetch_add(1, Ordering::Relaxed) + 1;

                if current_ticks > self.ticks_to_sleep {
                    self.awake[idx].store(0, Ordering::Relaxed); // Put to sleep
                }
            } else {
                self.sleep_ticks[idx].store(0, Ordering::Relaxed); // Reset if moving
            }
        }
    }
}