//! riftphys-vis-tool — 3D Visual Debugger (winit 0.30 + riftphys-render/wgpu 22.0 + egui 0.29)
use std::sync::Arc;
use std::{collections::HashSet, time::Instant};
use glam::{Vec3, Quat};
use parking_lot::Mutex;
use winit::{
    application::ApplicationHandler,
    dpi::{LogicalSize, PhysicalSize},
    keyboard::{KeyCode, PhysicalKey},
    window::{Window, WindowAttributes, WindowId, CursorGrabMode},
};
use winit::event::{ElementState, WindowEvent, KeyEvent, MouseButton, DeviceEvent, DeviceId};
use winit::event_loop::{ActiveEventLoop, EventLoop, DeviceEvents};
use riftphys_render::{Renderer, Instance, ShapeKind, trs};

use egui_wgpu::Renderer as EguiRenderPass;
use egui_wgpu::ScreenDescriptor;
use egui_winit::State as EguiWinitState;

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct NetEnt {
    pub id: u32, pub kind: u32,
    pub px: f32, pub py: f32, pub pz: f32,
    pub qx: f32, pub qy: f32, pub qz: f32, pub qw: f32,
    pub sx: f32, pub sy: f32, pub sz: f32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct OversightCommand {
    pub gravity_y: f32,
    pub drop_height: f32,
    pub force_reset: u32,
}

fn env_u32(key: &str, default: u32) -> u32 {
    std::env::var(key).ok().and_then(|s| s.parse().ok()).unwrap_or(default)
}

struct App {
    renderer: Option<Renderer>,
    window: Option<Arc<Window>>,
    last: Instant,
    pressed: HashSet<KeyCode>,
    rmb_down: bool,
    move_speed: f32,
    base_speed: f32,
    speed_step: usize,

    udp_socket: Option<std::net::UdpSocket>,
    net_ents: Arc<Mutex<Vec<NetEnt>>>,

    oversight_socket: Option<std::net::UdpSocket>,
    target_gravity: f32,
    target_drop_height: f32,

    egui_ctx: egui::Context,
    egui_winit_state: Option<EguiWinitState>,
    egui_renderer: Option<EguiRenderPass>,
}

const SPEED_STEPS: [f32; 10] = [1.0, 1.5, 2.0, 3.0, 5.0, 8.0, 12.0, 20.0, 35.0, 50.0];

impl App {
    fn new() -> Self {
        let udp_socket = std::net::UdpSocket::bind("127.0.0.1:49111").ok();
        if let Some(sock) = &udp_socket {
            sock.set_nonblocking(true).unwrap();
            eprintln!("Native UDP Listener active on 127.0.0.1:49111");
        } else {
            eprintln!("Failed to bind UDP 49111.");
        }

        let oversight_socket = std::net::UdpSocket::bind("127.0.0.1:0").ok();

        Self {
            window: None,
            renderer: None,
            last: Instant::now(),
            pressed: HashSet::new(),
            rmb_down: false,
            move_speed: 5.0,
            base_speed: 5.0,
            speed_step: 0,
            udp_socket,
            net_ents: Arc::new(Mutex::new(Vec::new())),
            oversight_socket,
            target_gravity: -9.81,
            target_drop_height: 0.005,

            egui_ctx: egui::Context::default(),
            egui_winit_state: None,
            egui_renderer: None,
        }
    }

    fn send_oversight(&self, force_reset: u32) {
        if let Some(sock) = &self.oversight_socket {
            let cmd = OversightCommand {
                gravity_y: self.target_gravity,
                drop_height: self.target_drop_height,
                force_reset,
            };

            let bytes: &[u8] = unsafe {
                std::slice::from_raw_parts(
                    &cmd as *const _ as *const u8,
                    std::mem::size_of::<OversightCommand>()
                )
            };
            let _ = sock.send_to(bytes, "127.0.0.1:49112");
        }
    }

    fn on_resize(&mut self, size: PhysicalSize<u32>) {
        if let Some(r) = self.renderer.as_mut() {
            r.resize(size.width.max(1), size.height.max(1));
        }
    }

    fn handle_key(&mut self, ke: &KeyEvent, el: &ActiveEventLoop) {
        if let PhysicalKey::Code(code) = ke.physical_key {
            match ke.state {
                ElementState::Pressed => {
                    match code {
                        KeyCode::Escape => { el.exit(); return; }
                        KeyCode::KeyR   => {
                            if let Some(r) = self.renderer.as_mut() {
                                r.camera.eye   = Vec3::new(-6.0, 4.0, 6.0);
                                r.camera.yaw   = 45_f32.to_radians();
                                r.camera.pitch = (-15_f32).to_radians();
                            }
                        }
                        KeyCode::BracketLeft => { if self.speed_step > 0 { self.speed_step -= 1; } }
                        KeyCode::BracketRight => { if self.speed_step + 1 < SPEED_STEPS.len() { self.speed_step += 1; } }
                        KeyCode::Digit1 => { self.speed_step = 0; }
                        KeyCode::Digit2 => { self.speed_step = 2; }
                        KeyCode::Digit3 => { self.speed_step = 4; }
                        KeyCode::Digit4 => { self.speed_step = 7; }
                        KeyCode::Digit5 => { self.speed_step = 9; }
                        _ => {}
                    }
                    self.pressed.insert(code);
                }
                ElementState::Released => { self.pressed.remove(&code); }
            }
        }
    }

    fn handle_mouse(&mut self, event: &WindowEvent) {
        let window = match self.window.as_ref() { Some(w) => w, None => return };
        if let WindowEvent::MouseInput { state, button: MouseButton::Right, .. } = *event {
            self.rmb_down = state == ElementState::Pressed;
            let _ = window.set_cursor_grab(if self.rmb_down { CursorGrabMode::Locked } else { CursorGrabMode::None });
            window.set_cursor_visible(!self.rmb_down);
        }
    }

    fn update_camera(&mut self, dt: f32) {
        let mut dir = Vec3::ZERO;
        let shift_down = self.pressed.contains(&KeyCode::ShiftLeft) || self.pressed.contains(&KeyCode::ShiftRight);

        if let Some(r) = self.renderer.as_mut() {
            if self.pressed.contains(&KeyCode::KeyW) { dir += r.camera.forward(); }
            if self.pressed.contains(&KeyCode::KeyS) { dir -= r.camera.forward(); }
            if self.pressed.contains(&KeyCode::KeyD) { dir += r.camera.right();   }
            if self.pressed.contains(&KeyCode::KeyA) { dir -= r.camera.right();   }
            if self.pressed.contains(&KeyCode::KeyE) { dir += Vec3::Y; }
            if self.pressed.contains(&KeyCode::KeyQ) { dir -= Vec3::Y; }

            if dir.length_squared() > 1e-6 {
                let step_scale = SPEED_STEPS[self.speed_step];
                let boost = if shift_down { 1.5 } else { 1.0 };
                let speed = self.base_speed * step_scale * boost;
                r.camera.eye += dir.normalize() * speed * dt;
            }
        }
    }

    fn render(&mut self) {
        let r = match self.renderer.as_mut() { Some(r) => r, None => return };
        let egui_state = match self.egui_winit_state.as_mut() { Some(s) => s, None => return };
        let egui_renderer = match self.egui_renderer.as_mut() { Some(er) => er, None => return };
        let window = match self.window.as_ref() { Some(w) => w, None => return };

        let mut net_instances = Vec::new();
        {
            let ents = self.net_ents.lock();
            for e in ents.iter() {
                let p = Vec3::new(e.px, e.py, e.pz);
                let q = Quat::from_xyzw(e.qx, e.qy, e.qz, e.qw);

                // THE FIX: The base mesh is a 1x1x1 unit cube.
                // Multiply the physics half-extents by 2.0 to get total visual width.
                let scale = Vec3::new(e.sx * 2.0, e.sy * 2.0, e.sz * 2.0);

                let kind = match e.kind {
                    1 => ShapeKind::Sphere { r: e.sx },
                    2 => ShapeKind::Box    { hx: e.sx, hy: e.sy, hz: e.sz },
                    3 => ShapeKind::CapsuleY { r: e.sx, hh: e.sy },
                    _ => ShapeKind::Sphere { r: 0.25 },
                };

                let color = if e.id == 99 {
                    Vec3::new(0.25, 0.25, 0.25)
                } else if e.id == 0 {
                    Vec3::new(1.0, 0.0, 0.0)
                } else {
                    Vec3::new(0.0, 0.5, 1.0)
                };

                net_instances.push(Instance {
                    model: trs(p, q, scale),
                    color,
                    kind,
                });
            }
        }

        let raw_input = egui_state.take_egui_input(window.as_ref());
        self.egui_ctx.begin_pass(raw_input);

        let mut local_gravity = self.target_gravity;
        let mut local_drop = self.target_drop_height;
        let mut force_reset = 0;

        egui::Window::new("URIEL OVERSIGHT")
            .fixed_pos(egui::pos2(20.0, 20.0))
            .show(&self.egui_ctx, |ui| {
                ui.heading("Physics Parameters");
                ui.add_space(5.0);

                ui.add(egui::Slider::new(&mut local_gravity, -20.0..=0.0).text("Gravity (y)"));
                ui.add(egui::Slider::new(&mut local_drop, 0.00..=1.0).text("Drop Height"));

                ui.add_space(10.0);

                if ui.button("EXECUTE FORCED RESET").clicked() {
                    force_reset = 1;
                }
            });

        let changed = (local_gravity != self.target_gravity) || (local_drop != self.target_drop_height);
        self.target_gravity = local_gravity;
        self.target_drop_height = local_drop;

        if changed || force_reset == 1 {
            if let Some(sock) = &self.oversight_socket {
                let cmd = OversightCommand {
                    gravity_y: self.target_gravity,
                    drop_height: self.target_drop_height,
                    force_reset,
                };

                let bytes: &[u8] = unsafe {
                    std::slice::from_raw_parts(
                        &cmd as *const _ as *const u8,
                        std::mem::size_of::<OversightCommand>()
                    )
                };
                let _ = sock.send_to(bytes, "127.0.0.1:49112");
            }
        }

        let full_output = self.egui_ctx.end_pass();
        let paint_jobs = self.egui_ctx.tessellate(full_output.shapes, full_output.pixels_per_point);
        let screen_descriptor = ScreenDescriptor {
            size_in_pixels: [r.width(), r.height()],
            pixels_per_point: window.scale_factor() as f32,
        };

        for (id, image_delta) in &full_output.textures_delta.set {
            egui_renderer.update_texture(r.device(), r.queue(), *id, image_delta);
        }

        match r.render(&net_instances, egui_renderer, &paint_jobs, &screen_descriptor) {
            Ok(()) => {}
            Err(wgpu::SurfaceError::Lost | wgpu::SurfaceError::Outdated) => {
                r.resize(window.inner_size().width.max(1), window.inner_size().height.max(1));
            }
            Err(wgpu::SurfaceError::OutOfMemory) => std::process::exit(1),
            Err(_) => {}
        }

        for id in &full_output.textures_delta.free {
            egui_renderer.free_texture(id);
        }
    }
}

impl ApplicationHandler for App {
    fn device_event(&mut self, _el: &ActiveEventLoop, _id: DeviceId, event: DeviceEvent) {
        if !self.rmb_down { return; }
        if let DeviceEvent::MouseMotion { delta: (dx, dy) } = event {
            if let Some(r) = self.renderer.as_mut() {
                r.camera.add_yaw_pitch(-(dx as f32) * 0.0018, -(dy as f32) * 0.0018);
            }
        }
    }

    fn resumed(&mut self, el: &ActiveEventLoop) {
        let attrs = WindowAttributes::default()
            .with_title("URIEL OVERSIGHT")
            .with_inner_size(LogicalSize::new(1280.0, 800.0));

        let window = Arc::new(el.create_window(attrs).expect("window"));
        el.listen_device_events(DeviceEvents::Always);

        let mut renderer = pollster::block_on(Renderer::new(window.clone()));
        renderer.resize(window.inner_size().width.max(1), window.inner_size().height.max(1));
        renderer.camera.eye   = Vec3::new(-6.0, 4.0, 6.0);
        renderer.camera.yaw   = 45_f32.to_radians();
        renderer.camera.pitch = (-15_f32).to_radians();

        let egui_winit_state = EguiWinitState::new(
            self.egui_ctx.clone(),
            egui::viewport::ViewportId::ROOT,
            &window,
            Some(window.scale_factor() as f32),
            None,
            None,
        );

        let egui_renderer = EguiRenderPass::new(renderer.device(), renderer.format(), None, 1, false);

        self.renderer = Some(renderer);
        self.window   = Some(window);
        self.egui_winit_state = Some(egui_winit_state);
        self.egui_renderer = Some(egui_renderer);
        self.last = Instant::now();
    }

    fn window_event(&mut self, el: &ActiveEventLoop, id: WindowId, event: WindowEvent) {
        if self.window.as_ref().map(|w| w.id()) != Some(id) { return; }

        if let Some(state) = self.egui_winit_state.as_mut() {
            let res = state.on_window_event(self.window.as_ref().unwrap(), &event);
            if res.consumed { return; }
        }

        match event {
            WindowEvent::CloseRequested => el.exit(),
            WindowEvent::Resized(sz) => self.on_resize(sz),
            WindowEvent::RedrawRequested => {
                let now = Instant::now();
                let dt_real = (now - self.last).as_secs_f32();
                self.last = now;

                self.update_camera(dt_real);
                self.render();
            }
            WindowEvent::KeyboardInput { event: ref ke, .. } => self.handle_key(ke, el),
            _ => self.handle_mouse(&event),
        }
    }

    fn about_to_wait(&mut self, _el: &ActiveEventLoop) {
        if let Some(sock) = &self.udp_socket {
            let mut buf = [0u32; 16384];
            let byte_buf = unsafe {
                std::slice::from_raw_parts_mut(buf.as_mut_ptr() as *mut u8, 65536)
            };

            while let Ok((amt, _)) = sock.recv_from(byte_buf) {
                let n_ents = amt / std::mem::size_of::<NetEnt>();
                let slice = unsafe {
                    std::slice::from_raw_parts(buf.as_ptr() as *const NetEnt, n_ents)
                };
                let mut out = self.net_ents.lock();
                out.clear();
                out.extend_from_slice(slice);
            }
        }

        if let Some(win) = self.window.as_ref() { win.request_redraw(); }
    }
}

fn main() -> anyhow::Result<()> {
    let event_loop = EventLoop::new()?;
    let mut app = App::new();
    event_loop.run_app(&mut app)?;
    Ok(())
}