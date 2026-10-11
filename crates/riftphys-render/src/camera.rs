use glam::{Mat4, Vec3};
use std::f32::consts::PI;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct CameraUBO {
    pub view_proj: [[f32; 4]; 4],
    pub eye_ws:    [f32; 4],
    pub light_dir: [f32; 4],
}

pub struct Camera {
    pub eye: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub fov_y_radians: f32,
    pub aspect: f32,
    pub znear: f32,
    pub zfar: f32,
}

impl Camera {
    pub fn new() -> Self {
        Self {
            eye: Vec3::new(-6.0, 4.0, 6.0),
            yaw: 45_f32.to_radians(),
            pitch: (-15_f32).to_radians(),
            fov_y_radians: 60_f32.to_radians(),
            aspect: 16.0/9.0,
            znear: 0.05,
            zfar: 1_000.0,
        }
    }
    pub fn view(&self) -> Mat4 {
        let dir = Vec3::new(self.yaw.cos()*self.pitch.cos(), self.pitch.sin(), self.yaw.sin()*self.pitch.cos());
        Mat4::look_at_rh(self.eye, self.eye + dir, Vec3::Y)
    }
    pub fn proj(&self) -> Mat4 {
        Mat4::perspective_rh(self.fov_y_radians, self.aspect.max(0.01), self.znear, self.zfar)
    }
    pub fn view_proj(&self) -> Mat4 { self.proj() * self.view() }
    pub fn forward(&self) -> Vec3 {
        Vec3::new(self.yaw.cos()*self.pitch.cos(), self.pitch.sin(), self.yaw.sin()*self.pitch.cos()).normalize()
    }
    pub fn right(&self) -> Vec3 { self.forward().cross(Vec3::Y).normalize() }
    pub fn up(&self) -> Vec3 { self.right().cross(self.forward()).normalize() }
    pub fn add_yaw_pitch(&mut self, dyaw: f32, dpitch: f32) {
        self.yaw = (self.yaw + dyaw + 2.0*PI) % (2.0*PI);
        self.pitch = (self.pitch + dpitch).clamp((-89_f32).to_radians(), (89_f32).to_radians());
    }
}