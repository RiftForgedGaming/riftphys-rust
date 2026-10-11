use glam::{Mat4, Vec3, Quat};

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub pos: [f32; 3],
    pub nrm: [f32; 3],
}
impl Vertex {
    pub const ATTRS: [wgpu::VertexAttribute; 2] = wgpu::vertex_attr_array![0=>Float32x3, 1=>Float32x3];
    pub fn layout<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRS,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct InstanceRaw {
    pub m0: [f32;4], pub m1: [f32;4], pub m2: [f32;4], pub m3: [f32;4],
    pub color: [f32;3], pub _pad: f32,
}
impl InstanceRaw {
    pub const ATTRS: [wgpu::VertexAttribute; 5] = [
        wgpu::VertexAttribute { shader_location: 2, offset: 0,  format: wgpu::VertexFormat::Float32x4 },
        wgpu::VertexAttribute { shader_location: 3, offset: 16, format: wgpu::VertexFormat::Float32x4 },
        wgpu::VertexAttribute { shader_location: 4, offset: 32, format: wgpu::VertexFormat::Float32x4 },
        wgpu::VertexAttribute { shader_location: 5, offset: 48, format: wgpu::VertexFormat::Float32x4 },
        wgpu::VertexAttribute { shader_location: 6, offset: 64, format: wgpu::VertexFormat::Float32x3 },
    ];
    pub fn layout<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: 16*4 + 16,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &Self::ATTRS,
        }
    }
}

pub enum ShapeKind {
    Box { hx: f32, hy: f32, hz: f32 },
    Sphere { r: f32 },
    CapsuleY { r: f32, hh: f32 },
}

pub struct Instance {
    pub model: Mat4,
    pub color: Vec3,
    pub kind: ShapeKind,
}
impl Instance {
    pub(crate) fn raw(&self) -> InstanceRaw {
        let (m0, m1, m2, m3) = (self.model.x_axis, self.model.y_axis, self.model.z_axis, self.model.w_axis);
        InstanceRaw {
            m0: m0.to_array(), m1: m1.to_array(), m2: m2.to_array(), m3: m3.to_array(),
            color: self.color.to_array(), _pad: 0.0
        }
    }
}

pub fn trs(position: Vec3, rotation: Quat, scale: Vec3) -> Mat4 {
    Mat4::from_translation(position) * Mat4::from_quat(rotation) * Mat4::from_scale(scale)
}