use wgpu::util::DeviceExt;
use riftphys_materials::materials as mats;
use riftphys_materials::vis::*;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GpuMatVisU {
    pub base_rough: [f32; 4],  // rgb, rough
    pub metal_pad:  [f32; 4],  // x = metal
}
unsafe impl bytemuck::Zeroable for GpuMatVisU {}
unsafe impl bytemuck::Pod      for GpuMatVisU {}

pub struct MatPalette {
    pub buf:    wgpu::Buffer,
    pub layout: wgpu::BindGroupLayout,
    pub bind:   wgpu::BindGroup,
}

pub fn build_palette(device: &wgpu::Device) -> MatPalette {
    let mut table = [GpuMatVisU { base_rough:[0.5,0.5,0.5,0.9], metal_pad:[0.0,0.0,0.0,0.0] }; 256];
    for i in 0..256 {
        let id = unsafe { std::mem::transmute::<u8, mats::MaterialId>(i as u8) };
        let v  = mat_vis(id);
        table[i] = GpuMatVisU {
            base_rough:[v.base_rgb[0], v.base_rgb[1], v.base_rgb[2], v.rough],
            metal_pad:[v.metal, 0.0, 0.0, 0.0]
        };
    }

    let buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("mat-palette-ub"),
        contents: bytemuck::cast_slice(&table),
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
    });

    let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("mat-palette-bgl"),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        }],
    });

    let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("mat-palette-bg"),
        layout: &layout,
        entries: &[wgpu::BindGroupEntry { binding: 0, resource: buf.as_entire_binding() }],
    });

    MatPalette { buf, layout, bind }
}