// crates/riftphys-vox/src/gpu_wgpu.rs
#![cfg(feature = "gpu-wgpu")] // <-- compile this file only when the feature is enabled

use std::sync::mpsc;
use std::time::Instant;

use bytemuck::{bytes_of, cast_slice, Pod, Zeroable};
use wgpu::util::DeviceExt;

use crate::{Ray, VoxelChunk};

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct HitGpu {
    pub t: f32,
    pub nx: f32,
    pub ny: f32,
    pub nz: f32,
    pub cx: u32,
    pub cy: u32,
    pub cz: u32,
    pub hit: u32,
}

pub struct GpuRaycaster {
    device: wgpu::Device,
    queue:  wgpu::Queue,
    pipeline: wgpu::ComputePipeline,
    bind_layout: wgpu::BindGroupLayout,
    _shader: wgpu::ShaderModule,
}

impl GpuRaycaster {
    pub fn new() -> Self {
        let instance = wgpu::Instance::default();
        let adapter = pollster::block_on(instance.request_adapter(
            &wgpu::RequestAdapterOptions::default(),
        ))
            .expect("wgpu adapter");

        let (device, queue) = pollster::block_on(adapter.request_device(
            &wgpu::DeviceDescriptor {
                label: Some("vox-device"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::downlevel_defaults()
                    .using_resolution(adapter.limits()),
                memory_hints: wgpu::MemoryHints::default(),
            },
            None,
        ))
            .expect("wgpu device");

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("vox-raycast"),
            source: wgpu::ShaderSource::Wgsl(include_str!("wgsl/raycast.wgsl").into()),
        });

        let bind_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("vox-bind-layout"),
            entries: &[
                // 0: voxels (RO storage)
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty:  wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // 1: cfg (uniform)
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty:  wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // 2: rays (RO storage)
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty:  wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // 3: hits (RW storage)
                wgpu::BindGroupLayoutEntry {
                    binding: 3,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty:  wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });

        let pipe_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("vox-pipeline-layout"),
            bind_group_layouts: &[&bind_layout],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("vox-pipeline"),
            layout: Some(&pipe_layout),
            module: &shader,
            entry_point: "main",
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            cache: None,
        });

        Self { device, queue, pipeline, bind_layout, _shader: shader }
    }

    /// Run compute kernel. Returns (wall-clock ms, hits)
    pub fn raycast_batch(&self, chunk: &VoxelChunk, rays: &[Ray]) -> (f64, Vec<HitGpu>) {
        // --- pack voxels BitVec -> u32 words (LSB-first)
        let nbits = chunk.solid.len();
        let mut words: Vec<u32> = vec![0u32; (nbits + 31) / 32];
        for i in 0..nbits {
            if chunk.solid[i] {
                let w = i / 32;
                let b = (i % 32) as u32;
                words[w] |= 1u32 << b;
            }
        }

        // --- cfg + rays structs
        #[repr(C)]
        #[derive(Clone, Copy, Pod, Zeroable)]
        struct Config { dims: [u32;4], params: [f32;4] }

        let cfg = Config {
            dims:   [chunk.dims.x, chunk.dims.y, chunk.dims.z, 0],
            params: [chunk.voxel_size, 0.0, 0.0, 0.0],
        };

        #[repr(C)]
        #[derive(Clone, Copy, Pod, Zeroable)]
        struct RayGpu { o: [f32;4], d_tmax: [f32;4] }

        let rays_gpu: Vec<RayGpu> = rays.iter().map(|r| RayGpu {
            o:      [r.o.x, r.o.y, r.o.z, 0.0],
            d_tmax: [r.d.x, r.d.y, r.d.z, r.tmax], // WGSL expects w=tmax
        }).collect();

        // --- buffers
        let vox_buf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label:    Some("voxels"),
            contents: cast_slice(&words),
            usage:    wgpu::BufferUsages::STORAGE,
        });

        let cfg_buf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label:    Some("cfg"),
            contents: bytes_of(&cfg),
            usage:    wgpu::BufferUsages::UNIFORM,
        });

        let ray_buf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label:    Some("rays"),
            contents: cast_slice(&rays_gpu),
            usage:    wgpu::BufferUsages::STORAGE,
        });

        let hit_bytes = (std::mem::size_of::<HitGpu>() as u64) * (rays.len() as u64);
        let hit_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("hits"),
            size:  hit_bytes,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let readback = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("hits-readback"),
            size:  hit_bytes,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label:  Some("vox-bind"),
            layout: &self.bind_layout,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: vox_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: cfg_buf.as_entire_binding()  },
                wgpu::BindGroupEntry { binding: 2, resource: ray_buf.as_entire_binding()  },
                wgpu::BindGroupEntry { binding: 3, resource: hit_buf.as_entire_binding()  },
            ],
        });

        // --- encode/submit
        let t0 = Instant::now();

        let mut enc = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("vox-enc"),
        });

        {
            let mut pass = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("vox-cpass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &bind, &[]);
            let wg = ((rays.len() as u32) + 63) / 64;
            pass.dispatch_workgroups(wg, 1, 1);
        }

        enc.copy_buffer_to_buffer(&hit_buf, 0, &readback, 0, hit_bytes);
        self.queue.submit(Some(enc.finish()));
        self.device.poll(wgpu::Maintain::Wait);

        let kernel_ms = t0.elapsed().as_secs_f64() * 1e3;

        // --- readback
        let slice = readback.slice(..);
        let (tx, rx) = mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |res| { let _ = tx.send(res); });
        self.device.poll(wgpu::Maintain::Wait);

        let hits = match rx.recv() {
            Ok(Ok(())) => {
                let data = slice.get_mapped_range();
                let out: Vec<HitGpu> = cast_slice(&data).to_vec();
                drop(data);
                readback.unmap();
                out
            }
            _ => Vec::new(),
        };

        (kernel_ms, hits)
    }
}
