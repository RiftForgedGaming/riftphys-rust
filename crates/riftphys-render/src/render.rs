use std::sync::Arc;
use winit::window::Window;
use wgpu::util::DeviceExt;

use crate::camera::{Camera, CameraUBO};
use crate::core_types::{Instance, InstanceRaw, ShapeKind, Vertex};
use crate::mesh_gen::{Mesh, cube_unit, uv_sphere, capsule_y, grid_lines_xz, upload_mesh, upload_lines};

pub const DEPTH_FMT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

pub(crate) fn make_depth(device: &wgpu::Device, w: u32, h: u32) -> (wgpu::Texture, wgpu::TextureView) {
    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("depth"),
        size: wgpu::Extent3d{ width: w.max(1), height: h.max(1), depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: DEPTH_FMT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    let view = tex.create_view(&wgpu::TextureViewDescriptor::default());
    (tex, view)
}

pub struct Renderer {
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue:  wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    depth:  (wgpu::Texture, wgpu::TextureView),

    mesh_pipe: wgpu::RenderPipeline,
    line_pipe: wgpu::RenderPipeline,

    cam_buf: wgpu::Buffer,
    cam_bind: wgpu::BindGroup,

    unit_box: Mesh,
    unit_sphere: Mesh,
    unit_capsule_y: Mesh,
    grid_lines: Mesh,

    pub camera: Camera,
}

impl Renderer {
    pub fn device(&self) -> &wgpu::Device { &self.device }
    pub fn queue(&self) -> &wgpu::Queue { &self.queue }
    pub fn format(&self) -> wgpu::TextureFormat { self.config.format }
    pub fn width(&self) -> u32 { self.config.width }
    pub fn height(&self) -> u32 { self.config.height }

    pub async fn new(window: Arc<Window>) -> Self {
        let size = window.inner_size();
        let instance = wgpu::Instance::default();

        let surface = instance.create_surface(window.clone())
            .expect("create wgpu surface");

        let adapter = instance.request_adapter(&wgpu::RequestAdapterOptions{
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: Some(&surface),
            force_fallback_adapter: false,
        }).await.expect("adapter");

        let (device, queue) = adapter.request_device(
            &wgpu::DeviceDescriptor {
                label: Some("device"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::downlevel_defaults().using_resolution(adapter.limits()),
                memory_hints: wgpu::MemoryHints::default(),
            },
            None
        ).await.expect("device");

        let format = surface.get_capabilities(&adapter).formats
            .into_iter().find(|f| f.is_srgb()).unwrap_or(wgpu::TextureFormat::Bgra8UnormSrgb);

        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format,
            width:  size.width.max(1),
            height: size.height.max(1),
            present_mode: wgpu::PresentMode::Fifo,
            alpha_mode:   wgpu::CompositeAlphaMode::Auto,
            view_formats: vec![format],
            desired_maximum_frame_latency: 2,
        };
        surface.configure(&device, &config);

        let (depth_tex, depth_view) = make_depth(&device, config.width, config.height);

        let cam_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("cam_bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                count: None,
            }],
        });

        let cam_buf = device.create_buffer(&wgpu::BufferDescriptor{
            label: Some("cam_ubo"),
            size: std::mem::size_of::<CameraUBO>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let cam_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("cam_bind"),
            layout: &cam_bgl,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: cam_buf.as_entire_binding() }],
        });

        let mesh_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor{
            label: Some("mesh.wgsl"),
            source: wgpu::ShaderSource::Wgsl(MESH_WGSL.into())
        });
        let line_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor{
            label: Some("line.wgsl"),
            source: wgpu::ShaderSource::Wgsl(LINE_WGSL.into())
        });

        let mesh_pipe = {
            let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor{
                label: Some("mesh_pl"),
                bind_group_layouts: &[&cam_bgl],
                push_constant_ranges: &[],
            });
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor{
                label: Some("mesh_pipe"),
                layout: Some(&pl),
                vertex: wgpu::VertexState {
                    module: &mesh_shader,
                    entry_point: "vs_main",
                    buffers: &[Vertex::layout(), InstanceRaw::layout()],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    module: &mesh_shader,
                    entry_point: "fs_main",
                    targets: &[Some(wgpu::ColorTargetState {
                        format,
                        blend: Some(wgpu::BlendState::REPLACE),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                }),
                primitive: wgpu::PrimitiveState {
                    topology: wgpu::PrimitiveTopology::TriangleList,
                    ..Default::default()
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH_FMT, depth_write_enabled: true, depth_compare: wgpu::CompareFunction::Less,
                    stencil: Default::default(), bias: Default::default()
                }),
                multisample: Default::default(),
                multiview: None,
                cache: None,
            })
        };

        let line_pipe = {
            let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor{
                label: Some("line_pl"),
                bind_group_layouts: &[&cam_bgl],
                push_constant_ranges: &[],
            });
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor{
                label: Some("line_pipe"),
                layout: Some(&pl),
                vertex: wgpu::VertexState {
                    module: &line_shader,
                    entry_point: "vs_main",
                    buffers: &[Vertex::layout()],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    module: &line_shader,
                    entry_point: "fs_main",
                    targets: &[Some(wgpu::ColorTargetState {
                        format,
                        blend: Some(wgpu::BlendState::REPLACE),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                }),
                primitive: wgpu::PrimitiveState {
                    topology: wgpu::PrimitiveTopology::LineList,
                    ..Default::default()
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH_FMT, depth_write_enabled: true, depth_compare: wgpu::CompareFunction::Less,
                    stencil: Default::default(), bias: Default::default()
                }),
                multisample: Default::default(),
                multiview: None,
                cache: None,
            })
        };

        let unit_box = upload_mesh(&device, &cube_unit());
        let unit_sphere = upload_mesh(&device, &uv_sphere(16, 16));
        let unit_capsule_y = upload_mesh(&device, &capsule_y(12, 6));
        let grid_lines = upload_lines(&device, &grid_lines_xz(200, 1.0));

        Self  {
            surface, device, queue, config, depth: (depth_tex, depth_view),
            mesh_pipe, line_pipe,
            cam_buf, cam_bind,
            unit_box, unit_sphere, unit_capsule_y, grid_lines,
            camera: Camera::new(),
        }
    }

    pub fn resize(&mut self, w: u32, h: u32) {
        if w == 0 || h == 0 { return; }
        self.config.width = w; self.config.height = h;
        self.surface.configure(&self.device, &self.config);
        self.depth = make_depth(&self.device, w, h);
        self.camera.aspect = w as f32 / h as f32;
    }

    pub fn render(
        &mut self,
        instances: &[Instance],
        egui_renderer: &mut egui_wgpu::Renderer,
        paint_jobs: &[egui::epaint::ClippedPrimitive],
        screen_descriptor: &egui_wgpu::ScreenDescriptor,
    ) -> Result<(), wgpu::SurfaceError> {
        let u = CameraUBO {
            view_proj: (self.camera.view_proj()).to_cols_array_2d(),
            eye_ws:    [self.camera.eye.x, self.camera.eye.y, self.camera.eye.z, 1.0],
            light_dir: [0.577, 0.577, 0.577, 0.0],
        };
        self.queue.write_buffer(&self.cam_buf, 0, bytemuck::bytes_of(&u));

        let frame = self.surface.get_current_texture()?;
        let view = frame.texture.create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor{ label: Some("encoder") });

        egui_renderer.update_buffers(&self.device, &self.queue, &mut encoder, paint_jobs, screen_descriptor);

        {
            let mut boxes    = Vec::<InstanceRaw>::new();
            let mut spheres  = Vec::<InstanceRaw>::new();
            let mut capsules = Vec::<InstanceRaw>::new();

            for inst in instances {
                match inst.kind {
                    ShapeKind::Box { .. }      => boxes.push(inst.raw()),
                    ShapeKind::Sphere { .. }   => spheres.push(inst.raw()),
                    ShapeKind::CapsuleY { .. } => capsules.push(inst.raw()),
                }
            }

            let boxes_buf = (!boxes.is_empty()).then(|| {
                self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("inst_buf.boxes"),
                    contents: bytemuck::cast_slice(&boxes),
                    usage: wgpu::BufferUsages::VERTEX,
                })
            });
            let spheres_buf = (!spheres.is_empty()).then(|| {
                self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("inst_buf.spheres"),
                    contents: bytemuck::cast_slice(&spheres),
                    usage: wgpu::BufferUsages::VERTEX,
                })
            });
            let capsules_buf = (!capsules.is_empty()).then(|| {
                self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("inst_buf.capsules"),
                    contents: bytemuck::cast_slice(&capsules),
                    usage: wgpu::BufferUsages::VERTEX,
                })
            });

            let mut rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("main_pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color{ r: 0.05, g: 0.06, b: 0.08, a: 1.0 }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth.1,
                    depth_ops: Some(wgpu::Operations {
                        load:  wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });

            rpass.set_pipeline(&self.line_pipe);
            rpass.set_bind_group(0, &self.cam_bind, &[]);
            rpass.set_vertex_buffer(0, self.grid_lines.vbuf.slice(..));
            rpass.set_index_buffer(self.grid_lines.ibuf.slice(..), wgpu::IndexFormat::Uint32);
            rpass.draw_indexed(0..self.grid_lines.icount, 0, 0..1);

            rpass.set_pipeline(&self.mesh_pipe);
            rpass.set_bind_group(0, &self.cam_bind, &[]);

            if let Some(buf) = &boxes_buf {
                rpass.set_vertex_buffer(0, self.unit_box.vbuf.slice(..));
                rpass.set_vertex_buffer(1, buf.slice(..));
                rpass.set_index_buffer(self.unit_box.ibuf.slice(..), wgpu::IndexFormat::Uint32);
                rpass.draw_indexed(0..self.unit_box.icount, 0, 0..(boxes.len() as u32));
            }

            if let Some(buf) = &spheres_buf {
                rpass.set_vertex_buffer(0, self.unit_sphere.vbuf.slice(..));
                rpass.set_vertex_buffer(1, buf.slice(..));
                rpass.set_index_buffer(self.unit_sphere.ibuf.slice(..), wgpu::IndexFormat::Uint32);
                rpass.draw_indexed(0..self.unit_sphere.icount, 0, 0..(spheres.len() as u32));
            }

            if let Some(buf) = &capsules_buf {
                rpass.set_vertex_buffer(0, self.unit_capsule_y.vbuf.slice(..));
                rpass.set_vertex_buffer(1, buf.slice(..));
                rpass.set_index_buffer(self.unit_capsule_y.ibuf.slice(..), wgpu::IndexFormat::Uint32);
                rpass.draw_indexed(0..self.unit_capsule_y.icount, 0, 0..(capsules.len() as u32));
            }
        }

        {
            // THE FIX: .forget_lifetime() detaches the compiler's strict link to the local encoder
            let mut egui_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("egui_pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations { load: wgpu::LoadOp::Load, store: wgpu::StoreOp::Store },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
            }).forget_lifetime();

            egui_renderer.render(&mut egui_pass, paint_jobs, screen_descriptor);
        }

        self.queue.submit(Some(encoder.finish()));
        frame.present();
        Ok(())
    }
}

const MESH_WGSL: &str = r#"
struct Camera {
  view_proj : mat4x4<f32>,
  eye_ws    : vec4<f32>,
  light_dir : vec4<f32>,
};
@group(0) @binding(0) var<uniform> uCam : Camera;

struct VsIn {
  @location(0) pos : vec3<f32>,
  @location(1) nrm : vec3<f32>,
  @location(2) i_m0 : vec4<f32>,
  @location(3) i_m1 : vec4<f32>,
  @location(4) i_m2 : vec4<f32>,
  @location(5) i_m3 : vec4<f32>,
  @location(6) i_color : vec3<f32>,
};
struct VsOut {
  @builtin(position) pos_cs : vec4<f32>,
  @location(0) n_ws : vec3<f32>,
  @location(1) p_ws : vec3<f32>,
  @location(2) color : vec3<f32>,
};
@vertex
fn vs_main(v: VsIn) -> VsOut {
  let M = mat4x4<f32>(v.i_m0, v.i_m1, v.i_m2, v.i_m3);
  let p_ws = (M * vec4<f32>(v.pos, 1.0)).xyz;

  let m0 = M[0].xyz;
  let m1 = M[1].xyz;
  let m2 = M[2].xyz;

  let sx = length(m0);
  let sy = length(m1);
  let sz = length(m2);

  let r0 = m0 / sx;
  let r1 = m1 / sy;
  let r2 = m2 / sz;

  let n_ws = normalize(mat3x3<f32>(r0 * (1.0 / sx), r1 * (1.0 / sy), r2 * (1.0 / sz)) * v.nrm);
  var o: VsOut;
  o.pos_cs = uCam.view_proj * vec4<f32>(p_ws, 1.0);
  o.n_ws = n_ws;
  o.p_ws = p_ws;
  o.color = v.i_color;
  return o;
}

@fragment
fn fs_main(i: VsOut) -> @location(0) vec4<f32> {
  let L = normalize(uCam.light_dir.xyz);
  let N = normalize(i.n_ws);
  let V = normalize(uCam.eye_ws.xyz - i.p_ws);
  let H = normalize(L + V);
  let ndotl = max(dot(N,L), 0.0);
  let diff = ndotl;
  let spec = pow(max(dot(N,H), 0.0), 32.0);
  let c = i.color * (0.08 + 0.92 * diff) + vec3<f32>(spec);
  return vec4<f32>(c, 1.0);
}
"#;

const LINE_WGSL: &str = r#"
struct Camera {
  view_proj : mat4x4<f32>,
  eye_ws    : vec4<f32>,
  light_dir : vec4<f32>,
};
@group(0) @binding(0) var<uniform> uCam : Camera;

struct VsIn {
  @location(0) pos : vec3<f32>,
  @location(1) nrm : vec3<f32>,
};
struct VsOut {
  @builtin(position) pos_cs : vec4<f32>,
  @location(0) color : vec3<f32>,
};
@vertex
fn vs_main(v: VsIn) -> VsOut {
  var o: VsOut;
  o.pos_cs = uCam.view_proj * vec4<f32>(v.pos, 1.0);
  o.color = vec3<f32>(0.2, 0.3, 0.45);
  return o;
}
@fragment
fn fs_main(i: VsOut) -> @location(0) vec4<f32> {
  return vec4<f32>(i.color, 1.0);
}
"#;