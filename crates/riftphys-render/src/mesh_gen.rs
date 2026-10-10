use glam::Vec3;
use std::f32::consts::PI;
use wgpu::util::DeviceExt;
use crate::core_types::Vertex;

pub(crate) struct Mesh {
    pub vbuf: wgpu::Buffer,
    pub ibuf: wgpu::Buffer,
    pub icount: u32,
}

pub(crate) fn upload_mesh(device: &wgpu::Device, (vtx, idx): &(Vec<Vertex>, Vec<u32>)) -> Mesh {
    let vbuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("mesh_vb"),
        contents: bytemuck::cast_slice(vtx.as_slice()),
        usage: wgpu::BufferUsages::VERTEX,
    });
    let ibuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("mesh_ib"),
        contents: bytemuck::cast_slice(idx.as_slice()),
        usage: wgpu::BufferUsages::INDEX,
    });
    Mesh { vbuf, ibuf, icount: idx.len() as u32 }
}

pub(crate) fn upload_lines(device: &wgpu::Device, (vtx, idx): &(Vec<Vertex>, Vec<u32>)) -> Mesh {
    upload_mesh(device, &(vtx.clone(), idx.clone()))
}

pub(crate) fn cube_unit() -> (Vec<Vertex>, Vec<u32>) {
    let p = [
        (Vec3::new(1.,-1.,-1.), Vec3::X), (Vec3::new(1.,-1., 1.), Vec3::X),
        (Vec3::new(1., 1., 1.), Vec3::X), (Vec3::new(1., 1.,-1.), Vec3::X),
        (Vec3::new(-1.,-1., 1.), -Vec3::X), (Vec3::new(-1.,-1.,-1.), -Vec3::X),
        (Vec3::new(-1., 1.,-1.), -Vec3::X), (Vec3::new(-1., 1., 1.), -Vec3::X),
        (Vec3::new(-1., 1.,-1.), Vec3::Y), (Vec3::new(1., 1.,-1.), Vec3::Y),
        (Vec3::new(1., 1., 1.), Vec3::Y), (Vec3::new(-1., 1., 1.), Vec3::Y),
        (Vec3::new(-1.,-1., 1.), -Vec3::Y), (Vec3::new(1.,-1., 1.), -Vec3::Y),
        (Vec3::new(1.,-1.,-1.), -Vec3::Y), (Vec3::new(-1.,-1.,-1.), -Vec3::Y),
        (Vec3::new(-1.,-1., 1.), Vec3::Z), (Vec3::new(-1., 1., 1.), Vec3::Z),
        (Vec3::new(1., 1., 1.), Vec3::Z), (Vec3::new(1.,-1., 1.), Vec3::Z),
        (Vec3::new(1.,-1.,-1.), -Vec3::Z), (Vec3::new(1., 1.,-1.), -Vec3::Z),
        (Vec3::new(-1., 1.,-1.), -Vec3::Z), (Vec3::new(-1.,-1.,-1.), -Vec3::Z),
    ];
    let mut v = Vec::with_capacity(24);
    for (pos, nrm) in p { v.push(Vertex{ pos: pos.to_array(), nrm: nrm.to_array() }); }
    let mut i = Vec::new();
    for f in 0..6 {
        let b = (f*4) as u32;
        i.extend_from_slice(&[b, b+1, b+2,  b, b+2, b+3]);
    }
    (v, i)
}

pub(crate) fn uv_sphere(segments: u32, rings: u32) -> (Vec<Vertex>, Vec<u32>) {
    let seg = segments.max(3);
    let rng = rings.max(2);
    let mut v = Vec::new();
    for y in 0..=rng {
        let vty = y as f32 / rng as f32;
        let phi = vty * PI;
        for x in 0..=seg {
            let vtx = x as f32 / seg as f32;
            let theta = vtx * 2.0*PI;
            let n = Vec3::new(theta.cos()*phi.sin(), phi.cos(), theta.sin()*phi.sin());
            v.push(Vertex{ pos: n.to_array(), nrm: n.to_array() });
        }
    }
    let mut i = Vec::new();
    let stride = seg + 1;
    for y in 0..rng {
        for x in 0..seg {
            let a = y*stride + x;
            let b = a + 1;
            let c = a + stride;
            let d = c + 1;
            i.extend_from_slice(&[a, c, b,  b, c, d].map(|k| k as u32));
        }
    }
    (v, i)
}

pub(crate) fn capsule_y(segments: u32, rings_half: u32) -> (Vec<Vertex>, Vec<u32>) {
    let seg = segments.max(6);
    let rng = rings_half.max(3);
    let mut v = Vec::new();
    let mut i = Vec::new();

    let y0 = -1.0_f32;
    let y1 =  1.0_f32;
    for y in [y0, y1] {
        for x in 0..=seg {
            let t = (x as f32 / seg as f32) * 2.0*PI;
            let n = Vec3::new(t.cos(), 0.0, t.sin());
            v.push(Vertex{ pos: [n.x, y, n.z], nrm: [n.x, 0.0, n.z] });
        }
    }
    let stride = seg + 1;
    for x in 0..seg {
        let a = x;
        let b = x + 1;
        let c = stride + x;
        let d = stride + x + 1;
        i.extend_from_slice(&[a, c, b,  b, c, d].map(|k| k as u32));
    }

    let base_top = v.len() as u32;
    for hemi in 0..2 {
        let sign = if hemi==0 { -1.0 } else { 1.0 };
        let y_center = if hemi==0 { y0 } else { y1 };
        for ry in 1..=rng {
            let vty = ry as f32 / (rng as f32 + 1.0);
            let phi = vty * 0.5*PI;
            let cy = y_center + sign * phi.sin();
            let r = phi.cos();
            for x in 0..=seg {
                let t = (x as f32 / seg as f32) * 2.0*PI;
                let n = Vec3::new(t.cos()*r, sign*phi.sin(), t.sin()*r).normalize();
                let pos = Vec3::new(t.cos()*r, cy, t.sin()*r);
                v.push(Vertex{ pos: pos.to_array(), nrm: n.to_array() });
            }
        }
        let rows = rng;
        for r in 0..rows {
            for x in 0..seg {
                let a = base_top + (r*stride + x) as u32;
                let b = a + 1;
                let c = a + stride as u32;
                let d = c + 1;
                i.extend_from_slice(&[a, c, b,  b, c, d]);
            }
        }
    }
    (v, i)
}

pub(crate) fn grid_lines_xz(half: i32, step: f32) -> (Vec<Vertex>, Vec<u32>) {
    let mut v = Vec::new();
    let mut idx = Vec::new();
    let mut push_line = |a: Vec3, b: Vec3| {
        let base = v.len() as u32;
        v.push(Vertex{ pos: a.to_array(), nrm: [0.0, 1.0, 0.0] });
        v.push(Vertex{ pos: b.to_array(), nrm: [0.0, 1.0, 0.0] });
        idx.extend_from_slice(&[base, base+1]);
    };
    for i in -half..=half {
        let x = i as f32 * step;
        push_line(Vec3::new(x, 0.0, -half as f32 * step), Vec3::new(x, 0.0, half as f32 * step));
    }
    for k in -half..=half {
        let z = k as f32 * step;
        push_line(Vec3::new(-half as f32 * step, 0.0, z), Vec3::new(half as f32 * step, 0.0, z));
    }
    (v, idx)
}
