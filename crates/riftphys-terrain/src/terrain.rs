// File: crates/riftphys-terrain/src/terrain.rs

use glam::{UVec2, Vec2};
use riftphys_core::det::{q6, safe_contact_dist};
use riftphys_core::types::Vec3;
use riftphys_materials::materials::MaterialId;
use riftphys_collision::environment::{SpatialEnvironment, EnvCollider};
use riftphys_core::types::Isometry;
use riftphys_geom::{Shape, Triangle};

/// Regular grid heightfield. Heights are in world units (before any extra y_offset).
#[derive(Clone, Debug)]
pub struct HeightField {
    pub dims: UVec2,     // nx, nz (columns in x, rows in z)
    pub cell: Vec2,      // sx, sz (world units per cell)
    pub origin: Vec2,    // world-space origin offset (x, z)
    pub heights: Vec<f32>,
    pub min_y: f32,
    pub max_y: f32,
    pub material: MaterialId,
}

impl HeightField {
    pub fn from_heights(dims: UVec2, cell: Vec2, origin: Vec2, mut heights: Vec<f32>, material: MaterialId) -> Self {
        let nx = dims.x as usize;
        let nz = dims.y as usize;
        assert!(nx >= 1 && nz >= 1, "HeightField dims must be >= 1");
        assert_eq!(nx * nz, heights.len(), "HeightField heights len mismatch");

        // Quantize + sanitize cell sizes (no NaN/0)
        let sx = if cell.x.is_finite() && cell.x > 0.0 { q6(cell.x) } else { 1.0 };
        let sz = if cell.y.is_finite() && cell.y > 0.0 { q6(cell.y) } else { 1.0 };
        let cell = Vec2::new(sx, sz);

        let (mut min_y, mut max_y) = (f32::INFINITY, f32::NEG_INFINITY);

        for h in &mut heights {
            // NaN/inf in terrain is poison; clamp deterministically.
            let v = if h.is_finite() { *h } else { 0.0 };
            *h = q6(v);
            min_y = min_y.min(*h);
            max_y = max_y.max(*h);
        }

        // Quantize bounds too (so derived stuff doesn’t differ by ulps)
        min_y = q6(min_y);
        max_y = q6(max_y);

        Self { dims, cell, origin, heights, min_y, max_y, material }
    }

    pub fn from_i16_grid_strided(
        width: u32,
        height: u32,
        stride: u32,
        cell: Vec2,
        heights_i16: &[i16],
        height_scale: f32,
        height_offset: f32,
        material: MaterialId,
    ) -> Result<Self, String> {
        if width == 0 || height == 0 {
            return Err("HeightField::from_i16_grid_strided: width/height == 0".into());
        }

        let stride = if stride == 0 { width } else { stride };
        if stride < width {
            return Err("HeightField::from_i16_grid_strided: stride < width".into());
        }

        let need = (stride as usize)
            .checked_mul(height as usize)
            .ok_or("HeightField::from_i16_grid_strided: stride*height overflow")?;

        if heights_i16.len() < need {
            return Err(format!(
                "HeightField::from_i16_grid_strided: heights len {} < needed {}",
                heights_i16.len(),
                need
            ));
        }

        // sanitize scale/offset deterministically
        let hs = if height_scale.is_finite() { height_scale } else { 0.0 };
        let ho = if height_offset.is_finite() { height_offset } else { 0.0 };

        // Convert to contiguous Vec<f32> of len width*height (no padding)
        let mut heights = Vec::<f32>::with_capacity((width as usize) * (height as usize));
        for z in 0..(height as usize) {
            let row = z * (stride as usize);
            for x in 0..(width as usize) {
                let s = heights_i16[row + x] as f32;
                let h = s * hs + ho;
                // from_heights() will sanitize + q6 quantize deterministically
                heights.push(h);
            }
        }

        Ok(Self::from_heights(UVec2::new(width, height), cell, Vec2::ZERO, heights, material))
    }

    #[cfg(feature = "image")]
    pub fn from_png_bytes(png: &[u8], cell: Vec2, y_scale: f32, material: MaterialId) -> image::ImageResult<Self> {
        let img = image::load_from_memory(png)?.to_luma8();
        let (w, h) = img.dimensions();
        let mut heights = Vec::with_capacity((w * h) as usize);

        let ys = if y_scale.is_finite() { y_scale } else { 0.0 };

        for z in 0..h {
            for x in 0..w {
                let v = img.get_pixel(x, z).0[0] as f32 / 255.0;
                heights.push(v * ys);
            }
        }

        Ok(Self::from_heights(UVec2::new(w, h), cell, Vec2::ZERO, heights, material))
    }

    /// RAW32 (little-endian f32), square, mapped onto a world-space tile.
    pub fn from_raw32_square(bytes: &[u8], world_size: Vec2, material: MaterialId) -> Result<Self, String> {
        if bytes.len() % 4 != 0 {
            return Err(format!("raw32 length {} not multiple of 4", bytes.len()));
        }
        let samples = bytes.len() / 4;

        let n_f = (samples as f64).sqrt();
        let n = n_f.round() as u32;
        if (n as usize) * (n as usize) != samples {
            return Err(format!("raw32 samples {} not a perfect square", samples));
        }

        // Validate world_size early; don't silently coerce to 0.
        if !(world_size.x.is_finite() && world_size.y.is_finite()) {
            return Err(format!("hf bad world_size (non-finite): {:?}", world_size));
        }
        if !(world_size.x > 0.0 && world_size.y > 0.0) {
            return Err(format!("hf bad world_size (must be >0): {:?}", world_size));
        }

        let mut heights = Vec::with_capacity(samples);
        for chunk in bytes.chunks_exact(4) {
            let mut h = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            if !h.is_finite() { h = 0.0; }
            heights.push(h);
        }

        let side = n.max(1);
        let dims = UVec2::new(side, side);

        // side = number of samples per axis (e.g. 4096)
        if side < 2 {
            return Err(format!("hf side must be >= 2 (got {})", side));
        }

        let denom = (side as f32) - 1.0; // NO clamping
        let cell_x = world_size.x / denom;
        let cell_z = world_size.y / denom;

        if !(cell_x.is_finite() && cell_z.is_finite() && cell_x > 0.0 && cell_z > 0.0) {
            return Err(format!(
                "hf bad cell size: side={} world_size={:?} cell=({},{})",
                side, world_size, cell_x, cell_z
            ));
        }

        let cell = Vec2::new(cell_x, cell_z);

        Ok(Self::from_heights(dims, cell, Vec2::ZERO, heights, material))
    }

    #[inline]
    fn idx(&self, x: i32, z: i32) -> usize {
        (x as usize) + (z as usize) * (self.dims.x as usize)
    }

    #[inline]
    fn h(&self, x: i32, z: i32) -> f32 {
        self.heights[self.idx(x, z)]
    }

    #[inline]
    fn clampf(x: f32, lo: f32, hi: f32) -> f32 {
        let (lo, hi) = if hi >= lo { (lo, hi) } else { (lo, lo) };
        x.max(lo).min(hi)
    }

    pub fn sample_height(&self, x: f32, z: f32) -> f32 {
        let nx = self.dims.x as i32;
        let nz = self.dims.y as i32;

        if nx <= 1 && nz <= 1 {
            return self.heights[0];
        }

        let sx = if self.cell.x.is_finite() && self.cell.x > 0.0 { self.cell.x } else { 1.0 };
        let sz = if self.cell.y.is_finite() && self.cell.y > 0.0 { self.cell.y } else { 1.0 };

        // Convert world position into local grid space
        let local_x = x - self.origin.x;
        let local_z = z - self.origin.y;

        let max_fx = (nx - 1) as f32 - 1.0e-6;
        let max_fz = (nz - 1) as f32 - 1.0e-6;

        let fx = Self::clampf(local_x / sx, 0.0, max_fx);
        let fz = Self::clampf(local_z / sz, 0.0, max_fz);

        let x0 = fx.floor() as i32; let x1 = (x0 + 1).min(nx - 1);
        let z0 = fz.floor() as i32; let z1 = (z0 + 1).min(nz - 1);

        // Sampling is not rounded (PRECISION_SPEC P1); the heights themselves were, on load.
        let tx = fx - x0 as f32;
        let tz = fz - z0 as f32;

        let h00 = self.h(x0, z0);
        let h10 = self.h(x1, z0);
        let h01 = self.h(x0, z1);
        let h11 = self.h(x1, z1);

        let a = h00 * (1.0 - tx) + h10 * tx;
        let b = h01 * (1.0 - tx) + h11 * tx;
        let y = a * (1.0 - tz) + b * tz;
        if y.is_finite() { y } else { 0.0 }
    }

    pub fn sample_normal(&self, x: f32, z: f32) -> Vec3 {
        let nx = self.dims.x as i32;
        let nz = self.dims.y as i32;
        if nx <= 1 || nz <= 1 {
            return Vec3::new(0.0, 1.0, 0.0);
        }

        let sx = self.cell.x.max(1.0e-6);
        let sz = self.cell.y.max(1.0e-6);

        let hx0 = self.sample_height(x - sx, z);
        let hx1 = self.sample_height(x + sx, z);
        let hz0 = self.sample_height(x, z - sz);
        let hz1 = self.sample_height(x, z + sz);

        let ddx = (hx1 - hx0) * (0.5 / sx);
        let ddz = (hz1 - hz0) * (0.5 / sz);

        let n = Vec3::new(-ddx, 1.0, -ddz);
        let n = n.normalize_or_zero();
        if n.is_finite() && n != Vec3::ZERO { n } else { Vec3::Y }
    }

    #[inline]
    pub fn contact_dist(cd: f32) -> f32 {
        safe_contact_dist(cd)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sample_height_no_panic_on_1x1() {
        let hf = HeightField::from_heights(UVec2::new(1, 1), Vec2::new(1.0, 1.0), Vec2::ZERO, vec![2.0], MaterialId::Default);
        assert_eq!(hf.sample_height(0.0, 0.0), 2.0);
        let n = hf.sample_normal(0.0, 0.0);
        assert!((n.y - 1.0).abs() < 1.0e-6);
    }

    #[test]
    fn bilinear_basic() {
        let hf = HeightField::from_heights(
            UVec2::new(2, 2),
            Vec2::new(1.0, 1.0),
            Vec2::ZERO,
            vec![0.0, 1.0,
                 2.0, 3.0],
            MaterialId::Default
        );
        let h = hf.sample_height(0.5, 0.5);
        assert!((h - 1.5).abs() < 1.0e-6);
    }

    #[test]
    fn normal_is_unitish() {
        let hf = HeightField::from_heights(
            UVec2::new(3, 3),
            Vec2::new(1.0, 1.0),
            Vec2::ZERO,
            vec![0.0, 0.0, 0.0,
                 0.0, 1.0, 2.0,
                 0.0, 2.0, 4.0],
            MaterialId::Default
        );
        let n = hf.sample_normal(1.0, 1.0);
        let len = (n.x*n.x + n.y*n.y + n.z*n.z).sqrt();
        assert!((len - 1.0).abs() < 1.0e-3);
    }
}

impl SpatialEnvironment for HeightField {
    fn query_aabb(&self, aabb: &riftphys_geom::Aabb, out: &mut Vec<EnvCollider>) {
        let nx = self.dims.x as i32;
        let nz = self.dims.y as i32;

        if nx < 2 || nz < 2 { return; }

        // Shift world-space AABB to local-space bounds for grid querying
        let local_min_x = aabb.min.x - self.origin.x;
        let local_max_x = aabb.max.x - self.origin.x;
        let local_min_z = aabb.min.z - self.origin.y;
        let local_max_z = aabb.max.z - self.origin.y;

        let min_gx = ((local_min_x / self.cell.x).floor() as i32 - 1).clamp(0, nx - 2);
        let max_gx = ((local_max_x / self.cell.x).ceil() as i32 + 1).clamp(0, nx - 2);
        let min_gz = ((local_min_z / self.cell.y).floor() as i32 - 1).clamp(0, nz - 2);
        let max_gz = ((local_max_z / self.cell.y).ceil() as i32 + 1).clamp(0, nz - 2);

        if aabb.min.y > self.max_y + 1.0 || aabb.max.y < self.min_y - 1.0 {
            return;
        }

        for gz in min_gz..=max_gz {
            for gx in min_gx..=max_gx {
                let h00 = self.h(gx, gz);
                let h10 = self.h(gx + 1, gz);
                let h01 = self.h(gx, gz + 1);
                let h11 = self.h(gx + 1, gz + 1);

                // Re-apply origin shift to push local grid coordinates back into world-space
                let x0 = (gx as f32) * self.cell.x + self.origin.x;
                let x1 = x0 + self.cell.x;
                let z0 = (gz as f32) * self.cell.y + self.origin.y;
                let z1 = z0 + self.cell.y;

                let p00 = Vec3::new(x0, h00, z0);
                let p10 = Vec3::new(x1, h10, z0);
                let p01 = Vec3::new(x0, h01, z1);
                let p11 = Vec3::new(x1, h11, z1);

                let tri1 = Triangle::new(p00, p01, p10);
                let tri2 = Triangle::new(p10, p01, p11);

                out.push(EnvCollider {
                    shape: Shape::Triangle(tri1),
                    transform: Isometry::IDENTITY,
                    material: self.material,
                    user_data: 0, // Safe default for raw static terrain
                });

                out.push(EnvCollider {
                    shape: Shape::Triangle(tri2),
                    transform: Isometry::IDENTITY,
                    material: self.material,
                    user_data: 0, // Safe default for raw static terrain
                });
            }
        }
    }
}

#[test]
fn test_heightfield_yields_triangles() {
    let hf = HeightField::from_heights(
        UVec2::new(100, 100),
        Vec2::new(4.0, 4.0),
        Vec2::ZERO,
        vec![0.0; 10000], // Flat ground at Y=0
        MaterialId::Default
    );

    let mut out = Vec::new();

    // 1. Test Sky AABB (Should be culled)
    let sky_aabb = riftphys_geom::Aabb {
        min: riftphys_core::types::Vec3::new(199.0, 49.0, 199.0),
        max: riftphys_core::types::Vec3::new(201.0, 51.0, 201.0),
    };
    hf.query_aabb(&sky_aabb, &mut out);
    assert_eq!(out.len(), 0, "Sky AABB should be culled by HeightField max_y bounds");

    // 2. Test Ground Intersecting AABB (Should return triangles)
    let ground_aabb = riftphys_geom::Aabb {
        min: riftphys_core::types::Vec3::new(199.0, -1.0, 199.0),
        max: riftphys_core::types::Vec3::new(201.0, 1.0, 201.0),
    };
    hf.query_aabb(&ground_aabb, &mut out);

    assert!(out.len() > 0, "HeightField failed to yield triangles for an overlapping AABB!");

    println!("Test Passed! Yielded {} triangles.", out.len());
    for (i, col) in out.iter().enumerate() {
        if let Shape::Triangle(tri) = &col.shape {
            println!("Tri {}: A={:?}, B={:?}, C={:?}", i, tri.a, tri.b, tri.c);
        }
    }
}