//! A small, deterministic CMA-ES (Hansen's covariance matrix adaptation evolution strategy) for
//! fitting a handful of physical parameters. It needs only objective values, which suits a
//! simulator with contacts (no useful gradients). Parameters are searched in the unit box
//! (each mapped linearly to [min, max]); candidates outside it are clamped.

/// Deterministic xorshift + Box-Muller normal source.
struct Normal { s: u64, spare: Option<f64> }

impl Normal {
    fn new(seed: u64) -> Self { Normal { s: seed.max(1), spare: None } }
    fn unit(&mut self) -> f64 {
        self.s ^= self.s << 13; self.s ^= self.s >> 7; self.s ^= self.s << 17;
        ((self.s >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    fn sample(&mut self) -> f64 {
        if let Some(z) = self.spare.take() { return z; }
        let (u, v) = (self.unit(), self.unit());
        let r = (-2.0 * u.ln()).sqrt();
        let t = 2.0 * std::f64::consts::PI * v;
        self.spare = Some(r * t.sin());
        r * t.cos()
    }
}

/// Symmetric eigen-decomposition by cyclic Jacobi rotations: (eigenvalues, eigenvectors as columns).
fn eigen(a: &[Vec<f64>]) -> (Vec<f64>, Vec<Vec<f64>>) {
    let n = a.len();
    let mut m = a.to_vec();
    let mut v = vec![vec![0.0; n]; n];
    for i in 0..n { v[i][i] = 1.0; }
    for _ in 0..100 {
        let off: f64 = (0..n).flat_map(|i| (0..n).filter(move |&j| j != i).map(move |j| (i, j))).map(|(i, j)| m[i][j] * m[i][j]).sum();
        if off < 1e-22 { break; }
        for p in 0..n {
            for q in p + 1..n {
                if m[p][q].abs() < 1e-300 { continue; }
                let theta = (m[q][q] - m[p][p]) / (2.0 * m[p][q]);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let t = if theta == 0.0 { 1.0 } else { t };
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..n {
                    let (mkp, mkq) = (m[k][p], m[k][q]);
                    m[k][p] = c * mkp - s * mkq;
                    m[k][q] = s * mkp + c * mkq;
                }
                for k in 0..n {
                    let (mpk, mqk) = (m[p][k], m[q][k]);
                    m[p][k] = c * mpk - s * mqk;
                    m[q][k] = s * mpk + c * mqk;
                }
                for k in 0..n {
                    let (vkp, vkq) = (v[k][p], v[k][q]);
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }
    ((0..n).map(|i| m[i][i].max(1e-20)).collect(), v)
}

pub struct FitResult {
    /// Best point in the unit box, its objective value, evaluations used
    pub x: Vec<f64>,
    pub f: f64,
    pub evals: usize,
    /// Best objective after each generation
    pub history: Vec<f64>,
}

/// Minimize `f` over the unit box [0, 1]^n from `x0`, initial step `sigma0`, at most `max_evals`.
pub fn minimize(mut f: impl FnMut(&[f64]) -> f64, x0: &[f64], sigma0: f64, max_evals: usize, seed: u64) -> FitResult {
    let n = x0.len();
    let mut rng = Normal::new(seed);
    let lambda = 4 + (3.0 * (n as f64).ln()).floor() as usize;
    let mu = lambda / 2;
    let raw: Vec<f64> = (0..mu).map(|i| ((mu as f64 + 0.5).ln() - ((i + 1) as f64).ln())).collect();
    let wsum: f64 = raw.iter().sum();
    let w: Vec<f64> = raw.iter().map(|x| x / wsum).collect();
    let mueff = 1.0 / w.iter().map(|x| x * x).sum::<f64>();
    let nf = n as f64;
    let cc = (4.0 + mueff / nf) / (nf + 4.0 + 2.0 * mueff / nf);
    let cs = (mueff + 2.0) / (nf + mueff + 5.0);
    let c1 = 2.0 / ((nf + 1.3).powi(2) + mueff);
    let cmu = (1.0 - c1).min(2.0 * (mueff - 2.0 + 1.0 / mueff) / ((nf + 2.0).powi(2) + mueff));
    let damps = 1.0 + 2.0 * (0.0f64).max(((mueff - 1.0) / (nf + 1.0)).sqrt() - 1.0) + cs;
    let chi_n = nf.sqrt() * (1.0 - 1.0 / (4.0 * nf) + 1.0 / (21.0 * nf * nf));

    let clamp = |x: &[f64]| -> Vec<f64> { x.iter().map(|v| v.clamp(0.0, 1.0)).collect() };
    let mut mean = clamp(x0);
    let mut sigma = sigma0;
    let mut c = vec![vec![0.0; n]; n];
    for i in 0..n { c[i][i] = 1.0; }
    let (mut pc, mut ps) = (vec![0.0; n], vec![0.0; n]);
    let mut best_x = mean.clone();
    let mut best_f = f(&best_x);
    let mut evals = 1;
    let mut history = vec![best_f];
    let mut gen = 0usize;

    while evals + lambda <= max_evals {
        gen += 1;
        let (d, b) = eigen(&c);
        let sd: Vec<f64> = d.iter().map(|x| x.sqrt()).collect();
        // Sample: y = B D z, x = mean + sigma y
        let mut pop: Vec<(f64, Vec<f64>, Vec<f64>)> = (0..lambda).map(|_| {
            let z: Vec<f64> = (0..n).map(|_| rng.sample()).collect();
            let y: Vec<f64> = (0..n).map(|i| (0..n).map(|k| b[i][k] * sd[k] * z[k]).sum()).collect();
            let x: Vec<f64> = (0..n).map(|i| mean[i] + sigma * y[i]).collect();
            let xc = clamp(&x);
            // Penalize clamping so the search stays inside the box
            let pen: f64 = x.iter().zip(&xc).map(|(a, b)| (a - b).powi(2)).sum();
            let fx = f(&xc) * (1.0 + pen);
            (fx, xc, y)
        }).collect();
        evals += lambda;
        pop.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        if pop[0].0 < best_f { best_f = pop[0].0; best_x = pop[0].1.clone(); }
        history.push(best_f);

        let old = mean.clone();
        let yw: Vec<f64> = (0..n).map(|i| (0..mu).map(|k| w[k] * pop[k].2[i]).sum()).collect();
        for i in 0..n { mean[i] = (old[i] + sigma * yw[i]).clamp(0.0, 1.0); }
        // C^(-1/2) yw = B D^-1 B^T yw
        let bty: Vec<f64> = (0..n).map(|k| (0..n).map(|i| b[i][k] * yw[i]).sum::<f64>() / sd[k]).collect();
        let cinv_y: Vec<f64> = (0..n).map(|i| (0..n).map(|k| b[i][k] * bty[k]).sum()).collect();
        for i in 0..n { ps[i] = (1.0 - cs) * ps[i] + (cs * (2.0 - cs) * mueff).sqrt() * cinv_y[i]; }
        let ps_norm = ps.iter().map(|x| x * x).sum::<f64>().sqrt();
        let hsig = ps_norm / (1.0 - (1.0 - cs).powi(2 * gen as i32)).sqrt() / chi_n < 1.4 + 2.0 / (nf + 1.0);
        for i in 0..n { pc[i] = (1.0 - cc) * pc[i] + if hsig { (cc * (2.0 - cc) * mueff).sqrt() * yw[i] } else { 0.0 }; }
        for i in 0..n {
            for j in 0..n {
                let rank_mu: f64 = (0..mu).map(|k| w[k] * pop[k].2[i] * pop[k].2[j]).sum();
                let delta = if hsig { 0.0 } else { cc * (2.0 - cc) * c[i][j] };
                c[i][j] = (1.0 - c1 - cmu) * c[i][j] + c1 * (pc[i] * pc[j] + delta) + cmu * rank_mu;
            }
        }
        sigma *= ((cs / damps) * (ps_norm / chi_n - 1.0)).exp();
        sigma = sigma.min(1.0);
        if sigma < 1e-8 { break; }
    }
    FitResult { x: best_x, f: best_f, evals, history }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_the_minimum_of_a_rotated_quadratic() {
        let target = [0.3, 0.7, 0.55];
        let f = |x: &[f64]| {
            let d: Vec<f64> = x.iter().zip(&target).map(|(a, b)| a - b).collect();
            d[0] * d[0] + 10.0 * (d[1] + 0.5 * d[0]).powi(2) + 100.0 * d[2] * d[2]
        };
        let r = minimize(f, &[0.5, 0.5, 0.5], 0.3, 2000, 7);
        for (a, b) in r.x.iter().zip(&target) { assert!((a - b).abs() < 1e-3, "{:?}", r.x); }
    }
}
