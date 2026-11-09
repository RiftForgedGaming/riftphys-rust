// SPDX-License-Identifier: RiftForged Proprietary License
//! PARITY_SPEC W6's bounce passes, the numerics: the contact matrix K = J M^-1 J^T of one
//! connected component of rows, held sparse (rows couple only through a shared body), a sparse
//! Cholesky in a fixed minimum-degree order, the velocity pass's interior-point LCP solve and the
//! pseudo pass's equality solve. f64 throughout, every sum in a fixed order, so the bits are
//! the same on every run. A dense factorisation is kept as the reference: with the check on
//! (`World::set_bounce_dense_reference`, tests only), the first Newton system of every velocity
//! solve is also solved densely and the difference reported; every solve reports its backward
//! error.

/// A symmetric sparse matrix, both triangles stored, each row's columns ascending (the diagonal
/// always present).
#[derive(Clone, Debug, Default)]
pub(super) struct Csr {
    pub n: usize,
    pub ptr: Vec<usize>,
    pub idx: Vec<usize>,
    pub val: Vec<f64>,
}

impl Csr {
    /// Builds from per-row (column, value) lists, columns ascending
    pub fn from_rows(rows: Vec<Vec<(usize, f64)>>) -> Csr {
        let n = rows.len();
        let mut m = Csr { n, ptr: Vec::with_capacity(n + 1), idx: Vec::new(), val: Vec::new() };
        m.ptr.push(0);
        for r in rows {
            for (c, v) in r { m.idx.push(c); m.val.push(v); }
            m.ptr.push(m.idx.len());
        }
        m
    }

    pub fn nnz(&self) -> usize { self.idx.len() }

    /// K + diag(c K_ii): a compliance, so the matrix is positive definite
    pub fn with_compliance(&self, c: f64) -> Csr {
        let mut m = self.clone();
        for i in 0..self.n {
            for t in m.ptr[i]..m.ptr[i + 1] { if m.idx[t] == i { m.val[t] *= 1.0 + c; } }
        }
        m
    }

    pub fn diag(&self, i: usize) -> f64 {
        (self.ptr[i]..self.ptr[i + 1]).find(|&t| self.idx[t] == i).map_or(0.0, |t| self.val[t])
    }

    /// (K x)_i
    pub fn row_dot(&self, i: usize, x: &[f64]) -> f64 {
        let mut s = 0.0f64;
        for t in self.ptr[i]..self.ptr[i + 1] { s += self.val[t] * x[self.idx[t]]; }
        s
    }

    pub fn mul(&self, x: &[f64]) -> Vec<f64> { (0..self.n).map(|i| self.row_dot(i, x)).collect() }

    fn dense(&self) -> Vec<f64> {
        let mut d = vec![0.0f64; self.n * self.n];
        for i in 0..self.n { for t in self.ptr[i]..self.ptr[i + 1] { d[i * self.n + self.idx[t]] = self.val[t]; } }
        d
    }
}

/// The elimination order and the factor's structure, from the row graph alone (never from
/// values): minimum degree, the lowest row index on ties (an ordered set keyed by degree, then
/// index). Column k of L (in elimination positions) holds entries `cp[k]..cp[k + 1]`, at
/// positions `ri`, ascending; `re[rp[p]..rp[p + 1]]` lists the entries at position p, columns
/// ascending, and `ce[e]` is the end of entry e's column.
#[derive(Clone, Debug, Default)]
pub(super) struct Symbolic {
    pub order: Vec<usize>,
    pub pos: Vec<usize>,
    cp: Vec<usize>,
    ri: Vec<usize>,
    rp: Vec<usize>,
    re: Vec<usize>,
    ce: Vec<usize>,
}

impl Symbolic {
    pub fn min_degree(k: &Csr) -> Symbolic {
        let n = k.n;
        let mut adj: Vec<Vec<usize>> = (0..n).map(|i| (k.ptr[i]..k.ptr[i + 1]).map(|t| k.idx[t]).filter(|&j| j != i).collect()).collect();
        let mut queue: std::collections::BTreeSet<(usize, usize)> = (0..n).map(|i| (adj[i].len(), i)).collect();
        let mut order = Vec::with_capacity(n);
        let mut nbs: Vec<Vec<usize>> = vec![Vec::new(); n];
        let mut merged: Vec<usize> = Vec::new();
        while let Some((_, v)) = queue.pop_first() {
            order.push(v);
            let nb = std::mem::take(&mut adj[v]);
            // The neighbours become a clique; v leaves their lists
            for &u in &nb {
                queue.remove(&(adj[u].len(), u));
                merged.clear();
                let (a, b) = (&adj[u], &nb);
                let (mut x, mut y) = (0, 0);
                while x < a.len() || y < b.len() {
                    let next = if y >= b.len() || (x < a.len() && a[x] < b[y]) { x += 1; a[x - 1] }
                        else if x >= a.len() || b[y] < a[x] { y += 1; b[y - 1] }
                        else { x += 1; y += 1; a[x - 1] };
                    if next != v && next != u { merged.push(next); }
                }
                std::mem::swap(&mut adj[u], &mut merged);
                queue.insert((adj[u].len(), u));
            }
            nbs[v] = nb;
        }
        let mut pos = vec![0usize; n];
        for (p, &r) in order.iter().enumerate() { pos[r] = p; }
        let mut cp = Vec::with_capacity(n + 1);
        let mut ri = Vec::new();
        cp.push(0);
        for &r in &order {
            let start = ri.len();
            ri.extend(nbs[r].iter().map(|&u| pos[u]));
            ri[start..].sort_unstable();
            cp.push(ri.len());
        }
        let mut count = vec![0usize; n + 1];
        for &p in &ri { count[p + 1] += 1; }
        for p in 0..n { count[p + 1] += count[p]; }
        let rp = count.clone();
        let mut re = vec![0usize; ri.len()];
        let mut ce = vec![0usize; ri.len()];
        let mut fill = count;
        for kc in 0..n {
            for e in cp[kc]..cp[kc + 1] {
                re[fill[ri[e]]] = e;
                fill[ri[e]] += 1;
                ce[e] = cp[kc + 1];
            }
        }
        Symbolic { order, pos, cp, ri, rp, re, ce }
    }

    /// Entries below the diagonal in L
    pub fn l_nnz(&self) -> usize { self.ri.len() }
}

/// A factor of K + diag(d): sparse (L in the symbolic order: its diagonal and the entries
/// below it, flat), or dense (the test reference)
enum Factor {
    Sparse { diag: Vec<f64>, vals: Vec<f64> },
    Dense(Vec<f64>),
}

fn factor(k: &Csr, sym: &Symbolic, d: &[f64], dense: bool) -> Factor {
    let n = k.n;
    if dense {
        let mut m = k.dense();
        for i in 0..n { m[i * n + i] += d[i]; }
        for i in 0..n {
            for j in 0..=i {
                let mut v = m[i * n + j];
                for t in 0..j { v -= m[i * n + t] * m[j * n + t]; }
                m[i * n + j] = if i == j { v.max(1e-300).sqrt() } else { v / m[j * n + j] };
            }
        }
        return Factor::Dense(m);
    }
    // Left-looking: column j gathers K's column, then the updates from every earlier column
    // with an entry at j
    let mut work = vec![0.0f64; n];
    let mut diag = vec![0.0f64; n];
    let mut vals = vec![0.0f64; sym.ri.len()];
    for j in 0..n {
        let r = sym.order[j];
        for t in k.ptr[r]..k.ptr[r + 1] {
            let p = sym.pos[k.idx[t]];
            if p >= j { work[p] += k.val[t]; }
        }
        work[j] += d[r];
        for &e in &sym.re[sym.rp[j]..sym.rp[j + 1]] {
            let ljk = vals[e];
            let end = sym.ce[e];
            for (t, &p) in sym.ri[e..end].iter().enumerate() { work[p] -= vals[e + t] * ljk; }
        }
        let ljj = work[j].max(1e-300).sqrt();
        diag[j] = ljj;
        work[j] = 0.0;
        for t in sym.cp[j]..sym.cp[j + 1] {
            let p = sym.ri[t];
            vals[t] = work[p] / ljj;
            work[p] = 0.0;
        }
    }
    Factor::Sparse { diag, vals }
}

fn solve(f: &Factor, sym: &Symbolic, b: &[f64]) -> Vec<f64> {
    let n = b.len();
    match f {
        Factor::Dense(m) => {
            let mut r = b.to_vec();
            for i in 0..n { let mut v = r[i]; for t in 0..i { v -= m[i * n + t] * r[t]; } r[i] = v / m[i * n + i]; }
            for i in (0..n).rev() { let mut v = r[i]; for t in i + 1..n { v -= m[t * n + i] * r[t]; } r[i] = v / m[i * n + i]; }
            r
        }
        Factor::Sparse { diag, vals } => {
            let mut y: Vec<f64> = sym.order.iter().map(|&r| b[r]).collect();
            for j in 0..n {
                y[j] /= diag[j];
                let yj = y[j];
                for t in sym.cp[j]..sym.cp[j + 1] { y[sym.ri[t]] -= vals[t] * yj; }
            }
            for j in (0..n).rev() {
                let mut v = y[j];
                for t in sym.cp[j]..sym.cp[j + 1] { v -= vals[t] * y[sym.ri[t]]; }
                y[j] = v / diag[j];
            }
            let mut x = vec![0.0f64; n];
            for (p, &r) in sym.order.iter().enumerate() { x[r] = y[p]; }
            x
        }
    }
}

/// The compliance added to K's diagonal, as a fraction of K_ii (constraint force mixing): K is
/// singular (four corners of a face, three freedoms), and K + diag(CFM K_ii) is positive
/// definite, so every Newton step and the solution are unique. It leaves a velocity error of
/// CFM x K_ii x the row's impulse, checked against the unregularised K by the stopping test and
/// reported as `SolveInfo::true_residual`. 1e-9 was too much at a 1000:1 mass ratio, where
/// K_ii x impulse reaches 1e4 m/s on the light box's rows (1e-5 m/s left, against the 1e-7
/// stopping tolerance); 1e-12 leaves 1e-8 there, and f64 holds the condition number.
pub(super) const CFM: f64 = 1e-12;

/// What one component's solve cost and how well it went
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct SolveInfo {
    pub solves: usize,
    pub converged: bool,
    pub k_nnz: usize,
    pub l_nnz: usize,
    /// With the check on: the first Newton system (well conditioned, D = K_ii), sparse against
    /// dense, the worst difference relative to the largest entry
    pub check: f64,
    /// The worst normwise backward error of any Newton solve, infinity norms:
    /// |(K + D) dy - rhs| / (|K + D| |dy| + |rhs|)
    pub backward: f64,
    /// The largest velocity error left on a pushing row, against the unregularised K (m/s)
    pub true_residual: f64,
}

/// |a - b| at worst, over b's largest entry
fn rel_diff(a: &[f64], b: &[f64]) -> f64 {
    let big = b.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    a.iter().zip(b).fold(0.0f64, |m, (x, y)| m.max((x - y).abs())) / big.max(1e-300)
}

/// The normwise backward error of x as a solution of (K + diag(d)) x = rhs, infinity norms
fn backward_error(k: &Csr, d: &[f64], x: &[f64], rhs: &[f64]) -> f64 {
    let n = k.n;
    let (mut res, mut norm) = (0.0f64, 0.0f64);
    for i in 0..n {
        let mut row = d[i].abs();
        for t in k.ptr[i]..k.ptr[i + 1] { row += k.val[t].abs(); }
        norm = norm.max(row);
        res = res.max((k.row_dot(i, x) + d[i] * x[i] - rhs[i]).abs());
    }
    let xn = x.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let bn = rhs.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    res / (norm * xn + bn).max(1e-300)
}

/// The velocity pass's push-only rows, a monotone LCP: find impulse changes x with each row's
/// total `acc0 + x >= 0` and velocity error w = K x - b >= 0, at most one nonzero per row (a row
/// that pushes meets its target; one that doesn't separates at least that fast). The problem is
/// degenerate on a pile, which defeated Gauss-Seidel (2,300 sweeps on a 10-box stack), active
/// sets (cycling) and projected Newton (zigzag). Primal-dual interior point with Mehrotra's
/// predictor-corrector, on K + diag(CFM K_ii):
/// - each solve factors K + CFM + diag(w / y), positive definite, sparse (`Symbolic`);
/// - the start is scaled per row: y0 = eta / K_ii, w0 = eta;
/// - converged when, against the unregularised K, each row either meets its target or carries
///   no impulse, and none approaches beyond it, to 1e-7 m/s (an f32 velocity's floor), with the
///   regularised residual under the same; a non-finite step ends the solve, not converged;
/// - at most `cap` solves.
pub(super) fn bounce_qp(k0: &Csr, b: &[f64], acc0: &[f64], cap: usize, check: bool) -> (Vec<f64>, SolveInfo) {
    let n = k0.n;
    let k = &k0.with_compliance(CFM);
    let sym = Symbolic::min_degree(k);
    let mut info = SolveInfo { k_nnz: k.nnz(), l_nnz: sym.l_nnz(), ..Default::default() };
    let kdiag: Vec<f64> = (0..n).map(|i| k0.diag(i)).collect();
    // y = acc0 + x >= 0 (the row's total impulse), w = K y + q >= 0, y_i w_i = 0
    let q: Vec<f64> = k.mul(acc0).iter().zip(b).map(|(ka, bi)| -ka - bi).collect();
    let eta = 1.0f64.max(q.iter().map(|v| v.abs()).fold(0.0, f64::max));
    let mut y: Vec<f64> = (0..n).map(|i| if kdiag[i] > 0.0 { eta / kdiag[i] } else { 1.0 }).collect();
    let mut w = vec![eta; n];
    let step = |y: &[f64], dy: &[f64], w: &[f64], dw: &[f64]| -> f64 {
        let mut a = 1.0f64;
        for i in 0..n {
            if dy[i] < 0.0 { a = a.min(-y[i] / dy[i]); }
            if dw[i] < 0.0 { a = a.min(-w[i] / dw[i]); }
        }
        a
    };
    for it in 1..=cap {
        info.solves = it;
        let ky = k.mul(&y);
        let r: Vec<f64> = (0..n).map(|i| ky[i] + q[i] - w[i]).collect();
        let mu = (0..n).map(|i| y[i] * w[i]).sum::<f64>() / n as f64;
        let rmax = r.iter().map(|v| v.abs()).fold(0.0f64, f64::max);
        // Judged against the unregularised K, in velocity (an absolute 1e-8 on the residual
        // stalled at f64 rounding at 1000:1 while y drifted, 51 solves instead of 8)
        let x: Vec<f64> = (0..n).map(|i| y[i] - acc0[i]).collect();
        let wt: Vec<f64> = k0.mul(&x).iter().zip(b).map(|(kx, bi)| kx - bi).collect();
        let tol = |i: usize| 1e-7 * (1.0 + b[i].abs());
        if rmax < 1e-7 && (0..n).all(|i| wt[i] >= -tol(i) && wt[i].abs().min(y[i] * kdiag[i]) < tol(i)) {
            info.converged = true;
            info.true_residual = (0..n).filter(|&i| y[i] * kdiag[i] > wt[i].abs()).map(|i| wt[i].abs()).fold(0.0, f64::max);
            return (x, info);
        }
        let d: Vec<f64> = (0..n).map(|i| w[i] / y[i]).collect();
        let f = factor(k, &sym, &d, false);
        let mut first = it == 1 && check;
        let mut worst_be = info.backward;
        let mut diff = info.check;
        // Newton on K dy - dw = -r, W dy + Y dw = target: dw = (target - W dy) / Y
        let mut dirs = |target: &[f64]| -> (Vec<f64>, Vec<f64>) {
            let rhs: Vec<f64> = (0..n).map(|i| -r[i] + target[i] / y[i]).collect();
            let dy = solve(&f, &sym, &rhs);
            worst_be = worst_be.max(backward_error(k, &d, &dy, &rhs));
            if first {
                // (i): the first system, D = K_ii at the start, against the dense reference
                let none = Symbolic::default();
                diff = diff.max(rel_diff(&dy, &solve(&factor(k, &none, &d, true), &none, &rhs)));
                first = false;
            }
            let dw: Vec<f64> = (0..n).map(|i| (target[i] - w[i] * dy[i]) / y[i]).collect();
            (dy, dw)
        };
        let aff: Vec<f64> = (0..n).map(|i| -y[i] * w[i]).collect();
        let (dya, dwa) = dirs(&aff);
        let aa = step(&y, &dya, &w, &dwa);
        let mu_aff = (0..n).map(|i| (y[i] + aa * dya[i]) * (w[i] + aa * dwa[i])).sum::<f64>() / n as f64;
        let ratio = mu_aff / mu.max(1e-300);
        // ratio * ratio * ratio, not powi: powi's precision is unspecified, and the hash
        // depends on these bits
        let sigma = (ratio * ratio * ratio).min(1.0);
        let cor: Vec<f64> = (0..n).map(|i| sigma * mu - y[i] * w[i] - dya[i] * dwa[i]).collect();
        let (dy, dw) = dirs(&cor);
        info.backward = worst_be;
        info.check = diff;
        let a = (0.995 * step(&y, &dy, &w, &dw)).min(1.0);
        if !(a.is_finite() && dy.iter().chain(&dw).all(|v| v.is_finite())) { break; }
        for i in 0..n {
            y[i] = (y[i] + a * dy[i]).max(1e-300);
            w[i] = (w[i] + a * dw[i]).max(1e-300);
        }
    }
    ((0..n).map(|i| y[i] - acc0[i]).collect(), info)
}

/// The pseudo pass: K x = u, factored as K + diag(CFM K_ii) in the same minimum-degree scheme.
/// Where the rows agree this is the solution. Where they can't (four corners of a face given
/// targets no rigid motion meets), the part of u along K's null space lands in x as large
/// opposing impulses, but those move no body: J^T takes them to zero, and the bodies'
/// pseudo-velocities are summed in f64, so they cancel there (unlike impulses applied one row at
/// a time in f32). Returns x and the solve's normwise backward error.
pub(super) fn pseudo_solve(k: &Csr, u: &[f64]) -> (Vec<f64>, f64) {
    let n = k.n;
    let d: Vec<f64> = (0..n).map(|i| CFM * k.diag(i)).collect();
    let sym = Symbolic::min_degree(k);
    let x = solve(&factor(k, &sym, &d, false), &sym, u);
    let be = backward_error(k, &d, &x, u);
    (x, be)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A chain of rows coupled to neighbours (a stack's shape), with a redundant pair
    fn chain(n: usize) -> Csr {
        Csr::from_rows((0..n).map(|i| {
            let mut r = Vec::new();
            if i > 0 { r.push((i - 1, -0.5)); }
            r.push((i, 2.0 + 0.1 * i as f64));
            if i + 1 < n { r.push((i + 1, -0.5)); }
            r
        }).collect())
    }

    #[test]
    fn sparse_and_dense_factors_agree() {
        let k = chain(40);
        let sym = Symbolic::min_degree(&k);
        let d: Vec<f64> = (0..40).map(|i| 1e-3 * i as f64).collect();
        let b: Vec<f64> = (0..40).map(|i| (i as f64 * 0.37).sin()).collect();
        let xs = solve(&factor(&k, &sym, &d, false), &sym, &b);
        let xd = solve(&factor(&k, &Symbolic::default(), &d, true), &Symbolic::default(), &b);
        for (s, t) in xs.iter().zip(&xd) { assert!((s - t).abs() <= 1e-12 * (1.0 + t.abs()), "{} vs {}", s, t); }
    }

    #[test]
    fn min_degree_is_a_permutation_and_fills_a_chain_not_at_all() {
        let k = chain(25);
        let sym = Symbolic::min_degree(&k);
        let mut o = sym.order.clone();
        o.sort_unstable();
        assert_eq!(o, (0..25).collect::<Vec<_>>());
        assert_eq!(sym.l_nnz(), 24);
    }
}
