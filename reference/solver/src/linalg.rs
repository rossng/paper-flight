//! Symmetric positive-definite block-sparse solver: minimum-degree ordering on the
//! vertex graph, then an up-looking sparse LDL^T factorisation (after T. Davis's LDL).
use crate::vec::M3;
use std::collections::HashMap;

pub struct Sparse {
    n: usize,
    perm: Vec<usize>,
    slot: HashMap<(u32, u32), u32>,
    cols: Vec<Vec<usize>>,
    blocks: Vec<[f64; 9]>,
    ap: Vec<usize>, ai: Vec<usize>, ax: Vec<f64>,
    lp: Vec<usize>, parent: Vec<isize>, li: Vec<usize>, lx: Vec<f64>, d: Vec<f64>,
    lnz: Vec<usize>, y: Vec<f64>, pattern: Vec<usize>, flag: Vec<usize>,
}

/// Greedy minimum degree on a dense adjacency matrix. Fine at a few hundred vertices.
fn minimum_degree(n: usize, pairs: &[(usize, usize)]) -> Vec<usize> {
    let mut adj = vec![false; n * n];
    for &(i, j) in pairs { if i != j { adj[i * n + j] = true; adj[j * n + i] = true; } }
    let mut degree: Vec<usize> = (0..n).map(|i| (0..n).filter(|&j| adj[i * n + j]).count()).collect();
    let mut done = vec![false; n];
    let mut perm = vec![0; n];
    let mut neighbours = Vec::new();
    for position in 0..n {
        let v = (0..n).filter(|&i| !done[i]).min_by_key(|&i| degree[i]).unwrap();
        done[v] = true; perm[v] = position;
        neighbours.clear();
        neighbours.extend((0..n).filter(|&j| !done[j] && adj[v * n + j]));
        for &a in &neighbours {
            degree[a] -= 1;
            for &b in &neighbours {
                if a < b && !adj[a * n + b] { adj[a * n + b] = true; adj[b * n + a] = true; degree[a] += 1; degree[b] += 1; }
            }
        }
    }
    perm
}

impl Sparse {
    /// `pairs` lists coupled vertices; the diagonal is always present.
    pub fn new(n: usize, pairs: &[(usize, usize)]) -> Self {
        let perm = minimum_degree(n, pairs);
        let mut cols: Vec<Vec<usize>> = (0..n).map(|j| vec![j]).collect();
        for &(i, j) in pairs {
            let (a, b) = (perm[i].min(perm[j]), perm[i].max(perm[j]));
            if a != b { cols[b].push(a); }
        }
        let mut slot = HashMap::new();
        let mut count = 0u32;
        for (j, col) in cols.iter_mut().enumerate() {
            col.sort_unstable(); col.dedup();
            for &i in col.iter() { slot.insert((i as u32, j as u32), count); count += 1; }
        }
        let size = 3 * n;
        let (mut ap, mut ai) = (vec![0usize; size + 1], Vec::new());
        for j in 0..n { for b in 0..3 {
            for &i in &cols[j] {
                if i < j { ai.extend([3 * i, 3 * i + 1, 3 * i + 2]); } else { for a in 0..=b { ai.push(3 * j + a); } }
            }
            ap[3 * j + b + 1] = ai.len();
        } }
        // Symbolic factorisation: elimination tree and column counts.
        let (mut parent, mut lnz, mut flag) = (vec![-1isize; size], vec![0usize; size], vec![0usize; size]);
        for k in 0..size {
            flag[k] = k;
            for p in ap[k]..ap[k + 1] {
                let mut i = ai[p];
                if i < k { while flag[i] != k {
                    if parent[i] == -1 { parent[i] = k as isize; }
                    lnz[i] += 1; flag[i] = k; i = parent[i] as usize;
                } }
            }
        }
        let mut lp = vec![0usize; size + 1];
        for k in 0..size { lp[k + 1] = lp[k] + lnz[k]; }
        let total = lp[size];
        Sparse {
            n, perm, slot, cols, blocks: vec![[0.0; 9]; count as usize], ax: vec![0.0; ai.len()], ap, ai,
            lp, parent, li: vec![0; total], lx: vec![0.0; total], d: vec![0.0; size], lnz, y: vec![0.0; size],
            pattern: vec![0; size], flag,
        }
    }
    pub fn clear(&mut self) { for b in self.blocks.iter_mut() { *b = [0.0; 9]; } }
    /// Add the 3x3 block d2/dx_i dx_j. Call once per unordered vertex pair; for i == j pass a symmetric block.
    #[inline]
    pub fn add(&mut self, i: usize, j: usize, b: &M3) {
        let (pi, pj) = (self.perm[i], self.perm[j]);
        if pi <= pj {
            let s = self.slot[&(pi as u32, pj as u32)] as usize;
            let t = &mut self.blocks[s];
            for a in 0..3 { for c in 0..3 { t[3 * a + c] += b[a][c]; } }
        } else {
            let s = self.slot[&(pj as u32, pi as u32)] as usize;
            let t = &mut self.blocks[s];
            for a in 0..3 { for c in 0..3 { t[3 * a + c] += b[c][a]; } }
        }
    }
    #[inline]
    pub fn add_diagonal(&mut self, i: usize, s: f64) {
        let p = self.perm[i];
        let t = &mut self.blocks[self.slot[&(p as u32, p as u32)] as usize];
        t[0] += s; t[4] += s; t[8] += s;
    }
    /// Numeric LDL^T. False when a pivot is not positive (matrix not positive definite).
    pub fn factor(&mut self) -> bool {
        let mut q = 0;
        let mut s = 0;
        for j in 0..self.n {
            let first = s;
            for b in 0..3 {
                s = first;
                for &i in &self.cols[j] {
                    let block = &self.blocks[s]; s += 1;
                    if i < j { for a in 0..3 { self.ax[q] = block[3 * a + b]; q += 1; } }
                    else { for a in 0..=b { self.ax[q] = block[3 * a + b]; q += 1; } }
                }
            }
        }
        let size = 3 * self.n;
        for k in 0..size {
            self.y[k] = 0.0; let mut top = size; self.flag[k] = k; self.lnz[k] = 0;
            for p in self.ap[k]..self.ap[k + 1] {
                let mut i = self.ai[p];
                self.y[i] += self.ax[p];
                let mut len = 0;
                while self.flag[i] != k { self.pattern[len] = i; len += 1; self.flag[i] = k; i = self.parent[i] as usize; }
                while len > 0 { top -= 1; len -= 1; self.pattern[top] = self.pattern[len]; }
            }
            self.d[k] = self.y[k]; self.y[k] = 0.0;
            for t in top..size {
                let i = self.pattern[t]; let yi = self.y[i]; self.y[i] = 0.0;
                let end = self.lp[i] + self.lnz[i];
                for p in self.lp[i]..end { self.y[self.li[p]] -= self.lx[p] * yi; }
                let l = yi / self.d[i];
                self.d[k] -= l * yi;
                self.li[end] = k; self.lx[end] = l; self.lnz[i] += 1;
            }
            if !(self.d[k] > 0.0) { return false; }
        }
        true
    }
    /// Solve A out = rhs in the caller's vertex order.
    pub fn solve(&self, rhs: &[f64], out: &mut [f64]) {
        let size = 3 * self.n;
        let mut x = vec![0.0; size];
        for i in 0..self.n { for a in 0..3 { x[3 * self.perm[i] + a] = rhs[3 * i + a]; } }
        for j in 0..size { let xj = x[j]; for p in self.lp[j]..self.lp[j + 1] { x[self.li[p]] -= self.lx[p] * xj; } }
        for j in 0..size { x[j] /= self.d[j]; }
        for j in (0..size).rev() { let mut s = x[j]; for p in self.lp[j]..self.lp[j + 1] { s -= self.lx[p] * x[self.li[p]]; } x[j] = s; }
        for i in 0..self.n { for a in 0..3 { out[3 * i + a] = x[3 * self.perm[i] + a]; } }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn solves_a_random_block_system() {
        let n = 40;
        let mut seed = 12345u64;
        let mut random = move || { seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407); ((seed >> 11) as f64) / ((1u64 << 53) as f64) - 0.5 };
        let mut pairs = Vec::new();
        for i in 0..n { for j in i + 1..n { if (i * 7 + j * 13) % 9 == 0 || j == i + 1 { pairs.push((i, j)); } } }
        let mut dense = vec![0.0; 9 * n * n];
        let mut sparse = Sparse::new(n, &pairs);
        for &(i, j) in &pairs {
            let mut b = [[0.0; 3]; 3];
            for a in 0..3 { for c in 0..3 { b[a][c] = random(); } }
            sparse.add(i, j, &b);
            for a in 0..3 { for c in 0..3 { dense[(3 * i + a) * 3 * n + 3 * j + c] += b[a][c]; dense[(3 * j + c) * 3 * n + 3 * i + a] += b[a][c]; } }
        }
        for i in 0..n { sparse.add_diagonal(i, 40.0); for a in 0..3 { dense[(3 * i + a) * 3 * n + 3 * i + a] += 40.0; } }
        assert!(sparse.factor());
        let rhs: Vec<f64> = (0..3 * n).map(|_| random()).collect();
        let mut x = vec![0.0; 3 * n];
        sparse.solve(&rhs, &mut x);
        for r in 0..3 * n {
            let ax: f64 = (0..3 * n).map(|c| dense[r * 3 * n + c] * x[c]).sum();
            assert!((ax - rhs[r]).abs() < 1e-10, "row {r}: {ax} vs {}", rhs[r]);
        }
    }
    #[test]
    fn rejects_an_indefinite_matrix() {
        let mut sparse = Sparse::new(2, &[(0, 1)]);
        sparse.add_diagonal(0, 1.0); sparse.add_diagonal(1, -1.0);
        assert!(!sparse.factor());
    }
}
