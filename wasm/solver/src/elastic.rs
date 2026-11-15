//! Direct solve of a substep's elastic response.
//!
//! Gauss–Seidel sweeps over stiff membrane and hinge constraints settle on a softer sheet
//! than the model describes, and more sweeps do not fix it (docs/reference-solver.md). This
//! takes Newton steps on the same implicit-Euler problem the sweeps approximate,
//!   minimise  |x − x̃|²_M / 2dt²  +  E_membrane(x)  +  E_hinge(x),
//! with a sparse LDLᵀ factorisation whose ordering and fill are fixed by the mesh. It then
//! hands the sweeps positions and multipliers that already satisfy every elastic constraint,
//! so they only have to reconcile contact.
//!
//! The matrix is handled as 3×3 blocks, one per pair of vertices: the factor is L D Lᵀ with
//! L unit lower triangular by block and D block diagonal. The vertex ordering and the block
//! slots depend only on the mesh; they are computed once in TypeScript (src/physics/) and
//! passed in as `ElasticSetup`, and the factor's pattern is worked out from them here.
use crate::fast::{R, W};
use crate::sweeps::{HingeBatch, MembraneBatch};

/// Newton steps taken on one factorisation.
const RENEW: usize = 3;

/// The matrix's pattern, fixed by the mesh.
pub struct ElasticSetup {
    /// Greedy minimum-degree ordering of the vertex graph: `perm[vertex]` is its position.
    pub perm: Vec<i32>,
    /// Upper triangle of the permuted matrix by vertex column, a 3×3 block per entry: column
    /// pointers, each block's row (ascending, so the diagonal block is a column's last) and
    /// its offset in the assembled blocks.
    pub ap: Vec<i32>,
    pub ai: Vec<i32>,
    pub source: Vec<i32>,
    /// Where each element's vertex pairs land: block offset, negative (as `!offset`) when the
    /// pair is stored transposed. One per vertex, six per face, ten per hinge.
    pub diagonal: Vec<i32>,
    pub face_slots: Vec<i32>,
    pub hinge_slots: Vec<i32>,
    pub block_len: usize,
}

struct Assembly {
    blocks: Vec<f64>,
    gradient: Vec<f64>,
    diagonal: Vec<i32>,
    face_slots: Vec<i32>,
    hinge_slots: Vec<i32>,
}

/// Newton solver for the elastic implicit-Euler problem of one substep.
pub struct ElasticSolver {
    n: usize,
    perm: Vec<i32>,
    ap: Vec<i32>,
    ai: Vec<i32>,
    source: Vec<i32>,
    /// L by block column: column pointers, each block's row, and nine values per block
    /// (row-major; rows are the row vertex's coordinates).
    lp: Vec<u32>,
    li: Vec<u32>,
    lx: Vec<f64>,
    /// L by block row, in the order a row's blocks are eliminated: row pointers, each
    /// block's column, and where the block is stored in `lx`.
    rp: Vec<u32>,
    rows: Vec<u32>,
    slot: Vec<u32>,
    /// Each diagonal block of D as its own L D Lᵀ: l10, l20, l21, then the pivots. Dividing
    /// by the pivots, not multiplying by their reciprocals, keeps the solve's rounding below
    /// a tenth of a picometre.
    d: Vec<f64>,
    /// Scratch: the row being eliminated (nine values per vertex), and a right-hand side.
    y: Vec<f64>,
    z: Vec<f64>,
    asm: Assembly,
    work: Vec<f64>,
    step: Vec<f64>,
    trial: Vec<f64>,
    carried: Vec<f64>,
    carried_step: f64,
}

/// Solve a diagonal block's system from its factor (see `ElasticSolver::d`).
#[inline(always)]
fn block_solve(d: &[f64], b: [f64; 3]) -> [f64; 3] {
    let z1 = b[1] - d[0] * b[0];
    let z2 = b[2] - d[1] * b[0] - d[2] * z1;
    let (x0, x1, x2) = (b[0] / d[3], z1 / d[4], z2 / d[5]);
    let x1 = x1 - d[2] * x2;
    [x0 - d[0] * x1 - d[1] * x2, x1, x2]
}

/// Add the 3×3 block d²/dx_i dx_j at a precomputed slot (transposed when the slot is negative).
#[inline(always)]
fn add_block(blocks: &mut [f64], slot: i32, v: [f64; 9]) {
    let [xx, xy, xz, yx, yy, yz, zx, zy, zz] = v;
    if slot >= 0 {
        let s = slot as usize;
        let b = &mut blocks[s..s + 9];
        b[0] += xx; b[1] += xy; b[2] += xz; b[3] += yx; b[4] += yy; b[5] += yz; b[6] += zx; b[7] += zy; b[8] += zz;
    } else {
        let o = (!slot) as usize;
        let b = &mut blocks[o..o + 9];
        b[0] += xx; b[1] += yx; b[2] += zx; b[3] += xy; b[4] += yy; b[5] += zy; b[6] += xz; b[7] += yz; b[8] += zz;
    }
}

impl ElasticSolver {
    pub fn new(n: usize, s: ElasticSetup) -> Self {
        let size = 3 * n;
        // Elimination tree and the number of blocks in each column of L.
        const NONE: usize = usize::MAX;
        let (mut parent, mut flag, mut count) = (vec![NONE; n], vec![NONE; n], vec![0u32; n]);
        for k in 0..n {
            flag[k] = k;
            for p in s.ap[k] as usize..s.ap[k + 1] as usize {
                let mut i = s.ai[p] as usize;
                while i < k && flag[i] != k {
                    if parent[i] == NONE { parent[i] = k; }
                    count[i] += 1;
                    flag[i] = k;
                    i = parent[i];
                }
            }
        }
        let mut lp = vec![0u32; n + 1];
        for k in 0..n { lp[k + 1] = lp[k] + count[k]; }
        let blocks = lp[n] as usize;
        // Each row's blocks in elimination order: a column comes after every column it
        // depends on (its descendants in the elimination tree).
        let (mut li, mut rp, mut rows, mut slot) = (vec![0u32; blocks], vec![0u32; n + 1], Vec::with_capacity(blocks), Vec::with_capacity(blocks));
        let (mut filled, mut pattern) = (vec![0u32; n], vec![0usize; n]);
        flag.fill(NONE);
        for k in 0..n {
            flag[k] = k;
            let mut top = n;
            for p in s.ap[k] as usize..s.ap[k + 1] as usize {
                let mut i = s.ai[p] as usize;
                let mut len = 0;
                while i < k && flag[i] != k {
                    pattern[len] = i;
                    len += 1;
                    flag[i] = k;
                    i = parent[i];
                }
                while len > 0 {
                    top -= 1;
                    len -= 1;
                    pattern[top] = pattern[len];
                }
            }
            for &i in &pattern[top..n] {
                let at = lp[i] + filled[i];
                filled[i] += 1;
                li[at as usize] = k as u32;
                rows.push(i as u32);
                slot.push(at);
            }
            rp[k + 1] = rows.len() as u32;
        }
        ElasticSolver {
            n,
            perm: s.perm,
            ap: s.ap,
            ai: s.ai,
            source: s.source,
            lp,
            li,
            lx: vec![0.0; 9 * blocks],
            rp,
            rows,
            slot,
            d: vec![0.0; 6 * n],
            y: vec![0.0; 9 * n],
            z: vec![0.0; size],
            asm: Assembly { blocks: vec![0.0; s.block_len], gradient: vec![0.0; size], diagonal: s.diagonal, face_slots: s.face_slots, hinge_slots: s.hinge_slots },
            work: vec![0.0; size],
            step: vec![0.0; size],
            trial: vec![0.0; size],
            carried: vec![0.0; size],
            carried_step: 0.0,
        }
    }
    /// Forget the carried correction (after a reset or any discontinuity in the state).
    pub fn reset(&mut self) { self.carried_step = 0.0; }

    /// Replace `x` (the unconstrained prediction) with the elastic implicit-Euler solution
    /// and set the sweeps' multipliers to match. `alpha` values are compliance/dt². Newton
    /// steps with a backtracking line search until the largest update is below `tolerance`
    /// metres. Returns false, leaving the prediction untouched, if `iterations` steps do not
    /// get there.
    #[allow(clippy::too_many_arguments)]
    pub fn solve(&mut self, x: &mut [f64], masses: &[f64], dt: f64, membranes: &mut MembraneBatch, hinges: &mut HingeBatch, iterations: usize, tolerance: f64) -> bool {
        let size = 3 * self.n;
        self.work[..size].copy_from_slice(&x[..size]);
        // Start from the previous substep's elastic correction: loads change slowly, so the
        // sheet's response to them does too. It scales with dt².
        let ratio = if self.carried_step > 0.0 { (dt * dt) / (self.carried_step * self.carried_step) } else { 0.0 };
        if ratio > 0.0 {
            // Keep the carried start only if it is better than the plain prediction; after an
            // impact it need not be.
            for k in 0..size { x[k] += ratio * self.carried[k]; }
            let e1 = assemble(&mut self.asm, &self.work, x, masses, dt, membranes, hinges, false, false);
            let e0 = assemble(&mut self.asm, &self.work, &self.work, masses, dt, membranes, hinges, false, false);
            if !(e1 <= e0) { x[..size].copy_from_slice(&self.work[..size]); }
        }
        let mut converged = false;
        for iteration in 0..iterations {
            // One factorisation serves a solve's first Newton steps (the shape barely changes
            // within a substep) and is renewed if they have not converged. It is not kept between
            // substeps: the membrane's stiff directions turn with the sheet.
            let renew = iteration % RENEW == 0;
            let energy = assemble(&mut self.asm, &self.work, x, masses, dt, membranes, hinges, true, renew);
            if renew && !self.factor() { break; }
            self.back_solve();
            let mut slope = 0.0;
            let mut largest: f64 = 0.0;
            {
                let g = &self.asm.gradient;
                let step = &self.step;
                for k in 0..size {
                    slope -= g[k] * step[k];
                    let m = step[k].abs();
                    if m > largest { largest = m; }
                }
            }
            // Backtrack until the incremental potential decreases enough; a step that cannot
            // decrease it (or a non-finite one) is not taken.
            let mut length = 1.0;
            let mut accepted = largest < tolerance;
            if accepted { converged = true; break; }
            while !accepted && length >= 1e-3 {
                for k in 0..size { self.trial[k] = x[k] - length * self.step[k]; }
                let e = assemble(&mut self.asm, &self.work, &self.trial, masses, dt, membranes, hinges, false, false);
                if e <= energy + 1e-4 * length * slope { accepted = true; break; }
                length /= 2.0;
            }
            if !accepted || !(largest < 1e3) { break; }
            x[..size].copy_from_slice(&self.trial[..size]);
            if largest * length < tolerance { converged = true; break; }
        }
        if !converged {
            // A partly converged shape carries unbalanced stiff forces. Hand the sweeps the plain
            // prediction instead, as if this solve had not run.
            x[..size].copy_from_slice(&self.work[..size]);
            membranes.lambda.fill(0.0);
            hinges.lambda.fill(0.0);
            self.carried_step = 0.0;
            return false;
        }
        for k in 0..size { self.carried[k] = x[k] - self.work[k]; }
        self.carried_step = dt;
        multipliers(x, membranes, hinges);
        true
    }

    /// Numeric L D Lᵀ factorisation of the assembled blocks, a row of L at a time. Returns
    /// false on a pivot that is not positive.
    fn factor(&mut self) -> bool {
        let (ap, ai, source, lp, li, rp, rows, slot) = (R(&self.ap), R(&self.ai), R(&self.source), R(&self.lp), R(&self.li), R(&self.rp), R(&self.rows), R(&self.slot));
        let blocks = R(&self.asm.blocks);
        let (mut lx, mut d, mut y) = (W(&mut self.lx), W(&mut self.d), W(&mut self.y));
        for k in 0..self.n {
            // Column k of the matrix above the diagonal is row k of its lower triangle,
            // transposed: y holds it as blocks with a row per coordinate of the other vertex.
            let (first, last) = (ap[k] as usize, ap[k + 1] as usize - 1);
            for p in first..last {
                let (to, from) = (9 * ai[p] as usize, source[p] as usize);
                for e in 0..9 { y[to + e] = blocks[from + e]; }
            }
            let from = source[last] as usize;
            let (mut d00, mut d01, mut d02, mut d11, mut d12, mut d22) = (blocks[from], blocks[from + 1], blocks[from + 2], blocks[from + 4], blocks[from + 5], blocks[from + 8]);
            for q in rp[k] as usize..rp[k + 1] as usize {
                let i = rows[q] as usize;
                let end = slot[q] as usize;
                let o = 9 * i;
                let v = [y[o], y[o + 1], y[o + 2], y[o + 3], y[o + 4], y[o + 5], y[o + 6], y[o + 7], y[o + 8]];
                for e in 0..9 { y[o + e] = 0.0; }
                // Take this block's effect off the later blocks of the row.
                for p in lp[i] as usize..end {
                    let (l, t) = (9 * p, 9 * li[p] as usize);
                    for r in 0..3 {
                        let (l0, l1, l2) = (lx[l + 3 * r], lx[l + 3 * r + 1], lx[l + 3 * r + 2]);
                        y[t + 3 * r] -= l0 * v[0] + l1 * v[3] + l2 * v[6];
                        y[t + 3 * r + 1] -= l0 * v[1] + l1 * v[4] + l2 * v[7];
                        y[t + 3 * r + 2] -= l0 * v[2] + l1 * v[5] + l2 * v[8];
                    }
                }
                // L(k,i) is (D_i⁻¹ v)ᵀ, and it takes L(k,i) v off the diagonal block.
                let di = &d.0[6 * i..6 * i + 6];
                let w0 = block_solve(di, [v[0], v[3], v[6]]);
                let w1 = block_solve(di, [v[1], v[4], v[7]]);
                let w2 = block_solve(di, [v[2], v[5], v[8]]);
                d00 -= w0[0] * v[0] + w0[1] * v[3] + w0[2] * v[6];
                d01 -= w0[0] * v[1] + w0[1] * v[4] + w0[2] * v[7];
                d02 -= w0[0] * v[2] + w0[1] * v[5] + w0[2] * v[8];
                d11 -= w1[0] * v[1] + w1[1] * v[4] + w1[2] * v[7];
                d12 -= w1[0] * v[2] + w1[1] * v[5] + w1[2] * v[8];
                d22 -= w2[0] * v[2] + w2[1] * v[5] + w2[2] * v[8];
                let l = 9 * end;
                lx[l] = w0[0]; lx[l + 1] = w0[1]; lx[l + 2] = w0[2];
                lx[l + 3] = w1[0]; lx[l + 4] = w1[1]; lx[l + 5] = w1[2];
                lx[l + 6] = w2[0]; lx[l + 7] = w2[1]; lx[l + 8] = w2[2];
            }
            // Mass and Gauss–Newton stiffness are positive definite; guard against rounding only.
            if !(d00 > 0.0) { return false; }
            let (l10, l20) = (d01 / d00, d02 / d00);
            let p1 = d11 - l10 * d01;
            if !(p1 > 0.0) { return false; }
            let rest = d12 - l20 * d01;
            let l21 = rest / p1;
            let p2 = d22 - l20 * d02 - l21 * rest;
            if !(p2 > 0.0) { return false; }
            let o = 6 * k;
            d[o] = l10; d[o + 1] = l20; d[o + 2] = l21; d[o + 3] = d00; d[o + 4] = p1; d[o + 5] = p2;
        }
        true
    }

    /// step = H⁻¹ gradient, in the caller's vertex order.
    fn back_solve(&mut self) {
        let n = self.n;
        let (lp, li, lx, d, perm) = (R(&self.lp), R(&self.li), R(&self.lx), R(&self.d), R(&self.perm));
        let mut z = W(&mut self.z);
        let g = R(&self.asm.gradient);
        for i in 0..n {
            let (p, k) = (3 * perm[i] as usize, 3 * i);
            z[p] = g[k]; z[p + 1] = g[k + 1]; z[p + 2] = g[k + 2];
        }
        for j in 0..n {
            let (a, b, c) = (z[3 * j], z[3 * j + 1], z[3 * j + 2]);
            for p in lp[j] as usize..lp[j + 1] as usize {
                let (l, t) = (9 * p, 3 * li[p] as usize);
                z[t] -= lx[l] * a + lx[l + 1] * b + lx[l + 2] * c;
                z[t + 1] -= lx[l + 3] * a + lx[l + 4] * b + lx[l + 5] * c;
                z[t + 2] -= lx[l + 6] * a + lx[l + 7] * b + lx[l + 8] * c;
            }
        }
        for j in (0..n).rev() {
            let [mut a, mut b, mut c] = block_solve(&d.0[6 * j..6 * j + 6], [z[3 * j], z[3 * j + 1], z[3 * j + 2]]);
            for p in lp[j] as usize..lp[j + 1] as usize {
                let (l, t) = (9 * p, 3 * li[p] as usize);
                let (u, v, w) = (z[t], z[t + 1], z[t + 2]);
                a -= lx[l] * u + lx[l + 3] * v + lx[l + 6] * w;
                b -= lx[l + 1] * u + lx[l + 4] * v + lx[l + 7] * w;
                c -= lx[l + 2] * u + lx[l + 5] * v + lx[l + 8] * w;
            }
            z[3 * j] = a; z[3 * j + 1] = b; z[3 * j + 2] = c;
        }
        let mut out = W(&mut self.step);
        for i in 0..n {
            let (p, k) = (3 * perm[i] as usize, 3 * i);
            out[k] = z[p]; out[k + 1] = z[p + 1]; out[k + 2] = z[p + 2];
        }
    }
}

/// Incremental potential at `x`; with `derivatives`, also its gradient, and with `hessian`
/// its (Gauss–Newton for hinges, positive part for membranes) Hessian blocks.
#[allow(clippy::too_many_arguments)]
fn assemble(asm: &mut Assembly, predicted: &[f64], x: &[f64], masses: &[f64], dt: f64, membranes: &MembraneBatch, hinges: &HingeBatch, derivatives: bool, hessian: bool) -> f64 {
    let n = masses.len();
    let inverse = 1.0 / (dt * dt);
    let faces = &membranes.faces;
    let (md, cd) = (&membranes.md, &membranes.cd);
    let ids = &hinges.ids;
    let hinge_count = hinges.lambda.len();
    let face_count = faces.len() / 3;
    let blocks = &mut asm.blocks;
    let mut g = W(&mut asm.gradient);
    let (x, predicted, masses) = (R(x), R(predicted), R(masses));
    let (faces, md, cd, ids, malpha, halpha) = (R(faces), R(md), R(cd), R(ids), R(&membranes.alpha), R(&hinges.alpha));
    let (face_slots, hinge_slots, diag) = (R(&asm.face_slots), R(&asm.hinge_slots), R(&asm.diagonal));
    let (target_sin, target_cos) = (R(&hinges.target_sin), R(&hinges.target_cos));
    let mut energy = 0.0;
    if hessian { blocks.fill(0.0); }
    for i in 0..n {
        let m = masses[i] * inverse;
        let k = 3 * i;
        let (dx, dy, dz) = (x[k] - predicted[k], x[k + 1] - predicted[k + 1], x[k + 2] - predicted[k + 2]);
        energy += m * (dx * dx + dy * dy + dz * dz) / 2.0;
        if derivatives {
            g[k] = m * dx; g[k + 1] = m * dy; g[k + 2] = m * dz;
            if hessian {
                let o = diag[i] as usize;
                blocks[o] += m; blocks[o + 4] += m; blocks[o + 8] += m;
            }
        }
    }
    for f in 0..face_count {
        let t = 3 * f;
        let (a, b, c) = (3 * faces[t] as usize, 3 * faces[t + 1] as usize, 3 * faces[t + 2] as usize);
        let (m0, m1, m2, c0, c1, c2) = (md[t], md[t + 1], md[t + 2], cd[t], cd[t + 1], cd[t + 2]);
        let ux = m0 * x[a] + m1 * x[b] + m2 * x[c];
        let uy = m0 * x[a + 1] + m1 * x[b + 1] + m2 * x[c + 1];
        let uz = m0 * x[a + 2] + m1 * x[b + 2] + m2 * x[c + 2];
        let vx = c0 * x[a] + c1 * x[b] + c2 * x[c];
        let vy = c0 * x[a + 1] + c1 * x[b + 1] + c2 * x[c + 1];
        let vz = c0 * x[a + 2] + c1 * x[b + 2] + c2 * x[c + 2];
        let s0 = (ux * ux + uy * uy + uz * uz - 1.0) / 2.0;
        let s1 = (vx * vx + vy * vy + vz * vz - 1.0) / 2.0;
        let s2 = ux * vx + uy * vy + uz * vz;
        // Stiffness is 1/(alpha dt²).
        let k0 = inverse / malpha[t];
        let k1 = inverse / malpha[t + 1];
        let k2 = inverse / malpha[t + 2];
        energy += (k0 * s0 * s0 + k1 * s1 * s1 + k2 * s2 * s2) / 2.0;
        if !derivatives { continue; }
        let (f0, f1, f2) = (k0 * s0, k1 * s1, k2 * s2);
        let tension_u = if f0 > 0.0 { f0 } else { 0.0 };
        let tension_v = if f1 > 0.0 { f1 } else { 0.0 };
        let mem_m = [m0, m1, m2];
        let mem_c = [c0, c1, c2];
        let mem_v = [a, b, c];
        let mut pair = 6 * f;
        for i in 0..3 {
            let (mi, ci, o) = (mem_m[i], mem_c[i], mem_v[i]);
            let (six, siy, siz) = (mi * vx + ci * ux, mi * vy + ci * uy, mi * vz + ci * uz);
            g[o] += f0 * mi * ux + f1 * ci * vx + f2 * six;
            g[o + 1] += f0 * mi * uy + f1 * ci * vy + f2 * siy;
            g[o + 2] += f0 * mi * uz + f1 * ci * vz + f2 * siz;
            if hessian {
                for j in i..3 {
                    let (mj, cj) = (mem_m[j], mem_c[j]);
                    let (sjx, sjy, sjz) = (mj * vx + cj * ux, mj * vy + cj * uy, mj * vz + cj * uz);
                    // A stretched direction's geometric stiffness is positive; a compressed one is
                    // dropped.
                    let uu = k0 * mi * mj;
                    let vv = k1 * ci * cj;
                    let diagonal = tension_u * mi * mj + tension_v * ci * cj;
                    add_block(blocks, face_slots[pair], [
                        uu * ux * ux + vv * vx * vx + k2 * six * sjx + diagonal, uu * ux * uy + vv * vx * vy + k2 * six * sjy, uu * ux * uz + vv * vx * vz + k2 * six * sjz,
                        uu * uy * ux + vv * vy * vx + k2 * siy * sjx, uu * uy * uy + vv * vy * vy + k2 * siy * sjy + diagonal, uu * uy * uz + vv * vy * vz + k2 * siy * sjz,
                        uu * uz * ux + vv * vz * vx + k2 * siz * sjx, uu * uz * uy + vv * vz * vy + k2 * siz * sjy, uu * uz * uz + vv * vz * vz + k2 * siz * sjz + diagonal,
                    ]);
                    pair += 1;
                }
            }
        }
    }
    for j in 0..hinge_count {
        let (a, b, c, d) = (3 * ids[4 * j] as usize, 3 * ids[4 * j + 1] as usize, 3 * ids[4 * j + 2] as usize, 3 * ids[4 * j + 3] as usize);
        let (ax, ay, az, bx, by, bz) = (x[a], x[a + 1], x[a + 2], x[b], x[b + 1], x[b + 2]);
        let (cx, cy, cz, dx, dy, dz) = (x[c], x[c + 1], x[c + 2], x[d], x[d + 1], x[d + 2]);
        let (ex, ey, ez) = (dx - cx, dy - cy, dz - cz);
        let el = (ex * ex + ey * ey + ez * ez).sqrt();
        let (ux, uy, uz, vx, vy, vz) = (cx - ax, cy - ay, cz - az, dx - ax, dy - ay, dz - az);
        let (rx, ry, rz, sx, sy, sz) = (dx - bx, dy - by, dz - bz, cx - bx, cy - by, cz - bz);
        // Signed dihedral angle and its gradients, as in the hinge sweep (sweeps.rs); the
        // gradients are adapted from PositionBasedDynamics (MIT), see THIRD_PARTY.md.
        let (mut n1x, mut n1y, mut n1z) = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
        let (mut n2x, mut n2y, mut n2z) = (ry * sz - rz * sy, rz * sx - rx * sz, rx * sy - ry * sx);
        let sq1 = n1x * n1x + n1y * n1y + n1z * n1z;
        let sq2 = n2x * n2x + n2y * n2y + n2z * n2z;
        if el < 1e-10 || sq1 < 1e-18 || sq2 < 1e-18 { continue; }
        let sin = ((n1y * n2z - n1z * n2y) * ex + (n1z * n2x - n1x * n2z) * ey + (n1x * n2y - n1y * n2x) * ez) / el;
        let cos = n1x * n2x + n1y * n2y + n1z * n2z;
        let (st, ct) = (target_sin[j], target_cos[j]);
        let cval = (sin * ct - cos * st).atan2(cos * ct + sin * st);
        let k = inverse / halpha[j];
        energy += k * cval * cval / 2.0;
        if !derivatives { continue; }
        let force = k * cval;
        n1x /= sq1; n1y /= sq1; n1z /= sq1; n2x /= sq2; n2y /= sq2; n2z /= sq2;
        let k2 = (vx * ex + vy * ey + vz * ez) / el;
        let l2 = (rx * ex + ry * ey + rz * ez) / el;
        let k3 = -(ux * ex + uy * ey + uz * ez) / el;
        let l3 = -(sx * ex + sy * ey + sz * ez) / el;
        let gr = [
            -el * n1x, -el * n1y, -el * n1z,
            -el * n2x, -el * n2y, -el * n2z,
            k2 * n1x + l2 * n2x, k2 * n1y + l2 * n2y, k2 * n1z + l2 * n2z,
            k3 * n1x + l3 * n2x, k3 * n1y + l3 * n2y, k3 * n1z + l3 * n2z,
        ];
        let hid = [a, b, c, d];
        let mut pair = 10 * j;
        for p in 0..4 {
            let o = hid[p];
            let (px, py, pz) = (gr[3 * p], gr[3 * p + 1], gr[3 * p + 2]);
            g[o] += force * px; g[o + 1] += force * py; g[o + 2] += force * pz;
            if hessian {
                for q in p..4 {
                    let (qx, qy, qz) = (k * gr[3 * q], k * gr[3 * q + 1], k * gr[3 * q + 2]);
                    add_block(blocks, hinge_slots[pair], [px * qx, px * qy, px * qz, py * qx, py * qy, py * qz, pz * qx, pz * qy, pz * qz]);
                    pair += 1;
                }
            }
        }
    }
    energy
}

/// The sweeps' multiplier for a constraint with value C is −C/alpha at elastic equilibrium.
fn multipliers(x: &[f64], membranes: &mut MembraneBatch, hinges: &mut HingeBatch) {
    let faces = &membranes.faces;
    let (md, cd) = (&membranes.md, &membranes.cd);
    let mut t = 0;
    while t < faces.len() {
        let (a, b, c) = (3 * faces[t] as usize, 3 * faces[t + 1] as usize, 3 * faces[t + 2] as usize);
        let (m0, m1, m2, c0, c1, c2) = (md[t], md[t + 1], md[t + 2], cd[t], cd[t + 1], cd[t + 2]);
        let ux = m0 * x[a] + m1 * x[b] + m2 * x[c];
        let uy = m0 * x[a + 1] + m1 * x[b + 1] + m2 * x[c + 1];
        let uz = m0 * x[a + 2] + m1 * x[b + 2] + m2 * x[c + 2];
        let vx = c0 * x[a] + c1 * x[b] + c2 * x[c];
        let vy = c0 * x[a + 1] + c1 * x[b + 1] + c2 * x[c + 1];
        let vz = c0 * x[a + 2] + c1 * x[b + 2] + c2 * x[c + 2];
        membranes.lambda[t] = -(ux * ux + uy * uy + uz * uz - 1.0) / 2.0 / membranes.alpha[t];
        membranes.lambda[t + 1] = -(vx * vx + vy * vy + vz * vz - 1.0) / 2.0 / membranes.alpha[t + 1];
        membranes.lambda[t + 2] = -(ux * vx + uy * vy + uz * vz) / membranes.alpha[t + 2];
        t += 3;
    }
    let ids = &hinges.ids;
    for j in 0..hinges.lambda.len() {
        let (a, b, c, d) = (3 * ids[4 * j] as usize, 3 * ids[4 * j + 1] as usize, 3 * ids[4 * j + 2] as usize, 3 * ids[4 * j + 3] as usize);
        let (ex, ey, ez) = (x[d] - x[c], x[d + 1] - x[c + 1], x[d + 2] - x[c + 2]);
        let el = (ex * ex + ey * ey + ez * ez).sqrt();
        let (ux, uy, uz, vx, vy, vz) = (x[c] - x[a], x[c + 1] - x[a + 1], x[c + 2] - x[a + 2], x[d] - x[a], x[d + 1] - x[a + 1], x[d + 2] - x[a + 2]);
        let (rx, ry, rz, sx, sy, sz) = (x[d] - x[b], x[d + 1] - x[b + 1], x[d + 2] - x[b + 2], x[c] - x[b], x[c + 1] - x[b + 1], x[c + 2] - x[b + 2]);
        let (n1x, n1y, n1z) = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
        let (n2x, n2y, n2z) = (ry * sz - rz * sy, rz * sx - rx * sz, rx * sy - ry * sx);
        if el < 1e-10 || n1x * n1x + n1y * n1y + n1z * n1z < 1e-18 || n2x * n2x + n2y * n2y + n2z * n2z < 1e-18 { hinges.lambda[j] = 0.0; continue; }
        let sin = ((n1y * n2z - n1z * n2y) * ex + (n1z * n2x - n1x * n2z) * ey + (n1x * n2y - n1y * n2x) * ez) / el;
        let cos = n1x * n2x + n1y * n2y + n1z * n2z;
        let (st, ct) = (hinges.target_sin[j], hinges.target_cos[j]);
        hinges.lambda[j] = -(sin * ct - cos * st).atan2(cos * ct + sin * st) / hinges.alpha[j];
    }
}
