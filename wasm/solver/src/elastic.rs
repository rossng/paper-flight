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
//! The symbolic set-up (ordering, pattern, elimination tree, block slots) depends only on
//! the mesh. It is computed once in TypeScript (src/physics/) and passed in as
//! `ElasticSetup`.
use crate::fast::{R, W};
use crate::sweeps::{HingeBatch, MembraneBatch};

/// Newton steps taken on one factorisation.
const RENEW: usize = 3;

/// The symbolic factorisation, fixed by the mesh.
pub struct ElasticSetup {
    /// Greedy minimum-degree ordering of the vertex graph: `perm[vertex]` is its position.
    pub perm: Vec<i32>,
    /// Upper triangle of the permuted matrix, by scalar column: column pointers, row indices,
    /// and for each entry the index into the 3×3 blocks it is copied from.
    pub ap: Vec<i32>,
    pub ai: Vec<i32>,
    pub source: Vec<i32>,
    /// Elimination tree.
    pub parent: Vec<i32>,
    /// Column pointers of the factor, from the column counts.
    pub lp: Vec<i32>,
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
    source: Vec<i32>,
    ap: Vec<i32>,
    ai: Vec<i32>,
    ax: Vec<f64>,
    lp: Vec<i32>,
    parent: Vec<i32>,
    lnz: Vec<i32>,
    li: Vec<i32>,
    lx: Vec<f64>,
    d: Vec<f64>,
    y: Vec<f64>,
    pattern: Vec<i32>,
    flag: Vec<i32>,
    asm: Assembly,
    work: Vec<f64>,
    step: Vec<f64>,
    trial: Vec<f64>,
    carried: Vec<f64>,
    carried_step: f64,
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
        let nnz = s.lp[size] as usize;
        let ax_len = s.ai.len();
        ElasticSolver {
            n,
            perm: s.perm,
            source: s.source,
            ap: s.ap,
            ai: s.ai,
            ax: vec![0.0; ax_len],
            lp: s.lp,
            parent: s.parent,
            lnz: vec![0; size],
            li: vec![0; nnz],
            lx: vec![0.0; nnz],
            d: vec![0.0; size],
            y: vec![0.0; size],
            pattern: vec![0; size],
            flag: vec![0; size],
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

    /// Numeric LDLᵀ factorisation of the assembled blocks. Returns false on a pivot that is
    /// not positive.
    fn factor(&mut self) -> bool {
        let size = 3 * self.n;
        let (ap, ai, lp, parent) = (R(&self.ap), R(&self.ai), R(&self.lp), R(&self.parent));
        let (mut ax, mut lnz, mut li, mut lx, mut d, mut y, mut pattern, mut flag) = (W(&mut self.ax), W(&mut self.lnz), W(&mut self.li), W(&mut self.lx), W(&mut self.d), W(&mut self.y), W(&mut self.pattern), W(&mut self.flag));
        let (blocks, source) = (R(&self.asm.blocks), R(&self.source));
        for q in 0..ax.0.len() { ax[q] = blocks[source[q] as usize]; }
        for k in 0..size {
            y[k] = 0.0;
            let mut top = size;
            flag[k] = k as i32;
            lnz[k] = 0;
            for p in ap[k] as usize..ap[k + 1] as usize {
                let mut i = ai[p] as usize;
                y[i] += ax[p];
                let mut len = 0usize;
                while flag[i] != k as i32 {
                    pattern[len] = i as i32;
                    len += 1;
                    flag[i] = k as i32;
                    i = parent[i] as usize;
                }
                while len > 0 {
                    top -= 1;
                    len -= 1;
                    pattern[top] = pattern[len];
                }
            }
            let mut pivot = y[k];
            y[k] = 0.0;
            while top < size {
                let i = pattern[top] as usize;
                let yi = y[i];
                y[i] = 0.0;
                let start = lp[i] as usize;
                let end = start + lnz[i] as usize;
                for p in start..end { y[li[p] as usize] -= lx[p] * yi; }
                let l = yi / d[i];
                pivot -= l * yi;
                li[end] = k as i32;
                lx[end] = l;
                lnz[i] += 1;
                top += 1;
            }
            // Mass and Gauss–Newton stiffness are positive definite; guard against rounding only.
            if !(pivot > 0.0) { return false; }
            d[k] = pivot;
        }
        true
    }

    /// step = H⁻¹ gradient, in the caller's vertex order.
    fn back_solve(&mut self) {
        let n = self.n;
        let size = 3 * n;
        let (lp, li, lx, d, perm) = (R(&self.lp), R(&self.li), R(&self.lx), R(&self.d), R(&self.perm));
        let mut y = W(&mut self.y);
        let g = R(&self.asm.gradient);
        for i in 0..n {
            let p = 3 * perm[i] as usize;
            let k = 3 * i;
            y[p] = g[k]; y[p + 1] = g[k + 1]; y[p + 2] = g[k + 2];
        }
        for j in 0..size {
            let yj = y[j];
            if yj != 0.0 {
                for p in lp[j] as usize..lp[j + 1] as usize { y[li[p] as usize] -= lx[p] * yj; }
            }
        }
        for j in (0..size).rev() {
            let mut s = y[j] / d[j];
            for p in lp[j] as usize..lp[j + 1] as usize { s -= lx[p] * y[li[p] as usize]; }
            y[j] = s;
        }
        let mut out = W(&mut self.step);
        for i in 0..n {
            let p = 3 * perm[i] as usize;
            let k = 3 * i;
            out[k] = y[p]; out[k + 1] = y[p + 1]; out[k + 2] = y[p + 2];
        }
        y.0.fill(0.0);
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
