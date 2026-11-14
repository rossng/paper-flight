//! The elastic constraint sweeps of a substep: XPBD projection of the membrane strains
//! and the hinge angles, the edge strain envelope, layer ties, and the axial edge damping
//! applied to velocities afterwards. All read flat xyz coordinates, three per vertex.
use crate::fast::{R, W};
use crate::js;

/// One membrane triangle per face, as structure-of-arrays. `md` and `cd` are the
/// constant reference shape gradients in the two orthogonal material directions, three
/// per face. A triangle-local metric preserves the exact folded rest shape. `alpha` is
/// compliance/dt² and `lambda` the multiplier, per mode: the two normal strains, then
/// shear.
pub struct MembraneBatch {
    pub faces: Vec<i32>,
    pub md: Vec<f64>,
    pub cd: Vec<f64>,
    pub alpha: Vec<f64>,
    pub lambda: Vec<f64>,
}
/// Structure-of-arrays hinge data: four vertex ids (the two wing vertices, then the
/// shared edge), rest angle, plastic offset and the sine and cosine of the target angle,
/// compliance/dt² and multiplier.
pub struct HingeBatch {
    pub ids: Vec<i32>,
    pub rest: Vec<f64>,
    pub offset: Vec<f64>,
    pub target_sin: Vec<f64>,
    pub target_cos: Vec<f64>,
    pub alpha: Vec<f64>,
    pub lambda: Vec<f64>,
}
/// Edges as vertex id pairs with their rest lengths.
pub struct EdgeBatch {
    pub ids: Vec<i32>,
    pub rest: Vec<f64>,
}

/// Legacy bonded-laminate attachments: vertex and three face ids, three weights each.
pub struct LayerTies {
    pub ids: Vec<i32>,
    pub weights: Vec<f64>,
}

impl HingeBatch {
    /// Set hinge j's permanent (plastic) offset from its rest angle.
    pub fn set_offset(&mut self, j: usize, offset: f64) {
        // An offset that has never been set is NaN, which never compares equal, so the
        // first call always sets the target.
        if self.offset[j] == offset { return; }
        self.offset[j] = offset;
        self.target_sin[j] = (self.rest[j] + offset).sin();
        self.target_cos[j] = (self.rest[j] + offset).cos();
    }
}

/// Diagonal orthotropic membrane energy: Et A/2 times squared Green normal
/// strain, Gt A/2 times squared engineering shear. Poisson membrane coupling is
/// omitted; explicit shear prevents skinny triangles collapsing like bar nets.
/// One XPBD sweep; `alpha` is compliance/dt² per mode.
pub fn solve_membranes(x: &mut [f64], w: &[f64], batch: &mut MembraneBatch, forward: bool) {
    let count = batch.faces.len() / 3;
    let (faces, md, cd, alpha) = (R(&batch.faces), R(&batch.md), R(&batch.cd), R(&batch.alpha));
    let mut lambda = W(&mut batch.lambda);
    let mut x = W(x);
    let w = R(w);
    for k in 0..count {
        let t = 3 * if forward { k } else { count - 1 - k };
        let (ia, ib, ic) = (faces[t] as usize, faces[t + 1] as usize, faces[t + 2] as usize);
        let (a, b, c) = (3 * ia, 3 * ib, 3 * ic);
        let (wa, wb, wc) = (w[ia], w[ib], w[ic]);
        let (mut ax, mut ay, mut az) = (x[a], x[a + 1], x[a + 2]);
        let (mut bx, mut by, mut bz) = (x[b], x[b + 1], x[b + 2]);
        let (mut cx, mut cy, mut cz) = (x[c], x[c + 1], x[c + 2]);
        let (m0, m1, m2, c0, c1, c2) = (md[t], md[t + 1], md[t + 2], cd[t], cd[t + 1], cd[t + 2]);
        // Each normal mode needs only its own material derivative. Recompute it
        // after the preceding Gauss-Seidel correction; shear still needs both.
        for mode in 0..2 {
            let (d0, d1, d2) = if mode == 1 { (c0, c1, c2) } else { (m0, m1, m2) };
            let (ex, ey, ez, fx, fy, fz) = (bx - ax, by - ay, bz - az, cx - ax, cy - ay, cz - az);
            let (ux, uy, uz) = (d1 * ex + d2 * fx, d1 * ey + d2 * fy, d1 * ez + d2 * fz);
            let cval = (ux * ux + uy * uy + uz * uz - 1.0) / 2.0;
            let stiffness = alpha[t + mode];
            let (g0x, g0y, g0z, g1x, g1y, g1z, g2x, g2y, g2z) = (d0 * ux, d0 * uy, d0 * uz, d1 * ux, d1 * uy, d1 * uz, d2 * ux, d2 * uy, d2 * uz);
            let den = stiffness + wa * (g0x * g0x + g0y * g0y + g0z * g0z) + wb * (g1x * g1x + g1y * g1y + g1z * g1z) + wc * (g2x * g2x + g2y * g2y + g2z * g2z);
            let delta = (-cval - stiffness * lambda[t + mode]) / den;
            lambda[t + mode] += delta;
            let (sa, sb, sc) = (wa * delta, wb * delta, wc * delta);
            ax += sa * g0x; ay += sa * g0y; az += sa * g0z;
            bx += sb * g1x; by += sb * g1y; bz += sb * g1z;
            cx += sc * g2x; cy += sc * g2y; cz += sc * g2z;
        }
        let (ex, ey, ez, fx, fy, fz) = (bx - ax, by - ay, bz - az, cx - ax, cy - ay, cz - az);
        let (ux, uy, uz) = (m1 * ex + m2 * fx, m1 * ey + m2 * fy, m1 * ez + m2 * fz);
        let (vx, vy, vz) = (c1 * ex + c2 * fx, c1 * ey + c2 * fy, c1 * ez + c2 * fz);
        let cval = ux * vx + uy * vy + uz * vz;
        let stiffness = alpha[t + 2];
        let (g0x, g0y, g0z) = (m0 * vx + c0 * ux, m0 * vy + c0 * uy, m0 * vz + c0 * uz);
        let (g1x, g1y, g1z) = (m1 * vx + c1 * ux, m1 * vy + c1 * uy, m1 * vz + c1 * uz);
        let (g2x, g2y, g2z) = (m2 * vx + c2 * ux, m2 * vy + c2 * uy, m2 * vz + c2 * uz);
        let den = stiffness + wa * (g0x * g0x + g0y * g0y + g0z * g0z) + wb * (g1x * g1x + g1y * g1y + g1z * g1z) + wc * (g2x * g2x + g2y * g2y + g2z * g2z);
        let delta = (-cval - stiffness * lambda[t + 2]) / den;
        lambda[t + 2] += delta;
        let (sa, sb, sc) = (wa * delta, wb * delta, wc * delta);
        x[a] = ax + sa * g0x; x[a + 1] = ay + sa * g0y; x[a + 2] = az + sa * g0z;
        x[b] = bx + sb * g1x; x[b + 1] = by + sb * g1y; x[b + 2] = bz + sb * g1z;
        x[c] = cx + sc * g2x; x[c + 1] = cy + sc * g2y; x[c + 2] = cz + sc * g2z;
    }
}

/// One XPBD sweep over every hinge, on the signed dihedral angle. `alpha` is
/// compliance/dt² per hinge. The signed-angle gradients are adapted from MIT-licensed
/// PositionBasedDynamics; see THIRD_PARTY.md.
pub fn solve_hinges(x: &mut [f64], w: &[f64], batch: &mut HingeBatch, forward: bool) {
    let count = batch.lambda.len();
    let (ids, target_sin, target_cos, alpha) = (R(&batch.ids), R(&batch.target_sin), R(&batch.target_cos), R(&batch.alpha));
    let mut lambda = W(&mut batch.lambda);
    let mut x = W(x);
    let w = R(w);
    for k in 0..count {
        let j = if forward { k } else { count - 1 - k };
        let (i0, i1, i2, i3) = (ids[4 * j] as usize, ids[4 * j + 1] as usize, ids[4 * j + 2] as usize, ids[4 * j + 3] as usize);
        let (a, b, c, d) = (3 * i0, 3 * i1, 3 * i2, 3 * i3);
        let (ax, ay, az, bx, by, bz) = (x[a], x[a + 1], x[a + 2], x[b], x[b + 1], x[b + 2]);
        let (cx, cy, cz, dx, dy, dz) = (x[c], x[c + 1], x[c + 2], x[d], x[d + 1], x[d + 2]);
        let (ex, ey, ez) = (dx - cx, dy - cy, dz - cz);
        let el = (ex * ex + ey * ey + ez * ez).sqrt();
        let (ux, uy, uz, vx, vy, vz) = (cx - ax, cy - ay, cz - az, dx - ax, dy - ay, dz - az);
        let (rx, ry, rz, sx, sy, sz) = (dx - bx, dy - by, dz - bz, cx - bx, cy - by, cz - bz);
        let (mut n1x, mut n1y, mut n1z) = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
        let (mut n2x, mut n2y, mut n2z) = (ry * sz - rz * sy, rz * sx - rx * sz, rx * sy - ry * sx);
        let sq1 = n1x * n1x + n1y * n1y + n1z * n1z;
        let sq2 = n2x * n2x + n2y * n2y + n2z * n2z;
        if el < 1e-10 || sq1 < 1e-18 || sq2 < 1e-18 { continue; }
        let sin = ((n1y * n2z - n1z * n2y) * ex + (n1z * n2x - n1x * n2z) * ey + (n1x * n2y - n1y * n2x) * ez) / el;
        let cos = n1x * n2x + n1y * n2y + n1z * n2z;
        let (st, ct) = (target_sin[j], target_cos[j]);
        // Angle error from the rotated (sin, cos) pair: already wrapped, and for
        // the usual small error a short odd series replaces atan2 (error < 1e-12).
        let y = sin * ct - cos * st;
        let z = cos * ct + sin * st;
        let cval = if z > 0.0 && y.abs() < 0.1 * z {
            let t = y / z;
            let q = t * t;
            t * (1.0 - q * (1.0 / 3.0 - q * (1.0 / 5.0 - q * (1.0 / 7.0 - q / 9.0))))
        } else {
            y.atan2(z)
        };
        n1x /= sq1; n1y /= sq1; n1z /= sq1; n2x /= sq2; n2y /= sq2; n2z /= sq2;
        let (g0x, g0y, g0z, g1x, g1y, g1z) = (-el * n1x, -el * n1y, -el * n1z, -el * n2x, -el * n2y, -el * n2z);
        let k2 = (vx * ex + vy * ey + vz * ez) / el;
        let l2 = (rx * ex + ry * ey + rz * ez) / el;
        let k3 = -(ux * ex + uy * ey + uz * ez) / el;
        let l3 = -(sx * ex + sy * ey + sz * ez) / el;
        let (g2x, g2y, g2z) = (k2 * n1x + l2 * n2x, k2 * n1y + l2 * n2y, k2 * n1z + l2 * n2z);
        let (g3x, g3y, g3z) = (k3 * n1x + l3 * n2x, k3 * n1y + l3 * n2y, k3 * n1z + l3 * n2z);
        let stiffness = alpha[j];
        let denom = stiffness + w[i0] * (g0x * g0x + g0y * g0y + g0z * g0z) + w[i1] * (g1x * g1x + g1y * g1y + g1z * g1z) + w[i2] * (g2x * g2x + g2y * g2y + g2z * g2z) + w[i3] * (g3x * g3x + g3y * g3y + g3z * g3z);
        let dl = (-cval - stiffness * lambda[j]) / denom;
        lambda[j] += dl;
        let (w0, w1, w2, w3) = (w[i0] * dl, w[i1] * dl, w[i2] * dl, w[i3] * dl);
        x[a] = ax + w0 * g0x; x[a + 1] = ay + w0 * g0y; x[a + 2] = az + w0 * g0z;
        x[b] = bx + w1 * g1x; x[b + 1] = by + w1 * g1y; x[b + 2] = bz + w1 * g1z;
        x[c] = cx + w2 * g2x; x[c + 1] = cy + w2 * g2y; x[c + 2] = cz + w2 * g2z;
        x[d] = dx + w3 * g3x; x[d + 1] = dy + w3 * g3y; x[d + 2] = dz + w3 * g3z;
    }
}

/// Pairwise axial damping dissipates internal strain rates, preserving linear/angular momentum.
pub fn damp_edges(x: &[f64], v: &mut [f64], w: &[f64], edges: &EdgeBatch, amount: f64) {
    let (ids, x, w) = (R(&edges.ids), R(x), R(w));
    let mut v = W(v);
    for j in 0..edges.rest.len() {
        let (ia, ib) = (ids[2 * j] as usize, ids[2 * j + 1] as usize);
        let (a, b) = (3 * ia, 3 * ib);
        let (nx, ny, nz) = (x[b] - x[a], x[b + 1] - x[a + 1], x[b + 2] - x[a + 2]);
        let l = (nx * nx + ny * ny + nz * nz).sqrt();
        if l < 1e-12 { continue; }
        let (dx, dy, dz) = (nx / l, ny / l, nz / l);
        let rate = (v[b] - v[a]) * dx + (v[b + 1] - v[a + 1]) * dy + (v[b + 2] - v[a + 2]) * dz;
        let impulse = amount * rate / (w[ia] + w[ib]);
        let (sa, sb) = (w[ia] * impulse, w[ib] * impulse);
        v[a] += sa * dx; v[a + 1] += sa * dy; v[a + 2] += sa * dz;
        v[b] -= sb * dx; v[b + 1] -= sb * dy; v[b + 2] -= sb * dz;
    }
}

/// Near-inextensible safety envelope for a game without tearing. This inequality
/// is inactive in the elastic range; it prevents under-resolved stiff modes from
/// turning a thin sheet into a rubber net. It is not an extra elastic modulus.
pub fn limit_edge_strains(x: &mut [f64], w: &[f64], edges: &EdgeBatch, forward: bool, limit: f64) {
    let (ids, rest, w) = (R(&edges.ids), R(&edges.rest), R(w));
    let mut x = W(x);
    let count = rest.len();
    for k in 0..count {
        let j = if forward { k } else { count - 1 - k };
        let (ia, ib) = (ids[2 * j] as usize, ids[2 * j + 1] as usize);
        let (a, b) = (3 * ia, 3 * ib);
        let (dx, dy, dz) = (x[a] - x[b], x[a + 1] - x[b + 1], x[a + 2] - x[b + 2]);
        let l = (dx * dx + dy * dy + dz * dz).sqrt();
        if l < 1e-12 { continue; }
        let target = js::max(rest[j] * (1.0 - limit), js::min(rest[j] * (1.0 + limit), l));
        // An inactive inequality has exactly zero impulse; avoid six position writes
        // in the overwhelmingly common elastic range without weakening the envelope.
        if target == l { continue; }
        let impulse = (target - l) / (l * (w[ia] + w[ib]));
        let (wa, wb) = (w[ia] * impulse, w[ib] * impulse);
        x[a] += wa * dx; x[a + 1] += wa * dy; x[a + 2] += wa * dz;
        x[b] -= wb * dx; x[b + 1] -= wb * dy; x[b + 2] -= wb * dz;
    }
}

/// True when every edge is within `limit` of its rest length.
pub fn edge_strains_within(x: &[f64], edges: &EdgeBatch, limit: f64) -> bool {
    let (ids, rest, x) = (R(&edges.ids), R(&edges.rest), R(x));
    for j in 0..rest.len() {
        let (a, b) = (3 * ids[2 * j] as usize, 3 * ids[2 * j + 1] as usize);
        let (dx, dy, dz) = (x[a] - x[b], x[a + 1] - x[b + 1], x[a + 2] - x[b + 2]);
        if !(((dx * dx + dy * dy + dz * dz).sqrt() / rest[j] - 1.0).abs() < limit) { return false; }
    }
    true
}

/// Effective laminate attachment, for every tie: hold a vertex at its weighted point of
/// a face. Conserves weighted position; not frictional contact.
pub fn solve_layer_ties(x: &mut [f64], w: &[f64], ties: &LayerTies) {
    for t in 0..ties.ids.len() / 4 {
        let vertex = ties.ids[4 * t] as usize;
        let face = [ties.ids[4 * t + 1] as usize, ties.ids[4 * t + 2] as usize, ties.ids[4 * t + 3] as usize];
        let weights = &ties.weights[3 * t..3 * t + 3];
        let mut den = 0.0;
        for j in 0..3 { den += w[face[j]] * (weights[j] * weights[j]); }
        let den = w[vertex] + den;
        for d in 0..3 {
            let mut held = 0.0;
            for j in 0..3 { held += x[3 * face[j] + d] * weights[j]; }
            let delta = -(x[3 * vertex + d] - held) / den;
            x[3 * vertex + d] += w[vertex] * delta;
            for j in 0..3 { x[3 * face[j] + d] -= w[face[j]] * weights[j] * delta; }
        }
    }
}
