//! The elastic constraint sweeps of a substep: XPBD projection of the membrane strains
//! and the hinge angles, the edge strain envelope, layer ties, and the axial edge damping
//! applied to velocities afterwards. All read flat xyz coordinates, three per vertex.
use crate::fast::{R, W};
use crate::js;
use crate::lanes::{sweep_order, Lanes, F2};

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
    /// The sweep's order: see `sweep_order`.
    order: Vec<i32>,
}
impl MembraneBatch {
    pub fn new(faces: Vec<i32>, md: Vec<f64>, cd: Vec<f64>, alpha: Vec<f64>, lambda: Vec<f64>) -> Self {
        let order = sweep_order(&faces, 3);
        MembraneBatch { faces, md, cd, alpha, lambda, order }
    }
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
    /// The sweep's order: see `sweep_order`.
    order: Vec<i32>,
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
    /// Hinges with no plastic offset set yet, and so no target angle.
    pub fn new(ids: Vec<i32>, rest: Vec<f64>, alpha: Vec<f64>, lambda: Vec<f64>) -> Self {
        let count = rest.len();
        let order = sweep_order(&ids, 4);
        HingeBatch { ids, rest, offset: vec![f64::NAN; count], target_sin: vec![0.0; count], target_cos: vec![0.0; count], alpha, lambda, order }
    }
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
/// One XPBD projection of face `f` (or of two faces that share no vertex); `alpha` is
/// compliance/dt² per mode.
#[allow(clippy::too_many_arguments)]
#[inline(always)]
unsafe fn membrane<T: Lanes>(x: *mut f64, w: *const f64, faces: *const i32, md: *const f64, cd: *const f64, alpha: *const f64, lambda: *mut f64, f: T::Idx) {
    let (ia, ib, ic) = (T::id(faces, f, 3, 0), T::id(faces, f, 3, 1), T::id(faces, f, 3, 2));
    let (wa, wb, wc) = (T::get(w, ia, 1, 0), T::get(w, ib, 1, 0), T::get(w, ic, 1, 0));
    let (mut ax, mut ay, mut az) = (T::get(x, ia, 3, 0), T::get(x, ia, 3, 1), T::get(x, ia, 3, 2));
    let (mut bx, mut by, mut bz) = (T::get(x, ib, 3, 0), T::get(x, ib, 3, 1), T::get(x, ib, 3, 2));
    let (mut cx, mut cy, mut cz) = (T::get(x, ic, 3, 0), T::get(x, ic, 3, 1), T::get(x, ic, 3, 2));
    let (m0, m1, m2) = (T::get(md, f, 3, 0), T::get(md, f, 3, 1), T::get(md, f, 3, 2));
    let (c0, c1, c2) = (T::get(cd, f, 3, 0), T::get(cd, f, 3, 1), T::get(cd, f, 3, 2));
    let (one, half) = (T::splat(1.0), T::splat(0.5));
    // The gradient of a normal strain at vertex i is d_i u, and of the shear m_i v + c_i u,
    // with u and v the deformed material directions. So the projections' denominators need
    // only u·u, u·v and v·v and these weighted sums of the shape gradients.
    let (pm0, pm1, pm2, pc0, pc1, pc2) = (wa * m0, wb * m1, wc * m2, wa * c0, wb * c1, wc * c2);
    let kmm = pm0 * m0 + pm1 * m1 + pm2 * m2;
    let kcc = pc0 * c0 + pc1 * c1 + pc2 * c2;
    let kmc = pm0 * c0 + pm1 * c1 + pm2 * c2;
    // Each normal mode needs only its own material derivative. Recompute it
    // after the preceding Gauss-Seidel correction; shear still needs both.
    for mode in 0..2 {
        let (d1, d2, p0, p1, p2, k) = if mode == 1 { (c1, c2, pc0, pc1, pc2, kcc) } else { (m1, m2, pm0, pm1, pm2, kmm) };
        let (ex, ey, ez, fx, fy, fz) = (bx - ax, by - ay, bz - az, cx - ax, cy - ay, cz - az);
        let (ux, uy, uz) = (d1 * ex + d2 * fx, d1 * ey + d2 * fy, d1 * ez + d2 * fz);
        let uu = ux * ux + uy * uy + uz * uz;
        let cval = (uu - one) * half;
        let stiffness = T::get(alpha, f, 3, mode);
        let multiplier = T::get(lambda, f, 3, mode);
        let delta = (-cval - stiffness * multiplier) / (stiffness + uu * k);
        (multiplier + delta).set(lambda, f, 3, mode);
        let (sa, sb, sc) = (p0 * delta, p1 * delta, p2 * delta);
        ax = ax + sa * ux; ay = ay + sa * uy; az = az + sa * uz;
        bx = bx + sb * ux; by = by + sb * uy; bz = bz + sb * uz;
        cx = cx + sc * ux; cy = cy + sc * uy; cz = cz + sc * uz;
    }
    let (ex, ey, ez, fx, fy, fz) = (bx - ax, by - ay, bz - az, cx - ax, cy - ay, cz - az);
    let (ux, uy, uz) = (m1 * ex + m2 * fx, m1 * ey + m2 * fy, m1 * ez + m2 * fz);
    let (vx, vy, vz) = (c1 * ex + c2 * fx, c1 * ey + c2 * fy, c1 * ez + c2 * fz);
    let cval = ux * vx + uy * vy + uz * vz;
    let (uu, vv) = (ux * ux + uy * uy + uz * uz, vx * vx + vy * vy + vz * vz);
    let stiffness = T::get(alpha, f, 3, 2);
    let multiplier = T::get(lambda, f, 3, 2);
    let delta = (-cval - stiffness * multiplier) / (stiffness + vv * kmm + (cval + cval) * kmc + uu * kcc);
    (multiplier + delta).set(lambda, f, 3, 2);
    let (ma, mb, mc, na, nb, nc) = (pm0 * delta, pm1 * delta, pm2 * delta, pc0 * delta, pc1 * delta, pc2 * delta);
    (ax + ma * vx + na * ux).set(x, ia, 3, 0); (ay + ma * vy + na * uy).set(x, ia, 3, 1); (az + ma * vz + na * uz).set(x, ia, 3, 2);
    (bx + mb * vx + nb * ux).set(x, ib, 3, 0); (by + mb * vy + nb * uy).set(x, ib, 3, 1); (bz + mb * vz + nb * uz).set(x, ib, 3, 2);
    (cx + mc * vx + nc * ux).set(x, ic, 3, 0); (cy + mc * vy + nc * uy).set(x, ic, 3, 1); (cz + mc * vz + nc * uz).set(x, ic, 3, 2);
}

/// One XPBD sweep over every membrane, in an order equivalent to index order (reversed
/// when not `forward`), two independent faces at a time where the mesh allows.
pub fn solve_membranes(x: &mut [f64], w: &[f64], batch: &mut MembraneBatch, forward: bool) {
    let slots = batch.order.len() / 2;
    let order = R(&batch.order);
    let (x, w, faces, md, cd, alpha, lambda) = (x.as_mut_ptr(), w.as_ptr(), batch.faces.as_ptr(), batch.md.as_ptr(), batch.cd.as_ptr(), batch.alpha.as_ptr(), batch.lambda.as_mut_ptr());
    for k in 0..slots {
        let s = 2 * if forward { k } else { slots - 1 - k };
        let (first, second) = (order[s] as usize, order[s + 1]);
        // Indices come from the mesh set-up and are in range by construction.
        unsafe {
            if second < 0 { membrane::<f64>(x, w, faces, md, cd, alpha, lambda, first); } else { membrane::<F2>(x, w, faces, md, cd, alpha, lambda, [first, second as usize]); }
        }
    }
}

/// One XPBD projection of hinge `j` (or of two hinges that share no vertex) on the signed
/// dihedral angle. False if it was left undone: one of a pair needs the rare path (a
/// degenerate hinge, or an angle error too large for the series), so each must be
/// projected alone. The signed-angle gradients are adapted from MIT-licensed
/// PositionBasedDynamics; see THIRD_PARTY.md.
#[allow(clippy::too_many_arguments)]
#[inline(always)]
unsafe fn hinge<T: Lanes>(x: *mut f64, w: *const f64, ids: *const i32, target_sin: *const f64, target_cos: *const f64, alpha: *const f64, lambda: *mut f64, j: T::Idx) -> bool {
    let (i0, i1, i2, i3) = (T::id(ids, j, 4, 0), T::id(ids, j, 4, 1), T::id(ids, j, 4, 2), T::id(ids, j, 4, 3));
    let (ax, ay, az, bx, by, bz) = (T::get(x, i0, 3, 0), T::get(x, i0, 3, 1), T::get(x, i0, 3, 2), T::get(x, i1, 3, 0), T::get(x, i1, 3, 1), T::get(x, i1, 3, 2));
    let (cx, cy, cz, dx, dy, dz) = (T::get(x, i2, 3, 0), T::get(x, i2, 3, 1), T::get(x, i2, 3, 2), T::get(x, i3, 3, 0), T::get(x, i3, 3, 1), T::get(x, i3, 3, 2));
    let (ex, ey, ez) = (dx - cx, dy - cy, dz - cz);
    let ee = ex * ex + ey * ey + ez * ez;
    let el = ee.sqrt();
    let (ux, uy, uz, vx, vy, vz) = (cx - ax, cy - ay, cz - az, dx - ax, dy - ay, dz - az);
    let (rx, ry, rz, sx, sy, sz) = (dx - bx, dy - by, dz - bz, cx - bx, cy - by, cz - bz);
    let (n1x, n1y, n1z) = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
    let (n2x, n2y, n2z) = (ry * sz - rz * sy, rz * sx - rx * sz, rx * sy - ry * sx);
    let sq1 = n1x * n1x + n1y * n1y + n1z * n1z;
    let sq2 = n2x * n2x + n2y * n2y + n2z * n2z;
    if T::any(el.lt(T::splat(1e-10)) | sq1.lt(T::splat(1e-18)) | sq2.lt(T::splat(1e-18))) { return !T::PAIR; }
    let one = T::splat(1.0);
    let (inverse_el, inverse1, inverse2) = (one / el, one / sq1, one / sq2);
    let sin = ((n1y * n2z - n1z * n2y) * ex + (n1z * n2x - n1x * n2z) * ey + (n1x * n2y - n1y * n2x) * ez) * inverse_el;
    let cos = n1x * n2x + n1y * n2y + n1z * n2z;
    let (st, ct) = (T::get(target_sin, j, 1, 0), T::get(target_cos, j, 1, 0));
    // Angle error from the rotated (sin, cos) pair: already wrapped, and for
    // the usual small error a short odd series replaces atan2 (error < 1e-12).
    let y = sin * ct - cos * st;
    let z = cos * ct + sin * st;
    let cval = if T::all(z.gt(T::splat(0.0)) & y.abs().lt(T::splat(0.1) * z)) {
        let t = y / z;
        let q = t * t;
        t * (one - q * (T::splat(1.0 / 3.0) - q * (T::splat(1.0 / 5.0) - q * (T::splat(1.0 / 7.0) - q * T::splat(1.0 / 9.0)))))
    } else {
        if T::PAIR { return false; }
        y.atan2(z)
    };
    // The wing vertices' gradients are the face normals scaled by −|e|/|n|², so their
    // squared lengths are |e|²/|n|²; the edge vertices' are combinations of the two.
    let (n1x, n1y, n1z, n2x, n2y, n2z) = (n1x * inverse1, n1y * inverse1, n1z * inverse1, n2x * inverse2, n2y * inverse2, n2z * inverse2);
    let (g0x, g0y, g0z, g1x, g1y, g1z) = (-el * n1x, -el * n1y, -el * n1z, -el * n2x, -el * n2y, -el * n2z);
    let k2 = (vx * ex + vy * ey + vz * ez) * inverse_el;
    let l2 = (rx * ex + ry * ey + rz * ez) * inverse_el;
    let k3 = -(ux * ex + uy * ey + uz * ez) * inverse_el;
    let l3 = -(sx * ex + sy * ey + sz * ez) * inverse_el;
    let (g2x, g2y, g2z) = (k2 * n1x + l2 * n2x, k2 * n1y + l2 * n2y, k2 * n1z + l2 * n2z);
    let (g3x, g3y, g3z) = (k3 * n1x + l3 * n2x, k3 * n1y + l3 * n2y, k3 * n1z + l3 * n2z);
    let stiffness = T::get(alpha, j, 1, 0);
    let (m0, m1, m2, m3) = (T::get(w, i0, 1, 0), T::get(w, i1, 1, 0), T::get(w, i2, 1, 0), T::get(w, i3, 1, 0));
    let denom = stiffness + m0 * (ee * inverse1) + m1 * (ee * inverse2) + m2 * (g2x * g2x + g2y * g2y + g2z * g2z) + m3 * (g3x * g3x + g3y * g3y + g3z * g3z);
    let multiplier = T::get(lambda, j, 1, 0);
    let dl = (-cval - stiffness * multiplier) / denom;
    (multiplier + dl).set(lambda, j, 1, 0);
    let (w0, w1, w2, w3) = (m0 * dl, m1 * dl, m2 * dl, m3 * dl);
    (ax + w0 * g0x).set(x, i0, 3, 0); (ay + w0 * g0y).set(x, i0, 3, 1); (az + w0 * g0z).set(x, i0, 3, 2);
    (bx + w1 * g1x).set(x, i1, 3, 0); (by + w1 * g1y).set(x, i1, 3, 1); (bz + w1 * g1z).set(x, i1, 3, 2);
    (cx + w2 * g2x).set(x, i2, 3, 0); (cy + w2 * g2y).set(x, i2, 3, 1); (cz + w2 * g2z).set(x, i2, 3, 2);
    (dx + w3 * g3x).set(x, i3, 3, 0); (dy + w3 * g3y).set(x, i3, 3, 1); (dz + w3 * g3z).set(x, i3, 3, 2);
    true
}

/// One XPBD sweep over every hinge, in an order equivalent to index order (reversed when
/// not `forward`), two independent hinges at a time where the mesh allows. `alpha` is
/// compliance/dt² per hinge.
pub fn solve_hinges(x: &mut [f64], w: &[f64], batch: &mut HingeBatch, forward: bool) {
    let slots = batch.order.len() / 2;
    let order = R(&batch.order);
    let (x, w, ids, target_sin, target_cos, alpha, lambda) = (x.as_mut_ptr(), w.as_ptr(), batch.ids.as_ptr(), batch.target_sin.as_ptr(), batch.target_cos.as_ptr(), batch.alpha.as_ptr(), batch.lambda.as_mut_ptr());
    for k in 0..slots {
        let s = 2 * if forward { k } else { slots - 1 - k };
        let (first, second) = (order[s] as usize, order[s + 1]);
        // Indices come from the mesh set-up and are in range by construction.
        unsafe {
            if second < 0 {
                hinge::<f64>(x, w, ids, target_sin, target_cos, alpha, lambda, first);
            } else if !hinge::<F2>(x, w, ids, target_sin, target_cos, alpha, lambda, [first, second as usize]) {
                hinge::<f64>(x, w, ids, target_sin, target_cos, alpha, lambda, first);
                hinge::<f64>(x, w, ids, target_sin, target_cos, alpha, lambda, second as usize);
            }
        }
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
