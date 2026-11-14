//! Contact between layers of the folded sheet that are coplanar at rest: a vertex of one
//! panel against a face of another. `detect` refreshes the active contacts each substep
//! and `solve` projects them, with friction, in the constraint sweeps.
//!
//! The candidate vertex–face pairs, grouped by face, and their seeded sides are built in
//! TypeScript (src/physics/layer-contact.ts) and passed in as `LayerSetup`. Ordering of
//! exactly coincident panels is selected by the first local opening, not by an inferred
//! fold-stack order.
use crate::fast::{R, W};
use crate::js;

/// Contact activates within NEAR (metres) of the face.
const NEAR: f64 = 0.0005;
/// Between full passes a candidate stays listed while within REACH (barycentric) and
/// REACH_GAP (metres) of its face.
const REACH: f64 = 0.25;
const REACH_GAP: f64 = 0.01;
/// An unloaded contact farther than this from closing may skip sweeps; it is rechecked
/// every COLD_SWEEPS sweeps.
const COLD_GAP: f64 = 1e-4;
const COLD_SWEEPS: usize = 8;

/// The candidate list and its side state, as flat arrays from the TypeScript set-up.
pub struct LayerSetup {
    pub vertex: Vec<i32>,
    pub face_index: Vec<i32>,
    pub faces: Vec<i32>,
    pub orientation: Vec<f64>,
    pub ambiguous: Vec<i32>,
    pub side_index: Vec<i32>,
    pub sign: Vec<f64>,
    pub chosen: Vec<i32>,
    pub side: Vec<f64>,
}

/// The candidate list and one substep's active contacts.
pub struct Layers {
    vertex: Vec<i32>,
    face_index: Vec<i32>,
    faces: Vec<i32>,
    orientation: Vec<f64>,
    ambiguous: Vec<u8>,
    side_index: Vec<i32>,
    sign: Vec<f64>,
    chosen: Vec<u8>,
    side: Vec<f64>,
    /// Per face: a, ab, ac, unit normal, previous unit normal, previous a, Gram terms.
    geometry: Vec<f64>,
    valid: Vec<u8>,
    /// Candidates within reach of their face at the last full pass.
    near: Vec<i32>,
    near_count: usize,
    pub count: usize,
    ids: Vec<i32>,
    /// Candidate index of each active contact.
    candidate: Vec<i32>,
    /// Per contact: weights, normal, old relative position, tangent impulse, normal impulse.
    data: Vec<f64>,
    /// Unloaded contacts last seen clear of their face; rechecked every COLD_SWEEPS sweeps.
    cold: Vec<u8>,
}

impl Layers {
    pub fn new(s: LayerSetup) -> Self {
        let count = s.vertex.len();
        let face_count = s.faces.len() / 3;
        Layers {
            vertex: s.vertex,
            face_index: s.face_index,
            faces: s.faces,
            orientation: s.orientation,
            ambiguous: s.ambiguous.iter().map(|&a| a as u8).collect(),
            side_index: s.side_index,
            sign: s.sign,
            chosen: s.chosen.iter().map(|&a| a as u8).collect(),
            side: s.side,
            geometry: vec![0.0; face_count * 22],
            valid: vec![0; face_count],
            near: vec![0; count],
            near_count: 0,
            count: 0,
            ids: Vec::new(),
            candidate: Vec::new(),
            data: Vec::new(),
            cold: Vec::new(),
        }
    }

    /// Side state per candidate (seeded sign, whether chosen) and per shared side.
    pub fn sides(&self) -> (&[f64], &[u8], &[f64]) { (&self.sign, &self.chosen, &self.side) }
    /// Active contacts: candidate index each, and thirteen values each (face weights, normal,
    /// old relative position, tangent impulse, normal impulse).
    pub fn contacts(&self) -> (&[i32], &[f64]) { (&self.candidate[..self.count], &self.data[..13 * self.count]) }
    /// Replace the active set by one contact between vertex 0 and face (1, 2, 3).
    pub fn adopt(&mut self, data: &[f64]) {
        self.count = 1;
        self.cold = vec![0];
        self.ids = vec![0, 1, 2, 3];
        self.candidate = vec![0];
        self.data = data[..13].to_vec();
    }
    /// Refresh barycentric contact points each substep so a vertex can slide across
    /// triangle boundaries. Candidate groups persist through separation and recontact.
    pub fn detect(&mut self, x: &[f64], old: &[f64], full: bool) {
        let faces = &self.faces;
        let g = &mut self.geometry;
        let valid = &mut self.valid;
        for f in 0..valid.len() {
            let (a, b, c) = (3 * faces[3 * f] as usize, 3 * faces[3 * f + 1] as usize, 3 * faces[3 * f + 2] as usize);
            let o = 22 * f;
            valid[f] = 0;
            let (ax, ay, az) = (x[a], x[a + 1], x[a + 2]);
            let (abx, aby, abz) = (x[b] - ax, x[b + 1] - ay, x[b + 2] - az);
            let (acx, acy, acz) = (x[c] - ax, x[c + 1] - ay, x[c + 2] - az);
            let (avx, avy, avz) = (aby * acz - abz * acy, abz * acx - abx * acz, abx * acy - aby * acx);
            let len = js::hypot3(avx, avy, avz);
            if len < 1e-12 { continue; }
            let aa = abx * abx + aby * aby + abz * abz;
            let bb = acx * acx + acy * acy + acz * acz;
            let cc = abx * acx + aby * acy + abz * acz;
            let den = aa * bb - cc * cc;
            if den < 1e-20 { continue; }
            let (ux, uy, uz) = (old[b] - old[a], old[b + 1] - old[a + 1], old[b + 2] - old[a + 2]);
            let (vx, vy, vz) = (old[c] - old[a], old[c + 1] - old[a + 1], old[c + 2] - old[a + 2]);
            let (ox, oy, oz) = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
            let inverse = 1.0 / len;
            let old_inverse = 1.0 / js::max(js::hypot3(ox, oy, oz), 1e-12);
            let gg = &mut g[o..o + 22];
            gg[0] = ax; gg[1] = ay; gg[2] = az; gg[3] = abx; gg[4] = aby; gg[5] = abz; gg[6] = acx; gg[7] = acy; gg[8] = acz;
            gg[9] = avx * inverse; gg[10] = avy * inverse; gg[11] = avz * inverse;
            gg[12] = ox * old_inverse; gg[13] = oy * old_inverse; gg[14] = oz * old_inverse;
            gg[15] = old[a]; gg[16] = old[a + 1]; gg[17] = old[a + 2];
            gg[18] = aa; gg[19] = bb; gg[20] = cc; gg[21] = den;
            valid[f] = 1;
        }
        self.count = 0;
        // A full pass visits every candidate and records those within reach of
        // their face. Later substeps of the same frame revisit only that list.
        let total = if full { self.vertex.len() } else { self.near_count };
        if full { self.near_count = 0; }
        // Candidates are grouped by face: keep its geometry in locals while it repeats.
        let mut f: isize = -1;
        let (mut ax, mut ay, mut az, mut nx, mut ny, mut nz) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let (mut onx, mut ony, mut onz, mut oax, mut oay, mut oaz) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let (mut abx, mut aby, mut abz, mut acx, mut acy, mut acz) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let (mut aa, mut bb, mut cc, mut den) = (0.0, 0.0, 0.0, 1.0);
        for q in 0..total {
            let i = if full { q } else { self.near[q] as usize };
            let state = self.side_index[i];
            if state >= 0 && self.side[state as usize] != 0.0 {
                self.sign[i] = self.side[state as usize] * self.orientation[i];
                self.chosen[i] = 1;
            }
            if self.face_index[i] as isize != f {
                f = self.face_index[i] as isize;
                let o = 22 * f as usize;
                let g = &self.geometry[o..o + 22];
                ax = g[0]; ay = g[1]; az = g[2]; abx = g[3]; aby = g[4]; abz = g[5]; acx = g[6]; acy = g[7]; acz = g[8];
                nx = g[9]; ny = g[10]; nz = g[11]; onx = g[12]; ony = g[13]; onz = g[14]; oax = g[15]; oay = g[16]; oaz = g[17];
                aa = g[18]; bb = g[19]; cc = g[20]; den = g[21];
            }
            let fu = f as usize;
            if self.valid[fu] == 0 { continue; }
            let k = 3 * self.vertex[i] as usize;
            let (x, old) = (R(x), R(old));
            let (px, py, pz) = (x[k] - ax, x[k + 1] - ay, x[k + 2] - az);
            let raw_gap = px * nx + py * ny + pz * nz;
            let old_gap = (old[k] - oax) * onx + (old[k + 1] - oay) * ony + (old[k + 2] - oaz) * onz;
            // Follow the actual previous side, including large rotations and recontact.
            // A rest ordering must never become a long-range attractive constraint.
            if self.ambiguous[i] != 0 && old_gap.abs() > 1e-8 {
                self.sign[i] = js::sign(old_gap);
                self.chosen[i] = 1;
                if state >= 0 { self.side[state as usize] = self.sign[i] * self.orientation[i]; }
            }
            if self.chosen[i] != 0 && raw_gap * self.sign[i] > NEAR && !full { continue; }
            let dab = px * abx + py * aby + pz * abz;
            let dac = px * acx + py * acy + pz * acz;
            let u = (dab * bb - dac * cc) / den;
            let v = (dac * aa - dab * cc) / den;
            if full {
                if u >= -REACH && v >= -REACH && u + v <= 1.0 + REACH && !(self.chosen[i] != 0 && raw_gap * self.sign[i] > NEAR + REACH_GAP) {
                    self.near[self.near_count] = i as i32;
                    self.near_count += 1;
                }
                if self.chosen[i] != 0 && raw_gap * self.sign[i] > NEAR { continue; }
            }
            if u < -1e-8 || v < -1e-8 || u + v > 1.0 + 1e-8 { continue; }
            if self.ambiguous[i] != 0 && self.chosen[i] == 0 {
                if raw_gap.abs() < 1e-8 { continue; }
                // Zero-thickness recipes have no ordered sides. The first opening selects
                // a side locally, rather than imposing incompatible ordering at shared seams.
                self.sign[i] = js::sign(raw_gap);
                self.chosen[i] = 1;
                if state >= 0 { self.side[state as usize] = self.sign[i] * self.orientation[i]; }
            }
            let s = self.sign[i];
            if raw_gap * s > NEAR { continue; }
            let n = self.count;
            self.count += 1;
            if self.cold.len() < self.count {
                self.cold.resize(self.count, 0);
                self.ids.resize(4 * self.count, 0);
                self.candidate.resize(self.count, 0);
                self.data.resize(13 * self.count, 0.0);
            }
            let w0 = 1.0 - u - v;
            let (fa, fb, fc) = (self.faces[3 * fu], self.faces[3 * fu + 1], self.faces[3 * fu + 2]);
            let (a, b, c) = (3 * fa as usize, 3 * fb as usize, 3 * fc as usize);
            self.candidate[n] = i as i32;
            self.ids[4 * n] = self.vertex[i];
            self.ids[4 * n + 1] = fa;
            self.ids[4 * n + 2] = fb;
            self.ids[4 * n + 3] = fc;
            let d = &mut self.data[13 * n..13 * n + 13];
            d[0] = w0; d[1] = u; d[2] = v; d[3] = nx * s; d[4] = ny * s; d[5] = nz * s;
            d[6] = old[k] - w0 * old[a] - u * old[b] - v * old[c];
            d[7] = old[k + 1] - w0 * old[a + 1] - u * old[b + 1] - v * old[c + 1];
            d[8] = old[k + 2] - w0 * old[a + 2] - u * old[b + 2] - v * old[c + 2];
            d[9] = 0.0; d[10] = 0.0; d[11] = 0.0; d[12] = 0.0;
            self.cold[n] = 0;
        }
    }

    /// Unilateral normal projection plus a Coulomb disk |lambda_t| <= mu lambda_n.
    /// Internal equal/opposite barycentric corrections preserve the mass centre.
    pub fn solve(&mut self, x: &mut [f64], w: &[f64], friction: f64, sweep: usize) {
        let recheck = sweep % COLD_SWEEPS == 0;
        let (mut x, w) = (W(x), R(w));
        for n in 0..self.count {
            if self.cold[n] != 0 && !recheck { continue; }
            let (iv, i0, i1, i2) = (self.ids[4 * n] as usize, self.ids[4 * n + 1] as usize, self.ids[4 * n + 2] as usize, self.ids[4 * n + 3] as usize);
            let (pv, a, b, c) = (3 * iv, 3 * i0, 3 * i1, 3 * i2);
            let d = &mut self.data[13 * n..13 * n + 13];
            let (k0, k1, k2, nx, ny, nz) = (d[0], d[1], d[2], d[3], d[4], d[5]);
            let rx = x[pv] - k0 * x[a] - k1 * x[b] - k2 * x[c];
            let ry = x[pv + 1] - k0 * x[a + 1] - k1 * x[b + 1] - k2 * x[c + 1];
            let rz = x[pv + 2] - k0 * x[a + 2] - k1 * x[b + 2] - k2 * x[c + 2];
            let gap = rx * nx + ry * ny + rz * nz;
            let lambda = d[12];
            // A separated contact that carries no impulse projects to exactly nothing:
            // friction is bounded by the (zero) normal multiplier.
            if lambda == 0.0 && gap >= 0.0 {
                self.cold[n] = if gap > COLD_GAP { 1 } else { 0 };
                continue;
            }
            self.cold[n] = 0;
            let wv = w[iv];
            let (w0, w1, w2) = (w[i0] * k0, w[i1] * k1, w[i2] * k2);
            let den = wv + w[i0] * (k0 * k0) + w[i1] * (k1 * k1) + w[i2] * (k2 * k2);
            if den <= 0.0 { continue; }
            let next = js::max(0.0, lambda - gap / den);
            let dl = next - lambda;
            d[12] = next;
            let (sx, sy, sz) = (rx - d[6], ry - d[7], rz - d[8]);
            let normal_slip = sx * nx + sy * ny + sz * nz;
            let tx = d[9] - (sx - normal_slip * nx) / den;
            let ty = d[10] - (sy - normal_slip * ny) / den;
            let tz = d[11] - (sz - normal_slip * nz) / den;
            let magnitude = js::hypot3(tx, ty, tz);
            let limit = friction * next;
            let factor = if magnitude > limit { limit / js::max(magnitude, 1e-20) } else { 1.0 };
            let (fx, fy, fz) = (tx * factor, ty * factor, tz * factor);
            let (ix, iy, iz) = (dl * nx + fx - d[9], dl * ny + fy - d[10], dl * nz + fz - d[11]);
            d[9] = fx; d[10] = fy; d[11] = fz;
            x[pv] += wv * ix; x[pv + 1] += wv * iy; x[pv + 2] += wv * iz;
            x[a] -= w0 * ix; x[a + 1] -= w0 * iy; x[a + 2] -= w0 * iz;
            x[b] -= w1 * ix; x[b + 1] -= w1 * iy; x[b + 2] -= w1 * iz;
            x[c] -= w2 * ix; x[c + 1] -= w2 * iy; x[c + 2] -= w2 * iz;
        }
    }
}
