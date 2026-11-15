//! Self-contact of the sheet: swept vertex–face and edge–edge detection with a refitted
//! BVH and cached broad-phase pair lists, and the unilateral projection that keeps
//! detected pairs on the side they came from. Tree topology, the edge list and the
//! one-ring adjacency are built in TypeScript (src/physics/) and passed in.
use crate::fast::{R, W};
use crate::js;
use crate::lanes::{Lanes, F2};

/// Features closer than this (metres) count as touching; also the padding of swept bounds.
const GAP_EPS: f64 = 1e-9;
/// Tolerance on the impact coordinates for a point to count as inside both features.
const PARAM_EPS: f64 = 1e-7;
/// Padding (metres) of the bounds the broad-phase pair lists are built from, and the
/// drift that makes the lists stale.
const PAIR_MARGIN: f64 = 0.003;
/// How far outside its features (in their coordinates) a carried contact may still act.
const BOUND: f64 = 0.05;
/// A contact separated by more than this (metres) is clearly separated and may skip
/// more sweeps.
const COLD_GAP: f64 = 3e-5;

/// Fixed topology BVH, refitted to swept bounds. No spatial-cell explosion for
/// fast motion and no all-pairs feature list retained between substeps.
///
/// Preorder layout retains the left-then-right visit order of the build. Escape
/// indices skip entire rejected subtrees without recursive calls or a stack.
pub struct Tree {
    /// Six bounds per node: low xyz, then high xyz.
    pub bounds: Vec<f64>,
    /// Feature index of a leaf node, −1 for an internal node.
    pub leaves: Vec<i32>,
    /// Right child of an internal node; its left child is the next node.
    pub right: Vec<i32>,
    /// The node after this node's subtree.
    pub escape: Vec<i32>,
}
impl Tree {
    pub fn new(leaves: Vec<i32>, right: Vec<i32>, escape: Vec<i32>) -> Self {
        Tree { bounds: vec![0.0; leaves.len() * 6], leaves, right, escape }
    }
    /// `boxes` holds six bounds per feature: low xyz, then high xyz.
    fn refit(&mut self, boxes: &[f64]) {
        let bounds = &mut self.bounds;
        for node in (0..self.leaves.len()).rev() {
            let offset = node * 6;
            let index = self.leaves[node];
            if index >= 0 {
                let source = index as usize * 6;
                bounds[offset..offset + 6].copy_from_slice(&boxes[source..source + 6]);
            } else {
                let left = offset + 6;
                let right = self.right[node] as usize * 6;
                for d in 0..3 {
                    bounds[offset + d] = js::min(bounds[left + d], bounds[right + d]);
                    bounds[offset + d + 3] = js::max(bounds[left + d + 3], bounds[right + d + 3]);
                }
            }
        }
    }
}

/// Isolate every root of the cubic c0 + c1 t + c2 t² + c3 t³ in [0,1] using its
/// derivative extrema. Unlike endpoint sign tests this also finds an even number of
/// crossings in a step. Returns the number of roots written to `out`, in ascending order.
pub(crate) fn cubic_roots(c0: f64, c1: f64, c2: f64, c3: f64, out: &mut [f64; 8]) -> usize {
    let magnitude = js::max(js::max(c0.abs(), c1.abs()), js::max(c2.abs(), c3.abs()));
    if magnitude < 1e-30 { return 0; }
    let (a, b, d, e) = (c0 / magnitude, c1 / magnitude, c2 / magnitude, c3 / magnitude);
    let epsilon = 1e-12;
    let mut cuts = [0.0f64; 4];
    let mut cut_count = 2;
    cuts[0] = 0.0;
    cuts[1] = 1.0;
    if e.abs() < epsilon {
        if d.abs() > epsilon {
            let t = -b / (2.0 * d);
            if t > 0.0 && t < 1.0 { cuts[cut_count] = t; cut_count += 1; }
        }
    } else {
        let disc = 4.0 * d * d - 12.0 * e * b;
        if disc >= 0.0 {
            let root = disc.sqrt();
            let t0 = (-2.0 * d - root) / (6.0 * e);
            let t1 = (-2.0 * d + root) / (6.0 * e);
            if t0 > 0.0 && t0 < 1.0 { cuts[cut_count] = t0; cut_count += 1; }
            if t1 > 0.0 && t1 < 1.0 { cuts[cut_count] = t1; cut_count += 1; }
        }
    }
    // At most four cuts: insertion sort.
    for i in 1..cut_count {
        let t = cuts[i];
        let mut j = i;
        while j > 0 && cuts[j - 1] > t { cuts[j] = cuts[j - 1]; j -= 1; }
        cuts[j] = t;
    }
    let mut count = 0;
    for i in 0..cut_count {
        let t = cuts[i];
        if (((e * t + d) * t + b) * t + a).abs() < epsilon { out[count] = t; count += 1; }
    }
    for i in 1..cut_count {
        let mut lo = cuts[i - 1];
        let mut hi = cuts[i];
        let mut vl = ((e * lo + d) * lo + b) * lo + a;
        if vl * (((e * hi + d) * hi + b) * hi + a) >= 0.0 { continue; }
        // The cubic is monotonic between cuts: Newton steps, kept inside a
        // shrinking bracket by bisection, converge in a handful of evaluations.
        let mut t = (lo + hi) / 2.0;
        let mut j = 0;
        while j < 40 && hi - lo > 1e-13 {
            let vm = ((e * t + d) * t + b) * t + a;
            if vl * vm <= 0.0 { hi = t; } else { lo = t; vl = vm; }
            if vm == 0.0 { break; }
            let next = t - vm / ((3.0 * e * t + 2.0 * d) * t + b);
            t = if next > lo && next < hi { next } else { (lo + hi) / 2.0 };
            j += 1;
        }
        out[count] = t;
        count += 1;
    }
    for i in 1..count {
        let t = out[i];
        let mut j = i;
        while j > 0 && out[j - 1] > t { out[j] = out[j - 1]; j -= 1; }
        out[j] = t;
    }
    // Merge roots closer than 1e-8 to their sorted predecessor.
    let mut kept = 0;
    let mut previous = 0.0;
    for i in 0..count {
        let t = out[i];
        if i == 0 || t - previous > 1e-8 { out[kept] = t; kept += 1; }
        previous = t;
    }
    kept
}

/// Frictionless, zero-clearance unilateral projection. Recomputing the normal
/// and projected weights gives a distance gradient with zero net force/torque.
/// Negative weights are intentional for a feature that moved outside its face
/// after impact; only swept detection can create such a constraint.
///
/// Returns how idle the contact is: 0 if it acted (or could not), 1 if it is separated,
/// 2 if it is separated by more than COLD_GAP.
#[allow(clippy::too_many_arguments)]
#[inline]
pub(crate) fn project(x: &mut [f64], w: &[f64], i0: usize, i1: usize, i2: usize, i3: usize, vf: bool, sign: f64, bounded: bool) -> u8 {
    let mut x = W(x);
    let w = R(w);
    let (a, b, c, d) = (3 * i0, 3 * i1, 3 * i2, 3 * i3);
    let origin = if vf { b } else { c };
    let edge_origin = if vf { b } else { a };
    let edge_end = if vf { c } else { b };
    let (rx, ry, rz) = (x[a] - x[origin], x[a + 1] - x[origin + 1], x[a + 2] - x[origin + 2]);
    let (ex, ey, ez) = (x[edge_end] - x[edge_origin], x[edge_end + 1] - x[edge_origin + 1], x[edge_end + 2] - x[edge_origin + 2]);
    let (fx, fy, fz) = (x[d] - x[origin], x[d + 1] - x[origin + 1], x[d + 2] - x[origin + 2]);
    let (nx, ny, nz) = (ey * fz - ez * fy, ez * fx - ex * fz, ex * fy - ey * fx);
    let nn = nx * nx + ny * ny + nz * nz;
    let raw = rx * nx + ry * ny + rz * nz;
    if raw * sign >= 0.0 { return if raw * raw > COLD_GAP * COLD_GAP * nn { 2 } else { 1 }; }
    let ee = ex * ex + ey * ey + ez * ez;
    let ff = fx * fx + fy * fy + fz * fz;
    let ef = ex * fx + ey * fy + ez * fz;
    if nn < 1e-24 || nn < 1e-12 * ee * ff { return 0; }
    let re = rx * ex + ry * ey + rz * ez;
    let rf = rx * fx + ry * fy + rz * fz;
    let u = (if vf { re * ff - rf * ef } else { ef * rf - ff * re }) / nn;
    let v = (rf * ee - re * ef) / nn;
    // A contact kept from the previous substep saw no impact in this one: it
    // may only act where the features actually overlap.
    if bounded && !(u >= -BOUND && v >= -BOUND && (if vf { u + v <= 1.0 + BOUND } else { u <= 1.0 + BOUND && v <= 1.0 + BOUND })) { return 0; }
    let c0 = if vf { 1.0 } else { 1.0 - u };
    let c1 = if vf { u + v - 1.0 } else { u };
    let c2 = if vf { -u } else { v - 1.0 };
    let c3 = -v;
    let den = w[i0] * c0 * c0 + w[i1] * c1 * c1 + w[i2] * c2 * c2 + w[i3] * c3 * c3;
    if den <= 0.0 { return 0; }
    let impulse = -raw / (nn * den);
    let (s0, s1, s2, s3) = (w[i0] * c0 * impulse, w[i1] * c1 * impulse, w[i2] * c2 * impulse, w[i3] * c3 * impulse);
    x[a] += s0 * nx; x[a + 1] += s0 * ny; x[a + 2] += s0 * nz;
    x[b] += s1 * nx; x[b + 1] += s1 * ny; x[b + 2] += s1 * nz;
    x[c] += s2 * nx; x[c + 1] += s2 * ny; x[c + 2] += s2 * nz;
    x[d] += s3 * nx; x[d + 1] += s3 * ny; x[d + 2] += s3 * nz;
    0
}

/// Projected impact coordinates at time t of the step lie inside both features.
#[allow(clippy::too_many_arguments)]
#[inline]
fn inside(o: &[f64], p: &[f64], t: f64, i0: usize, i1: usize, i2: usize, i3: usize, vf: bool) -> bool {
    let (o, p) = (R(o), R(p));
    let (a, b, c, d) = (3 * i0, 3 * i1, 3 * i2, 3 * i3);
    let at = |k: usize| o[k] + t * (p[k] - o[k]);
    let (ax, ay, az) = (at(a), at(a + 1), at(a + 2));
    let (bx, by, bz) = (at(b), at(b + 1), at(b + 2));
    let (cx, cy, cz) = (at(c), at(c + 1), at(c + 2));
    let (dx, dy, dz) = (at(d), at(d + 1), at(d + 2));
    let (rx, ry, rz) = if vf { (ax - bx, ay - by, az - bz) } else { (ax - cx, ay - cy, az - cz) };
    let (ex, ey, ez) = if vf { (cx - bx, cy - by, cz - bz) } else { (bx - ax, by - ay, bz - az) };
    let (fx, fy, fz) = if vf { (dx - bx, dy - by, dz - bz) } else { (dx - cx, dy - cy, dz - cz) };
    let (nx, ny, nz) = (ey * fz - ez * fy, ez * fx - ex * fz, ex * fy - ey * fx);
    let ee = ex * ex + ey * ey + ez * ez;
    let ff = fx * fx + fy * fy + fz * fz;
    let ef = ex * fx + ey * fy + ez * fz;
    let den = nx * nx + ny * ny + nz * nz;
    // Degenerate triangles and almost parallel edges have no reliable normal.
    if den < 1e-24 || den < 1e-12 * ee * ff { return false; }
    let re = rx * ex + ry * ey + rz * ez;
    let rf = rx * fx + ry * fy + rz * fz;
    let u = if vf { (re * ff - rf * ef) / den } else { (ef * rf - ff * re) / den };
    let v = (rf * ee - re * ef) / den;
    u >= -PARAM_EPS && v >= -PARAM_EPS && (if vf { u + v <= 1.0 + PARAM_EPS } else { u <= 1.0 + PARAM_EPS && v <= 1.0 + PARAM_EPS })
}

/// Coplanarity cubic a + b t + c t² + d t³ of four linearly moving vertices between `o`
/// (t = 0) and `p` (t = 1), and the squared normal at t = 0. The vertices are given by
/// role: the cubic is r · (e × f) with r = ia − ib, e = ic − ie and f = jf − ib.
#[allow(clippy::too_many_arguments)]
#[inline(always)]
unsafe fn coplanarity<T: Lanes>(o: *const f64, p: *const f64, ia: T::Idx, ib: T::Idx, ic: T::Idx, ie: T::Idx, jf: T::Idx) -> (T, T, T, T, T) {
    let at = |s: *const f64, i: T::Idx, d: usize| T::get(s, i, 3, d);
    let (rx, ry, rz) = (at(o, ia, 0) - at(o, ib, 0), at(o, ia, 1) - at(o, ib, 1), at(o, ia, 2) - at(o, ib, 2));
    let (ex, ey, ez) = (at(o, ic, 0) - at(o, ie, 0), at(o, ic, 1) - at(o, ie, 1), at(o, ic, 2) - at(o, ie, 2));
    let (fx, fy, fz) = (at(o, jf, 0) - at(o, ib, 0), at(o, jf, 1) - at(o, ib, 1), at(o, jf, 2) - at(o, ib, 2));
    let (drx, dry, drz) = (at(p, ia, 0) - at(p, ib, 0) - rx, at(p, ia, 1) - at(p, ib, 1) - ry, at(p, ia, 2) - at(p, ib, 2) - rz);
    let (dex, dey, dez) = (at(p, ic, 0) - at(p, ie, 0) - ex, at(p, ic, 1) - at(p, ie, 1) - ey, at(p, ic, 2) - at(p, ie, 2) - ez);
    let (dfx, dfy, dfz) = (at(p, jf, 0) - at(p, ib, 0) - fx, at(p, jf, 1) - at(p, ib, 1) - fy, at(p, jf, 2) - at(p, ib, 2) - fz);
    let (nx, ny, nz) = (ey * fz - ez * fy, ez * fx - ex * fz, ex * fy - ey * fx);
    let mx = dey * fz - dez * fy + ey * dfz - ez * dfy;
    let my = dez * fx - dex * fz + ez * dfx - ex * dfz;
    let mz = dex * fy - dey * fx + ex * dfy - ey * dfx;
    let (qx, qy, qz) = (dey * dfz - dez * dfy, dez * dfx - dex * dfz, dex * dfy - dey * dfx);
    let a = rx * nx + ry * ny + rz * nz;
    let b = drx * nx + dry * ny + drz * nz + rx * mx + ry * my + rz * mz;
    let c = drx * mx + dry * my + drz * mz + rx * qx + ry * qy + rz * qz;
    let d = drx * qx + dry * qy + drz * qz;
    let nn = nx * nx + ny * ny + nz * nz;
    (a, b, c, d, nn)
}
/// Whether the cubic keeps one sign over the step (by its Bernstein convex hull: all four
/// control values on one side exclude a root), and whether the features touch at its start.
#[inline(always)]
fn clearance<T: Lanes>(a: T, b: T, c: T, d: T, nn: T) -> (T::Mask, T::Mask) {
    let (zero, three) = (T::splat(0.0), T::splat(3.0));
    let b1 = a + b / three;
    let b2 = a + T::splat(2.0) * b / three + c / three;
    let b3 = a + b + c + d;
    let same_sign = (a.gt(zero) & b1.gt(zero) & b2.gt(zero) & b3.gt(zero)) | (a.lt(zero) & b1.lt(zero) & b2.lt(zero) & b3.lt(zero));
    let touching = nn.gt(T::splat(1e-24)) & (a * a).le(T::splat(GAP_EPS * GAP_EPS) * nn);
    (same_sign, touching)
}
/// Whether two feature pairs both stay on one side without touching, which is all the
/// narrow phase needs to know about nearly every pair.
#[allow(clippy::too_many_arguments)]
#[inline(always)]
unsafe fn both_clear(o: *const f64, p: *const f64, ia: [usize; 2], ib: [usize; 2], ic: [usize; 2], ie: [usize; 2], jf: [usize; 2]) -> bool {
    let (a, b, c, d, nn) = coplanarity::<F2>(o, p, ia, ib, ic, ie, jf);
    let (same_sign, touching) = clearance::<F2>(a, b, c, d, nn);
    F2::all(same_sign & !touching)
}

/// Self-contact detection and projection state for one mesh.
pub struct SelfContact {
    /// Flat one-ring adjacency bitset: `words` 32-bit words per vertex. The broad phase
    /// tests bits instead of hashing vertex ids; a vertex is its own neighbour.
    neighbors: Vec<u32>,
    words: usize,
    edges: Vec<i32>,
    faces: Vec<i32>,
    face_tree: Option<Tree>,
    edge_tree: Option<Tree>,
    /// Dense per-pair state replaces hashing in the narrow phase. A pair key is
    /// vertex*faces+face, or vertices*faces+edge*edges+edge; one byte each.
    previous_sign: Vec<i8>,
    active_flag: Vec<u8>,
    /// Active contacts this substep.
    pub count: usize,
    /// Active contacts in detection order: four vertex ids, pair key, side, kind, whether
    /// carried from the previous substep, and the idle code of the last projection.
    ids: Vec<i32>,
    keys: Vec<i32>,
    signs: Vec<i8>,
    vertex_face: Vec<u8>,
    carried: Vec<u8>,
    idle: Vec<u8>,
    /// Pair keys of the previous substep's contacts.
    previous_keys: Vec<i32>,
    /// Swept bounds of each vertex in the mean-translating frame, and of each face and edge.
    low: Vec<f64>,
    high: Vec<f64>,
    face_boxes: Vec<f64>,
    edge_boxes: Vec<f64>,
    /// Each vertex's swept bounds padded by GAP_EPS: low xyz, then high xyz.
    vertex_boxes: Vec<f64>,
    /// Listed pairs whose swept bounds overlap this substep.
    candidates: Vec<i32>,
    /// Broad-phase pair lists, in BVH visit order, built from bounds padded by
    /// PAIR_MARGIN. They remain a superset of every overlapping pair until some
    /// vertex drifts that far from `reference` in the mean-translating frame.
    reference: Vec<f64>,
    pairs_valid: bool,
    vf_pairs: Vec<i32>,
    edge_pairs: Vec<i32>,
    /// Detection time split, accumulated while profiling: set-up, pair rebuilds, narrow phase.
    pub timing: [f64; 3],
    /// This substep's narrow-phase tests, and vertex-face and edge-edge contacts added.
    pub stats: [f64; 3],
}

impl SelfContact {
    pub fn new(vertices: usize, neighbors: Vec<u32>, faces: Vec<i32>, edges: Vec<i32>, face_tree: Option<Tree>, edge_tree: Option<Tree>) -> Self {
        let words = vertices.div_ceil(32);
        let face_count = faces.len() / 3;
        let edge_count = edges.len() / 2;
        let pairs = vertices * face_count + edge_count * edge_count;
        SelfContact {
            neighbors,
            words,
            edges,
            faces,
            face_tree,
            edge_tree,
            previous_sign: vec![0; pairs],
            active_flag: vec![0; pairs],
            count: 0,
            ids: Vec::new(),
            keys: Vec::new(),
            signs: Vec::new(),
            vertex_face: Vec::new(),
            carried: Vec::new(),
            idle: Vec::new(),
            previous_keys: Vec::new(),
            low: vec![0.0; vertices * 3],
            high: vec![0.0; vertices * 3],
            face_boxes: vec![0.0; face_count * 6],
            edge_boxes: vec![0.0; edge_count * 6],
            vertex_boxes: vec![0.0; vertices * 6],
            candidates: Vec::new(),
            reference: vec![0.0; vertices * 3],
            pairs_valid: false,
            vf_pairs: Vec::new(),
            edge_pairs: Vec::new(),
            timing: [0.0; 3],
            stats: [0.0; 3],
        }
    }
    /// Clear the per-pair state of the previous and the active contacts.
    fn forget(&mut self) {
        for &key in &self.previous_keys { self.previous_sign[key as usize] = 0; }
        for i in 0..self.count { self.active_flag[self.keys[i] as usize] = 0; }
    }
    pub fn reset(&mut self) {
        self.forget();
        self.pairs_valid = false;
        self.previous_keys.clear();
        self.clear_active();
        self.stats = [0.0; 3];
    }
    fn clear_active(&mut self) {
        self.count = 0;
        self.ids.clear();
        self.keys.clear();
        self.signs.clear();
        self.vertex_face.clear();
        self.carried.clear();
        self.idle.clear();
    }
    /// Make this substep's contacts the previous ones and start an empty active set.
    pub fn begin_step(&mut self) {
        self.forget();
        self.previous_keys.clear();
        for i in 0..self.count {
            let key = self.keys[i];
            self.previous_keys.push(key);
            self.previous_sign[key as usize] = self.signs[i];
        }
        self.clear_active();
        self.stats = [0.0; 3];
    }
    /// Active contacts as (four vertex ids, kind 1 = vertex-face, side, pair key), seven values each.
    pub fn contacts(&self) -> Vec<i32> {
        (0..self.count).flat_map(|i| {
            let ids = &self.ids[4 * i..4 * i + 4];
            [ids[0], ids[1], ids[2], ids[3], self.vertex_face[i] as i32, self.signs[i] as i32, self.keys[i]]
        }).collect()
    }
    /// Size of the cached broad-phase pair lists (zero before the first detection).
    pub fn pair_count(&self) -> usize {
        if self.pairs_valid { self.vf_pairs.len() / 2 + self.edge_pairs.len() / 2 } else { 0 }
    }
    #[allow(clippy::too_many_arguments)]
    fn add(&mut self, i0: usize, i1: usize, i2: usize, i3: usize, vf: bool, sign: i8, key: usize, carried: bool) {
        self.count += 1;
        self.ids.extend_from_slice(&[i0 as i32, i1 as i32, i2 as i32, i3 as i32]);
        self.keys.push(key as i32);
        self.signs.push(sign);
        self.vertex_face.push(vf as u8);
        self.carried.push(carried as u8);
        self.idle.push(0);
        self.active_flag[key] = 1;
        self.stats[if vf { 1 } else { 2 }] += 1.0;
    }
    /// Keep last substep's contacts that still touch inside their features at
    /// `old` (or hover within `gap` on the side they were pushed to), with their
    /// side, so every sweep of this substep enforces them. Fresh crossings are
    /// added by detection.
    pub fn carry(&mut self, old: &[f64], gap: f64) {
        let face_count = self.faces.len() / 3;
        let edge_count = self.edges.len() / 2;
        let vertex_face_keys = old.len() / 3 * face_count;
        for n in 0..self.previous_keys.len() {
            let key = self.previous_keys[n] as usize;
            let vf = key < vertex_face_keys;
            let (i0, i1, i2, i3);
            if vf {
                let fi = key % face_count;
                i0 = (key - fi) / face_count;
                i1 = self.faces[3 * fi] as usize;
                i2 = self.faces[3 * fi + 1] as usize;
                i3 = self.faces[3 * fi + 2] as usize;
            } else {
                let pair = key - vertex_face_keys;
                let j = pair % edge_count;
                let i = (pair - j) / edge_count;
                i0 = self.edges[2 * i] as usize;
                i1 = self.edges[2 * i + 1] as usize;
                i2 = self.edges[2 * j] as usize;
                i3 = self.edges[2 * j + 1] as usize;
            }
            let ia = 3 * i0;
            let ib = 3 * if vf { i1 } else { i2 };
            let ic = 3 * if vf { i2 } else { i1 };
            let ie = 3 * if vf { i1 } else { i0 };
            let jf = 3 * i3;
            let (rx, ry, rz) = (old[ia] - old[ib], old[ia + 1] - old[ib + 1], old[ia + 2] - old[ib + 2]);
            let (ex, ey, ez) = (old[ic] - old[ie], old[ic + 1] - old[ie + 1], old[ic + 2] - old[ie + 2]);
            let (fx, fy, fz) = (old[jf] - old[ib], old[jf + 1] - old[ib + 1], old[jf + 2] - old[ib + 2]);
            let (nx, ny, nz) = (ey * fz - ez * fy, ez * fx - ex * fz, ex * fy - ey * fx);
            let a = rx * nx + ry * ny + rz * nz;
            let nn = nx * nx + ny * ny + nz * nz;
            if !(nn > 1e-24) { continue; }
            // Touching, or still close on the side it was last pushed to.
            let sign = self.previous_sign[key];
            let near = a * a <= GAP_EPS * GAP_EPS * nn || (a * sign as f64 > 0.0 && a * a <= gap * gap * nn);
            if near && inside(old, old, 0.0, i0, i1, i2, i3, vf) { self.add(i0, i1, i2, i3, vf, sign, key, true); }
        }
    }

    /// Narrow phase for one feature pair: add a contact if the pair crosses during the
    /// step, or touches now and was in contact in the previous substep.
    #[allow(clippy::too_many_arguments)]
    fn test(&mut self, o_slice: &[f64], p_slice: &[f64], i0: usize, i1: usize, i2: usize, i3: usize, vf: bool, key: usize) {
        // Cubic coplanarity polynomial of the linearly moving features.
        let (r0, r1, r2, r3, r4) = if vf { (i0, i1, i2, i1, i3) } else { (i0, i2, i1, i0, i3) };
        let (a, b, c, d, nn) = unsafe { coplanarity::<f64>(o_slice.as_ptr(), p_slice.as_ptr(), r0, r1, r2, r3, r4) };
        // Nearly every pair stays on one side without touching: decide that before
        // reading the (cache-unfriendly) per-pair history.
        let (same_sign, touching) = clearance::<f64>(a, b, c, d, nn);
        if same_sign && !touching { return; }
        if R(&self.active_flag)[key] != 0 { return; }
        let prior = if touching { R(&self.previous_sign)[key] } else { 0 };
        if prior == 0 && touching { return; }
        let mut sign: f64 = 0.0;
        if prior != 0 && inside(o_slice, p_slice, 0.0, i0, i1, i2, i3, vf) { sign = prior as f64; } else if same_sign { return; }
        if sign == 0.0 {
            let mut roots = [0.0f64; 8];
            let count = cubic_roots(a, b, c, d, &mut roots);
            for &t in roots.iter().take(count) {
                // Perfect initial coincidence has no stack order. Let it open; do not
                // create arbitrary opposition at a folded seam or a grazing touch.
                if t < 1e-8 && (nn <= 1e-24 || touching) { continue; }
                if !inside(o_slice, p_slice, t, i0, i1, i2, i3, vf) { continue; }
                let before = js::max(0.0, t - 1e-6);
                let after = js::min(1.0, t + 1e-6);
                let value_before = ((d * before + c) * before + b) * before + a;
                let value_after = ((d * after + c) * after + b) * after + a;
                if value_before * value_after > 0.0 { continue; }
                sign = js::sign(value_before);
                if sign != 0.0 { break; }
            }
        }
        // A NaN side adds no contact either.
        if sign == 0.0 || sign.is_nan() { return; }
        self.add(i0, i1, i2, i3, vf, sign as i8, key, false);
    }

    /// Feature bounds pad once per member vertex, as the swept box always has.
    fn fill(low: &[f64], high: &[f64], ids: &[i32], size: usize, boxes: &mut [f64]) {
        let features = ids.len() / size;
        let (low, high, ids) = (R(low), R(high), R(ids));
        for f in 0..features {
            let (mut lx, mut ly, mut lz) = (f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let (mut hx, mut hy, mut hz) = (f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
            for j in 0..size {
                let k = 3 * ids[f * size + j] as usize;
                lx = js::min(lx, low[k]) - GAP_EPS;
                ly = js::min(ly, low[k + 1]) - GAP_EPS;
                lz = js::min(lz, low[k + 2]) - GAP_EPS;
                hx = js::max(hx, high[k]) + GAP_EPS;
                hy = js::max(hy, high[k + 1]) + GAP_EPS;
                hz = js::max(hz, high[k + 2]) + GAP_EPS;
            }
            let b = &mut boxes[6 * f..6 * f + 6];
            b[0] = lx; b[1] = ly; b[2] = lz; b[3] = hx; b[4] = hy; b[5] = hz;
        }
    }

    /// Swept detection between flat xyz positions before (`o`) and after (`p`) the step.
    pub fn detect(&mut self, o: &[f64], p: &[f64]) {
        let t0 = crate::profile_now();
        let count = o.len() / 3;
        let cf = count as f64;
        // A linearly translating frame preserves every impact time and prevents a
        // fast rigid launch from inflating every swept box by the flight distance.
        let (mut mx, mut my, mut mz) = (0.0, 0.0, 0.0);
        let mut k = 0;
        while k < 3 * count {
            mx += (p[k] - o[k]) / cf;
            my += (p[k + 1] - o[k + 1]) / cf;
            mz += (p[k + 2] - o[k + 2]) / cf;
            k += 3;
        }
        {
            let (low, high) = (&mut self.low, &mut self.high);
            let mut k = 0;
            while k < 3 * count {
                low[k] = js::min(o[k], p[k] - mx);
                low[k + 1] = js::min(o[k + 1], p[k + 1] - my);
                low[k + 2] = js::min(o[k + 2], p[k + 2] - mz);
                high[k] = js::max(o[k], p[k] - mx);
                high[k + 1] = js::max(o[k + 1], p[k + 1] - my);
                high[k + 2] = js::max(o[k + 2], p[k + 2] - mz);
                let b = &mut self.vertex_boxes[2 * k..2 * k + 6];
                b[0] = low[k] - GAP_EPS; b[1] = low[k + 1] - GAP_EPS; b[2] = low[k + 2] - GAP_EPS;
                b[3] = high[k] + GAP_EPS; b[4] = high[k + 1] + GAP_EPS; b[5] = high[k + 2] + GAP_EPS;
                k += 3;
            }
        }
        let face_count = self.faces.len() / 3;
        let edge_count = self.edges.len() / 2;
        Self::fill(&self.low, &self.high, &self.faces, 3, &mut self.face_boxes);
        Self::fill(&self.low, &self.high, &self.edges, 2, &mut self.edge_boxes);
        // Largest per-axis drift of either endpoint from the pair-list reference,
        // after removing the common translation.
        let mut stale = !self.pairs_valid;
        if !stale {
            let r = &self.reference;
            let (mut tx, mut ty, mut tz) = (0.0, 0.0, 0.0);
            let mut k = 0;
            while k < 3 * count {
                tx += (o[k] - r[k]) / cf;
                ty += (o[k + 1] - r[k + 1]) / cf;
                tz += (o[k + 2] - r[k + 2]) / cf;
                k += 3;
            }
            let (ux, uy, uz) = (tx + mx, ty + my, tz + mz);
            let mut k = 0;
            while k < 3 * count && !stale {
                stale = !((o[k] - r[k] - tx).abs() <= PAIR_MARGIN && (o[k + 1] - r[k + 1] - ty).abs() <= PAIR_MARGIN && (o[k + 2] - r[k + 2] - tz).abs() <= PAIR_MARGIN
                    && (p[k] - r[k] - ux).abs() <= PAIR_MARGIN && (p[k + 1] - r[k + 1] - uy).abs() <= PAIR_MARGIN && (p[k + 2] - r[k + 2] - uz).abs() <= PAIR_MARGIN);
                k += 3;
            }
        }
        let t1 = crate::profile_now();
        if stale { self.build_pairs(o); }
        let t2 = crate::profile_now();
        // The pair lists are a superset. Keep the pairs whose swept bounds overlap in this
        // substep, without branching on a test that passes about half the time; then test
        // them two at a time, since nearly all stay clear.
        let pair_count = self.vf_pairs.len() / 2;
        let edge_pair_count = self.edge_pairs.len() / 2;
        let mut candidates = std::mem::take(&mut self.candidates);
        if candidates.len() < pair_count.max(edge_pair_count) { candidates.resize(pair_count.max(edge_pair_count), 0); }
        let (op, pp) = (o.as_ptr(), p.as_ptr());
        let mut found = 0;
        {
            let (pairs, fb, vb) = (R(&self.vf_pairs), R(&self.face_boxes), R(&self.vertex_boxes));
            let mut list = W(&mut candidates);
            for n in 0..pair_count {
                let k = 6 * pairs[2 * n] as usize;
                let b = 6 * pairs[2 * n + 1] as usize;
                let overlap = (fb[b] <= vb[k + 3]) & (vb[k] <= fb[b + 3]) & (fb[b + 1] <= vb[k + 4]) & (vb[k + 1] <= fb[b + 4]) & (fb[b + 2] <= vb[k + 5]) & (vb[k + 2] <= fb[b + 5]);
                list[found] = n as i32;
                found += overlap as usize;
            }
        }
        self.stats[0] += found as f64;
        let vertex_face = |this: &Self, n: usize| {
            let (pairs, faces) = (R(&this.vf_pairs), R(&this.faces));
            let (v, fi) = (pairs[2 * n] as usize, pairs[2 * n + 1] as usize);
            (v, faces[3 * fi] as usize, faces[3 * fi + 1] as usize, faces[3 * fi + 2] as usize, v * face_count + fi)
        };
        let mut at = 0;
        while at < found {
            let (v, f0, f1, f2, key) = vertex_face(self, candidates[at] as usize);
            if at + 1 < found {
                let (v2, g0, g1, g2, key2) = vertex_face(self, candidates[at + 1] as usize);
                if !unsafe { both_clear(op, pp, [v, v2], [f0, g0], [f1, g1], [f0, g0], [f2, g2]) } {
                    self.test(o, p, v, f0, f1, f2, true, key);
                    self.test(o, p, v2, g0, g1, g2, true, key2);
                }
            } else {
                self.test(o, p, v, f0, f1, f2, true, key);
            }
            at += 2;
        }
        let base = count * face_count;
        let mut found = 0;
        {
            let (pairs, eb) = (R(&self.edge_pairs), R(&self.edge_boxes));
            let mut list = W(&mut candidates);
            for n in 0..edge_pair_count {
                let (a, b) = (6 * pairs[2 * n] as usize, 6 * pairs[2 * n + 1] as usize);
                let overlap = (eb[b] <= eb[a + 3]) & (eb[a] <= eb[b + 3]) & (eb[b + 1] <= eb[a + 4]) & (eb[a + 1] <= eb[b + 4]) & (eb[b + 2] <= eb[a + 5]) & (eb[a + 2] <= eb[b + 5]);
                list[found] = n as i32;
                found += overlap as usize;
            }
        }
        self.stats[0] += found as f64;
        let edge_edge = |this: &Self, n: usize| {
            let (pairs, edges) = (R(&this.edge_pairs), R(&this.edges));
            let (i, j) = (pairs[2 * n] as usize, pairs[2 * n + 1] as usize);
            (edges[2 * i] as usize, edges[2 * i + 1] as usize, edges[2 * j] as usize, edges[2 * j + 1] as usize, base + i * edge_count + j)
        };
        let mut at = 0;
        while at < found {
            let (e0, e1, e2, e3, key) = edge_edge(self, candidates[at] as usize);
            if at + 1 < found {
                let (g0, g1, g2, g3, key2) = edge_edge(self, candidates[at + 1] as usize);
                if !unsafe { both_clear(op, pp, [e0, g0], [e2, g2], [e1, g1], [e0, g0], [e3, g3]) } {
                    self.test(o, p, e0, e1, e2, e3, false, key);
                    self.test(o, p, g0, g1, g2, g3, false, key2);
                }
            } else {
                self.test(o, p, e0, e1, e2, e3, false, key);
            }
            at += 2;
        }
        self.candidates = candidates;
        if let (Some(t0), Some(t1), Some(t2), Some(t3)) = (t0, t1, t2, crate::profile_now()) {
            self.timing[0] += t1 - t0;
            self.timing[1] += t2 - t1;
            self.timing[2] += t3 - t2;
        }
    }

    /// Collect nonadjacent feature pairs whose padded swept bounds overlap.
    fn build_pairs(&mut self, o: &[f64]) {
        let count = self.low.len() / 3;
        let edge_count = self.edges.len() / 2;
        let words = self.words;
        let pad = 2.0 * PAIR_MARGIN;
        self.reference.copy_from_slice(o);
        self.pairs_valid = true;
        self.vf_pairs.clear();
        self.edge_pairs.clear();
        let (low, high, faces, edges, neighbors) = (R(&self.low), R(&self.high), R(&self.faces), R(&self.edges), R(&self.neighbors));
        let bit = |row: usize, v: i32| neighbors[row + (v as u32 >> 5) as usize] & (1u32 << (v & 31)) != 0;
        if let Some(tree) = self.face_tree.as_mut() {
            tree.refit(&self.face_boxes);
            let (bounds, leaves, escape) = (R(&tree.bounds), R(&tree.leaves), R(&tree.escape));
            let nodes = leaves.len();
            for v in 0..count {
                let k = 3 * v;
                let (lx, ly, lz) = (low[k] - GAP_EPS - pad, low[k + 1] - GAP_EPS - pad, low[k + 2] - GAP_EPS - pad);
                let (hx, hy, hz) = (high[k] + GAP_EPS + pad, high[k + 1] + GAP_EPS + pad, high[k + 2] + GAP_EPS + pad);
                let row = v * words;
                let mut node = 0;
                while node < nodes {
                    let b = node * 6;
                    if !(bounds[b] <= hx && lx <= bounds[b + 3] && bounds[b + 1] <= hy && ly <= bounds[b + 4] && bounds[b + 2] <= hz && lz <= bounds[b + 5]) {
                        node = escape[node] as usize;
                        continue;
                    }
                    let fi = leaves[node];
                    node += 1;
                    if fi < 0 { continue; }
                    let fi = fi as usize;
                    if bit(row, faces[3 * fi]) || bit(row, faces[3 * fi + 1]) || bit(row, faces[3 * fi + 2]) { continue; }
                    self.vf_pairs.push(v as i32);
                    self.vf_pairs.push(fi as i32);
                }
            }
        }
        if let Some(tree) = self.edge_tree.as_mut() {
            tree.refit(&self.edge_boxes);
            let boxes = R(&self.edge_boxes);
            let (bounds, leaves, escape) = (R(&tree.bounds), R(&tree.leaves), R(&tree.escape));
            let nodes = leaves.len();
            for i in 0..edge_count {
                let k = 6 * i;
                let (lx, ly, lz, hx, hy, hz) = (boxes[k] - pad, boxes[k + 1] - pad, boxes[k + 2] - pad, boxes[k + 3] + pad, boxes[k + 4] + pad, boxes[k + 5] + pad);
                let (row_a, row_b) = (edges[2 * i] as usize * words, edges[2 * i + 1] as usize * words);
                let mut node = 0;
                while node < nodes {
                    let n = node * 6;
                    if !(bounds[n] <= hx && lx <= bounds[n + 3] && bounds[n + 1] <= hy && ly <= bounds[n + 4] && bounds[n + 2] <= hz && lz <= bounds[n + 5]) {
                        node = escape[node] as usize;
                        continue;
                    }
                    let j = leaves[node];
                    node += 1;
                    if j <= i as i32 { continue; }
                    let j = j as usize;
                    let (c, d) = (edges[2 * j], edges[2 * j + 1]);
                    if bit(row_a, c) || bit(row_a, d) || bit(row_b, c) || bit(row_b, d) { continue; }
                    self.edge_pairs.push(i as i32);
                    self.edge_pairs.push(j as i32);
                }
            }
        }
    }

    /// One projection sweep. With a `sweep` index, separated contacts are revisited every
    /// other sweep, and clearly separated ones every eighth.
    pub fn solve(&mut self, x: &mut [f64], w: &[f64], forward: bool, sweep: usize) {
        let skip = if sweep % 8 == 0 { 3 } else if sweep % 2 == 0 { 2 } else { 1 };
        let count = self.count;
        let (ids, kinds, signs, carried) = (R(&self.ids), R(&self.vertex_face), R(&self.signs), R(&self.carried));
        let mut idle = W(&mut self.idle);
        for k in 0..count {
            let j = if forward { k } else { count - 1 - k };
            if idle[j] >= skip { continue; }
            idle[j] = project(x, w, ids[4 * j] as usize, ids[4 * j + 1] as usize, ids[4 * j + 2] as usize, ids[4 * j + 3] as usize, kinds[j] == 1, signs[j] as f64, carried[j] == 1);
        }
    }
}
