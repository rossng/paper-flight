//! Aerodynamic loads on the sheet. The default, coupled model is a lifting surface: layer
//! shielding by spatial hash, a planar vortex lattice over span stations and chordwise
//! panels, and the transfer of each patch's force to its face's vertices. The alternative
//! treats each triangle as a flat plate at its own incidence. Also the prescribed wind
//! field both sample.
//!
//! The surface itself (span stations, chordwise bins and the patches clipped from the
//! faces) is built in TypeScript (src/physics/lifting-surface.ts) and passed in as
//! `SurfaceSetup`. Aerodynamic resolution is independent of the structural triangulation.
use crate::js;
use crate::lanes::{Lanes, F2};
use core::arch::wasm32::{f64x2, f64x2_mul, f64x2_splat, f64x2_sub, v128, v128_load, v128_store};
use std::f64::consts::PI;

/// Thin-sheet profile drag, attached or edge-on separated.
const PROFILE_DRAG: f64 = 0.035;
/// Polhamus leading-edge suction analogy for sharp-edged, low-aspect-ratio sheets: the
/// separated leading-edge vortex adds a normal force Kv sin^2(alpha). Kv = pi is the
/// slender-wing value; its lift and drag components are Kv sin^2 cos and Kv |sin^3|.
const VORTEX_LIFT: f64 = PI;
const FOUR_PI: f64 = 4.0 * PI;
/// Buckets in the shielding spatial hash.
const HASH_SIZE: usize = 4096;
/// Values accumulated per lattice panel.
const F: usize = 6;

/// Smoothstep from 0 at `lo` to 1 at `hi`.
#[inline]
fn smooth(lo: f64, hi: f64, x: f64) -> f64 {
    let t = js::clamp((x - lo) / (hi - lo), 0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}
/// Upward-normal (planar z) velocity at (px,py) from a unit vortex segment a->b.
#[inline(always)]
fn segment_wash<T: Lanes>(px: T, py: T, ax: T, ay: T, bx: T, by: T) -> T {
    let (r1x, r1y, r2x, r2y) = (px - ax, py - ay, px - bx, py - by);
    let cz = r1x * r2y - r1y * r2x;
    let l1 = (r1x * r1x + r1y * r1y).sqrt();
    let l2 = (r2x * r2x + r2y * r2y).sqrt();
    let (r0x, r0y) = (bx - ax, by - ay);
    // Collinear points (including a segment's own extension) receive no normal velocity.
    let collinear = cz.abs().lt(T::splat(1e-12) * (l1 * l2 + T::splat(1e-30)));
    T::select(collinear, T::splat(0.0), ((r0x * r1x + r0y * r1y) / l1 - (r0x * r2x + r0y * r2y) / l2) / (T::splat(FOUR_PI) * cz))
}
/// Upward-normal velocity at (px,py) from a unit semi-infinite vortex from (ax,ay)
/// downstream along -x.
#[inline(always)]
fn trailing_wash<T: Lanes>(px: T, py: T, ax: T, ay: T) -> T {
    let (r1x, r1y) = (px - ax, py - ay);
    let cz = -r1y;
    let l1 = (r1x * r1x + r1y * r1y).sqrt();
    let collinear = cz.abs().lt(T::splat(1e-12) * (l1 + T::splat(1e-30)));
    T::select(collinear, T::splat(0.0), (T::splat(1.0) - r1x / l1) / (T::splat(FOUR_PI) * cz))
}
/// Downwash at (px,py) from a unit horseshoe vortex: a leg in from downstream to (bx,ya),
/// the bound segment to (bx,yb), and a leg out from there downstream.
#[inline(always)]
fn horseshoe_wash<T: Lanes>(px: T, py: T, bx: T, ya: T, yb: T) -> T {
    segment_wash(px, py, bx, ya, bx, yb) + trailing_wash(px, py, bx, yb) - trailing_wash(px, py, bx, ya)
}
#[inline]
fn cell_hash(x: i32, y: i32, z: i32) -> usize {
    ((x.wrapping_mul(73856093) ^ y.wrapping_mul(19349663) ^ z.wrapping_mul(83492791)) & (HASH_SIZE as i32 - 1)) as usize
}

/// Deterministic, smooth synthetic air field: replay and flight sample the same
/// space/time function. This is prescribed wind, not a solved fluid or a wake.
pub struct Weather {
    pub wind: [f64; 3],
    pub wind_heading: f64,
    pub gusts: f64,
    pub turbulence: f64,
    /// A fun air feature fixed in the field: 0 none, 1 a thermal, 2 a tornado, 3 a fan.
    pub feature: u8,
}
impl Weather {
    /// Air velocity of the feature at a position, in field metres from the launcher.
    fn feature_velocity(&self, [x, y, z]: [f64; 3]) -> [f64; 3] {
        match self.feature {
            1 => [0.0, 3.5 * (-((x - 5.0) * (x - 5.0) + z * z) / (2.0 * 1.5 * 1.5)).exp(), 0.0],
            2 => {
                let (dx, dz) = (x - 6.0, z - 1.0);
                let r = js::hypot2(dx, dz) + 1e-6;
                let v = 16.0 * r / (1.0 + r * r);
                [-dz / r * v, 2.5 * (-r * r / 4.0).exp(), dx / r * v]
            }
            3 => [0.0, 0.0, 6.0 * (-((y - 1.6) * (y - 1.6)) / 0.5).exp() / (1.0 + 0.3 * js::hypot2(x, z + 2.0))],
            _ => [0.0; 3],
        }
    }
    /// Air velocity at `position` and `time`: the steady wind, a gust along its direction
    /// (or along `wind_heading`, in degrees, in still air), turbulence and the air feature.
    pub fn air_velocity(&self, time: f64, position: [f64; 3]) -> [f64; 3] {
        let speed = js::hypot3(self.wind[0], self.wind[1], self.wind[2]);
        let direction = if speed > 1e-9 {
            [self.wind[0] / speed, self.wind[1] / speed, self.wind[2] / speed]
        } else {
            let h = self.wind_heading * PI / 180.0;
            [h.cos(), 0.0, h.sin()]
        };
        let base = 0.5 + 0.5 * (time * 2.1 - 0.8).sin();
        let gust = self.gusts * base.powf(3.0);
        let [x, y, z] = position;
        // Each component is independent of its own coordinate: the imposed swirl
        // field has zero divergence, and varies across panels as well as over time.
        let t = self.turbulence;
        let air = [
            self.wind[0] + direction[0] * gust + t * 0.45 * ((1.7 * z + 2.9 * time).sin() + 0.5 * (4.3 * y - 5.1 * time).sin()),
            self.wind[1] + direction[1] * gust + t * 0.35 * ((2.3 * x - 3.1 * time + 0.7).sin() + 0.5 * (5.4 * z + 6.7 * time).sin()),
            self.wind[2] + direction[2] * gust + t * 0.55 * ((2.1 * y + 2.5 * time + 1.2).sin() + 0.5 * (4.1 * x - 4.7 * time).sin()),
        ];
        if self.feature == 0 { return air; }
        let f = self.feature_velocity(position);
        [f[0] + air[0], f[1] + air[1], f[2] + air[2]]
    }
    /// Whether the air velocity depends on time or position at all.
    pub fn varies(&self) -> bool { self.gusts != 0.0 || self.turbulence != 0.0 || self.feature != 0 }
}

/// The lifting surface, clipped from the mesh in material/rest span coordinates.
pub struct SurfaceSetup {
    pub faces: Vec<i32>,
    /// Material panel of each face, when the fold stacks layers that shield one another.
    pub owners: Option<Vec<i32>>,
    /// Orientation of each face's normal: −1 where the rest normal points down.
    pub signs: Vec<f64>,
    /// Span station boundaries in rest span coordinates, and the rest span.
    pub boundaries: Vec<f64>,
    pub span: f64,
    /// Chordwise bins per span station.
    pub chord_panels: usize,
    /// One patch per clipped polygon, at its area centroid: velocity is linear over a face
    /// and the normal constant. Per patch: its face, its span station, its lattice panel
    /// (station*chord_panels + chordwise bin, 0 = leading), the barycentric weights of its
    /// centroid, and its fraction of the face's area.
    pub patch_face: Vec<i32>,
    pub patch_station: Vec<i32>,
    pub patch_panel: Vec<i32>,
    pub patch_weights: Vec<f64>,
    pub patch_fraction: Vec<f64>,
    /// Nose, tail, left root and right root vertices.
    pub landmarks: [usize; 4],
    /// Distance (metres) within which a near-coplanar layer shields a patch.
    pub shielding: f64,
}

/// The lifting-surface model and its scratch storage.
pub struct Aero {
    s: SurfaceSetup,
    /// Spatial-hash storage: bucket heads, and chained (face, next) entries.
    hash_head: Vec<i32>,
    hash_face: Vec<i32>,
    hash_next: Vec<i32>,
    /// Per face: a, ab, ac, unit normal, |ab|^2, |ac|^2, ab.ac, Gram determinant.
    face_geometry: Vec<f64>,
    /// Faces covering too many cells to hash; tested against every query.
    overflow: Vec<i32>,
    /// Material panels found at the current query point.
    owners: Vec<i32>,
    /// Per face: area and oriented unit normal.
    normal: Vec<f64>,
    /// Per lattice panel, weighted by planform (exposed area projected on the lifting plane):
    /// area, speed, attached incidence, attached area, body x; then attached exposed area.
    strips: Vec<f64>,
    /// Per patch: direction, area, speed, incidence, attachment.
    samples: Vec<f64>,
    boundaries: Vec<f64>,
    chords: Vec<f64>,
    lead: Vec<f64>,
    speeds: Vec<f64>,
    angles: Vec<f64>,
    attachment: Vec<f64>,
    matrix: Vec<f64>,
    circulation: Vec<f64>,
    strip_circulation: Vec<f64>,
    downwash: Vec<f64>,
    /// Exposed fraction of each patch, and whether it is current. Layer shielding changes
    /// slowly; a caller that samples loads several times per frame may hold one frame's
    /// exposure by clearing `exposure_valid` only once per frame.
    pub exposure: Vec<f64>,
    pub exposure_valid: bool,
    /// Loads of the last evaluation, PANEL values each: centre, force, lift, incidence, area, stalled.
    pub panels: Vec<f64>,
}
/// Values per panel load.
pub const PANEL: usize = 12;

impl Aero {
    pub fn new(s: SurfaceSetup) -> Self {
        let n = s.boundaries.len() - 1;
        let lattice = n * s.chord_panels;
        let faces = s.faces.len() / 3;
        let patches = s.patch_face.len();
        Aero {
            hash_head: vec![-1; HASH_SIZE],
            hash_face: vec![0; 4096],
            hash_next: vec![0; 4096],
            face_geometry: vec![0.0; faces * 16],
            overflow: Vec::new(),
            owners: Vec::new(),
            normal: vec![0.0; faces * 4],
            strips: vec![0.0; lattice * F],
            samples: vec![0.0; patches * 7],
            boundaries: vec![0.0; n + 1],
            chords: vec![0.0; lattice],
            lead: vec![0.0; lattice],
            speeds: vec![0.0; lattice],
            angles: vec![0.0; lattice],
            attachment: vec![0.0; lattice],
            matrix: vec![0.0; lattice * (lattice + 1)],
            circulation: vec![0.0; lattice],
            strip_circulation: vec![0.0; n],
            downwash: vec![0.0; n],
            exposure: vec![0.0; patches],
            exposure_valid: false,
            panels: Vec::new(),
            s,
        }
    }

    /// Build the shielding hash over the current face geometry.
    ///
    /// Dynamic near-coplanar shielding; count material panels, not triangulation faces.
    /// A small spatial hash avoids an all-pairs query. No rest-state exposure cache.
    /// Hash collisions only add candidates to the exact geometric test in `shield`.
    ///
    /// Deformed face geometry is invariant across this evaluation's patch queries,
    /// not across substeps: peeling/sliding and replay always rebuild this cache.
    fn build_exposure(&mut self, p: &[f64]) {
        let cell = js::max(0.008, self.s.span / 16.0);
        let gap = self.s.shielding;
        let faces = &self.s.faces;
        let g = &mut self.face_geometry;
        self.hash_head.fill(-1);
        self.overflow.clear();
        let mut entries = 0usize;
        for fi in 0..faces.len() / 3 {
            let (ia, ib, ic) = (3 * faces[3 * fi] as usize, 3 * faces[3 * fi + 1] as usize, 3 * faces[3 * fi + 2] as usize);
            let a = [p[ia], p[ia + 1], p[ia + 2]];
            let b = [p[ib], p[ib + 1], p[ib + 2]];
            let c = [p[ic], p[ic + 1], p[ic + 2]];
            let o = 16 * fi;
            let (abx, aby, abz) = (b[0] - a[0], b[1] - a[1], b[2] - a[2]);
            let (acx, acy, acz) = (c[0] - a[0], c[1] - a[1], c[2] - a[2]);
            let (nx, ny, nz) = (aby * acz - abz * acy, abz * acx - abx * acz, abx * acy - aby * acx);
            let inverse = 1.0 / js::max(js::hypot3(nx, ny, nz), 1e-12);
            let aa = abx * abx + aby * aby + abz * abz;
            let bb = acx * acx + acy * acy + acz * acz;
            let cc = abx * acx + aby * acy + abz * acz;
            let gg = &mut g[o..o + 16];
            gg[0] = a[0]; gg[1] = a[1]; gg[2] = a[2]; gg[3] = abx; gg[4] = aby; gg[5] = abz; gg[6] = acx; gg[7] = acy; gg[8] = acz;
            gg[9] = nx * inverse; gg[10] = ny * inverse; gg[11] = nz * inverse; gg[12] = aa; gg[13] = bb; gg[14] = cc; gg[15] = aa * bb - cc * cc;
            let min3 = |u: f64, v: f64, w: f64| js::min(js::min(u, v), w);
            let max3 = |u: f64, v: f64, w: f64| js::max(js::max(u, v), w);
            let lx = ((min3(a[0], b[0], c[0]) - gap) / cell).floor();
            let hx = ((max3(a[0], b[0], c[0]) + gap) / cell).floor();
            let ly = ((min3(a[1], b[1], c[1]) - gap) / cell).floor();
            let hy = ((max3(a[1], b[1], c[1]) + gap) / cell).floor();
            let lz = ((min3(a[2], b[2], c[2]) - gap) / cell).floor();
            let hz = ((max3(a[2], b[2], c[2]) + gap) / cell).floor();
            let cells = (hx - lx + 1.0) * (hy - ly + 1.0) * (hz - lz + 1.0);
            if !(cells <= 512.0) { self.overflow.push(fi as i32); continue; }
            let cells = cells as usize;
            if entries + cells > self.hash_face.len() {
                let size = (self.hash_face.len() * 2).max(entries + cells);
                self.hash_face.resize(size, 0);
                self.hash_next.resize(size, 0);
            }
            let (lx, hx, ly, hy, lz, hz) = (lx as i64, hx as i64, ly as i64, hy as i64, lz as i64, hz as i64);
            for x in lx..=hx {
                for y in ly..=hy {
                    for z in lz..=hz {
                        let h = cell_hash(js::to_int32(x as f64), js::to_int32(y as f64), js::to_int32(z as f64));
                        self.hash_face[entries] = fi as i32;
                        self.hash_next[entries] = self.hash_head[h];
                        self.hash_head[h] = entries as i32;
                        entries += 1;
                    }
                }
            }
        }
    }

    /// Material panel of a face; without owners every face is its own panel.
    #[inline]
    fn owner(&self, face: usize) -> i32 {
        match &self.s.owners { Some(o) => o[face], None => face as i32 }
    }

    /// Exposed fraction at `point` on face `fi`: 1 / (number of distinct material panels
    /// there).
    fn exposure_at(&mut self, point: [f64; 3], fi: usize) -> f64 {
        let cell = js::max(0.008, self.s.span / 16.0);
        let gap = self.s.shielding;
        self.owners.clear();
        let own_owner = self.owner(fi);
        self.owners.push(own_owner);
        let own = 16 * fi;
        let g = &self.face_geometry;
        let (ox, oy, oz) = (g[own + 9], g[own + 10], g[own + 11]);
        let (px0, py0, pz0) = (point[0], point[1], point[2]);
        let h = cell_hash(js::to_int32((px0 / cell).floor()), js::to_int32((py0 / cell).floor()), js::to_int32((pz0 / cell).floor()));
        let mut e = self.hash_head[h];
        while e >= 0 {
            let j = self.hash_face[e as usize] as usize;
            self.shield(j, px0, py0, pz0, ox, oy, oz, gap);
            e = self.hash_next[e as usize];
        }
        for k in 0..self.overflow.len() {
            let j = self.overflow[k] as usize;
            self.shield(j, px0, py0, pz0, ox, oy, oz, gap);
        }
        1.0 / self.owners.len() as f64
    }
    /// Count face `j`'s material panel if it is new, near-parallel to the query face, within
    /// the shielding distance of the point, and the point projects inside it.
    #[allow(clippy::too_many_arguments)]
    #[inline]
    fn shield(&mut self, j: usize, px0: f64, py0: f64, pz0: f64, ox: f64, oy: f64, oz: f64, gap: f64) {
        let other = self.owner(j);
        let o = 16 * j;
        let g = &self.face_geometry;
        if self.owners.contains(&other) || (ox * g[o + 9] + oy * g[o + 10] + oz * g[o + 11]).abs() < 0.98 { return; }
        let (px, py, pz) = (px0 - g[o], py0 - g[o + 1], pz0 - g[o + 2]);
        let den = g[o + 15];
        if (px * g[o + 9] + py * g[o + 10] + pz * g[o + 11]).abs() > gap || den < 1e-20 { return; }
        let dab = px * g[o + 3] + py * g[o + 4] + pz * g[o + 5];
        let dac = px * g[o + 6] + py * g[o + 7] + pz * g[o + 8];
        let u = (dab * g[o + 13] - dac * g[o + 14]) / den;
        let v = (dac * g[o + 12] - dab * g[o + 14]) / den;
        if u >= -1e-8 && v >= -1e-8 && u + v <= 1.0 + 1e-8 { self.owners.push(other); }
    }

    /// Loads of the lifting surface at the current positions and velocities: adds each
    /// patch's load to `out` (flat xyz, per vertex) and records it in `panels`.
    #[allow(clippy::too_many_arguments)]
    pub fn loads(&mut self, positions: &[f64], velocities: &[f64], weather: &Weather, time: f64, enabled: bool, density: f64, out: &mut [f64]) {
        let n = self.s.boundaries.len() - 1;
        let [nose, tail, left, right] = self.s.landmarks;
        let pv = |i: usize| [positions[3 * i], positions[3 * i + 1], positions[3 * i + 2]];
        let unit = |a: [f64; 3]| {
            let s = 1.0 / js::max(js::hypot3(a[0], a[1], a[2]), 1e-12);
            [a[0] * s, a[1] * s, a[2] * s]
        };
        let (pn, pt, pl, pr) = (pv(nose), pv(tail), pv(left), pv(right));
        let forward = unit([pn[0] - pt[0], pn[1] - pt[1], pn[2] - pt[2]]);
        let span_vector = [pr[0] - pl[0], pr[1] - pl[1], pr[2] - pl[2]];
        let along = span_vector[0] * forward[0] + span_vector[1] * forward[1] + span_vector[2] * forward[2];
        let span_axis = unit([span_vector[0] - forward[0] * along, span_vector[1] - forward[1] * along, span_vector[2] - forward[2] * along]);
        let tail_p = pt;
        let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
        let count = positions.len() / 3;
        for i in 0..count {
            let p = pv(i);
            let c = (p[0] - tail_p[0]) * span_axis[0] + (p[1] - tail_p[1]) * span_axis[1] + (p[2] - tail_p[2]) * span_axis[2];
            if c < low { low = c; }
            if c > high { high = c; }
        }
        let span_scale = js::max(0.1, (high - low) / self.s.span);
        let up = unit([
            span_vector[1] * forward[2] - span_vector[2] * forward[1],
            span_vector[2] * forward[0] - span_vector[0] * forward[2],
            span_vector[0] * forward[1] - span_vector[1] * forward[0],
        ]);
        for i in 0..=n { self.boundaries[i] = self.s.boundaries[i] * span_scale; }
        // Layer shielding changes slowly: reuse the held exposure while it is valid.
        let reuse = self.exposure_valid;
        let shielding = self.s.owners.is_some() && !reuse;
        if !reuse { self.exposure_valid = true; }
        if shielding { self.build_exposure(positions); }
        let faces_len = self.s.faces.len() / 3;
        for fi in 0..faces_len {
            let f = &self.s.faces[3 * fi..3 * fi + 3];
            let (a, b, c) = (pv(f[0] as usize), pv(f[1] as usize), pv(f[2] as usize));
            let (ux, uy, uz, vx, vy, vz) = (b[0] - a[0], b[1] - a[1], b[2] - a[2], c[0] - a[0], c[1] - a[1], c[2] - a[2]);
            let (nx, ny, nz) = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
            let len = js::hypot3(nx, ny, nz);
            let inverse = self.s.signs[fi] / js::max(len, 1e-12);
            let g = &mut self.normal[4 * fi..4 * fi + 4];
            g[0] = len / 2.0; g[1] = nx * inverse; g[2] = ny * inverse; g[3] = nz * inverse;
        }
        // A vertical keel projects to nothing on the lifting plane, so it cannot inflate the
        // planar lattice.
        let m = self.s.chord_panels;
        let lattice = n * m;
        self.strips[..lattice * F].fill(0.0);
        let patches = self.s.patch_face.len();
        let fixed_wind = weather.wind;
        let varies = weather.varies();
        self.panels.clear();
        self.panels.resize(PANEL * patches, 0.0);
        for k in 0..patches {
            let pf = self.s.patch_face[k] as usize;
            let (w0, w1, w2) = (self.s.patch_weights[3 * k], self.s.patch_weights[3 * k + 1], self.s.patch_weights[3 * k + 2]);
            let g = 4 * pf;
            let (f0, f1, f2) = (self.s.faces[3 * pf] as usize, self.s.faces[3 * pf + 1] as usize, self.s.faces[3 * pf + 2] as usize);
            let (pa, pb, pc) = (pv(f0), pv(f1), pv(f2));
            let (va, vb, vc) = (3 * f0, 3 * f1, 3 * f2);
            let center = [pa[0] * w0 + pb[0] * w1 + pc[0] * w2, pa[1] * w0 + pb[1] * w1 + pc[1] * w2, pa[2] * w0 + pb[2] * w1 + pc[2] * w2];
            let wind = if varies { weather.air_velocity(time, center) } else { fixed_wind };
            let rx = velocities[va] * w0 + velocities[vb] * w1 + velocities[vc] * w2 - wind[0];
            let ry = velocities[va + 1] * w0 + velocities[vb + 1] * w1 + velocities[vc + 1] * w2 - wind[1];
            let rz = velocities[va + 2] * w0 + velocities[vb + 2] * w1 + velocities[vc + 2] * w2 - wind[2];
            let speed = js::hypot3(rx, ry, rz);
            let inverse = 1.0 / js::max(speed, 1e-12);
            let (dx, dy, dz) = (rx * inverse, ry * inverse, rz * inverse);
            let (nx, ny, nz) = (self.normal[g + 1], self.normal[g + 2], self.normal[g + 3]);
            let alpha = js::clamp(-(nx * dx + ny * dy + nz * dz), -1.0, 1.0).asin();
            let exposed = if reuse { self.exposure[k] } else if shielding { self.exposure_at(center, pf) } else { 1.0 };
            if !reuse { self.exposure[k] = exposed; }
            let area = self.normal[g] * self.s.patch_fraction[k] * exposed;
            // Thin-wing assumptions fade out for reversed flow, side-slip, steep panels and stall.
            let attached = (1.0 - smooth(0.21, 0.49, alpha.abs())) * smooth(0.25, 0.75, dx * forward[0] + dy * forward[1] + dz * forward[2]) * smooth(0.2, 0.7, (nx * up[0] + ny * up[1] + nz * up[2]).abs());
            let x = (center[0] - tail_p[0]) * forward[0] + (center[1] - tail_p[1]) * forward[1] + (center[2] - tail_p[2]) * forward[2];
            let plan = area * (nx * up[0] + ny * up[1] + nz * up[2]).abs();
            let strip = F * self.s.patch_panel[k] as usize;
            let st = &mut self.strips[strip..strip + F];
            st[0] += plan; st[1] += plan * speed; st[2] += plan * attached * alpha; st[3] += plan * attached; st[4] += plan * x; st[5] += area * attached;
            let o = 7 * k;
            let sm = &mut self.samples[o..o + 7];
            sm[0] = dx; sm[1] = dy; sm[2] = dz; sm[3] = area; sm[4] = speed; sm[5] = alpha; sm[6] = attached;
            let panel = &mut self.panels[PANEL * k..PANEL * k + PANEL];
            panel[0] = center[0]; panel[1] = center[1]; panel[2] = center[2];
            panel[9] = alpha; panel[10] = area; panel[11] = if attached < 0.5 { 1.0 } else { 0.0 };
        }
        // Equivalent panel chord is planform area over strip width (as in lifting-line strip
        // theory). Bins are laid end to end, leading bin first, centred on the strip's planform
        // centroid along the body axis. Contiguous panels keep every tangency point a quarter
        // chord clear of the neighbouring bound vortex, however the sheet deforms.
        let boundaries = &self.boundaries;
        for i in 0..n {
            let width = js::max(boundaries[i + 1] - boundaries[i], 1e-8);
            let (mut area, mut moment) = (0.0, 0.0);
            for p in i * m..(i + 1) * m { area += self.strips[F * p]; moment += self.strips[F * p + 4]; }
            let mut edge = moment / js::max(area, 1e-12) + area / width / 2.0;
            for p in i * m..(i + 1) * m {
                let a = self.strips[F * p];
                let attached = self.strips[F * p + 3];
                self.chords[p] = a / width;
                self.lead[p] = edge;
                edge -= self.chords[p];
                self.speeds[p] = self.strips[F * p + 1] / js::max(a, 1e-12);
                self.angles[p] = self.strips[F * p + 2] / js::max(attached, 1e-12);
                self.attachment[p] = attached / js::max(a, 1e-12);
            }
        }
        self.vortex_lattice(n, m);
        let boundaries = &self.boundaries;
        // Downwash (positive down) at strip centres from straight trailing legs of strip-total
        // circulation: the lifting-line induced angle used for induced drag.
        self.strip_circulation.fill(0.0);
        for p in 0..lattice { self.strip_circulation[p / m] += self.circulation[p]; }
        for i in 0..n {
            let y = (boundaries[i] + boundaries[i + 1]) / 2.0;
            let mut w = 0.0;
            for j in 0..n { w += self.strip_circulation[j] * (1.0 / (y - boundaries[j]) - 1.0 / (y - boundaries[j + 1])) / FOUR_PI; }
            self.downwash[i] = w;
        }
        for k in 0..patches {
            let i = self.s.patch_station[k] as usize;
            let p = self.s.patch_panel[k] as usize;
            let o = 7 * k;
            let pf = self.s.patch_face[k] as usize;
            let g = 4 * pf;
            let sm = &self.samples[o..o + 7];
            let (dx, dy, dz, area, speed, alpha, blend) = (sm[0], sm[1], sm[2], sm[3], sm[4], sm[5], sm[6]);
            let (nx, ny, nz) = (self.normal[g + 1], self.normal[g + 2], self.normal[g + 3]);
            // Kutta-Joukowski panel lift rho V Gamma dy, shared over the panel's attached exposed
            // area.
            let cl_coupled = 2.0 * self.circulation[p] * (boundaries[i + 1] - boundaries[i]) / js::max(self.speeds[p] * self.strips[F * p + 5], 1e-12);
            // Lattice fixes each panel's mean; local incidence retains sub-panel variations.
            let cl_attached = blend * cl_coupled + blend * 2.0 * PI * (alpha - self.angles[p]);
            let sine = alpha.sin();
            let vortex = blend * VORTEX_LIFT * sine * sine.abs();
            let bounded = js::clamp(cl_attached, -1.5, 1.5);
            let cl = bounded + vortex * alpha.cos() + (1.0 - blend) * 0.85 * (2.0 * alpha).sin();
            // Separated sheets keep the attached profile-drag floor; edge-on panels only add skin
            // friction.
            let cd = PROFILE_DRAG + (vortex * sine).abs() + (1.0 - blend) * 1.75 * sine * sine + js::max(0.0, bounded * self.downwash[i] / js::max(speed, 1e-8));
            let along = nx * dx + ny * dy + nz * dz;
            let (lx, ly, lz) = (nx - dx * along, ny - dy * along, nz - dz * along);
            let lift_scale = 1.0 / js::max(js::hypot3(lx, ly, lz), 1e-12);
            let q_a = if enabled { 0.5 * density * speed * speed * area } else { 0.0 };
            let lifting = q_a * cl * lift_scale;
            let drag = q_a * cd;
            let (lfx, lfy, lfz) = (lx * lifting, ly * lifting, lz * lifting);
            let force = [lfx - dx * drag, lfy - dy * drag, lfz - dz * drag];
            let panel = &mut self.panels[PANEL * k..PANEL * k + PANEL];
            panel[3] = force[0]; panel[4] = force[1]; panel[5] = force[2];
            panel[6] = lfx; panel[7] = lfy; panel[8] = lfz;
            // Partition of unity and linear precision preserve both force and moment exactly.
            for j in 0..3 {
                let target = 3 * self.s.faces[3 * pf + j] as usize;
                let weight = self.s.patch_weights[3 * k + j];
                out[target] += weight * force[0];
                out[target + 1] += weight * force[1];
                out[target + 2] += weight * force[2];
            }
        }
    }

    fn vortex_lattice(&mut self, stations: usize, m: usize) {
        solve_vortex_lattice(&self.boundaries, stations, m, &self.lead, &self.chords, &self.speeds, &self.angles, &self.attachment, &mut self.matrix, &mut self.circulation);
    }
}

/// Planar vortex lattice. Panel p = station*m + bin spans [boundaries[i], boundaries[i+1]]
/// and chordwise [lead-chord, lead] in body x (forward positive). A horseshoe has its bound
/// segment at the quarter chord and straight trailing legs to downstream infinity; the
/// flow-tangency point is at the three-quarter chord. Positive circulation lifts along +up
/// and induces downwash. Attachment a blends each row from tangency (a=1) to zero
/// circulation (a=0), as the lifting-line form scales its section slope. `matrix` is
/// scratch of n*(n+1); `circulation` receives n values, all zero if the system is
/// singular or the solution is not finite.
#[allow(clippy::too_many_arguments)]
pub fn solve_vortex_lattice(boundaries: &[f64], stations: usize, m: usize, lead: &[f64], chord: &[f64], speeds: &[f64], incidence: &[f64], attachment: &[f64], matrix: &mut [f64], circulation: &mut [f64]) {
    let n = stations * m;
    let width = n + 1;
    matrix.fill(0.0);
    circulation.fill(0.0);
    let active = |p: usize| chord[p] > 1e-9 && attachment[p] > 1e-9;
    for p in 0..n {
        let row = p * width;
        if !active(p) { matrix[row + p] = 1.0; continue; }
        let i = p / m;
        let px = lead[p] - 0.75 * chord[p];
        let py = (boundaries[i] + boundaries[i + 1]) / 2.0;
        // Two columns at a time; an inactive panel's column stays zero.
        let mut q = 0;
        while q + 1 < n {
            let (j, k) = (q / m, (q + 1) / m);
            let bx = F2(f64x2(lead[q] - 0.25 * chord[q], lead[q + 1] - 0.25 * chord[q + 1]));
            let wash = F2::splat(attachment[p]) * horseshoe_wash(F2::splat(px), F2::splat(py), bx, F2(f64x2(boundaries[j], boundaries[k])), F2(f64x2(boundaries[j + 1], boundaries[k + 1])));
            if active(q) { matrix[row + q] = wash.lane(0); }
            if active(q + 1) { matrix[row + q + 1] = wash.lane(1); }
            q += 2;
        }
        if q < n && active(q) {
            let j = q / m;
            matrix[row + q] = attachment[p] * horseshoe_wash(px, py, lead[q] - 0.25 * chord[q], boundaries[j], boundaries[j + 1]);
        }
        matrix[row + p] += (1.0 - attachment[p]) * matrix[row + p] / attachment[p];
        matrix[row + n] = attachment[p] * speeds[p] * incidence[p];
    }
    for k in 0..n {
        let mut pivot = k;
        for i in k + 1..n {
            if matrix[i * width + k].abs() > matrix[pivot * width + k].abs() { pivot = i; }
        }
        if pivot != k {
            for j in k..=n { matrix.swap(k * width + j, pivot * width + j); }
        }
        let diagonal = matrix[k * width + k];
        if diagonal.abs() < 1e-12 { circulation.fill(0.0); return; }
        for i in k + 1..n {
            let f = matrix[i * width + k] / diagonal;
            if f != 0.0 && !f.is_nan() {
                // Row i minus f times row k, from column k on, two columns at a time.
                let (above, below) = matrix.split_at_mut(i * width);
                let (source, target) = (&above[k * width + k..k * width + width], &mut below[k..width]);
                let pairs = source.len() / 2;
                let factor = f64x2_splat(f);
                for j in 0..pairs {
                    // Both rows hold at least 2 * pairs values from here.
                    unsafe {
                        let (from, to) = (source.as_ptr().add(2 * j) as *const v128, target.as_mut_ptr().add(2 * j) as *mut v128);
                        v128_store(to, f64x2_sub(v128_load(to), f64x2_mul(factor, v128_load(from))));
                    }
                }
                if source.len() % 2 == 1 {
                    let j = source.len() - 1;
                    target[j] -= f * source[j];
                }
            }
        }
    }
    for i in (0..n).rev() {
        let mut rhs = matrix[i * width + n];
        for j in i + 1..n { rhs -= matrix[i * width + j] * circulation[j]; }
        circulation[i] = rhs / matrix[i * width + i];
    }
    if circulation.iter().any(|c| !c.is_finite()) { circulation.fill(0.0); }
}

/// Quasi-steady low-aspect-ratio polar with a smooth, unvalidated separated-flow blend.
/// Returns (cl, cd, stalled).
pub fn coefficients(alpha: f64) -> (f64, f64, bool) {
    let a = alpha.abs();
    let (start, end) = (15.0 * PI / 180.0, 30.0 * PI / 180.0);
    let t = js::clamp((a - start) / (end - start), 0.0, 1.0);
    let blend = t * t * (3.0 - 2.0 * t);
    let attached = 2.7 * alpha;
    let cl = (1.0 - blend) * attached + blend * 0.85 * (2.0 * alpha).sin();
    let sine = alpha.sin();
    let cd = (1.0 - blend) * (0.035 + 0.18 * cl * cl) + blend * (0.08 + 1.75 * sine * sine);
    (cl, cd, a > start)
}

/// The uncoupled per-face model: each triangle is a flat plate at its own incidence.
/// Reads current vertices, areas, normals and velocities every substep; no rest normals.
/// Adds to `out`; fills `panels`.
#[allow(clippy::too_many_arguments)]
pub fn panel_loads(positions: &[f64], velocities: &[f64], faces: &[i32], area_weights: &[f64], weather: &Weather, time: f64, enabled: bool, density: f64, out: &mut [f64], panels: &mut Vec<f64>) {
    let count = faces.len() / 3;
    panels.clear();
    panels.resize(PANEL * count, 0.0);
    let varies = weather.varies();
    for fi in 0..count {
        let (a, b, c) = (3 * faces[3 * fi] as usize, 3 * faces[3 * fi + 1] as usize, 3 * faces[3 * fi + 2] as usize);
        let center = [(positions[a] + positions[b] + positions[c]) / 3.0, (positions[a + 1] + positions[b + 1] + positions[c + 1]) / 3.0, (positions[a + 2] + positions[b + 2] + positions[c + 2]) / 3.0];
        let u = [positions[b] - positions[a], positions[b + 1] - positions[a + 1], positions[b + 2] - positions[a + 2]];
        let w = [positions[c] - positions[a], positions[c + 1] - positions[a + 1], positions[c + 2] - positions[a + 2]];
        let av = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]];
        let twice_area = js::hypot3(av[0], av[1], av[2]);
        let area = twice_area / 2.0 * if area_weights.is_empty() { 1.0 } else { area_weights[fi] };
        let wind = if varies { weather.air_velocity(time, center) } else { weather.wind };
        let mut v = [0.0; 3];
        for d in 0..3 { v[d] = (velocities[a + d] + velocities[b + d] + velocities[c + d]) / 3.0 - wind[d]; }
        let speed = js::hypot3(v[0], v[1], v[2]);
        let panel = &mut panels[PANEL * fi..PANEL * fi + PANEL];
        panel[0] = center[0]; panel[1] = center[1]; panel[2] = center[2];
        panel[10] = area;
        if !(enabled && twice_area > 1e-12 && speed > 1e-8) { continue; }
        let (ia, is) = (1.0 / twice_area, 1.0 / speed);
        let n = [av[0] * ia, av[1] * ia, av[2] * ia];
        let direction = [v[0] * is, v[1] * is, v[2] * is];
        let s = js::clamp(n[0] * direction[0] + n[1] * direction[1] + n[2] * direction[2], -1.0, 1.0);
        let alpha = (-s).asin();
        let (cl, cd, stalled) = coefficients(alpha);
        // Lift perpendicular to velocity; drag always removes energy in still air.
        let raw = [n[0] - direction[0] * s, n[1] - direction[1] * s, n[2] - direction[2] * s];
        let inverse = 1.0 / js::max(js::hypot3(raw[0], raw[1], raw[2]), 1e-12);
        let q_a = 0.5 * density * speed * speed * area;
        for d in 0..3 {
            let lift = raw[d] * inverse;
            let force = q_a * (cl * lift - cd * direction[d]);
            panel[3 + d] = force;
            panel[6 + d] = q_a * cl * lift;
            out[a + d] += force / 3.0;
            out[b + d] += force / 3.0;
            out[c + d] += force / 3.0;
        }
        panel[9] = alpha;
        panel[11] = if stalled { 1.0 } else { 0.0 };
    }
}
