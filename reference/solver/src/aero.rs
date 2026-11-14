//! Aerodynamic loads: an independent implementation of the model in docs/physics-upgrade.md
//! (exposed-area quadrature, planar vortex lattice, Polhamus suction, separated-flow blend).
//! Nothing is cached or held between evaluations, layer shielding tests every face, and the
//! quadrature and lattice can be finer than the game's.
use crate::problem::Problem;
use crate::vec::*;
use std::f64::consts::PI;

pub const PROFILE_DRAG: f64 = 0.035;
pub const VORTEX_LIFT: f64 = PI;

pub struct Patch { pub face: usize, pub weights: V3, pub fraction: f64, pub station: usize, pub panel: usize }

pub struct Weather { pub wind: V3, pub heading: f64, pub gusts: f64, pub turbulence: f64 }
impl Weather {
    /// The game's prescribed, divergence-free synthetic air field.
    pub fn at(&self, time: f64, p: V3) -> V3 {
        if self.gusts == 0.0 && self.turbulence == 0.0 { return self.wind; }
        let speed = norm(self.wind);
        let direction = if speed > 1e-9 { scale(self.wind, 1.0 / speed) } else { [(self.heading * PI / 180.0).cos(), 0.0, (self.heading * PI / 180.0).sin()] };
        let gust = self.gusts * (0.5 + 0.5 * (time * 2.1 - 0.8).sin()).powi(3);
        let (x, y, z, t) = (p[0], p[1], p[2], self.turbulence);
        [
            self.wind[0] + direction[0] * gust + t * 0.45 * ((1.7 * z + 2.9 * time).sin() + 0.5 * (4.3 * y - 5.1 * time).sin()),
            self.wind[1] + direction[1] * gust + t * 0.35 * ((2.3 * x - 3.1 * time + 0.7).sin() + 0.5 * (5.4 * z + 6.7 * time).sin()),
            self.wind[2] + direction[2] * gust + t * 0.55 * ((2.1 * y + 2.5 * time + 1.2).sin() + 0.5 * (4.1 * x - 4.7 * time).sin()),
        ]
    }
}

pub struct Surface {
    pub patches: Vec<Patch>, pub boundaries: Vec<f64>, pub span: f64, pub chord_panels: usize,
    nose: usize, tail: usize, left: usize, right: usize,
    owners: Option<Vec<usize>>, signs: Vec<f64>, gap: f64,
}

#[derive(Default)]
pub struct Loads { pub force: V3, pub lift: V3, pub area: f64, pub stalled_area: f64 }

/// Clip a polygon given in barycentric coordinates of triangle `p` against a rest-space plane.
fn clip(p: &[V3; 3], poly: &[V3], axis: usize, limit: f64, sign: f64) -> Vec<V3> {
    let coordinate = |w: &V3| w[0] * p[0][axis] + w[1] * p[1][axis] + w[2] * p[2][axis];
    let mut out = Vec::with_capacity(poly.len() + 1);
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        let (da, db) = (sign * (coordinate(&a) - limit), sign * (coordinate(&b) - limit));
        if da >= 0.0 { out.push(a); }
        if (da < 0.0 && db > 0.0) || (da > 0.0 && db < 0.0) { out.push(add(a, scale(sub(b, a), da / (da - db)))); }
    }
    out
}
/// Area of a barycentric triangle as a fraction of its parent face.
fn fraction(a: V3, b: V3, c: V3) -> f64 { let (ab, ac) = (sub(b, a), sub(c, a)); (ab[1] * ac[2] - ab[2] * ac[1]).abs() }

fn refine(out: &mut Vec<(V3, f64)>, a: V3, b: V3, c: V3, face_area: f64, limit: f64, depth: usize) {
    let part = fraction(a, b, c);
    if part * face_area <= limit || depth >= 8 {
        if part > 0.0 { out.push((scale(add(add(a, b), c), 1.0 / 3.0), part)); }
        return;
    }
    let (ab, bc, ca) = (scale(add(a, b), 0.5), scale(add(b, c), 0.5), scale(add(c, a), 0.5));
    refine(out, a, ab, ca, face_area, limit, depth + 1); refine(out, ab, b, bc, face_area, limit, depth + 1);
    refine(out, ca, bc, c, face_area, limit, depth + 1); refine(out, ab, bc, ca, face_area, limit, depth + 1);
}

impl Surface {
    /// `stations` cosine-spaced span strips by rest z, each cut into `chord_panels` equal bins
    /// along rest x between that strip's leading and trailing extent. `patch_area` is the largest
    /// quadrature patch in m^2; zero reproduces the game's rule of one patch per clipped polygon.
    pub fn build(p: &Problem, stations: usize, chord_panels: usize, patch_area: f64) -> Surface {
        let (lo, hi) = p.rest.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), q| (lo.min(q[2]), hi.max(q[2])));
        let span = hi - lo;
        let boundaries: Vec<f64> = (0..=stations).map(|i| lo + span * (1.0 - (PI * i as f64 / stations as f64).cos()) / 2.0).collect();
        let mut signs = Vec::new();
        let mut strips: Vec<(usize, usize, Vec<V3>)> = Vec::new();
        let (mut front, mut back) = (vec![f64::NEG_INFINITY; stations], vec![f64::INFINITY; stations]);
        for (fi, face) in p.faces.iter().enumerate() {
            let q = face.map(|i| p.rest[i]);
            let normal = cross(sub(q[1], q[0]), sub(q[2], q[0]));
            signs.push(if normal[1] < 0.0 { -1.0 } else { 1.0 });
            for station in 0..stations {
                let whole = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
                let polygon = clip(&q, &clip(&q, &whole, 2, boundaries[station], 1.0), 2, boundaries[station + 1], -1.0);
                if polygon.len() < 3 { continue; }
                for w in &polygon {
                    let x = w[0] * q[0][0] + w[1] * q[1][0] + w[2] * q[2][0];
                    front[station] = front[station].max(x); back[station] = back[station].min(x);
                }
                strips.push((fi, station, polygon));
            }
        }
        let mut patches = Vec::new();
        for (fi, station, polygon) in &strips {
            let q = p.faces[*fi].map(|i| p.rest[i]);
            let face_area = norm(cross(sub(q[1], q[0]), sub(q[2], q[0]))) / 2.0;
            let chord = front[*station] - back[*station];
            for bin in 0..chord_panels {
                let piece = if chord_panels == 1 { polygon.clone() } else {
                    let rear = clip(&q, polygon, 0, front[*station] - chord * (bin + 1) as f64 / chord_panels as f64, 1.0);
                    clip(&q, &rear, 0, front[*station] - chord * bin as f64 / chord_panels as f64, -1.0)
                };
                let panel = station * chord_panels + bin;
                let (mut total, mut weights) = (0.0, [0.0; 3]);
                let mut fine = Vec::new();
                for k in 1..piece.len().saturating_sub(1) {
                    let (a, b, c) = (piece[0], piece[k], piece[k + 1]);
                    let part = fraction(a, b, c);
                    total += part; weights = add(weights, scale(add(add(a, b), c), part / 3.0));
                    if patch_area > 0.0 { refine(&mut fine, a, b, c, face_area, patch_area, 0); }
                }
                if total <= 1e-10 { continue; }
                if patch_area > 0.0 {
                    for (w, part) in fine { patches.push(Patch { face: *fi, weights: w, fraction: part, station: *station, panel }); }
                } else {
                    patches.push(Patch { face: *fi, weights: scale(weights, 1.0 / total), fraction: total, station: *station, panel });
                }
            }
        }
        Surface {
            patches, boundaries, span, chord_panels, nose: p.nose, tail: p.tail, left: p.left, right: p.right,
            owners: if p.has_owners { Some(p.owners.clone()) } else { None }, signs, gap: p.scalar("shieldingDistance"),
        }
    }

    /// Air loads on the current shape. Adds nodal forces to `out` (force and moment are
    /// transferred exactly through each patch's barycentric weights).
    pub fn loads(&self, x: &[f64], v: &[f64], faces: &[[usize; 3]], density: f64, weather: &Weather, time: f64, out: &mut [f64]) -> Loads {
        let n = self.boundaries.len() - 1;
        let m = self.chord_panels;
        let count = x.len() / 3;
        let tail = get(x, self.tail);
        let forward = unit(sub(get(x, self.nose), tail));
        let span_vector = sub(get(x, self.right), get(x, self.left));
        let span_axis = unit(sub(span_vector, scale(forward, dot(span_vector, forward))));
        let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
        for i in 0..count { let c = dot(sub(get(x, i), tail), span_axis); low = low.min(c); high = high.max(c); }
        let span_scale = ((high - low) / self.span).max(0.1);
        let up = unit(cross(span_vector, forward));
        let boundaries: Vec<f64> = self.boundaries.iter().map(|b| b * span_scale).collect();

        // Per face: corner, edges, unoriented unit normal, area.
        struct Geometry { a: V3, ab: V3, ac: V3, normal: V3, area: f64, aa: f64, bb: f64, cc: f64, den: f64 }
        let geometry: Vec<Geometry> = faces.iter().map(|f| {
            let (a, b, c) = (get(x, f[0]), get(x, f[1]), get(x, f[2]));
            let (ab, ac) = (sub(b, a), sub(c, a));
            let raw = cross(ab, ac);
            let len = norm(raw);
            let (aa, bb, cc) = (dot(ab, ab), dot(ac, ac), dot(ab, ac));
            Geometry { a, ab, ac, normal: scale(raw, 1.0 / len.max(1e-12)), area: len / 2.0, aa, bb, cc, den: aa * bb - cc * cc }
        }).collect();
        // Layer shielding: distinct material panels that are nearly parallel to this patch's face
        // and cover its centre within the shielding distance share the exposed area equally.
        let shields: Option<Vec<Vec<usize>>> = self.owners.as_ref().map(|owners| {
            (0..faces.len()).map(|i| (0..faces.len()).filter(|&j| owners[j] != owners[i] && dot(geometry[i].normal, geometry[j].normal).abs() >= 0.98).collect()).collect()
        });
        let exposure = |point: V3, fi: usize, seen: &mut Vec<usize>| -> f64 {
            let (Some(shields), Some(owners)) = (&shields, &self.owners) else { return 1.0 };
            seen.clear(); seen.push(owners[fi]);
            for &j in &shields[fi] {
                if seen.contains(&owners[j]) { continue; }
                let g = &geometry[j];
                let r = sub(point, g.a);
                if dot(r, g.normal).abs() > self.gap || g.den < 1e-20 { continue; }
                let (dab, dac) = (dot(r, g.ab), dot(r, g.ac));
                let (u, w) = ((dab * g.bb - dac * g.cc) / g.den, (dac * g.aa - dab * g.cc) / g.den);
                if u >= -1e-8 && w >= -1e-8 && u + w <= 1.0 + 1e-8 { seen.push(owners[j]); }
            }
            1.0 / seen.len() as f64
        };

        struct Sample { direction: V3, normal: V3, area: f64, speed: f64, alpha: f64, attached: f64 }
        // Per lattice panel: planform, planform x speed, attached planform x incidence,
        // attached planform, planform x body station, attached exposed area.
        let lattice = n * m;
        let mut strips = vec![[0.0f64; 6]; lattice];
        let mut samples = Vec::with_capacity(self.patches.len());
        let mut seen = Vec::new();
        for patch in &self.patches {
            let face = faces[patch.face];
            let w = patch.weights;
            let mut center = [0.0; 3]; let mut velocity = [0.0; 3];
            for j in 0..3 { center = add(center, scale(get(x, face[j]), w[j])); velocity = add(velocity, scale(get(v, face[j]), w[j])); }
            let relative = sub(velocity, weather.at(time, center));
            let speed = norm(relative);
            let direction = scale(relative, 1.0 / speed.max(1e-12));
            let g = &geometry[patch.face];
            let normal = scale(g.normal, self.signs[patch.face]);
            let alpha = (-dot(normal, direction)).clamp(-1.0, 1.0).asin();
            let area = g.area * patch.fraction * exposure(center, patch.face, &mut seen);
            let lifting = dot(normal, up).abs();
            // Thin-wing assumptions fade out for stall, reversed flow, side-slip and steep panels.
            let attached = (1.0 - smoothstep(0.21, 0.49, alpha.abs())) * smoothstep(0.25, 0.75, dot(direction, forward)) * smoothstep(0.2, 0.7, lifting);
            let station = dot(sub(center, tail), forward);
            let plan = area * lifting;
            let s = &mut strips[patch.panel];
            s[0] += plan; s[1] += plan * speed; s[2] += plan * attached * alpha; s[3] += plan * attached; s[4] += plan * station; s[5] += area * attached;
            samples.push(Sample { direction, normal, area, speed, alpha, attached });
        }
        // Equivalent chord is planform over strip width; a strip's bins are laid end to end,
        // leading bin first, centred on the strip's planform centroid along the body axis.
        let (mut chords, mut lead, mut speeds, mut angles, mut attachment) = (vec![0.0; lattice], vec![0.0; lattice], vec![0.0; lattice], vec![0.0; lattice], vec![0.0; lattice]);
        for i in 0..n {
            let width = (boundaries[i + 1] - boundaries[i]).max(1e-8);
            let (mut area, mut moment) = (0.0, 0.0);
            for p in i * m..(i + 1) * m { area += strips[p][0]; moment += strips[p][4]; }
            let mut edge = moment / area.max(1e-12) + area / width / 2.0;
            for p in i * m..(i + 1) * m {
                let (a, attached) = (strips[p][0], strips[p][3]);
                chords[p] = a / width; lead[p] = edge; edge -= chords[p];
                speeds[p] = strips[p][1] / a.max(1e-12); angles[p] = strips[p][2] / attached.max(1e-12); attachment[p] = attached / a.max(1e-12);
            }
        }
        let circulation = solve_vortex_lattice(&boundaries, m, &lead, &chords, &speeds, &angles, &attachment);
        let mut strip_circulation = vec![0.0; n];
        for p in 0..lattice { strip_circulation[p / m] += circulation[p]; }
        let downwash = trailing_downwash(&boundaries, &strip_circulation);

        let mut loads = Loads::default();
        for (patch, s) in self.patches.iter().zip(&samples) {
            let (i, p) = (patch.station, patch.panel);
            // Kutta-Joukowski panel lift rho V Gamma dy shared over the panel's attached exposed area.
            let coupled = 2.0 * circulation[p] * (boundaries[i + 1] - boundaries[i]) / (speeds[p] * strips[p][5]).max(1e-12);
            let attached = s.attached * coupled + s.attached * 2.0 * PI * (s.alpha - angles[p]);
            let sine = s.alpha.sin();
            let vortex = s.attached * VORTEX_LIFT * sine * sine.abs();
            let bounded = attached.clamp(-1.5, 1.5);
            let cl = bounded + vortex * s.alpha.cos() + (1.0 - s.attached) * 0.85 * (2.0 * s.alpha).sin();
            let cd = PROFILE_DRAG + (vortex * sine).abs() + (1.0 - s.attached) * 1.75 * sine * sine + (bounded * downwash[i] / s.speed.max(1e-8)).max(0.0);
            let along = dot(s.normal, s.direction);
            let lift_direction = sub(s.normal, scale(s.direction, along));
            let lift_scale = 1.0 / norm(lift_direction).max(1e-12);
            let q = 0.5 * density * s.speed * s.speed * s.area;
            let lift = scale(lift_direction, q * cl * lift_scale);
            let force = sub(lift, scale(s.direction, q * cd));
            let face = faces[patch.face];
            for j in 0..3 { axpy(out, face[j], patch.weights[j], force); }
            loads.force = add(loads.force, force); loads.lift = add(loads.lift, lift);
            loads.area += s.area; if s.attached < 0.5 { loads.stalled_area += s.area; }
        }
        loads
    }
}

const FOUR_PI: f64 = 4.0 * PI;
/// Normal velocity at (px, py) from a unit vortex segment a -> b in the lattice plane.
fn segment_wash(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let (r1x, r1y, r2x, r2y) = (px - ax, py - ay, px - bx, py - by);
    let cz = r1x * r2y - r1y * r2x;
    let (l1, l2, r0x, r0y) = (r1x.hypot(r1y), r2x.hypot(r2y), bx - ax, by - ay);
    if cz.abs() < 1e-12 * (l1 * l2 + 1e-30) { return 0.0; }
    ((r0x * r1x + r0y * r1y) / l1 - (r0x * r2x + r0y * r2y) / l2) / (FOUR_PI * cz)
}
/// Unit semi-infinite vortex from (ax, ay) downstream along -x.
fn trailing_wash(px: f64, py: f64, ax: f64, ay: f64) -> f64 {
    let (r1x, r1y) = (px - ax, py - ay);
    let (cz, l1) = (-r1y, r1x.hypot(r1y));
    if cz.abs() < 1e-12 * (l1 + 1e-30) { return 0.0; }
    (1.0 - r1x / l1) / (FOUR_PI * cz)
}
/// Planar horseshoe lattice: bound vortex at each panel's quarter chord, tangency at three
/// quarters. Attachment a scales a row from flow tangency (1) to zero circulation (0).
pub fn solve_vortex_lattice(boundaries: &[f64], m: usize, lead: &[f64], chord: &[f64], speeds: &[f64], incidence: &[f64], attachment: &[f64]) -> Vec<f64> {
    let n = (boundaries.len() - 1) * m;
    let width = n + 1;
    let mut matrix = vec![0.0; n * width];
    let active = |p: usize| chord[p] > 1e-9 && attachment[p] > 1e-9;
    for p in 0..n {
        let row = p * width;
        if !active(p) { matrix[row + p] = 1.0; continue; }
        let i = p / m;
        let (px, py) = (lead[p] - 0.75 * chord[p], (boundaries[i] + boundaries[i + 1]) / 2.0);
        for q in 0..n {
            if !active(q) { continue; }
            let j = q / m;
            let (bx, ya, yb) = (lead[q] - 0.25 * chord[q], boundaries[j], boundaries[j + 1]);
            matrix[row + q] = attachment[p] * (segment_wash(px, py, bx, ya, bx, yb) + trailing_wash(px, py, bx, yb) - trailing_wash(px, py, bx, ya));
        }
        matrix[row + p] += (1.0 - attachment[p]) * matrix[row + p] / attachment[p];
        matrix[row + n] = attachment[p] * speeds[p] * incidence[p];
    }
    for k in 0..n {
        let mut pivot = k;
        for i in k + 1..n { if matrix[i * width + k].abs() > matrix[pivot * width + k].abs() { pivot = i; } }
        if pivot != k { for j in k..=n { matrix.swap(k * width + j, pivot * width + j); } }
        let diagonal = matrix[k * width + k];
        if diagonal.abs() < 1e-12 { return vec![0.0; n]; }
        for i in k + 1..n {
            let f = matrix[i * width + k] / diagonal;
            if f != 0.0 { for j in k..=n { matrix[i * width + j] -= f * matrix[k * width + j]; } }
        }
    }
    let mut circulation = vec![0.0; n];
    for i in (0..n).rev() {
        let mut rhs = matrix[i * width + n];
        for j in i + 1..n { rhs -= matrix[i * width + j] * circulation[j]; }
        circulation[i] = rhs / matrix[i * width + i];
    }
    if circulation.iter().any(|c| !c.is_finite()) { return vec![0.0; n]; }
    circulation
}
/// Downwash (positive down) at strip centres from the trailing legs of strip circulation.
pub fn trailing_downwash(boundaries: &[f64], circulation: &[f64]) -> Vec<f64> {
    let n = boundaries.len() - 1;
    (0..n).map(|i| {
        let y = (boundaries[i] + boundaries[i + 1]) / 2.0;
        (0..n).map(|j| circulation[j] * (1.0 / (y - boundaries[j]) - 1.0 / (y - boundaries[j + 1])) / FOUR_PI).sum()
    }).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    /// Flat rectangular wing in the lattice alone: lift slope against Helmbold's formula.
    fn lift_slope(aspect: f64, stations: usize, m: usize) -> f64 {
        let (chord, alpha, speed) = (0.2, 0.05, 8.0);
        let span = aspect * chord;
        let boundaries: Vec<f64> = (0..=stations).map(|i| -span / 2.0 + span * (1.0 - (PI * i as f64 / stations as f64).cos()) / 2.0).collect();
        let n = stations * m;
        let lead: Vec<f64> = (0..n).map(|p| -((p % m) as f64) * chord / m as f64).collect();
        let circulation = solve_vortex_lattice(&boundaries, m, &lead, &vec![chord / m as f64; n], &vec![speed; n], &vec![alpha; n], &vec![1.0; n]);
        let lift: f64 = (0..n).map(|p| speed * circulation[p] * (boundaries[p / m + 1] - boundaries[p / m])).sum();
        2.0 * lift / (speed * speed * span * chord) / alpha
    }
    #[test]
    fn lattice_follows_helmbold_at_low_aspect_ratio() {
        for aspect in [1.0f64, 2.0, 6.0] {
            let helmbold = 2.0 * PI * aspect / (2.0 + (aspect * aspect + 4.0).sqrt());
            let slope = lift_slope(aspect, 32, 8);
            assert!((slope / helmbold - 1.0).abs() < 0.08, "AR {aspect}: {slope} vs {helmbold}");
        }
    }
    #[test]
    fn lattice_converges_under_refinement() {
        let (coarse, medium, fine) = (lift_slope(2.0, 12, 4), lift_slope(2.0, 24, 8), lift_slope(2.0, 48, 8));
        assert!((medium - fine).abs() < (coarse - fine).abs(), "{coarse} {medium} {fine}");
        assert!((medium / fine - 1.0).abs() < 0.03, "{coarse} {medium} {fine}");
    }
}
