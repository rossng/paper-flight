//! Time integration. Each step minimises the BDF2 incremental potential
//!   beta/2 |x - x_hat|^2_M + E_elastic(x) + E_gravity(x) + R_damping(x) + E_contact(x) - f_air . x
//! by Newton's method with a sparse direct solve and a backtracking line search, to a tight
//! displacement tolerance. Air loads are evaluated once per step on the extrapolated state.
//! There are no sweeps, strain limiters, held loads or contact schedules to tune.
use crate::aero::{Loads, Surface, Weather};
use crate::elements::*;
use crate::linalg::Sparse;
use crate::problem::Problem;
use crate::vec::*;
use std::collections::HashMap;
use std::f64::consts::PI;

pub const FRAME: f64 = 1.0 / 120.0;

#[derive(Clone)]
pub struct Options {
    /// Implicit steps per 1/120 s frame.
    pub steps_per_frame: usize,
    /// Vortex-lattice resolution; zero uses the problem's own (the game's parameters).
    pub aero_stations: usize, pub aero_chord: usize,
    /// Largest aerodynamic quadrature patch (m^2); 0 keeps one patch per clipped polygon.
    pub patch_area: f64,
    /// Penalty stiffness of layer and floor contact (N/m per vertex) and of static friction.
    pub contact_stiffness: f64,
    /// Vertex-face pairs closer than this (m) are tracked as contact candidates.
    pub contact_range: f64,
    /// A vertex overhanging a panel's edge by up to this much (m) still bears on the panel's plane,
    /// so layers creased together cannot slip around each other's fold line.
    pub contact_margin: f64,
    /// A coincident pair with no known stacking side takes the side it first opens to by this gap (m).
    pub side_gap: f64,
    /// A pair whose fold order is known keeps that side while closer than this (m) on entering range.
    pub fold_gap: f64,
    /// Newton stops when the largest vertex displacement update falls below this (m).
    pub tolerance: f64,
    /// Seconds to keep simulating after first touchdown; negative runs until rest or timeout.
    pub after_touchdown: f64,
    pub max_time: f64,
    /// Ignore stacking sides supplied by the problem (use first-opening sides, as the game does).
    pub ignore_sides: bool,
}
impl Default for Options {
    fn default() -> Self {
        Options { steps_per_frame: 64, aero_stations: 0, aero_chord: 0, patch_area: 2.5e-5, contact_stiffness: 2e4, contact_range: 1e-3,
            contact_margin: 5e-4, side_gap: 1e-9, fold_gap: 1e-6, tolerance: 1e-10, after_touchdown: 0.0, max_time: 12.0, ignore_sides: false }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Status { Flying, Settling, Landed, Finished, Stopped, Error }
impl Status {
    pub fn name(self) -> &'static str {
        match self { Status::Flying => "flying", Status::Settling => "settling", Status::Landed => "landed", Status::Finished => "finished", Status::Stopped => "stopped", Status::Error => "error" }
    }
}

#[derive(Clone, Copy, Default)]
struct Friction { anchor: V3, normal_force: f64, sliding: bool }
#[derive(Clone, Copy, Default)]
struct PairState { side: f64, preferred: f64, friction: Friction }
struct Contact { vertex: usize, owner: usize, face: [usize; 3], weights: V3, normal: V3 }

/// Coulomb friction with an elastic stick regularisation: a tangential spring of stiffness k
/// whose force saturates at `cap`. Returns energy, force and tangent stiffness at offset z.
fn huber(z: V3, k: f64, cap: f64) -> (f64, V3, f64, Option<V3>) {
    let length = norm(z);
    if k * length <= cap { (0.5 * k * length * length, scale(z, k), k, None) }
    else { let direction = scale(z, 1.0 / length); (cap * length - cap * cap / (2.0 * k), scale(direction, cap), cap / length, Some(direction)) }
}

#[derive(Default, Clone, Copy)]
pub struct Energy { pub kinetic: f64, pub elastic: f64, pub gravity: f64, pub air_work: f64, pub launch_work: f64, pub damping_loss: f64 }

pub struct Touchdown { pub time: f64, pub center: V3, pub distance: f64 }

pub struct Simulation {
    pub options: Options,
    pub n: usize, pub masses: Vec<f64>, pub total_mass: f64,
    rest: Vec<V3>, pub faces: Vec<[usize; 3]>, owners: Vec<usize>,
    membranes: Vec<Membrane>, hinges: Vec<crate::problem::Hinge>, pub plastic: Vec<Plastic>, edges: Vec<crate::problem::Edge>,
    edge_damping: Vec<f64>, yield_angle: f64, hardening: f64,
    surface: Surface, weather: Weather, density: f64, aero: bool,
    gravity: f64, launch_speed: f64, launch_duration: f64, launch_direction: V3,
    ground: f64, mu_static: f64, mu_kinetic: f64, mu_layer: f64,
    nose: usize, tail: usize, left: usize, right: usize,
    skip: Vec<bool>, static_pairs: Vec<(usize, usize)>,
    pub x: Vec<f64>, pub v: Vec<f64>, x_prev: Vec<f64>, v_prev: Vec<f64>, started: bool,
    pairs: HashMap<(usize, usize), PairState>, contacts: Vec<Contact>, ground_friction: Vec<Friction>,
    sparse: Option<Sparse>, pattern: Vec<(usize, [usize; 3])>,
    pub time: f64, pub steps: usize, pub status: Status, pub touchdown: Option<Touchdown>,
    pub start: V3, last_center: V3, pub path_length: f64, pub ground_distance: f64, quiet_time: f64,
    pub loads: Loads, pub energy: Energy, pub energy_start: f64, air_last: Vec<f64>, launch_last: f64,
    pub newton_iterations: usize, pub worst_newton: usize, pub line_searches: usize, pub regularised: usize, pub pattern_rebuilds: usize,
    pub peak_penetration: f64, pub peak_deformation: f64, pub peak_strain: f64, pub active_contacts: usize,
}

struct Step<'a> { beta: f64, rate: f64, x_hat: &'a [f64], length_hat: &'a [f64], x_start: &'a [f64], external: &'a [f64] }

impl Simulation {
    pub fn new(p: &Problem, mut options: Options) -> Simulation {
        let n = p.rest.len();
        if options.aero_stations == 0 { options.aero_stations = p.scalar_or("aeroStations", 12.0) as usize; }
        if options.aero_chord == 0 { options.aero_chord = p.scalar_or("aeroChordPanels", 4.0) as usize; }
        let surface = Surface::build(p, options.aero_stations, options.aero_chord, options.patch_area);
        let x: Vec<f64> = p.start.iter().flatten().copied().collect();
        let direction = [p.scalar("launchDirectionX"), p.scalar("launchDirectionY"), p.scalar("launchDirectionZ")];
        let (speed, duration) = (p.scalar("launchSpeed"), p.scalar("launchDuration"));
        let mut v = vec![0.0; 3 * n];
        if duration <= 0.0 { for i in 0..n { axpy(&mut v, i, speed, direction); } }
        // Contact never acts between a vertex and its own material panel, or (on meshes without
        // panels) the faces around it.
        let mut skip = vec![false; n * p.faces.len()];
        let mut around: Vec<Vec<usize>> = vec![vec![]; n];
        for (fi, f) in p.faces.iter().enumerate() { for &i in f { around[i].push(fi); } }
        for vertex in 0..n {
            for (fi, f) in p.faces.iter().enumerate() {
                skip[vertex * p.faces.len() + fi] = if p.has_owners { around[vertex].iter().any(|&g| p.owners[g] == p.owners[fi]) }
                    else { around[vertex].iter().any(|&g| p.faces[g].iter().any(|i| f.contains(i))) };
            }
        }
        let mut static_pairs = Vec::new();
        for f in &p.faces { for a in 0..3 { for b in a + 1..3 { static_pairs.push((f[a], f[b])); } } }
        for h in &p.hinges { for a in 0..4 { for b in a + 1..4 { static_pairs.push((h.ids[a], h.ids[b])); } } }
        let rate = p.scalar_or("edgeDampingRate", 18.0);
        let edge_damping = p.edges.iter().map(|e| rate / (1.0 / p.masses[e.a] + 1.0 / p.masses[e.b])).collect();
        let mut pairs = HashMap::new();
        if !options.ignore_sides { for s in &p.sides { pairs.insert((s.vertex, s.owner), PairState { side: s.sign, preferred: s.sign, friction: Friction::default() }); } }
        let total_mass = p.masses.iter().sum();
        let mut sim = Simulation {
            n, masses: p.masses.clone(), total_mass, rest: p.rest.clone(), faces: p.faces.clone(), owners: p.owners.clone(),
            membranes: build_membranes(p), plastic: vec![Plastic::default(); p.hinges.len()],
            hinges: p.hinges.iter().map(|h| crate::problem::Hinge { ids: h.ids, rest: h.rest, stiffness: h.stiffness, crease: h.crease }).collect(),
            edges: p.edges.iter().map(|e| crate::problem::Edge { a: e.a, b: e.b, rest: e.rest }).collect(),
            edge_damping, yield_angle: p.scalar("yieldAngle"), hardening: p.scalar("hardening"),
            surface, weather: Weather { wind: [p.scalar("windX"), p.scalar("windY"), p.scalar("windZ")], heading: p.scalar("windHeading"), gusts: p.scalar("gusts"), turbulence: p.scalar("turbulence") },
            density: p.scalar("airDensity"), aero: p.scalar("aero") != 0.0, gravity: p.scalar("gravity"),
            launch_speed: speed, launch_duration: duration, launch_direction: direction,
            ground: p.scalar("groundHeight"), mu_static: p.scalar("groundStaticFriction").max(0.0),
            mu_kinetic: p.scalar("groundKineticFriction").max(0.0).min(p.scalar("groundStaticFriction").max(0.0)), mu_layer: p.scalar("layerFriction"),
            nose: p.nose, tail: p.tail, left: p.left, right: p.right, skip, static_pairs,
            x_prev: x.clone(), v_prev: v.clone(), x, v, started: false,
            pairs, contacts: vec![], ground_friction: vec![Friction::default(); n], sparse: None, pattern: vec![],
            time: 0.0, steps: 0, status: Status::Flying, touchdown: None, start: [0.0; 3], last_center: [0.0; 3], path_length: 0.0,
            ground_distance: 0.0, quiet_time: 0.0, loads: Loads::default(), energy: Energy::default(), energy_start: 0.0, air_last: vec![], launch_last: 0.0,
            newton_iterations: 0, worst_newton: 0, line_searches: 0, regularised: 0, pattern_rebuilds: 0,
            peak_penetration: 0.0, peak_deformation: 0.0, peak_strain: 0.0, active_contacts: 0, options,
        };
        sim.start = sim.center(&sim.x); sim.last_center = sim.start;
        sim.measure_energy();
        sim.energy_start = sim.energy.kinetic + sim.energy.elastic + sim.energy.gravity;
        sim
    }
    pub fn active(&self) -> bool { matches!(self.status, Status::Flying | Status::Settling) }
    pub fn patches(&self) -> usize { self.surface.patches.len() }
    pub fn center(&self, array: &[f64]) -> V3 {
        let mut c = [0.0; 3];
        for i in 0..self.n { for d in 0..3 { c[d] += array[3 * i + d] * self.masses[i]; } }
        scale(c, 1.0 / self.total_mass)
    }
    fn lengths(&self, x: &[f64]) -> Vec<f64> { self.edges.iter().map(|e| norm(sub(get(x, e.a), get(x, e.b)))).collect() }
    fn launch_acceleration(&self, time: f64) -> f64 {
        if self.launch_duration <= 0.0 || time >= self.launch_duration { return 0.0; }
        self.launch_speed * PI / (2.0 * self.launch_duration) * (time / self.launch_duration * PI / 2.0).cos()
    }
    /// Membrane and bending energy of a shape.
    pub fn elastic_energy(&self, x: &[f64]) -> (f64, f64) {
        let membrane: f64 = self.membranes.iter().map(|m| m.energy(x)).sum();
        let mut bending = 0.0;
        for (h, p) in self.hinges.iter().zip(&self.plastic) {
            let c = wrap_angle(hinge_angle(h.ids.map(|i| get(x, i))) - h.rest - p.offset);
            bending += 0.5 * h.stiffness * c * c;
        }
        (membrane, bending)
    }
    /// Elastic nodal forces (minus the energy gradient) of a shape.
    pub fn elastic_forces(&self, x: &[f64]) -> Vec<f64> {
        let mut g = vec![0.0; 3 * self.n];
        for m in &self.membranes { m.add(x, &mut g, None); }
        self.add_hinges(x, &mut g, None);
        g.iter().map(|v| -v).collect()
    }
    pub fn air_forces(&self, x: &[f64], v: &[f64], time: f64) -> (Vec<f64>, Loads) {
        let mut f = vec![0.0; 3 * self.n];
        let loads = self.surface.loads(x, v, &self.faces, if self.aero { self.density } else { 0.0 }, &self.weather, time, &mut f);
        (f, loads)
    }
    fn add_hinges(&self, x: &[f64], g: &mut [f64], mut hess: Option<&mut Sparse>) -> f64 {
        let mut energy = 0.0;
        for (h, p) in self.hinges.iter().zip(&self.plastic) {
            let points = h.ids.map(|i| get(x, i));
            let c = wrap_angle(hinge_angle(points) - h.rest - p.offset);
            energy += 0.5 * h.stiffness * c * c;
            if let Some(gradient) = hinge_gradient(points) {
                for j in 0..4 { axpy(g, h.ids[j], h.stiffness * c, gradient[j]); }
                if let Some(hess) = hess.as_deref_mut() { add_outer(hess, &h.ids, &gradient, h.stiffness); }
            }
        }
        energy
    }
    fn measure_energy(&mut self) {
        let (membrane, bending) = self.elastic_energy(&self.x);
        let (mut kinetic, mut gravity) = (0.0, 0.0);
        for i in 0..self.n { let u = get(&self.v, i); kinetic += 0.5 * self.masses[i] * dot(u, u); gravity += self.masses[i] * self.gravity * self.x[3 * i + 1]; }
        self.energy.kinetic = kinetic; self.energy.elastic = membrane + bending; self.energy.gravity = gravity;
    }

    /// Incremental potential, its gradient and (approximate) Hessian at x.
    fn potential(&self, x: &[f64], step: &Step, g: &mut [f64], mut hess: Option<&mut Sparse>) -> f64 {
        g.fill(0.0);
        if let Some(h) = hess.as_deref_mut() { h.clear(); }
        let mut phi = 0.0;
        for i in 0..self.n {
            let m = self.masses[i];
            for d in 0..3 {
                let k = 3 * i + d;
                let offset = x[k] - step.x_hat[k];
                phi += 0.5 * step.beta * m * offset * offset - step.external[k] * x[k];
                g[k] += step.beta * m * offset - step.external[k];
            }
            phi += m * self.gravity * x[3 * i + 1]; g[3 * i + 1] += m * self.gravity;
            if let Some(h) = hess.as_deref_mut() { h.add_diagonal(i, step.beta * m); }
        }
        for m in &self.membranes { phi += m.add(x, g, hess.as_deref_mut()); }
        phi += self.add_hinges(x, g, hess.as_deref_mut());
        // Internal damping: a dashpot on every edge's rate of length change.
        for (j, e) in self.edges.iter().enumerate() {
            let d = sub(get(x, e.a), get(x, e.b));
            let l = norm(d);
            if l < 1e-12 { continue; }
            let (direction, k) = (scale(d, 1.0 / l), self.edge_damping[j] * step.rate);
            let stretch = l - step.length_hat[j];
            phi += 0.5 * k * stretch * stretch;
            axpy(g, e.a, k * stretch, direction); axpy(g, e.b, -k * stretch, direction);
            if let Some(h) = hess.as_deref_mut() { add_outer(h, &[e.a, e.b], &[direction, scale(direction, -1.0)], k); }
        }
        let k = self.options.contact_stiffness;
        for c in &self.contacts {
            let state = &self.pairs[&(c.vertex, c.owner)];
            if state.side == 0.0 { continue; }
            let ids = [c.vertex, c.face[0], c.face[1], c.face[2]];
            let coefficients = [1.0, -c.weights[0], -c.weights[1], -c.weights[2]];
            let (mut gap, mut slip) = (0.0, [0.0; 3]);
            for j in 0..4 {
                gap += coefficients[j] * dot(c.normal, get(x, ids[j]));
                slip = add(slip, scale(sub(get(x, ids[j]), get(step.x_start, ids[j])), coefficients[j]));
            }
            if gap < 0.0 {
                phi += 0.5 * k * gap * gap;
                let gradient = coefficients.map(|w| scale(c.normal, w));
                for j in 0..4 { axpy(g, ids[j], k * gap, gradient[j]); }
                if let Some(h) = hess.as_deref_mut() { add_outer(h, &ids, &gradient, k); }
            }
            let cap = self.mu_layer * state.friction.normal_force;
            if cap > 0.0 {
                let z = add(state.friction.anchor, sub(slip, scale(c.normal, dot(slip, c.normal))));
                let (energy, force, stiffness, direction) = huber(z, k, cap);
                phi += energy;
                for j in 0..4 { axpy(g, ids[j], coefficients[j], force); }
                if let Some(h) = hess.as_deref_mut() {
                    // Tangent stiffness: s (I - n n^T), less the slip direction once sliding.
                    let mut block = [[0.0; 3]; 3];
                    for a in 0..3 { for b in 0..3 {
                        block[a][b] = stiffness * ((a == b) as u8 as f64 - c.normal[a] * c.normal[b] - direction.map_or(0.0, |d| d[a] * d[b]));
                    } }
                    for i in 0..4 { for j in i..4 {
                        let mut scaled = block;
                        for row in scaled.iter_mut() { for v in row.iter_mut() { *v *= coefficients[i] * coefficients[j]; } }
                        h.add(ids[i], ids[j], &scaled);
                    } }
                }
            }
        }
        for i in 0..self.n {
            let depth = x[3 * i + 1] - self.ground;
            if depth < 0.0 {
                phi += 0.5 * k * depth * depth; g[3 * i + 1] += k * depth;
                if let Some(h) = hess.as_deref_mut() { h.add(i, i, &[[0.0, 0.0, 0.0], [0.0, k, 0.0], [0.0, 0.0, 0.0]]); }
            }
            let friction = &self.ground_friction[i];
            let cap = (if friction.sliding { self.mu_kinetic } else { self.mu_static }) * friction.normal_force;
            if cap > 0.0 {
                let z = [friction.anchor[0] + x[3 * i] - step.x_start[3 * i], 0.0, friction.anchor[2] + x[3 * i + 2] - step.x_start[3 * i + 2]];
                let (energy, force, stiffness, direction) = huber(z, k, cap);
                phi += energy; axpy(g, i, 1.0, force);
                if let Some(h) = hess.as_deref_mut() {
                    let d = direction.unwrap_or([0.0; 3]);
                    h.add(i, i, &[[stiffness * (1.0 - d[0] * d[0]), 0.0, -stiffness * d[0] * d[2]], [0.0; 3], [-stiffness * d[0] * d[2], 0.0, stiffness * (1.0 - d[2] * d[2])]]);
                }
            }
        }
        phi
    }

    /// Vertex-face pairs within range, one per (vertex, material panel): the face whose
    /// footprint contains the vertex best. Geometry is frozen for the step.
    fn detect_contacts(&mut self) {
        let faces = &self.faces;
        let count = faces.len();
        struct Plane { a: V3, ab: V3, ac: V3, normal: V3, aa: f64, bb: f64, cc: f64, den: f64, lo: V3, hi: V3 }
        let range = self.options.contact_range;
        let planes: Vec<Plane> = faces.iter().map(|f| {
            let (a, b, c) = (get(&self.x, f[0]), get(&self.x, f[1]), get(&self.x, f[2]));
            let (ab, ac) = (sub(b, a), sub(c, a));
            let (aa, bb, cc) = (dot(ab, ab), dot(ac, ac), dot(ab, ac));
            let (mut lo, mut hi) = ([0.0; 3], [0.0; 3]);
            for d in 0..3 { lo[d] = a[d].min(b[d]).min(c[d]) - range; hi[d] = a[d].max(b[d]).max(c[d]) + range; }
            Plane { a, ab, ac, normal: unit(cross(ab, ac)), aa, bb, cc, den: aa * bb - cc * cc, lo, hi }
        }).collect();
        let mut found: Vec<Contact> = Vec::new();
        let mut best: Vec<(usize, f64, usize, V3, f64)> = Vec::new();
        for vertex in 0..self.n {
            let p = get(&self.x, vertex);
            best.clear();
            for fi in 0..count {
                if self.skip[vertex * count + fi] { continue; }
                let plane = &planes[fi];
                if p[0] < plane.lo[0] || p[0] > plane.hi[0] || p[1] < plane.lo[1] || p[1] > plane.hi[1] || p[2] < plane.lo[2] || p[2] > plane.hi[2] || plane.den < 1e-24 { continue; }
                let r = sub(p, plane.a);
                let distance = dot(r, plane.normal);
                if distance.abs() > range { continue; }
                let (dab, dac) = (dot(r, plane.ab), dot(r, plane.ac));
                let (u, w) = ((dab * plane.bb - dac * plane.cc) / plane.den, (dac * plane.aa - dab * plane.cc) / plane.den);
                let inside = u.min(w).min(1.0 - u - w);
                // Distance by which the vertex overhangs the triangle, measured in its plane.
                let overhang = if inside >= 0.0 { 0.0 } else {
                    let q = sub(r, scale(plane.normal, distance));
                    let segment = |a: V3, e: V3| { let t = (dot(sub(q, a), e) / dot(e, e)).clamp(0.0, 1.0); norm(sub(q, add(a, scale(e, t)))) };
                    segment([0.0; 3], plane.ab).min(segment([0.0; 3], plane.ac)).min(segment(plane.ab, sub(plane.ac, plane.ab)))
                };
                if overhang > self.options.contact_margin { continue; }
                let owner = self.owners[fi];
                let candidate = (owner, inside - overhang, fi, [1.0 - u - w, u, w], distance);
                match best.iter_mut().find(|b| b.0 == owner) {
                    Some(b) => if candidate.1 > b.1 { *b = candidate; },
                    None => best.push(candidate),
                }
            }
            for &(owner, _, fi, weights, distance) in &best {
                let state = self.pairs.entry((vertex, owner)).or_default();
                // A pair entering range takes the side it is plainly on. Within the ambiguity gap it
                // takes the side the folds put it on, or failing that waits for its first opening.
                if state.side == 0.0 {
                    let gap = if state.preferred != 0.0 { self.options.fold_gap } else { self.options.side_gap };
                    state.side = if distance.abs() > gap { distance.signum() } else { state.preferred };
                }
                let normal = scale(planes[fi].normal, if state.side == 0.0 { 1.0 } else { state.side });
                // Keep the friction anchor tangential to the current contact plane.
                state.friction.anchor = sub(state.friction.anchor, scale(normal, dot(state.friction.anchor, normal)));
                found.push(Contact { vertex, owner, face: faces[fi], weights, normal });
            }
        }
        // A pair that left range forgets its side: it may come back around the panel's edge.
        let listed: std::collections::HashSet<(usize, usize)> = found.iter().map(|c| (c.vertex, c.owner)).collect();
        self.pairs.retain(|key, state| { if !listed.contains(key) { state.friction = Friction::default(); state.side = 0.0; } listed.contains(key) || state.preferred != 0.0 });
        let pattern: Vec<(usize, [usize; 3])> = found.iter().map(|c| (c.vertex, c.face)).collect();
        if self.sparse.is_none() || pattern != self.pattern {
            let mut pairs = self.static_pairs.clone();
            for c in &found {
                let ids = [c.vertex, c.face[0], c.face[1], c.face[2]];
                for a in 0..4 { for b in a + 1..4 { pairs.push((ids[a], ids[b])); } }
            }
            self.sparse = Some(Sparse::new(self.n, &pairs));
            self.pattern = pattern; self.pattern_rebuilds += 1;
        }
        self.contacts = found;
    }

    /// Advance one implicit step of 1/(120 steps_per_frame) s.
    pub fn step(&mut self) {
        if !self.active() { return; }
        let h = FRAME / self.options.steps_per_frame as f64;
        let n3 = 3 * self.n;
        // Air and launcher loads at the end of the step, on the extrapolated state.
        let predicted: Vec<f64> = (0..n3).map(|k| self.x[k] + h * self.v[k]).collect();
        let velocity: Vec<f64> = (0..n3).map(|k| if self.started { 2.0 * self.v[k] - self.v_prev[k] } else { self.v[k] }).collect();
        let (air, loads) = self.air_forces(&predicted, &velocity, self.time + h);
        self.loads = loads;
        let acceleration = self.launch_acceleration(self.time + h);
        let mut external = air.clone();
        for i in 0..self.n { axpy(&mut external, i, self.masses[i] * acceleration, self.launch_direction); }
        let external = external;

        let length_now = self.lengths(&self.x);
        let (beta, rate, x_hat, length_hat): (f64, f64, Vec<f64>, Vec<f64>) = if self.started {
            let length_prev = self.lengths(&self.x_prev);
            (9.0 / (4.0 * h * h), 1.5 / h,
             (0..n3).map(|k| (4.0 * self.x[k] - self.x_prev[k]) / 3.0 + 2.0 * h / 9.0 * (4.0 * self.v[k] - self.v_prev[k])).collect(),
             (0..self.edges.len()).map(|j| (4.0 * length_now[j] - length_prev[j]) / 3.0).collect())
        } else {
            (1.0 / (h * h), 1.0 / h, predicted.clone(), length_now.clone())
        };

        self.detect_contacts();
        let x_start = self.x.clone();
        let step = Step { beta, rate, x_hat: &x_hat, length_hat: &length_hat, x_start: &x_start, external: &external };
        let mut sparse = self.sparse.take().unwrap();
        let mut x = predicted;
        let (mut g, mut direction, mut trial, mut scratch) = (vec![0.0; n3], vec![0.0; n3], vec![0.0; n3], vec![0.0; n3]);
        let mut iterations = 0;
        let mut converged = false;
        while iterations < 60 {
            iterations += 1;
            let phi = self.potential(&x, &step, &mut g, Some(&mut sparse));
            let mut shift = 0.0;
            while !sparse.factor() {
                // Not positive definite (deep buckling): stiffen towards the mass matrix and retry.
                let extra = if shift == 0.0 { beta } else { shift };
                for i in 0..self.n { sparse.add_diagonal(i, extra * self.masses[i]); }
                shift += extra; self.regularised += 1;
                if shift > 1e12 * beta { self.status = Status::Error; self.sparse = Some(sparse); return; }
            }
            for k in 0..n3 { scratch[k] = -g[k]; }
            sparse.solve(&scratch, &mut direction);
            let slope: f64 = (0..n3).map(|k| g[k] * direction[k]).sum();
            let mut alpha = 1.0;
            loop {
                for k in 0..n3 { trial[k] = x[k] + alpha * direction[k]; }
                let value = self.potential(&trial, &step, &mut scratch, None);
                // Accept on sufficient decrease, or when the decrease is below rounding error.
                if value <= phi + 1e-4 * alpha * slope || alpha * slope.abs() <= 1e-14 * phi.abs().max(1e-30) { break; }
                alpha *= 0.5; self.line_searches += 1;
                if alpha < 1e-10 { break; }
            }
            let mut largest: f64 = 0.0;
            for k in 0..n3 { largest = largest.max((alpha * direction[k]).abs()); }
            std::mem::swap(&mut x, &mut trial);
            if !largest.is_finite() { self.status = Status::Error; self.sparse = Some(sparse); return; }
            if largest < self.options.tolerance { converged = true; break; }
        }
        self.sparse = Some(sparse);
        self.newton_iterations += iterations; self.worst_newton = self.worst_newton.max(iterations);
        if !converged { self.worst_newton = usize::MAX; }

        // Velocity from the same BDF formula, then history.
        let v_new: Vec<f64> = (0..n3).map(|k| if self.started { (3.0 * x[k] - 4.0 * self.x[k] + self.x_prev[k]) / (2.0 * h) } else { (x[k] - self.x[k]) / h }).collect();
        let length_new = self.lengths(&x);
        for j in 0..self.edges.len() {
            let speed = rate * (length_new[j] - length_hat[j]);
            self.energy.damping_loss += self.edge_damping[j] * speed * speed * h;
        }
        // Work of the applied loads, by the trapezoid rule over the step.
        if self.air_last.len() != n3 { self.air_last = air.clone(); self.launch_last = acceleration; }
        for k in 0..n3 {
            let dx = x[k] - self.x[k];
            self.energy.air_work += 0.5 * (air[k] + self.air_last[k]) * dx;
            self.energy.launch_work += 0.5 * (acceleration + self.launch_last) * self.masses[k / 3] * self.launch_direction[k % 3] * dx;
        }
        self.air_last = air; self.launch_last = acceleration;
        self.x_prev = std::mem::replace(&mut self.x, x);
        self.v_prev = std::mem::replace(&mut self.v, v_new);
        self.started = true; self.time += h; self.steps += 1;

        // Contact state for the next step: normal load, stick anchor or slip, deepest penetration.
        let k = self.options.contact_stiffness;
        self.active_contacts = 0;
        for c in &self.contacts {
            let state = self.pairs.get_mut(&(c.vertex, c.owner)).unwrap();
            if state.side == 0.0 { continue; }
            let ids = [c.vertex, c.face[0], c.face[1], c.face[2]];
            let coefficients = [1.0, -c.weights[0], -c.weights[1], -c.weights[2]];
            let (mut gap, mut slip) = (0.0, [0.0; 3]);
            for j in 0..4 { gap += coefficients[j] * dot(c.normal, get(&self.x, ids[j])); slip = add(slip, scale(sub(get(&self.x, ids[j]), get(&x_start, ids[j])), coefficients[j])); }
            let cap = self.mu_layer * state.friction.normal_force;
            let mut z = add(state.friction.anchor, sub(slip, scale(c.normal, dot(slip, c.normal))));
            if k * norm(z) > cap { z = scale(z, cap / (k * norm(z)).max(1e-300)); }
            let load = k * (-gap).max(0.0);
            state.friction = if load > 0.0 { Friction { anchor: z, normal_force: load, sliding: false } } else { Friction::default() };
            if load > 0.0 { self.active_contacts += 1; self.peak_penetration = self.peak_penetration.max(-gap); }
        }
        let mut touched = false;
        for i in 0..self.n {
            let depth = self.x[3 * i + 1] - self.ground;
            let load = k * (-depth).max(0.0);
            let friction = &mut self.ground_friction[i];
            if load > 0.0 {
                touched = true;
                let cap = (if friction.sliding { self.mu_kinetic } else { self.mu_static }) * friction.normal_force;
                let mut z = [friction.anchor[0] + self.x[3 * i] - x_start[3 * i], 0.0, friction.anchor[2] + self.x[3 * i + 2] - x_start[3 * i + 2]];
                // Sticking holds until the static limit; once sliding, the kinetic limit applies
                // until the spring relaxes inside it.
                let sliding = k * norm(z) > cap;
                if sliding { z = scale(z, cap / (k * norm(z)).max(1e-300)); }
                *friction = Friction { anchor: z, normal_force: load, sliding };
            } else { *friction = Friction::default(); }
        }
        for (hinge, plastic) in self.hinges.iter().zip(self.plastic.iter_mut()) {
            if hinge.crease { plastic.update(hinge_angle(hinge.ids.map(|i| get(&self.x, i))), hinge.rest, self.yield_angle, self.hardening); }
        }

        // Flight bookkeeping, with the game's touchdown, rest and timeout rules.
        let center = self.center(&self.x);
        self.path_length += norm(sub(center, self.last_center));
        if self.touchdown.is_some() { self.ground_distance += (center[0] - self.last_center[0]).hypot(center[2] - self.last_center[2]); }
        self.last_center = center;
        if self.touchdown.is_none() && touched {
            self.touchdown = Some(Touchdown { time: self.time, center, distance: (center[0] - self.start[0]).hypot(center[2] - self.start[2]) });
            self.status = Status::Settling;
        }
        let (mut energy, mut peak, mut near) = (0.0, 0.0f64, false);
        for i in 0..self.n {
            let u = get(&self.v, i);
            energy += self.masses[i] * dot(u, u); peak = peak.max(dot(u, u));
            near |= self.x[3 * i + 1] <= self.ground + 2e-4;
        }
        let quiet = near && self.time >= self.launch_duration && energy / self.total_mass < 1e-4 && peak < 1e-2;
        self.quiet_time = if quiet { self.quiet_time + h } else { 0.0 };
        if let Some(touchdown) = &self.touchdown {
            if touched && self.quiet_time >= 0.5 { self.status = Status::Landed; }
            else if self.time - touchdown.time >= 8.0 { self.status = Status::Finished; }
            else if self.options.after_touchdown >= 0.0 && self.time - touchdown.time >= self.options.after_touchdown { self.status = Status::Stopped; }
        } else if self.time >= self.options.max_time.min(12.0) - 1e-9 { self.status = Status::Finished; }
        if self.x.iter().any(|c| !(c.abs() <= 1000.0)) { self.status = Status::Error; }
    }

    /// Advance one 1/120 s frame and refresh the diagnostics.
    pub fn frame(&mut self) {
        for _ in 0..self.options.steps_per_frame { self.step(); if !self.active() { break; } }
        self.measure_energy();
        self.peak_deformation = self.peak_deformation.max(self.deformation());
        self.peak_strain = self.peak_strain.max(self.max_strain());
    }
    pub fn body_frame(&self, p: &dyn Fn(usize) -> V3) -> (V3, V3, V3, V3) {
        let origin = p(self.tail);
        let x = unit(sub(p(self.nose), origin));
        let y = unit(cross(sub(p(self.right), p(self.left)), x));
        (origin, x, y, unit(cross(x, y)))
    }
    /// RMS shape change after removing the landmark body frame (the game's deformation measure).
    pub fn deformation(&self) -> f64 {
        let (o0, x0, y0, z0) = self.body_frame(&|i| self.rest[i]);
        let (o1, x1, y1, z1) = self.body_frame(&|i| get(&self.x, i));
        let mut sum = 0.0;
        for i in 0..self.n {
            let (a, b) = (sub(get(&self.x, i), o1), sub(self.rest[i], o0));
            let d = [dot(a, x1) - dot(b, x0), dot(a, y1) - dot(b, y0), dot(a, z1) - dot(b, z0)];
            sum += dot(d, d);
        }
        (sum / self.n as f64).sqrt()
    }
    pub fn max_strain(&self) -> f64 {
        self.edges.iter().map(|e| (norm(sub(get(&self.x, e.a), get(&self.x, e.b))) / e.rest - 1.0).abs()).fold(0.0, f64::max)
    }
    /// Total energy now, less the starting energy and all accounted work and losses. What is left
    /// is the integrator's own dissipation plus contact, friction and plastic losses.
    pub fn energy_residual(&self) -> f64 {
        let e = &self.energy;
        e.kinetic + e.elastic + e.gravity - self.energy_start - e.air_work - e.launch_work + e.damping_loss
    }
}
