//! Paper Flight's real-time flight solver, compiled to WebAssembly. `Simulation`
//! (src/physics/simulation.ts) compiles the mesh and every piece of derived set-up in
//! TypeScript, passes them in as flat arrays, and advances the flight one substep at a
//! time through `sim_advance`.
//!
//! Plain C ABI: no wasm-bindgen. Buffers are staged with `stage_f64` / `stage_i32`, then
//! `sim_create` consumes them.
#![allow(clippy::missing_safety_doc)]

mod aero;
mod elastic;
mod fast;
mod ground;
mod js;
mod kernels;
mod lanes;
mod layer;
mod selfcontact;
mod sweeps;

use aero::{Aero, SurfaceSetup, Weather};
use elastic::{ElasticSetup, ElasticSolver};
use ground::{Ground, GROUND_HEIGHT};
use layer::{LayerSetup, Layers};
use selfcontact::{SelfContact, Tree};
use std::f64::consts::PI;
use sweeps::{EdgeBatch, HingeBatch, LayerTies, MembraneBatch};

const FIXED_DT: f64 = 1.0 / 120.0;
/// Substeps per frame in steady flight; launch, floor contact and light sheets take more.
const SUBSTEPS: f64 = 3.0;
// Work-saving schedule. Each item trades exactness for time where the difference was
// measured to be within run-to-run variation.
/// Scan every layer candidate on alternate frames; substeps between revisit the near list.
const LAYER_SCAN_FRAMES: u64 = 2;
/// A contact hovering this close (m) on its own side stays active into the next substep.
/// Tighter once on the floor, where lingering contacts delay settling.
const CARRY_GAP: f64 = 1e-4;
const RESTING_CARRY_GAP: f64 = 1e-5;
/// Below this many broad-phase pairs a second, early swept detection is cheap enough to keep.
const EARLY_DETECTION_PAIRS: usize = 12000;
/// Fewest constraint sweeps per substep when extra substeps share a frame's sweeps.
const MIN_SWEEPS: f64 = 18.0;
/// Substeps for a frame in which a vertex moves faster than 0.5 m/s near the floor.
const MOVING_CONTACT_SUBSTEPS: f64 = 8.0;
/// Floor-contact cleanup passes before the first strain-convergence check (then every fourth).
const MIN_CLEANUP_PASSES: usize = 4;

// ---- Timing of solver stages (optional; uses a host clock) ----
// The import is named in full: current toolchains refuse to link a WebAssembly module with
// symbols left undefined.
#[link(wasm_import_module = "env")]
extern "C" {
    /// Host clock in milliseconds (performance.now), only called while profiling.
    fn now() -> f64;
}
static mut PROFILING: bool = false;
pub const STAGES: usize = 11;
const STATE: usize = 27;
const S_SCHEDULE: usize = 0; // substep count, compliances, prediction, state sync
const S_AIR: usize = 1; // air loads
const S_ELASTIC: usize = 2; // direct elastic solve
const S_SWEEPS: usize = 3; // membrane, hinge and strain-limit sweeps
const S_LAYER_DETECT: usize = 4;
const S_LAYER_SOLVE: usize = 5;
const S_SELF_DETECT: usize = 6; // carry and swept detection, broad and narrow phase
const S_SELF_SOLVE: usize = 7; // self-contact projection, in sweeps and cleanup
const S_GROUND: usize = 8; // floor contact, position and velocity
const S_CLEANUP: usize = 9; // strain passes of the floor-contact cleanup
const S_POST: usize = 10; // velocity update, edge damping, plasticity, status
/// Host time in ms while profiling, else None.
#[inline(always)]
pub(crate) fn profile_now() -> Option<f64> { if unsafe { PROFILING } { Some(unsafe { now() }) } else { None } }
struct Clock { t: f64, on: bool }
impl Clock {
    #[inline(always)]
    fn start() -> Clock {
        let on = unsafe { PROFILING };
        Clock { t: if on { unsafe { now() } } else { 0.0 }, on }
    }
    #[inline(always)]
    fn lap(&mut self, prof: &mut [f64; STAGES], stage: usize) {
        if self.on {
            let t = unsafe { now() };
            prof[stage] += t - self.t;
            self.t = t;
        }
    }
}

/// Ideal spring relaxation, x/x0 = cos(omega t): the velocity an elastic launcher has
/// contributed by `time` in the absence of other loads.
pub(crate) fn spring_speed(time: f64, speed: f64, duration: f64) -> f64 {
    if time <= 0.0 { return 0.0; }
    if duration <= 0.0 || time >= duration { return speed; }
    speed * (time / duration * PI / 2.0).sin()
}
/// Exact impulse per unit mass across a step, including detachment. A zero duration is the
/// initial-velocity launch, which gives the sheet its speed before the first step.
pub(crate) fn spring_velocity_delta(time: f64, dt: f64, speed: f64, duration: f64) -> f64 {
    if duration <= 0.0 || dt <= 0.0 || time >= duration { return 0.0; }
    spring_speed(time + dt, speed, duration) - spring_speed(time, speed, duration)
}

/// Signed dihedral (geometry/dart.ts hingeAngle).
fn hinge_angle(p: &[f64], ids: &[i32]) -> f64 {
    let v = |i: usize| { let k = 3 * ids[i] as usize; [p[k], p[k + 1], p[k + 2]] };
    let (p0, p1, p2, p3) = (v(0), v(1), v(2), v(3));
    let sub = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let cross = |a: [f64; 3], b: [f64; 3]| [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    let e = sub(p3, p2);
    let n1 = cross(sub(p2, p0), sub(p3, p0));
    let n2 = cross(sub(p3, p1), sub(p2, p1));
    let s = cross(n1, n2);
    ((e[0] * s[0] + e[1] * s[1] + e[2] * s[2]) / js::max(js::hypot3(e[0], e[1], e[2]), 1e-12)).atan2(n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2])
}

/// Rate-independent return mapping with linear isotropic hardening for one crease: the
/// new permanent offset and accumulated plastic rotation, if the hinge yields at `angle`.
pub(crate) fn crease_yield(angle: f64, rest: f64, offset: f64, accumulated: f64, yield_angle: f64, hardening: f64) -> Option<(f64, f64)> {
    let error = js::wrap_angle(angle - rest - offset);
    let yield_now = yield_angle + hardening * accumulated;
    if error.abs() > yield_now {
        let increment = (error.abs() - yield_now) / (1.0 + hardening);
        Some((offset + js::sign(error) * increment, accumulated + increment))
    } else {
        None
    }
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
enum Status { Flying = 1, Settling = 2, Landed = 3, Finished = 4, Error = 5 }

struct Params {
    speed: f64, launch_duration: f64, gravity: f64, density: f64, aero: bool, substeps: f64, iterations: f64,
    newton_iterations: usize, contact_sweep_share: f64, paper_weight: f64, heading: f64, elevation: f64,
    thickness: f64, young_md: f64, young_cd: f64, shear: f64, friction: f64, yield_angle: f64, hardening: f64,
    direct: bool,
    /// Sliding layer contact; otherwise the legacy bonded laminate held by layer ties.
    contact: bool,
    /// The uncoupled per-face air loads instead of the lifting surface.
    panels: bool,
}

/// One 1/120 s frame under way.
struct Frame {
    substeps: usize,
    substep: usize,
    dt: f64,
    aero_stride: usize,
    iterations: f64,
    contact_substeps: f64,
    index: u64,
    direction: [f64; 3],
}

pub struct Sim {
    n: usize,
    p: Params,
    weather: Weather,
    positions: Vec<f64>,
    velocities: Vec<f64>,
    forces: Vec<f64>,
    masses: Vec<f64>,
    total_mass: f64,
    inv_mass: Vec<f64>,
    tributary: Vec<f64>,
    area_weights: Vec<f64>,
    ties: LayerTies,
    pending: Option<Frame>,
    /// Solver working state for the substep under way. `positions` and `velocities` stay
    /// authoritative between substeps, so readers and external edits see a consistent state.
    x: Vec<f64>,
    x_old: Vec<f64>,
    v: Vec<f64>,
    w: Vec<f64>,
    air: Vec<f64>,
    sweep_start: Vec<f64>,
    hinges: HingeBatch,
    hinge_k: Vec<f64>,
    hinge_crease: Vec<u8>,
    plastic_offset: Vec<f64>,
    plastic_accumulated: Vec<f64>,
    edges: EdgeBatch,
    membranes: MembraneBatch,
    membrane_area: Vec<f64>,
    elastic: ElasticSolver,
    aero: Aero,
    layers: Layers,
    selfc: SelfContact,
    ground: Ground,
    status: Status,
    time: f64,
    frame: u64,
    quiet_time: f64,
    start: [f64; 3],
    last_center: [f64; 3],
    path_length: f64,
    ground_distance: f64,
    touchdown: Option<(f64, [f64; 3], f64)>,
    /// Exported state: see `sim_state`.
    state: [f64; STATE],
    prof: [f64; STAGES],
    frames_stepped: f64,
}

impl Sim {
    fn active(&self) -> bool { self.status == Status::Flying || self.status == Status::Settling }
    fn ground_height(&self) -> f64 { GROUND_HEIGHT + self.p.thickness / 2.0 }

    fn center(&self) -> [f64; 3] {
        let mut c = [0.0; 3];
        for i in 0..self.n {
            for d in 0..3 { c[d] += self.positions[3 * i + d] * self.masses[i] / self.total_mass; }
        }
        c
    }

    /// The schedule of one 1/120 s frame, fixed from the state at its start.
    fn begin_frame(&mut self) -> Frame {
        let n = self.n;
        let p = &self.p;
        let launch_delta = spring_velocity_delta(self.time, FIXED_DT, p.speed, p.launch_duration).abs();
        // Very light sheets accelerate much faster under panel loads. Resolve them with
        // smaller substeps while keeping the recorded 120 Hz clock unchanged. Explicit
        // aerodynamic damping rate is ~ rho V pi A/m for a 2pi slope. Resolve the fastest
        // vertex mode; extra substeps preserve the 120 Hz outer clock.
        let mut rate: f64 = 0.0;
        if p.aero {
            for i in 0..n {
                let k = 3 * i;
                let air = self.weather.air_velocity(self.time, [self.positions[k], self.positions[k + 1], self.positions[k + 2]]);
                // Include the upcoming launcher impulse when estimating the fastest mode.
                let speed = js::hypot3(self.velocities[k] - air[0], self.velocities[k + 1] - air[1], self.velocities[k + 2] - air[2]) + launch_delta;
                rate = js::max(rate, p.density * PI * speed * self.tributary[i] * self.inv_mass[i]);
            }
        }
        // Resolve at most one degree of spring phase per substep while loaded. Low initial
        // airspeed alone underestimates the resolution needed as the load grows and coincident
        // folded layers first open in crosswind.
        let launch_substeps = if p.speed != 0.0 && self.time < p.launch_duration { (90.0 * FIXED_DT / p.launch_duration).ceil() } else { 0.0 };
        let ground_height = self.ground_height();
        // Resolve near-floor motion in roughly 2 mm increments, including lateral motion that
        // can tip a contacting nose/wing. Explicit substeps still win.
        let mut contact_substeps: f64 = 0.0;
        for i in 0..n {
            let k = 3 * i;
            let speed = js::hypot3(self.velocities[k], self.velocities[k + 1], self.velocities[k + 2]) + launch_delta;
            if self.positions[k + 1] - ground_height < 0.03 + speed * FIXED_DT {
                let moving = if speed > 0.5 { MOVING_CONTACT_SUBSTEPS } else { SUBSTEPS };
                contact_substeps = js::max(js::max(contact_substeps, moving), (FIXED_DT * (speed + p.gravity * FIXED_DT) / 0.002).ceil());
            }
        }
        let base_substeps = if p.paper_weight < 30.0 { SUBSTEPS * 3.0 } else { SUBSTEPS };
        let minimum = js::max(js::max(base_substeps, launch_substeps), contact_substeps);
        let aero_substeps = js::max(1.0, (FIXED_DT * rate / 0.8).ceil());
        let explicit = p.substeps != 0.0 && !p.substeps.is_nan();
        let substeps_f = if explicit { p.substeps } else { js::min(96.0, js::max(minimum, aero_substeps)) };
        let dt = FIXED_DT / substeps_f;
        let aero_stride = js::max(1.0, (substeps_f / aero_substeps).floor()) as usize;
        let index = self.frame;
        self.frame += 1;
        // Constraint sweeps converge faster per sweep at a smaller timestep. When floor
        // contact, the launch stroke or air loads raise the substep count above the sheet's
        // usual one, share that frame's sweeps between them (down to MIN_SWEEPS each) instead
        // of multiplying the work.
        let requested = p.iterations;
        let iterations = if !explicit && substeps_f > base_substeps { js::min(requested, js::max(MIN_SWEEPS, (requested * base_substeps / substeps_f).ceil())) } else { requested };
        self.w.copy_from_slice(&self.inv_mass);
        // Compliance over dt² of each hinge and of each membrane mode (modulus × thickness ×
        // area), for this frame's dt.
        for j in 0..self.hinge_k.len() {
            self.hinges.alpha[j] = 1.0 / js::max(self.hinge_k[j], 1e-8) / (dt * dt);
        }
        for j in 0..self.membrane_area.len() {
            let scale = p.thickness * self.membrane_area[j] * dt * dt;
            self.membranes.alpha[3 * j] = 1.0 / (p.young_md * scale);
            self.membranes.alpha[3 * j + 1] = 1.0 / (p.young_cd * scale);
            self.membranes.alpha[3 * j + 2] = 1.0 / (p.shear * scale);
        }
        let yaw = p.heading * PI / 180.0;
        let e = p.elevation * PI / 180.0;
        let direction = [e.cos() * yaw.cos(), e.sin(), e.cos() * yaw.sin()];
        // An explicit substep count may be fractional: dt divides the frame by it, and the
        // frame runs its ceiling.
        Frame { substeps: substeps_f.ceil() as usize, substep: 0, dt, aero_stride, iterations, contact_substeps, index, direction }
    }

    /// One substep of the frame. True when the flight ended in it.
    fn substep(&mut self, f: &Frame) -> bool {
        let mut clock = Clock::start();
        let mut prof = self.prof;
        let n = self.n;
        let p = &self.p;
        let (dt, substep) = (f.dt, f.substep);
        let ground_height = self.ground_height();
        let gravity = p.gravity;
        let (direct_mode, contact, newton_iterations, sweep_share, friction) = (p.direct, p.contact, p.newton_iterations, p.contact_sweep_share, p.friction);
        let (speed_param, duration) = (p.speed, p.launch_duration);
        let (density, aero_enabled, panels_model) = (p.density, p.aero, p.panels);
        let (yield_angle, hardening) = (p.yield_angle, p.hardening);
        self.x.copy_from_slice(&self.positions);
        self.v.copy_from_slice(&self.velocities);
        self.x_old.copy_from_slice(&self.x);
        clock.lap(&mut prof, S_SCHEDULE);
        // Air loads are sampled as often as their explicit stability needs, and held between
        // samples; structure and contact still advance every substep.
        if substep % f.aero_stride == 0 {
            self.forces.fill(0.0);
            // Layer shielding is refreshed once per frame.
            if substep == 0 { self.aero.exposure_valid = false; }
            if panels_model {
                aero::panel_loads(&self.positions, &self.velocities, &self.membranes.faces, &self.area_weights, &self.weather, self.time, aero_enabled, density, &mut self.forces, &mut self.aero.panels);
            } else {
                self.aero.loads(&self.positions, &self.velocities, &self.weather, self.time, aero_enabled, density, &mut self.forces);
            }
            self.air.copy_from_slice(&self.forces);
            clock.lap(&mut prof, S_AIR);
        }
        for i in 0..n {
            let k = 3 * i;
            self.forces[k] = self.air[k];
            self.forces[k + 1] = self.air[k + 1] - gravity * self.masses[i];
            self.forces[k + 2] = self.air[k + 2];
        }
        // Distribute the launcher load by mass: a supported launch with no added torque or
        // artificial vertex-to-vertex velocity discontinuity. Local hook/contact stresses are
        // outside this ideal acceleration model.
        let acceleration = spring_velocity_delta(self.time, dt, speed_param, duration) / dt;
        if acceleration != 0.0 && !acceleration.is_nan() {
            for i in 0..n {
                for d in 0..3 { self.forces[3 * i + d] += self.masses[i] * acceleration * f.direction[d]; }
            }
        }
        for i in 0..n {
            let k = 3 * i;
            let weight = self.w[i];
            self.v[k] += dt * self.forces[k] * weight;
            self.v[k + 1] += dt * self.forces[k + 1] * weight;
            self.v[k + 2] += dt * self.forces[k + 2] * weight;
            self.x[k] += dt * self.v[k];
            self.x[k + 1] += dt * self.v[k + 1];
            self.x[k + 2] += dt * self.v[k + 2];
        }
        for j in 0..self.plastic_offset.len() { self.hinges.set_offset(j, self.plastic_offset[j]); }
        clock.lap(&mut prof, S_SCHEDULE);
        // Elastic response first, exactly; the sweeps below then start from positions and
        // multipliers that satisfy every membrane and hinge, and reconcile contact.
        // Near the floor the sheet is held bent by contact, where Newton on the elastic
        // energy alone converges slowly and the sweeps are the better tool.
        let solved = direct_mode && f.contact_substeps == 0.0 && self.elastic.solve(&mut self.x, &self.masses, dt, &mut self.membranes, &mut self.hinges, newton_iterations, 1e-6);
        if !solved { self.membranes.lambda.fill(0.0); self.hinges.lambda.fill(0.0); }
        clock.lap(&mut prof, S_ELASTIC);
        if contact { self.layers.detect(&self.x, &self.x_old, substep == 0 && f.index % LAYER_SCAN_FRAMES == 0); } else { self.layers.count = 0; }
        clock.lap(&mut prof, S_LAYER_DETECT);
        self.selfc.begin_step();
        // Contacts that still touch, or hover just apart, stay active from the previous
        // substep: every sweep enforces them without re-detection, and resting stacks do not
        // chatter through repeated re-impact.
        if contact { self.selfc.carry(&self.x_old, if self.touchdown.is_some() { RESTING_CARRY_GAP } else { CARRY_GAP }); }
        clock.lap(&mut prof, S_SELF_DETECT);
        self.ground.reset();
        clock.lap(&mut prof, S_GROUND);
        // Small meshes can afford the early swept detection as well.
        let early = contact && self.selfc.pair_count() <= EARLY_DETECTION_PAIRS;
        // After a direct elastic solve the sweeps only act where contact, the floor or the
        // strain envelope does. Once a sweep moves nothing, one closing sweep (which runs the
        // last swept detection) ends the loop.
        let direct = solved;
        let mut settled = false;
        let sweeps = if direct { js::max(2.0, (f.iterations * sweep_share).ceil()) } else { f.iterations };
        let mut iteration = 0usize;
        // `iterations` may be fractional or zero, so the sweep count is compared as a float.
        while (iteration as f64) < sweeps {
            // Alternating sweep direction reduces directional Gauss-Seidel bias.
            let forward = iteration % 2 == 0;
            let closing = settled || iteration as f64 == sweeps - 1.0;
            if direct && !closing { self.sweep_start.copy_from_slice(&self.x); }
            sweeps::solve_membranes(&mut self.x, &self.w, &mut self.membranes, forward);
            sweeps::solve_hinges(&mut self.x, &self.w, &mut self.hinges, forward);
            if !contact { sweeps::solve_layer_ties(&mut self.x, &self.w, &self.ties); }
            sweeps::limit_edge_strains(&mut self.x, &self.w, &self.edges, forward, 0.02);
            clock.lap(&mut prof, S_SWEEPS);
            self.layers.solve(&mut self.x, &self.w, friction, iteration);
            clock.lap(&mut prof, S_LAYER_SOLVE);
            if contact {
                // Include structural projection in the sweep, not only free motion.
                // Contact is last so the strain limiter cannot silently undo it.
                // Swept detection on the last sweep adds every fresh crossing.
                if (early && iteration == 0) || closing { self.selfc.detect(&self.x_old, &self.x); clock.lap(&mut prof, S_SELF_DETECT); }
                self.selfc.solve(&mut self.x, &self.w, forward, iteration);
                clock.lap(&mut prof, S_SELF_SOLVE);
            }
            self.ground.solve(&mut self.x, &self.x_old, &self.w, ground_height);
            clock.lap(&mut prof, S_GROUND);
            if closing { break; }
            if direct {
                let mut moved: f64 = 0.0;
                for k in 0..3 * n {
                    let change = (self.x[k] - self.sweep_start[k]).abs();
                    if change > moved { moved = change; }
                }
                settled = moved < 1e-10;
                clock.lap(&mut prof, S_SWEEPS);
            }
            iteration += 1;
        }
        let ground_cleanup = self.ground.touched;
        let passes = if ground_cleanup { 32 } else if contact { 4 } else { 0 };
        for pass in 0..passes {
            // A pinned nose can concentrate compression into a few very short edges.
            // Reconcile the no-tearing envelope with both contact systems.
            if ground_cleanup { sweeps::limit_edge_strains(&mut self.x, &self.w, &self.edges, pass % 2 == 0, 0.02); clock.lap(&mut prof, S_CLEANUP); }
            if contact { self.selfc.solve(&mut self.x, &self.w, pass % 2 == 0, 0); }
            clock.lap(&mut prof, S_SELF_SOLVE);
            self.ground.solve(&mut self.x, &self.x_old, &self.w, ground_height);
            clock.lap(&mut prof, S_GROUND);
            // Most contacts converge within eight sweeps. Small triangles in dense folds need
            // extra reconciliation at impact, not a looser strain bound.
            if ground_cleanup && pass + 1 >= MIN_CLEANUP_PASSES && (pass + 1) % 4 == 0 && sweeps::edge_strains_within(&self.x, &self.edges, 0.05) { clock.lap(&mut prof, S_CLEANUP); break; }
            if ground_cleanup { clock.lap(&mut prof, S_CLEANUP); }
        }
        for k in 0..3 * n { self.v[k] = (self.x[k] - self.x_old[k]) / dt; }
        sweeps::damp_edges(&self.x, &mut self.v, &self.w, &self.edges, 1.0 - (-18.0 * dt).exp());
        clock.lap(&mut prof, S_POST);
        let supported = self.ground.solve_velocity(&self.x, &mut self.v, ground_height);
        clock.lap(&mut prof, S_GROUND);
        let mut finite = true;
        for k in 0..3 * n { if !(self.x[k].abs() <= 1000.0) { finite = false; break; } }
        if finite {
            self.positions.copy_from_slice(&self.x);
            self.velocities.copy_from_slice(&self.v);
        } else {
            self.positions.copy_from_slice(&self.x_old);
            self.velocities.fill(0.0);
        }
        if finite {
            for j in 0..self.hinge_crease.len() {
                if self.hinge_crease[j] == 0 { continue; }
                let angle = hinge_angle(&self.positions, &self.hinges.ids[4 * j..4 * j + 4]);
                if let Some((offset, accumulated)) = crease_yield(angle, self.hinges.rest[j], self.plastic_offset[j], self.plastic_accumulated[j], yield_angle, hardening) {
                    self.plastic_offset[j] = offset;
                    self.plastic_accumulated[j] = accumulated;
                }
            }
        }
        self.time += dt;
        if !finite { self.status = Status::Error; clock.lap(&mut prof, S_POST); self.prof = prof; return true; }
        let center = self.center();
        let lc = self.last_center;
        self.path_length += js::hypot3(center[0] - lc[0], center[1] - lc[1], center[2] - lc[2]);
        if self.touchdown.is_some() { self.ground_distance += js::hypot2(center[0] - lc[0], center[2] - lc[2]); }
        self.last_center = center;
        if self.touchdown.is_none() && self.ground.touched {
            let distance = js::hypot2(center[0] - self.start[0], center[2] - self.start[2]);
            self.touchdown = Some((self.time, center, distance));
            self.status = Status::Settling;
        }
        // Mass-weighted world-space kinetic energy includes rocking and flutter. A tiny layer
        // vertex may jitter while the sheet is otherwise at rest; an additional peak-speed
        // limit prevents sleeping a visibly moving tip.
        let (mut speed_squared, mut mass, mut peak_speed_squared) = (0.0, 0.0, 0.0);
        for i in 0..n {
            let k = 3 * i;
            let (a, b, c) = (self.velocities[k], self.velocities[k + 1], self.velocities[k + 2]);
            let sq = a * a + b * b + c * c;
            let m = self.masses[i];
            speed_squared += m * sq;
            mass += m;
            peak_speed_squared = js::max(peak_speed_squared, sq);
        }
        let mut near_ground = false;
        for i in 0..n { if self.positions[3 * i + 1] <= ground_height + 0.0002 { near_ground = true; break; } }
        self.quiet_time = if near_ground && self.time >= duration && speed_squared / mass < 0.01 * 0.01 && peak_speed_squared < 0.1 * 0.1 { self.quiet_time + dt } else { 0.0 };
        clock.lap(&mut prof, S_POST);
        self.prof = prof;
        // Allow submillimetre contact chatter during the dwell, but finish only on a substep
        // with actual compressive support from the floor.
        if self.touchdown.is_some() && supported && self.quiet_time >= 0.5 {
            self.velocities.fill(0.0);
            self.status = Status::Landed;
            return true;
        }
        // A gust can keep a light sheet moving. Report a timeout honestly, and allow a late
        // touchdown its own full settling window.
        let timeout = match self.touchdown { Some((t, _, _)) => self.time - t >= 8.0, None => self.time >= 12.0 };
        if timeout { self.status = Status::Finished; return true; }
        false
    }

    /// Advance one substep, starting a frame if none is under way. True when that
    /// completes a 1/120 s frame (or the flight).
    fn advance(&mut self) -> bool {
        if self.pending.is_none() {
            if !self.active() { return true; }
            self.pending = Some(self.begin_frame());
        }
        let mut f = self.pending.take().unwrap();
        let ended = self.substep(&f);
        f.substep += 1;
        if ended || f.substep >= f.substeps { self.frames_stepped += 1.0; return true; }
        self.pending = Some(f);
        false
    }

    fn publish(&mut self) {
        let s = &mut self.state;
        s[0] = self.time;
        s[1] = self.status as u8 as f64;
        match self.touchdown {
            Some((t, c, d)) => { s[2] = 1.0; s[3] = t; s[4] = c[0]; s[5] = c[1]; s[6] = c[2]; s[7] = d; }
            None => { s[2] = 0.0; }
        }
        s[8] = self.ground_distance;
        s[9] = self.path_length;
        s[10] = self.last_center[0]; s[11] = self.last_center[1]; s[12] = self.last_center[2];
        s[13] = self.frames_stepped;
        s[14] = self.selfc.stats[0]; s[15] = self.selfc.stats[1]; s[16] = self.selfc.stats[2];
        s[17] = self.selfc.count as f64;
        s[18] = self.layers.count as f64;
        s[19] = self.selfc.pair_count() as f64;
        s[20] = self.aero.panels.len() as f64 / aero::PANEL as f64;
        s[21] = if self.ground.touched { 1.0 } else { 0.0 };
        s[22] = match &self.pending { Some(f) => f.substep as f64, None => 0.0 };
        s[23] = match &self.pending { Some(f) => f.substeps as f64, None => 0.0 };
        // Self-contact detection split (ms, while profiling): set-up and drift check,
        // pair-list rebuilds, narrow phase over the listed pairs.
        s[24] = self.selfc.timing[0]; s[25] = self.selfc.timing[1]; s[26] = self.selfc.timing[2];
    }
}

// ---- C ABI ----

const F64_FIELDS: usize = 32;
const I32_FIELDS: usize = 40;
static mut STAGED_F64: Vec<Vec<f64>> = Vec::new();
static mut STAGED_I32: Vec<Vec<i32>> = Vec::new();

#[allow(static_mut_refs)]
pub(crate) unsafe fn staged_f64() -> &'static mut Vec<Vec<f64>> {
    if STAGED_F64.is_empty() { STAGED_F64 = (0..F64_FIELDS).map(|_| Vec::new()).collect(); }
    &mut STAGED_F64
}
#[allow(static_mut_refs)]
pub(crate) unsafe fn staged_i32() -> &'static mut Vec<Vec<i32>> {
    if STAGED_I32.is_empty() { STAGED_I32 = (0..I32_FIELDS).map(|_| Vec::new()).collect(); }
    &mut STAGED_I32
}

/// Allocate staging space for f64 field `field` of `len` values; JS writes into it.
#[no_mangle]
pub unsafe extern "C" fn stage_f64(field: usize, len: usize) -> *mut f64 {
    let s = staged_f64();
    s[field] = vec![0.0; len];
    s[field].as_mut_ptr()
}
#[no_mangle]
pub unsafe extern "C" fn stage_i32(field: usize, len: usize) -> *mut i32 {
    let s = staged_i32();
    s[field] = vec![0; len];
    s[field].as_mut_ptr()
}

pub(crate) mod field {
    // f64
    pub const SCALARS: usize = 0;
    pub const POSITIONS: usize = 1;
    pub const VELOCITIES: usize = 2;
    pub const MASSES: usize = 3;
    pub const TRIBUTARY: usize = 4;
    pub const MEMBRANE_MD: usize = 5;
    pub const MEMBRANE_CD: usize = 6;
    pub const MEMBRANE_AREA: usize = 7;
    pub const HINGE_REST: usize = 8;
    pub const HINGE_K: usize = 9;
    pub const EDGE_REST: usize = 10;
    // 11 is free.
    pub const BOUNDARIES: usize = 12;
    pub const PATCH_WEIGHTS: usize = 13;
    pub const PATCH_FRACTION: usize = 14;
    pub const FACE_SIGNS: usize = 15;
    pub const LAYER_ORIENTATION: usize = 16;
    pub const LAYER_SIGN: usize = 17;
    pub const LAYER_SIDE: usize = 18;
    pub const AREA_WEIGHTS: usize = 19;
    pub const TIE_WEIGHTS: usize = 20;
    // i32
    pub const FACES: usize = 0;
    pub const HINGE_IDS: usize = 1;
    pub const HINGE_CREASE: usize = 2;
    pub const EDGE_IDS: usize = 3;
    pub const PATCH_FACE: usize = 4;
    pub const PATCH_STATION: usize = 5;
    pub const PATCH_PANEL: usize = 6;
    pub const FACE_OWNERS: usize = 7;
    pub const PERM: usize = 8;
    pub const AP: usize = 9;
    pub const AI: usize = 10;
    pub const SOURCE: usize = 11;
    // 12 and 13 are free.
    pub const DIAGONAL: usize = 14;
    pub const FACE_SLOTS: usize = 15;
    pub const HINGE_SLOTS: usize = 16;
    pub const LAYER_VERTEX: usize = 17;
    pub const LAYER_FACE_INDEX: usize = 18;
    pub const LAYER_FACES: usize = 19;
    pub const LAYER_AMBIGUOUS: usize = 20;
    pub const LAYER_SIDE_INDEX: usize = 21;
    pub const LAYER_CHOSEN: usize = 22;
    pub const NEIGHBORS: usize = 23;
    pub const SELF_EDGES: usize = 24;
    pub const FACE_TREE_LEAVES: usize = 25;
    pub const FACE_TREE_RIGHT: usize = 26;
    pub const FACE_TREE_ESCAPE: usize = 27;
    pub const EDGE_TREE_LEAVES: usize = 28;
    pub const EDGE_TREE_RIGHT: usize = 29;
    pub const EDGE_TREE_ESCAPE: usize = 30;
    pub const LANDMARKS: usize = 31;
    pub const TIE_IDS: usize = 32;
}

/// Build a flight from the staged buffers (which are consumed): the state of a launched
/// TypeScript `Simulation`.
#[no_mangle]
pub unsafe extern "C" fn sim_create() -> *mut Sim {
    let f = staged_f64();
    let i = staged_i32();
    let mut take_f = |k: usize| std::mem::take(&mut f[k]);
    let s = take_f(field::SCALARS);
    let positions = take_f(field::POSITIONS);
    let velocities = take_f(field::VELOCITIES);
    let masses = take_f(field::MASSES);
    let tributary = take_f(field::TRIBUTARY);
    let md = take_f(field::MEMBRANE_MD);
    let cd = take_f(field::MEMBRANE_CD);
    let membrane_area = take_f(field::MEMBRANE_AREA);
    let hinge_rest = take_f(field::HINGE_REST);
    let hinge_k = take_f(field::HINGE_K);
    let edge_rest = take_f(field::EDGE_REST);
    let boundaries = take_f(field::BOUNDARIES);
    let patch_weights = take_f(field::PATCH_WEIGHTS);
    let patch_fraction = take_f(field::PATCH_FRACTION);
    let face_signs = take_f(field::FACE_SIGNS);
    let layer_orientation = take_f(field::LAYER_ORIENTATION);
    let layer_sign = take_f(field::LAYER_SIGN);
    let layer_side = take_f(field::LAYER_SIDE);
    let area_weights = take_f(field::AREA_WEIGHTS);
    let tie_weights = take_f(field::TIE_WEIGHTS);
    let mut take_i = |k: usize| std::mem::take(&mut i[k]);
    let faces = take_i(field::FACES);
    let hinge_ids = take_i(field::HINGE_IDS);
    let hinge_crease: Vec<u8> = take_i(field::HINGE_CREASE).iter().map(|&c| c as u8).collect();
    let edge_ids = take_i(field::EDGE_IDS);
    let patch_face = take_i(field::PATCH_FACE);
    let patch_station = take_i(field::PATCH_STATION);
    let patch_panel = take_i(field::PATCH_PANEL);
    let face_owners = take_i(field::FACE_OWNERS);
    let elastic_setup = ElasticSetup {
        perm: take_i(field::PERM), ap: take_i(field::AP), ai: take_i(field::AI), source: take_i(field::SOURCE),
        diagonal: take_i(field::DIAGONAL), face_slots: take_i(field::FACE_SLOTS), hinge_slots: take_i(field::HINGE_SLOTS),
        block_len: s[30] as usize,
    };
    let layer_setup = LayerSetup {
        vertex: take_i(field::LAYER_VERTEX), face_index: take_i(field::LAYER_FACE_INDEX), faces: take_i(field::LAYER_FACES),
        orientation: layer_orientation, ambiguous: take_i(field::LAYER_AMBIGUOUS), side_index: take_i(field::LAYER_SIDE_INDEX),
        sign: layer_sign, chosen: take_i(field::LAYER_CHOSEN), side: layer_side,
    };
    let neighbors: Vec<u32> = take_i(field::NEIGHBORS).iter().map(|&v| v as u32).collect();
    let self_edges = take_i(field::SELF_EDGES);
    let tree = |l: Vec<i32>, r: Vec<i32>, e: Vec<i32>| if l.is_empty() { None } else { Some(Tree::new(l, r, e)) };
    let face_tree = tree(take_i(field::FACE_TREE_LEAVES), take_i(field::FACE_TREE_RIGHT), take_i(field::FACE_TREE_ESCAPE));
    let edge_tree = tree(take_i(field::EDGE_TREE_LEAVES), take_i(field::EDGE_TREE_RIGHT), take_i(field::EDGE_TREE_ESCAPE));
    let lm = take_i(field::LANDMARKS);
    let landmarks = [lm[0] as usize, lm[1] as usize, lm[2] as usize, lm[3] as usize];
    let tie_ids = take_i(field::TIE_IDS);

    let n = masses.len();
    let hinge_count = hinge_rest.len();
    let face_count = faces.len() / 3;
    let p = Params {
        speed: s[0], launch_duration: s[1], gravity: s[2], density: s[3], aero: s[4] != 0.0, substeps: s[5], iterations: s[6],
        newton_iterations: s[7] as usize, contact_sweep_share: s[8], paper_weight: s[9], heading: s[10], elevation: s[11],
        thickness: s[20], young_md: s[21], young_cd: s[22], shear: s[23], friction: s[24], yield_angle: s[25], hardening: s[26],
        direct: s[32] != 0.0, contact: s[33] != 0.0, panels: s[34] != 0.0,
    };
    let weather = Weather { wind: [s[12], s[13], s[14]], wind_heading: s[15], gusts: s[16], turbulence: s[17], feature: s[42] as u8 };
    let surface = SurfaceSetup {
        faces: faces.clone(),
        owners: if s[31] != 0.0 { Some(face_owners) } else { None },
        signs: face_signs, boundaries, span: s[28], chord_panels: s[29] as usize,
        patch_face, patch_station, patch_panel, patch_weights, patch_fraction, landmarks, shielding: s[27],
    };
    let total_mass = masses.iter().fold(0.0, |a, b| a + b);
    let inv_mass = masses.iter().map(|m| 1.0 / m).collect();
    let mut sim = Box::new(Sim {
        n,
        weather,
        forces: vec![0.0; 3 * n],
        total_mass,
        inv_mass,
        tributary,
        area_weights,
        ties: LayerTies { ids: tie_ids, weights: tie_weights },
        pending: None,
        x: vec![0.0; 3 * n],
        x_old: vec![0.0; 3 * n],
        v: vec![0.0; 3 * n],
        w: vec![0.0; n],
        air: vec![0.0; 3 * n],
        sweep_start: vec![0.0; 3 * n],
        hinges: HingeBatch::new(hinge_ids, hinge_rest, vec![0.0; hinge_count], vec![0.0; hinge_count]),
        hinge_k,
        hinge_crease,
        plastic_offset: vec![0.0; hinge_count],
        plastic_accumulated: vec![0.0; hinge_count],
        edges: EdgeBatch { ids: edge_ids, rest: edge_rest },
        membranes: MembraneBatch::new(faces.clone(), md, cd, vec![0.0; 3 * face_count], vec![0.0; 3 * face_count]),
        membrane_area,
        elastic: ElasticSolver::new(n, elastic_setup),
        aero: Aero::new(surface),
        layers: Layers::new(layer_setup),
        selfc: SelfContact::new(n, neighbors, faces, self_edges, face_tree, edge_tree),
        ground: Ground::new(n, s[18], s[19]),
        status: Status::Flying,
        time: s[35],
        frame: 0,
        quiet_time: 0.0,
        start: [s[36], s[37], s[38]],
        last_center: [s[39], s[40], s[41]],
        path_length: 0.0,
        ground_distance: 0.0,
        touchdown: None,
        state: [0.0; STATE],
        prof: [0.0; STAGES],
        frames_stepped: 0.0,
        positions,
        velocities,
        masses,
        p,
    });
    sim.selfc.reset();
    sim.ground.reset();
    sim.elastic.reset();
    sim.publish();
    Box::into_raw(sim)
}

#[no_mangle]
pub unsafe extern "C" fn sim_free(sim: *mut Sim) { drop(Box::from_raw(sim)); }

/// Advance one substep. Returns 1 when that completes a 1/120 s frame or the flight
/// (and at once if the flight has already ended). Positions and velocities are read from
/// their buffers at the start of each substep, so the caller may edit them in between.
#[no_mangle]
pub unsafe extern "C" fn sim_advance(sim: *mut Sim) -> u32 {
    let sim = &mut *sim;
    let complete = sim.advance();
    sim.publish();
    complete as u32
}
/// Advance to the end of the current 1/120 s frame; returns the status.
#[no_mangle]
pub unsafe extern "C" fn sim_step(sim: *mut Sim) -> u32 {
    let sim = &mut *sim;
    while !sim.advance() {}
    sim.publish();
    sim.status as u32
}
/// Flat xyz per vertex; writable between substeps.
#[no_mangle]
pub unsafe extern "C" fn sim_positions(sim: *mut Sim) -> *mut f64 { (*sim).positions.as_mut_ptr() }
#[no_mangle]
pub unsafe extern "C" fn sim_velocities(sim: *mut Sim) -> *mut f64 { (*sim).velocities.as_mut_ptr() }
/// Air, weight and launcher force on each vertex in the last substep.
#[no_mangle]
pub unsafe extern "C" fn sim_forces(sim: *mut Sim) -> *const f64 { (*sim).forces.as_ptr() }
/// Air loads of the last sample, twelve values per panel: centre, force, lift, incidence,
/// area, stalled. The count is in the state.
#[no_mangle]
pub unsafe extern "C" fn sim_panels(sim: *mut Sim) -> *const f64 { (*sim).aero.panels.as_ptr() }
/// Permanent crease offset per hinge, then (`which` = 1) accumulated plastic rotation.
#[no_mangle]
pub unsafe extern "C" fn sim_plastic(sim: *mut Sim, which: u32) -> *const f64 {
    if which == 0 { (*sim).plastic_offset.as_ptr() } else { (*sim).plastic_accumulated.as_ptr() }
}
/// Layer-contact side state: 0 the seeded sign per candidate, 1 the chosen side per shared
/// side (both f64); `sim_layer_chosen` gives whether each candidate has chosen (bytes).
#[no_mangle]
pub unsafe extern "C" fn sim_layer_sides(sim: *mut Sim, which: u32) -> *const f64 {
    let (sign, _, side) = (*sim).layers.sides();
    if which == 0 { sign.as_ptr() } else { side.as_ptr() }
}
#[no_mangle]
pub unsafe extern "C" fn sim_layer_chosen(sim: *mut Sim) -> *const u8 { (*sim).layers.sides().1.as_ptr() }
/// Copy the active self-contacts into a staging buffer, seven i32 each (four vertex ids,
/// kind, side, pair key); the count is in the state.
#[no_mangle]
pub unsafe extern "C" fn sim_self_contacts(sim: *mut Sim) -> *const i32 {
    let s = staged_i32();
    s[field::SELF_EDGES] = (*sim).selfc.contacts();
    s[field::SELF_EDGES].as_ptr()
}
/// [time, status, touched down, touchdown time, touchdown centre xyz, touchdown distance,
///  ground distance, path length, last centre xyz, frames, narrow tests, vertex-face and
///  edge-edge contacts added, active self-contacts, active layer contacts, broad-phase
///  pairs, panels, floor touched, substep and substeps of the frame under way,
///  detection timing x3]
#[no_mangle]
pub unsafe extern "C" fn sim_state(sim: *mut Sim) -> *const f64 { (*sim).state.as_ptr() }
/// Accumulated stage times (ms) while profiling is on; see STAGES.
#[no_mangle]
pub unsafe extern "C" fn sim_profile(sim: *mut Sim) -> *mut f64 { (*sim).prof.as_mut_ptr() }
#[no_mangle]
pub unsafe extern "C" fn set_profiling(on: u32) { PROFILING = on != 0; }
