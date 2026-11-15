//! The solver's kernels one at a time, for the replay inspector (air loads of a recorded
//! frame) and for unit tests. Arguments and results travel in the staging buffers: the
//! set-up slots of `sim_create`, plus the scratch slots below.
#![allow(clippy::missing_safety_doc)]

use crate::aero::{self, Aero, SurfaceSetup, Weather};
use crate::elastic::{ElasticSetup, ElasticSolver};
use crate::ground::Ground;
use crate::layer::{LayerSetup, Layers};
use crate::selfcontact::{self, SelfContact, Tree};
use crate::sweeps::{self, HingeBatch, MembraneBatch};
use crate::{field, staged_f64, staged_i32};

// f64 scratch slots.
const X: usize = 21;
const OLD: usize = 22;
const W: usize = 23;
const A: usize = 24;
const B: usize = 25;
const C: usize = 26;
const D: usize = 27;
const E: usize = 28;
// i32 scratch slots.
const IDS: usize = 33;
const FLAGS: usize = 34;

unsafe fn take(slot: usize) -> Vec<f64> { std::mem::take(&mut staged_f64()[slot]) }
unsafe fn put(slot: usize, values: Vec<f64>) { staged_f64()[slot] = values; }
unsafe fn take_i(slot: usize) -> Vec<i32> { std::mem::take(&mut staged_i32()[slot]) }
unsafe fn put_i(slot: usize, values: Vec<i32>) { staged_i32()[slot] = values; }

/// Where a slot's current contents are, to read a result back.
#[no_mangle]
pub unsafe extern "C" fn staged_f64_at(slot: usize) -> *const f64 { staged_f64()[slot].as_ptr() }
#[no_mangle]
pub unsafe extern "C" fn staged_i32_at(slot: usize) -> *const i32 { staged_i32()[slot].as_ptr() }
#[no_mangle]
pub unsafe extern "C" fn staged_f64_len(slot: usize) -> usize { staged_f64()[slot].len() }
#[no_mangle]
pub unsafe extern "C" fn staged_i32_len(slot: usize) -> usize { staged_i32()[slot].len() }

fn weather(values: &[f64]) -> Weather { Weather { wind: [values[0], values[1], values[2]], wind_heading: values[3], gusts: values[4], turbulence: values[5], feature: values.get(6).copied().unwrap_or(0.0) as u8 } }

// ---- Air loads ----

/// A lifting surface from the staged surface slots. Scratch A holds shielding distance,
/// rest span, chordwise panels and whether faces have owners.
#[no_mangle]
pub unsafe extern "C" fn aero_create() -> *mut Aero {
    let s = take(A);
    let lm = take_i(field::LANDMARKS);
    let owners = take_i(field::FACE_OWNERS);
    Box::into_raw(Box::new(Aero::new(SurfaceSetup {
        faces: take_i(field::FACES),
        owners: if s[3] != 0.0 { Some(owners) } else { None },
        signs: take(field::FACE_SIGNS),
        boundaries: take(field::BOUNDARIES),
        span: s[1],
        chord_panels: s[2] as usize,
        patch_face: take_i(field::PATCH_FACE),
        patch_station: take_i(field::PATCH_STATION),
        patch_panel: take_i(field::PATCH_PANEL),
        patch_weights: take(field::PATCH_WEIGHTS),
        patch_fraction: take(field::PATCH_FRACTION),
        landmarks: [lm[0] as usize, lm[1] as usize, lm[2] as usize, lm[3] as usize],
        shielding: s[0],
    })))
}
#[no_mangle]
pub unsafe extern "C" fn aero_free(aero: *mut Aero) { drop(Box::from_raw(aero)); }
/// Coupled air loads of one state: positions and velocities in their slots, the weather
/// (wind xyz, heading, gusts, turbulence, air feature) in scratch A. Adds each vertex's force to
/// scratch X; leaves the panels in scratch B and returns how many.
#[no_mangle]
pub unsafe extern "C" fn aero_loads(aero: *mut Aero, time: f64, enabled: u32, density: f64) -> usize {
    let aero = &mut *aero;
    let (positions, velocities, mut out, w) = (take(field::POSITIONS), take(field::VELOCITIES), take(X), take(A));
    aero.exposure_valid = false;
    aero.loads(&positions, &velocities, &weather(&w), time, enabled != 0, density, &mut out);
    put(X, out);
    put(B, aero.panels.clone());
    aero.panels.len() / aero::PANEL
}
/// The uncoupled per-face loads: as `aero_loads`, with faces and optional area weights in
/// their slots and no lifting surface.
#[no_mangle]
pub unsafe extern "C" fn panel_loads(time: f64, enabled: u32, density: f64) -> usize {
    let (positions, velocities, mut out, w) = (take(field::POSITIONS), take(field::VELOCITIES), take(X), take(A));
    let (faces, weights) = (take_i(field::FACES), take(field::AREA_WEIGHTS));
    let mut panels = Vec::new();
    aero::panel_loads(&positions, &velocities, &faces, &weights, &weather(&w), time, enabled != 0, density, &mut out, &mut panels);
    put(X, out);
    let count = panels.len() / aero::PANEL;
    put(B, panels);
    count
}
/// Planar vortex lattice with `m` chordwise panels per station: boundaries in their slot;
/// leading edge, chord, speed, incidence and attachment per panel in scratch X, OLD, W, A, B.
/// Circulation per panel is left in scratch C.
#[no_mangle]
pub unsafe extern "C" fn vortex_lattice(m: usize) {
    let (boundaries, lead, chord, speeds, incidence, attachment) = (take(field::BOUNDARIES), take(X), take(OLD), take(W), take(A), take(B));
    let stations = boundaries.len() - 1;
    let n = stations * m;
    let mut matrix = vec![0.0; n * (n + 1)];
    let mut circulation = vec![0.0; n];
    aero::solve_vortex_lattice(&boundaries, stations, m, &lead, &chord, &speeds, &incidence, &attachment, &mut matrix, &mut circulation);
    put(C, circulation);
}
/// The uncoupled polar at one incidence: 0 lift coefficient, 1 drag coefficient, 2 stalled.
#[no_mangle]
pub extern "C" fn polar(alpha: f64, which: u32) -> f64 {
    let (cl, cd, stalled) = aero::coefficients(alpha);
    match which { 0 => cl, 1 => cd, _ => stalled as u8 as f64 }
}

// ---- Self-contact ----

#[no_mangle]
pub unsafe extern "C" fn selfc_create(vertices: usize) -> *mut SelfContact {
    let neighbors: Vec<u32> = take_i(field::NEIGHBORS).iter().map(|&v| v as u32).collect();
    let tree = |l: Vec<i32>, r: Vec<i32>, e: Vec<i32>| if l.is_empty() { None } else { Some(Tree::new(l, r, e)) };
    let face_tree = tree(take_i(field::FACE_TREE_LEAVES), take_i(field::FACE_TREE_RIGHT), take_i(field::FACE_TREE_ESCAPE));
    let edge_tree = tree(take_i(field::EDGE_TREE_LEAVES), take_i(field::EDGE_TREE_RIGHT), take_i(field::EDGE_TREE_ESCAPE));
    let mut contact = SelfContact::new(vertices, neighbors, take_i(field::FACES), take_i(field::SELF_EDGES), face_tree, edge_tree);
    contact.reset();
    Box::into_raw(Box::new(contact))
}
#[no_mangle]
pub unsafe extern "C" fn selfc_free(contact: *mut SelfContact) { drop(Box::from_raw(contact)); }
#[no_mangle]
pub unsafe extern "C" fn selfc_reset(contact: *mut SelfContact) { (*contact).reset(); }
#[no_mangle]
pub unsafe extern "C" fn selfc_begin(contact: *mut SelfContact) { (*contact).begin_step(); }
/// Keep last substep's contacts that still touch; positions before the step in scratch OLD.
#[no_mangle]
pub unsafe extern "C" fn selfc_carry(contact: *mut SelfContact, gap: f64) {
    let old = take(OLD);
    (*contact).carry(&old, gap);
    put(OLD, old);
}
/// Swept detection between scratch OLD and scratch X.
#[no_mangle]
pub unsafe extern "C" fn selfc_detect(contact: *mut SelfContact) {
    let (old, x) = (take(OLD), take(X));
    (*contact).detect(&old, &x);
    put(OLD, old);
    put(X, x);
}
/// One projection sweep on scratch X with inverse masses in scratch W.
#[no_mangle]
pub unsafe extern "C" fn selfc_solve(contact: *mut SelfContact, forward: u32, sweep: usize) {
    let (mut x, w) = (take(X), take(W));
    (*contact).solve(&mut x, &w, forward != 0, sweep);
    put(X, x);
    put(W, w);
}
/// Active contacts, seven i32 each, into scratch IDS; counters into scratch A as
/// [narrow tests, vertex-face added, edge-edge added, broad-phase pairs]. Returns the count.
#[no_mangle]
pub unsafe extern "C" fn selfc_report(contact: *mut SelfContact) -> usize {
    let contact = &*contact;
    put_i(IDS, contact.contacts());
    put(A, vec![contact.stats[0], contact.stats[1], contact.stats[2], contact.pair_count() as f64]);
    contact.count
}
/// One contact projection on four vertices (twelve coordinates in scratch X, four inverse
/// masses in scratch W): the kernel the sweep applies to each active contact.
#[no_mangle]
pub unsafe extern "C" fn contact_project(vertex_face: u32, sign: f64) {
    let (mut x, w) = (take(X), take(W));
    selfcontact::project(&mut x, &w, 0, 1, 2, 3, vertex_face != 0, sign, false);
    put(X, x);
    put(W, w);
}
/// Roots in [0, 1] of the cubic with coefficients in scratch X, into scratch OLD.
#[no_mangle]
pub unsafe extern "C" fn cubic_roots() -> usize {
    let c = take(X);
    let mut roots = [0.0; 8];
    let count = selfcontact::cubic_roots(c[0], c[1], c[2], c[3], &mut roots);
    put(OLD, roots[..count].to_vec());
    count
}

// ---- Layer contact ----

#[no_mangle]
pub unsafe extern "C" fn layers_create() -> *mut Layers {
    Box::into_raw(Box::new(Layers::new(LayerSetup {
        vertex: take_i(field::LAYER_VERTEX), face_index: take_i(field::LAYER_FACE_INDEX), faces: take_i(field::LAYER_FACES),
        orientation: take(field::LAYER_ORIENTATION), ambiguous: take_i(field::LAYER_AMBIGUOUS), side_index: take_i(field::LAYER_SIDE_INDEX),
        sign: take(field::LAYER_SIGN), chosen: take_i(field::LAYER_CHOSEN), side: take(field::LAYER_SIDE),
    })))
}
#[no_mangle]
pub unsafe extern "C" fn layers_free(layers: *mut Layers) { drop(Box::from_raw(layers)); }
/// Detect contacts at scratch X, with the positions before the step in scratch OLD.
/// Returns the number active; `layers_report` gives them.
#[no_mangle]
pub unsafe extern "C" fn layers_detect(layers: *mut Layers, full: u32) -> usize {
    let (x, old) = (take(X), take(OLD));
    (*layers).detect(&x, &old, full != 0);
    put(X, x);
    put(OLD, old);
    (*layers).count
}
/// Active contacts: candidate index each into scratch IDS, thirteen values each into
/// scratch A. Side state: sign per candidate into B, side per shared side into C, chosen
/// flag per candidate into scratch FLAGS.
#[no_mangle]
pub unsafe extern "C" fn layers_report(layers: *mut Layers) {
    let layers = &*layers;
    let (candidate, data) = layers.contacts();
    put_i(IDS, candidate.to_vec());
    put(A, data.to_vec());
    let (sign, chosen, side) = layers.sides();
    put(B, sign.to_vec());
    put(C, side.to_vec());
    put_i(FLAGS, chosen.iter().map(|&c| c as i32).collect());
}
/// Make the active set the one contact in scratch A, between vertex 0 and face (1, 2, 3).
#[no_mangle]
pub unsafe extern "C" fn layers_adopt(layers: *mut Layers) {
    let data = take(A);
    (*layers).adopt(&data);
}
/// One sweep on scratch X with inverse masses in scratch W.
#[no_mangle]
pub unsafe extern "C" fn layers_solve(layers: *mut Layers, friction: f64, sweep: usize) {
    let (mut x, w) = (take(X), take(W));
    (*layers).solve(&mut x, &w, friction, sweep);
    put(X, x);
    put(W, w);
}

// ---- Floor ----

#[no_mangle]
pub unsafe extern "C" fn ground_create(count: usize) -> *mut Ground { Box::into_raw(Box::new(Ground::new(count, 0.0, 0.0))) }
#[no_mangle]
pub unsafe extern "C" fn ground_free(ground: *mut Ground) { drop(Box::from_raw(ground)); }
#[no_mangle]
pub unsafe extern "C" fn ground_reset(ground: *mut Ground) { (*ground).reset(); }
/// Position projection on scratch X; positions before the step in OLD, inverse masses in W.
/// Returns whether any vertex has borne on the floor since the reset.
#[no_mangle]
pub unsafe extern "C" fn ground_solve(ground: *mut Ground, height: f64, static_friction: f64, kinetic_friction: f64) -> u32 {
    let ground = &mut *ground;
    ground.static_friction = static_friction;
    ground.kinetic_friction = kinetic_friction;
    let (mut x, old, w) = (take(X), take(OLD), take(W));
    ground.solve(&mut x, &old, &w, height);
    put(X, x);
    put(OLD, old);
    put(W, w);
    ground.touched as u32
}
/// Velocity pass: positions in scratch X, velocities in scratch OLD. Returns whether the
/// floor supports any vertex.
#[no_mangle]
pub unsafe extern "C" fn ground_solve_velocity(ground: *mut Ground, height: f64, static_friction: f64, kinetic_friction: f64) -> u32 {
    let ground = &mut *ground;
    ground.static_friction = static_friction;
    ground.kinetic_friction = kinetic_friction;
    let (x, mut v) = (take(X), take(OLD));
    let supported = ground.solve_velocity(&x, &mut v, height);
    put(X, x);
    put(OLD, v);
    supported as u32
}

// ---- Elastic solve and constraint sweeps ----

/// Membranes from the face, md and cd slots with compliance in scratch A and multipliers in B.
unsafe fn staged_membranes() -> MembraneBatch {
    MembraneBatch::new(take_i(field::FACES), take(field::MEMBRANE_MD), take(field::MEMBRANE_CD), take(A), take(B))
}
/// Hinges from the id and rest slots with compliance in scratch C, multipliers in D and
/// plastic offsets in E.
unsafe fn staged_hinges() -> HingeBatch {
    let (ids, rest, offsets) = (take_i(field::HINGE_IDS), take(field::HINGE_REST), take(E));
    let mut hinges = HingeBatch::new(ids, rest, take(C), take(D));
    for (j, &offset) in offsets.iter().enumerate() { hinges.set_offset(j, offset); }
    hinges
}
#[no_mangle]
pub unsafe extern "C" fn elastic_create(vertices: usize, blocks: usize) -> *mut ElasticSolver {
    let mut solver = ElasticSolver::new(vertices, ElasticSetup {
        perm: take_i(field::PERM), ap: take_i(field::AP), ai: take_i(field::AI), source: take_i(field::SOURCE),
        diagonal: take_i(field::DIAGONAL), face_slots: take_i(field::FACE_SLOTS), hinge_slots: take_i(field::HINGE_SLOTS),
        block_len: blocks,
    });
    solver.reset();
    Box::into_raw(Box::new(solver))
}
#[no_mangle]
pub unsafe extern "C" fn elastic_free(solver: *mut ElasticSolver) { drop(Box::from_raw(solver)); }
#[no_mangle]
pub unsafe extern "C" fn elastic_reset(solver: *mut ElasticSolver) { (*solver).reset(); }
/// Solve the implicit step for the staged membranes and hinges from the prediction in
/// scratch X (masses in W). Leaves positions in X and multipliers in B and D; returns
/// whether it converged.
#[no_mangle]
pub unsafe extern "C" fn elastic_solve(solver: *mut ElasticSolver, dt: f64, iterations: usize, tolerance: f64) -> u32 {
    let (mut membranes, mut hinges) = (staged_membranes(), staged_hinges());
    let (mut x, masses) = (take(X), take(W));
    let solved = (*solver).solve(&mut x, &masses, dt, &mut membranes, &mut hinges, iterations, tolerance);
    put(X, x);
    put(B, membranes.lambda);
    put(D, hinges.lambda);
    solved as u32
}
/// One sweep over the staged membranes on scratch X (inverse masses in W); multipliers back in B.
#[no_mangle]
pub unsafe extern "C" fn membranes_solve(forward: u32) {
    let mut membranes = staged_membranes();
    let (mut x, w) = (take(X), take(W));
    sweeps::solve_membranes(&mut x, &w, &mut membranes, forward != 0);
    put(X, x);
    put(B, membranes.lambda);
}
/// One sweep over the staged hinges on scratch X (inverse masses in W); multipliers back in D.
#[no_mangle]
pub unsafe extern "C" fn hinges_solve(forward: u32) {
    let mut hinges = staged_hinges();
    let (mut x, w) = (take(X), take(W));
    sweeps::solve_hinges(&mut x, &w, &mut hinges, forward != 0);
    put(X, x);
    put(D, hinges.lambda);
}
/// Crease yield for one hinge. Leaves [offset, accumulated] in scratch X; returns whether it yielded.
#[no_mangle]
pub unsafe extern "C" fn crease_plasticity(angle: f64, rest: f64, offset: f64, accumulated: f64, yield_angle: f64, hardening: f64) -> u32 {
    match crate::crease_yield(angle, rest, offset, accumulated, yield_angle, hardening) {
        Some((offset, accumulated)) => { put(X, vec![offset, accumulated]); 1 }
        None => 0,
    }
}

// ---- Weather and launcher ----

/// The prescribed air velocity at a point and time, for the weather in scratch A; into scratch X.
#[no_mangle]
pub unsafe extern "C" fn air_velocity(time: f64, x: f64, y: f64, z: f64) {
    let w = take(A);
    put(X, weather(&w).air_velocity(time, [x, y, z]).to_vec());
}
/// Velocity an ideal elastic launcher has contributed by `time`, and its increase over a step.
#[no_mangle]
pub extern "C" fn spring_speed(time: f64, speed: f64, duration: f64) -> f64 { crate::spring_speed(time, speed, duration) }
#[no_mangle]
pub extern "C" fn spring_velocity_delta(time: f64, dt: f64, speed: f64, duration: f64) -> f64 { crate::spring_velocity_delta(time, dt, speed, duration) }
