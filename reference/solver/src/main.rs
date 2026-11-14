//! Paper Flight reference solver. See docs/reference-solver.md.
mod aero;
mod elements;
mod linalg;
mod problem;
mod sim;
mod vec;

use problem::Problem;
use sim::{Options, Simulation, FRAME};
use std::fmt::Write as _;
use std::io::Write as _;
use vec::*;

const USAGE: &str = "usage:
  paperflight-reference run <problem> <result.json> [options]
  paperflight-reference loads <problem> <state> [options]     (static cross-check: air loads, elastic energy and forces)
options:
  --steps-per-frame N     implicit steps per 1/120 s (default 64)
  --aero-stations N       span stations of the vortex lattice (default: the problem's, as in the game)
  --aero-chord N          chordwise lattice panels (default: the problem's)
  --patch-area A          largest aerodynamic quadrature patch, m^2 (default 2.5e-5; 0 = one per clipped polygon)
  --contact-stiffness K   layer and floor penalty stiffness, N/m (default 2e4)
  --after-touchdown S     seconds to simulate after touchdown (default 0; negative = until rest)
  --max-time S            stop an airborne flight after S seconds (default 12)
  --first-opening-sides   ignore fold-order stacking sides from the problem file
  --states FILE           also write every frame's vertex positions as little-endian f32
  --quiet                 no progress output";

fn fail(message: &str) -> ! { eprintln!("{message}"); std::process::exit(2) }

fn json_vector(v: V3) -> String { format!("[{:.9},{:.9},{:.9}]", v[0], v[1], v[2]) }

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 3 { fail(USAGE); }
    let mut options = Options::default();
    let (mut states, mut quiet) = (None, false);
    let mut i = 3;
    while i < args.len() {
        let flag = args[i].clone();
        let mut value = || -> f64 { i += 1; args.get(i).and_then(|v| v.parse::<f64>().ok()).unwrap_or_else(|| fail(USAGE)) };
        match flag.as_str() {
            "--steps-per-frame" => options.steps_per_frame = value() as usize,
            "--aero-stations" => options.aero_stations = value() as usize,
            "--aero-chord" => options.aero_chord = value() as usize,
            "--patch-area" => options.patch_area = value(),
            "--contact-stiffness" => options.contact_stiffness = value(),
            "--after-touchdown" => options.after_touchdown = value(),
            "--max-time" => options.max_time = value(),
            "--first-opening-sides" => options.ignore_sides = true,
            "--states" => { i += 1; states = Some(args.get(i).cloned().unwrap_or_else(|| fail(USAGE))); }
            "--quiet" => quiet = true,
            other => fail(&format!("unknown option {other}\n{USAGE}")),
        }
        i += 1;
    }
    if options.steps_per_frame == 0 || options.aero_stations == 1 { fail(USAGE); }
    let text = std::fs::read_to_string(&args[1]).unwrap_or_else(|e| fail(&format!("cannot read {}: {e}", args[1])));
    let problem = Problem::parse(&text).unwrap_or_else(|e| fail(&format!("{}: {e}", args[1])));
    match args[0].as_str() {
        "run" => run(&problem, options, &args[2], states, quiet),
        "loads" => loads(&problem, options, &args[2]),
        _ => fail(USAGE),
    }
}

/// Evaluate the force models on a supplied state: one line of x y z vx vy vz per vertex.
fn loads(problem: &Problem, options: Options, state: &str) {
    let text = std::fs::read_to_string(state).unwrap_or_else(|e| fail(&format!("cannot read {state}: {e}")));
    let rows: Vec<Vec<f64>> = text.lines().filter(|l| !l.trim().is_empty()).map(|l| l.split_whitespace().map(|t| t.parse().unwrap()).collect()).collect();
    if rows.len() != problem.rest.len() { fail("state has the wrong number of vertices"); }
    let x: Vec<f64> = rows.iter().flat_map(|r| [r[0], r[1], r[2]]).collect();
    let v: Vec<f64> = rows.iter().flat_map(|r| [r[3], r[4], r[5]]).collect();
    let sim = Simulation::new(problem, options);
    let (air, summary) = sim.air_forces(&x, &v, problem.scalar_or("time", 0.0));
    let (membrane, bending) = sim.elastic_energy(&x);
    let list = |values: &[f64]| values.iter().map(|v| format!("{v:.12e}")).collect::<Vec<_>>().join(",");
    println!("{{\"patches\":{},\"air\":[{}],\"airForce\":{},\"membraneEnergy\":{:.12e},\"bendingEnergy\":{:.12e},\"elastic\":[{}]}}",
        sim.patches(), list(&air), json_vector(summary.force), membrane, bending, list(&sim.elastic_forces(&x)));
}

fn run(problem: &Problem, options: Options, output: &str, states: Option<String>, quiet: bool) {
    let started = std::time::Instant::now();
    let mut sim = Simulation::new(problem, options.clone());
    let mut state_file = states.as_ref().map(|path| std::io::BufWriter::new(std::fs::File::create(path).unwrap_or_else(|e| fail(&format!("cannot write {path}: {e}")))));
    let mut frames = String::new();
    let mut count = 0;
    let mut record = |sim: &Simulation, frames: &mut String, count: &mut usize| {
        let center = sim.center(&sim.x);
        let velocity = sim.center(&sim.v);
        let (_, forward, up, _) = sim.body_frame(&|i| get(&sim.x, i));
        let e = &sim.energy;
        if *count > 0 { frames.push(','); }
        write!(frames, "\n[{:.6},{:.7},{:.7},{:.7},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.4e},{:.4e},{:.5e},{:.5e},{:.5e},{:.4},{:.5e},{:.5e},{:.5e},{:.5e},{}]",
            sim.time, center[0], center[1], center[2], velocity[0], velocity[1], velocity[2], forward[0], forward[1], forward[2], up[0], up[1], up[2],
            sim.deformation(), sim.max_strain(), sim.loads.force[0], sim.loads.force[1], sim.loads.force[2],
            if sim.loads.area > 0.0 { sim.loads.stalled_area / sim.loads.area } else { 0.0 },
            e.kinetic, e.elastic, e.gravity, sim.energy_residual(), sim.active_contacts).unwrap();
        *count += 1;
        if let Some(file) = state_file.as_mut() { for c in &sim.x { file.write_all(&(*c as f32).to_le_bytes()).unwrap(); } }
    };
    record(&sim, &mut frames, &mut count);
    let mut next_report = 0.25;
    while sim.active() {
        sim.frame();
        record(&sim, &mut frames, &mut count);
        if !quiet && sim.time >= next_report {
            let c = sim.center(&sim.x);
            eprintln!("t={:.2}s centre=({:.3},{:.3},{:.3}) status={} newton/step={:.2} wall={:.0}s", sim.time, c[0], c[1], c[2], sim.status.name(),
                sim.newton_iterations as f64 / sim.steps as f64, started.elapsed().as_secs_f64());
            next_report += 0.25;
        }
    }
    let center = sim.center(&sim.x);
    let distance = (center[0] - sim.start[0]).hypot(center[2] - sim.start[2]);
    let touchdown = match &sim.touchdown {
        Some(t) => format!("{{\"time\":{:.6},\"center\":{},\"distance\":{:.7}}}", t.time, json_vector(t.center), t.distance),
        None => "null".into(),
    };
    let o = &sim.options;
    let json = format!(
"{{\"format\":\"paperflight-reference-result 1\",
\"options\":{{\"stepsPerFrame\":{},\"aeroStations\":{},\"aeroChord\":{},\"patchArea\":{:e},\"contactStiffness\":{:e},\"contactRange\":{:e},\"tolerance\":{:e},\"afterTouchdown\":{},\"foldOrderSides\":{}}},
\"mesh\":{{\"vertices\":{},\"faces\":{},\"aeroPatches\":{}}},
\"status\":\"{}\",\"time\":{:.6},\"touchdown\":{},
\"airTime\":{:.6},\"flightDistance\":{:.7},\"distance\":{:.7},\"groundDistance\":{:.7},\"pathLength\":{:.7},
\"peakDeformation\":{:.6e},\"peakStrain\":{:.6e},\"peakPenetration\":{:.6e},
\"solver\":{{\"steps\":{},\"newtonPerStep\":{:.3},\"worstNewton\":{},\"lineSearchHalvings\":{},\"regularisedSolves\":{},\"patternRebuilds\":{},\"wallSeconds\":{:.1}}},
\"energy\":{{\"start\":{:.6e},\"airWork\":{:.6e},\"launchWork\":{:.6e},\"dampingLoss\":{:.6e},\"residual\":{:.6e}}},
\"frameSeconds\":{:.9},
\"columns\":[\"time\",\"x\",\"y\",\"z\",\"vx\",\"vy\",\"vz\",\"forwardX\",\"forwardY\",\"forwardZ\",\"upX\",\"upY\",\"upZ\",\"deformation\",\"maxStrain\",\"airX\",\"airY\",\"airZ\",\"stallFraction\",\"kinetic\",\"elastic\",\"gravity\",\"energyResidual\",\"contacts\"],
\"frames\":[{}\n]}}\n",
        o.steps_per_frame, o.aero_stations, o.aero_chord, o.patch_area, o.contact_stiffness, o.contact_range, o.tolerance, o.after_touchdown, !o.ignore_sides && !problem.sides.is_empty(),
        sim.n, sim.faces.len(), sim.patches(), sim.status.name(), sim.time, touchdown,
        sim.touchdown.as_ref().map_or(sim.time, |t| t.time), sim.touchdown.as_ref().map_or(distance, |t| t.distance), distance, sim.ground_distance, sim.path_length,
        sim.peak_deformation, sim.peak_strain, sim.peak_penetration,
        sim.steps, sim.newton_iterations as f64 / sim.steps.max(1) as f64, if sim.worst_newton == usize::MAX { -1 } else { sim.worst_newton as i64 }, sim.line_searches, sim.regularised, sim.pattern_rebuilds, started.elapsed().as_secs_f64(),
        sim.energy_start, sim.energy.air_work, sim.energy.launch_work, sim.energy.damping_loss, sim.energy_residual(), FRAME, frames);
    std::fs::write(output, json).unwrap_or_else(|e| fail(&format!("cannot write {output}: {e}")));
    if let Some(mut file) = state_file { file.flush().unwrap(); }
    if !quiet { eprintln!("{}: {} after {:.3}s, flight {:.3} m, wall {:.1}s", output, sim.status.name(), sim.time, sim.touchdown.as_ref().map_or(distance, |t| t.distance), started.elapsed().as_secs_f64()); }
    if sim.status == sim::Status::Error { std::process::exit(1); }
}
