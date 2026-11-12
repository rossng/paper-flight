# Paper Flight — Backyard Test Pilot

A browser-only TypeScript / Vite / Three.js experiment in coupled paper-aeroplane flight. The aircraft is a free deformable particle mesh: gravity and per-triangle aerodynamic forces drive its translation, rotation, bending, and twist. There are no scripted flight trajectories, prescribed in-flight attitude changes, or in-flight position anchors. Workbench folds are animated separately using the recipe’s crease rotations.

## Run locally

Use Node.js **22.12+** (Node 24 recommended) and a modern desktop browser with WebGL 2.

```sh
cd paper-flight
npm ci
npm run dev
```

Open the URL printed by Vite, normally `http://localhost:5173`. The server only serves application files; all simulation runs in the browser. No account, backend, or API key is needed. Optional Google Fonts typography falls back to system fonts offline.

```sh
npm test          # numerical and behavioural regression checks
npm run benchmark # repeat the soft / standard / stiff flight measurements
npm run build     # type-check and produce dist/
npm run preview   # serve the production build
```

The download also includes a prebuilt `dist/`. To use it without installing Node dependencies, serve it with any static HTTP server, for example:

```sh
python3 -m http.server 8080 --directory dist
```

Then open `http://localhost:8080`. Do not open `index.html` using `file://`, because ES modules need an HTTP origin.

## Play

The game uses a full-screen craft-table and park theme, with a compact tool dock and flight HUD. It has two interfaces:

1. **Workbench:** choose a design from the illustrated plane picker. Drag the paper to orbit, scroll or pinch to zoom, and tap the paper or **Next fold** to animate one step. **Back** unfolds the last step. **Fold plane** completes the design immediately, and **Fly** also completes any remaining folds with the current settings before opening the launcher. Choose light (60 gsm), everyday (80 gsm), or sturdy (110 gsm) paper, a sheet size, and the wing V angle; use the −/+ buttons to change span. Reduced-motion mode skips fold animations.
2. **Throw:** pull the sling downward to store elastic energy, move sideways to aim, and release. The low-poly field has metre-scale trees, signs every five metres, and a visible flight trail. Switch between chase and launch-point cameras. After touchdown, try again or return to your folds. Your best distance is saved locally when browser storage is available. After landing, choose **Explore flight** to orbit/pan/zoom around the complete orange trajectory, replay or scrub the recorded aircraft deformation, and jump to takeoff, landing, or the replayed plane. WASD moves the review camera and Q/E moves it vertically. Replay uses detached recordings, not a new simulation.

Keyboard: all folding and material controls are buttons. Focus the plane in the sling, use left/right arrows to aim and down/up arrows to increase/decrease pull, then Space or Enter to launch (a default pull is used if none is set). Drag the attitude dial or use its −/+ buttons to set plane pitch independently through a full 360° rotation, including vertical, backward and inverted attitudes. The dial wraps through ±180° without a stop; W/S also adjusts pitch. With the dial focused, Page Up/Down turns 15°, Home levels the plane, and End inverts it. The launcher shows live pitch, speed, and power, and its bands attach to the visible plane. Pause and reset have buttons; R resets the field when focus is outside a button. Pointer cancellation releases the sling without launching.

The Original dart uses the guided effective mesh. Select The Basic, Folded Dart, or The Stable to step through actual fold definitions and fly their compiled layered material meshes. This remains a guided recipe tool rather than a freeform crease editor. Paper weight approximates thickness at fixed material density. Mass scales with sheet area and weight; bending rigidity scales with the cube of paper weight relative to the 80 gsm baseline. Wing adjustments preserve total paper mass.

The launcher resists further dragging as it stretches: pointer travel is mapped to extension with `travel = extension + extension² / (2 × 70)` in screen pixels, capped at 100 pixels of extension. This is an input-effort proxy, not measured hand force. Radial extension drives the power gauge and spring energy, and release follows the accelerating first quarter-cycle of the spring.

Launcher energy uses `v = sqrt(k * stretch² / mass)` with an effective spring constant of 0.8 N/m and up to 0.7 m stretch; release speed is capped at 9 m/s to stay near the tested solver envelope. Pull direction sets heading and elevation; plane attitude is controlled independently with the pitch dial. The dial changes the aircraft’s nose angle, while pull sets launch velocity and release elevation. Flight then evolves without scripted steering. The default physics API retains the original horizontal launch for regression tests and benchmarks.

## Geometry and mass

`src/geometry/dart.ts` builds a conventional dart silhouette, approximately 30 cm long and 18 cm in span, with **89 vertices and 150 triangles**. Two swept wings join a V-shaped keel. Wing-root folds and the centre crease have their own torsional stiffness. Flat interior edges resist bending. A small, predefined 6 mm trailing-edge reflex supplies geometric pitch trim; it is part of the resting shape and is never adjusted during flight.

The mesh is an **effective folded surface**, not a complete A4 sheet with every overlapping folded layer. Shared fold vertices connect the panels. Baseline mass is 5 g (scaled by sheet size squared and paper weight), lumped from face area with additional effective mass near the folded centre and nose. This approximates layering and yields a forward centre of mass. It is not a reconstruction of a particular folding sequence.

Geometry exposes rest vertices, indexed triangles, edge lengths, signed hinge angles, crease flags, masses, and body-frame landmarks. Another design can replace this factory while retaining the solver and rendering interfaces. A future FOLD importer should build these same data structures and explicitly resolve overlapping panels.

## Structure: compliant bar-and-hinge XPBD

The physics module does not import Three.js or browser APIs. All quantities use metres, kilograms, seconds, newtons, and radians.

Each edge enforces a nearly inextensible distance constraint:

```text
C_edge(x) = |x_i - x_j| - L_rest
edge compliance = 2e-8 m/N
```

Each pair of adjacent triangles has a signed dihedral constraint:

```text
C_hinge(x) = wrap(theta(x) - theta_rest)
E_hinge = 1/2 k C_hinge²
k_flat   = (3.5e-4 N m) × paper_multiplier × edge_length² / (A1 + A2)
k_crease = (2.5e-3 N m) × crease_multiplier × edge_length² / (A1 + A2)
compliance_hinge = 1/k
```

The rest area weighting approximates a discrete shell bending energy. These constants are effective prototype parameters, **not calibrated paper moduli**. Creases can be relatively stiff because the effective keel includes multiple folded layers; the crease multiplier is independent of the flat-sheet multiplier. Twist emerges from the coupled triangle hinges rather than a separate scripted wing-twist degree of freedom.

After external-force integration, XPBD projects each scalar constraint using inverse masses `w_i`:

```text
alpha_tilde = compliance / dt²
Delta_lambda = (-C - alpha_tilde × lambda) /
               (sum_i w_i |gradient_i C|² + alpha_tilde)
x_i += w_i × gradient_i C × Delta_lambda
```

Multipliers accumulate within a substep and reset for the next substep. Signed `atan2` angles preserve mountain/valley orientation, including the flat state. Degenerate triangles and zero-length edges are guarded. The analytic gradients are checked against finite differences.

A fixed outer step of **1/120 s** uses **3 substeps at 1/360 s** (9 at 1/1080 s for sheets under 30 gsm) and **24 alternating Gauss–Seidel sweeps** per substep. Near-zero stretch compliance and finite iteration count mean stretch is small, not exactly zero. Changing solver resolution changes truncation error and can change flight results; a convergence study is still needed.

Velocities are reconstructed from the corrected positions. Pairwise axial damping dissipates strain-rate motion while preserving linear and angular momentum for each pair; first-order integration also provides numerical damping. There is no global velocity drag masquerading as structural damping. Aerodynamic drag remains a separate physical force. Gravity is applied per vertex.

The aircraft is never pinned during flight. Torques arise from the spatial distribution of aerodynamic forces and mass; no stabilizing controller or extra pitch spring is present. A run stops at first vertex-ground contact, or after 12 simulated seconds. Impact, bounce, sliding, and crumpling are intentionally not simulated.

## Aerodynamics: current geometry, distributed loads

For each triangle, every substep:

```text
A = 1/2 |(x_b - x_a) cross (x_c - x_a)|
n = normalize((x_b - x_a) cross (x_c - x_a))
v_rel = (v_a + v_b + v_c)/3 - wind
u = normalize(v_rel)
alpha = asin(clamp(-u dot n, -1, 1))
q = 1/2 rho |v_rel|²
lift_direction = normalize(n - u (u dot n))
F = q A (CL lift_direction - CD u)
```

`F/3` is applied directly to each vertex. The resultant therefore acts at the triangle centroid and can bend the wing and rotate the whole aircraft. Areas, normals, and velocities come from the **deformed mesh**, including local deformation velocities. Original mesh normals never enter the force calculation. Air density defaults to 1.225 kg/m³. Wind defaults to zero; the launch weather pane selects a base breeze and optional gusts/turbulence. Each panel samples air velocity at its current centre and simulation time.

The empirical low-aspect-ratio polar is:

```text
b = smoothstep(15 degrees, 30 degrees, |alpha|)
CL = (1-b) × 2.7 alpha + b × 0.85 sin(2 alpha)
CD = (1-b) × (0.035 + 0.18 CL²) + b × (0.08 + 1.75 sin²(alpha))
```

This provides an attached-flow regime and a smooth separated-flow regime. It can produce gliding, diving, loss of lift, and tumbling as outcomes of the equations. The stall indicator reports the area fraction above 15° local incidence, **not a CFD-derived stall boundary**. Reversing triangle winding leaves the force unchanged. Lift is perpendicular to relative velocity; drag removes energy in still air.

### Aerodynamic limitations

- Quasi-steady local panels do not solve incompressible flow, circulation, pressure, or a wake. The drag term only approximates induced losses; neighbouring panels do not communicate through airflow.
- There is no dynamic stall, vortex lift model, separation hysteresis, Reynolds-number dependence, ground effect, or panel pitching moment.
- An orientation-only sheet polar does not distinguish a leading edge from a trailing edge. Reverse flight is treated as a symmetric flat-sheet case.
- Overlapping surfaces receive independent forces. There is **no shielding or occlusion**. The effective dart omits hidden paper layers to avoid counting them as extra lifting area; adding literal overlapping sheets without fixing this would overpredict force.
- No self-contact or tearing is implemented. Very soft creases and high-speed, high-pitch launches can fold the wings through one another. They may remain numerically finite while being physically inaccurate.
- Coarse discretization, finite solver sweeps, and first-order time integration can introduce damping, small asymmetry, and trim error. These need calibration before quantitative prediction.

## Visualisation and metrics

Three.js renders the actual deforming aircraft at physical scale in a procedural low-poly field. Trees and signs are visual landmarks, not collision obstacles; ground contact ends a flight. The workbench drawing is a schematic preview. The field reports horizontal distance from release, simulated air time, centre-of-mass height, and a personal best. The renderer also retains diagnostic mesh/force/heatmap capabilities for future developer views, but these are not part of the game UI.

The physics API exposes deformation RMS with body translation and rotation removed, hinge-angle RMS, edge strain, aerodynamic force, stall fraction, and integrated path length. The benchmark command still compares the original soft, standard, and stiff configurations.

## Validation and performance

`npm test` covers mesh topology and mass, hinge gradients, aerodynamic dissipation and winding invariance, translation and gravity, deterministic stepping and reset, stiffness coupling, parameter extremes, scaled paper mass and release height, directional release velocities, the new workbench extremes through ground contact, detached replay data, and the custom-folding geometry studies.

The production build and numerical suite pass. Headless Chrome interaction checks exercised folding, material selection, pointer launch through landing, keyboard launch, pause/reset, and a 390 px mobile layout without runtime errors or horizontal overflow. These are representative checks, not an exhaustive stability or physical validation study.

Physics remains fixed at 120 Hz with three internal substeps, or nine for very light sheets. Device-pixel ratio is capped at 2. Playback can slow under load rather than enlarging the physics timestep. Background-tab time is discarded. A capability message appears if WebGL cannot start.

## Project map

| Path | Responsibility |
| --- | --- |
| `src/geometry/dart.ts` | Effective dart topology, layers, edge and hinge rest state |
| `src/physics/constraints.ts` | Distance and signed dihedral XPBD, internal damping |
| `src/physics/aerodynamics.ts` | Empirical polar and current-panel forces |
| `src/physics/simulation.ts` | Free-flight integration, fixed clock, metrics |
| `src/physics/math.ts` | Renderer-independent vector math |
| `src/rendering/viewport.ts` | Three.js cameras and deforming meshes |
| `src/rendering/landscape.ts` | Procedural field, trees, clouds, distance signs |
| `src/main.ts`, `src/style.css` | Folding workbench, launcher gestures, results, responsive interface |
| `tests/physics.test.ts` | Numerical and acceptance regressions |
| `tests/benchmark.ts` | Repeatable flight and timing report |

## Investigated implementations and next steps

- [Origami Simulator](https://origamisimulator.org/) and [its source](https://github.com/amandaghassaei/OrigamiSimulator): informed the bar-and-hinge representation and separation of crease/face stiffness. Its GPU folding workflow is not embedded here; a small free-flight CPU solver was simpler for this prototype. No Origami Simulator code was copied.
- [PositionBasedDynamics](https://github.com/InteractiveComputerGraphics/PositionBasedDynamics): the MIT-licensed signed hinge gradient formulation was adapted from its dihedral constraint implementation. See `THIRD_PARTY.md`. Compiling the full C++ library to WASM was unnecessary for this mesh size.
- [Macklin, Müller, Chentanez: XPBD (2016)](https://matthias-research.github.io/pages/publications/XPBD.pdf): compliance-based constraint update.
- [Three.js documentation](https://threejs.org/docs/) and [Vite documentation](https://vite.dev/guide/): rendering and build tooling.

Useful next steps, in order:

1. Establish a rigid-body reference and mesh/timestep/iteration convergence. Calibrate mass distribution, shell rigidity, crease hysteresis, and aerodynamic polar against measured throws and static deflection tests.
2. Add self-contact and layer shielding before supporting arbitrary folded designs. Import crease patterns with explicit aerodynamic visibility and mass layers.
3. Replace the local polar with lifting-line or vortex-lattice/wake coupling for attached flow; account for the low aspect ratio and recognize that conventional VLM will not solve separated flow.
4. Move the CPU solver into a Web Worker if profiling shows UI contention; use WASM only after measuring a useful gain. Consider iterative fluid/structure coupling for added-mass effects.
5. For full flow experiments, prototype a separate WebGPU lattice-Boltzmann or incompressible finite-volume solver with moving immersed boundaries. Transfer panel velocities to the flow grid and integrate pressures/shear back onto the mesh with conservative force/torque exchange. Validate grid convergence, boundary conditions, fluid timestep stability, moving-boundary errors, and coupled stability against simpler cases first. A GPU solver is not justified by this small structural mesh alone.

## Licensing

Project code is provided under the MIT license. Adapted components and dependency notices are in `THIRD_PARTY.md` and `licenses/`. No generated art, paid assets, or third-party runtime services are required.

## Custom-folding research prototype

Open `/folding-study.html` or follow the link in the game help dialog. Three independent reconstructions (The Basic, Basic Dart, and The Stable) exercise a new material-coordinate model with explicit layer selection. The study supports 3D inspection during animated forward/reverse folds, fold history and FOLD exports. The panels rotate rigidly about the defined creases; this is zero-thickness kinematics, not a contact simulation. The Original dart quick-start uses a grouped dart recipe as its folding illustration while retaining its separate effective flight mesh. Use Fly this design at any step to complete it and launch it in the game. Flight uses the effective laminate approximation documented below; it is not a frictional contact simulation or a freeform editor. See [the design and findings](docs/custom-folding.md).

## Folded designs and flex review

The plane picker connects three fold recipes to the same flexible solver. `src/folding/flight-mesh.ts` triangulates and refines the material panels, preserves their mass distribution, identifies overlapping layers, and builds crease/interior hinges. Static overlap samples share aerodynamic area between coincident layers. Barycentric attachments keep overlapping material together as an effective laminate. Local bending multipliers depend on layer count, with an intermediate coupling assumption; sliding, peeling and finite-thickness contact are not resolved. See [the compiler model](docs/folded-flight.md).

In Explore flight, **Flex arrows** shows displacement relative to the undeformed plane after removing body translation and rotation. Arrow lengths can be exaggerated 5×–250×; aircraft geometry stays at true scale. Blue wire shows the aligned reference. **Paper layers** colours single-layer areas teal and heavily layered areas orange. **Follow plane** tracks the replay; **Full path** restores the trajectory overview.

## Special materials and weather

The purple Special selector adds tissue paper, kitchen foil and steel plate. Selecting an ordinary paper weight returns to paper. These are playful **effective presets**, not measured constitutive laws: tissue uses 18 gsm and soft bending/creases, foil uses 135 gsm with stiffer panels and softer creases, and nominal 1 mm steel uses 7850 gsm with much greater bending/crease stiffness. The same folded material areas set their masses, so the launcher automatically delivers less speed to heavier planes. Metal presets also change the rendered surface. The workbench still folds kinematically; it does not model the work or permanent deformation needed to fold steel.

The Weather button at the launcher opens still air, breeze (1.5 m/s) and blustery (3 m/s) choices. The direction means **where air blows toward**, with up on the compass meaning downfield (+X). Gusts add smooth intermittent bursts up to 1.5 m/s; turbulence adds a deterministic, spatially varying divergence-free swirl field, including vertical motion. The on-field indicator displays local air speed and horizontal direction; the windsock samples the field at its own position. These controls prescribe air velocity rather than solve a fluid or account for tree wakes. Settings stay fixed during a throw, and replay lift analysis evaluates the same air field at the replayed position and time.

Regression checks cover material mass scaling and launcher energy, spatial/temporal weather sampling, replay sampling, and full-power gusty crosswind flights for every special material on all four designs. Very light sheets use smaller physics substeps to keep panel loads from over-stretching the structure.
