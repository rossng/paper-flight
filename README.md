# Paper Flight — Backyard Test Pilot

[Play Paper Flight](https://rossng.github.io/paper-flight/) · [Build and deployment](https://github.com/rossng/paper-flight/actions/workflows/pages.yml)

A browser-only TypeScript / Vite / Three.js experiment in coupled paper-aeroplane flight. The aircraft is a free deformable particle mesh: gravity and coupled exposed-surface aerodynamic forces drive its translation, rotation, bending, and twist. There are no scripted flight trajectories, prescribed in-flight attitude changes, or in-flight position anchors. Workbench folds are animated separately using the recipe’s crease rotations.

## Run locally

Use Node.js **22.12+** (Node 24 recommended) and a modern desktop browser with WebGL 2.

```sh
cd paper-flight
npm ci
npm run dev
```

Open the URL printed by Vite, normally `http://localhost:5173`. The server only serves application files; all simulation runs in the browser. No account, backend, or API key is needed. Optional Google Fonts typography falls back to system fonts offline.

```sh
npm test          # focused numerical and behavioural regression checks
npm run test:slow # exhaustive full-flight sweeps only
npm run test:full # all checks, including full-flight sweeps (used by CI)
npm run benchmark # repeat the soft / standard / stiff flight measurements
npm run benchmark:physics # aerodynamic and flexible-solver convergence evidence
npm run benchmark:performance # warmed, repeated serial solver timings
npm run build     # type-check and produce dist/
npm run preview   # serve the production build
```

See [the performance report](docs/TASK_REPORT.md) for paired baseline comparisons,
browser frame measurements, worker scheduling, and launch prediction tradeoffs.

The default tests include the independent high-load collision bounds, ordinary
tissue fold retention, worker/replay equivalence, launcher behaviour and solver
invariants. `tests/slow/` holds the full-flight material/weather matrix, compiled
recipe landings and parameter extremes. Run `npm run test:full` for physics or
folding changes; CI always runs it. Both suites use at most two test workers.
Material/weather sweeps are split by design so one file does not serialize the
entire matrix. Solver resolution and numerical tolerances are unchanged.

Local timing on Node 26.12.1, macOS, with two workers (one run per command):

| Suite | Passing checks | Wall time |
| --- | ---: | ---: |
| Original `npm test` | 116 | 80.08 s |
| Focused `npm test` | 101 | 20.75 s |
| `npm run test:full` | 120 | 67.05 s |

The full suite retains every original flight scenario. Its count increases
because launch/translation assertions now run separately from landing checks.
Uniform-translation checks on compiled meshes use three steps instead of thirty;
the longer gravity/translation and contact regressions remain. These are local
suite timings, not CI performance guarantees.

The download also includes a prebuilt `dist/`. To use it without installing Node dependencies, serve it with any static HTTP server, for example:

```sh
python3 -m http.server 8080 --directory dist
```

Then open `http://localhost:8080`. Do not open `index.html` using `file://`, because ES modules need an HTTP origin.

## GitHub Pages deployment

The [Pages workflow](.github/workflows/pages.yml) runs `npm ci`, the test suite and the production build on pull requests and pushes to `main`, using Node 24. Successful `main` builds deploy `dist/` to **https://rossng.github.io/paper-flight/**. Pull requests only run checks. You can also redeploy `main` from the workflow’s **Run workflow** button.

Pages uses **GitHub Actions** as its publishing source and the `github-pages` environment. Deployment uses GitHub’s temporary token, with Pages and OIDC permissions confined to the deployment job; no personal access token or deployment secret is required. Official actions are pinned to release commits. Vite’s relative base keeps the game, folding study, favicon and assets working under the repository’s path.

## Play

The game uses a full-screen craft-table and park theme, with a compact tool dock and flight HUD. It has two interfaces:

1. **Workbench:** choose a design from the illustrated plane picker. Drag the paper to orbit, scroll or pinch to zoom, and tap the paper or **Next fold** to animate one step. **Back** unfolds the last step. **Fold plane** completes the design immediately, and **Fly** also completes any remaining folds with the current settings before opening the launcher. Choose light (60 gsm), everyday (80 gsm), or sturdy (110 gsm) paper, a sheet size, and the wing angle (Flat, Shallow or Steep); use the −/+ buttons to change span on supported designs. The Folded Dart and six newer studies keep their authored wing sizes, so only their wing-angle choices are shown. Reduced-motion mode skips fold animations.
2. **Throw:** pull the sling downward to set launch speed, move sideways to aim, and release. The low-poly field has metre-scale trees, signs every five metres, and a visible flight trail. Set **Release height** beside the pitch dial (0.2–100 metres), or choose hand height, 5 m, or 15 m. Height changes the actual physical release position and stays selected for retries. Switch between chase and launch-point cameras. After touchdown, try again or return to your folds. Your best distance is saved locally when browser storage is available. After landing, choose **Explore flight** to orbit/pan/zoom around the complete orange trajectory, replay or scrub the recorded aircraft deformation, and jump to takeoff, landing, or the replayed plane. WASD moves the review camera and Q/E moves it vertically. Replay uses detached recordings, not a new simulation.

Keyboard: all folding and material controls are buttons. Focus the plane in the sling, use left/right arrows to aim and down/up arrows to increase/decrease pull, then Space or Enter to launch (a default pull is used if none is set). Drag the attitude dial or use its −/+ buttons to set plane pitch independently through a full 360° rotation, including vertical, backward and inverted attitudes. The dial wraps through ±180° without a stop; W/S also adjusts pitch. With the dial focused, Page Up/Down turns 15°, Home levels the plane, and End inverts it. The launcher shows live pitch, speed, and power, and its bands attach to the visible plane. Pause and reset have buttons; R resets the field when focus is outside a button. Pointer cancellation releases the sling without launching.

The Original dart uses the guided effective mesh. Select any of the nine full-sheet designs to step through actual fold definitions and fly their compiled layered material meshes. **Library & downloads ↓** exports the selected completed design or all nine studies as FOLD, with the current workbench settings. Original dart is an effective surface and has no full-sheet export. This remains a guided recipe tool rather than a freeform crease editor. Paper weight approximates thickness at fixed material density. Mass scales with sheet area and weight; bending rigidity scales with the cube of paper weight relative to the 80 gsm baseline. Wing adjustments preserve total paper mass.

The launcher resists further dragging as it stretches: pointer travel is mapped to extension with `travel = extension + extension² / (2 × 70)` in screen pixels, capped at 100 pixels of extension. This is an input-effort proxy, not measured hand force. Radial extension drives the power gauge and nominal launch speed. The existing 125 ms visual sling snap remains a gesture preview before the physical simulation starts.

The physical launch now starts from rest. An ideal elastic profile supplies acceleration over `launchDuration = 0.125` seconds: with `omega = pi / (2 × launchDuration)`, the unloaded velocity is `v(t) = nominalSpeed × sin(omega × t)` and acceleration is `a(t) = nominalSpeed × omega × cos(omega × t)`. Acceleration is finite and strongest at release, then falls to zero at detachment. Each physics substep integrates the exact launcher impulse over its interval, including a step that crosses detachment. Gravity, aerodynamic loads, deformation and contact remain active; velocity is never reset to the target at the end. Replay and distance/time measurements include this acceleration phase. Set `launchDuration: 0` in the physics API for the previous instantaneous initial velocity.

Nominal speed still scales linearly from 0 to 11 m/s with pullback, regardless of aircraft mass. This corresponds to an effective spring with `k/m = omega²` and unloaded stroke `nominalSpeed/omega` (about 0.88 m at full pull), rather than one fixed rubber band for all materials. The launcher force is distributed in proportion to vertex mass along the fixed aim, so it adds no torque; local hook stresses and elastic feedback from drag into spring extension are not modeled. Actual exit speed can differ because other forces act during the stroke. Pull direction sets heading and elevation independently of aircraft pitch. Once the stroke ends, only the ordinary flight forces remain.

Run `npm run benchmark:launcher` to compare instantaneous and elastic launches at equal nominal speed/impulse; it writes [the comparison results](docs/launcher-benchmark-results.json). This checks the launch-shock hypothesis rather than guaranteeing longer flights. The profiles are illustrative and have not been calibrated to a physical launcher.

## Geometry and mass

`src/geometry/dart.ts` builds a conventional dart silhouette, approximately 30 cm long and 18 cm in span, with **89 vertices and 150 triangles**. Two swept wings join a V-shaped keel. Wing-root folds and the centre crease have their own torsional stiffness. Flat interior edges resist bending. A small, predefined 6 mm trailing-edge reflex supplies geometric pitch trim; it is part of the resting shape and is never adjusted during flight.

The mesh is an **effective folded surface**, not a complete A4 sheet with every overlapping folded layer. Shared fold vertices connect the panels. Baseline mass is 5 g (scaled by sheet size squared and paper weight), lumped from face area with additional effective mass near the folded centre and nose. This approximates layering and yields a forward centre of mass. It is not a reconstruction of a particular folding sequence.

Geometry exposes rest vertices, indexed triangles, edge lengths, signed hinge angles, crease flags, masses, and body-frame landmarks. Another design can replace this factory while retaining the solver and rendering interfaces. A future FOLD importer should build these same data structures and explicitly resolve overlapping panels.

## Structure and folded layers

The renderer-independent solver uses metres, kilograms, seconds, newtons and radians. Sheet properties now explicitly specify density, thickness, machine/cross-direction elastic moduli, shear modulus and Poisson ratio. Grammage and density determine thickness. Material coordinates survive fold compilation and refinement; directional membrane stretch/shear and plate bending follow the original paper grain. Bending scales with thickness cubed. Each physical layer contributes its own mass and bending energy.

XPBD triangle membrane and signed hinge constraints evolve the flexible sheet. Creases have an effective damaged-band width, retained stiffness, yield angle and isotropic hardening. Excess crease rotation produces permanent set during a flight; launch/reset restores the authored shape. Nominal properties are illustrative, not measured paper products.

Overlapping recipe layers use unilateral contact and Coulomb friction. They can slide across triangle boundaries and separate; no tensile glue holds them together. Additional swept vertex/face and edge/edge contact checks nonadjacent material across the sheet, including the Original dart. The zero-thickness kinematic folds retain their exact geometry, so contact uses zero-clearance midsurfaces, with sides inferred from local opening and the previous deformed state. Initially coincident stacks have no authored ordering, and finite projection sweeps still leave intersections; this is not a collision-free guarantee or a certified physical lock. The old bonded-laminate approximation remains an explicit benchmark option. See [the self-contact findings and measurements](docs/SELF_COLLISION_REPORT.md).

The outer physics clock remains **120 Hz**, with at least three substeps (nine for tissue), increasing when a local aerodynamic damping estimate requires smaller steps. During the spring stroke, adaptive stepping also resolves at most one degree of spring phase per substep (six minimum substeps at the default duration), subject to the existing 96-substep cap. An explicit `substeps` setting overrides adaptation. There are 24 alternating constraint sweeps. A near-inextensible strain envelope is a numerical safeguard for the no-tearing game, not a constitutive failure law. Pairwise internal damping preserves pair momentum. There are no flight anchors or attitude controllers. First ground contact or 12 simulated seconds ends the run.

## Coupled whole-wing aerodynamics

A separate exposed aerodynamic quadrature is mapped conservatively onto the flexible material mesh. Twelve span stations solve interacting circulation through trailing-vortex downwash. Geometry-dependent chord, span and incidence produce neighbour interaction, tip unloading and induced drag. Local incidence retains reflex trim moments. Loads and their moments transfer exactly through barycentric weights.

Compiled layers dynamically share near-coplanar exposed area; sliding or peeling updates shielding. Wind/gust/turbulence is evaluated at current patch positions and time. Replay uses the same circulation and exposure calculation on recorded positions and velocities.

This is a reduced **planar lifting-line** model. It blends smoothly to a dissipative separated flat-sheet polar for large incidence, reverse flow, sideslip and steep panels. It does not resolve low-aspect-ratio vortex lift, unsteady wakes, dynamic stall, full 3D sweep interaction or a pressure field. Shielding is local quadrature, not general flow occlusion. The old independent-panel polar remains available as `aeroModel: 'panels'` for comparison.

See [the equations, nominal properties, contact scope and limitations](docs/physics-upgrade.md), [folded-mesh integration](docs/folded-flight.md) and [validation/convergence evidence](docs/physics-validation.md). None of the flight distances or material presets is calibrated against measured throws.

## Visualisation and metrics

Three.js renders the actual deforming aircraft at physical scale in a procedural low-poly field. Its ground shadow follows the deformed plane during flight and replay, softening and fading with altitude. The runway occupies a matching opening in the grass, so the two surfaces do not overlap. Trees and signs are visual landmarks, not collision obstacles; ground contact ends a flight. The workbench drawing is a schematic preview. The field reports horizontal distance from release, simulated air time, centre-of-mass height, and a personal best. The renderer also retains diagnostic mesh/force/heatmap capabilities for future developer views, but these are not part of the game UI.

The physics API exposes deformation RMS with body translation and rotation removed, hinge-angle RMS, edge strain, aerodynamic force, stall fraction, and integrated path length. The benchmark commands compare soft/standard/stiff flights, panel/coupled aerodynamics, elliptic-wing convergence, structural resolution and contact/bonded layers.

## Validation and performance

`npm test` covers mesh topology and mass, hinge gradients, aerodynamic dissipation and winding invariance, translation and gravity, deterministic stepping and reset, stiffness coupling, parameter extremes, scaled paper mass and release height, directional release velocities, the new workbench extremes through ground contact, detached replay data, and the custom-folding geometry studies.

See [the expanded library validation](docs/PLANE_LIBRARY_REPORT.md), [the current performance and worker validation](docs/TASK_REPORT.md) and [self-contact findings](docs/SELF_COLLISION_REPORT.md). Earlier UI increments were checked in an isolated headless Chrome session for folding, keyboard launch, unrestricted pitch, recorded replay with lift/flex overlays, and a 390 px mobile layout without uncaught runtime errors or horizontal overflow. Earlier baseline checks also covered material selection, pointer launch and pause/reset. These are representative checks, not exhaustive device coverage or physical calibration.

Physics remains fixed at 120 Hz with at least three internal substeps, or nine for very light sheets; the spring-phase and aerodynamic damping estimates can increase that count. The browser runs one flight worker with a separate presentation clock. After a valid sling pull stays unchanged for 180 ms, it can precompute up to 250 ms of flight; a matching release adopts that work. Changing the launch or resetting cancels the prediction. The worker idles at its horizon, retains all 120 Hz replay samples, and pauses with the flight or hidden tab. If workers cannot start, computation falls back to bounded main-thread substep scheduling. Device-pixel ratio is capped at 2. Playback can slow under load rather than enlarging the physics timestep. Background-tab time is discarded. A capability message appears if WebGL cannot start.

## Project map

| Path | Responsibility |
| --- | --- |
| `src/geometry/dart.ts` | Effective dart topology, layers, edge and hinge rest state |
| `src/physics/constraints.ts`, `membrane.ts`, `sheet-material.ts` | Directional membrane/hinge XPBD, crease plasticity, internal damping |
| `src/physics/layer-contact.ts` | Sliding unilateral layer contact and Coulomb friction |
| `src/physics/self-contact.ts` | Swept vertex/face and edge/edge detection, broad phase and unilateral response |
| `src/physics/aerodynamics.ts`, `lifting-surface.ts` | Exposed quadrature, coupled circulation, separated fallback and baseline polar |
| `src/physics/simulation.ts` | Free-flight integration, fixed clock, metrics |
| `src/physics/math.ts` | Renderer-independent vector math |
| `src/rendering/viewport.ts` | Three.js cameras and deforming meshes |
| `src/rendering/landscape.ts` | Procedural field, trees, clouds, distance signs |
| `src/main.ts`, `src/style.css` | Folding workbench, launcher gestures, results, responsive interface |
| `tests/physics.test.ts` | Numerical and acceptance regressions |
| `tests/benchmark.ts`, `tests/physics-benchmark.ts` | Flight, convergence and timing reports |

## Investigated implementations and next steps

- [Origami Simulator](https://origamisimulator.org/) and [its source](https://github.com/amandaghassaei/OrigamiSimulator): informed the bar-and-hinge representation and separation of crease/face stiffness. Its GPU folding workflow is not embedded here; a small free-flight CPU solver was simpler for this prototype. No Origami Simulator code was copied.
- [PositionBasedDynamics](https://github.com/InteractiveComputerGraphics/PositionBasedDynamics): the MIT-licensed signed hinge gradient formulation was adapted from its dihedral constraint implementation. See `THIRD_PARTY.md`. Compiling the full C++ library to WASM was unnecessary for this mesh size.
- [Macklin, Müller, Chentanez: XPBD (2016)](https://matthias-research.github.io/pages/publications/XPBD.pdf): compliance-based constraint update.
- [Three.js documentation](https://threejs.org/docs/) and [Vite documentation](https://vite.dev/guide/): rendering and build tooling.

Useful next steps:

1. Calibrate nominal materials, crease hysteresis and aerodynamic response against static deflection tests and measured throws. The analytic lifting-line benchmark is not dart calibration.
2. Add authored layer stacking order, finite thickness and a collision-safe fallback for coupled impacts before arbitrary folded designs or crumpling.
3. Extend the planar lifting-line reduction to sweep/nonplanar vortex geometry and an unsteady wake, while retaining an explicit separated-flow fallback.
4. Improve numerical convergence and profile performance before moving the solver into a Worker or WASM. Strong flutter, added mass, dynamic stall and general flow shielding need a more complete model.

## Licensing

Project code is provided under the MIT license. Adapted components and dependency notices are in `THIRD_PARTY.md` and `licenses/`. No generated art, paid assets, or third-party runtime services are required.

## Plane library and downloads

The picker offers Original dart plus nine full-sheet studies: The Basic, Folded Dart, The Stable, Heavy Nose, Hunting, The Buzz, V-Wing, Underside and Raven. The six newer studies add pleated noses, rolled leading edges, winglets, a square-sheet glider and a landscape-sheet wing. All are independently authored reconstructions, not downloaded templates. The source table, reuse basis, export format and known differences are in [the plane library guide](docs/plane-library.md).

Open **Library & downloads ↓** to download a `.fold` or inspect the selected recipe and its reference. The folding study preserves sheet size, wing settings and material when returning to the game. The collection is a multi-model FOLD with nine original-sheet/folded-state pairs; use individual files for viewers without multi-model support.

## Custom-folding research prototype

Open `/folding-study.html` or follow the link in the game help dialog. Nine independent reconstructions exercise the material-coordinate model with explicit layer selection; The Basic, Basic Dart, and The Stable were its original fixtures. The study supports 3D inspection during animated forward/reverse folds, fold history and FOLD exports. The panels rotate rigidly about the defined creases; this is zero-thickness kinematics, not a contact simulation. The Original dart quick-start uses a grouped dart recipe as its folding illustration while retaining its separate effective flight mesh. Use Fly this design at any step to complete it and launch it in the game. Flight uses the exposed-surface and contact models documented below; it is not a freeform editor or a certified collision-free solver. See [the design and findings](docs/custom-folding.md).

## Folded designs and flex review

The plane picker connects nine fold recipes to the same flexible solver. `src/folding/flight-mesh.ts` triangulates and refines the material panels, preserves their mass distribution, identifies overlapping layers, and builds crease/interior hinges. Dynamic exposed patches share aerodynamic area between coincident panels. Unilateral contact and pressure-limited friction replace rigid attachments by default, allowing sliding and separation. Every material layer contributes its own directional stiffness; the old bonded multiplier is retained only in the explicit comparison mode. Contact preserves zero-thickness recipe geometry and does not resolve general finite-thickness self-collision. See [the compiler model](docs/folded-flight.md).

Explore flight keeps playback, the timeline and camera controls in a compact bottom bar. Open **Inspect** for flex, layers and balance markers; closing the panel leaves enabled overlays visible. The **•••** menu contains takeoff, landing and the score card.

In **Inspect**, **Flex arrows** shows displacement relative to the undeformed plane after removing body translation and rotation. Arrow lengths can be exaggerated 5×–250×; aircraft geometry stays at true scale. Blue wire shows the aligned reference. **Paper layers** colours single-layer areas teal and heavily layered areas orange. **Lift meter** opens a compact corner readout and closes the Inspect panel to keep the aircraft visible. Its × button hides the readout. It shows net lift in newtons, its signed upward component, and upward lift as a percentage of the plane’s weight at the replay time. It sums panel lift vectors before taking their magnitude, so opposing loads cancel; drag is excluded. The estimate uses the recorded deformed geometry and velocity with the flight’s weather settings. 100% means upward lift alone equals weight, not necessarily level flight. **Stall meter** adds another compact corner readout: the exposed-area fraction classified as separated flow and the area-weighted mean absolute local angle of attack. This uses the coupled model’s existing classification, including reversed flow and strong side-slip; it is not a calibrated stall boundary. No aerodynamic load gives an unavailable reading. **Follow** tracks the replay; **Full path** restores the trajectory overview.

## Special materials and weather

The whole purple Special tile opens an illustrated picker for tissue paper, kitchen foil and steel plate, with keyboard navigation and a regular-paper option. Selecting an ordinary paper weight returns to paper. These use explicit nominal sheet properties: tissue is 18 gsm at 60 µm, foil 135 gsm at 50 µm, and steel 7850 gsm at 1 mm. Paper/tissue have directional elastic stiffness; metals use isotropic moduli. Thickness, density and elasticity set mechanics, while effective crease parameters allow permanent bending. These are illustrative presets, not measured products. Tissue uses a resilient craft-sheet tuning: 3/1.44 GPa directional moduli, 40% crease retention, and a 0.9 rad elastic crease range. It keeps its 18 gsm mass and 60 µm thickness; bending rigidity is about 16% of classic paper, with stronger fold retention to avoid immediate crumpling. Material areas set mass. All materials share the same 0–11 m/s nominal launch-speed range; mass and stiffness affect the response during the spring stroke and subsequent flight. Tissue is slightly translucent in folding, flight and replay, with overlapping layers appearing denser. Metal presets also change the rendered surface. Workbench folds remain kinematic; they do not model the work or tooling needed to fold steel.

The Weather button at the launcher opens still air, breeze (1.5 m/s) and blustery (3 m/s) choices. The direction means **where air blows toward**, with up on the compass meaning downfield (+X). Gusts add smooth intermittent bursts up to 1.5 m/s; turbulence adds a deterministic, spatially varying divergence-free swirl field, including vertical motion. The on-field indicator displays local air speed and horizontal direction; the windsock samples the field at its own position. These controls prescribe air velocity rather than solve a fluid or account for tree wakes. Settings stay fixed during a throw, and replay lift analysis evaluates the same air field at the replayed position and time.

Regression checks cover material mass scaling and equal nominal launch impulses, spatial/temporal weather sampling, replay sampling, and full-power gusty crosswind flights for every special material on all four original designs. Very light sheets use smaller physics substeps to keep panel loads from over-stretching the structure.
