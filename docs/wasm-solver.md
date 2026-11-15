# The flight solver in WebAssembly

2026-11-14 · Apple M5, macOS arm64, Node 26.12.1, Rust 1.86 stable.

The real-time flight solver is a Rust crate, `wasm/solver/`, compiled to WebAssembly. It replaced a TypeScript solver with the same algorithm, schedule and constants, and is the only implementation: the game, the flight worker and the tests all step through it.

```sh
npm run wasm               # build the solver to src/physics/solver.wasm
npm run benchmark:realtime # simulated seconds per wall-clock second, per design
npm run benchmark:stages   # where the solver's time goes, by stage
```

`npm run dev`, `npm run build` and the `npm test` scripts build the solver first, so a Rust toolchain is needed to work on the game: install one from [rustup.rs](https://rustup.rs) and run `rustup target add wasm32-unknown-unknown`. The crate has no dependencies and builds in a few seconds; `wasm/build.mjs` runs cargo from the crate's directory so that `wasm/solver/.cargo/config.toml`, which turns on WebAssembly's SIMD, applies. `src/physics/solver.wasm` is a build product and is not committed.

## Where the line is

TypeScript (`src/physics/`) compiles the mesh and everything derived from it once per flight: membrane shape gradients, constraint batches, the lifting surface's patches, layer-contact candidates, self-contact adjacency and tree layout, and the elastic solve's ordering and symbolic factorisation. `Simulation` holds that set-up and the state the game reads. Nothing in TypeScript integrates.

Rust does everything per substep: the substep schedule, air loads and weather, the launcher, the direct elastic solve, the membrane, hinge and strain sweeps, layer contact, swept self-contact, floor contact, damping, crease plasticity, touchdown and the landed and finished rules.

- **Stepping.** `Simulation.stepUntil` creates a `SolverFlight` (`src/physics/solver.ts`) on the first step after a launch. It copies the set-up in once, then makes one call per substep. Positions and velocities are written in before each substep and read back after it, so the per-vertex arrays stay authoritative between substeps and may be edited there, as before. The worker still yields between substeps.
- **Single kernels.** `wasm/solver/src/kernels.rs` exports the solver's kernels one at a time: air loads of one state (the replay inspector uses this), the vortex lattice, self-contact detection and projection, layer contact, floor contact, the elastic solve, the constraint sweeps, crease yield, the wind field and the launcher profile. The TypeScript functions and classes with those names (`aerodynamicLoads`, `SelfContactSolver`, `GroundContactSolver`, `ElasticSolver.solve`, `solveHinges` and so on) are thin wrappers over them, so the unit tests exercise the solver that flies.
- **Loading.** `solver.ts` loads `solver.wasm` with a top-level await: by `fetch` in the page and in the worker (one shared, cached file), from disk under Node. The worker is built as an ES module for this, and holds messages that arrive before the solver has loaded.

The crate uses a plain C ABI with staging buffers; there is no wasm-bindgen. One reference routine stays in TypeScript, `solveLiftingLine`, which only tests use.

## How it was checked

The TypeScript solver no longer exists in the tree, so these checks cannot be re-run from it. They were run against the TypeScript solver at `2f00d32`.

**Bit for bit, in a validation build.** A build of the crate that imported the host's `Math.sin`, `cos`, `asin`, `atan2`, `exp` and `pow` and reproduced `Math.hypot` was stepped beside the TypeScript solver, one substep at a time. Positions, velocities, forces, time, status, touchdown, path length, peak deformation, crease offsets, contact statistics, air-load panels, layer sides and active self-contacts were identical at every substep, to the end of settling, for:

- all ten designs at the default launch;
- an instant launch, bonded layers, sweeps alone, the per-face air loads, explicit substeps and sweeps;
- tissue in wind, gusts and turbulence; foil at full power in a gusty crosswind; a trimmed Heavy Nose at 9 m/s; a drop onto the floor without air.

That is 19 flights and about 42,000 substeps. The validation build was then removed.

**The shipping build** uses Rust's own elementary functions and a plain square-root norm, which differ from JavaScript's in the last bit on a few per cent of arguments. Flights are sensitive to that, as they are to any small change:

| Design | Range, TypeScript | Range, WebAssembly | Air time, TS / WASM | Centres apart at 0.25 s | End state, TS / WASM |
| --- | ---: | ---: | --- | ---: | --- |
| Original dart | 8.354 m | 8.354 m | 2.627 / 2.627 s | 4e-14 m | landed / landed |
| The Basic | 5.414 | 5.409 | 1.382 / 1.380 | 3e-7 | landed / landed |
| Folded Dart | 4.110 | 4.115 | 0.834 / 0.835 | 8e-6 | landed / landed |
| The Stable | 3.490 | 3.492 | 0.708 / 0.709 | 2e-5 | landed / landed |
| Heavy Nose | 4.382 | 4.383 | 0.879 / 0.879 | 5e-6 | landed / landed |
| Hunting | 8.870 | 8.861 | 2.271 / 2.269 | 3e-5 | landed / finished |
| The Buzz | 4.156 | 4.157 | 0.835 / 0.835 | 2e-5 | finished / landed |
| V-Wing | 5.365 | 5.360 | 1.129 / 1.128 | 9e-6 | landed / landed |
| Underside | 6.052 | 6.058 | 1.216 / 1.217 | 2e-5 | landed / landed |
| Raven | 3.497 | 3.497 | 0.686 / 0.686 | 3e-5 | landed / landed |

Range and air time agree within 0.13%. After touchdown Hunting now times out where it used to settle, and The Buzz settles where it used to time out; both already did either at neighbouring launches. Against the [reference solver](reference-solver.md) the score is unchanged: mean range error 1.4%, worst 3%, mean separation at 0.5 s 1.5 cm.

**Tests.** `npm run test:full` passes all 221 tests. Two were adapted: the hinge-gradient test now recovers the gradients from one projection of the solver's hinge sweep, and the weather test describes varying wind as weather and a time instead of a callback. The browser UI check (`npm run test:browser-ui`) and the production-bundle flight (`npm run benchmark:browser -- --production`) pass in headless Chrome 157.

## Speed

`Simulation.step` on whole default-launch flights, TypeScript at `2f00d32` against WebAssembly, alternating in one process after a warm-up flight each, medians of three. The first two pairs of columns are simulated seconds per wall-clock second (higher is faster). **Like for like** is milliseconds per frame over the first N frames, with N the shortest airborne stretch of the six flights; later frames are not comparable because the landings differ.

| Design | Whole flight, TS → WASM | In the air | Like for like, ms per frame (N) | Speed-up | Worst frame, ms |
| --- | --- | --- | --- | ---: | --- |
| Original dart | 5.38 → 7.21 | 5.85 → 9.77 | 1.43 → 0.85 (316) | 1.67× | 8 → 7 |
| The Basic | 2.35 → 3.05 | 2.28 → 3.43 | 3.65 → 2.43 (166) | 1.51× | 12 → 11 |
| Folded Dart | 0.97 → 1.33 | 1.08 → 1.64 | 7.72 → 5.09 (101) | 1.52× | 49 → 40 |
| The Stable | 1.56 → 2.23 | 1.50 → 2.27 | 5.55 → 3.43 (85) | 1.62× | 33 → 24 |
| Heavy Nose | 0.65 → 1.02 | 1.08 → 1.71 | 7.72 → 4.86 (106) | 1.59× | 51 → 39 |
| Hunting | 1.06 → 1.96 | 1.48 → 2.32 | 5.63 → 3.60 (273) | 1.57× | 42 → 25 |
| The Buzz | 0.41 → 0.50 | 0.42 → 0.65 | 19.78 → 12.76 (101) | 1.55× | 143 → 98 |
| V-Wing | 1.47 → 2.17 | 1.59 → 2.51 | 5.24 → 3.31 (136) | 1.58× | 36 → 25 |
| Underside | 1.24 → 1.90 | 1.53 → 2.26 | 5.44 → 3.52 (146) | 1.55× | 54 → 30 |
| Raven | 0.39 → 0.60 | 0.52 → 0.83 | 16.16 → 10.08 (83) | 1.60× | 101 → 68 |

The solver is 1.5–1.7× faster per airborne frame. Eight designs now run faster than real time over the whole flight on this machine; The Buzz and Raven still do not.

Where the time goes (`npm run benchmark:stages`), in milliseconds of solver time per simulated second over the whole flight:

| Design | Air loads | Elastic solve | Membrane, hinge and strain sweeps | Layer contact | Self-contact | Floor | Other | Outside the solver | Total |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Original dart | 16 | 31 | 83 | 0 | 6 | 1 | 3 | 3 | 143 |
| The Basic | 25 | 66 | 164 | 16 | 49 | 1 | 3 | 5 | 331 |
| Folded Dart | 40 | 44 | 351 | 103 | 199 | 3 | 7 | 8 | 755 |
| The Stable | 27 | 39 | 129 | 55 | 191 | 1 | 3 | 4 | 448 |
| Heavy Nose | 47 | 32 | 268 | 180 | 435 | 3 | 6 | 6 | 978 |
| Hunting | 34 | 22 | 230 | 80 | 136 | 2 | 4 | 6 | 514 |
| The Buzz | 81 | 58 | 499 | 407 | 915 | 4 | 10 | 11 | 1984 |
| V-Wing | 32 | 22 | 191 | 57 | 152 | 2 | 4 | 4 | 464 |
| Underside | 34 | 32 | 180 | 65 | 193 | 2 | 4 | 5 | 515 |
| Raven | 68 | 60 | 357 | 300 | 824 | 3 | 9 | 8 | 1629 |

- **Outside the solver** is what `Simulation.step` spends copying state in and out each substep: about 1% of the total.
- **Against TypeScript**, measured during the port on three designs with a sampling profiler on the TypeScript side: air loads, contact detection and the elastic solve gained about 2×, while the membrane, hinge and strain sweeps gained about 1.1×. They are tight sequential arithmetic that the JavaScript engine already compiled well, and they are now a quarter to over half of the time. Self-contact is the largest stage on the layered designs, mostly its narrow phase over cached pair lists.
- **Download.** The page's script fell from 63 to 49 kB gzipped and the worker's from 65 to 33 kB uncompressed; `solver.wasm` adds 69 kB gzipped, fetched once and shared by both.
- **Not tried or not helpful.** Building with `+simd128` alone was within 1% of scalar: the kernels are gathers and sequential Gauss–Seidel, which do not vectorise by themselves. (They do once reordered: see [Faster kernels](#faster-kernels-2026-11-15).) Threads were not tried. They need `SharedArrayBuffer`, so cross-origin isolation, which GitHub Pages cannot provide without a service-worker shim, and within a step only detection and air loads are naturally parallel.

## Limits

- Browser runs were in headless Chrome only. Firefox and Safari were not run; both need module workers and WebAssembly's fixed-width SIMD (Firefox 114, Safari 16.4).
- A failed download of `solver.wasm` stops the page's script, as a failed script download would. The worker reports it as a flight error.
- Parameters are read when a flight's first step is taken. Changing `parameters` in the middle of a flight has no effect until the next launch; nothing in the game does that.
- The unit tests of single kernels copy their arguments in and out on every call. That is fine for tests and for the replay inspector, and is not how flights are stepped.

## Faster kernels (2026-11-15)

Same machine. The solver is about 1.5 times faster over whole flights: 1.7 times in the air and 1.4 times after touchdown, from 1.2 times (V-Wing) to 2.0 times (Original dart) by design. Solver time per simulated second, in milliseconds, summed over launches at 5.5, 6 and 6.5 m/s, with the solver as first ported to WebAssembly and this one run back to back for each design:

| Design | In the air | After touchdown | Whole flight | Real-time ratio |
| --- | --- | --- | --- | --- |
| Original dart | 102 → 52 (1.98×) | 192 → 94 (2.03×) | 140 → 70 (2.00×) | 7.2 → 14.3 |
| The Basic | 288 → 141 (2.04×) | 344 → 223 (1.54×) | 315 → 183 (1.73×) | 3.2 → 5.5 |
| Folded Dart | 607 → 383 (1.58×) | 909 → 763 (1.19×) | 799 → 614 (1.30×) | 1.25 → 1.63 |
| The Stable | 433 → 284 (1.53×) | 444 → 333 (1.33×) | 441 → 319 (1.38×) | 2.3 → 3.1 |
| Heavy Nose | 593 → 345 (1.72×) | 1096 → 941 (1.16×) | 948 → 737 (1.29×) | 1.06 → 1.36 |
| Hunting | 431 → 244 (1.76×) | 559 → 394 (1.42×) | 531 → 356 (1.49×) | 1.9 → 2.8 |
| The Buzz | 1552 → 941 (1.65×) | 2314 → 1188 (1.95×) | 2114 → 1163 (1.82×) | 0.47 → 0.86 |
| V-Wing | 390 → 259 (1.50×) | 392 → 352 (1.11×) | 391 → 329 (1.19×) | 2.6 → 3.0 |
| Underside | 429 → 259 (1.65×) | 589 → 413 (1.43×) | 520 → 354 (1.47×) | 1.9 → 2.8 |
| Raven | 1238 → 882 (1.40×) | 1954 → 1598 (1.22×) | 1711 → 1364 (1.25×) | 0.58 → 0.73 |
| Geometric mean | 1.67× | 1.41× | 1.47× | |

Landings differ between the two solvers, so one flight's ground phase is not like for like; that is why three launches are summed. Timings move by about 5% between runs, more on a busy machine. The Buzz and Raven are still slower than real time.

**What changed, in three steps.**

1. *Same arithmetic, same results.* Twenty flights (the ten designs at the default launch, and ten variants covering tissue, foil, weather, an instant launch, bonded layers, sweeps alone, the per-face air loads and explicit substeps) were bit for bit identical to the previous solver, every frame, after each of these:
   - The membrane and hinge sweeps visit constraints in an order that gives the same result as index order (two constraints that share no vertex commute) but puts independent constraints next to each other, and projects two at a time with WebAssembly's 128-bit SIMD (`wasm/solver/src/lanes.rs`). In index order nearly every constraint waits for its predecessor's result. This halves the sweeps where the mesh order allows; Folded Dart and V-Wing are numbered along strips, which leaves long chains and a smaller gain.
   - The self-contact narrow phase lists the pairs whose swept bounds overlap without branching, then tests two pairs at once; about nine in ten stay clear and need nothing more.
   - The vortex lattice builds its influence matrix and eliminates two columns at a time.
2. *Same algorithm, different rounding.* Flights drift from the previous solver's at rounding level (the Original dart by a few 10⁻¹² m after half a second) and, on layered designs, diverge as flights always do under small changes.
   - The elastic solve factorises the matrix as 3×3 blocks, one per vertex pair, instead of by scalar. A solve now converges to 0.1 pm where it used to stall between 0.5 and 1 pm. The factorisation was the largest single cost on lightly layered designs in the air.
   - The membrane and hinge kernels take their denominators from u·u, u·v and v·v and from the face normals' lengths instead of forming every gradient, and the hinge kernel divides five times instead of fourteen.
3. *One schedule change.* After a direct elastic solve the sweeps run a quarter of `iterations` (6 sweeps, 5 during the launch stroke), not half. They only reconcile contact there, and the measurements below do not distinguish 12, 9 and 6.

**Checks.** `npm run test:full` passes all 221 tests and `npm run test:reference` all 15. Against the [reference solver](reference-solver.md): mean range error 1.4%, worst 3.2% (before: 1.4% and 3%); mean separation at 0.5 s 1.5 cm (unchanged). Over 30 launches (ten designs at 5.5, 6 and 6.5 m/s), using `tests/fixtures/intersections.ts` every sixth frame:

| | Before | Now |
| --- | ---: | ---: |
| Peak transverse penetration in the air, mean | 0.23 mm | 0.28 mm |
| Peak transverse penetration after touchdown, mean | 1.59 mm | 1.26 mm |
| Worst edge strain in the air | 0.9% | 1.2% |
| Worst edge strain after touchdown, Underside excluded | 8.9% | 6.8% |
| Flights that settle rather than time out | 26 | 27 |

Mean range per design is within 3 cm of before. Tissue at 11 m/s in gusts and turbulence (Folded Dart, The Stable, Heavy Nose; 12 against 6 sweeps on one build): penetration in the air 0.29 against 0.31 mm, worst strain 0.3% against 0.4%. Underside's edges stretch many times over for a moment after touchdown at all three speeds, before and after alike (worst 17×); that is a fault in its landing, not something these changes introduced or fixed.

**Tried and left out.**
- Reordering the contact projections, or checking two separated contacts at once, gave nothing: those loops are bound by how many contacts they visit, about nine in ten of them separated, not by arithmetic or by dependency.
- Skipping contacts none of whose vertices had moved since their last projection saved nothing either. Enough contacts act in every pass to touch nearly every vertex.
- Dropping the early swept detection on small meshes was 15–25% faster on four designs and left penetration unchanged, but made Underside's landing strain on tissue several times larger.
- Interleaving the hinge sweep with the membrane sweep (which also keeps the result) did not shorten Folded Dart's or V-Wing's chains measurably.

No browser run was made for this change. The solver now needs WebAssembly SIMD, which raises the Safari floor from 15 to 16.4, and `solver.wasm` is 77 kB gzipped, up from 69.
