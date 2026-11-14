# The flight solver in WebAssembly

2026-11-14 · Apple M5, macOS arm64, Node 26.12.1, Rust 1.86 stable.

The real-time flight solver is a Rust crate, `wasm/solver/`, compiled to WebAssembly. It replaced a TypeScript solver with the same algorithm, schedule and constants, and is the only implementation: the game, the flight worker and the tests all step through it.

```sh
npm run wasm               # build the solver to src/physics/solver.wasm
npm run benchmark:realtime # simulated seconds per wall-clock second, per design
npm run benchmark:stages   # where the solver's time goes, by stage
```

`npm run dev`, `npm run build` and the `npm test` scripts build the solver first, so a Rust toolchain is needed to work on the game: install one from [rustup.rs](https://rustup.rs) and run `rustup target add wasm32-unknown-unknown`. The crate has no dependencies and builds in a few seconds. `src/physics/solver.wasm` is a build product and is not committed.

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
- **Not tried or not helpful.** Building with `+simd128` was within 1% of scalar: the kernels are gathers and sequential Gauss–Seidel, which do not vectorise by themselves. Threads were not tried. They need `SharedArrayBuffer`, so cross-origin isolation, which GitHub Pages cannot provide without a service-worker shim, and within a step only detection and air loads are naturally parallel.

## Limits

- Browser runs were in headless Chrome only. Firefox and Safari were not run; both need module workers (Firefox 114, Safari 15).
- A failed download of `solver.wasm` stops the page's script, as a failed script download would. The worker reports it as a flight error.
- Parameters are read when a flight's first step is taken. Changing `parameters` in the middle of a flight has no effect until the next launch; nothing in the game does that.
- The unit tests of single kernels copy their arguments in and out on every call. That is fine for tests and for the replay inspector, and is not how flights are stepped.
