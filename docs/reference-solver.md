# Reference solver

`reference/` holds a second flight solver, written in Rust, that integrates the game's physical model slowly and accurately. It exists to answer one question during development: how far is the real-time solver from the answer its own model implies? It does not run in the browser or in CI.

```sh
npm run reference                      # every design, default launch
npm run reference -- basic dart        # chosen designs
npm run reference -- --speeds 5,6,7    # several launch speeds each
npm run test:reference                 # cross-checks against the game (about 90 s)
```

Both need a Rust toolchain ([rustup.rs](https://rustup.rs)), as the game itself does; the solver has no dependencies and builds in a few seconds. Reference flights are stored in `reference/results/` and reused until the launch, the mesh, the material model or the solver's source changes, so repeated comparisons while working on the game only re-fly the game. `--stored-only` never runs the solver.

## What "reference" means here

The reference is the game's model, as documented in [the physics notes](physics-upgrade.md), solved to a tight tolerance. It is **not** a measurement of real paper: the materials, the aerodynamic law and the friction coefficients are the game's and are as uncalibrated as before. Agreement with the reference means the real-time solver is solving its equations well. It says nothing about whether those equations describe a real throw; [the model assessment](model-assessment.md) takes up that question.

The two solvers share the problem set-up and nothing else. `reference/export.ts` writes the mesh, masses, material constants, hinge rigidities, launch and weather that `Simulation` sets up into a text file. The Rust solver reads that file and has its own implementation of every force and of the time integration.

| | Game (`wasm/solver`) | Reference (`reference/solver`) |
| --- | --- | --- |
| Time integration | Position-based (XPBD), 3 or more substeps per 1/120 s | Implicit BDF2, 64 steps per 1/120 s |
| Elastic solve | Newton with a sparse direct solve to 1 µm, without contact, then 9–12 Gauss–Seidel sweeps; sweeps alone near the floor | Newton with a sparse direct solve, contact included, to 0.1 nm |
| Stretch | Compliant constraints plus a ±2% strain limiter | Membrane energy only; no limiter is needed |
| Air loads | Sampled as often as stability needs and held; shielding once per frame; one quadrature patch per clipped polygon | Every step, shielding every step, quadrature patches of at most 25 mm² |
| Layer contact | Rest-coplanar candidates on a scan schedule, swept contact, contacts carried between substeps | Every vertex against every other panel, every step |
| Stacking side of coincident layers | Chosen by whichever way a pair first opens | From the fold sequence (below) |
| Friction | Per-sweep Coulomb projection | Coulomb with an elastic stick, static and kinetic limits |

Everything in the left column that is there to save time is absent on the right. The vortex lattice, the separated-flow blend, the material constants, the laminate factor for stacked layers, crease plasticity, edge damping and the launcher are the same model in both.

### Fold-order stacking

The game cannot tell which of two coincident layers is on top, so it lets the first relative motion decide. A flap pressed against the sheet it was folded onto can therefore pass through it. The reference derives the order from the recipe (`reference/stack-order.ts`): a flat fold leaves the layers it moved on the side they swung through, above every layer that stayed; layers that moved together keep their order. This orders 84–93% of coincident vertex–panel pairs; the rest touch a panel only along its edge and fall back to the game's rule. `--first-opening-sides` switches the fold order off, which isolates this one modelling difference.

Layers are still ideal: zero thickness, and no nesting of layers around a shared crease. A vertex that overhangs a panel's edge by less than 0.5 mm still bears on that panel's plane, which stops layers creased together from slipping around each other's fold line.

## Why it can be trusted

`npm run test:reference` checks that the reference solves the same model, and that it solves it correctly.

- **Same air loads.** With the game's lattice and one patch per polygon, the Rust lifting surface reproduces the game's nodal air forces to 1 part in 10⁹ on mid-flight states of five design and weather combinations, including tissue in gusts and turbulence.
- **Same elastic energy.** Membrane and bending energy match the game's constraint definitions to 1 part in 10⁹, and the forces are the gradient of that energy.
- **The game's sweep solver converges to it.** Given enough sweeps and substeps the sweeps alone approach the reference, on flights where the stacking side does not matter:

  | Sweeps alone (substeps × sweeps) | Original dart range | V-Wing range | Underside range |
  | --- | ---: | ---: | ---: |
  | Default (automatic × 24) | 3.89 m | 2.58 m | 5.78 m |
  | 12 × 48–60 | 4.76 m | 5.06 m | 6.02 m |
  | 24 × 100–120 | 6.99 m | 5.27 m | 6.02 m |
  | 48 × 200 | 8.03 m | | |
  | **Reference** | **8.43 m** | **5.28 m** | **6.25 m** |

  The Original dart has no layers, so nothing but numerical resolution separates the two solvers there. At 0.25 s the game at 48 × 100 is within 1 cm of the reference.

- **Energy balances.** On the Original dart, launch work in, air work out and stored energy account for the whole flight to within 0.1% of the launch energy. Layered designs lose a further 0.2–9% to friction, crease yield and contact, which the audit does not itemise.
- **Its own numbers are converged.**

  | Setting | Original dart | The Basic | Heavy Nose |
  | --- | ---: | ---: | ---: |
  | 16 steps per frame | 8.430 m | 5.535 m | 4.498 m |
  | 32 | 8.429 m | 5.535 m | 4.498 m |
  | **64 (default)** | **8.428 m** | **5.534 m** | **4.498 m** |
  | 128 | 8.428 m | 5.534 m | 4.497 m |
  | Contact stiffness ÷4, ×4 | | 5.530, 5.544 m | 4.496, 4.498 m |
  | Quadrature patches 100 mm², 6 mm² | 8.399, 8.437 m | 5.513, 5.537 m | |

  Range to first touchdown moves by millimetres with the time step and by under 0.5% with the other numerical settings. Peak penetration between layers is 1–15 µm and peak edge strain below 0.1%.

Solver unit tests cover the sparse factorisation, energy gradients against finite differences, rest-state and rigid-motion invariance, crease plasticity, and the vortex lattice against Helmbold's lift slope.

## Reading the comparison

`npm run reference` flies each launch with both solvers up to first touchdown and prints, per flight: range and air time from each, the distance between the two centres of mass at 0.25 s, 0.5 s and 1 s, the angle between the two nose directions at 0.5 s, and peak in-flight flex. Flights are sensitive to small differences, so late separations grow quickly; the early checkpoints and a spread of launch speeds (`--speeds`) are the stable signals. `--json out.json` writes the table for before/after comparison of a solver change, and `--game key=value` changes a setting for the game's run only (for example `--game iterations=48`).

Default launch (80 gsm paper, 6 m/s, still air), measured 2026-11-14. "Sweeps alone" is the game before its [direct elastic solve](realtime-solver.md) (`--game elasticSolve=sweeps`), which this comparison led to:

| Design | Reference range | Sweeps alone | Game now | Apart at 0.5 s, before → now | Peak flex, reference / game now |
| --- | ---: | ---: | ---: | ---: | ---: |
| Original dart | 8.43 m | 3.89 m (−54%) | 8.35 m (−1%) | 29.3 → 0.4 cm | 13.0 / 13.1 mm |
| The Basic | 5.53 m | 4.89 m (−12%) | 5.41 m (−2%) | 6.4 → 2.6 cm | 3.9 / 3.2 mm |
| Folded Dart | 4.12 m | 3.93 m (−5%) | 4.11 m (0%) | 4.4 → 0.9 cm | 0.2 / 0.3 mm |
| The Stable | 3.52 m | 3.49 m (−1%) | 3.49 m (−1%) | 1.9 → 1.7 cm | 0.3 / 0.7 mm |
| Heavy Nose | 4.50 m | 4.22 m (−6%) | 4.38 m (−3%) | 5.9 → 3.3 cm | 0.4 / 0.7 mm |
| Hunting | 8.67 m | 7.21 m (−17%) | 8.87 m (+2%) | 5.7 → 0.9 cm | 0.2 / 0.9 mm |
| The Buzz | 4.17 m | 3.98 m (−4%) | 4.16 m (0%) | 3.6 → 0.6 cm | 0.1 / 0.1 mm |
| V-Wing | 5.28 m | 2.58 m (−51%) | 5.36 m (+2%) | 19.7 → 0.9 cm | 0.3 / 0.7 mm |
| Underside | 6.25 m | 5.78 m (−8%) | 6.05 m (−3%) | 6.0 → 3.7 cm | 0.2 / 0.2 mm |
| Raven | 3.49 m | 3.48 m (0%) | 3.50 m (0%) | 0.5 → 0.2 cm | 0.0 / 0.0 mm |

With sweeps alone the game was short on every design (mean range error 16%, worst 54%), because the under-converged constraint solve acts as a softer sheet: V-Wing flexed 37.7 mm against 0.3 mm. With the direct solve the mean error is 1.4% and the worst 3%.

Two things other than solver accuracy enter this table. The fold-order stacking is a modelling difference: with `--first-opening-sides` the reference flies The Basic 6.43 m instead of 5.53 m, The Stable 3.50 m instead of 3.52 m and Heavy Nose 4.41 m instead of 4.50 m. And five designs (Folded Dart, Stable, Heavy Nose, Buzz, Raven) trim at zero lift without elevator, so their default flights are close to ballistic and exercise little of the solver; `--set elevator=6` gives them a glide.

The reference takes 25–200 s per flight on one core of an Apple M5, several at once.

## Options

| Option | Default | Meaning |
| --- | --- | --- |
| `--steps-per-frame N` | 64 | Implicit steps per 1/120 s |
| `--patch-area A` | 2.5e-5 | Largest aerodynamic quadrature patch, m². 0 uses the game's one patch per clipped polygon |
| `--aero-stations N`, `--aero-chord N` | the launch's `aeroStations`, `aeroChordPanels` | Vortex-lattice resolution. Raising it answers a different question: how much the 12 × 4 lattice itself costs |
| `--contact-stiffness K` | 2e4 | Penalty stiffness of layer and floor contact, N/m per vertex |
| `--after-touchdown S` | 0 | Seconds to continue after touchdown; negative runs to rest |
| `--first-opening-sides` | off | Use the game's first-opening rule for every coincident pair |

The solver binary can also be run directly: `reference/solver/target/release/paperflight-reference` prints its usage.

## Limits

- It is the game's model. Unsteady aerodynamics, the added mass of air, finite thickness, tearing and real material data are missing from both.
- Contact is vertex against face. Two edges can cross without a vertex passing through a face; the reference does not detect that. Penetration is reported per flight and stays in the tens of micrometres in the flights above.
- Friction and contact normals are frozen over each 0.13 ms step, and a new contact has no friction for its first step.
- After touchdown the flight is dominated by impact and sliding, where penalty contact and the game's projection differ most and neither is validated. The comparison stops at touchdown by default.
- Some flights have steps where Newton stops on its iteration cap rather than its tolerance; the comparison names them.
- Stored flights are tied to the exact problem text. A change to the mesh compiler, materials or launch defaults makes them stale and they are solved again on the next run.
