# Real-time solver

2026-11-13 · Apple M5 (10 logical CPUs), macOS arm64, Node 26.12.1 · base `c5ec0af`.

The solver now keeps pace with real time for most folded designs, including Heavy Nose in the air. The Buzz and Raven are several times faster but still slower than real time. The physical model (forces, materials, constraints, 120 Hz clock, substep rules) is unchanged; the work per substep is organised differently, and a few schedule changes make flights numerically different from before.

## Measurements

`npm run benchmark:realtime` flies every design with the default launch (80 gsm paper, 6 m/s, still air) and reports simulated seconds per second of solver time. It excludes rendering and worker transfer. Before: one run of the same loop on `c5ec0af`. After: second (warm) run.

| Design | Vertices / faces | Before | After | After, in the air | After, on the ground |
| --- | ---: | ---: | ---: | ---: | ---: |
| Original dart | 89 / 150 | 2.31 | 5.2 | 6.6 | 3.8 |
| The Basic | 119 / 216 | 0.53 | 2.9 | 3.2 | 2.8 |
| Folded Dart | 193 / 360 | 0.17 | 1.3 | 1.5 | 1.3 |
| The Stable | 108 / 186 | 0.38 | 1.8 | 1.9 | 1.8 |
| Heavy Nose | 162 / 288 | 0.16 | 0.9–1.1 | 1.2–1.3 | 0.8–1.1 |
| Hunting | 173 / 304 | 0.24 | 1.3 | 1.4 | 1.3 |
| The Buzz | 310 / 580 | 0.06 | 0.3 | 0.35 | 0.3 |
| V-Wing | 119 / 216 | 0.33 | 1.7 | 1.8 | 1.6 |
| Underside | 138 / 238 | 0.21 | 1.0 | 1.4 | 0.8 |
| Raven | 201 / 362 | 0.05 | 0.4 | 0.45 | 0.4 |

Ratios vary by roughly ±10% between runs, and the ground figure depends on how the plane happens to land. Tissue raises the substep count and is slower: Original 2.6, Basic 1.4, Stable 0.8, Folded Dart 0.6, Heavy Nose 0.4.

`npm test` takes about 10 s (previously 53 s) and `npm run test:full` about 30 s on this machine.

These are Node measurements of the solver alone. No browser run was made for this change.

## What changed

Same arithmetic, less overhead. Trajectories matched the previous solver to rounding (about 1e-13 m after a few steps) before the schedule changes below were added.

- The constraint sweeps (membrane, hinge, strain limit, layer contact, self-contact, ground) run on flat `Float64Array` coordinates and structure-of-arrays constraint data. The per-vertex arrays (`positions`, `velocities`, `forces`) remain the public state and are synchronised every substep. The per-vertex functions used by tests wrap the same kernels.
- Swept self-contact keeps dense per-pair state instead of hash maps, an allocation-free cubic root finder (Newton steps inside a bisection bracket), and broad-phase pair lists that are reused until a vertex drifts 3 mm in the mean-translating frame. Each detection still applies the exact per-substep swept-box test to every listed pair, in the original visit order.
- Layer contact skips the projection of a separated, unloaded contact (which was exactly zero) and reads flat candidate data.
- Aerodynamic loads are computed without per-patch temporaries; the shielding lookup uses an integer spatial hash.
- The hinge angle error uses a short series instead of `atan2` when the error is below 0.1 rad (error under 1e-12).

Schedule changes. These alter results.

- **One swept detection per substep on large stacks.** Contacts that still touch, or hover within 0.1 mm on their own side (10 µm after touchdown), stay active from the previous substep and are enforced through every sweep; they act only where the features still overlap. Detection on the last sweep adds fresh crossings. Meshes with at most 12,000 broad-phase pairs (typically Original, Basic, Stable, V-Wing and Underside) keep the early detection as well.
- **Air loads are sampled as often as their stability criterion needs** (the existing `ρVπA/m` estimate) and held between samples, rather than every substep. Layer shielding is refreshed once per frame.
- **Layer-contact candidates are scanned in full every second frame.** Other substeps revisit candidates within a quarter of a face width and 1 cm of their face. Unloaded contacts more than 0.1 mm clear are rechecked every eighth sweep. Separated self-contacts are rechecked every second sweep, or every eighth beyond 30 µm.
- **Extra substeps share a frame's sweeps.** When the launch stroke, floor contact or air loads raise the substep count above the sheet's usual one, each substep runs `24 × usual / substeps` sweeps, but never fewer than 18. Explicit `substeps` settings are unaffected.
- **The worker may run 1 s ahead of the display** (previously 0.25 s), and a held pull precomputes up to that horizon. Lead gained while the solver is faster than real time carries playback through the launch stroke and impact.

## Effect on behaviour

Flights are sensitive to small numerical changes, so individual distances and landing poses differ from before, as they already did between nearby launch speeds. The checks below compare distributions.

Peak transverse penetration between layers, mean of 12 launches at neighbouring speeds, using `tests/fixtures/intersections.ts`:

| Case | Previous schedule | Now |
| --- | ---: | ---: |
| Ordinary launch, Heavy Nose paper | 1.9 mm | 1.7 mm |
| Ordinary launch, Stable paper | 2.8 mm | 2.0 mm |
| Ordinary launch, Folded Dart tissue | 0.6 mm | 0.8 mm |
| High load (11 m/s, gusts), Folded Dart paper | 1.7 mm | 0.9 mm |
| High load, Folded Dart tissue | 1.4 mm | 1.8 mm |
| High load, Stable paper | 6.1 mm | 6.6 mm |
| High load, Heavy Nose paper | 2.9 mm | 3.4 mm |

Run-to-run spread within each cell is about ±30%, so these are equal within noise except that tissue under high load is somewhat deeper. Peak deformation in default flights stays in the same range (Heavy Nose 16 mm before, 18–21 mm now).

All ten designs reach a landed or timed-out end state as before; The Buzz and Hunting sometimes time out instead of settling, which The Buzz already did.

Alternatives that were measured and rejected because they changed behaviour more: dropping the early detection without carrying contacts (penetration up to 3.7× deeper in tissue), carrying contacts on both sides of the surface (stack stiffens; peak deformation falls by 60%), skipping hovering layer contacts on alternate sweeps (tissue penetration 2.3× deeper), a 12-sweep minimum (momentary 300–700% edge strain on Raven during launch), and removing edge–edge tests (penetration more than doubles).

## Limits

- The Buzz and Raven remain 2–3× slower than real time. Their cost is dominated by tens of thousands of edge–edge pairs between coincident layers, each of which needs a swept test every substep.
- Heavy Nose and Underside are close to 1× after touchdown, and the launch stroke runs at about 0.7×. With no held pull before release, the presentation model predicts roughly 0.2–0.5 s of accumulated slow-down over a Heavy Nose flight; with half a second of held pull, under 0.1 s.
- Tissue on the heavier designs is still below real time.
- Slower machines scale all ratios down.

## Reproduce

```sh
npm ci
npm run test:full
npm run benchmark:realtime                 # all designs, paper
npm run benchmark:realtime -- heavy-nose   # one design
npm run benchmark:realtime -- --material tissue
```

## Update after the chordwise vortex lattice (base `9910ecd`)

The lattice, laminated stacks and trim changes slowed the solver again: chordwise clipping doubled the aerodynamic patch count, and stiffer stacks skid for longer after touchdown, which is the most expensive regime (extra floor-contact substeps, each with at least 18 sweeps).

- **One aerodynamic patch per clipped polygon**, at its area centroid, instead of a fan of sub-triangles. Velocity is linear over a face and the normal constant, so the fan only resampled the same field. About a third fewer patches; default flight distances move by under 1%.
- **Eight substeps, not twelve, while a vertex moves faster than 0.5 m/s near the floor.** The 2 mm travel rule still raises the count for fast impacts.
- **Floor-contact cleanup checks convergence from the fourth pass** (previously the eighth), then every fourth as before.

Solver seconds per wall-clock second, same machine, default launch:

| Design | `53beacc` | `9910ecd` | Now |
| --- | ---: | ---: | ---: |
| Original dart | 5.2 | 4.7 | 4.6 |
| The Basic | 3.0 | 2.3 | 2.5 |
| Folded Dart | 1.3 | 0.8 | 1.0 |
| The Stable | 1.9 | 1.4 | 1.5 |
| Heavy Nose | 0.9 | 0.7 | 0.8 |
| Hunting | 1.5 | 1.2 | 1.4 |
| V-Wing | 1.7 | 1.7 | 1.7 |
| Underside | 1.1 | 1.1 | 1.3 |
| Raven | 0.46 | 0.32 | 0.36 |

Whole-flight ratios are not like for like across the three columns: flights now settle sooner, so fewer cheap resting frames dilute the impact frames. Solver time spent behind real time after touchdown (mean of 5.5, 6 and 6.5 m/s launches) fell from 0.88 s to 0.52 s on Folded Dart, 1.76 s to 0.96 s on Heavy Nose and 3.5 s to 2.6 s on Raven.

Peak transverse penetration after touchdown over the same launches: Folded Dart 2.3 mm before, 2.4 mm now; Heavy Nose 1.7 mm before, 2.4 mm now. Peak edge strain stays at 2%.

Rejected: a 12-sweep minimum outside the launch stroke was about 10% faster again but gave 35% momentary edge strain on Raven at first impact (4% with 18 sweeps).
