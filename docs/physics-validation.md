# Numerical evidence for the physics increment

Measured locally on 2026-11-12 with Node v26.12.1. Reproduce with `npm test`, `npm run build`, `npm run benchmark:physics` and `npm run benchmark`. [Raw benchmark results](physics-benchmark-results.json) retain the numeric output. These checks validate implementation properties and expose numerical error; they do not calibrate paper flight against measurements.

## Automated checks

**77 tests pass** after rebasing onto `main` at `411a45b`, up from 54 at checkpoint `2f9fef8`; TypeScript checking and the production Vite build pass. Existing acceptance bounds were retained. The old layer-coupling comparison now explicitly uses `layerModel: 'bonded'`, because default sliding contact intentionally does not apply bonded stiffness multipliers.

New coverage includes force and three-axis moment transfer, rigid/Galilean/winding invariance, untouched-wing response to opposite-wing deformation, wingtip unloading, aspect-ratio response, elliptic-wing convergence, reverse/high-angle/tumbling dissipation, dynamic exposure during peeling, contact side/triangle transitions, free separation, friction limits, internal momentum balance, directional material scaling, grain coordinates through refinement, membrane shear, permanent crease set/reversal/reset and replay re-evaluation. Existing tests still exercise complete flights, wind, all special materials on all four designs, unrestricted pitch, mass, launcher energy and kinematic fold geometry.

The high-angle small Stable still needs the numerical strain envelope; residual strain on tiny triangles can exceed its nominal ±2% target with finite sweeps. The model has no tearing and these launches are not evidence that a real sheet would remain intact.

## Rebase integration checks

The rebased branch retains the independent 0–11 m/s launcher, release-height control, translucent tissue, illustrated material picker, shadows, non-overlapping runway, fixed-width Folded Dart controls and compact replay tools. All twelve special-material/design combinations passed full-speed 11 m/s gusty-crosswind launches. Browser checks on port 5174 verified an 11 m/s launch using the coupled/contact defaults, replay lift values against recomputed coupled panel loads, scrubbing/playback, corner-meter dismissal, mobile layout, model switching and Flat/Shallow/Steep controls. The build passes. The numerical benchmark tables below remain the original physics-branch measurements; the rebase did not change their solver settings.

## Main integration: stall readout and resilient tissue

After fast-forwarding the physics branch into main, **83 tests pass** and the production build passes. The stall readout is checked for exposed-area weighting, hidden patches, opposing incidence signs, no-load handling and increasing classification at high incidence. Browser checks confirm replay recomputation, independent corner readouts, mobile layout and reset.

The 18 gsm tissue preset now uses 3/1.44 GPa directional moduli and 0.72 GPa shear modulus, crease retention 0.4, yield angle 0.9 rad and hardening 0.25. Density, thickness and rendered translucency are unchanged. These are game-tuned craft-tissue properties, not product measurements. Ordinary 7.15 m/s launches at 15° pitch and 11.8° elevation gave:

| Design | Flight s | Range m | Peak flex mm |
| --- | ---: | ---: | ---: |
| Original dart | 4.988 | 5.578 | 11.32 |
| Basic | 1.794 | 2.884 | 18.74 |
| Folded Dart | 1.581 | 3.179 | 4.74 |
| Stable | 1.761 | 2.100 | 17.77 |

All landed without permanent crease deformation in these trials. Added regressions bound early deformation and permanent crease set on all four designs; the full-speed gusty-crosswind tests also pass. A browser throw of the Original tissue dart reproduced 5.578 m range and 11.32 mm peak flex, with its 18 gsm weight and 0.7 opacity retained. The remaining numerical and calibration limits below still apply.

## Aerodynamic convergence

An elliptic wing has span 2 m, root chord 0.4 m, speed 8 m/s and incidence 0.05 rad. The comparison uses `CL = 2 pi alpha / (1 + 2/AR)` and `CDi = CL²/(pi AR)`, with the discretized area for AR. This tests the circulation kernel in an attached, planar reference case.

| Stations | CL | Lift error | CDi | Induced-drag error |
| ---: | ---: | ---: | ---: | ---: |
| 8 | 0.24650 | +3.38% | 0.002642 | −13.99% |
| 16 | 0.24310 | +1.76% | 0.002742 | −7.47% |
| 32 | 0.24117 | +0.90% | 0.002799 | −3.82% |
| 64 | 0.24013 | +0.46% | 0.002828 | −1.93% |

The fixed 12-station aerodynamic quadrature was also evaluated on the same 1 × 0.2 m rectangle using 6, 12, 24 and 48 structural span segments. At velocity `(8, −0.6, 0)` m/s, all coupled results were `F = (−0.1315264, 2.6522531, 0)` N to floating-point roundoff. The independent-panel baseline produced `(−0.2138032, 1.6140576, 0)` N. The change is a coupled-model response, not a renamed coefficient. Neither value is experimental validation.

## Flexible-flight resolution

The default Original dart is evaluated at a matched 0.4 s, with identical initial conditions. Positions are metres and deformation is RMS after removing body motion.

| Substeps | Sweeps | Mesh rows/half-columns | Centre X / Y | Deformation mm | Peak edge strain |
| --- | ---: | --- | --- | ---: | ---: |
| Automatic (game) | 24 | 8 / 5 | 1.8166 / 2.0129 | 15.18 | 1.86% |
| 3 fixed | 24 | 8 / 5 | 1.9567 / 1.7543 | 8.76 | 2.00% |
| 6 fixed | 24 | 8 / 5 | 1.6811 / 2.1083 | 12.79 | 0.85% |
| 9 fixed | 24 | 8 / 5 | 1.6461 / 2.1414 | 11.15 | 0.48% |
| 9 fixed | 48 | 8 / 5 | 1.6516 / 2.1540 | 10.45 | 0.18% |
| 9 fixed | 48 | 12 / 7 | 1.6698 / 2.1134 | 11.52 | 0.68% |
| 9 fixed | 48 | 16 / 9 | 1.7151 / 2.0762 | 12.50 | 0.97% |

At fixed mesh, the 6→9-substep centre change is 4.82 cm, versus 44.86 cm for 3→6. Doubling sweeps at nine substeps changes the centre by another 1.38 cm. Automatic game stepping differs from the 9/48 result by about 21.7 cm. **Default flexible flight is not numerically converged.** The strain limiter is active in some coarse runs. Mesh refinement changes both discretization and sampled reflex geometry, and its differences are not monotonically shrinking; this is not proof of a continuum limit. A higher-quality shell mesh and tighter coupled integration remain necessary for quantitative prediction.

## Folded layers and cost

At a matched 0.4 s under the same launch, contact preserves mass but allows more flex than the bonded reference:

| Design | Contact RMS mm | Bonded RMS mm | Contact ms/step | Bonded ms/step |
| --- | ---: | ---: | ---: | ---: |
| Basic | 10.186 | 0.801 | 12.7 | 11.5 |
| Folded Dart | 2.529 | 0.768 | 30.7 | 20.2 |
| Stable | 3.489 | 0.153 | 13.5 | 11.5 |

These timings are single local runs, including startup/JIT effects and diagnostic work. They are not browser-frame guarantees. Folded cases can exceed the 8.33 ms physics budget; the existing fixed clock slows simulated progression instead of enlarging timesteps. Tissue can require additional substeps. This performance limit is explicit; a Worker or further contact optimization is future work.

The separate full-flight Original-dart benchmark, after warmup, measured:

| Stiffness override | Flight s | Range m | Peak flex mm | Mean physics ms/step |
| ---: | ---: | ---: | ---: | ---: |
| 0.2 | 1.417 | 1.665 | 25.40 | 2.91 |
| 1 | 1.964 | 4.952 | 17.66 | 2.75 |
| 5 | 2.881 | 8.828 | 11.52 | 2.68 |

All three landed. Different stiffness still produces different deformation, forces and trajectories without scripted steering. These are simulator outputs, not measured real-paper ranges.

## Browser smoke check

A separate headless Chrome profile used only this worktree's server on port 5174. Original dart and Stable completed keyboard launches after folding; the pitch dial accepted 180° and returned to 15°. Both recorded flights opened in replay, scrubbed to mid-flight, and displayed recomputed lift markers and flex arrows without uncaught runtime exceptions. A 390 × 844 viewport had document width 390 px and no horizontal overflow. This is a representative smoke check, not exhaustive browser/device coverage. The temporary server and browser were stopped afterwards; port 5173 was not used.

## Remaining validation limits

Material constants, effective crease widths/yield angles, Coulomb coefficients, separated-flow coefficients and shielding tolerances need physical measurements. The aerodynamic analytic check covers a planar attached wing; swept low-aspect-ratio paper shapes are a harder regime. Contact is discrete and limited to initially coplanar groups, not finite-thickness continuous self-collision. Game-time membrane strain limiting and finite-sweep contact are approximations with residual error. The implementation is a working increment with these limits exposed, not a calibrated engineering flight predictor.
