# Is the flight model itself a good one?

Measured 2026-11-14 with the [reference solver](reference-solver.md), which removes numerical error from the question. Short answer: the structural model is sound for an in-flight paper sheet, the aerodynamic model gives plausible forces but does not resolve pitch trim at its default resolution, and two layer-mechanics choices are uncalibrated guesses that each move range by 10–20%. None of it has been compared with a real throw, and that is the only thing that would make any of these numbers "ground truth".

## What was measured

**Pitch trim depends on the lattice resolution.** Rigid-shape polars at 6 m/s from the game's aerodynamic kernel, 12 × 4 lattice (the default) against 48 × 8:

| Design | Best L/D | Trim incidence 12 × 4 → 48 × 8 | Lift at trim ÷ weight | Static margin (body lengths) |
| --- | ---: | ---: | ---: | ---: |
| Original dart | 4.5 → 4.2 | −0.7° → −0.5° | −0.2 → −0.1 | 0.02 |
| The Basic | 3.1 → 2.6 | 6.5° → 10.5° | 1.2 → 2.4 | 0.04 |
| Folded Dart | 3.6 → 3.5 | 11.1° → 11.1° | 0.0 | 0.08 |
| The Stable | 4.3 → 4.0 | 0.6° → 0.8° | 0.2 | 0.16 |
| Heavy Nose | 5.5 → 5.4 | 0.3° → 0.4° | 0.0 | 0.10 |
| Hunting | 4.6 → 4.5 | 4.6° → 8.5° | 1.9 → 3.1 | 0.02 |
| The Buzz | 5.1 → 5.0 | 0.1° → 0.1° | 0.0 | 0.14 |
| V-Wing | 6.2 → 5.9 | 6.5° → 6.6° | 0.3 | 0.04 |
| Underside | 7.4 → 6.9 | 0.5° → 3.2° | 0.0 → 1.4 | 0.02 |
| Raven | 6.0 → 5.8 | 0.3° → 0.3° | 0.0 | 0.20 |

Incidence is measured from the mesh's body axis, which on the Folded Dart sits about 11° from the wing plane; its trim is still at zero lift. Lift and drag are nearly converged: best L/D moves by 2–8% (17% on The Basic). The pitching moment is not. Three designs move their trim point by 3–4°, and Underside goes from trimming at zero lift to carrying 1.4 times its weight. In full reference flights the same refinement changes range as follows:

| Design | 12 × 4 | 24 × 8 | 48 × 8 |
| --- | ---: | ---: | ---: |
| Original dart | 8.43 m | 8.04 m | 7.89 m |
| The Basic | 5.53 m | 5.92 m | 5.51 m |
| Folded Dart | 4.12 m | 3.94 m | 3.80 m |
| Underside | 6.25 m | 11.87 m | 10.66 m |

Static margins are 2–5% of body length on half the designs, so the small moment error of a coarse lattice decides where the plane trims. This is the largest model-side uncertainty found, and it is as large as the solver error on the worst designs.

**Most folded designs do not glide by default.** Folded Dart, Stable, Heavy Nose, Buzz and Raven trim at zero lift with `elevator: 0`. Their default flights last 0.7–0.9 s and cover 3.5–4.5 m, close to a thrown stone from 1.8 m at 6 m/s (0.61 s, 3.6 m). Real examples of these planes glide because their trailing edges are bent up, by the folder or by the fold itself. With 6° of elevator the reference flies Folded Dart 5.84 m, Stable 4.48 m, Raven 4.83 m and Underside past 16.7 m.

**The laminate factor is a guess that matters.** Stacked layers bend as one sheet `n` layers thick (`stackCoupling = 1`, total rigidity `n³D`), the glued limit. Loose layers give `nD`. Real folded stacks sit between, held by friction and by the folds at their ends. Reference range against the coupling exponent:

| Design | `stackCoupling` 1 | 0.5 | 0 |
| --- | ---: | ---: | ---: |
| The Basic | 5.53 m | 5.21 m | 5.17 m |
| Hunting | 8.67 m | 8.53 m | 7.69 m |
| Underside | 6.25 m | 6.13 m | 5.35 m |
| V-Wing | 5.28 m | 5.94 m | 6.26 m |

**Stacking order matters on some designs.** Letting coincident layers pick a side by first motion (the game's rule) instead of the fold order changes The Basic's range from 5.53 m to 6.43 m, and other designs by under 2%.

## What was estimated, not measured

- **Added mass of air.** A plate of chord `c` accelerating normal to itself carries about `π ρ c / 4` of air per unit area: 0.10 kg/m² at a 10 cm chord, against 0.08 kg/m² for the paper. Low aspect ratio reduces it, but it is of the same order as the sheet's own mass. The model has none, so wing-flex and pitch oscillation frequencies are too high and gust response too sharp. Steady glide is unaffected.
- **Quasi-steady loads.** Pitch and flex motions at 5–10 Hz have reduced frequency `π f c / V` of about 0.3–0.6, where unsteady lift is 30–40% below the quasi-steady value and lags. Trim and glide angle are unaffected; damping of fast motions is not reliable.
- **Reynolds number.** Chord Reynolds numbers are 4×10⁴ to 10⁵. A fixed profile drag of 0.035 and the attached/separated blend thresholds (12–28°) are reasonable for thin plates in that range but were chosen, not fitted.
- **Bending.** `D = E t³ / 12(1 − ν²)` with 4 GPa and 100 µm gives 3.4×10⁻⁴ N m, a normal value for 80 gsm copier paper. The hinge discretisation reproduces `D` exactly for bending across grid-aligned edges and is mesh-dependent otherwise, by tens of percent on irregular fans.
- **Thickness.** Layers have none. Buzz's nose is 48 layers, 4.8 mm of real paper, modelled as a plane.

## Does the reference need a better model?

For the job it was built for, no. Changes to the real-time solver should be judged against the same equations solved accurately, otherwise solver error and model error cannot be told apart. The baseline table shows there is 16% mean and 54% worst-case solver error to recover before model error is the larger term.

For the question "does the game fly like paper", a better model would not settle it either. The honest next steps, in order of value:

1. **Raise the lattice resolution in the game or fix the moment.** This is cheap to test now: `npm run reference -- --aero-stations 48 --aero-chord 8` shows what a converged lattice does to every design.
2. **Calibrate against throws.** Range and air time for three or four designs from a repeatable launcher, with measured elevator angle, would fix profile drag, the trim defaults and the laminate exponent together. Published low-aspect-ratio thin-plate data at these Reynolds numbers (for example Torres and Mueller's flat-plate wings) would calibrate lift, drag and centre of pressure without a launcher. I could not find a published paper-aeroplane dataset with usable numbers.
3. **Replace the laminate factor with friction between layers of real thickness**, so stack stiffness comes from contact instead of an exponent. This belongs in the reference first, as a check on `stackCoupling`.
4. **Add added mass**, then unsteady wake terms, to the reference only. They change transients, not range, and are the most work.

Fold-order stacking is already in the reference and is worth moving into the game.
