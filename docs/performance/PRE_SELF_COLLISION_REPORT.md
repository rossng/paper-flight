# Physics performance task report

Worktree: `codex/physics-performance`. Baseline: `26cc85fe47f0d5283e311af8a4e7ef657a6abebe`. The implementation and evidence are local, uncommitted changes; nothing was merged, pushed, deployed or published. No other worktrees or agents were used.

## What changed

- Browser integration can suspend a 120 Hz outer step between its existing internal substeps. A 12 ms physics budget returns control to rendering/input after the next complete substep. Synchronous `Simulation.step()` still completes the same outer step. Pausing retains suspended work, reset/launch discards it, and recording/trail/peak-deformation updates occur only on completion. Catch-up debt is capped at 100 ms and excess wall time is counted in `FixedClock.droppedTime`.
- Aerodynamic exposure queries prepare each deformed triangle's geometry once per evaluation, reuse the owner set, and avoid per-query vector/bucket copies. The cache is local to one call: sliding, peeling, weather, replay and subsequent substeps still use current geometry and exposure.
- Membrane normal modes calculate only the derivative they use. Compliance coefficients are prepared once per outer step, including the actual adaptive timestep and current material properties. The Gauss–Seidel order and shear calculation are preserved.
- Previous-position and membrane-multiplier scratch buffers are reused. Recordings write directly into detached Float32 arrays. Fixed-arity vector lengths avoid spreading arrays. Inactive edge-strain inequalities skip their zero-impulse position writes.

The 120 Hz clock, adaptive substep rule (including tissue's minimum nine), 24 sweeps, material constants, aerodynamic model, contact generation/projection and strain envelope are retained. Scheduling reduces the amount of wall time spent in one callback; it does not reduce the physical work per simulated second. Under load, flight runs slower than real time. A single substep remains indivisible and can exceed the budget.

## Profiling method

Measurements use one serial benchmark process, without this task's tests/browser running concurrently. Tests are capped at two workers. Machine: Apple M5, 10 logical CPUs, 24 GiB RAM, macOS Darwin 25.6.0 arm64; Node v26.12.1. Other independent tasks share this machine. Load and timing changed substantially during the session, so absolute numbers are observations, not dedicated-machine guarantees.

`tests/performance-benchmark.ts` warms two complete 48-step trajectories per implementation/scenario, then measures five repeated 0.4 s trajectories. The paired mode alternates baseline/candidate order within each repeat in a single process. Setup/compilation, external metrics, deformation-field queries and recording are timed outside solver steps. Internal peak-deformation accounting remains inside the solver. Reports include mesh sizes, actual adaptive substep counts, per-step median/p95/max, each run total, simulated/wall-time ratio, runtime and host load. Compressed numerical checkpoints accompany the timing JSON.

Initial CPU sampling attributed 19.5% to hinge projection, 16.3% to membrane projection, 13.3% to contact projection, 6.9% to contact detection and 5.7% to the strain envelope. Lifting-surface callbacks accounted for 11.1%, vector helpers 7.4%, and GC 2.1%. These are sampled self times including warm-up/setup, not an additive breakdown of instrumented phases. See `performance/baseline.json.cpuprofile`. Constraint work and exposure queries warranted attention; dense lifting-line solution, diagnostics and recording were much smaller costs. Rendering is evaluated separately below.

## Headless results

Final interleaved measurements are in [paired-final.json](performance/paired-final.json). No CPU profiler was running during these timings. The one-minute host load was 2.66 before and 3.12 after; earlier measurements taken during much higher contention are retained in `baseline.json` and the explicitly marked intermediate `paired.json`. Those earlier absolute timings are not used to calculate the final speedups.

Each step advances 1/120 s. All cases use 24 sweeps, default adaptive substeps, coupled exposure aerodynamics and contact. Ordinary cases use 6 m/s, 8° pitch, zero elevation and still air; demanding tissue cases use 11 m/s, 8° pitch, 3 m/s crosswind, gusts 1.5 and turbulence 1. The real-time ratios below exclude rendering, recording and external diagnostic queries.

| Scenario | Vertices / faces | Substeps median / max | Step median / p95 ms, before → after | Real-time ratio, before → after |
| --- | ---: | ---: | ---: | ---: |
| Original / paper | 89 / 150 | 3 / 4 | 2.26 / 3.19 → 2.02 / 2.91 | 3.419 → 3.766 |
| Basic / paper | 119 / 216 | 3 / 4 | 7.35 / 8.05 → 6.56 / 8.38 | 1.123 → 1.240 |
| Folded Dart / paper | 193 / 360 | 3 / 4 | 17.85 / 18.83 → 16.40 / 18.21 | 0.458 → 0.499 |
| Stable / paper | 108 / 186 | 3 / 4 | 8.46 / 9.85 → 7.38 / 8.97 | 0.972 → 1.114 |
| Original / tissue | 89 / 150 | 11 / 32 | 13.84 / 35.13 → 13.03 / 30.92 | 0.487 → 0.530 |
| Folded Dart / tissue | 193 / 360 | 9 / 25 | 60.32 / 132.86 → 55.51 / 117.83 | 0.116 → 0.125 |

Median per-pair reduction in total solver time over 48 steps:

| Scenario | Median saving | Range across five pairs |
| --- | ---: | ---: |
| Original / paper | 8.8% | 8.3–9.6% |
| Basic / paper | 9.8% | 7.2–12.2% |
| Folded Dart / paper | 7.9% | 7.3–8.4% |
| Stable / paper | 12.5% | 9.2–15.9% |
| Original / tissue | 8.5% | 6.3–9.3% |
| Folded Dart / tissue | 7.9% | 5.7–14.5% |

Every final paired run improved total solver time, but not every tail statistic improved (Basic's p95 increased). This supports a modest throughput improvement, not a universal latency bound. Compiled Folded Dart and tissue remain CPU-heavy. The final Folded Dart has 528 hinges, 552 edges, 646 aerodynamic patches and 26,128 contact candidates; contact still runs in every substep/sweep.

Numerical comparison at 0.1, 0.2, 0.3 and 0.4 s in all six cases checked **313,320 finite numeric values with maximum absolute difference 0**, against an allowed tolerance of 1e-12. This includes every vertex's position, velocity and force, trajectory/force/deformation/strain/stall metrics, peak deformation, crease plastic offsets, adaptive substep counts, contact counts and candidate side signs. Statuses match. See [comparison](performance/paired-final.json.comparison.json) and the accompanying compressed states. This verifies preservation of this implementation, not the physical accuracy of its contact model.

## Browser results

Actual full-flight measurements, two measured flights after one warm-up per scenario. Median ranges span the two runs; p95/max columns use the worse run. “Real time” is simulated seconds / elapsed browser-frame seconds, so 1.0 means keeping pace. Ordinary launches use the UI's 7.15 m/s, 15° pitch and 11.8° elevation; tissue uses 11 m/s, 16° elevation, 3 m/s crosswind, gusts and turbulence.

| Scenario | Median callback ms, before → after | p95 ms, before → after | Max ms, before → after | Real-time ratio, before → after |
| --- | ---: | ---: | ---: | ---: |
| Original / paper | 11.6 → 9.3 | 22.8 → 13.8 | 39.5 → 14.2 | 0.997 → 0.997 |
| Folded Dart / paper | 292–295 → 13.5–18.4 | 417.7 → 22.7 | 417.7 → 25.5 | 0.333–0.350 → 0.278–0.339 |
| Original / tissue | 184.8–186.6 → 13.2 | 208.5 → 14.0 | 430.3 → 14.7 | 0.522–0.524 → 0.503–0.506 |
| Folded Dart / tissue | 987.6–1069.7 → 13.0–13.2 | 1603.0 → 18.7 | 1603.0 → 19.8 | 0.093–0.097 → 0.115 |

Before: 190 Long Tasks across these eight measured flights (0 ordinary Original, 32 ordinary Folded Dart, 130 tissue Original, 28 tissue Folded Dart). After: **zero Long Tasks** in the same eight flights. This is a large responsiveness improvement, not a fixed-frame-rate promise. The worst post-change frame interval was 49.9 ms; a callback and a displayed frame are different measurements. Demanding cases remain slower than real time. The budget can reduce throughput when yielding leaves CPU time unused; the Folded Dart/paper repeats show that tradeoff and shared-machine variability.

Renderer self-inclusive callback medians were roughly 0.7–1.1 ms before and 0.5–0.8 ms after; neither renderer nor recording dominated the long frames. No renderer rewrite was warranted. Ordinary flight with the 12 ms budget remained near real time while still yielding frequently.

Both repeats of each scenario have **exactly equal final simulation time, center and peak deformation** before/after in the same Chrome runtime. All cases landed. Pause froze simulation time; resume advanced it; reset restored ready state and closed replay. Replay sought mid-flight, played forward, showed numerical flex/lift/stall readouts, enabled layers/follow and returned to a fresh launch. There were no application runtime exceptions. The initial profiling-harness attempts exposed Vite module-version and disabled-button timing assumptions; these were corrected, and only completed successful runs appear in `performance/browser-after.json`.

Evidence: [before](performance/browser-baseline.json), [after](performance/browser-after.json). Browser timings are separate from headless solver throughput and include the cost of returning to rendering between internal substeps.

## Validation

- `npm ci`: passed (27 packages installed).
- Baseline `npm test -- --test-concurrency=2`: **83/83 passed**, 43.28 s. Baseline `npm run build`: passed. No baseline test failures were found.
- Final `npm test`: **91/91 passed**, 44.76 s, with the script now explicitly limited to two test workers. Final `npm run build`: passed, including all benchmark/test TypeScript. The existing Vite warning about the 517 kB Three.js chunk remains.
- `npm run benchmark:performance -- --source /private/tmp/physics-performance-baseline --compare-source . --output docs/performance/paired-final.json`: passed, six scenarios, two warm-ups and five alternating measured pairs each, one worker.
- `npm run benchmark:compare -- docs/performance/paired-final.json`: passed; all 313,320 compared numbers finite and identical.
- `node tests/browser-performance.mjs --output docs/performance/browser-after.json`: passed all four scenarios, one warm-up plus two measured complete flights each, replay and pause/reset checks; zero runtime errors.
- `git diff --check`: passed.

The eight added regression tests cover synchronous versus yielded states and recordings for paper/tissue on Original and Folded Dart, reset during suspended work, synchronous completion of partial work, bounded clock debt, pause/resume, final landing capture, and material/timestep coefficient refresh. Existing tests still cover coupled forces/moments, exposure peeling, contact separation/sliding/friction, crease response and finite complete flights across all materials/designs.

## Reproduction

```sh
npm ci
npm test
npm run build

# Archive only this worktree's baseline; do not switch another checkout.
mkdir -p /private/tmp/physics-performance-baseline
git archive 26cc85fe47f0d5283e311af8a4e7ef657a6abebe | tar -x -C /private/tmp/physics-performance-baseline
ln -s "$PWD/node_modules" /private/tmp/physics-performance-baseline/node_modules
npm run benchmark:performance -- --source /private/tmp/physics-performance-baseline --compare-source . --output /private/tmp/paired.json
npm run benchmark:compare -- /private/tmp/paired.json
# Optional V8 CPU profile; avoid running other local compute at the same time.
npm run benchmark:performance -- --profile --output /private/tmp/current-profile.json
```

The archive/link commands assume a fresh temporary directory. `--case dart-tissue`, `--runs`, `--steps` and `--warmups` bound or customize the benchmark. The baseline is an archive of the original source, with shared installed dependencies, rather than an assumption that another task's checkout is unchanged.

Browser profiling uses the real launcher and application frame loop over CDP, with no production debug hooks or profiling dependencies:

```sh
npm run dev -- --host 127.0.0.1 --port 5197 --strictPort
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless=new --remote-debugging-port=9297 --user-data-dir=/private/tmp/physics-performance-chrome --no-first-run --no-default-browser-check about:blank
npm run benchmark:browser -- --url http://127.0.0.1:5197/ --cdp 9297 --output /private/tmp/browser-performance.json
```

Use unused ports and an isolated browser profile. The browser harness measures one warm-up flight and two complete flights per case at 1280×800/DPR 1, records rAF callback/interval distributions and Long Tasks, and separately times solver, renderer, recording and diagnostic calls. Nested diagnostic times overlap solver/renderer time. `steps` counts solver API calls; with yielding, some calls finish only part of an outer step. Chrome 157.0.8103.78 was used. Headless WebGL and JavaScript callback times do not measure physical display presentation, GPU completion or performance on mobile hardware.

To try the change interactively, open the printed local URL, choose a design/material, finish folding, and launch with Space or the sling. Folded Dart plus tissue, full power, blustery crosswind, gusts and turbulence stresses the scheduler. Pause/resume, Retry, and Explore flight should remain usable while physics retains its existing quality. The browser harness also exercises replay scrubbing/playback, flex/layers/lift/stall overlays, camera follow and reset.

## Integration and limits

`layer-contact.ts` is unchanged. The self-contact branch will likely touch the same integration loop in `simulation.ts`; retain its contact changes inside each full internal substep, preserve the previous-position snapshot for that substep, and yield only after contact/constraints/velocities/status checks complete. `constraints.ts` has only a no-op early return in `limitEdgeStrain`; keep any collision changes to that function when resolving a merge. Re-run numerical comparisons and browser timings after integration: more comprehensive contact can increase the indivisible substep cost.

Reset cancels the pending iterator before replacing solver state. Future mesh/material editing during a flight must rebuild/reset the simulation, or explicitly define how a pending step should be invalidated. Current controls already rebuild or restrict such changes. The aerodynamic geometry cache never survives an evaluation; prepared membrane coefficients never survive an outer step.

A worker was not introduced: cooperative substep scheduling addresses the measured catch-up stalls without state transfer, replay ownership or control-message complexity. It does not remove total CPU cost. Larger meshes, expensive contact, GC and contention can still produce long frames; worker execution remains a possible next step if one internal substep becomes too expensive. Renderer diagnostics could be gated further, but they were not the dominant flight cost in this profile.

The isolated Vite server and Chrome test process were stopped after validation.

The existing contact, convergence, material and aerodynamic limitations still apply. These flights are not physically calibrated predictions.
