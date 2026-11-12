The plane-library task adds six studies (nine full-sheet recipes plus the effective dart), main-screen FOLD downloads and a collection export. See [its findings and validation](PLANE_LIBRARY_REPORT.md) and [source/reuse table](plane-library.md). The earlier performance report is preserved below.

# Self-collision performance and speculative flight

2026-11-12 · `codex/physics-performance` · Apple M5 (10 logical CPUs, 24 GiB), macOS arm64, Node 26.12.1, Chrome 157.0.8103.78.

## Integration

Rebased the earlier performance work onto the self-collision worktree's actual source. That worktree had uncommitted changes on `26cc85f`; rebasing onto its branch alone would have omitted the collision implementation. A read-only copy was hash-checked twice and committed locally as `43ef0d9` (`codex/self-collision-performance-base`). The rebased performance checkpoint is `26bd729`, the baseline for this report. The other worktree was not edited. [Source hashes](performance/self-collision-source.json), [collision findings](SELF_COLLISION_REPORT.md), and [earlier performance findings](performance/PRE_SELF_COLLISION_REPORT.md) are preserved. Recovery branch: `codex/physics-performance-before-self-collision`.

The implementation and measurements are organized into focused commits for the requested pull request. The later request to commit and open a PR supersedes the original brief’s instruction to leave work unpublished. Nothing was merged into main or deployed. No additional agents were used. Baseline tests passed **105/105** in 108.94 s and its build passed before further optimization.

## Implementation

- One dedicated module worker owns the simulation, including contact and plastic history. It advances toward a requested simulation time, independently of animation frames, yielding between complete internal substeps after a nominal 12 ms work budget. It schedules no work once it reaches that horizon. An indivisible substep can exceed the budget; it runs off the main thread.
- Rendering has its own clock. The worker targets **250 ms ahead** (with at most one internal substep of overshoot); the display consumes only poses at or before its current presentation time. Slow computation slows playback without skipping physical steps or enlarging the timestep. Background time and long frame debt are discarded. Pause/help/hidden-tab handling freezes presentation and pauses the worker.
- A valid sling pull unchanged for **180 ms** starts the same bounded computation before release. The key includes the design and every launch parameter: pull-derived speed/heading/elevation, pitch, height, material, geometry settings and weather. An exact matching release adopts the existing worker and its recorded prefix. Changes cancel it. Reset terminates it and ignores stale replies. There is at most one forecast/flight worker, and speculative poses, landing, and trails remain invisible until their presentation time.
- Live poses, velocities, forces, aerodynamic panels and plastic offsets travel as detached **Float64** copies. Every completed outer step retains its existing **Float32** replay frame; partial display updates may be coalesced, recording samples may not. Transfers never detach solver storage. Startup failure falls back to the previous bounded main-thread scheduler. Failure after receiving state interrupts the flight, since the display mirror does not own integration history.
- Self-contact uses a flat preorder swept BVH with escape indices, compact adjacency bitsets, reusable narrow-phase scratch, and a retained ordered contact array instead of rebuilding it for every projection sweep. Visit order, candidate decisions, root calculations and projection order are preserved. The Dart adjacency bitsets occupy about 5.3 KiB; their storage grows quadratically with vertex count and would need revisiting for much larger future meshes.

The performance changes preserve the 120 Hz outer clock, adaptive substeps (including tissue's minimum nine and the ceiling of 96), 24 constraint sweeps, both contact detections per substep and four finishing sweeps. No force, material, collision, or recording fidelity was removed by those optimizations. The later elastic-launch increment intentionally changes the launch forces and the minimum substeps during its stroke, as described below. WebGPU/WASM were not needed for this change; the sequential solver remains CPU-bound and has not been ported to a different numerical backend.

## Later launcher and UI increments

Commit `734ea2e` adds the existing elastic-launch work: the plane starts at rest and receives a mass-proportional ideal spring impulse over 125 ms. Gravity, aerodynamics and contact remain active, and actual exit speed is not overwritten. Adaptive stepping includes the upcoming launch impulse and resolves at most one degree of spring phase per substep, subject to the existing ceiling. `launchDuration: 0` selects the previous instantaneous launch. This is a model change, not a performance optimization or a calibrated spring model. The five added launcher tests cover impulse integration, aim/mass invariance, continued external loads, reset and adaptive resolution.

[Launcher comparison data](launcher-benchmark-results.json) cover paper/tissue on Original and Folded Dart at equal nominal speed. All eight flights landed. First-step air loads decrease with the spring profile; flight distance and deformation do not improve uniformly. The replay inspector also receives an SVG chevron and explicit alignment for its active-count badge.

The performance tables and browser evidence below were measured **before** the elastic-launch change, at the behavior preserved by worker checkpoint `00ee89d`. Their exact-state comparisons establish the performance changes’ numerical preservation, not equivalence between the two launch models. Use `--launch-duration 0` to reproduce the headless comparisons against `26bd729`; use the worker checkpoint to reproduce the earlier browser behavior. The current default includes the spring stroke. A post-integration legacy-paper comparison with that explicit zero-duration flag also passed at tolerance 1e-12.

## Profiling method

The [combined baseline CPU profile](performance/self-collision-profile-before.json.cpuprofile) samples all eight paper/tissue/design cases. Sampled self time included hinge projection 12.9%, BVH query 12.3%, overlap tests 3.5%, self-contact callbacks 9.6%, membrane projection 10.4%, rest-layer projection 9.7%, aerodynamic callbacks 6.2%, rest-layer detection 4.7%, and GC 1.8%. This motivated collision traversal/filtering work. The profile includes setup/warm-up and is not an instrumented additive phase breakdown. Its old `context.baseline` label comes from the original harness default; the actual source is the archived combined checkpoint `26bd729`.

The final solver benchmark alternates baseline/candidate within each repeat, serially: two warm-ups, then five measured 48-step (0.4 simulated second) trajectories per case. Paper uses 6 m/s, 8° pitch and still air; tissue uses 11 m/s, 3 m/s crosswind, gusts 1.5 and turbulence 1. Compilation, recording and external diagnostic queries are timed separately. No profiler, own tests or own browser flight ran concurrently with timing. Tests use at most two workers. Other tasks share the machine, so contention and JIT remain sources of uncertainty.

Flattening the tree alone produced small/noisy gains; that intermediate experiment is retained in `performance/self-collision-flat-intermediate.json`. Only the final measurements below include compact adjacency filtering. A worker improves main-thread availability, not the amount of physics required per simulated second. Forecasting spends otherwise idle time on one possible launch and can waste that work when the pull changes.

## Solver measurements

[Final paired data](performance/self-collision-paired.json) include all repeats, step tails, diagnostic costs, mesh sizes and substep counts. Real-time ratio is simulated time / solver wall time; it excludes rendering and transfer. A ratio below one means the solver cannot keep pace with real time.

| Scenario | Vertices / faces | Substeps median / max | Step median / p95 ms, before → after | Real-time ratio, before → after | Median paired time saving (range) |
| --- | ---: | ---: | ---: | ---: | ---: |
| legacy-paper | 89 / 150 | 3 / 4 | 2.64 / 3.98 → 3.02 / 4.11 | 2.879 → 2.770 | 1.2% (-8.2–4.6%) |
| basic-paper | 119 / 216 | 3 / 4 | 9.19 / 11.14 → 8.94 / 11.46 | 0.874 → 0.900 | 3.1% (-2.8–5.3%) |
| dart-paper | 193 / 360 | 3 / 4 | 31.00 / 34.12 → 29.14 / 34.55 | 0.267 → 0.280 | 5.4% (4.3–6.3%) |
| stable-paper | 108 / 186 | 3 / 4 | 18.79 / 20.55 → 17.28 / 20.29 | 0.443 → 0.482 | 7.5% (3.2–12.5%) |
| legacy-tissue | 89 / 150 | 11 / 32 | 14.72 / 38.23 → 14.55 / 37.17 | 0.450 → 0.456 | 1.0% (-4.1–3.1%) |
| dart-tissue | 193 / 360 | 10 / 25 | 97.69 / 202.42 → 92.04 / 189.22 | 0.071 → 0.077 | 8.0% (4.8–9.1%) |
| basic-tissue | 119 / 216 | 9 / 25 | 32.87 / 69.05 → 31.68 / 67.01 | 0.217 → 0.224 | 3.5% (1.1–4.2%) |
| stable-tissue | 108 / 186 | 9 / 25 | 48.30 / 100.67 → 43.53 / 89.82 | 0.151 → 0.168 | 9.2% (7.6–11.3%) |

Saved checkpoints at 0.1/0.2/0.3/0.4 s compare **431,876 finite numbers**, maximum absolute difference **0**, tolerance 1e-12. The comparison covers positions, velocities, forces, trajectory/deformation/strain/stall metrics, crease offsets, adaptive counts, layer-side signs, and self-contact identities, signs and statistics. Statuses match. [Comparison](performance/self-collision-paired.json.comparison.json).

One-minute host load was 3.16 before and 3.47 after. Compiled cases show modest throughput gains; Original results overlap noise, and some tail measures regress. Difficult contact flights remain substantially slower than real time. These numbers do not support a fixed frame-rate or universal speedup claim.

A separate 42-step high-load run applies 11 m/s, crosswind, gusts and turbulence to **both** paper and tissue on all four designs. Another **435,758 finite numbers match exactly** against the combined baseline. This run has one repetition and is numerical validation, not additional throughput evidence. [High-load comparison](performance/self-collision-high-load.json.comparison.json).

## Browser measurements

The real application ran in isolated headless Chrome at 1280×800/DPR 1: one complete warm-up and two measured complete flights for Original/Folded Dart in paper/tissue. After the change, every measured launch creates a fresh worker. Default keyboard launch uses 7.15 m/s, 15° pitch and 11.8° elevation; tissue uses full power and crosswind/gusts/turbulence. These timing launches release immediately, so they do **not** benefit from a held-pull forecast. Before/after browser phases are separate sessions, not interleaved pairs.

| Scenario | Callback p95 ms, before → after (two runs) | Worst callback ms, before → after | Real-time ratio, before → after |
| --- | ---: | ---: | ---: |
| legacy-paper | 16.9, 16.2 → 1.0, 1.5 | 18.7 → 3.5 | 0.817, 0.707 → 0.974, 0.967 |
| dart-paper | 24.3, 24.2 → 0.9, 0.9 | 30.9 → 1.1 | 0.227, 0.189 → 0.264, 0.264 |
| legacy-tissue | 13.8, 15.1 → 0.7, 0.7 | 18.2 → 1.0 | 0.438, 0.381 → 0.612, 0.597 |
| dart-tissue | 23.8, 32.4 → 0.8, 0.8 | 41.0 → 1.1 | 0.063, 0.032 → 0.086, 0.085 |

Typical final callbacks took 0.4–0.6 ms; recorded main-thread solver time was zero. Long Tasks (>50 ms) across the eight measured flights: **1 → 0**. Worst callback: **41.0 → 3.5 ms**. Worst final rAF interval was **33.3 ms**, so these callback numbers are not a fixed display-rate guarantee. The renderer remains around 0.4–0.6 ms typically. Dart/tissue still advances at only about 0.086 simulated seconds per wall second; responsiveness and real-time physics are different outcomes.

All eight final **status/time/center/peak-deformation** records match exactly in Chrome. All four scenarios also passed sustained launch/landing, pause/resume, reset, replay seeking/playback, flex/layers/lift/stall overlays and follow-camera checks. No application runtime exceptions were recorded. [Before](performance/self-collision-browser-before.json), [after](performance/self-collision-browser-after.json).

Separate [forecast browser checks](performance/launch-forecast-browser.json) verified 0.25 s computed while the launcher remained ready at time zero with one trail point, idling at that horizon, exact worker adoption on both keyboard and native pointer release, and a landed pose/velocity/time/peak-deformation match to a direct solve. Pitch changes and reset canceled the old prediction. Forced worker-startup failure still landed through the fallback. The first native-pointer test hit-tested before reset had updated the camera; the harness now waits for that render before locating the sling, and the repeated check passes.

The [production bundle smoke check](performance/worker-production-browser.json) passed with the actual emitted worker asset: launch, landing, detached replay/playback and reset, with zero runtime exceptions. No debug hooks were added to application code.

## Validation and reproduction

PR integration validation: `npm test` **116/116** in 117.82 s, and `npm run build` passed with all launcher, worker and UI changes present. The final elastic-launch default also passed [production launch/replay/reset](performance/pr-production-browser.json) and [held-pull prediction, cancellation and fallback](performance/pr-forecast-browser.json), with zero runtime exceptions. These are functional smoke checks, not replacements for the earlier timing measurements.

- Combined baseline: `npm test` **105/105**, 108.94 s; `npm run build` passed.
- Performance checkpoint: `npm test` **111/111**, 85.62 s; `npm run build` passed. These suite durations are not performance benchmarks. The build emits a separate ~35 kB worker asset. The existing ~517 kB Three.js chunk warning remains.
- Six new worker tests cover exact paper/tissue contact-flight states and replay samples after actual buffer detachment; idle horizons and pause/resume; hidden future poses/trails; forecast adoption; early terminal predictions; stale/duplicate replies; startup fallback and later failure; debounce, cancellation and worker reuse. Existing numerical tests retain the independent high-load transverse-intersection bounds and full-flight/material tests.
- Both eight-scenario numerical comparisons pass at tolerance 1e-12, with zero observed difference across **867,634** finite values. `git diff --check` passes.

Reproduce the paired solver measurements without changing another worktree:

```sh
npm ci
npm test
npm run build
snapshot_dir=$(mktemp -d /private/tmp/paper-flight-selfcontact.XXXXXX)
git archive 26bd729 | tar -x -C "$snapshot_dir"
ln -s "$PWD/node_modules" "$snapshot_dir/node_modules"
npm run benchmark:performance -- --suite contact --launch-duration 0 --source "$snapshot_dir" --compare-source . --baseline 26bd729 --runs 5 --warmups 2 --output /private/tmp/contact-paired.json
npm run benchmark:compare -- /private/tmp/contact-paired.json
npm run benchmark:performance -- --suite contact --launch-duration 0 --load high --steps 42 --source "$snapshot_dir" --compare-source . --baseline 26bd729 --runs 1 --warmups 0 --output /private/tmp/contact-high.json
npm run benchmark:compare -- /private/tmp/contact-high.json
# Optional CPU sampling (run separately from timings):
npm run benchmark:performance -- --suite contact --launch-duration 0 --profile --runs 2 --warmups 1 --output /private/tmp/contact-profile.json
```

Browser commands, in separate terminals; choose unused ports and an isolated Chrome profile. Run browser flights after solver benchmarks/tests finish:

```sh
npm run dev -- --host 127.0.0.1 --port 5197 --strictPort
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless=new --remote-debugging-port=9297 --user-data-dir=/private/tmp/physics-performance-chrome --no-first-run --no-default-browser-check about:blank
npm run benchmark:browser -- --url http://127.0.0.1:5197/ --output /private/tmp/contact-browser.json
npm run benchmark:browser -- --forecast-only --url http://127.0.0.1:5197/ --output /private/tmp/forecast-browser.json
# Serve dist after npm run build, then check the actual bundled worker:
npm run preview -- --host 127.0.0.1 --port 5199 --strictPort
npm run benchmark:browser -- --production --url http://127.0.0.1:5199/ --output /private/tmp/production-browser.json
```

To try it, fold a plane, pull the sling back and hold it still briefly, then release. Changing pitch or height while holding causes a fresh forecast. Try Folded Dart with tissue, full power, crosswind, gusts and turbulence for the expensive case. Pause/resume, Retry and Explore flight remain available. Forecasting is automatic and adds no UI controls.

## Remaining limits

The worker removes solver work from animation callbacks; it cannot make an expensive trajectory inherently real-time. Its finite look-ahead is exhausted when sustained compute throughput is too low. Warm-up/module startup, state packing/transfer, browser scheduling and a second simulation's memory are additional costs. Prediction consumes CPU before a release and discards work if the launch changes. Frame callbacks do not include all worker-message tasks or GPU completion; Long Tasks and frame intervals provide additional but still incomplete responsiveness evidence. No mobile/browser-wide or fixed-FPS guarantee is made.

The collision model remains the one inherited from the self-collision worktree: initially coincident layers lack a global stack order; finite projection sweeps leave intersections; degeneracy, one-ring exclusions, zero thickness and linear swept paths leave uncovered cases. Smaller measured transverse depth is not a no-intersection guarantee. See [the preserved collision report](SELF_COLLISION_REPORT.md). The optimizations preserve these numerical results, not physical calibration.

Future editing during flight must cancel/rebuild the worker or explicitly define state migration. Never resume integration in the display mirror: it intentionally lacks contact/membrane history. The numerical solver remains independently callable through synchronous `Simulation.step()` for tests and offline tools.

The initial isolated browser and test servers were stopped after performance validation. The development server was subsequently restarted at the user’s request; PR-only browser and preview processes are stopped after their checks.
