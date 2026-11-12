# Paper self-contact findings

2026-11-12 · base `26cc85f` · Node `v26.12.1`.

Implemented a bounded swept-contact increment, **not a no-intersection guarantee**. It greatly reduces the reproduced Original-dart crossing and reduces peak transverse depth in the sampled compiled recipes. Initially coincident recipe stacks still intersect. These flights are numerical tests, not physically calibrated predictions.

## Investigation and change

- The original `layer-contact.ts` creates no candidates for the Original dart, and only rest-coplanar owner groups for compiled recipes. Endpoint barycentric tests miss fast vertex crossings and edge-only impacts. The compiler preserves material connectivity and coincident layers, but supplies no physical stack order.
- Contact previously ran before `limitEdgeStrain`; that last structural correction could undo contact. Contact now runs last in each sweep. Swept detection includes structural displacement after the first and final sweeps, followed by four contact-only finishing sweeps.
- `self-contact.ts` adds vertex/face and edge/edge detection across nonadjacent material. Refitted swept BVHs cull pairs in a linearly translating frame. Cubic coplanarity roots are isolated using derivative extrema; impact coordinates must lie inside the triangle/segments. Scalar normal projection recomputes geometry each iteration. It does not change material stiffness, masses, thickness, aerodynamic loads, timestep selection or initial recipe coordinates.
- Shared vertices and one-ring material neighbors are excluded to protect connected seams. Unknown coincident sides within 1 nm may open freely; no stack rank is invented. Previously detected touching sides survive into the next substep. Separation is unilateral, and launch/reset clears history. Existing rest-layer friction remains; new swept contacts are frictionless. Explicit bonded mode keeps its previous behavior.
- Corrections preserve mass centre and linear impulse. Unequal-mass tests also verify zero instantaneous torque for each normal projection. Finite-step angular momentum and the existing frozen layer/friction projections still have numerical error.

## Reproduction and measurements

The diagnostic runs **42 outer steps / 0.35 s**, starting at 11 m/s, with wind `(0,0,3)` m/s, gusts 1.5 and turbulence 1, for paper and tissue on all four designs. Before changing the solver, the Original paper dart first intersected at step 17 and reached **13.6608 mm** transverse depth. Recipe intersections already appeared at step 1: first local opening does not establish a consistent global layer order.

`tests/fixtures/intersections.ts` independently checks proper transverse triangle intersections at each outer-step endpoint. It excludes shared vertices, coplanar overlap and touches within 0.1 µm. Its depth is the smaller endpoint distance of a crossing edge to the opposing triangle plane; it is **not** a global minimum separation or a swept certificate. Peak pair count and peak depth may occur at different times. Full-flight tests additionally check finite state, strain and ordinary fold retention, but do not certify absence of intersections between samples or later in flight.

Paired baseline/current measurements ([raw JSON](self-contact-benchmark-results.json)):

| Material / design | Peak depth mm, before → after | Peak intersecting pairs, before → after | Mean ms / outer step, before → after |
| --- | ---: | ---: | ---: |
| Paper / Original | 13.661 → **0.021** | 12 → 2 | 7.50 → 9.87 |
| Paper / Basic | 10.627 → 3.363 | 191 → 227 | 21.07 → 32.34 |
| Paper / Folded Dart | 2.695 → 1.068 | 902 → 1182 | 51.75 → 86.86 |
| Paper / Stable | 18.785 → 8.159 | 639 → 906 | 17.19 → 40.73 |
| Tissue / Original | 0 → 0 | 0 → 0 | 25.11 → 28.02 |
| Tissue / Basic | 9.775 → 8.131 | 190 → 218 | 36.93 → 66.77 |
| Tissue / Folded Dart | 2.326 → 1.252 | 764 → 1072 | 138.95 → 375.23 |
| Tissue / Stable | 23.435 → 9.349 | 730 → 734 | 58.42 → 131.35 |

Every case remained finite and flying at 0.35 s. Peak edge strain in these changed runs ranged from 0.23% to 2.38%. The Original-paper depth fell by about 99.85%, but its residual two pairs still rule out a collision-free claim. Tissue-Original deformation was unchanged in this sample (9.991 mm final flex); contact does not globally freeze the sheet. Compiled intersections remain substantial, and pair counts often increase. These results verify reduced depth in these launches, not universal improvement under every load.

The paired timings show roughly 1.1–2.7× cost, with particularly expensive tissue stacks; most configurations exceed an 8.33 ms outer-step budget. An allocation-heavy first implementation was profiled and replaced with scalar narrow-phase/projection arithmetic and translating-frame bounds. These measurements predate the [worker and performance improvements](performance.md). Timing includes startup/JIT and shared-machine contention; use the paired command below for another local measurement.

## Validation and commands

The six new high-load bounds were replayed against the original Simulation constructor: **all six failed**, reproducing 13.661 / 10.627 / 2.695 / 18.785 / 2.326 / 23.435 mm for Original-paper, Basic-paper, Dart-paper, Stable-paper, Dart-tissue and Stable-tissue respectively. The isolated tests cover a fast vertex ending outside the target footprint, an edge-only crossing, multiple coplanarity roots, unequal-mass impulse/torque balance, rigid/winding invariance, neighbor exclusions, initial coincidence, separation/recontact, broad-phase culling and deterministic reset.

The final command `npm test -- --test-concurrency=1` passed **97/97 tests** in **142.84 s**, including all existing complete special-material/gusty flights, ordinary tissue fold-retention tests and folding geometry tests. The new high-load bounds pass on the changed solver. `npm run build` passed TypeScript checking and the Vite production build; Vite reports a 517.28 kB Three.js chunk warning. `git diff --check` passed. No UI or folding-animation code changed, so no browser/UI check was required or claimed.

Re-run the normal checks and current benchmark:

```sh
npm ci --no-audit --no-fund
npm test -- --test-concurrency=2
npm run build
npm run benchmark:contact
```

For an exact baseline comparison without altering the checkout, materialize the dispatched Simulation module with absolute imports to its unchanged supporting modules:

```sh
python3 - <<'PY'
from pathlib import Path
import subprocess, re
root = Path.cwd()
s = subprocess.check_output(['git', 'show', '26cc85f:src/physics/simulation.ts'], text=True)
def absolute(m):
    path = (root / 'src/physics' / m[1]).resolve().with_suffix('.ts')
    return "from '" + str(path) + "'"
s = re.sub(r"from '(\.[^']+)'", absolute, s)
Path('/tmp/paper-self-collision-baseline-simulation.mts').write_text(s)
PY
node --import tsx tests/self-contact-benchmark.ts /tmp/paper-self-collision-baseline-simulation.mts
PAPER_CONTACT_BASELINE=/tmp/paper-self-collision-baseline-simulation.mts node --import tsx --test --test-concurrency=1 --test-name-pattern='high-load' tests/self-contact.test.ts
```

The last command intentionally exits 1 on the old solver. The paired benchmark alternates baseline/current per design/material and times only `Simulation.step`, excluding compilation and the independent intersection diagnostic. Results are wall-clock samples on a shared machine with JIT and contention, not device/frame guarantees.

## Remaining limits and trying the result

- Initially coincident stacks are still unordered; independent first openings can conflict near folded seams. Existing crossings are not untangled. Some compiled cases have **more** intersecting triangle pairs despite smaller peak depth. Local side history and simultaneous contacts can overconstrain an ambiguous stack.
- Detection assumes linear vertex paths between snapshots. It does not certify intermediate nonlinear projection paths, collision response created after the final detection, repeated impacts, or arbitrarily fast rotations. Even detected double crossings can escape endpoint response. A fail-safe impact-zone/step-subdivision method is not implemented.
- Degenerate triangles, almost parallel/collinear edges, coplanar sliding overlap and one-ring exclusions leave uncovered cases. Contact is zero clearance; paper thickness affects mechanics, not collision spacing. There is no finite-thickness stacking or guaranteed locking flap.
- Normal correction can leave residual strain after the final limiter pass. Finite projection sweeps leave residual intersections. More sweeps alone do not resolve ambiguous initial ordering.
- Swept checks cost additional CPU, especially on layered Dart/Stable meshes. The fixed clock can therefore slow playback further; a better ordered-stack representation and further solver optimisation remain necessary. See the [later performance measurements](performance.md) for worker results.

To try it, run `npm run dev -- --port 5187 --strictPort` on an unused port, keep the default contact model, and launch the Original dart or a compiled recipe at full power with blustery weather, gusts and turbulence. Tissue and ordinary launches retain the existing controls. Use replay/flex inspection to view the result, and use `npm run benchmark:contact` for measured intersections rather than inferring safety from the image.
