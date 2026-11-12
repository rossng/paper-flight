# Plane library task report

## Result

Added six independently authored sheet studies: **Heavy Nose, Hunting, The Buzz, V-Wing, Underside and Raven**. The shared library now exposes **10 playable entries: nine full-sheet recipes and Original dart**. Original dart remains an effective mesh and is explicitly unavailable as a full-sheet FOLD export.

The main-screen **Library & downloads ↓** control downloads the selected completed sheet using current settings, or all nine sheets as an 18-frame multi-model `.fold`. Individual files contain original material coordinates, folded coordinates, topology, recipe history, settings, source attribution and the complete project MIT notice. The study uses the same IDs/settings and exports its currently displayed step. Navigation is relative; downloads use browser Blobs.

The [source/reuse table and limitations](plane-library.md) records the primary research. No external airplane `.fold` assets were imported. An actual Origami Simulator airplane SVG and MIT-licensed FOLD example files were verified, but they do not supply ready-to-fly plane recipes. Fold ’N Fly templates/photos are not redistributed. Cut and accordion-fold references are documented separately and excluded from the playable count. V-Wing starts from an entire square sheet; it does not pretend to preserve the rectangular offcut from the source tutorial.

## Investigation and implementation

Read the original brief (now preserved as `docs/PLANE_LIBRARY_TASK_BRIEF.md`), `README.md`, `docs/custom-folding.md`, `docs/folded-flight.md`, the material folding model/exporter, compiler, study and picker. No applicable `AGENTS.md` was found. The initial implementation stayed in this checkout without agents, merge, push, deployment or publication. A later explicit request authorized rebasing onto `origin/main`, pushing the branch and opening a PR.

- Centralized titles, capabilities and limitations in `src/folding/library.ts`, backed by recipes and their source URLs.
- Added six distinct fold sequences, including layer-selected nose pleats, rolled bands, winglets and landscape-sheet construction. All retain full material area and existing solver principles.
- Added shared export and navigation-setting helpers. New recipes use authored span; existing adjustable recipes retain their controls.
- Added a scrollable picker, main download dialog, cached thumbnails and responsive controls. The study's original-sheet diagram now fits square and landscape sheets.
- Added structural/export/parameter regressions and extended existing animated-fold, compiler and full-flight tests to the new recipes. Capped the normal test script at two workers.
- Export tests caught shared wing-axis arrays being scaled twice at non-default sizes. Replacing axes independently fixes it without changing the flight solver. Initial new test fixtures also required JSON normalization and material-derived stiffness; these were test expectation corrections.

## Initial validation before rebasing

Environment: Node 26.12.1, npm lockfile dependencies, Vite 7.3.8, isolated headless Google Chrome with SwiftShader. Test and browser compute were bounded to at most two workers (one test worker while Chrome ran).

Baseline, before implementation:

- `npm ci` — installed lockfile dependencies.
- `npm test -- --test-concurrency=2` — **83/83 passed**, 68.465 seconds.
- `npm run build` — passed; existing Three.js chunk-size warning (>500 kB).

Implementation validation:

- `node --import tsx --test --test-concurrency=2 tests/folding.test.ts tests/folded-flight.test.ts` — **39/39 passed** at the first integration checkpoint.
- `node --import tsx --test --test-concurrency=2 tests/library.test.ts` — **12/12 passed** after the export fix and again after collection description changes. Tests cover all nine sheets at default, small/narrow/flat/light and large/broad/steep/heavy settings; finite coordinates, material disk topology, edge lengths, area/mass, matching flight vertices, recipe reconstruction, license text, partial exports, collection frames and navigation parameters.
- `npm test -- --test-concurrency=1` — **119/119 passed**, 159.216 seconds while browser validation ran. No failed, skipped or cancelled tests.
- `npm run build` — passed with both HTML entry points and relative assets. The unchanged Three.js chunk is 517.28 kB (130.73 kB gzip); the existing size warning remains.
- `PLAYWRIGHT_MODULE=/tmp/paper-flight-qa/node_modules/playwright/index.mjs node --import tsx tests/browser-library.mjs` — passed under `/paper-flight/`: all **10 designs selected, folded and launched to ground contact**, all **nine main individual downloads** matched expected serialized FOLD data and filenames, all **nine partial study downloads**, collection, non-default settings, keyboard navigation and 390 × 844 layout checked. **Zero uncaught runtime errors or local HTTP failures**. Actual browser distances ranged from 3.3 to 6.9 simulated metres at the keyboard launcher’s defaults.
- Screenshot review found the picker’s last option behind the mobile tool dock. Raised the picker’s parent stacking layer and added a pointer hit-test regression. `PLAYWRIGHT_MODULE=/tmp/paper-flight-qa/node_modules/playwright/index.mjs node --import tsx tests/browser-library.mjs --layout-only` **passed** on the final production build, including pointer selection of the last option, desktop/mobile screenshots, study return links, settings, nine partial exports and collection. No runtime errors or local HTTP failures.
- `git diff --check` — passed.

Representative CPU flights for the six additions used 80 gsm, size/span 1, 8° dihedral, 7 m/s, 15° pitch, 8° elevation and 1.8 m release height in still air. Every run reached ground contact with finite positions:

| Study | Simulated distance (m) | Simulated time (s) | Final maximum edge strain |
| --- | ---: | ---: | ---: |
| Heavy Nose | 4.861 | 1.119 | 0.0138 |
| Hunting | 6.173 | 2.047 | 0.0200 |
| The Buzz | 4.533 | 0.981 | 0.0200 |
| V-Wing | 4.335 | 1.178 | 0.0175 |
| Underside | 4.615 | 0.869 | 0.0200 |
| Raven | 4.383 | 0.821 | 0.0091 |

These are numerical smoke measurements, **not calibrated flight predictions or rankings**. The full-sheet mass is 4.8257968 g at default Letter size/80 gsm; square V-Wing is 3.7290248 g. The animated-fold regressions check intermediate rotations, rigid panels, intact seams and conserved area.

## How to try it

Run `npm run dev -- --port 5193 --strictPort` using a free port, choose any plane in the main picker, and open **Library & downloads ↓**. Change paper, size and wing settings, download the selected plane or collection, then follow **Inspect folds & source**. Undo a fold and export the partial state, or use **Fly this design** to return with the same settings and launch.

Browser acceptance is reproducible with the optional Playwright script `tests/browser-library.mjs`. It needs Playwright installed separately (no runtime dependency was added):

```sh
# Serve the production build beneath an actual repository-path prefix.
mkdir -p /tmp/plane-library-host
ln -s "$PWD/dist" /tmp/plane-library-host/paper-flight
python3 -m http.server 5193 --bind 127.0.0.1 --directory /tmp/plane-library-host
# In another terminal:
PLAYWRIGHT_MODULE=/tmp/paper-flight-qa/node_modules/playwright/index.mjs \
  node --import tsx tests/browser-library.mjs
```

`LIBRARY_URL`, `CHROME_PATH` and `LIBRARY_ARTIFACTS` can override the test URL, Chrome executable and output directory. The actual validation URL is `http://127.0.0.1:5193/paper-flight/`; downloaded files and screenshots are under `/tmp/plane-library-browser/`.

The preview server used for validation was stopped after the checks. Screenshots and example downloads remain in `/tmp/plane-library-browser/` for review.

## Remaining limitations

The six additions are independent, approximate reconstructions of referenced folding ideas; they are not dimensional replicas of downloadable plans. Complete finite-thickness layer order, collision-free guarantees, certified locks, tape/staples, cuts and unsupported accordion operations remain outside scope. Rebased flight includes upstream’s swept self-contact checks, which can still leave crossings after finite projection sweeps. All new designs have representative ordinary-flight coverage, not an exhaustive matrix of every weather/material/launch combination. Internal FOLD edges remain unassigned and no general importer was added. Multi-model viewer support varies; individual files are available. Real-paper construction and measured throw calibration remain future work.

## Rebase and pull-request integration

Rebased onto `origin/main` at `547a4a4`, preserving its swept self-contact, module-worker flight scheduler, finite spring launch and replay inspector. Conflict resolution retained the upstream instantaneous-launch setting only for the analytical free-translation test; representative flights use the new default spring stroke. The upstream performance findings remain in `docs/TASK_REPORT.md`, which links to this report. The original library scope is preserved in `docs/PLANE_LIBRARY_TASK_BRIEF.md`; the upstream root brief remains intact.

The measurements in the earlier table predate this rebase and must not be treated as current spring-launch/self-contact results. Rebased validation is reported separately below.

- `npm test` — **152/152 passed**, 161.953 seconds, with no failed, skipped or cancelled tests. This includes all nine sheet recipes completing representative flights with the current spring-launch and self-contact defaults, the existing worker/contact/material regressions, and FOLD structural/round-trip checks.
- `npm run build` — passed; both pages and the module-worker asset are emitted with relative paths. The existing Three.js chunk-size warning remains.
- `PLAYWRIGHT_MODULE=/tmp/paper-flight-qa/node_modules/playwright/index.mjs LIBRARY_URL=http://127.0.0.1:5194/paper-flight/ LIBRARY_ARTIFACTS=/tmp/plane-library-rebase-browser node --import tsx tests/browser-library.mjs` — **passed** on the rebased production build: ten worker launches reached ground contact, nine individual downloads matched serialized geometry and filenames, nine partial study exports, collection, settings, keyboard and mobile checks passed. No runtime errors or local HTTP failures. Screenshots and downloaded examples are in `/tmp/plane-library-rebase-browser/`.
- `git diff --check` — passed. The temporary production server on port 5194 was stopped after validation; the requested dev server on port 5193 remains available.
