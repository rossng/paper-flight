# Plane library and FOLD downloads

The picker contains **10 playable designs: nine complete sheet recipes and the legacy effective dart**. Six sheet studies were added: Heavy Nose, Hunting, The Buzz, V-Wing, Underside and Raven. They introduce different fold sequences and material distributions, rather than presets for the same mesh. Their names identify reference ideas, not exact digital replicas or verified flight performance.

Choose a plane, adjust its workbench settings, then open **Library & downloads ↓** on the main screen. **Download this plane .fold** finishes that recipe with the current settings, without changing the workbench's fold progress. **Folding reference** opens the original instructions for that design. Use **Next fold** and **Back** in the workbench to inspect the folds. Downloads work without a server API.

**Download all sheet designs .fold** creates one FOLD 1.2 `multiModel` document, containing nine independent original-sheet/folded-state frame pairs. Some external viewers support only one model/frame; use individual downloads there. This app does not implement generic FOLD import.

## Primary sources and reuse basis

Checked 12 November 2026. Coordinates, labels, procedural history and thumbnails are independently authored here. All generated geometry carries the project's full Blue Oak Model License 1.0.0. No third-party photos, printable templates or downloaded meshes are bundled. Reference designers are not inferred from the tutorial publisher.

| Reference / asset | What is implemented or found | Reuse basis / limitations |
| --- | --- | --- |
| [The Basic](https://www.foldnfly.com/0.html), [Basic Dart](https://www.foldnfly.com/1.html), [The Stable](https://www.foldnfly.com/2.html) | Existing corner-fold, swept-dart and selective-lock reconstructions | Existing independently authored Blue Oak recipes; source artwork not included. Stable's flap is geometric, not a certified lock. |
| [Heavy-Nosed Plane](https://www.foldnfly.com/7.html) | `heavy-nose`: dart shoulders followed by a nose reversal and two selective pleat folds | Independent coordinates; authored 22 mm wing root. The returning flap selects nose layers, leaving the tail intact. |
| [Hunting Flight](https://www.foldnfly.com/6.html) | `hunting`: three leading-edge rolls, corners, two winglets, wing deployment | Independent study with 25.4 mm fold bands and 12 mm winglets. Winglets are prepared before deployment. No optional tape. |
| [The Buzz](https://www.foldnfly.com/3.html) | `buzz`: corner folds, nose reversal, shoulders, reversed nose hem, wings | Independent geometry with a 13 mm hem and root. Optional tape and tail trimming adjustments are absent. |
| [V-Wing](https://www.foldnfly.com/39.html) | `v-wing`: square-sheet nose reversal, second corner folds, tapered wing roots | Starts from a **215.9 × 215.9 mm square**, retaining its entire area. The source begins by trimming a rectangle; that cutting operation is not represented. No staple. |
| [Underside Plane](https://www.foldnfly.com/27.html) | `underside`: landscape sheet, two leading bands, paired local corner folds, nose turn and broad wings | Independently chosen corner dimensions; approximate reconstruction. No optional tape. |
| [Raven](https://www.foldnfly.com/50.html) | `raven`: dart shoulders, reversed nose, second corners and selective returning flap | Independent no-tear study; does not reproduce the source's possible finite-thickness tearing. |
| [Fold 'N Fly terms](https://www.foldnfly.com/terms.php) | Public redistribution permission is not supplied for source templates/artwork | Their authorization is limited to personal, non-commercial use; public/commercial use requires permission. We link references and distribute our own procedural studies, not their materials. |
| [Origami Simulator airplane.svg](https://github.com/amandaghassaei/OrigamiSimulator/blob/main/assets/Origami/airplane.svg), [license](https://github.com/amandaghassaei/OrigamiSimulator/blob/main/LICENSE) | Actual airplane **SVG crease pattern**, blob `bbd00f70b069634459f72af967b9e41bd0c4300f`; not a downloaded `.fold` flight-ready model | Repository MIT notice verified (Amanda Ghassaei, 2018). Not imported or redistributed; would need conversion, deployment, material scale and layer validation. Repository licensing is not a claim of separate design rights. |
| [FOLD examples](https://github.com/edemaine/fold/tree/main/examples), [diagonal-cp.fold](https://github.com/edemaine/fold/blob/main/examples/diagonal-cp.fold), [license](https://github.com/edemaine/fold/blob/main/LICENSE) | Verified reusable FOLD examples: box, diagonal, simple and square twist. Inspected diagonal geometry; blob `9f12ea09c0a54ddd8c5c6cb5a714891e841dda63` | MIT (Erik Demaine, Jason Ku, Robert Lang, 2016), notice must be retained if reused. They are interchange fixtures, not airplane designs; none bundled. |
| [FOLD 1.2 specification](https://github.com/edemaine/fold/blob/main/doc/spec.md) | Frame structure, metre units and multi-model collection format | Format reference; exporter independently implemented. |

## Researched references outside the playable count

[Lock-Bottom](https://www.foldnfly.com/29.html), [Fast Glider](https://www.foldnfly.com/48.html), [Lift Off](https://www.foldnfly.com/38.html) and [The Square](https://www.foldnfly.com/17.html) require cuts. [Stealth Glider](https://www.foldnfly.com/43.html), [Water Plane](https://www.foldnfly.com/12.html), [Trooper](https://www.foldnfly.com/51.html), [Gliding Plane](https://www.foldnfly.com/9.html) and [Flying Shovel](https://www.foldnfly.com/59.html) introduce accordion/pocket/tucking sequences not reconstructed here. These are references, not supported designs or supplied downloads. The source-backed library is therefore nine independent studies, **zero imported third-party airplane FOLD assets**.

## Geometry and interchange

Every recipe is a complete material disk. Default Letter area is 603.2246 cm² and mass at 80 gsm is 4.8257968 g. V-Wing starts with 466.1281 cm² and 3.7290248 g. Underside rotates the Letter dimensions to landscape. Uniform size scaling multiplies both dimensions and crease axes; area and mass scale quadratically. Unsupported span controls are hidden and ignored consistently in export and flight.

Each individual FOLD has original 2D material coordinates at the root and a self-contained 3D folded frame. Indices identify material points, so coincident layers remain distinct. Boundary edges are `B`; internal partitions remain `U`. Complete mountain/valley angles and layer order are intentionally unspecified. The namespaced recipe extension can reconstruct the same geometry and applied step count in this implementation. Export tests compare those reconstructed vertices and the compiled flight mesh, rather than claiming support in every external FOLD viewer.

The common registry is `src/folding/library.ts`, backed by `recipes.ts`. The main picker, downloads, query validation and launch use its IDs. `design.ts` applies wing/material choices; `downloads.ts` also applies sheet-size scaling. Relative navigation and browser Blob downloads do not assume a root deployment path.

Material area, seams, rigid panels and finite numerical flight are tested. No physical throws have calibrated these studies. Folding remains zero-thickness kinematics; the flight solver’s swept self-contact and initial-layer contact do not prove collision-free folding, full stacking order or a working frictional lock. New designs retain their authored span; all expose wing angle, sheet size and material selection.

See [validation results](plane-library-validation.md) for recorded checks.

## Trimmed starting points

Game URLs accept the fold settings (`design`, `elevator`, `dihedral`, `span`, `size`, `paperWeight`, `material`), the release (`throw` angle, launch `trim`, `height`) and `mode=throw` to open the folded plane straight in the field. Launch speed always comes from the pull.

These settings came from an offline search: each design flown from hand height (1.8 m) at 6 m/s (just over half pull), elevator 0–20°, throw 0/10/20° and launch trim 0/5/10°, still air, 80 gsm. The best setting was then re-flown at 4.5 and 8 m/s and from 5 m. Distances are straight-line flight distance; they are simulation results, not physical calibration.

| Design | Link | Hand, 6 m/s | 4.5 / 8 m/s | From 5 m | Untrimmed default |
| --- | --- | ---: | ---: | ---: | ---: |
| Underside | `?design=underside&elevator=8&throw=10&trim=0&mode=throw` | 18.3 m, 6.6 s | 14.8 / 21.2 m | 27 m | 6.5 m |
| Heavy Nose | `?design=heavy-nose&elevator=16&throw=10&trim=10&mode=throw` | 10.4 m, 3.2 s | 8.0 / 11.8 m | 12.7 m | 4.8 m |
| Raven | `?design=raven&elevator=20&throw=0&trim=0&mode=throw` | 8.2 m | 5.4 / 10.7 m | 17.8 m | 4.0 m |
| Folded Dart | `?design=dart&elevator=16&throw=0&trim=0&mode=throw` | 7.9 m | 5.8 / 9.4 m | 10.3 m | 4.5 m |
| Hunting | `?design=hunting&throw=10&trim=0&mode=throw` | 7.5 m | 5.6 / 9.3 m | 11.9 m | 7.1 m |
| The Stable | `?design=stable&elevator=20&throw=10&trim=0&mode=throw` | 6.6 m | 4.5 / 8.9 m | 11.7 m | 4.0 m |
| The Buzz | `?design=buzz&elevator=20&throw=10&trim=0&mode=throw` | 6.4 m | 4.4 / 8.3 m | 10.4 m | 4.2 m |
| The Basic | `?design=basic&elevator=8&throw=10&trim=5&mode=throw` | 5.7 m | 4.6 / 6.7 m | 6.9 m | 5.2 m |
| Original dart | `?design=legacy&throw=10&trim=0&mode=throw` | 4.4 m | 7.3 / 2.6 m | 14.9 m | 3.7 m |
| V-Wing | `?design=v-wing&elevator=4&throw=10&trim=5&mode=throw` | 4.4 m | 2.9 / 4.8 m | 6.9 m | 2.4 m |

Hunting's winglets prevent an elevator fold. The Original dart flies best untrimmed and gently thrown; hard throws loop it. V-Wing trims near 2.6 m/s and stalls into a spin after normal throws.
