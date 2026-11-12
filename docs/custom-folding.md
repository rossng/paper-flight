# Custom folding: representation study and implementation route

The existing workbench changes an effective dart mesh. It cannot preserve the identity of a sheet through arbitrary folding, distinguish coincident layers, or describe a locking flap. The new sheet model in `src/folding/` now compiles the supported recipes into a flight mesh using the [flexible flight bridge](folded-flight.md). A future custom editor should author this same model. Changing `createDart` parameters is not sufficient.

## Executable proof

Open `/folding-study.html` (also linked from the game's help dialog). Choose a design, inspect its fold history, orbit the 3D result, compare the original-sheet crease partition, and export the current state as `.fold`.

The independent reconstructions use US Letter dimensions, 215.9 × 279.4 mm, and 80 gsm paper. Each retains 603.2246 cm² of material and 4.8258 g of mass. These are source-referenced geometric studies, not exact digital reproductions of a downloaded template. Wing-root positions are authored parameters; the deployed dimensions are checked against published measurements within a 5% tolerance.

| Reference | Reason for choosing it | Representation exercised |
| --- | --- | --- |
| [The Basic](https://www.foldnfly.com/0.html) | Minimal conventional aircraft | Centre crease, corner folds, independent deployment of two wings |
| [Basic Dart](https://www.foldnfly.com/1.html) | More heavily layered swept wings | Successive shoulder folds across existing folded panels; tapered wing roots |
| [The Stable](https://www.foldnfly.com/2.html) | A useful counterexample to treating every fold as a whole-stack rotation | Nose reversal, overlapping shoulders, and a locking flap selected separately from underlying material |

The source instructions also guide the choice of operations. Our low-level operation count differs from illustrated step counts: paired folds become separate actions, and wing deployment is explicit. Published photographs and printable artwork are not bundled.

All three sequences preserve area, rigid-panel distances, material seam continuity, and a manifold disk topology at every step. The resulting wing spans are symmetric. An additional test checks that the Stable's lock leaves underlying panels untouched; another rejects a deliberately tearing layer selection. This is geometric evidence, not a proof of collision-free folding or the physical holding force of a lock.

## The data model

- **Sheet:** original outline, dimensions, paper mass per area; the flight solver derives thickness from grammage/density and uses explicit directional elastic and crease properties.
- **Panel:** a polygon whose vertices retain both original 2D sheet coordinates and current 3D coordinates. Splitting a panel interpolates both coordinate sets. Material area, rather than projected area, determines mass.
- **Operation:** a crease axis, a moving half-plane, signed rotation, and explicit layer selection. Panel lineage tags survive splits, so the nose flap can be selected after other folds create additional facets.
- **History:** an ordered recipe replayed from the original sheet. The prototype's undo recomputes a prefix of that recipe. A future edit can invalidate later references and must report that rather than quietly choosing different panels.
- **Topology:** vertices are identified in original-sheet coordinates. Coincident 3D points from different paper layers stay distinct. Edge subdivisions are conformed before export, preventing T-junctions from masquerading as torn seams.

`checkedFold` rejects operations that introduce a seam gap, stretch panels, or lose the expected sheet topology. It does not certify physical accessibility or contact. The model currently has zero thickness, and it does not compute a complete stacking order. The Stable example proves selective flap geometry, not frictional locking.

## Interchange

[FOLD 1.2](https://github.com/edemaine/fold/blob/main/doc/spec.md) provides vertices, edges, faces, folded-state frames, crease assignments, and layer-order fields. Use it for interchange, with our procedural history stored in a namespaced extension.

The prototype exports an original-sheet frame and a folded-state frame, plus `paperflight:recipe` and the applied step count. Internal edges are marked unassigned rather than inventing mountain/valley information; complete fold-angle and face-order export remains future work. A generic FOLD state does not by itself reconstruct the interactive sequence, layer choices, or editing intent. Full FOLD import is not implemented.

## Recommended first editor

1. Start from one of the three recipes or a blank rectangular sheet. Keep the paper large on the existing craft table.
2. Draw a crease on a selected planar face or stack. Snap endpoints to edges, midpoints, existing creases, and mirrored counterparts.
3. Click the moving side. Show exactly which layers will move; offer a stack view to pick one flap or all connected layers. This is required before supporting the Stable lock.
4. Drag the flap through an angle. Preview the fold without committing it; show a blocked fold when it would tear, collide, or violate supported constraints. Commit creates one undoable history command.
5. Keep a visible fold history with undo/redo, mirrored folds, and editable crease endpoints. Rebuild dependent steps after edits and flag broken layer references.
6. Open the wings for flight, separately from flat construction folds. Store deployment angles and later trim changes explicitly.

Start with straight creases and supported planar-layer selections. Do not initially promise arbitrary curved folds, wet folding, squash folds, or crumpling. Those need different deformation/contact machinery.

## Bridge to the flight solver

A geometrically folded sheet cannot safely be passed straight to the present aerodynamic loop: every overlapping triangle would receive airflow forces, multiplying the effective lifting area.

For a fully physical custom-design bridge, the remaining requirements are below. The current bridge implements material-mass triangulation, directional membrane/hinge constraints, plastic crease offsets, dynamic exposure and a coupled spanwise aerodynamic model. Contact is restricted to initially coplanar groups, with zero-clearance midsurfaces; general collision detection and stacking order remain open:

1. Triangulate conformed panels, preserving real crease edges separately from numerical subdivision edges. Carry original material area into vertex masses.
2. Build the deployed rest shape and hinge constraints from the recipe; retain folded-layer membership, thickness and local stiffness rather than replacing everything with one stiffness multiplier.
3. Implement layer contact and collision checks, or an explicitly constrained effective laminate approximation for supported stacked regions. A locking flap cannot be represented by visual overlap alone.
4. Determine exposed aerodynamic surfaces and shielding. Avoid applying full aerodynamic loads independently to every hidden layer. Keep force/torque transfer back to material mass conservative.
5. Verify static shape and mass distribution, then benchmark the three reference designs against real folds and throws. Published distances are comparison context, not solver calibration targets on their own.

The game retains Original dart and also offers all three compiled studies. The study now has a Fly this design button once the wings are deployed. These designs use the documented restricted sliding-contact and exposed-surface model. The old bonded laminate remains an explicit benchmark option; full finite-thickness self-contact is still absent.

## Next reference after the three no-cut designs

[Lock-Bottom](https://www.foldnfly.com/29.html) adds wingtip folds and tail cuts for control surfaces. It is a useful next test after explicit cut topology is implemented. Cuts need separate material boundaries and changed connectivity; they cannot be represented as another crease angle. This design is researched but not encoded in the current fixtures.

## Verification

`npm test` includes the numerical flight checks, detached flight-recording checks, and seven folding tests. Browser checks cover every sample's fold states and exports. `npm run build` produces both the game and the folding study.
