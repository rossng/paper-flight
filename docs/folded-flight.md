# Folded paper in flight: current approximation

The Basic, Folded Dart and The Stable now run through the deformable flight solver. The Original dart remains available as a baseline. The source recipe is compiled each time a flight setup is rebuilt; there is no substitute dart trajectory or artificial stabilising controller.

## Mass and geometry

The compiler takes the actual deployed panel positions. Material-space topology keeps coincident paper layers distinct. Conforming centroid fans and shared-edge refinement produce a solver mesh without changing the material outline. Every triangle contributes its material area × paper grammage to its three vertices. Uniform size scaling multiplies area and mass by size squared. The folded centre of mass is preserved, including the additional mass concentrated in a folded nose. No heuristic nose weights are added.

Wing-width edits change the wing-root crease in the recipe rather than stretching the deployed sheet. Wing V changes the two deployment angles. Both retain the complete sheet area. Paper weight changes material mass and the global single-layer bending multiplier.

## Directional sheets and sliding contact

Original material UVs are carried through centroid fans and edge refinement. The solver uses them for machine/cross-direction membrane stiffness and directional hinge bending. Grammage and nominal material density set thickness; elastic moduli set stretch/shear and bending response. Fold creases retain their authored rest angles and acquire a separate permanent offset when their effective yield angle is exceeded. See [the material equations and nominal parameters](physics-upgrade.md).

Default `layerModel: 'contact'` replaces rigid ties with unilateral vertex/triangle contact across initially coplanar material groups. Projected barycentric contact points update each substep, so vertices can move across target triangle boundaries. Contact resists crossing the approached surface, permits opening, and applies Coulomb friction only under compression. Equal/opposite inverse-mass-weighted corrections preserve the sheet's mass centre. Each actual material layer supplies its own bending energy; no bonded layer-count multiplier is applied in contact mode.

Swept vertex/face and edge/edge contact now supplements these layer contacts, with a refitted bounding-volume hierarchy and material-neighbor exclusions. It covers new encounters outside the initially coplanar groups. Detection includes the predicted motion and the final structural projection, and contact runs after strain limiting. New swept contacts are frictionless; the existing layer path retains Coulomb friction.

This remains a zero-clearance midsurface approximation that preserves the zero-thickness recipe geometry. A physically certified stacking order is unavailable: sides are selected by initial local opening and then the previous deformed state. Initially overlapping layers can open into intersecting arrangements, and finite sweeps do not resolve all subsequent collisions. This is not finite-thickness stacking or proof that a locking flap holds. The workbench folding animation is still purely kinematic. See [the measured improvements and remaining intersections](TASK_REPORT.md).

For explicit comparisons, `layerModel: 'bonded'` restores the old barycentric ties and local `n^(2c)` bending multiplier, with compilation `layerCoupling = c` in [0,1], default 0.5. Its ties prohibit sliding/peeling. That model is retained as a benchmark approximation, not silently mixed into the default contact mechanics.

## Exposed aerodynamic surface

The compiler retains material identities and the baseline's 48-sample static exposure weights. Default flight builds a separate spanwise aerodynamic quadrature, computes dynamic near-coplanar exposure at current patch positions, and solves coupled circulation/downwash. Sliding and separation can therefore change aerodynamic area during flight. Static compiler weights are used only by the independent-panel comparison path (`aeroModel: 'panels'`).

Patch barycentric weights interpolate position and velocity and distribute force to the same structural triangle. This preserves both force and moment. Layer masses are never removed by exposure. The planar lifting-line model includes span interaction, tip unloading and induced drag, with a separated-flow fallback for large incidence and reverse flight. It is not full 3D flow around a folded stack. See [the aerodynamic model and limitations](physics-upgrade.md).

## Replay displacement field

Replay stores the actual simulated vertex positions. At each replay time, the rest mesh is aligned with the current body frame formed from nose, tail and span landmarks. Orange arrows show the difference between that aligned rest mesh and recorded positions. Blue wire shows the reference; the aircraft itself is not magnified. The displayed RMS is in actual millimetres, independent of arrow exaggeration (5×, 10×, 25×, 100× or 250×).

These arrows show **displacement, not aerodynamic force**. They remove translation and rotation, but the registration is based on body landmarks rather than a least-squares fit; severe landmark flex can influence the reported field. The layer overlay shows local averaged layer count, not per-vertex lumped mass or stress.

## Verification and limits

Replay also records vertex velocities. The centre-of-mass marker uses the material masses, including every overlapping layer. The centre-of-lift marker re-evaluates the same aerodynamic model on interpolated positions and velocities, with the flight's wind, air density, dynamic exposure and coupled circulation; it never uses the final landed frame's loads for an earlier moment. Lift is retained separately from drag without changing the force integrated by the solver.

The displayed lift location is a signed centroid: panel lift is projected onto the net lift direction, then each panel centre is weighted by that signed contribution. Opposing contributions are retained. The marker hides when net lift is below 10⁻⁸ N or 0.1% of summed lift magnitudes. For nonparallel loads this is an estimate, not an exact 3D centre of pressure, a fixed aerodynamic centre, or a stability verdict. See [NASA's explanation of distributed aerodynamic force and centre of pressure](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/center-of-pressure/).

Takeoff and Landing pause playback and seek to the corresponding recorded moment before framing the plane. Follow mode therefore remains anchored to the frame being displayed. Full path turns follow and the close-inspection overlays off.

Tests check source mass and folded centre of mass, material coordinates, reference laminate multipliers, conservative aerodynamic transfer, dynamic exposure, sliding/separating contact and friction, rigid-motion removal, finite complete flights, crease permanent set, and replay re-evaluation. The old local bending-coupling comparison explicitly selects bonded mode. [The benchmark report](physics-validation.md) includes analytic lifting-line and numerical resolution studies.

The examples remain a practical integration of the recipes. Flight distances, crease constants and friction coefficients are uncalibrated; contact cannot certify arbitrary folds. No tearing, general continuous self-contact or collision-aware workbench folding is implemented.
