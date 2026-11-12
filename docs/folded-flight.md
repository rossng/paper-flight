# Folded paper in flight: current approximation

The Basic, Folded Dart and The Stable now run through the deformable flight solver. The Original dart remains available as a baseline. The source recipe is compiled each time a flight setup is rebuilt; there is no substitute dart trajectory or artificial stabilising controller.

## Mass and geometry

The compiler takes the actual deployed panel positions. Material-space topology keeps coincident paper layers distinct. Conforming centroid fans and shared-edge refinement produce a solver mesh without changing the material outline. Every triangle contributes its material area × paper grammage to its three vertices. Uniform size scaling multiplies area and mass by size squared. The folded centre of mass is preserved, including the additional mass concentrated in a folded nose. No heuristic nose weights are added.

Wing-width edits change the wing-root crease in the recipe rather than stretching the deployed sheet. Wing V changes the two deployment angles. Both retain the complete sheet area. Paper weight changes material mass and the global single-layer bending multiplier.

## Laminate attachment and stiffness

Every vertex lying on a different overlapping material panel can be attached to a barycentric point on that panel. The constraint projects the vertex and target triangle with inverse-mass weights; it does not pin either to the world. Free translation and total centre of mass are tested. These attachments maintain a folded laminate and approximate the holding of locked flaps.

This is **not frictional contact**: layers cannot slide, peel or separate. There is no finite-thickness collision solver. The attachments may make overlapped regions too rigid, especially where unlike tessellations are coupled.

We estimate local layer count in the deployed shape. Each hinge gets an additional bending multiplier `n^(2c)`, where `n` is the mean local count across its two faces and `c` is an explicit compilation parameter in [0,1]. Because all material layers already contribute their own bending constraints, this roughly interpolates between an `n D` aggregate lower bound and an `n³ D` bonded upper bound. The default `c = 0.5` is an empirical intermediate assumption, not a measured paper-friction law. Crease stiffness remains distinct from interior bending stiffness.

Thickness-sensitive plate bending motivates the limiting behaviour; see [MIT Structural Mechanics, plate bending](https://ocw.mit.edu/courses/2-080j-structural-mechanics-fall-2013/f8fd2ad49d100766335b4e129a8a4791_MIT2_080JF13_Lecture7.pdf). Constraint integration builds on the existing [XPBD formulation](https://mmacklin.com/xpbd.pdf). Neither source validates this specific laminate approximation.

## Aerodynamic exposure

Applying full lift and drag to every overlapping layer would multiply the lifting area. Instead, 48 deterministic area samples per triangle estimate the reciprocal overlap count. Each triangle's current aerodynamic area is multiplied by that rest-state exposure fraction. Exactly coincident duplicate surfaces consequently share one effective area, while all their mass remains present.

This is approximate area integration, not resolved airflow. It handles coplanar overlap, not arbitrary upstream shadowing or a wake. The exposure weights stay fixed because the attachment model prevents layer separation; they would need to be recomputed or replaced if peeling/sliding is introduced. Force still uses current deformed triangle geometry, normals and velocities.

## Replay displacement field

Replay stores the actual simulated vertex positions. At each replay time, the rest mesh is aligned with the current body frame formed from nose, tail and span landmarks. Orange arrows show the difference between that aligned rest mesh and recorded positions. Blue wire shows the reference; the aircraft itself is not magnified. The displayed RMS is in actual millimetres, independent of arrow exaggeration (5×, 10×, 25×, 100× or 250×).

These arrows show **displacement, not aerodynamic force**. They remove translation and rotation, but the registration is based on body landmarks rather than a least-squares fit; severe landmark flex can influence the reported field. The layer overlay shows local averaged layer count, not per-vertex lumped mass or stress.

## Verification and limits

Replay also records vertex velocities. The centre-of-mass marker uses the material masses, including every overlapping layer. The centre-of-lift marker re-evaluates the same aerodynamic model on interpolated positions and velocities, with the flight's wind, air density and layer exposure weights; it never uses the final landed frame's loads for an earlier moment. Lift is retained separately from drag without changing the force integrated by the solver.

The displayed lift location is a signed centroid: panel lift is projected onto the net lift direction, then each panel centre is weighted by that signed contribution. Opposing contributions are retained. The marker hides when net lift is below 10⁻⁸ N or 0.1% of summed lift magnitudes. For nonparallel loads this is an estimate, not an exact 3D centre of pressure, a fixed aerodynamic centre, or a stability verdict. See [NASA's explanation of distributed aerodynamic force and centre of pressure](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/center-of-pressure/).

Takeoff and Landing pause playback and seek to the corresponding recorded moment before framing the plane. Follow mode therefore remains anchored to the frame being displayed. Full path turns follow and the close-inspection overlays off.

Tests check source mass and folded centre of mass, layer-local hinge multipliers, aerodynamic area sharing, rigid-motion removal from flex vectors, mass scaling, free translation, finite complete flights for all three designs, and one extreme launch. The local coupling test compares deformation for identical mass/geometry under different laminate stiffness settings. Browser checks cover launching the three designs, replay/scrubbing, flex scaling, follow-camera and the layer overlay on desktop and mobile.

The examples fly, deform and land, but flight distances and material constants are not calibrated against real paper. No general self-contact, tearing or finite-thickness contact has been added. This is a practical integration of the fold definitions, with its approximation kept explicit.
