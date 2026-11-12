# Coupled flight and sheet mechanics

This increment replaces the default independent-panel aerodynamics and rigid laminate attachments. It retains the 120 Hz outer clock, actual material mass, kinematic fold recipes, launcher energy, unrestricted pitch, weather and detached replay. All numerical quantities use SI units. These are engineering/game approximations, **not calibrated predictions of a real paper aeroplane**.

## Exposed aerodynamic surface

`buildLiftingSurface` clips each rest triangle into 12 cosine-spaced span stations. The resulting quadrature patches retain their parent material triangle, fractional area and barycentric position. Aerodynamic resolution is independent of structural tessellation. On every evaluation, patch positions, normals, area and velocity come from the current flexible mesh. The full deformed span is projected into the current body frame; keel-root landmarks set orientation, not span magnitude.

For compiled sheets, a spatial hash finds nearly parallel overlapping material panels at each patch. Reciprocal overlap counts share their exposed area. The distance tolerance is `max(0.2 mm, 2 thicknesses)` and the absolute normal-dot threshold is 0.98. Owners distinguish a physical panel from triangles that merely tessellate it. Exposure is recomputed after sliding or opening and during replay. No static rest exposure weights enter the default coupled path. The Original dart is already an effective exposed surface and needs no layer shielding.

This is a separate aerodynamic quadrature/strip representation, not full lift on every material face. A patch load `F` at `x = sum(w_i x_i)` is transferred as `f_i = w_i F`. Since `sum(w_i)=1`, both the resultant and `sum(x_i cross f_i)` exactly match the aerodynamic load and moment. Lift and drag remain separate for replay markers.

Limitations: shielding is near-coplanar quadrature, not ray tracing or a flow solution around separated stacks. Small exposed regions can be missed by quadrature. Its hard distance/normal thresholds can change exposure abruptly. Separated layers recover area but are still collapsed into one spanwise lifting-line model; this is not a resolved biplane or wake-shadow model. Remeshing or new folds in flight are unsupported.

## Spanwise coupling and fallback

`solveLiftingLine` solves a dense circulation system every aerodynamic evaluation. Strip chord is exposed area divided by current strip width. With piecewise-constant circulation and straight trailing vortices, the downwash influence at strip centre `y_i` from strip `j` is:

```text
D_ij = [1/(y_i - y_j,left) - 1/(y_i - y_j,right)] / (4 pi)
w_i  = sum_j D_ij Gamma_j
Gamma_i + pi c_i a_i sum_j D_ij Gamma_j = pi c_i a_i V_i alpha_i
CL_i = 2 Gamma_i / (V_i c_i)
```

Here `a_i` is the area-weighted attached fraction, not the two-dimensional lift slope (which is `2 pi`). Circulation jumps to zero at the free tips. The matrix has off-diagonal influence: changing incidence or chord at one strip changes neighbouring loads. Tip unloading, span/chord response and induced drag consequently emerge from this interaction. Geometry and local wind determine the area-weighted strip incidence and speed. A pivoted direct solve handles the small matrix without a circulation-history cache.

Attached loading is apportioned over attached patches. A zero-mean local incidence correction retains chordwise reflex/trim moments. Induced drag is `max(0, CL_attached w / V)`, in addition to profile drag 0.035. The nonnegative local clamp is a dissipative safeguard for distorted configurations; it suppresses local induced thrust and is not a Trefftz-plane drag calculation.

The attached fraction fades smoothly between 0.21 and 0.49 radians incidence (about 12–28 degrees), body-forward flow alignment 0.25–0.75, and panel/up alignment 0.2–0.7. Reverse flight, broadside panels, strong sideslip and large incidence therefore use the separated flat-sheet law:

```text
CL_sep = 0.85 sin(2 alpha)
CD_sep = 0.08 + 1.75 sin(alpha)^2
```

Attached CL is bounded to ±1.5 before the separated contribution is added. Lift is perpendicular to the local air-relative velocity and drag opposes it. With still air, instantaneous external aerodynamic power is nonpositive, including for deforming/tumbling meshes, because the same barycentric map interpolates velocity and transfers force. A finite explicit step can still have integration error.

This is **planar lifting-line**, not an implementation of AVL or a general vortex-lattice/wake solver. Sweep and nonplanarity enter through exposed area, incidence and attachment blending, not full 3D vortex geometry. There is no wake roll-up, unsteady shedding, dynamic stall, Reynolds dependence, vortex lift, ground effect or airfoil pitching-moment model. Low-aspect-ratio swept paper darts lie outside the most reliable regime. Fallback thresholds and coefficients remain unmeasured. The stall HUD is an attached-fraction diagnostic, not a measured separation boundary.

The [AVL primer](https://web.mit.edu/drela/Public/web/avl/avl_doc.txt) motivates interacting thin lifting surfaces and explicitly limits attached potential-flow methods to appropriate angles and slowly varying motion. This much smaller model does not inherit AVL's capabilities or validation. An elliptic-wing analytic lifting-line comparison checks our discretization in its intended regime; it does not validate dart flight.

`Parameters.aeroModel = 'panels'` retains the original independent-panel polar and static exposure weights for reference benchmarks. Direct `aerodynamicLoads` callers without an exposed surface also retain that path. The game defaults to `'coupled'`.

## Explicit sheet properties and directional mechanics

Thickness is inferred from grammage and nominal density: `t = gsm / (1000 rho_material)`. Compiled mesh mass remains the original sheet material area times grammage; neither contact nor aerodynamic exposure changes it. Original dart mass retains its documented effective-layer distribution. Geometry is not inflated when thickness changes.

Nominal inputs in `sheet-material.ts`:

| Material | gsm | Density kg/m³ | Thickness µm | E_MD / E_CD GPa | G GPa | Poisson MD | D_MD N m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Paper | 80 | 800 | 100 | 4 / 2 | 1 | 0.25 | 0.000344 |
| Tissue | 18 | 300 | 60 | 0.25 / 0.12 | 0.06 | 0.20 | 0.00000459 |
| Foil | 135 | 2700 | 50 | 69 / 69 | 25.94 | 0.33 | 0.000807 |
| Steel | 7850 | 7850 | 1000 | 200 / 200 | 76.92 | 0.30 | 18.315 |

These chosen values are illustrative, not measurements of particular products. The 50 µm foil is a heavy foil illustration; not all kitchen foil has that thickness. Steel folding remains a kinematic game operation, without work, tooling or fracture simulation.

Original material UVs survive centroid fans and shared-edge refinement. Paper machine direction (MD) follows original sheet +v, its long edge; cross direction (CD) follows +u. A triangle-local reference metric preserves the exact rest shape, including the effective dart without a literal flat-sheet parametrization.

Each triangle has XPBD membrane constraints for MD/CD Green normal strains and engineering shear. Their energy coefficients are respectively `E_MD t A`, `E_CD t A`, and `G t A`. Thus extension and shear stiffness scale linearly with thickness. Membrane Poisson coupling is omitted. This is a diagonal orthotropic membrane approximation, not a full constitutive finite-element law. The former edge network remains for strain diagnostics/damping and a numerical strain envelope, not as the elastic membrane model.

Hinges use a directional engineering modulus and plate rigidity:

```text
1/E(theta) = cos^4(theta)/E_MD + sin^4(theta)/E_CD
             + (1/G - 2 nu_MD/E_MD) sin²(theta) cos²(theta)
D(theta) = E(theta) t³ / [12 (1 - nu_MD² E_CD/E_MD)]
k_flat = D(theta) edge_length² / (A_left + A_right)
```

The angle is the material bending direction perpendicular to the hinge. Each physical layer supplies its own bending constraints; contact mode adds no bonded `n³` multiplier. Hinge discretization is an effective directional shell approximation, not a complete orthotropic curvature tensor. [Discrete Shells](https://www.cs.columbia.edu/cg/pdfs/10_ds.pdf) supplies the rest-dihedral/dual-area framework; our anisotropic material and crease choices are additional approximations. Direction-dependent elastic compliance is also discussed in the primary study [Directional dependence of the mechanical properties of aged paper](https://www.sciencedirect.com/science/article/pii/S0167663621002623); that study does not calibrate these presets.

Existing `stiffness` control values are interpreted relative to each material's derived reference ratio; `materialSettings` supplies that ratio and the simulation converts it to a multiplier of physical rigidity. For ordinary paper this preserves the existing `(gsm/80)^3` control API without multiplying thickness cubed twice. Direct construction without an explicit stiffness automatically chooses the material ratio. Crease overrides remain independent.

## Permanent crease deformation

A crease uses `k = D * retention * edge_length / crease_width`. These are deliberately explicit effective damaged-band parameters:

| Material | Band width mm | Retained rigidity | Yield angle rad | Hardening ratio | Friction |
| --- | ---: | ---: | ---: | ---: | ---: |
| Paper | 1 | 0.20 | 0.35 | 0.08 | 0.45 |
| Tissue | 1 | 0.12 | 0.25 | 0.03 | 0.50 |
| Foil | 0.5 | 0.025 | 0.08 | 0.02 | 0.30 |
| Steel | 3 | 0.30 | 0.04 | 0.10 | 0.35 |

Signed recipe rest angles remain immutable. Each crease carries a permanent offset and accumulated plastic rotation. After a substep, a rate-independent return mapping moves excess elastic angle into permanent rotation, with isotropic hardening. Unloading retains the new crease; reversing the bend accumulates hardening instead of erasing history. Reset/launch clears it. Yield angles are effective crease parameters, not bulk metal yield strains. There is no distributed face plasticity, tearing, moisture model or fatigue.

## Sliding contact

The default uses unilateral vertex/triangle contact over the deployed rest-coplanar material groups. Different material identities stay distinct. Candidate groups include neighbouring target triangles, so sliding can cross a tessellation boundary. Every substep refreshes the projected barycentric contact point. Contact blocks motion through the approached side, applies no tensile constraint when layers open, and adds Coulomb friction limited by the compressive normal multiplier: `|lambda_t| <= mu lambda_n`. Without pressure, there is no frictional glue. Corrections are distributed with inverse mass and equal/opposite barycentric weights, preserving the mass centre.

The zero-thickness recipe cannot supply a physically unique side for perfectly coincident sheets. The first local opening selects that side, shared across target triangles; subsequent contacts use the previous deformed side. This avoids imposing contradictory rest ordering across selective fold seams. It is **not** a reconstruction of fold stacking order. Existing coincident midsurfaces have zero clearance; thickness controls mass and stiffness but is not a geometric collision offset. This preserves the exact sheet and launch geometry.

The original layer contact points/normals are frozen during a substep's projection sweeps. Normal impulses conserve moment at detection; tangential corrections at nonzero gaps and finite rotations have projection error.

An additional swept contact solver now checks nonadjacent vertices/faces and edge pairs across the whole mesh. A fixed-topology BVH is refitted to swept bounds in a translating frame. Narrow phase isolates roots of the cubic coplanarity polynomial over linear vertex motion and checks barycentric/segment coordinates at impact. Shared vertices and their one-ring material neighbors are excluded, preserving connected fold seams. Detection runs after the first and final structural sweeps; normal contact runs after every strain-limiter pass, followed by four contact-only finishing sweeps.

Swept normal response recomputes distance gradients, normals and projected weights during iteration. Inverse-mass corrections preserve mass centre and linear impulse and have zero instantaneous net torque. This is not a claim of exact angular-momentum conservation over a finite integrated step. New contacts are frictionless; the rest-coplanar layer path retains pressure-limited friction. No thickness offset, stiffness change or position anchor is introduced. Unknown coincident sides within 1 nm are allowed to open; an established touching side persists into the next substep and reset clears it.

This is bounded collision detection and response, not a fail-safe pipeline: degenerate/parallel features, initially intersecting stacks, multiple impacts during one step, and contacts created by the last response can remain unresolved. Linear swept detection does not reconstruct every intermediate nonlinear constraint trajectory. An endpoint response can also miss the intermediate penetration of a double crossing even when both roots are detected. Existing layer-normal/side limitations remain. It cannot certify a locking flap, collision-free folding or arbitrary crumpling. The [Bridson–Fedkiw–Anderson contact paper](https://www.graphics.stanford.edu/papers/cloth-sig02/) explains why robust cloth needs more than proximity response; its fail-safe collision pipeline is not implemented here. See [the self-contact regression and timing evidence](TASK_REPORT.md).

For comparisons, `layerModel: 'bonded'` explicitly restores the old barycentric ties and compiler `layerCoupling` multipliers. That path is an effective laminate reference. Default contact ignores those multipliers, preventing double-counted layer rigidity.

## Integration, replay and verification

The outer frame remains 1/120 s with at least three substeps (nine for tissue). Automatic stepping estimates the fastest explicit aerodynamic damping rate `rho_air pi V A_tributary / mass` and increases substeps to keep `dt rate <= 0.8`, with a 96-substep ceiling. This is a stability heuristic, not a rigorous error controller. `substeps` can override it for controlled convergence experiments. `iterations` defaults to 24 alternating sweeps.

An edge-strain inequality targets ±2% as a near-inextensible **numerical safety envelope** for this no-tearing game. It is inactive inside that band and is not a measured yield or failure model. Other constraints can leave larger residual strain with finite sweeps, particularly on tiny/skinny triangles. Extreme flexible flight still needs more resolution; finite output alone is not physical validation. Pairwise axial damping retains the original internal damping mechanism.

Replay stores positions and velocities and re-evaluates exposed patches, circulation and deterministic weather at the sampled time. No live contact/plastic history or final-frame forces enter the lift marker. Interpolation and Float32 recording introduce small errors. The marker remains a signed lift centroid, not a unique 3D centre of pressure.

Run `npm test`, `npm run build`, `npm run benchmark` and `npm run benchmark:physics`. The added tests cover rigid/Galilean/winding invariance; conservative aerodynamic transfer; tip unloading and neighbour response; aspect ratio and elliptic-wing convergence; separated/reverse/tumbling passivity; dynamic peeling exposure; contact separation, sliding, friction and moment balance; material scaling, grain preservation, membrane response and crease hysteresis/reset; and coupled replay. Existing complete-flight tests exercise all supported designs/materials, weather, launcher, pitch and folding. Numerical results and measured timing are recorded in [the benchmark report](physics-validation.md).
