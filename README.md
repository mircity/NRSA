# NRSA — Structural Analysis & RCC Design Engine

NRSA is a from-scratch structural analysis and reinforced-concrete
design engine, built as a proper compiled C++ library rather than an
in-browser solver — the architecture this project replaces (a
single-file HTML/JS app with a dense O(n³) in-browser solver) hit a hard
ceiling on model size and element sophistication that only a real
sparse-matrix, compiled engine can get past.

## Status

**`src/design/` scope: complete.** Every member-design type this
project set out to cover — RCC Beam, Column, Slab, Shear Wall, all
three Foundation cases (isolated, combined, pile cap), and now Steel
(beam flexure/shear, column compression, combined interaction) — is
implemented and unit-tested. `src/bbs/` (Bar Bending Schedule) and
`src/boq/` (Bill of Quantities) — the first two "surrounding tooling"
modules item 13 lays out a build order for — are now also implemented:
`bbs/` converts the reinforcement AREAS every design/ module above
already computes into real bar counts/diameters and per-bar-mark cut
lengths, rolled up into a schedule with total length/weight per mark
and per diameter; `boq/` rolls concrete volume, formwork area (for
rectangular beams, columns, slabs, walls, and footings), and rebar
weight (pulled directly from a `bbs::BarBendingSchedule`) into a
quantity takeoff. See item 13 in the Roadmap below for full scope and
what's still open (`reports/`, `validation/`, `modeler/`, `bim/`,
`api/`).

`src/core/` (model data), `src/fem/` (Matrix, FrameElement3D w/
geometric stiffness, SparseMatrix+Solver, ShellElement, EigenSolver),
`src/analysis/` (GlobalAssembly, StaticAnalysis, ModalAnalysis,
ResponseSpectrum, PDelta, NonlinearAnalysis — every item originally
planned for this folder), and six pieces of `src/design/`
(BNBC2020 load combinations, RCCBeam flexure+shear design, RCCColumn
biaxial P-M interaction design/check, RCCSlab one-way + two-way
edge-supported slab design, RCCShearWall in-plane shear design +
story-shear distribution + out-of-plane flexure, and RCCFoundation
isolated square footing design — bearing-area sizing, ACI 318-19
two-way/one-way shear-governed thickness, and critical-section
flexural design; and RCCFoundation combined footing — two-or-more
columns along one axis, plan sized/centered on the service-load
centroid, beam-statics moment/shear envelope, thickness solved
against both one-way shear and EVERY column's own two-way/punching
shear, top+bottom flexural design; and pile cap design -- rigid-cap
pile reactions, column-through-cap and per-pile punching shear,
two-way flexural design; and `design/Steel` — doubly-symmetric I-shape
section properties built parametrically, Chapter F2 compact-section
flexural design/check (yielding + inelastic/elastic lateral-torsional
buckling), Chapter G2.1 shear design/check, Chapter E3 flexural-
buckling compression design/check, and Chapter H1.1 combined axial-
flexure interaction) are implemented and unit-tested, and `bbs/`
(Bar Bending Schedule) and `boq/` (Bill of Quantities) are now also
implemented on top of that design output — see the Roadmap below for
the full test count and what `modeler/`, `bim/`, `reports/`, `api/`,
and `validation/` still involve; none of it is small.
Validation throughout favors independent closed-form/hand-calculated/
round-trip cross-checks over internal-consistency checks alone (see
each class's own doc comment and the Roadmap below for specifics) —
this caught several real bugs before they went any further, most
notably: a sign error in the first Jacobi rotation formula
(`EigenSolver`), a flawed test assumption about degenerate eigenspaces
(`ModalAnalysis`), a test-setup mistake in the local↔global axis
mapping for a vertical member (`PDelta`), a genuine gap in
`fem::Solver`'s positive-definiteness check — it only caught curvature
NEAR zero, not clearly NEGATIVE, so a structure loaded past its
buckling capacity could silently "converge" to a physically impossible
answer instead of throwing (fixed, with a dedicated regression test,
`test_sparse_solver.cpp::testIndefiniteMatrixThrows`) — and, most
recently, an incorrect combination-generation guard in
`design/BNBC2020` that would generate a "0.9D + 1.0W" combination even
for a model with no wind load at all (fixed; caught by a test asserting
that combination shouldn't appear without wind present).

A shared `analysis/GlobalAssembly` module now backs `StaticAnalysis`,
`ModalAnalysis`, `PDelta`, and `NonlinearAnalysis` — the near-identical
assembly/recovery logic that had accumulated across those files (up to
294 lines in one case) is now one ~230-line module, verified via a full
regression sweep with bit-for-bit identical numeric output on every
case that had a prior run to compare against.

A working example (`examples/portal_frame_example.cpp`) builds and
solves a small portal frame end to end. `src/design/`, `src/bbs/`,
and `src/boq/` are implemented (see the tree below); `src/modeler/`,
`src/bim/`, `src/reports/`, `src/api/`, and `src/validation/` are laid
out but NOT yet implemented — that is real, substantial work still
ahead, not a rounding error; see "Roadmap" for what's next and in what
order.

```
src/
  core/        Node, Element, Material, Section, Load, Model      — IMPLEMENTED
  fem/         Matrix (dense, w/ Gaussian-elim solver,
               for element-sized blocks)                          — IMPLEMENTED
               FrameElement3D (12-DOF 3D beam-column, incl.
               geometric/P-Delta stiffness)                        — IMPLEMENTED
               SparseMatrix + Solver (sparse global assembly,
               Jacobi-preconditioned Conjugate Gradient)          — IMPLEMENTED
               ShellElement (24-DOF rectangular flat shell:
               membrane + Mindlin bending w/ SRI + drilling)      — IMPLEMENTED
               EigenSolver (Jacobi rotation, generalized
               symmetric eigenproblem, lumped mass)               — IMPLEMENTED
  analysis/    GlobalAssembly (shared element classification/
               assembly/force-recovery — used by all of the below)  — IMPLEMENTED
               StaticAnalysis (sparse-solved, one load case;
               frame + shell elements COUPLED in one system;
               Core elements still unsupported — see roadmap)     — IMPLEMENTED
               ModalAnalysis (lumped mass, dense eigen-solve
               on the same coupled frame+shell stiffness)         — IMPLEMENTED
               ResponseSpectrum (tabulated spectrum, SRSS/CQC
               modal combination, mass participation)             — IMPLEMENTED
               PDelta (linear geometric-stiffness P-Delta,
               frame elements only)                                — IMPLEMENTED
               NonlinearAnalysis (geometric/P-Delta self-
               consistency via fixed-point iteration)              — IMPLEMENTED
  design/      BNBC2020 (load combinations), RCCBeam (flexure+shear),
               RCCColumn (biaxial P-M interaction),
               RCCSlab (one-way + two-way edge-supported design),
               RCCShearWall (in-plane shear design, story-shear
               distribution by stiffness, out-of-plane flexure)     — IMPLEMENTED
               RCCFoundation (isolated square footing: bearing-area
               sizing, ACI 318-19 two-way + one-way shear-governed
               thickness, critical-section flexural design)         — IMPLEMENTED
               RCCFoundation (combined footing: 2+ columns along one
               axis, beam-statics moment/shear, one-way + per-column
               two-way shear, top/bottom flexural design)            — IMPLEMENTED
               RCCFoundation (pile cap: rigid-cap pile reactions,
               column-through-cap + per-pile punching shear, one-way
               shear, two-way flexural design)                       — IMPLEMENTED
               Steel (AISC 360-16: beam flexure Ch.F2, shear Ch.G2.1,
               column compression Ch.E3, combined interaction Ch.H1.1) — IMPLEMENTED
  bbs/         Bar Bending Schedule -- real bar counts/diameters from
               design/'s required areas, cut lengths (straight bars
               w/ standard hooks, closed rectangular stirrups/ties),
               rolled up by mark and by diameter                    — IMPLEMENTED
  boq/         Bill of Quantities -- concrete volume + formwork area
               (rectangular beam/column/slab/wall/footing), rebar
               weight pulled from a bbs::BarBendingSchedule, rolled
               up by concrete grade and by bar diameter              — IMPLEMENTED
  modeler/     Geometry, Grid, Selection                          — not yet started
  bim/         BIM/IFC round-trip                                 — not yet started
  reports/     PDF/HTML report generation                          — not yet started
  api/         Public C API / bindings                             — not yet started
  validation/  Cross-module design/calculation auditing            — not yet started
tests/         Unit tests (currently: test_core.cpp)
examples/      Example driver programs
docs/          Design notes, formulation references
prototype/     NRSA_RCC.html — the original single-file browser
               prototype this engine is written to replace; kept as a
               working functional reference (see prototype/README.md)
```

## Why this layering

- **`core/`** owns the *model* — the durable description of a building
  (nodes, member connectivity, materials, sections, loads) that gets
  saved, loaded, and edited. It knows nothing about how to solve
  anything.
- **`fem/`** owns the *finite-element math* — stiffness matrices, the
  sparse solver, eigen-decomposition for modal analysis. It consumes a
  `core::Model` and produces numbers; it does not know what a "beam" or
  a "shear wall" means, only what a `FrameElement3D` or `ShellElement`
  is.
- **`analysis/`** owns the *analysis procedures* — static, modal,
  response-spectrum, P-Delta, nonlinear pushover — each one an
  orchestration of `fem/` primitives against a `core::Model`.
- **`design/`** owns *code-based member design* (BNBC 2020 / ACI 318)
  — it consumes analysis results and produces required reinforcement,
  section adequacy checks, etc. This is the layer equivalent to the
  original prototype's `design*()` functions.
- **`modeler/`**, **`bim/`**, **`boq/`**, **`bbs/`**, **`reports/`**,
  **`api/`**, **`validation/`** are the surrounding tooling: building
  the model interactively, IFC/BIM interop, quantity takeoff, rebar
  scheduling, generating issued documents, exposing a stable API for a
  UI to call, and the cross-cutting audit/QA layer (the equivalent of
  the prototype's Model Check + Calc Validation Engine).
- **`prototype/`** is that "original prototype" itself —
  `NRSA_RCC.html`, a working single-file browser app — kept in the repo
  as a running functional reference, not as a module `src/` builds on.

## Building

Requires CMake >= 3.16 and a C++17 compiler.

```sh
mkdir build && cd build
cmake ..
cmake --build .
ctest          # run the unit test suite
```

## Roadmap (suggested build order)

1. ~~`fem/Matrix` — dense matrix/vector types~~ — **done**: `Matrix`
   (dense, row-major), `Vector`, matrix-matrix/matrix-vector products,
   transpose, a symmetry-check helper (every element stiffness matrix
   this library produces should pass it), and `Matrix::solve()` — a
   partial-pivoted Gaussian elimination that throws on a singular/
   mechanism system rather than returning garbage. Verified against a
   hand-calculated two-spring chain in `tests/test_matrix.cpp`, not just
   internal consistency checks.
2. ~~`fem/FrameElement3D` — the 12-DOF two-node 3D frame stiffness
   formulation~~ — **done**: axial, torsion, and biaxial (Timoshenko-
   corrected, degrading cleanly to Euler-Bernoulli when a Section has no
   shear area) bending, plus the local-to-global transformation. Not
   validated against internal-consistency checks alone —
   `tests/test_frame_element.cpp` checks the tip deflection and rotation
   of a cantilever under a point load, the elongation of an axially
   loaded bar, and the twist of a circular shaft under torque, each
   against the exact closed-form textbook result (PL³/3EI, PL²/2EI,
   PL/EA, TL/GJ). Does not yet handle end releases (pins) — `core::
   Element` already carries that data (`endRelease()`), but
   `FrameElement3D` doesn't consume it yet; static condensation of a
   released DOF is a natural next addition once `analysis/
   StaticAnalysis` exists to exercise it.
3. ~~`fem/SparseMatrix` + `fem/Solver` — sparse assembly and a direct
   linear solve~~ — **done**, with one deliberate deviation from the
   original plan worth calling out: `Solver` uses **Jacobi-
   preconditioned Conjugate Gradient (PCG)**, an iterative method,
   rather than a direct sparse factorization (Cholesky/LDLT). A
   restrained, properly-supported linear-elastic stiffness matrix is
   symmetric positive-definite — exactly PCG's convergence guarantee —
   and PCG's O(nnz) per-iteration cost never densifies the matrix the
   way a factorization without a fill-reducing reordering (AMD, nested
   dissection, ...) would; implementing that reordering correctly is a
   meaningfully larger undertaking than PCG itself and was judged not
   worth blocking this milestone on. `SparseMatrix` stores only each
   row's upper triangle (stiffness matrices are always symmetric) and
   exploits that in `multiply()`; a zero/negative diagonal or a failure
   to converge both throw with a diagnostic message, the same "say why,
   don't return garbage" pattern used everywhere else in this project.
   Validated against: a hand-calculated spring chain (identical case to
   the one `Matrix::solve` was checked against), a 19-DOF spring chain
   cross-checked directly against the dense solver's answer (agreement
   to 1e-6), and — most importantly — `analysis/StaticAnalysis` itself
   now runs on this path with ZERO regressions in its own test suite or
   the example program's output (bit-for-bit identical to the dense-
   solver run beforehand on that well-conditioned model). A direct
   sparse factorization remains a reasonable future addition mainly for
   very ill-conditioned models where PCG convergence can slow.
   UPDATE (see item 10): the original positive-definiteness check here
   only caught curvature near zero, not clearly negative — `analysis/
   NonlinearAnalysis`'s own testing surfaced a real case (a structure
   loaded past its buckling capacity) this missed, silently returning a
   physically backwards answer instead of throwing. Fixed by checking
   the SIGN of p^T·K·p, not just its magnitude; see item 10 for the
   full account and the dedicated regression test.
4. ~~`analysis/StaticAnalysis` — wire `Model` + `fem/` together for a
   first end-to-end linear static solve~~ — **done**, and as of item 3
   above, now solving through the sparse/PCG path rather than the dense
   one it started on — exactly the "StaticAnalysis's own logic won't
   need to change, just what it calls" swap this roadmap entry
   originally predicted. Handles frame elements (Beam/Column/Brace)
   only — Wall/Core/Slab elements are collected into
   `StaticAnalysisResult::skippedElementIds` rather than silently
   dropped, pending `fem::ShellElement`. Distributed element loads
   (self-weight, slab reactions onto beams) are similarly not yet
   converted to equivalent nodal loads — only `core::NodalLoad` is
   applied — also an explicit, temporary scope limit, not a silent gap.
   Validated end-to-end (not just at the FrameElement3D level) against:
   a single-element cantilever's hand-calculated tip deflection/
   rotation, a two-element cantilever matching that same closed form
   (confirming mesh-independence and correct multi-element assembly),
   a portal frame's global force equilibrium, and a generic per-DOF
   equilibrium-residual diagnostic every model (not just the
   hand-calculable ones) gets checked against automatically. See
   `examples/portal_frame_example.cpp` for a complete runnable
   demonstration.
5. ~~`fem/ShellElement` + `fem/EigenSolver` — the genuine 4/8-node shell
   element~~ — **`ShellElement` done** (`EigenSolver` still pending —
   see below). A 24-DOF (6/node) flat shell combining: a bilinear
   isoparametric plane-stress MEMBRANE (full 2x2 Gauss), a
   Reissner-Mindlin plate BENDING formulation with the bending
   (curvature) terms at full 2x2 integration and the transverse-shear
   terms at REDUCED 1x1 integration — the "selective reduced
   integration" (SRI) that avoids shear locking as thickness -> 0 — and
   a small artificial DRILLING stiffness (standard practice for a flat
   shell built from separate membrane+plate parts, which otherwise
   leaves the in-plane rotation DOF singular at any node with no frame
   element attached). Local rotation DOFs are defined as literal
   rotation-vector components (not "slopes"), specifically so the SAME
   3x3 rotation used for translations also transforms rotations
   correctly to global — the property that will let a `ShellElement`
   and a `FrameElement3D` share a node's Rx/Ry/Rz DOFs with no separate
   sign convention to reconcile once they're assembled together.
   SCOPE: rectangular (in-plane) elements only, mirroring the browser
   prototype's own "rectangular bounding box" limit (see
   `prototype/README.md`) — a general skewed quadrilateral needs a
   non-constant isoparametric Jacobian, a real but distinct addition.
   Validated with finite-element PATCH TESTS (not just internal
   consistency) — the standard acceptance test for any new element:
   nodal displacements taken from an EXACT linear (membrane) or
   constant-curvature (bending) field must reproduce that exact
   constant-strain/curvature state. Specifically checked: rigid-body
   translation gives zero strain energy; a linear membrane
   displacement field gives a self-equilibrated force set; a
   Kirchhoff-consistent (exactly-zero-shear-strain) constant-curvature
   bending field reproduces the closed-form bending energy
   0.5·D₁₁·κ²·Area EXACTLY across a 100x thickness range spanning into
   the thin-plate regime — direct proof the SRI formulation avoids
   shear locking, not just a plausibility argument; local and global
   (including a non-axis-aligned, tilted panel) stiffness symmetry; and
   that a degenerate/non-rectangular quadrilateral is rejected rather
   than silently producing a wrong element.
   `ShellElement` is now ALSO wired into `analysis/StaticAnalysis` —
   Wall/Slab elements with exactly 4 nodes assemble into the SAME
   global system as the frame elements, genuinely coupled (sharing
   stiffness in one solve), not run separately and merged after the
   fact. The flagship validation for this
   (`tests/test_shell_static_analysis.cpp`) builds four columns
   supporting one slab panel at their tops and applies a point load at
   only ONE column: a frame-only version of that same model (no slab)
   gives the other three columns EXACTLY zero reaction — nothing
   connects them to the load; the slab-coupled version gives them
   genuinely nonzero reactions (one even slightly negative — real
   plate-uplift physics, a corner load can put an adjacent corner into
   local tension), which is only possible because the shell physically
   ties the column tops together. Global equilibrium (sum of reactions
   = applied load) holds in both. Core elements still land in
   `skippedElementIds` — the tube idealization needs a footprint
   width/depth/thickness input `core::Element` doesn't carry yet, a
   distinct addition from the Wall/Slab case. `fem/EigenSolver` is
   still needed before `analysis/ModalAnalysis` or `analysis/
   ResponseSpectrum` can exist — now the more clearly-next piece, since
   `StaticAnalysis` itself is validated across frame, shell, and
   coupled frame+shell models.
6. ~~`fem/EigenSolver` — needed before `analysis/ModalAnalysis` or
   `analysis/ResponseSpectrum` can exist~~ — **done**: solves the
   generalized symmetric eigenproblem K·φ = ω²·M·φ for a lumped
   (diagonal) mass matrix via mass-normalization (M⁻¹ᐟ² is trivial
   elementwise for a diagonal M) reducing to a standard eigenproblem,
   diagonalized with the classic cyclic JACOBI ROTATION method — simple,
   robust, guaranteed to converge for any symmetric matrix, at the
   deliberate cost of being dense/O(n³)-per-sweep and finding ALL n
   eigenpairs even when only the lowest few matter (the same honest
   tradeoff `fem::Solver` documents for PCG over a direct sparse
   factorization — a Lanczos/subspace-iteration upgrade extracting only
   the lowest k modes from a sparse K/M without ever densifying it is a
   reasonable, meaningfully larger future addition once model sizes
   that actually need it are in play). **Caught a real bug before it
   went anywhere**: the first implementation used a hand-derived
   rotation-angle formula that looked plausible and compiled cleanly,
   but silently gave WRONG eigenvalues (verified against a 2-DOF
   spring-mass chain's closed-form characteristic-equation roots, an
   independent method from Jacobi rotation) — replaced with the
   well-established Numerical-Recipes-style formulation (solving
   directly for tan of the rotation angle via the smaller root of a
   quadratic, rather than working with the angle itself), re-verified
   against the same closed form, now exact. Also validated: mode shapes
   are mass-orthonormal (φₖᵀMφⱼ = δₖⱼ, the property that makes them
   usable directly in a modal-combination formula without a separate
   normalization pass) and directly satisfy the residual K·φₖ -
   λₖ·M·φₖ ≈ 0 for every mode, independent of how the modes were
   computed internally.
   `analysis/ModalAnalysis` is now ALSO done — it assembles the SAME
   coupled frame+shell stiffness `StaticAnalysis` uses, builds a lumped
   mass vector from each node's `core::Node::translationalMass()` /
   `rotationalMass()`, and calls `EigenSolver` on the result. A free
   rotational DOF with no rotational mass explicitly assigned gets a
   small NAMED REGULARIZATION mass (a documented fraction of the
   model's average translational mass — see the class doc comment)
   just to keep the eigenproblem well-posed, rather than requiring
   every model to hand-assign physically-meaningful rotational inertia
   most preliminary RC models don't have data for; genuine
   diaphragm-level torsional mass (which DOES matter for real seismic
   torsion) is explicitly NOT modeled yet and would need a real,
   separate addition. Validated against a single-story shear-building's
   natural frequency, matched EXACTLY (not just approximately) to the
   classic closed form sqrt(12·E·I/L³ / m)/2π — using a Section built
   with zero shear area specifically so the comparison is against pure
   Euler-Bernoulli theory, since `Section::rectangular()`'s normal
   Timoshenko shear correction legitimately (and correctly) softens a
   stocky column relative to that simple closed form, a real behavior
   this suite's `test_frame_element.cpp` already validates separately
   — conflating the two in one test would have meant deriving a
   shear-corrected closed form instead of using the textbook one
   directly. Also caught a test-design mistake of my own along the way:
   an early version of this test used two identical, fully decoupled
   columns and asserted a specific pattern on which mode came out
   first — invalid, because two identical uncoupled oscillators
   produce a genuinely DEGENERATE eigenspace (equal frequencies) whose
   individual eigenvector split is basis-dependent, not something a
   test should assert a specific answer for; simplified to an
   unambiguous single-column case instead of trying to special-case the
   degeneracy. `NOT YET DONE`: `analysis/ResponseSpectrum` (combining
   these mode shapes with a design spectrum via SRSS/CQC), `PDelta`,
   and `NonlinearAnalysis`.
7. ~~`analysis/ResponseSpectrum` (combining these mode shapes with a
   design spectrum via SRSS/CQC)~~ — **done**. `ResponseSpectrum`
   itself is a deliberately generic tabulated (period, Sa) lookup with
   linear interpolation — NOT tied to any one code's spectrum shape
   (BNBC 2020, ASCE 7, ...); that shape-specific formula belongs in
   `design/BNBC2020` (still not built) as a function that PRODUCES the
   table this class consumes, keeping the analysis layer reusable
   across codes. `ResponseSpectrumAnalysis` runs `ModalAnalysis`
   internally (same coupled frame+shell stiffness), computes each
   mode's participation factor and effective modal mass (reported
   directly — the cumulative mass-participation check, typically
   requiring ≥90% in each direction, is a real required engineering
   check, not an implementation detail to hide), and combines peak
   modal displacements/base shear via either SRSS or CQC (Der
   Kiureghian cross-correlation) — CQC's general formula reduces
   exactly to SRSS when every cross-term is forced to zero, so both
   methods share one combination code path rather than two parallel
   implementations that could drift apart. Validated against: linear
   interpolation and its boundary (hold-constant, not extrapolate)
   behavior; and — the real proof — a genuinely single-DOF system's
   combined displacement and base shear matched EXACTLY against the
   classic equivalent-static-force closed form (u = Sa·m/k, V = Sa·m),
   since with only one mode there is no approximation (SRSS, CQC, modal
   truncation) left to hide an error behind; SRSS and CQC also checked
   to agree closely for two well-separated, uncorrelated modes (as
   they must, since CQC's cross-term vanishes as modes separate).
   NOT YET DONE: per-node reaction / per-element internal-force
   recovery combined the same way (only displacements and overall base
   shear are computed today) — a natural next addition following the
   same per-mode-then-combine pattern already used here; and
   `analysis/PDelta`, `analysis/NonlinearAnalysis`.
8. ~~`analysis/PDelta`~~ — **done**: linear (geometric-stiffness) P-Delta,
   the standard practical approximation to second-order behavior used
   in seismic/gravity design practice (e.g. ASCE 7 / BNBC 2020
   stability-coefficient methods) — NOT a full nonlinear large-
   displacement analysis (that's `analysis/NonlinearAnalysis`, still
   not built). `FrameElement3D` gained a `geometricStiffnessLocal()` /
   `geometricStiffnessGlobal()` pair (the standard consistent
   geometric stiffness matrix for a 2-node cubic-Hermite beam-column
   element); `PDeltaAnalysis` runs an ordinary gravity-case
   `StaticAnalysis` first to recover each frame element's axial force,
   then re-assembles Ke+Kg and solves the lateral case against that
   augmented, P-softened stiffness. Validated three ways: (1) the
   isolated geometric-stiffness formula's discretized buckling load for
   a cantilever matches the classical Euler value `π²EI/(4L²)` to
   within the well-known ~0.75% single-element discretization error —
   and specifically on the correct (over-, not under-) estimating side,
   the Rayleigh-Ritz upper-bound property this discretization is
   provably supposed to have, a stronger check than a bare tolerance
   alone; (2) `PDeltaAnalysis`'s own result matches an independent
   direct 2x2 linear-algebra solve of the same augmented system; (3) at
   zero axial load, `PDeltaAnalysis` reproduces an ordinary
   `StaticAnalysis` run EXACTLY (Kg(P=0) is algebraically the zero
   matrix — not an approximate check). The resulting amplification
   factor for a representative test case (2.33x at P/Pcr≈0.57) also
   lands almost exactly on the classic engineering approximation
   1/(1−P/Pcr) (2.34x) — an independent sanity check beyond the direct
   linear-algebra comparison. Caught a genuine test-setup mistake along
   the way (not a library bug): for a VERTICAL member, this engine's
   local-to-global axis convention pairs global Ux with global Ry (not
   Rz) in bending, and local axial is global Uz, not Uy — the first
   version of this test restrained the wrong rotational DOF and
   accidentally blocked the very axial load path being tested,
   producing a silently-zero axial force; worked out the full
   local↔global mapping from the axis-convention formula itself to fix
   it, rather than guessing until numbers matched.
   SCOPE: only frame elements contribute geometric stiffness (Wall/Slab
   shells do not yet); axial force per member is taken as a single
   representative value (exact for a 2-force member with no distributed
   axial load, an explicit simplification otherwise).
9. ~~`analysis/GlobalAssembly` refactor~~ — **done**: the near-identical
   element-classification/construction/assembly/force-recovery logic
   that had accumulated across `StaticAnalysis.cpp`, `ModalAnalysis.cpp`,
   and `PDelta.cpp` (three consumers, past the "two doesn't justify the
   refactor risk" call made when `ModalAnalysis.cpp` was added) is now
   one shared module — `isFrameLike`/`isShellLike`, `buildElements()`
   (classifies + constructs every element into frame/shell/skipped
   buckets), `assembleStiffness()` (with an optional per-frame-element
   extra-stiffness hook, exactly what `PDeltaAnalysis` needs for
   geometric stiffness and `StaticAnalysis`/`ModalAnalysis` simply don't
   pass), `accumulateNodalInternalForces()`, `recoverFrameEndForces()`,
   and `buildReactionsAndResidual()`. Net effect on the three
   consumers: `StaticAnalysis.cpp` 322→77 lines, `ModalAnalysis.cpp`
   184→81, `PDelta.cpp` 294→105 — each now reads as "assemble, solve,
   recover" using the shared building blocks, not a page of scatter-loop
   bookkeeping. Verified with a full regression sweep across all 11
   existing test suites (68 tests) AFTER the refactor, with NO test
   changes required and — checked explicitly, not just "tests still
   pass" — bit-for-bit identical numeric output on every case that had
   a previous run to compare against (the portal-frame example, the
   coupled slab-on-columns flagship test, the shear-building modal
   frequency, the P-Delta amplification factor), the strongest evidence
   a pure refactor didn't silently change behavior.
10. ~~`analysis/NonlinearAnalysis`~~ — **done**, and with it every item
   originally planned for `src/analysis/`. Geometrically NONLINEAR
   static analysis for a linear-ELASTIC material: a fixed-point
   (successive-substitution) iteration on each frame element's axial
   force — start every element at P=0, assemble Ke+Kg(P), solve the
   FULL applied load (gravity and lateral together, in ONE load case —
   unlike `PDeltaAnalysis`'s two-case split), recover new axial forces,
   repeat until they stop changing. This is a genuine superset of
   `PDeltaAnalysis`'s single linear pass, not a separate formulation:
   the SAME `geometricStiffnessGlobal()` and the SAME
   `analysis/GlobalAssembly` helpers, just iterated to self-consistency
   instead of applied once. MATERIAL nonlinearity (plastic hinges,
   concrete cracking, a pushover capacity curve) is explicitly NOT
   modeled — a distinct, substantially larger addition for later.
   Validated primarily by an EXACT cross-check against
   `PDeltaAnalysis`: for the same decoupled cantilever-column model
   used in `PDeltaAnalysis`'s own test (axial force is exactly
   independent of the lateral load in that model — a straight
   prismatic member's elastic stiffness has no axial-transverse
   coupling), feeding gravity+lateral through `NonlinearAnalysis` as
   one combined case must give EXACTLY the same displacement as
   `PDeltaAnalysis` fed the same loads split into two cases — and it
   does, converging in 2 iterations. Also checked: zero applied load
   converges immediately to zero displacement; and — the discovery
   that mattered most from this addition — loading the same column
   just past its known buckling capacity should make the iteration
   fail, which surfaced a real gap in `fem::Solver`'s own positive-
   definiteness check (see the Status section above and item 3's own
   note below): the original check only caught curvature NEAR zero, not
   clearly NEGATIVE, so an over-buckling-load case could "converge" to
   a physically backwards answer (a displacement opposite the applied
   load's direction) rather than throwing. Fixed in `fem::Solver`
   itself — checking the SIGN of p^T·K·p, not just whether it's near
   zero — with its own dedicated regression test
   (`testIndefiniteMatrixThrows`, using a right-hand side deliberately
   aligned with the matrix's negative-eigenvalue eigenvector, since an
   RHS that happens to avoid that direction entirely can "solve" an
   indefinite matrix without incident and would have made a weaker
   test), then confirmed against a full 12-suite regression sweep (71
   tests) with zero other regressions.
11. ~~`design/BNBC2020`, `design/RCCBeam`~~ — **started** (not all of
   item 11 — `RCCColumn`/`RCCSlab`/`ShearWall`/`Steel`/`Foundation`
   remain, see below). `design/BNBC2020::generateBasicCombinations()`
   generates the seven ACI 318-19 Section 5.3.1 / BNBC 2020 basic
   strength-level load combinations from whichever load-case ids are
   supplied, skipping any combination whose defining load type (wind
   for the two wind combos, seismic for the two seismic combos) isn't
   actually present — caught a real bug in my own first version of this
   exact logic: the wind/seismic combinations were guarded by "dead OR
   wind" instead of requiring wind specifically, so "0.9D + 1.0W" was
   being generated for models with no wind load at all (just silently
   reducing to "0.9D" under a misleading name) — a unit test asserting
   that combination should NOT appear for a dead+live-only model caught
   it immediately. `design/RCCBeam` does singly-reinforced rectangular
   beam flexural design (Mu -> required As, checked against ACI 318-19
   Section 9.6.1.2 minimum steel and a tension-controlled maximum) and
   shear design (Vu -> required stirrup spacing, ACI 318-19 Chapter 9 /
   BNBC 2020 Part 6 Chapter 6). Validated primarily by ROUND-TRIP
   checks — design As for a given Mu, then independently recompute
   phi*Mn from that As using the standard Whitney stress-block formula
   and confirm it reproduces Mu exactly (not approximately — this is
   the literal definition of "designed for Mu"); same idea for shear
   (design a spacing, recompute phi*Vn from it, confirm it reproduces
   Vu). This kind of round-trip check is a stronger, more decisive
   validation than comparing against a remembered textbook example
   number, and doesn't depend on correctly recalling one.
   `design/RCCColumn` — **done**: rectangular tied-column design under
   axial load + biaxial bending. Builds a full strain-compatibility P-M
   interaction diagram independently about each axis (Whitney stress
   block, linear strain distribution, ecu=0.003, elastic-perfectly-
   plastic steel, phi varying continuously with net tensile strain per
   ACI 318-19 Table 21.2.2), then combines the two via Bresler's
   reciprocal load method (1/Pn = 1/Pnx + 1/Pny - 1/Po). Validated by:
   an independent closed-form check that `axialCapacityPo()` matches a
   hand-calculated value; a cross-check that the interaction diagram's
   own high-neutral-axis-depth tail converges to that same closed-form
   Po via a completely different code path (the strain-compatibility
   sweep) — two independent derivations of the same physical quantity
   agreeing is a much stronger check than either one alone; a physical-
   symmetry check that a symmetrically reinforced section under
   near-pure compression produces zero moment about its own centroid;
   an ACI Table 21.2.2 threshold check that phi lands at exactly 0.65 or
   0.90 at the compression-controlled and tension-controlled extremes;
   and a defining-property check that biaxial capacity can never exceed
   either uniaxial capacity alone. One known, documented limitation
   flagged rather than hidden: Bresler's method loses accuracy below
   Pu/Po ~ 0.10 (ACI 318-19 Commentary R22.4.2) — `designBiaxialColumn`
   detects that regime and returns a `lowAxialLoadWarning` rather than a
   falsely precise-looking number; the more accurate load-contour method
   for that regime is not implemented.
   `design/RCCSlab` — **done**: one-way slab flexural design for a
   1-meter design strip (Whitney stress block, same as RCCBeam, but with
   its OWN minimum — ACI 318-19 24.4.3.2 shrinkage-and-temperature steel
   on the gross section, not RCCBeam's 1.4/fy beam minimum, since a slab
   strip doesn't carry the same brittle-first-crack risk a beam's
   minimum guards against), a one-way shear adequacy check (ACI 318-19
   22.5.5.1, phi*Vc only — slabs aren't ordinarily stirrup-reinforced),
   and two-way (edge-supported) panel design via the RANKINE (elastic
   load-distribution) METHOD: total factored load split between the
   short/long spans in proportion to L_long^4/(L_short^4+L_long^4), each
   direction's simply-supported wl^2/8 then scaled by an ACI 318-19
   Table 6.5.2-style continuous-span coefficient (keyed by 0/1/2
   continuous edges) for positive and negative moment separately.
   Explicitly flagged as a real but SIMPLIFIED method — not the
   ACI 318-19 Direct Design Method (that applies to column-supported
   flat plates/slabs, a different system from this beam-supported case)
   and not the exact tabulated BNBC 2020 / ACI 318-63 "Method 3"
   elastic-plate-theory coefficients for the 9 standard edge-condition
   cases, which are not reproduced here — `designTwoWaySlabPanel()`
   states this in its own result rather than presenting the answer as
   code-table-exact, the same honesty pattern as RCCColumn's Bresler
   low-axial-load warning. Validated by: a round-trip check (design As
   for a given Mu, independently recompute phi*Mn from that As via the
   same Whitney formula, confirm it reproduces Mu exactly — the same
   strategy RCCBeam's own tests use); a conservation check that the
   Rankine load split's two shares sum back to exactly the total applied
   load; a symmetry check that a square, uniformly-continuous panel
   splits the load and moment evenly between both directions; and a
   check that a fully simply-supported panel produces exactly zero
   negative moment in both directions (no continuity to generate one).
   `design/RCCShearWall` — **done**: in-plane shear design (ACI 318-19
   Section 11.5.4: Vn = Acv*(alphaC*lambda*sqrt(fc')+rhoT*fy), capped at
   8*Acv*lambda*sqrt(fc') per Section 11.5.4.3, alphaC varying linearly
   with hw/lw between the squat (3.0) and slender (2.0) limits), the
   Section 11.6.2 coupling of minimum VERTICAL reinforcement to how much
   HORIZONTAL reinforcement a squat wall (hw/lw<2.5) actually has (a real
   code interaction easy to miss if you only look up Table 11.6.1 in
   isolation), an approximate story-shear-to-wall distribution by
   cantilever stiffness proportion (same style this engine's column
   lateral-distribution already uses), and out-of-plane flexural design
   for a 1-meter wall strip (same Whitney-stress-block mechanics as
   RCCBeam/RCCSlab, but governed by the wall's own Section 11.6.1
   vertical-reinforcement minimum, not either of those modules'
   minimums). Explicitly NOT modeled, flagged rather than assumed away:
   ACI 318-19 Section 18.10.6 special-boundary-element design for walls
   in a seismic system, and coupling-beam design between wall piers (a
   distinct module, matching the prototype's separate check) -- a wall's
   in-plane axial-moment (P-M) capacity is already covered by treating it
   as a wide column via `RCCColumn::designBiaxialColumn` (ACI 318-19
   Section 11.5.3 explicitly permits this), so that is not duplicated
   here. Validated by: a round-trip check (design rhoT for a Vu, recompute
   phi*Vn from that rhoT, confirm it reproduces Vu, the same strategy
   RCCBeam's own tests use); closed-form checks of alphaC at and between
   its two anchor points; a closed-form check of the Section 11.6.2
   vertical-coupling formula directly against the code equation,
   independent of the design function; a conservation check that
   distributed wall shears sum back to exactly the input story shear; a
   symmetry check that two identical walls split a story shear evenly;
   and the same round-trip strategy applied to the out-of-plane flexure
   design.

11. `RCCFoundation` — isolated (square) spread footing design — **done**:
    plan area sized from service load / net allowable bearing pressure,
    then thickness solved by stepping up from a 300mm minimum until
    BOTH ACI 318-19 two-way (punching, Section 22.6.5.2, SI 0.17/0.083/
    0.33 coefficients) and one-way (beam, Section 22.5.5.1, reusing
    `RCCSlab::checkOneWayShear` directly since a uniformly-pressured
    footing strip is mechanically the same as a uniformly-loaded slab
    strip) shear checks pass at that thickness, then flexural steel
    designed at the critical section at the column face (reusing
    `RCCBeam::designFlexure` with the full footing width as "b" — the
    same intensity-based result a 1m-strip formulation would give).
    Deliberately goes beyond the prototype's own isolated-footing case:
    `prototype/NRSA_RCC.html`'s `designFootingForColumn()` sizes
    isolated-footing thickness from a fixed side/7 rule of thumb and
    never runs an explicit shear check there (only its pile-cap and
    combined-footing branches do) — this module solves for a
    code-checked minimum thickness instead. SCOPE: square,
    concentrically-loaded footings under a single column's factored
    axial load only; combined/strap footings and pile foundations are
    not implemented. Validated by: an independent closed-form
    recomputation of the governing two-way shear coefficient for a
    square column (`beta`=1, isolating the `vc1` case), a round-trip
    check (design As at the critical-section moment, recompute phi*Mn
    from that As, confirm it reproduces or exceeds the demand — the
    same strategy RCCBeam's/RCCSlab's own tests use), and a sanity
    check that the net punching-shear demand never exceeds the column's
    gross factored load (it subtracts the bearing pressure already
    reacting under the critical perimeter's own footprint).

    `RCCFoundation` also now covers combined footing design — **done**:
    two or more columns along one straight axis on a single rectangular
    footing (caller supplies the width B; a combined footing's width is
    ordinarily set by a site/property-line constraint, not derived from
    a formula, so this module doesn't guess one). The footing is
    centered on the SERVICE-load centroid along its length (giving
    uniform net bearing pressure along that axis by construction,
    matching `designCombinedFootingGroup()`'s own method in the
    prototype), then treated as a beam under uniform upward pressure
    plus the columns' downward point loads — ordinary statics gives the
    sagging/hogging moment and shear envelopes directly. Thickness is
    solved against BOTH the one-way shear check along the beam axis AND
    every individual column's own two-way/punching shear check — the
    prototype's combined-footing branch only checks one-way shear, so
    the punching check here is the same "add the code check the
    prototype skipped" pattern the isolated-footing module above
    already follows. Top (hogging) and bottom (sagging) longitudinal
    steel are both designed at their respective governing sections.
    SCOPE, deliberately not matching the prototype's own combined-
    footing case: no B-direction (transverse) eccentricity/kern check
    and no transverse cantilever-bending steel design (both assume the
    columns sit on the footing's B-centerline); every column's punching
    perimeter is treated as a full 4-sided interior perimeter even near
    the footing's short ends. Validated by: hand statics on a two-
    column case (confirming not just the magnitude but the SIGN of the
    governing moment at the columns vs. at midspan — this geometry
    turned out to hog at midspan and sag only slightly at the columns,
    the mirror image of an ordinary overhanging beam, since here the
    distributed term is the upward soil reaction and the point loads
    are the downward columns; the test asserts that sign rather than
    assuming the more "obvious" sagging-at-midspan case), a geometry-
    vs-bearing-area governing-case check for the plan sizing, and the
    same phi*Mn round-trip strategy as the isolated-footing steel
    (applied to whichever of top/bottom actually governs).

    `RCCFoundation` also now covers pile cap design — **done**, closing
    out `Foundation`'s full scope from the prototype (isolated / combined
    / pile-cap are the only three cases the prototype itself has -- there
    is no separate "strap footing" case). Method matches the prototype's
    pile-cap pipeline: pile POSITIONS are a caller input (exact layout/
    spacing is a geotechnical/constructability decision, not a code-
    derived quantity — the same reasoning combined-footing width is a
    caller input above), cap plan is the pile group's bounding box plus
    pile diameter and edge distance, and every pile carries an equal
    share of Pu (the "rigid cap" idealization, concentric column load
    only — eccentric column moment transfer to the cap is explicitly out
    of scope, the same limitation the prototype itself documents).
    Thickness is solved against FOUR simultaneous checks: column
    punching through the cap (reusing `checkTwoWayShear`, checked
    conservatively against the full Pu unnetted against nearby pile
    reactions — the same simplification the prototype's own
    `pileCapRequiredDepthIn()` uses), every individual pile's own
    reaction punching UP through the cap (circular critical perimeter,
    with the exterior-angle truncation the prototype uses for piles
    close enough to a cap edge to cut off part of that circle), and
    one-way shear in both directions from pile reactions beyond each
    critical section (reusing `checkOneWayShear`). Flexural steel in
    both directions is designed from the rigid-cap pile-reaction moments
    at the column faces (reusing `designFlexure`). Validated by: a
    bounding-box hand calc for the plan size, a zero-moment-demand case
    (column face covers every pile -> no moment, by construction), a
    hand-calculated pile-reaction moment for a 4-pile group, an explicit
    truncation check (an interior pile on a large cap keeps its full
    circular perimeter; the same pile on a tightly-sized cap gets a
    measurably smaller one), and the same phi*Mn round-trip plus a
    square-symmetric-group cross-check (both directions must design
    identically) for the full pipeline.

   STILL NOT DONE (at the time item 11 above was written): `Steel`
   (see item 12, now done) and everything in `modeler/`, `bim/`,
   `boq/`, `bbs/`, `reports/`, `api/`, and `validation/`, none of
   which has been started.

12. ~~`design/Steel`~~ — **done**, closing out `src/design/` in full.
   Scope, per AISC 360-16 and matching the prototype's own beam/column
   steel scope: `WShapeSection` properties (A, Ix/Iy, Sx/Sy, Zx/Zy,
   rx/ry, J, Cw, rts) built PARAMETRICALLY from plate dimensions
   (d/bf/tf/tw) — this project ships no literal AISC Manual shape
   table, so `buildWShape()` is the equivalent of looking one up, the
   same pattern `core::Section`'s own `rectangular()`/`circular()`/
   `hollowRectangularTube()` factories already use; root fillets are
   not modeled, the same "thin-walled, no fillet" idealization
   `hollowRectangularTube()` already carries. `design/Steel::
   designFlexuralCapacity` covers Chapter F2 for doubly-symmetric
   COMPACT I-shapes bent about the major axis only — yielding (Mp),
   inelastic LTB (F2-2), and elastic LTB (F2-3/F2-4); `checkCompactness`
   (Table B4.1b) gates this and flags (not silently misapplies) a
   non-compact section, since Chapter F3/F4/F5 aren't implemented.
   `checkShearCapacity` covers Chapter G2.1 for an unstiffened web
   (kv=5.34) only — tension-field action (Chapter G3) isn't modeled.
   `designCompressionCapacity` covers Chapter E3 flexural buckling
   about whichever principal axis governs — torsional/flexural-
   torsional buckling (Chapter E4) and slender-element local buckling
   (Chapter E7) aren't modeled, a real but distinct addition mainly
   relevant to singly-symmetric/unsymmetric or thin built-up shapes,
   not a typical rolled doubly-symmetric W. `checkCombinedInteraction`
   covers Chapter H1.1 (both H1-1a and H1-1b, selected by Pr/Pc) given
   already-factored demands and already-phi-reduced capacities — no
   torsion interaction (H3) and no second-order amplification (that's
   the caller's responsibility, the same boundary `RCCColumn` already
   draws by taking factored Pu/Mu rather than performing its own
   second-order analysis). Composite (steel+concrete) member design
   and bolted/welded connection design remain explicitly out of scope,
   as originally planned — both distinct modules in their own right.
   Validated by: an independent parallel-axis re-derivation of Ix (a
   different code path from the "hollow minus notches" formula
   `buildWShape` itself uses) agreeing exactly; a shape-factor
   (Zx/Sx, Zy/Sy) sanity range check against known I-shape physical
   bounds; a closed-form check that Lb=0 gives Mn=Mp=Fy*Zx exactly;
   an exact check that the inelastic-LTB segment (F2-2) hits its own
   right-hand anchor point (0.7*Fy*Sx) exactly at Lb=Lr, by
   construction — paired with a NEAR-continuity check (not exact) that
   the elastic-LTB formula (F2-4) lands close to that same value there
   too, since Lr (F2-6) and F2-4 are algebraic inverses of each other
   but AISC publishes both with independently-rounded constants
   (0.078, 1.95, 6.76) that don't cancel out perfectly — a real ~0.1%
   feature of the code's own published coefficients, caught by testing
   both sides of the boundary rather than assuming symmetry; a closed-
   form check that a stocky web's shear capacity reduces to exactly
   0.6*Fy*Aw (Cv1=1, phi=1.0, G2.1a) while a deliberately slender web
   is flagged into G2.1b with Cv1<1; a short-column check that Fcr
   approaches Fy as KL/r->0; an independent re-derivation of the Euler
   elastic-buckling stress directly from I and KL (bypassing r=sqrt(I/A)
   entirely) agreeing with `designCompressionCapacity`'s own KL/r-based
   path; a similar near-continuity check (not exact, same E3-2/E3-3
   rounded-constant reasoning as F2 above) at the KL/r=4.71*sqrt(E/Fy)
   inelastic/elastic transition; and closed-form hand calculations of
   both H1-1a and H1-1b at and away from their own Pr/Pc=0.2 switch
   point.

13. `modeler/`, `bim/`, `boq/`, `bbs/`, `reports/`, `api/`,
   `validation/` — the surrounding tooling layer. Suggested order,
   since each depends on stable design output from the layer(s)
   before it:
   - ~~`bbs/` (Bar Bending Schedule) — consumes the reinforcement
     already computed by `RCCBeam`/`RCCColumn`/`RCCSlab`/
     `RCCShearWall`/`RCCFoundation` and turns it into cut/bend
     schedules per bar mark~~ — **done**. `practicalBarCount()`
     rounds a required steel area up to a whole number of bars of a
     chosen real diameter (the step every design/ module above stops
     short of, by design — they report areas, not buildable bar
     lists); `straightBarCutLengthMm()` and `stirrupCutLengthMm()`
     turn a member's clear dimension into an actual cut length,
     accounting for standard hook extensions (ACI 318-19 Table 25.3.2,
     applied uniformly to primary and stirrup/tie hooks — a documented
     simplification, see the module header) and the generic
     rebar-detailing bend-length deduction convention (1×db per 45° of
     bend). `makeFlexuralBarEntry()`, `makeStirrupEntry()`, and
     `makeColumnLongitudinalEntries()` wire this directly to
     `RCCBeam`'s `FlexuralDesignResult`/`ShearDesignResult` and
     `RCCColumn`'s `RectColumnLayout` respectively; `RCCFoundation`
     and `RCCShearWall` bars are also straight/perimeter bars and are
     scheduled via the same generic `straightBarCutLengthMm()`/
     `stirrupCutLengthMm()` helpers rather than needing their own
     dedicated builders. `simplifiedTensionLapSpliceLengthMm()`
     estimates a column bar's lap-splice length — explicitly flagged
     as a preliminary, code-*referenced*-but-not-*exact* stand-in for
     ACI 318-19 Section 25.4.2.3's full cover/spacing/confinement-
     dependent equation, which this module's callers don't
     necessarily have the inputs for at a first-pass-schedule stage.
     SCOPE: only straight bars (0–2 end hooks) and closed 2-leg
     rectangular stirrups/ties are modeled — L-bars, cranked (bent-up)
     bars, and multi-leg (>2) stirrups are real, distinct additions
     not covered here. `BarBendingSchedule` rolls entries up by total
     weight and by diameter, the input `boq/` (next in this list) will
     need for rebar cost. Validated by: closed-form checks of the hook-
     extension and bend-deduction formulas at their code-table/floor
     boundaries (e.g. a 10mm bar's 90° hook hitting the 75mm floor
     rather than 6×db); hand-calculated straight-bar and stirrup cut
     lengths; a cross-check that this module's density-derived unit
     weight agrees with the independent d²/162 shop-drawing rule of
     thumb to within ~0.2% for six ordinary bar sizes; an exact-
     boundary check that a required area exactly equal to N whole bars
     doesn't round up to N+1; and an end-to-end check that
     `makeColumnLongitudinalEntries()` run against a real
     `generateRectangularLayout()` output reproduces that layout's own
     bar count and diameter.
   - ~~`boq/` (Bill of Quantities) — concrete volume, formwork area, and
     rebar weight (from `bbs/`) rolled up into a cost/quantity
     takeoff~~ — **done**, covering the five rectangular member types
     `core::Section`/`design/` already scope to (beam, column, one-way/
     two-way slab panel, straight wall segment, rectangular isolated
     footing): `rectangularVolumeM3()` plus a per-member-type formwork-
     area function (beam: soffit+2 sides, top open; column: all 4
     faces; slab: soffit only; wall: both faces; footing: 4 side faces
     only, bottom/top unformed) — each documented with the ordinary,
     unobstructed casting condition it assumes, and what a different
     real site condition (a free slab edge, a single-face-formed
     basement wall) would need instead. `QuantityTakeoff::
     setRebarSchedule()` pulls total weight and the per-diameter
     breakdown directly out of a finished `bbs::BarBendingSchedule` —
     the exact hand-off this Roadmap entry originally called for.
     Concrete items are grouped by fc' (MPa) as the takeoff's own BOQ
     "grade" key rather than introducing a separate named-grade
     concept. Also reports rebar weight per m^3 of concrete
     (`rebarWeightPerConcreteVolumeKgPerM3()`) — a standard, informational-
     only quantity-surveying sanity metric, not something this module
     validates against an expected range. SCOPE: no unit-cost/rate
     schedule or priced total is modeled (quantities only, deliberately
     left for the caller); non-rectangular members (circular columns,
     the hollow-tube core, combined/pile-cap footings' own plans) are
     not covered. Validated by: hand-calculated volume/formwork checks
     for all five member-type builders (including the mm-to-m thickness
     conversion for slab/wall/footing inputs); a grouping check that
     two fc'=28 items and one fc'=35 item split correctly by grade; an
     end-to-end check that `setRebarSchedule()` against a real
     `bbs::BarBendingSchedule` reproduces that schedule's own total and
     per-diameter weights exactly; a hand-calculated kg/m^3 check; the
     zero-with-no-concrete edge case; and a check that calling
     `setRebarSchedule()` twice REPLACES rather than accumulates the
     rebar totals (a takeoff has exactly one rebar schedule).
   - `reports/` — PDF/HTML generation of the design calculations
     already produced by `src/design/`, formatted as an issued
     calculation package.
   - `validation/` — cross-module audit layer (the prototype's Model
     Check + Calc Validation Engine equivalent): consistency checks
     that span multiple design modules at once (e.g. does a column's
     supplied section match what the analysis model assumed, do
     beam/slab reactions balance against what the supporting member
     was designed for).
   - `modeler/` (Geometry, Grid, Selection) and `bim/` (IFC round-trip)
     — the interactive-modeling and BIM-interop layer; larger, more
     UI-facing efforts best tackled once the design/reporting pipeline
     above is stable, since they're what a future interactive front
     end would be built on top of.
   - `api/` (public C API) — a stable public surface for a UI to call;
     naturally comes last, once the shape of what needs exposing is
     clear from having built the layers above.

## License

See `LICENSE`.

## Drawing Generation Engine — Phase 1+2 (in progress)

`src/drawing/ElevationView.h/.cpp` implements the first two phases of a
scoped, multi-phase plan for the Drawing Generation Engine (Roadmap item
13): geometry extraction (projecting a Model's nodes/elements onto one
of the three principal planes) and 2D line-drawing SVG generation
(member outlines at true section depth, node markers, labels). No
analysis-result overlay, reinforcement detail, or sheet composition yet
— those are later phases and are not started. Verified by compiling
clean with `-Wall -Wextra` and running `tests/test_elevation_view.cpp`,
which checks projected coordinates, section-depth pull-through, and SVG
output against the same known portal-frame geometry used in
`examples/portal_frame_example.cpp`.

## A note on Settlement (Schmertmann)

The Schmertmann SPT-N layered settlement method lives in
`prototype/NRSA_RCC.html` (the legacy single-file prototype), not in
the compiled C++ engine under `src/design/`. Its own doc comment
already discloses scope limits (Es-from-N is a fixed Bowles-1997
screening correlation, not real CPT data; no creep/time factor;
axisymmetric square/circular Iz shape only, not a separate
strip-footing shape) — this was confirmed by reading the file, not
assumed.
