# NRSA_RCC.html — the original browser prototype

This is the single-file, in-browser RCC structural-design application
that the `src/` C++ engine in the rest of this repo is written to
**replace**. It's kept here for reference, not as something to keep
extending — it has already hit the architectural ceiling that's the
whole reason this repo exists:

- **Solver**: a dense, O(n³), in-browser JavaScript solver, capped at a
  fixed node count to keep the browser tab from freezing. `src/fem/`
  is moving this onto a real sparse solver (`SparseMatrix` + `Solver`
  in the roadmap), which removes that cap.
- **Elements**: frame elements plus an equivalent wide-column-spine
  idealization for shear walls and lift/stair cores, and a slab plate
  solver that only runs one panel at a time (batch-swept across all
  panels, but never coupled into the same stiffness matrix as the
  frame). `src/fem/ShellElement` (roadmap) is meant to eventually give
  slabs and walls genuine coupled shell stiffness in the same solve as
  the frame — the one thing the browser architecture could not do.
- **Everything else** — RCC beam/column/slab/footing design per BNBC
  2020 / ACI 318, load generation, model checking, a calculation
  validation engine, differential settlement, detail drawings with DXF
  export — is fully working in this prototype and is the functional
  reference the `src/design/`, `src/validation/`, `src/bbs/`, etc.
  modules should eventually match or exceed.

## How to use it

It's a single self-contained HTML file — no build step, no server.
Open `NRSA_RCC.html` directly in a browser.

## Relationship to `src/`

Nothing in `src/` depends on this file, and nothing here depends on
`src/` — they don't share code. This folder exists so both artifacts
from this project's history live in one place, and so the specific
formulas/conventions already validated in the prototype (see its
in-app "Roadmap" module for a checklist of what's implemented) have
somewhere obvious to be cross-checked against while porting the same
logic into `src/design/` and friends.

## Recent changes (since the original snapshot)

Despite the "not something to keep extending" framing above, two
targeted upgrades have since been made directly to this prototype, at
the project owner's request, each hand-verified against a closed-form
or textbook check before being kept:

- **P-Delta / Second-Order (Response Spectrum & P-Delta module):**
  added a coupled multi-story second-order solve, `(K − K_g)·u = F`,
  alongside the original per-story θ-magnifier. It solves the whole
  story stack together instead of amplifying each story in isolation,
  so a soft/irregular story shows its own amplification instead of a
  uniform value, and flags genuine sway instability (loss of
  positive-definiteness) rather than only a θ-threshold heuristic.
- **Pushover (Nonlinear Analysis module):** column hinge strength now
  comes from a fiber-layer (8-layer) P-M interaction check (ACI
  Whitney stress block) at each column's actual sustained axial load,
  replacing the earlier flexure-only `0.5·Ast·fy·d` estimate that
  ignored axial load entirely.
- **Member-Level Frame Pushover (new sub-module, Nonlinear Analysis
  module):** a genuine 2D finite-element pushover for one selected
  grid line at a time — real nodes, real column/beam stiffness, real
  member-end plastic hinges (not one lumped hinge per whole story).
  Column ends reuse the fiber P-M capacity above; beam ends use a new
  ACI singly-reinforced flexure check (conservatively takes the
  smaller of the positive/negative capacity at both ends). Solved with
  the classical event-to-event elasto-plastic method. Validated
  outside this file against the closed-form fixed-base portal-frame
  sway mechanism (Pc = 4·Mp/h) and a 2-story/3-line frame — both
  matched. Known limits, disclosed in-app: one grid line per run (no
  rigid-diaphragm coupling across lines), gravity axial load held
  constant through the push (no P-Delta/overturning axial feedback),
  no strain hardening after first hinge.
- **Slab-Frame coupling (Slab FEM Mesh / Sweep / Slab Design modules):**
  slab edge boundary conditions no longer default to a binary "Fixed
  if another panel adjoins, Simply Supported otherwise" guess. When a
  real beam is modelled along an edge, its own Saint-Venant torsional
  stiffness (k=GJ/L, using the same cross-sectional constant C that
  ACI 318 §8.10.5.2 uses for βt) is applied as a genuine elastic
  rotational spring at that edge in the plate FEM, instead of an
  all-or-nothing constraint. Falls back to a wall (fixed) or the
  previous adjacent-panel heuristic where no edge beam exists.
  Validated outside this file: with the FEM's own boundary conditions
  set to the two extremes, a beam-coupled deflection correctly lands
  between the "all Simply Supported" and "all Fixed" bounds, and moves
  toward "Fixed" as the beam is made stiffer and toward "Simply
  Supported" as it's made slender — matches expected plate-on-elastic-
  edge behavior. Known limits, disclosed in-app: single-rectangle ACI
  torsional constant only (no T-beam/flange contribution), one
  governing beam per edge, and this is one-way coupling (beam
  stiffness informs the slab's edge restraint; the slab's own moment
  does not yet feed back to redesign that beam in the same pass).
- **Wall (Strip) Footing design + Footing/Wall combination (Foundation
  module):** the Foundation module previously only ever looked at
  columns — walls had no footing design at all. Added a genuine strip-
  footing design per running foot of wall (self-weight-based sizing,
  ACI Whitney-block flexure + one-way shear, same rigor as the
  existing column footing design), plus the "Footing/Wall combination"
  case: a column bearing on or next to a wall has its load checked
  against the wall footing's local bearing capacity (dispersion length
  = column width + 2×depth) and the footing is flagged for local
  widening if that capacity is exceeded. Validated outside this file
  with a synthetic project: a heavy (150 kip) combined column
  correctly triggers the widening flag, a light (8 kip) one correctly
  doesn't. Known limits, disclosed in-app: only the wall's own self-
  weight is traced automatically (any load framing into the wall from
  floors/beams must be entered manually), and the combination is a
  local-widening check, not the fully coupled beam-on-elastic-
  foundation solve the column-to-column Combined Footing design above
  uses.
- **Settlement detail — Schmertmann layered method for sand
  (Foundation module):** the "Settlement Estimate" previously used a
  single bulk Es value looked up from the soil TYPE label alone,
  ignoring the real SPT-N-vs-depth profile already entered in the
  Geotechnical module's boreholes. For granular (sand/rock) soils it
  now runs Schmertmann's (1970) strain-influence-factor method against
  that real profile (Es(kPa)=400(N+6), Bowles 1997 screening
  correlation), so a weak layer sitting right under the footing (where
  the triangular Iz factor peaks) now correctly drives more settlement
  than the same weak layer positioned deeper — something a single bulk
  Es value cannot distinguish. Clay soils are unchanged (Schmertmann
  doesn't apply; the existing elastic + consolidation method remains).
  Validated outside this file: looser sand settles more than denser
  sand for the same load, and a weak layer near the footing base
  settles more than the identical weak layer placed deeper — both
  matched expected trends. Known limits, disclosed in-app: Es-from-N
  is a single fixed correlation (a real CPT-based Schmertmann normally
  uses cone resistance, not SPT-N), no creep/time (C2) factor, and the
  axisymmetric (square/circular) Iz shape is used for all footings
  (not the separate strip-footing distribution).

All three of this project's chosen groupings — Analysis engine, Design
checks, and (partially) Drawing/Report — have now had real, hand-
validated work done. Remaining: real (non-schematic) elevation/section
drawings with multi-load-case force diagrams. See the in-app Roadmap
module and `NRSA_Roadmap.pdf` in this repo's `docs/` folder for full,
current status.

