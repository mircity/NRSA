#pragma once

#include <string>
#include <vector>

#include "design/RCCBeam.h"
#include "design/RCCSlab.h"

namespace nrsa::design {

// ---------------------------------------------------------------------------
// RCCFoundation -- isolated (square) spread footing design per ACI 318-19
// Chapter 13.2 (footing-specific provisions) / Section 22.6 (two-way
// shear) / Section 22.5 (one-way shear) / BNBC 2020 Part 6 Chapter 6.
//
// SCOPE: square, concentrically-loaded isolated footings under a single
// column's factored axial load only -- no applied moment/eccentricity, no
// combined or strap footings, no piles. This matches the prototype's own
// "isolated" footing case (prototype/NRSA_RCC.html's
// designFootingForColumn(), non-pile branch) in loading assumption, but
// goes beyond it in one deliberate respect: the prototype sizes isolated-
// footing THICKNESS from a fixed side/7 rule of thumb and never runs an
// explicit shear check for this case (only its pile-cap and combined-
// footing branches check shear there) -- this module instead solves for
// the minimum thickness that satisfies BOTH two-way (punching) and
// one-way (beam) shear per the actual code provisions, the same "flag
// it, don't paper over it" approach this engine takes everywhere else
// (fem::Solver's positive-definiteness fix, RCCColumn's Bresler-regime
// note, etc.). Combined/strap footings and pile foundations, matching
// the rest of the prototype's foundation scope, are not yet implemented
// here -- see README's Roadmap.
//
// One-way shear reuses nrsa::design::checkOneWayShear (RCCSlab.h)
// directly: under a uniform net bearing pressure, a footing's one-way
// shear strip is mechanically identical to a uniformly-loaded slab
// strip, so there's no reason to duplicate that formula. Flexural design
// similarly reuses nrsa::design::designFlexure (RCCBeam.h) -- see
// designIsolatedFooting()'s doc comment for why passing the FULL footing
// width as "b" gives the same intensity-based result as a 1m-strip
// formulation would.
// ---------------------------------------------------------------------------

struct FootingAreaSizingResult {
    double areaRequiredM2 = 0.0;
    double sideM = 0.0;  // square footing side, sqrt(area) -- NOT rounded to a
                         // constructible increment; the caller rounds up as needed.
    std::string note;
};

// Sizes a square footing's plan area from the (unfactored) SERVICE axial
// load and the NET allowable soil bearing pressure -- i.e. already
// reduced for footing self-weight/backfill overburden and already
// divided by whatever factor of safety the geotechnical report
// specifies. Both stay outside this module's scope, the same load
// convention the prototype's own qAllow (already divided by fsBearing)
// uses.
FootingAreaSizingResult sizeSquareFootingArea(double serviceLoadKN, double netAllowableBearingKPa);

struct TwoWayShearCheckResult {
    double b0Mm = 0.0;    // critical-perimeter length, ACI 318-19 22.6.4.1 (d/2 from column faces)
    double vcKN = 0.0;    // unreduced two-way shear capacity (governing of the three ACI formulas)
    double phiVcKN = 0.0;
    double vuKN = 0.0;    // net punching demand actually checked against phiVcKN
    bool adequate = false;
    std::string note;
};

// Two-way (punching) shear check at the critical section d/2 from the
// column faces, ACI 318-19 Section 22.6.5.2 (SI coefficients 0.17/0.083/
// 0.33 -- the same normal-weight-concrete, lambda=1 form RCCBeam's and
// RCCSlab's own Vc formulas use, NOT the imperial 2/4/40 coefficients the
// prototype's pile-cap sizing uses for the same code section -- see
// prototype's own pileCapRequiredDepthIn() comment on that distinction).
// colBMm/colHMm are the column's two plan dimensions and dMm the
// footing's effective depth, both assumed to produce an INTERIOR-column
// critical perimeter (alpha_s = 40) -- edge/corner footings, whose
// perimeter is truncated at the footing edge, are not modeled here.
// vuKN is the net punching demand: the column's factored load minus the
// factored net bearing pressure already reacting directly under the
// critical perimeter's own footprint (see designIsolatedFooting for how
// that net demand is derived).
TwoWayShearCheckResult checkTwoWayShear(double vuKN, double colBMm, double colHMm, double dMm,
                                         double fcMPa);

struct DesignIsolatedFootingResult {
    double sideM = 0.0;               // square footing side actually used (raw geometric requirement)
    double thicknessMm = 0.0;         // solved to satisfy both shear checks below
    double effectiveDepthMm = 0.0;    // thicknessMm - coverMm - barDiaMm/2
    double netBearingPressureKPa = 0.0;      // service-load bearing pressure actually developed
    double factoredBearingPressureKPa = 0.0;  // qu, driving the shear/moment demands below
    TwoWayShearCheckResult twoWayShear;
    OneWayShearCheckResult oneWayShear;  // from RCCSlab.h -- see header note on reuse
    FlexuralDesignResult flexure;        // from RCCBeam.h, critical section at the column face
    std::string note;
};

// Full isolated square footing design: sizes the plan area from the
// service load and net allowable bearing pressure, iteratively solves
// for the minimum thickness (stepped by thicknessStepMm from
// minThicknessMm) that satisfies BOTH shear checks, then designs the
// critical-section flexural steel (ACI 318-19 13.2.7.1: moment taken at
// the face of the column, governing direction only, since a rectangular
// column on a square footing gives two different cantilever lengths).
//
// puKN is the column's FACTORED axial load (drives the shear/moment
// demands via quKPa); serviceLoadKN is the UNFACTORED service load
// (drives plan sizing against netAllowableBearingKPa). These are two
// different numbers for the same column, and both are required -- the
// same load duality the prototype's own designFootingForColumn keeps
// (PuKip vs PserviceKN).
//
// Throws std::runtime_error if maxIterations is exhausted before both
// shear checks pass (an undersized starting thickness/step/column-to-
// footing ratio, not a real design outcome to silently return).
DesignIsolatedFootingResult designIsolatedFooting(double puKN, double serviceLoadKN,
                                                   double netAllowableBearingKPa, double colBM,
                                                   double colHM, double coverMm, double barDiaMm,
                                                   double fcMPa, double fyMPa,
                                                   double minThicknessMm = 300.0,
                                                   double thicknessStepMm = 25.0,
                                                   int maxIterations = 200);

// ---------------------------------------------------------------------------
// Combined footing -- a single rectangular footing carrying two or more
// columns laid out along ONE straight axis (the footing's "length", L;
// its perpendicular in-plan dimension is the footing "width", B, and IS
// a caller-supplied input, not derived here -- see doc comment below).
// Method matches prototype/NRSA_RCC.html's designCombinedFootingGroup():
// the footing is centered on the SERVICE-load centroid along L, giving a
// uniform net bearing pressure along that axis by construction, and the
// footing is then treated as a simply-supported-on-soil "beam" carrying
// a uniform upward line load (from that uniform pressure) and downward
// point loads at each column -- ordinary statics gives the sagging/
// hogging moment and shear envelopes directly, no beam-on-elastic-
// foundation analysis.
//
// DELIBERATE SCOPE LIMITS versus the prototype's own combined-footing
// case (documented rather than silently assumed away, same policy as
// the isolated-footing note above):
//   - No B-direction (transverse) eccentricity / kern check, and no
//     transverse cantilever bending design under each column -- both
//     assume the columns sit on the footing's B-centerline.
//   - Every column's punching-shear critical perimeter is treated as a
//     full 4-sided INTERIOR perimeter, even for a column near the
//     footing's short end where the true perimeter would be truncated
//     (3-sided) -- same simplification checkTwoWayShear already carries
//     for isolated footings.
//   - Two-way (punching) shear per column IS checked, unlike the
//     prototype's combined-footing branch, which only checks one-way
//     shear along the beam axis -- the same "add the code check the
//     prototype skipped" pattern the isolated-footing module follows.
// ---------------------------------------------------------------------------

struct CombinedFootingColumnLoad {
    double positionM = 0.0;    // position along the footing's LONGITUDINAL (L) axis, in
                                // whatever consistent coordinate system the caller uses
                                // for every column in the group
    double puKN = 0.0;         // factored axial load
    double serviceLoadKN = 0.0;
    double colLengthM = 0.0;   // column plan dimension along the L axis
    double colWidthM = 0.0;    // column plan dimension along the B axis
};

struct CombinedFootingPlanResult {
    double lengthM = 0.0;             // footing length L along the longitudinal axis
    double widthM = 0.0;              // footing width B, echoed back from the caller's input
    double footingStartM = 0.0;       // position (caller's coordinate system) of the footing's start edge
    double resultantPositionM = 0.0;  // service-load centroid position -- sits at footingStartM + lengthM/2 by construction
    std::string note;
};

// Sizes the footing's plan length L for a caller-chosen width B: L is the
// larger of (a) total service load / (B * net allowable bearing) and
// (b) the minimum length needed to keep every column at least
// edgeClearanceM inside the footing edge while staying centered on the
// service-load centroid (so BOTH edge-clearance conditions -- nearest
// column to the start edge and nearest column to the end edge -- must
// hold simultaneously, not just the overall span between the outermost
// columns as a single lump check).
CombinedFootingPlanResult sizeCombinedFootingPlan(const std::vector<CombinedFootingColumnLoad>& columns,
                                                   double widthM, double netAllowableBearingKPa,
                                                   double edgeClearanceM = 0.3);

struct CombinedFootingInternalForces {
    double maxSaggingMomentKNm = 0.0;   // positive (bottom-tension) moment, TOTAL across the full width B
    double saggingPositionM = 0.0;      // measured from plan.footingStartM
    double maxHoggingMomentKNm = 0.0;   // magnitude of the negative (top-tension) moment, TOTAL across width B
    double hoggingPositionM = 0.0;
    double governingShearKN = 0.0;      // magnitude, TOTAL across width B, sampled at column-face-adjacent critical sections when effectiveDepthM > 0
    double governingShearPositionM = 0.0;
};

// Computes the sagging/hogging moment envelope and governing one-way
// shear for the footing-as-beam under uniform net factored pressure plus
// the columns' factored point loads. When effectiveDepthM > 0, shear is
// additionally sampled at +/-d from each column position (the standard
// one-way-shear critical-section locations); pass 0.0 (the default) for
// a first-pass estimate before an effective depth is known.
CombinedFootingInternalForces computeCombinedFootingInternalForces(
    const std::vector<CombinedFootingColumnLoad>& columns, const CombinedFootingPlanResult& plan,
    double effectiveDepthM = 0.0, int numSamples = 200);

struct DesignCombinedFootingResult {
    CombinedFootingPlanResult plan;
    double thicknessMm = 0.0;
    double effectiveDepthMm = 0.0;
    double factoredBearingPressureKPa = 0.0;
    CombinedFootingInternalForces forces;
    FlexuralDesignResult bottomFlexure;  // sagging-moment steel (bottom, longitudinal)
    FlexuralDesignResult topFlexure;     // hogging-moment steel (top, longitudinal) -- 0 if no hogging region exists
    OneWayShearCheckResult oneWayShear;
    std::vector<TwoWayShearCheckResult> punchingPerColumn;  // index-aligned with the input columns vector
    std::string note;
};

// Full combined-footing design: plans the footing for the given width B,
// then solves for the minimum thickness (stepped by thicknessStepMm from
// minThicknessMm) satisfying BOTH the one-way shear check along the beam
// axis and every column's two-way (punching) shear check, then designs
// longitudinal bottom (sagging) and top (hogging) flexural steel at the
// governing sections. Throws std::runtime_error if maxIterations is
// exhausted first (see designIsolatedFooting's note -- same reasoning).
DesignCombinedFootingResult designCombinedFooting(const std::vector<CombinedFootingColumnLoad>& columns,
                                                   double widthM, double netAllowableBearingKPa,
                                                   double coverMm, double barDiaMm, double fcMPa,
                                                   double fyMPa, double edgeClearanceM = 0.3,
                                                   double minThicknessMm = 350.0,
                                                   double thicknessStepMm = 25.0,
                                                   int maxIterations = 200);

// ---------------------------------------------------------------------------
// Pile cap -- a single rectangular cap over a group of piles under one
// concentric column load. Method matches prototype/NRSA_RCC.html's
// pile-cap pipeline (pileCapBoundingFt / pileCapRequiredDepthIn /
// pileIndividualPunchingCheck / pileCapMomentDemand / pileCapOneWayShear):
// the "rigid cap" idealization -- an equal reaction at every pile from a
// concentric column load, cap plan sized as the bounding box of the pile
// group plus pile diameter and edge distance, thickness governed by
// column-punching-through-cap and per-pile punching-up-through-cap, and
// flexure/one-way shear both computed from pile reactions beyond the
// relevant critical section.
//
// DELIBERATE SCOPE LIMIT, same one the prototype itself documents:
// eccentric column moment transfer to the cap is not modeled -- puKN is
// assumed CONCENTRIC on the pile group, so every pile carries puKN/n. Pile
// axial capacity, geotechnical group efficiency, and lateral load
// distribution are all outside this module too (those live in the
// geotechnical design, upstream of "does this cap survive the reactions
// piles are assumed to already be carrying"). Pile POSITIONS are a
// caller input rather than derived from an arrangement-table heuristic --
// exact layout/spacing is a geotechnical/constructability decision, not
// a code-derived quantity, the same reasoning combined-footing width is
// a caller input above.
//
// Column-punching-through-cap reuses checkTwoWayShear directly, checked
// against the FULL puKN (not netted against nearby pile reactions) --
// the same conservative simplification pileCapRequiredDepthIn() uses in
// the prototype.
// ---------------------------------------------------------------------------

struct PilePosition {
    double xM = 0.0;  // relative to the column/cap centroid (concentric-load assumption)
    double yM = 0.0;
};

struct PileCapPlanResult {
    double lengthM = 0.0;  // cap extent along x (pile bounding box + pile diameter + 2*edge distance)
    double widthM = 0.0;   // cap extent along y
    std::string note;
};

// Cap plan size as the piles' bounding box, expanded by the pile diameter
// and edgeDistanceM on every side (edgeDistanceM is the clear distance
// from the outermost pile's face to the cap edge).
PileCapPlanResult sizePileCapPlan(const std::vector<PilePosition>& piles, double pileDiameterM,
                                  double edgeDistanceM = 0.15);

struct PilePunchingCheckResult {
    double xM = 0.0, yM = 0.0;  // pile position, echoed
    double b0Mm = 0.0;          // effective critical-perimeter length -- reduced from a full circle
                                // when the pile sits close enough to a cap edge to truncate it
    double phiVcKN = 0.0;
    double demandKN = 0.0;      // puKN / number of piles (rigid-cap, equal reaction)
    bool adequate = false;
    bool truncated = false;
};

// Per-pile punching check: each pile's own (equal-share) reaction
// punching UP through the cap around that pile, critical perimeter at
// d/2 from the pile face (ACI 318-19 22.6, circular column/pile form,
// beta=1). When a pile sits closer to a cap edge than the critical
// radius, that side of the circular perimeter is cut off (truncated) --
// modeled via the exterior subtended angle at each of the four cap edges,
// with the remaining perimeter floored at a quarter circle as a
// conservative stand-in for the more complex exact geometry a corner
// pile's two overlapping truncations would otherwise need.
std::vector<PilePunchingCheckResult> checkPileIndividualPunching(const std::vector<PilePosition>& piles,
                                                                   const PileCapPlanResult& plan,
                                                                   double pileDiameterM, double dMm,
                                                                   double puKN, double fcMPa);

struct PileCapMomentDemand {
    double momentAlongXKNm = 0.0;  // from piles beyond the column face at x=+-colBM/2 -- bends the cap about the y-axis
    double momentAlongYKNm = 0.0;  // from piles beyond the column face at y=+-colHM/2 -- bends the cap about the x-axis
};

// Rigid-cap moment demand at the column faces: each pile contributes
// puKN/n times its distance beyond the relevant column face (ACI 318-19
// 13.2.7.1's "critical section at the column face", applied to discrete
// pile reactions instead of a continuous bearing pressure).
PileCapMomentDemand computePileCapMomentDemand(const std::vector<PilePosition>& piles, double puKN,
                                                double colBM, double colHM);

// One-way shear demand at the critical section d from the column face
// (ACI 318-19 22.5.5.1's usual d-offset, applied to discrete pile
// reactions beyond that section rather than a continuous pressure): sum
// of puKN/n for every pile whose center lies beyond the critical section,
// returned as a per-meter-width intensity (dividing by the cap's extent
// perpendicular to the axis in question) ready for checkOneWayShear.
double computePileCapOneWayShearDemandPerM(const std::vector<PilePosition>& piles, double puKN,
                                            double colHalfM, double dM, double perpendicularExtentM,
                                            bool alongXAxis);

struct DesignPileCapResult {
    PileCapPlanResult plan;
    double thicknessMm = 0.0;
    double effectiveDepthMm = 0.0;
    TwoWayShearCheckResult columnPunching;             // column punching through the cap (vs full puKN, unnetted)
    std::vector<PilePunchingCheckResult> pilePunching;  // index-aligned with the input piles vector
    OneWayShearCheckResult oneWayShearX;
    OneWayShearCheckResult oneWayShearY;
    FlexuralDesignResult flexureX;  // steel running in x, resisting momentAlongXKNm, section width = plan.widthM
    FlexuralDesignResult flexureY;  // steel running in y, resisting momentAlongYKNm, section width = plan.lengthM
    std::string note;
};

// Full pile cap design: plans the cap from the pile group, then solves
// for the minimum thickness (stepped by thicknessStepMm from
// minThicknessMm) satisfying column punching, EVERY pile's individual
// punching check, and one-way shear in both directions, then designs
// flexural steel in both directions from the rigid-cap pile-reaction
// moments. Throws std::runtime_error if maxIterations is exhausted first
// (see designIsolatedFooting's note -- same reasoning).
DesignPileCapResult designPileCap(const std::vector<PilePosition>& piles, double puKN, double colBM,
                                   double colHM, double pileDiameterM, double coverMm, double barDiaMm,
                                   double fcMPa, double fyMPa, double edgeDistanceM = 0.15,
                                   double minThicknessMm = 450.0, double thicknessStepMm = 25.0,
                                   int maxIterations = 200);

}  // namespace nrsa::design
