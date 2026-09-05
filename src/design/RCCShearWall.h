#pragma once

#include <string>
#include <vector>

namespace nrsa::design {

// ---------------------------------------------------------------------------
// RCCShearWall -- reinforced-concrete structural/shear wall design per
// ACI 318-19 Chapter 11 / BNBC 2020 Part 6 Chapter 6, matching the scope
// already built out in the browser prototype (prototype/NRSA_RCC.html,
// "Wall / Shear Wall Design" module) for these same checks:
//
//   1. STORY-SHEAR DISTRIBUTION -- the governing story shear (from whatever
//      lateral analysis produced it) is distributed across the walls at
//      that story by an approximate cantilever stiffness proportion
//      (Vwall = Vstory * Iwall/hwall^3 / sum(Ij/hj^3)), the same style this
//      engine's column lateral-load distribution already uses. This is a
//      simplified rigid-diaphragm proportion, not a full 3D coupled
//      wall-frame interaction analysis (that comes from analysis/
//      StaticAnalysis's shell elements once wall elements are wired into
//      it -- see the module README).
//   2. IN-PLANE SHEAR DESIGN -- ACI 318-19 Section 11.5.4:
//         Vn = Acv*(alphaC*lambda*sqrt(fc') + rhoT*fy)
//      capped at Vn <= 8*Acv*lambda*sqrt(fc') (Section 11.5.4.3), with
//      alphaC varying with the wall's height-to-length (hw/lw) aspect
//      ratio per Section 11.5.4.3 (3.0 for squat walls hw/lw<=1.5, 2.0 for
//      slender walls hw/lw>=2.0, linear in between), phi=0.75. Required
//      horizontal (transverse) reinforcement ratio rhoT is solved for
//      directly from Vn = Vu/phi, then raised to the ACI 318-19 Section
//      11.6.1 minimum if that governs; vertical (longitudinal)
//      reinforcement then follows from Section 11.6.2's coupling to rhoT
//      for walls with hw/lw < 2.5 (a real code interaction most designers
//      forget: a squat wall's minimum VERTICAL steel actually depends on
//      how much horizontal steel it has).
//   3. OUT-OF-PLANE FLEXURE -- a below-grade or exterior wall spanning
//      vertically between floor levels as a 1-meter design strip, same
//      Whitney-stress-block mechanics RCCBeam::designFlexure and
//      RCCSlab::designOneWaySlab use, but with the wall's OWN ACI 318-19
//      Section 11.6.1 minimum vertical-reinforcement ratio (0.0012 for
//      deformed bars <=16mm at fy>=420MPa, 0.0015 for larger bars) --
//      neither RCCBeam's 1.4/fy beam minimum nor RCCSlab's shrinkage-
//      temperature minimum applies here, since a wall's ACI-specified
//      minimum is a distinct, generally smaller provision written
//      specifically for wall panels.
//
// NOT MODELED (a real, distinct piece of work each, flagged rather than
// silently assumed away): boundary-element (special confined boundary
// zone) design per ACI 318-19 Section 18.10.6 for walls in a seismic-
// force-resisting system; wall flexural (axial-moment interaction)
// capacity design -- ACI 318-19 Section 11.5.3 permits treating a wall as
// a wide column for this, so RCCColumn::designBiaxialColumn already
// covers a rectangular wall pier's in-plane P-M design if invoked with
// the wall's own b/h and reinforcement layout, and is not duplicated
// here; and coupling-beam design between wall piers (a distinct module,
// matching the prototype's separate "Coupling Beam Check").
// ---------------------------------------------------------------------------

// ACI 318-19 Section 11.5.4.3: alphaC varies linearly with the wall
// segment's height-to-length aspect ratio between the squat (hw/lw<=1.5,
// alphaC=3.0) and slender (hw/lw>=2.0, alphaC=2.0) limits.
double inPlaneShearAlphaC(double hwOverLw);

// ACI 318-19 Table 11.6.1 minimum reinforcement ratios (relative to gross
// concrete area) for a wall not part of a special seismic system.
// horizontal (transverse) minimum:
double minHorizontalWallReinforcementRatio(double barDiaMm);
// vertical (longitudinal) minimum, BEFORE any Section 11.6.2 in-plane-
// shear-coupling adjustment (see verticalReinforcementRatioFromShear
// below, which is what designInPlaneShear actually applies):
double minVerticalWallReinforcementRatio(double barDiaMm);

// ACI 318-19 Section 11.6.2: for hw/lw < 2.5, a wall's minimum vertical
// (longitudinal) reinforcement ratio is coupled to how much horizontal
// (transverse) reinforcement it actually has -- NOT simply the Table
// 11.6.1 vertical minimum by itself. For hw/lw >= 2.5, rho_l = rho_t
// directly (no separate vertical minimum floor beyond that).
double verticalReinforcementRatioFromShear(double rhoT, double hwOverLw, double rhoLMinTable);

struct InPlaneShearDesignResult {
    double acvMm2 = 0.0;
    double alphaC = 0.0;
    double rhoTRequired = 0.0;   // solved directly from Vn = Vu/phi, before flooring to the minimum
    double rhoTUsed = 0.0;       // max(rhoTRequired, Table 11.6.1 minimum) -- what's actually reported
    double rhoLRequired = 0.0;   // Section 11.6.2 vertical steel, coupled to rhoTUsed
    double vnKN = 0.0;
    double vnCapKN = 0.0;        // 8*Acv*lambda*sqrt(fc') upper bound, Section 11.5.4.3
    double phiVnKN = 0.0;
    double ratio = 0.0;          // Vu / phi*Vn
    bool adequate = false;
    bool capGoverned = false;    // Vu/phi exceeds the 8*Acv*sqrt(fc') cap -- no amount of steel helps
    std::string note;
};

// Designs the horizontal and (Section 11.6.2-coupled) vertical
// reinforcement ratios for a wall segment's in-plane shear demand.
// lwM is the wall's in-plane length, tM its thickness, hwM the story
// (unsupported) height used for the hw/lw aspect ratio. barDiaMm is the
// reinforcement diameter used to pick the Table 11.6.1 minimum band.
InPlaneShearDesignResult designInPlaneShear(double vuKN, double lwM, double tM, double hwM,
                                             double fcMPa, double fyMPa, double barDiaMm);

// One wall segment's geometry for story-shear distribution.
struct WallSegment {
    double lengthM = 0.0;
    double thicknessM = 0.0;
};

// Distributes a story's total lateral shear across the wall segments at
// that story by an approximate cantilever stiffness proportion (Vwall_i =
// Vstory * I_i/h^3 / sum(I_j/h^3)), all walls at the story sharing the
// same story height h. Returns one shear value per input wall, in the
// same order; the returned values sum to storyShearKN by construction
// (verified by tests) since this is a straight proportional split, not
// an independent per-wall capacity computation. Empty input returns an
// empty result rather than dividing by zero.
std::vector<double> distributeStoryShearByStiffness(const std::vector<WallSegment>& walls,
                                                     double storyHeightM, double storyShearKN);

struct OutOfPlaneFlexureResult {
    double asRequiredMm2PerM = 0.0;
    double asMinMm2PerM = 0.0;   // ACI 318-19 Section 11.6.1 vertical minimum, per meter width
    double rhoProvided = 0.0;
    bool governedByMinimum = false;
    bool exceedsMaximum = false;
    std::string note;
};

// Out-of-plane flexural design for a 1-meter-wide vertical wall strip
// (e.g. a below-grade wall under combined hydrostatic/earth pressure, or
// an exterior wall under wind), same Whitney-stress-block mechanics as
// RCCBeam::designFlexure / RCCSlab::designOneWaySlab but governed by the
// wall's own Section 11.6.1 vertical-reinforcement minimum rather than
// either of those modules' minimums (see header note above). tMm is the
// overall wall thickness.
OutOfPlaneFlexureResult designOutOfPlaneFlexure(double muKNmPerM, double tMm, double coverMm,
                                                 double barDiaMm, double fcMPa, double fyMPa);

}  // namespace nrsa::design
