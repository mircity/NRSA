#pragma once

#include <vector>

#include "analysis/StaticAnalysis.h"
#include "core/Model.h"
#include "design/RCCBeam.h"
#include "design/RCCColumn.h"
#include "design/RCCFoundation.h"

namespace nrsa::autodesign {

// Roadmap Section 9 ("Auto Design"): "the engineer just clicks Run
// Design" and NRSA runs the whole chain: Analysis -> Load Combination
// -> Member Forces -> Section Check -> Reinforcement Design ->
// Serviceability -> Drift -> Stability -> Detailing -> BBS -> BOQ ->
// Report.
//
// SCOPE, STATED PLAINLY: this orchestrates the first FIVE links of
// that chain -- Analysis, Load Combination, Member Forces, Section
// Check, Reinforcement Design -- for BEAM elements only, reusing
// EVERY piece already built and independently verified elsewhere in
// this project (analysis::StaticAnalysis, design::generateBasicCombinations,
// design::designFlexure/designShear) rather than reimplementing any of
// them. Column/Wall/Slab auto-design, Serviceability/Drift/Stability
// checks, Detailing/BBS/BOQ/Report generation are NOT included in this
// pass -- chaining those in too is a natural next step once this first
// slice is validated, not attempted here in one pass (BBS and BOQ
// already exist as separate modules -- see src/bbs, src/boq -- wiring
// this orchestrator's output into them is exactly that natural next
// step).
//
// METHOD: for each requested LoadCombination, this SUPERPOSES (linear
// combination) the per-load-case StaticAnalysis results already run
// for every load case that combination references -- exactly how a
// factored combination's member forces are supposed to be computed
// for a LINEAR analysis, not a separate direct solve per combination.
// The governing (envelope) moment/shear for each beam is the largest
// magnitude seen across every combination AND both of the beam's own
// end forces.
//
// KNOWN GAP INHERITED FROM StaticAnalysis: distributed/self-weight
// loads are not yet converted to equivalent nodal loads there (see
// StaticAnalysis.h's own class doc comment) -- so a beam loaded only
// by its own self-weight/a distributed load and no nodal load will
// show near-zero envelope forces here too, understating true demand.
// This orchestrator does not work around that; it inherits it,
// honestly, rather than silently patching over an upstream gap.
struct BeamDesignSummary {
    int elementId = 0;
    double muKNm = 0.0;                    // governing (envelope) design moment magnitude
    double vuKN = 0.0;                     // governing (envelope) design shear magnitude
    int governingMomentCombinationId = -1;
    int governingShearCombinationId = -1;
    design::FlexuralDesignResult flexure;
    design::ShearDesignResult shear;
};

// Runs the chain for every Beam-kind element in model. fcMPa/fyMPa are
// applied UNIFORMLY across every beam (core::Material does not yet
// carry a concrete-strength-grade field of its own -- a natural next
// addition; until then this is an explicit, caller-supplied uniform
// assumption, not a per-element lookup). Effective depth is computed
// as (Section::depth - coverMm - assumedLongBarDiaMm/2) -- the actual
// longitudinal bar diameter isn't known until AFTER flexural design
// picks it (the same chicken-and-egg every RC design tool resolves by
// assuming a bar size for the first pass), so assumedLongBarDiaMm is a
// caller-supplied assumption for that first pass, not a promise the
// final detailed bar will be exactly that size. stirrupDiaMm/
// stirrupLegs are likewise uniform. Throws whatever
// analysis::StaticAnalysis::run throws (e.g. a non-positive-definite
// system) if the model itself is not analyzable.
std::vector<BeamDesignSummary> runAutoDesignForBeams(Model& model,
                                                       const std::vector<LoadCombination>& combinations,
                                                       double fcMPa, double fyMPa, double coverMm,
                                                       double assumedLongBarDiaMm, double stirrupDiaMm,
                                                       int stirrupLegs);

// ADDED 2026-08-30: the same Analysis->Load Combination->Member
// Forces->Section Check->Reinforcement Design chain, for Column-kind
// elements, using design::designBiaxialColumn (the same
// hand-calc-verified Bresler check design::RCCColumn's own tests
// exercise) instead of RCCBeam's flexure/shear.
//
// METHOD DIFFERENCE FROM BEAMS: a column's capacity depends on Pu,
// Mux, AND Muy TOGETHER (the Bresler interaction, not three
// independently-checked limits) -- so this does NOT envelope axial
// force and biaxial moments independently across combinations the way
// beam design envelopes moment and shear separately. Instead, for
// EVERY (load combination, member end) pair, it runs the actual
// (Pu, Mux, Muy) triple from THAT specific case through
// designBiaxialColumn and keeps whichever single pair produced the
// worst demand/capacity ratio (Pu/phiPn) -- the physically correct way
// to find the governing case for an interaction check, not a
// per-component envelope that could mix and match numbers that never
// actually occurred together.
//
// STILL BEAM-CHAIN'S SAME KNOWN GAP: inherits StaticAnalysis's
// distributed/self-weight load limitation (see the class doc comment
// above). Wall and Slab auto-design are NOT included even now --
// StaticAnalysisResult only reports FRAME element forces
// (elementForces), so Wall/Slab (both 4-node SHELL elements) have no
// force output to design from yet; that is a StaticAnalysis-level gap,
// not something this orchestrator works around.
//
// KNOWN CAVEAT: which of FrameEndForces' momentY/momentZ maps to
// designBiaxialColumn's "Mux" (strong-axis bending, using the full
// depth per Section::rectangular's Iz=b*h^3/12) versus "Muy"
// (weak-axis, using Iy=h*b^3/12) was RESOLVED 2026-08-30 by tracing
// Section::rectangular's own Iz/Iy definitions and RCCColumn.cpp's own
// documented Mx=strong-axis convention: momentZ pairs with Mux,
// momentY pairs with Muy. An EARLIER version of this file had these
// two swapped -- invisible in testing until a genuinely RECTANGULAR
// (non-square) column test was added, because a square column's two
// axes are interchangeable and the swap made no numerical difference
// there. Fixed; see the fix's own commit-style comment in the .cpp for
// the full derivation.
struct ColumnDesignSummary {
    int elementId = 0;
    double puKN = 0.0;
    double muxKNm = 0.0;
    double muyKNm = 0.0;
    int governingCombinationId = -1;
    int governingEnd = 1;  // 1 or 2
    double demandCapacityRatio = 0.0;
    design::BiaxialDesignResult biaxial;
};

// barsAlongB/barsAlongH follow design::generateRectangularLayout's own
// parameterization. fcMPa/fyMPa/coverMm/barDiaMm/barsAlongB/barsAlongH
// apply uniformly to every column, same explicit-uniform-assumption
// caveat as runAutoDesignForBeams.
std::vector<ColumnDesignSummary> runAutoDesignForColumns(Model& model,
                                                           const std::vector<LoadCombination>& combinations,
                                                           double fcMPa, double fyMPa, double coverMm,
                                                           double barDiaMm, int barsAlongB, int barsAlongH);

// ADDED 2026-08-30 (closes the "Foundation sizing not wired in" gap
// noted for Sections 2/20): designs an isolated footing under each
// column whose BASE node (identified as the node, among its own two
// end nodes, that is fully translationally restrained -- i.e. an
// actual support) sits on. Reuses design::designIsolatedFooting (the
// existing, previously-verified Section 8 module) -- no new footing
// design math here, only the wiring from a column's own AutoDesign
// result to that function's inputs.
//
// APPROXIMATION, STATED PLAINLY: design::designIsolatedFooting needs
// BOTH a factored ultimate load (Pu, for strength/reinforcement design)
// AND an unfactored SERVICE load (for bearing-pressure sizing against
// the soil's allowable bearing capacity) -- but this project's
// AutoDesign pipeline only tracks FACTORED combinations throughout
// (see runAutoDesignForColumns), so no genuine unfactored reaction is
// available without re-running StaticAnalysis under a separate,
// all-factors-1.0 combination (a real, buildable next step, not done
// here). Instead, serviceLoadKN is approximated as
// puKN / assumedAverageLoadFactor (a caller-supplied divisor -- ACI/
// ASCE literature commonly cites an average combined load factor
// around 1.4-1.6 for typical D+L-dominated loading, so 1.5 is a
// reasonable default, NOT a substitute for actually computing the
// real service-level reaction). A caller with a genuine service load
// figure should call design::designIsolatedFooting directly instead of
// this wrapper.
struct FoundationDesignSummary {
    int columnElementId = 0;
    int baseNodeId = 0;
    double puKN = 0.0;
    double approximateServiceLoadKN = 0.0;
    design::DesignIsolatedFootingResult footing;
};

std::vector<FoundationDesignSummary> runAutoDesignForFoundations(
    Model& model, const std::vector<ColumnDesignSummary>& columnResults, double netAllowableBearingKPa,
    double fcMPa, double fyMPa, double coverMm, double barDiaMm, double assumedAverageLoadFactor = 1.5);

}  // namespace nrsa::autodesign
