#pragma once

#include "autodesign/AutoDesign.h"
#include "core/Model.h"
#include "design/BNBC2020.h"
#include "modeler/AutoModelGenerator.h"

namespace nrsa::oneclick {

// Roadmap Section 20 ("One Click Structural Design"): "NEW PROJECT ->
// IMPORT/DRAW PLAN -> AUTO MODEL -> MATERIAL -> DESIGN CODE -> LOAD ->
// LOAD COMBINATION -> ANALYSIS -> MODEL CHECK -> AUTO DESIGN ->
// OPTIMIZATION -> REINFORCEMENT -> DRAWINGS -> BBS -> BOQ -> REPORT."
//
// GENUINE METHOD: chains modeler::runAutoModelGeneration (Section 2)
// with autodesign::runAutoDesignForBeams/runAutoDesignForColumns/
// runAutoDesignForFoundations (Section 9) into ONE function call --
// literally "click one button, get a designed building," for the
// SUBSET of that full workflow this project has actually built and
// verified: AUTO MODEL, LOAD (self-weight only), LOAD COMBINATION,
// ANALYSIS, AUTO DESIGN (beams, columns, and now foundations). NOT
// included, stated plainly: Wall/Slab auto-design (see
// autodesign::AutoDesign.h -- StaticAnalysis doesn't report shell
// forces yet), Optimization is a SEPARATE call a caller chooses to
// make (optimization::ColumnOptimizer), and Drawings/BBS/BOQ/Report
// are separate, already-existing modules this orchestrator does not
// yet wire together (a natural next addition -- the pattern for
// wiring one more stage in is the same as how AutoDesign/Foundation
// themselves got wired in here, not a new kind of problem).
struct OneClickResult {
    modeler::GeneratedModel geometry;
    std::vector<autodesign::BeamDesignSummary> beamDesigns;
    std::vector<autodesign::ColumnDesignSummary> columnDesigns;
    std::vector<autodesign::FoundationDesignSummary> foundationDesigns;
};

// Generates the building (per buildingParams), adds self-weight dead
// load (buildingParams.generateSelfWeightLoad must be true with a
// valid selfWeightLoadCaseId -- this function does not silently turn
// it on), builds BNBC 2020 basic combinations from the given
// BnbcLoadCaseIds, and runs beam+column+foundation auto-design against
// them. netAllowableBearingKPa/foundationCoverMm/foundationBarDiaMm
// feed runAutoDesignForFoundations (see its own doc comment for the
// approximate-service-load caveat that still applies here). Throws
// whatever runAutoModelGeneration/runAutoDesignForBeams/
// runAutoDesignForColumns/runAutoDesignForFoundations themselves throw
// for invalid inputs.
OneClickResult runOneClickDesign(Model& model, const modeler::BuildingGenerationParameters& buildingParams,
                                  const design::BnbcLoadCaseIds& loadCaseIds, double fcMPa, double fyMPa,
                                  double coverMm, double longBarDiaMm, double stirrupDiaMm,
                                  int stirrupLegs, double columnBarDiaMm, int columnBarsAlongB,
                                  int columnBarsAlongH, double netAllowableBearingKPa,
                                  double foundationCoverMm, double foundationBarDiaMm);

}  // namespace nrsa::oneclick
