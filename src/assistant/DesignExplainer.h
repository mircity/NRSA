#pragma once

#include <string>
#include <vector>

#include "design/RCCColumn.h"

namespace nrsa::assistant {

// Roadmap Section 10 ("AI Design Assistant"): an "NRSA Engineering
// Assistant" panel an engineer asks things like "Why is Column C24
// failing?" and gets back an explanation plus a suggested fix -- but
// "AI is NEVER the authority for calculation; calculation is the
// deterministic engine, AI is only the explanation/assistant layer"
// (the roadmap's own words).
//
// SCOPE, STATED PLAINLY: this module builds exactly that
// explanation layer -- and ONLY that layer -- from REAL deterministic
// engine output (design::BiaxialDesignResult, the same Bresler
// interaction check design::RCCColumn already computes and this
// project has hand-calc-verified). It does NOT embed a language model
// in this C++ engine (there is no ML/LLM runtime here, and even if
// there were, hard-coding one specific model's weights into a
// structural-analysis engine's build would be an unusual, brittle
// coupling) -- what it produces is a fully DETERMINISTIC, TEMPLATE-
// BASED natural-language rendering of a real engineering result:
// same inputs always produce the same sentence, because there is no
// model in the loop, just formatted numbers and a small set of
// if/else rules for which suggested fix to mention. A real "ask
// anything" assistant panel would still need this structured data as
// its factual grounding either way (exactly the "AI explains, engine
// calculates" split the roadmap itself insists on) -- wiring an actual
// LLM on top of this structured explanation (to handle open-ended
// phrasing/follow-up questions) is a deployment/product decision for
// wherever this engine is embedded, not something this offline C++
// library can package itself.
struct ColumnFailureExplanation {
    std::string headline;               // e.g. "C24 fails - interaction ratio = 1.18"
    double interactionRatio = 0.0;      // Pu/Pn (biaxial)
    bool fails = false;
    std::vector<std::string> suggestedFixes;  // e.g. "Increase column size to ..." / "Increase longitudinal reinforcement from X% to Y%"
};

// Builds the explanation directly from a real design::BiaxialDesignResult
// (the caller runs design::designBiaxialColumn itself -- this function
// does no calculation of its own, only formats and interprets an
// already-computed result, per the class's own scope statement). puKN
// is passed separately since BiaxialDesignResult itself does not store
// the demand it was evaluated against. columnLabel is used verbatim in
// the headline (e.g. "C24"). If the column passes
// (puKN <= result.phiPnKN), fails is false and suggestedFixes is
// empty -- there is nothing to suggest fixing.
ColumnFailureExplanation explainColumnResult(const std::string& columnLabel, double puKN,
                                              const design::BiaxialDesignResult& result,
                                              double currentSteelRatio, double widthM, double depthM);

// ADDED 2026-08-30: the same explanation pattern for a beam shear
// check, broadening coverage beyond columns (closing the "only covers
// Column" scope gap). Same rules: deterministic, template-based,
// formats a REAL design::ShearDesignResult (from design::designShear,
// already hand-calc verified in test_rcc_beam.cpp), no calculation of
// its own.
struct BeamShearExplanation {
    std::string headline;
    double demandCapacityRatio = 0.0;  // Vu / (phi*Vn)
    bool fails = false;
    std::vector<std::string> suggestedFixes;
};

// phiVnKN is phi*Vc + phi*Vs at the ACTUALLY PROVIDED stirrup spacing
// -- the caller computes this from design::ShearDesignResult's own
// vcKN plus the stirrup contribution at whatever spacing was actually
// detailed (design::ShearDesignResult itself reports the REQUIRED
// spacing for Vu to exactly govern, not a capacity at a chosen
// spacing -- these are two different numbers, and this function needs
// the latter).
BeamShearExplanation explainBeamShearResult(const std::string& beamLabel, double vuKN, double phiVnKN,
                                             double currentSpacingMm);

}  // namespace nrsa::assistant
