#pragma once

#include <vector>

#include "design/RCCColumn.h"

namespace nrsa::optimization {

// Roadmap Section 11 ("Automatic Optimization"): "the engineer says
// 'Optimize Building' and NRSA tries different configurations (column
// size, beam size, wall thickness, shear wall location, rebar
// diameter, foundation size), then optimizes for minimum cost + code
// compliance + minimum drift + constructability, together."
//
// SCOPE, STATED PLAINLY: whole-building, multi-objective (cost + drift
// + constructability all at once, across every member category) topology
// optimization is a large, genuinely research-grade undertaking -- not
// something to approximate badly in one pass. What this module does is
// narrower but real: given a SINGLE column's actual demand (Pu, Mux,
// Muy) and a caller-supplied list of CANDIDATE (size, bar layout)
// configurations, it evaluates each one's real cost (concrete volume +
// steel weight, at caller-given unit prices) and real code compliance
// (via the already-verified design::designBiaxialColumn -- the SAME
// Bresler capacity check design::RCCColumn uses, not a separate
// simplified estimate), filters to only the compliant options, and
// returns the minimum-cost one -- exactly the roadmap's own worked
// example (Option A/B/C cost table, pick the cheapest one that meets
// drift/code limits), just for one column's cross-section rather than
// the whole building's beam/wall/foundation sizes at once. Extending
// this same evaluate-and-filter pattern to beams/walls/foundations, or
// searching many columns/stories together with a shared drift
// constraint, is a natural next addition -- the pattern is the same
// piece of work three more times, not a fundamentally different
// algorithm, but genuinely more work than one pass covers.
struct ColumnCandidate {
    std::string label;         // e.g. "Option A"
    double widthM = 0.0;
    double depthM = 0.0;
    double coverMm = 40.0;
    double barDiaMm = 20.0;
    int barsAlongB = 3;
    int barsAlongH = 3;
    double concreteUnitPriceKNPerM3 = 0.0;  // e.g. currency per m^3
    double steelUnitPriceKNPerTon = 0.0;    // e.g. currency per metric ton
};

struct ColumnOptionResult {
    std::string label;
    bool codeCompliant = false;
    double demandCapacityRatio = 0.0;  // Pu / Pn (biaxial), <=1.0 means compliant on axial-biaxial check
    double concreteVolumeM3 = 0.0;
    double steelWeightKg = 0.0;
    double totalCostKN = 0.0;          // concrete cost + steel cost, in whatever currency unit prices were given
};

struct OptimizationResult {
    std::vector<ColumnOptionResult> allOptions;  // every candidate, evaluated, in input order
    int bestOptionIndex = -1;                    // index into allOptions of the cheapest CODE-COMPLIANT option, or -1 if none qualify
};

// Evaluates every candidate against the given demand (per-column-length
// assumed 1.0 m for concrete volume purposes -- a caller optimizing a
// specific story-height column should scale concreteUnitPriceKNPerM3's
// resulting volume externally, or pass an already-length-scaled
// candidate; this function itself works per unit length, kept simple
// and explicit rather than silently assuming a story height it wasn't
// given) and returns every option's real cost/compliance plus which
// one is cheapest among the compliant ones. fcMPa/fyMPa apply to every
// candidate uniformly (matching design::designBiaxialColumn's own
// per-call material inputs). Throws std::invalid_argument if
// candidates is empty.
OptimizationResult optimizeColumnSection(const std::vector<ColumnCandidate>& candidates, double puKN,
                                          double muxKNm, double muyKNm, double fcMPa, double fyMPa);

// ADDED 2026-08-30: removes the "caller must enumerate every candidate
// by hand" limitation for the common case of searching a SQUARE column
// over a size range -- generates one ColumnCandidate for every size
// from minSizeM to maxSizeM (inclusive) in steps of stepM, all sharing
// the same cover/bar-diameter/layout/unit-price inputs, with a label
// like "300x300", "325x325", etc. Feed the result straight into
// optimizeColumnSection to get a genuine systematic search rather than
// a caller-curated shortlist. Still scoped to ONE column
// cross-section's SIZE only -- bar diameter, layout pattern, and
// (still) whole-building multi-member search remain the caller's/a
// future addition's job; this is one dimension of the search made
// automatic, not all of them. Throws std::invalid_argument if
// minSizeM/maxSizeM/stepM are non-positive or minSizeM > maxSizeM.
std::vector<ColumnCandidate> generateSquareColumnCandidates(double minSizeM, double maxSizeM,
                                                              double stepM, double coverMm,
                                                              double barDiaMm, int barsAlongB,
                                                              int barsAlongH,
                                                              double concreteUnitPriceKNPerM3,
                                                              double steelUnitPriceKNPerTon);

}  // namespace nrsa::optimization
