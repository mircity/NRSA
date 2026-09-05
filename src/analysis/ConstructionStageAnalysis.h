#pragma once

#include <unordered_map>
#include <vector>

#include "analysis/StaticAnalysis.h"  // NodeVector6
#include "core/Model.h"

namespace nrsa::analysis {

// One stage of a staged-construction sequence: which elements are
// ACTIVE (already built and load-carrying) as of this stage, and which
// LoadCase id represents the INCREMENTAL load newly applied at this
// stage (e.g. that floor's self-weight, added the moment it's cast --
// NOT the whole building's dead load applied all at once).
struct ConstructionStage {
    std::string label;
    std::vector<int> activeElementIds;
    int incrementalLoadCaseId = -1;  // -1 means "no new load this stage, just a new element set"
};

struct ConstructionStageResult {
    std::string label;
    std::unordered_map<int, NodeVector6> incrementalDisplacements;  // this stage's own increment
    std::unordered_map<int, NodeVector6> cumulativeDisplacements;   // running total through this stage
};

// STAGED (incremental) construction analysis -- the standard tall-
// building method (the same one SAP2000/ETABS call "Nonlinear Staged
// Construction", used in its linear-elastic form here): elements that
// don't exist yet in a given stage contribute ZERO stiffness, and each
// stage's newly-applied load is resisted only by the STIFFNESS ACTIVE
// AT THAT STAGE, not by the final, fully-built structure. Total
// displacement at the end is the SUM of each stage's incremental
// displacement -- genuinely different from (and, for a tall building,
// meaningfully less than what you'd wrongly compute via) a single
// one-shot StaticAnalysis of the finished structure under its total
// load, because a lower floor's columns shorten under load applied
// BEFORE the upper floors existed to add their own stiffness, while
// load applied by upper floors is resisted by the by-then-more-
// complete structure.
//
// MECHANISM: for each stage, every node not touched by any element in
// that stage's activeElementIds is temporarily fully restrained
// (restrainAll()) so it contributes no free DOFs to that stage's
// solve (a not-yet-built node has no meaningful stiffness path and
// must not appear as an unrestrained mechanism); the Model's ORIGINAL
// restraint pattern (the real, permanent supports) is saved before the
// first stage and exactly restored -- for every node -- once the last
// stage finishes, whether it completes normally or throws.
//
// SCOPE: linear-elastic only (no creep/shrinkage/time-dependent
// material behavior folded in automatically -- combine with
// analysis::CreepShrinkageAnalysis's equivalent loads on a per-stage
// basis if that's needed, this class does not do it for you). Element
// REMOVAL (e.g. temporary shoring struck out partway through) is not
// supported -- activeElementIds must be a non-decreasing sequence
// across stages (this is checked and throws std::invalid_argument if
// violated), since "elements disappearing over time" needs materially
// different bookkeeping (redistributing the force that removed element
// was carrying) that is a natural next addition, not this pass's job.
std::vector<ConstructionStageResult> runConstructionStages(
    Model& model, const std::vector<ConstructionStage>& stages);

}  // namespace nrsa::analysis
