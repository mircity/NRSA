#pragma once

#include <vector>

#include "core/Load.h"
#include "core/Model.h"

namespace nrsa::design {

// Identifies which core::LoadCase (by id, from a Model) represents
// each load TYPE for combination purposes. -1 (the default) means
// "not present in this model" -- generateBasicCombinations skips any
// combination whose every nonzero term would reference a missing case,
// so a model without wind or seismic loads simply doesn't get those
// combinations, rather than generating a combination that silently
// evaluates to a dead-load-only case under a wind-combination's name.
struct BnbcLoadCaseIds {
    int dead = -1;      // D
    int live = -1;      // L
    int liveRoof = -1;  // Lr
    int snow = -1;      // S
    int rain = -1;      // R
    int wind = -1;      // W
    int seismic = -1;   // E -- already the full code-level seismic
                        // demand (E = Eh +/- Ev per ACI 318-19 Section
                        // 5.3.3 / BNBC 2020's equivalent); this class
                        // does not itself split E into horizontal/
                        // vertical components, it takes E as a single
                        // supplied load case.
};

// Generates the standard basic strength-level (factored/LRFD) load
// combinations -- ACI 318-19 Section 5.3.1 Eq. (5.3.1a) through
// (5.3.1g), numerically identical to BNBC 2020 Part 6 Chapter 2's
// factored combination table. Returns only combinations for which
// every load type they reference is actually present (see
// BnbcLoadCaseIds's own doc comment); ids for the returned
// LoadCombinations start at startId and increase sequentially, so a
// caller can pick a startId that avoids colliding with any
// LoadCombination ids already added to the Model.
//
// SCOPE: only the seven BASIC combinations are generated -- special
// load combinations (e.g. for flood, ponding, or atmospheric ice, ACI
// 318-19 Sections 5.3.4-5.3.10) are not, since they depend on load
// types this engine does not yet have dedicated support for computing
// in the first place.
std::vector<LoadCombination> generateBasicCombinations(const BnbcLoadCaseIds& ids, int startId = 1000);

}  // namespace nrsa::design
