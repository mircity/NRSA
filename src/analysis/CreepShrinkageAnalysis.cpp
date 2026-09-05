#include "analysis/CreepShrinkageAnalysis.h"

#include <stdexcept>

#include "analysis/TemperatureAnalysis.h"

namespace nrsa::analysis {

double effectiveCreepModulus(double E_kPa, double creepCoefficient) {
    if (creepCoefficient < 0.0) {
        throw std::invalid_argument("effectiveCreepModulus: creep coefficient cannot be negative");
    }
    return E_kPa / (1.0 + creepCoefficient);
}

std::vector<CreepShrinkageResult> applyUniformShrinkage(
    Model& model, const std::vector<int>& elementIds, double shrinkageStrain, int loadCaseId) {
    // Mathematically identical to a uniform temperature change with
    // eps0 = shrinkageStrain directly (alpha*deltaT collapses to just
    // the strain itself) -- reuse the same equivalent-nodal-load
    // routine rather than duplicating the geometry/transform logic.
    auto tempResults = applyUniformTemperatureChange(model, elementIds, /*deltaTDegC=*/1.0,
                                                       /*alphaPerDegC=*/shrinkageStrain, loadCaseId);
    std::vector<CreepShrinkageResult> results;
    results.reserve(tempResults.size());
    for (const auto& r : tempResults) results.push_back({r.elementId, r.axialForceMagnitude});
    return results;
}

}  // namespace nrsa::analysis
