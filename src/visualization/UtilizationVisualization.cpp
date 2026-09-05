#include "visualization/UtilizationVisualization.h"

namespace nrsa::visualization {

UtilizationColor colorForRatio(double ratio) {
    if (ratio < 0.7) return UtilizationColor::Green;
    if (ratio <= 1.0) return UtilizationColor::Yellow;
    return UtilizationColor::Red;
}

std::vector<ElementUtilization> buildColumnUtilizationMap(
    const std::vector<autodesign::ColumnDesignSummary>& columns) {
    std::vector<ElementUtilization> result;
    for (const auto& c : columns) {
        ElementUtilization u;
        u.elementId = c.elementId;
        u.ratio = c.demandCapacityRatio;
        u.color = colorForRatio(u.ratio);
        result.push_back(u);
    }
    return result;
}

}  // namespace nrsa::visualization
