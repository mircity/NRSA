#pragma once

#include <string>
#include <vector>

#include "autodesign/AutoDesign.h"

namespace nrsa::visualization {

// Roadmap Section 19 ("Result Visualization"): "3D model-e rong-coded
// contour dekhabe" (color-coded contours on the 3D model) for
// Moment/Shear/Axial/Displacement/Stress/Drift/Reinforcement Ratio.
//
// SCOPE, STATED PLAINLY: there is no 3D viewport/rendering engine
// anywhere in this project (same structural reason Section 2's
// interactive CAD UI is out of scope) -- an actual color-coded 3D
// contour display needs a GUI framework this offline C++ engine
// doesn't have and can't package itself. What this module produces is
// the DATA a real viewport would need to draw that contour: a
// utilization ratio (demand/capacity) per element, mapped to a
// standard traffic-light color category. Wiring this data into an
// actual 3D renderer is a deployment/frontend decision, not something
// this module does.
enum class UtilizationColor { Green, Yellow, Red };

struct ElementUtilization {
    int elementId = 0;
    double ratio = 0.0;
    UtilizationColor color = UtilizationColor::Green;
};

// Standard traffic-light thresholds: ratio < 0.7 -> Green (comfortable
// margin), 0.7 <= ratio <= 1.0 -> Yellow (adequate but tight),
// ratio > 1.0 -> Red (inadequate).
UtilizationColor colorForRatio(double ratio);

std::vector<ElementUtilization> buildColumnUtilizationMap(
    const std::vector<autodesign::ColumnDesignSummary>& columns);

}  // namespace nrsa::visualization
