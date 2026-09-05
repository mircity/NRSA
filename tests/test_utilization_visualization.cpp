#include <cassert>
#include <iostream>

#include "visualization/UtilizationVisualization.h"

using namespace nrsa::visualization;
using namespace nrsa::autodesign;

// Hand-calc: threshold boundary behavior, exact per the documented rule.
static void testColorThresholdsHandCalc() {
    assert(colorForRatio(0.5) == UtilizationColor::Green);
    assert(colorForRatio(0.699) == UtilizationColor::Green);
    assert(colorForRatio(0.7) == UtilizationColor::Yellow);
    assert(colorForRatio(1.0) == UtilizationColor::Yellow);
    assert(colorForRatio(1.001) == UtilizationColor::Red);
    assert(colorForRatio(2.0) == UtilizationColor::Red);
    std::cout << "  testColorThresholdsHandCalc OK\n";
}

static void testColumnUtilizationMapping() {
    ColumnDesignSummary c1, c2, c3;
    c1.elementId = 1; c1.demandCapacityRatio = 0.5;
    c2.elementId = 2; c2.demandCapacityRatio = 0.9;
    c3.elementId = 3; c3.demandCapacityRatio = 1.2;

    auto result = buildColumnUtilizationMap({c1, c2, c3});
    assert(result.size() == 3);
    assert(result[0].color == UtilizationColor::Green);
    assert(result[1].color == UtilizationColor::Yellow);
    assert(result[2].color == UtilizationColor::Red);
    assert(result[2].elementId == 3);
    std::cout << "  testColumnUtilizationMapping OK\n";
}

int main() {
    std::cout << "test_utilization_visualization:\n";
    testColorThresholdsHandCalc();
    testColumnUtilizationMapping();
    std::cout << "All tests passed.\n";
    return 0;
}
