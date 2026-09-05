#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "construction/CostEstimate.h"

using namespace nrsa::construction;

static bool approxEqual(double a, double b, double relTol = 1e-6) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// Hand-calc: concrete=50m^3 @ 10/m^3 = 500; steel=2000kg=2ton @ 100/ton
// = 200; formwork=300m^2 @ 5/m^2 = 1500; total = 500+200+1500=2200.
static void testCostAggregationHandCalc() {
    auto est = estimateProjectCost(50.0, 10.0, 2000.0, 100.0, 300.0, 5.0);
    assert(approxEqual(est.concreteCost, 500.0));
    assert(approxEqual(est.steelCost, 200.0));
    assert(approxEqual(est.formworkCost, 1500.0));
    assert(approxEqual(est.totalCost, 2200.0));
    std::cout << "  testCostAggregationHandCalc (total=" << est.totalCost << ") OK\n";
}

int main() {
    std::cout << "test_cost_estimate:\n";
    testCostAggregationHandCalc();
    std::cout << "All tests passed.\n";
    return 0;
}
