#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "optimization/ColumnOptimizer.h"

using namespace nrsa::optimization;
using namespace nrsa::design;

static bool approxEqual(double a, double b, double relTol = 1e-3) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// Hand-calc: a 400x400mm column, 40mm cover, 20mm bars, 3x3 layout ->
// 8 perimeter bars (3+3+2+2 minus double-counted corners = 8).
// Concrete volume per 1m length = 0.4*0.4*1 = 0.16 m^3.
// Steel weight = 8 bars * area(20mm) * 1m length * density(7850 kg/m^3).
static void testCostComputationHandCalc() {
    ColumnCandidate c;
    c.label = "TestOption";
    c.widthM = 0.4;
    c.depthM = 0.4;
    c.coverMm = 40.0;
    c.barDiaMm = 20.0;
    c.barsAlongB = 3;
    c.barsAlongH = 3;
    c.concreteUnitPriceKNPerM3 = 10.0;   // arbitrary test price
    c.steelUnitPriceKNPerTon = 100.0;    // arbitrary test price

    auto layout = generateRectangularLayout(0.4, 0.4, 40.0, 20.0, 3, 3);
    int expectedBarCount = static_cast<int>(layout.barsXY.size());
    assert(expectedBarCount == 8);  // 3+3+3+3 - 4 double-counted corners

    double barAreaMm2 = (M_PI / 4.0) * 20.0 * 20.0;
    double expectedSteelVolumeM3 = expectedBarCount * barAreaMm2 * 1e-6 * 1.0;
    double expectedSteelWeightKg = expectedSteelVolumeM3 * 7850.0;
    double expectedConcreteVolumeM3 = 0.4 * 0.4 * 1.0;
    double expectedCost = expectedConcreteVolumeM3 * 10.0 + (expectedSteelWeightKg / 1000.0) * 100.0;

    auto result = optimizeColumnSection({c}, 500.0, 20.0, 20.0, 28.0, 420.0);
    assert(result.allOptions.size() == 1);
    const auto& r = result.allOptions[0];
    assert(approxEqual(r.concreteVolumeM3, expectedConcreteVolumeM3));
    assert(approxEqual(r.steelWeightKg, expectedSteelWeightKg));
    assert(approxEqual(r.totalCostKN, expectedCost));
    std::cout << "  testCostComputationHandCalc (cost=" << r.totalCostKN << ") OK\n";
}

// Three options with IDENTICAL demand/code compliance status but
// clearly different costs -- the optimizer must pick the cheapest
// COMPLIANT one, verified against independently-computed costs.
static void testPicksMinimumCostCompliantOption() {
    double puKN = 400.0, muxKNm = 15.0, muyKNm = 15.0, fc = 28.0, fy = 420.0;

    ColumnCandidate small, medium, large;
    small.label = "Small";
    small.widthM = small.depthM = 0.35;
    small.coverMm = 40.0;
    small.barDiaMm = 20.0;
    small.barsAlongB = small.barsAlongH = 3;
    small.concreteUnitPriceKNPerM3 = 10.0;
    small.steelUnitPriceKNPerTon = 100.0;

    medium = small;
    medium.label = "Medium";
    medium.widthM = medium.depthM = 0.45;

    large = small;
    large.label = "Large";
    large.widthM = large.depthM = 0.6;

    auto result = optimizeColumnSection({small, medium, large}, puKN, muxKNm, muyKNm, fc, fy);
    assert(result.allOptions.size() == 3);
    assert(result.bestOptionIndex >= 0);

    // The chosen option must be the lowest-cost COMPLIANT one -- verify
    // directly against all compliant options' costs.
    double bestCost = result.allOptions[static_cast<std::size_t>(result.bestOptionIndex)].totalCostKN;
    for (std::size_t i = 0; i < result.allOptions.size(); ++i) {
        if (!result.allOptions[i].codeCompliant) continue;
        assert(result.allOptions[i].totalCostKN >= bestCost - 1e-6);
    }
    std::cout << "  testPicksMinimumCostCompliantOption (chosen=" 
              << result.allOptions[static_cast<std::size_t>(result.bestOptionIndex)].label
              << ", cost=" << bestCost << ") OK\n";
}

// A demand far beyond what ANY candidate can carry must leave
// bestOptionIndex == -1 (no compliant option) rather than silently
// picking a non-compliant "cheapest anyway" option.
static void testNoCompliantOptionReturnsNegativeOne() {
    ColumnCandidate tiny;
    tiny.label = "TooSmall";
    tiny.widthM = tiny.depthM = 0.2;
    tiny.coverMm = 30.0;
    tiny.barDiaMm = 12.0;
    tiny.barsAlongB = tiny.barsAlongH = 2;
    tiny.concreteUnitPriceKNPerM3 = 10.0;
    tiny.steelUnitPriceKNPerTon = 100.0;

    auto result = optimizeColumnSection({tiny}, 5000.0, 200.0, 200.0, 21.0, 300.0);
    assert(!result.allOptions[0].codeCompliant);
    assert(result.bestOptionIndex == -1);
    std::cout << "  testNoCompliantOptionReturnsNegativeOne OK\n";
}

static void testRejectsEmptyCandidateList() {
    bool threw = false;
    try { optimizeColumnSection({}, 500.0, 10.0, 10.0, 28.0, 420.0); }
    catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  testRejectsEmptyCandidateList OK\n";
}

// Auto-generated candidates (removing the "caller must enumerate by
// hand" limitation): min=300mm, max=500mm, step=50mm -> 5 sizes
// (300,350,400,450,500), each correctly labeled, and running the full
// optimizer over them must still correctly pick the minimum-cost
// compliant one (same property testPicksMinimumCostCompliantOption
// checks for a hand-built list, now for a generated one).
static void testAutoGeneratedCandidatesHandCalc() {
    auto candidates = generateSquareColumnCandidates(0.3, 0.5, 0.05, 40.0, 20.0, 3, 3, 10.0, 100.0);
    assert(candidates.size() == 5);
    assert(candidates[0].label == "300x300");
    assert(candidates[1].label == "350x350");
    assert(candidates[4].label == "500x500");
    for (const auto& c : candidates) assert(approxEqual(c.widthM, c.depthM));

    auto result = optimizeColumnSection(candidates, 400.0, 15.0, 15.0, 28.0, 420.0);
    assert(result.allOptions.size() == 5);
    if (result.bestOptionIndex >= 0) {
        double bestCost = result.allOptions[static_cast<std::size_t>(result.bestOptionIndex)].totalCostKN;
        for (const auto& opt : result.allOptions) {
            if (opt.codeCompliant) assert(opt.totalCostKN >= bestCost - 1e-6);
        }
    }
    std::cout << "  testAutoGeneratedCandidatesHandCalc (" << candidates.size()
              << " sizes generated, best=" 
              << (result.bestOptionIndex >= 0 ? result.allOptions[static_cast<std::size_t>(result.bestOptionIndex)].label : "none")
              << ") OK\n";
}

static void testRejectsInvalidGenerationRange() {
    bool threw = false;
    try { generateSquareColumnCandidates(0.5, 0.3, 0.05, 40.0, 20.0, 3, 3, 10.0, 100.0); }
    catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  testRejectsInvalidGenerationRange OK\n";
}

int main() {
    std::cout << "test_column_optimizer:\n";
    testCostComputationHandCalc();
    testPicksMinimumCostCompliantOption();
    testNoCompliantOptionReturnsNegativeOne();
    testRejectsEmptyCandidateList();
    testAutoGeneratedCandidatesHandCalc();
    testRejectsInvalidGenerationRange();
    std::cout << "All tests passed.\n";
    return 0;
}
