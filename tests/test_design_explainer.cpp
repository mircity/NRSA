#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "assistant/DesignExplainer.h"

using namespace nrsa::assistant;
using namespace nrsa::design;

static bool approxEqual(double a, double b, double relTol = 1e-3) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// Hand-calc: Pu=1000kN, phiPnKN=847.5kN -> ratio=1000/847.5=1.1799...
// matching the roadmap's own worked example format ("interaction ratio = 1.18").
static void testFailingColumnHandCalc() {
    BiaxialDesignResult result;
    result.phiPnKN = 847.5;
    result.adequate = false;

    double puKN = 1000.0;
    auto exp = explainColumnResult("C24", puKN, result, 0.021, 0.4, 0.4);

    double expectedRatio = puKN / result.phiPnKN;
    assert(approxEqual(exp.interactionRatio, expectedRatio));
    assert(approxEqual(exp.interactionRatio, 1.180, 1e-3));
    assert(exp.fails);
    assert(exp.headline.find("C24") != std::string::npos);
    assert(exp.headline.find("fails") != std::string::npos);
    assert(!exp.suggestedFixes.empty());

    // Hand-calc: size scale factor = sqrt(1.1799) = 1.0862;
    // newWidth = 400*1.0862 = 434.5mm -> ceil to nearest 25mm = 450mm
    double scaleFactor = std::sqrt(expectedRatio);
    double expectedNewWidthMm = std::ceil((400.0 * scaleFactor) / 25.0) * 25.0;
    assert(approxEqual(expectedNewWidthMm, 450.0));
    bool foundSizeFix = false;
    for (const auto& fix : exp.suggestedFixes) {
        if (fix.find("450") != std::string::npos) foundSizeFix = true;
    }
    assert(foundSizeFix);

    // Hand-calc: suggested steel ratio = min(0.021*1.1799, 0.04) = 0.02478 -> 2.5%
    double expectedSuggestedRatio = std::min(0.021 * expectedRatio, 0.04);
    assert(approxEqual(expectedSuggestedRatio, 0.02478, 1e-3));
    bool foundSteelFix = false;
    for (const auto& fix : exp.suggestedFixes) {
        if (fix.find("2.1%") != std::string::npos) foundSteelFix = true;
    }
    assert(foundSteelFix);

    std::cout << "  testFailingColumnHandCalc (ratio=" << exp.interactionRatio
              << ", headline=\"" << exp.headline << "\") OK\n";
}

// A passing column must report fails=false and zero suggested fixes --
// there's nothing to suggest fixing.
static void testPassingColumnHasNoSuggestions() {
    BiaxialDesignResult result;
    result.phiPnKN = 1000.0;
    result.adequate = true;

    auto exp = explainColumnResult("C10", 800.0, result, 0.015, 0.4, 0.4);
    assert(!exp.fails);
    assert(exp.suggestedFixes.empty());
    assert(approxEqual(exp.interactionRatio, 0.8));
    assert(exp.headline.find("OK") != std::string::npos);
    std::cout << "  testPassingColumnHasNoSuggestions OK\n";
}

// The exact boundary (Pu == phiPn, ratio == 1.0) must NOT be flagged
// as failing (fails is defined as Pu > phiPnKN, strictly greater).
static void testExactBoundaryIsNotFailing() {
    BiaxialDesignResult result;
    result.phiPnKN = 500.0;
    auto exp = explainColumnResult("C5", 500.0, result, 0.02, 0.35, 0.35);
    assert(!exp.fails);
    assert(approxEqual(exp.interactionRatio, 1.0));
    std::cout << "  testExactBoundaryIsNotFailing OK\n";
}

// Hand-calc: Vu=150kN, phiVn=120kN -> ratio=150/120=1.25.
// Suggested new spacing = currentSpacing/ratio = 200/1.25 = 160mm.
static void testBeamShearExplanationHandCalc() {
    auto exp = explainBeamShearResult("B7", 150.0, 120.0, 200.0);
    assert(exp.fails);
    assert(approxEqual(exp.demandCapacityRatio, 1.25));
    assert(exp.headline.find("B7") != std::string::npos);
    assert(exp.headline.find("shear") != std::string::npos);
    assert(!exp.suggestedFixes.empty());
    bool foundSpacingFix = false;
    for (const auto& fix : exp.suggestedFixes) if (fix.find("160") != std::string::npos) foundSpacingFix = true;
    assert(foundSpacingFix);
    std::cout << "  testBeamShearExplanationHandCalc (ratio=" << exp.demandCapacityRatio << ") OK\n";
}

static void testBeamShearPassingHasNoSuggestions() {
    auto exp = explainBeamShearResult("B2", 80.0, 120.0, 200.0);
    assert(!exp.fails);
    assert(exp.suggestedFixes.empty());
    std::cout << "  testBeamShearPassingHasNoSuggestions OK\n";
}

int main() {
    std::cout << "test_design_explainer:\n";
    testFailingColumnHandCalc();
    testPassingColumnHasNoSuggestions();
    testExactBoundaryIsNotFailing();
    testBeamShearExplanationHandCalc();
    testBeamShearPassingHasNoSuggestions();
    std::cout << "All tests passed.\n";
    return 0;
}
