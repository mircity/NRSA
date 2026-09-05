#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "design/RCCSlab.h"

using namespace nrsa::design;

static bool approxEqual(double a, double b, double relTol = 1e-3) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// Hand-calc: ACI 318-19 24.4.3.2 shrinkage-temperature ratio bands.
static void testShrinkageTemperatureRatioHandCalc() {
    assert(approxEqual(shrinkageTemperatureSteelRatio(280.0), 0.0020));
    assert(approxEqual(shrinkageTemperatureSteelRatio(420.0), 0.0018));
    double expected500 = std::max(0.0014, 0.0018 * 420.0 / 500.0);
    assert(approxEqual(shrinkageTemperatureSteelRatio(500.0), expected500));
    std::cout << "  testShrinkageTemperatureRatioHandCalc OK\n";
}

// Round-trip: design As for a given Mu (large enough that demand, not
// the shrinkage-temperature minimum, governs), then independently
// recompute phi*Mn and verify it reproduces Mu.
static void testOneWaySlabFlexureRoundTrip() {
    double muKNmPerM = 20.0, hMm = 180.0, coverMm = 25.0, barDiaMm = 12.0, fc = 28.0, fy = 420.0;
    auto result = designOneWaySlab(muKNmPerM, hMm, coverMm, barDiaMm, fc, fy);
    assert(!result.governedByMinimum);
    assert(!result.exceedsMaximum);

    double bMm = 1000.0, dMm = hMm - coverMm - barDiaMm / 2.0;
    double a = (result.asRequiredMm2PerM * fy) / (0.85 * fc * bMm);
    double phiMnNmm = 0.9 * result.asRequiredMm2PerM * fy * (dMm - a / 2.0);
    double phiMnKNm = phiMnNmm / 1.0e6;
    assert(approxEqual(phiMnKNm, muKNmPerM, 1e-3));

    // ACI 318-19 24.3.2 max spacing: lesser of 3h=540mm or 450mm -> 450mm.
    assert(approxEqual(result.maxBarSpacingMm, 450.0));
    std::cout << "  testOneWaySlabFlexureRoundTrip (As=" << result.asRequiredMm2PerM
              << " mm2/m, phi*Mn recomputed=" << phiMnKNm << " kNm) OK\n";
}

static void testOneWaySlabZeroMomentGovernedByMinimum() {
    auto result = designOneWaySlab(0.0, 180.0, 25.0, 12.0, 28.0, 420.0);
    assert(result.governedByMinimum);
    double expectedAsMin = 0.0018 * 180.0 * 1000.0;  // fy=420 band
    assert(approxEqual(result.asRequiredMm2PerM, expectedAsMin, 1e-6));
    std::cout << "  testOneWaySlabZeroMomentGovernedByMinimum OK\n";
}

// Hand-calc: Vc = 0.17*sqrt(fc')*b*d, ACI 318-19 Eq. 22.5.5.1.
static void testOneWayShearHandCalc() {
    double hMm = 180.0, coverMm = 25.0, barDiaMm = 12.0, fc = 28.0;
    double dMm = hMm - coverMm - barDiaMm / 2.0;
    double expectedVcKN = 0.17 * std::sqrt(fc) * 1000.0 * dMm / 1000.0;
    auto result = checkOneWayShear(50.0, hMm, coverMm, barDiaMm, fc);
    assert(approxEqual(result.vcKN, expectedVcKN));
    assert(approxEqual(result.phiVcKN, 0.75 * expectedVcKN));
    std::cout << "  testOneWayShearHandCalc (Vc=" << expectedVcKN << " kN/m) OK\n";
}

// Hand-calc, Rankine method, BOTH directions simply supported (zero
// negative moment, positive moment = w*L^2/8 exactly):
// shortSpan=4m, longSpan=6m, wu=10 kN/m^2.
//   La^4=256, Lb^4=1296, sum=1552
//   shortLoad = wu*Lb^4/sum = 10*1296/1552 = 8.35052 kN/m^2
//   longLoad  = wu*La^4/sum = 10*256/1552  = 1.64948 kN/m^2
//   (conservation: shortLoad+longLoad must equal wu exactly)
//   shortSpanPosMoment = shortLoad*La^2/8 = 8.35052*16/8 = 16.7010 kNm/m
//   longSpanPosMoment  = longLoad*Lb^2/8  = 1.64948*36/8 = 7.42268 kNm/m
static void testTwoWaySlabBothSimpleHandCalc() {
    double La = 4.0, Lb = 6.0, wu = 10.0;
    auto result = designTwoWaySlabPanel(La, Lb, wu, 180.0, 25.0, 12.0,
                                         EdgeContinuity::BothSimple, EdgeContinuity::BothSimple,
                                         28.0, 420.0);

    double La4 = 256.0, Lb4 = 1296.0, sum = La4 + Lb4;
    double expectedShortLoad = wu * Lb4 / sum;
    double expectedLongLoad = wu * La4 / sum;
    assert(approxEqual(result.shortSpanLoadKNPerM2, expectedShortLoad));
    assert(approxEqual(result.longSpanLoadKNPerM2, expectedLongLoad));
    assert(approxEqual(result.shortSpanLoadKNPerM2 + result.longSpanLoadKNPerM2, wu, 1e-9));

    double expectedShortPosMoment = expectedShortLoad * La * La / 8.0;
    double expectedLongPosMoment = expectedLongLoad * Lb * Lb / 8.0;
    assert(approxEqual(result.shortSpanPosMomentKNm, expectedShortPosMoment));
    assert(approxEqual(result.longSpanPosMomentKNm, expectedLongPosMoment));

    // Both simple -> zero negative moment in both directions.
    assert(approxEqual(result.shortSpanNegMomentKNm, 0.0, 1e-9));
    assert(approxEqual(result.longSpanNegMomentKNm, 0.0, 1e-9));

    std::cout << "  testTwoWaySlabBothSimpleHandCalc (shortM=" << result.shortSpanPosMomentKNm
              << " kNm/m, longM=" << result.longSpanPosMomentKNm << " kNm/m) OK\n";
}

static void testTwoWaySlabRejectsSwappedSpans() {
    bool threw = false;
    try {
        designTwoWaySlabPanel(6.0, 4.0, 10.0, 180.0, 25.0, 12.0, EdgeContinuity::BothSimple,
                               EdgeContinuity::BothSimple, 28.0, 420.0);
    } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  testTwoWaySlabRejectsSwappedSpans OK\n";
}

static void testRejectsInvalidInputs() {
    bool threw = false;
    try { designOneWaySlab(-1.0, 180.0, 25.0, 12.0, 28.0, 420.0); }
    catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  testRejectsInvalidInputs OK\n";
}

int main() {
    std::cout << "test_rcc_slab:\n";
    testShrinkageTemperatureRatioHandCalc();
    testOneWaySlabFlexureRoundTrip();
    testOneWaySlabZeroMomentGovernedByMinimum();
    testOneWayShearHandCalc();
    testTwoWaySlabBothSimpleHandCalc();
    testTwoWaySlabRejectsSwappedSpans();
    testRejectsInvalidInputs();
    std::cout << "All tests passed.\n";
    return 0;
}
