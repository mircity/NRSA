#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "design/RCCSlab.h"

using namespace nrsa::design;

static bool approxEqual(double a, double b, double relTol = 1e-3) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// Independently recompute phi*Mn from a given As via the standard
// Whitney stress-block formula, for a 1-meter design strip -- the same
// round-trip validation strategy RCCBeam's own tests use.
static double phiMnFromAsKNm(double asMm2PerM, double hMm, double coverMm, double barDiaMm,
                              double fcMPa, double fyMPa) {
    double dMm = hMm - coverMm - barDiaMm / 2.0;
    double bMm = 1000.0;
    double aMm = asMm2PerM * fyMPa / (0.85 * fcMPa * bMm);
    double mnNmm = asMm2PerM * fyMPa * (dMm - aMm / 2.0);
    return 0.9 * mnNmm / 1.0e6;
}

// ---- shrinkageTemperatureSteelRatio ----------------------------------

static void testShrinkageTempRatioBands() {
    assert(approxEqual(shrinkageTemperatureSteelRatio(280.0), 0.0020));
    assert(approxEqual(shrinkageTemperatureSteelRatio(300.0), 0.0020));
    assert(approxEqual(shrinkageTemperatureSteelRatio(420.0), 0.0018));
    // Above 420 MPa, ratio scales down as 0.0018*420/fy, floored at 0.0014.
    double expected = 0.0018 * 420.0 / 500.0;
    assert(approxEqual(shrinkageTemperatureSteelRatio(500.0), expected));
    assert(shrinkageTemperatureSteelRatio(5000.0) >= 0.0014 - 1e-9);
    std::cout << "  testShrinkageTempRatioBands OK\n";
}

// ---- designOneWaySlab --------------------------------------------------

static void testOneWaySlabRoundTripAtModerateMoment() {
    // A moment comfortably above the shrinkage-temperature minimum, so
    // the flexural demand itself governs -- design As, then independently
    // recompute phi*Mn from that As and confirm it reproduces Mu, exactly
    // the same round-trip strategy RCCBeam's tests use.
    double hMm = 150.0, coverMm = 20.0, barDiaMm = 12.0, fcMPa = 25.0, fyMPa = 420.0;
    double muKNmPerM = 25.0;
    auto result = designOneWaySlab(muKNmPerM, hMm, coverMm, barDiaMm, fcMPa, fyMPa);
    assert(!result.governedByMinimum);
    assert(!result.exceedsMaximum);
    double phiMn = phiMnFromAsKNm(result.asRequiredMm2PerM, hMm, coverMm, barDiaMm, fcMPa, fyMPa);
    assert(approxEqual(phiMn, muKNmPerM, 1e-2));
    std::cout << "  testOneWaySlabRoundTripAtModerateMoment OK\n";
}

static void testOneWaySlabGovernedByMinimumAtLowMoment() {
    double hMm = 150.0, coverMm = 20.0, barDiaMm = 12.0, fcMPa = 25.0, fyMPa = 420.0;
    auto result = designOneWaySlab(0.5, hMm, coverMm, barDiaMm, fcMPa, fyMPa);
    assert(result.governedByMinimum);
    assert(approxEqual(result.asRequiredMm2PerM, result.asMinMm2PerM));
    // Gross-area basis: As,min = 0.0018 * h * b for Grade 420 (b=1000mm/m).
    assert(approxEqual(result.asMinMm2PerM, 0.0018 * hMm * 1000.0));
    std::cout << "  testOneWaySlabGovernedByMinimumAtLowMoment OK\n";
}

static void testOneWaySlabZeroMomentGovernedByMinimum() {
    auto result = designOneWaySlab(0.0, 150.0, 20.0, 12.0, 25.0, 420.0);
    assert(result.governedByMinimum);
    assert(result.asRequiredMm2PerM > 0.0);
    std::cout << "  testOneWaySlabZeroMomentGovernedByMinimum OK\n";
}

static void testOneWaySlabMaxSpacingCap() {
    // ACI 318-19 24.3.2: lesser of 3h and 450mm.
    auto thin = designOneWaySlab(1.0, 120.0, 20.0, 10.0, 25.0, 420.0);
    assert(approxEqual(thin.maxBarSpacingMm, 3.0 * 120.0));
    auto thick = designOneWaySlab(1.0, 250.0, 20.0, 10.0, 25.0, 420.0);
    assert(approxEqual(thick.maxBarSpacingMm, 450.0));
    std::cout << "  testOneWaySlabMaxSpacingCap OK\n";
}

static void testOneWaySlabThrowsWhenMomentExceedsCapacity() {
    bool threw = false;
    try {
        // A large moment on a thin slab -- exceeds the balanced-section
        // limit for a singly reinforced section entirely.
        designOneWaySlab(500.0, 100.0, 20.0, 10.0, 21.0, 420.0);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testOneWaySlabThrowsWhenMomentExceedsCapacity OK\n";
}

// ---- checkOneWayShear ----------------------------------------------------

static void testOneWayShearAdequateAndInadequate() {
    double hMm = 150.0, coverMm = 20.0, barDiaMm = 12.0, fcMPa = 25.0;
    auto low = checkOneWayShear(20.0, hMm, coverMm, barDiaMm, fcMPa);
    assert(low.adequate);
    auto high = checkOneWayShear(500.0, hMm, coverMm, barDiaMm, fcMPa);
    assert(!high.adequate);
    assert(high.phiVcKN < 500.0);
    std::cout << "  testOneWayShearAdequateAndInadequate OK\n";
}

// ---- designTwoWaySlabPanel ------------------------------------------------

static void testTwoWayLoadSplitConservesTotal() {
    double wu = 12.0;
    auto result = designTwoWaySlabPanel(4.0, 5.0, wu, 150.0, 20.0, 10.0,
                                         EdgeContinuity::BothContinuous, EdgeContinuity::BothContinuous,
                                         25.0, 420.0);
    // Rankine split allocates the whole load between the two directions.
    assert(approxEqual(result.shortSpanLoadKNPerM2 + result.longSpanLoadKNPerM2, wu));
    // Short (stiffer) direction should carry the larger share.
    assert(result.shortSpanLoadKNPerM2 > result.longSpanLoadKNPerM2);
    std::cout << "  testTwoWayLoadSplitConservesTotal OK\n";
}

static void testTwoWaySquarePanelSplitsEvenly() {
    double wu = 10.0;
    auto result = designTwoWaySlabPanel(4.0, 4.0, wu, 150.0, 20.0, 10.0,
                                         EdgeContinuity::BothSimple, EdgeContinuity::BothSimple,
                                         25.0, 420.0);
    assert(approxEqual(result.shortSpanLoadKNPerM2, wu / 2.0));
    assert(approxEqual(result.longSpanLoadKNPerM2, wu / 2.0));
    assert(approxEqual(result.shortSpanPosMomentKNm, result.longSpanPosMomentKNm));
    std::cout << "  testTwoWaySquarePanelSplitsEvenly OK\n";
}

static void testTwoWaySimplySupportedHasZeroNegativeMoment() {
    auto result = designTwoWaySlabPanel(3.5, 5.5, 10.0, 150.0, 20.0, 10.0,
                                         EdgeContinuity::BothSimple, EdgeContinuity::BothSimple,
                                         25.0, 420.0);
    assert(approxEqual(result.shortSpanNegMomentKNm, 0.0, 1e-6));
    assert(approxEqual(result.longSpanNegMomentKNm, 0.0, 1e-6));
    assert(result.shortSpanPosMomentKNm > 0.0);
    std::cout << "  testTwoWaySimplySupportedHasZeroNegativeMoment OK\n";
}

static void testTwoWayThrowsWhenSpansSwapped() {
    bool threw = false;
    try {
        designTwoWaySlabPanel(5.0, 4.0, 10.0, 150.0, 20.0, 10.0,
                               EdgeContinuity::BothSimple, EdgeContinuity::BothSimple, 25.0, 420.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testTwoWayThrowsWhenSpansSwapped OK\n";
}

int main() {
    std::cout << "RCCSlab tests:\n";
    testShrinkageTempRatioBands();
    testOneWaySlabRoundTripAtModerateMoment();
    testOneWaySlabGovernedByMinimumAtLowMoment();
    testOneWaySlabZeroMomentGovernedByMinimum();
    testOneWaySlabMaxSpacingCap();
    testOneWaySlabThrowsWhenMomentExceedsCapacity();
    testOneWayShearAdequateAndInadequate();
    testTwoWayLoadSplitConservesTotal();
    testTwoWaySquarePanelSplitsEvenly();
    testTwoWaySimplySupportedHasZeroNegativeMoment();
    testTwoWayThrowsWhenSpansSwapped();
    std::cout << "All RCCSlab tests passed.\n";
    return 0;
}
