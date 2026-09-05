#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "design/RCCShearWall.h"

using namespace nrsa::design;

static bool approxEqual(double a, double b, double relTol = 1e-3) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// Hand-calc: alphaC piecewise linear, ACI 318-19 Section 11.5.4.3.
static void testAlphaCHandCalc() {
    assert(approxEqual(inPlaneShearAlphaC(1.0), 3.0));   // squat limit
    assert(approxEqual(inPlaneShearAlphaC(1.5), 3.0));
    assert(approxEqual(inPlaneShearAlphaC(2.0), 2.0));
    assert(approxEqual(inPlaneShearAlphaC(3.0), 2.0));   // slender limit
    assert(approxEqual(inPlaneShearAlphaC(1.75), 2.5));  // midpoint, linear interp
    std::cout << "  testAlphaCHandCalc OK\n";
}

static void testMinReinforcementRatiosHandCalc() {
    assert(approxEqual(minHorizontalWallReinforcementRatio(12.0), 0.0020));
    assert(approxEqual(minHorizontalWallReinforcementRatio(20.0), 0.0025));
    assert(approxEqual(minVerticalWallReinforcementRatio(12.0), 0.0012));
    assert(approxEqual(minVerticalWallReinforcementRatio(20.0), 0.0015));
    std::cout << "  testMinReinforcementRatiosHandCalc OK\n";
}

// Hand-calc, ACI 318-19 Eq. 11.6.2, direct substitution:
// hwOverLw=1.0, rhoT=0.006, rhoLMinTable=0.0012 ->
// rhoL = 0.0025 + 0.5*(2.5-1.0)*(0.006-0.0025) = 0.005125
static void testVerticalCouplingFormulaHandCalc() {
    double result = verticalReinforcementRatioFromShear(0.006, 1.0, 0.0012);
    double expected = 0.0025 + 0.5 * (2.5 - 1.0) * (0.006 - 0.0025);
    assert(approxEqual(result, expected));
    assert(approxEqual(result, 0.005125, 1e-6));
    // hw/lw >= 2.5: no coupling, straight max(rhoT, table minimum).
    assert(approxEqual(verticalReinforcementRatioFromShear(0.006, 2.5, 0.0012), 0.006));
    assert(approxEqual(verticalReinforcementRatioFromShear(0.0005, 3.0, 0.0012), 0.0012));
    std::cout << "  testVerticalCouplingFormulaHandCalc OK\n";
}

// Round-trip: design rhoT for a given Vu (chosen large enough that the
// shear demand, not the code minimum, governs), then independently
// recompute Vn = Acv*(alphaC*sqrt(fc')+rhoT*fy) and verify phi*Vn
// reproduces Vu.
static void testInPlaneShearRoundTrip() {
    double vuKN = 2000.0, lwM = 1.0, tM = 0.15, hwM = 3.0, fc = 28.0, fy = 420.0, barDia = 12.0;
    auto result = designInPlaneShear(vuKN, lwM, tM, hwM, fc, fy, barDia);

    assert(!result.capGoverned);
    assert(result.rhoTRequired > minHorizontalWallReinforcementRatio(barDia));  // demand governs, not minimum

    double acvMm2 = tM * 1000.0 * lwM * 1000.0;
    assert(approxEqual(result.acvMm2, acvMm2));
    assert(approxEqual(result.alphaC, 2.0));  // hw/lw=3.0 -> slender limit

    double vnRecomputedN = acvMm2 * (result.alphaC * std::sqrt(fc) + result.rhoTUsed * fy);
    double phiVnRecomputed = 0.75 * vnRecomputedN / 1000.0;
    assert(approxEqual(phiVnRecomputed, vuKN, 1e-3));

    // hw/lw=3.0 >= 2.5 -> vertical ratio is a direct max(rhoT, table min), no coupling.
    assert(approxEqual(result.rhoLRequired, result.rhoTUsed, 1e-6));
    std::cout << "  testInPlaneShearRoundTrip (rhoT=" << result.rhoTUsed
              << ", phi*Vn recomputed=" << phiVnRecomputed << " kN, Vu=" << vuKN << " kN) OK\n";
}

// A demand exceeding the 8*Acv*sqrt(fc') cap must be flagged
// capGoverned=true, not silently return an enormous rhoT.
static void testInPlaneShearCapGoverned() {
    auto result = designInPlaneShear(50000.0, 1.0, 0.15, 3.0, 28.0, 420.0, 12.0);
    assert(result.capGoverned);
    assert(!result.adequate);
    std::cout << "  testInPlaneShearCapGoverned OK\n";
}

// Exact property (documented in the header): the distributed shears
// must sum to EXACTLY the input story shear, regardless of stiffness
// split -- this is a straight proportional allocation, not independent
// per-wall computation.
static void testStoryShearDistributionSumsExactly() {
    std::vector<WallSegment> walls = {{4.0, 0.3}, {2.0, 0.25}, {6.0, 0.3}};
    double storyShear = 1500.0;
    auto shares = distributeStoryShearByStiffness(walls, 3.2, storyShear);
    assert(shares.size() == 3);
    double sum = shares[0] + shares[1] + shares[2];
    assert(approxEqual(sum, storyShear, 1e-9));

    // Two IDENTICAL walls must get exactly equal shares.
    std::vector<WallSegment> equalWalls = {{5.0, 0.3}, {5.0, 0.3}};
    auto equalShares = distributeStoryShearByStiffness(equalWalls, 3.0, 1000.0);
    assert(approxEqual(equalShares[0], equalShares[1], 1e-9));
    assert(approxEqual(equalShares[0], 500.0, 1e-6));

    // A wall with 2x the length has I proportional to length^3 -> 8x
    // the stiffness -> should take exactly 8x the share of an
    // otherwise-identical half-length wall.
    std::vector<WallSegment> mixedWalls = {{4.0, 0.3}, {2.0, 0.3}};
    auto mixedShares = distributeStoryShearByStiffness(mixedWalls, 3.0, 900.0);
    assert(approxEqual(mixedShares[0] / mixedShares[1], 8.0, 1e-6));
    std::cout << "  testStoryShearDistributionSumsExactly OK\n";
}

// Round-trip: design As for a given Mu (chosen so demand, not the
// minimum, governs), then independently recompute phi*Mn from that As
// and verify it reproduces Mu.
static void testOutOfPlaneFlexureRoundTrip() {
    double muKNmPerM = 25.0, tMm = 200.0, coverMm = 40.0, barDiaMm = 12.0, fc = 28.0, fy = 420.0;
    auto result = designOutOfPlaneFlexure(muKNmPerM, tMm, coverMm, barDiaMm, fc, fy);
    assert(!result.governedByMinimum);
    assert(!result.exceedsMaximum);

    double bMm = 1000.0, dMm = tMm - coverMm - barDiaMm / 2.0;
    double a = (result.asRequiredMm2PerM * fy) / (0.85 * fc * bMm);
    double phiMnNmm = 0.9 * result.asRequiredMm2PerM * fy * (dMm - a / 2.0);
    double phiMnKNm = phiMnNmm / 1.0e6;
    assert(approxEqual(phiMnKNm, muKNmPerM, 1e-3));
    std::cout << "  testOutOfPlaneFlexureRoundTrip (As=" << result.asRequiredMm2PerM
              << " mm2/m, phi*Mn recomputed=" << phiMnKNm << " kNm, Mu=" << muKNmPerM << " kNm) OK\n";
}

static void testOutOfPlaneFlexureZeroMomentGovernedByMinimum() {
    auto result = designOutOfPlaneFlexure(0.0, 200.0, 40.0, 12.0, 28.0, 420.0);
    assert(result.governedByMinimum);
    double expectedAsMin = 0.0012 * 1000.0 * 200.0;  // barDia<=16 -> 0.0012
    assert(approxEqual(result.asRequiredMm2PerM, expectedAsMin, 1e-6));
    std::cout << "  testOutOfPlaneFlexureZeroMomentGovernedByMinimum OK\n";
}

static void testRejectsInvalidInputs() {
    bool threw = false;
    try { designInPlaneShear(-100.0, 1.0, 0.15, 3.0, 28.0, 420.0, 12.0); }
    catch (const std::invalid_argument&) { threw = true; }
    assert(threw);

    threw = false;
    try { distributeStoryShearByStiffness({{4.0, 0.3}}, -1.0, 500.0); }
    catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  testRejectsInvalidInputs OK\n";
}

int main() {
    std::cout << "test_rcc_shearwall:\n";
    testAlphaCHandCalc();
    testMinReinforcementRatiosHandCalc();
    testVerticalCouplingFormulaHandCalc();
    testInPlaneShearRoundTrip();
    testInPlaneShearCapGoverned();
    testStoryShearDistributionSumsExactly();
    testOutOfPlaneFlexureRoundTrip();
    testOutOfPlaneFlexureZeroMomentGovernedByMinimum();
    testRejectsInvalidInputs();
    std::cout << "All tests passed.\n";
    return 0;
}
