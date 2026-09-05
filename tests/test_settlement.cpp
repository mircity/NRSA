#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "design/Settlement.h"

using namespace nrsa::design;

static bool approxEqual(double a, double b, double relTol = 1e-6) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

static void testBowlesModulusHandCalc() {
    // Es = 400*(N+6): N=18 -> 400*24 = 9600 kPa
    assert(approxEqual(soilModulusFromSptN_Bowles1997(18.0), 9600.0));
    assert(approxEqual(soilModulusFromSptN_Bowles1997(0.0), 2400.0));
    std::cout << "  testBowlesModulusHandCalc OK\n";
}

static void testIzPeakHandCalc() {
    // IzPeak = 0.5 + 0.1*sqrt(netQ/sigmaVoDf): netQ=200, sigmaVoDf=50 -> ratio=4 -> sqrt=2 -> 0.5+0.2=0.7
    assert(approxEqual(schmertmannIzPeak(200.0, 50.0), 0.7));
    // Clamp: sigmaVoDf below 1 kPa clamped to 1.
    assert(approxEqual(schmertmannIzPeak(1.0, 0.0), schmertmannIzPeak(1.0, 1.0)));
    std::cout << "  testIzPeakHandCalc OK\n";
}

static void testC1HandCalc() {
    // C1 = 1 - 0.5*(sigmaVoDf/netQ): netQ=200, sigmaVoDf=50 -> 1-0.5*0.25=0.875
    assert(approxEqual(schmertmannDepthCorrectionC1(200.0, 50.0), 0.875));
    // Clamp to 0.5 minimum when overburden is large relative to netQ.
    assert(approxEqual(schmertmannDepthCorrectionC1(100.0, 500.0), 0.5));
    std::cout << "  testC1HandCalc OK\n";
}

static void testStrainInfluenceFactorShapeHandCalc() {
    double B = 2.0, izPeak = 0.6;
    assert(approxEqual(schmertmannStrainInfluenceFactor(0.0, B, izPeak), 0.1));
    assert(approxEqual(schmertmannStrainInfluenceFactor(B / 2.0, B, izPeak), izPeak));
    assert(approxEqual(schmertmannStrainInfluenceFactor(2.0 * B, B, izPeak), 0.0));
    // Midpoint of the rising segment: z=B/4 -> halfway between 0.1 and izPeak.
    assert(approxEqual(schmertmannStrainInfluenceFactor(B / 4.0, B, izPeak), (0.1 + izPeak) / 2.0));
    std::cout << "  testStrainInfluenceFactorShapeHandCalc OK\n";
}

// Closed-form check: for UNIFORM soil (constant N -> constant Es) over
// the full 0..2B influence zone, the area under the piecewise-linear Iz
// diagram is exactly B*(0.025 + izPeak) (derived by hand: trapezoid
// from (0,0.1) to (B/2,izPeak) plus triangle from (B/2,izPeak) to
// (2B,0)). Splitting the profile into 4 equal layers of thickness B/2
// each puts a layer boundary EXACTLY at the z=B/2 kink, so the
// midpoint-rule evaluation used internally is exact for each layer
// (each lies entirely on one linear side of the kink) -- the numerical
// result should match the closed form to floating-point precision, not
// just approximately.
static void testUniformSoilClosedForm() {
    double B = 2.0;
    double N = 15.0;
    double netQ = 150.0;
    double sigmaVoDf = 40.0;

    double izPeak = schmertmannIzPeak(netQ, sigmaVoDf);
    double c1 = schmertmannDepthCorrectionC1(netQ, sigmaVoDf);
    double es = soilModulusFromSptN_Bowles1997(N);

    double areaClosedForm = B * (0.025 + izPeak);
    double expectedSettlement = c1 * netQ * areaClosedForm / es;

    std::vector<SoilLayer> layers = {
        {B / 2.0, N}, {B / 2.0, N}, {B / 2.0, N}, {B / 2.0, N}};
    auto result = runSchmertmannSettlement(B, netQ, sigmaVoDf, layers);

    assert(approxEqual(result.settlementM, expectedSettlement, 1e-9));
    assert(approxEqual(result.izPeak, izPeak));
    assert(approxEqual(result.depthCorrectionC1, c1));

    // Sanity: sum of per-layer contributions must equal the total.
    double sumContrib = 0.0;
    for (double c : result.layerContributionsM) sumContrib += c;
    assert(approxEqual(sumContrib, result.settlementM, 1e-9));

    std::cout << "  testUniformSoilClosedForm (expected=" << expectedSettlement
              << " m, got=" << result.settlementM << " m) OK\n";
}

static void testRejectsInsufficientLayerCoverage() {
    std::vector<SoilLayer> layers = {{0.5, 15.0}};  // only 0.5m, need 2B=4m for B=2
    bool threw = false;
    try {
        runSchmertmannSettlement(2.0, 150.0, 40.0, layers);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testRejectsInsufficientLayerCoverage OK\n";
}

static void testLayeredProfileMatchesSplitEquivalent() {
    // A single thick uniform layer split into two thinner layers of the
    // same soil must give the identical total settlement (splitting a
    // uniform layer changes nothing physically).
    double B = 3.0, N = 20.0, netQ = 180.0, sigmaVoDf = 60.0;
    std::vector<SoilLayer> oneLayer = {{2.0 * B, N}};
    std::vector<SoilLayer> twoLayers = {{B, N}, {B, N}};
    auto r1 = runSchmertmannSettlement(B, netQ, sigmaVoDf, oneLayer);
    auto r2 = runSchmertmannSettlement(B, netQ, sigmaVoDf, twoLayers);
    // Not expected to be bit-identical (midpoint rule differs across the
    // kink for the one-layer case vs. exact split for two), but should
    // be very close since Iz is smooth away from the kink.
    assert(approxEqual(r1.settlementM, r2.settlementM, 0.05));
    std::cout << "  testLayeredProfileMatchesSplitEquivalent OK\n";
}

int main() {
    std::cout << "test_settlement:\n";
    testBowlesModulusHandCalc();
    testIzPeakHandCalc();
    testC1HandCalc();
    testStrainInfluenceFactorShapeHandCalc();
    testUniformSoilClosedForm();
    testRejectsInsufficientLayerCoverage();
    testLayeredProfileMatchesSplitEquivalent();
    std::cout << "All tests passed.\n";
    return 0;
}
