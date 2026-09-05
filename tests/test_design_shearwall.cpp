#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "design/RCCShearWall.h"

using namespace nrsa::design;

static bool approxEqual(double a, double b, double relTol = 1e-3) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// ---------------------------------------------------------------------------
// alphaC: closed-form checks at and between the two ACI 318-19 Section
// 11.5.4.3 anchor points.
// ---------------------------------------------------------------------------

static void testAlphaCAtLimitsAndMidpoint() {
    assert(approxEqual(inPlaneShearAlphaC(1.0), 3.0));
    assert(approxEqual(inPlaneShearAlphaC(1.5), 3.0));
    assert(approxEqual(inPlaneShearAlphaC(2.0), 2.0));
    assert(approxEqual(inPlaneShearAlphaC(3.0), 2.0));
    assert(approxEqual(inPlaneShearAlphaC(1.75), 2.5));  // exact midpoint, linear interpolation
    std::cout << "  testAlphaCAtLimitsAndMidpoint OK\n";
}

// ---------------------------------------------------------------------------
// designInPlaneShear: round-trip check -- design a rhoT for a given Vu,
// then independently recompute phi*Vn from that rhoT and confirm it
// reproduces Vu (the same round-trip strategy RCCBeam/RCCColumn use).
// ---------------------------------------------------------------------------

static void testShearDesignRoundTrip() {
    // 3m long, 250mm thick wall, 3m story height (hw/lw = 1.0, squat -> alphaC=3.0),
    // moderate demand comfortably above the minimum-reinforcement floor.
    double lwM = 3.0, tM = 0.25, hwM = 3.0, fc = 28.0, fy = 414.0, barDia = 12.0;
    double acvMm2 = tM * 1000.0 * lwM * 1000.0;
    double alphaC = inPlaneShearAlphaC(hwM / lwM);
    // Pick a Vu deliberately above what alphaC*sqrt(fc) alone (i.e. rhoT=0)
    // would give at phi=0.75, so the design is governed by demand, not the
    // Section 11.6.1 minimum -- otherwise the round-trip would just be
    // checking the minimum, not the solved-for rhoT path.
    double vnAtZeroRho = acvMm2 * alphaC * std::sqrt(fc) / 1000.0;
    double vuKN = 0.75 * vnAtZeroRho * 1.5;  // comfortably past phi*Vn(rhoT=0)

    auto result = designInPlaneShear(vuKN, lwM, tM, hwM, fc, fy, barDia);
    assert(!result.capGoverned);
    assert(result.rhoTUsed > minHorizontalWallReinforcementRatio(barDia) - 1e-9);

    // Independent recomputation from the reported rhoTUsed.
    double vnCheckN = acvMm2 * (result.alphaC * std::sqrt(fc) + result.rhoTUsed * fy);
    double phiVnCheck = 0.75 * vnCheckN / 1000.0;
    assert(approxEqual(phiVnCheck, result.phiVnKN, 1e-6));
    assert(result.adequate);
    assert(approxEqual(phiVnCheck, vuKN, 1e-2));  // demand-governed -> should reproduce Vu almost exactly
    std::cout << "  testShearDesignRoundTrip OK\n";
}

static void testShearDesignGovernedByMinimum() {
    // Negligible Vu -- the Section 11.6.1 minimum should govern outright.
    auto result = designInPlaneShear(1.0, 4.0, 0.3, 3.0, 28.0, 414.0, 12.0);
    assert(approxEqual(result.rhoTUsed, minHorizontalWallReinforcementRatio(12.0)));
    assert(result.adequate);
    std::cout << "  testShearDesignGovernedByMinimum OK\n";
}

static void testShearDesignCapGoverned() {
    // Absurdly large Vu on a thin wall -- must exceed the 8*Acv*sqrt(fc') cap
    // and be flagged as inadequate regardless of reinforcement.
    auto result = designInPlaneShear(50000.0, 2.0, 0.15, 3.0, 28.0, 414.0, 12.0);
    assert(result.capGoverned);
    assert(!result.adequate);
    std::cout << "  testShearDesignCapGoverned OK\n";
}

// ---------------------------------------------------------------------------
// Section 11.6.2 vertical-steel coupling: closed-form checks against the
// code equation directly, independent of designInPlaneShear.
// ---------------------------------------------------------------------------

static void testVerticalRatioCouplingClosedForm() {
    double rhoT = 0.0040;
    double rhoLMinTable = 0.0012;
    // hw/lw = 2.5 or above: rho_l = rho_t directly.
    assert(approxEqual(verticalReinforcementRatioFromShear(rhoT, 2.5, rhoLMinTable), rhoT));
    assert(approxEqual(verticalReinforcementRatioFromShear(rhoT, 3.0, rhoLMinTable), rhoT));
    // hw/lw = 0.5 (very squat): rho_l = 0.0025 + 0.5*(2.5-0.5)*(0.0040-0.0025) = 0.0040
    double expected = 0.0025 + 0.5 * (2.5 - 0.5) * (rhoT - 0.0025);
    assert(approxEqual(verticalReinforcementRatioFromShear(rhoT, 0.5, rhoLMinTable), expected));
    // Never below the Table 11.6.1 vertical minimum even when the formula
    // would (numerically) dip under it for rhoT at or near 0.0025.
    assert(verticalReinforcementRatioFromShear(0.0025, 1.0, 0.0015) >= 0.0015 - 1e-12);
    std::cout << "  testVerticalRatioCouplingClosedForm OK\n";
}

// ---------------------------------------------------------------------------
// distributeStoryShearByStiffness: conservation check (shares sum back to
// the total) and a symmetry check (two identical walls split the shear
// evenly), independent of any single wall's absolute stiffness value.
// ---------------------------------------------------------------------------

static void testDistributionConservesTotalShear() {
    std::vector<WallSegment> walls = {{4.0, 0.25}, {2.5, 0.30}, {6.0, 0.20}};
    double storyShear = 850.0;
    auto shares = distributeStoryShearByStiffness(walls, 3.0, storyShear);
    assert(shares.size() == 3);
    double sum = shares[0] + shares[1] + shares[2];
    assert(approxEqual(sum, storyShear, 1e-9));
    std::cout << "  testDistributionConservesTotalShear OK\n";
}

static void testDistributionSymmetricWallsSplitEvenly() {
    std::vector<WallSegment> walls = {{3.0, 0.25}, {3.0, 0.25}};
    auto shares = distributeStoryShearByStiffness(walls, 3.0, 400.0);
    assert(approxEqual(shares[0], 200.0));
    assert(approxEqual(shares[1], 200.0));
    std::cout << "  testDistributionSymmetricWallsSplitEvenly OK\n";
}

static void testDistributionEmptyInputReturnsEmpty() {
    auto shares = distributeStoryShearByStiffness({}, 3.0, 400.0);
    assert(shares.empty());
    std::cout << "  testDistributionEmptyInputReturnsEmpty OK\n";
}

// ---------------------------------------------------------------------------
// designOutOfPlaneFlexure: round-trip check, same strategy as
// RCCBeam::designFlexure's own test -- design As for a given Mu, then
// independently recompute phi*Mn from that As and confirm it reproduces Mu.
// ---------------------------------------------------------------------------

static void testOutOfPlaneRoundTrip() {
    double tMm = 250.0, coverMm = 40.0, barDia = 16.0, fc = 28.0, fy = 414.0;
    double dMm = tMm - coverMm - barDia / 2.0;
    // Pick Mu comfortably above the minimum so the design is demand-governed.
    double asMin = minVerticalWallReinforcementRatio(barDia) * 1000.0 * tMm;
    double phi = 0.9;
    double a_forMin = asMin * fy / (0.85 * fc * 1000.0);
    double muAtMinNmm = phi * asMin * fy * (dMm - a_forMin / 2.0);
    double muKNmPerM = 2.0 * muAtMinNmm / 1.0e6;

    auto result = designOutOfPlaneFlexure(muKNmPerM, tMm, coverMm, barDia, fc, fy);
    assert(!result.governedByMinimum);

    double a = result.asRequiredMm2PerM * fy / (0.85 * fc * 1000.0);
    double phiMnCheckNmm = phi * result.asRequiredMm2PerM * fy * (dMm - a / 2.0);
    double phiMnCheckKNm = phiMnCheckNmm / 1.0e6;
    assert(approxEqual(phiMnCheckKNm, muKNmPerM, 1e-3));
    std::cout << "  testOutOfPlaneRoundTrip OK\n";
}

static void testOutOfPlaneGovernedByMinimum() {
    auto result = designOutOfPlaneFlexure(0.0, 250.0, 40.0, 16.0, 28.0, 414.0);
    assert(result.governedByMinimum);
    assert(approxEqual(result.asRequiredMm2PerM, result.asMinMm2PerM));
    std::cout << "  testOutOfPlaneGovernedByMinimum OK\n";
}

static void testRejectsNegativeShearDemand() {
    bool threw = false;
    try {
        designInPlaneShear(-1.0, 3.0, 0.25, 3.0, 28.0, 414.0, 12.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testRejectsNegativeShearDemand OK\n";
}

int main() {
    std::cout << "Running RCCShearWall design tests...\n";
    testAlphaCAtLimitsAndMidpoint();
    testShearDesignRoundTrip();
    testShearDesignGovernedByMinimum();
    testShearDesignCapGoverned();
    testVerticalRatioCouplingClosedForm();
    testDistributionConservesTotalShear();
    testDistributionSymmetricWallsSplitEvenly();
    testDistributionEmptyInputReturnsEmpty();
    testOutOfPlaneRoundTrip();
    testOutOfPlaneGovernedByMinimum();
    testRejectsNegativeShearDemand();
    std::cout << "All RCCShearWall tests passed.\n";
    return 0;
}
