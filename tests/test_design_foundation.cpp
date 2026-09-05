#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "design/RCCFoundation.h"

using namespace nrsa::design;

static bool approxEqual(double a, double b, double relTol = 1e-3) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// Independently recompute phi*Mn from a given As via the standard Whitney
// stress-block formula for a section of width bM (meters) -- the same
// round-trip validation strategy RCCBeam's/RCCSlab's own tests use.
static double phiMnFromAsKNm(double asMm2, double bM, double dM, double fcMPa, double fyMPa) {
    double bMm = bM * 1000.0, dMm = dM * 1000.0;
    double aMm = asMm2 * fyMPa / (0.85 * fcMPa * bMm);
    double mnNmm = asMm2 * fyMPa * (dMm - aMm / 2.0);
    return 0.9 * mnNmm / 1.0e6;
}

// ---------------------------------------------------------------------------
// sizeSquareFootingArea: plain hand-calculated closed form.
// ---------------------------------------------------------------------------

static void testSizeSquareFootingAreaHandCalc() {
    // 1200 kN service load, 150 kPa net allowable bearing -> 8 m^2, side = sqrt(8).
    auto result = sizeSquareFootingArea(1200.0, 150.0);
    assert(approxEqual(result.areaRequiredM2, 8.0));
    assert(approxEqual(result.sideM, std::sqrt(8.0)));
    std::cout << "  testSizeSquareFootingAreaHandCalc OK\n";
}

static void testSizeSquareFootingAreaRejectsNonPositive() {
    bool threw1 = false, threw2 = false;
    try {
        sizeSquareFootingArea(0.0, 150.0);
    } catch (const std::invalid_argument&) {
        threw1 = true;
    }
    try {
        sizeSquareFootingArea(1000.0, -1.0);
    } catch (const std::invalid_argument&) {
        threw2 = true;
    }
    assert(threw1 && threw2);
    std::cout << "  testSizeSquareFootingAreaRejectsNonPositive OK\n";
}

// ---------------------------------------------------------------------------
// checkTwoWayShear: independent closed-form recomputation of the governing
// ACI 318-19 22.6.5.2 coefficient for a SQUARE column (beta=1, so the
// beta-dependent formula and the flat 0.33 cap are the only two candidates).
// ---------------------------------------------------------------------------

static void testTwoWayShearMatchesHandCalcForSquareColumn() {
    double colBMm = 400.0, colHMm = 400.0, dMm = 350.0, fcMPa = 25.0;
    double b0Mm = 2.0 * (colBMm + dMm) + 2.0 * (colHMm + dMm);  // 2*(750)+2*(750) = 3000 mm

    // beta=1 -> vc1 formula: 0.17*(1+2/1)*sqrt(fc)*b0*d = 0.51*sqrt(fc)*b0*d
    double vc1N = 0.51 * std::sqrt(fcMPa) * b0Mm * dMm;
    double alphaS = 40.0;
    double vc2N = 0.083 * (alphaS * dMm / b0Mm + 2.0) * std::sqrt(fcMPa) * b0Mm * dMm;
    double vc3N = 0.33 * std::sqrt(fcMPa) * b0Mm * dMm;
    double vcNExpected = std::min({vc1N, vc2N, vc3N});
    double phiVcKNExpected = 0.75 * vcNExpected / 1000.0;

    auto result = checkTwoWayShear(500.0, colBMm, colHMm, dMm, fcMPa);
    assert(approxEqual(result.b0Mm, b0Mm));
    assert(approxEqual(result.phiVcKN, phiVcKNExpected));
    assert(result.adequate == (500.0 <= phiVcKNExpected));
    std::cout << "  testTwoWayShearMatchesHandCalcForSquareColumn OK\n";
}

static void testTwoWayShearInadequateWhenDemandExceedsCapacity() {
    auto result = checkTwoWayShear(1.0e9, 400.0, 400.0, 200.0, 25.0);
    assert(!result.adequate);
    std::cout << "  testTwoWayShearInadequateWhenDemandExceedsCapacity OK\n";
}

// ---------------------------------------------------------------------------
// designIsolatedFooting: end-to-end -- both shear checks must pass at the
// returned thickness (that's the solver's own convergence condition, but
// re-asserted here independently), and the flexural steel must round-trip
// to at least the design moment via an independently-computed phi*Mn.
// ---------------------------------------------------------------------------

static void testDesignIsolatedFootingConvergesAndSatisfiesBothShearChecks() {
    double puKN = 900.0, serviceLoadKN = 650.0, qAllowKPa = 150.0;
    double colBM = 0.4, colHM = 0.4, coverMm = 75.0, barDiaMm = 16.0;
    double fcMPa = 25.0, fyMPa = 420.0;

    auto result = designIsolatedFooting(puKN, serviceLoadKN, qAllowKPa, colBM, colHM, coverMm,
                                         barDiaMm, fcMPa, fyMPa);

    assert(result.twoWayShear.adequate);
    assert(result.oneWayShear.adequate);
    assert(result.thicknessMm >= 300.0 - 1e-9);
    assert(approxEqual(result.effectiveDepthMm, result.thicknessMm - coverMm - barDiaMm / 2.0));

    // Bearing pressure sanity: factored/service pressures should be
    // strictly positive and the plan side should match the independent
    // area-sizing formula exactly (the solver doesn't touch plan area,
    // only thickness).
    auto areaCheck = sizeSquareFootingArea(serviceLoadKN, qAllowKPa);
    assert(approxEqual(result.sideM, areaCheck.sideM));
    assert(result.factoredBearingPressureKPa > 0.0);

    // Flexural round-trip: recompute phi*Mn from the returned As at the
    // full-footing-width design strip and confirm it's at least the
    // moment demand the solver used (it may exceed it slightly if the
    // minimum-reinforcement floor governed).
    double dM = result.effectiveDepthMm / 1000.0;
    double phiMn = phiMnFromAsKNm(result.flexure.asRequiredMm2, result.sideM, dM, fcMPa, fyMPa);
    double lc1 = (result.sideM - colBM) / 2.0 - dM;
    double lc2 = (result.sideM - colHM) / 2.0 - dM;
    double lcGoverning = std::max({0.0, lc1, lc2});
    double muExpectedKNm = result.factoredBearingPressureKPa * lcGoverning * lcGoverning / 2.0 * result.sideM;
    assert(phiMn >= muExpectedKNm - 1e-2 || result.flexure.governedByMinimum);
    std::cout << "  testDesignIsolatedFootingConvergesAndSatisfiesBothShearChecks OK\n";
}

static void testDesignIsolatedFootingNetPunchingDemandBelowGrossPu() {
    // The net two-way shear demand subtracts the bearing pressure already
    // reacting under the critical perimeter's own footprint, so it must
    // never exceed the column's gross factored load.
    auto result = designIsolatedFooting(1500.0, 1100.0, 180.0, 0.45, 0.45, 75.0, 16.0, 28.0, 420.0);
    assert(result.twoWayShear.vuKN <= 1500.0 + 1e-6);
    assert(result.twoWayShear.vuKN >= 0.0);
    std::cout << "  testDesignIsolatedFootingNetPunchingDemandBelowGrossPu OK\n";
}

static void testDesignIsolatedFootingRejectsInvalidInputs() {
    bool threw = false;
    try {
        designIsolatedFooting(-100.0, 650.0, 150.0, 0.4, 0.4, 75.0, 16.0, 25.0, 420.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testDesignIsolatedFootingRejectsInvalidInputs OK\n";
}

// ---------------------------------------------------------------------------
// sizeCombinedFootingPlan / computeCombinedFootingInternalForces /
// designCombinedFooting -- two symmetric columns, hand-checkable by
// elementary statics.
// ---------------------------------------------------------------------------

static std::vector<CombinedFootingColumnLoad> twoSymmetricColumns() {
    // Two identical columns 4m apart, positions 0 and 4 (arbitrary global
    // coords) -- centroid must land exactly at x=2 by symmetry.
    CombinedFootingColumnLoad c1;
    c1.positionM = 0.0;
    c1.puKN = 800.0;
    c1.serviceLoadKN = 600.0;
    c1.colLengthM = 0.4;
    c1.colWidthM = 0.4;

    CombinedFootingColumnLoad c2 = c1;
    c2.positionM = 4.0;

    return {c1, c2};
}

static void testSizeCombinedFootingPlanCentersOnLoadCentroidForSymmetricLoads() {
    auto columns = twoSymmetricColumns();
    // area = (600+600)/150 = 8 m^2; width 2m -> length-by-area = 4m. But
    // edge-clearance geometry (0.3m clear beyond each column's 0.4m-wide
    // face, symmetric about the centroid) needs L = 2*((2-(-0.2))+0.3) = 5m,
    // which governs here.
    auto plan = sizeCombinedFootingPlan(columns, 2.0, 150.0, 0.3);
    assert(approxEqual(plan.resultantPositionM, 2.0));
    assert(approxEqual(plan.lengthM, 5.0));
    assert(approxEqual(plan.footingStartM, 2.0 - plan.lengthM / 2.0));
    std::cout << "  testSizeCombinedFootingPlanCentersOnLoadCentroidForSymmetricLoads OK\n";
}

static void testSizeCombinedFootingPlanGeometryGovernsWhenColumnsAreFarApart() {
    auto columns = twoSymmetricColumns();
    columns[1].positionM = 20.0;  // columns 20m apart -- area-based length (small loads/width) can't reach this
    auto plan = sizeCombinedFootingPlan(columns, 2.0, 150.0, 0.3);
    // Centroid still at the midpoint by symmetry; length must at least span
    // both columns' outer faces plus clearance on each side.
    assert(approxEqual(plan.resultantPositionM, 10.0));
    double requiredSpan = 20.0 + 0.4 /*half of each column's 0.4m, both sides*/ + 2 * 0.3;
    assert(plan.lengthM >= requiredSpan - 1e-9);
    std::cout << "  testSizeCombinedFootingPlanGeometryGovernsWhenColumnsAreFarApart OK\n";
}

static void testCombinedFootingInternalForcesMatchHandStatics() {
    // Two columns near the footing's ends (short 0.5m overhangs past each
    // one) under uniform upward pressure sized to their own total load is
    // mechanically the sign-mirror of a normal overhanging beam under
    // gravity load (here the "distributed load" is the upward soil
    // reaction and the "point loads" are the downward columns) -- so the
    // physically correct result is a small SAGGING moment right at each
    // column (from the short overhang) and a large HOGGING moment at
    // midspan between them, not the other way around. Verified here
    // against hand statics at both locations.
    auto columns = twoSymmetricColumns();
    auto plan = sizeCombinedFootingPlan(columns, 2.0, 150.0, 0.3);  // -> L=5m, columns at local 0.5/4.5

    double totalPu = 1600.0;
    double quKPa = totalPu / (plan.lengthM * plan.widthM);
    double wKNPerM = quKPa * plan.widthM;

    auto forces = computeCombinedFootingInternalForces(columns, plan);

    double pos0Local = columns[0].positionM - plan.footingStartM;  // 0.5m
    double mAtColumnExpected = wKNPerM * pos0Local * pos0Local / 2.0;  // no Pu term yet at s=pos0Local
    assert(approxEqual(forces.maxSaggingMomentKNm, mAtColumnExpected, 1e-2));
    assert(approxEqual(forces.saggingPositionM, pos0Local, 1e-2));

    double sMid = plan.lengthM / 2.0;
    double mMidExpected = wKNPerM * sMid * sMid / 2.0 - columns[0].puKN * (sMid - pos0Local);
    assert(mMidExpected < 0.0);  // confirms this is genuinely the hogging region
    assert(approxEqual(forces.maxHoggingMomentKNm, -mMidExpected, 1e-2));
    assert(approxEqual(forces.hoggingPositionM, sMid, 1e-2));
    std::cout << "  testCombinedFootingInternalForcesMatchHandStatics OK\n";
}

static void testDesignCombinedFootingConvergesAndSatisfiesAllShearChecks() {
    auto columns = twoSymmetricColumns();
    auto result = designCombinedFooting(columns, 2.0, 150.0, 75.0, 16.0, 25.0, 420.0);

    assert(result.oneWayShear.adequate);
    assert(result.punchingPerColumn.size() == columns.size());
    for (const auto& pw : result.punchingPerColumn) assert(pw.adequate);
    assert(result.thicknessMm >= 350.0 - 1e-9);

    // This geometry hogs at midspan (see testCombinedFootingInternalForces...),
    // so it's the TOP steel's round-trip that matters here -- same
    // recompute-phi*Mn-from-As strategy as elsewhere in this file.
    double dM = result.effectiveDepthMm / 1000.0;
    double phiMnTop = phiMnFromAsKNm(result.topFlexure.asRequiredMm2, result.plan.widthM, dM, 25.0, 420.0);
    assert(phiMnTop >= result.forces.maxHoggingMomentKNm - 1e-2 || result.topFlexure.governedByMinimum);
    std::cout << "  testDesignCombinedFootingConvergesAndSatisfiesAllShearChecks OK\n";
}

static void testDesignCombinedFootingRejectsSingleColumn() {
    auto columns = twoSymmetricColumns();
    columns.pop_back();
    bool threw = false;
    try {
        designCombinedFooting(columns, 2.0, 150.0, 75.0, 16.0, 25.0, 420.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testDesignCombinedFootingRejectsSingleColumn OK\n";
}

// ---------------------------------------------------------------------------
// Pile cap -- 4-pile square arrangement under a concentric column load,
// hand-checkable by symmetry and elementary statics.
// ---------------------------------------------------------------------------

static std::vector<PilePosition> fourPileSquareGroup(double halfSpacingM) {
    return {
        {-halfSpacingM, -halfSpacingM},
        {halfSpacingM, -halfSpacingM},
        {-halfSpacingM, halfSpacingM},
        {halfSpacingM, halfSpacingM},
    };
}

static void testSizePileCapPlanMatchesBoundingBoxHandCalc() {
    auto piles = fourPileSquareGroup(0.6);  // pile centers at +-0.6m, spacing 1.2m
    double pileDiaM = 0.4;
    auto plan = sizePileCapPlan(piles, pileDiaM, 0.15);
    // bounding box span = 1.2m each direction, + pile dia 0.4 + 2*0.15 edge = 1.2+0.4+0.3 = 1.9
    assert(approxEqual(plan.lengthM, 1.9));
    assert(approxEqual(plan.widthM, 1.9));
    std::cout << "  testSizePileCapPlanMatchesBoundingBoxHandCalc OK\n";
}

static void testPileCapMomentDemandZeroWhenColumnCoversAllPiles() {
    // Column half-width bigger than the pile offset -> no pile lies beyond
    // the column face -> zero moment demand in both directions.
    auto piles = fourPileSquareGroup(0.3);
    auto moments = computePileCapMomentDemand(piles, 1200.0, 1.0, 1.0);  // colB=colH=1.0m, half=0.5m > 0.3m offset
    assert(approxEqual(moments.momentAlongXKNm, 0.0, 1e-6));
    assert(approxEqual(moments.momentAlongYKNm, 0.0, 1e-6));
    std::cout << "  testPileCapMomentDemandZeroWhenColumnCoversAllPiles OK\n";
}

static void testPileCapMomentDemandMatchesHandCalc() {
    auto piles = fourPileSquareGroup(0.6);  // 2 piles beyond each face at offset 0.6m
    double puKN = 1200.0;
    double colBM = 0.4, colHM = 0.4;  // half = 0.2m
    auto moments = computePileCapMomentDemand(piles, puKN, colBM, colHM);
    double perPileKN = puKN / 4.0;
    // 2 piles beyond x=+0.2 at x=0.6 -> moment = 2 * perPile * (0.6-0.2)
    double expected = 2.0 * perPileKN * (0.6 - 0.2);
    assert(approxEqual(moments.momentAlongXKNm, expected));
    assert(approxEqual(moments.momentAlongYKNm, expected));  // symmetric group
    std::cout << "  testPileCapMomentDemandMatchesHandCalc OK\n";
}

static void testCheckPileIndividualPunchingFlagsEdgePileAsTruncated() {
    // A pile placed right at the cap's own edge (offset = half the cap
    // length minus a hair) must show a truncated perimeter; a pile safely
    // in the interior of a much bigger cap must not.
    std::vector<PilePosition> piles = {{0.0, 0.0}};
    PileCapPlanResult bigPlan;
    bigPlan.lengthM = 10.0;
    bigPlan.widthM = 10.0;
    auto interior = checkPileIndividualPunching(piles, bigPlan, 0.4, 400.0, 500.0, 25.0);
    assert(!interior[0].truncated);

    PileCapPlanResult tightPlan;
    tightPlan.lengthM = 0.7;  // pile at x=0 is only 0.35m from each edge, inside the ~0.4m critical radius
    tightPlan.widthM = 0.7;
    auto edge = checkPileIndividualPunching(piles, tightPlan, 0.4, 400.0, 500.0, 25.0);
    assert(edge[0].truncated);
    assert(edge[0].b0Mm < interior[0].b0Mm);  // truncation must reduce the perimeter
    std::cout << "  testCheckPileIndividualPunchingFlagsEdgePileAsTruncated OK\n";
}

static void testDesignPileCapConvergesAndSatisfiesAllChecks() {
    auto piles = fourPileSquareGroup(0.6);
    double puKN = 1200.0, colBM = 0.4, colHM = 0.4, pileDiaM = 0.4;
    double coverMm = 75.0, barDiaMm = 16.0, fcMPa = 25.0, fyMPa = 420.0;

    auto result = designPileCap(piles, puKN, colBM, colHM, pileDiaM, coverMm, barDiaMm, fcMPa, fyMPa);

    assert(result.columnPunching.adequate);
    assert(result.oneWayShearX.adequate && result.oneWayShearY.adequate);
    assert(result.pilePunching.size() == piles.size());
    for (const auto& pw : result.pilePunching) assert(pw.adequate);
    assert(result.thicknessMm >= 450.0 - 1e-9);

    // Flexural round-trip in the X direction (symmetric group -> Y should
    // match by symmetry, checked separately below).
    double dM = result.effectiveDepthMm / 1000.0;
    double phiMnX = phiMnFromAsKNm(result.flexureX.asRequiredMm2, result.plan.widthM, dM, fcMPa, fyMPa);
    auto moments = computePileCapMomentDemand(piles, puKN, colBM, colHM);
    assert(phiMnX >= moments.momentAlongXKNm - 1e-2 || result.flexureX.governedByMinimum);

    // Square, symmetric pile group and column -> the two directions'
    // demands (and hence designs) must be identical.
    assert(approxEqual(moments.momentAlongXKNm, moments.momentAlongYKNm));
    assert(approxEqual(result.flexureX.asRequiredMm2, result.flexureY.asRequiredMm2));
    std::cout << "  testDesignPileCapConvergesAndSatisfiesAllChecks OK\n";
}

static void testDesignPileCapRejectsInvalidInputs() {
    auto piles = fourPileSquareGroup(0.6);
    bool threw = false;
    try {
        designPileCap(piles, -100.0, 0.4, 0.4, 0.4, 75.0, 16.0, 25.0, 420.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testDesignPileCapRejectsInvalidInputs OK\n";
}

int main() {
    std::cout << "RCCFoundation tests:\n";
    testSizeSquareFootingAreaHandCalc();
    testSizeSquareFootingAreaRejectsNonPositive();
    testTwoWayShearMatchesHandCalcForSquareColumn();
    testTwoWayShearInadequateWhenDemandExceedsCapacity();
    testDesignIsolatedFootingConvergesAndSatisfiesBothShearChecks();
    testDesignIsolatedFootingNetPunchingDemandBelowGrossPu();
    testDesignIsolatedFootingRejectsInvalidInputs();
    testSizeCombinedFootingPlanCentersOnLoadCentroidForSymmetricLoads();
    testSizeCombinedFootingPlanGeometryGovernsWhenColumnsAreFarApart();
    testCombinedFootingInternalForcesMatchHandStatics();
    testDesignCombinedFootingConvergesAndSatisfiesAllShearChecks();
    testDesignCombinedFootingRejectsSingleColumn();
    testSizePileCapPlanMatchesBoundingBoxHandCalc();
    testPileCapMomentDemandZeroWhenColumnCoversAllPiles();
    testPileCapMomentDemandMatchesHandCalc();
    testCheckPileIndividualPunchingFlagsEdgePileAsTruncated();
    testDesignPileCapConvergesAndSatisfiesAllChecks();
    testDesignPileCapRejectsInvalidInputs();
    std::cout << "All RCCFoundation tests passed.\n";
    return 0;
}
