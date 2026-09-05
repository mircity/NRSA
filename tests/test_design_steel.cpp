#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "design/Steel.h"

using namespace nrsa::design;

static bool approxEqual(double a, double b, double relTol = 1e-2) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// A representative stocky, roughly W16x50-scale doubly-symmetric shape
// used across most tests below (mm): d=410, bf=180, tf=16, tw=10.
static WShapeSection sampleShape() {
    return buildWShape(410.0, 180.0, 16.0, 10.0, "sample");
}

// ---------------------------------------------------------------------------
// buildWShape: exact-formula and independent-derivation checks.
// ---------------------------------------------------------------------------

static void testAreaMatchesDirectSum() {
    auto s = sampleShape();
    double h = 410.0 - 2.0 * 16.0;
    double expected = 2.0 * 180.0 * 16.0 + h * 10.0;
    assert(approxEqual(s.areaMm2, expected, 1e-9));
    std::cout << "  testAreaMatchesDirectSum OK\n";
}

// Ix via the "hollow minus notches" formula the implementation uses is
// cross-checked here against a completely independent parallel-axis
// derivation: two flange rectangles (about their own centroid, shifted to
// the section centroid) plus the web rectangle about its own centroid
// (already centered, no shift needed).
static void testIxMatchesParallelAxisDerivation() {
    auto s = sampleShape();
    double h = 410.0 - 2.0 * 16.0;
    double flangeSelfI = 180.0 * 16.0 * 16.0 * 16.0 / 12.0;
    double flangeShift = (410.0 - 16.0) / 2.0;  // distance from section centroid to flange centroid
    double flangeContribution = 2.0 * (flangeSelfI + 180.0 * 16.0 * flangeShift * flangeShift);
    double webContribution = 10.0 * h * h * h / 12.0;
    double expected = flangeContribution + webContribution;
    assert(approxEqual(s.ixMm4, expected, 1e-6));
    std::cout << "  testIxMatchesParallelAxisDerivation OK\n";
}

static void testShapeFactorsInPhysicalRange() {
    auto s = sampleShape();
    double shapeFactorX = s.zxMm3 / s.sxMm3;
    double shapeFactorY = s.zyMm3 / s.syMm3;
    // A doubly-symmetric I-shape's strong-axis shape factor is well known
    // to fall around 1.10-1.20; weak axis is higher, roughly 1.5.
    assert(shapeFactorX > 1.05 && shapeFactorX < 1.25);
    assert(shapeFactorY > 1.3 && shapeFactorY < 1.7);
    std::cout << "  testShapeFactorsInPhysicalRange OK\n";
}

static void testRejectsDegenerateGeometry() {
    bool threw = false;
    try {
        buildWShape(30.0, 180.0, 16.0, 10.0);  // 2*tf >= d
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testRejectsDegenerateGeometry OK\n";
}

// ---------------------------------------------------------------------------
// checkCompactness
// ---------------------------------------------------------------------------

static void testStockyShapeIsCompact() {
    auto s = sampleShape();
    auto c = checkCompactness(s, 345.0);  // Fy=345MPa (Grade 50)
    assert(c.compact);
    std::cout << "  testStockyShapeIsCompact OK\n";
}

static void testSlenderPlatesFlaggedNonCompact() {
    // Deliberately thin flange/web relative to their width/depth.
    auto slender = buildWShape(600.0, 300.0, 6.0, 4.0);
    auto c = checkCompactness(slender, 345.0);
    assert(!c.flangeCompact || !c.webCompact);
    assert(!c.compact);
    std::cout << "  testSlenderPlatesFlaggedNonCompact OK\n";
}

// ---------------------------------------------------------------------------
// designFlexuralCapacity
// ---------------------------------------------------------------------------

static void testYieldingRegimeMatchesClosedFormMp() {
    auto s = sampleShape();
    double fy = 345.0, e = 200000.0;
    auto r = designFlexuralCapacity(s, fy, e, 0.0);  // Lb=0 -- fully braced
    assert(r.regime == FlexuralCapacityResult::Regime::Yielding);
    double expectedMp = fy * s.zxMm3 / 1.0e6;
    assert(approxEqual(r.mnKNm, expectedMp, 1e-9));
    assert(approxEqual(r.phiMnKNm, 0.90 * expectedMp, 1e-9));
    std::cout << "  testYieldingRegimeMatchesClosedFormMp OK\n";
}

static void testLpLessThanLr() {
    auto s = sampleShape();
    auto r = designFlexuralCapacity(s, 345.0, 200000.0, 0.0);
    assert(r.lpM > 0.0 && r.lrM > r.lpM);
    std::cout << "  testLpLessThanLr OK\n";
}

// AISC F2-2 (inelastic LTB, the linear segment between (Lp,Mp) and
// (Lr,0.7*Fy*Sx)) hits its own right-hand anchor point EXACTLY at Lb=Lr, by
// construction -- checked below. F2-3/F2-4 (elastic LTB) is only
// NEAR-continuous with that same anchor there, not exactly: Lr itself
// (F2-6) is the algebraic inverse of the elastic Fcr formula, but AISC
// publishes both with independently rounded constants (0.078, 1.95, 6.76),
// so solving one and substituting into the other doesn't cancel out
// perfectly -- a real, small (~0.1%) discontinuity baked into the code's
// own published coefficients, not a bug in this implementation. Both
// properties are checked directly, at the tolerance each one actually
// supports, rather than assuming both are exact.
static void testFlexuralContinuityAtLr() {
    auto s = sampleShape();
    double fy = 345.0, e = 200000.0;
    auto atLr = designFlexuralCapacity(s, fy, e, 0.0);  // just need Lr from any call
    double lr = atLr.lrM;

    auto justBelow = designFlexuralCapacity(s, fy, e, lr - 1e-6);
    auto justAbove = designFlexuralCapacity(s, fy, e, lr + 1e-6);

    double sevenTenthsFySx = 0.7 * fy * s.sxMm3 / 1.0e6;
    assert(approxEqual(justBelow.mnKNm, sevenTenthsFySx, 1e-6));  // exact, by construction
    assert(approxEqual(justAbove.mnKNm, sevenTenthsFySx, 5e-3));  // near-continuous only, see above
    assert(justBelow.regime == FlexuralCapacityResult::Regime::InelasticLTB);
    assert(justAbove.regime == FlexuralCapacityResult::Regime::ElasticLTB);
    std::cout << "  testFlexuralContinuityAtLr OK\n";
}

static void testFlexuralMnNeverExceedsMp() {
    auto s = sampleShape();
    double fy = 345.0, e = 200000.0;
    auto mp = designFlexuralCapacity(s, fy, e, 0.0).mpKNm;
    for (double lb : {0.5, 1.0, 2.0, 3.0, 5.0, 8.0, 15.0, 30.0}) {
        auto r = designFlexuralCapacity(s, fy, e, lb);
        assert(r.mnKNm <= mp + 1e-6);
    }
    std::cout << "  testFlexuralMnNeverExceedsMp OK\n";
}

static void testNonCompactSectionFlaggedNotComputed() {
    auto slender = buildWShape(600.0, 300.0, 6.0, 4.0);
    auto r = designFlexuralCapacity(slender, 345.0, 200000.0, 2.0);
    assert(r.sectionNotCompact);
    assert(r.mnKNm == 0.0 && r.phiMnKNm == 0.0);
    std::cout << "  testNonCompactSectionFlaggedNotComputed OK\n";
}

// ---------------------------------------------------------------------------
// checkShearCapacity
// ---------------------------------------------------------------------------

static void testStockyWebShearMatchesClosedForm() {
    auto s = sampleShape();
    double fy = 345.0, e = 200000.0;
    auto r = checkShearCapacity(s, fy, e);
    // Sample shape h/tw = (410-32)/10 = 37.8, well under 2.24*sqrt(E/Fy)~53.9
    // -- G2.1a should govern with Cv1=phi=1.0 exactly.
    assert(approxEqual(r.phi, 1.0, 1e-9));
    assert(approxEqual(r.cv1, 1.0, 1e-9));
    double expectedVn = 0.6 * fy * (s.dMm * s.twMm) / 1000.0;
    assert(approxEqual(r.vnKN, expectedVn, 1e-9));
    assert(approxEqual(r.phiVnKN, expectedVn, 1e-9));
    std::cout << "  testStockyWebShearMatchesClosedForm OK\n";
}

static void testSlenderWebShearReduced() {
    // Deliberately thin web (large h/tw) to force G2.1b with Cv1 < 1.
    auto thin = buildWShape(600.0, 200.0, 14.0, 5.0);
    auto r = checkShearCapacity(thin, 345.0, 200000.0);
    assert(approxEqual(r.phi, 0.90, 1e-9));
    assert(r.cv1 < 1.0);
    std::cout << "  testSlenderWebShearReduced OK\n";
}

// ---------------------------------------------------------------------------
// designCompressionCapacity
// ---------------------------------------------------------------------------

static void testShortColumnApproachesFy() {
    auto s = sampleShape();
    double fy = 345.0, e = 200000.0;
    // A very short effective length -> KL/r near zero -> Fcr should
    // approach Fy itself (negligible buckling reduction).
    auto r = designCompressionCapacity(s, fy, e, 0.05, 0.05);
    assert(!r.elasticBuckling);
    assert(approxEqual(r.fcrMPa, fy, 0.02));
    std::cout << "  testShortColumnApproachesFy OK\n";
}

static void testWeakAxisGovernsUnbracedColumn() {
    auto s = sampleShape();
    // Same physical length both directions, unequal r -- weak axis (y)
    // must give the larger KL/r and therefore govern.
    auto r = designCompressionCapacity(s, 345.0, 200000.0, 4.0, 4.0);
    assert(r.klrY > r.klrX);
    assert(approxEqual(r.governingKlr, r.klrY, 1e-9));
    std::cout << "  testWeakAxisGovernsUnbracedColumn OK\n";
}

// The E3-2/E3-3 transition at KL/r = 4.71*sqrt(E/Fy) is a real, if
// deliberately slightly non-smooth, feature of AISC's own formulas (E3-2's
// 0.658^(Fy/Fe) form and E3-3's 0.877*Fe form are not mathematically forced
// to coincide there, unlike F2's Lr point) -- checked here for near-
// continuity (a few percent), not exact equality, and documented as such.
static void testCompressionNearContinuityAtSlendernessLimit() {
    auto s = sampleShape();
    double fy = 345.0, e = 200000.0;
    double limitKlr = 4.71 * std::sqrt(e / fy);
    // Solve for an effective length (both axes equal, ry governs since
    // ry<rx) that lands weak-axis KL/r right at the limit.
    double klM = limitKlr * s.ryMm / 1000.0;

    auto justBelow = designCompressionCapacity(s, fy, e, klM - 1e-4, klM - 1e-4);
    auto justAbove = designCompressionCapacity(s, fy, e, klM + 1e-4, klM + 1e-4);
    assert(!justBelow.elasticBuckling);
    assert(justAbove.elasticBuckling);
    assert(approxEqual(justBelow.fcrMPa, justAbove.fcrMPa, 0.02));
    std::cout << "  testCompressionNearContinuityAtSlendernessLimit OK\n";
}

static void testCompressionMatchesIndependentEulerFormula() {
    auto s = sampleShape();
    double fy = 345.0, e = 200000.0;
    double klM = 20.0;  // deliberately long -- elastic buckling governs
    auto r = designCompressionCapacity(s, fy, e, klM, klM);
    assert(r.elasticBuckling);
    // Independent recomputation of Euler buckling load from I and KL
    // directly (not via r=sqrt(I/A) and KL/r), converted back to a stress.
    double klMm = klM * 1000.0;
    double peN = M_PI * M_PI * e * s.iyMm4 / (klMm * klMm);  // weak axis governs
    double expectedFcr = 0.877 * (peN / s.areaMm2);
    assert(approxEqual(r.fcrMPa, expectedFcr, 1e-6));
    std::cout << "  testCompressionMatchesIndependentEulerFormula OK\n";
}

// ---------------------------------------------------------------------------
// checkCombinedInteraction
// ---------------------------------------------------------------------------

static void testInteractionEquationSelectionAtThreshold() {
    // Pr/Pc exactly 0.2 -- H1-1a's own defining boundary.
    auto atLimit = checkCombinedInteraction(20.0, 100.0, 10.0, 100.0, 0.0, 100.0);
    assert(atLimit.equation == "H1-1a");
    auto justBelow = checkCombinedInteraction(19.999, 100.0, 10.0, 100.0, 0.0, 100.0);
    assert(justBelow.equation == "H1-1b");
    std::cout << "  testInteractionEquationSelectionAtThreshold OK\n";
}

static void testInteractionHandCalculation() {
    // Pr/Pc = 0.5 (>=0.2, H1-1a): ratio = 0.5 + 8/9*(0.3+0.2) = 0.5+0.4444 = 0.9444
    auto r = checkCombinedInteraction(50.0, 100.0, 30.0, 100.0, 20.0, 100.0);
    assert(approxEqual(r.ratio, 0.9444, 1e-3));
    assert(r.adequate);

    // Pr/Pc = 0.1 (<0.2, H1-1b): ratio = 0.05 + (0.3+0.2) = 0.55
    auto r2 = checkCombinedInteraction(10.0, 100.0, 30.0, 100.0, 20.0, 100.0);
    assert(approxEqual(r2.ratio, 0.55, 1e-6));
    assert(r2.adequate);
    std::cout << "  testInteractionHandCalculation OK\n";
}

static void testInteractionFlagsInadequateMember() {
    auto r = checkCombinedInteraction(90.0, 100.0, 80.0, 100.0, 80.0, 100.0);
    assert(!r.adequate);
    assert(r.ratio > 1.0);
    std::cout << "  testInteractionFlagsInadequateMember OK\n";
}

static void testInteractionRejectsNonPositiveCapacities() {
    bool threw = false;
    try {
        checkCombinedInteraction(10.0, 0.0, 10.0, 100.0, 10.0, 100.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testInteractionRejectsNonPositiveCapacities OK\n";
}

int main() {
    std::cout << "Running Steel design tests...\n";
    testAreaMatchesDirectSum();
    testIxMatchesParallelAxisDerivation();
    testShapeFactorsInPhysicalRange();
    testRejectsDegenerateGeometry();
    testStockyShapeIsCompact();
    testSlenderPlatesFlaggedNonCompact();
    testYieldingRegimeMatchesClosedFormMp();
    testLpLessThanLr();
    testFlexuralContinuityAtLr();
    testFlexuralMnNeverExceedsMp();
    testNonCompactSectionFlaggedNotComputed();
    testStockyWebShearMatchesClosedForm();
    testSlenderWebShearReduced();
    testShortColumnApproachesFy();
    testWeakAxisGovernsUnbracedColumn();
    testCompressionNearContinuityAtSlendernessLimit();
    testCompressionMatchesIndependentEulerFormula();
    testInteractionEquationSelectionAtThreshold();
    testInteractionHandCalculation();
    testInteractionFlagsInadequateMember();
    testInteractionRejectsNonPositiveCapacities();
    std::cout << "All Steel design tests passed.\n";
    return 0;
}
