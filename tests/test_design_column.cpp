#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "design/RCCColumn.h"

using namespace nrsa::design;

static bool approxEqual(double a, double b, double relTol = 1e-2) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// ---------------------------------------------------------------------------
// generateRectangularLayout: geometry + bar-count sanity.
// ---------------------------------------------------------------------------

static void testLayoutBarCountAndSymmetry() {
    // 400x600mm column, 40mm cover, 25mm bars, 3 bars/face along B, 4 along H.
    auto layout = generateRectangularLayout(0.4, 0.6, 40.0, 25.0, 3, 4);
    // 4 corners + (3-2)*2 mid-B bars + (4-2)*2 mid-H bars = 4 + 2 + 4 = 10
    assert(layout.barsXY.size() == 10);

    // Every bar must lie strictly inside the gross section and at the
    // expected edge distance (cover + dia/2 = 52.5mm) on whichever face it
    // sits on -- an independent geometric check, not just "did it not crash".
    double de = 40.0 + 25.0 / 2.0;
    for (const auto& [x, y] : layout.barsXY) {
        bool onVerticalFace = approxEqual(x, de, 1e-6) || approxEqual(x, layout.bMm - de, 1e-6);
        bool onHorizontalFace = approxEqual(y, de, 1e-6) || approxEqual(y, layout.hMm - de, 1e-6);
        assert(onVerticalFace || onHorizontalFace);
        assert(x >= de - 1e-6 && x <= layout.bMm - de + 1e-6);
        assert(y >= de - 1e-6 && y <= layout.hMm - de + 1e-6);
    }
    std::cout << "  testLayoutBarCountAndSymmetry OK\n";
}

// ---------------------------------------------------------------------------
// axialCapacityPo: hand-calculated closed form, independent of the
// strain-compatibility sweep entirely.
// ---------------------------------------------------------------------------

static void testPoMatchesHandCalculation() {
    // 400x400mm column, fc'=28MPa, fy=414MPa, Ast=8-25mm bars.
    double barArea = M_PI / 4.0 * 25.0 * 25.0;  // 490.87 mm^2
    double ast = 8 * barArea;                    // 3926.99 mm^2
    double ag = 400.0 * 400.0;                   // 160000 mm^2
    // Hand calc: Po = 0.85*28*(160000-3926.99) + 414*3926.99
    //               = 0.85*28*156073.01 + 1625773.86
    //               = 3714458.94 + 1625773.86 = 5340232.8 N = 5340.23 kN
    double expectedPoKN = (0.85 * 28.0 * (ag - ast) + 414.0 * ast) / 1000.0;
    double actual = axialCapacityPo(400.0, 400.0, ast, 28.0, 414.0);
    assert(approxEqual(actual, expectedPoKN, 1e-6));
    assert(approxEqual(actual, 5340.2, 1e-2));  // pinned numeric value, hand-verified above
    std::cout << "  testPoMatchesHandCalculation OK\n";
}

// ---------------------------------------------------------------------------
// computeUniaxialInteractionDiagram: the diagram's own tail must converge
// to the SAME Po the closed-form formula gives -- an independent
// cross-check between two different code paths that should agree.
// ---------------------------------------------------------------------------

static void testDiagramConvergesToClosedFormPo() {
    double barArea = M_PI / 4.0 * 25.0 * 25.0;
    std::vector<RebarLayer> layers = {
        {52.5, 3 * barArea},              // bottom layer (3 bars)
        {200.0, 2 * barArea},             // mid-depth layer (2 bars, one per side)
        {347.5, 3 * barArea},             // top layer (3 bars)
    };
    double ast = 8 * barArea;
    auto diagram = computeUniaxialInteractionDiagram(400.0, 400.0, layers, 28.0, 414.0);
    double expectedPo = axialCapacityPo(400.0, 400.0, ast, 28.0, 414.0);
    // Last point uses c=6*depth, deep enough that a is long since clipped
    // to h and every layer's strain has essentially reached ecu -- should
    // be within a fraction of a percent of the exact closed-form value.
    assert(approxEqual(diagram.back().pnKN, expectedPo, 5e-3));
    std::cout << "  testDiagramConvergesToClosedFormPo OK\n";
}

static void testDiagramPureCompressionHasZeroMoment() {
    // Symmetric layout -> at large c (near-uniform compression strain),
    // moment about the centroid must vanish by symmetry -- an independent
    // physical-sense check, not just "the code agrees with itself".
    double barArea = M_PI / 4.0 * 25.0 * 25.0;
    std::vector<RebarLayer> layers = {
        {52.5, 3 * barArea}, {200.0, 2 * barArea}, {347.5, 3 * barArea},
    };
    auto diagram = computeUniaxialInteractionDiagram(400.0, 400.0, layers, 28.0, 414.0);
    assert(std::abs(diagram.back().mnKNm) < 1.0);  // near-zero, in kN*m, on section carrying ~5300kN
    std::cout << "  testDiagramPureCompressionHasZeroMoment OK\n";
}

static void testPhiTransitionsCorrectly() {
    // phi must be 0.65 at compression-controlled points (large c, small
    // et) and 0.90 at tension-controlled points (small c, large et) --
    // checked against the exact ACI 318-19 Table 21.2.2 thresholds, an
    // independent formula from the one under test.
    double barArea = M_PI / 4.0 * 25.0 * 25.0;
    std::vector<RebarLayer> layers = {
        {52.5, 3 * barArea}, {200.0, 2 * barArea}, {347.5, 3 * barArea},
    };
    auto diagram = computeUniaxialInteractionDiagram(400.0, 400.0, layers, 28.0, 414.0);
    assert(approxEqual(diagram.back().phi, 0.65, 1e-6));    // deep compression end
    assert(approxEqual(diagram.front().phi, 0.90, 1e-6));   // shallow c, tension-controlled end
    double ey = 414.0 / 200000.0;
    for (const auto& pt : diagram) {
        if (pt.etExtreme <= ey) assert(approxEqual(pt.phi, 0.65, 1e-6));
        if (pt.etExtreme >= 0.005) assert(approxEqual(pt.phi, 0.90, 1e-6));
        assert(pt.phi >= 0.65 - 1e-9 && pt.phi <= 0.90 + 1e-9);
    }
    std::cout << "  testPhiTransitionsCorrectly OK\n";
}

// ---------------------------------------------------------------------------
// designBiaxialColumn: end-to-end behavior checks.
// ---------------------------------------------------------------------------

static void testBiaxialCapacityLessThanEitherUniaxialCapacity() {
    // A square column with equal ex, ey must have a biaxial capacity no
    // greater than either uniaxial capacity alone -- this is the defining
    // physical property of the interaction being modeled at all (loading
    // about two axes at once can only ever be as good as or worse than
    // loading about one), independent of the exact numbers.
    double b = 0.45, h = 0.45;
    auto result = designBiaxialColumn(b, h, 40.0, 25.0, 4, 4, 28.0, 414.0, 1500.0, 150.0, 150.0);
    assert(result.pnBiaxialKN <= result.pnxKN + 1e-6);
    assert(result.pnBiaxialKN <= result.pnyKN + 1e-6);
    std::cout << "  testBiaxialCapacityLessThanEitherUniaxialCapacity OK\n";
}

static void testSquareColumnSymmetricLoadGivesEqualUniaxialCapacities() {
    // A square, symmetrically reinforced column loaded with Mux=Muy must
    // give Pnx == Pny by the section's own symmetry -- independent
    // geometric-sense check.
    auto result = designBiaxialColumn(0.45, 0.45, 40.0, 25.0, 4, 4, 28.0, 414.0, 1500.0, 150.0, 150.0);
    assert(approxEqual(result.pnxKN, result.pnyKN, 1e-6));
    std::cout << "  testSquareColumnSymmetricLoadGivesEqualUniaxialCapacities OK\n";
}

static void testPureAxialCaseApproachesPnMax() {
    // Zero moment demand -> should recover a capacity at/near the 0.80*Po
    // tied-column cap, not some arbitrarily different number.
    auto result = designBiaxialColumn(0.4, 0.4, 40.0, 25.0, 3, 3, 28.0, 414.0, 100.0, 0.5, 0.5);
    assert(approxEqual(result.pnBiaxialKN, result.pnMaxKN, 0.05));
    std::cout << "  testPureAxialCaseApproachesPnMax OK\n";
}

static void testInadequateSectionFlagged() {
    // Deliberately overload a small, lightly reinforced column -- must
    // come back inadequate, not silently "pass".
    auto result = designBiaxialColumn(0.25, 0.25, 40.0, 16.0, 2, 2, 21.0, 414.0, 3000.0, 200.0, 200.0);
    assert(!result.adequate);
    std::cout << "  testInadequateSectionFlagged OK\n";
}

static void testRejectsNonPositiveAxialLoad() {
    bool threw = false;
    try {
        designBiaxialColumn(0.4, 0.4, 40.0, 25.0, 3, 3, 28.0, 414.0, 0.0, 100.0, 100.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testRejectsNonPositiveAxialLoad OK\n";
}

static void testLowAxialLoadWarningFlagged() {
    // Pu deliberately well under 10% of Po -- Bresler's known weak spot.
    double barArea = M_PI / 4.0 * 20.0 * 20.0;
    double ast = 8 * barArea;
    double po = axialCapacityPo(400.0, 400.0, ast, 28.0, 414.0);
    auto result = designBiaxialColumn(0.4, 0.4, 40.0, 20.0, 3, 3, 28.0, 414.0, 0.05 * po, 50.0, 50.0);
    assert(result.lowAxialLoadWarning);
    std::cout << "  testLowAxialLoadWarningFlagged OK\n";
}

int main() {
    std::cout << "Running RCCColumn design tests...\n";
    testLayoutBarCountAndSymmetry();
    testPoMatchesHandCalculation();
    testDiagramConvergesToClosedFormPo();
    testDiagramPureCompressionHasZeroMoment();
    testPhiTransitionsCorrectly();
    testBiaxialCapacityLessThanEitherUniaxialCapacity();
    testSquareColumnSymmetricLoadGivesEqualUniaxialCapacities();
    testPureAxialCaseApproachesPnMax();
    testInadequateSectionFlagged();
    testRejectsNonPositiveAxialLoad();
    testLowAxialLoadWarningFlagged();
    std::cout << "All RCCColumn tests passed.\n";
    return 0;
}
