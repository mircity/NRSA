#include <cassert>
#include <cmath>
#include <iostream>

#include "analysis/MovingLoadAnalysis.h"

using namespace nrsa::analysis;

static bool approxEqual(double a, double b, double tol) { return std::abs(a - b) <= tol; }

// Textbook influence-line ordinate check (Hibbeler "Structural
// Analysis", simply-supported beam): unit load at a = L/2 on a L=10m
// span, moment IL at x = L/2 (a<=x boundary case) = (L-x)*a/L = 5*5/10 = 2.5
static void testMomentInfluenceOrdinateHandCalc() {
    double L = 10.0;
    assert(approxEqual(momentInfluenceOrdinate(L, 5.0, 5.0), 2.5, 1e-9));
    // Off the span entirely -> zero.
    assert(approxEqual(momentInfluenceOrdinate(L, 5.0, -1.0), 0.0, 1e-9));
    assert(approxEqual(momentInfluenceOrdinate(L, 5.0, 11.0), 0.0, 1e-9));
    // At a support (a=0), IL should be zero everywhere.
    assert(approxEqual(momentInfluenceOrdinate(L, 5.0, 0.0), 0.0, 1e-9));
    std::cout << "  testMomentInfluenceOrdinateHandCalc OK\n";
}

// Shear IL: at midspan, unit load just left (a=4.999) vs just right
// (a=5.001) should show the classic sign jump of magnitude ~1.
static void testShearInfluenceOrdinateJump() {
    double L = 10.0;
    double vLeft = shearInfluenceOrdinate(L, 5.0, 4.999);
    double vRight = shearInfluenceOrdinate(L, 5.0, 5.001);
    assert(vLeft < 0.0);
    assert(vRight > 0.0);
    assert(approxEqual(vRight - vLeft, 1.0, 1e-2));
    std::cout << "  testShearInfluenceOrdinateJump OK\n";
}

// Classic hand-calc: single point load P at midspan of a simply
// supported beam, L span -> Mmax = P*L/4 at midspan.
static void testSinglePointLoadMidspanMoment() {
    double L = 8.0, P = 50.0;  // kN
    std::vector<MovingPointLoad> train = {{P, 0.0}};
    auto envelope = runMovingLoadEnvelope(L, train, /*sections=*/41, /*step=*/0.02);

    // Find the section closest to midspan.
    const MovingLoadEnvelopePoint* mid = nullptr;
    for (const auto& pt : envelope) {
        if (!mid || std::abs(pt.x - L / 2.0) < std::abs(mid->x - L / 2.0)) mid = &pt;
    }
    double expected = P * L / 4.0;
    assert(approxEqual(mid->maxMoment, expected, 0.5));  // 0.5 kN*m tolerance for discretization
    std::cout << "  testSinglePointLoadMidspanMoment (expected=" << expected
              << ", got=" << mid->maxMoment << ") OK\n";
}

// Classic hand-calc: single point load P moving across span -> max
// shear anywhere = P (occurs as the load sits right at a support).
static void testSinglePointLoadMaxShear() {
    double L = 6.0, P = 40.0;
    std::vector<MovingPointLoad> train = {{P, 0.0}};
    auto envelope = runMovingLoadEnvelope(L, train, 31, 0.02);
    double maxAbsShear = 0.0;
    for (const auto& pt : envelope) maxAbsShear = std::max(maxAbsShear, std::abs(pt.maxShear));
    assert(approxEqual(maxAbsShear, P, 0.5));
    std::cout << "  testSinglePointLoadMaxShear OK\n";
}

static void testRejectsInvalidInputs() {
    bool threw = false;
    try { runMovingLoadEnvelope(-5.0, {{10.0, 0.0}}); }
    catch (const std::invalid_argument&) { threw = true; }
    assert(threw);

    threw = false;
    try { runMovingLoadEnvelope(10.0, {}); }
    catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  testRejectsInvalidInputs OK\n";
}

int main() {
    std::cout << "test_moving_load_analysis:\n";
    testMomentInfluenceOrdinateHandCalc();
    testShearInfluenceOrdinateJump();
    testSinglePointLoadMidspanMoment();
    testSinglePointLoadMaxShear();
    testRejectsInvalidInputs();
    std::cout << "All tests passed.\n";
    return 0;
}
