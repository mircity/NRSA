#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "design/RCCBeam.h"

using namespace nrsa::design;

static bool approxEqual(double a, double b, double relTol = 0.01) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// Independent hand-calc, textbook ACI 318-19 Whitney-block method:
// b=300mm, d=450mm, fc'=28MPa, fy=420MPa, Mu=200 kN*m.
//   Rn = Mu/(phi*b*d^2) = 200e6/(0.9*300*450^2) = 3.658 MPa
//   rho = (0.85fc'/fy)*(1-sqrt(1-2Rn/(0.85fc'))) = 0.009507
//   As = rho*b*d = 1283.4 mm^2
// Computed independently (not copied from the module under test) and
// cross-checked against designFlexure's actual output.
static void testFlexureHandCalc() {
    auto result = designFlexure(200.0, 0.3, 0.45, 28.0, 420.0);
    double expectedRho = 0.009507;
    double expectedAs = expectedRho * 300.0 * 450.0;  // 1283.4 mm^2
    assert(approxEqual(result.rhoProvided, expectedRho, 0.005));
    assert(approxEqual(result.asRequiredMm2, expectedAs, 0.005));
    assert(!result.governedByMinimum);
    assert(!result.exceedsMaximum);

    double expectedRhoMin = std::max(1.4 / 420.0, std::sqrt(28.0) / (4.0 * 420.0));
    assert(approxEqual(result.rhoMin, expectedRhoMin, 1e-6));
    std::cout << "  testFlexureHandCalc (expected As=" << expectedAs
              << " mm2, got=" << result.asRequiredMm2 << " mm2) OK\n";
}

// Zero-moment case: must be governed entirely by ACI 318-19
// Section 9.6.1.2 minimum reinforcement, rho_min * b * d.
static void testFlexureZeroMomentGovernedByMinimum() {
    auto result = designFlexure(0.0, 0.3, 0.45, 28.0, 420.0);
    assert(result.governedByMinimum);
    double expectedRhoMin = std::max(1.4 / 420.0, std::sqrt(28.0) / (4.0 * 420.0));
    assert(approxEqual(result.rhoProvided, expectedRhoMin, 1e-6));
    assert(approxEqual(result.asRequiredMm2, expectedRhoMin * 300.0 * 450.0, 1e-3));
    std::cout << "  testFlexureZeroMomentGovernedByMinimum OK\n";
}

// An enormous moment demand on a small section must throw (exceeds
// what ANY singly-reinforced section of this size can carry).
static void testFlexureRejectsImpossibleDemand() {
    bool threw = false;
    try { designFlexure(5000.0, 0.2, 0.3, 21.0, 420.0); }
    catch (const std::runtime_error&) { threw = true; }
    assert(threw);
    std::cout << "  testFlexureRejectsImpossibleDemand OK\n";
}

// Independent hand-calc for shear, ACI 318-19 Eq. 22.5.5.1 (lambda=1):
// b=300mm, d=450mm, fc'=28MPa, fy=420MPa, Vu=150kN, 10mm 2-leg stirrups.
//   Vc = 0.17*sqrt(28)*300*450/1000 = 121.44 kN
//   phi*Vc/2 = 45.54 kN < Vu -> stirrups required
//   Vs_required = Vu/phi - Vc = 200 - 121.44 = 78.56 kN
//   Av = 2*(pi/4)*10^2 = 157.08 mm^2
//   s_required = Av*fy*d/(Vs_required*1000) = 378.0 mm
//   maxSpacing = min(d/2, 600) = 225 mm (governs, since 378 > 225)
static void testShearHandCalc() {
    auto result = designShear(150.0, 0.3, 0.45, 28.0, 420.0, 10.0, 2);
    double expectedVc = 0.17 * std::sqrt(28.0) * 300.0 * 450.0 / 1000.0;
    assert(approxEqual(result.vcKN, expectedVc, 0.005));
    assert(result.stirrupsRequired);
    assert(approxEqual(result.maxSpacingMm, 225.0, 0.01));
    assert(approxEqual(result.requiredSpacingMm, 225.0, 0.01));  // maxSpacing governs over s_required=378mm
    assert(!result.exceedsMaximumVs);
    std::cout << "  testShearHandCalc (expected Vc=" << expectedVc << " kN) OK\n";
}

// Very low shear (below phi*Vc/2): no stirrups required at all.
static void testShearBelowThresholdNoStirrups() {
    auto result = designShear(20.0, 0.3, 0.45, 28.0, 420.0, 10.0, 2);
    assert(!result.stirrupsRequired);
    assert(approxEqual(result.requiredSpacingMm, 0.0, 1e-9));
    std::cout << "  testShearBelowThresholdNoStirrups OK\n";
}

static void testRejectsInvalidInputs() {
    bool threw = false;
    try { designFlexure(-5.0, 0.3, 0.45, 28.0, 420.0); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    threw = false;
    try { designShear(50.0, 0.3, 0.45, 28.0, 420.0, 10.0, 1); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);  // legs must be >= 2
    std::cout << "  testRejectsInvalidInputs OK\n";
}

int main() {
    std::cout << "test_rcc_beam:\n";
    testFlexureHandCalc();
    testFlexureZeroMomentGovernedByMinimum();
    testFlexureRejectsImpossibleDemand();
    testShearHandCalc();
    testShearBelowThresholdNoStirrups();
    testRejectsInvalidInputs();
    std::cout << "All tests passed.\n";
    return 0;
}
