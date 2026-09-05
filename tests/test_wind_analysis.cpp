#include <cassert>
#include <cmath>
#include <iostream>

#include "analysis/WindAnalysis.h"
#include "core/Model.h"
#include "core/Node.h"

using namespace nrsa;
using namespace nrsa::analysis;

static bool approxEqual(double a, double b, double relTol = 1e-3) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// Hand-calc: Kz at z = zg is exactly 2.01 (the power term becomes 1).
static void testKzAtGradientHeight() {
    ExposureConstants exposure = ExposureConstants::forCategory(ExposureCategory::C);
    assert(approxEqual(velocityPressureExposureCoefficient(exposure.zg, exposure), 2.01));
    std::cout << "  testKzAtGradientHeight OK\n";
}

// Hand-calc: below zmin, Kz is held constant at its zmin value (not
// allowed to decrease further, per ASCE 7 / BNBC 2020 convention).
static void testKzClampedBelowZmin() {
    ExposureConstants exposure = ExposureConstants::forCategory(ExposureCategory::C);
    double kzAtZmin = velocityPressureExposureCoefficient(exposure.zmin, exposure);
    double kzBelow = velocityPressureExposureCoefficient(0.5, exposure);
    assert(approxEqual(kzAtZmin, kzBelow));
    std::cout << "  testKzClampedBelowZmin OK\n";
}

// Hand-calc: qz = 0.613 * Kz * Kzt * Kd * V^2 * I, all factors = 1
// except Kz and V, so this reduces to 0.613*Kz*V^2 directly.
static void testVelocityPressureHandCalc() {
    WindParameters p;
    p.basicWindSpeedMs = 50.0;
    p.topographicFactor = 1.0;
    p.directionalityFactor = 1.0;
    p.importanceFactor = 1.0;
    p.exposure = ExposureCategory::C;

    ExposureConstants exposure = ExposureConstants::forCategory(ExposureCategory::C);
    double heightM = 20.0;
    double Kz = velocityPressureExposureCoefficient(heightM, exposure);
    double expected = 0.613 * Kz * 50.0 * 50.0;
    assert(approxEqual(velocityPressure(heightM, p), expected));
    std::cout << "  testVelocityPressureHandCalc OK\n";
}

// Full pipeline hand-calc on a uniform 3-story stack: with equal story
// heights, the two interior stories should get equal tributary height
// (one full story height each) and the top/bottom stories half that,
// so total tributary height across all stories must equal the total
// building height.
static void testTributaryHeightsSumToTotalHeight() {
    Model model("wind test");
    Node n0(1, 0, 0, 0);
    model.addNode(n0);
    Node n1(2, 0, 0, 3.0);
    model.addNode(n1);
    Node n2(3, 0, 0, 6.0);
    model.addNode(n2);
    Node n3(4, 0, 0, 9.0);
    model.addNode(n3);

    WindParameters p;
    p.basicWindSpeedMs = 47.0;
    p.tributaryWidthM = 10.0;
    auto result = runStaticWind(model, p);

    assert(result.storyForces.size() == 4);
    // Recompute implied tributary heights from force/pressure/width and
    // sum them — should equal 9.0 m (base to roof) exactly.
    double sumTrib = 0.0;
    for (const auto& s : result.storyForces) {
        double netCp = p.windwardCp - p.leewardCp;
        double pressure = s.velocityPressurePa * p.gustFactor * netCp;
        double trib = s.force / (pressure * p.tributaryWidthM);
        sumTrib += trib;
    }
    assert(approxEqual(sumTrib, 9.0));
    std::cout << "  testTributaryHeightsSumToTotalHeight OK\n";
}

// Pressure increases monotonically with height (Kz increases with z),
// so story force should also increase with elevation for equal
// tributary heights... verify at least qz itself is monotonic.
static void testVelocityPressureIncreasesWithHeight() {
    WindParameters p;
    double q1 = velocityPressure(3.0, p);
    double q2 = velocityPressure(30.0, p);
    assert(q2 > q1);
    std::cout << "  testVelocityPressureIncreasesWithHeight OK\n";
}

static void testApplyAsNodalLoadsSplitsEvenly() {
    Model model("wind apply test");
    Node n0(1, 0, 0, 0);
    model.addNode(n0);
    Node n1(2, 0, 0, 3.0);
    model.addNode(n1);
    Node n1b(3, 5, 0, 3.0);  // same story as n1
    model.addNode(n1b);

    WindParameters p;
    auto result = runStaticWind(model, p);
    applyAsNodalLoads(model, result, /*loadCaseId=*/2, /*directionX=*/false);

    double total = 0.0;
    for (const auto& l : model.nodalLoads()) {
        assert(l.loadCaseId == 2);
        assert(l.Fx == 0.0);
        total += l.Fy;
    }
    assert(approxEqual(total, result.totalForce));
    std::cout << "  testApplyAsNodalLoadsSplitsEvenly OK\n";
}

static void testRejectsSingleStoryModel() {
    Model model("single story");
    Node n(1, 0, 0, 0);
    model.addNode(n);
    WindParameters p;
    bool threw = false;
    try {
        runStaticWind(model, p);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testRejectsSingleStoryModel OK\n";
}

int main() {
    std::cout << "test_wind_analysis:\n";
    testKzAtGradientHeight();
    testKzClampedBelowZmin();
    testVelocityPressureHandCalc();
    testTributaryHeightsSumToTotalHeight();
    testVelocityPressureIncreasesWithHeight();
    testApplyAsNodalLoadsSplitsEvenly();
    testRejectsSingleStoryModel();
    std::cout << "All tests passed.\n";
    return 0;
}
