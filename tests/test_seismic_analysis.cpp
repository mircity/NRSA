#include <cassert>
#include <cmath>
#include <iostream>

#include "analysis/SeismicAnalysis.h"
#include "core/Model.h"
#include "core/Node.h"

using namespace nrsa;
using namespace nrsa::analysis;

static bool approxEqual(double a, double b, double relTol = 1e-3) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// Hand-calc: normalizedSpectralShape at the plateau (TB <= T <= TC)
// should just be S * 2.5 * eta, independent of T.
static void testPlateauShape() {
    SiteCoefficients site{1.15, 0.20, 0.6, 2.0};
    double eta = dampingCorrectionFactor(5.0);  // 5% damping -> eta = 1.0 exactly
    assert(approxEqual(eta, 1.0));
    assert(approxEqual(normalizedSpectralShape(0.3, site, eta), 1.15 * 2.5));
    assert(approxEqual(normalizedSpectralShape(0.6, site, eta), 1.15 * 2.5));  // right at TC
    std::cout << "  testPlateauShape OK\n";
}

// Hand-calc: at T=0, shape = S * (1 + 0) = S.
static void testShapeAtZeroPeriod() {
    SiteCoefficients site{1.2, 0.15, 0.5, 2.0};
    double eta = dampingCorrectionFactor(5.0);
    assert(approxEqual(normalizedSpectralShape(0.0, site, eta), 1.2));
    std::cout << "  testShapeAtZeroPeriod OK\n";
}

// Hand-calc: beyond TD, shape decays as S*2.5*eta*(TC*TD/T^2).
static void testDecayBeyondTD() {
    SiteCoefficients site{1.15, 0.20, 0.6, 2.0};
    double eta = 1.0;
    double T = 3.0;
    double expected = 1.15 * 2.5 * 1.0 * (0.6 * 2.0 / (3.0 * 3.0));
    assert(approxEqual(normalizedSpectralShape(T, site, eta), expected));
    std::cout << "  testDecayBeyondTD OK\n";
}

// Full pipeline hand-calc: a simple 2-story stack (base + one mass
// level) with a period forced onto the plateau via overridePeriodSeconds,
// so the base shear reduces to a closed-form V = (2/3)*Z*I/R*S*2.5*eta*W.
static void testBaseShearHandCalc() {
    Model model("seismic test");
    // Base (restrained, zero mass) at z=0, one mass level at z=3m.
    Node base(1, 0, 0, 0);
    base.restrainAll();
    model.addNode(base);
    Node roof(2, 0, 0, 3.0);
    double massKg = 50000.0;  // 50 t lumped mass
    roof.setTranslationalMass(massKg);
    model.addNode(roof);

    SeismicParameters params;
    params.Z = 0.20;
    params.I = 1.0;
    params.R = 5.0;
    params.dampingRatioPercent = 5.0;
    params.site = SiteCoefficients{1.15, 0.20, 0.6, 2.0};
    params.overridePeriodSeconds = 0.3;  // forced onto the plateau (TB=0.2, TC=0.6)

    auto result = runEquivalentStaticSeismic(model, params);

    double eta = 1.0;
    double Cs = 1.15 * 2.5 * eta;  // plateau value
    double expectedSa = (2.0 / 3.0) * 0.20 * 1.0 / 5.0 * Cs;
    assert(approxEqual(result.designSpectralAcceleration, expectedSa));

    double expectedW = massKg * 9.81;
    assert(approxEqual(result.totalSeismicWeight, expectedW));

    double expectedV = expectedSa * expectedW;
    assert(approxEqual(result.baseShear, expectedV));

    // Only one story has mass (the base is massless), so it must carry
    // the entire base shear.
    assert(result.storyForces.size() == 2);
    assert(approxEqual(result.storyForces[0].weight, 0.0, 1e-6));
    assert(approxEqual(result.storyForces[1].force, expectedV));

    std::cout << "  testBaseShearHandCalc OK\n";
}

// Vertical distribution hand-calc across three equally-massed stories:
// Fx proportional to wx*hx^k. With k=1 (T<=0.5s) and equal masses at
// heights 3,6,9m, the ratio of forces should be exactly 3:6:9 = 1:2:3.
static void testVerticalDistributionRatio() {
    Model model("distribution test");
    Node base(1, 0, 0, 0);
    base.restrainAll();
    model.addNode(base);
    for (int i = 1; i <= 3; ++i) {
        Node n(i + 1, 0, 0, 3.0 * i);
        n.setTranslationalMass(10000.0);  // equal mass every story
        model.addNode(n);
    }

    SeismicParameters params;
    params.overridePeriodSeconds = 0.3;  // < 0.5s -> k = 1
    params.site = SiteCoefficients{1.15, 0.20, 0.6, 2.0};

    auto result = runEquivalentStaticSeismic(model, params);
    // storyForces[0] is the massless base; [1..3] are the three levels.
    double f1 = result.storyForces[1].force;
    double f2 = result.storyForces[2].force;
    double f3 = result.storyForces[3].force;
    assert(approxEqual(f2 / f1, 2.0));
    assert(approxEqual(f3 / f1, 3.0));
    // And they must sum to the base shear.
    assert(approxEqual(f1 + f2 + f3, result.baseShear));
    std::cout << "  testVerticalDistributionRatio OK\n";
}

static void testApplyAsNodalLoadsSplitsEvenly() {
    Model model("apply test");
    Node base(1, 0, 0, 0);
    base.restrainAll();
    model.addNode(base);
    Node n2(2, 0, 0, 3.0);
    n2.setTranslationalMass(10000.0);
    model.addNode(n2);
    Node n3(3, 5, 0, 3.0);  // same story as n2
    n3.setTranslationalMass(10000.0);
    model.addNode(n3);

    SeismicParameters params;
    params.overridePeriodSeconds = 0.3;
    params.site = SiteCoefficients{1.15, 0.20, 0.6, 2.0};
    auto result = runEquivalentStaticSeismic(model, params);

    int loadCaseId = 1;
    applyAsNodalLoads(model, result, loadCaseId, /*directionX=*/true);
    assert(model.nodalLoads().size() == 2);
    double total = 0.0;
    for (const auto& l : model.nodalLoads()) {
        assert(l.loadCaseId == loadCaseId);
        assert(l.Fy == 0.0);
        total += l.Fx;
    }
    // topmost story force split equally between its two nodes
    double storyForce = 0.0;
    for (const auto& s : result.storyForces)
        if (s.nodeIds.size() == 2) storyForce = s.force;
    assert(approxEqual(total, storyForce));
    std::cout << "  testApplyAsNodalLoadsSplitsEvenly OK\n";
}

static void testRejectsSingleStoryModel() {
    Model model("single story");
    Node n(1, 0, 0, 0);
    n.setTranslationalMass(1000.0);
    model.addNode(n);
    SeismicParameters params;
    bool threw = false;
    try {
        runEquivalentStaticSeismic(model, params);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testRejectsSingleStoryModel OK\n";
}

// Regression lock: BNBC 2020 Table 6.2.16 was fetched directly from the
// official code text (2026-08-27) and every site coefficient below was
// confirmed to match this project's hardcoded SiteCoefficients::forSoilType
// exactly. This test pins those exact cited values so a future edit
// that silently drifts from the verified table gets caught immediately.
static void testSiteCoefficientsMatchBNBC2020Table6216() {
    auto check = [](SoilType t, double S, double TB, double TC, double TD) {
        auto c = SiteCoefficients::forSoilType(t);
        assert(approxEqual(c.S, S));
        assert(approxEqual(c.TB, TB));
        assert(approxEqual(c.TC, TC));
        assert(approxEqual(c.TD, TD));
    };
    check(SoilType::SA, 1.0, 0.15, 0.40, 2.0);
    check(SoilType::SB, 1.2, 0.15, 0.50, 2.0);
    check(SoilType::SC, 1.15, 0.20, 0.60, 2.0);
    check(SoilType::SD, 1.35, 0.20, 0.80, 2.0);
    check(SoilType::SE, 1.4, 0.15, 0.50, 2.0);
    std::cout << "  testSiteCoefficientsMatchBNBC2020Table6216 OK\n";
}

// Same regression lock for BNBC 2020 Table 6.2.20 (period coefficients).
static void testPeriodCoefficientsMatchBNBC2020Table6220() {
    auto conc = PeriodCoefficients::concreteMomentFrame();
    assert(approxEqual(conc.Ct, 0.0466));
    assert(approxEqual(conc.m, 0.9));
    auto steel = PeriodCoefficients::steelMomentFrame();
    assert(approxEqual(steel.Ct, 0.0724));
    assert(approxEqual(steel.m, 0.8));
    auto other = PeriodCoefficients::other();
    assert(approxEqual(other.Ct, 0.0488));
    assert(approxEqual(other.m, 0.75));
    std::cout << "  testPeriodCoefficientsMatchBNBC2020Table6220 OK\n";
}

int main() {
    std::cout << "test_seismic_analysis:\n";
    testPlateauShape();
    testShapeAtZeroPeriod();
    testDecayBeyondTD();
    testBaseShearHandCalc();
    testVerticalDistributionRatio();
    testApplyAsNodalLoadsSplitsEvenly();
    testRejectsSingleStoryModel();
    testSiteCoefficientsMatchBNBC2020Table6216();
    testPeriodCoefficientsMatchBNBC2020Table6220();
    std::cout << "All tests passed.\n";
    return 0;
}
