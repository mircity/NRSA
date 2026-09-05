#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "analysis/ResponseSpectrum.h"
#include "core/Element.h"
#include "core/Material.h"
#include "core/Model.h"
#include "core/Node.h"
#include "core/Section.h"

using namespace nrsa;
using namespace nrsa::analysis;

static bool approxEqual(double a, double b, double relTol = 1e-4) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

static void testLinearInterpolation() {
    ResponseSpectrum spec({0.0, 1.0, 2.0}, {5.0, 10.0, 4.0});
    assert(approxEqual(spec.accelerationAt(0.5), 7.5));   // midpoint of first segment
    assert(approxEqual(spec.accelerationAt(1.5), 7.0));   // midpoint of second segment
    assert(approxEqual(spec.accelerationAt(0.0), 5.0));   // exact knot
    assert(approxEqual(spec.accelerationAt(-1.0), 5.0));  // below range -> hold first value
    assert(approxEqual(spec.accelerationAt(5.0), 4.0));   // above range -> hold last value
    std::cout << "  testLinearInterpolation OK\n";
}

static void testRejectsNonIncreasingPeriods() {
    bool threw = false;
    try {
        ResponseSpectrum bad({1.0, 0.5, 2.0}, {1.0, 2.0, 3.0});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testRejectsNonIncreasingPeriods OK\n";
}

// THE key validation: for a genuinely single-DOF (single-mode) system,
// the response spectrum method has an EXACT, well-known closed form —
// it must reduce precisely to the classic equivalent-static-force
// result: peak displacement = Sa(T)/omega^2 (since Gamma=1 exactly for
// a 1-DOF mass-normalized system: phi = 1/sqrt(m), Gamma = phi*m*1 =
// sqrt(m) ... let's just verify numerically against u = Sa(T)*m/k
// directly, the textbook static-equivalent formula, an independent
// closed form from how ResponseSpectrumAnalysis computes it
// internally).
static void testSingleModeReducesToEquivalentStaticForce() {
    double H = 3.0, E_kPa = 25.0e6, mass = 10.0;
    Section colSec;
    colSec.width = colSec.depth = 0.3;
    colSec.area = 0.09;
    colSec.momentOfInertiaZ = colSec.momentOfInertiaY = 0.3 * std::pow(0.3, 3) / 12.0;
    colSec.torsionalConstant = colSec.momentOfInertiaZ * 2.0;
    double I = colSec.momentOfInertiaZ;
    double k = 12.0 * E_kPa * I / std::pow(H, 3);

    Model model("single dof for RS");
    Node base(1, 0, 0, 0), roof(2, 0, 0, H);
    base.restrainAll();
    roof.restrain(DOF::Uy); roof.restrain(DOF::Uz);
    roof.restrain(DOF::Rx); roof.restrain(DOF::Ry); roof.restrain(DOF::Rz);
    roof.setTranslationalMass(mass);
    model.addNode(base);
    model.addNode(roof);
    Material conc(1, "test", MaterialKind::Concrete, E_kPa, 0.2, 23.6);
    model.addMaterial(conc);
    int secId = model.addSection(colSec);
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));

    double flatSa = 3.5;  // m/s^2, constant across all periods -> exact regardless of T
    ResponseSpectrum spectrum({0.0, 10.0}, {flatSa, flatSa});
    ResponseSpectrumAnalysis rsa(model, spectrum);
    ResponseSpectrumAnalysis::Options opts;
    opts.direction = 0;  // X
    auto result = rsa.run(opts);

    // Closed form: static-equivalent displacement u = Sa*m/k (from
    // F=Sa*m, u=F/k) — independent of Gamma/mass-normalization
    // bookkeeping, a direct physical check.
    double expectedDisp = flatSa * mass / k;
    assert(approxEqual(result.displacements[2].Ux, expectedDisp, 1e-6));

    // Closed form base shear for a 1-DOF system: V = Sa*m exactly
    // (100% mass participation).
    double expectedBaseShear = flatSa * mass;
    assert(approxEqual(result.baseShear, expectedBaseShear, 1e-6));

    // Mass participation must be ~100% for a genuinely 1-DOF system.
    assert(result.participation.size() >= 1);
    assert(approxEqual(result.participation[0].cumulativeMassRatio, 1.0, 1e-6));

    std::cout << "  testSingleModeReducesToEquivalentStaticForce OK (u="
              << result.displacements[2].Ux << "m, V=" << result.baseShear << "kN)\n";
}

static void testSRSSAndCQCAgreeWhenModesAreWellSeparatedAndUncorrelated() {
    // Two well-separated (far apart in frequency), fully DECOUPLED
    // single-DOF oscillators (independent columns, no shared node) —
    // decoupled AND far apart in period means their CQC cross-
    // correlation is negligible, so SRSS and CQC should agree closely.
    Model model("two well-separated oscillators");
    double H1 = 3.0, H2 = 8.0;  // very different heights -> very different periods
    double mass = 10.0, E_kPa = 25.0e6;
    Section colSec;
    colSec.width = colSec.depth = 0.3;
    colSec.area = 0.09;
    colSec.momentOfInertiaZ = colSec.momentOfInertiaY = 0.3 * std::pow(0.3, 3) / 12.0;
    colSec.torsionalConstant = colSec.momentOfInertiaZ * 2.0;

    Node base1(1, 0, 0, 0), base2(2, 10, 0, 0);
    Node top1(3, 0, 0, H1), top2(4, 10, 0, H2);
    base1.restrainAll(); base2.restrainAll();
    for (Node* n : {&top1, &top2}) {
        n->restrain(DOF::Uy); n->restrain(DOF::Uz);
        n->restrain(DOF::Rx); n->restrain(DOF::Ry); n->restrain(DOF::Rz);
        n->setTranslationalMass(mass);
    }
    model.addNode(base1); model.addNode(base2);
    model.addNode(top1); model.addNode(top2);
    Material conc(1, "test", MaterialKind::Concrete, E_kPa, 0.2, 23.6);
    model.addMaterial(conc);
    int secId = model.addSection(colSec);
    model.addElement(Element(1, ElementKind::Column, {1, 3}, 1, secId, "C1"));
    model.addElement(Element(2, ElementKind::Column, {2, 4}, 1, secId, "C2"));

    ResponseSpectrum spectrum({0.0, 10.0}, {4.0, 1.0});  // decreasing Sa with period, typical shape
    ResponseSpectrumAnalysis rsa(model, spectrum);
    ResponseSpectrumAnalysis::Options optsSRSS, optsCQC;
    optsSRSS.method = ModalCombinationMethod::SRSS;
    optsCQC.method = ModalCombinationMethod::CQC;
    auto resultSRSS = rsa.run(optsSRSS);
    auto resultCQC = rsa.run(optsCQC);

    assert(approxEqual(resultSRSS.baseShear, resultCQC.baseShear, 0.02));  // within 2%
    std::cout << "  testSRSSAndCQCAgreeWhenModesAreWellSeparatedAndUncorrelated OK (SRSS="
              << resultSRSS.baseShear << " CQC=" << resultCQC.baseShear << ")\n";
}

int main() {
    std::cout << "test_response_spectrum:\n";
    testLinearInterpolation();
    testRejectsNonIncreasingPeriods();
    testSingleModeReducesToEquivalentStaticForce();
    testSRSSAndCQCAgreeWhenModesAreWellSeparatedAndUncorrelated();
    std::cout << "All response spectrum tests passed.\n";
    return 0;
}
