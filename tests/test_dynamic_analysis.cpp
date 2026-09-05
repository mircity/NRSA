#include <cassert>
#include <cmath>
#include <iostream>

#include "analysis/DynamicAnalysis.h"
#include "core/Element.h"
#include "core/Material.h"
#include "core/Model.h"
#include "core/Node.h"
#include "core/Section.h"

using namespace nrsa;
using namespace nrsa::analysis;

static bool approxEqual(double a, double b, double relTol) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

static void testRayleighCoefficientsHandCalc() {
    // Hand-calc: solve the 2x2 system directly for a known pair and
    // verify the resulting (alpha,beta) reproduce xi at BOTH frequencies.
    double w1 = 5.0, w2 = 20.0, xi = 0.05;
    auto c = rayleighCoefficients(w1, w2, xi);
    double xi1 = c.alphaMass / (2.0 * w1) + c.betaStiffness * w1 / 2.0;
    double xi2 = c.alphaMass / (2.0 * w2) + c.betaStiffness * w2 / 2.0;
    assert(approxEqual(xi1, xi, 1e-9));
    assert(approxEqual(xi2, xi, 1e-9));
    std::cout << "  testRayleighCoefficientsHandCalc OK\n";
}

// Undamped SDOF cantilever ("shear building of one"): fixed-base
// vertical column, lumped mass at the free tip, restrained to move
// only along global X (lateral). Classic closed form for a SUDDENLY
// APPLIED constant force F0 on an undamped SDOF at rest:
//   d(t) = (F0/k) * (1 - cos(wn*t)),   wn = sqrt(k/m)
// with k = 3EI/L^3 (fixed-free cantilever lateral stiffness) -- the
// same closed-form family BucklingAnalysis/PDelta already lean on
// elsewhere in this project. Mass is in kN*s^2/m (the same unit
// ModalAnalysis's own tests use — see test_modal_analysis.cpp's
// comment on this project's kN-based mass convention), so k/m comes
// out directly in rad^2/s^2 with no separate unit conversion needed.
static void testUndampedStepLoadHandCalc() {
    Model model("dynamic sdof test");
    double L = 4.0;
    Node n1(1, 0, 0, 0), n2(2, 0, 0, L);
    n1.restrainAll();
    // Isolate SDOF behavior: only Ux free at the tip.
    n2.restrain(DOF::Uy, true);
    n2.restrain(DOF::Uz, true);
    n2.restrain(DOF::Rx, true);
    n2.restrain(DOF::Ry, true);
    n2.restrain(DOF::Rz, true);
    double mass = 5.0;  // kN*s^2/m
    n2.setTranslationalMass(mass);
    model.addNode(n1);
    model.addNode(n2);

    double E = 25.0e6;  // kPa
    Material conc(1, "C25", MaterialKind::Concrete, E, 0.2, 23.6);
    model.addMaterial(conc);
    Section sec = Section::rectangular(0.3, 0.5);
    double Iz = sec.momentOfInertiaZ;
    int secId = model.addSection(sec);
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));

    double k = 3.0 * E * Iz / std::pow(L, 3);  // kN/m
    double wn = std::sqrt(k / mass);           // rad/s
    double T = 2.0 * M_PI / wn;

    double F0 = 30.0;  // kN
    DynamicAnalysisOptions options;
    options.dtSeconds = T / 400.0;
    options.totalDurationSeconds = T;  // one full period
    options.damping = {0.0, 0.0};      // undamped

    ForcingFunction forcing = [F0](double, int n) {
        return std::vector<double>(static_cast<std::size_t>(n), F0);
    };

    auto result = runNewmarkBeta(model, forcing, options);
    assert(!result.steps.empty());

    auto closedForm = [&](double t) { return (F0 / k) * (1.0 - std::cos(wn * t)); };

    // Check displacement near t = T/4 and t = T/2 against the closed form.
    for (double tCheck : {T / 4.0, T / 2.0}) {
        const DynamicTimeStep* closest = nullptr;
        for (const auto& s : result.steps) {
            if (!closest || std::abs(s.time - tCheck) < std::abs(closest->time - tCheck)) closest = &s;
        }
        double got = closest->displacement.at(2).Ux;
        double expected = closedForm(closest->time);
        assert(approxEqual(got, expected, 0.02));  // 2% tolerance for Newmark discretization
    }
    std::cout << "  testUndampedStepLoadHandCalc OK\n";
}

static void testRejectsZeroFreeDofs() {
    Model model("fully restrained");
    Node n1(1, 0, 0, 0);
    n1.restrainAll();
    n1.setTranslationalMass(100.0);
    model.addNode(n1);
    ForcingFunction forcing = [](double, int n) { return std::vector<double>(static_cast<std::size_t>(n), 0.0); };
    DynamicAnalysisOptions options;
    bool threw = false;
    try { runNewmarkBeta(model, forcing, options); }
    catch (const std::runtime_error&) { threw = true; }
    assert(threw);
    std::cout << "  testRejectsZeroFreeDofs OK\n";
}

int main() {
    std::cout << "test_dynamic_analysis:\n";
    testRayleighCoefficientsHandCalc();
    testUndampedStepLoadHandCalc();
    testRejectsZeroFreeDofs();
    std::cout << "All tests passed.\n";
    return 0;
}
