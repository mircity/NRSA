#include <cassert>
#include <cmath>
#include <iostream>

#include "analysis/TemperatureAnalysis.h"
#include "core/Element.h"
#include "core/Material.h"
#include "core/Model.h"
#include "core/Node.h"
#include "core/Section.h"

using namespace nrsa;
using namespace nrsa::analysis;

static bool approxEqual(double a, double b, double relTol = 1e-6) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

static void testAxialForceFormula() {
    // Hand-calc: F = E*A*eps0 directly.
    double E = 25.0e6, A = 0.15, eps0 = 1.0e-5;
    double expected = 25.0e6 * 0.15 * 1.0e-5;
    assert(approxEqual(axialForceFromInitialStrain(E, A, eps0), expected));
    std::cout << "  testAxialForceFormula OK\n";
}

// Horizontal member along global X -- FrameElement3D's default local
// axis convention gives local-x = global X directly for a horizontal
// member (see test_static_analysis.cpp's own comment on why a vertical
// member maps to Z instead), so the equivalent thermal load should
// land entirely on global Fx, split with opposite sign between the two
// end nodes.
static void testEquivalentLoadHandCalc() {
    Model model("temperature test");
    double L = 5.0;
    Node n1(1, 0, 0, 0), n2(2, L, 0, 0);
    n1.restrainAll();
    model.addNode(n1);
    model.addNode(n2);

    double E = 25.0e6;  // kPa
    Material conc(1, "C25", MaterialKind::Concrete, E, 0.2, 23.6);
    model.addMaterial(conc);
    Section sec = Section::rectangular(0.3, 0.5);
    double A = sec.area;
    int secId = model.addSection(sec);
    model.addElement(Element(1, ElementKind::Beam, {1, 2}, 1, secId, "B1"));

    double deltaT = 20.0;      // deg C rise
    double alpha = 1.0e-5;     // /deg C
    int loadCaseId = 1;
    auto results = applyUniformTemperatureChange(model, {1}, deltaT, alpha, loadCaseId);

    assert(results.size() == 1);
    double expectedF = E * A * alpha * deltaT;
    assert(approxEqual(results[0].axialForceMagnitude, expectedF));

    // Two nodal loads should have been appended, self-equilibrated,
    // pushing the ends apart (n1 gets -F in global X, n2 gets +F).
    assert(model.nodalLoads().size() == 2);
    double fx1 = 0.0, fx2 = 0.0;
    for (const auto& l : model.nodalLoads()) {
        assert(l.loadCaseId == loadCaseId);
        assert(approxEqual(l.Fy, 0.0, 1e-9));
        assert(approxEqual(l.Fz, 0.0, 1e-9));
        if (l.nodeId == 1) fx1 = l.Fx;
        if (l.nodeId == 2) fx2 = l.Fx;
    }
    assert(approxEqual(fx1, -expectedF));
    assert(approxEqual(fx2, expectedF));
    std::cout << "  testEquivalentLoadHandCalc OK\n";
}

static void testCoolingProducesOppositeSign() {
    Model model("cooling test");
    Node n1(1, 0, 0, 0), n2(2, 4.0, 0, 0);
    n1.restrainAll();
    model.addNode(n1);
    model.addNode(n2);
    Material steel = Material::structuralSteel(1, 345.0);
    model.addMaterial(steel);
    int secId = model.addSection(Section::rectangular(0.2, 0.2));
    model.addElement(Element(1, ElementKind::Brace, {1, 2}, 1, secId, "Br1"));

    auto results = applyUniformTemperatureChange(model, {1}, -15.0, 1.2e-5, 1);
    assert(results[0].axialForceMagnitude < 0.0);  // cooling -> negative (ends pulled together)
    std::cout << "  testCoolingProducesOppositeSign OK\n";
}

static void testRejectsNonFrameOrMissingElement() {
    Model model("bad element test");
    Node n1(1, 0, 0, 0);
    model.addNode(n1);
    bool threw = false;
    try {
        applyUniformTemperatureChange(model, {999}, 10.0, 1e-5, 1);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testRejectsNonFrameOrMissingElement OK\n";
}

int main() {
    std::cout << "test_temperature_analysis:\n";
    testAxialForceFormula();
    testEquivalentLoadHandCalc();
    testCoolingProducesOppositeSign();
    testRejectsNonFrameOrMissingElement();
    std::cout << "All tests passed.\n";
    return 0;
}
