#include <cassert>
#include <cmath>
#include <iostream>

#include "analysis/CreepShrinkageAnalysis.h"
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

static void testEffectiveModulusHandCalc() {
    // Hand-calc: Eeff = E/(1+phi). E.g. E=25 GPa, phi=2.0 -> Eeff = 25/3 GPa.
    double E = 25.0e6;
    assert(approxEqual(effectiveCreepModulus(E, 2.0), E / 3.0));
    assert(approxEqual(effectiveCreepModulus(E, 0.0), E));  // no creep -> unchanged
    std::cout << "  testEffectiveModulusHandCalc OK\n";
}

static void testEffectiveModulusRejectsNegativePhi() {
    bool threw = false;
    try { effectiveCreepModulus(25.0e6, -0.5); }
    catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  testEffectiveModulusRejectsNegativePhi OK\n";
}

// Shrinkage strain plays exactly the same role as alpha*deltaT in
// TemperatureAnalysis -- a NEGATIVE strain (shortening) should pull the
// ends TOGETHER, producing the opposite-sign equivalent load pattern
// from a temperature RISE.
static void testShrinkageEquivalentLoadHandCalc() {
    Model model("shrinkage test");
    double L = 6.0;
    Node n1(1, 0, 0, 0), n2(2, L, 0, 0);
    n1.restrainAll();
    model.addNode(n1);
    model.addNode(n2);

    double E = 24.0e6;
    Material conc(1, "C24", MaterialKind::Concrete, E, 0.2, 23.6);
    model.addMaterial(conc);
    Section sec = Section::rectangular(0.25, 0.4);
    double A = sec.area;
    int secId = model.addSection(sec);
    model.addElement(Element(1, ElementKind::Beam, {1, 2}, 1, secId, "B1"));

    double shrinkageStrain = -0.0004;  // typical BNBC/ACI order-of-magnitude estimate
    auto results = applyUniformShrinkage(model, {1}, shrinkageStrain, /*loadCaseId=*/2);

    assert(results.size() == 1);
    double expectedF = E * A * shrinkageStrain;  // negative
    assert(expectedF < 0.0);
    assert(approxEqual(results[0].axialForceMagnitude, expectedF));

    double fx1 = 0.0, fx2 = 0.0;
    for (const auto& l : model.nodalLoads()) {
        assert(l.loadCaseId == 2);
        if (l.nodeId == 1) fx1 = l.Fx;
        if (l.nodeId == 2) fx2 = l.Fx;
    }
    // Shrinkage (negative strain) => n1 gets +|F| (pulled toward n2), n2 gets -|F|.
    assert(approxEqual(fx1, -expectedF));
    assert(approxEqual(fx2, expectedF));
    assert(fx1 > 0.0);
    assert(fx2 < 0.0);
    std::cout << "  testShrinkageEquivalentLoadHandCalc OK\n";
}

int main() {
    std::cout << "test_creep_shrinkage_analysis:\n";
    testEffectiveModulusHandCalc();
    testEffectiveModulusRejectsNegativePhi();
    testShrinkageEquivalentLoadHandCalc();
    std::cout << "All tests passed.\n";
    return 0;
}
