#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "analysis/BucklingAnalysis.h"
#include "analysis/StaticAnalysis.h"
#include "core/Element.h"
#include "core/Load.h"
#include "core/Material.h"
#include "core/Model.h"
#include "core/Node.h"
#include "core/Section.h"
#include "fem/FrameElement3D.h"
#include "fem/Matrix.h"

using namespace nrsa;
using namespace nrsa::analysis;
using namespace nrsa::fem;

static bool approxEqual(double a, double b, double relTol = 1e-4) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

static void testBucklingLoadFactorMatchesDirectTwoDofQuadratic() {
    double E_kPa = 25.0e6, L = 4.0;
    Material conc(1, "test", MaterialKind::Concrete, E_kPa, 0.2, 23.6);
    Section sec;
    sec.area = 0.09;
    sec.momentOfInertiaZ = 0.3 * std::pow(0.3, 3) / 12.0;
    sec.momentOfInertiaY = sec.momentOfInertiaZ;
    sec.torsionalConstant = sec.momentOfInertiaZ * 2.0;
    double I = sec.momentOfInertiaZ;

    double referenceAxial = 500.0;

    Model model("cantilever buckling");
    Node base(1, 0, 0, 0), tip(2, 0, 0, L);
    base.restrainAll();
    tip.restrain(DOF::Uy);
    tip.restrain(DOF::Rx);
    tip.restrain(DOF::Rz);
    model.addNode(base);
    model.addNode(tip);
    model.addMaterial(conc);
    int secId = model.addSection(sec);
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));

    LoadCase gravity(1, "Gravity", LoadCaseType::Dead);
    model.addLoadCase(gravity);
    model.addNodalLoad(NodalLoad{2, 1, 0.0, 0.0, -referenceAxial, 0.0, 0.0, 0.0});

    BucklingAnalysis buckling(model);
    auto result = buckling.run(1);
    assert(result.skippedElementIds.empty());
    assert(!result.modes.empty());
    double computedPcr = result.modes[0].loadFactor * referenceAxial;

    FrameElement3D elem({0, 0, 0}, {L, 0, 0}, conc, sec);
    Matrix Ke = elem.localStiffness();
    Matrix Kg = elem.geometricStiffnessLocal(1.0);
    double Ke11 = Ke(7, 7), Ke12 = Ke(7, 11), Ke22 = Ke(11, 11);
    double Kg11 = Kg(7, 7), Kg12 = Kg(7, 11), Kg22 = Kg(11, 11);
    double a = Kg11 * Kg22 - Kg12 * Kg12;
    double b = Ke11 * Kg22 + Ke22 * Kg11 - 2.0 * Ke12 * Kg12;
    double c = Ke11 * Ke22 - Ke12 * Ke12;
    double disc = std::sqrt(b * b - 4.0 * a * c);
    double pRoot1 = (-b + disc) / (2.0 * a);
    double pRoot2 = (-b - disc) / (2.0 * a);
    double pCompressionRoot = std::abs(pRoot1) < std::abs(pRoot2) ? pRoot1 : pRoot2;
    double directPcr = -pCompressionRoot;

    assert(approxEqual(computedPcr, directPcr, 1e-6));

    double eulerPcr = M_PI * M_PI * E_kPa * I / (4.0 * L * L);
    assert(approxEqual(computedPcr, eulerPcr, 0.01));
    assert(computedPcr > eulerPcr);

    std::cout << "  testBucklingLoadFactorMatchesDirectTwoDofQuadratic OK (Pcr=" << computedPcr
              << " kN, direct=" << directPcr << " kN, Euler=" << eulerPcr
              << " kN, loadFactor=" << result.modes[0].loadFactor << ")\n";
}

static void testZeroReferenceLoadFindsNoBucklingModes() {
    double L = 4.0;
    Material conc = Material::concrete(1, 28.0);
    Section sec = Section::rectangular(0.3, 0.3);

    Model model("no reference load");
    Node base(1, 0, 0, 0), tip(2, 0, 0, L);
    base.restrainAll();
    tip.restrain(DOF::Uy);
    tip.restrain(DOF::Rx);
    tip.restrain(DOF::Rz);
    model.addNode(base);
    model.addNode(tip);
    model.addMaterial(conc);
    int secId = model.addSection(sec);
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));

    LoadCase gravity(1, "Gravity (empty)", LoadCaseType::Dead);
    model.addLoadCase(gravity);

    BucklingAnalysis buckling(model);
    auto result = buckling.run(1);
    assert(result.modes.empty());
    std::cout << "  testZeroReferenceLoadFindsNoBucklingModes OK\n";
}

int main() {
    std::cout << "test_buckling_analysis:\n";
    testBucklingLoadFactorMatchesDirectTwoDofQuadratic();
    testZeroReferenceLoadFindsNoBucklingModes();
    std::cout << "All buckling analysis tests passed.\n";
    return 0;
}
