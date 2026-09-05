#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "analysis/ModalAnalysis.h"
#include "core/Element.h"
#include "core/Material.h"
#include "core/Model.h"
#include "core/Node.h"
#include "core/Section.h"

using namespace nrsa;
using namespace nrsa::analysis;

static bool approxEqual(double a, double b, double relTol = 1e-3) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// A single-story "shear building" idealization: one lumped mass at roof
// level on two identical columns, base fixed. This is THE textbook
// single-DOF (lateral) modal case: omega = sqrt(k_lateral / m), with
// k_lateral = sum of each column's lateral (shear-building) stiffness
// 12*E*I/L^3 (both ends effectively fixed against rotation by the rigid
// roof mass idealization... — to make this cleanly 1-DOF and match the
// textbook 12EI/L^3 formula exactly, the roof-level nodes are given a
// large rotational mass here specifically so the roof genuinely
// translates without rotating, matching the shear-building assumption
// the closed form itself depends on).
static void testSingleStoryShearBuildingAgainstClosedForm() {
    double H = 3.0;
    double E_kPa = 25.0e6;
    // Built with zero shear area (pure Euler-Bernoulli — see
    // FrameElement3D's own doc comment on how a Section with no shear
    // area degrades cleanly to Euler-Bernoulli) specifically so the
    // classic 12*E*I/L^3 shear-building formula applies exactly; the
    // Timoshenko shear correction Section::rectangular() would normally
    // include is real and legitimate (it measurably softens a stocky
    // column like this one), but comparing against it would mean
    // deriving a corrected closed form instead of using the textbook
    // one directly.
    Section colSec;
    colSec.width = colSec.depth = 0.3;
    colSec.area = 0.3 * 0.3;
    colSec.momentOfInertiaZ = colSec.momentOfInertiaY = 0.3 * std::pow(0.3, 3) / 12.0;
    colSec.torsionalConstant = colSec.momentOfInertiaZ * 2.0;  // not used by this test
    double I = colSec.momentOfInertiaZ;
    double mass = 10.0;  // kN*s^2/m, lumped entirely at the roof node

    // A SINGLE column, base fixed, roof node restrained against
    // everything except lateral (Ux) translation — genuinely 1-DOF, no
    // degenerate eigenspace to worry about (two identical, uncoupled
    // columns each carrying their own mass would give a perfectly
    // degenerate pair of equal-frequency modes whose individual shapes
    // are then basis-dependent — not a meaningful thing to assert on).
    Model model("single column shear building");
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

    ModalAnalysis modal(model);
    auto result = modal.run();
    assert(result.skippedElementIds.empty());
    assert(result.modes.size() == 1);  // exactly one free DOF

    double k = 12.0 * E_kPa * I / std::pow(H, 3);
    double expectedFreqHz = std::sqrt(k / mass) / (2.0 * M_PI);
    assert(approxEqual(result.modes[0].naturalFrequencyHz, expectedFreqHz, 1e-6));

    std::cout << "  testSingleStoryShearBuildingAgainstClosedForm OK (f1="
              << result.modes[0].naturalFrequencyHz << " Hz, expected=" << expectedFreqHz << " Hz)\n";
}

static void testModesAreSortedAscending() {
    Model model("multi-mode sanity");
    double H = 3.0;
    Node base(1, 0, 0, 0), top(2, 0, 0, H);
    base.restrainAll();
    top.setTranslationalMass(5.0);
    model.addNode(base);
    model.addNode(top);
    Material conc = Material::concrete(1, 28.0);
    model.addMaterial(conc);
    int secId = model.addSection(Section::rectangular(0.3, 0.3));
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));

    ModalAnalysis modal(model);
    auto result = modal.run();
    for (std::size_t i = 1; i < result.modes.size(); ++i) {
        assert(result.modes[i].eigenvalue >= result.modes[i - 1].eigenvalue);
    }
    std::cout << "  testModesAreSortedAscending OK (" << result.modes.size() << " modes)\n";
}

static void testRejectsModelWithNoMass() {
    Model model("no mass assigned");
    Node base(1, 0, 0, 0), top(2, 0, 0, 3.0);  // no mass set on 'top'
    base.restrainAll();
    model.addNode(base);
    model.addNode(top);
    Material conc = Material::concrete(1, 28.0);
    model.addMaterial(conc);
    int secId = model.addSection(Section::rectangular(0.3, 0.3));
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));

    ModalAnalysis modal(model);
    bool threw = false;
    try {
        modal.run();
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testRejectsModelWithNoMass OK\n";
}

static void testRotationalRegularizationDoesNotDistortTranslationalMode() {
    // Run the same single-column cantilever twice: once with the
    // default (tiny) rotational-mass regularization, once with an even
    // SMALLER one — the lowest (translational) mode's frequency should
    // barely change, confirming the regularization is negligible for
    // the real, mass-governed mode rather than silently participating
    // in it.
    Model model("regularization sensitivity");
    Node base(1, 0, 0, 0), top(2, 0, 0, 3.0);
    base.restrainAll();
    top.setTranslationalMass(8.0);
    model.addNode(base);
    model.addNode(top);
    Material conc = Material::concrete(1, 28.0);
    model.addMaterial(conc);
    int secId = model.addSection(Section::rectangular(0.3, 0.3));
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));

    ModalAnalysis modal(model);
    ModalAnalysis::Options optsDefault;
    ModalAnalysis::Options optsTiny;
    optsTiny.rotationalMassRegularization = 1e-9;

    Model modelCopyForSecondRun = model;  // Model is copyable (value-type collections)
    ModalAnalysis modalA(model);
    ModalAnalysis modalB(modelCopyForSecondRun);
    auto resultA = modalA.run(optsDefault);
    auto resultB = modalB.run(optsTiny);

    assert(approxEqual(resultA.modes[0].naturalFrequencyHz, resultB.modes[0].naturalFrequencyHz, 1e-4));
    std::cout << "  testRotationalRegularizationDoesNotDistortTranslationalMode OK\n";
}

int main() {
    std::cout << "test_modal_analysis:\n";
    testSingleStoryShearBuildingAgainstClosedForm();
    testModesAreSortedAscending();
    testRejectsModelWithNoMass();
    testRotationalRegularizationDoesNotDistortTranslationalMode();
    std::cout << "All modal analysis tests passed.\n";
    return 0;
}
