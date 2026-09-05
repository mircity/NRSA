#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "analysis/NonlinearAnalysis.h"
#include "analysis/PDelta.h"
#include "core/Element.h"
#include "core/Load.h"
#include "core/Material.h"
#include "core/Model.h"
#include "core/Node.h"
#include "core/Section.h"

using namespace nrsa;
using namespace nrsa::analysis;

static bool approxEqual(double a, double b, double relTol = 1e-6) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// THE decisive cross-check: the same cantilever-column model used in
// tests/test_pdelta.cpp, where axial force is EXACTLY decoupled from
// the lateral load (a straight prismatic member's elastic local
// stiffness has no axial-transverse coupling -- only the geometric
// stiffness affects the transverse DOFs, and that itself doesn't
// change what the axial force IS, only how much the transverse DOFs
// resist). Because of that decoupling, NonlinearAnalysis (fed gravity
// + lateral load together in ONE case) and PDeltaAnalysis (fed the
// SAME loads but split into two cases) must converge to EXACTLY the
// same displacement -- not "closely", exactly, since there is no real
// coupling for the two methods' different load-splitting to create any
// difference through. A mismatch here would mean a genuine bug in one
// class or the other.
static void testMatchesPDeltaExactlyForDecoupledModel() {
    double L = 4.0;
    Material conc(1, "test", MaterialKind::Concrete, 25.0e6, 0.2, 23.6);
    Section sec;
    sec.area = 0.09;
    sec.momentOfInertiaZ = 0.3 * std::pow(0.3, 3) / 12.0;
    sec.momentOfInertiaY = sec.momentOfInertiaZ;
    sec.torsionalConstant = sec.momentOfInertiaZ * 2.0;

    double axialCompression = 1500.0, H = 10.0;

    // Model A: PDeltaAnalysis, gravity and lateral in separate cases
    // (matching test_pdelta.cpp exactly).
    Model modelA("pdelta reference");
    {
        Node base(1, 0, 0, 0), tip(2, 0, 0, L);
        base.restrainAll();
        tip.restrain(DOF::Uy); tip.restrain(DOF::Rx); tip.restrain(DOF::Rz);
        modelA.addNode(base);
        modelA.addNode(tip);
        modelA.addMaterial(conc);
        int secId = modelA.addSection(sec);
        modelA.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));
        modelA.addLoadCase(LoadCase(1, "Gravity", LoadCaseType::Dead));
        modelA.addLoadCase(LoadCase(2, "Lateral", LoadCaseType::Wind));
        modelA.addNodalLoad(NodalLoad{2, 1, 0.0, 0.0, -axialCompression, 0.0, 0.0, 0.0});
        modelA.addNodalLoad(NodalLoad{2, 2, H, 0.0, 0.0, 0.0, 0.0, 0.0});
    }
    PDeltaAnalysis pdelta(modelA);
    auto pdeltaResult = pdelta.run(1, 2);

    // Model B: NonlinearAnalysis, gravity AND lateral combined into a
    // SINGLE load case.
    Model modelB("nonlinear combined");
    {
        Node base(1, 0, 0, 0), tip(2, 0, 0, L);
        base.restrainAll();
        tip.restrain(DOF::Uy); tip.restrain(DOF::Rx); tip.restrain(DOF::Rz);
        modelB.addNode(base);
        modelB.addNode(tip);
        modelB.addMaterial(conc);
        int secId = modelB.addSection(sec);
        modelB.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));
        modelB.addLoadCase(LoadCase(1, "Combined", LoadCaseType::Other));
        modelB.addNodalLoad(NodalLoad{2, 1, H, 0.0, -axialCompression, 0.0, 0.0, 0.0});
    }
    NonlinearAnalysis nonlinear(modelB);
    auto nlResult = nonlinear.run(1);

    assert(approxEqual(nlResult.staticResult.displacements[2].Ux,
                        pdeltaResult.displacements[2].Ux, 1e-5));
    assert(nlResult.staticResult.maxFreeDofResidualNorm < 1e-4);
    // For this fully decoupled case, convergence should be essentially
    // immediate: iteration 1 establishes the correct axial force from a
    // Ke-only solve, iteration 2 confirms it doesn't change further.
    assert(nlResult.iterations <= 3);

    std::cout << "  testMatchesPDeltaExactlyForDecoupledModel OK (u_nl="
              << nlResult.staticResult.displacements[2].Ux << "m, u_pdelta="
              << pdeltaResult.displacements[2].Ux << "m, iterations=" << nlResult.iterations << ")\n";
}

static void testZeroLoadGivesZeroDisplacement() {
    Material conc = Material::concrete(1, 28.0);
    Section sec = Section::rectangular(0.3, 0.3);
    Model model("zero load");
    Node base(1, 0, 0, 0), tip(2, 0, 0, 3.0);
    base.restrainAll();
    tip.restrain(DOF::Uy); tip.restrain(DOF::Rx); tip.restrain(DOF::Rz);
    model.addNode(base);
    model.addNode(tip);
    model.addMaterial(conc);
    int secId = model.addSection(sec);
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));
    model.addLoadCase(LoadCase(1, "Empty", LoadCaseType::Other));
    // No loads at all.

    NonlinearAnalysis nonlinear(model);
    auto result = nonlinear.run(1);
    assert(std::abs(result.staticResult.displacements[2].Ux) < 1e-9);
    assert(std::abs(result.staticResult.displacements[2].Uz) < 1e-9);
    std::cout << "  testZeroLoadGivesZeroDisplacement OK (converged in "
              << result.iterations << " iterations)\n";
}

static void testDivergesNearBucklingLoad() {
    // Load the same column with an axial force very close to its
    // known buckling load (~2622 kN, from test_frame_element.cpp's
    // Euler cantilever check) plus a lateral disturbance -- the
    // structure is effectively unstable here, so the fixed-point
    // iteration should FAIL to converge (or the linear solve inside an
    // iteration should itself detect non-positive-definiteness) rather
    // than silently returning a plausible-looking but meaningless
    // number.
    double L = 4.0;
    Material conc(1, "test", MaterialKind::Concrete, 25.0e6, 0.2, 23.6);
    Section sec;
    sec.area = 0.09;
    sec.momentOfInertiaZ = 0.3 * std::pow(0.3, 3) / 12.0;
    sec.momentOfInertiaY = sec.momentOfInertiaZ;
    sec.torsionalConstant = sec.momentOfInertiaZ * 2.0;

    Model model("near buckling");
    Node base(1, 0, 0, 0), tip(2, 0, 0, L);
    base.restrainAll();
    tip.restrain(DOF::Uy); tip.restrain(DOF::Rx); tip.restrain(DOF::Rz);
    model.addNode(base);
    model.addNode(tip);
    model.addMaterial(conc);
    int secId = model.addSection(sec);
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));
    model.addLoadCase(LoadCase(1, "Combined", LoadCaseType::Other));
    double axialCompression = 2700.0;  // just above the ~2622 kN buckling load
    model.addNodalLoad(NodalLoad{2, 1, 1.0, 0.0, -axialCompression, 0.0, 0.0, 0.0});

    NonlinearAnalysis nonlinear(model);
    bool threw = false;
    try {
        nonlinear.run(1);
    } catch (const std::exception&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testDivergesNearBucklingLoad OK\n";
}

int main() {
    std::cout << "test_nonlinear_analysis:\n";
    testMatchesPDeltaExactlyForDecoupledModel();
    testZeroLoadGivesZeroDisplacement();
    testDivergesNearBucklingLoad();
    std::cout << "All nonlinear analysis tests passed.\n";
    return 0;
}
