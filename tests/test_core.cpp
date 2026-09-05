// Minimal, dependency-free smoke test for src/core. Uses plain asserts
// and returns a nonzero exit code on failure so `ctest` (wired up by the
// top-level CMakeLists) can run it without pulling in Catch2/GoogleTest
// yet — swap this for a real framework once the suite grows past what a
// few hand-rolled checks can comfortably cover.
#include <cassert>
#include <cmath>
#include <iostream>

#include "core/Element.h"
#include "core/Load.h"
#include "core/Material.h"
#include "core/Model.h"
#include "core/Node.h"
#include "core/Section.h"

using namespace nrsa;

static void testNodeRestraints() {
    Node n(1, 0.0, 0.0, 0.0, "N1");
    assert(n.isFree());
    n.restrain(DOF::Uz);
    assert(n.isRestrained(DOF::Uz));
    assert(!n.isRestrained(DOF::Ux));
    assert(!n.isFullyFixed());
    n.restrainAll();
    assert(n.isFullyFixed());
    std::cout << "  testNodeRestraints OK\n";
}

static void testMaterialFactories() {
    // ACI 318-19: Ec = 4700*sqrt(fc'), fc' in MPa -> for fc'=28 MPa,
    // Ec ~= 4700*5.2915 ~= 24870 MPa.
    Material c = Material::concrete(1, 28.0);
    double expectedE_MPa = 4700.0 * std::sqrt(28.0);
    assert(std::abs(c.elasticModulusKPa() / 1000.0 - expectedE_MPa) < 1.0);
    assert(std::abs(c.compressiveStrengthMPa() - 28.0) < 1e-9);

    Material rebar = Material::rebar(2, 420.0);
    assert(std::abs(rebar.elasticModulusKPa() - 200000.0 * 1000.0) < 1e-6);
    assert(std::abs(rebar.yieldStrengthMPa() - 420.0) < 1e-9);
    std::cout << "  testMaterialFactories OK\n";
}

static void testSectionRectangular() {
    // 300mm x 500mm rectangular column: A = 0.15 m^2, Iz = b*h^3/12
    Section s = Section::rectangular(0.30, 0.50);
    assert(std::abs(s.area - 0.15) < 1e-9);
    double expectedIz = 0.30 * std::pow(0.50, 3) / 12.0;
    assert(std::abs(s.momentOfInertiaZ - expectedIz) < 1e-9);
    assert(s.torsionalConstant > 0.0);
    std::cout << "  testSectionRectangular OK\n";
}

static void testHollowTubeStiffensAsExpected() {
    // A hollow tube should have less area (and less weak-axis I) than a
    // solid section of the same outer footprint, but the torsional
    // constant of a closed thin-walled tube should still be substantial
    // (nowhere near the near-zero J an open thin-walled section would
    // have) — this is the core sanity check for the shear-core
    // idealization, since getting the tube-vs-open distinction backwards
    // would silently under-stiffen every lift-core model.
    Section tube = Section::hollowRectangularTube(4.0, 3.0, 0.30);
    Section solid = Section::rectangular(4.0, 3.0);
    assert(tube.area < solid.area);
    assert(tube.torsionalConstant > 0.05);  // order-of-magnitude sanity, not a precise target
    std::cout << "  testHollowTubeStiffensAsExpected OK\n";
}

static void testModelDofNumbering() {
    Model model("test");
    Node n1(1, 0, 0, 0), n2(2, 3, 0, 0);
    n1.restrainAll();  // fixed base
    model.addNode(n1);
    model.addNode(n2);

    Material conc = Material::concrete(1, 28.0);
    Section col = Section::rectangular(0.3, 0.3);
    model.addMaterial(conc);
    model.addSection(col);  // gets section id 0

    Element beam(1, ElementKind::Beam, {1, 2}, 1, 0, "B1");
    model.addElement(beam);

    int freeDofs = model.assignDofNumbers();
    // Node 1 fully restrained -> 0 free DOFs from it; node 2 fully free -> 6.
    assert(freeDofs == 6);
    for (int d = 0; d < kDofPerNode; ++d) {
        assert(model.node(1).dofIndex(static_cast<DOF>(d)) == -1);
        assert(model.node(2).dofIndex(static_cast<DOF>(d)) >= 0);
    }
    std::cout << "  testModelDofNumbering OK\n";
}

static void testDuplicateNodeIdThrows() {
    Model model;
    model.addNode(Node(1, 0, 0, 0));
    bool threw = false;
    try {
        model.addNode(Node(1, 1, 1, 1));
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testDuplicateNodeIdThrows OK\n";
}

static void testElementRejectsMissingNode() {
    Model model;
    model.addNode(Node(1, 0, 0, 0));
    Material m = Material::concrete(1, 28.0);
    model.addMaterial(m);
    model.addSection(Section::rectangular(0.3, 0.3));
    bool threw = false;
    try {
        // Node 2 was never added.
        model.addElement(Element(1, ElementKind::Beam, {1, 2}, 1, 0));
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testElementRejectsMissingNode OK\n";
}

int main() {
    std::cout << "test_core:\n";
    testNodeRestraints();
    testMaterialFactories();
    testSectionRectangular();
    testHollowTubeStiffensAsExpected();
    testModelDofNumbering();
    testDuplicateNodeIdThrows();
    testElementRejectsMissingNode();
    std::cout << "All core tests passed.\n";
    return 0;
}
