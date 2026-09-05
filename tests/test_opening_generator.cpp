#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <set>

#include "core/Material.h"
#include "core/Model.h"
#include "core/Section.h"
#include "modeler/OpeningGenerator.h"

using namespace nrsa;
using namespace nrsa::modeler;

static bool approxEqual(double a, double b, double tol = 1e-9) { return std::abs(a - b) <= tol; }

// A flat 6m x 4m panel in the z=0 plane, with a centered opening
// covering the middle third in each direction (u,v in [1/3, 2/3]).
// Hand-calc: opening should span x in [2,4], y in [4/3, 8/3].
static void testCenteredOpeningGeometryHandCalc() {
    Model model("opening geometry test");
    Node n1(1, 0, 0, 0), n2(2, 6, 0, 0), n3(3, 6, 4, 0), n4(4, 0, 4, 0);
    model.addNode(n1); model.addNode(n2); model.addNode(n3); model.addNode(n4);
    model.addMaterial(Material::concrete(1, 28.0));
    int secId = model.addSection(Section::shell(0.15));

    int nextNodeId = 100, nextElementId = 100;
    auto result = generatePanelWithRectangularOpening(model, 1, 2, 3, 4, 1.0 / 3, 2.0 / 3, 1.0 / 3, 2.0 / 3,
                                                        1, secId, ElementKind::Slab, nextNodeId, nextElementId,
                                                        "TestPanel");

    assert(result.newNodeIds.size() == 12);
    assert(result.panelElementIds.size() == 8);

    // Find the opening's 4 corner nodes by their expected coordinates.
    bool foundBottomLeft = false, foundTopRight = false;
    for (int id : result.newNodeIds) {
        const Node& n = model.node(id);
        if (approxEqual(n.x(), 2.0) && approxEqual(n.y(), 4.0 / 3)) foundBottomLeft = true;
        if (approxEqual(n.x(), 4.0) && approxEqual(n.y(), 8.0 / 3)) foundTopRight = true;
    }
    assert(foundBottomLeft);
    assert(foundTopRight);
    std::cout << "  testCenteredOpeningGeometryHandCalc OK\n";
}

// Every generated element must be a valid 4-node Slab; and the total
// AREA of the 8 panels (each a quad, area via shoelace on the planar
// z=0 panel) must equal the original panel's area MINUS the opening's
// area -- a strong, independent geometric cross-check.
static void testTotalAreaHandCalc() {
    Model model("area test");
    double W = 6.0, H = 4.0;
    Node n1(1, 0, 0, 0), n2(2, W, 0, 0), n3(3, W, H, 0), n4(4, 0, H, 0);
    model.addNode(n1); model.addNode(n2); model.addNode(n3); model.addNode(n4);
    model.addMaterial(Material::concrete(1, 28.0));
    int secId = model.addSection(Section::shell(0.15));

    double uStart = 0.25, uEnd = 0.75, vStart = 0.4, vEnd = 0.6;
    int nextNodeId = 100, nextElementId = 100;
    auto result = generatePanelWithRectangularOpening(model, 1, 2, 3, 4, uStart, uEnd, vStart, vEnd, 1,
                                                        secId, ElementKind::Slab, nextNodeId, nextElementId, "P");

    double totalArea = 0.0;
    for (int id : result.panelElementIds) {
        const Element& e = model.element(id);
        assert(e.kind() == ElementKind::Slab);
        assert(e.nodeCount() == 4);
        // Shoelace formula for a planar quad in the z=0 plane.
        double area = 0.0;
        for (int k = 0; k < 4; ++k) {
            const Node& a = model.node(e.nodeId(static_cast<std::size_t>(k)));
            const Node& b = model.node(e.nodeId(static_cast<std::size_t>((k + 1) % 4)));
            area += a.x() * b.y() - b.x() * a.y();
        }
        totalArea += std::abs(area) / 2.0;
    }

    double openingArea = (uEnd - uStart) * W * (vEnd - vStart) * H;
    double expectedArea = W * H - openingArea;
    assert(approxEqual(totalArea, expectedArea, 1e-9));
    std::cout << "  testTotalAreaHandCalc (expected=" << expectedArea << ", got=" << totalArea << ") OK\n";
}

static void testRejectsInvalidFractions() {
    Model model("invalid opening test");
    Node n1(1, 0, 0, 0), n2(2, 6, 0, 0), n3(3, 6, 4, 0), n4(4, 0, 4, 0);
    model.addNode(n1); model.addNode(n2); model.addNode(n3); model.addNode(n4);
    model.addMaterial(Material::concrete(1, 28.0));
    int secId = model.addSection(Section::shell(0.15));
    int nextNodeId = 100, nextElementId = 100;

    bool threw = false;
    try {
        generatePanelWithRectangularOpening(model, 1, 2, 3, 4, 0.6, 0.4, 0.3, 0.7, 1, secId,
                                             ElementKind::Slab, nextNodeId, nextElementId, "Bad");
    } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  testRejectsInvalidFractions OK\n";
}

// No duplicate node ids created, and node/element id counters advance
// by exactly 12 and 8 respectively.
static void testIdCountersAdvanceCorrectly() {
    Model model("id counter test");
    Node n1(1, 0, 0, 0), n2(2, 6, 0, 0), n3(3, 6, 4, 0), n4(4, 0, 4, 0);
    model.addNode(n1); model.addNode(n2); model.addNode(n3); model.addNode(n4);
    model.addMaterial(Material::concrete(1, 28.0));
    int secId = model.addSection(Section::shell(0.15));

    int nextNodeId = 50, nextElementId = 200;
    generatePanelWithRectangularOpening(model, 1, 2, 3, 4, 0.3, 0.7, 0.3, 0.7, 1, secId,
                                         ElementKind::Slab, nextNodeId, nextElementId, "P");
    assert(nextNodeId == 62);      // 50 + 12
    assert(nextElementId == 208);  // 200 + 8

    std::set<int> ids;
    for (const auto& n : model.nodes()) assert(ids.insert(n.id()).second);
    std::cout << "  testIdCountersAdvanceCorrectly OK\n";
}

int main() {
    std::cout << "test_opening_generator:\n";
    testCenteredOpeningGeometryHandCalc();
    testTotalAreaHandCalc();
    testRejectsInvalidFractions();
    testIdCountersAdvanceCorrectly();
    std::cout << "All tests passed.\n";
    return 0;
}
