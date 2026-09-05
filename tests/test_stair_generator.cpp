#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "core/Material.h"
#include "core/Model.h"
#include "core/Section.h"
#include "modeler/StairGenerator.h"

using namespace nrsa;
using namespace nrsa::modeler;

static bool approxEqual(double a, double b, double tol = 1e-9) { return std::abs(a - b) <= tol; }

// Hand-calc: a flight rising 1.5m over a 3m horizontal run has incline
// length = sqrt(3^2 + 1.5^2) = sqrt(9+2.25) = sqrt(11.25).
static void testInclineLengthHandCalc() {
    Model model("incline length test");
    model.addMaterial(Material::concrete(1, 28.0));
    int secId = model.addSection(Section::shell(0.15));
    int nextNodeId = 1, nextElementId = 1;

    auto result = generateStairFlight(model, {0, 0, 0}, {3, 0, 1.5}, 1.2, 6, 1, secId,
                                       nextNodeId, nextElementId, "Stair1");

    double expected = std::sqrt(3.0 * 3.0 + 1.5 * 1.5);
    assert(approxEqual(result.inclineLengthM, expected));
    std::cout << "  testInclineLengthHandCalc (expected=" << expected << ") OK\n";
}

// Node/element counts: subdivisionCount=6 -> 7 cross-sections * 2 nodes
// = 14 nodes, 6 panels.
static void testNodeAndElementCountHandCalc() {
    Model model("count test");
    model.addMaterial(Material::concrete(1, 28.0));
    int secId = model.addSection(Section::shell(0.15));
    int nextNodeId = 1, nextElementId = 1;

    auto result = generateStairFlight(model, {0, 0, 0}, {3, 0, 1.5}, 1.2, 6, 1, secId,
                                       nextNodeId, nextElementId, "Stair1");
    assert(result.nodeIds.size() == 14);
    assert(result.panelElementIds.size() == 6);
    for (int id : result.panelElementIds) {
        assert(model.element(id).kind() == ElementKind::Slab);
        assert(model.element(id).nodeCount() == 4);
    }
    std::cout << "  testNodeAndElementCountHandCalc OK\n";
}

// Width hand-calc: for a flight running purely along global X with
// default up=Z, the width direction must be purely along global Y, so
// each cross-section's left/right nodes should be exactly
// widthM/2 apart in Y and identical in X and Z.
static void testWidthDirectionHandCalc() {
    Model model("width direction test");
    model.addMaterial(Material::concrete(1, 28.0));
    int secId = model.addSection(Section::shell(0.15));
    int nextNodeId = 1, nextElementId = 1;
    double width = 1.5;

    auto result = generateStairFlight(model, {0, 0, 0}, {4, 0, 2.0}, width, 4, 1, secId,
                                       nextNodeId, nextElementId, "Stair1");

    for (std::size_t s = 0; s < result.nodeIds.size(); s += 2) {
        const Node& left = model.node(result.nodeIds[s]);
        const Node& right = model.node(result.nodeIds[s + 1]);
        assert(approxEqual(left.x(), right.x()));
        assert(approxEqual(left.z(), right.z()));
        assert(approxEqual(std::abs(right.y() - left.y()), width));
    }
    std::cout << "  testWidthDirectionHandCalc OK\n";
}

static void testRejectsInvalidInputs() {
    Model model("invalid stair test");
    model.addMaterial(Material::concrete(1, 28.0));
    int secId = model.addSection(Section::shell(0.15));
    int nextNodeId = 1, nextElementId = 1;

    bool threw = false;
    try {
        generateStairFlight(model, {0, 0, 0}, {0, 0, 0}, 1.2, 4, 1, secId, nextNodeId, nextElementId, "S");
    } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);

    threw = false;
    try {
        generateStairFlight(model, {0, 0, 0}, {0, 0, 3.0}, 1.2, 4, 1, secId, nextNodeId, nextElementId, "S");
    } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);  // purely vertical run -> no horizontal width direction

    threw = false;
    try {
        generateStairFlight(model, {0, 0, 0}, {3, 0, 1.5}, -1.0, 4, 1, secId, nextNodeId, nextElementId, "S");
    } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);

    std::cout << "  testRejectsInvalidInputs OK\n";
}

int main() {
    std::cout << "test_stair_generator:\n";
    testInclineLengthHandCalc();
    testNodeAndElementCountHandCalc();
    testWidthDirectionHandCalc();
    testRejectsInvalidInputs();
    std::cout << "All tests passed.\n";
    return 0;
}
