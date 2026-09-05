#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>

#include "core/Element.h"
#include "core/Model.h"
#include "core/Node.h"
#include "core/Section.h"
#include "io/CsvIO.h"

using namespace nrsa;
using namespace nrsa::io;

static bool approxEqual(double a, double b, double tol = 1e-9) { return std::abs(a - b) <= tol; }

// Full round-trip: build a model with varied restraints/mass/labels,
// export to CSV, import into a FRESH model, and verify every field
// matches exactly -- the whole point of CSV being the full-fidelity
// format (unlike DXF).
static void testFullRoundTripHandCalc() {
    Model original("roundtrip source");
    Node n1(1, 0, 0, 0, "Base");
    n1.restrainAll();
    original.addNode(n1);

    Node n2(2, 5, 3, 4, "Top");
    n2.restrain(DOF::Uy, true);
    n2.setTranslationalMass(12.5);
    n2.setRotationalMass(0.1, 0.2, 0.3);
    original.addNode(n2);

    original.addElement(Element(1, ElementKind::Column, {1, 2}, 1, 1, "MyColumn"));

    const char* nodesPath = "/tmp/test_csv_nodes.csv";
    const char* elementsPath = "/tmp/test_csv_elements.csv";
    exportCsv(original, nodesPath, elementsPath);

    Model restored("roundtrip target");
    auto result = importCsv(restored, nodesPath, elementsPath);
    assert(result.nodesImported == 2);
    assert(result.elementsImported == 1);

    const Node& r1 = restored.node(1);
    assert(approxEqual(r1.x(), 0) && approxEqual(r1.y(), 0) && approxEqual(r1.z(), 0));
    assert(r1.label() == "Base");
    assert(r1.isRestrained(DOF::Ux) && r1.isRestrained(DOF::Uy) && r1.isRestrained(DOF::Uz));
    assert(r1.isRestrained(DOF::Rx) && r1.isRestrained(DOF::Ry) && r1.isRestrained(DOF::Rz));

    const Node& r2 = restored.node(2);
    assert(approxEqual(r2.x(), 5) && approxEqual(r2.y(), 3) && approxEqual(r2.z(), 4));
    assert(r2.label() == "Top");
    assert(!r2.isRestrained(DOF::Ux));
    assert(r2.isRestrained(DOF::Uy));
    assert(approxEqual(r2.translationalMass(), 12.5));
    auto rm = r2.rotationalMass();
    assert(approxEqual(rm[0], 0.1) && approxEqual(rm[1], 0.2) && approxEqual(rm[2], 0.3));

    const Element& e = restored.element(1);
    assert(e.kind() == ElementKind::Column);
    assert(e.materialId() == 1 && e.sectionId() == 1);
    assert(e.nodeCount() == 2 && e.nodeId(0) == 1 && e.nodeId(1) == 2);
    assert(e.label() == "MyColumn");

    std::remove(nodesPath);
    std::remove(elementsPath);
    std::cout << "  testFullRoundTripHandCalc OK\n";
}

// A 4-node Slab element (shell) must also round-trip its full node list
// (not just the first 2), since CSV's node list column is
// semicolon-separated and variable-length.
static void testShellElementRoundTrip() {
    Model original("shell test");
    for (int i = 1; i <= 4; ++i) original.addNode(Node(i, i, 0, 0));
    original.addElement(Element(1, ElementKind::Slab, {1, 2, 3, 4}, 1, 1, "SlabA"));

    const char* nodesPath = "/tmp/test_csv_nodes2.csv";
    const char* elementsPath = "/tmp/test_csv_elements2.csv";
    exportCsv(original, nodesPath, elementsPath);

    Model restored("shell restored");
    importCsv(restored, nodesPath, elementsPath);
    const Element& e = restored.element(1);
    assert(e.kind() == ElementKind::Slab);
    assert(e.nodeCount() == 4);
    for (std::size_t i = 0; i < 4; ++i) assert(e.nodeId(i) == static_cast<int>(i + 1));

    std::remove(nodesPath);
    std::remove(elementsPath);
    std::cout << "  testShellElementRoundTrip OK\n";
}

static void testRejectsMissingFile() {
    Model model("missing file test");
    bool threw = false;
    try { importCsv(model, "/tmp/does_not_exist_nodes.csv", "/tmp/does_not_exist_elements.csv"); }
    catch (const std::runtime_error&) { threw = true; }
    assert(threw);
    std::cout << "  testRejectsMissingFile OK\n";
}

int main() {
    std::cout << "test_csv_io:\n";
    testFullRoundTripHandCalc();
    testShellElementRoundTrip();
    testRejectsMissingFile();
    std::cout << "All tests passed.\n";
    return 0;
}
