#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <set>

#include "core/Element.h"
#include "core/Model.h"
#include "core/Node.h"
#include "io/DxfIO.h"

using namespace nrsa;
using namespace nrsa::io;

static bool approxEqual(double a, double b, double tol = 1e-6) { return std::abs(a - b) <= tol; }

// DXF is geometry-only (documented, lossy for material/section) -- so
// the round-trip check that matters is GEOMETRY + ELEMENT KIND, not
// exact node/element ids or counts (import creates fresh, non-merged
// nodes per the documented scope). This test: build a model with one
// column and one beam, export, import into a fresh model, and verify
// an element with matching kind AND matching (unordered) endpoint
// coordinates exists in the imported model.
static bool hasMatchingElement(const Model& model, ElementKind kind, std::array<double, 3> p1,
                                std::array<double, 3> p2) {
    for (const auto& e : model.elements()) {
        if (e.kind() != kind || e.nodeCount() != 2) continue;
        const Node& a = model.node(e.nodeId(0));
        const Node& b = model.node(e.nodeId(1));
        bool forward = approxEqual(a.x(), p1[0]) && approxEqual(a.y(), p1[1]) && approxEqual(a.z(), p1[2]) &&
                        approxEqual(b.x(), p2[0]) && approxEqual(b.y(), p2[1]) && approxEqual(b.z(), p2[2]);
        bool reverse = approxEqual(a.x(), p2[0]) && approxEqual(a.y(), p2[1]) && approxEqual(a.z(), p2[2]) &&
                       approxEqual(b.x(), p1[0]) && approxEqual(b.y(), p1[1]) && approxEqual(b.z(), p1[2]);
        if (forward || reverse) return true;
    }
    return false;
}

static void testRoundTripPreservesGeometryAndKind() {
    Model original("dxf roundtrip source");
    original.addNode(Node(1, 0, 0, 0));
    original.addNode(Node(2, 0, 0, 3.5));
    original.addNode(Node(3, 5.0, 0, 3.5));
    original.addElement(Element(1, ElementKind::Column, {1, 2}, 1, 1, "Col1"));
    original.addElement(Element(2, ElementKind::Beam, {2, 3}, 1, 1, "Beam1"));

    const char* path = "/tmp/test_dxf_roundtrip.dxf";
    exportDxf(original, path);

    Model restored("dxf roundtrip target");
    int nextNodeId = 1, nextElementId = 1;
    auto result = importDxf(restored, path, 1, 1, nextNodeId, nextElementId);

    assert(result.elementsImported == 2);
    assert(hasMatchingElement(restored, ElementKind::Column, {0, 0, 0}, {0, 0, 3.5}));
    assert(hasMatchingElement(restored, ElementKind::Beam, {0, 0, 3.5}, {5.0, 0, 3.5}));
    assert(result.unrecognizedLayers.empty());  // NRSA's own layer convention must round-trip cleanly

    std::remove(path);
    std::cout << "  testRoundTripPreservesGeometryAndKind OK\n";
}

// A shell (4-node Slab) element must export as exactly 4 LINE entities
// (its 4 boundary edges) -- verified by counting "LINE" occurrences in
// the raw file text, an independent check of exportDxf's own stated
// behavior rather than re-parsing through importDxf.
static void testShellExportsFourBoundaryEdges() {
    Model model("shell export test");
    model.addNode(Node(1, 0, 0, 0));
    model.addNode(Node(2, 4, 0, 0));
    model.addNode(Node(3, 4, 4, 0));
    model.addNode(Node(4, 0, 4, 0));
    model.addElement(Element(1, ElementKind::Slab, {1, 2, 3, 4}, 1, 1, "SlabA"));

    const char* path = "/tmp/test_dxf_shell.dxf";
    exportDxf(model, path);

    std::ifstream f(path);
    std::string line;
    int lineCount = 0, pointCount = 0;
    while (std::getline(f, line)) {
        if (line == "LINE") lineCount++;
        if (line == "POINT") pointCount++;
    }
    assert(lineCount == 4);   // 4 boundary edges
    assert(pointCount == 4);  // 4 corner nodes as POINT entities

    std::remove(path);
    std::cout << "  testShellExportsFourBoundaryEdges OK\n";
}

// A LINE on a layer NOT following the NRSA_* convention must still
// import (as a generic Beam-kind element), and must be flagged in
// unrecognizedLayers -- verifies best-effort import of external DXF
// geometry, per the documented scope.
static void testUnrecognizedLayerImportsAsGenericBeam() {
    const char* path = "/tmp/test_dxf_external.dxf";
    std::ofstream f(path);
    f << "0\nSECTION\n2\nENTITIES\n"
      << "0\nLINE\n8\nSOME_OTHER_CAD_LAYER\n10\n0\n20\n0\n30\n0\n11\n10\n21\n0\n31\n0\n"
      << "0\nENDSEC\n0\nEOF\n";
    f.close();

    Model model("external dxf test");
    int nextNodeId = 1, nextElementId = 1;
    auto result = importDxf(model, path, 1, 1, nextNodeId, nextElementId);

    assert(result.elementsImported == 1);
    assert(result.unrecognizedLayers.size() == 1);
    assert(result.unrecognizedLayers[0] == "SOME_OTHER_CAD_LAYER");
    assert(model.element(1).kind() == ElementKind::Beam);  // defaulted

    std::remove(path);
    std::cout << "  testUnrecognizedLayerImportsAsGenericBeam OK\n";
}

static void testRejectsMissingFile() {
    Model model("missing file test");
    int nextNodeId = 1, nextElementId = 1;
    bool threw = false;
    try { importDxf(model, "/tmp/does_not_exist.dxf", 1, 1, nextNodeId, nextElementId); }
    catch (const std::runtime_error&) { threw = true; }
    assert(threw);
    std::cout << "  testRejectsMissingFile OK\n";
}

int main() {
    std::cout << "test_dxf_io:\n";
    testRoundTripPreservesGeometryAndKind();
    testShellExportsFourBoundaryEdges();
    testUnrecognizedLayerImportsAsGenericBeam();
    testRejectsMissingFile();
    std::cout << "All tests passed.\n";
    return 0;
}
