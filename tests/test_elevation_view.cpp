// Verifies drawing::ElevationView Phase 1 (projection) + Phase 2 (SVG
// generation) against the SAME portal-frame geometry used in
// examples/portal_frame_example.cpp (H=3.5m story height, W=5.0m bay) --
// so "correct" here means "matches known input coordinates", not just
// internal self-consistency.
#include <cassert>
#include <cmath>
#include <iostream>

#include "core/Element.h"
#include "core/Material.h"
#include "core/Model.h"
#include "core/Node.h"
#include "core/Section.h"
#include "drawing/ElevationView.h"

using namespace nrsa;
using namespace nrsa::drawing;

namespace {

Model buildPortalFrame() {
    Model model("Portal frame for drawing test");
    double H = 3.5, W = 5.0;
    Node n1(1, 0, 0, 0, "Base-1");
    Node n2(2, 0, 0, H, "Roof-1");
    Node n3(3, W, 0, H, "Roof-2");
    Node n4(4, W, 0, 0, "Base-2");
    n1.restrainAll();
    n4.restrainAll();
    model.addNode(n1);
    model.addNode(n2);
    model.addNode(n3);
    model.addNode(n4);

    Material conc = Material::concrete(1, 28.0, "C28");
    model.addMaterial(conc);
    int colSecId = model.addSection(Section::rectangular(0.30, 0.30, "300x300 column"));
    int beamSecId = model.addSection(Section::rectangular(0.30, 0.50, "300x500 beam"));

    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, colSecId, "C1"));
    model.addElement(Element(2, ElementKind::Beam, {2, 3}, 1, beamSecId, "B1"));
    model.addElement(Element(3, ElementKind::Column, {4, 3}, 1, colSecId, "C2"));
    return model;
}

}  // namespace

static void testProjectNodesXZ() {
    Model model = buildPortalFrame();
    auto nodes = projectNodes(model, ViewPlane::ElevationXZ);
    assert(nodes.size() == 4);
    // Base-1 (id 1): x=0,z=0 -> u=0,v=0
    assert(std::abs(nodes[0].u - 0.0) < 1e-9 && std::abs(nodes[0].v - 0.0) < 1e-9);
    // Roof-1 (id 2): x=0,z=3.5 -> u=0,v=3.5
    assert(std::abs(nodes[1].u - 0.0) < 1e-9 && std::abs(nodes[1].v - 3.5) < 1e-9);
    // Roof-2 (id 3): x=5,z=3.5 -> u=5,v=3.5
    assert(std::abs(nodes[2].u - 5.0) < 1e-9 && std::abs(nodes[2].v - 3.5) < 1e-9);
    // Base-2 (id 4): x=5,z=0 -> u=5,v=0
    assert(std::abs(nodes[3].u - 5.0) < 1e-9 && std::abs(nodes[3].v - 0.0) < 1e-9);
    std::cout << "  testProjectNodesXZ OK\n";
}

static void testProjectNodesYZDropsX() {
    // A frame built entirely in the X-Z plane (Y=0 everywhere) should
    // collapse to a single vertical line (u constant at 0) on a YZ
    // elevation -- confirms the "wrong axis dropped" bug class is
    // actually caught, not just that SOME numbers came out.
    Model model = buildPortalFrame();
    auto nodes = projectNodes(model, ViewPlane::ElevationYZ);
    for (const auto& n : nodes) {
        assert(std::abs(n.u - 0.0) < 1e-9);  // every node has Y=0
    }
    std::cout << "  testProjectNodesYZDropsX OK\n";
}

static void testProjectElementsCountAndSectionDepth() {
    Model model = buildPortalFrame();
    auto members = projectElements(model, ViewPlane::ElevationXZ);
    assert(members.size() == 3);  // 2 columns + 1 beam, all 2-node

    // Column C1: Base-1(0,0)->Roof-1(0,3.5), section 300x300 -> depth 0.30
    const auto& c1 = members[0];
    assert(c1.kind == ElementKind::Column);
    assert(std::abs(c1.u1 - 0.0) < 1e-9 && std::abs(c1.v1 - 0.0) < 1e-9);
    assert(std::abs(c1.u2 - 0.0) < 1e-9 && std::abs(c1.v2 - 3.5) < 1e-9);
    assert(std::abs(c1.sectionDepthM - 0.30) < 1e-9);

    // Beam B1: Roof-1(0,3.5)->Roof-2(5,3.5), section 300x500 -> depth 0.50
    const auto& b1 = members[1];
    assert(b1.kind == ElementKind::Beam);
    assert(std::abs(b1.v1 - 3.5) < 1e-9 && std::abs(b1.v2 - 3.5) < 1e-9);
    assert(std::abs(b1.u2 - b1.u1 - 5.0) < 1e-9);
    assert(std::abs(b1.sectionDepthM - 0.50) < 1e-9);

    std::cout << "  testProjectElementsCountAndSectionDepth OK\n";
}

static void testBoundingBoxMatchesFrameEnvelope() {
    Model model = buildPortalFrame();
    auto nodes = projectNodes(model, ViewPlane::ElevationXZ);
    BoundingBoxUV box = computeBoundingBox(nodes);
    assert(std::abs(box.minU - 0.0) < 1e-9);
    assert(std::abs(box.maxU - 5.0) < 1e-9);
    assert(std::abs(box.minV - 0.0) < 1e-9);
    assert(std::abs(box.maxV - 3.5) < 1e-9);
    assert(std::abs(box.width() - 5.0) < 1e-9);
    assert(std::abs(box.height() - 3.5) < 1e-9);
    std::cout << "  testBoundingBoxMatchesFrameEnvelope OK\n";
}

static void testSvgWellFormedAndSized() {
    Model model = buildPortalFrame();
    ElevationSvgOptions opts;
    opts.pixelsPerMeter = 60.0;
    opts.marginPx = 40.0;
    std::string svg = generateElevationSvg(model, ViewPlane::ElevationXZ, opts);

    assert(svg.find("<svg") != std::string::npos);
    assert(svg.find("</svg>") != std::string::npos);

    // 3 members -> exactly 3 <polygon> outlines; 4 nodes -> exactly 4
    // <circle> markers. Counted, not just "contains a polygon", so a
    // regression that drops or duplicates a member/node is caught.
    auto count = [&](const std::string& needle) {
        size_t n = 0, pos = 0;
        while ((pos = svg.find(needle, pos)) != std::string::npos) { ++n; pos += needle.size(); }
        return n;
    };
    assert(count("<polygon") == 3);
    assert(count("<circle") == 4);

    // viewBox width should equal (span_u * scale + 2*margin) =
    // (5.0*60 + 80) = 380; height = (3.5*60 + 80) = 290.
    assert(svg.find("viewBox=\"0 0 380 290\"") != std::string::npos);

    // Member/node labels should appear verbatim (escaped -- none of
    // these labels contain XML-special characters, so a plain find is
    // a valid check here).
    assert(svg.find("C1") != std::string::npos);
    assert(svg.find("B1") != std::string::npos);
    assert(svg.find("Roof-1") != std::string::npos);

    std::cout << "  testSvgWellFormedAndSized OK\n";
}

static void testSvgHandlesDegenerateSingleNodeModel() {
    // A model with one free-floating node and no elements must not
    // produce a zero/negative-size viewBox (the min(width,0.5) guard).
    Model model("Empty-ish");
    Node n(1, 2.0, 0.0, 1.0, "Solo");
    model.addNode(n);
    std::string svg = generateElevationSvg(model, ViewPlane::ElevationXZ);
    assert(svg.find("<svg") != std::string::npos);
    assert(svg.find("<circle") != std::string::npos);
    std::cout << "  testSvgHandlesDegenerateSingleNodeModel OK\n";
}

int main() {
    std::cout << "Running ElevationView tests...\n";
    testProjectNodesXZ();
    testProjectNodesYZDropsX();
    testProjectElementsCountAndSectionDepth();
    testBoundingBoxMatchesFrameEnvelope();
    testSvgWellFormedAndSized();
    testSvgHandlesDegenerateSingleNodeModel();
    std::cout << "All ElevationView tests passed.\n";
    return 0;
}
