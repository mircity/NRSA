#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "analysis/StaticAnalysis.h"
#include "core/Element.h"
#include "core/Load.h"
#include "core/Material.h"
#include "core/Model.h"
#include "core/Node.h"
#include "core/Section.h"

using namespace nrsa;
using namespace nrsa::analysis;

static bool approxEqual(double a, double b, double relTol = 1e-4) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// Single-element cantilever COLUMN (vertical member), end to end
// through Model + StaticAnalysis. A vertical member is used
// deliberately rather than a horizontal one: FrameElement3D's default
// local-axis convention (see FrameElement3D::computeLocalAxes) gives a
// vertical member the clean, unambiguous mapping local-x=global Z,
// local-y=global X, local-z=global Y — so a lateral load applied along
// global X and the resulting global Ux/Ry displacements can be compared
// directly against the local-coordinate closed-form results, with no
// sign/axis bookkeeping needed to translate between the two. (A
// horizontal member's default axes are equally valid but map to global
// Z/−Y instead of the more guessable Y/Z — still correct, just a worse
// choice for a test that wants the comparison itself to be obviously
// trustworthy rather than another place a sign error could hide.)
static void testCantileverEndToEnd() {
    Model model("cantilever column");
    double L = 4.0;
    Node n1(1, 0, 0, 0), n2(2, 0, 0, L);  // vertical member, base to tip
    n1.restrainAll();  // fixed base
    model.addNode(n1);
    model.addNode(n2);

    double E_kPa = 25.0e6;
    Material conc(1, "test", MaterialKind::Concrete, E_kPa, 0.2, 23.6);
    model.addMaterial(conc);
    Section sec = Section::rectangular(0.3, 0.5);
    int secId = model.addSection(sec);

    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));

    LoadCase lc(1, "Test Load", LoadCaseType::Other);
    model.addLoadCase(lc);
    double P = 20.0;  // kN, lateral load along global X at the free tip
    NodalLoad load{2, 1, P, 0.0, 0.0, 0.0, 0.0, 0.0};
    model.addNodalLoad(load);

    StaticAnalysis fea(model);
    auto result = fea.run(1);

    double Iz = 0.3 * std::pow(0.5, 3) / 12.0;
    double expectedDelta = P * std::pow(L, 3) / (3.0 * E_kPa * Iz);
    double expectedTheta = P * L * L / (2.0 * E_kPa * Iz);
    assert(approxEqual(result.displacements[2].Ux, expectedDelta));
    assert(approxEqual(result.displacements[2].Ry, expectedTheta));

    // Reaction at the fixed base: force = -P (equilibrium), moment
    // about global Y = P*L (cantilever base moment).
    assert(approxEqual(result.reactions[1].Ux, -P, 1e-3));
    assert(approxEqual(result.reactions[1].Ry, -P * L, 1e-3));

    // Residual at every free DOF should be ~0 for a converged linear
    // solve — this is the generic self-consistency check every model,
    // not just this hand-calculable one, can be run through.
    assert(result.maxFreeDofResidualNorm < 1e-6);

    assert(result.skippedElementIds.empty());
    std::cout << "  testCantileverEndToEnd OK\n";
}

// A two-element continuous cantilever (base -> mid -> tip, tip-loaded)
// — the smallest model that actually exercises multi-element assembly
// (a single-element model can't catch a scatter-indexing bug that only
// shows up when two elements share a node). Checked against the
// classic continuous-beam result for a cantilever with an interior
// node under a tip point load: since there's no interior support (the
// midpoint is just a modeling node, not a support), the exact tip
// deflection/rotation must come out identical to a single-element
// cantilever of the same total length and EI — a stiffness method is
// mesh-independent for a prismatic member under this loading, so
// matching the one-element closed form here is a genuine, meaningful
// check, not a coincidence of the specific split chosen.
static void testTwoElementCantileverMatchesSingleElement() {
    Model model("two-element cantilever column");
    double L = 4.0;
    Node n1(1, 0, 0, 0), n2(2, 0, 0, L / 2.0), n3(3, 0, 0, L);
    n1.restrainAll();
    model.addNode(n1);
    model.addNode(n2);
    model.addNode(n3);

    double E_kPa = 25.0e6;
    Material conc(1, "test", MaterialKind::Concrete, E_kPa, 0.2, 23.6);
    model.addMaterial(conc);
    Section sec = Section::rectangular(0.3, 0.5);
    int secId = model.addSection(sec);

    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));
    model.addElement(Element(2, ElementKind::Column, {2, 3}, 1, secId, "C2"));

    LoadCase lc(1, "Test Load", LoadCaseType::Other);
    model.addLoadCase(lc);
    double P = 20.0;
    model.addNodalLoad(NodalLoad{3, 1, P, 0.0, 0.0, 0.0, 0.0, 0.0});

    StaticAnalysis fea(model);
    auto result = fea.run(1);

    double Iz = 0.3 * std::pow(0.5, 3) / 12.0;
    double expectedDelta = P * std::pow(L, 3) / (3.0 * E_kPa * Iz);
    assert(approxEqual(result.displacements[3].Ux, expectedDelta));
    assert(approxEqual(result.reactions[1].Ux, -P, 1e-3));
    assert(approxEqual(result.reactions[1].Ry, -P * L, 1e-3));
    assert(result.maxFreeDofResidualNorm < 1e-6);
    // Both elements' local shear at their shared node (node2) must be
    // equal and OPPOSITE, not equal — node2 carries no external load in
    // this model, so nodal equilibrium requires the force element1
    // exerts on node2 to exactly cancel the force element2 exerts on
    // node1 (the same physical node). Both elements share the same
    // orientation here (collinear, vertical), so this really is a pure
    // sign flip, not an axis-convention difference between the two.
    assert(approxEqual(result.elementForces[1].shearY2, -result.elementForces[2].shearY1, 1e-3));
    std::cout << "  testTwoElementCantileverMatchesSingleElement OK\n";
}

// A simple portal frame (two columns + one beam) under a lateral load
// at roof level — no closed-form hand calc attempted here (a portal
// frame's exact solution needs its own derivation), but GLOBAL
// EQUILIBRIUM is a model-independent check that must hold for ANY
// correctly assembled/solved structure: the sum of the horizontal
// reactions must equal the applied horizontal load, and the sum of
// vertical reactions must equal the applied vertical load (zero here).
// This is exactly the kind of check that would have caught a sign
// error or a mis-scattered DOF that a single hand-calculable case might
// not exercise.
static void testPortalFrameGlobalEquilibrium() {
    Model model("portal frame");
    double H = 3.5, W = 5.0;
    Node n1(1, 0, 0, 0), n2(2, 0, 0, H), n3(3, W, 0, H), n4(4, W, 0, 0);
    n1.restrainAll();
    n4.restrainAll();
    model.addNode(n1);
    model.addNode(n2);
    model.addNode(n3);
    model.addNode(n4);

    Material conc = Material::concrete(1, 28.0);
    model.addMaterial(conc);
    int colSec = model.addSection(Section::rectangular(0.3, 0.3));
    int beamSec = model.addSection(Section::rectangular(0.3, 0.5));

    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, colSec, "C1"));
    model.addElement(Element(2, ElementKind::Beam, {2, 3}, 1, beamSec, "B1"));
    model.addElement(Element(3, ElementKind::Column, {4, 3}, 1, colSec, "C2"));

    LoadCase lc(1, "Lateral", LoadCaseType::Seismic);
    model.addLoadCase(lc);
    double Hload = 30.0;  // kN lateral at roof level (node 2)
    model.addNodalLoad(NodalLoad{2, 1, Hload, 0.0, 0.0, 0.0, 0.0, 0.0});

    StaticAnalysis fea(model);
    auto result = fea.run(1);

    double sumRx = result.reactions[1].Ux + result.reactions[4].Ux;
    double sumUz = result.reactions[1].Uz + result.reactions[4].Uz;
    // Reactions are the force the SUPPORT exerts on the structure, so
    // equilibrium reads: applied + reaction = 0 for a structure with no
    // other external force -> sumRx should equal -Hload.
    assert(approxEqual(sumRx, -Hload, 1e-3));
    assert(std::abs(sumUz) < 1e-6);  // no applied vertical load anywhere
    assert(result.maxFreeDofResidualNorm < 1e-6);
    assert(result.skippedElementIds.empty());
    std::cout << "  testPortalFrameGlobalEquilibrium OK\n";
}

static void testUnsupportedElementKindIsSkippedNotSilentlyDropped() {
    Model model("with a wall");
    Node n1(1, 0, 0, 0), n2(2, 3, 0, 0);
    n1.restrainAll();
    model.addNode(n1);
    model.addNode(n2);
    Material conc = Material::concrete(1, 28.0);
    model.addMaterial(conc);
    int sec = model.addSection(Section::rectangular(0.3, 0.3));
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, sec));
    model.addElement(Element(2, ElementKind::Wall, {1, 2}, 1, sec));  // not frame-like

    LoadCase lc(1, "L", LoadCaseType::Other);
    model.addLoadCase(lc);
    model.addNodalLoad(NodalLoad{2, 1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0});

    StaticAnalysis fea(model);
    auto result = fea.run(1);
    assert(result.skippedElementIds.size() == 1);
    assert(result.skippedElementIds[0] == 2);
    std::cout << "  testUnsupportedElementKindIsSkippedNotSilentlyDropped OK\n";
}

int main() {
    std::cout << "test_static_analysis:\n";
    testCantileverEndToEnd();
    testTwoElementCantileverMatchesSingleElement();
    testPortalFrameGlobalEquilibrium();
    testUnsupportedElementKindIsSkippedNotSilentlyDropped();
    std::cout << "All static analysis tests passed.\n";
    return 0;
}
