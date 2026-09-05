#include <cassert>
#include <cmath>
#include <iostream>

#include "analysis/ConstructionStageAnalysis.h"
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

// Two-segment vertical cantilever stack: node1(fixed base, z=0) --
// segment A -- node2(z=L1) -- segment B -- node3(z=L1+L2).
// Stage 1: only segment A exists; a lateral load P1 is applied at node2
//   (then the free tip) -> node2 deflects as a SIMPLE cantilever of
//   length L1: delta2_stage1 = P1*L1^3/(3EI).
// Stage 2: segment B is added; a lateral load P2 is applied at node3
//   (now the free tip of the FULL L1+L2 cantilever) -> this INCREMENTAL
//   load sees the full two-segment cantilever stiffness, not segment
//   A's alone:
//     incremental node3 deflection = P2*(L1+L2)^3/(3EI)
//     incremental node2 deflection (point along the cantilever, at
//     distance L1 from the fixed base, under a tip load at L1+L2) =
//     P2*L1^2*(3*(L1+L2)-L1)/(6EI)   [standard cantilever formula]
// The KEY physical assertion this test exists to make: node2's stage-1
// deflection does NOT get further affected by the stage-2 load's own
// bending of segment A in a way that matches "as if the whole
// structure had existed from day one" -- it accumulates as
// stage1 + stage2's own incremental contribution, which is exactly
// what staged (as opposed to one-shot) construction analysis means.
static void testTwoStageCantileverHandCalc() {
    Model model("construction stage test");
    double L1 = 3.0, L2 = 3.0;
    Node n1(1, 0, 0, 0), n2(2, 0, 0, L1), n3(3, 0, 0, L1 + L2);
    n1.restrainAll();
    model.addNode(n1);
    model.addNode(n2);
    model.addNode(n3);

    double E = 25.0e6;  // kPa
    Material conc(1, "C25", MaterialKind::Concrete, E, 0.2, 23.6);
    model.addMaterial(conc);
    Section sec = Section::rectangular(0.3, 0.5);
    double Iz = sec.momentOfInertiaZ;  // strong-axis bending, matches vertical-member local-z convention
    int secId = model.addSection(sec);

    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "ColA"));
    model.addElement(Element(2, ElementKind::Column, {2, 3}, 1, secId, "ColB"));

    double P1 = 20.0, P2 = 15.0;  // kN, lateral (global X)
    LoadCase lc1(1, "Stage1 Load", LoadCaseType::Other);
    LoadCase lc2(2, "Stage2 Load", LoadCaseType::Other);
    model.addLoadCase(lc1);
    model.addLoadCase(lc2);
    model.addNodalLoad({2, 1, P1, 0.0, 0.0, 0.0, 0.0, 0.0});
    model.addNodalLoad({3, 2, P2, 0.0, 0.0, 0.0, 0.0, 0.0});

    std::vector<ConstructionStage> stages = {
        {"Stage 1: Column A built", {1}, 1},
        {"Stage 2: Column B added", {1, 2}, 2},
    };
    auto results = runConstructionStages(model, stages);
    assert(results.size() == 2);

    double delta2_stage1_expected = P1 * std::pow(L1, 3) / (3.0 * E * Iz);
    double node2_after_stage1 = results[0].cumulativeDisplacements.at(2).Ux;
    assert(approxEqual(node2_after_stage1, delta2_stage1_expected));
    // node3 doesn't exist yet in stage 1 -> zero displacement.
    assert(approxEqual(results[0].cumulativeDisplacements.at(3).Ux, 0.0, 1e-9));

    double Ltot = L1 + L2;
    double delta3_stage2_incremental = P2 * std::pow(Ltot, 3) / (3.0 * E * Iz);
    double delta2_stage2_incremental =
        P2 * L1 * L1 * (3.0 * Ltot - L1) / (6.0 * E * Iz);

    double node3_final = results[1].cumulativeDisplacements.at(3).Ux;
    double node2_final = results[1].cumulativeDisplacements.at(2).Ux;

    assert(approxEqual(node3_final, delta3_stage2_incremental));
    assert(approxEqual(node2_final, delta2_stage1_expected + delta2_stage2_incremental));

    // Sanity: the model's real base restraint must be restored afterward.
    assert(model.node(1).isRestrained(DOF::Ux));
    assert(!model.node(2).isRestrained(DOF::Ux));

    std::cout << "  testTwoStageCantileverHandCalc OK\n";
}

static void testRejectsElementRemoval() {
    Model model("removal test");
    Node n1(1, 0, 0, 0), n2(2, 0, 0, 3.0);
    n1.restrainAll();
    model.addNode(n1);
    model.addNode(n2);
    Material conc(1, "C25", MaterialKind::Concrete, 25.0e6, 0.2, 23.6);
    model.addMaterial(conc);
    int secId = model.addSection(Section::rectangular(0.3, 0.5));
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));

    std::vector<ConstructionStage> stages = {
        {"Stage 1", {1}, -1},
        {"Stage 2 (removes element 1)", {}, -1},
    };
    bool threw = false;
    try { runConstructionStages(model, stages); }
    catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  testRejectsElementRemoval OK\n";
}

int main() {
    std::cout << "test_construction_stage_analysis:\n";
    testTwoStageCantileverHandCalc();
    testRejectsElementRemoval();
    std::cout << "All tests passed.\n";
    return 0;
}
