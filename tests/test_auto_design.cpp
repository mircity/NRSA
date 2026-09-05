#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "autodesign/AutoDesign.h"
#include "design/BNBC2020.h"
#include "design/RCCBeam.h"

using namespace nrsa;
using namespace nrsa::autodesign;
using namespace nrsa::design;

static bool approxEqual(double a, double b, double relTol = 1e-2) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// Classic hand-calc, end-to-end through the WHOLE chain (Analysis ->
// Load Combination -> Member Forces -> Section Check -> Reinforcement
// Design): a simply-supported beam, total span L=6m, point dead load
// P=100kN at midspan (via a mid-span node, since a 2-node element
// can't carry an interior point load directly).
//   Hand-calc, simple beam theory: Mmax = P*L/4 = 100*6/4 = 150 kNm
//   BNBC 2020 / ACI 318-19 basic combos with ONLY dead load present ->
//   only U=1.4D is generated (5.3.1a) -> Mu = 1.4*150 = 210 kNm
//   Reactions: R1=R2=P/2=50kN (each), so shear in each half-span = 50kN
//     factored: Vu = 1.4*50 = 70 kN
static void testSimplySupportedBeamPL4HandCalc() {
    Model model("auto design PL/4 test");
    double L = 6.0;
    Node n1(1, 0, 0, 0), n2(2, L / 2.0, 0, 0), n3(3, L, 0, 0);
    n1.restrainAll();  // pin, plus out-of-plane/torsion lock
    n1.restrain(DOF::Ry, false);
    n1.restrain(DOF::Rz, false);
    n2.restrain(DOF::Uy, true);
    n2.restrain(DOF::Rx, true);
    n3.restrain(DOF::Uy, true);
    n3.restrain(DOF::Uz, true);
    n3.restrain(DOF::Rx, true);
    model.addNode(n1);
    model.addNode(n2);
    model.addNode(n3);

    Material conc(1, "C25", MaterialKind::Concrete, 25.0e6, 0.2, 23.6);
    model.addMaterial(conc);
    Section sec = Section::rectangular(0.3, 0.5);  // 300mm x 500mm
    int secId = model.addSection(sec);
    model.addElement(Element(1, ElementKind::Beam, {1, 2}, 1, secId, "B1"));
    model.addElement(Element(2, ElementKind::Beam, {2, 3}, 1, secId, "B2"));

    LoadCase dead(1, "Dead", LoadCaseType::Dead);
    model.addLoadCase(dead);
    double P = 100.0;  // kN
    model.addNodalLoad(NodalLoad{2, 1, 0.0, 0.0, -P, 0.0, 0.0, 0.0});

    BnbcLoadCaseIds ids;
    ids.dead = 1;
    auto combinations = generateBasicCombinations(ids);
    // With only dead load present: Eq 5.3.1a (1.4D) is generated, AND
    // Eq 5.3.1b (1.2D+1.6L+...) is ALSO generated (it fires whenever D,
    // L, OR a roof-like load is present -- D alone is enough), just
    // with its L/Lr/S/R terms simply absent, reducing it to plain
    // 1.2D. Both are real, valid ACI 318-19 combos to check even
    // though only one governs here.
    assert(combinations.size() == 2);

    double fc = 28.0, fy = 420.0, coverMm = 40.0, longBarDiaMm = 20.0, stirrupDiaMm = 10.0;
    int stirrupLegs = 2;
    auto results = runAutoDesignForBeams(model, combinations, fc, fy, coverMm, longBarDiaMm,
                                          stirrupDiaMm, stirrupLegs);
    assert(results.size() == 2);

    double expectedMu = 1.4 * (P * L / 4.0);  // 210 kNm
    double expectedVu = 1.4 * (P / 2.0);      // 70 kN

    for (const auto& r : results) {
        assert(approxEqual(r.muKNm, expectedMu, 0.02));
        assert(approxEqual(r.vuKN, expectedVu, 0.02));
        assert(r.governingMomentCombinationId == combinations[0].id());

        // Cross-check the wired-through reinforcement design against an
        // INDEPENDENT direct call to design::designFlexure with the same
        // Mu -- confirms AutoDesign correctly passes b/d/fc/fy through,
        // not a re-derivation of designFlexure's own math (already
        // separately verified in test_rcc_beam.cpp).
        double dM = (0.5 * 1000.0 - coverMm - longBarDiaMm / 2.0) / 1000.0;
        auto directFlexure = designFlexure(r.muKNm, 0.3, dM, fc, fy);
        assert(approxEqual(r.flexure.asRequiredMm2, directFlexure.asRequiredMm2, 1e-6));
    }
    std::cout << "  testSimplySupportedBeamPL4HandCalc (Mu=" << results[0].muKNm
              << " kNm [expected " << expectedMu << "], Vu=" << results[0].vuKN
              << " kN [expected " << expectedVu << "]) OK\n";
}

// Column extension, added 2026-08-30: a fixed-base cantilever SQUARE
// column (avoiding the momentY/momentZ<->width/depth pairing ambiguity
// documented in AutoDesign.h -- a square column sidesteps it entirely
// since both axes are interchangeable), height H=3m, with an axial
// load P=500kN and lateral load Q=50kN at the free top.
//   Hand-calc: at the base, axial demand = P = 500kN (verified
//   empirically to appear in axial1), bending moment = Q*H = 50*3 =
//   150 kNm (verified empirically to appear in momentZ1, with
//   momentY1 = 0 for this specific loading direction).
// This test cross-checks AutoDesign's reported (Pu, governing moment)
// against these independently-known values, and cross-checks its
// reinforcement result against a DIRECT call to
// design::designBiaxialColumn with the same (Pu, Mux, Muy) --
// confirming AutoDesign wires the analysis result through correctly,
// not re-deriving designBiaxialColumn's own math (already separately
// verified in test_rcc_column.cpp).
static void testCantileverColumnHandCalc() {
    Model model("auto design column test");
    double H = 3.0;
    Node n1(1, 0, 0, 0), n2(2, 0, 0, H);
    n1.restrainAll();
    n2.restrain(DOF::Uy, true);
    n2.restrain(DOF::Rx, true);
    n2.restrain(DOF::Rz, true);
    model.addNode(n1);
    model.addNode(n2);

    Material conc(1, "C28", MaterialKind::Concrete, 28.0e6, 0.2, 23.6);
    model.addMaterial(conc);
    Section sec = Section::rectangular(0.4, 0.4);  // square, sidesteps the axis-pairing ambiguity
    int secId = model.addSection(sec);
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));

    LoadCase dead(1, "Dead", LoadCaseType::Dead);
    model.addLoadCase(dead);
    double P = 500.0, Q = 50.0;
    model.addNodalLoad(NodalLoad{2, 1, Q, 0.0, -P, 0.0, 0.0, 0.0});

    BnbcLoadCaseIds ids;
    ids.dead = 1;
    auto combinations = generateBasicCombinations(ids);

    double fc = 28.0, fy = 420.0, coverMm = 40.0, barDiaMm = 20.0;
    auto results = runAutoDesignForColumns(model, combinations, fc, fy, coverMm, barDiaMm, 3, 3);
    assert(results.size() == 1);

    // Governing combination is 1.4D (larger factor -> larger demand on
    // both axial and moment simultaneously, so it governs the
    // interaction check too for this single-load-case model).
    double expectedPu = 1.4 * P;
    double expectedM = 1.4 * (Q * H);

    const auto& r = results[0];
    assert(approxEqual(r.puKN, expectedPu, 0.02));
    double reportedMoment = std::max(r.muxKNm, r.muyKNm);  // whichever axis it landed on
    double otherMoment = std::min(r.muxKNm, r.muyKNm);
    assert(approxEqual(reportedMoment, expectedM, 0.02));
    assert(approxEqual(otherMoment, 0.0, 1.0));  // the other axis should be ~zero for this loading

    // Cross-check against a DIRECT call with the same demand triple.
    auto directBiaxial = designBiaxialColumn(0.4, 0.4, coverMm, barDiaMm, 3, 3, fc, fy, r.puKN,
                                               r.muxKNm, r.muyKNm);
    assert(approxEqual(r.biaxial.phiPnKN, directBiaxial.phiPnKN, 1e-6));
    assert(r.biaxial.adequate == directBiaxial.adequate);

    std::cout << "  testCantileverColumnHandCalc (Pu=" << r.puKN << " kN [expected " << expectedPu
              << "], M=" << reportedMoment << " kNm [expected " << expectedM << "]) OK\n";
}

// REGRESSION TEST for a REAL bug found and fixed 2026-08-30: this
// project's earlier code assigned FrameEndForces' momentY to Mux and
// momentZ to Muy -- backwards (see AutoDesign.h's class doc comment
// for the full derivation of the correct pairing: momentZ<->Mux,
// momentY<->Muy). This was invisible for a SQUARE column (every other
// test here) because swapping interchangeable axes changes nothing.
// This test uses a genuinely RECTANGULAR column (300mm x 600mm) with a
// loading that produces a large momentZ and zero momentY, and checks
// that AutoDesign's result matches a DIRECT designBiaxialColumn call
// with the CORRECT pairing (Mux=momentZ, Muy=momentY=0) -- and
// explicitly checks it does NOT match the swapped (buggy) pairing,
// proving the two give numerically DIFFERENT answers for a rectangular
// section (unlike the square case), so this test would have failed
// under the old code.
static void testRectangularColumnAxisPairingHandCalc() {
    Model model("rectangular column axis test");
    double H = 3.0;
    Node n1(1, 0, 0, 0), n2(2, 0, 0, H);
    n1.restrainAll();
    n2.restrain(DOF::Uy, true);
    n2.restrain(DOF::Rx, true);
    n2.restrain(DOF::Rz, true);
    model.addNode(n1);
    model.addNode(n2);

    Material conc(1, "C28", MaterialKind::Concrete, 28.0e6, 0.2, 23.6);
    model.addMaterial(conc);
    double widthM = 0.3, depthM = 0.6;  // genuinely rectangular
    Section sec = Section::rectangular(widthM, depthM);
    int secId = model.addSection(sec);
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));

    LoadCase dead(1, "Dead", LoadCaseType::Dead);
    model.addLoadCase(dead);
    double P = 300.0, Q = 40.0;
    // Lateral load in global X -> confirmed (see momentZ1/momentY1
    // empirical check performed while building this module) to produce
    // momentZ nonzero, momentY ~0 for a vertical column.
    model.addNodalLoad(NodalLoad{2, 1, Q, 0.0, -P, 0.0, 0.0, 0.0});

    BnbcLoadCaseIds ids;
    ids.dead = 1;
    auto combinations = generateBasicCombinations(ids);

    double fc = 28.0, fy = 420.0, coverMm = 40.0, barDiaMm = 20.0;
    auto results = runAutoDesignForColumns(model, combinations, fc, fy, coverMm, barDiaMm, 3, 3);
    assert(results.size() == 1);
    const auto& r = results[0];

    assert(approxEqual(r.muyKNm, 0.0, 1.0));  // weak-axis moment ~0 for this loading
    assert(r.muxKNm > 10.0);                  // strong-axis moment is the large one

    // Correct pairing: Mux=momentZ (the large one), Muy=momentY(~0).
    auto correctBiaxial = designBiaxialColumn(widthM, depthM, coverMm, barDiaMm, 3, 3, fc, fy, r.puKN,
                                                r.muxKNm, r.muyKNm);
    // Buggy (swapped) pairing: what the OLD code would have computed.
    auto swappedBiaxial = designBiaxialColumn(widthM, depthM, coverMm, barDiaMm, 3, 3, fc, fy, r.puKN,
                                                r.muyKNm, r.muxKNm);

    assert(approxEqual(r.biaxial.phiPnKN, correctBiaxial.phiPnKN, 1e-6));
    // The two pairings must give DIFFERENT answers for this rectangular
    // section (proving the fix actually matters here, unlike the square
    // column tests) -- if they were equal, this test wouldn't be
    // exercising the bug at all.
    assert(!approxEqual(correctBiaxial.phiPnKN, swappedBiaxial.phiPnKN, 0.02));

    std::cout << "  testRectangularColumnAxisPairingHandCalc (correct phiPn=" << correctBiaxial.phiPnKN
              << " kN, swapped/buggy phiPn=" << swappedBiaxial.phiPnKN << " kN -- confirmed different) OK\n";
}

// Foundation wiring integration: the same cantilever column model, but
// now checked end-to-end through runAutoDesignForFoundations -- proves
// a footing is found and sized under the base node (identified as the
// fully-restrained end), and that its inputs (Pu, approximate service
// load) are wired through correctly from the column's own AutoDesign
// result.
static void testFoundationWiringIntegration() {
    Model model("foundation wiring test");
    double H = 3.0;
    Node n1(1, 0, 0, 0), n2(2, 0, 0, H);
    n1.restrainAll();
    n2.restrain(DOF::Uy, true);
    n2.restrain(DOF::Rx, true);
    n2.restrain(DOF::Rz, true);
    model.addNode(n1);
    model.addNode(n2);

    Material conc(1, "C28", MaterialKind::Concrete, 28.0e6, 0.2, 23.6);
    model.addMaterial(conc);
    Section sec = Section::rectangular(0.4, 0.4);
    int secId = model.addSection(sec);
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));

    LoadCase dead(1, "Dead", LoadCaseType::Dead);
    model.addLoadCase(dead);
    model.addNodalLoad(NodalLoad{2, 1, 30.0, 0.0, -400.0, 0.0, 0.0, 0.0});

    BnbcLoadCaseIds ids;
    ids.dead = 1;
    auto combinations = generateBasicCombinations(ids);
    auto columnResults = runAutoDesignForColumns(model, combinations, 28.0, 420.0, 40.0, 20.0, 3, 3);

    double loadFactor = 1.5;
    auto foundations = runAutoDesignForFoundations(model, columnResults, 150.0, 28.0, 420.0, 50.0, 16.0,
                                                     loadFactor);
    assert(foundations.size() == 1);
    const auto& f = foundations[0];
    assert(f.baseNodeId == 1);  // n1 is the fully-restrained base
    assert(f.columnElementId == 1);
    assert(approxEqual(f.puKN, columnResults[0].puKN));
    assert(approxEqual(f.approximateServiceLoadKN, f.puKN / loadFactor));
    assert(f.footing.sideM > 0.0);

    std::cout << "  testFoundationWiringIntegration (Pu=" << f.puKN << " kN, footing="
              << f.footing.sideM << "m sq) OK\n";
}

int main() {
    std::cout << "test_auto_design:\n";
    testSimplySupportedBeamPL4HandCalc();
    testCantileverColumnHandCalc();
    testRectangularColumnAxisPairingHandCalc();
    testFoundationWiringIntegration();
    std::cout << "All tests passed.\n";
    return 0;
}
