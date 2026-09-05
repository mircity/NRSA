#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "analysis/PDelta.h"
#include "analysis/StaticAnalysis.h"
#include "core/Element.h"
#include "core/Load.h"
#include "core/Material.h"
#include "core/Model.h"
#include "core/Node.h"
#include "core/Section.h"
#include "fem/FrameElement3D.h"
#include "fem/Matrix.h"

using namespace nrsa;
using namespace nrsa::analysis;
using namespace nrsa::fem;

static bool approxEqual(double a, double b, double relTol = 1e-4) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// A cantilever column: gravity case applies a compressive axial load
// P_axial at the free tip; the SAME model's lateral case applies a
// small transverse point load H at the same tip. P-Delta must amplify
// the lateral displacement relative to an ordinary (non-augmented)
// static solve of the lateral case alone.
//
// THE decisive check: solve the exact 2x2 system det math directly
// (Ke_sub + P*Kg_sub, same reduction used in
// tests/test_frame_element.cpp's buckling test) for the lateral tip
// displacement under H WITH the P-Delta augmentation, and compare
// against PDeltaAnalysis's own result — an independent hand-computation
// using the same underlying matrices, not a re-derivation of the
// physics from scratch, but enough to catch an assembly/sign/wiring
// bug in PDeltaAnalysis itself (as opposed to the element formulas,
// already validated separately in test_frame_element.cpp).
static void testPDeltaAmplificationMatchesDirectTwoDofSolve() {
    double E_kPa = 25.0e6, L = 4.0;
    Material conc(1, "test", MaterialKind::Concrete, E_kPa, 0.2, 23.6);
    Section sec;  // zero shear area, matching test_frame_element.cpp's convention
    sec.area = 0.09;
    sec.momentOfInertiaZ = 0.3 * std::pow(0.3, 3) / 12.0;
    sec.momentOfInertiaY = sec.momentOfInertiaZ;
    sec.torsionalConstant = sec.momentOfInertiaZ * 2.0;

    double axialCompression = 1500.0;  // kN, well below the ~2622 kN buckling load found earlier
    double H = 10.0;                    // kN lateral

    Model model("cantilever pdelta");
    Node base(1, 0, 0, 0), tip(2, 0, 0, L);
    base.restrainAll();
    // For a VERTICAL member, FrameElement3D's default local axes are
    // ex=global Z (axial), ey=global X, ez=global Y — and since the
    // SAME rotation maps both translations and rotations, that gives
    // local Rx=global Rz (torsion), local Ry=global Rx, local
    // Rz=global Ry. So bending plane 1 (local Uy/Rz) is (global Ux,
    // global Ry) — NOT global Rz, which the first version of this test
    // restrained by mistake, silently locking out the very bending
    // plane the lateral load needs and (separately) restraining Uz
    // (the AXIAL direction here) blocked the gravity load path too,
    // producing an axial force of exactly zero in the first attempt at
    // this test. Restrain only what plane-2 bending (global Uy/Rx) and
    // torsion (global Rz) would otherwise contribute, leaving Ux, Ry
    // (bending), and Uz (axial) free.
    tip.restrain(DOF::Uy);
    tip.restrain(DOF::Rx);
    tip.restrain(DOF::Rz);
    model.addNode(base);
    model.addNode(tip);
    model.addMaterial(conc);
    int secId = model.addSection(sec);
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));

    LoadCase gravity(1, "Gravity", LoadCaseType::Dead);
    LoadCase lateral(2, "Lateral", LoadCaseType::Wind);
    model.addLoadCase(gravity);
    model.addLoadCase(lateral);
    // Axial load: compressive force pushes DOWN on the free tip of a
    // VERTICAL column -> a force in -global Z at the tip node.
    model.addNodalLoad(NodalLoad{2, 1, 0.0, 0.0, -axialCompression, 0.0, 0.0, 0.0});
    model.addNodalLoad(NodalLoad{2, 2, H, 0.0, 0.0, 0.0, 0.0, 0.0});

    PDeltaAnalysis pdelta(model);
    auto result = pdelta.run(1, 2);
    assert(result.skippedElementIds.empty());
    assert(result.maxFreeDofResidualNorm < 1e-4);

    // Independent direct 2x2 solve: Ke_sub + P*Kg_sub (P tension-positive
    // -> compression means P = -axialCompression), solved for [Ux, Ry]
    // at the tip under lateral force H (only the Ux component has an
    // applied force; Ry has none).
    FrameElement3D elem({0, 0, 0}, {0, 0, L}, conc, sec);
    Matrix Ke = elem.localStiffness();
    Matrix Kg = elem.geometricStiffnessGlobal(-axialCompression);
    Matrix KeGlobal = elem.globalStiffness();
    // For a vertical member with the library's default axis convention
    // (ex=global Z, ey=global X, ez=global Y — see FrameElement3D's own
    // doc comment / test_frame_element.cpp's vertical-member test),
    // local Uy IS global Ux directly, so indices 7 (node2 local Uy) and
    // 11 (node2 local Rz) map directly to node2's global Ux and Rz.
    Matrix Ktotal = KeGlobal + Kg;
    double K11 = Ktotal(6, 6), K12 = Ktotal(6, 10), K22 = Ktotal(10, 10);
    // 2x2 solve for [Ux, Ry-equivalent] under {H, 0}: Cramer's rule.
    double det = K11 * K22 - K12 * K12;
    double expectedUx = (H * K22 - 0.0 * K12) / det;

    assert(approxEqual(result.displacements[2].Ux, expectedUx, 1e-4));

    // Amplification check: the SAME lateral case run through an
    // ordinary StaticAnalysis (no P-Delta) must give a SMALLER
    // displacement — compression softens the structure, so P-Delta
    // must amplify, not merely differ.
    StaticAnalysis ordinary(model);
    auto ordinaryResult = ordinary.run(2);
    assert(result.displacements[2].Ux > ordinaryResult.displacements[2].Ux);

    double amplification = result.displacements[2].Ux / ordinaryResult.displacements[2].Ux;
    std::cout << "  testPDeltaAmplificationMatchesDirectTwoDofSolve OK (u_pdelta="
              << result.displacements[2].Ux << "m, u_ordinary=" << ordinaryResult.displacements[2].Ux
              << "m, amplification=" << amplification << ")\n";
}

static void testZeroAxialLoadMatchesOrdinaryStaticAnalysis() {
    // With NO axial load in the gravity case, P-Delta's geometric
    // stiffness contribution is exactly zero, so its result must match
    // an ordinary StaticAnalysis of the lateral case exactly (not just
    // approximately — Kg(P=0) is algebraically the zero matrix).
    double L = 4.0;
    Material conc = Material::concrete(1, 28.0);
    Section sec = Section::rectangular(0.3, 0.3);

    Model model("no axial load");
    Node base(1, 0, 0, 0), tip(2, 0, 0, L);
    base.restrainAll();
    // Same corrected restraint pattern as the test above — see its
    // comment for the local/global axis mapping this depends on.
    tip.restrain(DOF::Uy);
    tip.restrain(DOF::Rx);
    tip.restrain(DOF::Rz);
    model.addNode(base);
    model.addNode(tip);
    model.addMaterial(conc);
    int secId = model.addSection(sec);
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, secId, "C1"));

    LoadCase gravity(1, "Gravity (empty)", LoadCaseType::Dead);
    LoadCase lateral(2, "Lateral", LoadCaseType::Wind);
    model.addLoadCase(gravity);
    model.addLoadCase(lateral);
    // No load in case 1 at all.
    model.addNodalLoad(NodalLoad{2, 2, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    PDeltaAnalysis pdelta(model);
    auto pdeltaResult = pdelta.run(1, 2);
    StaticAnalysis ordinary(model);
    auto ordinaryResult = ordinary.run(2);

    assert(approxEqual(pdeltaResult.displacements[2].Ux, ordinaryResult.displacements[2].Ux, 1e-8));
    std::cout << "  testZeroAxialLoadMatchesOrdinaryStaticAnalysis OK\n";
}

int main() {
    std::cout << "test_pdelta:\n";
    testPDeltaAmplificationMatchesDirectTwoDofSolve();
    testZeroAxialLoadMatchesOrdinaryStaticAnalysis();
    std::cout << "All P-Delta tests passed.\n";
    return 0;
}
