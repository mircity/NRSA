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

static bool approxEqual(double a, double b, double relTol = 1e-3) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// A single cantilevered slab panel — one edge fully fixed, the opposite
// edge free — under a vertical point load at a free corner. No
// closed-form hand calc attempted (a plate's exact solution needs a
// Navier series), but global equilibrium — total applied load must
// equal the negative sum of vertical reactions — is a model-independent
// check any correctly assembled/solved shell model must satisfy, the
// same kind of check the portal-frame test used for FrameElement3D.
static void testCantileveredSlabPanelEquilibrium() {
    Model model("cantilevered slab panel");
    double a = 3.0, b = 2.0, t = 0.15;
    Node n1(1, 0, 0, 0), n2(2, a, 0, 0), n3(3, a, b, 0), n4(4, 0, b, 0);
    // Fix the whole edge n1-n2 (all 6 DOF at each), leave n3, n4 free.
    n1.restrainAll();
    n2.restrainAll();
    model.addNode(n1);
    model.addNode(n2);
    model.addNode(n3);
    model.addNode(n4);

    Material conc = Material::concrete(1, 28.0);
    model.addMaterial(conc);
    int shellSec = model.addSection(Section::shell(t));
    model.addElement(Element(1, ElementKind::Slab, {1, 2, 3, 4}, 1, shellSec, "S1"));

    LoadCase lc(1, "Point Load", LoadCaseType::Live);
    model.addLoadCase(lc);
    double P = 10.0;  // kN downward at the free corner n3
    model.addNodalLoad(NodalLoad{3, 1, 0.0, 0.0, -P, 0.0, 0.0, 0.0});

    StaticAnalysis fea(model);
    auto result = fea.run(1);

    double sumRz = result.reactions[1].Uz + result.reactions[2].Uz;
    // Reaction is the force the support exerts on the structure;
    // equilibrium: applied + reaction = 0 -> sumRz should equal +P
    // (support pushes UP to balance the downward applied load).
    assert(approxEqual(sumRz, P, 1e-2));
    assert(result.maxFreeDofResidualNorm < 1e-4);
    assert(result.skippedElementIds.empty());
    // The loaded free corner should deflect downward (negative Uz).
    assert(result.displacements[3].Uz < 0.0);
    std::cout << "  testCantileveredSlabPanelEquilibrium OK\n";
}

// THE FLAGSHIP TEST: four columns supporting a single slab panel at
// their tops, all sharing nodes (not just touching) — a genuinely
// COUPLED shell+frame model solved in ONE system, which is the entire
// point of building fem::ShellElement and wiring it into
// StaticAnalysis. A point load applied at just ONE column's top node
// is compared between two models built from the SAME columns:
//   - FRAME-ONLY (no slab element connecting the tops): each column is
//     then just an independent cantilever. The loaded column carries
//     the entire load; the other three, having no load applied and no
//     connection to anything that does, carry EXACTLY zero — not
//     "very little", identically zero, because nothing links them to
//     the load at all.
//   - SLAB-COUPLED (the same 4 columns, now with a shell tying their
//     tops together): the unloaded columns must now show a genuinely
//     NONZERO reaction — proof that real structural coupling is
//     happening through the shell's bending stiffness, something a
//     frame-only model is structurally incapable of producing at all,
//     regardless of how large or small that redistributed share turns
//     out to be. (Columns this stiff axially relative to a plate's
//     bending stiffness legitimately DO carry the overwhelming
//     majority of a point load straight down the loaded column in real
//     structures too — the interesting result here is that the other
//     three columns pick up ANY force, including one showing slight
//     uplift, which is genuine plate-bending physics: a corner load on
//     a point-supported plate can put an adjacent corner into tension.)
static double runFourColumnModel(bool includeSlab, double P, double reactions[4]) {
    Model model(includeSlab ? "slab on four columns" : "four columns, no slab");
    double a = 4.0, b = 3.0, H = 3.0, t = 0.18;

    Node base1(1, 0, 0, 0), base2(2, a, 0, 0), base3(3, a, b, 0), base4(4, 0, b, 0);
    Node top1(5, 0, 0, H), top2(6, a, 0, H), top3(7, a, b, H), top4(8, 0, b, H);
    base1.restrainAll(); base2.restrainAll(); base3.restrainAll(); base4.restrainAll();
    model.addNode(base1); model.addNode(base2); model.addNode(base3); model.addNode(base4);
    model.addNode(top1);  model.addNode(top2);  model.addNode(top3);  model.addNode(top4);

    Material conc = Material::concrete(1, 28.0);
    model.addMaterial(conc);
    int colSec = model.addSection(Section::rectangular(0.35, 0.35));

    model.addElement(Element(1, ElementKind::Column, {1, 5}, 1, colSec, "C1"));
    model.addElement(Element(2, ElementKind::Column, {2, 6}, 1, colSec, "C2"));
    model.addElement(Element(3, ElementKind::Column, {3, 7}, 1, colSec, "C3"));
    model.addElement(Element(4, ElementKind::Column, {4, 8}, 1, colSec, "C4"));
    if (includeSlab) {
        int slabSec = model.addSection(Section::shell(t));
        model.addElement(Element(5, ElementKind::Slab, {5, 6, 7, 8}, 1, slabSec, "S1"));
    }

    LoadCase lc(1, "Point Load", LoadCaseType::Live);
    model.addLoadCase(lc);
    model.addNodalLoad(NodalLoad{5, 1, 0.0, 0.0, -P, 0.0, 0.0, 0.0});  // downward at column 1's top only

    StaticAnalysis fea(model);
    auto result = fea.run(1);
    reactions[0] = result.reactions[1].Uz;
    reactions[1] = result.reactions[2].Uz;
    reactions[2] = result.reactions[3].Uz;
    reactions[3] = result.reactions[4].Uz;
    return result.maxFreeDofResidualNorm;
}

static void testCoupledSlabOnColumnsRedistributesLoad() {
    double P = 40.0;
    double rFrameOnly[4], rSlabCoupled[4];

    double residFrame = runFourColumnModel(false, P, rFrameOnly);
    assert(residFrame < 1e-4);
    // Frame-only: columns 2-4 carry EXACTLY zero — nothing connects
    // them to the load at all.
    assert(approxEqual(rFrameOnly[0], P, 1e-3));
    assert(std::abs(rFrameOnly[1]) < 1e-9);
    assert(std::abs(rFrameOnly[2]) < 1e-9);
    assert(std::abs(rFrameOnly[3]) < 1e-9);

    double residSlab = runFourColumnModel(true, P, rSlabCoupled);
    assert(residSlab < 1e-4);
    // Global equilibrium still holds with the slab present.
    double sum = rSlabCoupled[0] + rSlabCoupled[1] + rSlabCoupled[2] + rSlabCoupled[3];
    assert(approxEqual(sum, P, 1e-2));
    // The real point: with the slab, at least one previously-exactly-
    // zero column reaction is now genuinely, meaningfully nonzero —
    // something only possible because the shell physically ties the
    // column tops together. A threshold well above solver/round-off
    // noise (1e-9 in the frame-only case above) but well below "a
    // large fraction of P", since real coupling through a plate this
    // much less axially stiff than the columns legitimately stays
    // small in magnitude.
    bool anyRedistributed = std::abs(rSlabCoupled[1]) > 1e-4 || std::abs(rSlabCoupled[2]) > 1e-4 ||
                             std::abs(rSlabCoupled[3]) > 1e-4;
    assert(anyRedistributed);
    // The loaded column still carries the large majority — physically
    // correct given a stiff column against a comparatively flexible
    // plate, not a sign the coupling "isn't working".
    assert(rSlabCoupled[0] > 0.9 * P);

    std::cout << "  testCoupledSlabOnColumnsRedistributesLoad OK\n"
              << "    frame-only:   R1=" << rFrameOnly[0] << " R2=" << rFrameOnly[1]
              << " R3=" << rFrameOnly[2] << " R4=" << rFrameOnly[3] << " kN\n"
              << "    slab-coupled: R1=" << rSlabCoupled[0] << " R2=" << rSlabCoupled[1]
              << " R3=" << rSlabCoupled[2] << " R4=" << rSlabCoupled[3] << " kN (sum="
              << sum << ")\n";
}

int main() {
    std::cout << "test_shell_static_analysis:\n";
    testCantileveredSlabPanelEquilibrium();
    testCoupledSlabOnColumnsRedistributesLoad();
    std::cout << "All shell static analysis tests passed.\n";
    return 0;
}
