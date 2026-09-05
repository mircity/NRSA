// Minimal end-to-end example: build a small single-bay portal frame
// (two columns + one beam), apply a lateral load, solve it, and print
// displacements/reactions/element forces — the smallest complete
// demonstration of core::Model -> analysis::StaticAnalysis.
//
// Run from the build directory after `cmake --build .`:
//   ./portal_frame_example
#include <iomanip>
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

int main() {
    Model model("Single-bay portal frame example");

    // Geometry: 3.5m story height, 5m bay width.
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

    LoadCase lateral(1, "Wind-X", LoadCaseType::Wind);
    model.addLoadCase(lateral);
    model.addNodalLoad(NodalLoad{2, 1, 30.0, 0.0, 0.0, 0.0, 0.0, 0.0});  // 30 kN lateral at roof

    StaticAnalysis fea(model);
    auto result = fea.run(1);

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "Displacements (global):\n";
    for (const auto& n : model.nodes()) {
        const auto& d = result.displacements[n.id()];
        std::cout << "  " << n.label() << ": Ux=" << d.Ux << "m Uz=" << d.Uz
                  << "m Ry=" << d.Ry << "rad\n";
    }

    std::cout << "\nReactions (restrained DOFs only):\n";
    for (const auto& n : model.nodes()) {
        if (!n.isFullyFixed()) continue;
        const auto& r = result.reactions[n.id()];
        std::cout << "  " << n.label() << ": Rx=" << r.Ux << "kN Rz=" << r.Uz
                  << "kN My=" << r.Ry << "kN.m\n";
    }

    std::cout << "\nElement local end forces:\n";
    for (const auto& e : model.elements()) {
        const auto& f = result.elementForces[e.id()];
        std::cout << "  " << e.label() << ": axial1=" << f.axial1 << "kN shearY1=" << f.shearY1
                  << "kN momentZ1=" << f.momentZ1 << "kN.m  |  axial2=" << f.axial2
                  << "kN shearY2=" << f.shearY2 << "kN momentZ2=" << f.momentZ2 << "kN.m\n";
    }

    std::cout << "\nMax free-DOF equilibrium residual: " << result.maxFreeDofResidualNorm
              << " (should be ~0)\n";
    return 0;
}
