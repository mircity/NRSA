#include "modeler/OpeningGenerator.h"

#include <array>
#include <stdexcept>

#include "core/Element.h"
#include "core/Node.h"

namespace nrsa::modeler {

namespace {

std::array<double, 3> bilinear(const std::array<double, 3>& p1, const std::array<double, 3>& p2,
                                const std::array<double, 3>& p3, const std::array<double, 3>& p4,
                                double u, double v) {
    std::array<double, 3> result{};
    for (int k = 0; k < 3; ++k) {
        result[static_cast<std::size_t>(k)] =
            (1 - u) * (1 - v) * p1[static_cast<std::size_t>(k)] + u * (1 - v) * p2[static_cast<std::size_t>(k)] +
            u * v * p3[static_cast<std::size_t>(k)] + (1 - u) * v * p4[static_cast<std::size_t>(k)];
    }
    return result;
}

}  // namespace

PanelOpeningResult generatePanelWithRectangularOpening(
    Model& model, int n1, int n2, int n3, int n4, double uStart, double uEnd, double vStart,
    double vEnd, int materialId, int sectionId, ElementKind kind, int& nextNodeId,
    int& nextElementId, const std::string& namePrefix) {
    if (!(0.0 < uStart && uStart < uEnd && uEnd < 1.0) || !(0.0 < vStart && vStart < vEnd && vEnd < 1.0)) {
        throw std::invalid_argument(
            "generatePanelWithRectangularOpening: uStart/uEnd/vStart/vEnd must satisfy "
            "0 < start < end < 1");
    }

    std::array<double, 3> p1{model.node(n1).x(), model.node(n1).y(), model.node(n1).z()};
    std::array<double, 3> p2{model.node(n2).x(), model.node(n2).y(), model.node(n2).z()};
    std::array<double, 3> p3{model.node(n3).x(), model.node(n3).y(), model.node(n3).z()};
    std::array<double, 3> p4{model.node(n4).x(), model.node(n4).y(), model.node(n4).z()};

    double uVals[4] = {0.0, uStart, uEnd, 1.0};
    double vVals[4] = {0.0, vStart, vEnd, 1.0};

    // grid[vi][ui] -> node id, for the 4x4 parametric grid.
    int grid[4][4];
    PanelOpeningResult result;
    for (int vi = 0; vi < 4; ++vi) {
        for (int ui = 0; ui < 4; ++ui) {
            bool isCorner = (ui == 0 || ui == 3) && (vi == 0 || vi == 3);
            if (isCorner) {
                if (ui == 0 && vi == 0) grid[vi][ui] = n1;
                else if (ui == 3 && vi == 0) grid[vi][ui] = n2;
                else if (ui == 3 && vi == 3) grid[vi][ui] = n3;
                else grid[vi][ui] = n4;  // ui==0 && vi==3
            } else {
                auto pos = bilinear(p1, p2, p3, p4, uVals[ui], vVals[vi]);
                int id = nextNodeId++;
                Node n(id, pos[0], pos[1], pos[2],
                       namePrefix + "_grid_u" + std::to_string(ui) + "_v" + std::to_string(vi));
                model.addNode(n);
                grid[vi][ui] = id;
                result.newNodeIds.push_back(id);
            }
        }
    }

    // 8 panels: every (ui,vi) cell in the 3x3 arrangement except the
    // center (ui==1, vi==1), which is the opening itself.
    for (int vi = 0; vi < 3; ++vi) {
        for (int ui = 0; ui < 3; ++ui) {
            if (ui == 1 && vi == 1) continue;  // the opening
            int a = grid[vi][ui];
            int b = grid[vi][ui + 1];
            int c = grid[vi + 1][ui + 1];
            int d = grid[vi + 1][ui];
            int id = nextElementId++;
            model.addElement(Element(id, kind, {a, b, c, d}, materialId, sectionId,
                                      namePrefix + "_p" + std::to_string(ui) + "_" + std::to_string(vi)));
            result.panelElementIds.push_back(id);
        }
    }

    return result;
}

}  // namespace nrsa::modeler
