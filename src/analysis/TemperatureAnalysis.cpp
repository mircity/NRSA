#include "analysis/TemperatureAnalysis.h"

#include <array>
#include <cmath>
#include <stdexcept>

#include "analysis/GlobalAssembly.h"

namespace nrsa::analysis {

double axialForceFromInitialStrain(double E_kPa, double area_m2, double initialStrain) {
    return E_kPa * area_m2 * initialStrain;
}

std::vector<AxialInitialStrainResult> applyUniformTemperatureChange(
    Model& model, const std::vector<int>& elementIds, double deltaTDegC,
    double alphaPerDegC, int loadCaseId) {
    std::vector<AxialInitialStrainResult> results;
    double eps0 = alphaPerDegC * deltaTDegC;

    for (int elementId : elementIds) {
        if (!model.hasElement(elementId)) {
            throw std::invalid_argument(
                "applyUniformTemperatureChange: no element with id " + std::to_string(elementId));
        }
        const Element& elem = model.element(elementId);
        if (!isFrameLike(elem.kind()) || elem.nodeCount() != 2) {
            throw std::invalid_argument(
                "applyUniformTemperatureChange: element " + std::to_string(elementId) +
                " is not a 2-node frame-like element (Beam/Column/Brace)");
        }
        const Node& n1 = model.node(elem.nodeId(0));
        const Node& n2 = model.node(elem.nodeId(1));
        double dx = n2.x() - n1.x(), dy = n2.y() - n1.y(), dz = n2.z() - n1.z();
        double L = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (L < 1e-9) {
            throw std::invalid_argument(
                "applyUniformTemperatureChange: element " + std::to_string(elementId) +
                " has zero length");
        }
        std::array<double, 3> ex{dx / L, dy / L, dz / L};

        double E = model.material(elem.materialId()).elasticModulusKPa();
        double A = model.section(elem.sectionId()).area;
        double F = axialForceFromInitialStrain(E, A, eps0);  // pushes ends apart when eps0 > 0

        NodalLoad l1;
        l1.nodeId = n1.id();
        l1.loadCaseId = loadCaseId;
        l1.Fx = -F * ex[0]; l1.Fy = -F * ex[1]; l1.Fz = -F * ex[2];
        model.addNodalLoad(l1);

        NodalLoad l2;
        l2.nodeId = n2.id();
        l2.loadCaseId = loadCaseId;
        l2.Fx = F * ex[0]; l2.Fy = F * ex[1]; l2.Fz = F * ex[2];
        model.addNodalLoad(l2);

        results.push_back({elementId, F});
    }
    return results;
}

}  // namespace nrsa::analysis
