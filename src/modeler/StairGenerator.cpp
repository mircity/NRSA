#include "modeler/StairGenerator.h"

#include <cmath>
#include <stdexcept>

#include "core/Element.h"
#include "core/Node.h"

namespace nrsa::modeler {

namespace {

std::array<double, 3> sub(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
std::array<double, 3> cross(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double norm(const std::array<double, 3>& a) { return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }
std::array<double, 3> scale(const std::array<double, 3>& a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
std::array<double, 3> add(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

}  // namespace

StairFlightResult generateStairFlight(Model& model, std::array<double, 3> bottomPoint,
                                       std::array<double, 3> topPoint, double widthM,
                                       int subdivisionCount, int materialId, int sectionId,
                                       int& nextNodeId, int& nextElementId,
                                       const std::string& namePrefix, std::array<double, 3> upVector) {
    if (subdivisionCount < 1) throw std::invalid_argument("generateStairFlight: subdivisionCount must be >= 1");
    if (widthM <= 0.0) throw std::invalid_argument("generateStairFlight: widthM must be positive");

    std::array<double, 3> run3D = sub(topPoint, bottomPoint);
    double totalLength = norm(run3D);
    if (totalLength < 1e-9) {
        throw std::invalid_argument("generateStairFlight: bottom and top points coincide");
    }

    // Horizontal projection of the run direction (drop the component
    // along upVector) to derive the width direction perpendicular to it.
    double upNorm = norm(upVector);
    std::array<double, 3> upUnit = scale(upVector, 1.0 / upNorm);
    double runDotUp = run3D[0] * upUnit[0] + run3D[1] * upUnit[1] + run3D[2] * upUnit[2];
    std::array<double, 3> runHorizontal = sub(run3D, scale(upUnit, runDotUp));
    double runHorizontalNorm = norm(runHorizontal);
    if (runHorizontalNorm < 1e-9) {
        throw std::invalid_argument(
            "generateStairFlight: run direction is purely vertical -- no horizontal component "
            "to measure a perpendicular width direction against");
    }

    std::array<double, 3> widthDir = cross(upUnit, scale(runHorizontal, 1.0 / runHorizontalNorm));
    double widthDirNorm = norm(widthDir);
    widthDir = scale(widthDir, 1.0 / widthDirNorm);
    std::array<double, 3> halfWidthVec = scale(widthDir, widthM / 2.0);

    StairFlightResult result;
    result.inclineLengthM = totalLength;

    std::vector<int> leftIds, rightIds;
    for (int s = 0; s <= subdivisionCount; ++s) {
        double t = static_cast<double>(s) / subdivisionCount;
        std::array<double, 3> center = add(bottomPoint, scale(run3D, t));
        std::array<double, 3> left = sub(center, halfWidthVec);
        std::array<double, 3> right = add(center, halfWidthVec);

        int leftId = nextNodeId++;
        model.addNode(Node(leftId, left[0], left[1], left[2],
                            namePrefix + "_L" + std::to_string(s)));
        leftIds.push_back(leftId);
        result.nodeIds.push_back(leftId);

        int rightId = nextNodeId++;
        model.addNode(Node(rightId, right[0], right[1], right[2],
                            namePrefix + "_R" + std::to_string(s)));
        rightIds.push_back(rightId);
        result.nodeIds.push_back(rightId);
    }

    for (int s = 0; s < subdivisionCount; ++s) {
        int a = leftIds[static_cast<std::size_t>(s)];
        int b = rightIds[static_cast<std::size_t>(s)];
        int c = rightIds[static_cast<std::size_t>(s + 1)];
        int d = leftIds[static_cast<std::size_t>(s + 1)];
        int id = nextElementId++;
        // Modeled as ElementKind::Slab -- an inclined waist slab is, in
        // every structural sense that matters to the analysis engine, a
        // Slab shell element; there is no dedicated "Stair" ElementKind
        // (see class doc comment).
        model.addElement(Element(id, ElementKind::Slab, {a, b, c, d}, materialId, sectionId,
                                  namePrefix + "_panel" + std::to_string(s)));
        result.panelElementIds.push_back(id);
    }

    return result;
}

}  // namespace nrsa::modeler
