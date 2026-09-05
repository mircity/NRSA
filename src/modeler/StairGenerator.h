#pragma once

#include <array>
#include <vector>

#include "core/Model.h"

namespace nrsa::modeler {

// Generates a single straight stair FLIGHT as an inclined WAIST SLAB
// strip -- the standard structural idealization for stair analysis
// (the actual step/riser/nosing geometry is an architectural, not
// structural, concern; a stair flight is modeled as a flat inclined
// plate of the waist thickness, exactly the way it's done in every
// FEM-based structural package). This is a documented simplification,
// not an oversight -- individual step geometry adds no stiffness or
// load-path information a structural analysis needs.
//
// GEOMETRY: given a bottom landing point and a top landing point (any
// two 3D points -- the horizontal distance between them is the "run",
// the vertical distance is the "rise") and a flight WIDTH (horizontal,
// perpendicular to the run direction), this generates
// subdivisionCount+1 cross-sections evenly spaced along the incline,
// each with 2 nodes (left and right edge of the flight width), and
// subdivisionCount quad shell panels connecting consecutive
// cross-sections -- i.e. exactly the same "N+1 cross-sections, N
// panels" pattern AutoModelGenerator's slab/wall generation already
// uses, applied along an inclined line instead of a horizontal grid
// line.
//
// SCOPE: a SINGLE straight flight only -- no landing, no turn
// (dog-leg/scissor stairs need two flights plus a landing slab, easily
// built by calling this twice with the landing as the shared point,
// but that composition is left to the caller, not automated here).
// The "width direction" is derived from the run direction and a
// caller-supplied UP vector (defaults to global Z) via a cross
// product -- for a flight that isn't perfectly horizontal-run (which
// is every real stair, since it also rises), the width direction is
// horizontal and perpendicular to the run's horizontal projection,
// matching how a real stair's width is measured.
struct StairFlightResult {
    std::vector<int> nodeIds;      // (subdivisionCount+1)*2 nodes, ordered [section][side] (side 0=left,1=right)
    std::vector<int> panelElementIds;
    double inclineLengthM = 0.0;   // sqrt(run^2 + rise^2), the actual sloped length of the waist slab
};

// Generates the flight into model. Throws std::invalid_argument if
// subdivisionCount < 1, widthM <= 0, or bottom and top points coincide
// (zero-length flight) or their horizontal projection is zero-length
// (a purely vertical "flight" has no well-defined run direction to
// measure width perpendicular to).
StairFlightResult generateStairFlight(Model& model, std::array<double, 3> bottomPoint,
                                       std::array<double, 3> topPoint, double widthM,
                                       int subdivisionCount, int materialId, int sectionId,
                                       int& nextNodeId, int& nextElementId,
                                       const std::string& namePrefix,
                                       std::array<double, 3> upVector = {0.0, 0.0, 1.0});

}  // namespace nrsa::modeler
