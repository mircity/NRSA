#pragma once

#include <vector>

#include "core/Model.h"

namespace nrsa::modeler {

// Cuts a rectangular opening into an existing planar quad panel (a
// slab or wall panel) by re-meshing it into 8 quad panels arranged
// around the hole -- the standard "picture frame" mesh every FEM
// modeler uses for a panel-with-a-hole, rather than trying to give a
// single element an actual hole in it (shell elements are solid quads
// -- a hole means MORE, smaller elements around where the hole is, not
// a hole IN an element).
//
// PARAMETERIZATION: the panel is defined by its 4 existing corner node
// ids, ordered n1 (u=0,v=0) -> n2 (u=1,v=0) -> n3 (u=1,v=1) -> n4
// (u=0,v=1) -- the same winding AutoModelGenerator's slabs/walls
// already use, so this function is a drop-in replacement for "add one
// quad element" wherever a panel needs an opening instead. The opening
// itself is given as fractional (0..1) insets along each parametric
// axis (uStart < uEnd, vStart < vEnd, both strictly inside (0,1)) --
// e.g. a centered opening covering the middle third in both directions
// is uStart=1/3, uEnd=2/3, vStart=1/3, vEnd=2/3. Node positions for the
// 12 new grid points are found by bilinear interpolation of the 4
// corner coordinates, which is exact for any PLANAR quad (slabs and
// walls both are, by construction, in this project) but would silently
// distort a non-planar/warped quad -- not a concern here since nothing
// in this codebase creates warped panels, but worth stating since nothing
// checks for it.
//
// SCOPE: rectangular openings only, one per panel, fully inside the
// panel (not touching an edge) -- an opening flush against one edge
// (common for a door opening reaching the floor) needs a different,
// simpler 6-panel mesh and is a natural next addition, not built here.
struct PanelOpeningResult {
    std::vector<int> newNodeIds;       // the 12 newly-created nodes (corners are reused, not duplicated)
    std::vector<int> panelElementIds;  // the 8 generated quad elements
};

// Generates the 8-panel frame mesh into model, using and incrementing
// nextNodeId/nextElementId in place (the same "caller-owned incrementing
// counter" convention AutoModelGenerator uses, so the two compose
// without id collisions when used together). Throws std::invalid_argument
// if uStart/uEnd/vStart/vEnd are not strictly ordered within (0,1).
PanelOpeningResult generatePanelWithRectangularOpening(
    Model& model, int n1, int n2, int n3, int n4, double uStart, double uEnd, double vStart,
    double vEnd, int materialId, int sectionId, ElementKind kind, int& nextNodeId,
    int& nextElementId, const std::string& namePrefix);

}  // namespace nrsa::modeler
