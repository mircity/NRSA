#pragma once

#include <array>
#include <string>
#include <vector>

#include "core/Element.h"
#include "core/Model.h"

namespace nrsa::drawing {

// ---------------------------------------------------------------------------
// Drawing Generation Engine -- Phase 1 (geometry extraction) + Phase 2
// (2D line-drawing projection), per the scoped plan agreed before writing
// any of this: this file does NOT touch analysis results (Phase 3+) or
// design/reinforcement output (Phase 5+). It only answers "where does this
// 3D model project to on a flat elevation/plan view, in real coordinates
// pulled straight from core::Model" -- nothing here is a placeholder or a
// schematic stand-in the way the old prototype's drawing tab was.
//
// SCOPE / KNOWN LIMITATIONS (stated up front, not discovered later):
// - Orthographic parallel projection onto one of the three principal
//   planes only (XZ, YZ, XY). No perspective, no arbitrary section cut
//   plane, no hidden-line removal -- a column directly behind another in
//   the view direction will simply overlap it. True hidden-line/3D-aware
//   rendering is out of this phase's scope.
// - A member is drawn as a rectangle of width == its Section's `depth`
//   (Phase-2 simplification: the projected in-plane dimension is always
//   taken from Section::depth, not from whichever of depth/width is
//   actually perpendicular to the view for a rotated section). Fine for
//   the common case (rectangular sections with depth as the visually
//   dominant dimension); not yet correct for a member whose section is
//   rotated about its own local axis relative to the view plane.
// - Members that run mostly perpendicular to the view plane (e.g. a
//   Y-direction beam shown on an XZ elevation) still project -- typically
//   to a short or degenerate segment -- exactly as a real orthographic
//   projection would. This phase does not omit or specially flag them;
//   deciding which members a given elevation should even show (grid-line
//   filtering) is Phase 7 (sheet composition), not here.
// - No title block, scale bar, dimension lines, or multi-view sheet
//   layout -- this produces one <svg> containing one view. Phase 7.
// ---------------------------------------------------------------------------

enum class ViewPlane {
    ElevationXZ,  // u = global X, v = global Z (Y ignored) -- "front" elevation
    ElevationYZ,  // u = global Y, v = global Z (X ignored) -- "side" elevation
    PlanXY        // u = global X, v = global Y (Z ignored) -- plan/floor view
};

// A node's position after dropping one global coordinate per ViewPlane.
// Units stay in meters (the same units core::Node stores) -- no pixel
// scaling happens until SVG generation.
struct ProjectedNode {
    int id = 0;
    double u = 0.0;
    double v = 0.0;
    std::string label;
};

// An element's two-point centerline after projection, plus enough section
// data to draw it as a real (if simplified, see header note) rectangle
// rather than a bare line.
struct ProjectedMember {
    int elementId = 0;
    ElementKind kind = ElementKind::Beam;
    std::string label;
    double u1 = 0.0, v1 = 0.0;
    double u2 = 0.0, v2 = 0.0;
    double sectionDepthM = 0.0;   // drawn perpendicular-to-axis width -- see
                                   // header note on the depth-vs-width
                                   // simplification
    std::string sectionName;
};

struct BoundingBoxUV {
    double minU = 0.0, maxU = 0.0, minV = 0.0, maxV = 0.0;
    double width() const { return maxU - minU; }
    double height() const { return maxV - minV; }
};

// Projects every node in the model onto the given plane. Order matches
// model.nodes() (insertion order), not sorted by id.
std::vector<ProjectedNode> projectNodes(const Model& model, ViewPlane plane);

// Projects every 2-node element (Beam/Column/Brace; elements with more
// than 2 nodes -- shells/slabs -- are skipped in this phase, see README)
// onto the given plane, pulling section depth from model.section(id).
std::vector<ProjectedMember> projectElements(const Model& model, ViewPlane plane);

// Bounding box (in projected meters, not pixels) over a set of projected
// nodes -- used by the SVG generator to size the viewBox, but exposed
// separately so callers/tests can check projection correctness without
// generating SVG at all.
BoundingBoxUV computeBoundingBox(const std::vector<ProjectedNode>& nodes);

struct ElevationSvgOptions {
    double pixelsPerMeter = 60.0;
    double marginPx = 40.0;
    double nodeRadiusPx = 3.0;
    bool drawNodeLabels = true;
    bool drawMemberLabels = true;
};

// Renders one view (nodes + member outlines, centerline-only -- no
// analysis overlay, no reinforcement detail; see file header for what's
// deliberately NOT in this phase) as a single self-contained SVG string.
// "Up" in the model (+Z, or +V for a plan view) is drawn toward the top
// of the image, matching normal elevation/plan drawing convention (SVG's
// own y-down coordinate system is flipped internally for this).
std::string generateElevationSvg(const Model& model, ViewPlane plane,
                                  const ElevationSvgOptions& options = {});

}  // namespace nrsa::drawing
