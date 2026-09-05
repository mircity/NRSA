#pragma once

#include <array>
#include <optional>
#include <vector>

namespace nrsa::recognition {

// Roadmap Section 4 ("Intelligent Auto Recognition"): the roadmap's own
// text frames this as "AI/CV-assisted recognition" of an imported
// architectural floor plan, detecting Wall/Column/Door/Window/Slab.
//
// SCOPE, STATED PLAINLY: genuine image-based computer vision (taking a
// raster/scanned floor plan and running a trained detection model over
// it) needs a CV/ML stack and trained weights this offline environment
// does not have -- faking that with a placeholder would be dishonest.
// What IS implementable, and genuinely useful, is GEOMETRIC pattern
// recognition over VECTOR CAD line data (e.g. from io::importDxf) --
// the same class of algorithm real CAD-to-BIM tools use for
// vector-format floor plans: a column is a small closed rectangular
// loop of 4 lines; a wall drawn in the common architectural
// double-line convention is two long parallel lines a consistent
// short distance apart. This module does exactly that classification.
//
// FURTHER SCOPE LIMIT: this does NOT do automatic loop-finding or
// line-pairing over an unstructured "soup" of line segments (that's a
// graph/topology search — group lines by shared endpoints, which is a
// substantial algorithm in its own right and a natural next addition).
// It CLASSIFIES candidate groups the caller already identified (e.g.
// a DXF's own closed-POLYLINE entities, or line groups from a
// separate clustering pass) as Column / Wall / neither. Doors and
// windows are not addressed here at all -- recognizing them needs
// symbol/block matching (a door swing arc, a window's double-line-
// with-gap convention), a distinct pattern class from the two
// implemented here.
struct Point3 {
    double x = 0, y = 0, z = 0;
};
struct LineSegment {
    Point3 p1, p2;
};

struct DetectedColumn {
    Point3 centroid;
    double widthM = 0.0;
    double depthM = 0.0;
};

// Checks whether 4 line segments form a closed, (near-)rectangular
// loop within a small tolerance, AND that the rectangle's dimensions
// both fall within [minSizeM, maxSizeM] (the size window that
// distinguishes "this rectangle is a column's cross-section" from "this
// rectangle is a room outline" or "this rectangle is a rug pattern")
// -- returns std::nullopt if either check fails.
std::optional<DetectedColumn> classifyAsColumn(const std::array<LineSegment, 4>& loop,
                                                 double minSizeM = 0.1, double maxSizeM = 1.5,
                                                 double toleranceM = 0.02);

struct DetectedWall {
    Point3 centerlineStart, centerlineEnd;
    double thicknessM = 0.0;
    double lengthM = 0.0;
};

// Checks whether two line segments are parallel, a consistent
// perpendicular distance apart within [minThicknessM, maxThicknessM]
// (the architectural double-line wall convention), and overlap along
// their shared direction by at least minOverlapFraction of the shorter
// segment's length -- returns std::nullopt if any check fails.
std::optional<DetectedWall> classifyAsWall(const LineSegment& a, const LineSegment& b,
                                            double minThicknessM = 0.075, double maxThicknessM = 0.6,
                                            double minOverlapFraction = 0.5, double toleranceM = 0.02);

// Runs classifyAsColumn over every candidate loop and classifyAsWall
// over every candidate line pair, returning the roadmap's own
// "Detected: N Columns, M Walls" style summary.
struct RecognitionSummary {
    std::vector<DetectedColumn> columns;
    std::vector<DetectedWall> walls;
    int unclassifiedLoops = 0;
    int unclassifiedPairs = 0;
};
RecognitionSummary runAutoRecognition(const std::vector<std::array<LineSegment, 4>>& candidateLoops,
                                       const std::vector<std::pair<LineSegment, LineSegment>>& candidatePairs);

// Door and Window recognition, added 2026-08-27. SCOPE: like
// classifyAsColumn/classifyAsWall above, these classify an ALREADY-
// IDENTIFIED opening (a gap in a wall's double-line, which finding in
// the first place needs the same kind of segment-chaining/gap-
// detection logic classifyAsColumn uses for closed loops, generalized
// to an OPEN chain with a gap -- not built here, see scope note below)
// using the extra entity that distinguishes a door from a window in
// standard 2D architectural CAD convention:
//   - a DOOR has a swing ARC near the opening, of a radius
//     approximately equal to the opening width (the arc traces the
//     door leaf swinging open, radius = leaf length = opening width),
//     spanning roughly a quarter turn (60-100 degrees is the usual
//     drawn range, covering nearly-square to slightly-eased swings).
//   - a WINDOW has no swing arc, but DOES have one or more short
//     parallel lines drawn WITHIN the opening gap (the frame/sill/
//     mullion lines) -- an opening with neither an arc nor any
//     interior lines is ambiguous (could be an undrawn/rough opening)
//     and is classified as neither.
//
// FURTHER SCOPE LIMIT: finding the gap itself (walking an otherwise-
// continuous double-line wall and noticing where it breaks) is NOT
// implemented -- exactly the same "caller supplies the candidate,
// this classifies it" boundary as every other function in this file.
struct CircularArc {
    Point3 center;
    double radiusM = 0.0;
    double startAngleDeg = 0.0;  // in the arc's own plane (2D floor-plan convention: XY plane, z constant)
    double endAngleDeg = 0.0;    // measured counterclockwise from startAngleDeg
};

struct DetectedDoor {
    Point3 hingePoint;
    double widthM = 0.0;
    double swingDegrees = 0.0;
};

// Classifies a wall opening of the given width as a door if a nearby
// swing arc's radius matches the opening width (within toleranceM) and
// its angular span falls within [minSwingDeg, maxSwingDeg] -- returns
// std::nullopt otherwise. The hinge point is the arc's own center
// (that IS where a door's hinge is, by the geometry of how the swing
// arc is drawn).
std::optional<DetectedDoor> classifyAsDoorOpening(double openingWidthM, const CircularArc& swingArc,
                                                   double minWidthM = 0.6, double maxWidthM = 1.2,
                                                   double minSwingDeg = 60.0, double maxSwingDeg = 100.0,
                                                   double toleranceM = 0.05);

struct DetectedWindow {
    double widthM = 0.0;
    int sillLineCount = 0;
};

// Classifies a wall opening of the given width as a window if its
// width falls within [minWidthM, maxWidthM] AND at least one frame/
// sill line was found drawn inside the gap (sillLineCountInGap >= 1 --
// an opening with zero interior lines is NOT classified as a window,
// since a plain gap with nothing else is ambiguous, see class doc
// comment).
std::optional<DetectedWindow> classifyAsWindowOpening(double openingWidthM, int sillLineCountInGap,
                                                       double minWidthM = 0.4, double maxWidthM = 3.0);

}  // namespace nrsa::recognition
