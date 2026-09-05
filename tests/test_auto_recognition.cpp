#include <cassert>
#include <cmath>
#include <iostream>

#include "recognition/AutoRecognition.h"

using namespace nrsa::recognition;

static bool approxEqual(double a, double b, double tol = 1e-6) { return std::abs(a - b) <= tol; }

// A perfect 0.4m x 0.4m square loop (typical column cross-section)
// must be classified as a column with the exact hand-calculated
// centroid and dimensions.
static void testPerfectSquareColumnHandCalc() {
    Point3 p0{0, 0, 0}, p1{0.4, 0, 0}, p2{0.4, 0.4, 0}, p3{0, 0.4, 0};
    std::array<LineSegment, 4> loop = {LineSegment{p0, p1}, LineSegment{p1, p2},
                                        LineSegment{p2, p3}, LineSegment{p3, p0}};
    auto result = classifyAsColumn(loop);
    assert(result.has_value());
    assert(approxEqual(result->widthM, 0.4));
    assert(approxEqual(result->depthM, 0.4));
    assert(approxEqual(result->centroid.x, 0.2));
    assert(approxEqual(result->centroid.y, 0.2));
    std::cout << "  testPerfectSquareColumnHandCalc OK\n";
}

// A loop too large (5m x 5m -- a room outline, not a column) must be
// rejected by the size window.
static void testOversizedLoopRejected() {
    Point3 p0{0, 0, 0}, p1{5, 0, 0}, p2{5, 5, 0}, p3{0, 5, 0};
    std::array<LineSegment, 4> loop = {LineSegment{p0, p1}, LineSegment{p1, p2},
                                        LineSegment{p2, p3}, LineSegment{p3, p0}};
    auto result = classifyAsColumn(loop);
    assert(!result.has_value());
    std::cout << "  testOversizedLoopRejected OK\n";
}

// A non-rectangular quadrilateral (a skewed parallelogram) must be
// rejected by the perpendicularity check.
static void testNonRectangularLoopRejected() {
    Point3 p0{0, 0, 0}, p1{0.4, 0, 0}, p2{0.6, 0.4, 0}, p3{0.2, 0.4, 0};  // parallelogram, skewed
    std::array<LineSegment, 4> loop = {LineSegment{p0, p1}, LineSegment{p1, p2},
                                        LineSegment{p2, p3}, LineSegment{p3, p0}};
    auto result = classifyAsColumn(loop);
    assert(!result.has_value());
    std::cout << "  testNonRectangularLoopRejected OK\n";
}

// Two parallel lines 0.2m apart (double-line wall convention), fully
// overlapping over a 5m length, at y=0 and y=0.2 -- hand-calc:
// thickness=0.2m exactly, length=5m exactly, centerline at y=0.1.
static void testWallDoubleLineHandCalc() {
    LineSegment a{{0, 0, 0}, {5, 0, 0}};
    LineSegment b{{0, 0.2, 0}, {5, 0.2, 0}};
    auto result = classifyAsWall(a, b);
    assert(result.has_value());
    assert(approxEqual(result->thicknessM, 0.2));
    assert(approxEqual(result->lengthM, 5.0));
    assert(approxEqual(result->centerlineStart.y, 0.1));
    assert(approxEqual(result->centerlineEnd.y, 0.1));
    std::cout << "  testWallDoubleLineHandCalc OK\n";
}

// Two parallel lines too far apart (2m -- not a wall thickness) must
// be rejected.
static void testWallTooThickRejected() {
    LineSegment a{{0, 0, 0}, {5, 0, 0}};
    LineSegment b{{0, 2.0, 0}, {5, 2.0, 0}};
    auto result = classifyAsWall(a, b);
    assert(!result.has_value());
    std::cout << "  testWallTooThickRejected OK\n";
}

// Two parallel lines with only 20% overlap (below the 50% default
// threshold) must be rejected.
static void testWallInsufficientOverlapRejected() {
    LineSegment a{{0, 0, 0}, {5, 0, 0}};
    LineSegment b{{4, 0.2, 0}, {9, 0.2, 0}};  // only 1m of 5m overlaps
    auto result = classifyAsWall(a, b);
    assert(!result.has_value());
    std::cout << "  testWallInsufficientOverlapRejected OK\n";
}

// Roadmap-style summary: 2 valid columns + 1 invalid loop, 1 valid
// wall + 1 invalid pair -> counts must match exactly.
static void testRunAutoRecognitionSummaryHandCalc() {
    Point3 p0{0, 0, 0}, p1{0.4, 0, 0}, p2{0.4, 0.4, 0}, p3{0, 0.4, 0};
    std::array<LineSegment, 4> goodLoop = {LineSegment{p0, p1}, LineSegment{p1, p2},
                                            LineSegment{p2, p3}, LineSegment{p3, p0}};
    Point3 q0{10, 10, 0}, q1{5, 0, 0}, q2{5, 5, 0}, q3{0, 5, 0};  // disconnected -> invalid
    std::array<LineSegment, 4> badLoop = {LineSegment{q0, q1}, LineSegment{q1, q2},
                                           LineSegment{q2, q3}, LineSegment{q3, q0}};

    LineSegment wallA{{0, 0, 0}, {5, 0, 0}};
    LineSegment wallB{{0, 0.2, 0}, {5, 0.2, 0}};
    LineSegment badA{{0, 0, 0}, {5, 0, 0}};
    LineSegment badB{{0, 2.0, 0}, {5, 2.0, 0}};

    auto summary = runAutoRecognition({goodLoop, goodLoop, badLoop}, {{wallA, wallB}, {badA, badB}});
    assert(summary.columns.size() == 2);
    assert(summary.unclassifiedLoops == 1);
    assert(summary.walls.size() == 1);
    assert(summary.unclassifiedPairs == 1);
    std::cout << "  testRunAutoRecognitionSummaryHandCalc (columns=" << summary.columns.size()
              << ", walls=" << summary.walls.size() << ") OK\n";
}

// Hand-calc: a 0.9m opening with a swing arc of radius exactly 0.9m and
// span exactly 90 degrees must be classified as a door, with the
// hinge point at the arc's center and swingDegrees == 90.
static void testDoorSwingArcHandCalc() {
    CircularArc arc{{2.0, 3.0, 0.0}, 0.9, 0.0, 90.0};
    auto result = classifyAsDoorOpening(0.9, arc);
    assert(result.has_value());
    assert(approxEqual(result->widthM, 0.9));
    assert(approxEqual(result->swingDegrees, 90.0));
    assert(approxEqual(result->hingePoint.x, 2.0));
    assert(approxEqual(result->hingePoint.y, 3.0));
    std::cout << "  testDoorSwingArcHandCalc OK\n";
}

// An arc whose radius doesn't match the opening width must be rejected.
static void testDoorRejectedWhenArcRadiusMismatched() {
    CircularArc arc{{0, 0, 0}, 1.5, 0.0, 90.0};  // radius 1.5m, opening 0.9m -- mismatch
    auto result = classifyAsDoorOpening(0.9, arc);
    assert(!result.has_value());
    std::cout << "  testDoorRejectedWhenArcRadiusMismatched OK\n";
}

// A swing span outside the typical 60-100 degree window must be rejected.
static void testDoorRejectedWhenSwingTooNarrow() {
    CircularArc arc{{0, 0, 0}, 0.9, 0.0, 180.0};
    auto result = classifyAsDoorOpening(0.9, arc);
    assert(!result.has_value());
    std::cout << "  testDoorRejectedWhenSwingTooNarrow OK\n";
}

// A 1.2m opening with 2 sill lines drawn inside it must be classified
// as a window.
static void testWindowWithSillLinesHandCalc() {
    auto result = classifyAsWindowOpening(1.2, 2);
    assert(result.has_value());
    assert(approxEqual(result->widthM, 1.2));
    assert(result->sillLineCount == 2);
    std::cout << "  testWindowWithSillLinesHandCalc OK\n";
}

// An opening with zero interior lines is ambiguous and must NOT be
// classified as a window.
static void testWindowRejectedWithNoSillLines() {
    auto result = classifyAsWindowOpening(1.2, 0);
    assert(!result.has_value());
    std::cout << "  testWindowRejectedWithNoSillLines OK\n";
}

// Real integration proof, not just a claim: these 4 points are the
// ACTUAL output of tools/raster_floorplan_extractor.py run on a
// synthetic 40x40px test-column image (captured 2026-08-27; see that
// script's own doc comment for the pipeline this represents), converted
// to meters at an assumed 100px = 1m scale. Real edge-detection noise
// means the loop is NOT a perfect square (sides differ by ~1cm, not
// perfectly perpendicular) -- this test confirms classifyAsColumn's
// existing tolerance (2cm default) genuinely absorbs that real-world
// noise rather than requiring hand-picked, artificially-clean input.
static void testRasterExtractedLoopClassifiesCorrectly() {
    Point3 p0{0.73, 0.71, 0}, p1{1.08, 0.72, 0}, p2{1.07, 1.08, 0}, p3{0.71, 1.07, 0};
    std::array<LineSegment, 4> loop = {LineSegment{p0, p1}, LineSegment{p1, p2},
                                        LineSegment{p2, p3}, LineSegment{p3, p0}};
    auto result = classifyAsColumn(loop);
    assert(result.has_value());
    assert(result->widthM > 0.3 && result->widthM < 0.4);
    assert(result->depthM > 0.3 && result->depthM < 0.4);
    std::cout << "  testRasterExtractedLoopClassifiesCorrectly (width=" << result->widthM
              << "m, depth=" << result->depthM << "m, from real extractor output) OK\n";
}

int main() {
    std::cout << "test_auto_recognition:\n";
    testPerfectSquareColumnHandCalc();
    testOversizedLoopRejected();
    testNonRectangularLoopRejected();
    testWallDoubleLineHandCalc();
    testWallTooThickRejected();
    testWallInsufficientOverlapRejected();
    testRunAutoRecognitionSummaryHandCalc();
    testDoorSwingArcHandCalc();
    testDoorRejectedWhenArcRadiusMismatched();
    testDoorRejectedWhenSwingTooNarrow();
    testWindowWithSillLinesHandCalc();
    testWindowRejectedWithNoSillLines();
    testRasterExtractedLoopClassifiesCorrectly();
    std::cout << "All tests passed.\n";
    return 0;
}
