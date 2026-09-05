#include "recognition/AutoRecognition.h"

#include <algorithm>
#include <cmath>

namespace nrsa::recognition {

namespace {

double dist(const Point3& a, const Point3& b) {
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Point3 sub(const Point3& a, const Point3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Point3 add(const Point3& a, const Point3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Point3 scale(const Point3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(const Point3& a, const Point3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double norm(const Point3& a) { return std::sqrt(dot(a, a)); }

}  // namespace

std::optional<DetectedColumn> classifyAsColumn(const std::array<LineSegment, 4>& loop, double minSizeM,
                                                 double maxSizeM, double toleranceM) {
    // Chain the 4 segments into vertices, requiring each consecutive
    // pair to share an endpoint within toleranceM (loop[0].p1 is taken
    // as the starting vertex).
    std::array<Point3, 4> v;
    v[0] = loop[0].p1;
    Point3 cur = loop[0].p2;
    v[1] = cur;
    for (int i = 1; i < 3; ++i) {
        const LineSegment& seg = loop[static_cast<std::size_t>(i)];
        Point3 next;
        if (dist(seg.p1, cur) <= toleranceM) next = seg.p2;
        else if (dist(seg.p2, cur) <= toleranceM) next = seg.p1;
        else return std::nullopt;  // doesn't connect -- not a closed loop
        v[static_cast<std::size_t>(i + 1)] = next;
        cur = next;
    }
    // The 4th segment must close the loop back to v[0].
    const LineSegment& last = loop[3];
    bool closes = (dist(last.p1, cur) <= toleranceM && dist(last.p2, v[0]) <= toleranceM) ||
                  (dist(last.p2, cur) <= toleranceM && dist(last.p1, v[0]) <= toleranceM);
    if (!closes) return std::nullopt;

    // Rectangularity: opposite sides equal length, adjacent sides perpendicular.
    double side01 = dist(v[0], v[1]), side12 = dist(v[1], v[2]);
    double side23 = dist(v[2], v[3]), side30 = dist(v[3], v[0]);
    if (std::abs(side01 - side23) > toleranceM || std::abs(side12 - side30) > toleranceM) {
        return std::nullopt;
    }
    Point3 e01 = sub(v[1], v[0]), e12 = sub(v[2], v[1]);
    double cosAngle = dot(e01, e12) / (norm(e01) * norm(e12) + 1e-12);
    if (std::abs(cosAngle) > 0.05) return std::nullopt;  // not ~90 degrees (cos(90+/-3deg) ~ 0.05)

    double width = side01, depth = side12;
    if (width < minSizeM || width > maxSizeM || depth < minSizeM || depth > maxSizeM) {
        return std::nullopt;
    }

    Point3 centroid = scale(add(add(v[0], v[1]), add(v[2], v[3])), 0.25);
    return DetectedColumn{centroid, width, depth};
}

std::optional<DetectedWall> classifyAsWall(const LineSegment& a, const LineSegment& b,
                                            double minThicknessM, double maxThicknessM,
                                            double minOverlapFraction, double toleranceM) {
    Point3 dirA = sub(a.p2, a.p1);
    Point3 dirB = sub(b.p2, b.p1);
    double lenA = norm(dirA), lenB = norm(dirB);
    if (lenA < 1e-9 || lenB < 1e-9) return std::nullopt;
    Point3 uA = scale(dirA, 1.0 / lenA);
    Point3 uB = scale(dirB, 1.0 / lenB);

    // Parallel check (allow anti-parallel too, since a wall's two edge
    // lines are commonly drawn in opposite winding directions).
    double cosAngle = dot(uA, uB);
    if (std::abs(std::abs(cosAngle) - 1.0) > 0.02) return std::nullopt;  // ~within ~8 degrees

    // Perpendicular distance from line B to the infinite line through A.
    Point3 ap1ToBp1 = sub(b.p1, a.p1);
    double alongA = dot(ap1ToBp1, uA);
    Point3 closestOnA = add(a.p1, scale(uA, alongA));
    double perpDist = dist(closestOnA, b.p1);
    if (perpDist < minThicknessM - toleranceM || perpDist > maxThicknessM + toleranceM) {
        return std::nullopt;
    }

    // Overlap along A's direction: project both A's and B's endpoints
    // onto uA, take the overlap of the two intervals.
    double aStart = 0.0, aEnd = lenA;
    double bStart = dot(sub(b.p1, a.p1), uA);
    double bEnd = dot(sub(b.p2, a.p1), uA);
    if (bStart > bEnd) std::swap(bStart, bEnd);
    double overlapStart = std::max(aStart, bStart);
    double overlapEnd = std::min(aEnd, bEnd);
    double overlapLen = std::max(0.0, overlapEnd - overlapStart);
    double shorterLen = std::min(lenA, lenB);
    if (shorterLen < 1e-9 || overlapLen / shorterLen < minOverlapFraction) return std::nullopt;

    Point3 centerlineStart = add(a.p1, scale(uA, overlapStart));
    // Centerline sits halfway between A and B, perpendicular to the
    // shared direction (from A's closest point toward B, half the gap).
    Point3 midpointOffset = scale(sub(b.p1, closestOnA), 0.5);
    Point3 centerStart = add(centerlineStart, midpointOffset);
    Point3 centerEnd = add(add(a.p1, scale(uA, overlapEnd)), midpointOffset);

    return DetectedWall{centerStart, centerEnd, perpDist, overlapLen};
}

RecognitionSummary runAutoRecognition(const std::vector<std::array<LineSegment, 4>>& candidateLoops,
                                       const std::vector<std::pair<LineSegment, LineSegment>>& candidatePairs) {
    RecognitionSummary summary;
    for (const auto& loop : candidateLoops) {
        auto col = classifyAsColumn(loop);
        if (col) summary.columns.push_back(*col);
        else summary.unclassifiedLoops++;
    }
    for (const auto& [a, b] : candidatePairs) {
        auto wall = classifyAsWall(a, b);
        if (wall) summary.walls.push_back(*wall);
        else summary.unclassifiedPairs++;
    }
    return summary;
}

std::optional<DetectedDoor> classifyAsDoorOpening(double openingWidthM, const CircularArc& swingArc,
                                                   double minWidthM, double maxWidthM, double minSwingDeg,
                                                   double maxSwingDeg, double toleranceM) {
    if (openingWidthM < minWidthM || openingWidthM > maxWidthM) return std::nullopt;
    if (std::abs(swingArc.radiusM - openingWidthM) > toleranceM) return std::nullopt;

    double span = swingArc.endAngleDeg - swingArc.startAngleDeg;
    while (span < 0.0) span += 360.0;
    while (span > 360.0) span -= 360.0;
    if (span < minSwingDeg || span > maxSwingDeg) return std::nullopt;

    return DetectedDoor{swingArc.center, openingWidthM, span};
}

std::optional<DetectedWindow> classifyAsWindowOpening(double openingWidthM, int sillLineCountInGap,
                                                       double minWidthM, double maxWidthM) {
    if (openingWidthM < minWidthM || openingWidthM > maxWidthM) return std::nullopt;
    if (sillLineCountInGap < 1) return std::nullopt;
    return DetectedWindow{openingWidthM, sillLineCountInGap};
}

}  // namespace nrsa::recognition
