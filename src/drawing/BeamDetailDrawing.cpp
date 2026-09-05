#include "drawing/BeamDetailDrawing.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace nrsa::drawing {

int stirrupCountForLength(double lengthMm, double coverMm, double stirrupSpacingMm) {
    if (stirrupSpacingMm <= 0.0) throw std::invalid_argument("stirrupCountForLength: spacing must be positive");
    double zoneLength = lengthMm - 2.0 * coverMm;
    if (zoneLength < 0.0) return 0;
    return static_cast<int>(std::floor(zoneLength / stirrupSpacingMm)) + 1;
}

std::vector<double> stirrupPositionsMm(const BeamDetailParams& p) {
    std::vector<double> positions;
    if (p.supportStirrupSpacingMm <= 0.0) {
        // Old uniform-everywhere behavior, unchanged.
        int count = stirrupCountForLength(p.lengthMm, p.coverMm, p.stirrupSpacingMm);
        for (int i = 0; i < count; ++i) positions.push_back(p.coverMm + i * p.stirrupSpacingMm);
        return positions;
    }

    // Graduated spacing: closer stirrups within supportZoneLengthMm of
    // EACH end (ACI 318-19 9.7.6.2.2's real pattern), wider spacing in
    // the middle -- see class doc comment.
    double leftZoneEnd = p.coverMm + p.supportZoneLengthMm;
    double rightZoneStart = p.lengthMm - p.coverMm - p.supportZoneLengthMm;

    for (double x = p.coverMm; x <= leftZoneEnd + 1e-6; x += p.supportStirrupSpacingMm) {
        positions.push_back(x);
    }
    double lastLeft = positions.back();

    std::vector<double> rightZone;
    for (double x = p.lengthMm - p.coverMm; x >= rightZoneStart - 1e-6; x -= p.supportStirrupSpacingMm) {
        rightZone.push_back(x);
    }
    std::reverse(rightZone.begin(), rightZone.end());
    double firstRight = rightZone.front();

    for (double x = lastLeft + p.stirrupSpacingMm; x < firstRight - 1e-6; x += p.stirrupSpacingMm) {
        positions.push_back(x);
    }
    for (double x : rightZone) positions.push_back(x);

    return positions;
}

std::string generateBeamDetailSvg(const BeamDetailParams& p) {
    if (p.lengthMm <= 0.0 || p.widthMm <= 0.0 || p.depthMm <= 0.0) {
        throw std::invalid_argument("generateBeamDetailSvg: length/width/depth must be positive");
    }
    if (p.stirrupSpacingMm <= 0.0) {
        throw std::invalid_argument("generateBeamDetailSvg: stirrupSpacingMm must be positive");
    }
    double maxBarDia = std::max(p.topBarDiaMm, p.bottomBarDiaMm);
    double requiredDepth = 2.0 * (p.coverMm + p.stirrupDiaMm + maxBarDia / 2.0);
    if (requiredDepth > p.depthMm) {
        throw std::invalid_argument(
            "generateBeamDetailSvg: cover+stirrup+bar radius on both faces (" +
            std::to_string(requiredDepth) + " mm) exceeds section depth (" +
            std::to_string(p.depthMm) + " mm)");
    }
    if (p.curtailedBottomBarCount > p.bottomBarCount) {
        throw std::invalid_argument(
            "generateBeamDetailSvg: curtailedBottomBarCount cannot exceed bottomBarCount");
    }

    // Drawing coordinates: a small margin around the member so bar
    // lines/stirrup ticks that sit exactly at the edges are not clipped.
    double margin = 20.0;
    double svgWidth = p.lengthMm + 2 * margin;
    double svgHeight = p.depthMm + 2 * margin;

    double topBarY = margin + p.coverMm + p.stirrupDiaMm + p.topBarDiaMm / 2.0;
    double bottomBarY = margin + p.depthMm - (p.coverMm + p.stirrupDiaMm + p.bottomBarDiaMm / 2.0);

    std::ostringstream svg;
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 " << svgWidth << " " << svgHeight
        << "\">\n";

    // Member outline.
    svg << "  <rect x=\"" << margin << "\" y=\"" << margin << "\" width=\"" << p.lengthMm
        << "\" height=\"" << p.depthMm << "\" fill=\"none\" stroke=\"black\" stroke-width=\"2\"/>\n";

    // Top and bottom longitudinal bar lines (the full-length bars --
    // bottomBarCount minus any curtailed count still run the full length).
    svg << "  <line x1=\"" << margin << "\" y1=\"" << topBarY << "\" x2=\"" << (margin + p.lengthMm)
        << "\" y2=\"" << topBarY << "\" stroke=\"black\" stroke-width=\"" << p.topBarDiaMm / 4.0
        << "\"/>\n";
    svg << "  <line x1=\"" << margin << "\" y1=\"" << bottomBarY << "\" x2=\"" << (margin + p.lengthMm)
        << "\" y2=\"" << bottomBarY << "\" stroke=\"black\" stroke-width=\"" << p.bottomBarDiaMm / 4.0
        << "\"/>\n";

    // Curtailed bottom bars (opt-in): a shorter line, offset slightly
    // below the full-length bottom bar line so both are visible,
    // spanning [curtailmentDistanceFromEndMm, length-curtailmentDistanceFromEndMm].
    if (p.curtailedBottomBarCount > 0) {
        double curtailY = bottomBarY + p.bottomBarDiaMm / 2.0 + 4.0;
        double xStart = margin + p.curtailmentDistanceFromEndMm;
        double xEnd = margin + p.lengthMm - p.curtailmentDistanceFromEndMm;
        svg << "  <line x1=\"" << xStart << "\" y1=\"" << curtailY << "\" x2=\"" << xEnd << "\" y2=\""
            << curtailY << "\" stroke=\"black\" stroke-width=\"" << p.bottomBarDiaMm / 4.0
            << "\" stroke-dasharray=\"6,3\"/>\n";
    }

    // Stirrup tick marks: vertical lines spanning the full stirrup
    // depth (between the two longitudinal bar lines' outer extents),
    // at the positions stirrupPositionsMm computes (uniform, or
    // graduated near supports if supportStirrupSpacingMm is set).
    auto positions = stirrupPositionsMm(p);
    double stirrupTop = margin + p.coverMm;
    double stirrupBottom = margin + p.depthMm - p.coverMm;
    for (double posMm : positions) {
        double x = margin + posMm;
        svg << "  <line x1=\"" << x << "\" y1=\"" << stirrupTop << "\" x2=\"" << x << "\" y2=\""
            << stirrupBottom << "\" stroke=\"black\" stroke-width=\"1\"/>\n";
    }

    if (!p.label.empty()) {
        svg << "  <text x=\"" << margin << "\" y=\"" << (margin - 5.0) << "\" font-size=\"14\">"
            << p.label << "</text>\n";
    }

    svg << "</svg>\n";
    return svg.str();
}

}  // namespace nrsa::drawing
