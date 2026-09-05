#include <sstream>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "drawing/BeamDetailDrawing.h"

using namespace nrsa::drawing;

static bool approxEqual(double a, double b, double tol = 1e-6) { return std::abs(a - b) <= tol; }

// Hand-calc: length=6000mm, cover=40mm, spacing=150mm.
// zoneLength = 6000-80=5920, count = floor(5920/150)+1 = floor(39.47)+1 = 39+1 = 40
static void testStirrupCountHandCalc() {
    int count = stirrupCountForLength(6000.0, 40.0, 150.0);
    int expected = static_cast<int>(std::floor((6000.0 - 80.0) / 150.0)) + 1;
    assert(expected == 40);
    assert(count == expected);
    std::cout << "  testStirrupCountHandCalc (count=" << count << ") OK\n";
}

// The actual generated SVG must contain exactly as many stirrup <line>
// elements as stirrupCountForLength predicts, PLUS the 2 longitudinal
// bar lines -- an independent count via parsing the real output text,
// not just trusting the internal loop matches its own helper function.
static void testSvgLineCountMatchesStirrupCount() {
    BeamDetailParams p;
    p.lengthMm = 6000.0;
    p.widthMm = 300.0;
    p.depthMm = 500.0;
    p.coverMm = 40.0;
    p.topBarCount = 2;
    p.topBarDiaMm = 16.0;
    p.bottomBarCount = 3;
    p.bottomBarDiaMm = 20.0;
    p.stirrupDiaMm = 10.0;
    p.stirrupSpacingMm = 150.0;
    p.label = "B1";

    std::string svg = generateBeamDetailSvg(p);

    int lineCount = 0;
    std::size_t pos = 0;
    while ((pos = svg.find("<line", pos)) != std::string::npos) { lineCount++; pos += 5; }

    int expectedStirrups = stirrupCountForLength(p.lengthMm, p.coverMm, p.stirrupSpacingMm);
    assert(lineCount == expectedStirrups + 2);  // +2 for top/bottom longitudinal bars

    assert(svg.find("<svg") == 0);
    assert(svg.find("</svg>") != std::string::npos);
    assert(svg.find("B1") != std::string::npos);
    assert(svg.find("<rect") != std::string::npos);

    std::cout << "  testSvgLineCountMatchesStirrupCount (lines=" << lineCount
              << ", expected stirrups+2=" << (expectedStirrups + 2) << ") OK\n";
}

// Bar vertical positions must respect cover+stirrup+half-bar-diameter
// from each face -- verified by extracting the y1 attribute of the
// first two <line> elements (top and bottom bars) and checking against
// the hand-derived offset formula.
static void testBarPositionHandCalc() {
    BeamDetailParams p;
    p.lengthMm = 4000.0;
    p.widthMm = 300.0;
    p.depthMm = 500.0;
    p.coverMm = 40.0;
    p.topBarDiaMm = 20.0;
    p.bottomBarDiaMm = 20.0;
    p.stirrupDiaMm = 10.0;
    p.stirrupSpacingMm = 200.0;

    std::string svg = generateBeamDetailSvg(p);
    double margin = 20.0;
    double expectedTopY = margin + p.coverMm + p.stirrupDiaMm + p.topBarDiaMm / 2.0;   // 20+40+10+10=80
    double expectedBottomY = margin + p.depthMm - (p.coverMm + p.stirrupDiaMm + p.bottomBarDiaMm / 2.0);

    std::ostringstream expectedTopStr, expectedBottomStr;
    expectedTopStr << "y1=\"" << expectedTopY << "\"";
    expectedBottomStr << "y1=\"" << expectedBottomY << "\"";
    assert(svg.find(expectedTopStr.str()) != std::string::npos);
    assert(svg.find(expectedBottomStr.str()) != std::string::npos);
    std::cout << "  testBarPositionHandCalc (topY=" << expectedTopY << ", bottomY=" << expectedBottomY
              << ") OK\n";
}

static void testRejectsBarsThatDontFit() {
    BeamDetailParams p;
    p.lengthMm = 4000.0;
    p.widthMm = 300.0;
    p.depthMm = 100.0;  // too shallow for the cover+stirrup+bar stack below
    p.coverMm = 40.0;
    p.topBarDiaMm = 25.0;
    p.bottomBarDiaMm = 25.0;
    p.stirrupDiaMm = 10.0;
    p.stirrupSpacingMm = 150.0;
    bool threw = false;
    try { generateBeamDetailSvg(p); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  testRejectsBarsThatDontFit OK\n";
}

static void testRejectsInvalidDimensions() {
    BeamDetailParams p;
    p.lengthMm = -100.0;
    p.widthMm = 300.0;
    p.depthMm = 500.0;
    bool threw = false;
    try { generateBeamDetailSvg(p); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  testRejectsInvalidDimensions OK\n";
}

// Hand-calc: graduated stirrup spacing, length=6000, cover=40,
// supportZoneLength=1000, supportSpacing=100, midSpacing=200.
//   leftmost position must be exactly at cover=40.
//   rightmost position must be exactly at length-cover=5960.
//   spacing between the first two positions (both within the support
//     zone) must be exactly supportSpacing=100.
//   spacing between the last two positions (both within the support
//     zone) must be exactly supportSpacing=100.
//   somewhere in the middle, spacing must widen to exactly midSpacing=200.
static void testGraduatedStirrupSpacingHandCalc() {
    BeamDetailParams p;
    p.lengthMm = 6000.0;
    p.widthMm = 300.0;
    p.depthMm = 500.0;
    p.coverMm = 40.0;
    p.stirrupSpacingMm = 200.0;             // midspan spacing
    p.supportStirrupSpacingMm = 100.0;      // support-zone spacing
    p.supportZoneLengthMm = 1000.0;

    auto positions = stirrupPositionsMm(p);
    assert(!positions.empty());
    assert(approxEqual(positions.front(), 40.0));
    assert(approxEqual(positions.back(), 5960.0));
    assert(approxEqual(positions[1] - positions[0], 100.0));
    assert(approxEqual(positions.back() - positions[positions.size() - 2], 100.0));

    // Every consecutive gap must never EXCEED the midspan spacing (the
    // wider of the two designed spacings -- the actual code-driven
    // upper limit on stirrup spacing) -- most gaps are exactly
    // supportSpacing or midSpacing, but the single transition point
    // where the middle zone meets a support zone can land on a
    // "closing" remainder gap when the zone lengths don't divide
    // evenly (exactly the same situation a real shop drawing handles
    // by tightening one spacing to fit -- always TIGHTER than the
    // wider spacing, never looser, so never unsafe). This is a real,
    // documented property of stirrupPositionsMm, not a test workaround.
    bool foundMidspanGap = false;
    for (std::size_t i = 1; i < positions.size(); ++i) {
        double gap = positions[i] - positions[i - 1];
        assert(gap <= 200.0 + 1e-6);  // never exceeds the wider (midspan) spacing
        if (approxEqual(gap, 200.0)) foundMidspanGap = true;
    }
    assert(foundMidspanGap);

    // Monotonically increasing, no duplicates.
    for (std::size_t i = 1; i < positions.size(); ++i) assert(positions[i] > positions[i - 1]);

    std::cout << "  testGraduatedStirrupSpacingHandCalc (count=" << positions.size() << ") OK\n";
}

// When supportStirrupSpacingMm is left at 0 (default), stirrupPositionsMm
// must reduce to EXACTLY the old uniform behavior (same count and same
// first position as stirrupCountForLength/the old hard-coded formula).
static void testGraduatedSpacingDisabledMatchesUniformBehavior() {
    BeamDetailParams p;
    p.lengthMm = 6000.0;
    p.widthMm = 300.0;
    p.depthMm = 500.0;
    p.coverMm = 40.0;
    p.stirrupSpacingMm = 150.0;
    // supportStirrupSpacingMm left at default 0.0 -> uniform mode.

    auto positions = stirrupPositionsMm(p);
    int expectedCount = stirrupCountForLength(p.lengthMm, p.coverMm, p.stirrupSpacingMm);
    assert(static_cast<int>(positions.size()) == expectedCount);
    assert(approxEqual(positions.front(), p.coverMm));
    assert(approxEqual(positions[1] - positions[0], p.stirrupSpacingMm));
    std::cout << "  testGraduatedSpacingDisabledMatchesUniformBehavior OK\n";
}

// Bar curtailment: a dashed line for curtailed bars must appear in the
// SVG spanning exactly [curtailmentDistanceFromEndMm,
// length-curtailmentDistanceFromEndMm], verified by parsing the actual
// generated x1/x2 attributes.
static void testBarCurtailmentHandCalc() {
    BeamDetailParams p;
    p.lengthMm = 6000.0;
    p.widthMm = 300.0;
    p.depthMm = 500.0;
    p.coverMm = 40.0;
    p.bottomBarCount = 4;
    p.curtailedBottomBarCount = 2;
    p.curtailmentDistanceFromEndMm = 1200.0;

    std::string svg = generateBeamDetailSvg(p);
    assert(svg.find("stroke-dasharray") != std::string::npos);

    double margin = 20.0;
    double expectedX1 = margin + p.curtailmentDistanceFromEndMm;
    double expectedX2 = margin + p.lengthMm - p.curtailmentDistanceFromEndMm;
    std::ostringstream x1Str, x2Str;
    x1Str << "x1=\"" << expectedX1 << "\"";
    x2Str << "x2=\"" << expectedX2 << "\"";
    assert(svg.find(x1Str.str()) != std::string::npos);
    assert(svg.find(x2Str.str()) != std::string::npos);
    std::cout << "  testBarCurtailmentHandCalc OK\n";
}

// curtailedBottomBarCount=0 (default) must produce NO dashed line --
// old behavior unchanged when the feature isn't used.
static void testNoCurtailmentByDefault() {
    BeamDetailParams p;
    p.lengthMm = 4000.0;
    p.widthMm = 300.0;
    p.depthMm = 500.0;
    std::string svg = generateBeamDetailSvg(p);
    assert(svg.find("stroke-dasharray") == std::string::npos);
    std::cout << "  testNoCurtailmentByDefault OK\n";
}

static void testRejectsCurtailmentExceedingBarCount() {
    BeamDetailParams p;
    p.lengthMm = 4000.0;
    p.widthMm = 300.0;
    p.depthMm = 500.0;
    p.bottomBarCount = 2;
    p.curtailedBottomBarCount = 5;  // more than exist
    bool threw = false;
    try { generateBeamDetailSvg(p); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  testRejectsCurtailmentExceedingBarCount OK\n";
}

int main() {
    std::cout << "test_beam_detail_drawing:\n";
    testStirrupCountHandCalc();
    testSvgLineCountMatchesStirrupCount();
    testBarPositionHandCalc();
    testRejectsBarsThatDontFit();
    testRejectsInvalidDimensions();
    testGraduatedStirrupSpacingHandCalc();
    testGraduatedSpacingDisabledMatchesUniformBehavior();
    testBarCurtailmentHandCalc();
    testNoCurtailmentByDefault();
    testRejectsCurtailmentExceedingBarCount();
    std::cout << "All tests passed.\n";
    return 0;
}
