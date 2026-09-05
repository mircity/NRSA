#pragma once

#include <string>
#include <vector>

namespace nrsa::drawing {

// Roadmap Section 12 ("Automatic Reinforcement Detailing"): "BBS
// module already exists -- the next step is graphical detailing."
// This is that next step, for a SINGLE beam's longitudinal elevation:
// an SVG showing the member outline, top and bottom longitudinal bars
// running its length, and stirrups drawn as vertical tick marks at
// the actual computed spacing.
//
// SCOPE, STATED PLAINLY: uniform reinforcement only -- one top bar
// group, one bottom bar group, and ONE stirrup spacing applied for the
// member's full length. Real detailing curtails bars partway along
// the span (fewer bars at midspan/near supports where moment demand
// drops) and typically uses closer stirrup spacing near supports than
// at midspan (ACI 318-19 Section 9.7.6.2.2's own graduated spacing
// zones) -- drawing THAT needs per-zone bar/stirrup data this function
// does not take, since design::designFlexure/designShear (as currently
// scoped) also produce one uniform answer per call, not a per-zone
// envelope. This draws exactly what those functions currently give you
// -- a real, useful single-zone shop-reference view, not a complete
// multi-zone placement drawing.
//
// No title block, dimension lines, or bar-mark callout labels beyond
// the plain text this function itself adds -- composing this into a
// full sheet is Drawing Generation Engine Phase 7's job (see
// ElevationView.h's own scope note), not duplicated here.
// SCOPE, STATED PLAINLY (updated 2026-08-30): the original version of
// this function drew uniform reinforcement/stirrup spacing only. Two
// real detailing features are now supported as OPT-IN extensions
// (default values reproduce the exact old uniform-only behavior, so
// nothing that worked before changes):
//   - GRADUATED STIRRUP SPACING: ACI 318-19 Section 9.7.6.2.2's own
//     real pattern -- closer stirrup spacing within a zone near each
//     support (where shear demand is highest), wider spacing in the
//     middle. Set supportStirrupSpacingMm and supportZoneLengthMm to
//     use this; leave supportStirrupSpacingMm at its 0 default for the
//     old uniform-everywhere behavior.
//   - BAR CURTAILMENT: some bottom bars stopping short of the full
//     span length rather than running end-to-end (fewer bars needed
//     away from the highest-moment region). Set curtailedBottomBarCount
//     and curtailmentDistanceFromEndMm to draw this; leave
//     curtailedBottomBarCount at its 0 default for the old
//     full-length-only behavior.
// STILL NOT DONE: the curtailment cutoff point and support-zone length
// are CALLER-SUPPLIED, not derived from an actual moment/shear
// envelope along the span (design::designFlexure/designShear, as
// currently scoped, give one Mu/Vu per call, not a diagram) -- a
// caller wanting code-derived cutoff points needs to compute them
// externally (e.g. from a moment diagram) and pass the resulting
// distances in. This draws a real graduated/curtailed layout correctly
// GIVEN those distances; it does not derive the distances itself.
struct BeamDetailParams {
    double lengthMm = 0.0;
    double widthMm = 0.0;
    double depthMm = 0.0;
    double coverMm = 40.0;

    int topBarCount = 2;
    double topBarDiaMm = 16.0;
    int bottomBarCount = 2;
    double bottomBarDiaMm = 16.0;

    double stirrupDiaMm = 10.0;
    double stirrupSpacingMm = 150.0;

    // Graduated stirrup spacing (opt-in -- see class doc comment).
    double supportStirrupSpacingMm = 0.0;  // 0 = disabled, use stirrupSpacingMm everywhere
    double supportZoneLengthMm = 0.0;      // length from EACH end using supportStirrupSpacingMm

    // Bar curtailment (opt-in -- see class doc comment).
    int curtailedBottomBarCount = 0;             // how many of bottomBarCount stop short of full length
    double curtailmentDistanceFromEndMm = 0.0;   // curtailed bars span [this, length-this]

    std::string label;  // e.g. "B1" -- drawn as a text label
};

// Generates a single <svg>...</svg> string: a side-elevation rectangle
// (lengthMm x depthMm), a top bar line and a bottom bar line each
// drawn at coverMm+stirrupDiaMm+barDia/2 from the respective face
// (the standard cover-to-bar-centerline offset), and vertical stirrup
// tick marks every stirrupSpacingMm along the length starting at
// coverMm from each end. Throws std::invalid_argument if
// stirrupSpacingMm <= 0, lengthMm <= 0, widthMm <= 0, or depthMm <= 0,
// or if 2*(coverMm+stirrupDiaMm+max(topBarDiaMm,bottomBarDiaMm)/2)
// exceeds depthMm (the bars wouldn't physically fit within the
// section).
std::string generateBeamDetailSvg(const BeamDetailParams& params);

// The number of stirrup tick marks generateBeamDetailSvg will draw in
// the OLD uniform-spacing mode -- exposed separately so a caller (or a
// test) can check the count without parsing the SVG text:
// floor((lengthMm - 2*coverMm) / stirrupSpacingMm) + 1 (one at each end
// of the stirrup zone, plus one per additional full spacing between
// them).
int stirrupCountForLength(double lengthMm, double coverMm, double stirrupSpacingMm);

// The actual x-positions (mm, measured from the member's start) of
// every stirrup generateBeamDetailSvg will draw, honoring graduated
// spacing when supportStirrupSpacingMm > 0 (see BeamDetailParams'
// class doc comment): supportStirrupSpacingMm within
// supportZoneLengthMm of EACH end, stirrupSpacingMm in the middle zone
// between them. KNOWN PROPERTY: when the middle zone's length isn't an
// exact multiple of stirrupSpacingMm, the single gap where the middle
// zone meets a support zone is shortened to fit (never lengthened) --
// the same thing a real shop drawing does by hand, and always the safe
// direction (a shorter gap is more conservative, never less). Every
// gap is therefore guaranteed <= stirrupSpacingMm (the wider of the
// two spacings), even though not every individual gap need equal
// exactly supportStirrupSpacingMm or stirrupSpacingMm. Exposed
// separately so a caller/test can check exact positions, not just a
// count. When supportStirrupSpacingMm is 0 (the default), this reduces
// to the same uniform spacing stirrupCountForLength describes.
std::vector<double> stirrupPositionsMm(const BeamDetailParams& params);

}  // namespace nrsa::drawing
