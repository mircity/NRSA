#pragma once

#include <map>
#include <string>
#include <vector>

#include "design/RCCBeam.h"
#include "design/RCCColumn.h"

namespace nrsa::bbs {

// ---------------------------------------------------------------------------
// bbs -- Bar Bending Schedule generation, the first `src/bbs/`/`src/bim/`/
// `src/boq/`/`src/reports/`/`src/api/`/`src/validation/` "surrounding
// tooling" module to be built, per the README Roadmap item 13 build order
// (bbs -> boq -> reports -> validation -> modeler/bim -> api, since bbs is
// the only one of these that needs nothing beyond design output that
// already exists for every RCC member type).
//
// This module consumes the reinforcement AREAS/ratios already computed by
// design::RCCBeam / design::RCCColumn / design::RCCSlab / design::
// RCCShearWall / design::RCCFoundation and turns them into an actual
// buildable bar list: real bar diameters and counts (an area alone is not
// buildable -- someone has to round it to N bars of a real diameter),
// individual bar CUT lengths (accounting for standard hooks/bends, not
// just the member's clear dimension), and a rolled-up schedule of total
// length and weight per bar mark and per diameter -- exactly what a
// fabrication shop drawing needs and what `boq/` (the next module in the
// roadmap) will need to price rebar by weight.
//
// SCOPE and DELIBERATE SIMPLIFICATIONS (flagged here up front, the same
// "say what's approximate, don't hide it" pattern every design/ module in
// this project already follows):
//
//   - Hook-extension lengths (standardHookExtensionMm) are ACI 318-19
//     Section 25.3's stirrup/tie hook formula (Table 25.3.2: 6db, not less
//     than 75mm, for both 90 deg and 135 deg hooks) applied UNIFORMLY to both
//     primary-bar hooks and stirrup/tie hooks, rather than also
//     implementing Table 25.3.1's separate (bar-size-banded, angle-
//     dependent bend-diameter) primary-hook provisions. This under-states
//     the true minimum extension for large-diameter 90 deg primary-bar hooks
//     in the exact code table; adequate for a first-pass shop schedule,
//     not a substitute for the full Table 25.3.1 lookup when hook capacity
//     genuinely governs.
//   - Bend-length deductions (bendDeductionMm) use the widely-taught
//     generic rebar-detailing convention (deduction = angle/45 * 1*db --
//     i.e. 1db per 45 deg of bend: 2db at 90 deg, 3db at 135 deg, 4db at
//     180 deg) to convert an "outer dimension" bar layout into a shorter
//     actual cut length. This convention is standard industry practice
//     for shop-drawing takeoff, not itself an ACI 318 provision -- ACI 318
//     specifies hook/bend GEOMETRY (extension, bend diameter) but does not
//     mandate a bend-deduction formula for cutting-length takeoff.
//   - Lap splice length (simplifiedTensionLapSpliceLengthMm) is a
//     preliminary, code-REFERENCED but not code-EXACT estimate -- it does
//     not implement ACI 318-19 Section 25.4.2.3's full cover/spacing/
//     confinement-dependent development-length equation (that equation
//     needs a confinement/transverse-reinforcement index (Ktr) and clear
//     cover/spacing this module's callers don't necessarily have on hand
//     for a first-pass schedule). Flagged via this function's own doc
//     comment; a real quantity-surveying/permit submission needs the full
//     Section 25.4.2.3 calculation, not implemented here.
//   - Bar SHAPES covered: straight bars (with 0/1/2 end hooks) and closed
//     rectangular stirrups/ties (2-leg, uniform diameter, matching what
//     RCCBeam::designShear/RCCColumn tie detailing and RCCShearWall
//     horizontal reinforcement all actually need). L-bars, cranked
//     (bent-up) bars, circular/spiral ties, and multi-leg (>2) stirrups
//     are NOT modeled -- a real, distinct addition each.
//   - Steel unit weight uses the standard density-derived formula (not the
//     rougher d^2/162 rule of thumb some shop drawings use, though the two
//     agree to within ~0.1%, verified in this module's own tests).
// ---------------------------------------------------------------------------

constexpr double kSteelDensityKgPerM3 = 7850.0;

// Cross-sectional area of a single round bar, mm^2.
double barAreaMm2(double barDiaMm);

// Mass per unit length of a single round bar, kg/m, from first principles
// (area * steel density) -- NOT the d^2/162 rule of thumb, though the two
// agree to within about 0.1% (verified in tests).
double unitWeightKgPerM(double barDiaMm);

// Rounds a required steel area up to a whole number of bars of the given
// diameter, floored at minBars (2 by default -- a single bar is never a
// real reinforcement layout for a beam/column/footing strip). Throws if
// asRequiredMm2 is negative or barDiaMm is not positive.
int practicalBarCount(double asRequiredMm2, double barDiaMm, int minBars = 2);

// ACI 318-19 Section 25.3 hook angles this module supports.
enum class HookAngle {
    None = 0,
    Bend90 = 90,
    Bend135 = 135,
    Bend180 = 180
};

// Straight extension length beyond the bend point for a standard hook, per
// ACI 318-19 Table 25.3.2 (6*db, not less than 75mm, for both 90 deg and
// 135 deg) -- applied uniformly to primary and stirrup/tie hooks alike; see
// the header note above on why this is a documented simplification for
// large-diameter 90 deg primary hooks. A 180 deg hook's extension is 4*db,
// not less than 65mm, per the same table's 180 deg row. HookAngle::None
// returns 0.
double standardHookExtensionMm(double barDiaMm, HookAngle angle);

// Generic rebar-detailing bend-length deduction convention: 1*db per 45
// degrees of bend (2db @ 90 deg, 3db @ 135 deg, 4db @ 180 deg) -- see the
// header note above on this being industry shop-drawing practice, not an
// ACI 318 provision itself. HookAngle::None returns 0.
double bendDeductionMm(double barDiaMm, HookAngle angle);

// Preliminary tension lap-splice length estimate -- SEE THE HEADER NOTE
// ABOVE: this is a simplified, code-referenced-but-not-exact stand-in for
// ACI 318-19 Section 25.4.2.3's full development-length equation. Returns
// max(300mm, (fyMPa / (1.7 * sqrt(fcMPa))) * barDiaMm) -- a commonly used
// preliminary working estimate, not a substitute for the full code
// calculation once confinement/cover/spacing data is available.
double simplifiedTensionLapSpliceLengthMm(double barDiaMm, double fyMPa, double fcMPa);

enum class BarShape {
    Straight,      // 0, 1, or 2 end hooks
    RectStirrup    // closed 2-leg rectangular stirrup/tie, 135 deg seismic hooks
};

// One bar mark: a group of identical bars (same shape, diameter, and cut
// length) that a shop drawing would list as a single row. count/cutLengthMm
// describe ONE bar; totalLengthM()/totalWeightKg() are the row's rolled-up
// quantities.
struct BarMarkEntry {
    std::string markId;       // caller-assigned label, e.g. "B1-T1", "C4-L"
    std::string memberLabel;  // caller-assigned member description, e.g. "Beam B1, Story 2"
    BarShape shape = BarShape::Straight;
    double barDiaMm = 0.0;
    int count = 0;
    double cutLengthMm = 0.0;  // length of ONE bar, already net of hook/bend adjustments

    double totalLengthM() const;
    double totalWeightKg() const;
};

// Cut length of a straight bar spanning a clear member length, with an
// independently-chosen hook (or no hook) at each end. Equivalent formula:
//   cutLength = clearLengthMm
//             + startHookExtension - startBendDeduction   (0 if HookAngle::None)
//             + endHookExtension   - endBendDeduction      (0 if HookAngle::None)
// Throws if clearLengthMm or barDiaMm is not positive.
double straightBarCutLengthMm(double clearLengthMm, double barDiaMm,
                               HookAngle startHook, HookAngle endHook);

// Cut length of a closed 2-leg rectangular stirrup/tie. outerWidthMm/
// outerDepthMm are the stirrup's own OUTER dimensions (i.e. already
// b-2*cover / d-2*cover of the member it wraps -- this function does not
// itself subtract cover). Method: perimeter of the outer rectangle, plus
// two 135 deg hook extensions (the seismic-detailing hook this project's
// design/ modules assume for ties/stirrups), minus bend-length deductions
// at the four 90 deg corners and the two 135 deg hook bends. Throws if
// either dimension or barDiaMm is not positive.
double stirrupCutLengthMm(double outerWidthMm, double outerDepthMm, double barDiaMm);

// Builds one BarMarkEntry for a beam's (or one-way slab strip's, treated
// as a 1m-wide "beam") main flexural reinforcement: bar count comes from
// rounding flex.asRequiredMm2 up to whole bars of barDiaMm
// (practicalBarCount), cut length from straightBarCutLengthMm.
BarMarkEntry makeFlexuralBarEntry(const std::string& markId, const std::string& memberLabel,
                                   const design::FlexuralDesignResult& flex, double barDiaMm,
                                   double clearLengthMm, HookAngle startHook, HookAngle endHook);

// Builds one BarMarkEntry for a beam's/wall's transverse stirrup or tie
// reinforcement: bar count from spacing shear.requiredSpacingMm over the
// member's clear length (one extra bar to close both ends), cut length
// from stirrupCutLengthMm. legs > 2 is rejected (see header SCOPE note --
// multi-leg stirrups are not modeled); the shear demand's OWN required
// leg count must already be 2 (i.e. the ShearDesignResult passed in must
// have been designed for a 2-leg stirrup) for the returned bar count to
// mean what it says.
BarMarkEntry makeStirrupEntry(const std::string& markId, const std::string& memberLabel,
                               const design::ShearDesignResult& shear, double memberClearLengthMm,
                               double outerWidthMm, double outerDepthMm, double barDiaMm, int legs);

// Builds one BarMarkEntry PER DISTINCT BAR DIAMETER present in a column's
// RectColumnLayout (in practice, one entry, since generateRectangularLayout
// only ever produces a single diameter -- but this stays correct even if a
// future mixed-diameter layout constructor is added). All bars run the
// full clear story height plus (if includeLapSplice) one lap splice length
// on top, per simplifiedTensionLapSpliceLengthMm -- see that function's
// own note on this being a preliminary estimate.
std::vector<BarMarkEntry> makeColumnLongitudinalEntries(const std::string& markPrefix,
                                                         const std::string& memberLabel,
                                                         const design::RectColumnLayout& layout,
                                                         double clearStoryHeightMm,
                                                         bool includeLapSplice, double fyMPa,
                                                         double fcMPa);

// A full bar bending schedule: an ordered list of bar marks plus rolled-up
// totals. Entries are kept in the order added (the order a shop drawing
// would typically list them, member by member) -- weightByDiameterKg()
// re-sorts only for the summary table.
class BarBendingSchedule {
public:
    void addEntry(BarMarkEntry entry);

    const std::vector<BarMarkEntry>& entries() const { return entries_; }
    double totalWeightKg() const;

    // Diameter (mm) -> total weight (kg) across every entry of that
    // diameter, for a schedule's usual "summary by bar size" table.
    std::map<double, double> weightByDiameterKg() const;

private:
    std::vector<BarMarkEntry> entries_;
};

}  // namespace nrsa::bbs
