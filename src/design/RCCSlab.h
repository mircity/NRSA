#pragma once

#include <string>

namespace nrsa::design {

// ---------------------------------------------------------------------------
// RCCSlab -- one-way and two-way (edge-supported) reinforced-concrete slab
// design per ACI 318-19 Chapter 7/8 / BNBC 2020 Part 6 Chapter 6.
//
// ONE-WAY SLABS: flexural design for a 1-meter-wide design strip, governed
// either by the moment demand or by ACI 318-19 Section 7.6.1.1's slab
// minimum -- the shrinkage-and-temperature reinforcement of Section
// 24.4.3.2, NOT the beam minimum (1.4/fy, sqrt(fc')/4fy) RCCBeam::
// designFlexure uses. A beam's minimum exists to guard against a lightly
// reinforced section failing suddenly right at first cracking; a one-way
// slab strip, backed by two-way redistribution and a much shallower,
// less brittle section, does not need that same margin, so the code
// gives it its own (generally smaller) ACI-specified minimum instead of
// reusing RCCBeam's.
//
// TWO-WAY SLABS (edge-supported, matching this engine's "beamSupported"
// slab idealization -- see prototype/README.md): moment demand is found
// via the RANKINE (elastic load-distribution) METHOD -- the total
// factored load is split between the short and long spans in proportion
// to L_long^4 / (L_short^4 + L_long^4) (the classic deflection-
// compatibility result for two intersecting simply-supported strips of
// unit width), then each direction's simply-supported moment w*L^2/8 is
// scaled by an ACI 318-19 Table 6.5.2-style continuous-span coefficient
// (keyed by how many of that direction's two edges are continuous: 0/1/2)
// for positive and negative moment separately. This is a real, commonly
// taught approximate method for two-way slabs on non-yielding (beam)
// supports -- it is explicitly NOT the ACI 318-19 Direct Design Method
// (which applies to column-supported flat plates/flat slabs, a different
// structural system from this beam-supported case) and NOT the exact
// tabulated elastic-plate-theory coefficients BNBC 2020 / the older
// ACI 318-63 "Method 3" publish for the 9 standard edge-condition cases;
// those tables are not reproduced here. designTwoWaySlabPanel() states
// this simplification in its result rather than presenting the answer as
// code-table-exact.
// ---------------------------------------------------------------------------

// ACI 318-19 Section 24.4.3.2 shrinkage-and-temperature steel ratio
// (relative to the GROSS concrete area, h*b) -- also serves as the
// flexural minimum for a one-way slab per Section 7.6.1.1. Depends only
// on fy (deformed Grade 40/50 vs Grade 60-and-up bands); for fy above
// 420 MPa the ratio scales down as 0.0018*420/fy, floored at 0.0014, per
// the code's own footnote for that case.
double shrinkageTemperatureSteelRatio(double fyMPa);

struct OneWaySlabDesignResult {
    double asRequiredMm2PerM = 0.0;  // required tension steel, per meter width
    double asMinMm2PerM = 0.0;       // ACI 318-19 24.4.3.2 / 7.6.1.1 minimum, per meter width
    double rhoProvided = 0.0;        // asRequiredMm2PerM / (1000*d), before rounding to real bars
    bool governedByMinimum = false;
    bool exceedsMaximum = false;     // demand exceeds the tension-controlled limit for this thickness
    double maxBarSpacingMm = 0.0;    // ACI 318-19 Section 24.3.2: lesser of 3h or 450mm
    std::string note;
};

// One-way slab flexural design for a 1-meter-wide design strip. hMm is the
// overall slab thickness, coverMm the clear cover to the main bars, and
// barDiaMm the main-bar diameter being checked (needed both for
// d = h - cover - dia/2 and because the minimum is expressed per this
// same strip).
OneWaySlabDesignResult designOneWaySlab(double muKNmPerM, double hMm, double coverMm,
                                         double barDiaMm, double fcMPa, double fyMPa);

// One-way shear adequacy check for a slab strip (ACI 318-19 22.5.5.1,
// same Vc formula RCCBeam::designShear uses). Slabs are essentially never
// stirrup-reinforced in ordinary practice, so this only reports whether
// phi*Vc alone is enough; if not, the practical fixes are a thicker slab
// or shear reinforcement/studs, neither modeled here.
struct OneWayShearCheckResult {
    double vcKN = 0.0;
    double phiVcKN = 0.0;
    bool adequate = false;
    std::string note;
};
OneWayShearCheckResult checkOneWayShear(double vuKNPerM, double hMm, double coverMm,
                                         double barDiaMm, double fcMPa);

// Continuity condition of a span's two supporting edges, used to select
// the ACI 318-19 Table 6.5.2-style continuous-span moment coefficients
// consumed by designTwoWaySlabPanel(). BothSimple is plain statics
// (wl^2/8, zero negative moment); OneContinuous and BothContinuous use
// the code table's "two-span" and "interior span, >2 spans" entries
// respectively (a documented, conservative simplification -- see the
// header's own note on this not being span-count- or span-length-exact).
enum class EdgeContinuity {
    BothSimple = 0,     // both ends simply supported
    OneContinuous = 1,  // one end continuous, other simple (treated as a code-table "end span")
    BothContinuous = 2  // both ends continuous (treated as a code-table "interior span")
};

struct TwoWaySlabDesignResult {
    double shortSpanLoadKNPerM2 = 0.0;   // wa: share of the total load carried by the short direction
    double longSpanLoadKNPerM2 = 0.0;    // wb: share carried by the long direction
    double shortSpanPosMomentKNm = 0.0;  // per meter width, short direction, midspan
    double shortSpanNegMomentKNm = 0.0;  // per meter width, short direction, continuous support(s)
    double longSpanPosMomentKNm = 0.0;
    double longSpanNegMomentKNm = 0.0;
    OneWaySlabDesignResult shortSpanPosSteel;
    OneWaySlabDesignResult shortSpanNegSteel;
    OneWaySlabDesignResult longSpanPosSteel;
    OneWaySlabDesignResult longSpanNegSteel;
    std::string note;  // always notes the Rankine-method simplification -- see header comment
};

// Two-way (edge-supported) slab panel design. shortSpanM/longSpanM are
// clear spans; shortSpanM must be <= longSpanM (this function does not
// swap mismatched inputs for you, to keep "short"/"long" unambiguous in
// the result -- swap your own inputs if needed). wuKNPerM2 is the total
// factored uniform load (self-weight + superimposed + live, already
// factored) over the whole panel.
TwoWaySlabDesignResult designTwoWaySlabPanel(double shortSpanM, double longSpanM, double wuKNPerM2,
                                              double hMm, double coverMm, double barDiaMm,
                                              EdgeContinuity shortDirContinuity,
                                              EdgeContinuity longDirContinuity,
                                              double fcMPa, double fyMPa);

}  // namespace nrsa::design
