#pragma once

#include <map>
#include <string>
#include <vector>

#include "bbs/BarSchedule.h"

namespace nrsa::boq {

// ---------------------------------------------------------------------------
// boq -- Bill of Quantities, the second "surrounding tooling" module in the
// README Roadmap item 13 build order (bbs -> boq -> reports -> validation
// -> modeler/bim -> api). Rolls concrete volume, formwork area, and rebar
// weight (the last one straight from `bbs::BarBendingSchedule`, per the
// Roadmap's own description of this module) into a cost/quantity takeoff.
//
// SCOPE and DELIBERATE SIMPLIFICATIONS (the same up-front honesty pattern
// every module in this project uses):
//   - Only RECTANGULAR members are covered (beam, column, one-way/two-way
//     slab panel, straight wall segment, and rectangular isolated footing)
//     -- matching the rectangular-section scope core::Section::rectangular()
//     and every design/ module built so far already carry. Non-rectangular
//     shapes (circular columns, the hollow-tube core section, combined/
//     pile-cap footings' own non-square plans) are not covered -- a real,
//     distinct addition each.
//   - Formwork-area formulas assume the ORDINARY, unobstructed casting
//     condition for each member type, explicitly noted at each function:
//     a beam forms its soffit + two sides only (top left open for a
//     monolithic slab pour or an open finish); a column forms all four
//     faces; a slab forms its soffit only (no separate edge/stop-end
//     formwork -- correct for an interior panel sharing edges with beams,
//     NOT for a free/cantilevered slab edge); a wall forms BOTH faces
//     (not the single-face condition of, say, a basement wall cast
//     against an earlier-poured retaining wall or lagging); a footing
//     forms its four side faces only (bottom poured against a blinding/
//     soil bed, top left open). Any of these real site-specific
//     conditions that differ from the assumed one needs its own
//     computation, not provided here.
//   - No unit COSTS/rates are modeled -- this module produces QUANTITIES
//     (volume, area, weight) only; attaching a rate schedule and rolling
//     up to a priced total is a distinct, project/market-specific
//     addition deliberately left to the caller.
//   - Concrete is grouped by its fc' (MPa) as the BOQ's own "grade" key,
//     the same grouping convention design/'s own functions already take
//     fc' as an input for -- there is no separate named "grade" concept
//     (e.g. "C25", "C30") introduced here.
// ---------------------------------------------------------------------------

// Volume of a rectangular prismatic member (beam, column, or straight
// wall/footing segment treated as one block): b x h x length, all
// meters, returned in m^3. Throws if any dimension is not positive.
double rectangularVolumeM3(double bM, double hM, double lengthM);

// Beam formwork area: soffit (bottom) + two vertical sides, length
// running the member's length -- top is assumed open (see header SCOPE
// note). Returns m^2. Throws if any dimension is not positive.
double rectangularBeamFormworkAreaM2(double bM, double hM, double lengthM);

// Column formwork area: all four vertical faces over the clear height.
// Returns m^2. Throws if any dimension is not positive.
double rectangularColumnFormworkAreaM2(double bM, double hM, double heightM);

// Slab formwork area: soffit only (see header SCOPE note on edge
// formwork not being included). Returns m^2. Throws if either dimension
// is not positive.
double slabFormworkAreaM2(double bM, double lM);

// Wall formwork area: both faces over the wall's length x height.
// Returns m^2. Throws if either dimension is not positive.
double wallFormworkAreaM2(double lengthM, double heightM);

// Rectangular footing formwork area: the four side (edge) faces only,
// i.e. perimeter x thickness -- bottom and top are not formed (see
// header SCOPE note). Returns m^2. Throws if any dimension is not
// positive.
double footingFormworkAreaM2(double planBM, double planLM, double thicknessM);

// One member's concrete quantity: volume plus the formwork area needed
// to cast it, grouped for BOQ purposes by its concrete grade (fc', MPa).
struct ConcreteQuantity {
    std::string memberLabel;
    double fcMPa = 0.0;
    double volumeM3 = 0.0;
    double formworkAreaM2 = 0.0;
};

// Builders for the five member types this module covers -- each is a
// thin, self-documenting wrapper around the geometry functions above
// plus a fc' tag, so a caller building a full-project takeoff doesn't
// have to re-derive which formwork formula applies to which member type.
ConcreteQuantity makeBeamConcreteQuantity(const std::string& memberLabel, double bM, double hM,
                                           double lengthM, double fcMPa);
ConcreteQuantity makeColumnConcreteQuantity(const std::string& memberLabel, double bM, double hM,
                                             double heightM, double fcMPa);
// hMm is the slab's overall thickness in mm (matching design::RCCSlab's
// own hMm convention), bM/lM its plan dimensions in meters.
ConcreteQuantity makeSlabConcreteQuantity(const std::string& memberLabel, double bM, double lM,
                                           double hMm, double fcMPa);
// tMm is the wall's overall thickness in mm (matching design::
// RCCShearWall's own tMm convention).
ConcreteQuantity makeWallConcreteQuantity(const std::string& memberLabel, double lengthM,
                                           double heightM, double tMm, double fcMPa);
// thicknessMm matches design::RCCFoundation's own thicknessMm convention;
// planBM/planLM are the footing's plan dimensions (equal for a square
// isolated footing).
ConcreteQuantity makeFootingConcreteQuantity(const std::string& memberLabel, double planBM,
                                              double planLM, double thicknessMm, double fcMPa);

// A full project (or building-story) quantity takeoff: concrete items
// from the builders above, plus rebar weight pulled directly from a
// bbs::BarBendingSchedule -- exactly the hand-off the README Roadmap
// describes ("rebar weight (from bbs/) rolled up into a cost/quantity
// takeoff").
class QuantityTakeoff {
public:
    void addConcrete(ConcreteQuantity q);

    // Pulls total rebar weight AND the per-diameter breakdown directly
    // out of a finished bbs::BarBendingSchedule. Calling this more than
    // once replaces the previously-set rebar totals rather than adding
    // to them (a project takeoff has exactly one rebar schedule, not
    // several to sum) -- build one combined BarBendingSchedule first if
    // rebar from multiple sources needs to be included.
    void setRebarSchedule(const bbs::BarBendingSchedule& schedule);

    const std::vector<ConcreteQuantity>& concreteItems() const { return concreteItems_; }

    double totalConcreteVolumeM3() const;
    double totalFormworkAreaM2() const;
    double totalRebarWeightKg() const;

    // fc' (MPa) -> total volume (m^3) across every concrete item of that
    // grade, for a takeoff's usual "summary by concrete grade" table.
    std::map<double, double> concreteVolumeByGradeM3() const;

    // Diameter (mm) -> total weight (kg), passed straight through from
    // the bbs::BarBendingSchedule set via setRebarSchedule().
    std::map<double, double> rebarWeightByDiameterKg() const { return rebarWeightByDiaKg_; }

    // Total rebar weight (kg) per m^3 of concrete -- a standard BOQ/
    // quantity-surveying sanity metric (typical cast-in-place RC
    // buildings run roughly 80-150 kg/m^3 depending on member type and
    // seismic demand), informational only -- this module does not flag
    // or validate against any particular expected range. Returns 0 if
    // no concrete has been added yet.
    double rebarWeightPerConcreteVolumeKgPerM3() const;

private:
    std::vector<ConcreteQuantity> concreteItems_;
    double rebarWeightKg_ = 0.0;
    std::map<double, double> rebarWeightByDiaKg_;
};

}  // namespace nrsa::boq
