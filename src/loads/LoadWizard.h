#pragma once

#include <unordered_map>
#include <vector>

#include "core/Model.h"

namespace nrsa::loads {

// Roadmap Section 5 ("Load Wizard"): dead-load COMPONENTS beyond
// self-weight (floor finish, partition, ceiling, waterproofing, wall
// load) and live load BY ROOM TYPE, converted to equivalent nodal
// loads on a set of panel corner nodes -- the same "quarter-split
// tributary load" mechanism modeler::AutoModelGenerator's own
// self-weight and analysis::WindAnalysis/SeismicAnalysis's story
// forces all use, applied here to area/line loads sourced from
// occupancy tables instead of member geometry.
//
// LIVE LOAD VALUES -- VERIFIED against BNBC 2020 Table 6.2.3 (fetched
// and checked directly against the official code text, 2026-08-27):
//   Residential: 2.00 kPa ("Dwellings... all other areas except stairs
//     and balconies" / "Hotels and multifamily houses, private rooms")
//   Office: 2.40 kPa ("Office Buildings... Offices")
//   Hospital: 2.00 kPa ("Hospitals... Patient rooms" -- note Operating
//     rooms/laboratories are higher, at 2.90 kPa; this single RoomType
//     uses the patient-room value as the more common default)
//   School: 2.00 kPa ("Schools... Classrooms")
//   Parking: 2.00 kPa ("Garages (passenger vehicles only)")
//   Commercial: 4.80 kPa ("Stores... Retail... First floor" -- upper
//     floor retail is lower, at 3.60 kPa; this single RoomType uses
//     the first-floor value as the more conservative default)
//   Storage: 6.00 kPa ("Storage warehouses... Light" -- Heavy storage
//     is 12.00 kPa; Light is the more common default)
//   Roof: 1.00 kPa (Table 6.2.4, Type I "Ordinary flat roof")
// Each of these is a single representative value where BNBC's own
// table gives a range across sub-cases (e.g. hospital patient room vs.
// operating room) -- the sub-case distinction BNBC draws is exactly
// the kind of detail a real Load Wizard UI would let the engineer
// pick explicitly; this enum's one-value-per-broad-category shape is
// a deliberate simplification, not a claim that BNBC only has 8 numbers.
enum class RoomType { Residential, Office, Hospital, School, Parking, Commercial, Storage, Roof };

double liveLoadKPaForRoomType(RoomType type);

// Dead-load components beyond self-weight, all in kPa (area loads) or
// left at 0 to skip a component -- these have NO sensible universal
// default (floor finish thickness/material varies project to project),
// so every field must be set explicitly by the caller; the values here
// are NOT defaulted to anything resembling "typical", unlike the live
// load table above which at least has a defensible occupancy-based
// starting point.
struct DeadLoadComponents {
    double floorFinishKPa = 0.0;
    double partitionKPa = 0.0;
    double ceilingKPa = 0.0;
    double waterproofingKPa = 0.0;
};

// Applies (floorFinish+partition+ceiling+waterproofing+liveLoad-for-
// roomType) as ONE combined area pressure (kPa) over a rectangular
// panel, split evenly across its 4 corner nodes -- same tributary
// simplification AutoModelGenerator's slab self-weight uses. Returns
// the total force (kN) applied (pressure x panelAreaM2), for the
// caller's own bookkeeping/reporting. deadLoadCaseId and
// liveLoadCaseId may be the same id (combined into one LoadCase) or
// different (kept separate, e.g. for applying different load factors
// later) -- passing -1 for either skips applying that component
// entirely (useful for applying dead and live loads in separate calls
// across many panels without recomputing which is which each time).
struct PanelLoadResult {
    double totalDeadForceKN = 0.0;
    double totalLiveForceKN = 0.0;
};
PanelLoadResult applyRoomLoadToPanel(Model& model, const std::vector<int>& panelCornerNodeIds,
                                      double panelAreaM2, const DeadLoadComponents& deadLoad,
                                      RoomType roomType, int deadLoadCaseId, int liveLoadCaseId);

// Wall (line) load: a wall's own weight per unit length of the beam it
// sits on, computed from the wall's material unit weight, thickness,
// and height -- NOT the wall's self-weight as a standalone element
// (that's already covered if the wall itself is modeled as a shell
// element via modeler::AutoModelGenerator's exterior-wall generation;
// THIS function is for the common alternative modeling choice where a
// wall is represented as a LINE LOAD on the beam beneath it instead of
// as its own shell element -- the two are alternatives, not both
// applied to the same wall). Returns the line load in kN/m.
double wallLineLoadKNPerM(double wallUnitWeightKNm3, double wallThicknessM, double wallHeightM);

}  // namespace nrsa::loads
