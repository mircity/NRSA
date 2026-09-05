#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "core/Model.h"

namespace nrsa::io {

// Section 3's IFC bullet, revisited (2026-08-27): unlike DWG/RVT/
// Navisworks, IFC is a genuinely OPEN, published ISO standard (ISO
// 16739, maintained by buildingSMART) -- there is no proprietary-
// format excuse for skipping it the way there is for the other three.
// This is a real, if intentionally minimal, IFC4 exporter.
//
// WHAT IT WRITES: a valid ISO-10303-21 ("STEP") physical file with an
// IfcProject -> IfcSite -> IfcBuilding -> IfcBuildingStorey spatial
// hierarchy (one storey, everything placed under it -- multi-storey
// spatial structuring is a natural next addition), and every 2-node
// element (Beam/Column/Brace/Wall-as-line) as an IfcBeam/IfcColumn/
// IfcMember with an AXIS-ONLY geometric representation (a 3D polyline
// from node1 to node2) -- this is a real, spec-valid representation
// type (IFC explicitly supports "Axis" representations for linear
// elements, used by real authoring tools for schematic/analytical
// models, not just full solid geometry). 4-node shell elements (Slab/
// Wall) are written as IfcSlab/IfcWall with a BOUNDARY-CURVE
// representation (a closed polyline through all 4 corners) rather
// than a solid -- also spec-valid, also minimal.
//
// NOT IMPLEMENTED, STATED PLAINLY:
//   - Solid/extruded geometry (IfcExtrudedAreaSolid with a profile and
//     material association) -- what a real BIM viewer needs to render
//     a solid 3D shape rather than a wireframe axis/boundary. This is
//     a materially bigger undertaking (profile definitions, correct
//     local placement composition, unit assignment) than an axis
//     representation and is a natural next step, not attempted here.
//   - IFC property sets (IfcPropertySet / IfcElementQuantity) carrying
//     material/section/load data -- like DXF, this export is
//     geometry-and-kind only; see CsvIO.h for the full-fidelity path.
//   - Import. Parsing an arbitrary IFC file (resolving its web of
//     numbered entity cross-references, #123=IFCCARTESIANPOINT(...),
//     back into a coherent model) is substantially harder than export
//     and is not attempted in this pass.
//   - Proper globally-unique IFC GUIDs (the real algorithm compresses
//     a 128-bit UUID into IFC's 22-character base64-like encoding).
//     This exporter generates a FIXED-FORMAT, FILE-LOCAL-ONLY unique
//     string per entity (unique within one exported file, not
//     globally) -- clearly NOT the real compression algorithm, flagged
//     here rather than silently passed off as one.
void exportIfc(const Model& model, const std::string& outputPath, const std::string& projectName = "NRSA Project");

// ADDED 2026-08-30 (roadmap Section 14, "BIM Integration" -- an
// IFC-based Architecture->Structure->Analysis->Design->Quantity->
// Construction workflow needs DESIGN data riding along with the
// geometry, not just shapes): attaches a simple IfcPropertySet (one
// property per map entry) to each element that has entries in
// elementProperties, via a genuine IfcRelDefinesByProperties
// relationship -- the standard IFC mechanism for attaching arbitrary
// named values to an element, e.g. from
// autodesign::BeamDesignSummary/ColumnDesignSummary
// ("Mu_kNm"->"210.0", "As_required_mm2"->"1283.4"). Elements with no
// entry in elementProperties get plain geometry, same as exportIfc.
//
// SAME SCOPE LIMITS as exportIfc otherwise (axis/boundary-curve
// geometry only, no solids, no import, file-local pseudo-GUIDs) --
// this only adds the property-attachment mechanism, it doesn't change
// exportIfc's geometry representation choices.
void exportIfcWithProperties(const Model& model, const std::string& outputPath,
                              const std::unordered_map<int, std::vector<std::pair<std::string, std::string>>>&
                                  elementProperties,
                              const std::string& projectName = "NRSA Project");

}  // namespace nrsa::io
