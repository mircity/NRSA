#pragma once

#include <string>

#include "core/Model.h"

namespace nrsa::io {

// Section 3 ("DXF/DWG Import + Export") scope, stated plainly up front:
//
// DXF is a well-documented, text-based (ASCII) format -- genuinely
// implementable without a proprietary SDK. This module writes and
// reads a MINIMAL, VALID subset of it: a bare ENTITIES section
// containing LINE entities (one per 2-node frame/wall/slab-edge
// element) and POINT entities (one per node), each tagged with a
// LAYER name that encodes the element kind -- since raw DXF geometry
// has no native concept of "this line is a structural column", NRSA's
// own layer-naming convention (NRSA_NODE, NRSA_COLUMN, NRSA_BEAM,
// NRSA_WALL, NRSA_SLAB_EDGE) is what makes round-tripping through DXF
// preserve element kind. A DXF file from another source that doesn't
// use these layer names will still import as plain geometry (see
// importDxf's behavior below).
//
// NOT implemented, and why:
//   - Native DWG (binary, proprietary Autodesk format, no public spec)
//   - IFC (STEP/EXPRESS schema -- a full, correct implementation is a
//     multi-month effort in its own right, not something to fake with
//     a partial, non-conformant writer)
//   - RVT (Revit's proprietary binary format) / Navisworks (NWD/NWC,
//     also proprietary) -- the roadmap document itself already flags
//     these three as infeasible ("proprietary format") for the same
//     reason DWG is; that assessment is correct and is not revisited
//     here.
//   - True .xlsx (OOXML zip+XML) binary export -- see CsvIO.h; CSV
//     opens directly in Excel and is what's implemented instead.
// A model round-tripped through DXF loses MATERIAL and SECTION
// assignments (DXF geometry has no field for them) -- only geometry
// and element-kind-via-layer survive. CsvIO.h's format is
// full-fidelity for exactly this reason; DXF is for CAD interop, CSV
// is for complete NRSA-to-NRSA or NRSA-to-spreadsheet data exchange.
struct DxfImportResult {
    int nodesImported = 0;
    int elementsImported = 0;
    std::vector<std::string> unrecognizedLayers;  // layer names seen that don't match the NRSA_* convention -- still imported as generic Beam-kind elements, but flagged here so the caller knows kind was guessed
};

// Writes model's nodes (as POINT entities, layer NRSA_NODE) and every
// 2-node element (as a LINE entity, layer NRSA_<KIND>) to a minimal
// valid ASCII DXF (R12-compatible group-code ENTITIES section) at
// outputPath. Shell elements (Slab/Wall, 4 nodes) are written as their
// 4 boundary edges (4 LINE entities each) rather than as a single
// entity, since DXF has no native quad-face primitive in this minimal
// subset. Throws std::runtime_error if the file cannot be opened for
// writing.
void exportDxf(const Model& model, const std::string& outputPath);

// Reads LINE and POINT entities from a DXF file at inputPath (the
// ENTITIES section's group-code stream is parsed directly -- HEADER/
// TABLES/BLOCKS sections, if present, are skipped, not interpreted) and
// adds corresponding Nodes and 2-node Elements to model, starting from
// nextNodeId/nextElementId (incremented in place, same
// caller-owned-counter convention as modeler::runAutoModelGeneration).
// POINT entities become Nodes; LINE entities become 2-node Elements,
// with ElementKind read from the LAYER name if it matches the NRSA_*
// convention (see class doc comment), defaulting to ElementKind::Beam
// (and recording the layer in DxfImportResult::unrecognizedLayers)
// otherwise. Every imported element is assigned materialId/sectionId
// as given (DXF carries no material/section data of its own -- see
// class doc comment). Coincident LINE endpoints across multiple
// entities are NOT merged into shared nodes -- each LINE's two
// endpoints become two new Nodes even if another LINE ended at the
// same coordinates (node-merging by coordinate is a natural next
// addition, not built here). Throws std::runtime_error if the file
// cannot be opened, and std::invalid_argument if a LINE/POINT entity's
// coordinate group codes are malformed.
DxfImportResult importDxf(Model& model, const std::string& inputPath, int materialId, int sectionId,
                           int& nextNodeId, int& nextElementId);

}  // namespace nrsa::io
