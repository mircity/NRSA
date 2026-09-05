#pragma once

#include <string>

#include "core/Model.h"

namespace nrsa::io {

// Full-fidelity CSV export/import of a Model's nodes and elements --
// unlike DxfIO (CAD interop, geometry-only, lossy), this is NRSA's own
// schema, so it round-trips EVERYTHING: material id, section id,
// restraints, and (for nodes) translational/rotational mass, not just
// geometry. Two files are written/read (nodes and elements separately)
// since they're different tables with different columns -- the same
// two-file convention any spreadsheet-based data exchange uses.
//
// This is also what covers the roadmap's "Excel" import/export bullet:
// CSV opens directly in Excel (File > Open, or double-click on most
// systems) and Excel saves back to CSV -- there is no separate .xlsx
// (OOXML zip+XML) binary writer here, since that format needs its own
// dedicated library this offline environment doesn't have. If a true
// .xlsx binary is genuinely required later, this CSV writer's per-row
// logic is the part that would be reused; only the container format
// would need to change.
struct CsvExportPaths {
    std::string nodesCsvPath;
    std::string elementsCsvPath;
};

// Writes model.nodes() to nodesPath (columns: id,x,y,z,restrainUx,
// restrainUy,restrainUz,restrainRx,restrainRy,restrainRz,
// translationalMass,rotMassX,rotMassY,rotMassZ,label) and
// model.elements() to elementsPath (columns: id,kind,materialId,
// sectionId,nodeIds (semicolon-separated, since node count varies
// 2 or 4),label). Throws std::runtime_error if either file cannot be
// opened for writing.
void exportCsv(const Model& model, const std::string& nodesPath, const std::string& elementsPath);

struct CsvImportResult {
    int nodesImported = 0;
    int elementsImported = 0;
};

// Reads nodesPath and elementsPath (in exportCsv's own format -- this
// is NOT a general-purpose arbitrary-CSV importer, it reads back
// exactly what exportCsv writes) and adds the corresponding Nodes and
// Elements to model, using the ids given IN THE FILE (not
// renumbering) -- so this is meant for restoring a model into an
// EMPTY Model (or one known not to collide on those ids), not for
// merging into an already-populated one. Throws std::runtime_error if
// either file cannot be opened, and std::invalid_argument on a
// malformed row (wrong column count, unparseable number, or an
// unrecognized ElementKind name).
CsvImportResult importCsv(Model& model, const std::string& nodesPath, const std::string& elementsPath);

}  // namespace nrsa::io
