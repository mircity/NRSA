#pragma once

#include <string>

#include "autodesign/AutoDesign.h"

namespace nrsa::reports {

// Roadmap Sections 13 (Drawing Generation Engine -- "Column Schedule",
// "Beam Schedule") and 18 (Professional Report Generator -- "Beam
// Design", "Column Design" sections of the report): a tabular
// text/CSV-style schedule generated directly from real
// autodesign::AutoDesign results -- no new calculation, a formatter
// over already-verified data, the same "explanation/formatting layer
// over a real result" pattern as Sections 10 and 16.
//
// SCOPE: plain CSV text (opens in Excel/any spreadsheet, consistent
// with io::CsvIO's own choice for the same reason -- no dedicated
// document/PDF layout engine in this C++ project). Column/Beam
// schedules only (Wall/Slab/Footing schedules blocked on the same
// StaticAnalysis shell-force gap AutoDesign itself documents).
std::string generateBeamScheduleCsv(const std::vector<autodesign::BeamDesignSummary>& beams);
std::string generateColumnScheduleCsv(const std::vector<autodesign::ColumnDesignSummary>& columns);

}  // namespace nrsa::reports
