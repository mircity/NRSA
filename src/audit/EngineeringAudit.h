#pragma once

#include <string>

#include "design/RCCBeam.h"

namespace nrsa::audit {

// Roadmap Section 16 ("Engineering Audit System"): every check should
// show INPUT -> FORMULA -> CODE CLAUSE -> CALCULATION -> CHECK ->
// RESULT (the roadmap's own example: "Required Ast=xxx, Provided
// Ast=xxx, Governing Clause=BNBC/ACI...").
//
// GENUINE METHOD: this formats that trail directly from a REAL
// design::FlexuralDesignResult -- the same struct
// design::designFlexure returns and tests/test_rcc_beam.cpp already
// hand-calc-verified -- plus the ACI 318-19 clause numbers
// RCCBeam.cpp's own source comments already cite (9.6.1.2 for minimum
// steel, 22.2.2.4.1 for the Whitney block, 21.2.2 for phi). No new
// calculation happens here; this is a formatter over an
// already-computed, already-verified result, same "explanation layer,
// not calculation layer" split Section 10's DesignExplainer follows.
struct AuditTrailEntry {
    std::string label;       // e.g. "Beam B12 flexure"
    std::string input;
    std::string formula;
    std::string codeClause;
    std::string calculation;
    std::string checkText;
    std::string result;      // "PASS" or "FAIL"
};

// Builds the audit trail for a flexural design result. memberLabel is
// used verbatim (e.g. "B12").
AuditTrailEntry buildFlexuralAuditTrail(const std::string& memberLabel, double muKNm, double bM,
                                          double dM, double fcMPa, double fyMPa,
                                          const design::FlexuralDesignResult& result);

}  // namespace nrsa::audit
