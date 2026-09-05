#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "audit/EngineeringAudit.h"

using namespace nrsa::audit;
using namespace nrsa::design;


// The audit trail must faithfully reflect a REAL design result (not
// invent numbers) -- built from the same textbook case
// test_rcc_beam.cpp hand-calc-verifies (expected As=1283.44 mm^2).
static void testAuditTrailReflectsRealResult() {
    auto result = designFlexure(200.0, 0.3, 0.45, 28.0, 420.0);
    auto entry = buildFlexuralAuditTrail("B12", 200.0, 0.3, 0.45, 28.0, 420.0, result);

    assert(entry.label == "B12 flexure");
    assert(entry.input.find("200") != std::string::npos);
    assert(entry.calculation.find("1283") != std::string::npos);
    assert(entry.result == "PASS");
    assert(!result.governedByMinimum);
    assert(entry.codeClause.find("22.2.2.4.1") != std::string::npos);
    std::cout << "  testAuditTrailReflectsRealResult (calc=\"" << entry.calculation << "\") OK\n";
}

// A minimum-governed case must cite the minimum-reinforcement clause,
// not the Whitney-block clause.
static void testAuditTrailCitesMinimumClauseWhenGoverned() {
    auto result = designFlexure(0.0, 0.3, 0.45, 28.0, 420.0);
    auto entry = buildFlexuralAuditTrail("B5", 0.0, 0.3, 0.45, 28.0, 420.0, result);
    assert(result.governedByMinimum);
    assert(entry.codeClause.find("9.6.1.2") != std::string::npos);
    std::cout << "  testAuditTrailCitesMinimumClauseWhenGoverned OK\n";
}

int main() {
    std::cout << "test_engineering_audit:\n";
    testAuditTrailReflectsRealResult();
    testAuditTrailCitesMinimumClauseWhenGoverned();
    std::cout << "All tests passed.\n";
    return 0;
}
