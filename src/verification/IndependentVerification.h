#pragma once

#include <string>

#include "design/RCCColumn.h"

namespace nrsa::verification {

// Roadmap Section 17 ("Independent Verification Engine"): Design
// Engine -> Result -> Independent Validation Engine -> PASS/WARNING/
// FAIL, "so the software's reliability is enhanced."
//
// GENUINE METHOD, NOT A RUBBER STAMP: re-deriving a result via a
// DIFFERENT calculation path than the primary design function used,
// then comparing -- exactly the same idea this project's own test
// suite has used throughout (e.g. test_rcc_column.cpp's exact
// closed-form proof that the interaction diagram's largest-c point
// must equal design::axialCapacityPo -- two different code paths
// through the same physical quantity). This module packages that
// pattern as a runtime, callable CHECK rather than only a build-time
// test: for a real project's actual column result, does the
// closed-form Po formula agree with what the (separately coded)
// interaction-diagram sweep converges to at its final point? If they
// disagree beyond a tight tolerance, something is wrong in one of the
// two code paths -- exactly the kind of bug an independent check is
// FOR.
//
// SCOPE: this ONE cross-check (Po via closed form vs. Po via the
// interaction diagram's convergence point) for columns. A full
// "Independent Verification Engine" covering beams/walls/slabs would
// need an equally genuine SECOND, independently-coded path for each of
// those too (not just re-calling the same function) -- a natural next
// addition, not fabricated here by re-running the same function twice
// and calling that "independent."
enum class VerificationVerdict { Pass, Warning, Fail };

struct ColumnVerificationResult {
    VerificationVerdict verdict = VerificationVerdict::Fail;
    double poFromClosedForm = 0.0;
    double poFromInteractionDiagram = 0.0;
    double percentDifference = 0.0;
    std::string note;
};

// Runs the cross-check for a bMx h rectangular column with the given
// reinforcement. Pass: agreement within 0.5% (should be near-exact for
// a correctly-implemented pair, per the exact algebraic identity
// test_rcc_column.cpp proves). Warning: within 5% (some discrepancy,
// investigate). Fail: beyond 5% (the two code paths meaningfully
// disagree -- do not trust either result until resolved).
ColumnVerificationResult verifyColumnAxialCapacity(double widthM, double depthM, double coverMm,
                                                     double barDiaMm, int barsAlongB, int barsAlongH,
                                                     double fcMPa, double fyMPa);

}  // namespace nrsa::verification
