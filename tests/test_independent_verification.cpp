#include <cassert>
#include <cmath>
#include <iostream>

#include "verification/IndependentVerification.h"

using namespace nrsa::verification;

// A real, correctly-implemented column must PASS -- the two
// independent code paths (closed-form Po vs interaction-diagram
// convergence) must agree near-exactly, per the algebraic identity
// test_rcc_column.cpp proves by hand.
static void testCorrectColumnPasses() {
    auto result = verifyColumnAxialCapacity(0.4, 0.4, 40.0, 20.0, 3, 3, 28.0, 420.0);
    assert(result.verdict == VerificationVerdict::Pass);
    assert(result.percentDifference < 0.5);
    std::cout << "  testCorrectColumnPasses (" << result.note << ") OK\n";
}

// Different section sizes/reinforcement must still agree (this isn't
// a coincidence of one specific geometry).
static void testVariousGeometriesAllPass() {
    struct Case { double w, h, cover, dia; int nb, nh; };
    std::vector<Case> cases = {
        {0.3, 0.3, 40, 16, 3, 3}, {0.5, 0.7, 50, 25, 4, 5}, {0.35, 0.35, 40, 20, 4, 4},
    };
    for (const auto& c : cases) {
        auto result = verifyColumnAxialCapacity(c.w, c.h, c.cover, c.dia, c.nb, c.nh, 28.0, 420.0);
        assert(result.verdict == VerificationVerdict::Pass);
    }
    std::cout << "  testVariousGeometriesAllPass OK\n";
}

int main() {
    std::cout << "test_independent_verification:\n";
    testCorrectColumnPasses();
    testVariousGeometriesAllPass();
    std::cout << "All tests passed.\n";
    return 0;
}
