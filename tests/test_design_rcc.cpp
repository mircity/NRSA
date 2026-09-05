#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "core/Load.h"
#include "core/Model.h"
#include "design/BNBC2020.h"
#include "design/RCCBeam.h"

using namespace nrsa;
using namespace nrsa::design;

static bool approxEqual(double a, double b, double relTol = 1e-3) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// ---- BNBC2020 tests -------------------------------------------------

static void testGeneratesAllSevenWhenEverythingPresent() {
    BnbcLoadCaseIds ids;
    ids.dead = 1; ids.live = 2; ids.liveRoof = 3; ids.snow = 4;
    ids.rain = 5; ids.wind = 6; ids.seismic = 7;
    auto combos = generateBasicCombinations(ids);
    assert(combos.size() == 7);
    std::cout << "  testGeneratesAllSevenWhenEverythingPresent OK\n";
}

static void testSkipsCombinationsForAbsentLoadTypes() {
    // Only dead and live present -- no wind, seismic, or roof-like
    // loads. Combinations 1.4D and 1.2D+1.6L+... should still appear;
    // 0.9D+1.0W and 1.2D+1.0E+... should NOT (nothing to combine).
    BnbcLoadCaseIds ids;
    ids.dead = 1;
    ids.live = 2;
    auto combos = generateBasicCombinations(ids);
    bool foundWindOnly = false, foundSeismicOnly = false;
    for (const auto& c : combos) {
        if (c.name() == "0.9D + 1.0W") foundWindOnly = true;
        if (c.name() == "1.2D + 1.0E + 1.0L + 0.2S") foundSeismicOnly = true;
    }
    assert(!foundWindOnly);
    assert(!foundSeismicOnly);
    bool found14D = false;
    for (const auto& c : combos) {
        if (c.name() == "1.4D") {
            found14D = true;
            assert(approxEqual(c.factorFor(1), 1.4));
            assert(approxEqual(c.factorFor(2), 0.0));  // live not in this combo
        }
    }
    assert(found14D);
    std::cout << "  testSkipsCombinationsForAbsentLoadTypes OK\n";
}

static void testFactorsMatchAci31819Exactly() {
    BnbcLoadCaseIds ids;
    ids.dead = 1; ids.live = 2; ids.wind = 3;
    auto combos = generateBasicCombinations(ids);
    bool found = false;
    for (const auto& c : combos) {
        if (c.name() == "1.2D + 1.0W + 1.0L + 0.5(Lr or S or R)") {
            found = true;
            assert(approxEqual(c.factorFor(1), 1.2));
            assert(approxEqual(c.factorFor(3), 1.0));
            assert(approxEqual(c.factorFor(2), 1.0));
        }
    }
    assert(found);
    std::cout << "  testFactorsMatchAci31819Exactly OK\n";
}

// ---- RCCBeam flexure tests ------------------------------------------

// THE decisive test: round-trip capacity check. Design As for a given
// Mu, then INDEPENDENTLY recompute the section's nominal moment
// capacity phi*Mn from that As using the standard Whitney stress-block
// formula, and verify it reproduces Mu (not just "close" -- this is
// the literal definition of what "design for Mu" means, so an
// approximate match would itself indicate a bug).
static void testFlexuralDesignRoundTrip() {
    double muKNm = 150.0, bM = 0.30, dM = 0.55, fcMPa = 28.0, fyMPa = 420.0;
    auto result = designFlexure(muKNm, bM, dM, fcMPa, fyMPa);
    assert(!result.governedByMinimum);
    assert(!result.exceedsMaximum);

    double bMm = bM * 1000.0, dMm = dM * 1000.0;
    double a = (result.asRequiredMm2 * fyMPa) / (0.85 * fcMPa * bMm);
    double phiMnNmm = 0.9 * result.asRequiredMm2 * fyMPa * (dMm - a / 2.0);
    double phiMnKNm = phiMnNmm / 1.0e6;

    assert(approxEqual(phiMnKNm, muKNm, 1e-6));
    std::cout << "  testFlexuralDesignRoundTrip OK (As=" << result.asRequiredMm2
              << "mm^2, phi*Mn recomputed=" << phiMnKNm << "kNm, Mu=" << muKNm << "kNm)\n";
}

static void testFlexuralDesignGovernedByMinimum() {
    // A tiny moment on a generously-sized section -- minimum
    // reinforcement should govern, not the moment demand.
    double muKNm = 2.0, bM = 0.30, dM = 0.55, fcMPa = 28.0, fyMPa = 420.0;
    auto result = designFlexure(muKNm, bM, dM, fcMPa, fyMPa);
    assert(result.governedByMinimum);
    double expectedRhoMin = std::max(1.4 / fyMPa, std::sqrt(fcMPa) / (4.0 * fyMPa));
    assert(approxEqual(result.rhoProvided, expectedRhoMin, 1e-6));
    std::cout << "  testFlexuralDesignGovernedByMinimum OK\n";
}

static void testFlexuralDesignThrowsWhenImpossible() {
    // An enormous moment on a tiny section -- no singly-reinforced
    // design is possible; must throw rather than return NaN/garbage.
    double muKNm = 5000.0, bM = 0.20, dM = 0.30, fcMPa = 21.0, fyMPa = 420.0;
    bool threw = false;
    try {
        designFlexure(muKNm, bM, dM, fcMPa, fyMPa);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testFlexuralDesignThrowsWhenImpossible OK\n";
}

static void testFlexuralRejectsInvalidInputs() {
    bool threw = false;
    try {
        designFlexure(-1.0, 0.3, 0.5, 28.0, 420.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testFlexuralRejectsInvalidInputs OK\n";
}

// ---- RCCBeam shear tests ---------------------------------------------

static void testShearNotRequiredForSmallVu() {
    auto result = designShear(20.0, 0.30, 0.55, 28.0, 420.0, 10.0, 2);
    assert(!result.stirrupsRequired);
    std::cout << "  testShearNotRequiredForSmallVu OK\n";
}

// THE decisive shear test: round-trip. Design a required spacing for a
// given Vu, then recompute Vs = Av*fy*d/s from that spacing and verify
// phi*(Vc+Vs) reproduces Vu.
static void testShearDesignRoundTrip() {
    double vuKN = 250.0, bM = 0.30, dM = 0.55, fcMPa = 28.0, fyMPa = 420.0;
    double stirrupDiaMm = 10.0;
    int legs = 2;
    auto result = designShear(vuKN, bM, dM, fcMPa, fyMPa, stirrupDiaMm, legs);
    assert(result.stirrupsRequired);
    assert(!result.exceedsMaximumVs);
    assert(result.requiredSpacingMm > 0.0);

    double dMm = dM * 1000.0;
    double avMm2 = legs * (M_PI / 4.0) * stirrupDiaMm * stirrupDiaMm;
    double vsProvidedN = (avMm2 * fyMPa * dMm) / result.requiredSpacingMm;
    double vsProvidedKN = vsProvidedN / 1000.0;
    double phiVnKN = 0.75 * (result.vcKN + vsProvidedKN);

    // The spacing was clamped to maxSpacingMm if the raw calculation
    // exceeded it -- in that case phi*Vn will EXCEED Vu (extra,
    // conservative capacity from the spacing cap), not match exactly.
    // Only expect an exact round-trip when the spacing wasn't clamped.
    double rawSpacing = (avMm2 * fyMPa * dMm) / ((vuKN / 0.75 - result.vcKN) * 1000.0);
    if (rawSpacing <= result.maxSpacingMm) {
        assert(approxEqual(phiVnKN, vuKN, 1e-6));
    } else {
        assert(phiVnKN >= vuKN);
    }
    std::cout << "  testShearDesignRoundTrip OK (s=" << result.requiredSpacingMm
              << "mm, phi*Vn recomputed=" << phiVnKN << "kN, Vu=" << vuKN << "kN)\n";
}

static void testShearExceedsMaximumFlagsCorrectly() {
    auto result = designShear(2000.0, 0.25, 0.40, 21.0, 420.0, 10.0, 2);
    assert(result.exceedsMaximumVs);
    std::cout << "  testShearExceedsMaximumFlagsCorrectly OK\n";
}

int main() {
    std::cout << "test_design_rcc:\n";
    testGeneratesAllSevenWhenEverythingPresent();
    testSkipsCombinationsForAbsentLoadTypes();
    testFactorsMatchAci31819Exactly();
    testFlexuralDesignRoundTrip();
    testFlexuralDesignGovernedByMinimum();
    testFlexuralDesignThrowsWhenImpossible();
    testFlexuralRejectsInvalidInputs();
    testShearNotRequiredForSmallVu();
    testShearDesignRoundTrip();
    testShearExceedsMaximumFlagsCorrectly();
    std::cout << "All design (BNBC2020 + RCCBeam) tests passed.\n";
    return 0;
}
