#include <cassert>
#include <cmath>
#include <iostream>

#include "codes/CodeRegistry.h"

using namespace nrsa::codes;
using namespace nrsa::analysis;

static bool approxEqual(double a, double b, double tol = 1e-9) { return std::abs(a - b) <= tol; }

static void testSeismicFacadeForwardsExactly() {
    auto direct = SiteCoefficients::forSoilType(SoilType::SD);
    auto viaRegistry = seismicSiteCoefficients(DesignCode::BNBC2020, SoilType::SD);
    assert(approxEqual(direct.S, viaRegistry.S));
    assert(approxEqual(direct.TB, viaRegistry.TB));
    assert(approxEqual(direct.TC, viaRegistry.TC));
    assert(approxEqual(direct.TD, viaRegistry.TD));
    std::cout << "  testSeismicFacadeForwardsExactly OK\n";
}

static void testWindFacadeForwardsExactly() {
    auto direct = ExposureConstants::forCategory(ExposureCategory::C);
    auto viaRegistry = windExposureConstants(DesignCode::BNBC2020, ExposureCategory::C);
    assert(approxEqual(direct.alpha, viaRegistry.alpha));
    assert(approxEqual(direct.zg, viaRegistry.zg));
    assert(approxEqual(direct.zmin, viaRegistry.zmin));
    std::cout << "  testWindFacadeForwardsExactly OK\n";
}

// ASCE7 and Eurocode8 are now genuinely implemented (shared-formula
// reuse, see class doc comment) -- verify they return the SAME values
// BNBC2020 does (since they're the same underlying table).
static void testAsce7AndEurocode8ShareBNBCValues() {
    auto bnbcWind = windExposureConstants(DesignCode::BNBC2020, ExposureCategory::B);
    auto asce7Wind = windExposureConstants(DesignCode::ASCE7, ExposureCategory::B);
    assert(approxEqual(bnbcWind.alpha, asce7Wind.alpha));
    assert(approxEqual(bnbcWind.zg, asce7Wind.zg));

    auto bnbcSeismic = seismicSiteCoefficients(DesignCode::BNBC2020, SoilType::SC);
    auto ec8Seismic = seismicSiteCoefficients(DesignCode::Eurocode8, SoilType::SC);
    assert(approxEqual(bnbcSeismic.S, ec8Seismic.S));
    assert(approxEqual(bnbcSeismic.TC, ec8Seismic.TC));

    assert(hasWindProvisions(DesignCode::ASCE7));
    assert(!hasSeismicProvisions(DesignCode::ASCE7));  // ASCE7 entry here covers wind only
    assert(hasSeismicProvisions(DesignCode::Eurocode8));
    assert(!hasWindProvisions(DesignCode::Eurocode8));  // Eurocode8 entry here covers seismic only
    std::cout << "  testAsce7AndEurocode8ShareBNBCValues OK\n";
}

static void testUnimplementedCodesRejectLoudly() {
    assert(isImplemented(DesignCode::BNBC2020));
    assert(!isImplemented(DesignCode::Eurocode2));
    assert(!isImplemented(DesignCode::IS456));
    assert(!isImplemented(DesignCode::AS1170));

    bool threw = false;
    try { seismicSiteCoefficients(DesignCode::ACI318, SoilType::SC); }
    catch (const std::invalid_argument&) { threw = true; }
    assert(threw);

    threw = false;
    try { windExposureConstants(DesignCode::Eurocode8, ExposureCategory::B); }  // seismic-only entry, no wind
    catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  testUnimplementedCodesRejectLoudly OK\n";
}

// ACI318 concrete beam design now genuinely implemented -- verify it
// forwards to nrsa::design::designFlexure/designShear exactly, and that
// Eurocode2/IS456 (structurally different provisions) still reject.
static void testAci318ConcreteFacadeForwardsExactly() {
    assert(hasConcreteBeamProvisions(DesignCode::BNBC2020));
    assert(hasConcreteBeamProvisions(DesignCode::ACI318));
    assert(!hasConcreteBeamProvisions(DesignCode::Eurocode2));
    assert(!hasConcreteBeamProvisions(DesignCode::IS456));
    assert(isImplemented(DesignCode::ACI318));

    auto direct = nrsa::design::designFlexure(200.0, 0.3, 0.45, 28.0, 420.0);
    auto viaRegistry = concreteBeamFlexuralDesign(DesignCode::ACI318, 200.0, 0.3, 0.45, 28.0, 420.0);
    assert(approxEqual(direct.asRequiredMm2, viaRegistry.asRequiredMm2));
    assert(approxEqual(direct.rhoProvided, viaRegistry.rhoProvided));

    auto directShear = nrsa::design::designShear(150.0, 0.3, 0.45, 28.0, 420.0, 10.0, 2);
    auto viaRegistryShear = concreteBeamShearDesign(DesignCode::ACI318, 150.0, 0.3, 0.45, 28.0, 420.0, 10.0, 2);
    assert(approxEqual(directShear.vcKN, viaRegistryShear.vcKN));
    assert(approxEqual(directShear.requiredSpacingMm, viaRegistryShear.requiredSpacingMm));

    bool threw = false;
    try { concreteBeamFlexuralDesign(DesignCode::Eurocode2, 200.0, 0.3, 0.45, 28.0, 420.0); }
    catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  testAci318ConcreteFacadeForwardsExactly OK\n";
}

int main() {
    std::cout << "test_code_registry:\n";
    testSeismicFacadeForwardsExactly();
    testWindFacadeForwardsExactly();
    testAsce7AndEurocode8ShareBNBCValues();
    testUnimplementedCodesRejectLoudly();
    testAci318ConcreteFacadeForwardsExactly();
    std::cout << "All tests passed.\n";
    return 0;
}
