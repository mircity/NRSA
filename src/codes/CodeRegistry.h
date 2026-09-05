#pragma once

#include <string>

#include "analysis/SeismicAnalysis.h"
#include "analysis/WindAnalysis.h"
#include "design/RCCBeam.h"

namespace nrsa::codes {

// Roadmap Section 6 ("Global Code Engine"): the roadmap's own vision is
// "NRSA CODE ENGINE: Bangladesh -> BNBC 2020/2023, USA -> ACI/ASCE/
// AISC, Europe -> Eurocode 2/3/8, India -> IS 456/1893/13920,
// Australia, Custom Code" -- a full multi-country registry of design
// provisions an engineer selects from with one dropdown.
//
// SCOPE, STATED PLAINLY: building genuinely correct provisions for
// five-plus international codes from scratch is not realistic -- each
// is its own multi-hundred-page document. What this registry actually
// does is narrower but still real, and was GENUINELY EXTENDED
// (2026-08-27) after research confirmed something worth acting on:
// BNBC 2020's wind provisions are an explicit, published adaptation of
// ASCE 7-05 (its qz formula and exposure-category constants match
// ASCE 7 exactly), and BNBC 2020's seismic design spectrum (Eq.
// 6.2.34-6.2.35) is explicitly the Eurocode 8 Type 1 elastic spectrum
// shape with the same site coefficients. That means
// analysis::WindAnalysis and analysis::SeismicAnalysis's ALREADY-
// TESTED formulas are not just "BNBC's formulas" -- they are, for wind,
// also literally ASCE 7's formula, and for seismic, also literally
// Eurocode 8 Type 1's spectrum. Registering DesignCode::ASCE7 and
// DesignCode::Eurocode8 alongside BNBC2020 in this registry is
// therefore NOT fabricating new-code support -- it's recognizing that
// the one formula already built and verified genuinely serves three
// names. ACI318 (a CONCRETE design code, not wind/seismic), Eurocode2
// (concrete), IS456 (concrete), and AS1170 (Australia's loading code,
// not yet cross-checked against anything in this project) remain
// entirely unimplemented -- selecting them throws, loudly, rather than
// silently returning wrong or made-up numbers.
//
// CAVEAT ON THE Eurocode8 ENTRY: Eurocode 8 defines TWO spectrum
// types (Type 1 for higher-seismicity regions, Type 2 for lower) with
// different site-coefficient tables. Only Type 1 -- the one BNBC 2020
// adopted -- is covered here; Type 2 is not.
//
// This remains intentionally a THIN facade, not a reimplementation --
// it forwards to SeismicAnalysis/WindAnalysis's own (already
// hand-calc-tested) functions rather than duplicating their formulas,
// so there is exactly one place either formula lives.
enum class DesignCode { BNBC2020, ASCE7, Eurocode8, ACI318, Eurocode2, IS456, AS1170, Custom };

std::string designCodeName(DesignCode code);

// Returns the requested code's seismic site coefficients for the given
// soil type. Implemented for BNBC2020 and Eurocode8 (Type 1 spectrum
// only -- see class doc comment) via analysis::SiteCoefficients::
// forSoilType. Throws std::invalid_argument for every other DesignCode.
analysis::SiteCoefficients seismicSiteCoefficients(DesignCode code, analysis::SoilType soil);

// Returns the requested code's wind exposure constants for the given
// exposure category. Implemented for BNBC2020 and ASCE7 via
// analysis::ExposureConstants::forCategory. Throws std::invalid_argument
// for every other DesignCode.
analysis::ExposureConstants windExposureConstants(DesignCode code, analysis::ExposureCategory exposure);

// Whether `code` has seismic site coefficients implemented (BNBC2020, Eurocode8).
bool hasSeismicProvisions(DesignCode code);

// Whether `code` has wind exposure constants implemented (BNBC2020, ASCE7).
bool hasWindProvisions(DesignCode code);

// Whether `code` has ANY provisions implemented (either of the above).
bool isImplemented(DesignCode code);

// CONCRETE DESIGN, ADDED 2026-08-27: design::designFlexure/designShear
// (src/design/RCCBeam.h) were independently hand-calc verified this
// session (tests/test_rcc_beam.cpp -- an ACI 318-19 Whitney-block
// textbook example, matched exactly) after having been unverified
// self-reported code from an earlier session. That verification also
// resolved something worth acting on here: RCCBeam.h's own formulas
// (phi=0.9 flexure/0.75 shear, Vc=0.17*sqrt(fc')*b*d, the ACI 318-19
// section numbers it cites) are cited AS ACI 318-19 -- BNBC 2020's own
// concrete chapter is a well-documented adoption of ACI 318's strength
// design method (the same "one already-verified formula legitimately
// serves two code names" situation as ASCE7/Eurocode8 above, this time
// for concrete flexure/shear rather than wind/seismic). Registering
// ACI318 here for concrete design is that same recognition, not new
// fabrication.
//
// Eurocode2 (partial-safety-factor limit state format, different
// stress block) and IS456 (its own working-stress/limit-state hybrid
// tradition) do NOT share ACI 318's formulas the way BNBC does --
// their concrete provisions are genuinely, structurally different, not
// just differently-named copies of the same thing. They remain
// unimplemented for concrete design, same as before.
bool hasConcreteBeamProvisions(DesignCode code);

// Forwards to design::designFlexure. Implemented for BNBC2020 and
// ACI318 (see doc comment above). Throws std::invalid_argument for
// every other DesignCode.
design::FlexuralDesignResult concreteBeamFlexuralDesign(DesignCode code, double muKNm, double bM,
                                                          double dM, double fcMPa, double fyMPa);

// Forwards to design::designShear. Same BNBC2020/ACI318-only scope.
design::ShearDesignResult concreteBeamShearDesign(DesignCode code, double vuKN, double bM, double dM,
                                                    double fcMPa, double fyMPa, double stirrupDiaMm,
                                                    int legs);

}  // namespace nrsa::codes
