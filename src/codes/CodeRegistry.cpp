#include "codes/CodeRegistry.h"

#include <stdexcept>

namespace nrsa::codes {

std::string designCodeName(DesignCode code) {
    switch (code) {
        case DesignCode::BNBC2020: return "BNBC 2020 (Bangladesh)";
        case DesignCode::ASCE7: return "ASCE 7 (USA, wind/seismic loads)";
        case DesignCode::Eurocode8: return "Eurocode 8 Type 1 (Europe, seismic)";
        case DesignCode::ACI318: return "ACI 318 (USA, concrete design)";
        case DesignCode::Eurocode2: return "Eurocode 2 (Europe, concrete design)";
        case DesignCode::IS456: return "IS 456 (India, concrete design)";
        case DesignCode::AS1170: return "AS 1170 (Australia)";
        case DesignCode::Custom: return "Custom Code";
    }
    return "Unknown";
}

bool hasSeismicProvisions(DesignCode code) {
    return code == DesignCode::BNBC2020 || code == DesignCode::Eurocode8;
}

bool hasWindProvisions(DesignCode code) {
    return code == DesignCode::BNBC2020 || code == DesignCode::ASCE7;
}

bool hasConcreteBeamProvisions(DesignCode code) {
    return code == DesignCode::BNBC2020 || code == DesignCode::ACI318;
}

bool isImplemented(DesignCode code) {
    return hasSeismicProvisions(code) || hasWindProvisions(code) || hasConcreteBeamProvisions(code);
}

analysis::SiteCoefficients seismicSiteCoefficients(DesignCode code, analysis::SoilType soil) {
    if (!hasSeismicProvisions(code)) {
        throw std::invalid_argument(
            "seismicSiteCoefficients: " + designCodeName(code) +
            " has no seismic provisions implemented in this registry (see codes::CodeRegistry.h's "
            "class doc comment for scope)");
    }
    return analysis::SiteCoefficients::forSoilType(soil);
}

analysis::ExposureConstants windExposureConstants(DesignCode code, analysis::ExposureCategory exposure) {
    if (!hasWindProvisions(code)) {
        throw std::invalid_argument(
            "windExposureConstants: " + designCodeName(code) +
            " has no wind provisions implemented in this registry (see codes::CodeRegistry.h's "
            "class doc comment for scope)");
    }
    return analysis::ExposureConstants::forCategory(exposure);
}

design::FlexuralDesignResult concreteBeamFlexuralDesign(DesignCode code, double muKNm, double bM,
                                                          double dM, double fcMPa, double fyMPa) {
    if (!hasConcreteBeamProvisions(code)) {
        throw std::invalid_argument(
            "concreteBeamFlexuralDesign: " + designCodeName(code) +
            " has no concrete beam provisions implemented in this registry (see "
            "codes::CodeRegistry.h's class doc comment for scope)");
    }
    return design::designFlexure(muKNm, bM, dM, fcMPa, fyMPa);
}

design::ShearDesignResult concreteBeamShearDesign(DesignCode code, double vuKN, double bM, double dM,
                                                    double fcMPa, double fyMPa, double stirrupDiaMm,
                                                    int legs) {
    if (!hasConcreteBeamProvisions(code)) {
        throw std::invalid_argument(
            "concreteBeamShearDesign: " + designCodeName(code) +
            " has no concrete beam provisions implemented in this registry (see "
            "codes::CodeRegistry.h's class doc comment for scope)");
    }
    return design::designShear(vuKN, bM, dM, fcMPa, fyMPa, stirrupDiaMm, legs);
}

}  // namespace nrsa::codes
