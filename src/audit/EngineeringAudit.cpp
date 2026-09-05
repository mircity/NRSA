#include "audit/EngineeringAudit.h"

#include <sstream>

namespace nrsa::audit {

AuditTrailEntry buildFlexuralAuditTrail(const std::string& memberLabel, double muKNm, double bM,
                                          double dM, double fcMPa, double fyMPa,
                                          const design::FlexuralDesignResult& result) {
    AuditTrailEntry e;
    e.label = memberLabel + " flexure";

    std::ostringstream input;
    input.precision(4);
    input << "Mu=" << muKNm << " kNm, b=" << bM << " m, d=" << dM << " m, fc'=" << fcMPa
          << " MPa, fy=" << fyMPa << " MPa";
    e.input = input.str();

    e.formula = "Rn=Mu/(phi*b*d^2); rho=(0.85fc'/fy)*(1-sqrt(1-2Rn/(0.85fc'))); As=rho*b*d";

    e.codeClause = result.governedByMinimum
                       ? "ACI 318-19 9.6.1.2 (minimum reinforcement)"
                       : "ACI 318-19 22.2.2.4.1 (Whitney rectangular stress block)";

    std::ostringstream calc;
    calc.precision(4);
    calc << "rho_required=" << result.rhoProvided << ", rho_min=" << result.rhoMin
         << ", As_required=" << result.asRequiredMm2 << " mm^2";
    e.calculation = calc.str();

    std::ostringstream check;
    check.precision(4);
    check << "As_required (" << result.asRequiredMm2 << " mm^2) "
          << (result.exceedsMaximum ? ">" : "<=") << " As_max limit";
    e.checkText = check.str();

    e.result = result.exceedsMaximum ? "FAIL" : "PASS";
    return e;
}

}  // namespace nrsa::audit
