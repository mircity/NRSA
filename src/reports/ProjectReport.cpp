#include "reports/ProjectReport.h"

#include <sstream>

namespace nrsa::reports {

std::string generateBeamScheduleCsv(const std::vector<autodesign::BeamDesignSummary>& beams) {
    std::ostringstream csv;
    csv << "ElementId,Mu_kNm,Vu_kN,As_required_mm2,GovernedByMinimum,StirrupsRequired,RequiredSpacing_mm\n";
    for (const auto& b : beams) {
        csv << b.elementId << ',' << b.muKNm << ',' << b.vuKN << ',' << b.flexure.asRequiredMm2 << ','
            << (b.flexure.governedByMinimum ? "Yes" : "No") << ','
            << (b.shear.stirrupsRequired ? "Yes" : "No") << ',' << b.shear.requiredSpacingMm << '\n';
    }
    return csv.str();
}

std::string generateColumnScheduleCsv(const std::vector<autodesign::ColumnDesignSummary>& columns) {
    std::ostringstream csv;
    csv << "ElementId,Pu_kN,Mux_kNm,Muy_kNm,DemandCapacityRatio,Adequate\n";
    for (const auto& c : columns) {
        csv << c.elementId << ',' << c.puKN << ',' << c.muxKNm << ',' << c.muyKNm << ','
            << c.demandCapacityRatio << ',' << (c.biaxial.adequate ? "Yes" : "No") << '\n';
    }
    return csv.str();
}

}  // namespace nrsa::reports
