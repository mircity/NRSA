#include "oneclick/OneClickDesign.h"

namespace nrsa::oneclick {

OneClickResult runOneClickDesign(Model& model, const modeler::BuildingGenerationParameters& buildingParams,
                                  const design::BnbcLoadCaseIds& loadCaseIds, double fcMPa, double fyMPa,
                                  double coverMm, double longBarDiaMm, double stirrupDiaMm,
                                  int stirrupLegs, double columnBarDiaMm, int columnBarsAlongB,
                                  int columnBarsAlongH, double netAllowableBearingKPa,
                                  double foundationCoverMm, double foundationBarDiaMm) {
    OneClickResult result;
    result.geometry = modeler::runAutoModelGeneration(model, buildingParams);

    auto combinations = design::generateBasicCombinations(loadCaseIds);

    result.beamDesigns = autodesign::runAutoDesignForBeams(model, combinations, fcMPa, fyMPa, coverMm,
                                                             longBarDiaMm, stirrupDiaMm, stirrupLegs);
    result.columnDesigns = autodesign::runAutoDesignForColumns(model, combinations, fcMPa, fyMPa, coverMm,
                                                                 columnBarDiaMm, columnBarsAlongB,
                                                                 columnBarsAlongH);
    result.foundationDesigns = autodesign::runAutoDesignForFoundations(
        model, result.columnDesigns, netAllowableBearingKPa, fcMPa, fyMPa, foundationCoverMm,
        foundationBarDiaMm);
    return result;
}

}  // namespace nrsa::oneclick
