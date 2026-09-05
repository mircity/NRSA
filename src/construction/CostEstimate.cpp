#include "construction/CostEstimate.h"

namespace nrsa::construction {

ProjectCostEstimate estimateProjectCost(double concreteVolumeM3, double concreteUnitPricePerM3,
                                         double steelWeightKg, double steelUnitPricePerTon,
                                         double formworkAreaM2, double formworkUnitPricePerM2) {
    ProjectCostEstimate est;
    est.concreteCost = concreteVolumeM3 * concreteUnitPricePerM3;
    est.steelCost = (steelWeightKg / 1000.0) * steelUnitPricePerTon;
    est.formworkCost = formworkAreaM2 * formworkUnitPricePerM2;
    est.totalCost = est.concreteCost + est.steelCost + est.formworkCost;
    return est;
}

}  // namespace nrsa::construction
