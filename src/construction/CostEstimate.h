#pragma once

namespace nrsa::construction {

// Roadmap Section 15 ("Construction Intelligence" -- "Cost Estimate"
// bullet specifically; Quantity Takeoff/Material/Rebar/Concrete
// Schedule are already covered by src/boq and src/bbs). A simple,
// honest aggregator: total project cost = concrete volume x unit
// price + steel weight x unit price + formwork area x unit price.
// SCOPE: does NOT include Construction Sequence, Progress Tracking, or
// Clash Detection -- those need construction-schedule and 3D-geometry-
// intersection capabilities this project doesn't have; naming this
// module "Cost Estimate" rather than "Construction Intelligence" is
// deliberate, to not imply more than this one piece covers.
struct ProjectCostEstimate {
    double concreteCost = 0.0;
    double steelCost = 0.0;
    double formworkCost = 0.0;
    double totalCost = 0.0;
};

ProjectCostEstimate estimateProjectCost(double concreteVolumeM3, double concreteUnitPricePerM3,
                                         double steelWeightKg, double steelUnitPricePerTon,
                                         double formworkAreaM2, double formworkUnitPricePerM2);

}  // namespace nrsa::construction
