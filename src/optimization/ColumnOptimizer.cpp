#include "optimization/ColumnOptimizer.h"
#include <cmath>

#include <stdexcept>

namespace nrsa::optimization {

namespace {
constexpr double kSteelDensityKgPerM3 = 7850.0;  // standard structural/reinforcing steel density
}

OptimizationResult optimizeColumnSection(const std::vector<ColumnCandidate>& candidates, double puKN,
                                          double muxKNm, double muyKNm, double fcMPa, double fyMPa) {
    if (candidates.empty()) {
        throw std::invalid_argument("optimizeColumnSection: no candidates given");
    }

    OptimizationResult result;
    double bestCost = -1.0;

    for (const auto& c : candidates) {
        ColumnOptionResult r;
        r.label = c.label;

        auto layout = design::generateRectangularLayout(c.widthM, c.depthM, c.coverMm, c.barDiaMm,
                                                          c.barsAlongB, c.barsAlongH);
        auto biaxial = design::designBiaxialColumn(c.widthM, c.depthM, c.coverMm, c.barDiaMm,
                                                     c.barsAlongB, c.barsAlongH, fcMPa, fyMPa, puKN,
                                                     muxKNm, muyKNm);

        r.demandCapacityRatio = biaxial.pnBiaxialKN > 0.0 ? puKN / biaxial.pnBiaxialKN : 1e9;
        r.codeCompliant = biaxial.adequate;

        double steelVolumeM3 =
            static_cast<double>(layout.barsXY.size()) * layout.barAreaMm2 * 1.0e-6 * 1.0;  // per 1m length
        double grossVolumeM3 = c.widthM * c.depthM * 1.0;  // per 1m length; steel volume not subtracted (standard simplification)
        r.concreteVolumeM3 = grossVolumeM3;
        r.steelWeightKg = steelVolumeM3 * kSteelDensityKgPerM3;

        double steelWeightTon = r.steelWeightKg / 1000.0;
        r.totalCostKN = r.concreteVolumeM3 * c.concreteUnitPriceKNPerM3 + steelWeightTon * c.steelUnitPriceKNPerTon;

        result.allOptions.push_back(r);
    }

    for (std::size_t i = 0; i < result.allOptions.size(); ++i) {
        const auto& r = result.allOptions[i];
        if (!r.codeCompliant) continue;
        if (bestCost < 0.0 || r.totalCostKN < bestCost) {
            bestCost = r.totalCostKN;
            result.bestOptionIndex = static_cast<int>(i);
        }
    }

    return result;
}

std::vector<ColumnCandidate> generateSquareColumnCandidates(double minSizeM, double maxSizeM,
                                                              double stepM, double coverMm,
                                                              double barDiaMm, int barsAlongB,
                                                              int barsAlongH,
                                                              double concreteUnitPriceKNPerM3,
                                                              double steelUnitPriceKNPerTon) {
    if (minSizeM <= 0.0 || maxSizeM <= 0.0 || stepM <= 0.0 || minSizeM > maxSizeM) {
        throw std::invalid_argument(
            "generateSquareColumnCandidates: sizes/step must be positive and minSizeM <= maxSizeM");
    }
    std::vector<ColumnCandidate> candidates;
    for (double size = minSizeM; size <= maxSizeM + 1e-9; size += stepM) {
        ColumnCandidate c;
        int sizeMm = static_cast<int>(std::round(size * 1000.0));
        c.label = std::to_string(sizeMm) + "x" + std::to_string(sizeMm);
        c.widthM = size;
        c.depthM = size;
        c.coverMm = coverMm;
        c.barDiaMm = barDiaMm;
        c.barsAlongB = barsAlongB;
        c.barsAlongH = barsAlongH;
        c.concreteUnitPriceKNPerM3 = concreteUnitPriceKNPerM3;
        c.steelUnitPriceKNPerTon = steelUnitPriceKNPerTon;
        candidates.push_back(c);
    }
    return candidates;
}

}  // namespace nrsa::optimization
