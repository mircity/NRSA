#include "design/Settlement.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nrsa::design {

double soilModulusFromSptN_Bowles1997(double sptN) {
    if (sptN < 0.0) throw std::invalid_argument("soilModulusFromSptN_Bowles1997: N cannot be negative");
    return 400.0 * (sptN + 6.0);
}

double schmertmannIzPeak(double netQ, double sigmaVoDf) {
    double denom = std::max(1.0, sigmaVoDf);
    double ratio = std::max(0.01, netQ / denom);
    return 0.5 + 0.1 * std::sqrt(ratio);
}

double schmertmannDepthCorrectionC1(double netQ, double sigmaVoDf) {
    if (netQ <= 0.0) throw std::invalid_argument("schmertmannDepthCorrectionC1: netQ must be positive");
    double c1 = 1.0 - 0.5 * (sigmaVoDf / netQ);
    return std::max(0.5, c1);
}

double schmertmannStrainInfluenceFactor(double z, double B, double izPeak) {
    if (z < 0.0) throw std::invalid_argument("schmertmannStrainInfluenceFactor: z cannot be negative");
    double zPeak = B / 2.0;
    double zMax = 2.0 * B;
    if (z >= zMax) return 0.0;
    if (z <= zPeak) {
        // Linear from (0, 0.1) to (zPeak, izPeak).
        return 0.1 + (izPeak - 0.1) * (z / zPeak);
    }
    // Linear from (zPeak, izPeak) to (zMax, 0).
    return izPeak * (zMax - z) / (zMax - zPeak);
}

SchmertmannResult runSchmertmannSettlement(double B, double netQ, double sigmaVoDf,
                                            const std::vector<SoilLayer>& layers) {
    if (B <= 0.0) throw std::invalid_argument("runSchmertmannSettlement: footing width must be positive");
    if (layers.empty()) throw std::invalid_argument("runSchmertmannSettlement: no soil layers given");

    double zMax = 2.0 * B;
    double totalThickness = 0.0;
    for (const auto& l : layers) totalThickness += l.thicknessM;
    if (totalThickness < zMax - 1e-6) {
        throw std::invalid_argument(
            "runSchmertmannSettlement: soil layers only cover " + std::to_string(totalThickness) +
            " m, less than the 2B = " + std::to_string(zMax) + " m influence zone");
    }

    double izPeak = schmertmannIzPeak(netQ, sigmaVoDf);
    double c1 = schmertmannDepthCorrectionC1(netQ, sigmaVoDf);

    SchmertmannResult result;
    result.izPeak = izPeak;
    result.depthCorrectionC1 = c1;
    result.influenceZoneDepthM = zMax;
    result.layerContributionsM.reserve(layers.size());

    double sumIzOverEsDz = 0.0;
    double z = 0.0;
    for (const auto& layer : layers) {
        double layerTop = z;
        double layerBottom = z + layer.thicknessM;
        double effectiveBottom = std::min(layerBottom, zMax);
        double effectiveThickness = effectiveBottom - layerTop;
        double contribution = 0.0;
        if (effectiveThickness > 0.0) {
            double zMid = layerTop + effectiveThickness / 2.0;
            double iz = schmertmannStrainInfluenceFactor(zMid, B, izPeak);
            double es = soilModulusFromSptN_Bowles1997(layer.sptN);
            double izOverEsDz = (iz / es) * effectiveThickness;
            sumIzOverEsDz += izOverEsDz;
            contribution = c1 * netQ * izOverEsDz;
        }
        result.layerContributionsM.push_back(contribution);
        z = layerBottom;
        if (z >= zMax) break;
    }

    result.settlementM = c1 * netQ * sumIzOverEsDz;
    return result;
}

}  // namespace nrsa::design
