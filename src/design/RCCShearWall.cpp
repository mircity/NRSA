#include "design/RCCShearWall.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace nrsa::design {

double inPlaneShearAlphaC(double hwOverLw) {
    if (hwOverLw <= 1.5) return 3.0;
    if (hwOverLw >= 2.0) return 2.0;
    return 3.0 - (hwOverLw - 1.5) / 0.5;
}

double minHorizontalWallReinforcementRatio(double barDiaMm) {
    return (barDiaMm <= 16.0) ? 0.0020 : 0.0025;
}

double minVerticalWallReinforcementRatio(double barDiaMm) {
    return (barDiaMm <= 16.0) ? 0.0012 : 0.0015;
}

double verticalReinforcementRatioFromShear(double rhoT, double hwOverLw, double rhoLMinTable) {
    if (hwOverLw >= 2.5) {
        return std::max(rhoT, rhoLMinTable);
    }
    // ACI 318-19 Eq. 11.6.2: rho_l = 0.0025 + 0.5*(2.5-hw/lw)*(rho_t-0.0025),
    // floored at the Table 11.6.1 vertical minimum. Note the formula is
    // anchored at 0.0025 (the horizontal-minimum band), not at
    // rhoLMinTable itself -- that anchor is a fixed code constant
    // independent of which vertical-minimum band this wall's bar size
    // happens to fall in.
    double rhoL = 0.0025 + 0.5 * (2.5 - hwOverLw) * (rhoT - 0.0025);
    return std::max({rhoL, rhoLMinTable, 0.0025});
}

InPlaneShearDesignResult designInPlaneShear(double vuKN, double lwM, double tM, double hwM,
                                             double fcMPa, double fyMPa, double barDiaMm) {
    if (vuKN < 0.0) throw std::invalid_argument("designInPlaneShear: Vu must be non-negative");
    if (lwM <= 0.0 || tM <= 0.0 || hwM <= 0.0) {
        throw std::invalid_argument("designInPlaneShear: lw, t, and hw must be positive");
    }
    if (fcMPa <= 0.0 || fyMPa <= 0.0) {
        throw std::invalid_argument("designInPlaneShear: fc' and fy must be positive");
    }

    InPlaneShearDesignResult result;
    double lwMm = lwM * 1000.0, tMm = tM * 1000.0, hwMm = hwM * 1000.0;
    double hwOverLw = hwMm / lwMm;
    result.acvMm2 = tMm * lwMm;
    result.alphaC = inPlaneShearAlphaC(hwOverLw);

    double lambda = 1.0;
    double phi = 0.75;

    result.vnCapKN = 8.0 * result.acvMm2 * lambda * std::sqrt(fcMPa) / 1000.0;
    double vnNeededKN = vuKN / phi;

    if (vnNeededKN > result.vnCapKN) {
        result.capGoverned = true;
        result.rhoTRequired = std::numeric_limits<double>::infinity();
        result.rhoTUsed = std::numeric_limits<double>::infinity();
        result.note = "Vu/phi exceeds the ACI 318-19 Section 11.5.4.3 upper bound "
                      "(8*Acv*lambda*sqrt(fc')) -- no amount of horizontal reinforcement makes "
                      "this section adequate; increase thickness or length.";
        result.vnKN = result.vnCapKN;
        result.phiVnKN = phi * result.vnKN;
        result.ratio = vuKN / result.phiVnKN;
        result.adequate = false;
        return result;
    }

    double rhoTBase = (vnNeededKN * 1000.0 / result.acvMm2 - result.alphaC * std::sqrt(fcMPa)) / fyMPa;
    result.rhoTRequired = std::max(0.0, rhoTBase);

    double rhoTMin = minHorizontalWallReinforcementRatio(barDiaMm);
    result.rhoTUsed = std::max(result.rhoTRequired, rhoTMin);
    if (result.rhoTRequired < rhoTMin) {
        result.note = "Horizontal reinforcement governed by the ACI 318-19 Section 11.6.1 minimum, "
                      "not the shear demand.";
    }

    double rhoLMinTable = minVerticalWallReinforcementRatio(barDiaMm);
    result.rhoLRequired = verticalReinforcementRatioFromShear(result.rhoTUsed, hwOverLw, rhoLMinTable);

    double vnN = result.acvMm2 * (result.alphaC * lambda * std::sqrt(fcMPa) + result.rhoTUsed * fyMPa);
    result.vnKN = std::min(vnN / 1000.0, result.vnCapKN);
    result.phiVnKN = phi * result.vnKN;
    result.ratio = (result.phiVnKN > 0.0) ? vuKN / result.phiVnKN : std::numeric_limits<double>::infinity();
    result.adequate = result.ratio <= 1.0 + 1e-9;
    return result;
}

std::vector<double> distributeStoryShearByStiffness(const std::vector<WallSegment>& walls,
                                                     double storyHeightM, double storyShearKN) {
    if (walls.empty()) return {};
    if (storyHeightM <= 0.0) {
        throw std::invalid_argument("distributeStoryShearByStiffness: story height must be positive");
    }
    double hMm = storyHeightM * 1000.0;
    std::vector<double> stiffness(walls.size());
    double sumK = 0.0;
    for (std::size_t i = 0; i < walls.size(); ++i) {
        const auto& w = walls[i];
        if (w.lengthM <= 0.0 || w.thicknessM <= 0.0) {
            throw std::invalid_argument("distributeStoryShearByStiffness: wall length and thickness must be positive");
        }
        double tMm = w.thicknessM * 1000.0, lMm = w.lengthM * 1000.0;
        double iMm4 = tMm * lMm * lMm * lMm / 12.0;
        stiffness[i] = iMm4 / (hMm * hMm * hMm);
        sumK += stiffness[i];
    }
    std::vector<double> result(walls.size());
    for (std::size_t i = 0; i < walls.size(); ++i) {
        result[i] = storyShearKN * stiffness[i] / sumK;
    }
    return result;
}

OutOfPlaneFlexureResult designOutOfPlaneFlexure(double muKNmPerM, double tMm, double coverMm,
                                                 double barDiaMm, double fcMPa, double fyMPa) {
    if (muKNmPerM < 0.0) throw std::invalid_argument("designOutOfPlaneFlexure: Mu must be non-negative");
    if (tMm <= 0.0) throw std::invalid_argument("designOutOfPlaneFlexure: thickness must be positive");
    if (fcMPa <= 0.0 || fyMPa <= 0.0) {
        throw std::invalid_argument("designOutOfPlaneFlexure: fc' and fy must be positive");
    }

    double bMm = 1000.0;  // 1-meter design strip
    double dMm = tMm - coverMm - barDiaMm / 2.0;
    if (dMm <= 0.0) throw std::invalid_argument("designOutOfPlaneFlexure: cover+bar/2 exceeds thickness");

    OutOfPlaneFlexureResult result;
    double phi = 0.9;
    double muNmm = muKNmPerM * 1.0e6;

    result.asMinMm2PerM = minVerticalWallReinforcementRatio(barDiaMm) * bMm * tMm;

    if (muNmm < 1e-9) {
        result.asRequiredMm2PerM = result.asMinMm2PerM;
        result.rhoProvided = result.asMinMm2PerM / (bMm * dMm);
        result.governedByMinimum = true;
        result.note = "Zero (or negligible) moment demand -- governed by the ACI 318-19 Section "
                      "11.6.1 vertical-reinforcement minimum.";
        return result;
    }

    double Rn = muNmm / (phi * bMm * dMm * dMm);
    double discriminant = 1.0 - (2.0 * Rn) / (0.85 * fcMPa);
    if (discriminant < 0.0) {
        throw std::runtime_error(
            "designOutOfPlaneFlexure: Mu exceeds what a singly-reinforced strip of this "
            "thickness can carry -- increase thickness.");
    }
    double rhoComputed = (0.85 * fcMPa / fyMPa) * (1.0 - std::sqrt(discriminant));

    double beta1 = (fcMPa <= 28.0) ? 0.85 : std::max(0.65, 0.85 - 0.05 * (fcMPa - 28.0) / 7.0);
    double rhoMax = 0.85 * fcMPa * beta1 * 0.375 / fyMPa;

    double asComputed = rhoComputed * bMm * dMm;
    if (asComputed < result.asMinMm2PerM) {
        result.asRequiredMm2PerM = result.asMinMm2PerM;
        result.governedByMinimum = true;
        result.note = "Governed by the ACI 318-19 Section 11.6.1 vertical-reinforcement minimum, "
                      "not the moment demand.";
    } else {
        result.asRequiredMm2PerM = asComputed;
    }
    if (rhoComputed > rhoMax) {
        result.exceedsMaximum = true;
        result.note = "Required steel ratio exceeds the tension-controlled limit -- increase the "
                      "wall thickness.";
    }
    result.rhoProvided = result.asRequiredMm2PerM / (bMm * dMm);
    return result;
}

}  // namespace nrsa::design
