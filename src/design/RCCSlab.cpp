#include "design/RCCSlab.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nrsa::design {

double shrinkageTemperatureSteelRatio(double fyMPa) {
    if (fyMPa <= 0.0) throw std::invalid_argument("shrinkageTemperatureSteelRatio: fy must be positive");
    if (fyMPa <= 300.0) return 0.0020;  // Grade 40/50 deformed bars, ACI 318-19 24.4.3.2
    if (fyMPa <= 420.0) return 0.0018;  // Grade 60 deformed bars, the common case
    return std::max(0.0014, 0.0018 * 420.0 / fyMPa);
}

OneWaySlabDesignResult designOneWaySlab(double muKNmPerM, double hMm, double coverMm,
                                         double barDiaMm, double fcMPa, double fyMPa) {
    if (muKNmPerM < 0.0) throw std::invalid_argument("designOneWaySlab: Mu must be non-negative");
    if (hMm <= 0.0 || coverMm < 0.0 || barDiaMm <= 0.0) {
        throw std::invalid_argument("designOneWaySlab: h, cover and bar diameter must be positive");
    }
    if (fcMPa <= 0.0 || fyMPa <= 0.0) {
        throw std::invalid_argument("designOneWaySlab: fc' and fy must be positive");
    }

    double dMm = hMm - coverMm - barDiaMm / 2.0;
    if (dMm <= 0.0) {
        throw std::invalid_argument("designOneWaySlab: cover + half bar diameter exceeds slab thickness");
    }
    double bMm = 1000.0;  // 1-meter design strip
    double phi = 0.9;

    OneWaySlabDesignResult result;
    result.asMinMm2PerM = shrinkageTemperatureSteelRatio(fyMPa) * hMm * bMm;  // gross-area basis
    result.maxBarSpacingMm = std::min(3.0 * hMm, 450.0);  // ACI 318-19 24.3.2

    double muNmm = muKNmPerM * 1.0e6;
    if (muNmm < 1e-9) {
        result.asRequiredMm2PerM = result.asMinMm2PerM;
        result.rhoProvided = result.asMinMm2PerM / (bMm * dMm);
        result.governedByMinimum = true;
        result.note = "Zero (or negligible) moment demand -- governed by the ACI 318-19 24.4.3.2 "
                      "shrinkage-and-temperature minimum.";
        return result;
    }

    double Rn = muNmm / (phi * bMm * dMm * dMm);
    double discriminant = 1.0 - (2.0 * Rn) / (0.85 * fcMPa);
    if (discriminant < 0.0) {
        throw std::runtime_error(
            "designOneWaySlab: Mu exceeds what this thickness can carry as singly reinforced "
            "(required Rn=" + std::to_string(Rn) + " MPa exceeds the balanced-section limit) -- "
            "increase the slab thickness.");
    }
    double rhoComputed = (0.85 * fcMPa / fyMPa) * (1.0 - std::sqrt(discriminant));
    double asFlexure = rhoComputed * bMm * dMm;

    double beta1 = (fcMPa <= 28.0) ? 0.85 : std::max(0.65, 0.85 - 0.05 * (fcMPa - 28.0) / 7.0);
    double rhoMax = 0.85 * fcMPa * beta1 * 0.375 / fyMPa;  // same tension-controlled proxy as RCCBeam
    if (rhoComputed > rhoMax) {
        result.exceedsMaximum = true;
        result.note = "Required steel ratio exceeds the tension-controlled limit -- increase the "
                      "slab thickness.";
    }

    if (asFlexure < result.asMinMm2PerM) {
        result.asRequiredMm2PerM = result.asMinMm2PerM;
        result.governedByMinimum = true;
        if (result.note.empty()) {
            result.note = "Governed by the ACI 318-19 24.4.3.2 shrinkage-and-temperature minimum, "
                          "not the moment demand.";
        }
    } else {
        result.asRequiredMm2PerM = asFlexure;
    }
    result.rhoProvided = result.asRequiredMm2PerM / (bMm * dMm);
    return result;
}

OneWayShearCheckResult checkOneWayShear(double vuKNPerM, double hMm, double coverMm,
                                         double barDiaMm, double fcMPa) {
    if (vuKNPerM < 0.0) throw std::invalid_argument("checkOneWayShear: Vu must be non-negative");
    if (hMm <= 0.0 || coverMm < 0.0 || barDiaMm <= 0.0) {
        throw std::invalid_argument("checkOneWayShear: h, cover and bar diameter must be positive");
    }
    if (fcMPa <= 0.0) throw std::invalid_argument("checkOneWayShear: fc' must be positive");

    double dMm = hMm - coverMm - barDiaMm / 2.0;
    if (dMm <= 0.0) {
        throw std::invalid_argument("checkOneWayShear: cover + half bar diameter exceeds slab thickness");
    }
    double bMm = 1000.0;
    double phi = 0.75;

    OneWayShearCheckResult result;
    double vcN = 0.17 * std::sqrt(fcMPa) * bMm * dMm;  // ACI 318-19 Eq. 22.5.5.1
    result.vcKN = vcN / 1000.0;
    result.phiVcKN = phi * result.vcKN;
    result.adequate = vuKNPerM <= result.phiVcKN;
    result.note = result.adequate
        ? "phi*Vc alone is adequate -- no shear reinforcement needed (the ordinary case for slabs)."
        : "phi*Vc alone is not enough -- increase the slab thickness, since slabs are not "
          "ordinarily stirrup-reinforced in this engine's scope.";
    return result;
}

namespace {

double positiveMomentDivisor(EdgeContinuity c) {
    switch (c) {
        case EdgeContinuity::BothSimple: return 8.0;
        case EdgeContinuity::OneContinuous: return 11.0;    // ACI 318-19 Table 6.5.2, two-span end case
        case EdgeContinuity::BothContinuous: return 16.0;   // interior span
    }
    return 8.0;
}

double negativeMomentDivisor(EdgeContinuity c) {
    switch (c) {
        case EdgeContinuity::BothSimple: return 0.0;        // no continuity -- no negative moment
        case EdgeContinuity::OneContinuous: return 9.0;     // first interior support, two-span case
        case EdgeContinuity::BothContinuous: return 11.0;   // interior support, >2 spans
    }
    return 0.0;
}

}  // namespace

TwoWaySlabDesignResult designTwoWaySlabPanel(double shortSpanM, double longSpanM, double wuKNPerM2,
                                              double hMm, double coverMm, double barDiaMm,
                                              EdgeContinuity shortDirContinuity,
                                              EdgeContinuity longDirContinuity,
                                              double fcMPa, double fyMPa) {
    if (shortSpanM <= 0.0 || longSpanM <= 0.0) {
        throw std::invalid_argument("designTwoWaySlabPanel: spans must be positive");
    }
    if (shortSpanM > longSpanM) {
        throw std::invalid_argument("designTwoWaySlabPanel: shortSpanM must be <= longSpanM");
    }
    if (wuKNPerM2 < 0.0) {
        throw std::invalid_argument("designTwoWaySlabPanel: wu must be non-negative");
    }

    double La = shortSpanM, Lb = longSpanM;
    double La4 = La * La * La * La, Lb4 = Lb * Lb * Lb * Lb;

    TwoWaySlabDesignResult result;
    result.shortSpanLoadKNPerM2 = wuKNPerM2 * Lb4 / (La4 + Lb4);
    result.longSpanLoadKNPerM2 = wuKNPerM2 * La4 / (La4 + Lb4);

    double posDivA = positiveMomentDivisor(shortDirContinuity);
    double negDivA = negativeMomentDivisor(shortDirContinuity);
    double posDivB = positiveMomentDivisor(longDirContinuity);
    double negDivB = negativeMomentDivisor(longDirContinuity);

    result.shortSpanPosMomentKNm = result.shortSpanLoadKNPerM2 * La * La / posDivA;
    result.shortSpanNegMomentKNm =
        (negDivA > 0.0) ? result.shortSpanLoadKNPerM2 * La * La / negDivA : 0.0;
    result.longSpanPosMomentKNm = result.longSpanLoadKNPerM2 * Lb * Lb / posDivB;
    result.longSpanNegMomentKNm =
        (negDivB > 0.0) ? result.longSpanLoadKNPerM2 * Lb * Lb / negDivB : 0.0;

    result.shortSpanPosSteel = designOneWaySlab(result.shortSpanPosMomentKNm, hMm, coverMm, barDiaMm, fcMPa, fyMPa);
    result.shortSpanNegSteel = designOneWaySlab(result.shortSpanNegMomentKNm, hMm, coverMm, barDiaMm, fcMPa, fyMPa);
    result.longSpanPosSteel = designOneWaySlab(result.longSpanPosMomentKNm, hMm, coverMm, barDiaMm, fcMPa, fyMPa);
    result.longSpanNegSteel = designOneWaySlab(result.longSpanNegMomentKNm, hMm, coverMm, barDiaMm, fcMPa, fyMPa);

    result.note = "Moments computed via the Rankine (elastic load-distribution) method with "
                  "ACI 318-19 Table 6.5.2-style continuous-span coefficients -- not the exact "
                  "BNBC 2020 / ACI 318-63 tabulated two-way moment coefficients or a full "
                  "plate-theory analysis; see RCCSlab.h for the documented limitation.";
    return result;
}

}  // namespace nrsa::design
