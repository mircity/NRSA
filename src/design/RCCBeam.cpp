#include "design/RCCBeam.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nrsa::design {

FlexuralDesignResult designFlexure(double muKNm, double bM, double dM, double fcMPa, double fyMPa) {
    if (muKNm < 0.0) throw std::invalid_argument("designFlexure: Mu must be non-negative");
    if (bM <= 0.0 || dM <= 0.0) throw std::invalid_argument("designFlexure: b and d must be positive");
    if (fcMPa <= 0.0 || fyMPa <= 0.0) {
        throw std::invalid_argument("designFlexure: fc' and fy must be positive");
    }

    double bMm = bM * 1000.0, dMm = dM * 1000.0;
    double phi = 0.9;
    double muNmm = muKNm * 1.0e6;  // 1 kN*m = 1e6 N*mm

    FlexuralDesignResult result;

    if (muNmm < 1e-9) {
        // Zero demand -- governed entirely by minimum reinforcement.
        result.rhoMin = std::max(1.4 / fyMPa, std::sqrt(fcMPa) / (4.0 * fyMPa));
        result.asRequiredMm2 = result.rhoMin * bMm * dMm;
        result.rhoProvided = result.rhoMin;
        result.governedByMinimum = true;
        result.note = "Zero (or negligible) moment demand -- governed by ACI 318-19 Section 9.6.1.2 minimum reinforcement.";
        return result;
    }

    double Rn = muNmm / (phi * bMm * dMm * dMm);
    double discriminant = 1.0 - (2.0 * Rn) / (0.85 * fcMPa);
    if (discriminant < 0.0) {
        throw std::runtime_error(
            "designFlexure: Mu exceeds what ANY singly-reinforced section of this size can carry "
            "(required Rn=" + std::to_string(Rn) + " MPa exceeds the balanced-section limit) -- "
            "increase the section size or use compression reinforcement (not modeled here).");
    }
    double rhoComputed = (0.85 * fcMPa / fyMPa) * (1.0 - std::sqrt(discriminant));

    result.rhoMin = std::max(1.4 / fyMPa, std::sqrt(fcMPa) / (4.0 * fyMPa));

    double beta1 = (fcMPa <= 28.0) ? 0.85 : std::max(0.65, 0.85 - 0.05 * (fcMPa - 28.0) / 7.0);
    // Tension-controlled limit c/d = 0.375 for the standard
    // ecu=0.003/et=0.005 strain-compatibility check (Grade 420/60
    // reinforcement) -- see the header's own doc comment on this being
    // a simplified proxy, not ACI 318-19's exact net-tensile-strain
    // provision.
    result.rhoMax = 0.85 * fcMPa * beta1 * 0.375 / fyMPa;

    double rhoFinal = rhoComputed;
    if (rhoComputed < result.rhoMin) {
        rhoFinal = result.rhoMin;
        result.governedByMinimum = true;
        result.note = "Governed by ACI 318-19 Section 9.6.1.2 minimum reinforcement, not the moment demand.";
    }
    if (rhoComputed > result.rhoMax) {
        result.exceedsMaximum = true;
        result.note = "Required steel ratio exceeds the tension-controlled limit -- increase the "
                      "section size or add compression reinforcement (not modeled here).";
    }

    result.rhoProvided = rhoFinal;
    result.asRequiredMm2 = rhoFinal * bMm * dMm;
    return result;
}

ShearDesignResult designShear(double vuKN, double bM, double dM, double fcMPa, double fyMPa,
                               double stirrupDiaMm, int legs) {
    if (vuKN < 0.0) throw std::invalid_argument("designShear: Vu must be non-negative");
    if (bM <= 0.0 || dM <= 0.0) throw std::invalid_argument("designShear: b and d must be positive");
    if (fcMPa <= 0.0 || fyMPa <= 0.0) {
        throw std::invalid_argument("designShear: fc' and fy must be positive");
    }
    if (stirrupDiaMm <= 0.0 || legs < 2) {
        throw std::invalid_argument("designShear: stirrup diameter must be positive and legs >= 2");
    }

    double bMm = bM * 1000.0, dMm = dM * 1000.0;
    double phi = 0.75;

    ShearDesignResult result;
    // ACI 318-19 Eq. 22.5.5.1 (normal-weight concrete, lambda=1.0).
    double VcN = 0.17 * std::sqrt(fcMPa) * bMm * dMm;
    result.vcKN = VcN / 1000.0;

    if (vuKN <= phi * result.vcKN / 2.0) {
        result.stirrupsRequired = false;
        result.note = "Vu <= phi*Vc/2 -- no stirrups required by ACI 318-19 Section 9.6.3.1.";
        return result;
    }
    result.stirrupsRequired = true;

    double avMm2 = legs * (M_PI / 4.0) * stirrupDiaMm * stirrupDiaMm;
    double vsRequiredKN = std::max(0.0, vuKN / phi - result.vcKN);

    double vsMaxKN = 0.66 * std::sqrt(fcMPa) * bMm * dMm / 1000.0;
    if (vsRequiredKN > vsMaxKN) {
        result.exceedsMaximumVs = true;
        result.note = "Required Vs exceeds the ACI 318-19 Section 22.5.1.2 upper limit "
                      "(0.66*sqrt(fc')*b*d) -- the section is too small for this shear regardless "
                      "of stirrup spacing; increase the section size.";
    }

    // ACI 318-19 Section 9.7.6.2.2 spacing cap: tighter if Vs is "high".
    bool highVs = vsRequiredKN > 0.33 * std::sqrt(fcMPa) * bMm * dMm / 1000.0;
    result.maxSpacingMm = highVs ? std::min(dMm / 4.0, 300.0) : std::min(dMm / 2.0, 600.0);

    if (vsRequiredKN < 1e-9) {
        // Vu is between phi*Vc/2 and phi*Vc -- minimum/nominal
        // stirrups at the maximum spacing govern, not a Vs-derived
        // spacing (Vs demand is essentially zero here).
        result.requiredSpacingMm = result.maxSpacingMm;
        if (result.note.empty()) {
            result.note = "Vu is between phi*Vc/2 and phi*Vc -- minimum stirrups at maximum spacing govern.";
        }
        return result;
    }

    double vsRequiredN = vsRequiredKN * 1000.0;
    double sRequiredMm = (avMm2 * fyMPa * dMm) / vsRequiredN;
    result.requiredSpacingMm = std::min(sRequiredMm, result.maxSpacingMm);
    return result;
}

}  // namespace nrsa::design
