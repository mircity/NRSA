#include "design/Steel.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nrsa::design {

WShapeSection buildWShape(double dMm, double bfMm, double tfMm, double twMm,
                           const std::string& name) {
    if (dMm <= 0.0 || bfMm <= 0.0 || tfMm <= 0.0 || twMm <= 0.0) {
        throw std::invalid_argument("buildWShape: all plate dimensions must be positive");
    }
    if (2.0 * tfMm >= dMm) {
        throw std::invalid_argument("buildWShape: flange thickness leaves no room for a web (2*tf >= d)");
    }
    if (twMm >= bfMm) {
        throw std::invalid_argument("buildWShape: web thickness must be less than flange width");
    }

    WShapeSection s;
    s.name = name;
    s.dMm = dMm;
    s.bfMm = bfMm;
    s.tfMm = tfMm;
    s.twMm = twMm;

    double h = dMm - 2.0 * tfMm;  // clear web depth between flanges

    s.areaMm2 = 2.0 * bfMm * tfMm + h * twMm;

    // Ix: full bf x d rectangle minus the two (bf-tw) x h "notches" either
    // side of the web -- the standard closed-form I-shape formula.
    s.ixMm4 = (bfMm * dMm * dMm * dMm - (bfMm - twMm) * h * h * h) / 12.0;
    // Iy: two flange plates (about their own strong axis, no parallel-axis
    // term needed since they're centered on the y-axis) plus the web plate.
    s.iyMm4 = (2.0 * tfMm * bfMm * bfMm * bfMm + h * twMm * twMm * twMm) / 12.0;

    s.sxMm3 = s.ixMm4 / (dMm / 2.0);
    s.syMm3 = s.iyMm4 / (bfMm / 2.0);

    // Plastic modulus: 2 * (first moment of half-area about the mid-depth
    // plastic-neutral-axis) -- flange contribution + web-half contribution.
    s.zxMm3 = bfMm * tfMm * (dMm - tfMm) + twMm * h * h / 4.0;
    s.zyMm3 = tfMm * bfMm * bfMm / 2.0 + h * twMm * twMm / 4.0;

    s.rxMm = std::sqrt(s.ixMm4 / s.areaMm2);
    s.ryMm = std::sqrt(s.iyMm4 / s.areaMm2);

    // St. Venant torsional constant for an open thin-walled section built
    // from rectangular plates, J = (1/3)*sum(b_i*t_i^3) -- no fillet
    // correction (see header doc comment).
    s.jMm4 = (2.0 * bfMm * tfMm * tfMm * tfMm + h * twMm * twMm * twMm) / 3.0;

    s.hoMm = dMm - tfMm;  // distance between flange centroids
    // Warping constant for a doubly-symmetric I-shape, Cw = Iy*ho^2/4.
    s.cwMm6 = s.iyMm4 * s.hoMm * s.hoMm / 4.0;
    // AISC F2-7: rts^2 = sqrt(Iy*Cw)/Sx.
    s.rtsMm = std::sqrt(std::sqrt(s.iyMm4 * s.cwMm6) / s.sxMm3);

    return s;
}

CompactnessResult checkCompactness(const WShapeSection& section, double fyMPa) {
    if (fyMPa <= 0.0) throw std::invalid_argument("checkCompactness: Fy must be positive");
    // E is fixed at 200000 MPa throughout structural steel per AISC 360-16
    // -- checkCompactness only needs Fy, the caller supplies E separately
    // to the capacity functions that actually use it in a formula.
    constexpr double kE = 200000.0;

    CompactnessResult r;
    r.lambdaFlange = (section.bfMm / 2.0) / section.tfMm;
    r.lambdaPFlange = 0.38 * std::sqrt(kE / fyMPa);
    r.flangeCompact = r.lambdaFlange <= r.lambdaPFlange;

    double h = section.dMm - 2.0 * section.tfMm;
    r.lambdaWeb = h / section.twMm;
    r.lambdaPWeb = 3.76 * std::sqrt(kE / fyMPa);
    r.webCompact = r.lambdaWeb <= r.lambdaPWeb;

    r.compact = r.flangeCompact && r.webCompact;
    return r;
}

FlexuralCapacityResult designFlexuralCapacity(const WShapeSection& section, double fyMPa, double eMPa,
                                              double lbM, double cb) {
    if (fyMPa <= 0.0 || eMPa <= 0.0) throw std::invalid_argument("designFlexuralCapacity: Fy and E must be positive");
    if (lbM < 0.0) throw std::invalid_argument("designFlexuralCapacity: Lb must be non-negative");
    if (cb <= 0.0) throw std::invalid_argument("designFlexuralCapacity: Cb must be positive");

    FlexuralCapacityResult r;
    auto compactness = checkCompactness(section, fyMPa);
    if (!compactness.compact) {
        r.sectionNotCompact = true;
        r.note = "Section is not compact (Table B4.1b) -- Chapter F3/F4/F5 provisions would govern, "
                 "not implemented here; Mn/phiMn are not computed.";
        return r;
    }

    double lbMm = lbM * 1000.0;

    r.mpKNm = fyMPa * section.zxMm3 / 1.0e6;  // N*mm -> kN*m

    // AISC F2-5.
    double lpMm = 1.76 * section.ryMm * std::sqrt(eMPa / fyMPa);
    r.lpM = lpMm / 1000.0;

    // AISC F2-6 (c=1.0 for a doubly-symmetric I-shape).
    double jcOverSxHo = section.jMm4 / (section.sxMm3 * section.hoMm);
    double term = jcOverSxHo + std::sqrt(jcOverSxHo * jcOverSxHo + 6.76 * std::pow(0.7 * fyMPa / eMPa, 2));
    double lrMm = 1.95 * section.rtsMm * (eMPa / (0.7 * fyMPa)) * std::sqrt(term);
    r.lrM = lrMm / 1000.0;

    double sevenTenthsFySxKNm = 0.7 * fyMPa * section.sxMm3 / 1.0e6;

    if (lbMm <= lpMm) {
        r.regime = FlexuralCapacityResult::Regime::Yielding;
        r.mnKNm = r.mpKNm;
        r.note = "Lb <= Lp -- flexural yielding governs, no lateral-torsional-buckling reduction.";
    } else if (lbMm <= lrMm) {
        r.regime = FlexuralCapacityResult::Regime::InelasticLTB;
        double mn = cb * (r.mpKNm - (r.mpKNm - sevenTenthsFySxKNm) * (lbMm - lpMm) / (lrMm - lpMm));
        r.mnKNm = std::min(mn, r.mpKNm);
        r.note = "Lp < Lb <= Lr -- inelastic lateral-torsional buckling (AISC F2-2).";
    } else {
        r.regime = FlexuralCapacityResult::Regime::ElasticLTB;
        double ratio = lbMm / section.rtsMm;
        double fcrMPa = (cb * M_PI * M_PI * eMPa) / (ratio * ratio) *
                         std::sqrt(1.0 + 0.078 * jcOverSxHo * ratio * ratio);
        double mn = fcrMPa * section.sxMm3 / 1.0e6;
        r.mnKNm = std::min(mn, r.mpKNm);
        r.note = "Lb > Lr -- elastic lateral-torsional buckling (AISC F2-3/F2-4).";
    }

    r.phiMnKNm = 0.90 * r.mnKNm;
    return r;
}

ShearCapacityResult checkShearCapacity(const WShapeSection& section, double fyMPa, double eMPa) {
    if (fyMPa <= 0.0 || eMPa <= 0.0) throw std::invalid_argument("checkShearCapacity: Fy and E must be positive");

    ShearCapacityResult r;
    r.awMm2 = section.dMm * section.twMm;  // AISC G2.1: Aw = d*tw (full depth, unstiffened rolled shape)

    double h = section.dMm - 2.0 * section.tfMm;
    r.hOverTw = h / section.twMm;

    constexpr double kv = 5.34;  // unstiffened web

    if (r.hOverTw <= 2.24 * std::sqrt(eMPa / fyMPa)) {
        // AISC G2.1a -- webs of rolled I-shapes with a stocky enough h/tw.
        r.phi = 1.00;
        r.cv1 = 1.0;
    } else {
        // AISC G2.1b.
        r.phi = 0.90;
        double lim = 1.10 * std::sqrt(kv * eMPa / fyMPa);
        r.cv1 = (r.hOverTw <= lim) ? 1.0 : (lim / r.hOverTw);
    }

    double vnN = 0.6 * fyMPa * r.awMm2 * r.cv1;
    r.vnKN = vnN / 1000.0;
    r.phiVnKN = r.phi * r.vnKN;
    return r;
}

CompressionCapacityResult designCompressionCapacity(const WShapeSection& section, double fyMPa, double eMPa,
                                                     double kxLxM, double kyLyM) {
    if (fyMPa <= 0.0 || eMPa <= 0.0) {
        throw std::invalid_argument("designCompressionCapacity: Fy and E must be positive");
    }
    if (kxLxM <= 0.0 || kyLyM <= 0.0) {
        throw std::invalid_argument("designCompressionCapacity: effective lengths must be positive");
    }

    CompressionCapacityResult r;
    r.klrX = (kxLxM * 1000.0) / section.rxMm;
    r.klrY = (kyLyM * 1000.0) / section.ryMm;
    r.governingKlr = std::max(r.klrX, r.klrY);  // larger KL/r -> lower Fcr -> governs

    r.feMPa = (M_PI * M_PI * eMPa) / (r.governingKlr * r.governingKlr);  // AISC E3-4

    double limit = 4.71 * std::sqrt(eMPa / fyMPa);
    if (r.governingKlr <= limit) {
        // AISC E3-2 -- inelastic buckling.
        r.elasticBuckling = false;
        r.fcrMPa = std::pow(0.658, fyMPa / r.feMPa) * fyMPa;
    } else {
        // AISC E3-3 -- elastic buckling.
        r.elasticBuckling = true;
        r.fcrMPa = 0.877 * r.feMPa;
    }

    double pnN = r.fcrMPa * section.areaMm2;
    r.pnKN = pnN / 1000.0;
    r.phi = 0.90;
    r.phiPnKN = r.phi * r.pnKN;
    return r;
}

CombinedInteractionResult checkCombinedInteraction(double prKN, double pcKN,
                                                    double mrxKNm, double mcxKNm,
                                                    double mryKNm, double mcyKNm) {
    if (pcKN <= 0.0 || mcxKNm <= 0.0 || mcyKNm <= 0.0) {
        throw std::invalid_argument("checkCombinedInteraction: available capacities Pc/Mcx/Mcy must be positive");
    }
    if (prKN < 0.0 || mrxKNm < 0.0 || mryKNm < 0.0) {
        throw std::invalid_argument("checkCombinedInteraction: required demands must be non-negative");
    }

    CombinedInteractionResult r;
    r.prOverPc = prKN / pcKN;
    double momentTerm = mrxKNm / mcxKNm + mryKNm / mcyKNm;

    if (r.prOverPc >= 0.2) {
        r.equation = "H1-1a";
        r.ratio = r.prOverPc + (8.0 / 9.0) * momentTerm;
    } else {
        r.equation = "H1-1b";
        r.ratio = r.prOverPc / 2.0 + momentTerm;
    }

    r.adequate = r.ratio <= 1.0 + 1e-9;
    return r;
}

}  // namespace nrsa::design
