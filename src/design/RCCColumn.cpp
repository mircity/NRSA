#include "design/RCCColumn.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace nrsa::design {

namespace {
constexpr double kEcu = 0.003;      // ACI 318-19 22.2.2.1 ultimate concrete strain
constexpr double kEs = 200000.0;    // MPa, reinforcing steel modulus

double beta1For(double fcMPa) {
    if (fcMPa <= 28.0) return 0.85;
    return std::max(0.65, 0.85 - 0.05 * (fcMPa - 28.0) / 7.0);
}

// ACI 318-19 Table 21.2.2, tied-column values (no spiral case modeled).
double phiFromNetTensileStrain(double et, double fyMPa) {
    double eyStrain = fyMPa / kEs;
    if (et <= eyStrain) return 0.65;
    if (et >= 0.005) return 0.90;
    return 0.65 + (et - eyStrain) * (0.25 / (0.005 - eyStrain));
}
}  // namespace

RectColumnLayout generateRectangularLayout(double bM, double hM, double coverMm, double barDiaMm,
                                            int nFaceBarsAlongB, int nFaceBarsAlongH) {
    if (bM <= 0.0 || hM <= 0.0) throw std::invalid_argument("generateRectangularLayout: b and h must be positive");
    if (coverMm < 0.0 || barDiaMm <= 0.0) {
        throw std::invalid_argument("generateRectangularLayout: cover must be >= 0 and bar diameter > 0");
    }
    if (nFaceBarsAlongB < 2 || nFaceBarsAlongH < 2) {
        throw std::invalid_argument("generateRectangularLayout: each face needs >= 2 bars (the two corners)");
    }

    double bMm = bM * 1000.0, hMm = hM * 1000.0;
    double de = coverMm + barDiaMm / 2.0;  // bar-center distance from each face
    if (2.0 * de >= bMm || 2.0 * de >= hMm) {
        throw std::invalid_argument("generateRectangularLayout: cover+bar size leaves no room inside the section");
    }

    RectColumnLayout layout;
    layout.bMm = bMm;
    layout.hMm = hMm;
    layout.barAreaMm2 = (M_PI / 4.0) * barDiaMm * barDiaMm;

    // Four corners.
    layout.barsXY.push_back({de, de});
    layout.barsXY.push_back({bMm - de, de});
    layout.barsXY.push_back({de, hMm - de});
    layout.barsXY.push_back({bMm - de, hMm - de});

    // Intermediate bars along the bottom/top faces (varying x, fixed y),
    // excluding the corners already placed.
    int nMidB = nFaceBarsAlongB - 2;
    for (int i = 1; i <= nMidB; ++i) {
        double x = de + (bMm - 2.0 * de) * static_cast<double>(i) / static_cast<double>(nMidB + 1);
        layout.barsXY.push_back({x, de});
        layout.barsXY.push_back({x, hMm - de});
    }

    // Intermediate bars along the left/right faces (varying y, fixed x).
    int nMidH = nFaceBarsAlongH - 2;
    for (int i = 1; i <= nMidH; ++i) {
        double y = de + (hMm - 2.0 * de) * static_cast<double>(i) / static_cast<double>(nMidH + 1);
        layout.barsXY.push_back({de, y});
        layout.barsXY.push_back({bMm - de, y});
    }

    return layout;
}

double axialCapacityPo(double bMm, double hMm, double astMm2, double fcMPa, double fyMPa) {
    if (bMm <= 0.0 || hMm <= 0.0) throw std::invalid_argument("axialCapacityPo: b and h must be positive");
    if (astMm2 < 0.0) throw std::invalid_argument("axialCapacityPo: Ast must be non-negative");
    if (fcMPa <= 0.0 || fyMPa <= 0.0) throw std::invalid_argument("axialCapacityPo: fc' and fy must be positive");
    double agMm2 = bMm * hMm;
    if (astMm2 > agMm2) throw std::invalid_argument("axialCapacityPo: Ast cannot exceed Ag");
    // ACI 318-19 Eq. 22.4.2.2, force in N.
    double poN = 0.85 * fcMPa * (agMm2 - astMm2) + fyMPa * astMm2;
    return poN / 1000.0;  // kN
}

std::vector<PMDiagramPoint> computeUniaxialInteractionDiagram(double widthMm, double depthMm,
                                                                const std::vector<RebarLayer>& layers,
                                                                double fcMPa, double fyMPa, int numPoints) {
    if (widthMm <= 0.0 || depthMm <= 0.0) {
        throw std::invalid_argument("computeUniaxialInteractionDiagram: width and depth must be positive");
    }
    if (layers.empty()) throw std::invalid_argument("computeUniaxialInteractionDiagram: at least one rebar layer required");
    if (fcMPa <= 0.0 || fyMPa <= 0.0) {
        throw std::invalid_argument("computeUniaxialInteractionDiagram: fc' and fy must be positive");
    }
    if (numPoints < 10) throw std::invalid_argument("computeUniaxialInteractionDiagram: numPoints too coarse");

    double beta1 = beta1For(fcMPa);
    double dMax = 0.0;
    for (const auto& l : layers) dMax = std::max(dMax, l.distFromFaceMm);

    // Sweep c from a small fraction of the depth (deep into the
    // tension-controlled range) out to several depths beyond the section
    // itself, so a=beta1*c has long since clipped to depthMm and every
    // layer's strain has asymptotically approached ecu -- i.e. Pn has
    // converged to (very nearly) Po by the last point.
    double cMin = 0.02 * depthMm;
    double cMax = 6.0 * depthMm;

    std::vector<PMDiagramPoint> diagram;
    diagram.reserve(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(numPoints - 1);
        double c = cMin + t * (cMax - cMin);
        double a = std::min(beta1 * c, depthMm);

        double ccN = 0.85 * fcMPa * widthMm * a;                    // concrete compression force, N
        double mcNmm = ccN * (depthMm / 2.0 - a / 2.0);              // moment about section centroid

        double pnN = ccN;
        double mnNmm = mcNmm;
        double etExtreme = 0.0;

        for (const auto& l : layers) {
            double strain = kEcu * (c - l.distFromFaceMm) / c;  // + compression
            double stress = std::clamp(kEs * strain, -fyMPa, fyMPa);
            double forceN = l.totalAreaMm2 * stress;
            if (l.distFromFaceMm <= a) {
                // Bar sits inside the Whitney block: the concrete term
                // above already counted this area as concrete, so net out
                // the double-counted concrete stress.
                forceN -= l.totalAreaMm2 * 0.85 * fcMPa;
            }
            pnN += forceN;
            mnNmm += forceN * (depthMm / 2.0 - l.distFromFaceMm);

            if (l.distFromFaceMm == dMax) {
                etExtreme = -strain;  // sign-flip: +ve net tensile strain when the extreme layer is in tension
            }
        }

        PMDiagramPoint pt;
        pt.cMm = c;
        pt.pnKN = pnN / 1000.0;
        pt.mnKNm = mnNmm / 1.0e6;
        pt.etExtreme = etExtreme;
        pt.phi = phiFromNetTensileStrain(etExtreme, fyMPa);
        pt.phiPnKN = pt.phi * pt.pnKN;
        pt.phiMnKNm = pt.phi * pt.mnKNm;
        diagram.push_back(pt);
    }

    return diagram;
}

UniaxialCapacityAtE capacityAtEccentricity(const std::vector<PMDiagramPoint>& diagram, double targetEMm) {
    if (diagram.size() < 2) throw std::invalid_argument("capacityAtEccentricity: diagram needs >= 2 points");

    UniaxialCapacityAtE result;
    // diagram is already ordered by increasing c => increasing Pn and
    // (over the compression-member range we care about) decreasing
    // e = Mn/Pn. Walk it looking for the bracket where e crosses the
    // target; skip near-zero-Pn points where e is numerically meaningless.
    constexpr double kMinPnKN = 1e-6;

    bool haveLower = false;
    double eLower = 0.0, pnLower = 0.0, phiLower = 0.0;
    for (const auto& pt : diagram) {
        if (pt.pnKN <= kMinPnKN) continue;
        double e = pt.mnKNm / pt.pnKN * 1000.0;  // kN*m / kN -> m, x1000 -> mm (matches distFromFaceMm units)
        if (haveLower && eLower >= targetEMm && e <= targetEMm) {
            double span = eLower - e;
            double frac = (span > 1e-9) ? (eLower - targetEMm) / span : 0.0;
            result.pnKN = pnLower + frac * (pt.pnKN - pnLower);
            result.phi = phiLower + frac * (pt.phi - phiLower);
            return result;
        }
        haveLower = true;
        eLower = e;
        pnLower = pt.pnKN;
        phiLower = pt.phi;
    }

    // Target eccentricity never bracketed: either bigger than anything the
    // diagram spans (very high moment / low axial load -- outside what a
    // compression-member check like this one covers) or smaller than the
    // diagram's smallest e (essentially pure axial load; return the
    // highest-Pn point as the best available estimate rather than failing).
    if (!diagram.empty() && targetEMm <= (diagram.back().mnKNm / std::max(diagram.back().pnKN, kMinPnKN) * 1000.0)) {
        result.pnKN = diagram.back().pnKN;
        result.phi = diagram.back().phi;
        return result;
    }
    result.outOfRange = true;
    return result;
}

BiaxialDesignResult designBiaxialColumn(double bM, double hM, double coverMm, double barDiaMm,
                                         int nFaceBarsAlongB, int nFaceBarsAlongH, double fcMPa, double fyMPa,
                                         double puKN, double muxKNm, double muyKNm) {
    if (puKN <= 0.0) {
        throw std::invalid_argument(
            "designBiaxialColumn: Pu must be > 0 (net compression) -- this module does not handle net "
            "axial tension or pure-bending members; use RCCBeam for those.");
    }
    if (muxKNm < 0.0 || muyKNm < 0.0) throw std::invalid_argument("designBiaxialColumn: Mux/Muy must be non-negative");

    RectColumnLayout layout = generateRectangularLayout(bM, hM, coverMm, barDiaMm, nFaceBarsAlongB, nFaceBarsAlongH);
    double astMm2 = static_cast<double>(layout.barsXY.size()) * layout.barAreaMm2;

    // Group bars into layers for X-axis bending (moment Mx: strain varies
    // along the H/y dimension; Whitney block width is B) and for Y-axis
    // bending (moment My: strain varies along the B/x dimension; width H).
    std::map<long long, double> layersXKey, layersYKey;  // key = rounded coordinate (0.01mm) -> summed area
    for (const auto& [x, y] : layout.barsXY) {
        layersXKey[std::llround(y * 100.0)] += layout.barAreaMm2;
        layersYKey[std::llround(x * 100.0)] += layout.barAreaMm2;
    }
    std::vector<RebarLayer> layersX, layersY;
    for (const auto& [key, area] : layersXKey) layersX.push_back({static_cast<double>(key) / 100.0, area});
    for (const auto& [key, area] : layersYKey) layersY.push_back({static_cast<double>(key) / 100.0, area});

    auto diagX = computeUniaxialInteractionDiagram(layout.bMm, layout.hMm, layersX, fcMPa, fyMPa);
    auto diagY = computeUniaxialInteractionDiagram(layout.hMm, layout.bMm, layersY, fcMPa, fyMPa);

    BiaxialDesignResult result;
    result.poKN = axialCapacityPo(layout.bMm, layout.hMm, astMm2, fcMPa, fyMPa);
    result.pnMaxKN = 0.80 * result.poKN;  // ACI 318-19 Section 22.4.2.1, tied column

    result.exMm = (muxKNm * 1.0e6) / (puKN * 1000.0);
    result.eyMm = (muyKNm * 1.0e6) / (puKN * 1000.0);

    auto capX = capacityAtEccentricity(diagX, result.exMm);
    auto capY = capacityAtEccentricity(diagY, result.eyMm);

    if (capX.outOfRange || capY.outOfRange) {
        result.note =
            "Demand eccentricity falls outside what this section can carry in net compression at all "
            "(moment too large relative to Pu for a compression-member check) -- increase the section, "
            "add reinforcement, or re-check as a beam-column at a different load combination.";
        result.adequate = false;
        return result;
    }

    result.pnxKN = capX.pnKN;
    result.pnyKN = capY.pnKN;

    // Bresler reciprocal load method, ACI 318-19 Commentary R22.4.2.
    double invPn = 1.0 / result.pnxKN + 1.0 / result.pnyKN - 1.0 / result.poKN;
    if (invPn <= 0.0) {
        // Degenerate case (can happen right at very low axial load, where
        // the reciprocal-load approximation itself is known to break
        // down) -- fall back to the more conservative of the two uniaxial
        // capacities rather than returning a nonsensical negative/infinite
        // combined capacity.
        result.pnBiaxialKN = std::min(result.pnxKN, result.pnyKN);
        result.lowAxialLoadWarning = true;
    } else {
        result.pnBiaxialKN = std::min(1.0 / invPn, result.pnMaxKN);
    }

    // Conservative choice: whichever axis' uniaxial case is closer to
    // compression-controlled governs phi for the combined check.
    result.phi = std::min(capX.phi, capY.phi);
    result.phiPnKN = result.phi * result.pnBiaxialKN;
    result.adequate = result.phiPnKN >= puKN;

    if (puKN / result.poKN < 0.10) {
        result.lowAxialLoadWarning = true;
    }
    if (result.lowAxialLoadWarning) {
        result.note =
            "Pu/Po < 0.10 -- Bresler's reciprocal load method is known (ACI 318-19 Commentary R22.4.2) to "
            "lose accuracy in this lightly-loaded regime; the load-contour method would be more accurate "
            "here but is not implemented in this module. Treat this result as approximate.";
    } else if (!result.adequate) {
        result.note = "phi*Pn < Pu at this eccentricity -- section or reinforcement is inadequate for this demand.";
    } else {
        result.note = "Adequate.";
    }

    return result;
}

}  // namespace nrsa::design
