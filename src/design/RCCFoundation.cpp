#include "design/RCCFoundation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nrsa::design {

FootingAreaSizingResult sizeSquareFootingArea(double serviceLoadKN, double netAllowableBearingKPa) {
    if (serviceLoadKN <= 0.0) {
        throw std::invalid_argument("sizeSquareFootingArea: service load must be positive");
    }
    if (netAllowableBearingKPa <= 0.0) {
        throw std::invalid_argument("sizeSquareFootingArea: net allowable bearing pressure must be positive");
    }

    FootingAreaSizingResult result;
    result.areaRequiredM2 = serviceLoadKN / netAllowableBearingKPa;
    result.sideM = std::sqrt(result.areaRequiredM2);
    result.note = "Square footing sized from service load / net allowable bearing pressure; "
                  "side is the raw sqrt(area) requirement, not rounded to a constructible increment.";
    return result;
}

TwoWayShearCheckResult checkTwoWayShear(double vuKN, double colBMm, double colHMm, double dMm,
                                         double fcMPa) {
    if (colBMm <= 0.0 || colHMm <= 0.0 || dMm <= 0.0) {
        throw std::invalid_argument("checkTwoWayShear: column dimensions and depth must be positive");
    }
    if (fcMPa <= 0.0) throw std::invalid_argument("checkTwoWayShear: fc' must be positive");

    TwoWayShearCheckResult result;
    double b0Mm = 2.0 * (colBMm + dMm) + 2.0 * (colHMm + dMm);  // ACI 318-19 22.6.4.1, interior column
    result.b0Mm = b0Mm;

    double beta = std::max(colBMm, colHMm) / std::min(colBMm, colHMm);
    double alphaS = 40.0;  // ACI 318-19 22.6.5.3, interior column

    double vc1N = 0.17 * (1.0 + 2.0 / beta) * std::sqrt(fcMPa) * b0Mm * dMm;
    double vc2N = 0.083 * (alphaS * dMm / b0Mm + 2.0) * std::sqrt(fcMPa) * b0Mm * dMm;
    double vc3N = 0.33 * std::sqrt(fcMPa) * b0Mm * dMm;
    double vcN = std::min({vc1N, vc2N, vc3N});

    double phi = 0.75;
    result.vcKN = vcN / 1000.0;
    result.phiVcKN = phi * result.vcKN;
    result.vuKN = vuKN;
    result.adequate = vuKN <= result.phiVcKN;
    result.note = result.adequate
        ? "phi*Vc (two-way/punching, ACI 318-19 22.6.5.2) is adequate at this thickness."
        : "phi*Vc (two-way/punching) is not enough at this thickness -- increase the footing depth.";
    return result;
}

DesignIsolatedFootingResult designIsolatedFooting(double puKN, double serviceLoadKN,
                                                   double netAllowableBearingKPa, double colBM,
                                                   double colHM, double coverMm, double barDiaMm,
                                                   double fcMPa, double fyMPa, double minThicknessMm,
                                                   double thicknessStepMm, int maxIterations) {
    if (puKN <= 0.0) throw std::invalid_argument("designIsolatedFooting: Pu must be positive");
    if (colBM <= 0.0 || colHM <= 0.0) {
        throw std::invalid_argument("designIsolatedFooting: column dimensions must be positive");
    }
    if (coverMm < 0.0 || barDiaMm <= 0.0) {
        throw std::invalid_argument("designIsolatedFooting: cover and bar diameter must be valid");
    }
    if (minThicknessMm <= 0.0 || thicknessStepMm <= 0.0 || maxIterations <= 0) {
        throw std::invalid_argument("designIsolatedFooting: thickness stepping parameters must be positive");
    }

    DesignIsolatedFootingResult result;

    FootingAreaSizingResult areaSizing = sizeSquareFootingArea(serviceLoadKN, netAllowableBearingKPa);
    double sideM = areaSizing.sideM;
    result.sideM = sideM;
    result.netBearingPressureKPa = serviceLoadKN / (sideM * sideM);

    double quKPa = puKN / (sideM * sideM);  // factored net bearing pressure
    result.factoredBearingPressureKPa = quKPa;

    // Governing (larger) one-way cantilever length, in meters, for a given
    // effective depth d (m) -- the column's shorter footprint dimension
    // leaves the LONGER cantilever, which is the more demanding direction
    // for both one-way shear and flexure.
    auto governingCantileverM = [&](double dM) {
        double lc1 = (sideM - colBM) / 2.0 - dM;
        double lc2 = (sideM - colHM) / 2.0 - dM;
        return std::max({0.0, lc1, lc2});
    };

    bool converged = false;
    double thicknessMm = minThicknessMm;
    double dMm = 0.0;
    TwoWayShearCheckResult twoWay;
    OneWayShearCheckResult oneWay;

    for (int iter = 0; iter < maxIterations; ++iter) {
        dMm = thicknessMm - coverMm - barDiaMm / 2.0;
        if (dMm <= 0.0) {
            thicknessMm += thicknessStepMm;
            continue;
        }
        double dM = dMm / 1000.0;

        // Two-way shear: net punching demand is Pu minus the factored
        // bearing pressure already reacting directly beneath the critical
        // perimeter's own footprint (standard footing-design practice --
        // that portion of the column load never has to punch through the
        // slab because it's already balanced against soil pressure
        // immediately below it).
        double critAreaM2 = (colBM + dM) * (colHM + dM);
        double vuTwoWayKN = std::max(0.0, puKN - quKPa * critAreaM2);
        twoWay = checkTwoWayShear(vuTwoWayKN, colBM * 1000.0, colHM * 1000.0, dMm, fcMPa);

        // One-way shear: demand at the critical section d from the column
        // face, governing (longer-cantilever) direction. checkOneWayShear
        // expects a per-meter-width intensity, which is exactly qu*Lc
        // regardless of the footing's actual width (uniform pressure).
        double lcM = governingCantileverM(dM);
        double vuOneWayPerM = quKPa * lcM;
        oneWay = checkOneWayShear(vuOneWayPerM, thicknessMm, coverMm, barDiaMm, fcMPa);

        if (twoWay.adequate && oneWay.adequate) {
            converged = true;
            break;
        }
        thicknessMm += thicknessStepMm;
    }

    if (!converged) {
        throw std::runtime_error(
            "designIsolatedFooting: exceeded maxIterations without satisfying both shear checks -- "
            "check minThicknessMm/thicknessStepMm/maxIterations, or the column-to-footing load ratio "
            "may genuinely need a much larger footing than sizeSquareFootingArea alone suggests.");
    }

    result.thicknessMm = thicknessMm;
    result.effectiveDepthMm = dMm;
    result.twoWayShear = twoWay;
    result.oneWayShear = oneWay;

    double dM = dMm / 1000.0;
    double lcM = governingCantileverM(dM);
    double muPerMKNm = quKPa * lcM * lcM / 2.0;  // ACI 318-19 13.2.7.1, critical section at column face
    double muTotalKNm = muPerMKNm * sideM;       // full-footing-width strip -- see header doc comment
    result.flexure = designFlexure(muTotalKNm, sideM, dM, fcMPa, fyMPa);

    result.note = "Isolated square footing, concentric axial load only (no combined footing/pile "
                  "cases); thickness solved to satisfy both ACI 318-19 two-way and one-way shear, "
                  "then flexural steel designed at the critical section at the column face.";
    return result;
}

// ---------------------------------------------------------------------------
// Combined footing
// ---------------------------------------------------------------------------

CombinedFootingPlanResult sizeCombinedFootingPlan(const std::vector<CombinedFootingColumnLoad>& columns,
                                                   double widthM, double netAllowableBearingKPa,
                                                   double edgeClearanceM) {
    if (columns.size() < 2) {
        throw std::invalid_argument("sizeCombinedFootingPlan: need at least two columns");
    }
    if (widthM <= 0.0) throw std::invalid_argument("sizeCombinedFootingPlan: widthM must be positive");
    if (netAllowableBearingKPa <= 0.0) {
        throw std::invalid_argument("sizeCombinedFootingPlan: net allowable bearing pressure must be positive");
    }
    if (edgeClearanceM < 0.0) {
        throw std::invalid_argument("sizeCombinedFootingPlan: edgeClearanceM must not be negative");
    }

    double totalServiceKN = 0.0;
    for (const auto& c : columns) {
        if (c.puKN <= 0.0 || c.serviceLoadKN <= 0.0) {
            throw std::invalid_argument("sizeCombinedFootingPlan: column loads must be positive");
        }
        if (c.colLengthM <= 0.0 || c.colWidthM <= 0.0) {
            throw std::invalid_argument("sizeCombinedFootingPlan: column dimensions must be positive");
        }
        totalServiceKN += c.serviceLoadKN;
    }

    double xR = 0.0;
    for (const auto& c : columns) xR += c.positionM * c.serviceLoadKN;
    xR /= totalServiceKN;

    double areaM2 = totalServiceKN / netAllowableBearingKPa;
    double lengthByArea = areaM2 / widthM;

    double geomMinFromLeft = 0.0, geomMinFromRight = 0.0;
    for (const auto& c : columns) {
        double nearEdge = c.positionM - c.colLengthM / 2.0;
        double farEdge = c.positionM + c.colLengthM / 2.0;
        geomMinFromLeft = std::max(geomMinFromLeft, (xR - nearEdge) + edgeClearanceM);
        geomMinFromRight = std::max(geomMinFromRight, (farEdge - xR) + edgeClearanceM);
    }
    double geomMinL = 2.0 * std::max(geomMinFromLeft, geomMinFromRight);

    CombinedFootingPlanResult result;
    result.lengthM = std::max(lengthByArea, geomMinL);
    result.widthM = widthM;
    result.footingStartM = xR - result.lengthM / 2.0;
    result.resultantPositionM = xR;
    result.note = result.lengthM > lengthByArea + 1e-9
        ? "Length governed by column edge-clearance geometry, not bearing area."
        : "Length governed by required bearing area.";
    return result;
}

CombinedFootingInternalForces computeCombinedFootingInternalForces(
    const std::vector<CombinedFootingColumnLoad>& columns, const CombinedFootingPlanResult& plan,
    double effectiveDepthM, int numSamples) {
    if (numSamples < 2) throw std::invalid_argument("computeCombinedFootingInternalForces: numSamples too small");

    double totalPuKN = 0.0;
    for (const auto& c : columns) totalPuKN += c.puKN;
    double areaM2 = plan.lengthM * plan.widthM;
    double quKPa = areaM2 > 0.0 ? totalPuKN / areaM2 : 0.0;
    double wKNPerM = quKPa * plan.widthM;  // net upward line load along the beam axis

    std::vector<double> posM;
    posM.reserve(columns.size());
    for (const auto& c : columns) posM.push_back(c.positionM - plan.footingStartM);

    auto momentAt = [&](double s) {
        double m = wKNPerM * s * s / 2.0;
        for (std::size_t i = 0; i < columns.size(); ++i) {
            if (posM[i] <= s) m -= columns[i].puKN * (s - posM[i]);
        }
        return m;
    };
    auto shearAt = [&](double s) {
        double v = wKNPerM * s;
        for (std::size_t i = 0; i < columns.size(); ++i) {
            if (posM[i] <= s) v -= columns[i].puKN;
        }
        return v;
    };

    CombinedFootingInternalForces result;
    auto considerMoment = [&](double s) {
        double m = momentAt(s);
        if (m > result.maxSaggingMomentKNm) {
            result.maxSaggingMomentKNm = m;
            result.saggingPositionM = s;
        }
        if (-m > result.maxHoggingMomentKNm) {
            result.maxHoggingMomentKNm = -m;
            result.hoggingPositionM = s;
        }
    };
    auto considerShear = [&](double s) {
        if (s < 0.0 || s > plan.lengthM) return;
        double v = std::abs(shearAt(s));
        if (v > result.governingShearKN) {
            result.governingShearKN = v;
            result.governingShearPositionM = s;
        }
    };

    for (int i = 0; i <= numSamples; ++i) {
        double s = plan.lengthM * static_cast<double>(i) / numSamples;
        considerMoment(s);
        considerShear(s);
    }
    for (double s : posM) considerMoment(s);
    if (effectiveDepthM > 0.0) {
        for (double s : posM) {
            considerShear(s - effectiveDepthM);
            considerShear(s + effectiveDepthM);
        }
    } else {
        for (double s : posM) considerShear(s);
    }

    return result;
}

DesignCombinedFootingResult designCombinedFooting(const std::vector<CombinedFootingColumnLoad>& columns,
                                                   double widthM, double netAllowableBearingKPa,
                                                   double coverMm, double barDiaMm, double fcMPa,
                                                   double fyMPa, double edgeClearanceM,
                                                   double minThicknessMm, double thicknessStepMm,
                                                   int maxIterations) {
    if (coverMm < 0.0 || barDiaMm <= 0.0) {
        throw std::invalid_argument("designCombinedFooting: cover and bar diameter must be valid");
    }
    if (minThicknessMm <= 0.0 || thicknessStepMm <= 0.0 || maxIterations <= 0) {
        throw std::invalid_argument("designCombinedFooting: thickness stepping parameters must be positive");
    }

    DesignCombinedFootingResult result;
    result.plan = sizeCombinedFootingPlan(columns, widthM, netAllowableBearingKPa, edgeClearanceM);

    double totalPuKN = 0.0;
    for (const auto& c : columns) totalPuKN += c.puKN;
    double areaM2 = result.plan.lengthM * result.plan.widthM;
    result.factoredBearingPressureKPa = totalPuKN / areaM2;

    bool converged = false;
    double thicknessMm = minThicknessMm;
    double dMm = 0.0;
    CombinedFootingInternalForces forces;
    OneWayShearCheckResult oneWay;
    std::vector<TwoWayShearCheckResult> punching;

    for (int iter = 0; iter < maxIterations; ++iter) {
        dMm = thicknessMm - coverMm - barDiaMm / 2.0;
        if (dMm <= 0.0) {
            thicknessMm += thicknessStepMm;
            continue;
        }
        double dM = dMm / 1000.0;

        forces = computeCombinedFootingInternalForces(columns, result.plan, dM);
        double vuOneWayPerM = forces.governingShearKN / result.plan.widthM;
        oneWay = checkOneWayShear(vuOneWayPerM, thicknessMm, coverMm, barDiaMm, fcMPa);

        punching.clear();
        bool allPunchingOk = true;
        for (const auto& c : columns) {
            double critAreaM2 = (c.colLengthM + dM) * (c.colWidthM + dM);
            double vuTwoWayKN = std::max(0.0, c.puKN - result.factoredBearingPressureKPa * critAreaM2);
            auto pw = checkTwoWayShear(vuTwoWayKN, c.colLengthM * 1000.0, c.colWidthM * 1000.0, dMm, fcMPa);
            allPunchingOk = allPunchingOk && pw.adequate;
            punching.push_back(pw);
        }

        if (oneWay.adequate && allPunchingOk) {
            converged = true;
            break;
        }
        thicknessMm += thicknessStepMm;
    }

    if (!converged) {
        throw std::runtime_error(
            "designCombinedFooting: exceeded maxIterations without satisfying all shear checks -- "
            "check minThicknessMm/thicknessStepMm/maxIterations, or the loads may genuinely need a "
            "wider footing (widthM) than the one supplied.");
    }

    result.thicknessMm = thicknessMm;
    result.effectiveDepthMm = dMm;
    result.forces = forces;
    result.oneWayShear = oneWay;
    result.punchingPerColumn = punching;

    double dM = dMm / 1000.0;
    result.bottomFlexure = designFlexure(forces.maxSaggingMomentKNm, result.plan.widthM, dM, fcMPa, fyMPa);
    result.topFlexure = designFlexure(forces.maxHoggingMomentKNm, result.plan.widthM, dM, fcMPa, fyMPa);

    result.note = "Combined footing along one axis, columns assumed centered on the footing's "
                  "B-centerline (no transverse eccentricity/kern check, no transverse cantilever "
                  "steel design); every column's punching perimeter treated as a full 4-sided "
                  "interior perimeter even near the footing ends.";
    return result;
}

// ---------------------------------------------------------------------------
// Pile cap
// ---------------------------------------------------------------------------

PileCapPlanResult sizePileCapPlan(const std::vector<PilePosition>& piles, double pileDiameterM,
                                  double edgeDistanceM) {
    if (piles.empty()) throw std::invalid_argument("sizePileCapPlan: need at least one pile");
    if (pileDiameterM <= 0.0) throw std::invalid_argument("sizePileCapPlan: pileDiameterM must be positive");
    if (edgeDistanceM < 0.0) throw std::invalid_argument("sizePileCapPlan: edgeDistanceM must not be negative");

    double minX = piles[0].xM, maxX = piles[0].xM, minY = piles[0].yM, maxY = piles[0].yM;
    for (const auto& p : piles) {
        minX = std::min(minX, p.xM);
        maxX = std::max(maxX, p.xM);
        minY = std::min(minY, p.yM);
        maxY = std::max(maxY, p.yM);
    }

    PileCapPlanResult result;
    result.lengthM = (maxX - minX) + pileDiameterM + 2.0 * edgeDistanceM;
    result.widthM = (maxY - minY) + pileDiameterM + 2.0 * edgeDistanceM;
    result.note = "Cap sized as the pile group's bounding box plus pile diameter and edge distance "
                  "on every side; not a code-required formula, just plan geometry.";
    return result;
}

std::vector<PilePunchingCheckResult> checkPileIndividualPunching(const std::vector<PilePosition>& piles,
                                                                   const PileCapPlanResult& plan,
                                                                   double pileDiameterM, double dMm,
                                                                   double puKN, double fcMPa) {
    if (piles.empty()) throw std::invalid_argument("checkPileIndividualPunching: need at least one pile");
    if (pileDiameterM <= 0.0 || dMm <= 0.0 || fcMPa <= 0.0) {
        throw std::invalid_argument("checkPileIndividualPunching: pile diameter, depth, and fc' must be positive");
    }

    double rMm = (pileDiameterM * 1000.0 + dMm) / 2.0;
    double demandKN = puKN / static_cast<double>(piles.size());
    const double fullCircle = 2.0 * std::acos(-1.0);

    std::vector<PilePunchingCheckResult> results;
    results.reserve(piles.size());
    for (const auto& p : piles) {
        double distLeftMm = (p.xM + plan.lengthM / 2.0) * 1000.0;
        double distRightMm = (plan.lengthM / 2.0 - p.xM) * 1000.0;
        double distBotMm = (p.yM + plan.widthM / 2.0) * 1000.0;
        double distTopMm = (plan.widthM / 2.0 - p.yM) * 1000.0;

        double exteriorAngle = 0.0;
        for (double dist : {distLeftMm, distRightMm, distBotMm, distTopMm}) {
            if (dist < rMm) {
                double ratio = std::max(-1.0, std::min(1.0, dist / rMm));
                exteriorAngle += 2.0 * std::acos(ratio);
            }
        }
        double remainingAngle = std::max(fullCircle * 0.25, fullCircle - exteriorAngle);
        double b0Mm = rMm * remainingAngle;

        double vc1 = 0.17 * (1.0 + 2.0 / 1.0) * std::sqrt(fcMPa);  // beta=1, circular pile
        double vc2 = 0.083 * (40.0 * dMm / std::max(1.0, b0Mm) + 2.0) * std::sqrt(fcMPa);
        double vc3 = 0.33 * std::sqrt(fcMPa);
        double vcMPa = std::min({vc1, vc2, vc3});
        double phiVcKN = 0.75 * vcMPa * b0Mm * dMm / 1000.0;

        PilePunchingCheckResult r;
        r.xM = p.xM;
        r.yM = p.yM;
        r.b0Mm = b0Mm;
        r.phiVcKN = phiVcKN;
        r.demandKN = demandKN;
        r.adequate = phiVcKN >= demandKN;
        r.truncated = exteriorAngle > 0.0;
        results.push_back(r);
    }
    return results;
}

PileCapMomentDemand computePileCapMomentDemand(const std::vector<PilePosition>& piles, double puKN,
                                                double colBM, double colHM) {
    if (piles.empty()) throw std::invalid_argument("computePileCapMomentDemand: need at least one pile");
    double perPileKN = puKN / static_cast<double>(piles.size());

    auto faceMoment = [&](double coordOf(const PilePosition&), double halfM) {
        double mPos = 0.0, mNeg = 0.0;
        for (const auto& p : piles) {
            double c = coordOf(p);
            if (c > halfM) mPos += perPileKN * (c - halfM);
            if (c < -halfM) mNeg += perPileKN * (-halfM - c);
        }
        return std::max(mPos, mNeg);
    };

    PileCapMomentDemand result;
    result.momentAlongXKNm = faceMoment([](const PilePosition& p) { return p.xM; }, colBM / 2.0);
    result.momentAlongYKNm = faceMoment([](const PilePosition& p) { return p.yM; }, colHM / 2.0);
    return result;
}

double computePileCapOneWayShearDemandPerM(const std::vector<PilePosition>& piles, double puKN,
                                            double colHalfM, double dM, double perpendicularExtentM,
                                            bool alongXAxis) {
    if (piles.empty()) throw std::invalid_argument("computePileCapOneWayShearDemandPerM: need at least one pile");
    if (perpendicularExtentM <= 0.0) {
        throw std::invalid_argument("computePileCapOneWayShearDemandPerM: perpendicularExtentM must be positive");
    }
    double perPileKN = puKN / static_cast<double>(piles.size());
    double critM = colHalfM + dM;
    double totalKN = 0.0;
    for (const auto& p : piles) {
        double c = alongXAxis ? p.xM : p.yM;
        if (std::abs(c) > critM) totalKN += perPileKN;
    }
    return totalKN / perpendicularExtentM;
}

DesignPileCapResult designPileCap(const std::vector<PilePosition>& piles, double puKN, double colBM,
                                   double colHM, double pileDiameterM, double coverMm, double barDiaMm,
                                   double fcMPa, double fyMPa, double edgeDistanceM,
                                   double minThicknessMm, double thicknessStepMm, int maxIterations) {
    if (puKN <= 0.0) throw std::invalid_argument("designPileCap: Pu must be positive");
    if (colBM <= 0.0 || colHM <= 0.0) {
        throw std::invalid_argument("designPileCap: column dimensions must be positive");
    }
    if (coverMm < 0.0 || barDiaMm <= 0.0) {
        throw std::invalid_argument("designPileCap: cover and bar diameter must be valid");
    }
    if (minThicknessMm <= 0.0 || thicknessStepMm <= 0.0 || maxIterations <= 0) {
        throw std::invalid_argument("designPileCap: thickness stepping parameters must be positive");
    }

    DesignPileCapResult result;
    result.plan = sizePileCapPlan(piles, pileDiameterM, edgeDistanceM);

    bool converged = false;
    double thicknessMm = minThicknessMm;
    double dMm = 0.0;
    TwoWayShearCheckResult columnPunching;
    std::vector<PilePunchingCheckResult> pilePunching;
    OneWayShearCheckResult oneWayX, oneWayY;

    for (int iter = 0; iter < maxIterations; ++iter) {
        dMm = thicknessMm - coverMm - barDiaMm / 2.0;
        if (dMm <= 0.0) {
            thicknessMm += thicknessStepMm;
            continue;
        }
        double dM = dMm / 1000.0;

        columnPunching = checkTwoWayShear(puKN, colBM * 1000.0, colHM * 1000.0, dMm, fcMPa);
        pilePunching = checkPileIndividualPunching(piles, result.plan, pileDiameterM, dMm, puKN, fcMPa);
        bool allPilePunchingOk = true;
        for (const auto& pw : pilePunching) allPilePunchingOk = allPilePunchingOk && pw.adequate;

        double vuXPerM = computePileCapOneWayShearDemandPerM(piles, puKN, colBM / 2.0, dM,
                                                              result.plan.widthM, true);
        double vuYPerM = computePileCapOneWayShearDemandPerM(piles, puKN, colHM / 2.0, dM,
                                                              result.plan.lengthM, false);
        oneWayX = checkOneWayShear(vuXPerM, thicknessMm, coverMm, barDiaMm, fcMPa);
        oneWayY = checkOneWayShear(vuYPerM, thicknessMm, coverMm, barDiaMm, fcMPa);

        if (columnPunching.adequate && allPilePunchingOk && oneWayX.adequate && oneWayY.adequate) {
            converged = true;
            break;
        }
        thicknessMm += thicknessStepMm;
    }

    if (!converged) {
        throw std::runtime_error(
            "designPileCap: exceeded maxIterations without satisfying all shear checks -- check "
            "minThicknessMm/thicknessStepMm/maxIterations, or the pile group/column load ratio may "
            "genuinely need a different pile layout than the one supplied.");
    }

    result.thicknessMm = thicknessMm;
    result.effectiveDepthMm = dMm;
    result.columnPunching = columnPunching;
    result.pilePunching = pilePunching;
    result.oneWayShearX = oneWayX;
    result.oneWayShearY = oneWayY;

    double dM = dMm / 1000.0;
    auto moments = computePileCapMomentDemand(piles, puKN, colBM, colHM);
    result.flexureX = designFlexure(moments.momentAlongXKNm, result.plan.widthM, dM, fcMPa, fyMPa);
    result.flexureY = designFlexure(moments.momentAlongYKNm, result.plan.lengthM, dM, fcMPa, fyMPa);

    result.note = "Pile cap under a single concentric column load (rigid-cap method: equal reaction "
                  "per pile); eccentric column moment transfer to the cap is not modeled.";
    return result;
}

}  // namespace nrsa::design
