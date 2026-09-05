#pragma once

#include <vector>

#include "core/Load.h"
#include "core/Model.h"

namespace nrsa::analysis {

// VERIFIED against BNBC 2020 Part 6 Chapter 2 Table 6.2.16 (fetched and
// checked directly, 2026-08-27): S/TB/TC/TD for SA..SE match this
// table exactly. BNBC 2020's seismic design spectrum (Eq. 6.2.34-6.2.35,
// implemented in normalizedSpectralShape/runEquivalentStaticSeismic
// below) is explicitly the Eurocode 8 Type 1 elastic spectrum shape,
// which is why Eurocode 8 can reuse these same site coefficients (see
// codes::CodeRegistry) rather than needing a separate table.
enum class SoilType { SA, SB, SC, SD, SE };

struct SiteCoefficients {
    double S = 1.0;
    double TB = 0.15;
    double TC = 0.4;
    double TD = 2.0;

    static SiteCoefficients forSoilType(SoilType t);
};

// VERIFIED against BNBC 2020 Table 6.2.20 (fetched and checked directly,
// 2026-08-27): Ct/m for concrete/steel moment frames and "all other
// structural systems" match this table exactly.
struct PeriodCoefficients {
    double Ct = 0.0466;
    double m = 0.9;

    static PeriodCoefficients concreteMomentFrame() { return {0.0466, 0.9}; }
    static PeriodCoefficients steelMomentFrame() { return {0.0724, 0.8}; }
    static PeriodCoefficients other() { return {0.0488, 0.75}; }
};

struct SeismicParameters {
    double Z = 0.20;
    double I = 1.0;
    double R = 5.0;
    double dampingRatioPercent = 5.0;
    SiteCoefficients site = SiteCoefficients::forSoilType(SoilType::SC);
    PeriodCoefficients periodCoeffs = PeriodCoefficients::concreteMomentFrame();
    double overridePeriodSeconds = -1.0;
    bool directionX = true;
};

struct SeismicStoryForce {
    double elevation = 0.0;
    double weight = 0.0;
    double force = 0.0;
    std::vector<int> nodeIds;
};

struct SeismicAnalysisResult {
    double fundamentalPeriodSeconds = 0.0;
    double designSpectralAcceleration = 0.0;
    double baseShear = 0.0;
    double totalSeismicWeight = 0.0;
    std::vector<SeismicStoryForce> storyForces;
};

double normalizedSpectralShape(double periodSeconds, const SiteCoefficients& site, double eta);
double dampingCorrectionFactor(double dampingRatioPercent);
SeismicAnalysisResult runEquivalentStaticSeismic(const Model& model, const SeismicParameters& params);
void applyAsNodalLoads(Model& model, const SeismicAnalysisResult& result, int loadCaseId, bool directionX);

}  // namespace nrsa::analysis
