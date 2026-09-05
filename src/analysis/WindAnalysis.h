#pragma once

#include <vector>

#include "core/Load.h"
#include "core/Model.h"

namespace nrsa::analysis {

// VERIFIED (2026-08-27): BNBC 2020's wind provisions are an explicit
// adaptation of ASCE 7-05 (confirmed via published comparison studies
// of BNBC 2020 vs ASCE 7 wind provisions), and this project's qz
// formula (velocityPressure() below, 0.613*Kz*Kzt*Kd*V^2*I) matches
// BNBC 2020's own qz=0.000613*Kz*Kd*Kzt*V^2*I exactly (0.000613 in
// kPa == 0.613 in Pa -- same formula, different unit). The exposure
// constants below (alpha, zg, zmin for B/C/D) are the ASCE 7 values
// BNBC 2020 carries over unchanged.
enum class ExposureCategory { B, C, D };

struct ExposureConstants {
    double alpha = 9.5;
    double zg = 274.0;
    double zmin = 4.6;

    static ExposureConstants forCategory(ExposureCategory c);
};

struct WindParameters {
    double basicWindSpeedMs = 47.0;
    double importanceFactor = 1.0;
    double topographicFactor = 1.0;
    double directionalityFactor = 0.85;
    double gustFactor = 0.85;
    double windwardCp = 0.8;
    double leewardCp = -0.5;
    double tributaryWidthM = 1.0;
    ExposureCategory exposure = ExposureCategory::B;
    bool directionX = true;
};

struct WindStoryForce {
    double elevation = 0.0;
    double velocityPressurePa = 0.0;
    double force = 0.0;
    std::vector<int> nodeIds;
};

struct WindAnalysisResult {
    std::vector<WindStoryForce> storyForces;
    double totalForce = 0.0;
};

double velocityPressureExposureCoefficient(double heightM, const ExposureConstants& exposure);
double velocityPressure(double heightM, const WindParameters& params);
WindAnalysisResult runStaticWind(const Model& model, const WindParameters& params);
void applyAsNodalLoads(Model& model, const WindAnalysisResult& result, int loadCaseId, bool directionX);

}  // namespace nrsa::analysis
