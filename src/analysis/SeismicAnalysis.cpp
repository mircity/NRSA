#include "analysis/SeismicAnalysis.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace nrsa::analysis {

namespace {
constexpr double kGravity = 9.81;
constexpr double kElevationToleranceM = 0.001;
}

SiteCoefficients SiteCoefficients::forSoilType(SoilType t) {
    switch (t) {
        case SoilType::SA: return {1.0, 0.15, 0.4, 2.0};
        case SoilType::SB: return {1.2, 0.15, 0.5, 2.0};
        case SoilType::SC: return {1.15, 0.20, 0.6, 2.0};
        case SoilType::SD: return {1.35, 0.20, 0.8, 2.0};
        case SoilType::SE: return {1.4, 0.15, 0.5, 2.0};
    }
    return {};
}

double dampingCorrectionFactor(double dampingRatioPercent) {
    double eta = std::sqrt(10.0 / (5.0 + dampingRatioPercent));
    return std::max(eta, 0.55);
}

double normalizedSpectralShape(double T, const SiteCoefficients& site, double eta) {
    if (T < 0.0) throw std::invalid_argument("normalizedSpectralShape: negative period");
    const double S = site.S, TB = site.TB, TC = site.TC, TD = site.TD;
    if (T <= TB) return S * (1.0 + (T / TB) * (2.5 * eta - 1.0));
    else if (T <= TC) return S * 2.5 * eta;
    else if (T <= TD) return S * 2.5 * eta * (TC / T);
    else return S * 2.5 * eta * (TC * TD / (T * T));
}

namespace {
std::vector<SeismicStoryForce> groupByStory(const Model& model) {
    std::map<double, SeismicStoryForce> stories;
    for (const auto& node : model.nodes()) {
        double z = node.z();
        auto it = std::find_if(stories.begin(), stories.end(), [&](const auto& kv) {
            return std::abs(kv.first - z) < kElevationToleranceM;
        });
        if (it == stories.end()) {
            SeismicStoryForce s;
            s.elevation = z;
            stories[z] = s;
            it = stories.find(z);
        }
        it->second.weight += node.translationalMass() * kGravity;
        it->second.nodeIds.push_back(node.id());
    }
    std::vector<SeismicStoryForce> result;
    result.reserve(stories.size());
    for (auto& kv : stories) result.push_back(kv.second);
    return result;
}
}

SeismicAnalysisResult runEquivalentStaticSeismic(const Model& model, const SeismicParameters& params) {
    SeismicAnalysisResult result;
    result.storyForces = groupByStory(model);
    if (result.storyForces.empty()) throw std::runtime_error("runEquivalentStaticSeismic: model has no nodes");

    double baseElevation = result.storyForces.front().elevation;
    double hn = result.storyForces.back().elevation - baseElevation;
    if (hn <= 0.0) throw std::runtime_error("runEquivalentStaticSeismic: model has only one distinct story elevation");

    result.fundamentalPeriodSeconds = params.overridePeriodSeconds > 0.0
                                           ? params.overridePeriodSeconds
                                           : params.periodCoeffs.Ct * std::pow(hn, params.periodCoeffs.m);

    double eta = dampingCorrectionFactor(params.dampingRatioPercent);
    double Cs = normalizedSpectralShape(result.fundamentalPeriodSeconds, params.site, eta);
    result.designSpectralAcceleration = (2.0 / 3.0) * params.Z * params.I / params.R * Cs;

    result.totalSeismicWeight = 0.0;
    for (const auto& s : result.storyForces) result.totalSeismicWeight += s.weight;
    result.baseShear = result.designSpectralAcceleration * result.totalSeismicWeight;

    double T = result.fundamentalPeriodSeconds;
    double k = T <= 0.5 ? 1.0 : (T >= 2.5 ? 2.0 : 1.0 + (T - 0.5) / 2.0);

    double denom = 0.0;
    for (const auto& s : result.storyForces) {
        double hx = s.elevation - baseElevation;
        denom += s.weight * std::pow(hx, k);
    }
    if (denom <= 0.0) {
        throw std::runtime_error("runEquivalentStaticSeismic: total weight*height^k is zero");
    }
    for (auto& s : result.storyForces) {
        double hx = s.elevation - baseElevation;
        s.force = result.baseShear * (s.weight * std::pow(hx, k)) / denom;
    }
    return result;
}

void applyAsNodalLoads(Model& model, const SeismicAnalysisResult& result, int loadCaseId, bool directionX) {
    for (const auto& s : result.storyForces) {
        if (s.nodeIds.empty() || s.force == 0.0) continue;
        double perNode = s.force / static_cast<double>(s.nodeIds.size());
        for (int nodeId : s.nodeIds) {
            NodalLoad load;
            load.nodeId = nodeId;
            load.loadCaseId = loadCaseId;
            if (directionX) load.Fx = perNode; else load.Fy = perNode;
            model.addNodalLoad(load);
        }
    }
}

}  // namespace nrsa::analysis
