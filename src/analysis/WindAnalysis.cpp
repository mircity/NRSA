#include "analysis/WindAnalysis.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace nrsa::analysis {

namespace {
constexpr double kElevationToleranceM = 0.001;
}

ExposureConstants ExposureConstants::forCategory(ExposureCategory c) {
    switch (c) {
        case ExposureCategory::B: return {7.0, 365.0, 9.1};
        case ExposureCategory::C: return {9.5, 274.0, 4.6};
        case ExposureCategory::D: return {11.5, 213.0, 2.1};
    }
    return {};
}

double velocityPressureExposureCoefficient(double heightM, const ExposureConstants& exposure) {
    double z = std::max(heightM, exposure.zmin);
    return 2.01 * std::pow(z / exposure.zg, 2.0 / exposure.alpha);
}

double velocityPressure(double heightM, const WindParameters& params) {
    ExposureConstants exposure = ExposureConstants::forCategory(params.exposure);
    double Kz = velocityPressureExposureCoefficient(heightM, exposure);
    double V = params.basicWindSpeedMs;
    return 0.613 * Kz * params.topographicFactor * params.directionalityFactor * V * V * params.importanceFactor;
}

namespace {
struct StoryBucket {
    double elevation = 0.0;
    std::vector<int> nodeIds;
};

std::vector<StoryBucket> groupByStory(const Model& model) {
    std::map<double, StoryBucket> stories;
    for (const auto& node : model.nodes()) {
        double z = node.z();
        auto it = std::find_if(stories.begin(), stories.end(), [&](const auto& kv) {
            return std::abs(kv.first - z) < kElevationToleranceM;
        });
        if (it == stories.end()) {
            StoryBucket b;
            b.elevation = z;
            stories[z] = b;
            it = stories.find(z);
        }
        it->second.nodeIds.push_back(node.id());
    }
    std::vector<StoryBucket> result;
    result.reserve(stories.size());
    for (auto& kv : stories) result.push_back(kv.second);
    return result;
}
}

WindAnalysisResult runStaticWind(const Model& model, const WindParameters& params) {
    WindAnalysisResult result;
    auto buckets = groupByStory(model);
    if (buckets.empty()) throw std::runtime_error("runStaticWind: model has no nodes");
    if (buckets.size() < 2) throw std::runtime_error("runStaticWind: model has only one distinct story elevation");

    double netCp = params.windwardCp - params.leewardCp;

    for (std::size_t i = 0; i < buckets.size(); ++i) {
        WindStoryForce s;
        s.elevation = buckets[i].elevation;
        s.nodeIds = buckets[i].nodeIds;
        s.velocityPressurePa = velocityPressure(s.elevation, params);

        double below = (i == 0) ? 0.0 : (s.elevation - buckets[i - 1].elevation) / 2.0;
        double above = (i + 1 == buckets.size()) ? 0.0 : (buckets[i + 1].elevation - s.elevation) / 2.0;
        double tributaryHeight = below + above;

        double pressure = s.velocityPressurePa * params.gustFactor * netCp;
        s.force = pressure * params.tributaryWidthM * tributaryHeight;
        result.totalForce += s.force;
        result.storyForces.push_back(s);
    }
    return result;
}

void applyAsNodalLoads(Model& model, const WindAnalysisResult& result, int loadCaseId, bool directionX) {
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
