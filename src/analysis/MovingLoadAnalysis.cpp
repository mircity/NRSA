#include "analysis/MovingLoadAnalysis.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nrsa::analysis {

double momentInfluenceOrdinate(double L, double x, double a) {
    if (a < -1e-9 || a > L + 1e-9) return 0.0;  // load off the span contributes nothing
    a = std::clamp(a, 0.0, L);
    if (a <= x) return (L - x) * a / L;
    return x * (L - a) / L;
}

double shearInfluenceOrdinate(double L, double x, double a) {
    if (a < -1e-9 || a > L + 1e-9) return 0.0;
    a = std::clamp(a, 0.0, L);
    if (a < x) return -a / L;
    return (L - a) / L;
}

std::vector<MovingLoadEnvelopePoint> runMovingLoadEnvelope(
    double L, const std::vector<MovingPointLoad>& train, int sectionCount, double step) {
    if (L <= 0.0) throw std::invalid_argument("runMovingLoadEnvelope: span must be positive");
    if (train.empty()) throw std::invalid_argument("runMovingLoadEnvelope: empty load train");
    if (sectionCount < 2) throw std::invalid_argument("runMovingLoadEnvelope: need at least 2 sections");
    if (step <= 0.0) throw std::invalid_argument("runMovingLoadEnvelope: positionStepM must be positive");

    double trainSpan = 0.0;
    for (const auto& p : train) trainSpan = std::max(trainSpan, p.offsetFromFirst);

    std::vector<MovingLoadEnvelopePoint> envelope(static_cast<std::size_t>(sectionCount));
    for (int s = 0; s < sectionCount; ++s) {
        envelope[static_cast<std::size_t>(s)].x = L * static_cast<double>(s) / (sectionCount - 1);
    }

    // Sweep the train's reference-load position from just off the left
    // end to just off the right end, so every axle gets a turn crossing
    // every section.
    for (double refPos = -trainSpan; refPos <= L + trainSpan + 1e-9; refPos += step) {
        for (auto& pt : envelope) {
            double M = 0.0, V = 0.0;
            for (const auto& p : train) {
                double a = refPos + p.offsetFromFirst;
                M += p.magnitude * momentInfluenceOrdinate(L, pt.x, a);
                V += p.magnitude * shearInfluenceOrdinate(L, pt.x, a);
            }
            if (std::abs(M) > std::abs(pt.maxMoment)) {
                pt.maxMoment = M;
                pt.trainPositionAtMaxMoment = refPos;
            }
            if (std::abs(V) > std::abs(pt.maxShear)) {
                pt.maxShear = V;
                pt.trainPositionAtMaxShear = refPos;
            }
        }
    }
    return envelope;
}

}  // namespace nrsa::analysis
