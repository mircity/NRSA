#include "verification/IndependentVerification.h"

#include <map>
#include <cmath>
#include <sstream>

namespace nrsa::verification {

ColumnVerificationResult verifyColumnAxialCapacity(double widthM, double depthM, double coverMm,
                                                     double barDiaMm, int barsAlongB, int barsAlongH,
                                                     double fcMPa, double fyMPa) {
    ColumnVerificationResult r;

    auto layout = design::generateRectangularLayout(widthM, depthM, coverMm, barDiaMm, barsAlongB, barsAlongH);
    double astMm2 = static_cast<double>(layout.barsXY.size()) * layout.barAreaMm2;

    // Path 1: the closed-form formula (takes mm, not m).
    r.poFromClosedForm = design::axialCapacityPo(widthM * 1000.0, depthM * 1000.0, astMm2, fcMPa, fyMPa);

    // Path 2: independently, the interaction diagram's own sweep,
    // reading off its LAST point (largest neutral-axis depth) -- a
    // completely different code path (strain compatibility + Whitney
    // block integration over every rebar layer) that happens to
    // converge to the same physical quantity, per the algebraic
    // identity this project's own tests already prove by hand.
    std::map<long long, double> layerKeyToArea;
    for (const auto& [x, y] : layout.barsXY) layerKeyToArea[std::llround(y * 100.0)] += layout.barAreaMm2;
    std::vector<design::RebarLayer> layers;
    for (const auto& [key, area] : layerKeyToArea) layers.push_back({static_cast<double>(key) / 100.0, area});

    auto diagram = design::computeUniaxialInteractionDiagram(widthM * 1000.0, depthM * 1000.0, layers,
                                                               fcMPa, fyMPa, 20);
    r.poFromInteractionDiagram = diagram.back().pnKN;

    double denom = std::max(1e-9, std::abs(r.poFromClosedForm));
    r.percentDifference = std::abs(r.poFromClosedForm - r.poFromInteractionDiagram) / denom * 100.0;

    std::ostringstream note;
    note.precision(4);
    note << "Po(closed-form)=" << r.poFromClosedForm << " kN, Po(diagram convergence)="
         << r.poFromInteractionDiagram << " kN, difference=" << r.percentDifference << "%";
    r.note = note.str();

    if (r.percentDifference <= 0.5) r.verdict = VerificationVerdict::Pass;
    else if (r.percentDifference <= 5.0) r.verdict = VerificationVerdict::Warning;
    else r.verdict = VerificationVerdict::Fail;

    return r;
}

}  // namespace nrsa::verification
