#include "assistant/DesignExplainer.h"

#include <cmath>
#include <sstream>

namespace nrsa::assistant {

ColumnFailureExplanation explainColumnResult(const std::string& columnLabel, double puKN,
                                              const design::BiaxialDesignResult& result,
                                              double currentSteelRatio, double widthM, double depthM) {
    ColumnFailureExplanation exp;
    exp.interactionRatio = result.phiPnKN > 0.0 ? puKN / result.phiPnKN : 1e9;
    exp.fails = puKN > result.phiPnKN;

    std::ostringstream headline;
    headline.precision(3);
    if (exp.fails) {
        headline << columnLabel << " fails - interaction ratio = " << exp.interactionRatio;
    } else {
        headline << columnLabel << " OK - interaction ratio = " << exp.interactionRatio;
    }
    exp.headline = headline.str();

    if (!exp.fails) return exp;  // nothing to suggest fixing

    // Suggestion 1: size increase. First-order approximation only --
    // nominal axial capacity scales roughly with gross area (Ag) for a
    // given reinforcement ratio and stress state, so scaling both
    // in-plane dimensions by sqrt(interactionRatio) scales Ag by
    // exactly interactionRatio, a reasonable first pass at "how much
    // bigger" -- NOT a re-run of the actual design check. Rounded up to
    // the nearest 25mm, the common practical sizing increment.
    double scaleFactor = std::sqrt(exp.interactionRatio);
    double newWidthMm = std::ceil((widthM * 1000.0 * scaleFactor) / 25.0) * 25.0;
    double newDepthMm = std::ceil((depthM * 1000.0 * scaleFactor) / 25.0) * 25.0;
    std::ostringstream sizeFix;
    sizeFix.precision(0);
    sizeFix << std::fixed << "Increase column size to " << newWidthMm << "x" << newDepthMm
            << " mm (approximate -- re-check with design::designBiaxialColumn before finalizing)";
    exp.suggestedFixes.push_back(sizeFix.str());

    // Suggestion 2: reinforcement increase. Same first-order proportional
    // scaling, capped at a practical 4% (below ACI 318-19's absolute
    // 8% maximum, but a commonly-used practical ceiling before a size
    // increase is preferred instead).
    double suggestedRatio = std::min(currentSteelRatio * exp.interactionRatio, 0.04);
    if (suggestedRatio > currentSteelRatio) {
        std::ostringstream steelFix;
        steelFix.precision(1);
        steelFix << std::fixed << "Increase longitudinal reinforcement from "
                 << (currentSteelRatio * 100.0) << "% to " << (suggestedRatio * 100.0) << "%";
        exp.suggestedFixes.push_back(steelFix.str());
    }

    return exp;
}

BeamShearExplanation explainBeamShearResult(const std::string& beamLabel, double vuKN, double phiVnKN,
                                             double currentSpacingMm) {
    BeamShearExplanation exp;
    exp.demandCapacityRatio = phiVnKN > 0.0 ? vuKN / phiVnKN : 1e9;
    exp.fails = vuKN > phiVnKN;

    std::ostringstream headline;
    headline.precision(3);
    headline << beamLabel << (exp.fails ? " fails in shear - ratio = " : " OK in shear - ratio = ")
              << exp.demandCapacityRatio;
    exp.headline = headline.str();

    if (!exp.fails) return exp;

    // Suggestion: reduce stirrup spacing proportionally (Vs, and hence
    // Vn, scales roughly with 1/spacing for a fixed stirrup bar) --
    // same documented first-order-approximation caveat as
    // explainColumnResult's size/steel suggestions.
    double newSpacingMm = currentSpacingMm / exp.demandCapacityRatio;
    std::ostringstream fix;
    fix.precision(0);
    fix << std::fixed << "Reduce stirrup spacing from " << currentSpacingMm << " mm to approximately "
        << newSpacingMm << " mm (approximate -- re-check with design::designShear before finalizing)";
    exp.suggestedFixes.push_back(fix.str());

    return exp;
}

}  // namespace nrsa::assistant
