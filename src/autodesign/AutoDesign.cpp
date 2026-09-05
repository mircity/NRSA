#include "autodesign/AutoDesign.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>

namespace nrsa::autodesign {

using analysis::FrameEndForces;
using analysis::StaticAnalysis;
using analysis::StaticAnalysisResult;

namespace {

FrameEndForces scaleAndAdd(const FrameEndForces& acc, const FrameEndForces& term, double factor) {
    FrameEndForces r = acc;
    r.axial1 += factor * term.axial1;
    r.shearY1 += factor * term.shearY1;
    r.shearZ1 += factor * term.shearZ1;
    r.torsion1 += factor * term.torsion1;
    r.momentY1 += factor * term.momentY1;
    r.momentZ1 += factor * term.momentZ1;
    r.axial2 += factor * term.axial2;
    r.shearY2 += factor * term.shearY2;
    r.shearZ2 += factor * term.shearZ2;
    r.torsion2 += factor * term.torsion2;
    r.momentY2 += factor * term.momentY2;
    r.momentZ2 += factor * term.momentZ2;
    return r;
}

}  // namespace

std::vector<BeamDesignSummary> runAutoDesignForBeams(Model& model,
                                                       const std::vector<LoadCombination>& combinations,
                                                       double fcMPa, double fyMPa, double coverMm,
                                                       double assumedLongBarDiaMm, double stirrupDiaMm,
                                                       int stirrupLegs) {
    // ---- Step 1: Analysis -- run each distinct referenced load case once ----
    std::set<int> loadCaseIds;
    for (const auto& combo : combinations)
        for (const auto& [caseId, factor] : combo.factors())
            if (factor != 0.0) loadCaseIds.insert(caseId);

    std::unordered_map<int, StaticAnalysisResult> resultsByLoadCase;
    StaticAnalysis analysis(model);
    for (int caseId : loadCaseIds) resultsByLoadCase[caseId] = analysis.run(caseId);

    // ---- Step 2: Load Combination + Member Forces -- superpose per combination ----
    std::vector<int> beamElementIds;
    for (const auto& e : model.elements())
        if (e.kind() == ElementKind::Beam) beamElementIds.push_back(e.id());

    std::unordered_map<int, BeamDesignSummary> summaries;
    for (int id : beamElementIds) {
        BeamDesignSummary s;
        s.elementId = id;
        summaries[id] = s;
    }

    for (const auto& combo : combinations) {
        std::unordered_map<int, FrameEndForces> comboForces;
        for (int id : beamElementIds) comboForces[id] = FrameEndForces{};

        for (const auto& [caseId, factor] : combo.factors()) {
            if (factor == 0.0) continue;
            const auto& caseResult = resultsByLoadCase.at(caseId);
            for (int id : beamElementIds) {
                auto it = caseResult.elementForces.find(id);
                if (it == caseResult.elementForces.end()) continue;  // e.g. skipped element
                comboForces[id] = scaleAndAdd(comboForces[id], it->second, factor);
            }
        }

        // ---- Step 3: envelope Mu/Vu across both ends and all combinations ----
        for (int id : beamElementIds) {
            const auto& f = comboForces[id];
            double momentCandidate = std::max(std::abs(f.momentZ1), std::abs(f.momentZ2));
            double shearCandidate = std::max(std::abs(f.shearY1), std::abs(f.shearY2));

            auto& s = summaries[id];
            if (momentCandidate > s.muKNm) {
                s.muKNm = momentCandidate;
                s.governingMomentCombinationId = combo.id();
            }
            if (shearCandidate > s.vuKN) {
                s.vuKN = shearCandidate;
                s.governingShearCombinationId = combo.id();
            }
        }
    }

    // ---- Step 4: Section Check + Reinforcement Design ----
    std::vector<BeamDesignSummary> results;
    for (int id : beamElementIds) {
        const Element& e = model.element(id);
        const Section& sec = model.section(e.sectionId());
        double bM = sec.width;
        double dM = (sec.depth * 1000.0 - coverMm - assumedLongBarDiaMm / 2.0) / 1000.0;

        auto& s = summaries[id];
        s.flexure = design::designFlexure(s.muKNm, bM, dM, fcMPa, fyMPa);
        s.shear = design::designShear(s.vuKN, bM, dM, fcMPa, fyMPa, stirrupDiaMm, stirrupLegs);
        results.push_back(s);
    }

    return results;
}

std::vector<ColumnDesignSummary> runAutoDesignForColumns(Model& model,
                                                           const std::vector<LoadCombination>& combinations,
                                                           double fcMPa, double fyMPa, double coverMm,
                                                           double barDiaMm, int barsAlongB, int barsAlongH) {
    // ---- Step 1: Analysis -- run each distinct referenced load case once ----
    std::set<int> loadCaseIds;
    for (const auto& combo : combinations)
        for (const auto& [caseId, factor] : combo.factors())
            if (factor != 0.0) loadCaseIds.insert(caseId);

    std::unordered_map<int, StaticAnalysisResult> resultsByLoadCase;
    StaticAnalysis analysis(model);
    for (int caseId : loadCaseIds) resultsByLoadCase[caseId] = analysis.run(caseId);

    std::vector<int> columnElementIds;
    for (const auto& e : model.elements())
        if (e.kind() == ElementKind::Column) columnElementIds.push_back(e.id());

    std::unordered_map<int, ColumnDesignSummary> summaries;
    for (int id : columnElementIds) {
        ColumnDesignSummary s;
        s.elementId = id;
        s.demandCapacityRatio = -1.0;  // sentinel: no case evaluated yet
        summaries[id] = s;
    }

    // ---- Step 2/3: Load Combination + Member Forces, checked TOGETHER
    // (Pu,Mux,Muy) per (combination, end) -- see class doc comment for
    // why this is NOT a per-component envelope. ----
    for (const auto& combo : combinations) {
        std::unordered_map<int, FrameEndForces> comboForces;
        for (int id : columnElementIds) comboForces[id] = FrameEndForces{};

        for (const auto& [caseId, factor] : combo.factors()) {
            if (factor == 0.0) continue;
            const auto& caseResult = resultsByLoadCase.at(caseId);
            for (int id : columnElementIds) {
                auto it = caseResult.elementForces.find(id);
                if (it == caseResult.elementForces.end()) continue;
                comboForces[id] = scaleAndAdd(comboForces[id], it->second, factor);
            }
        }

        for (int id : columnElementIds) {
            const auto& f = comboForces[id];
            const Element& e = model.element(id);
            const Section& sec = model.section(e.sectionId());

            for (int end = 1; end <= 2; ++end) {
                double puKN = std::abs(end == 1 ? f.axial1 : f.axial2);
                // FIXED 2026-08-30 (was backwards -- a real bug found by
                // tracing Section::rectangular's own Iz/Iy definitions):
                // Section::rectangular(b,h) sets momentOfInertiaZ=b*h^3/12
                // (STRONG axis, using the full depth h) and
                // momentOfInertiaY=h*b^3/12 (weak axis, using width b).
                // FrameEndForces' momentZ pairs with Iz (confirmed
                // separately: a horizontal beam's gravity bending, which
                // uses the section's strong axis by convention, appears in
                // momentZ/shearY, matching test_static_analysis.cpp's own
                // established pairing). design::designBiaxialColumn's "Mx"
                // is explicitly documented (RCCColumn.cpp) as the
                // strong-axis case (Whitney block depth = hM, the larger
                // dimension) -- so Mx <-> momentZ, My <-> momentY. The
                // previous version of this file had these swapped, which
                // would have silently mis-paired moment-to-axis for any
                // RECTANGULAR (non-square) column -- invisible in every
                // test so far because they all used SQUARE columns, where
                // swapping the two axes changes nothing. Fixed here, and
                // tests/test_auto_design.cpp now includes a genuinely
                // RECTANGULAR column case that would have failed under
                // the old (buggy) assignment.
                double muxKNm = std::abs(end == 1 ? f.momentZ1 : f.momentZ2);
                double muyKNm = std::abs(end == 1 ? f.momentY1 : f.momentY2);

                auto biaxial = design::designBiaxialColumn(sec.width, sec.depth, coverMm, barDiaMm,
                                                             barsAlongB, barsAlongH, fcMPa, fyMPa,
                                                             puKN, muxKNm, muyKNm);
                double ratio = biaxial.phiPnKN > 0.0 ? puKN / biaxial.phiPnKN : 1e9;

                auto& s = summaries[id];
                if (ratio > s.demandCapacityRatio) {
                    s.puKN = puKN;
                    s.muxKNm = muxKNm;
                    s.muyKNm = muyKNm;
                    s.governingCombinationId = combo.id();
                    s.governingEnd = end;
                    s.demandCapacityRatio = ratio;
                    s.biaxial = biaxial;
                }
            }
        }
    }

    std::vector<ColumnDesignSummary> results;
    for (int id : columnElementIds) results.push_back(summaries[id]);
    return results;
}

std::vector<FoundationDesignSummary> runAutoDesignForFoundations(
    Model& model, const std::vector<ColumnDesignSummary>& columnResults, double netAllowableBearingKPa,
    double fcMPa, double fyMPa, double coverMm, double barDiaMm, double assumedAverageLoadFactor) {
    std::vector<FoundationDesignSummary> results;

    for (const auto& col : columnResults) {
        const Element& e = model.element(col.elementId);
        if (e.nodeCount() != 2) continue;

        // Find which of the column's two end nodes is the actual
        // support (fully translationally restrained) -- that's where a
        // footing belongs, not the free/connected top end.
        int baseNodeId = -1;
        for (std::size_t i = 0; i < e.nodeCount(); ++i) {
            const Node& n = model.node(e.nodeId(i));
            if (n.isRestrained(DOF::Ux) && n.isRestrained(DOF::Uy) && n.isRestrained(DOF::Uz)) {
                baseNodeId = e.nodeId(i);
                break;
            }
        }
        if (baseNodeId < 0) continue;  // no restrained end -- not a ground-bearing column

        const Section& sec = model.section(e.sectionId());

        FoundationDesignSummary fs;
        fs.columnElementId = col.elementId;
        fs.baseNodeId = baseNodeId;
        fs.puKN = col.puKN;
        fs.approximateServiceLoadKN = col.puKN / assumedAverageLoadFactor;
        fs.footing = design::designIsolatedFooting(col.puKN, fs.approximateServiceLoadKN,
                                                     netAllowableBearingKPa, sec.width, sec.depth,
                                                     coverMm, barDiaMm, fcMPa, fyMPa);
        results.push_back(fs);
    }

    return results;
}

}  // namespace nrsa::autodesign
