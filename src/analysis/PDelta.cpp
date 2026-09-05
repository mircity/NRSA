#include "analysis/PDelta.h"

#include <stdexcept>

#include "analysis/GlobalAssembly.h"
#include "fem/Solver.h"

namespace nrsa::analysis {

StaticAnalysisResult PDeltaAnalysis::run(int axialLoadCaseId, int lateralLoadCaseId) {
    // ---- Pass 1: ordinary static solve under the axial (typically
    // gravity) load case, purely to recover each frame element's axial
    // force. Reuses StaticAnalysis outright rather than duplicating ITS
    // logic -- only the augmented second pass below needs the extra
    // geometric-stiffness assembly.
    StaticAnalysis gravityPass(model_);
    auto gravityResult = gravityPass.run(axialLoadCaseId);

    std::unordered_map<int, double> axialTensionPositive;  // elementId -> P (tension +)
    for (const auto& kv : gravityResult.elementForces) {
        int elementId = kv.first;
        const FrameEndForces& forces = kv.second;
        // axial1 == -axial2 exactly, by construction of the axial
        // stiffness sub-block (f0 = kAxial*(u0-u6), f6 = -f0) -- no
        // averaging needed, just a direct sign-convention flip: axial1
        // is documented as positive = compression, so P (tension-
        // positive, what geometricStiffnessGlobal expects) is its
        // negation.
        axialTensionPositive[elementId] = -forces.axial1;
    }

    // ---- Pass 2: re-assemble Ke + Kg (frame elements only -- see class
    // doc comment on Wall/Slab not yet contributing geometric
    // stiffness) and solve the LATERAL load case against it.
    int freeDofCount = model_.assignDofNumbers();
    if (freeDofCount == 0) {
        throw std::runtime_error(
            "PDeltaAnalysis::run: model has zero free DOFs -- every node is fully restrained, "
            "so there is nothing to solve.");
    }

    StaticAnalysisResult result;
    BuiltElements built = buildElements(model_);
    result.skippedElementIds = built.skippedElementIds;

    // The geometric-stiffness hook: for each frame element, look up its
    // axial force from pass 1 (0 if somehow absent) and return its
    // geometricStiffnessGlobal(P) as the "extra" term assembleStiffness
    // adds alongside the elastic globalStiffness().
    auto frameExtra = [&axialTensionPositive](const BuiltFrameElement& b) {
        auto it = axialTensionPositive.find(b.elementId);
        double P = (it != axialTensionPositive.end()) ? it->second : 0.0;
        return b.frame.geometricStiffnessGlobal(P);
    };

    fem::SparseMatrix K(static_cast<std::size_t>(freeDofCount));
    assembleStiffness(built, K, frameExtra);

    fem::Vector F(static_cast<std::size_t>(freeDofCount), 0.0);
    for (const auto& load : model_.nodalLoads()) {
        if (load.loadCaseId != lateralLoadCaseId) continue;
        const Node& n = model_.node(load.nodeId);
        auto d = n.dofIndices();
        double comps[6] = {load.Fx, load.Fy, load.Fz, load.Mx, load.My, load.Mz};
        for (int i = 0; i < 6; ++i) {
            if (d[i] >= 0) F(static_cast<std::size_t>(d[i])) += comps[i];
        }
    }

    // Solver::solve throws if the AUGMENTED system is not positive-
    // definite -- meaningfully different from StaticAnalysis's own
    // "unrestrained mechanism" case: here it can also mean the gravity
    // load alone has pushed some member(s) into (or past) their
    // buckling load, a genuine structural result worth surfacing, not
    // a bug to work around.
    fem::Vector uFreeVec = fem::Solver::solve(K, F).x;
    std::vector<double> uFree(uFreeVec.size());
    for (std::size_t i = 0; i < uFreeVec.size(); ++i) uFree[i] = uFreeVec(i);

    for (const auto& n : model_.nodes()) {
        NodeVector6 v;
        auto d = n.dofIndices();
        for (int i = 0; i < 6; ++i) {
            if (d[i] >= 0) v.set(static_cast<DOF>(i), uFree[static_cast<std::size_t>(d[i])]);
        }
        result.displacements[n.id()] = v;
    }

    // Local end forces reported here are from the ELASTIC local
    // stiffness only (not elastic+geometric) -- see
    // recoverFrameEndForces's own doc comment for why: the physically
    // meaningful axial/shear/moment a design check wants is the real
    // internal stress resultant; the geometric stiffness term exists
    // to capture the P-Delta softening effect on the GLOBAL
    // equilibrium/displacement solution, not as a separate internal
    // force component of its own.
    result.elementForces = recoverFrameEndForces(built, uFree);
    auto nodalInternalForce = accumulateNodalInternalForces(model_, built, uFree, frameExtra);
    result.reactions = buildReactionsAndResidual(model_, nodalInternalForce, lateralLoadCaseId,
                                                  result.maxFreeDofResidualNorm);

    return result;
}

}  // namespace nrsa::analysis
