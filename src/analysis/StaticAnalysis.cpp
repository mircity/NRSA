#include "analysis/StaticAnalysis.h"

#include <stdexcept>

#include "analysis/GlobalAssembly.h"
#include "fem/Solver.h"

namespace nrsa::analysis {

StaticAnalysisResult StaticAnalysis::run(int loadCaseId) {
    StaticAnalysisResult result;

    int freeDofCount = model_.assignDofNumbers();
    if (freeDofCount == 0) {
        throw std::runtime_error(
            "StaticAnalysis::run: model has zero free DOFs -- every node is fully restrained, "
            "so there is nothing to solve.");
    }

    BuiltElements built = buildElements(model_);
    result.skippedElementIds = built.skippedElementIds;

    // ---- Assemble the global (free-DOF-only) stiffness matrix and
    // load vector -- sparse assembly (see fem::SparseMatrix::add's own
    // doc comment on the upper-triangle-only scatter). This is what
    // actually removes the O(n^2) memory / O(n^3) solve ceiling the
    // dense fem::Matrix path had -- see the README roadmap note on why
    // PCG rather than a dense direct solve.
    fem::SparseMatrix K(static_cast<std::size_t>(freeDofCount));
    assembleStiffness(built, K);

    fem::Vector F(static_cast<std::size_t>(freeDofCount), 0.0);
    for (const auto& load : model_.nodalLoads()) {
        if (load.loadCaseId != loadCaseId) continue;
        const Node& n = model_.node(load.nodeId);
        auto d = n.dofIndices();
        double comps[6] = {load.Fx, load.Fy, load.Fz, load.Mx, load.My, load.Mz};
        for (int i = 0; i < 6; ++i) {
            if (d[i] >= 0) F(static_cast<std::size_t>(d[i])) += comps[i];
        }
    }

    // ---- Solve. fem::Solver::solve throws on a non-positive-definite
    // system (see its own doc comment) -- most commonly an unrestrained
    // mechanism: a node, or a whole disconnected sub-structure, with no
    // stiffness path to a support. That thrown error is deliberately
    // allowed to propagate rather than being caught here, since
    // StaticAnalysis has no way to know what the right recovery is --
    // the caller does.
    fem::Vector uFreeVec = fem::Solver::solve(K, F).x;
    std::vector<double> uFree(uFreeVec.size());
    for (std::size_t i = 0; i < uFreeVec.size(); ++i) uFree[i] = uFreeVec(i);

    // ---- Recover full per-node global displacement vectors (0 at every
    // restrained DOF -- no support settlement modeled in this pass; see
    // the class-level doc comment).
    for (const auto& n : model_.nodes()) {
        NodeVector6 v;
        auto d = n.dofIndices();
        for (int i = 0; i < 6; ++i) {
            if (d[i] >= 0) v.set(static_cast<DOF>(i), uFree[static_cast<std::size_t>(d[i])]);
        }
        result.displacements[n.id()] = v;
    }

    // ---- Per-element local end forces (frame elements only) and the
    // per-node internal-force accumulation used to recover reactions/
    // residuals -- both via the shared GlobalAssembly helpers.
    result.elementForces = recoverFrameEndForces(built, uFree);
    auto nodalInternalForce = accumulateNodalInternalForces(model_, built, uFree);
    result.reactions =
        buildReactionsAndResidual(model_, nodalInternalForce, loadCaseId, result.maxFreeDofResidualNorm);

    return result;
}

}  // namespace nrsa::analysis
