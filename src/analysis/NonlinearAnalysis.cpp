#include "analysis/NonlinearAnalysis.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

#include "analysis/GlobalAssembly.h"
#include "fem/Solver.h"

namespace nrsa::analysis {

NonlinearAnalysis::Result NonlinearAnalysis::run(int loadCaseId, Options opts) {
    int freeDofCount = model_.assignDofNumbers();
    if (freeDofCount == 0) {
        throw std::runtime_error(
            "NonlinearAnalysis::run: model has zero free DOFs -- every node is fully restrained, "
            "so there is nothing to solve.");
    }

    BuiltElements built = buildElements(model_);

    // Current axial-force estimate per frame element (tension-positive
    // -- what fem::FrameElement3D::geometricStiffnessGlobal expects),
    // starting from zero: the first iteration is therefore an ordinary
    // linear-elastic solve with no geometric stiffness at all, and the
    // axial forces it recovers become the estimate the second iteration
    // uses.
    std::unordered_map<int, double> axialTensionPositive;
    for (const auto& b : built.frames) axialTensionPositive[b.elementId] = 0.0;

    auto frameExtra = [&axialTensionPositive](const BuiltFrameElement& b) {
        auto it = axialTensionPositive.find(b.elementId);
        double P = (it != axialTensionPositive.end()) ? it->second : 0.0;
        return b.frame.geometricStiffnessGlobal(P);
    };

    std::vector<double> uFree(static_cast<std::size_t>(freeDofCount), 0.0);
    double relativeChange = 0.0;
    int iterationsUsed = 0;
    bool converged = false;

    for (int iter = 0; iter < opts.maxIterations; ++iter) {
        fem::SparseMatrix K(static_cast<std::size_t>(freeDofCount));
        assembleStiffness(built, K, frameExtra);

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

        fem::Vector uFreeVec = fem::Solver::solve(K, F).x;
        for (std::size_t i = 0; i < uFreeVec.size(); ++i) uFree[i] = uFreeVec(i);

        auto endForces = recoverFrameEndForces(built, uFree);
        double maxAbsChange = 0.0, maxAbsScale = 1.0;
        std::unordered_map<int, double> newAxial;
        for (const auto& b : built.frames) {
            double newP = -endForces[b.elementId].axial1;  // compression-positive -> tension-positive
            double oldP = axialTensionPositive[b.elementId];
            maxAbsChange = std::max(maxAbsChange, std::abs(newP - oldP));
            maxAbsScale = std::max(maxAbsScale, std::abs(newP));
            newAxial[b.elementId] = newP;
        }
        axialTensionPositive = std::move(newAxial);
        relativeChange = maxAbsChange / maxAbsScale;
        iterationsUsed = iter + 1;

        if (relativeChange <= opts.convergenceTolerance) {
            converged = true;
            break;
        }
    }

    if (!converged) {
        throw std::runtime_error(
            "NonlinearAnalysis::run: geometric self-consistency did not converge within " +
            std::to_string(opts.maxIterations) + " iterations (final relative axial-force change " +
            std::to_string(relativeChange) + ") -- the load may be at or beyond the structure's "
            "buckling capacity; see the class doc comment on this method's convergence limits.");
    }

    // ---- Build the final StaticAnalysisResult from the converged
    // state, using the SAME recovery helpers PDeltaAnalysis uses (local
    // end forces from the elastic stiffness only; reactions/residual
    // from the converged Ke+Kg accumulation) -- see PDeltaAnalysis's own
    // doc comment for why local end forces don't get a separate
    // "geometric" component of their own.
    Result result;
    result.staticResult.skippedElementIds = built.skippedElementIds;
    for (const auto& n : model_.nodes()) {
        NodeVector6 v;
        auto d = n.dofIndices();
        for (int i = 0; i < 6; ++i) {
            if (d[i] >= 0) v.set(static_cast<DOF>(i), uFree[static_cast<std::size_t>(d[i])]);
        }
        result.staticResult.displacements[n.id()] = v;
    }
    result.staticResult.elementForces = recoverFrameEndForces(built, uFree);
    auto nodalInternalForce = accumulateNodalInternalForces(model_, built, uFree, frameExtra);
    result.staticResult.reactions = buildReactionsAndResidual(
        model_, nodalInternalForce, loadCaseId, result.staticResult.maxFreeDofResidualNorm);

    result.iterations = iterationsUsed;
    result.finalRelativeChange = relativeChange;
    return result;
}

}  // namespace nrsa::analysis
