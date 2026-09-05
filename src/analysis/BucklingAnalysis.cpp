#include "analysis/BucklingAnalysis.h"

#include <algorithm>
#include <stdexcept>
#include <unordered_map>

#include "analysis/GlobalAssembly.h"
#include "fem/EigenSolver.h"
#include "fem/SparseMatrix.h"

namespace nrsa::analysis {

namespace {

fem::Matrix toDense(const fem::SparseMatrix& K) {
    std::size_t n = K.size();
    fem::Matrix dense(n, n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i; j < n; ++j) {
            double v = K.get(i, j);
            if (v == 0.0) continue;
            dense(i, j) = v;
            dense(j, i) = v;
        }
    }
    return dense;
}

}  // namespace

BucklingAnalysisResult BucklingAnalysis::run(int referenceLoadCaseId, Options opts) {
    StaticAnalysis referencePass(model_);
    auto referenceResult = referencePass.run(referenceLoadCaseId);

    std::unordered_map<int, double> axialTensionPositive;
    for (const auto& kv : referenceResult.elementForces) {
        axialTensionPositive[kv.first] = -kv.second.axial1;
    }

    int freeDofCount = model_.assignDofNumbers();
    if (freeDofCount == 0) {
        throw std::runtime_error(
            "BucklingAnalysis::run: model has zero free DOFs -- every node is fully restrained, "
            "so there is nothing to solve.");
    }

    BuiltElements built = buildElements(model_);

    auto frameExtra = [&axialTensionPositive](const BuiltFrameElement& b) {
        auto it = axialTensionPositive.find(b.elementId);
        double P = (it != axialTensionPositive.end()) ? it->second : 0.0;
        return b.frame.geometricStiffnessGlobal(P);
    };

    fem::SparseMatrix KeSparse(static_cast<std::size_t>(freeDofCount));
    assembleStiffness(built, KeSparse, nullptr);

    fem::SparseMatrix KePlusKgSparse(static_cast<std::size_t>(freeDofCount));
    assembleStiffness(built, KePlusKgSparse, frameExtra);

    fem::Matrix Ke = toDense(KeSparse);
    fem::Matrix KePlusKg = toDense(KePlusKgSparse);
    fem::Matrix Kg = KePlusKg - Ke;

    fem::Matrix L = Ke.choleskyLower();
    fem::Matrix Linv = L.inverseLowerTriangular();
    fem::Matrix C = Linv * Kg * Linv.transpose();
    for (std::size_t i = 0; i < C.rows(); ++i) {
        for (std::size_t j = i + 1; j < C.cols(); ++j) {
            double avg = 0.5 * (C(i, j) + C(j, i));
            C(i, j) = avg;
            C(j, i) = avg;
        }
    }

    auto eig = fem::EigenSolver::solveSymmetric(C, opts.eigenOptions);

    BucklingAnalysisResult result;
    result.skippedElementIds = built.skippedElementIds;

    constexpr double kNegativeEps = 1e-9;
    for (std::size_t k = 0; k < eig.eigenvalues.size(); ++k) {
        double gamma = eig.eigenvalues[k];
        if (gamma >= -kNegativeEps) continue;
        double lambda = -1.0 / gamma;

        std::vector<double> psi(eig.eigenvectors.rows());
        for (std::size_t i = 0; i < psi.size(); ++i) psi[i] = eig.eigenvectors(i, k);
        fem::Matrix LinvT = Linv.transpose();
        std::vector<double> phi(psi.size(), 0.0);
        for (std::size_t i = 0; i < phi.size(); ++i) {
            double sum = 0.0;
            for (std::size_t j = 0; j < psi.size(); ++j) sum += LinvT(i, j) * psi[j];
            phi[i] = sum;
        }

        result.modes.push_back(BucklingMode{lambda, std::move(phi)});
        if (opts.modesRequested > 0 &&
            static_cast<int>(result.modes.size()) >= opts.modesRequested) {
            break;
        }
    }

    return result;
}

}  // namespace nrsa::analysis
