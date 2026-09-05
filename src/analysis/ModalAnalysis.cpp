#include "analysis/ModalAnalysis.h"

#include <stdexcept>

#include "analysis/GlobalAssembly.h"

namespace nrsa::analysis {

std::unordered_map<int, NodeVector6> expandModeShape(const Model& model, const fem::ModeShape& mode) {
    std::unordered_map<int, NodeVector6> out;
    for (const auto& n : model.nodes()) {
        NodeVector6 v;
        auto d = n.dofIndices();
        for (int i = 0; i < 6; ++i) {
            if (d[i] >= 0) v.set(static_cast<DOF>(i), mode.shape(static_cast<std::size_t>(d[i])));
        }
        out[n.id()] = v;
    }
    return out;
}

ModalAnalysisResult ModalAnalysis::run(Options opts) {
    ModalAnalysisResult result;

    int freeDofCount = model_.assignDofNumbers();
    if (freeDofCount == 0) {
        throw std::runtime_error(
            "ModalAnalysis::run: model has zero free DOFs -- every node is fully restrained, "
            "so there is nothing to analyze.");
    }

    BuiltElements built = buildElements(model_);
    result.skippedElementIds = built.skippedElementIds;

    fem::SparseMatrix K(static_cast<std::size_t>(freeDofCount));
    assembleStiffness(built, K);

    // ---- Build the lumped mass vector (see class doc comment for the
    // rotational-mass regularization rationale).
    std::vector<double> mass(static_cast<std::size_t>(freeDofCount), 0.0);
    double sumTransMass = 0.0;
    int nTransMassed = 0;
    for (const auto& n : model_.nodes()) {
        auto d = n.dofIndices();
        double m = n.translationalMass();
        auto rm = n.rotationalMass();
        double comps[6] = {m, m, m, rm[0], rm[1], rm[2]};
        for (int i = 0; i < 6; ++i) {
            if (d[i] >= 0) mass[static_cast<std::size_t>(d[i])] = comps[i];
        }
        if (m > 0.0) { sumTransMass += m; ++nTransMassed; }
    }
    if (nTransMassed == 0) {
        throw std::runtime_error(
            "ModalAnalysis::run: no node in this model has positive translational mass -- "
            "assign mass (core::Node::setTranslationalMass), typically at floor/story level, "
            "before running a modal analysis.");
    }
    double avgTransMass = sumTransMass / nTransMassed;
    double regMass = opts.rotationalMassRegularization * avgTransMass;
    for (auto& m : mass) {
        if (m <= 0.0) m = regMass;
    }

    // EigenSolver::solve requires a dense Matrix, but K here is a
    // SparseMatrix (the whole point of using sparse assembly for the
    // stiffness matrix itself). Convert once -- this dense conversion's
    // cost is exactly what fem::EigenSolver's own doc comment already
    // names as its deliberate, temporary scope limit.
    fem::Matrix Kdense(static_cast<std::size_t>(freeDofCount), static_cast<std::size_t>(freeDofCount), 0.0);
    for (std::size_t i = 0; i < static_cast<std::size_t>(freeDofCount); ++i) {
        for (std::size_t j = i; j < static_cast<std::size_t>(freeDofCount); ++j) {
            double v = K.get(i, j);
            if (v != 0.0) { Kdense(i, j) = v; Kdense(j, i) = v; }
        }
    }
    result.modes = fem::EigenSolver::solve(Kdense, mass, opts.eigenOptions);
    return result;
}

}  // namespace nrsa::analysis
