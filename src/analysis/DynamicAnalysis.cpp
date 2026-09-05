#include "analysis/DynamicAnalysis.h"

#include <cmath>
#include <stdexcept>

#include "analysis/GlobalAssembly.h"
#include "fem/Solver.h"

namespace nrsa::analysis {

RayleighCoefficients rayleighCoefficients(double omega1, double omega2, double dampingRatio) {
    if (omega1 <= 0.0 || omega2 <= 0.0 || std::abs(omega1 - omega2) < 1e-9) {
        throw std::invalid_argument(
            "rayleighCoefficients: omega1 and omega2 must be positive and distinct");
    }
    // Solve [1/(2w1) w1/2; 1/(2w2) w2/2] * [a;b] = [xi;xi] via Cramer's rule.
    double a11 = 1.0 / (2.0 * omega1), a12 = omega1 / 2.0;
    double a21 = 1.0 / (2.0 * omega2), a22 = omega2 / 2.0;
    double det = a11 * a22 - a12 * a21;
    double alpha = (dampingRatio * a22 - a12 * dampingRatio) / det;
    double beta = (a11 * dampingRatio - dampingRatio * a21) / det;
    return {alpha, beta};
}

namespace {

std::vector<double> buildLumpedMassVector(const Model& model, int freeDofCount, double regFactor) {
    std::vector<double> mass(static_cast<std::size_t>(freeDofCount), 0.0);
    double sumTransMass = 0.0;
    int nTransMassed = 0;
    for (const auto& n : model.nodes()) {
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
            "runNewmarkBeta: no node in this model has positive translational mass -- assign "
            "mass (core::Node::setTranslationalMass) before running a dynamic analysis.");
    }
    double avgTransMass = sumTransMass / nTransMassed;
    double regMass = regFactor * avgTransMass;
    for (auto& m : mass) if (m <= 0.0) m = regMass;
    return mass;
}

std::unordered_map<int, NodeVector6> expandFreeDofVector(const Model& model,
                                                           const std::vector<double>& vec) {
    std::unordered_map<int, NodeVector6> out;
    for (const auto& n : model.nodes()) {
        NodeVector6 v;
        auto d = n.dofIndices();
        for (int i = 0; i < 6; ++i) {
            if (d[i] >= 0) v.set(static_cast<DOF>(i), vec[static_cast<std::size_t>(d[i])]);
        }
        out[n.id()] = v;
    }
    return out;
}

}  // namespace

DynamicAnalysisResult runNewmarkBeta(Model& model, const ForcingFunction& forcing,
                                      const DynamicAnalysisOptions& options) {
    DynamicAnalysisResult result;

    int freeDofCount = model.assignDofNumbers();
    if (freeDofCount == 0) {
        throw std::runtime_error(
            "runNewmarkBeta: model has zero free DOFs -- every node is fully restrained, "
            "so there is nothing to integrate.");
    }

    BuiltElements built = buildElements(model);
    result.skippedElementIds = built.skippedElementIds;

    fem::SparseMatrix K(static_cast<std::size_t>(freeDofCount));
    assembleStiffness(built, K);

    std::vector<double> m =
        buildLumpedMassVector(model, freeDofCount, options.rotationalMassRegularization);

    // Newmark constant-average-acceleration constants (beta=1/4, gamma=1/2).
    const double beta = 0.25, gamma = 0.5;
    double dt = options.dtSeconds;
    if (dt <= 0.0) throw std::invalid_argument("runNewmarkBeta: dtSeconds must be positive");
    double a0 = 1.0 / (beta * dt * dt);
    double a1 = gamma / (beta * dt);
    double a2 = 1.0 / (beta * dt);
    double a3 = 1.0 / (2.0 * beta) - 1.0;
    double a4 = gamma / beta - 1.0;
    double a5 = (dt / 2.0) * (gamma / beta - 2.0);

    double alphaM = options.damping.alphaMass;
    double betaK = options.damping.betaStiffness;

    // K_eff = (1 + a1*betaK)*K + (a0 + a1*alphaM)*diag(m) -- built ONCE,
    // reused for every timestep (Rayleigh damping + Newmark's own
    // constants make the effective stiffness time-invariant as long as
    // dt is constant, which it is here).
    fem::SparseMatrix Keff = K.scaledCopy(1.0 + a1 * betaK);
    double diagScale = a0 + a1 * alphaM;
    for (std::size_t i = 0; i < static_cast<std::size_t>(freeDofCount); ++i) {
        Keff.add(i, i, diagScale * m[i]);
    }

    std::size_t n = static_cast<std::size_t>(freeDofCount);
    fem::Vector d(n, 0.0), v(n, 0.0), acc(n, 0.0);

    int steps = static_cast<int>(std::ceil(options.totalDurationSeconds / dt));

    // t = 0 initial state.
    {
        DynamicTimeStep step0;
        step0.time = 0.0;
        std::vector<double> zero(n, 0.0);
        step0.displacement = expandFreeDofVector(model, zero);
        step0.velocity = expandFreeDofVector(model, zero);
        step0.acceleration = expandFreeDofVector(model, zero);
        result.steps.push_back(std::move(step0));
    }

    for (int stepIdx = 1; stepIdx <= steps; ++stepIdx) {
        double t = stepIdx * dt;

        std::vector<double> Fnext = forcing(t, freeDofCount);
        if (Fnext.size() != n) {
            throw std::invalid_argument("runNewmarkBeta: forcing function returned wrong size");
        }
        fem::Vector F(n, 0.0);
        for (std::size_t i = 0; i < n; ++i) F(i) = Fnext[i];

        // Effective force: F_eff = F + M*(a0*d + a2*v + a3*acc)
        //                          + C*(a1*d + a4*v + a5*acc)
        // with C = alphaM*M + betaK*K, M diagonal.
        fem::Vector cInput(n, 0.0);
        fem::Vector mTerm(n, 0.0);
        for (std::size_t i = 0; i < n; ++i) {
            cInput(i) = a1 * d(i) + a4 * v(i) + a5 * acc(i);
            mTerm(i) = m[i] * (a0 * d(i) + a2 * v(i) + a3 * acc(i));
        }
        fem::Vector cTermFromK = K.multiply(cInput) * betaK;
        fem::Vector cTermFromM(n, 0.0);
        for (std::size_t i = 0; i < n; ++i) cTermFromM(i) = alphaM * m[i] * cInput(i);

        fem::Vector Feff = F + mTerm + cTermFromK + cTermFromM;

        fem::Vector dNext = fem::Solver::solve(Keff, Feff).x;

        fem::Vector accNext(n, 0.0), vNext(n, 0.0);
        for (std::size_t i = 0; i < n; ++i) {
            accNext(i) = a0 * (dNext(i) - d(i)) - a2 * v(i) - a3 * acc(i);
            vNext(i) = v(i) + dt * ((1.0 - gamma) * acc(i) + gamma * accNext(i));
        }

        d = dNext; v = vNext; acc = accNext;

        std::vector<double> dVec(d.data().begin(), d.data().end());
        std::vector<double> vVec(v.data().begin(), v.data().end());
        std::vector<double> aVec(acc.data().begin(), acc.data().end());

        DynamicTimeStep step;
        step.time = t;
        step.displacement = expandFreeDofVector(model, dVec);
        step.velocity = expandFreeDofVector(model, vVec);
        step.acceleration = expandFreeDofVector(model, aVec);
        result.steps.push_back(std::move(step));
    }

    return result;
}

ForcingFunction buildSeismicForceFn(const Model& model, const std::vector<double>& groundAccelMs2,
                                     double recordDt, bool directionX) {
    if (groundAccelMs2.empty()) {
        throw std::invalid_argument("buildSeismicForceFn: empty ground acceleration record");
    }
    if (recordDt <= 0.0) {
        throw std::invalid_argument("buildSeismicForceFn: recordDt must be positive");
    }

    // Capture, once, the mass and "excited-direction" indicator at
    // every free DOF, using the model's CURRENT dof numbering (the
    // caller must have already called Model::assignDofNumbers(), which
    // runNewmarkBeta itself does before it would ever invoke this
    // forcing function).
    int freeDofCount = model.freeDofCount();
    std::vector<double> massAtDof(static_cast<std::size_t>(freeDofCount), 0.0);
    std::vector<double> influenceAtDof(static_cast<std::size_t>(freeDofCount), 0.0);
    for (const auto& n : model.nodes()) {
        auto d = n.dofIndices();
        double m = n.translationalMass();
        int targetLocalDof = directionX ? static_cast<int>(DOF::Ux) : static_cast<int>(DOF::Uy);
        int gdof = d[static_cast<std::size_t>(targetLocalDof)];
        if (gdof >= 0) {
            massAtDof[static_cast<std::size_t>(gdof)] = m;
            influenceAtDof[static_cast<std::size_t>(gdof)] = 1.0;
        }
    }

    return [massAtDof, influenceAtDof, groundAccelMs2, recordDt](double t, int n) -> std::vector<double> {
        std::vector<double> F(static_cast<std::size_t>(n), 0.0);
        std::size_t idx = static_cast<std::size_t>(std::floor(t / recordDt));
        if (idx >= groundAccelMs2.size()) idx = groundAccelMs2.size() - 1;
        double ag = groundAccelMs2[idx];
        for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
            F[i] = -massAtDof[i] * influenceAtDof[i] * ag;
        }
        return F;
    };
}

}  // namespace nrsa::analysis
