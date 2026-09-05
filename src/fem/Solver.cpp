#include "fem/Solver.h"

#include <cmath>
#include <stdexcept>

namespace nrsa::fem {

Solver::Result Solver::solve(const SparseMatrix& K, const Vector& F, Options opts) {
    std::size_t n = K.size();
    if (F.size() != n) {
        throw std::invalid_argument("Solver::solve: F size does not match K");
    }

    // Jacobi preconditioner: M^-1 = diag(1/K_ii). Also doubles as the
    // same "is this system even solvable" sanity check Matrix::solve's
    // dense pivot-magnitude test performs — a zero or negative diagonal
    // entry in a properly assembled stiffness matrix means that DOF has
    // no stiffness contribution from anything (an unrestrained node, or
    // a whole disconnected sub-structure with no path to a support),
    // which PCG (which requires K to be SPD) cannot proceed with
    // regardless of preconditioning.
    Vector invDiag(n);
    for (std::size_t i = 0; i < n; ++i) {
        double d = K.diagonal(i);
        if (d <= 0.0) {
            throw std::runtime_error(
                "Solver::solve: non-positive diagonal at DOF " + std::to_string(i) +
                " — the system is not positive-definite, almost always because that DOF "
                "(or a whole disconnected sub-structure containing it) has no stiffness path "
                "to a support. Check for a node with no restraint and no connected element.");
        }
        invDiag(i) = 1.0 / d;
    }

    int maxIter = opts.maxIterations > 0 ? opts.maxIterations
                                          : std::max(100, static_cast<int>(2 * n));

    Vector x(n, 0.0);
    Vector r = F;  // r0 = F - K*x0, x0 = 0 -> r0 = F
    double normF = std::sqrt(F.dot(F));
    if (normF < 1e-300) {
        // Zero load vector -> zero solution, trivially converged.
        return Result{x, 0, 0.0};
    }

    auto applyPreconditioner = [&](const Vector& v) {
        Vector z(n);
        for (std::size_t i = 0; i < n; ++i) z(i) = v(i) * invDiag(i);
        return z;
    };

    Vector z = applyPreconditioner(r);
    Vector p = z;
    double rz = r.dot(z);

    int iter = 0;
    double relResidual = std::sqrt(r.dot(r)) / normF;
    for (; iter < maxIter; ++iter) {
        if (relResidual <= opts.relativeTolerance) break;

        Vector Kp = K.multiply(p);
        double pKp = p.dot(Kp);
        // For a genuinely symmetric POSITIVE-DEFINITE matrix, p^T*K*p
        // must be strictly positive for any nonzero search direction p
        // — this is the standard PCG "negative curvature" diagnostic
        // for a matrix that is NOT positive-definite (e.g. a structure
        // loaded at or beyond its buckling capacity via a geometric-
        // stiffness term, which is exactly how analysis::
        // NonlinearAnalysis discovered this check needed strengthening:
        // the original version here only caught pKp NEAR ZERO, missing
        // the case where pKp goes clearly NEGATIVE — which doesn't
        // stall the iteration, it lets it "converge" to a numerically
        // plausible-LOOKING but physically wrong answer instead (e.g. a
        // displacement in the opposite direction from the applied
        // load, which cannot happen for a stable structure). Checking
        // the SIGN, not just the magnitude, is what actually catches
        // that failure mode.
        if (pKp <= 1e-300) {
            throw std::runtime_error(
                "Solver::solve: negative or zero curvature detected (p^T*K*p <= 0) at iteration " +
                std::to_string(iter) + " — the matrix is not positive-definite. For a structural "
                "stiffness matrix this most often means an unrestrained mechanism, or (if a "
                "geometric/P-Delta stiffness term is included) that the applied load has reached "
                "or exceeded the structure's buckling capacity.");
        }
        double alpha = rz / pKp;
        for (std::size_t i = 0; i < n; ++i) x(i) += alpha * p(i);
        for (std::size_t i = 0; i < n; ++i) r(i) -= alpha * Kp(i);

        relResidual = std::sqrt(r.dot(r)) / normF;
        if (relResidual <= opts.relativeTolerance) { ++iter; break; }

        Vector zNew = applyPreconditioner(r);
        double rzNew = r.dot(zNew);
        double beta = rzNew / rz;
        for (std::size_t i = 0; i < n; ++i) p(i) = zNew(i) + beta * p(i);
        z = zNew;
        rz = rzNew;
    }

    if (relResidual > opts.relativeTolerance) {
        throw std::runtime_error(
            "Solver::solve: did not converge within " + std::to_string(maxIter) +
            " iterations (final relative residual " + std::to_string(relResidual) +
            ", target " + std::to_string(opts.relativeTolerance) +
            ") — the system may be very ill-conditioned (extreme stiffness ratios between "
            "adjacent elements); a direct sparse factorization would be more robust here "
            "(see the Solver class doc comment).");
    }

    return Result{x, iter, relResidual};
}

}  // namespace nrsa::fem
