#pragma once

#include <optional>
#include <string>

#include "fem/Matrix.h"
#include "fem/SparseMatrix.h"

namespace nrsa::fem {

// Solves K*x = F for a SparseMatrix K using Preconditioned Conjugate
// Gradient (Jacobi/diagonal preconditioner) — a DELIBERATE choice of
// iterative rather than direct solve; see the README roadmap entry for
// the reasoning, summarized here: a restrained, properly-supported
// linear-elastic stiffness matrix is symmetric positive-definite, which
// is exactly the condition PCG is guaranteed to converge for, and PCG's
// per-iteration cost is O(nnz) — it never densifies the matrix the way
// a naive sparse factorization without fill-reducing reordering would.
// A direct sparse factorization (Cholesky/LDLT with e.g. an
// approximate-minimum-degree reordering) remains a reasonable future
// upgrade — mainly for very ill-conditioned models (extreme stiffness
// ratios between adjacent elements) where PCG convergence can slow —
// but is a meaningfully larger undertaking (fill-reducing ordering,
// symbolic factorization) than this class, and PCG is a legitimate,
// widely-used production technique for large sparse SPD systems in its
// own right, not a stopgap.
// Options controlling Solver::solve's convergence behavior.
struct SolverOptions {
    double relativeTolerance = 1e-10;  // ||r|| / ||F|| convergence target
    int maxIterations = 0;             // 0 = auto (2 * matrix size, floored at 100)
};

// Everything Solver::solve reports back: the solution itself plus
// convergence diagnostics (iteration count, final residual) that are
// genuinely useful when tuning tolerance/iteration caps for a
// difficult model, not just decoration.
struct SolverResult {
    Vector x;
    int iterations = 0;
    double finalRelativeResidual = 0.0;
};

class Solver {
public:
    using Options = SolverOptions;
    using Result = SolverResult;

    // Throws std::runtime_error if:
    //  - K has a zero or negative diagonal entry (not SPD — almost
    //    always an unrestrained DOF/mechanism, the same failure mode
    //    Matrix::solve's dense path detects via a near-zero pivot);
    //  - PCG fails to reach the requested tolerance within
    //    maxIterations (a very ill-conditioned system — see the
    //    class-level note on when a direct factorization would help
    //    instead).
    static Result solve(const SparseMatrix& K, const Vector& F, Options opts = Options{});
};

}  // namespace nrsa::fem
