#pragma once

#include <stdexcept>
#include <vector>

#include "fem/Matrix.h"

namespace nrsa::fem {

struct ModeShape {
    double eigenvalue;         // omega^2, (rad/s)^2
    double naturalFrequencyHz;
    double period;             // seconds
    Vector shape;              // mass-normalized: shape^T * M * shape == 1
};

// Solves the generalized symmetric eigenproblem K*phi = omega^2*M*phi
// for a LUMPED (diagonal) mass matrix — exactly the mass idealization a
// building's floor/story masses are normally represented with for a
// modal or response-spectrum analysis.
//
// METHOD: mass-normalize via the diagonal M^-1/2 (trivial elementwise,
// since M is diagonal) to reduce the generalized problem to a STANDARD
// symmetric eigenproblem A*psi = lambda*psi, A = M^-1/2 * K * M^-1/2,
// then diagonalize A with the classic cyclic JACOBI ROTATION method —
// simple, numerically robust, and guaranteed to converge for any
// symmetric matrix, at the cost of being a DENSE, O(n^3)-per-sweep
// algorithm that finds ALL n eigenpairs even when only the lowest few
// (the modes that actually matter for a response-spectrum analysis) are
// wanted.
//
// SCOPE: this mirrors the same honest tradeoff fem::Solver documents
// for the dense-vs-sparse LINEAR solve — appropriate for the
// small-to-moderate free-DOF counts this stage of the engine is built
// for, not yet the fully scalable answer. A production-scale
// alternative (Lanczos or subspace iteration, extracting only the
// lowest k modes from a SPARSE K/M without ever forming a dense matrix)
// is a reasonable future upgrade once model sizes that actually need it
// are in play — implementing it correctly (including the numerically
// delicate re-orthogonalization Lanczos needs) is a meaningfully larger
// undertaking than this class, and was judged not worth blocking a
// first working modal-analysis pipeline on, the same call made for PCG
// over a direct sparse factorization in fem::Solver.
// Options controlling EigenSolver::solve's Jacobi convergence behavior.
struct EigenSolverOptions {
    double tolerance = 1e-12;  // off-diagonal Frobenius norm (relative) to stop at
    int maxSweeps = 100;       // a full cyclic sweep visits every (p,q) pair once
};

// Result of diagonalizing a plain dense symmetric matrix directly (no
// mass matrix / generalized-eigenproblem framing involved) -- see
// EigenSolver::solveSymmetric.
struct DenseSymmetricEigenResult {
    std::vector<double> eigenvalues;  // ascending
    Matrix eigenvectors;              // column k is the eigenvector for eigenvalues[k]
};

class EigenSolver {
public:
    using Options = EigenSolverOptions;

    // Diagonalizes an arbitrary dense symmetric matrix A directly, via
    // the same cyclic Jacobi rotation method solve() uses internally --
    // exposed as a public building block for OTHER generalized
    // eigenproblems this engine needs to reduce to a standard symmetric
    // form THEMSELVES before handing the result here. The first (and,
    // as of this writing, only) consumer is analysis::BucklingAnalysis,
    // which reduces K*phi = -lambda*Kg*phi to a standard symmetric
    // eigenproblem via a Cholesky congruence transform (see that
    // class) -- this method only does the final "diagonalize a
    // symmetric matrix" step, the same step solve() does after its own
    // mass-normalization reduction.
    //
    // Throws whatever jacobiEigenDecomposition itself throws (matrix
    // not square; Jacobi failing to converge within opts.maxSweeps).
    static DenseSymmetricEigenResult solveSymmetric(Matrix A, Options opts = Options{});

    // Every diagonal mass entry must be strictly positive — a
    // zero/negative lumped mass at some DOF makes M^-1/2 undefined,
    // almost always because that DOF was never given a mass (e.g. a
    // rotational DOF with no rotational inertia assigned). Throws
    // std::invalid_argument rather than silently treating it as zero
    // (which would produce an infinite or NaN frequency).
    //
    // Returns all n modes, SORTED ASCENDING by eigenvalue (== lowest
    // natural frequency first, matching how "mode 1, mode 2, ..." is
    // conventionally numbered).
    static std::vector<ModeShape> solve(const Matrix& K, const std::vector<double>& diagonalMass,
                                         Options opts = Options{});

private:
    // The Jacobi sweep itself, operating on a plain dense symmetric
    // matrix — factored out from solve() so it has no knowledge of the
    // mass-normalization step, keeping the "diagonalize a symmetric
    // matrix" algorithm reusable/testable independent of the
    // generalized-eigenproblem framing built on top of it.
    static void jacobiEigenDecomposition(Matrix& A, Matrix& V, Options opts);
};

}  // namespace nrsa::fem
