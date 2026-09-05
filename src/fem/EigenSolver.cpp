#include "fem/EigenSolver.h"

#include <algorithm>
#include <cmath>

namespace nrsa::fem {

void EigenSolver::jacobiEigenDecomposition(Matrix& A, Matrix& V, Options opts) {
    std::size_t n = A.rows();
    if (A.cols() != n) throw std::invalid_argument("EigenSolver: matrix must be square");

    auto offDiagonalNorm2 = [&]() {
        double s = 0.0;
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = i + 1; j < n; ++j) s += A(i, j) * A(i, j);
        return s;
    };
    double initialNorm2 = offDiagonalNorm2();
    if (initialNorm2 < 1e-300) return;  // already diagonal

    for (int sweep = 0; sweep < opts.maxSweeps; ++sweep) {
        for (std::size_t p = 0; p < n; ++p) {
            for (std::size_t q = p + 1; q < n; ++q) {
                double apq = A(p, q);
                if (std::abs(apq) < 1e-300) continue;

                double app = A(p, p), aqq = A(q, q);
                // Classic (Numerical Recipes-style) Jacobi rotation:
                // solve for t = tan(rotation angle) directly via the
                // smaller root of t^2 + 2*t*theta - 1 = 0 (theta =
                // cot(2*rotation angle)), which is both more numerically
                // stable near theta=0 and simpler to get algebraically
                // right than working with the rotation angle itself —
                // verified against the closed-form 2x2 eigenvalues of
                // tests/test_eigen_solver.cpp's spring-mass case before
                // relying on it here.
                double theta = (aqq - app) / (2.0 * apq);
                double t = (theta >= 0.0 ? 1.0 : -1.0) /
                           (std::abs(theta) + std::sqrt(theta * theta + 1.0));
                double c = 1.0 / std::sqrt(t * t + 1.0);
                double s = t * c;
                double tau = s / (1.0 + c);
                double h = t * apq;

                A(p, p) = app - h;
                A(q, q) = aqq + h;
                A(p, q) = 0.0;
                A(q, p) = 0.0;

                for (std::size_t k = 0; k < n; ++k) {
                    if (k == p || k == q) continue;
                    double akp = A(k, p), akq = A(k, q);
                    double newKp = akp - s * (akq + tau * akp);
                    double newKq = akq + s * (akp - tau * akq);
                    A(k, p) = newKp; A(p, k) = newKp;
                    A(k, q) = newKq; A(q, k) = newKq;
                }

                // Accumulate the rotation into V (eigenvector matrix).
                for (std::size_t k = 0; k < n; ++k) {
                    double vkp = V(k, p), vkq = V(k, q);
                    V(k, p) = vkp - s * (vkq + tau * vkp);
                    V(k, q) = vkq + s * (vkp - tau * vkq);
                }
            }
        }
        // Relative convergence check against the ORIGINAL off-diagonal
        // magnitude, not an absolute one, so this scales correctly
        // regardless of the matrix's own units/magnitude.
        if (offDiagonalNorm2() < opts.tolerance * opts.tolerance * initialNorm2) return;
    }
    // Falling through the sweep loop without an early return means the
    // requested tolerance wasn't reached — Jacobi is guaranteed to
    // converge eventually for any symmetric matrix, so hitting the
    // sweep cap here means opts.maxSweeps was set too low for this
    // matrix's size/conditioning, not that the algorithm failed;
    // throwing (rather than silently returning a partially-converged
    // result) matches the "say why, don't return garbage" pattern used
    // by fem::Matrix::solve and fem::Solver::solve elsewhere in this
    // project.
    throw std::runtime_error(
        "EigenSolver: Jacobi diagonalization did not converge within " +
        std::to_string(opts.maxSweeps) + " sweeps — try increasing Options::maxSweeps.");
}

DenseSymmetricEigenResult EigenSolver::solveSymmetric(Matrix A, Options opts) {
    std::size_t n = A.rows();
    Matrix V = Matrix::identity(n);
    jacobiEigenDecomposition(A, V, opts);

    std::vector<std::size_t> order(n);
    for (std::size_t i = 0; i < n; ++i) order[i] = i;
    std::sort(order.begin(), order.end(),
              [&](std::size_t a, std::size_t b) { return A(a, a) < A(b, b); });

    DenseSymmetricEigenResult result;
    result.eigenvalues.resize(n);
    result.eigenvectors = Matrix(n, n);
    for (std::size_t k = 0; k < n; ++k) {
        std::size_t src = order[k];
        result.eigenvalues[k] = A(src, src);
        for (std::size_t i = 0; i < n; ++i) result.eigenvectors(i, k) = V(i, src);
    }
    return result;
}

std::vector<ModeShape> EigenSolver::solve(const Matrix& K, const std::vector<double>& diagonalMass,
                                           Options opts) {
    std::size_t n = K.rows();
    if (K.cols() != n) throw std::invalid_argument("EigenSolver::solve: K must be square");
    if (diagonalMass.size() != n) {
        throw std::invalid_argument("EigenSolver::solve: mass vector size does not match K");
    }
    std::vector<double> invSqrtM(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (diagonalMass[i] <= 0.0) {
            throw std::invalid_argument(
                "EigenSolver::solve: non-positive mass at DOF " + std::to_string(i) +
                " — every DOF entering a modal analysis needs a positive lumped mass "
                "(a rotational DOF with no assigned rotational inertia is a common cause; "
                "see core::Node::setRotationalMass).");
        }
        invSqrtM[i] = 1.0 / std::sqrt(diagonalMass[i]);
    }

    // A = M^-1/2 * K * M^-1/2 — trivial elementwise scaling since
    // M^-1/2 is diagonal: A(i,j) = K(i,j) * invSqrtM[i] * invSqrtM[j].
    Matrix A(n, n);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j)
            A(i, j) = K(i, j) * invSqrtM[i] * invSqrtM[j];

    Matrix V = Matrix::identity(n);
    jacobiEigenDecomposition(A, V, opts);

    // Eigenvalues are now A's diagonal; eigenvectors are V's columns
    // (in the mass-NORMALIZED psi basis — psi_k^T psi_j = delta_kj).
    // Physical mode shapes phi_k = M^-1/2 * psi_k satisfy the mass-
    // normalization a modal/response-spectrum analysis wants directly:
    // phi_k^T * M * phi_k = psi_k^T (M^-1/2 M M^-1/2) psi_k = psi_k^T psi_k = 1
    // (since M^-1/2 M M^-1/2 = I by construction).
    std::vector<ModeShape> modes(n);
    for (std::size_t k = 0; k < n; ++k) {
        double lambda = A(k, k);
        // A negative or ~zero eigenvalue indicates a rigid-body mode or
        // an unrestrained mechanism (K is only positive-SEMI-definite
        // for an unrestrained model) — clamp to zero rather than taking
        // sqrt of a small negative number from round-off, but leave a
        // genuinely negative eigenvalue visible (a real modeling
        // problem, not roundoff, if it's not tiny) by NOT clamping
        // large negative values — the caller's own sanity checking
        // (e.g. StaticAnalysis-style residual/positive-definiteness
        // checks) is a more appropriate place to reject that than
        // silently swallowing it here.
        double omega2 = (lambda > 0.0) ? lambda : 0.0;
        double omega = std::sqrt(omega2);
        Vector phi(n);
        for (std::size_t i = 0; i < n; ++i) phi(i) = V(i, k) * invSqrtM[i];
        modes[k] = ModeShape{lambda, omega / (2.0 * M_PI), omega > 1e-300 ? 2.0 * M_PI / omega : 0.0, phi};
    }
    std::sort(modes.begin(), modes.end(),
              [](const ModeShape& a, const ModeShape& b) { return a.eigenvalue < b.eigenvalue; });
    return modes;
}

}  // namespace nrsa::fem
