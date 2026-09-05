#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "fem/Solver.h"
#include "fem/SparseMatrix.h"

using namespace nrsa::fem;

static bool approxEqual(double a, double b, double relTol = 1e-6) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

static void testAssemblyAndSymmetricGet() {
    SparseMatrix K(3);
    K.add(0, 0, 4.0);
    K.add(0, 1, -1.0);  // also represents (1,0)
    K.add(1, 1, 4.0);
    K.add(1, 2, -1.0);
    K.add(2, 2, 4.0);
    assert(approxEqual(K.get(0, 1), -1.0));
    assert(approxEqual(K.get(1, 0), -1.0));  // symmetric lookup
    assert(approxEqual(K.diagonal(1), 4.0));
    assert(K.nnz() == 5);  // 3 diagonal + 2 off-diagonal (upper triangle only)
    std::cout << "  testAssemblyAndSymmetricGet OK\n";
}

static void testRepeatedAddAccumulates() {
    // Simulates two elements both contributing to the same shared DOF —
    // exactly what happens when two frame elements meet at a node.
    SparseMatrix K(2);
    K.add(0, 0, 10.0);
    K.add(0, 0, 5.0);  // second element's contribution to the same entry
    assert(approxEqual(K.diagonal(0), 15.0));
    std::cout << "  testRepeatedAddAccumulates OK\n";
}

static void testMultiplyMatchesDenseByHand() {
    // K = [[4,-1,0],[-1,4,-1],[0,-1,4]], x = [1,2,3]
    // K*x = [4*1-1*2, -1*1+4*2-1*3, -1*2+4*3] = [2, 4, 10]
    SparseMatrix K(3);
    K.add(0, 0, 4.0); K.add(0, 1, -1.0);
    K.add(1, 1, 4.0); K.add(1, 2, -1.0);
    K.add(2, 2, 4.0);
    Vector x{1.0, 2.0, 3.0};
    Vector y = K.multiply(x);
    assert(approxEqual(y(0), 2.0));
    assert(approxEqual(y(1), 4.0));
    assert(approxEqual(y(2), 10.0));
    std::cout << "  testMultiplyMatchesDenseByHand OK\n";
}

static void testSolveSpringChainAgainstHandCalc() {
    // Same two-spring-chain case used to validate the dense
    // Matrix::solve() in test_matrix.cpp — now through the sparse/PCG
    // path, checked against the identical hand-calculated answer, so
    // any divergence between the two solvers would show up as a test
    // failure in exactly one of the two suites.
    //   k1=k2=100 kN/m, F=50kN at the free end
    //   -> u2=0.5m, u3=1.0m (u1 fixed/eliminated)
    double k1 = 100.0, k2 = 100.0, F = 50.0;
    SparseMatrix K(2);
    K.add(0, 0, k1 + k2);
    K.add(0, 1, -k2);
    K.add(1, 1, k2);
    Vector Fvec{0.0, F};
    auto result = Solver::solve(K, Fvec);
    assert(approxEqual(result.x(0), 0.5));
    assert(approxEqual(result.x(1), 1.0));
    assert(result.finalRelativeResidual <= 1e-10 * 1.001);  // met the default tolerance
    std::cout << "  testSolveSpringChainAgainstHandCalc OK\n";
}

static void testSolveMatchesDenseOnLargerSystem() {
    // A 1D chain of 20 springs, each k=200 kN/m, fixed at both ends,
    // with a distinct load at every interior DOF — big enough that a
    // hand calc isn't practical, so this cross-checks the sparse/PCG
    // path against the already-validated dense Matrix::solve() instead
    // (same assembly, two different solvers, answers must agree).
    const int n = 19;  // 20 springs -> 19 free interior DOFs (both ends fixed)
    double k = 200.0;
    SparseMatrix Ksparse(n);
    Matrix Kdense(n, n, 0.0);
    Vector F(n);
    for (int i = 0; i < n; ++i) {
        double diag = (i == 0 || i == n - 1) ? 2.0 * k : 2.0 * k;  // interior node always sees 2 springs
        Ksparse.add(i, i, diag);
        Kdense(i, i) = diag;
        if (i + 1 < n) {
            Ksparse.add(i, i + 1, -k);
            Kdense(i, i + 1) = -k;
            Kdense(i + 1, i) = -k;
        }
        F(i) = 10.0 * (i + 1);  // arbitrary distinct load per DOF
    }
    auto sparseResult = Solver::solve(Ksparse, F, Solver::Options{1e-12, 0});
    std::vector<double> Fstd(n);
    for (int i = 0; i < n; ++i) Fstd[i] = F(i);
    auto denseResult = Matrix::solve(Kdense, Fstd);
    for (int i = 0; i < n; ++i) {
        assert(approxEqual(sparseResult.x(i), denseResult[i], 1e-6));
    }
    std::cout << "  testSolveMatchesDenseOnLargerSystem OK (density="
              << Ksparse.density() << ", converged in " << sparseResult.iterations
              << " iterations)\n";
}

static void testZeroDiagonalThrows() {
    // DOF 1 has no stiffness contribution at all (a mechanism/floating
    // node) -- Solver must reject this the same way the dense path
    // rejects a near-zero pivot, rather than returning garbage.
    SparseMatrix K(2);
    K.add(0, 0, 10.0);
    // K(1,1) never set -> diagonal() returns 0.0.
    Vector F{1.0, 1.0};
    bool threw = false;
    try {
        Solver::solve(K, F);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testZeroDiagonalThrows OK\n";
}

// A genuinely INDEFINITE matrix (positive diagonal entries individually,
// but not positive-definite overall) -- the exact failure mode
// analysis::NonlinearAnalysis's near-buckling test uncovered: a naive
// diagonal-only check would miss this (both diagonal entries here are
// positive), but the negative-curvature check inside PCG's own
// iteration must still catch it. K = [[1,2],[2,1]] has eigenvalues 3
// (eigenvector [1,1]) and -1 (eigenvector [1,-1]) -- indefinite by
// construction. The right-hand side matters here: an RHS that happens
// to lie entirely along the POSITIVE eigenvector (e.g. [1,1]) never
// excites the unstable direction and can "solve" without incident, so
// F is chosen with a genuine component along the negative-eigenvalue
// eigenvector [1,-1] specifically to guarantee the breakdown is
// actually reached, not just theoretically present in the matrix.
static void testIndefiniteMatrixThrows() {
    SparseMatrix K(2);
    K.add(0, 0, 1.0);
    K.add(0, 1, 2.0);
    K.add(1, 1, 1.0);
    Vector F{1.0, -1.0};  // aligned with the negative-eigenvalue eigenvector
    bool threw = false;
    try {
        Solver::solve(K, F);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testIndefiniteMatrixThrows OK\n";
}

static void testZeroLoadReturnsZeroSolution() {
    SparseMatrix K(3);
    K.add(0, 0, 4.0); K.add(1, 1, 4.0); K.add(2, 2, 4.0);
    Vector F(3, 0.0);
    auto result = Solver::solve(K, F);
    for (std::size_t i = 0; i < 3; ++i) assert(approxEqual(result.x(i) + 1.0, 1.0));  // ~0
    std::cout << "  testZeroLoadReturnsZeroSolution OK\n";
}

int main() {
    std::cout << "test_sparse_solver:\n";
    testAssemblyAndSymmetricGet();
    testRepeatedAddAccumulates();
    testMultiplyMatchesDenseByHand();
    testSolveSpringChainAgainstHandCalc();
    testSolveMatchesDenseOnLargerSystem();
    testZeroDiagonalThrows();
    testIndefiniteMatrixThrows();
    testZeroLoadReturnsZeroSolution();
    std::cout << "All sparse solver tests passed.\n";
    return 0;
}
