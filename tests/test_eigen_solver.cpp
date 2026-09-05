#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "fem/EigenSolver.h"
#include "fem/Matrix.h"

using namespace nrsa::fem;

static bool approxEqual(double a, double b, double relTol = 1e-6) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

static void testDiagonalMatrixReturnsItsOwnDiagonal() {
    // A matrix that's already diagonal must return exactly its own
    // diagonal entries as eigenvalues (with mass=1 everywhere, the
    // generalized problem reduces to the plain eigenproblem K*phi =
    // lambda*phi).
    Matrix K(3, 3, 0.0);
    K(0, 0) = 5.0; K(1, 1) = 2.0; K(2, 2) = 9.0;
    std::vector<double> mass = {1.0, 1.0, 1.0};
    auto modes = EigenSolver::solve(K, mass);
    assert(modes.size() == 3);
    assert(approxEqual(modes[0].eigenvalue, 2.0));  // sorted ascending
    assert(approxEqual(modes[1].eigenvalue, 5.0));
    assert(approxEqual(modes[2].eigenvalue, 9.0));
    std::cout << "  testDiagonalMatrixReturnsItsOwnDiagonal OK\n";
}

// The definitive cross-check: a 2-DOF spring-mass chain (mass1-spring-
// mass2, mass1 also grounded through another spring) has a
// closed-form characteristic equation solvable directly with the
// quadratic formula — an INDEPENDENT method from the N-dimensional
// Jacobi routine being tested, even though N happens to be 2 here.
static void testTwoDofSpringMassAgainstClosedForm() {
    double m1 = 2.0, m2 = 1.0, k1 = 300.0, k2 = 150.0;
    Matrix K(2, 2, 0.0);
    K(0, 0) = k1 + k2; K(0, 1) = -k2;
    K(1, 0) = -k2;     K(1, 1) = k2;
    std::vector<double> mass = {m1, m2};

    auto modes = EigenSolver::solve(K, mass);
    assert(modes.size() == 2);

    // Closed form: A = M^-1/2 K M^-1/2, characteristic quadratic
    // lambda^2 - trace*lambda + det = 0.
    double A00 = (k1 + k2) / m1;
    double A11 = k2 / m2;
    double A01 = -k2 / std::sqrt(m1 * m2);
    double trace = A00 + A11;
    double det = A00 * A11 - A01 * A01;
    double disc = std::sqrt(trace * trace - 4.0 * det);
    double lambdaLow = (trace - disc) / 2.0;
    double lambdaHigh = (trace + disc) / 2.0;

    assert(approxEqual(modes[0].eigenvalue, lambdaLow, 1e-8));
    assert(approxEqual(modes[1].eigenvalue, lambdaHigh, 1e-8));

    // Cross-check the reported natural frequency/period are consistent
    // with the eigenvalue itself (omega=sqrt(lambda), f=omega/2pi,
    // T=1/f).
    double omega0 = std::sqrt(modes[0].eigenvalue);
    assert(approxEqual(modes[0].naturalFrequencyHz, omega0 / (2.0 * M_PI)));
    assert(approxEqual(modes[0].period, 1.0 / modes[0].naturalFrequencyHz));

    std::cout << "  testTwoDofSpringMassAgainstClosedForm OK (T1=" << modes[0].period
              << "s, T2=" << modes[1].period << "s)\n";
}

static void testEigenvectorsAreMassOrthonormal() {
    // phi_k^T * M * phi_j should be 1 for k==j, 0 otherwise — the
    // defining property of mass-normalized mode shapes, and what makes
    // them usable directly in a modal-combination (SRSS/CQC) formula
    // later without a separate normalization step.
    Matrix K(3, 3, 0.0);
    K(0, 0) = 400; K(0, 1) = -200;               K(1, 0) = -200;
    K(1, 1) = 500; K(1, 2) = -300; K(2, 1) = -300; K(2, 2) = 300;
    std::vector<double> mass = {2.0, 1.5, 1.0};
    auto modes = EigenSolver::solve(K, mass);
    assert(modes.size() == 3);

    for (std::size_t k = 0; k < 3; ++k) {
        for (std::size_t j = 0; j < 3; ++j) {
            double sum = 0.0;
            for (std::size_t i = 0; i < 3; ++i) {
                sum += modes[k].shape(i) * mass[i] * modes[j].shape(i);
            }
            double expected = (k == j) ? 1.0 : 0.0;
            assert(std::abs(sum - expected) < 1e-6);
        }
    }
    std::cout << "  testEigenvectorsAreMassOrthonormal OK\n";
}

static void testEigenvectorsSatisfyKPhiEqualsLambdaMPhi() {
    // Direct residual check: K*phi_k - lambda_k*M*phi_k should be ~0 —
    // the literal definition of a generalized eigenpair, independent of
    // how modes[] was computed internally.
    Matrix K(3, 3, 0.0);
    K(0, 0) = 400; K(0, 1) = -200;               K(1, 0) = -200;
    K(1, 1) = 500; K(1, 2) = -300; K(2, 1) = -300; K(2, 2) = 300;
    std::vector<double> mass = {2.0, 1.5, 1.0};
    auto modes = EigenSolver::solve(K, mass);

    for (const auto& m : modes) {
        Vector Kphi = K * m.shape;
        double maxResidual = 0.0;
        for (std::size_t i = 0; i < 3; ++i) {
            double residual = Kphi(i) - m.eigenvalue * mass[i] * m.shape(i);
            maxResidual = std::max(maxResidual, std::abs(residual));
        }
        assert(maxResidual < 1e-6 * std::max(1.0, std::abs(m.eigenvalue)));
    }
    std::cout << "  testEigenvectorsSatisfyKPhiEqualsLambdaMPhi OK\n";
}

static void testRejectsNonPositiveMass() {
    Matrix K = Matrix::identity(2);
    std::vector<double> mass = {1.0, 0.0};  // zero mass at DOF 1
    bool threw = false;
    try {
        EigenSolver::solve(K, mass);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testRejectsNonPositiveMass OK\n";
}

static void testMismatchedSizesThrows() {
    Matrix K = Matrix::identity(3);
    std::vector<double> mass = {1.0, 1.0};  // wrong size
    bool threw = false;
    try {
        EigenSolver::solve(K, mass);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testMismatchedSizesThrows OK\n";
}

int main() {
    std::cout << "test_eigen_solver:\n";
    testDiagonalMatrixReturnsItsOwnDiagonal();
    testTwoDofSpringMassAgainstClosedForm();
    testEigenvectorsAreMassOrthonormal();
    testEigenvectorsSatisfyKPhiEqualsLambdaMPhi();
    testRejectsNonPositiveMass();
    testMismatchedSizesThrows();
    std::cout << "All eigen solver tests passed.\n";
    return 0;
}
