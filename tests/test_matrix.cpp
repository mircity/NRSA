#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "fem/Matrix.h"

using namespace nrsa::fem;

static bool approxEqual(double a, double b, double tol = 1e-9) {
    return std::abs(a - b) <= tol * std::max({1.0, std::abs(a), std::abs(b)});
}

static void testBasicArithmetic() {
    Matrix a(2, 2);
    a(0, 0) = 1; a(0, 1) = 2;
    a(1, 0) = 3; a(1, 1) = 4;
    Matrix b = Matrix::identity(2) * 2.0;  // [[2,0],[0,2]]

    Matrix sum = a + b;
    assert(approxEqual(sum(0, 0), 3) && approxEqual(sum(1, 1), 6));

    Matrix prod = a * b;  // a * 2I == 2a
    assert(approxEqual(prod(0, 0), 2) && approxEqual(prod(1, 1), 8));

    Matrix t = a.transpose();
    assert(approxEqual(t(0, 1), 3) && approxEqual(t(1, 0), 2));
    std::cout << "  testBasicArithmetic OK\n";
}

static void testSymmetryCheck() {
    Matrix k(2, 2);
    k(0, 0) = 4; k(0, 1) = -2; k(1, 0) = -2; k(1, 1) = 4;
    assert(k.isSymmetric());
    k(1, 0) = -2.5;
    assert(!k.isSymmetric());
    std::cout << "  testSymmetryCheck OK\n";
}

static void testSolveKnownSystem() {
    // Classic textbook system with an exact integer solution:
    //   2x + y = 5
    //   x + 3y = 10   ->  x=1, y=3
    Matrix A(2, 2);
    A(0, 0) = 2; A(0, 1) = 1;
    A(1, 0) = 1; A(1, 1) = 3;
    std::vector<double> b = {5, 10};
    auto x = Matrix::solve(A, b);
    assert(approxEqual(x[0], 1.0));
    assert(approxEqual(x[1], 3.0));
    std::cout << "  testSolveKnownSystem OK\n";
}

static void testSolveRequiresPivot() {
    // A(0,0) is zero, so a naive (non-pivoting) elimination would divide
    // by zero on the first step — this is exactly the case partial
    // pivoting exists to handle.
    //   0x + 2y = 4
    //   3x + 1y = 5    ->  from eq1: y=2; eq2: 3x+2=5 -> x=1
    Matrix A(2, 2);
    A(0, 0) = 0; A(0, 1) = 2;
    A(1, 0) = 3; A(1, 1) = 1;
    std::vector<double> b = {4, 5};
    auto x = Matrix::solve(A, b);
    assert(approxEqual(x[0], 1.0));
    assert(approxEqual(x[1], 2.0));
    std::cout << "  testSolveRequiresPivot OK\n";
}

static void testSolveDetectsSingular() {
    // Row 2 is exactly 2x row 1 -> singular.
    Matrix A(2, 2);
    A(0, 0) = 1; A(0, 1) = 2;
    A(1, 0) = 2; A(1, 1) = 4;
    std::vector<double> b = {1, 2};
    bool threw = false;
    try {
        Matrix::solve(A, b);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testSolveDetectsSingular OK\n";
}

static void testSpringChainAgainstHandCalc() {
    // Two-spring chain, both k=100 kN/m, node 1 fixed, force 50 kN at
    // node 3 (free end): a trivial but genuine stiffness-method sanity
    // check — same assembly pattern a frame element's axial stiffness
    // will use, just at 1 DOF/node instead of 6.
    //   node1 --k1-- node2 --k2-- node3
    // Global K (DOFs = node2, node3 free; node1 restrained/eliminated):
    //   [ k1+k2   -k2 ] [u2]   [0 ]
    //   [  -k2     k2 ] [u3] = [50]
    double k1 = 100.0, k2 = 100.0, F = 50.0;
    Matrix K(2, 2);
    K(0, 0) = k1 + k2; K(0, 1) = -k2;
    K(1, 0) = -k2;      K(1, 1) = k2;
    std::vector<double> F_vec = {0.0, F};
    auto u = Matrix::solve(K, F_vec);
    // Hand calc: u3 = F/k2 + F/k1 (two springs in series carrying the
    // same 50 kN) = 0.5 + 0.5 = 1.0 m; u2 = F/k1 = 0.5 m.
    assert(approxEqual(u[0], 0.5));
    assert(approxEqual(u[1], 1.0));
    std::cout << "  testSpringChainAgainstHandCalc OK\n";
}

static void testVectorOps() {
    Vector a{1.0, 2.0, 3.0};
    Vector b{4.0, 5.0, 6.0};
    assert(approxEqual(a.dot(b), 32.0));  // 1*4+2*5+3*6
    Vector c = a + b;
    assert(approxEqual(c(0), 5.0) && approxEqual(c(2), 9.0));
    assert(approxEqual(Vector{3.0, 4.0}.norm(), 5.0));  // 3-4-5 triangle
    std::cout << "  testVectorOps OK\n";
}

static void testMatrixVectorProduct() {
    Matrix I = Matrix::identity(3);
    Vector v{7.0, 8.0, 9.0};
    Vector r = I * v;
    assert(approxEqual(r(0), 7.0) && approxEqual(r(1), 8.0) && approxEqual(r(2), 9.0));
    std::cout << "  testMatrixVectorProduct OK\n";
}

int main() {
    std::cout << "test_matrix:\n";
    testBasicArithmetic();
    testSymmetryCheck();
    testSolveKnownSystem();
    testSolveRequiresPivot();
    testSolveDetectsSingular();
    testSpringChainAgainstHandCalc();
    testVectorOps();
    testMatrixVectorProduct();
    std::cout << "All matrix tests passed.\n";
    return 0;
}
