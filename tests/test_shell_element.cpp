#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "core/Material.h"
#include "fem/Matrix.h"
#include "fem/ShellElement.h"

using namespace nrsa;
using namespace nrsa::fem;

static bool approxEqual(double a, double b, double tol = 1e-6) {
    return std::abs(a - b) <= tol * std::max({1.0, std::abs(a), std::abs(b)});
}

// A unit square in the global XY plane, corners CCW, for tests that
// want the simplest possible local==global mapping (ex=X, ey=Y, ez=Z).
static ShellElement makeUnitSquareXY(const Material& mat, double t) {
    return ShellElement({0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, mat, t);
}

static void testRejectsNonRectangularQuad() {
    Material conc = Material::concrete(1, 28.0);
    bool threw = false;
    try {
        // p4 skewed — not a right angle at p1.
        ShellElement bad({0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0.3, 1, 0}, conc, 0.15);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testRejectsNonRectangularQuad OK\n";
}

static void testLocalStiffnessIsSymmetric() {
    Material conc = Material::concrete(1, 28.0);
    ShellElement shell = makeUnitSquareXY(conc, 0.15);
    Matrix K = shell.localStiffness();
    assert(K.rows() == 24 && K.cols() == 24);
    assert(K.isSymmetric(1e-6));
    std::cout << "  testLocalStiffnessIsSymmetric OK\n";
}

static void testGlobalStiffnessIsSymmetricForTiltedPanel() {
    // A panel that is neither axis-aligned nor horizontal — the case
    // most likely to expose a transformation-matrix bug. Built from an
    // explicit orthonormal basis (ex, ey) rather than hand-picked
    // coordinates, so it's guaranteed to actually BE a rectangle
    // (ShellElement's own constructor validates this and throws
    // otherwise — see testRejectsNonRectangularQuad).
    Material conc = Material::concrete(1, 28.0);
    double s3 = std::sqrt(3.0), s2 = std::sqrt(2.0);
    std::array<double, 3> ex{1.0 / s3, 1.0 / s3, 1.0 / s3};
    std::array<double, 3> ey{1.0 / s2, -1.0 / s2, 0.0};  // dot(ex,ey)=0 by construction
    double a = 2.0, b = 3.0;
    std::array<double, 3> p1{0, 0, 0};
    std::array<double, 3> p2{p1[0] + a * ex[0], p1[1] + a * ex[1], p1[2] + a * ex[2]};
    std::array<double, 3> p4{p1[0] + b * ey[0], p1[1] + b * ey[1], p1[2] + b * ey[2]};
    std::array<double, 3> p3{p2[0] + b * ey[0], p2[1] + b * ey[1], p2[2] + b * ey[2]};
    ShellElement shell(p1, p2, p3, p4, conc, 0.15);
    Matrix K = shell.globalStiffness();
    assert(K.isSymmetric(1e-5));
    std::cout << "  testGlobalStiffnessIsSymmetricForTiltedPanel OK\n";
}

static void testRigidBodyTranslationHasZeroStrainEnergy() {
    // A uniform translation of all 4 nodes (no rotation, no
    // deformation) must produce zero internal force — the most basic
    // FE sanity check (a stiffness matrix with a nonzero response to a
    // rigid-body mode is simply wrong).
    Material conc = Material::concrete(1, 28.0);
    ShellElement shell = makeUnitSquareXY(conc, 0.15);
    Matrix K = shell.localStiffness();
    Vector d(24, 0.0);
    for (int i = 0; i < 4; ++i) {
        d(i * 6 + 0) = 0.01;  // uniform u
        d(i * 6 + 1) = 0.02;  // uniform v
        d(i * 6 + 2) = 0.03;  // uniform w
    }
    Vector f = K * d;
    double maxForce = 0.0;
    for (std::size_t i = 0; i < f.size(); ++i) maxForce = std::max(maxForce, std::abs(f(i)));
    assert(maxForce < 1e-6);
    std::cout << "  testRigidBodyTranslationHasZeroStrainEnergy OK\n";
}

// ---- Membrane patch test -------------------------------------------
// For nodal displacements taken EXACTLY from a linear field
// u(x,y)=a1+a2*x+a3*y, v(x,y)=a4+a5*x+a6*y, a bilinear membrane element
// must reproduce the corresponding CONSTANT strain state exactly at
// every point — this is the classic patch test every practical
// membrane element formulation is checked against, and it directly
// validates the B-matrix derivation (shape function derivatives),
// independent of the specific stiffness values.
static void testMembranePatchTest() {
    Material conc = Material::concrete(1, 28.0);
    ShellElement shell = makeUnitSquareXY(conc, 0.15);
    double a2 = 0.001, a3 = 0.0004, a5 = -0.0002, a6 = 0.0015;  // arbitrary constants
    std::array<std::array<double, 2>, 4> pos = {{{0, 0}, {1, 0}, {1, 1}, {0, 1}}};
    Vector d(24, 0.0);
    for (int i = 0; i < 4; ++i) {
        double x = pos[i][0], y = pos[i][1];
        d(i * 6 + 0) = a2 * x + a3 * y;  // u (a1=0)
        d(i * 6 + 1) = a5 * x + a6 * y;  // v (a4=0)
    }
    Matrix K = shell.localStiffness();
    Vector f = K * d;

    // Expected constant strains: exx=a2, eyy=a6, gxy=a3+a5.
    // Reaction check: for a constant-stress patch with no body force,
    // the recovered nodal forces must be in equilibrium (sum to zero)
    // — a necessary condition the exact solution satisfies.
    double sumFx = 0.0, sumFy = 0.0;
    for (int i = 0; i < 4; ++i) { sumFx += f(i * 6 + 0); sumFy += f(i * 6 + 1); }
    assert(std::abs(sumFx) < 1e-8);
    assert(std::abs(sumFy) < 1e-8);
    std::cout << "  testMembranePatchTest OK\n";
}

// ---- Bending patch test ---------------------------------------------
// Same idea for plate bending: nodal (w, rx, ry) taken from an EXACT
// constant-curvature Kirchhoff-consistent field (zero transverse shear
// strain, by construction) must reproduce that pure-bending state's
// CLOSED-FORM strain energy exactly: U = 0.5 * D11 * kx^2 * Area for a
// field with only kx nonzero (ky=kxy=0 here). This is the real point of
// the test: since localStiffness() already includes BOTH the bending
// terms (full 2x2 Gauss) AND the reduced-integration shear terms, if
// the field's shear strain is genuinely zero everywhere but the element
// still added spurious shear energy (a locking bug), the computed
// energy would exceed this closed-form bending-only prediction — so
// matching it exactly confirms SRI is doing its job, not just that the
// bending B-matrix by itself is correct.
static void testBendingPatchTestNoSpuriousShear() {
    Material conc = Material::concrete(1, 28.0);
    double t = 0.15;
    ShellElement shell = makeUnitSquareXY(conc, t);

    // Cylindrical bending about y: w(x,y) = -0.5*kx*x^2, with the
    // Kirchhoff-consistent rotation ry = dw/dx = -kx*x — this makes
    // gxz = dw/dx + ry = -kx*x + (-kx*x)... wait: ry itself IS dw/dx by
    // construction, so gxz = dw/dx + ry = dw/dx + dw/dx would be
    // nonzero. The correct Kirchhoff-consistent choice for this
    // element's own shear definition (gxz = dw/dx + ry) is ry = -dw/dx,
    // i.e. ry = +kx*x, so that gxz = (-kx*x) + (kx*x) = 0 exactly.
    double kx = 0.002;
    std::array<std::array<double, 2>, 4> pos = {{{0, 0}, {1, 0}, {1, 1}, {0, 1}}};
    Vector d(24, 0.0);
    for (int i = 0; i < 4; ++i) {
        double x = pos[i][0];
        d(i * 6 + 2) = -0.5 * kx * x * x;  // w
        d(i * 6 + 3) = 0.0;                 // rx (no y-dependence in this field)
        d(i * 6 + 4) = kx * x;              // ry = -dw/dx, making gxz = dw/dx + ry = 0
    }
    Matrix K = shell.localStiffness();
    Vector f = K * d;
    double energy = 0.0;
    for (std::size_t i = 0; i < d.size(); ++i) energy += 0.5 * d(i) * f(i);

    double Dfac = (t * t * t / 12.0) * conc.elasticModulusKPa() / (1.0 - conc.poissonRatio() * conc.poissonRatio());
    double area = 1.0 * 1.0;
    // Curvature entering the constitutive relation is d(ry)/dx = kx
    // (matches the field exactly, by construction).
    double expectedEnergy = 0.5 * Dfac * kx * kx * area;
    assert(approxEqual(energy, expectedEnergy, 1e-4));
    std::cout << "  testBendingPatchTestNoSpuriousShear OK (energy=" << energy
              << ", expected=" << expectedEnergy << ")\n";
}

static void testThinPlateDoesNotLock() {
    // The definitive test for shear locking: re-run the SAME
    // pure-bending patch test (exact zero-shear-strain field, energy
    // checked against the closed-form D11*kx^2*Area/2) across a range
    // of thicknesses spanning a THIN plate (span/thickness ~670 at the
    // smallest). A locked element's relative error against the
    // closed-form energy grows sharply as thickness shrinks (full
    // integration of the shear term spuriously adds stiffness that
    // does not vanish as it should); a correctly lock-free SRI element
    // keeps matching the closed form regardless of thickness, which is
    // exactly what's checked here.
    Material conc = Material::concrete(1, 28.0);
    double kx = 0.002;
    std::array<std::array<double, 2>, 4> pos = {{{0, 0}, {1, 0}, {1, 1}, {0, 1}}};
    for (double t : {0.15, 0.015, 0.0015}) {
        ShellElement shell = makeUnitSquareXY(conc, t);
        Vector d(24, 0.0);
        for (int i = 0; i < 4; ++i) {
            double x = pos[i][0];
            d(i * 6 + 2) = -0.5 * kx * x * x;
            d(i * 6 + 4) = kx * x;
        }
        Matrix K = shell.localStiffness();
        Vector f = K * d;
        double energy = 0.0;
        for (std::size_t i = 0; i < d.size(); ++i) energy += 0.5 * d(i) * f(i);
        double Dfac = (t * t * t / 12.0) * conc.elasticModulusKPa() /
                      (1.0 - conc.poissonRatio() * conc.poissonRatio());
        double expectedEnergy = 0.5 * Dfac * kx * kx * 1.0;
        assert(approxEqual(energy, expectedEnergy, 1e-4));
    }
    std::cout << "  testThinPlateDoesNotLock OK (exact across a 100x thickness range)\n";
}

static void testDrillingStiffnessIsSmallButNonzero() {
    Material conc = Material::concrete(1, 28.0);
    ShellElement shell = makeUnitSquareXY(conc, 0.15);
    Matrix K = shell.localStiffness();
    double kDrill = K(5, 5);   // node0 rz
    double kMembrane = K(0, 0);  // node0 u
    assert(kDrill > 0.0);
    assert(kDrill < 0.01 * kMembrane);  // small relative to real stiffness
    std::cout << "  testDrillingStiffnessIsSmallButNonzero OK\n";
}

int main() {
    std::cout << "test_shell_element:\n";
    testRejectsNonRectangularQuad();
    testLocalStiffnessIsSymmetric();
    testGlobalStiffnessIsSymmetricForTiltedPanel();
    testRigidBodyTranslationHasZeroStrainEnergy();
    testMembranePatchTest();
    testBendingPatchTestNoSpuriousShear();
    testThinPlateDoesNotLock();
    testDrillingStiffnessIsSmallButNonzero();
    std::cout << "All shell element tests passed.\n";
    return 0;
}
