#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "core/Material.h"
#include "core/Section.h"
#include "fem/FrameElement3D.h"
#include "fem/Matrix.h"

using namespace nrsa;
using namespace nrsa::fem;

static bool approxEqual(double a, double b, double relTol = 1e-6) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

static void testLocalStiffnessIsSymmetric() {
    Material steel = Material::structuralSteel(1, 250.0);
    Section sec = Section::rectangular(0.3, 0.5);
    FrameElement3D elem({0, 0, 0}, {4, 0, 0}, steel, sec);
    Matrix k = elem.localStiffness();
    assert(k.rows() == 12 && k.cols() == 12);
    assert(k.isSymmetric());
    std::cout << "  testLocalStiffnessIsSymmetric OK\n";
}

static void testGlobalStiffnessIsSymmetric() {
    Material steel = Material::structuralSteel(1, 250.0);
    Section sec = Section::rectangular(0.3, 0.5);
    // A skew (non-axis-aligned) member — the case most likely to expose
    // a transformation-matrix bug, since axis-aligned members can
    // accidentally look right even with a broken T.
    FrameElement3D elem({0, 0, 0}, {3, 4, 5}, steel, sec);
    Matrix K = elem.globalStiffness();
    assert(K.isSymmetric(1e-6));
    std::cout << "  testGlobalStiffnessIsSymmetric OK\n";
}

static void testTransformationMatrixIsOrthogonal() {
    // T should be a pure rotation (repeated 4x): T * T^T == I.
    Material steel = Material::structuralSteel(1, 250.0);
    Section sec = Section::rectangular(0.3, 0.5);
    FrameElement3D elem({1, 2, 3}, {5, -1, 7}, steel, sec);
    Matrix T = elem.transformationMatrix();
    Matrix shouldBeI = T * T.transpose();
    Matrix I = Matrix::identity(kFrameDof);
    Matrix diff = shouldBeI - I;
    assert(diff.frobeniusNorm() < 1e-9);
    std::cout << "  testTransformationMatrixIsOrthogonal OK\n";
}

static void testAxialElongationAgainstHandCalc() {
    // Fixed at node1; apply an axial force at node2's free Ux DOF only
    // and solve the reduced 1x1 system directly against the analytical
    // elongation delta = P*L/(E*A) — a textbook mechanics-of-materials
    // result, independent of this element's own bending/torsion code.
    double E_kPa = 200.0e6;  // 200 GPa in kPa
    double A = 0.01;         // m^2
    double L = 3.0;          // m
    double P = 500.0;        // kN
    Material mat(1, "test", MaterialKind::StructuralSteel, E_kPa, 0.3, 78.5);
    Section sec = Section::rectangular(std::sqrt(A), std::sqrt(A));  // area = A exactly
    FrameElement3D elem({0, 0, 0}, {L, 0, 0}, mat, sec);
    Matrix k = elem.localStiffness();
    // Local DOF 6 is node2's Ux (axial); local DOF 0 is node1's Ux
    // (restrained -> removed). Reduced 1x1 system: k(6,6)*u = P.
    double kAxial = k(6, 6);
    double u = P / kAxial;
    double expected = P * L / (E_kPa * A);
    assert(approxEqual(u, expected));
    std::cout << "  testAxialElongationAgainstHandCalc OK\n";
}

static void testCantileverTipDeflectionAgainstHandCalc() {
    // Classic cantilever: fixed at node1, transverse point load P at the
    // free end (node2), deflection along the load direction:
    //   delta = P*L^3 / (3*E*I)
    //   theta = P*L^2 / (2*E*I)   (tip rotation, same sign convention)
    // Reduce the 12x12 local matrix to just node2's [Uy, Rz] DOFs (local
    // indices 7, 11) by solving that 2x2 sub-block directly — node1 is
    // fully fixed so its rows/cols simply don't participate.
    double E_kPa = 25.0e6;   // ~25 GPa, plausible concrete Ec in kPa
    double L = 4.0;
    double Iz = 0.3 * std::pow(0.5, 3) / 12.0;  // 300x500mm rectangular column, strong axis
    Material mat(1, "test", MaterialKind::Concrete, E_kPa, 0.2, 23.6);
    Section sec = Section::rectangular(0.3, 0.5);
    FrameElement3D elem({0, 0, 0}, {L, 0, 0}, mat, sec);
    Matrix k = elem.localStiffness();

    // node2's Uy = local idx 7, Rz = local idx 11.
    Matrix k22(2, 2);
    k22(0, 0) = k(7, 7);   k22(0, 1) = k(7, 11);
    k22(1, 0) = k(11, 7);  k22(1, 1) = k(11, 11);
    double P = 20.0;  // kN transverse load, no applied moment
    std::vector<double> F = {P, 0.0};
    auto u = Matrix::solve(k22, F);

    double expectedDelta = P * std::pow(L, 3) / (3.0 * E_kPa * Iz);
    double expectedTheta = P * L * L / (2.0 * E_kPa * Iz);
    assert(approxEqual(u[0], expectedDelta, 1e-4));
    assert(approxEqual(u[1], expectedTheta, 1e-4));
    std::cout << "  testCantileverTipDeflectionAgainstHandCalc OK\n";
}

static void testTorsionAgainstHandCalc() {
    // theta = T*L / (G*J)
    double E_kPa = 200.0e6;
    double poisson = 0.3;
    double G_kPa = E_kPa / (2.0 * (1.0 + poisson));
    double L = 2.0;
    Material mat(1, "test", MaterialKind::StructuralSteel, E_kPa, poisson, 78.5);
    Section sec = Section::circular(0.2);  // 200mm round shaft, exact polar J
    FrameElement3D elem({0, 0, 0}, {L, 0, 0}, mat, sec);
    Matrix k = elem.localStiffness();
    double kTorsion = k(9, 9);  // node2's Rx, node1 fixed
    double T = 15.0;            // kN*m applied torque
    double theta = T / kTorsion;
    double J = M_PI * std::pow(0.2, 4) / 32.0;
    double expected = T * L / (G_kPa * J);
    assert(approxEqual(theta, expected));
    std::cout << "  testTorsionAgainstHandCalc OK\n";
}

static void testShearCorrectionReducesToEulerBernoulliWhenNoShearArea() {
    // A Section with shearAreaY = shearAreaZ = 0 (the Section default,
    // if a caller never set them) must fall back to pure Euler-Bernoulli
    // — phi = 0 — not silently divide by zero or produce a different
    // (wrong) stiffness.
    Material steel = Material::structuralSteel(1, 250.0);
    Section sec;  // all zero except what we set below
    sec.area = 0.01;
    sec.momentOfInertiaZ = 8.0e-5;
    sec.momentOfInertiaY = 8.0e-5;
    sec.torsionalConstant = 1.0e-5;
    // shearAreaY/Z left at 0 deliberately.
    FrameElement3D elem({0, 0, 0}, {3, 0, 0}, steel, sec);
    Matrix k = elem.localStiffness();
    double L = 3.0;
    double expected_k_uu = 12.0 * steel.elasticModulusKPa() * sec.momentOfInertiaZ / std::pow(L, 3);
    assert(approxEqual(k(1, 1), expected_k_uu));
    std::cout << "  testShearCorrectionReducesToEulerBernoulliWhenNoShearArea OK\n";
}

static void testVerticalMemberDoesNotThrow() {
    // The degenerate case computeLocalAxes() has an explicit fallback
    // for: a perfectly vertical member, where the default reference
    // (global Z) is parallel to the member axis and can't be used to
    // build a perpendicular local z.
    Material conc = Material::concrete(1, 28.0);
    Section sec = Section::rectangular(0.3, 0.3);
    FrameElement3D column({0, 0, 0}, {0, 0, 3.5}, conc, sec);
    Matrix K = column.globalStiffness();
    assert(K.isSymmetric(1e-6));
    // Sanity: a vertical column's global Uz-Uz (index 2,2) stiffness
    // should equal its local axial stiffness EA/L, since local x is
    // exactly global z here.
    double expectedAxial = conc.elasticModulusKPa() * sec.area / 3.5;
    assert(approxEqual(K(2, 2), expectedAxial, 1e-4));
    std::cout << "  testVerticalMemberDoesNotThrow OK\n";
}

static void testZeroLengthThrows() {
    Material steel = Material::structuralSteel(1, 250.0);
    Section sec = Section::rectangular(0.3, 0.3);
    bool threw = false;
    try {
        FrameElement3D bad({1, 1, 1}, {1, 1, 1}, steel, sec);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testZeroLengthThrows OK\n";
}

// THE decisive test for geometricStiffnessLocal(): for a fixed-free
// cantilever, the combined elastic+geometric stiffness at the free end
// becomes singular exactly at the member's buckling load. A SINGLE
// 2-node cubic-Hermite element does NOT give the exact Euler load for
// this support case (that exact match is a property of the
// simply-supported/pinned-pinned case instead, where the true buckled
// shape happens to be reproducible by the assumed cubic shape
// functions) — what IS a well-established, provable property here is
// that a single-element discretization gives an UPPER BOUND on the
// true critical load (a Rayleigh-Ritz consequence: assumed shape
// functions that don't exactly match the true buckling mode always
// overestimate stiffness), converging down toward the exact Euler
// value only with mesh refinement. This test checks BOTH facts: the
// computed critical load is close to (within about 1%) the classical
// Pcr = pi^2*E*I/(4*L^2), AND it is on the correct (over-, not under-)
// side of it — checking the direction of the discrepancy matches
// theory is a stronger check than a bare tolerance alone, since a sign
// error elsewhere could otherwise coincidentally land within the same
// numeric tolerance from the wrong side.
static void testGeometricStiffnessMatchesEulerCantileverBucklingLoad() {
    double E_kPa = 25.0e6, L = 4.0;
    Material conc(1, "test", MaterialKind::Concrete, E_kPa, 0.2, 23.6);
    Section sec;  // zero shear area -> pure Euler-Bernoulli, matching the classic formula exactly
    sec.area = 0.09;
    sec.momentOfInertiaZ = 0.3 * std::pow(0.3, 3) / 12.0;
    sec.momentOfInertiaY = sec.momentOfInertiaZ;
    sec.torsionalConstant = sec.momentOfInertiaZ * 2.0;
    double I = sec.momentOfInertiaZ;

    FrameElement3D elem({0, 0, 0}, {L, 0, 0}, conc, sec);
    Matrix Ke = elem.localStiffness();
    Matrix Kg = elem.geometricStiffnessLocal(1.0);  // unit tension -> scales linearly with P

    // Reduce to node2's (Uy, Rz) 2x2 sub-block — node1 is conceptually
    // fully fixed (this is exactly what the earlier cantilever
    // tip-deflection test in this file also does).
    double Ke11 = Ke(7, 7), Ke12 = Ke(7, 11), Ke22 = Ke(11, 11);
    double Kg11 = Kg(7, 7), Kg12 = Kg(7, 11), Kg22 = Kg(11, 11);

    // det(Ke + P*Kg) = a*P^2 + b*P + c, solved directly (independent of
    // any matrix/eigen code in this library). Both roots come out
    // negative here (compression-destabilizing only — tension never
    // buckles a straight column, so there is no positive root); the
    // physically relevant buckling load is the SMALLER-magnitude one
    // (instability is reached there first as compression increases
    // from zero).
    double a = Kg11 * Kg22 - Kg12 * Kg12;
    double b = Ke11 * Kg22 + Ke22 * Kg11 - 2.0 * Ke12 * Kg12;
    double c = Ke11 * Ke22 - Ke12 * Ke12;
    double disc = std::sqrt(b * b - 4.0 * a * c);
    double pRoot1 = (-b + disc) / (2.0 * a);
    double pRoot2 = (-b - disc) / (2.0 * a);
    assert(pRoot1 < 0.0 && pRoot2 < 0.0);  // both compression, as expected
    double pCompressionRoot = std::abs(pRoot1) < std::abs(pRoot2) ? pRoot1 : pRoot2;

    double eulerPcr = M_PI * M_PI * E_kPa * I / (4.0 * L * L);
    double computedPcr = -pCompressionRoot;
    // Close to the classic value (single-element discretization error,
    // not an exact match — see the function's own doc comment)...
    assert(approxEqual(computedPcr, eulerPcr, 0.01));
    // ...and specifically an OVER-estimate, matching the Rayleigh-Ritz
    // upper-bound property this discretization is known to have.
    assert(computedPcr > eulerPcr);

    std::cout << "  testGeometricStiffnessMatchesEulerCantileverBucklingLoad OK (Pcr="
              << computedPcr << " kN, Euler=" << eulerPcr << " kN, ratio="
              << computedPcr / eulerPcr << ")\n";
}

int main() {
    std::cout << "test_frame_element:\n";
    testLocalStiffnessIsSymmetric();
    testGlobalStiffnessIsSymmetric();
    testTransformationMatrixIsOrthogonal();
    testAxialElongationAgainstHandCalc();
    testCantileverTipDeflectionAgainstHandCalc();
    testTorsionAgainstHandCalc();
    testShearCorrectionReducesToEulerBernoulliWhenNoShearArea();
    testVerticalMemberDoesNotThrow();
    testZeroLengthThrows();
    testGeometricStiffnessMatchesEulerCantileverBucklingLoad();
    std::cout << "All frame element tests passed.\n";
    return 0;
}
