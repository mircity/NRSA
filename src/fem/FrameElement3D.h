#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

#include "core/Material.h"
#include "core/Section.h"
#include "fem/Matrix.h"

namespace nrsa::fem {

// DOF order within this element's 12x12 local/global matrices:
// node1[Ux,Uy,Uz,Rx,Ry,Rz], node2[Ux,Uy,Uz,Rx,Ry,Rz] — the same six-per-
// node order core::DOF uses, just concatenated for two nodes, so
// scattering into a global assembly is a direct index lookup through
// each node's core::Node::dofIndices().
inline constexpr int kFrameDof = 12;

// The two-node, 12-DOF, straight, prismatic 3D frame (beam-column)
// element — axial, torsion, and biaxial bending, each independently
// uncoupled in LOCAL coordinates (the standard Euler-Bernoulli/
// Timoshenko frame element; no coupling term appears in the classic
// formulation because axial/torsion/bending are independent for a
// straight prismatic member under linear-elastic theory). Optional
// Timoshenko shear-deformation correction is applied automatically
// whenever the Section supplies nonzero shear areas; a Section with
// shearAreaY = shearAreaZ = 0 degrades cleanly to pure Euler-Bernoulli
// (phi = 0), so callers never need to choose a beam theory explicitly —
// it falls out of whether the Section was built with shear areas.
class FrameElement3D {
public:
    // p1, p2: node coordinates in the global system, meters.
    // refVector: a point not on the member's own axis, used only to fix
    // rotation of the local y/z axes about the local x (member) axis —
    // exactly the "local axis reference vector" role
    // core::Element::localAxisReference() plays. If refVector is the
    // zero vector (the default), the standard SAP2000/ETABS-style
    // convention is used instead: local z points toward global Z unless
    // the member itself is vertical, in which case local z points
    // toward global X.
    FrameElement3D(std::array<double, 3> p1, std::array<double, 3> p2,
                    const Material& material, const Section& section,
                    std::array<double, 3> refVector = {0.0, 0.0, 0.0})
        : p1_(p1), p2_(p2), E_(material.elasticModulusKPa()), G_(material.shearModulusKPa()),
          A_(section.area), Iy_(section.momentOfInertiaY), Iz_(section.momentOfInertiaZ),
          J_(section.torsionalConstant), Asy_(section.shearAreaY), Asz_(section.shearAreaZ) {
        double dx = p2_[0] - p1_[0], dy = p2_[1] - p1_[1], dz = p2_[2] - p1_[2];
        L_ = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (L_ < 1e-9) {
            throw std::invalid_argument("FrameElement3D: zero-length member (p1 == p2)");
        }
        computeLocalAxes(refVector);
    }

    double length() const { return L_; }
    std::array<double, 3> localAxisX() const { return ex_; }
    std::array<double, 3> localAxisY() const { return ey_; }
    std::array<double, 3> localAxisZ() const { return ez_; }

    // 12x12 stiffness in the element's own LOCAL coordinate system.
    // Standard uncoupled-DOF frame stiffness (see e.g. Przemieniecki,
    // "Theory of Matrix Structural Analysis", Ch. 6) with the Timoshenko
    // shear correction factors phi_y, phi_z folded into the bending
    // terms (both reduce to the classic Euler-Bernoulli coefficients
    // when phi = 0, i.e. when the Section has no shear area set).
    Matrix localStiffness() const {
        Matrix k(kFrameDof, kFrameDof, 0.0);
        double L = L_, L2 = L * L, L3 = L2 * L;

        // Axial (Ux at each end).
        double kAxial = E_ * A_ / L;
        k(0, 0) = kAxial;  k(0, 6) = -kAxial;
        k(6, 0) = -kAxial; k(6, 6) = kAxial;

        // Torsion (Rx at each end).
        double kTorsion = G_ * J_ / L;
        k(3, 3) = kTorsion;  k(3, 9) = -kTorsion;
        k(9, 3) = -kTorsion; k(9, 9) = kTorsion;

        // Bending about local z (deflection in local y, i.e. Uy/Rz DOFs)
        // — shear correction phi_z = 12*E*Iz / (G*Asy*L^2), zero if Asy
        // wasn't set on the Section.
        double phiZ = (Asy_ > 0.0) ? (12.0 * E_ * Iz_) / (G_ * Asy_ * L2) : 0.0;
        addBendingBlock(k, /*uIdx1=*/1, /*rIdx1=*/5, /*uIdx2=*/7, /*rIdx2=*/11,
                         E_ * Iz_, L, L2, L3, phiZ, /*sign=*/+1.0);

        // Bending about local y (deflection in local z, i.e. Uz/Ry DOFs)
        // — shear correction phi_y = 12*E*Iy / (G*Asz*L^2). Sign
        // convention flips relative to the z-bending block because a
        // positive Ry (right-hand rule about local y) and a positive Uz
        // are related with the opposite handedness that Rz/Uy have —
        // the standard result every 3D frame-element derivation carries
        // (see Przemieniecki Ch. 6, or McGuire/Gallagher/Ziemian
        // "Matrix Structural Analysis" Ch. 5).
        double phiY = (Asz_ > 0.0) ? (12.0 * E_ * Iy_) / (G_ * Asz_ * L2) : 0.0;
        addBendingBlock(k, /*uIdx1=*/2, /*rIdx1=*/4, /*uIdx2=*/8, /*rIdx2=*/10,
                         E_ * Iy_, L, L2, L3, phiY, /*sign=*/-1.0);

        return k;
    }

    // The GEOMETRIC (P-Delta) stiffness contribution, local coordinates
    // — the standard consistent geometric stiffness matrix for a
    // 2-node beam-column element with cubic (Hermite) transverse
    // deflection shape functions (see e.g. Przemieniecki Ch. 8, or
    // McGuire/Gallagher/Ziemian Ch. 12). axialForceTensionPositive is
    // exactly what its name says: POSITIVE for tension, NEGATIVE for
    // compression — the opposite convention from
    // analysis::FrameEndForces::axial1 (documented there as positive =
    // compression), so a caller pulling P from that struct must negate
    // it first; see analysis::PDeltaAnalysis for where that conversion
    // happens.
    //
    // With this sign convention, adding this matrix to localStiffness()
    // makes a tension-loaded member STIFFER (a taut cable effect) and a
    // compression-loaded member SOFTER (the destabilizing P-Delta
    // effect, eventually reaching zero net lateral stiffness at the
    // member's buckling load) — the physically correct direction for
    // both cases falls out of the SAME formula and sign convention, not
    // two different cases to get right independently. Identical in
    // both bending planes (Uy/Rz and Uz/Ry) — unlike localStiffness()'s
    // elastic bending block, there is no right-hand-rule handedness
    // flip between planes here, since P-Delta softening is a symmetric,
    // sign-consistent effect in both transverse directions.
    Matrix geometricStiffnessLocal(double axialForceTensionPositive) const {
        Matrix kg(kFrameDof, kFrameDof, 0.0);
        addGeometricBlock(kg, /*uIdx1=*/1, /*rIdx1=*/5, /*uIdx2=*/7, /*rIdx2=*/11,
                           axialForceTensionPositive, L_);
        addGeometricBlock(kg, /*uIdx1=*/2, /*rIdx1=*/4, /*uIdx2=*/8, /*rIdx2=*/10,
                           axialForceTensionPositive, L_);
        return kg;
    }

    // Same idea as globalStiffness(), for the geometric stiffness.
    Matrix geometricStiffnessGlobal(double axialForceTensionPositive) const {
        Matrix T = transformationMatrix();
        Matrix kg = geometricStiffnessLocal(axialForceTensionPositive);
        return T.transpose() * kg * T;
    }

    // 12x12 block-diagonal rotation matrix (four repeats of the same
    // 3x3 direction-cosine block, one per translational+rotational
    // triple) mapping GLOBAL displacement DOFs to LOCAL ones:
    // {u}_local = [T]{u}_global. Global stiffness is then
    // [T]^T [k_local] [T].
    Matrix transformationMatrix() const {
        Matrix T(kFrameDof, kFrameDof, 0.0);
        Matrix R(3, 3);
        for (int c = 0; c < 3; ++c) {
            R(0, c) = ex_[c];
            R(1, c) = ey_[c];
            R(2, c) = ez_[c];
        }
        for (int block = 0; block < 4; ++block) {
            int off = block * 3;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    T(off + i, off + j) = R(i, j);
        }
        return T;
    }

    // The element's stiffness in the GLOBAL coordinate system, ready to
    // be scattered into a structure-level assembly using each end
    // node's global DOF indices.
    Matrix globalStiffness() const {
        Matrix T = transformationMatrix();
        Matrix k = localStiffness();
        // Explicitly (T^T k T) rather than assuming any shortcut —
        // correctness over cleverness for a piece this foundational.
        return T.transpose() * k * T;
    }

private:
    // Builds one bending plane's four 4-entry stiffness sub-block
    // (translation-translation, translation-rotation, rotation-rotation
    // at both ends) directly into the appropriate rows/cols of the full
    // 12x12 matrix, so the two bending planes (z-bending and y-bending)
    // share one implementation instead of duplicating the algebra with
    // just the sign/index differences hand-copied (and liable to drift
    // out of sync under a future edit).
    static void addBendingBlock(Matrix& k, int uIdx1, int rIdx1, int uIdx2, int rIdx2,
                                 double EI, double L, double L2, double L3,
                                 double phi, double sign) {
        double denom = 1.0 + phi;
        double k_uu = 12.0 * EI / (L3 * denom);
        double k_ur = 6.0 * EI / (L2 * denom);
        double k_rr_same = (4.0 + phi) * EI / (L * denom);
        double k_rr_cross = (2.0 - phi) * EI / (L * denom);

        // Translation-translation.
        k(uIdx1, uIdx1) += k_uu;   k(uIdx1, uIdx2) += -k_uu;
        k(uIdx2, uIdx1) += -k_uu;  k(uIdx2, uIdx2) += k_uu;

        // Translation-rotation (sign flips the coupling term only — the
        // pure translation-translation and rotation-rotation terms are
        // identical between the two bending planes; only how rotation
        // couples to translation differs by handedness).
        k(uIdx1, rIdx1) += sign * k_ur;  k(rIdx1, uIdx1) += sign * k_ur;
        k(uIdx1, rIdx2) += sign * k_ur;  k(rIdx2, uIdx1) += sign * k_ur;
        k(uIdx2, rIdx1) += -sign * k_ur; k(rIdx1, uIdx2) += -sign * k_ur;
        k(uIdx2, rIdx2) += -sign * k_ur; k(rIdx2, uIdx2) += -sign * k_ur;

        // Rotation-rotation.
        k(rIdx1, rIdx1) += k_rr_same;  k(rIdx1, rIdx2) += k_rr_cross;
        k(rIdx2, rIdx1) += k_rr_cross; k(rIdx2, rIdx2) += k_rr_same;
    }

    // The standard consistent geometric stiffness 4x4 block (see
    // geometricStiffnessLocal's own doc comment for the sign
    // convention and physical interpretation), embedded into the same
    // translation/rotation index pattern addBendingBlock uses — but
    // with a single, sign-symmetric form shared by BOTH bending planes
    // (no "sign" parameter needed here, unlike addBendingBlock).
    static void addGeometricBlock(Matrix& kg, int uIdx1, int rIdx1, int uIdx2, int rIdx2,
                                   double P, double L) {
        double k_uu = (6.0 * P) / (5.0 * L);
        double k_ur = P / 10.0;
        double k_rr_same = (2.0 * P * L) / 15.0;
        double k_rr_cross = -(P * L) / 30.0;

        kg(uIdx1, uIdx1) += k_uu;   kg(uIdx1, uIdx2) += -k_uu;
        kg(uIdx2, uIdx1) += -k_uu;  kg(uIdx2, uIdx2) += k_uu;

        kg(uIdx1, rIdx1) += k_ur;  kg(rIdx1, uIdx1) += k_ur;
        kg(uIdx1, rIdx2) += k_ur;  kg(rIdx2, uIdx1) += k_ur;
        kg(uIdx2, rIdx1) += -k_ur; kg(rIdx1, uIdx2) += -k_ur;
        kg(uIdx2, rIdx2) += -k_ur; kg(rIdx2, uIdx2) += -k_ur;

        kg(rIdx1, rIdx1) += k_rr_same;  kg(rIdx1, rIdx2) += k_rr_cross;
        kg(rIdx2, rIdx1) += k_rr_cross; kg(rIdx2, rIdx2) += k_rr_same;
    }

    void computeLocalAxes(std::array<double, 3> refVector) {
        double dx = p2_[0] - p1_[0], dy = p2_[1] - p1_[1], dz = p2_[2] - p1_[2];
        ex_ = normalize({dx, dy, dz});

        std::array<double, 3> ref = refVector;
        bool haveRef = (ref[0] != 0.0 || ref[1] != 0.0 || ref[2] != 0.0);
        if (!haveRef) {
            // Default convention: is the member (near enough) vertical?
            // If so global Z is parallel/antiparallel to ex_ and can't
            // be used to build a perpendicular axis, so fall back to
            // global X as the reference instead.
            bool nearVertical = std::abs(ex_[0]) < 1e-6 && std::abs(ex_[1]) < 1e-6;
            ref = nearVertical ? std::array<double, 3>{1.0, 0.0, 0.0}
                                : std::array<double, 3>{0.0, 0.0, 1.0};
        }
        // ez = ex cross ref (then normalized); ey = ez cross ex — this
        // ordering is what keeps {ex, ey, ez} a right-handed set with ez
        // ending up close to the reference direction, matching the
        // convention most structural analysis packages (and the local
        // z = "up-ish" convention used in the earlier browser prototype
        // for a wall's out-of-plane axis) use for a horizontal member.
        std::array<double, 3> ezRaw = cross(ex_, ref);
        double ezNorm = norm(ezRaw);
        if (ezNorm < 1e-9) {
            // Reference vector was parallel to the member axis after
            // all (degenerate input) — fall back to global X, then Y,
            // whichever isn't parallel to ex_.
            std::array<double, 3> altRef = (std::abs(ex_[0]) < 0.99)
                ? std::array<double, 3>{1.0, 0.0, 0.0}
                : std::array<double, 3>{0.0, 1.0, 0.0};
            ezRaw = cross(ex_, altRef);
        }
        ez_ = normalize(ezRaw);
        ey_ = normalize(cross(ez_, ex_));
    }

    static std::array<double, 3> cross(const std::array<double, 3>& a, const std::array<double, 3>& b) {
        return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }
    static double norm(const std::array<double, 3>& v) {
        return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }
    static std::array<double, 3> normalize(const std::array<double, 3>& v) {
        double n = norm(v);
        if (n < 1e-12) throw std::invalid_argument("FrameElement3D: cannot normalize a zero vector");
        return {v[0] / n, v[1] / n, v[2] / n};
    }

    std::array<double, 3> p1_, p2_;
    double E_, G_, A_, Iy_, Iz_, J_, Asy_, Asz_;
    double L_ = 0.0;
    std::array<double, 3> ex_{}, ey_{}, ez_{};
};

}  // namespace nrsa::fem
