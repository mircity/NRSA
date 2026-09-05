#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

#include "core/Material.h"
#include "fem/Matrix.h"

namespace nrsa::fem {

inline constexpr int kShellNodes = 4;
inline constexpr int kShellDof = 24;  // 4 nodes x 6 DOF/node (Ux,Uy,Uz,Rx,Ry,Rz)

// A flat, 4-node, rectangular shell element combining:
//   - MEMBRANE (in-plane) stiffness: a standard bilinear isoparametric
//     plane-stress element, 2 Gauss points per direction (exact for a
//     bilinear field on a rectangle);
//   - PLATE BENDING (out-of-plane) stiffness: a Reissner-Mindlin plate
//     element, with the bending (curvature) terms at full 2x2
//     integration and the transverse-shear terms at REDUCED 1x1
//     integration — this "selective reduced integration" (SRI) is what
//     lets a Mindlin element remain accurate for THIN plates, where
//     full integration of the shear terms would over-stiffen the
//     element ("shear locking") as thickness -> 0;
//   - a small artificial DRILLING stiffness on each node's in-plane
//     (normal-axis) rotation, needed because neither the membrane nor
//     the bending formulation above gives that DOF any physical
//     stiffness of its own — without it, a model built only from shell
//     elements (no frame element at any node) would have a singular
//     drilling DOF at every node. This is a standard, well-known
//     stabilization for exactly this reason (see e.g. Cook, Malkus &
//     Plesha, "Concepts and Applications of Finite Element Analysis"),
//     not a hack specific to this implementation, and its magnitude is
//     deliberately kept small relative to the real membrane stiffness
//     so it does not meaningfully affect true structural behavior.
//
// SCOPE: rectangular (in its own plane) elements only — the 4 corners
// must be planar and form a right angle at every corner. A general
// (skewed/trapezoidal) quadrilateral shell needs a proper isoparametric
// Jacobian (non-constant across the element) instead of the constant
// Jacobian a rectangle allows; that is a real, larger addition, not
// implemented here yet (mirrors the same "rectangular bounding box"
// limitation the browser prototype's slab FEM already had — see
// prototype/README.md).
//
// Local-to-global rotation transformation: BOTH the translational
// (Ux,Uy,Uz) and rotational (Rx,Ry,Rz) DOF triples at each node use the
// exact same 3x3 rotation as FrameElement3D's — a physical rotation
// vector transforms the same way a physical displacement vector does
// under a change of coordinate frame, and this element's local rotation
// DOFs are defined as literal rotation-vector components (see the
// bending-strain derivation in addBendingAndShearStiffness below), not
// as "slopes" — this is what keeps a shell element's Rx/Ry/Rz DOFs
// directly compatible with a FrameElement3D's Rx/Ry/Rz at a shared
// node, with no extra sign convention to reconcile when the two are
// assembled together.
class ShellElement {
public:
    // Corners MUST be given in order (either CW or CCW, consistently)
    // around the rectangle — p1->p2 and p1->p4 are treated as the two
    // in-plane edge directions.
    ShellElement(std::array<double, 3> p1, std::array<double, 3> p2,
                 std::array<double, 3> p3, std::array<double, 3> p4,
                 const Material& material, double thickness)
        : p_{p1, p2, p3, p4}, E_(material.elasticModulusKPa()),
          nu_(material.poissonRatio()), t_(thickness) {
        if (t_ <= 0.0) throw std::invalid_argument("ShellElement: thickness must be positive");
        computeLocalAxesAndDimensions();
    }

    double width() const { return a_; }   // local-x extent
    double height() const { return b_; }  // local-y extent
    std::array<double, 3> normal() const { return ez_; }

    // 24x24 stiffness in the element's own local coordinate system, DOF
    // order [node1: u,v,w,rx,ry,rz][node2: ...]...[node4: ...].
    Matrix localStiffness() const {
        Matrix K(kShellDof, kShellDof, 0.0);
        addMembraneStiffness(K);
        addBendingAndShearStiffness(K);
        addDrillingStiffness(K);
        return K;
    }

    // 24x24 block-diagonal rotation (8 repeats of the same 3x3
    // direction-cosine block — one per node's translation triple, one
    // per node's rotation triple) mapping GLOBAL DOFs to LOCAL ones.
    Matrix transformationMatrix() const {
        Matrix T(kShellDof, kShellDof, 0.0);
        Matrix R(3, 3);
        for (int c = 0; c < 3; ++c) {
            R(0, c) = ex_[c];
            R(1, c) = ey_[c];
            R(2, c) = ez_[c];
        }
        for (int block = 0; block < 8; ++block) {
            int off = block * 3;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    T(off + i, off + j) = R(i, j);
        }
        return T;
    }

    Matrix globalStiffness() const {
        Matrix T = transformationMatrix();
        Matrix K = localStiffness();
        return T.transpose() * K * T;
    }

private:
    // ---- Geometry ---------------------------------------------------
    void computeLocalAxesAndDimensions() {
        auto sub = [](const std::array<double, 3>& u, const std::array<double, 3>& v) {
            return std::array<double, 3>{u[0] - v[0], u[1] - v[1], u[2] - v[2]};
        };
        std::array<double, 3> e12 = sub(p_[1], p_[0]);
        std::array<double, 3> e14 = sub(p_[3], p_[0]);
        a_ = norm(e12);
        if (a_ < 1e-9) throw std::invalid_argument("ShellElement: degenerate edge p1-p2");
        ex_ = scale(e12, 1.0 / a_);

        std::array<double, 3> n = cross(e12, e14);
        double nNorm = norm(n);
        if (nNorm < 1e-9) throw std::invalid_argument("ShellElement: corners are collinear/degenerate");
        ez_ = scale(n, 1.0 / nNorm);
        ey_ = cross(ez_, ex_);  // already unit: ez_ and ex_ are orthonormal

        b_ = dot(e14, ey_);
        if (b_ < 1e-9) throw std::invalid_argument("ShellElement: degenerate edge p1-p4");

        // Validate a true rectangle: p1-p4 must have no component along
        // ex_ (i.e. the p1-p4 edge is exactly perpendicular to p1-p2),
        // and p3 must close the rectangle at p1 + a*ex + b*ey. A
        // tolerance relative to the element's own size, not an absolute
        // one, so this check scales correctly for both a small footing
        // and a large slab panel.
        double scale_ = std::max(a_, b_);
        double skew = std::abs(dot(e14, ex_));
        if (skew > 1e-6 * scale_) {
            throw std::invalid_argument(
                "ShellElement: corners do not form a right angle at p1 (skew=" +
                std::to_string(skew) + ") — only rectangular shell elements are supported "
                "in this version (see the class doc comment).");
        }
        std::array<double, 3> expectedP3 = {p_[0][0] + a_ * ex_[0] + b_ * ey_[0],
                                             p_[0][1] + a_ * ex_[1] + b_ * ey_[1],
                                             p_[0][2] + a_ * ex_[2] + b_ * ey_[2]};
        double closureError = norm(sub(p_[2], expectedP3));
        if (closureError > 1e-6 * scale_) {
            throw std::invalid_argument(
                "ShellElement: corners are not coplanar/rectangular (closure error=" +
                std::to_string(closureError) + ") — only rectangular shell elements are "
                "supported in this version (see the class doc comment).");
        }
    }

    // ---- Shape functions and derivatives, at natural coords (xi,eta)
    // in [-1,1]x[-1,1], corners ordered (-1,-1),(1,-1),(1,1),(-1,1) to
    // match p1..p4. Returns {N, dN/dx, dN/dy} for all 4 nodes at once —
    // dN/dx = dN/dxi * (2/a), dN/dy = dN/deta * (2/b), the constant
    // Jacobian a rectangle gives (no per-point Jacobian inversion
    // needed, unlike a general quadrilateral).
    struct ShapeData {
        std::array<double, 4> N, dNdx, dNdy;
    };
    ShapeData shapeFunctions(double xi, double eta) const {
        ShapeData s;
        double signsXi[4] = {-1, 1, 1, -1};
        double signsEta[4] = {-1, -1, 1, 1};
        for (int i = 0; i < 4; ++i) {
            double sx = signsXi[i], se = signsEta[i];
            s.N[i] = 0.25 * (1 + sx * xi) * (1 + se * eta);
            double dNdxi = 0.25 * sx * (1 + se * eta);
            double dNdeta = 0.25 * se * (1 + sx * xi);
            s.dNdx[i] = dNdxi * (2.0 / a_);
            s.dNdy[i] = dNdeta * (2.0 / b_);
        }
        return s;
    }

    // ---- Membrane (plane-stress) stiffness, 2x2 full Gauss ------------
    void addMembraneStiffness(Matrix& K) const {
        double Dfac = E_ / (1.0 - nu_ * nu_);
        Matrix D(3, 3, 0.0);
        D(0, 0) = Dfac;        D(0, 1) = Dfac * nu_;
        D(1, 0) = Dfac * nu_;  D(1, 1) = Dfac;
        D(2, 2) = Dfac * (1.0 - nu_) / 2.0;

        double detJ = (a_ / 2.0) * (b_ / 2.0);
        const double gp = 1.0 / std::sqrt(3.0);
        const double pts[2] = {-gp, gp};

        // Local index within the 24-DOF element for node i's u, v.
        auto uIdx = [](int i) { return i * 6 + 0; };
        auto vIdx = [](int i) { return i * 6 + 1; };

        for (double xi : pts) {
            for (double eta : pts) {
                ShapeData s = shapeFunctions(xi, eta);
                Matrix B(3, 8, 0.0);
                for (int i = 0; i < 4; ++i) {
                    B(0, 2 * i) = s.dNdx[i];
                    B(1, 2 * i + 1) = s.dNdy[i];
                    B(2, 2 * i) = s.dNdy[i];
                    B(2, 2 * i + 1) = s.dNdx[i];
                }
                Matrix BtDB = B.transpose() * D * B;  // 8x8
                double w = 1.0 * 1.0 * detJ * t_;      // Gauss weight (1 each) * detJ * thickness
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        K(uIdx(i), uIdx(j)) += w * BtDB(2 * i, 2 * j);
                        K(uIdx(i), vIdx(j)) += w * BtDB(2 * i, 2 * j + 1);
                        K(vIdx(i), uIdx(j)) += w * BtDB(2 * i + 1, 2 * j);
                        K(vIdx(i), vIdx(j)) += w * BtDB(2 * i + 1, 2 * j + 1);
                    }
                }
            }
        }
    }

    // ---- Plate bending (curvature, full 2x2) + transverse shear
    // (reduced 1x1) stiffness — Reissner-Mindlin, local DOFs per node
    // (w, rx, ry) where rx/ry are literal rotation-vector components
    // (see class doc comment). Derivation: a physical rotation vector
    // (rx,ry,rz) about the local origin displaces a point offset
    // (0,0,zeta) from the mid-surface by (ry*zeta, -rx*zeta, 0) —
    // standard rigid-rotation kinematics (rotation x offset), which
    // IS the Mindlin plate kinematic assumption once zeta-independence
    // of w is added. Differentiating that in-plane displacement field
    // gives the curvatures below; adding d(w)/dx, d(w)/dy gives the
    // transverse shear strains.
    void addBendingAndShearStiffness(Matrix& K) const {
        double Dfac = (t_ * t_ * t_ / 12.0) * E_ / (1.0 - nu_ * nu_);
        Matrix Db(3, 3, 0.0);
        Db(0, 0) = Dfac;        Db(0, 1) = Dfac * nu_;
        Db(1, 0) = Dfac * nu_;  Db(1, 1) = Dfac;
        Db(2, 2) = Dfac * (1.0 - nu_) / 2.0;

        double G = E_ / (2.0 * (1.0 + nu_));
        const double shearCorrectionFactor = 5.0 / 6.0;
        double Ds_diag = shearCorrectionFactor * G * t_;
        Matrix Ds(2, 2, 0.0);
        Ds(0, 0) = Ds_diag;
        Ds(1, 1) = Ds_diag;

        double detJ = (a_ / 2.0) * (b_ / 2.0);

        auto wIdx = [](int i) { return i * 6 + 2; };
        auto rxIdx = [](int i) { return i * 6 + 3; };
        auto ryIdx = [](int i) { return i * 6 + 4; };

        // --- Bending (curvature) terms: full 2x2 Gauss.
        {
            const double gp = 1.0 / std::sqrt(3.0);
            const double pts[2] = {-gp, gp};
            for (double xi : pts) {
                for (double eta : pts) {
                    ShapeData s = shapeFunctions(xi, eta);
                    Matrix Bb(3, 12, 0.0);
                    for (int i = 0; i < 4; ++i) {
                        int rxc = 3 * i + 1, ryc = 3 * i + 2;
                        Bb(0, ryc) = s.dNdx[i];                          // kx = d(ry)/dx
                        Bb(1, rxc) = -s.dNdy[i];                         // ky = -d(rx)/dy
                        Bb(2, rxc) = -s.dNdx[i]; Bb(2, ryc) = s.dNdy[i];  // kxy
                    }
                    Matrix BtDB = Bb.transpose() * Db * Bb;  // 12x12, ordered (w,rx,ry) per node
                    double w = detJ;
                    scatterBendingBlock(K, BtDB, w, wIdx, rxIdx, ryIdx);
                }
            }
        }
        // --- Shear terms: REDUCED 1x1 Gauss (the SRI step that avoids
        // shear locking as t -> 0; see class doc comment).
        {
            ShapeData s = shapeFunctions(0.0, 0.0);
            Matrix Bs(2, 12, 0.0);
            for (int i = 0; i < 4; ++i) {
                int wc = 3 * i, rxc = 3 * i + 1, ryc = 3 * i + 2;
                Bs(0, wc) = s.dNdx[i]; Bs(0, ryc) = s.N[i];   // gxz = dw/dx + ry
                Bs(1, wc) = s.dNdy[i]; Bs(1, rxc) = -s.N[i];  // gyz = dw/dy - rx
            }
            Matrix BtDB = Bs.transpose() * Ds * Bs;
            double w = 4.0 * detJ;  // 1-point rule weight (2*2) * detJ
            scatterBendingBlock(K, BtDB, w, wIdx, rxIdx, ryIdx);
        }
    }

    template <typename WIdx, typename RxIdx, typename RyIdx>
    static void scatterBendingBlock(Matrix& K, const Matrix& BtDB, double weight,
                                     WIdx wIdx, RxIdx rxIdx, RyIdx ryIdx) {
        // BtDB is 12x12 with LOCAL sub-index order (w,rx,ry) per node —
        // map each of those 3 sub-indices to this node's actual global-
        // within-element index (wIdx/rxIdx/ryIdx) for the scatter.
        auto mapIdx = [&](int i, int sub) {
            return sub == 0 ? wIdx(i) : sub == 1 ? rxIdx(i) : ryIdx(i);
        };
        for (int i = 0; i < 4; ++i) {
            for (int si = 0; si < 3; ++si) {
                int gi = mapIdx(i, si);
                for (int j = 0; j < 4; ++j) {
                    for (int sj = 0; sj < 3; ++sj) {
                        int gj = mapIdx(j, sj);
                        K(gi, gj) += weight * BtDB(3 * i + si, 3 * j + sj);
                    }
                }
            }
        }
    }

    // ---- Drilling (normal-axis rotation) stabilization ----------------
    void addDrillingStiffness(Matrix& K) const {
        // Average membrane diagonal stiffness as the reference scale,
        // then apply a small fraction of it to each node's drilling
        // DOF — large enough to remove the singularity, small enough
        // to negligibly affect real behavior (standard practice; see
        // class doc comment).
        double avgMembraneDiag = 0.0;
        for (int i = 0; i < 4; ++i) avgMembraneDiag += K(i * 6 + 0, i * 6 + 0);
        avgMembraneDiag /= 4.0;
        const double drillingFactor = 1.0e-4;
        double kDrill = drillingFactor * avgMembraneDiag;
        for (int i = 0; i < 4; ++i) K(i * 6 + 5, i * 6 + 5) += kDrill;
    }

    static std::array<double, 3> cross(const std::array<double, 3>& a, const std::array<double, 3>& b) {
        return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }
    static double dot(const std::array<double, 3>& a, const std::array<double, 3>& b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }
    static double norm(const std::array<double, 3>& v) { return std::sqrt(dot(v, v)); }
    static std::array<double, 3> scale(const std::array<double, 3>& v, double s) {
        return {v[0] * s, v[1] * s, v[2] * s};
    }

    std::array<std::array<double, 3>, 4> p_;
    double E_, nu_, t_;
    double a_ = 0.0, b_ = 0.0;
    std::array<double, 3> ex_{}, ey_{}, ez_{};
};

}  // namespace nrsa::fem
