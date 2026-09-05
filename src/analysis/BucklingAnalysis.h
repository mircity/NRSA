#pragma once

#include <vector>

#include "analysis/StaticAnalysis.h"
#include "core/Model.h"
#include "fem/EigenSolver.h"
#include "fem/Matrix.h"

namespace nrsa::analysis {

// LINEAR (eigenvalue) buckling analysis: the standard small-deflection
// idealization used to estimate a structure's elastic critical load --
// e.g. checking a frame's effective-length/K-factor assumptions, or
// getting a first-pass sense of how far a design sits from instability
// under a given reference load pattern. This is NOT a full nonlinear
// (large-displacement, load-stepped) buckling/collapse analysis -- see
// analysis::NonlinearAnalysis for that -- it is the classical
// generalized-eigenproblem approach every mainstream structural
// package (SAP2000, ETABS, STAAD, ...) offers as "Buckling Analysis":
//
//   1. Run an ordinary StaticAnalysis of a REFERENCE load case (e.g.
//      1.0D + 1.0L) to recover each frame element's axial force, then
//      form its geometric stiffness Kg_ref (fem::FrameElement3D::
//      geometricStiffnessGlobal) at that force -- exactly PDeltaAnalysis's
//      Pass 1, reused verbatim rather than re-derived here.
//   2. Because geometricStiffnessGlobal(P) is LINEAR in the axial force
//      P, scaling the reference load by a factor lambda scales Kg_ref
//      by that same lambda. The structure buckles at whatever lambda
//      makes the augmented stiffness singular:
//          det(Ke + lambda * Kg_ref) = 0
//      i.e. the generalized eigenproblem Ke*phi = -lambda*Kg_ref*phi.
//   3. Solved here via a Cholesky congruence transform (Ke = L*L^T,
//      C = L^-1 * Kg_ref * L^-T) reducing it to the STANDARD symmetric
//      eigenproblem C*psi = gamma*psi that fem::EigenSolver::
//      solveSymmetric diagonalizes; lambda = -1/gamma for each
//      negative gamma (see BucklingAnalysis.cpp for the derivation).
//      The smallest positive lambda -- the governing (first) buckling
//      mode -- is EigenSolver's most-negative gamma, i.e. its FIRST
//      returned eigenvalue (ascending order).
//
// VERIFICATION: for a single pin-ended prismatic column under pure
// axial load, this reduces to the classical Euler formula
// Pcr = pi^2*E*I / L^2 -- tests/test_buckling_analysis.cpp checks the
// computed load factor against that closed form directly, the same
// "hand-calc / classical-mechanism" verification standard the
// P-Delta/Pushover/Settlement modules were held to.
//
// SCOPE: inherits every scope limitation PDeltaAnalysis documents --
// frame elements only contribute geometric stiffness (Wall/Slab do
// not yet), and the reference axial force per frame element is a
// single representative value (average of the two, algebraically
// equal-and-opposite end values for a 2-force member with no
// distributed axial load). A model whose CRITICAL member is a wall or
// carries meaningful distributed axial load along its own length will
// not have that captured yet -- the same explicitly-named gap, not a
// silently swallowed one.
struct BucklingMode {
    double loadFactor;               // lambda: reference load must scale by this to buckle
    std::vector<double> shapeFreeDof;  // reduced (free-DOF-only) buckled shape, arbitrary scale
};

struct BucklingAnalysisResult {
    // Ascending by loadFactor (mode 1 = governing/lowest buckling mode
    // first) -- ONLY modes with a genuine positive load factor are
    // included; a generalized-eigenproblem branch with no positive
    // lambda (the reference load pattern puts that mode's members in
    // net tension, which never buckles) is omitted rather than reported
    // as a meaningless negative or infinite number.
    std::vector<BucklingMode> modes;

    // Same meaning as StaticAnalysisResult::skippedElementIds.
    std::vector<int> skippedElementIds;
};

struct BucklingAnalysisOptions {
    // How many of the lowest modes to keep in the result (the
    // dense Jacobi solve underneath always computes all n regardless --
    // this only trims what's returned). 0 = keep all positive-lambda
    // modes found.
    int modesRequested = 4;
    fem::EigenSolverOptions eigenOptions{};
};

class BucklingAnalysis {
public:
    using Options = BucklingAnalysisOptions;

    explicit BucklingAnalysis(Model& model) : model_(model) {}

    // referenceLoadCaseId: the load case whose scaled multiple is
    // sought at buckling (typically a gravity combination such as
    // 1.0D + 1.0L, matching how PDeltaAnalysis's axialLoadCaseId is
    // normally used).
    //
    // Throws whatever the underlying StaticAnalysis solve throws, plus
    // std::runtime_error if the model has zero free DOFs or if the
    // assembled elastic stiffness is not positive-definite (an
    // unrestrained/mechanism model -- see Matrix::choleskyLower).
    BucklingAnalysisResult run(int referenceLoadCaseId, Options opts = Options{});

private:
    Model& model_;
};

}  // namespace nrsa::analysis
