#pragma once

#include "analysis/StaticAnalysis.h"
#include "core/Model.h"

namespace nrsa::analysis {

// Geometrically NONLINEAR static analysis for a linear-ELASTIC
// material -- i.e. this captures true, self-consistent second-order
// (P-Delta) behavior by ITERATING member axial forces to convergence,
// rather than PDeltaAnalysis's single linear pass (which fixes each
// member's axial force from a SEPARATE gravity-only solve and never
// revisits it). MATERIAL nonlinearity -- plastic hinges, concrete
// cracking, a pushover capacity curve -- is explicitly NOT modeled
// here; that would need its own hinge-state/moment-rotation
// formulation layered on top of this, a distinct and substantially
// larger addition (see the README roadmap).
//
// METHOD: fixed-point (successive-substitution) iteration, not a full
// residual-based Newton-Raphson -- a deliberate, documented scope
// choice, the same kind of tradeoff this project has made before (PCG
// over a direct sparse factorization in fem::Solver, Jacobi rotation
// over a more elaborate eigensolver in fem::EigenSolver): starting from
// zero axial force in every frame element, each iteration (1) assembles
// Ke + Kg(current axial forces) exactly like PDeltaAnalysis's single
// pass, (2) solves the FULL applied load (a single load case -- gravity
// and lateral effects together, unlike PDeltaAnalysis's two-case
// split) against that stiffness, (3) recovers each frame element's
// axial force from the new displacement, and (4) checks how much those
// axial forces changed from the previous iteration. This converges
// reliably for STABLE structures (well below their buckling load) --
// for a material that stays linear-elastic, the only nonlinearity is
// this geometric self-consistency loop, which is a much better-
// conditioned fixed point than a general nonlinear residual would be.
// Very close to a bifurcation/buckling point, convergence can slow or
// fail (a thrown, not silent, result -- see run()'s own doc comment); a
// full Newton-Raphson on the residual would be more robust there, and
// is a reasonable future upgrade once a case actually needs it.
struct NonlinearAnalysisOptions {
    double convergenceTolerance = 1e-6;  // relative change in axial force between iterations
    int maxIterations = 50;
};

struct NonlinearAnalysisResult {
    StaticAnalysisResult staticResult;  // final converged displacements/reactions/element forces
    int iterations = 0;
    double finalRelativeChange = 0.0;
};

class NonlinearAnalysis {
public:
    using Options = NonlinearAnalysisOptions;
    using Result = NonlinearAnalysisResult;

    explicit NonlinearAnalysis(Model& model) : model_(model) {}

    // Throws std::runtime_error if the iteration does not converge
    // within maxIterations (most commonly because the load is at or
    // beyond the structure's buckling capacity -- a genuine structural
    // result worth surfacing, not a bug to paper over), or whatever the
    // underlying linear solve throws (see fem::Solver::solve's own doc
    // comment) if any individual iteration's augmented system is itself
    // not positive-definite.
    Result run(int loadCaseId, Options opts = Options{});

private:
    Model& model_;
};

}  // namespace nrsa::analysis
