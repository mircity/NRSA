#pragma once

#include <functional>
#include <unordered_map>
#include <vector>

#include "analysis/StaticAnalysis.h"  // NodeVector6
#include "core/Model.h"

namespace nrsa::analysis {

// Direct time-integration (Newmark-beta, constant-average-acceleration
// -- beta=1/4, gamma=1/2, UNCONDITIONALLY STABLE for a linear system,
// the standard default every commercial package uses) of the full
// equation of motion:
//     M*a(t) + C*v(t) + K*d(t) = F(t)
//
// This ONE module deliberately covers TWO Roadmap line items --
// "Time History" and general "Dynamic Analysis" -- because they are
// the SAME numerical method applied to two different forcing inputs:
//   - "Time History" (seismic): F(t) = EFFECTIVE EARTHQUAKE FORCE
//     built from a ground-acceleration record via buildSeismicForceFn()
//     below (F_eff(t) = -M * r * ag(t), r = unit influence vector in
//     the excited direction -- the standard formulation, e.g. Chopra
//     "Dynamics of Structures" Ch. 13).
//   - general "Dynamic Analysis": F(t) = any explicitly-supplied,
//     directly time-varying NODAL force (impact, machine vibration,
//     blast pressure-time history, etc.) via a caller-supplied
//     ForcingFunction.
// Both go through the exact same runNewmarkBeta() below; only how you
// build the ForcingFunction differs. Splitting these into two separate
// classes would mean two copies of the same 40-line integration loop.
//
// DAMPING: Rayleigh (mass- and stiffness-proportional) damping,
// C = alphaR*M + betaR*K -- the standard form every commercial package
// defaults to because it lets a SPARSE, DIAGONAL-plus-K-scaled
// effective stiffness be assembled once and reused every timestep
// (this module builds K and M once and never needs a separate C
// matrix in memory -- see .cpp). rayleighCoefficients() below derives
// (alphaR, betaR) from a target damping ratio at two reference
// frequencies, the usual way of picking Rayleigh coefficients in
// practice (e.g. matching 5% damping at the first and a higher mode).
using ForcingFunction = std::function<std::vector<double>(double timeSeconds, int freeDofCount)>;

struct RayleighCoefficients {
    double alphaMass = 0.0;
    double betaStiffness = 0.0;
};

// Solves for (alphaR, betaR) such that the damping ratio at BOTH
// omega1 and omega2 (rad/s) equals dampingRatio -- the standard
// 2-equation Rayleigh-damping fit (Chopra Ch. 11):
//   xi_i = alphaR/(2*omega_i) + betaR*omega_i/2
RayleighCoefficients rayleighCoefficients(double omega1, double omega2, double dampingRatio);

// One time step's full state, expanded to per-node NodeVector6 the
// same way StaticAnalysisResult::displacements is shaped.
struct DynamicTimeStep {
    double time = 0.0;
    std::unordered_map<int, NodeVector6> displacement;
    std::unordered_map<int, NodeVector6> velocity;
    std::unordered_map<int, NodeVector6> acceleration;
};

struct DynamicAnalysisResult {
    std::vector<DynamicTimeStep> steps;  // includes t=0 (initial conditions) as steps[0]
    std::vector<int> skippedElementIds;
};

struct DynamicAnalysisOptions {
    double dtSeconds = 0.01;
    double totalDurationSeconds = 10.0;
    RayleighCoefficients damping;
    // Rotational-mass regularization, same meaning and same default as
    // ModalAnalysis::Options::rotationalMassRegularization -- a free
    // rotational DOF with no explicitly assigned rotational mass would
    // otherwise be undefined for a mass-matrix-based dynamic solve.
    double rotationalMassRegularization = 1e-6;
};

// Runs the Newmark-beta integration from rest (d0 = v0 = 0) under the
// given forcing function, sampling every dtSeconds up to
// totalDurationSeconds. Throws std::runtime_error under the same
// conditions ModalAnalysis::run does (zero free DOFs, or no node with
// positive translational mass).
DynamicAnalysisResult runNewmarkBeta(Model& model, const ForcingFunction& forcing,
                                      const DynamicAnalysisOptions& options);

// Builds the standard effective-seismic-force ForcingFunction from a
// ground-ACCELERATION record (m/s^2, evenly sampled at recordDt
// seconds apart, held at its last value beyond the record's own
// duration): F_eff(t) = -M * r * ag(t), r = 1 at every free
// translational DOF in the excited direction (true=X, false=Y), 0
// elsewhere -- the model's own translationalMass() per node supplies
// M, matching exactly what runNewmarkBeta's own mass-vector assembly
// uses, so the two are always consistent with each other.
ForcingFunction buildSeismicForceFn(const Model& model, const std::vector<double>& groundAccelMs2,
                                     double recordDt, bool directionX);

}  // namespace nrsa::analysis
