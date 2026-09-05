#pragma once

#include <vector>

#include "core/Load.h"
#include "core/Model.h"

namespace nrsa::analysis {

// Long-term concrete effects, ACI 318-19 §24.5 / BNBC 2020 §6.1
// treatment: CREEP (modifies stiffness, via an effective/reduced
// modulus) and SHRINKAGE (an equivalent-nodal-load problem
// mathematically IDENTICAL to TemperatureAnalysis's uniform initial
// axial strain -- shrinkage strain plays exactly the role alpha*deltaT
// plays there, just with a directly-given strain instead of a
// temperature times a coefficient; see
// analysis::axialForceFromInitialStrain, reused here rather than
// re-derived).
//
// SCOPE: age-adjusted effective modulus method (ACI 209R-92 style, the
// standard hand/spreadsheet-level long-term analysis) -- NOT a
// step-by-step time-integrated creep model (which would track stress
// history and apply the creep integral incrementally, needed for
// structures with significant stress redistribution over time, e.g.
// segmentally-constructed bridges). Adequate for a first-pass
// long-term deflection/force estimate on an otherwise-linear model;
// the step-by-step method is a natural next addition, same
// "explicit, temporary scope limit" pattern used throughout this
// project.
struct CreepShrinkageResult {
    int elementId;
    double axialForceMagnitude = 0.0;  // kN, shrinkage-equivalent force pair (same sign convention as TemperatureAnalysis: positive shrinkage strain pulls the ends together)
};

// Effective (age-adjusted) modulus for sustained-load creep deformation,
// ACI 209R-92 / CEB-FIP model code form: E_eff = E / (1 + phi), where
// phi is the creep coefficient (ratio of creep strain to initial
// elastic strain) at the age/duration of interest. A caller wanting a
// long-term STIFFNESS analysis builds a second Material with this
// reduced modulus and assigns it to the relevant elements for that
// run -- this function computes the number, it does not mutate the
// Model (Material is shared/referenced by id across elements, so
// silently mutating one in place would silently change every OTHER
// element using that same Material too).
double effectiveCreepModulus(double E_kPa, double creepCoefficient);

// Applies a uniform shrinkage strain (dimensionless, e.g. -0.0004 for
// a typical -400 microstrain BNBC/ACI estimate -- NEGATIVE because
// shrinkage is a strain-inducing SHORTENING, the opposite sign
// convention from a temperature RISE) to every 2-node frame element in
// elementIds, appending the resulting self-equilibrated NodalLoad pair
// under loadCaseId. Internally this is exactly
// applyUniformTemperatureChange with eps0 = shrinkageStrain directly
// (no alpha/deltaT decomposition) -- same equivalent-nodal-load theory,
// see TemperatureAnalysis.h's doc comment for the derivation.
std::vector<CreepShrinkageResult> applyUniformShrinkage(
    Model& model, const std::vector<int>& elementIds, double shrinkageStrain, int loadCaseId);

}  // namespace nrsa::analysis
