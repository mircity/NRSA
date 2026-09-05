#pragma once

#include <unordered_map>
#include <vector>

#include "core/Load.h"
#include "core/Model.h"

namespace nrsa::analysis {

// Equivalent-nodal-load conversion for a uniform temperature change (or
// any other uniform INITIAL AXIAL STRAIN — see the shared helper below,
// which analysis::CreepShrinkageAnalysis reuses for shrinkage strain,
// the mathematically identical case with a directly-given strain
// instead of alpha*deltaT).
//
// THEORY: for a straight prismatic bar with uniform thermal strain
// eps0 = alpha*deltaT, the standard FEM "initial strain" equivalent
// nodal load (e.g. Cook/Malkus/Plesha "Concepts and Applications of
// Finite Element Analysis", or Przemieniecki Ch. 15) is a
// self-equilibrated pair of AXIAL forces of magnitude E*A*eps0 pushing
// the two ends APART (a temperature RISE with alpha>0 wants to
// elongate the member; pushing the ends apart is exactly what
// reproduces that elongation in an otherwise-unloaded, unrestrained
// member, and what produces the correct restraining COMPRESSIVE force
// in a fully restrained one). Since local axis x IS the member's
// global unit direction vector, no 12x12 transformation matrix is
// needed -- the equivalent force at each node is just
// magnitude * unit-direction-vector.
//
// SCOPE: uniform axial strain only (uniform temperature change through
// the section, or uniform shrinkage). A THROUGH-DEPTH temperature
// GRADIENT (top face hotter/cooler than bottom, inducing curvature/
// equivalent moment rather than pure axial force) is a real and
// sometimes governing BNBC/AASHTO load case for exposed slabs and
// bridge decks -- NOT modeled here; a natural next addition once this
// uniform-strain base case is validated, same "explicit, temporary
// scope limit" pattern used throughout this project.
struct AxialInitialStrainResult {
    int elementId;
    double axialForceMagnitude = 0.0;  // N, the E*A*eps0 pair magnitude
};

// The core shared computation: equivalent nodal force magnitude
// E*A*eps0 for one prismatic member. E in kPa (Model convention), A in
// m^2, eps0 dimensionless (strain) -- returns N given the project's
// kPa*m^2 = kN convention... (Model's E is stored in kPa; kPa*m^2 = kN,
// so the return value here is in kN, matching every other force field
// in this codebase, e.g. NodalLoad::Fx).
double axialForceFromInitialStrain(double E_kPa, double area_m2, double initialStrain);

// Applies a uniform temperature change deltaT (deg C) to every 2-node
// frame element in elementIds (Beam/Column/Brace -- Wall/Slab shell
// elements are not supported by this uniform-axial-strain formulation)
// using thermal expansion coefficient alphaPerDegC, and appends the
// resulting self-equilibrated NodalLoad pair for each element under
// loadCaseId. Throws std::invalid_argument if an id in elementIds does
// not exist or is not a 2-node frame-like element.
std::vector<AxialInitialStrainResult> applyUniformTemperatureChange(
    Model& model, const std::vector<int>& elementIds, double deltaTDegC,
    double alphaPerDegC, int loadCaseId);

}  // namespace nrsa::analysis
