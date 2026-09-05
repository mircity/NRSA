#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "core/Model.h"

namespace nrsa::analysis {

// Global displacements (or, when reused as a reaction container, forces)
// at one node, in the same Ux,Uy,Uz,Rx,Ry,Rz order as DOF.
struct NodeVector6 {
    double Ux = 0.0, Uy = 0.0, Uz = 0.0, Rx = 0.0, Ry = 0.0, Rz = 0.0;

    double value(DOF dof) const {
        switch (dof) {
            case DOF::Ux: return Ux;
            case DOF::Uy: return Uy;
            case DOF::Uz: return Uz;
            case DOF::Rx: return Rx;
            case DOF::Ry: return Ry;
            case DOF::Rz: return Rz;
        }
        return 0.0;
    }
    void set(DOF dof, double v) {
        switch (dof) {
            case DOF::Ux: Ux = v; break;
            case DOF::Uy: Uy = v; break;
            case DOF::Uz: Uz = v; break;
            case DOF::Rx: Rx = v; break;
            case DOF::Ry: Ry = v; break;
            case DOF::Rz: Rz = v; break;
        }
    }
    void add(DOF dof, double v) { set(dof, value(dof) + v); }
};

// A frame element's internal end forces, in its OWN local coordinate
// system, node1 end then node2 end. Sign convention matches the local
// stiffness matrix's own DOF order directly (this is the local
// stiffness times the local end displacements — no separate sign-flip
// table to keep in sync): a positive axial1 pulls node1 in the -local-x
// direction (i.e. it's the force the element exerts ON node1), which is
// the same convention every matrix-structural-analysis text uses for
// raw stiffness-recovered end forces (NOT yet converted to a
// "tension positive on both ends" beam-diagram convention — that
// conversion belongs to whatever consumes this for a BMD/SFD plot).
struct FrameEndForces {
    double axial1 = 0, shearY1 = 0, shearZ1 = 0, torsion1 = 0, momentY1 = 0, momentZ1 = 0;
    double axial2 = 0, shearY2 = 0, shearZ2 = 0, torsion2 = 0, momentY2 = 0, momentZ2 = 0;
};

// Full result of one linear static solve: nodal displacements, nodal
// reactions (meaningful only at restrained DOFs — see the field
// comment), per-element local end forces, and a residual/diagnostic
// list of anything StaticAnalysis could not process.
struct StaticAnalysisResult {
    std::unordered_map<int, NodeVector6> displacements;  // node id -> global displacement
    // Reaction at a node's RESTRAINED DOFs only; a free DOF's entry here
    // is instead the equilibrium RESIDUAL (sum of internal element
    // forces minus applied external load) — for a converged linear
    // solve this should be ~0 at every free DOF, so a nonzero residual
    // there is itself a useful diagnostic (see
    // StaticAnalysisResult::maxFreeDofResidual()).
    std::unordered_map<int, NodeVector6> reactions;
    std::unordered_map<int, FrameEndForces> elementForces;  // FRAME elements only — see class doc comment

    // Element ids that StaticAnalysis could not process. As of
    // fem::ShellElement being wired in, this is now just: Wall/Slab
    // elements given anything other than exactly 4 nodes (a 2-node
    // "wall as a line" — the browser prototype's wide-column-spine
    // idealization — isn't yet supported here), and Core elements
    // always (the prototype's lift/stair-core tube idealization needs a
    // footprint width/depth/thickness input core::Element doesn't carry
    // yet, not 4 corner nodes). Surfaced explicitly rather than
    // silently ignored, same "say what wasn't done" pattern used
    // throughout this project.
    std::vector<int> skippedElementIds;

    double maxFreeDofResidualNorm = 0.0;
};

// Assembles the global linear system for ONE load case from a
// Model — frame elements (Beam/Column/Brace) via fem::FrameElement3D,
// AND Wall/Slab elements with exactly 4 nodes via fem::ShellElement, in
// the SAME assembly and solve — this is what makes a slab and the
// columns/beams supporting it genuinely COUPLED (sharing stiffness in
// one system), not just two separate analyses whose results happen to
// sit in the same result object. See StaticAnalysisResult::
// skippedElementIds for what still isn't handled. Solves with
// fem::Solver::solve() (sparse, Jacobi-preconditioned Conjugate
// Gradient; see fem::Solver's own doc comment for why iterative was
// chosen over a direct sparse factorization), and recovers
// displacements, reactions (both frame- and shell-element internal
// forces feed into the reaction/residual accounting), and per-FRAME-
// element local end forces — shell elements do not yet get a
// per-element stress/moment-per-unit-width result of their own (a
// meaningfully different kind of recovery, at Gauss points, than a
// frame element's single end-force set; a natural next addition).
//
// Distributed element loads (DistributedLoad /
// PartialDistributedLoad) are NOT yet converted to equivalent nodal
// loads — only NodalLoad entries are applied. A member with only
// a self-weight/distributed load and no nodal load will currently solve
// as unloaded; this is an explicit, temporary scope limit (equivalent
// nodal load conversion is a natural next addition once this base
// pipeline is validated), not a silent gap — the README roadmap notes
// it as the next thing to add here.
class StaticAnalysis {
public:
    explicit StaticAnalysis(Model& model) : model_(model) {}

    // Runs a single load case (nodal loads matching loadCaseId only —
    // see the class-level note on distributed loads) and returns the
    // full result. Throws std::runtime_error if the assembled system is
    // not positive-definite (see fem::Solver::solve) — most commonly
    // caused by a node or a whole disconnected sub-structure with no
    // stiffness path to a support.
    StaticAnalysisResult run(int loadCaseId);

private:
    Model& model_;
};

}  // namespace nrsa::analysis
