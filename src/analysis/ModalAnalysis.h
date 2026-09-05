#pragma once

#include <unordered_map>
#include <vector>

#include "analysis/StaticAnalysis.h"  // reuses NodeVector6, DOF
#include "core/Model.h"
#include "fem/EigenSolver.h"

namespace nrsa::analysis {

struct ModalAnalysisResult {
    // Sorted ascending by eigenvalue (== lowest natural frequency
    // first). Each ModeShape::shape is a REDUCED (free-DOF-only)
    // vector, in the same global DOF numbering Model::assignDofNumbers
    // produced — use expandModeShape() to turn one into a per-node
    // result the way StaticAnalysisResult::displacements is shaped.
    std::vector<fem::ModeShape> modes;

    // Same meaning as StaticAnalysisResult::skippedElementIds — see
    // that struct's doc comment; Core elements and any Wall/Slab
    // without exactly 4 nodes land here rather than being silently
    // dropped from the stiffness assembly.
    std::vector<int> skippedElementIds;
};

// Expands one mode's reduced (free-DOF-only) shape vector into a full
// per-node NodeVector6 map (0 at every restrained DOF) — the same
// recovery pattern StaticAnalysis uses for displacements, so a mode
// shape can be inspected/plotted the same way a static displacement
// result is.
std::unordered_map<int, NodeVector6> expandModeShape(const Model& model, const fem::ModeShape& mode);

// Assembles the SAME global stiffness matrix StaticAnalysis would (frame
// elements via fem::FrameElement3D, Wall/Slab elements with exactly 4
// nodes via fem::ShellElement — see StaticAnalysisResult's doc comment
// for what's still unsupported), builds a LUMPED mass vector from each
// node's core::Node::translationalMass() (applied to that node's Ux, Uy,
// Uz DOFs) and core::Node::rotationalMass() (applied to Rx, Ry, Rz), and
// solves the generalized eigenproblem via fem::EigenSolver.
//
// ROTATIONAL MASS REGULARIZATION: a free rotational DOF with no
// rotational mass explicitly assigned (core::Node::setRotationalMass)
// has zero mass by core::Node's own default — undefined for
// fem::EigenSolver, which requires every free DOF to have positive
// mass. Rather than requiring every model to hand-assign a physically
// meaningful rotational inertia at every node (rarely available data
// for a preliminary RC building model, and rarely significant compared
// to a floor's translational mass for a first-pass frequency estimate
// — genuine torsional/rotational inertia mainly matters at the
// diaphragm level for real seismic torsion, a story-level concept this
// class does not yet model), any such DOF is instead given a small
// NUMERICAL REGULARIZATION mass — Options::rotationalMassRegularization
// times the model's average positive translational mass — just large
// enough to keep the eigenproblem well-posed, deliberately small enough
// to leave the real, translational-mass-governed mode shapes and
// frequencies essentially unaffected. This is a named, documented
// approximation, not a hidden one: a model that needs genuine
// diaphragm-level torsional mass modeled explicitly (most real seismic
// analyses do) will need that added as a real feature later, not
// discovered by reading this comment after the fact.
//
// Throws std::runtime_error if NO node in the model has positive
// translational mass at all (nothing to build even a regularization
// scale from — almost certainly a model where mass was never
// assigned), and propagates whatever fem::EigenSolver itself throws
// (e.g. Jacobi failing to converge within the sweep cap).
// Options controlling ModalAnalysis::run.
struct ModalAnalysisOptions {
    double rotationalMassRegularization = 1e-6;
    fem::EigenSolver::Options eigenOptions{};
};

class ModalAnalysis {
public:
    using Options = ModalAnalysisOptions;

    explicit ModalAnalysis(Model& model) : model_(model) {}

    ModalAnalysisResult run(Options opts = Options{});

private:
    Model& model_;
};

}  // namespace nrsa::analysis
