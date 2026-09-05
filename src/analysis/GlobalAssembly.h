#pragma once

#include <array>
#include <functional>
#include <unordered_map>
#include <vector>

#include "analysis/StaticAnalysis.h"  // NodeVector6, FrameEndForces, DOF
#include "core/Model.h"
#include "fem/FrameElement3D.h"
#include "fem/Matrix.h"
#include "fem/ShellElement.h"
#include "fem/SparseMatrix.h"

namespace nrsa::analysis {

// Everything below was, until this module existed, duplicated nearly
// verbatim across StaticAnalysis.cpp, ModalAnalysis.cpp, and
// PDelta.cpp -- three consumers of the same "classify this Model's
// elements, build the fem:: objects, assemble a stiffness matrix,
// recover internal forces" logic, each with its own anonymous-
// namespace copy. Extracted here once a third consumer made the
// duplication clearly worth the (small) risk of touching already-
// tested code -- see each of those files' own history/comments for the
// "two consumers doesn't justify the refactor risk yet" judgment call
// that preceded this.

bool isFrameLike(ElementKind kind);
bool isShellLike(const Element& elem);

struct BuiltFrameElement {
    int elementId;
    fem::FrameElement3D frame;
    std::array<int, fem::kFrameDof> gdof;
};
struct BuiltShellElement {
    int elementId;
    fem::ShellElement shell;
    std::array<int, fem::kShellDof> gdof;
};
struct BuiltElements {
    std::vector<BuiltFrameElement> frames;
    std::vector<BuiltShellElement> shells;
    // Same meaning everywhere it's surfaced: elements StaticAnalysis/
    // ModalAnalysis/PDeltaAnalysis could not process (Core elements
    // always; Wall/Slab without exactly 4 nodes) -- see
    // StaticAnalysisResult::skippedElementIds for the full explanation.
    std::vector<int> skippedElementIds;
};

// Classifies and constructs every element in the Model (resolving node
// coordinates/material/section through the Model) into frame/shell/
// skipped buckets. Requires Model::assignDofNumbers() to have already
// been called (the built elements' gdof arrays read each node's
// current DOF numbering directly).
BuiltElements buildElements(const Model& model);

// Optional per-frame-element EXTRA global stiffness contribution,
// added alongside each frame element's ordinary elastic
// globalStiffness() during assembly/recovery -- this is exactly the
// hook PDeltaAnalysis uses to add geometric (P-Delta) stiffness;
// StaticAnalysis and ModalAnalysis simply don't pass one (nullptr --
// the default -- means "no extra term").
using FrameExtraStiffnessFn = std::function<fem::Matrix(const BuiltFrameElement&)>;

// Assembles K (upper-triangle sparse -- see fem::SparseMatrix::add's
// own doc comment on why only the upper triangle is scattered) from
// already-built elements.
void assembleStiffness(const BuiltElements& built, fem::SparseMatrix& K,
                        const FrameExtraStiffnessFn& frameExtra = nullptr);

// Recovers full per-node internal-force accumulation (frame AND shell
// elements both contribute) from a solved free-DOF displacement
// vector -- nodal equilibrium means this accumulated total equals the
// externally applied load at a free DOF (residual ~0 for a converged
// solve) or the reaction at a restrained one; see
// buildReactionsAndResidual below for turning this into that split.
std::unordered_map<int, NodeVector6> accumulateNodalInternalForces(
    const Model& model, const BuiltElements& built, const std::vector<double>& uFree,
    const FrameExtraStiffnessFn& frameExtra = nullptr);

// Recovers per-FRAME-element local end forces (axial/shear/torsion/
// moment at each end, in the element's own local coordinates) from a
// solved free-DOF displacement vector -- always from the ELASTIC local
// stiffness only, even when frameExtra added geometric stiffness to
// the global assembly (see PDeltaAnalysis's own doc comment on why:
// the physically meaningful internal stress resultant a design check
// wants doesn't get a separate "geometric" force component of its
// own -- that term exists purely to capture P-Delta softening in the
// global equilibrium/displacement solution).
std::unordered_map<int, FrameEndForces> recoverFrameEndForces(
    const BuiltElements& built, const std::vector<double>& uFree);

// Splits an already-accumulated internal-force map into the final
// reactions/residual result: a restrained DOF's entry becomes its
// reaction; a free DOF's entry becomes internal-minus-applied (the
// residual diagnostic -- see StaticAnalysisResult's own doc comment).
// Also reports the largest free-DOF residual magnitude found, the same
// convergence diagnostic both StaticAnalysis and PDeltaAnalysis
// surface.
std::unordered_map<int, NodeVector6> buildReactionsAndResidual(
    const Model& model, const std::unordered_map<int, NodeVector6>& nodalInternalForce,
    int loadCaseId, double& maxFreeDofResidualNormOut);

}  // namespace nrsa::analysis
