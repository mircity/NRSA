#pragma once

#include "analysis/StaticAnalysis.h"
#include "core/Model.h"

namespace nrsa::analysis {

// Linear (geometric-stiffness) P-Delta analysis: a well-established
// LINEAR approximation to second-order behavior, standard in seismic
// and gravity design practice (e.g. the stability coefficient / P-Delta
// amplification methods in ASCE 7, BNBC 2020) — NOT the same as a
// genuinely nonlinear (large-displacement, iterative) second-order
// analysis, which belongs in analysis/NonlinearAnalysis instead. This
// class does exactly two linear solves:
//   1. Runs axialLoadCaseId (typically the gravity combination, e.g.
//      1.0D + 1.0L) through an ordinary StaticAnalysis to recover each
//      frame element's axial force.
//   2. Builds an augmented stiffness matrix Ke + Kg, where Kg is every
//      frame element's fem::FrameElement3D::geometricStiffnessGlobal()
//      evaluated at the axial force found in step 1, then solves
//      lateralLoadCaseId (typically wind or seismic) against THAT
//      stiffness instead of the ordinary elastic one.
// The result is the P-Delta-AMPLIFIED lateral response: compression
// (the normal case for gravity-loaded columns) softens the lateral
// stiffness, so displacements and member forces under the lateral case
// come out larger than an ordinary (non-P-Delta) StaticAnalysis run of
// the same lateral case alone would show — the whole point of doing
// this in two passes rather than one.
//
// SCOPE: only frame elements (Beam/Column/Brace) contribute geometric
// stiffness — shell elements (Wall/Slab) do not yet have a geometric-
// stiffness formulation in this engine, so a model relying heavily on
// wall P-Delta effects will not see that contribution reflected here
// yet; this mirrors the same "say what wasn't done" pattern used for
// Core elements and skippedElementIds elsewhere. Axial force per frame
// element is taken as the average of its two end values (they are
// equal and opposite for a 2-force member in equilibrium with no
// distributed axial load — an explicit, temporary simplification for
// any member that DOES carry a distributed axial load, e.g. a raked
// column under self-weight along its own axis; a future addition could
// use the local axial force distribution instead of a single
// representative value).
class PDeltaAnalysis {
public:
    explicit PDeltaAnalysis(Model& model) : model_(model) {}

    // Throws whatever the underlying StaticAnalysis solves throw (see
    // that class's own doc comment) — most commonly an unrestrained
    // mechanism, or (specific to this class) a compressive axial force
    // large enough to make the augmented system non-positive-definite,
    // i.e. the structure has effectively buckled under the gravity load
    // alone before the lateral case is even applied — a genuine, not
    // spurious, result worth surfacing rather than masking.
    StaticAnalysisResult run(int axialLoadCaseId, int lateralLoadCaseId);

private:
    Model& model_;
};

}  // namespace nrsa::analysis
