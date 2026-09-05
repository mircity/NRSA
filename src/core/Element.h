#pragma once

#include <array>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace nrsa {

// What kind of structural member this Element represents. This is a
// MODELING classification (what an engineer would call it), separate
// from the FINITE-ELEMENT FORMULATION used to analyze it — a Wall or
// Core, for instance, gets analyzed with the shell/tube formulations in
// fem/, not with a frame element, even though both are "Element" here.
// design/ modules switch on this to pick which code-check routine
// applies (RCCBeam vs RCCColumn vs ShearWall).
enum class ElementKind {
    Beam, Column, Wall, Core, Slab, Brace, Spring, Rigid
};

// A structural element: the two (or more, for a shell) nodes it
// connects, plus which Material and Section it uses, plus enough
// geometric orientation data for a frame element's local axis system to
// be built unambiguously. This is a MODEL-level object — id, materiaL id and section id are
// database-style references (looked up through Model), not owning
// pointers, so an Element can be serialized/copied without dragging the
// whole model graph along with it.
//
// This class intentionally does NOT own a stiffness matrix or know how
// to assemble one — that behavior lives in the fem::FrameElement3D /
// fem::ShellElement formulations built from an Element + its resolved
// Material/Section by the analysis layer. Element is the durable model
// description; the fem:: classes are the disposable, analysis-specific
// numerical objects built fresh for each solve.
class Element {
public:
    Element(int id, ElementKind kind, std::vector<int> nodeIds,
            int materialId, int sectionId, std::string label = "")
        : id_(id), kind_(kind), nodeIds_(std::move(nodeIds)),
          materialId_(materialId), sectionId_(sectionId), label_(std::move(label)) {
        if (nodeIds_.size() < 2) {
            throw std::invalid_argument(
                "Element " + std::to_string(id) + ": needs at least 2 nodes, got " +
                std::to_string(nodeIds_.size()));
        }
    }

    int id() const { return id_; }
    ElementKind kind() const { return kind_; }
    const std::string& label() const { return label_; }
    void setLabel(std::string l) { label_ = std::move(l); }

    const std::vector<int>& nodeIds() const { return nodeIds_; }
    int nodeId(std::size_t localIndex) const { return nodeIds_.at(localIndex); }
    std::size_t nodeCount() const { return nodeIds_.size(); }

    int materialId() const { return materialId_; }
    int sectionId() const { return sectionId_; }

    // For a 2-node frame element: an optional reference vector used to
    // fix rotation about the local x (member) axis — the same role a
    // "local axis 3" reference point plays in ETABS/SAP2000. If unset
    // (the default {0,0,0}), the element formulation falls back to the
    // usual convention of using global Z as the reference unless the
    // member itself is vertical, in which case global X is used instead.
    void setLocalAxisReference(std::array<double, 3> refVector) {
        localAxisRef_ = refVector;
        hasLocalAxisRef_ = true;
    }
    bool hasLocalAxisReference() const { return hasLocalAxisRef_; }
    std::array<double, 3> localAxisReference() const { return localAxisRef_; }

    // Rigid end offsets / end releases are common enough in real models
    // (beam-column joint rigid zones, pinned beam ends) to belong here
    // rather than being bolted on later — start empty (fully rigid, no
    // offset) until a caller sets them.
    void setEndRelease(std::size_t localNodeIndex, std::array<bool, 6> releaseDof) {
        endReleases_[localNodeIndex] = releaseDof;
    }
    bool hasEndRelease(std::size_t localNodeIndex) const {
        return endReleases_.find(localNodeIndex) != endReleases_.end();
    }
    std::array<bool, 6> endRelease(std::size_t localNodeIndex) const {
        auto it = endReleases_.find(localNodeIndex);
        if (it == endReleases_.end()) return {false, false, false, false, false, false};
        return it->second;
    }

private:
    int id_;
    ElementKind kind_;
    std::vector<int> nodeIds_;
    int materialId_;
    int sectionId_;
    std::string label_;
    std::array<double, 3> localAxisRef_{0.0, 0.0, 0.0};
    bool hasLocalAxisRef_ = false;
    std::map<std::size_t, std::array<bool, 6>> endReleases_;
};

}  // namespace nrsa
