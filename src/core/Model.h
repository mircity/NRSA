#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/Element.h"
#include "core/Load.h"
#include "core/Material.h"
#include "core/Node.h"
#include "core/Section.h"

namespace nrsa {

// The structural model: every Node, Element, Material, Section, and
// Load in a single project, plus the DOF-numbering pass an analysis
// needs before it can assemble a global stiffness matrix. Model owns
// its Nodes/Elements/Materials/Sections outright (by value, in
// insertion-ordered vectors) and hands out integer ids the way a
// database would — every cross-reference elsewhere (Element::nodeIds,
// Element::materialId, NodalLoad::nodeId, ...) is one of these ids, not
// a raw pointer, so the whole model stays trivially copyable and
// serializable.
//
// This class deliberately does NOT know how to solve anything — no
// stiffness assembly, no equation solving. That is fem::Solver's job,
// operating on a Model plus the DOF map this class produces. Keeping
// Model itself solver-agnostic is what lets the same Model feed a
// linear static solve, a modal solve, and a nonlinear pushover without
// three different data structures.
class Model {
public:
    Model() = default;
    explicit Model(std::string name) : name_(std::move(name)) {}

    const std::string& name() const { return name_; }
    void setName(std::string n) { name_ = std::move(n); }

    // ---- Adding entities -----------------------------------------------
    // Each add* returns the id actually assigned (== the id passed in,
    // echoed back for chaining convenience) after appending; ids must be
    // unique within their own collection or add* throws.

    int addNode(Node node);
    int addMaterial(Material material);
    int addSection(Section section);
    int addElement(Element element);
    int addLoadCase(LoadCase loadCase);
    int addLoadCombination(LoadCombination combo);
    void addNodalLoad(NodalLoad load) { nodalLoads_.push_back(load); }
    void addDistributedLoad(DistributedLoad load) { distributedLoads_.push_back(load); }
    void addPartialDistributedLoad(PartialDistributedLoad load) {
        partialDistributedLoads_.push_back(load);
    }

    // ---- Lookup ----------------------------------------------------------
    // Throws std::out_of_range if the id doesn't exist — callers that
    // need a non-throwing check should go through hasNode()/hasElement()
    // first (kept explicit rather than returning a nullable pointer, so
    // "not found" can never be silently mistaken for a valid zero-init
    // object downstream).
    Node& node(int id);
    const Node& node(int id) const;
    Material& material(int id);
    const Material& material(int id) const;
    Section& section(int id);
    const Section& section(int id) const;
    Element& element(int id);
    const Element& element(int id) const;

    bool hasNode(int id) const { return nodeIndex_.count(id) > 0; }
    bool hasElement(int id) const { return elementIndex_.count(id) > 0; }

    const std::vector<Node>& nodes() const { return nodes_; }
    const std::vector<Element>& elements() const { return elements_; }
    const std::vector<Material>& materials() const { return materials_; }
    const std::vector<Section>& sections() const { return sections_; }
    const std::vector<LoadCase>& loadCases() const { return loadCases_; }
    const std::vector<LoadCombination>& loadCombinations() const { return loadCombinations_; }
    const std::vector<NodalLoad>& nodalLoads() const { return nodalLoads_; }
    const std::vector<DistributedLoad>& distributedLoads() const { return distributedLoads_; }
    const std::vector<PartialDistributedLoad>& partialDistributedLoads() const {
        return partialDistributedLoads_;
    }

    // ---- DOF numbering ---------------------------------------------------
    // Walks every node in insertion order and assigns a sequential
    // global equation number to each of its six DOFs, skipping any DOF
    // the node has restrained (which keeps its index at -1, meaning "no
    // equation — this DOF's displacement is a prescribed zero, not an
    // unknown"). Must be re-run after ANY restraint changes on ANY node
    // before the numbering can be trusted again; assembly code that
    // reads a stale numbering will silently scatter into the wrong rows/
    // columns rather than fail loudly, so this is cheap enough
    // (O(nodes)) that callers should just re-run it whenever in doubt
    // rather than trying to track staleness themselves.
    // Returns the total number of free (unrestrained) DOFs — i.e. the
    // size of the global stiffness matrix a Solver needs to allocate.
    int assignDofNumbers();

    int freeDofCount() const { return freeDofCount_; }

private:
    std::string name_;

    std::vector<Node> nodes_;
    std::vector<Material> materials_;
    std::vector<Section> sections_;
    std::vector<Element> elements_;
    std::vector<LoadCase> loadCases_;
    std::vector<LoadCombination> loadCombinations_;
    std::vector<NodalLoad> nodalLoads_;
    std::vector<DistributedLoad> distributedLoads_;
    std::vector<PartialDistributedLoad> partialDistributedLoads_;

    // id -> index into the corresponding vector above, so lookups by id
    // stay O(1) even though ids aren't necessarily contiguous (a model
    // loaded from a file that skips ids, or one with elements deleted
    // and re-added during editing, both still work).
    std::unordered_map<int, std::size_t> nodeIndex_;
    std::unordered_map<int, std::size_t> materialIndex_;
    std::unordered_map<int, std::size_t> sectionIndex_;
    std::unordered_map<int, std::size_t> elementIndex_;
    std::unordered_map<int, std::size_t> loadCaseIndex_;
    std::unordered_map<int, std::size_t> loadCombinationIndex_;

    int freeDofCount_ = 0;
};

}  // namespace nrsa
