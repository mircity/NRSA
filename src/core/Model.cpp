#include "core/Model.h"

#include <stdexcept>

namespace nrsa {

int Model::addNode(Node node) {
    int id = node.id();
    if (nodeIndex_.count(id)) {
        throw std::invalid_argument("Model::addNode: duplicate node id " + std::to_string(id));
    }
    nodeIndex_[id] = nodes_.size();
    nodes_.push_back(std::move(node));
    return id;
}

int Model::addMaterial(Material material) {
    int id = material.id();
    if (materialIndex_.count(id)) {
        throw std::invalid_argument("Model::addMaterial: duplicate material id " + std::to_string(id));
    }
    materialIndex_[id] = materials_.size();
    materials_.push_back(std::move(material));
    return id;
}

int Model::addSection(Section section) {
    // Sections aren't given an explicit id by the caller (Section has no
    // id field of its own — a Section is a value type identified only by
    // its position/name) so Model assigns one on insertion, matching how
    // Element::sectionId() references it.
    int id = static_cast<int>(sections_.size());
    sectionIndex_[id] = sections_.size();
    sections_.push_back(std::move(section));
    return id;
}

int Model::addElement(Element element) {
    int id = element.id();
    if (elementIndex_.count(id)) {
        throw std::invalid_argument("Model::addElement: duplicate element id " + std::to_string(id));
    }
    for (int nid : element.nodeIds()) {
        if (!hasNode(nid)) {
            throw std::invalid_argument(
                "Model::addElement: element " + std::to_string(id) +
                " references node " + std::to_string(nid) + ", which doesn't exist yet — "
                "add every node before the elements that connect to it.");
        }
    }
    elementIndex_[id] = elements_.size();
    elements_.push_back(std::move(element));
    return id;
}

int Model::addLoadCase(LoadCase loadCase) {
    int id = loadCase.id();
    if (loadCaseIndex_.count(id)) {
        throw std::invalid_argument("Model::addLoadCase: duplicate load case id " + std::to_string(id));
    }
    loadCaseIndex_[id] = loadCases_.size();
    loadCases_.push_back(std::move(loadCase));
    return id;
}

int Model::addLoadCombination(LoadCombination combo) {
    int id = combo.id();
    if (loadCombinationIndex_.count(id)) {
        throw std::invalid_argument("Model::addLoadCombination: duplicate combination id " + std::to_string(id));
    }
    loadCombinationIndex_[id] = loadCombinations_.size();
    loadCombinations_.push_back(std::move(combo));
    return id;
}

Node& Model::node(int id) { return nodes_.at(nodeIndex_.at(id)); }
const Node& Model::node(int id) const { return nodes_.at(nodeIndex_.at(id)); }
Material& Model::material(int id) { return materials_.at(materialIndex_.at(id)); }
const Material& Model::material(int id) const { return materials_.at(materialIndex_.at(id)); }
Section& Model::section(int id) { return sections_.at(sectionIndex_.at(id)); }
const Section& Model::section(int id) const { return sections_.at(sectionIndex_.at(id)); }
Element& Model::element(int id) { return elements_.at(elementIndex_.at(id)); }
const Element& Model::element(int id) const { return elements_.at(elementIndex_.at(id)); }

int Model::assignDofNumbers() {
    int next = 0;
    for (auto& n : nodes_) {
        for (int d = 0; d < kDofPerNode; ++d) {
            auto dof = static_cast<DOF>(d);
            if (n.isRestrained(dof)) {
                n.setDofIndex(dof, -1);
            } else {
                n.setDofIndex(dof, next++);
            }
        }
    }
    freeDofCount_ = next;
    return freeDofCount_;
}

}  // namespace nrsa
