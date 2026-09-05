#pragma once

#include <array>
#include <cstddef>
#include <string>

namespace nrsa {

// Global DOF order used everywhere in NRSA: 3 translations then 3
// rotations, in a right-handed X-Y-Z system with Z vertical (up) unless a
// Model overrides that convention for a specific analysis.
enum class DOF : int { Ux = 0, Uy = 1, Uz = 2, Rx = 3, Ry = 4, Rz = 5 };
inline constexpr int kDofPerNode = 6;

// A single structural node (joint). Owns its own restraint flags and,
// once Model::assignDofNumbers() has run, the global equation numbers for
// its six DOFs (or -1 for a restrained DOF, which never gets an equation
// number).
class Node {
public:
    Node(int id, double x, double y, double z, std::string label = "")
        : id_(id), x_(x), y_(y), z_(z), label_(std::move(label)) {
        restrained_.fill(false);
        dofIndex_.fill(-1);
    }

    int id() const { return id_; }
    const std::string& label() const { return label_; }
    void setLabel(std::string label) { label_ = std::move(label); }

    double x() const { return x_; }
    double y() const { return y_; }
    double z() const { return z_; }
    std::array<double, 3> position() const { return {x_, y_, z_}; }

    // Restraints ------------------------------------------------------
    // Fixes/frees a single DOF. Fixing a DOF that already has an assigned
    // equation number invalidates that numbering — callers must re-run
    // Model::assignDofNumbers() after changing any restraint.
    void restrain(DOF dof, bool fixed = true) {
        restrained_[static_cast<std::size_t>(dof)] = fixed;
    }
    void restrainAll() { restrained_.fill(true); }
    void freeAll() { restrained_.fill(false); }
    bool isRestrained(DOF dof) const {
        return restrained_[static_cast<std::size_t>(dof)];
    }
    // True only if every one of the six DOFs is restrained (a full fixed
    // support) — a node with some but not all DOFs restrained is a
    // partial/roller-type support and returns false here.
    bool isFullyFixed() const {
        for (bool r : restrained_) if (!r) return false;
        return true;
    }
    bool isFree() const {
        for (bool r : restrained_) if (r) return false;
        return true;
    }

    // DOF numbering -----------------------------------------------------
    // Set by Model::assignDofNumbers(); -1 means "restrained, no
    // equation". Not intended to be called directly outside Model.
    void setDofIndex(DOF dof, int globalIndex) {
        dofIndex_[static_cast<std::size_t>(dof)] = globalIndex;
    }
    int dofIndex(DOF dof) const {
        return dofIndex_[static_cast<std::size_t>(dof)];
    }
    // Convenience: the six global indices in Ux,Uy,Uz,Rx,Ry,Rz order,
    // -1 for any restrained DOF — this is the array element/assembly
    // code indexes into when scattering a 6x6 (or 12x12 two-node) local
    // stiffness block into the global matrix.
    std::array<int, kDofPerNode> dofIndices() const { return dofIndex_; }

    // Lumped nodal mass (translational; consistent-mass / rotational
    // inertia is left at zero unless a caller sets it explicitly via
    // setRotationalMass — most RC building models only need translational
    // mass for a modal/response-spectrum run).
    void setTranslationalMass(double m) { massTranslational_ = m; }
    double translationalMass() const { return massTranslational_; }
    void setRotationalMass(double ix, double iy, double iz) {
        massRotational_ = {ix, iy, iz};
    }
    std::array<double, 3> rotationalMass() const { return massRotational_; }

private:
    int id_;
    double x_, y_, z_;
    std::string label_;
    std::array<bool, kDofPerNode> restrained_{};
    std::array<int, kDofPerNode> dofIndex_{};
    double massTranslational_ = 0.0;
    std::array<double, 3> massRotational_{0.0, 0.0, 0.0};
};

}  // namespace nrsa
