#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

namespace nrsa {

// A named load pattern (BNBC/ASCE terminology: "load case") — Dead,
// Live, Wind-X, EQ-Y, etc. Analysis is always run one LoadCase at a
// time; LoadCombination (below) is what mixes them with factors
// afterward, exactly like every commercial package separates "case"
// from "combo".
enum class LoadCaseType {
    Dead, Live, LiveRoof, Wind, Seismic, Snow, Rain,
    Temperature, Hydrostatic, Flood, Moving, Other
};

class LoadCase {
public:
    LoadCase(int id, std::string name, LoadCaseType type)
        : id_(id), name_(std::move(name)), type_(type) {}

    int id() const { return id_; }
    const std::string& name() const { return name_; }
    LoadCaseType type() const { return type_; }

private:
    int id_;
    std::string name_;
    LoadCaseType type_;
};

// A concentrated load/moment applied directly at a node, in the global
// coordinate system, under one LoadCase.
struct NodalLoad {
    int nodeId;
    int loadCaseId;
    double Fx = 0.0, Fy = 0.0, Fz = 0.0;
    double Mx = 0.0, My = 0.0, Mz = 0.0;
};

// A uniformly distributed load along a two-node frame element's own
// local axis system (wy, wz transverse; wx axial) — the common case for
// self-weight and slab-reaction loads applied onto beams. Trapezoidal/
// partial-length loads are represented as an additional, separate entry
// with explicit start/end fractions rather than as fields on this one
// struct, keeping the always-uniform case (the overwhelming majority of
// calls) free of unused fields.
struct DistributedLoad {
    int elementId;
    int loadCaseId;
    double wx = 0.0, wy = 0.0, wz = 0.0;  // force per unit length, local axes
};

struct PartialDistributedLoad {
    int elementId;
    int loadCaseId;
    double wx = 0.0, wy = 0.0, wz = 0.0;
    double startFraction = 0.0;  // 0..1 along the element's length
    double endFraction = 1.0;
};

// A factored combination of LoadCases — e.g. 1.2D + 1.6L per BNBC
// 2020 §2.7 / ACI 318-19 §5.3. The design modules (design/BNBC2020 etc.)
// are what actually enumerate the standard combination set for a given
// code; LoadCombination itself is just the (case, factor) container an
// analysis or design check evaluates against.
class LoadCombination {
public:
    LoadCombination(int id, std::string name) : id_(id), name_(std::move(name)) {}

    int id() const { return id_; }
    const std::string& name() const { return name_; }

    void addFactor(int loadCaseId, double factor) { factors_[loadCaseId] = factor; }
    double factorFor(int loadCaseId) const {
        auto it = factors_.find(loadCaseId);
        return it == factors_.end() ? 0.0 : it->second;
    }
    const std::map<int, double>& factors() const { return factors_; }

private:
    int id_;
    std::string name_;
    std::map<int, double> factors_;  // loadCaseId -> factor
};

}  // namespace nrsa
