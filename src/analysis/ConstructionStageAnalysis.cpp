#include "analysis/ConstructionStageAnalysis.h"

#include <algorithm>
#include <set>
#include <stdexcept>
#include <vector>

#include "analysis/GlobalAssembly.h"
#include "fem/Solver.h"

namespace nrsa::analysis {

namespace {

std::array<bool, 6> captureRestraints(const Node& n) {
    std::array<bool, 6> r{};
    for (int i = 0; i < 6; ++i) r[static_cast<std::size_t>(i)] = n.isRestrained(static_cast<DOF>(i));
    return r;
}

void applyRestraints(Node& n, const std::array<bool, 6>& r) {
    for (int i = 0; i < 6; ++i) n.restrain(static_cast<DOF>(i), r[static_cast<std::size_t>(i)]);
}

void addVec(NodeVector6& acc, const NodeVector6& inc) {
    for (int i = 0; i < 6; ++i) {
        DOF d = static_cast<DOF>(i);
        acc.set(d, acc.value(d) + inc.value(d));
    }
}

// All node ids in the model, captured once up front (Model exposes a
// mutable per-id node(id) accessor but not a mutable whole-vector
// getter -- iterating ids via the const nodes() list and then going
// through node(id) for each is the intended mutation pattern, avoiding
// a const_cast on the model's own storage).
std::vector<int> allNodeIds(const Model& model) {
    std::vector<int> ids;
    ids.reserve(model.nodes().size());
    for (const auto& n : model.nodes()) ids.push_back(n.id());
    return ids;
}

}  // namespace

std::vector<ConstructionStageResult> runConstructionStages(
    Model& model, const std::vector<ConstructionStage>& stages) {
    if (stages.empty()) throw std::invalid_argument("runConstructionStages: no stages given");

    // Non-decreasing element-set check: every element active in an
    // earlier stage must still be listed active in every later one.
    std::set<int> prevActive;
    for (const auto& stage : stages) {
        std::set<int> cur(stage.activeElementIds.begin(), stage.activeElementIds.end());
        if (!std::includes(cur.begin(), cur.end(), prevActive.begin(), prevActive.end())) {
            throw std::invalid_argument(
                "runConstructionStages: stage '" + stage.label +
                "' does not include every element active in a prior stage -- element removal "
                "mid-sequence is not supported (see class doc comment)");
        }
        prevActive = cur;
    }

    const std::vector<int> nodeIds = allNodeIds(model);

    // Save every node's REAL restraint pattern before touching anything.
    std::unordered_map<int, std::array<bool, 6>> originalRestraints;
    for (int id : nodeIds) originalRestraints[id] = captureRestraints(model.node(id));

    auto restoreOriginalRestraints = [&]() {
        for (int id : nodeIds) applyRestraints(model.node(id), originalRestraints.at(id));
    };

    std::vector<ConstructionStageResult> results;
    std::unordered_map<int, NodeVector6> cumulative;
    for (int id : nodeIds) cumulative[id] = NodeVector6{};

    try {
        for (const auto& stage : stages) {
            std::set<int> active(stage.activeElementIds.begin(), stage.activeElementIds.end());

            // Which nodes does at least one currently-active element touch?
            // (computed from the Model's element list directly -- cheaper
            // than building fem element objects just to read node ids.)
            std::set<int> activeNodes;
            for (int eid : active) {
                const Element& e = model.element(eid);
                for (std::size_t i = 0; i < e.nodeCount(); ++i) activeNodes.insert(e.nodeId(i));
            }

            // Not-yet-built nodes: fully restrain for this stage's solve
            // (no meaningful stiffness path yet). Active nodes: restore
            // their REAL restraint pattern (undoing any previous stage's
            // temporary restrainAll() on a node that has since come
            // online).
            for (int id : nodeIds) {
                if (activeNodes.count(id)) applyRestraints(model.node(id), originalRestraints.at(id));
                else model.node(id).restrainAll();
            }

            int freeDofCount = model.assignDofNumbers();
            ConstructionStageResult stageResult;
            stageResult.label = stage.label;

            if (freeDofCount > 0 && !active.empty()) {
                // IMPORTANT: BuiltFrameElement/BuiltShellElement freeze
                // their global DOF indices (gdof) at buildElements() call
                // time -- since assignDofNumbers() just renumbered every
                // free DOF differently for THIS stage's active set, the
                // build must happen AFTER assignDofNumbers(), every
                // stage, not once up front (a one-time build's gdof would
                // silently point at the wrong equations for every stage
                // but the one it was built for).
                BuiltElements allBuiltThisStage = buildElements(model);
                BuiltElements filtered;
                for (const auto& f : allBuiltThisStage.frames)
                    if (active.count(f.elementId)) filtered.frames.push_back(f);
                for (const auto& s : allBuiltThisStage.shells)
                    if (active.count(s.elementId)) filtered.shells.push_back(s);


                fem::SparseMatrix K(static_cast<std::size_t>(freeDofCount));
                assembleStiffness(filtered, K);

                fem::Vector F(static_cast<std::size_t>(freeDofCount), 0.0);
                if (stage.incrementalLoadCaseId >= 0) {
                    for (const auto& load : model.nodalLoads()) {
                        if (load.loadCaseId != stage.incrementalLoadCaseId) continue;
                        const Node& n = model.node(load.nodeId);
                        auto d = n.dofIndices();
                        double comps[6] = {load.Fx, load.Fy, load.Fz, load.Mx, load.My, load.Mz};
                        for (int i = 0; i < 6; ++i)
                            if (d[i] >= 0) F(static_cast<std::size_t>(d[i])) += comps[i];
                    }
                }

                bool hasLoad = false;
                for (std::size_t i = 0; i < F.size(); ++i)
                    if (F(i) != 0.0) { hasLoad = true; break; }

                if (hasLoad) {
                    fem::Vector uFreeVec = fem::Solver::solve(K, F).x;
                    for (int id : nodeIds) {
                        if (!activeNodes.count(id)) continue;
                        const Node& n = model.node(id);
                        NodeVector6 v;
                        auto d = n.dofIndices();
                        for (int i = 0; i < 6; ++i)
                            if (d[i] >= 0) v.set(static_cast<DOF>(i), uFreeVec(static_cast<std::size_t>(d[i])));
                        stageResult.incrementalDisplacements[id] = v;
                        addVec(cumulative[id], v);
                    }
                }
            }

            stageResult.cumulativeDisplacements = cumulative;
            results.push_back(stageResult);
        }
    } catch (...) {
        restoreOriginalRestraints();
        throw;
    }

    restoreOriginalRestraints();
    return results;
}

}  // namespace nrsa::analysis
