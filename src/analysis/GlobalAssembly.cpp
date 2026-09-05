#include "analysis/GlobalAssembly.h"

#include <algorithm>
#include <cmath>

namespace nrsa::analysis {

bool isFrameLike(ElementKind kind) {
    return kind == ElementKind::Beam || kind == ElementKind::Column || kind == ElementKind::Brace;
}

bool isShellLike(const Element& elem) {
    return (elem.kind() == ElementKind::Wall || elem.kind() == ElementKind::Slab) &&
           elem.nodeCount() == 4;
}

namespace {

fem::FrameElement3D buildFrameElement(const Model& model, const Element& elem) {
    const Node& n1 = model.node(elem.nodeId(0));
    const Node& n2 = model.node(elem.nodeId(1));
    const Material& mat = model.material(elem.materialId());
    const Section& sec = model.section(elem.sectionId());
    std::array<double, 3> p1{n1.x(), n1.y(), n1.z()};
    std::array<double, 3> p2{n2.x(), n2.y(), n2.z()};
    std::array<double, 3> ref = elem.hasLocalAxisReference() ? elem.localAxisReference()
                                                              : std::array<double, 3>{0.0, 0.0, 0.0};
    return fem::FrameElement3D(p1, p2, mat, sec, ref);
}

fem::ShellElement buildShellElement(const Model& model, const Element& elem) {
    std::array<std::array<double, 3>, 4> p;
    for (int i = 0; i < 4; ++i) {
        const Node& n = model.node(elem.nodeId(static_cast<std::size_t>(i)));
        p[static_cast<std::size_t>(i)] = {n.x(), n.y(), n.z()};
    }
    const Material& mat = model.material(elem.materialId());
    const Section& sec = model.section(elem.sectionId());
    return fem::ShellElement(p[0], p[1], p[2], p[3], mat, sec.thickness);
}

std::array<int, fem::kFrameDof> globalDofIndices(const Node& n1, const Node& n2) {
    std::array<int, fem::kFrameDof> out{};
    auto d1 = n1.dofIndices();
    auto d2 = n2.dofIndices();
    for (int i = 0; i < 6; ++i) out[i] = d1[i];
    for (int i = 0; i < 6; ++i) out[6 + i] = d2[i];
    return out;
}

std::array<int, fem::kShellDof> shellGlobalDofIndices(const std::array<const Node*, 4>& nodes) {
    std::array<int, fem::kShellDof> out{};
    for (int nodeI = 0; nodeI < 4; ++nodeI) {
        auto d = nodes[static_cast<std::size_t>(nodeI)]->dofIndices();
        for (int i = 0; i < 6; ++i) out[nodeI * 6 + i] = d[i];
    }
    return out;
}

}  // namespace

BuiltElements buildElements(const Model& model) {
    BuiltElements built;
    for (const auto& elem : model.elements()) {
        if (isFrameLike(elem.kind())) {
            const Node& n1 = model.node(elem.nodeId(0));
            const Node& n2 = model.node(elem.nodeId(1));
            built.frames.push_back(
                BuiltFrameElement{elem.id(), buildFrameElement(model, elem), globalDofIndices(n1, n2)});
        } else if (isShellLike(elem)) {
            std::array<const Node*, 4> nodes;
            for (int i = 0; i < 4; ++i) {
                nodes[static_cast<std::size_t>(i)] = &model.node(elem.nodeId(static_cast<std::size_t>(i)));
            }
            built.shells.push_back(
                BuiltShellElement{elem.id(), buildShellElement(model, elem), shellGlobalDofIndices(nodes)});
        } else {
            built.skippedElementIds.push_back(elem.id());
        }
    }
    return built;
}

void assembleStiffness(const BuiltElements& built, fem::SparseMatrix& K,
                        const FrameExtraStiffnessFn& frameExtra) {
    for (const auto& b : built.frames) {
        fem::Matrix kg = b.frame.globalStiffness();
        if (frameExtra) kg = kg + frameExtra(b);
        for (int i = 0; i < fem::kFrameDof; ++i) {
            int gi = b.gdof[i];
            if (gi < 0) continue;
            for (int j = i; j < fem::kFrameDof; ++j) {  // upper triangle only
                int gj = b.gdof[j];
                if (gj < 0) continue;
                K.add(static_cast<std::size_t>(gi), static_cast<std::size_t>(gj), kg(i, j));
            }
        }
    }
    for (const auto& b : built.shells) {
        fem::Matrix kg = b.shell.globalStiffness();
        for (int i = 0; i < fem::kShellDof; ++i) {
            int gi = b.gdof[i];
            if (gi < 0) continue;
            for (int j = i; j < fem::kShellDof; ++j) {
                int gj = b.gdof[j];
                if (gj < 0) continue;
                K.add(static_cast<std::size_t>(gi), static_cast<std::size_t>(gj), kg(i, j));
            }
        }
    }
}

std::unordered_map<int, NodeVector6> accumulateNodalInternalForces(
    const Model& model, const BuiltElements& built, const std::vector<double>& uFree,
    const FrameExtraStiffnessFn& frameExtra) {
    std::unordered_map<int, NodeVector6> nodalInternalForce;

    for (const auto& b : built.frames) {
        const Element& elem = model.element(b.elementId);
        int n1Id = elem.nodeId(0), n2Id = elem.nodeId(1);

        fem::Vector uElemGlobal(fem::kFrameDof, 0.0);
        for (int i = 0; i < fem::kFrameDof; ++i) {
            int gi = b.gdof[i];
            uElemGlobal(static_cast<std::size_t>(i)) =
                (gi >= 0) ? uFree[static_cast<std::size_t>(gi)] : 0.0;
        }
        fem::Matrix Kg = b.frame.globalStiffness();
        if (frameExtra) Kg = Kg + frameExtra(b);
        fem::Vector fElemGlobal = Kg * uElemGlobal;

        double comps1[6], comps2[6];
        for (int i = 0; i < 6; ++i) comps1[i] = fElemGlobal(static_cast<std::size_t>(i));
        for (int i = 0; i < 6; ++i) comps2[i] = fElemGlobal(static_cast<std::size_t>(6 + i));
        auto& acc1 = nodalInternalForce[n1Id];
        auto& acc2 = nodalInternalForce[n2Id];
        for (int i = 0; i < 6; ++i) {
            acc1.add(static_cast<DOF>(i), comps1[i]);
            acc2.add(static_cast<DOF>(i), comps2[i]);
        }
    }

    for (const auto& b : built.shells) {
        const Element& elem = model.element(b.elementId);
        fem::Vector uElemGlobal(fem::kShellDof, 0.0);
        for (int i = 0; i < fem::kShellDof; ++i) {
            int gi = b.gdof[i];
            uElemGlobal(static_cast<std::size_t>(i)) =
                (gi >= 0) ? uFree[static_cast<std::size_t>(gi)] : 0.0;
        }
        fem::Matrix Kg = b.shell.globalStiffness();
        fem::Vector fElemGlobal = Kg * uElemGlobal;
        for (int nodeI = 0; nodeI < 4; ++nodeI) {
            int nodeId = elem.nodeId(static_cast<std::size_t>(nodeI));
            auto& acc = nodalInternalForce[nodeId];
            for (int i = 0; i < 6; ++i) {
                acc.add(static_cast<DOF>(i), fElemGlobal(static_cast<std::size_t>(nodeI * 6 + i)));
            }
        }
    }

    return nodalInternalForce;
}

std::unordered_map<int, FrameEndForces> recoverFrameEndForces(
    const BuiltElements& built, const std::vector<double>& uFree) {
    std::unordered_map<int, FrameEndForces> result;
    for (const auto& b : built.frames) {
        fem::Vector uElemGlobal(fem::kFrameDof, 0.0);
        for (int i = 0; i < fem::kFrameDof; ++i) {
            int gi = b.gdof[i];
            uElemGlobal(static_cast<std::size_t>(i)) =
                (gi >= 0) ? uFree[static_cast<std::size_t>(gi)] : 0.0;
        }
        fem::Matrix T = b.frame.transformationMatrix();
        fem::Vector uLocal = T * uElemGlobal;
        fem::Matrix Kl = b.frame.localStiffness();
        fem::Vector fLocal = Kl * uLocal;

        FrameEndForces ef;
        ef.axial1 = fLocal(0);   ef.shearY1 = fLocal(1);  ef.shearZ1 = fLocal(2);
        ef.torsion1 = fLocal(3); ef.momentY1 = fLocal(4); ef.momentZ1 = fLocal(5);
        ef.axial2 = fLocal(6);   ef.shearY2 = fLocal(7);  ef.shearZ2 = fLocal(8);
        ef.torsion2 = fLocal(9); ef.momentY2 = fLocal(10); ef.momentZ2 = fLocal(11);
        result[b.elementId] = ef;
    }
    return result;
}

std::unordered_map<int, NodeVector6> buildReactionsAndResidual(
    const Model& model, const std::unordered_map<int, NodeVector6>& nodalInternalForce,
    int loadCaseId, double& maxFreeDofResidualNormOut) {
    std::unordered_map<int, NodeVector6> reactions;
    double maxResidual = 0.0;
    for (const auto& n : model.nodes()) {
        auto it = nodalInternalForce.find(n.id());
        NodeVector6 internal = (it != nodalInternalForce.end()) ? it->second : NodeVector6{};
        NodeVector6 out;
        for (int i = 0; i < 6; ++i) {
            auto dof = static_cast<DOF>(i);
            if (n.isRestrained(dof)) {
                out.set(dof, internal.value(dof));
            } else {
                double applied = 0.0;
                for (const auto& load : model.nodalLoads()) {
                    if (load.loadCaseId != loadCaseId || load.nodeId != n.id()) continue;
                    double comps[6] = {load.Fx, load.Fy, load.Fz, load.Mx, load.My, load.Mz};
                    applied += comps[i];
                }
                double residual = internal.value(dof) - applied;
                out.set(dof, residual);
                maxResidual = std::max(maxResidual, std::abs(residual));
            }
        }
        reactions[n.id()] = out;
    }
    maxFreeDofResidualNormOut = maxResidual;
    return reactions;
}

}  // namespace nrsa::analysis
