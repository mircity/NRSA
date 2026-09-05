#include "modeler/AutoModelGenerator.h"

#include <stdexcept>
#include <unordered_map>

#include "core/Element.h"
#include "core/Node.h"
#include "core/Section.h"

namespace nrsa::modeler {

namespace {

void validateParams(const Model& model, const BuildingGenerationParameters& p) {
    if (p.storeys < 1) throw std::invalid_argument("runAutoModelGeneration: storeys must be >= 1");
    if (p.baysX < 1 || p.baysY < 1) {
        throw std::invalid_argument("runAutoModelGeneration: baysX and baysY must be >= 1");
    }
    if (p.bayWidthXm <= 0.0 || p.bayWidthYm <= 0.0 || p.floorHeightM <= 0.0) {
        throw std::invalid_argument("runAutoModelGeneration: bay widths and floor height must be positive");
    }
    if (p.columnWidthM <= 0.0 || p.columnDepthM <= 0.0 || p.beamWidthM <= 0.0 || p.beamDepthM <= 0.0) {
        throw std::invalid_argument("runAutoModelGeneration: column/beam dimensions must be positive");
    }
    try {
        model.material(p.materialId);
    } catch (const std::out_of_range&) {
        throw std::invalid_argument(
            "runAutoModelGeneration: materialId " + std::to_string(p.materialId) +
            " does not exist in the model -- add it first (this generator only builds "
            "geometry, not materials; see class doc comment)");
    }
    if (p.generateExteriorWalls && p.wallThicknessM <= 0.0) {
        throw std::invalid_argument("runAutoModelGeneration: wallThicknessM must be positive");
    }
    if (p.generateSelfWeightLoad && p.selfWeightLoadCaseId < 0) {
        throw std::invalid_argument(
            "runAutoModelGeneration: generateSelfWeightLoad is true but selfWeightLoadCaseId "
            "was not set -- add a LoadCase to the model first and pass its id");
    }
}

}  // namespace

GeneratedModel runAutoModelGeneration(Model& model, const BuildingGenerationParameters& p) {
    validateParams(model, p);
    double unitWeight = model.material(p.materialId).unitWeightKNm3();  // kN/m^3

    int rows = p.baysY + 1;  // grid lines in Y
    int cols = p.baysX + 1;  // grid lines in X
    int floors = p.storeys + 1;  // z-levels including base

    GeneratedModel gen;
    gen.nodeIds.assign(static_cast<std::size_t>(floors),
                        std::vector<std::vector<int>>(static_cast<std::size_t>(rows),
                                                        std::vector<int>(static_cast<std::size_t>(cols), -1)));
    std::unordered_map<int, double> nodeSelfWeightKN;  // accumulated across every category below
    auto addNodeWeight = [&](int nodeId, double w) {
        nodeSelfWeightKN[nodeId] += w;
        gen.totalSelfWeightKN += w;
    };

    // ---- Grid nodes ----
    int nextNodeId = p.nodeIdStart;
    for (int k = 0; k < floors; ++k) {
        double z = k * p.floorHeightM;
        for (int j = 0; j < rows; ++j) {
            double y = j * p.bayWidthYm;
            for (int i = 0; i < cols; ++i) {
                double x = i * p.bayWidthXm;
                int id = nextNodeId++;
                Node n(id, x, y, z, "N_F" + std::to_string(k) + "_" + std::to_string(j) + "_" + std::to_string(i));
                if (k == 0) n.restrainAll();  // fixed base -- see class doc comment
                model.addNode(n);
                gen.nodeIds[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] = id;
            }
        }
    }

    // ---- Sections (one column section, one beam section, shared by every generated member) ----
    int columnSectionId = model.addSection(Section::rectangular(p.columnWidthM, p.columnDepthM, "AutoGen_Column"));
    int beamSectionId = model.addSection(Section::rectangular(p.beamWidthM, p.beamDepthM, "AutoGen_Beam"));
    double columnArea = p.columnWidthM * p.columnDepthM;
    double beamArea = p.beamWidthM * p.beamDepthM;

    // ---- Columns: every grid line, connecting each consecutive floor pair ----
    int nextElementId = p.elementIdStart;
    gen.idRanges.columnsFirst = nextElementId;
    for (int k = 0; k < p.storeys; ++k) {
        for (int j = 0; j < rows; ++j) {
            for (int i = 0; i < cols; ++i) {
                int nBottom = gen.nodeIds[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)][static_cast<std::size_t>(i)];
                int nTop = gen.nodeIds[static_cast<std::size_t>(k + 1)][static_cast<std::size_t>(j)][static_cast<std::size_t>(i)];
                int id = nextElementId++;
                model.addElement(Element(id, ElementKind::Column, {nBottom, nTop}, p.materialId, columnSectionId,
                                          "C_F" + std::to_string(k + 1) + "_" + std::to_string(j) + "_" + std::to_string(i)));
                gen.columnElementIds.push_back(id);
                double w = unitWeight * columnArea * p.floorHeightM;
                addNodeWeight(nBottom, w / 2.0);
                addNodeWeight(nTop, w / 2.0);
            }
        }
    }
    gen.idRanges.columnsCount = static_cast<int>(gen.columnElementIds.size());

    // ---- Beams: at every floor level ABOVE the base (k=1..storeys), both directions ----
    gen.idRanges.beamsFirst = nextElementId;
    for (int k = 1; k < floors; ++k) {
        // X-direction beams: connect (i,j)-(i+1,j) for each row j.
        for (int j = 0; j < rows; ++j) {
            for (int i = 0; i < p.baysX; ++i) {
                int n1 = gen.nodeIds[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)][static_cast<std::size_t>(i)];
                int n2 = gen.nodeIds[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)][static_cast<std::size_t>(i + 1)];
                int id = nextElementId++;
                model.addElement(Element(id, ElementKind::Beam, {n1, n2}, p.materialId, beamSectionId,
                                          "BX_F" + std::to_string(k) + "_" + std::to_string(j) + "_" + std::to_string(i)));
                gen.beamElementIds.push_back(id);
                double w = unitWeight * beamArea * p.bayWidthXm;
                addNodeWeight(n1, w / 2.0);
                addNodeWeight(n2, w / 2.0);
            }
        }
        // Y-direction beams: connect (i,j)-(i,j+1) for each column i.
        for (int i = 0; i < cols; ++i) {
            for (int j = 0; j < p.baysY; ++j) {
                int n1 = gen.nodeIds[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)][static_cast<std::size_t>(i)];
                int n2 = gen.nodeIds[static_cast<std::size_t>(k)][static_cast<std::size_t>(j + 1)][static_cast<std::size_t>(i)];
                int id = nextElementId++;
                model.addElement(Element(id, ElementKind::Beam, {n1, n2}, p.materialId, beamSectionId,
                                          "BY_F" + std::to_string(k) + "_" + std::to_string(j) + "_" + std::to_string(i)));
                gen.beamElementIds.push_back(id);
                double w = unitWeight * beamArea * p.bayWidthYm;
                addNodeWeight(n1, w / 2.0);
                addNodeWeight(n2, w / 2.0);
            }
        }
    }
    gen.idRanges.beamsCount = static_cast<int>(gen.beamElementIds.size());

    // ---- Slabs: one shell panel per bay, at every floor level above the base ----
    if (p.generateSlabs) {
        int slabSectionId = model.addSection(Section::shell(p.slabThicknessM, "AutoGen_Slab"));
        gen.idRanges.slabsFirst = nextElementId;
        for (int k = 1; k < floors; ++k) {
            for (int j = 0; j < p.baysY; ++j) {
                for (int i = 0; i < p.baysX; ++i) {
                    int n1 = gen.nodeIds[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)][static_cast<std::size_t>(i)];
                    int n2 = gen.nodeIds[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)][static_cast<std::size_t>(i + 1)];
                    int n3 = gen.nodeIds[static_cast<std::size_t>(k)][static_cast<std::size_t>(j + 1)][static_cast<std::size_t>(i + 1)];
                    int n4 = gen.nodeIds[static_cast<std::size_t>(k)][static_cast<std::size_t>(j + 1)][static_cast<std::size_t>(i)];
                    int id = nextElementId++;
                    model.addElement(Element(id, ElementKind::Slab, {n1, n2, n3, n4}, p.materialId, slabSectionId,
                                              "S_F" + std::to_string(k) + "_" + std::to_string(j) + "_" + std::to_string(i)));
                    gen.slabElementIds.push_back(id);
                    double w = unitWeight * p.slabThicknessM * p.bayWidthXm * p.bayWidthYm;
                    addNodeWeight(n1, w / 4.0);
                    addNodeWeight(n2, w / 4.0);
                    addNodeWeight(n3, w / 4.0);
                    addNodeWeight(n4, w / 4.0);
                }
            }
        }
        gen.idRanges.slabsCount = static_cast<int>(gen.slabElementIds.size());
    }

    // ---- Exterior (perimeter) walls: every bay along the four outer
    // boundary edges, every floor, no openings (see class doc comment) ----
    if (p.generateExteriorWalls) {
        int wallSectionId = model.addSection(Section::shell(p.wallThicknessM, "AutoGen_Wall"));
        gen.idRanges.wallsFirst = nextElementId;
        for (int k = 0; k < p.storeys; ++k) {
            // South (j=0) and north (j=rows-1) boundary edges: X-direction wall panels.
            for (int jBoundary : {0, rows - 1}) {
                for (int i = 0; i < p.baysX; ++i) {
                    int n1 = gen.nodeIds[static_cast<std::size_t>(k)][static_cast<std::size_t>(jBoundary)][static_cast<std::size_t>(i)];
                    int n2 = gen.nodeIds[static_cast<std::size_t>(k)][static_cast<std::size_t>(jBoundary)][static_cast<std::size_t>(i + 1)];
                    int n3 = gen.nodeIds[static_cast<std::size_t>(k + 1)][static_cast<std::size_t>(jBoundary)][static_cast<std::size_t>(i + 1)];
                    int n4 = gen.nodeIds[static_cast<std::size_t>(k + 1)][static_cast<std::size_t>(jBoundary)][static_cast<std::size_t>(i)];
                    int id = nextElementId++;
                    model.addElement(Element(id, ElementKind::Wall, {n1, n2, n3, n4}, p.materialId, wallSectionId,
                                              "WX_F" + std::to_string(k + 1) + "_" + std::to_string(jBoundary) + "_" + std::to_string(i)));
                    gen.wallElementIds.push_back(id);
                    double w = unitWeight * p.wallThicknessM * p.bayWidthXm * p.floorHeightM;
                    addNodeWeight(n1, w / 4.0);
                    addNodeWeight(n2, w / 4.0);
                    addNodeWeight(n3, w / 4.0);
                    addNodeWeight(n4, w / 4.0);
                }
            }
            // West (i=0) and east (i=cols-1) boundary edges: Y-direction wall panels.
            // Corner bays already got an X-direction panel above; this adds
            // the PERPENDICULAR panel meeting it at the corner, not a
            // duplicate of it (different plane, different node pairs).
            for (int iBoundary : {0, cols - 1}) {
                for (int j = 0; j < p.baysY; ++j) {
                    int n1 = gen.nodeIds[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)][static_cast<std::size_t>(iBoundary)];
                    int n2 = gen.nodeIds[static_cast<std::size_t>(k)][static_cast<std::size_t>(j + 1)][static_cast<std::size_t>(iBoundary)];
                    int n3 = gen.nodeIds[static_cast<std::size_t>(k + 1)][static_cast<std::size_t>(j + 1)][static_cast<std::size_t>(iBoundary)];
                    int n4 = gen.nodeIds[static_cast<std::size_t>(k + 1)][static_cast<std::size_t>(j)][static_cast<std::size_t>(iBoundary)];
                    int id = nextElementId++;
                    model.addElement(Element(id, ElementKind::Wall, {n1, n2, n3, n4}, p.materialId, wallSectionId,
                                              "WY_F" + std::to_string(k + 1) + "_" + std::to_string(iBoundary) + "_" + std::to_string(j)));
                    gen.wallElementIds.push_back(id);
                    double w = unitWeight * p.wallThicknessM * p.bayWidthYm * p.floorHeightM;
                    addNodeWeight(n1, w / 4.0);
                    addNodeWeight(n2, w / 4.0);
                    addNodeWeight(n3, w / 4.0);
                    addNodeWeight(n4, w / 4.0);
                }
            }
        }
        gen.idRanges.wallsCount = static_cast<int>(gen.wallElementIds.size());
    }

    // ---- Self-weight dead load: apply the accumulated per-node totals ----
    if (p.generateSelfWeightLoad) {
        for (const auto& [nodeId, w] : nodeSelfWeightKN) {
            NodalLoad load;
            load.nodeId = nodeId;
            load.loadCaseId = p.selfWeightLoadCaseId;
            load.Fz = -w;
            model.addNodalLoad(load);
        }
    }

    return gen;
}

}  // namespace nrsa::modeler
