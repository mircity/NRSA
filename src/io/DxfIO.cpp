#include "io/DxfIO.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include "core/Element.h"
#include "core/Node.h"

namespace nrsa::io {

namespace {

std::string layerForKind(ElementKind kind) {
    switch (kind) {
        case ElementKind::Beam: return "NRSA_BEAM";
        case ElementKind::Column: return "NRSA_COLUMN";
        case ElementKind::Wall: return "NRSA_WALL";
        case ElementKind::Core: return "NRSA_CORE";
        case ElementKind::Slab: return "NRSA_SLAB";
        case ElementKind::Brace: return "NRSA_BRACE";
        case ElementKind::Spring: return "NRSA_SPRING";
        case ElementKind::Rigid: return "NRSA_RIGID";
    }
    return "NRSA_BEAM";
}

bool kindForLayer(const std::string& layer, ElementKind& outKind) {
    static const std::pair<const char*, ElementKind> kMap[] = {
        {"NRSA_BEAM", ElementKind::Beam},     {"NRSA_COLUMN", ElementKind::Column},
        {"NRSA_WALL", ElementKind::Wall},     {"NRSA_CORE", ElementKind::Core},
        {"NRSA_SLAB", ElementKind::Slab},     {"NRSA_BRACE", ElementKind::Brace},
        {"NRSA_SPRING", ElementKind::Spring}, {"NRSA_RIGID", ElementKind::Rigid},
    };
    for (const auto& [name, kind] : kMap) {
        if (layer == name) { outKind = kind; return true; }
    }
    return false;
}

void writePoint(std::ofstream& f, const std::string& layer, double x, double y, double z) {
    f << "0\nPOINT\n8\n" << layer << "\n10\n" << x << "\n20\n" << y << "\n30\n" << z << "\n";
}

void writeLine(std::ofstream& f, const std::string& layer, double x1, double y1, double z1, double x2,
               double y2, double z2) {
    f << "0\nLINE\n8\n" << layer << "\n10\n" << x1 << "\n20\n" << y1 << "\n30\n" << z1
      << "\n11\n" << x2 << "\n21\n" << y2 << "\n31\n" << z2 << "\n";
}

}  // namespace

void exportDxf(const Model& model, const std::string& outputPath) {
    std::ofstream f(outputPath);
    if (!f) throw std::runtime_error("exportDxf: cannot open '" + outputPath + "' for writing");

    f << "0\nSECTION\n2\nENTITIES\n";

    for (const auto& n : model.nodes()) {
        writePoint(f, "NRSA_NODE", n.x(), n.y(), n.z());
    }

    for (const auto& e : model.elements()) {
        std::string layer = layerForKind(e.kind());
        if (e.nodeCount() == 2) {
            const Node& a = model.node(e.nodeId(0));
            const Node& b = model.node(e.nodeId(1));
            writeLine(f, layer, a.x(), a.y(), a.z(), b.x(), b.y(), b.z());
        } else if (e.nodeCount() == 4) {
            // 4 boundary edges, since this minimal DXF subset has no
            // quad-face primitive (see class doc comment).
            for (std::size_t k = 0; k < 4; ++k) {
                const Node& a = model.node(e.nodeId(k));
                const Node& b = model.node(e.nodeId((k + 1) % 4));
                writeLine(f, layer, a.x(), a.y(), a.z(), b.x(), b.y(), b.z());
            }
        }
        // Elements with other node counts are silently skipped -- this
        // minimal DXF subset only knows POINT and LINE primitives.
    }

    f << "0\nENDSEC\n0\nEOF\n";
}

DxfImportResult importDxf(Model& model, const std::string& inputPath, int materialId, int sectionId,
                           int& nextNodeId, int& nextElementId) {
    std::ifstream f(inputPath);
    if (!f) throw std::runtime_error("importDxf: cannot open '" + inputPath + "' for reading");

    DxfImportResult result;

    std::string codeLine, valueLine;
    std::string currentType;
    std::string currentLayer = "0";
    double x1 = 0, y1 = 0, z1 = 0, x2 = 0, y2 = 0, z2 = 0;

    auto flush = [&]() {
        if (currentType == "POINT") {
            int id = nextNodeId++;
            model.addNode(Node(id, x1, y1, z1, "DXF_Import_" + std::to_string(id)));
            result.nodesImported++;
        } else if (currentType == "LINE") {
            int nA = nextNodeId++;
            model.addNode(Node(nA, x1, y1, z1, "DXF_Import_" + std::to_string(nA)));
            int nB = nextNodeId++;
            model.addNode(Node(nB, x2, y2, z2, "DXF_Import_" + std::to_string(nB)));
            result.nodesImported += 2;

            ElementKind kind;
            if (!kindForLayer(currentLayer, kind)) {
                kind = ElementKind::Beam;
                if (currentLayer != "NRSA_NODE") {
                    result.unrecognizedLayers.push_back(currentLayer);
                }
            }
            int id = nextElementId++;
            model.addElement(Element(id, kind, {nA, nB}, materialId, sectionId,
                                      "DXF_Import_" + std::to_string(id)));
            result.elementsImported++;
        }
        currentType.clear();
        currentLayer = "0";
        x1 = y1 = z1 = x2 = y2 = z2 = 0.0;
    };

    while (std::getline(f, codeLine) && std::getline(f, valueLine)) {
        // Trim trailing carriage returns (files saved with CRLF line endings).
        if (!codeLine.empty() && codeLine.back() == '\r') codeLine.pop_back();
        if (!valueLine.empty() && valueLine.back() == '\r') valueLine.pop_back();

        int code;
        try {
            code = std::stoi(codeLine);
        } catch (const std::exception&) {
            throw std::invalid_argument("importDxf: malformed group code '" + codeLine + "'");
        }

        if (code == 0) {
            flush();
            if (valueLine == "LINE" || valueLine == "POINT") currentType = valueLine;
            continue;
        }
        if (currentType.empty()) continue;  // outside any LINE/POINT entity -- ignore (HEADER/TABLES/etc.)

        try {
            switch (code) {
                case 8: currentLayer = valueLine; break;
                case 10: x1 = std::stod(valueLine); break;
                case 20: y1 = std::stod(valueLine); break;
                case 30: z1 = std::stod(valueLine); break;
                case 11: x2 = std::stod(valueLine); break;
                case 21: y2 = std::stod(valueLine); break;
                case 31: z2 = std::stod(valueLine); break;
                default: break;  // ignore any other group code in this minimal subset
            }
        } catch (const std::exception&) {
            throw std::invalid_argument("importDxf: malformed coordinate value '" + valueLine + "'");
        }
    }
    flush();  // in case the file doesn't end with an explicit ENDSEC/EOF pair

    return result;
}

}  // namespace nrsa::io
