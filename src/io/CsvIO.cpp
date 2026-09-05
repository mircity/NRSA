#include "io/CsvIO.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "core/Element.h"
#include "core/Node.h"

namespace nrsa::io {

namespace {

std::string kindToString(ElementKind k) {
    switch (k) {
        case ElementKind::Beam: return "Beam";
        case ElementKind::Column: return "Column";
        case ElementKind::Wall: return "Wall";
        case ElementKind::Core: return "Core";
        case ElementKind::Slab: return "Slab";
        case ElementKind::Brace: return "Brace";
        case ElementKind::Spring: return "Spring";
        case ElementKind::Rigid: return "Rigid";
    }
    return "Beam";
}

bool stringToKind(const std::string& s, ElementKind& out) {
    static const std::pair<const char*, ElementKind> kMap[] = {
        {"Beam", ElementKind::Beam}, {"Column", ElementKind::Column}, {"Wall", ElementKind::Wall},
        {"Core", ElementKind::Core}, {"Slab", ElementKind::Slab},     {"Brace", ElementKind::Brace},
        {"Spring", ElementKind::Spring}, {"Rigid", ElementKind::Rigid},
    };
    for (const auto& [name, k] : kMap) {
        if (s == name) { out = k; return true; }
    }
    return false;
}

std::vector<std::string> splitOn(const std::string& s, char delim) {
    // Manual index-based split (not getline-based): getline(ss, item, delim)
    // silently drops a trailing empty field when the line ends right at a
    // delimiter (e.g. a row whose last column, like an empty label, is
    // blank) -- caught by test_csv_io.cpp's shell-element round-trip,
    // which has exactly that shape ("...,0,0," with nothing after the
    // final comma).
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (true) {
        std::size_t pos = s.find(delim, start);
        if (pos == std::string::npos) {
            parts.push_back(s.substr(start));
            break;
        }
        parts.push_back(s.substr(start, pos - start));
        start = pos + 1;
    }
    return parts;
}

std::string trimCr(std::string s) {
    if (!s.empty() && s.back() == '\r') s.pop_back();
    return s;
}

}  // namespace

void exportCsv(const Model& model, const std::string& nodesPath, const std::string& elementsPath) {
    std::ofstream nf(nodesPath);
    if (!nf) throw std::runtime_error("exportCsv: cannot open '" + nodesPath + "' for writing");
    nf << "id,x,y,z,restrainUx,restrainUy,restrainUz,restrainRx,restrainRy,restrainRz,"
          "translationalMass,rotMassX,rotMassY,rotMassZ,label\n";
    for (const auto& n : model.nodes()) {
        auto rm = n.rotationalMass();
        nf << n.id() << ',' << n.x() << ',' << n.y() << ',' << n.z() << ','
           << n.isRestrained(DOF::Ux) << ',' << n.isRestrained(DOF::Uy) << ',' << n.isRestrained(DOF::Uz) << ','
           << n.isRestrained(DOF::Rx) << ',' << n.isRestrained(DOF::Ry) << ',' << n.isRestrained(DOF::Rz) << ','
           << n.translationalMass() << ',' << rm[0] << ',' << rm[1] << ',' << rm[2] << ',' << n.label() << '\n';
    }

    std::ofstream ef(elementsPath);
    if (!ef) throw std::runtime_error("exportCsv: cannot open '" + elementsPath + "' for writing");
    ef << "id,kind,materialId,sectionId,nodeIds,label\n";
    for (const auto& e : model.elements()) {
        ef << e.id() << ',' << kindToString(e.kind()) << ',' << e.materialId() << ',' << e.sectionId() << ',';
        for (std::size_t i = 0; i < e.nodeCount(); ++i) {
            if (i > 0) ef << ';';
            ef << e.nodeId(i);
        }
        ef << ',' << e.label() << '\n';
    }
}

CsvImportResult importCsv(Model& model, const std::string& nodesPath, const std::string& elementsPath) {
    CsvImportResult result;

    std::ifstream nf(nodesPath);
    if (!nf) throw std::runtime_error("importCsv: cannot open '" + nodesPath + "' for reading");
    std::string line;
    std::getline(nf, line);  // header
    while (std::getline(nf, line)) {
        line = trimCr(line);
        if (line.empty()) continue;
        auto cols = splitOn(line, ',');
        if (cols.size() < 15) {
            throw std::invalid_argument("importCsv: malformed node row (expected 15 columns): " + line);
        }
        try {
            int id = std::stoi(cols[0]);
            double x = std::stod(cols[1]), y = std::stod(cols[2]), z = std::stod(cols[3]);
            Node n(id, x, y, z, cols[14]);
            n.restrain(DOF::Ux, cols[4] == "1");
            n.restrain(DOF::Uy, cols[5] == "1");
            n.restrain(DOF::Uz, cols[6] == "1");
            n.restrain(DOF::Rx, cols[7] == "1");
            n.restrain(DOF::Ry, cols[8] == "1");
            n.restrain(DOF::Rz, cols[9] == "1");
            n.setTranslationalMass(std::stod(cols[10]));
            n.setRotationalMass(std::stod(cols[11]), std::stod(cols[12]), std::stod(cols[13]));
            model.addNode(n);
            result.nodesImported++;
        } catch (const std::exception&) {
            throw std::invalid_argument("importCsv: malformed node row: " + line);
        }
    }

    std::ifstream ef(elementsPath);
    if (!ef) throw std::runtime_error("importCsv: cannot open '" + elementsPath + "' for reading");
    std::getline(ef, line);  // header
    while (std::getline(ef, line)) {
        line = trimCr(line);
        if (line.empty()) continue;
        auto cols = splitOn(line, ',');
        if (cols.size() < 6) {
            throw std::invalid_argument("importCsv: malformed element row (expected 6 columns): " + line);
        }
        try {
            int id = std::stoi(cols[0]);
            ElementKind kind;
            if (!stringToKind(cols[1], kind)) {
                throw std::invalid_argument("importCsv: unrecognized element kind '" + cols[1] + "'");
            }
            int materialId = std::stoi(cols[2]);
            int sectionId = std::stoi(cols[3]);
            auto nodeIdStrs = splitOn(cols[4], ';');
            std::vector<int> nodeIds;
            for (const auto& s : nodeIdStrs) nodeIds.push_back(std::stoi(s));
            std::string label = cols.size() > 5 ? cols[5] : "";
            model.addElement(Element(id, kind, nodeIds, materialId, sectionId, label));
            result.elementsImported++;
        } catch (const std::invalid_argument&) {
            throw;
        } catch (const std::exception&) {
            throw std::invalid_argument("importCsv: malformed element row: " + line);
        }
    }

    return result;
}

}  // namespace nrsa::io
