#include "io/IfcIO.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "core/Element.h"
#include "core/Node.h"

namespace nrsa::io {

namespace {

// NOT the real IFC GUID compression algorithm (which packs a 128-bit
// UUID into this alphabet) -- see class doc comment. This just gives
// every entity a distinct, fixed-format, 22-character string, unique
// WITHIN one exported file, built directly from its STEP entity number.
std::string pseudoGuid(int entityId) {
    static const char* alphabet =
        "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz_$";
    std::string s(22, '0');
    unsigned long long v = static_cast<unsigned long long>(entityId);
    for (int i = 21; i >= 0 && v > 0; --i) {
        s[static_cast<std::size_t>(i)] = alphabet[v % 64];
        v /= 64;
    }
    return s;
}

std::string ifcElementEntityName(ElementKind kind) {
    switch (kind) {
        case ElementKind::Beam: return "IFCBEAM";
        case ElementKind::Column: return "IFCCOLUMN";
        case ElementKind::Brace: return "IFCMEMBER";
        case ElementKind::Wall: return "IFCWALL";
        case ElementKind::Core: return "IFCWALL";
        case ElementKind::Slab: return "IFCSLAB";
        case ElementKind::Spring: return "IFCMEMBER";
        case ElementKind::Rigid: return "IFCMEMBER";
    }
    return "IFCMEMBER";
}

// Minimal STEP-file writer: tracks the next free entity number and
// writes "#N = ENTITY(...)" lines, returning N so callers can
// cross-reference it in a later entity.
class StepWriter {
public:
    explicit StepWriter(std::ofstream& out) : out_(out) {}

    int emit(const std::string& entityText) {
        int id = next_++;
        out_ << '#' << id << "=" << entityText << ";\n";
        return id;
    }

private:
    std::ofstream& out_;
    int next_ = 1;
};

std::string point(double x, double y, double z) {
    std::ostringstream ss;
    ss << "IFCCARTESIANPOINT((" << x << ',' << y << ',' << z << "))";
    return ss.str();
}

}  // namespace

namespace {

void writeIfcBody(std::ofstream& f, const Model& model, const std::string& outputPath,
                   const std::string& projectName,
                   const std::unordered_map<int, std::vector<std::pair<std::string, std::string>>>*
                       elementProperties) {
    f << "ISO-10303-21;\n"
      << "HEADER;\n"
      << "FILE_DESCRIPTION((''),'2;1');\n"
      << "FILE_NAME('" << outputPath << "','',(''),(''),'NRSA IfcIO exporter (minimal, axis-only geometry)','','');\n"
      << "FILE_SCHEMA(('IFC4'));\n"
      << "ENDSEC;\n"
      << "DATA;\n";

    StepWriter w(f);

    int origin = w.emit(point(0, 0, 0));
    int zDir = w.emit("IFCDIRECTION((0.,0.,1.))");
    int xDir = w.emit("IFCDIRECTION((1.,0.,0.))");
    int worldPlacement = w.emit("IFCAXIS2PLACEMENT3D(#" + std::to_string(origin) + ",#" +
                                 std::to_string(zDir) + ",#" + std::to_string(xDir) + ")");
    int context = w.emit("IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,1.0E-5,#" +
                          std::to_string(worldPlacement) + ",$)");
    int lengthUnit = w.emit("IFCSIUNIT(*,.LENGTHUNIT.,$,.METRE.)");
    int unitAssignment = w.emit("IFCUNITASSIGNMENT((#" + std::to_string(lengthUnit) + "))");

    int project = w.emit("IFCPROJECT('" + pseudoGuid(1) + "',$,'" + projectName +
                          "',$,$,$,$,(#" + std::to_string(context) + "),#" +
                          std::to_string(unitAssignment) + ")");

    int sitePlacement = w.emit("IFCLOCALPLACEMENT($,#" + std::to_string(worldPlacement) + ")");
    int site = w.emit("IFCSITE('" + pseudoGuid(2) + "',$,'Site',$,$,#" + std::to_string(sitePlacement) +
                       ",$,$,.ELEMENT.,$,$,$,$,$)");

    int buildingPlacement = w.emit("IFCLOCALPLACEMENT(#" + std::to_string(sitePlacement) + ",#" +
                                    std::to_string(worldPlacement) + ")");
    int building = w.emit("IFCBUILDING('" + pseudoGuid(3) +
                           "',$,'Building',$,$,#" + std::to_string(buildingPlacement) +
                           ",$,$,.ELEMENT.,$,$,$)");

    int storeyPlacement = w.emit("IFCLOCALPLACEMENT(#" + std::to_string(buildingPlacement) + ",#" +
                                  std::to_string(worldPlacement) + ")");
    int storey = w.emit("IFCBUILDINGSTOREY('" + pseudoGuid(4) +
                         "',$,'Storey',$,$,#" + std::to_string(storeyPlacement) +
                         ",$,$,.ELEMENT.,0.)");

    w.emit("IFCRELAGGREGATES('" + pseudoGuid(5) + "',$,$,$,#" + std::to_string(project) +
            ",(#" + std::to_string(site) + "))");
    w.emit("IFCRELAGGREGATES('" + pseudoGuid(6) + "',$,$,$,#" + std::to_string(site) +
            ",(#" + std::to_string(building) + "))");
    w.emit("IFCRELAGGREGATES('" + pseudoGuid(7) + "',$,$,$,#" + std::to_string(building) +
            ",(#" + std::to_string(storey) + "))");

    std::vector<int> elementIds;
    int propSetCounter = 0;
    for (const auto& e : model.elements()) {
        std::vector<int> pts;
        for (std::size_t i = 0; i < e.nodeCount(); ++i) {
            const Node& n = model.node(e.nodeId(i));
            pts.push_back(w.emit(point(n.x(), n.y(), n.z())));
        }
        if (e.nodeCount() == 4) pts.push_back(pts[0]);  // close the boundary loop

        std::ostringstream ptList;
        for (std::size_t i = 0; i < pts.size(); ++i) {
            if (i > 0) ptList << ',';
            ptList << '#' << pts[i];
        }
        int polyline = w.emit("IFCPOLYLINE((" + ptList.str() + "))");
        std::string repType = e.nodeCount() == 2 ? "Axis" : "FootPrint";
        int shapeRep = w.emit("IFCSHAPEREPRESENTATION(#" + std::to_string(context) + ",'" + repType +
                               "','Curve3D',(#" + std::to_string(polyline) + "))");
        int shape = w.emit("IFCPRODUCTDEFINITIONSHAPE($,$,(#" + std::to_string(shapeRep) + "))");
        int placement = w.emit("IFCLOCALPLACEMENT(#" + std::to_string(storeyPlacement) + ",#" +
                                std::to_string(worldPlacement) + ")");

        std::string entityName = ifcElementEntityName(e.kind());
        int elementId = w.emit(entityName + "('" + pseudoGuid(1000 + e.id()) + "',$,'" + e.label() +
                                "',$,$,#" + std::to_string(placement) + ",#" + std::to_string(shape) + ",$)");
        elementIds.push_back(elementId);

        // ---- BIM property set attachment (roadmap Section 14) ----
        if (elementProperties) {
            auto it = elementProperties->find(e.id());
            if (it != elementProperties->end() && !it->second.empty()) {
                std::vector<int> propIds;
                for (const auto& [name, value] : it->second) {
                    int prop = w.emit("IFCPROPERTYSINGLEVALUE('" + name + "',$,IFCTEXT('" + value + "'),$)");
                    propIds.push_back(prop);
                }
                std::ostringstream propList;
                for (std::size_t i = 0; i < propIds.size(); ++i) {
                    if (i > 0) propList << ',';
                    propList << '#' << propIds[i];
                }
                int propSet = w.emit("IFCPROPERTYSET('" + pseudoGuid(2000 + e.id()) +
                                      "',$,'NRSA Design Data',$,(" + propList.str() + "))");
                w.emit("IFCRELDEFINESBYPROPERTIES('" + pseudoGuid(3000 + e.id()) + "',$,$,$,(#" +
                        std::to_string(elementId) + "),#" + std::to_string(propSet) + ")");
                propSetCounter++;
            }
        }
    }
    (void)propSetCounter;

    if (!elementIds.empty()) {
        std::ostringstream idList;
        for (std::size_t i = 0; i < elementIds.size(); ++i) {
            if (i > 0) idList << ',';
            idList << '#' << elementIds[i];
        }
        w.emit("IFCRELCONTAINEDINSPATIALSTRUCTURE('" + pseudoGuid(999999) + "',$,$,$,(" + idList.str() +
                "),#" + std::to_string(storey) + ")");
    }

    f << "ENDSEC;\nEND-ISO-10303-21;\n";
}

}  // namespace

void exportIfc(const Model& model, const std::string& outputPath, const std::string& projectName) {
    std::ofstream f(outputPath);
    if (!f) throw std::runtime_error("exportIfc: cannot open '" + outputPath + "' for writing");
    writeIfcBody(f, model, outputPath, projectName, nullptr);
}

void exportIfcWithProperties(const Model& model, const std::string& outputPath,
                              const std::unordered_map<int, std::vector<std::pair<std::string, std::string>>>&
                                  elementProperties,
                              const std::string& projectName) {
    std::ofstream f(outputPath);
    if (!f) throw std::runtime_error("exportIfcWithProperties: cannot open '" + outputPath + "' for writing");
    writeIfcBody(f, model, outputPath, projectName, &elementProperties);
}

}  // namespace nrsa::io
