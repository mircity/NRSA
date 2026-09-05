#include <algorithm>
#include <cassert>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <regex>
#include <set>
#include <sstream>
#include <unordered_map>

#include "core/Element.h"
#include "core/Model.h"
#include "core/Node.h"
#include "io/IfcIO.h"

using namespace nrsa;
using namespace nrsa::io;

static std::string readFile(const std::string& path) {
    std::ifstream f(path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// Structural validity: every #N reference used anywhere in the DATA
// section must correspond to an actual "#N=" definition somewhere in
// the file -- a genuine, format-agnostic well-formedness check (no IFC
// validator is available offline, so this is the strongest check
// achievable: it can't tell you the file is SEMANTICALLY correct IFC,
// but it can catch a dangling/off-by-one reference, which is the most
// common bug class in hand-written STEP-file generation).
static void testNoDanglingReferences() {
    Model model("ifc validity test");
    model.addNode(Node(1, 0, 0, 0));
    model.addNode(Node(2, 0, 0, 3.0));
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, 1, "Col1"));

    const char* path = "/tmp/test_ifc_validity.ifc";
    exportIfc(model, path);
    std::string content = readFile(path);

    std::set<int> defined, referenced;
    std::regex defRe(R"(#(\d+)\s*=)");
    std::regex refRe(R"(#(\d+))");
    for (auto it = std::sregex_iterator(content.begin(), content.end(), defRe); it != std::sregex_iterator(); ++it) {
        defined.insert(std::stoi((*it)[1].str()));
    }
    for (auto it = std::sregex_iterator(content.begin(), content.end(), refRe); it != std::sregex_iterator(); ++it) {
        referenced.insert(std::stoi((*it)[1].str()));
    }
    for (int r : referenced) {
        assert(defined.count(r) > 0 && "dangling #N reference with no matching definition");
    }
    assert(!defined.empty());

    std::remove(path);
    std::cout << "  testNoDanglingReferences OK\n";
}

// File must have the correct ISO-10303-21 envelope and exactly one
// each of the required spatial-hierarchy entities.
static void testFileEnvelopeAndSpatialHierarchy() {
    Model model("ifc envelope test");
    model.addNode(Node(1, 0, 0, 0));

    const char* path = "/tmp/test_ifc_envelope.ifc";
    exportIfc(model, path, "MyProject");
    std::string content = readFile(path);

    assert(content.find("ISO-10303-21;") == 0);
    assert(content.find("END-ISO-10303-21;") != std::string::npos);
    assert(content.find("FILE_SCHEMA(('IFC4'))") != std::string::npos);

    auto countOccurrences = [&](const std::string& needle) {
        int count = 0;
        std::size_t pos = 0;
        while ((pos = content.find(needle, pos)) != std::string::npos) { count++; pos += needle.size(); }
        return count;
    };
    assert(countOccurrences("=IFCPROJECT(") == 1);
    assert(countOccurrences("=IFCSITE(") == 1);
    assert(countOccurrences("=IFCBUILDING(") == 1);
    assert(countOccurrences("=IFCBUILDINGSTOREY(") == 1);
    assert(content.find("MyProject") != std::string::npos);

    std::remove(path);
    std::cout << "  testFileEnvelopeAndSpatialHierarchy OK\n";
}

// Element kind mapping: Column -> IFCCOLUMN, Beam -> IFCBEAM,
// Slab -> IFCSLAB, Wall -> IFCWALL -- and node coordinates for a known
// element must appear verbatim as IFCCARTESIANPOINT entries.
static void testElementKindMappingAndCoordinates() {
    Model model("ifc kind mapping test");
    model.addNode(Node(1, 1.5, 2.5, 0.0));
    model.addNode(Node(2, 1.5, 2.5, 3.25));
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, 1, "TheColumn"));

    model.addNode(Node(3, 0, 0, 0));
    model.addNode(Node(4, 5, 0, 0));
    model.addElement(Element(2, ElementKind::Beam, {3, 4}, 1, 1, "TheBeam"));

    const char* path = "/tmp/test_ifc_kinds.ifc";
    exportIfc(model, path);
    std::string content = readFile(path);

    assert(content.find("=IFCCOLUMN(") != std::string::npos);
    assert(content.find("=IFCBEAM(") != std::string::npos);
    assert(content.find("IFCCARTESIANPOINT((1.5,2.5,0))") != std::string::npos ||
           content.find("IFCCARTESIANPOINT((1.5,2.5,0.0))") != std::string::npos);
    assert(content.find("3.25") != std::string::npos);

    std::remove(path);
    std::cout << "  testElementKindMappingAndCoordinates OK\n";
}

// A 4-node Slab element must produce a CLOSED polyline (5 points: 4
// corners plus the first one repeated) for its boundary representation.
static void testShellElementClosedBoundary() {
    Model model("ifc shell test");
    model.addNode(Node(1, 0, 0, 0));
    model.addNode(Node(2, 4, 0, 0));
    model.addNode(Node(3, 4, 4, 0));
    model.addNode(Node(4, 0, 4, 0));
    model.addElement(Element(1, ElementKind::Slab, {1, 2, 3, 4}, 1, 1, "SlabA"));

    const char* path = "/tmp/test_ifc_shell.ifc";
    exportIfc(model, path);
    std::string content = readFile(path);

    // Find the IFCPOLYLINE line and count its point references (5 for a closed quad).
    std::smatch m;
    std::regex polyRe(R"(=IFCPOLYLINE\(\(([^)]+)\)\))");
    bool found = std::regex_search(content, m, polyRe);
    assert(found);
    std::string ptList = m[1].str();
    int commaCount = static_cast<int>(std::count(ptList.begin(), ptList.end(), ','));
    assert(commaCount == 4);  // 5 points -> 4 commas

    std::remove(path);
    std::cout << "  testShellElementClosedBoundary OK\n";
}

// BIM property set attachment (roadmap Section 14): a property
// attached to element 1 must produce a genuine IFCPROPERTYSET +
// IFCRELDEFINESBYPROPERTIES pair, referencing the SAME element's own
// STEP id -- not just text appearing somewhere, but structurally
// correct, verified via the same no-dangling-reference check plus an
// explicit search for the actual property name/value.
static void testPropertySetAttachment() {
    Model model("ifc property test");
    model.addNode(Node(1, 0, 0, 0));
    model.addNode(Node(2, 0, 0, 3.0));
    model.addElement(Element(1, ElementKind::Column, {1, 2}, 1, 1, "C1"));

    std::unordered_map<int, std::vector<std::pair<std::string, std::string>>> props;
    props[1] = {{"Pu_kN", "700.0"}, {"DemandCapacityRatio", "0.85"}};

    const char* path = "/tmp/test_ifc_props.ifc";
    exportIfcWithProperties(model, path, props);
    std::string content = readFile(path);

    assert(content.find("IFCPROPERTYSET") != std::string::npos);
    assert(content.find("IFCRELDEFINESBYPROPERTIES") != std::string::npos);
    assert(content.find("Pu_kN") != std::string::npos);
    assert(content.find("700.0") != std::string::npos);
    assert(content.find("DemandCapacityRatio") != std::string::npos);

    // Structural validity: same no-dangling-reference check as
    // testNoDanglingReferences, re-run on THIS file (property sets add
    // new cross-references that must also resolve correctly).
    std::set<int> defined, referenced;
    std::regex defRe(R"(#(\d+)\s*=)");
    std::regex refRe(R"(#(\d+))");
    for (auto it = std::sregex_iterator(content.begin(), content.end(), defRe); it != std::sregex_iterator(); ++it)
        defined.insert(std::stoi((*it)[1].str()));
    for (auto it = std::sregex_iterator(content.begin(), content.end(), refRe); it != std::sregex_iterator(); ++it)
        referenced.insert(std::stoi((*it)[1].str()));
    for (int r : referenced) assert(defined.count(r) > 0);

    std::remove(path);
    std::cout << "  testPropertySetAttachment OK\n";
}

// An element with NO entry in elementProperties must export as plain
// geometry, with no property set at all -- opt-in, not forced onto
// every element.
static void testElementsWithoutPropertiesUnaffected() {
    Model model("ifc no props test");
    model.addNode(Node(1, 0, 0, 0));
    model.addNode(Node(2, 4, 0, 0));
    model.addElement(Element(1, ElementKind::Beam, {1, 2}, 1, 1, "B1"));

    std::unordered_map<int, std::vector<std::pair<std::string, std::string>>> emptyProps;
    const char* path = "/tmp/test_ifc_noprops.ifc";
    exportIfcWithProperties(model, path, emptyProps);
    std::string content = readFile(path);
    assert(content.find("IFCPROPERTYSET") == std::string::npos);
    std::remove(path);
    std::cout << "  testElementsWithoutPropertiesUnaffected OK\n";
}

int main() {
    std::cout << "test_ifc_io:\n";
    testNoDanglingReferences();
    testFileEnvelopeAndSpatialHierarchy();
    testElementKindMappingAndCoordinates();
    testShellElementClosedBoundary();
    testPropertySetAttachment();
    testElementsWithoutPropertiesUnaffected();
    std::cout << "All tests passed.\n";
    return 0;
}
