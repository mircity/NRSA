#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <set>

#include "core/Load.h"
#include "core/Material.h"
#include "core/Model.h"
#include "modeler/AutoModelGenerator.h"

using namespace nrsa;
using namespace nrsa::modeler;

static bool approxEqual(double a, double b, double relTol = 1e-6) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// The exact worked example from the roadmap document itself (Section 2,
// "Auto Model Generation" bullet): "10-storey, 4 bays x 5 bays, Floor
// height = 3.2 m, Column = 450x450, Beam = 300x500" -- NRSA is supposed
// to generate Grid -> Columns -> Beams -> Slabs -> Walls -> Loads
// automatically from just those parameters. This test runs exactly
// that example and hand-calculates the expected node/column/beam/slab
// counts from first principles (grid combinatorics), matching them
// against the generator's actual output.
static void testRoadmapWorkedExampleHandCalc() {
    Model model("roadmap worked example");
    Material conc = Material::concrete(1, 28.0);
    model.addMaterial(conc);

    BuildingGenerationParameters params;
    params.storeys = 10;
    params.baysX = 4;
    params.baysY = 5;
    params.bayWidthXm = 6.0;   // roadmap doesn't state a bay width; any positive value works
    params.bayWidthYm = 6.0;
    params.floorHeightM = 3.2;
    params.columnWidthM = 0.45;
    params.columnDepthM = 0.45;
    params.beamWidthM = 0.3;
    params.beamDepthM = 0.5;
    params.materialId = 1;

    auto gen = runAutoModelGeneration(model, params);

    // Hand-calc: grid nodes = (baysX+1)*(baysY+1)*(storeys+1)
    int expectedNodes = (4 + 1) * (5 + 1) * (10 + 1);  // 5*6*11 = 330
    assert(expectedNodes == 330);
    int actualNodeCount = 0;
    for (const auto& floor : gen.nodeIds)
        for (const auto& row : floor)
            actualNodeCount += static_cast<int>(row.size());
    assert(actualNodeCount == expectedNodes);
    assert(static_cast<int>(model.nodes().size()) == expectedNodes);

    // Hand-calc: columns = (baysX+1)*(baysY+1)*storeys
    int expectedColumns = (4 + 1) * (5 + 1) * 10;  // 5*6*10 = 300
    assert(expectedColumns == 300);
    assert(static_cast<int>(gen.columnElementIds.size()) == expectedColumns);

    // Hand-calc: beams per floor = X-dir (baysX*(baysY+1)) + Y-dir ((baysX+1)*baysY)
    //   = 4*6 + 5*5 = 24 + 25 = 49; over `storeys` floors above base = 49*10 = 490
    int beamsPerFloor = 4 * 6 + 5 * 5;
    assert(beamsPerFloor == 49);
    int expectedBeams = beamsPerFloor * 10;  // 490
    assert(static_cast<int>(gen.beamElementIds.size()) == expectedBeams);

    // Hand-calc: slabs per floor = baysX*baysY = 20; over 10 floors = 200
    int expectedSlabs = 4 * 5 * 10;  // 200
    assert(static_cast<int>(gen.slabElementIds.size()) == expectedSlabs);

    // Every generated element id must be unique across all categories combined.
    std::set<int> allIds;
    for (int id : gen.columnElementIds) assert(allIds.insert(id).second);
    for (int id : gen.beamElementIds) assert(allIds.insert(id).second);
    for (int id : gen.slabElementIds) assert(allIds.insert(id).second);

    std::cout << "  testRoadmapWorkedExampleHandCalc (nodes=" << actualNodeCount
              << ", columns=" << gen.columnElementIds.size()
              << ", beams=" << gen.beamElementIds.size()
              << ", slabs=" << gen.slabElementIds.size() << ") OK\n";
}

// Base-level nodes must be fully restrained (fixed base assumption,
// documented in the class doc comment); every node above the base must
// be free.
static void testBaseIsFixedUpperFloorsAreFree() {
    Model model("restraint test");
    model.addMaterial(Material::concrete(1, 28.0));
    BuildingGenerationParameters params;
    params.storeys = 2;
    params.baysX = 1;
    params.baysY = 1;
    params.materialId = 1;
    auto gen = runAutoModelGeneration(model, params);

    for (const auto& row : gen.nodeIds[0]) {
        for (int id : row) assert(model.node(id).isRestrained(DOF::Ux));
    }
    for (std::size_t k = 1; k < gen.nodeIds.size(); ++k) {
        for (const auto& row : gen.nodeIds[k]) {
            for (int id : row) assert(!model.node(id).isRestrained(DOF::Ux));
        }
    }
    std::cout << "  testBaseIsFixedUpperFloorsAreFree OK\n";
}

// Geometry sanity: grid spacing must exactly match the requested bay
// widths and floor height (simple 1-bay-square check, hand-verified
// coordinates).
static void testGridCoordinatesHandCalc() {
    Model model("coordinates test");
    model.addMaterial(Material::concrete(1, 28.0));
    BuildingGenerationParameters params;
    params.storeys = 1;
    params.baysX = 1;
    params.baysY = 1;
    params.bayWidthXm = 5.0;
    params.bayWidthYm = 7.0;
    params.floorHeightM = 3.5;
    params.materialId = 1;
    auto gen = runAutoModelGeneration(model, params);

    int n00_0 = gen.nodeIds[0][0][0];
    int n11_1 = gen.nodeIds[1][1][1];
    assert(model.node(n00_0).x() == 0.0 && model.node(n00_0).y() == 0.0 && model.node(n00_0).z() == 0.0);
    assert(model.node(n11_1).x() == 5.0 && model.node(n11_1).y() == 7.0 && model.node(n11_1).z() == 3.5);
    std::cout << "  testGridCoordinatesHandCalc OK\n";
}

static void testSlabsCanBeDisabled() {
    Model model("no slabs test");
    model.addMaterial(Material::concrete(1, 28.0));
    BuildingGenerationParameters params;
    params.storeys = 1;
    params.baysX = 1;
    params.baysY = 1;
    params.materialId = 1;
    params.generateSlabs = false;
    auto gen = runAutoModelGeneration(model, params);
    assert(gen.slabElementIds.empty());
    std::cout << "  testSlabsCanBeDisabled OK\n";
}

static void testRejectsInvalidInputs() {
    Model model("invalid test");
    model.addMaterial(Material::concrete(1, 28.0));
    BuildingGenerationParameters params;
    params.materialId = 1;

    params.storeys = 0;
    bool threw = false;
    try { runAutoModelGeneration(model, params); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);

    params.storeys = 1;
    params.materialId = 999;  // doesn't exist
    threw = false;
    try { runAutoModelGeneration(model, params); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);

    std::cout << "  testRejectsInvalidInputs OK\n";
}

// Hand-calc: perimeter walls on a 2x2-bay, 1-storey building. Boundary
// edges: 2 X-direction edges (south, north) x 2 bays each = 4 panels;
// 2 Y-direction edges (west, east) x 2 bays each = 4 panels. Total = 8
// wall panels for 1 storey (no walls generated at interior bay lines).
static void testPerimeterWallCountHandCalc() {
    Model model("wall count test");
    model.addMaterial(Material::concrete(1, 28.0));
    BuildingGenerationParameters params;
    params.storeys = 1;
    params.baysX = 2;
    params.baysY = 2;
    params.materialId = 1;
    params.generateExteriorWalls = true;
    auto gen = runAutoModelGeneration(model, params);

    int expectedWallPanels = 2 * params.baysX + 2 * params.baysY;  // 4+4=8
    assert(expectedWallPanels == 8);
    assert(static_cast<int>(gen.wallElementIds.size()) == expectedWallPanels);

    // Every wall element must actually be ElementKind::Wall with 4 nodes.
    for (int id : gen.wallElementIds) {
        assert(model.element(id).kind() == ElementKind::Wall);
        assert(model.element(id).nodeCount() == 4);
    }
    std::cout << "  testPerimeterWallCountHandCalc OK\n";
}

// Hand-calc: total self-weight of a simple 1-bay, 1-storey building
// (no walls, no slab disabled) computed independently from first
// principles (4 columns + 4 beams, since a 1x1-bay grid has 2x2=4 grid
// lines each carrying one column, and 2 X-beams + 2 Y-beams at the one
// floor level, plus 1 slab panel) and compared against
// GeneratedModel::totalSelfWeightKN.
static void testSelfWeightHandCalc() {
    Model model("self weight test");
    Material conc = Material::concrete(1, 28.0);
    model.addMaterial(conc);
    double unitWeight = conc.unitWeightKNm3();

    BuildingGenerationParameters params;
    params.storeys = 1;
    params.baysX = 1;
    params.baysY = 1;
    params.bayWidthXm = 5.0;
    params.bayWidthYm = 4.0;
    params.floorHeightM = 3.0;
    params.columnWidthM = 0.4;
    params.columnDepthM = 0.4;
    params.beamWidthM = 0.3;
    params.beamDepthM = 0.5;
    params.slabThicknessM = 0.15;
    params.materialId = 1;
    params.generateSlabs = true;

    LoadCase lc(1, "Self Weight", LoadCaseType::Dead);
    model.addLoadCase(lc);
    params.generateSelfWeightLoad = true;
    params.selfWeightLoadCaseId = 1;

    auto gen = runAutoModelGeneration(model, params);

    // Hand-calc, independent of the generator's own internals:
    double columnWeight = unitWeight * (0.4 * 0.4) * 3.0 * 4;   // 4 columns (2x2 grid lines), each 3m tall
    double beamXWeight = unitWeight * (0.3 * 0.5) * 5.0 * 2;    // 2 X-beams (rows j=0,1), each 5m long
    double beamYWeight = unitWeight * (0.3 * 0.5) * 4.0 * 2;    // 2 Y-beams (cols i=0,1), each 4m long
    double slabWeight = unitWeight * 0.15 * 5.0 * 4.0 * 1;      // 1 slab panel
    double expectedTotal = columnWeight + beamXWeight + beamYWeight + slabWeight;

    assert(approxEqual(gen.totalSelfWeightKN, expectedTotal, 1e-6));

    // Every NodalLoad's magnitude must sum (in Fz) to exactly -totalSelfWeightKN.
    double sumFz = 0.0;
    for (const auto& l : model.nodalLoads()) {
        assert(l.loadCaseId == 1);
        sumFz += l.Fz;
    }
    assert(approxEqual(sumFz, -expectedTotal, 1e-6));

    std::cout << "  testSelfWeightHandCalc (expected=" << expectedTotal
              << " kN, got=" << gen.totalSelfWeightKN << " kN) OK\n";
}

static void testSelfWeightRequiresLoadCaseId() {
    Model model("missing load case id test");
    model.addMaterial(Material::concrete(1, 28.0));
    BuildingGenerationParameters params;
    params.materialId = 1;
    params.generateSelfWeightLoad = true;  // selfWeightLoadCaseId left at default -1
    bool threw = false;
    try { runAutoModelGeneration(model, params); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  testSelfWeightRequiresLoadCaseId OK\n";
}

int main() {
    std::cout << "test_auto_model_generator:\n";
    testRoadmapWorkedExampleHandCalc();
    testBaseIsFixedUpperFloorsAreFree();
    testGridCoordinatesHandCalc();
    testSlabsCanBeDisabled();
    testRejectsInvalidInputs();
    testPerimeterWallCountHandCalc();
    testSelfWeightHandCalc();
    testSelfWeightRequiresLoadCaseId();
    std::cout << "All tests passed.\n";
    return 0;
}
