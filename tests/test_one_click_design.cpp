#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "core/Material.h"
#include "oneclick/OneClickDesign.h"

using namespace nrsa;
using namespace nrsa::oneclick;
using namespace nrsa::modeler;

// Full pipeline, one call: a small 1-storey, 2x2-bay building with
// self-weight dead load, run through OneClickDesign end to end. This
// is a STRUCTURAL/INTEGRATION check (every generated member gets a
// design result, wired through with no crashes/mismatched ids) rather
// than a fresh hand-calc -- the individual pieces (AutoModelGenerator,
// AutoDesign for beams/columns) already have their OWN hand-calc
// verification elsewhere; this test's job is to prove the CHAIN
// itself works end-to-end, matching how the roadmap's own "One Click"
// vision is about the chaining, not new math.
static void testOneClickPipelineIntegration() {
    Model model("one click test");
    Material conc = Material::concrete(1, 28.0);
    model.addMaterial(conc);

    BuildingGenerationParameters params;
    params.storeys = 1;
    params.baysX = 2;
    params.baysY = 2;
    params.bayWidthXm = 5.0;
    params.bayWidthYm = 5.0;
    params.floorHeightM = 3.2;
    params.columnWidthM = 0.4;
    params.columnDepthM = 0.4;
    params.beamWidthM = 0.3;
    params.beamDepthM = 0.5;
    params.materialId = 1;
    params.generateSlabs = true;
    params.generateSelfWeightLoad = true;
    params.selfWeightLoadCaseId = 1;

    design::BnbcLoadCaseIds ids;
    ids.dead = 1;

    auto result = runOneClickDesign(model, params, ids, 28.0, 420.0, 40.0, 20.0, 10.0, 2, 20.0, 3, 3,
                                     150.0, 50.0, 16.0);

    // Structural check: every generated beam/column got a design
    // result, matched by element id, none missing.
    assert(result.beamDesigns.size() == result.geometry.beamElementIds.size());
    assert(result.columnDesigns.size() == result.geometry.columnElementIds.size());
    assert(!result.beamDesigns.empty());
    assert(!result.columnDesigns.empty());

    // Every ground-floor column (base node fully restrained, per
    // AutoModelGenerator's own fixed-base convention) must have gotten
    // a foundation design -- exactly the columns whose base sits at the
    // building's base level.
    assert(!result.foundationDesigns.empty());
    assert(result.foundationDesigns.size() == result.geometry.nodeIds[0].size() * result.geometry.nodeIds[0][0].size());

    // Self-weight was applied -> at least SOME beam must see nonzero
    // demand (not every summary silently zero, which would indicate
    // the load never actually reached the analysis).
    bool anyBeamHasDemand = false;
    for (const auto& b : result.beamDesigns) if (b.muKNm > 0.0 || b.vuKN > 0.0) anyBeamHasDemand = true;
    assert(anyBeamHasDemand);

    bool anyColumnHasDemand = false;
    for (const auto& c : result.columnDesigns) if (c.puKN > 0.0) anyColumnHasDemand = true;
    assert(anyColumnHasDemand);

    std::cout << "  testOneClickPipelineIntegration (" << result.beamDesigns.size() << " beams, "
              << result.columnDesigns.size() << " columns, " << result.foundationDesigns.size()
              << " foundations, all designed) OK\n";
}

int main() {
    std::cout << "test_one_click_design:\n";
    testOneClickPipelineIntegration();
    std::cout << "All tests passed.\n";
    return 0;
}
