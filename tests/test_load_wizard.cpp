#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include "core/Model.h"
#include "core/Node.h"
#include "loads/LoadWizard.h"

using namespace nrsa;
using namespace nrsa::loads;

static bool approxEqual(double a, double b, double tol = 1e-9) { return std::abs(a - b) <= tol; }

// Hand-calc: 5m x 4m office panel, floor finish 1.0 + partition 1.0 +
// ceiling 0.2 + waterproofing 0 = 2.2 kPa dead; office live load per
// the table = 2.4 kPa. Total dead force = 2.2*20 = 44 kN, split over 4
// nodes = 11 kN each. Total live force = 2.4*20 = 48 kN, split = 12 kN each.
static void testPanelLoadHandCalc() {
    Model model("panel load test");
    for (int i = 1; i <= 4; ++i) model.addNode(Node(i, i, 0, 0));

    DeadLoadComponents dead;
    dead.floorFinishKPa = 1.0;
    dead.partitionKPa = 1.0;
    dead.ceilingKPa = 0.2;

    double area = 5.0 * 4.0;
    auto result = applyRoomLoadToPanel(model, {1, 2, 3, 4}, area, dead, RoomType::Office,
                                        /*deadLoadCaseId=*/1, /*liveLoadCaseId=*/2);

    assert(approxEqual(result.totalDeadForceKN, 2.2 * 20.0));
    assert(approxEqual(result.totalLiveForceKN, 2.4 * 20.0));

    double sumDeadFz = 0.0, sumLiveFz = 0.0;
    for (const auto& l : model.nodalLoads()) {
        if (l.loadCaseId == 1) sumDeadFz += l.Fz;
        if (l.loadCaseId == 2) sumLiveFz += l.Fz;
    }
    assert(approxEqual(sumDeadFz, -2.2 * 20.0));
    assert(approxEqual(sumLiveFz, -2.4 * 20.0));
    std::cout << "  testPanelLoadHandCalc OK\n";
}

// Skipping a load case (pass -1) must apply nothing for that component.
static void testSkipsComponentWhenLoadCaseIsNegativeOne() {
    Model model("skip test");
    for (int i = 1; i <= 4; ++i) model.addNode(Node(i, i, 0, 0));
    DeadLoadComponents dead;
    dead.floorFinishKPa = 1.0;
    auto result = applyRoomLoadToPanel(model, {1, 2, 3, 4}, 10.0, dead, RoomType::Storage,
                                        /*deadLoadCaseId=*/1, /*liveLoadCaseId=*/-1);
    assert(result.totalLiveForceKN == 0.0);
    for (const auto& l : model.nodalLoads()) assert(l.loadCaseId != 2);  // no live load case applied
    std::cout << "  testSkipsComponentWhenLoadCaseIsNegativeOne OK\n";
}

// Hand-calc: wall line load = unit weight * thickness * height.
static void testWallLineLoadHandCalc() {
    double q = wallLineLoadKNPerM(19.0, 0.125, 3.0);  // typical brick unit weight ~19 kN/m3
    assert(approxEqual(q, 19.0 * 0.125 * 3.0));
    std::cout << "  testWallLineLoadHandCalc OK\n";
}

static void testRejectsInvalidInputs() {
    bool threw = false;
    try { wallLineLoadKNPerM(19.0, -0.1, 3.0); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);

    Model model("invalid panel test");
    model.addNode(Node(1, 0, 0, 0));
    DeadLoadComponents dead;
    threw = false;
    try { applyRoomLoadToPanel(model, {1}, -5.0, dead, RoomType::Office, 1, 2); }
    catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  testRejectsInvalidInputs OK\n";
}

// Regression lock: BNBC 2020 Table 6.2.3 was fetched directly from the
// official code text (2026-08-27) and every value below was confirmed
// to match. This pins the cited values so a future edit that silently
// drifts from the verified table gets caught immediately.
static void testLiveLoadsMatchBNBC2020Table623() {
    assert(approxEqual(liveLoadKPaForRoomType(RoomType::Residential), 2.00));
    assert(approxEqual(liveLoadKPaForRoomType(RoomType::Office), 2.40));
    assert(approxEqual(liveLoadKPaForRoomType(RoomType::Hospital), 2.00));
    assert(approxEqual(liveLoadKPaForRoomType(RoomType::School), 2.00));
    assert(approxEqual(liveLoadKPaForRoomType(RoomType::Parking), 2.00));
    assert(approxEqual(liveLoadKPaForRoomType(RoomType::Commercial), 4.80));
    assert(approxEqual(liveLoadKPaForRoomType(RoomType::Storage), 6.00));
    assert(approxEqual(liveLoadKPaForRoomType(RoomType::Roof), 1.00));
    std::cout << "  testLiveLoadsMatchBNBC2020Table623 OK\n";
}

int main() {
    std::cout << "test_load_wizard:\n";
    testPanelLoadHandCalc();
    testSkipsComponentWhenLoadCaseIsNegativeOne();
    testWallLineLoadHandCalc();
    testRejectsInvalidInputs();
    testLiveLoadsMatchBNBC2020Table623();
    std::cout << "All tests passed.\n";
    return 0;
}
