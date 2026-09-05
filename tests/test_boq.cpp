#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "bbs/BarSchedule.h"
#include "boq/QuantityTakeoff.h"

using namespace nrsa;
using namespace nrsa::boq;

static bool approxEqual(double a, double b, double relTol = 1e-3) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// ---- geometry helpers ---------------------------------------------------

static void testRectangularVolumeMatchesHandCalc() {
    // 0.3 x 0.5 x 6.0m beam -> 0.9 m^3
    double v = rectangularVolumeM3(0.3, 0.5, 6.0);
    assert(approxEqual(v, 0.9));
    std::cout << "  testRectangularVolumeMatchesHandCalc OK\n";
}

static void testRectangularVolumeRejectsNonPositive() {
    bool threw = false;
    try {
        rectangularVolumeM3(0.0, 0.5, 6.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testRectangularVolumeRejectsNonPositive OK\n";
}

static void testBeamFormworkMatchesHandCalc() {
    // b=0.3, h=0.5, L=6.0 -> (0.3 + 2*0.5)*6.0 = 1.3*6.0 = 7.8 m^2
    double a = rectangularBeamFormworkAreaM2(0.3, 0.5, 6.0);
    assert(approxEqual(a, 7.8));
    std::cout << "  testBeamFormworkMatchesHandCalc OK\n";
}

static void testColumnFormworkMatchesHandCalc() {
    // b=0.4, h=0.4, height=3.0 -> 2*(0.4+0.4)*3.0 = 4.8 m^2
    double a = rectangularColumnFormworkAreaM2(0.4, 0.4, 3.0);
    assert(approxEqual(a, 4.8));
    std::cout << "  testColumnFormworkMatchesHandCalc OK\n";
}

static void testSlabFormworkIsSoffitOnly() {
    // 5m x 6m panel -> 30 m^2 (soffit only, no edge formwork -- see header note)
    double a = slabFormworkAreaM2(5.0, 6.0);
    assert(approxEqual(a, 30.0));
    std::cout << "  testSlabFormworkIsSoffitOnly OK\n";
}

static void testWallFormworkIsBothFaces() {
    // 4m long x 3m high -> 2*4*3 = 24 m^2
    double a = wallFormworkAreaM2(4.0, 3.0);
    assert(approxEqual(a, 24.0));
    std::cout << "  testWallFormworkIsBothFaces OK\n";
}

static void testFootingFormworkIsPerimeterTimesThickness() {
    // 2m x 2m plan, 0.4m thick -> 2*(2+2)*0.4 = 3.2 m^2
    double a = footingFormworkAreaM2(2.0, 2.0, 0.4);
    assert(approxEqual(a, 3.2));
    std::cout << "  testFootingFormworkIsPerimeterTimesThickness OK\n";
}

// ---- builders -------------------------------------------------------------

static void testMakeBeamConcreteQuantityMatchesGeometryFunctions() {
    auto q = makeBeamConcreteQuantity("Beam B1", 0.3, 0.5, 6.0, 28.0);
    assert(approxEqual(q.volumeM3, rectangularVolumeM3(0.3, 0.5, 6.0)));
    assert(approxEqual(q.formworkAreaM2, rectangularBeamFormworkAreaM2(0.3, 0.5, 6.0)));
    assert(approxEqual(q.fcMPa, 28.0));
    assert(q.memberLabel == "Beam B1");
    std::cout << "  testMakeBeamConcreteQuantityMatchesGeometryFunctions OK\n";
}

static void testMakeSlabConcreteQuantityConvertsThicknessFromMm() {
    // 5m x 6m panel, 150mm thick -> volume = 5*6*0.15 = 4.5 m^3
    auto q = makeSlabConcreteQuantity("Slab S1", 5.0, 6.0, 150.0, 28.0);
    assert(approxEqual(q.volumeM3, 4.5));
    assert(approxEqual(q.formworkAreaM2, 30.0));
    std::cout << "  testMakeSlabConcreteQuantityConvertsThicknessFromMm OK\n";
}

static void testMakeWallConcreteQuantityConvertsThicknessFromMm() {
    // 4m x 3m wall, 250mm thick -> volume = 4*3*0.25 = 3.0 m^3
    auto q = makeWallConcreteQuantity("Wall W1", 4.0, 3.0, 250.0, 28.0);
    assert(approxEqual(q.volumeM3, 3.0));
    assert(approxEqual(q.formworkAreaM2, 24.0));
    std::cout << "  testMakeWallConcreteQuantityConvertsThicknessFromMm OK\n";
}

static void testMakeFootingConcreteQuantityConvertsThicknessFromMm() {
    // 2m x 2m plan, 400mm thick -> volume = 2*2*0.4 = 1.6 m^3
    auto q = makeFootingConcreteQuantity("Footing F1", 2.0, 2.0, 400.0, 28.0);
    assert(approxEqual(q.volumeM3, 1.6));
    assert(approxEqual(q.formworkAreaM2, 3.2));
    std::cout << "  testMakeFootingConcreteQuantityConvertsThicknessFromMm OK\n";
}

static void testMakeSlabConcreteQuantityRejectsNonPositiveThickness() {
    bool threw = false;
    try {
        makeSlabConcreteQuantity("Slab S1", 5.0, 6.0, 0.0, 28.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testMakeSlabConcreteQuantityRejectsNonPositiveThickness OK\n";
}

// ---- QuantityTakeoff rollups -----------------------------------------------

static void testTakeoffTotalsSumAllConcreteItems() {
    QuantityTakeoff takeoff;
    auto beam = makeBeamConcreteQuantity("Beam B1", 0.3, 0.5, 6.0, 28.0);
    auto col = makeColumnConcreteQuantity("Column C1", 0.4, 0.4, 3.0, 28.0);
    double expectedVolume = beam.volumeM3 + col.volumeM3;
    double expectedFormwork = beam.formworkAreaM2 + col.formworkAreaM2;
    takeoff.addConcrete(beam);
    takeoff.addConcrete(col);
    assert(approxEqual(takeoff.totalConcreteVolumeM3(), expectedVolume));
    assert(approxEqual(takeoff.totalFormworkAreaM2(), expectedFormwork));
    assert(takeoff.concreteItems().size() == 2);
    std::cout << "  testTakeoffTotalsSumAllConcreteItems OK\n";
}

static void testTakeoffGroupsVolumeByGrade() {
    QuantityTakeoff takeoff;
    auto beam28 = makeBeamConcreteQuantity("Beam B1", 0.3, 0.5, 6.0, 28.0);
    auto col28 = makeColumnConcreteQuantity("Column C1", 0.4, 0.4, 3.0, 28.0);
    auto footing35 = makeFootingConcreteQuantity("Footing F1", 2.0, 2.0, 400.0, 35.0);
    takeoff.addConcrete(beam28);
    takeoff.addConcrete(col28);
    takeoff.addConcrete(footing35);

    auto byGrade = takeoff.concreteVolumeByGradeM3();
    assert(byGrade.size() == 2);
    assert(approxEqual(byGrade.at(28.0), beam28.volumeM3 + col28.volumeM3));
    assert(approxEqual(byGrade.at(35.0), footing35.volumeM3));
    std::cout << "  testTakeoffGroupsVolumeByGrade OK\n";
}

static void testTakeoffPullsRebarWeightFromBbsSchedule() {
    bbs::BarBendingSchedule schedule;
    bbs::BarMarkEntry e1;
    e1.barDiaMm = 16.0;
    e1.count = 5;
    e1.cutLengthMm = 5128.0;
    bbs::BarMarkEntry e2;
    e2.barDiaMm = 8.0;
    e2.count = 21;
    e2.cutLengthMm = 1438.0;
    schedule.addEntry(e1);
    schedule.addEntry(e2);

    QuantityTakeoff takeoff;
    takeoff.setRebarSchedule(schedule);

    assert(approxEqual(takeoff.totalRebarWeightKg(), schedule.totalWeightKg()));
    auto byDia = takeoff.rebarWeightByDiameterKg();
    auto expectedByDia = schedule.weightByDiameterKg();
    assert(byDia.size() == expectedByDia.size());
    for (const auto& [dia, wt] : expectedByDia) {
        assert(approxEqual(byDia.at(dia), wt));
    }
    std::cout << "  testTakeoffPullsRebarWeightFromBbsSchedule OK\n";
}

static void testTakeoffRebarPerVolumeMatchesHandCalc() {
    QuantityTakeoff takeoff;
    takeoff.addConcrete(makeColumnConcreteQuantity("Column C1", 0.4, 0.4, 3.0, 28.0));  // 0.48 m^3

    bbs::BarBendingSchedule schedule;
    bbs::BarMarkEntry e;
    e.barDiaMm = 20.0;
    e.count = 8;
    e.cutLengthMm = 3000.0;
    schedule.addEntry(e);
    takeoff.setRebarSchedule(schedule);

    double expected = schedule.totalWeightKg() / 0.48;
    assert(approxEqual(takeoff.rebarWeightPerConcreteVolumeKgPerM3(), expected));
    std::cout << "  testTakeoffRebarPerVolumeMatchesHandCalc OK\n";
}

static void testTakeoffRebarPerVolumeZeroWithNoConcrete() {
    QuantityTakeoff takeoff;
    bbs::BarBendingSchedule schedule;
    bbs::BarMarkEntry e;
    e.barDiaMm = 16.0;
    e.count = 4;
    e.cutLengthMm = 3000.0;
    schedule.addEntry(e);
    takeoff.setRebarSchedule(schedule);
    assert(approxEqual(takeoff.rebarWeightPerConcreteVolumeKgPerM3(), 0.0));
    std::cout << "  testTakeoffRebarPerVolumeZeroWithNoConcrete OK\n";
}

static void testTakeoffSetRebarScheduleReplacesNotAdds() {
    QuantityTakeoff takeoff;
    bbs::BarBendingSchedule schedule1;
    bbs::BarMarkEntry e1;
    e1.barDiaMm = 16.0;
    e1.count = 4;
    e1.cutLengthMm = 3000.0;
    schedule1.addEntry(e1);
    takeoff.setRebarSchedule(schedule1);
    double firstTotal = takeoff.totalRebarWeightKg();

    bbs::BarBendingSchedule schedule2;
    bbs::BarMarkEntry e2;
    e2.barDiaMm = 12.0;
    e2.count = 2;
    e2.cutLengthMm = 2000.0;
    schedule2.addEntry(e2);
    takeoff.setRebarSchedule(schedule2);

    assert(!approxEqual(takeoff.totalRebarWeightKg(), firstTotal));
    assert(approxEqual(takeoff.totalRebarWeightKg(), schedule2.totalWeightKg()));
    std::cout << "  testTakeoffSetRebarScheduleReplacesNotAdds OK\n";
}

int main() {
    std::cout << "Running boq (Quantity Takeoff) tests...\n";
    testRectangularVolumeMatchesHandCalc();
    testRectangularVolumeRejectsNonPositive();
    testBeamFormworkMatchesHandCalc();
    testColumnFormworkMatchesHandCalc();
    testSlabFormworkIsSoffitOnly();
    testWallFormworkIsBothFaces();
    testFootingFormworkIsPerimeterTimesThickness();
    testMakeBeamConcreteQuantityMatchesGeometryFunctions();
    testMakeSlabConcreteQuantityConvertsThicknessFromMm();
    testMakeWallConcreteQuantityConvertsThicknessFromMm();
    testMakeFootingConcreteQuantityConvertsThicknessFromMm();
    testMakeSlabConcreteQuantityRejectsNonPositiveThickness();
    testTakeoffTotalsSumAllConcreteItems();
    testTakeoffGroupsVolumeByGrade();
    testTakeoffPullsRebarWeightFromBbsSchedule();
    testTakeoffRebarPerVolumeMatchesHandCalc();
    testTakeoffRebarPerVolumeZeroWithNoConcrete();
    testTakeoffSetRebarScheduleReplacesNotAdds();
    std::cout << "All boq tests passed.\n";
    return 0;
}
