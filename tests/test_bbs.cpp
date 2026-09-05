#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "bbs/BarSchedule.h"
#include "design/RCCBeam.h"
#include "design/RCCColumn.h"

using namespace nrsa;
using namespace nrsa::bbs;

static bool approxEqual(double a, double b, double relTol = 1e-3) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// ---- barAreaMm2 / unitWeightKgPerM -----------------------------------

static void testBarAreaMatchesClosedForm() {
    double area = barAreaMm2(16.0);
    assert(approxEqual(area, M_PI / 4.0 * 16.0 * 16.0));
    std::cout << "  testBarAreaMatchesClosedForm OK\n";
}

static void testUnitWeightMatchesD2Over162RuleOfThumb() {
    // The d^2/162 shop-drawing rule of thumb and this module's
    // density-derived formula should agree to within ~0.2% for any
    // ordinary bar size -- both are the same physics, just rounded
    // differently.
    for (double d : {10.0, 12.0, 16.0, 20.0, 25.0, 32.0}) {
        double ruleOfThumb = (d * d) / 162.0;
        double exact = unitWeightKgPerM(d);
        assert(approxEqual(exact, ruleOfThumb, 0.003));
    }
    std::cout << "  testUnitWeightMatchesD2Over162RuleOfThumb OK\n";
}

static void testUnitWeightRejectsNonPositiveDiameter() {
    bool threw = false;
    try {
        unitWeightKgPerM(0.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testUnitWeightRejectsNonPositiveDiameter OK\n";
}

// ---- practicalBarCount ------------------------------------------------

static void testPracticalBarCountRoundsUpToWholeBars() {
    // 900mm^2 needed, 16mm bars (area 201.06mm^2 each) -> 4.48 -> 5 bars.
    int n = practicalBarCount(900.0, 16.0);
    assert(n == 5);
    std::cout << "  testPracticalBarCountRoundsUpToWholeBars OK\n";
}

static void testPracticalBarCountRespectsMinimum() {
    // A tiny required area should still floor at minBars, not round to 1.
    int n = practicalBarCount(10.0, 25.0, 2);
    assert(n == 2);
    std::cout << "  testPracticalBarCountRespectsMinimum OK\n";
}

static void testPracticalBarCountExactMultipleDoesNotOverRound() {
    // Exactly 4 bars' worth of area should give 4, not 5 (guards against
    // floating point pushing an exact case over the ceiling boundary).
    double exactly4 = 4.0 * barAreaMm2(20.0);
    int n = practicalBarCount(exactly4, 20.0);
    assert(n == 4);
    std::cout << "  testPracticalBarCountExactMultipleDoesNotOverRound OK\n";
}

static void testPracticalBarCountRejectsNegativeArea() {
    bool threw = false;
    try {
        practicalBarCount(-1.0, 16.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testPracticalBarCountRejectsNegativeArea OK\n";
}

// ---- hook extension / bend deduction -----------------------------------

static void testHookExtensionMatchesAci25_3_2ClosedForm() {
    // 16mm bar: 6*16=96 > 75 -> governs for 90/135; 4*16=64 < 65 -> 65mm floor governs for 180.
    assert(approxEqual(standardHookExtensionMm(16.0, HookAngle::Bend90), 96.0));
    assert(approxEqual(standardHookExtensionMm(16.0, HookAngle::Bend135), 96.0));
    assert(approxEqual(standardHookExtensionMm(16.0, HookAngle::Bend180), 65.0));
    assert(approxEqual(standardHookExtensionMm(16.0, HookAngle::None), 0.0));
    // 10mm bar: 6*10=60 < 75 -> the 75mm floor governs instead.
    assert(approxEqual(standardHookExtensionMm(10.0, HookAngle::Bend90), 75.0));
    std::cout << "  testHookExtensionMatchesAci25_3_2ClosedForm OK\n";
}

static void testBendDeductionScalesWithAngle() {
    assert(approxEqual(bendDeductionMm(16.0, HookAngle::Bend90), 32.0));
    assert(approxEqual(bendDeductionMm(16.0, HookAngle::Bend135), 48.0));
    assert(approxEqual(bendDeductionMm(16.0, HookAngle::Bend180), 64.0));
    assert(approxEqual(bendDeductionMm(16.0, HookAngle::None), 0.0));
    std::cout << "  testBendDeductionScalesWithAngle OK\n";
}

// ---- straightBarCutLengthMm --------------------------------------------

static void testStraightBarNoHooksEqualsClearLength() {
    double cl = straightBarCutLengthMm(5000.0, 16.0, HookAngle::None, HookAngle::None);
    assert(approxEqual(cl, 5000.0));
    std::cout << "  testStraightBarNoHooksEqualsClearLength OK\n";
}

static void testStraightBarWithHooksMatchesHandCalc() {
    // clear=5000, both ends 90deg hooks on a 16mm bar:
    // extension=96 each, deduction=32 each -> net +64 per end -> +128 total.
    double cl = straightBarCutLengthMm(5000.0, 16.0, HookAngle::Bend90, HookAngle::Bend90);
    assert(approxEqual(cl, 5128.0));
    std::cout << "  testStraightBarWithHooksMatchesHandCalc OK\n";
}

static void testStraightBarRejectsNonPositiveInputs() {
    bool threw = false;
    try {
        straightBarCutLengthMm(0.0, 16.0, HookAngle::None, HookAngle::None);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testStraightBarRejectsNonPositiveInputs OK\n";
}

// ---- stirrupCutLengthMm -------------------------------------------------

static void testStirrupCutLengthMatchesHandCalc() {
    // outerWidth=250, outerDepth=450, dia=8mm:
    // perimeter = 2*(250+450) = 1400
    // hook ext (135deg, 2 hooks): 6*8=48 < 75 -> 75 each -> +150
    // deductions: 4 corners @ 90deg (2*8=16 each) = 64; 2 hooks @ 135deg (3*8=24 each) = 48
    // total deduction = 112
    // cutLength = 1400 + 150 - 112 = 1438
    double cl = stirrupCutLengthMm(250.0, 450.0, 8.0);
    assert(approxEqual(cl, 1438.0));
    std::cout << "  testStirrupCutLengthMatchesHandCalc OK\n";
}

static void testStirrupCutLengthRejectsNonPositiveInputs() {
    bool threw = false;
    try {
        stirrupCutLengthMm(-1.0, 450.0, 8.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testStirrupCutLengthRejectsNonPositiveInputs OK\n";
}

// ---- simplifiedTensionLapSpliceLengthMm ---------------------------------

static void testLapSpliceMatchesHandCalc() {
    // fy=414, fc'=28, db=16: (414/(1.7*sqrt(28)))*16
    double expected = (414.0 / (1.7 * std::sqrt(28.0))) * 16.0;
    double got = simplifiedTensionLapSpliceLengthMm(16.0, 414.0, 28.0);
    assert(approxEqual(got, expected));
    assert(got > 300.0);  // sanity: this case shouldn't hit the 300mm floor
    std::cout << "  testLapSpliceMatchesHandCalc OK\n";
}

static void testLapSpliceFloorsAtMinimum() {
    // A tiny bar diameter should hit the 300mm floor rather than a smaller value.
    double got = simplifiedTensionLapSpliceLengthMm(6.0, 300.0, 40.0);
    assert(approxEqual(got, 300.0));
    std::cout << "  testLapSpliceFloorsAtMinimum OK\n";
}

// ---- BarMarkEntry rollups -------------------------------------------------

static void testBarMarkEntryTotalsMatchHandCalc() {
    BarMarkEntry e;
    e.barDiaMm = 16.0;
    e.count = 5;
    e.cutLengthMm = 5128.0;
    assert(approxEqual(e.totalLengthM(), 5.0 * 5.128));
    double expectedWeight = e.totalLengthM() * unitWeightKgPerM(16.0);
    assert(approxEqual(e.totalWeightKg(), expectedWeight));
    std::cout << "  testBarMarkEntryTotalsMatchHandCalc OK\n";
}

// ---- makeFlexuralBarEntry / makeStirrupEntry -----------------------------

static void testMakeFlexuralBarEntryRoundsAreaAndSetsCutLength() {
    design::FlexuralDesignResult flex;
    flex.asRequiredMm2 = 900.0;
    BarMarkEntry e = makeFlexuralBarEntry("B1-T1", "Beam B1", flex, 16.0, 5000.0,
                                           HookAngle::Bend90, HookAngle::Bend90);
    assert(e.count == 5);
    assert(approxEqual(e.cutLengthMm, 5128.0));
    assert(e.shape == BarShape::Straight);
    assert(e.markId == "B1-T1");
    std::cout << "  testMakeFlexuralBarEntryRoundsAreaAndSetsCutLength OK\n";
}

static void testMakeStirrupEntryComputesSpacingBasedCount() {
    design::ShearDesignResult shear;
    shear.stirrupsRequired = true;
    shear.requiredSpacingMm = 150.0;
    BarMarkEntry e = makeStirrupEntry("B1-S1", "Beam B1", shear, 3000.0, 250.0, 450.0, 8.0, 2);
    // ceil(3000/150) + 1 = 20 + 1 = 21
    assert(e.count == 21);
    assert(e.shape == BarShape::RectStirrup);
    assert(approxEqual(e.cutLengthMm, 1438.0));
    std::cout << "  testMakeStirrupEntryComputesSpacingBasedCount OK\n";
}

static void testMakeStirrupEntryZeroWhenNotRequired() {
    design::ShearDesignResult shear;
    shear.stirrupsRequired = false;
    BarMarkEntry e = makeStirrupEntry("B1-S1", "Beam B1", shear, 3000.0, 250.0, 450.0, 8.0, 2);
    assert(e.count == 0);
    std::cout << "  testMakeStirrupEntryZeroWhenNotRequired OK\n";
}

static void testMakeStirrupEntryRejectsMoreThanTwoLegs() {
    design::ShearDesignResult shear;
    shear.stirrupsRequired = true;
    shear.requiredSpacingMm = 150.0;
    bool threw = false;
    try {
        makeStirrupEntry("B1-S1", "Beam B1", shear, 3000.0, 250.0, 450.0, 8.0, 4);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testMakeStirrupEntryRejectsMoreThanTwoLegs OK\n";
}

// ---- makeColumnLongitudinalEntries ---------------------------------------

static void testMakeColumnLongitudinalEntriesMatchesLayoutBarCount() {
    auto layout = design::generateRectangularLayout(0.4, 0.4, 40.0, 20.0, 3, 3);
    auto entries = makeColumnLongitudinalEntries("C4-L", "Column C4, Story 2", layout, 3000.0,
                                                  /*includeLapSplice=*/false, 414.0, 28.0);
    assert(entries.size() == 1);
    assert(entries[0].count == static_cast<int>(layout.barsXY.size()));
    assert(approxEqual(entries[0].barDiaMm, 20.0));
    assert(approxEqual(entries[0].cutLengthMm, 3000.0));
    std::cout << "  testMakeColumnLongitudinalEntriesMatchesLayoutBarCount OK\n";
}

static void testMakeColumnLongitudinalEntriesAddsLapSplice() {
    auto layout = design::generateRectangularLayout(0.4, 0.4, 40.0, 20.0, 3, 3);
    auto entries = makeColumnLongitudinalEntries("C4-L", "Column C4, Story 2", layout, 3000.0,
                                                  /*includeLapSplice=*/true, 414.0, 28.0);
    double expectedLap = simplifiedTensionLapSpliceLengthMm(20.0, 414.0, 28.0);
    assert(approxEqual(entries[0].cutLengthMm, 3000.0 + expectedLap));
    std::cout << "  testMakeColumnLongitudinalEntriesAddsLapSplice OK\n";
}

static void testMakeColumnLongitudinalEntriesRejectsEmptyLayout() {
    design::RectColumnLayout empty;
    bool threw = false;
    try {
        makeColumnLongitudinalEntries("C4-L", "Column C4", empty, 3000.0, false, 414.0, 28.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
    std::cout << "  testMakeColumnLongitudinalEntriesRejectsEmptyLayout OK\n";
}

// ---- BarBendingSchedule rollups -------------------------------------------

static void testScheduleTotalsSumAllEntries() {
    BarBendingSchedule schedule;
    BarMarkEntry e1;
    e1.barDiaMm = 16.0;
    e1.count = 5;
    e1.cutLengthMm = 5128.0;
    BarMarkEntry e2;
    e2.barDiaMm = 8.0;
    e2.count = 21;
    e2.cutLengthMm = 1438.0;
    schedule.addEntry(e1);
    schedule.addEntry(e2);

    double expectedTotal = e1.totalWeightKg() + e2.totalWeightKg();
    assert(approxEqual(schedule.totalWeightKg(), expectedTotal));
    assert(schedule.entries().size() == 2);
    std::cout << "  testScheduleTotalsSumAllEntries OK\n";
}

static void testScheduleGroupsWeightByDiameter() {
    BarBendingSchedule schedule;
    BarMarkEntry e1;
    e1.barDiaMm = 16.0;
    e1.count = 3;
    e1.cutLengthMm = 4000.0;
    BarMarkEntry e2;
    e2.barDiaMm = 16.0;
    e2.count = 2;
    e2.cutLengthMm = 6000.0;
    BarMarkEntry e3;
    e3.barDiaMm = 8.0;
    e3.count = 10;
    e3.cutLengthMm = 1000.0;
    schedule.addEntry(e1);
    schedule.addEntry(e2);
    schedule.addEntry(e3);

    auto byDia = schedule.weightByDiameterKg();
    assert(byDia.size() == 2);
    double expected16 = e1.totalWeightKg() + e2.totalWeightKg();
    assert(approxEqual(byDia.at(16.0), expected16));
    assert(approxEqual(byDia.at(8.0), e3.totalWeightKg()));
    std::cout << "  testScheduleGroupsWeightByDiameter OK\n";
}

int main() {
    std::cout << "Running bbs (Bar Bending Schedule) tests...\n";
    testBarAreaMatchesClosedForm();
    testUnitWeightMatchesD2Over162RuleOfThumb();
    testUnitWeightRejectsNonPositiveDiameter();
    testPracticalBarCountRoundsUpToWholeBars();
    testPracticalBarCountRespectsMinimum();
    testPracticalBarCountExactMultipleDoesNotOverRound();
    testPracticalBarCountRejectsNegativeArea();
    testHookExtensionMatchesAci25_3_2ClosedForm();
    testBendDeductionScalesWithAngle();
    testStraightBarNoHooksEqualsClearLength();
    testStraightBarWithHooksMatchesHandCalc();
    testStraightBarRejectsNonPositiveInputs();
    testStirrupCutLengthMatchesHandCalc();
    testStirrupCutLengthRejectsNonPositiveInputs();
    testLapSpliceMatchesHandCalc();
    testLapSpliceFloorsAtMinimum();
    testBarMarkEntryTotalsMatchHandCalc();
    testMakeFlexuralBarEntryRoundsAreaAndSetsCutLength();
    testMakeStirrupEntryComputesSpacingBasedCount();
    testMakeStirrupEntryZeroWhenNotRequired();
    testMakeStirrupEntryRejectsMoreThanTwoLegs();
    testMakeColumnLongitudinalEntriesMatchesLayoutBarCount();
    testMakeColumnLongitudinalEntriesAddsLapSplice();
    testMakeColumnLongitudinalEntriesRejectsEmptyLayout();
    testScheduleTotalsSumAllEntries();
    testScheduleGroupsWeightByDiameter();
    std::cout << "All bbs tests passed.\n";
    return 0;
}
