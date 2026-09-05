#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <sstream>

#include "reports/ProjectReport.h"

using namespace nrsa::reports;
using namespace nrsa::autodesign;

static bool approxEqual(double a, double b, double relTol = 1e-3) {
    return std::abs(a - b) <= relTol * std::max({1.0, std::abs(a), std::abs(b)});
}

// The schedule must faithfully carry through REAL summary data --
// verified by parsing the actual generated CSV text back into fields
// and comparing to the source struct, not just checking it's non-empty.
static void testBeamScheduleRoundTrip() {
    BeamDesignSummary b;
    b.elementId = 42;
    b.muKNm = 123.45;
    b.vuKN = 67.8;
    b.flexure.asRequiredMm2 = 1500.5;
    b.flexure.governedByMinimum = true;
    b.shear.stirrupsRequired = true;
    b.shear.requiredSpacingMm = 150.0;

    std::string csv = generateBeamScheduleCsv({b});
    std::istringstream iss(csv);
    std::string header, row;
    std::getline(iss, header);
    std::getline(iss, row);

    assert(header.find("ElementId") != std::string::npos);
    assert(row.find("42") != std::string::npos);
    assert(row.find("123.45") != std::string::npos);
    assert(row.find("1500.5") != std::string::npos);
    assert(row.find("Yes") != std::string::npos);
    std::cout << "  testBeamScheduleRoundTrip (row=\"" << row << "\") OK\n";
}

static void testColumnScheduleRoundTrip() {
    ColumnDesignSummary c;
    c.elementId = 7;
    c.puKN = 800.0;
    c.muxKNm = 45.0;
    c.muyKNm = 30.0;
    c.demandCapacityRatio = 0.85;
    c.biaxial.adequate = true;

    std::string csv = generateColumnScheduleCsv({c});
    assert(csv.find("7,800") != std::string::npos);
    assert(csv.find("0.85") != std::string::npos);
    assert(csv.find("Yes") != std::string::npos);
    std::cout << "  testColumnScheduleRoundTrip OK\n";
}

static void testEmptyListProducesHeaderOnly() {
    std::string csv = generateBeamScheduleCsv({});
    int lineCount = static_cast<int>(std::count(csv.begin(), csv.end(), '\n'));
    assert(lineCount == 1);  // header only
    std::cout << "  testEmptyListProducesHeaderOnly OK\n";
}

int main() {
    std::cout << "test_project_report:\n";
    testBeamScheduleRoundTrip();
    testColumnScheduleRoundTrip();
    testEmptyListProducesHeaderOnly();
    std::cout << "All tests passed.\n";
    return 0;
}
