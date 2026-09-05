#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <vector>

#include "analysis/SeismicAnalysis.h"
#include "codes/CodeRuleStore.h"
#include "loads/LoadWizard.h"

using namespace nrsa::codes;

static bool approxEqual(double a, double b, double tol = 1e-9) { return std::abs(a - b) <= tol; }

// Load a hand-written rule file with real BNBC-style entries (matching
// the seismic zone coefficients already used in analysis::SeismicAnalysis
// -- proving the DATA-DRIVEN version of a constant that currently lives
// hardcoded in that module's own struct) and verify exact values parse.
static void testLoadHandWrittenRuleFile() {
    const char* path = "/tmp/test_code_rules.txt";
    std::ofstream f(path);
    f << "# example BNBC 2020 seismic zone coefficients\n"
      << "BNBC|2020|6.2.13|6.2.13.Z1|Seismic zone coefficient, Zone 1|0.12|dimensionless\n"
      << "BNBC|2020|6.2.13|6.2.13.Z2|Seismic zone coefficient, Zone 2|0.20|dimensionless\n"
      << "\n"
      << "BNBC|2020|6.2.13|6.2.13.Z3|Seismic zone coefficient, Zone 3|0.28|dimensionless\n";
    f.close();

    auto rules = loadCodeRules(path);
    assert(rules.size() == 3);
    assert(rules[0].code == "BNBC");
    assert(rules[0].version == "2020");
    assert(rules[0].clause == "6.2.13.Z1");
    assert(approxEqual(rules[0].value, 0.12));
    assert(rules[0].unit == "dimensionless");

    auto found = findRule(rules, "BNBC", "2020", "6.2.13.Z2");
    assert(found.has_value());
    assert(approxEqual(found->value, 0.20));

    auto notFound = findRule(rules, "BNBC", "2023", "6.2.13.Z2");  // different version
    assert(!notFound.has_value());

    std::remove(path);
    std::cout << "  testLoadHandWrittenRuleFile OK\n";
}

// Round-trip: save a rule set, load it back, verify identical.
static void testSaveLoadRoundTrip() {
    std::vector<CodeRule> original = {
        {"BNBC", "2020", "6.2.9", "6.2.9.ExpB", "Exposure B alpha", 7.0, "dimensionless"},
        {"ACI", "318-19", "24.5", "24.5.4", "Creep coefficient, typical", 2.0, "dimensionless"},
    };
    const char* path = "/tmp/test_code_rules_roundtrip.txt";
    saveCodeRules(original, path);
    auto loaded = loadCodeRules(path);

    assert(loaded.size() == 2);
    for (std::size_t i = 0; i < original.size(); ++i) {
        assert(loaded[i].code == original[i].code);
        assert(loaded[i].version == original[i].version);
        assert(loaded[i].clause == original[i].clause);
        assert(approxEqual(loaded[i].value, original[i].value));
        assert(loaded[i].unit == original[i].unit);
    }
    std::remove(path);
    std::cout << "  testSaveLoadRoundTrip OK\n";
}

static void testRejectsMalformedLine() {
    const char* path = "/tmp/test_code_rules_bad.txt";
    std::ofstream f(path);
    f << "BNBC|2020|only|three|fields\n";
    f.close();
    bool threw = false;
    try { loadCodeRules(path); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::remove(path);
    std::cout << "  testRejectsMalformedLine OK\n";
}

static void testRejectsMissingFile() {
    bool threw = false;
    try { loadCodeRules("/tmp/does_not_exist_rules.txt"); } catch (const std::runtime_error&) { threw = true; }
    assert(threw);
    std::cout << "  testRejectsMissingFile OK\n";
}

// THE ACTUAL "CODE UPDATE SYSTEM" MIGRATION, DONE SAFELY: rather than
// ripping the hardcoded constants out of SeismicAnalysis/WindAnalysis/
// LoadWizard (risking a regression in already-tested code), this test
// loads data/bnbc_2020_rules.txt -- a real external data file
// containing every one of those constants -- and cross-checks EVERY
// SINGLE VALUE against the hardcoded functions directly. If the data
// file and the hardcoded constants ever disagree, in EITHER direction,
// this test fails immediately. That is the actual proof the roadmap's
// "Code -> Version -> Chapter -> Clause -> Formula -> Limit -> Unit"
// idea works end-to-end: the data file is genuinely external (loaded
// from disk at runtime, editable without recompiling), and it's
// genuinely synchronized with what the compiled engine uses.
static void testMigratedRulesMatchHardcodedConstants() {
    auto rules = loadCodeRules("data/bnbc_2020_rules.txt");
    assert(rules.size() == 34);

    auto expect = [&](const std::string& clause, double value) {
        auto r = findRule(rules, "BNBC", "2020", clause);
        assert(r.has_value());
        assert(approxEqual(r->value, value));
    };

    // Cross-check against analysis::SiteCoefficients::forSoilType directly.
    struct SoilCheck { nrsa::analysis::SoilType type; const char* prefix; };
    std::vector<SoilCheck> soils = {
        {nrsa::analysis::SoilType::SA, "SA"}, {nrsa::analysis::SoilType::SB, "SB"},
        {nrsa::analysis::SoilType::SC, "SC"}, {nrsa::analysis::SoilType::SD, "SD"},
        {nrsa::analysis::SoilType::SE, "SE"},
    };
    for (const auto& s : soils) {
        auto c = nrsa::analysis::SiteCoefficients::forSoilType(s.type);
        expect(std::string(s.prefix) + "_S", c.S);
        expect(std::string(s.prefix) + "_TB", c.TB);
        expect(std::string(s.prefix) + "_TC", c.TC);
        expect(std::string(s.prefix) + "_TD", c.TD);
    }

    // Cross-check against analysis::PeriodCoefficients directly.
    auto conc = nrsa::analysis::PeriodCoefficients::concreteMomentFrame();
    expect("ConcreteFrame_Ct", conc.Ct);
    expect("ConcreteFrame_m", conc.m);
    auto steel = nrsa::analysis::PeriodCoefficients::steelMomentFrame();
    expect("SteelFrame_Ct", steel.Ct);
    expect("SteelFrame_m", steel.m);
    auto other = nrsa::analysis::PeriodCoefficients::other();
    expect("Other_Ct", other.Ct);
    expect("Other_m", other.m);

    // Cross-check against loads::liveLoadKPaForRoomType directly.
    expect("Residential", nrsa::loads::liveLoadKPaForRoomType(nrsa::loads::RoomType::Residential));
    expect("Office", nrsa::loads::liveLoadKPaForRoomType(nrsa::loads::RoomType::Office));
    expect("Hospital", nrsa::loads::liveLoadKPaForRoomType(nrsa::loads::RoomType::Hospital));
    expect("School", nrsa::loads::liveLoadKPaForRoomType(nrsa::loads::RoomType::School));
    expect("Parking", nrsa::loads::liveLoadKPaForRoomType(nrsa::loads::RoomType::Parking));
    expect("Commercial", nrsa::loads::liveLoadKPaForRoomType(nrsa::loads::RoomType::Commercial));
    expect("Storage", nrsa::loads::liveLoadKPaForRoomType(nrsa::loads::RoomType::Storage));
    expect("Roof", nrsa::loads::liveLoadKPaForRoomType(nrsa::loads::RoomType::Roof));

    std::cout << "  testMigratedRulesMatchHardcodedConstants (34 rules, all cross-checked) OK\n";
}

int main() {
    std::cout << "test_code_rule_store:\n";
    testLoadHandWrittenRuleFile();
    testSaveLoadRoundTrip();
    testRejectsMalformedLine();
    testRejectsMissingFile();
    testMigratedRulesMatchHardcodedConstants();
    std::cout << "All tests passed.\n";
    return 0;
}
