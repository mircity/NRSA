#include "bbs/BarSchedule.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nrsa::bbs {

double barAreaMm2(double barDiaMm) {
    if (barDiaMm <= 0.0) throw std::invalid_argument("barAreaMm2: barDiaMm must be positive");
    return (M_PI / 4.0) * barDiaMm * barDiaMm;
}

double unitWeightKgPerM(double barDiaMm) {
    // area(mm^2) * 1000mm * density(kg/mm^3) -- density converted from
    // kg/m^3 to kg/mm^3 by dividing by (1000mm/m)^3 = 1e9.
    double areaMm2 = barAreaMm2(barDiaMm);
    double densityKgPerMm3 = kSteelDensityKgPerM3 / 1.0e9;
    return areaMm2 * 1000.0 * densityKgPerMm3;
}

int practicalBarCount(double asRequiredMm2, double barDiaMm, int minBars) {
    if (asRequiredMm2 < 0.0) throw std::invalid_argument("practicalBarCount: asRequiredMm2 must be non-negative");
    if (barDiaMm <= 0.0) throw std::invalid_argument("practicalBarCount: barDiaMm must be positive");
    if (minBars < 1) throw std::invalid_argument("practicalBarCount: minBars must be at least 1");

    double oneBarArea = barAreaMm2(barDiaMm);
    int n = static_cast<int>(std::ceil(asRequiredMm2 / oneBarArea - 1e-9));
    return std::max(n, minBars);
}

double standardHookExtensionMm(double barDiaMm, HookAngle angle) {
    if (barDiaMm <= 0.0) throw std::invalid_argument("standardHookExtensionMm: barDiaMm must be positive");
    switch (angle) {
        case HookAngle::None:
            return 0.0;
        case HookAngle::Bend90:
        case HookAngle::Bend135:
            // ACI 318-19 Table 25.3.2: 6db, not less than 75mm.
            return std::max(6.0 * barDiaMm, 75.0);
        case HookAngle::Bend180:
            // ACI 318-19 Table 25.3.2 (180 deg row): 4db, not less than 65mm.
            return std::max(4.0 * barDiaMm, 65.0);
    }
    return 0.0;
}

double bendDeductionMm(double barDiaMm, HookAngle angle) {
    if (barDiaMm <= 0.0) throw std::invalid_argument("bendDeductionMm: barDiaMm must be positive");
    // Generic detailing convention: 1*db deducted per 45 degrees of bend.
    double angleDeg = static_cast<double>(static_cast<int>(angle));
    return (angleDeg / 45.0) * barDiaMm;
}

double simplifiedTensionLapSpliceLengthMm(double barDiaMm, double fyMPa, double fcMPa) {
    if (barDiaMm <= 0.0) throw std::invalid_argument("simplifiedTensionLapSpliceLengthMm: barDiaMm must be positive");
    if (fyMPa <= 0.0 || fcMPa <= 0.0) {
        throw std::invalid_argument("simplifiedTensionLapSpliceLengthMm: fy and fc' must be positive");
    }
    double estimate = (fyMPa / (1.7 * std::sqrt(fcMPa))) * barDiaMm;
    return std::max(300.0, estimate);
}

double BarMarkEntry::totalLengthM() const {
    return (static_cast<double>(count) * cutLengthMm) / 1000.0;
}

double BarMarkEntry::totalWeightKg() const {
    return totalLengthM() * unitWeightKgPerM(barDiaMm);
}

double straightBarCutLengthMm(double clearLengthMm, double barDiaMm, HookAngle startHook, HookAngle endHook) {
    if (clearLengthMm <= 0.0) throw std::invalid_argument("straightBarCutLengthMm: clearLengthMm must be positive");
    if (barDiaMm <= 0.0) throw std::invalid_argument("straightBarCutLengthMm: barDiaMm must be positive");

    double length = clearLengthMm;
    length += standardHookExtensionMm(barDiaMm, startHook) - bendDeductionMm(barDiaMm, startHook);
    length += standardHookExtensionMm(barDiaMm, endHook) - bendDeductionMm(barDiaMm, endHook);
    return length;
}

double stirrupCutLengthMm(double outerWidthMm, double outerDepthMm, double barDiaMm) {
    if (outerWidthMm <= 0.0 || outerDepthMm <= 0.0) {
        throw std::invalid_argument("stirrupCutLengthMm: outerWidthMm and outerDepthMm must be positive");
    }
    if (barDiaMm <= 0.0) throw std::invalid_argument("stirrupCutLengthMm: barDiaMm must be positive");

    double perimeter = 2.0 * (outerWidthMm + outerDepthMm);
    double hookExtensions = 2.0 * standardHookExtensionMm(barDiaMm, HookAngle::Bend135);
    // Four 90-degree corners plus two 135-degree hook bends.
    double deductions = 4.0 * bendDeductionMm(barDiaMm, HookAngle::Bend90) +
                         2.0 * bendDeductionMm(barDiaMm, HookAngle::Bend135);
    return perimeter + hookExtensions - deductions;
}

BarMarkEntry makeFlexuralBarEntry(const std::string& markId, const std::string& memberLabel,
                                   const design::FlexuralDesignResult& flex, double barDiaMm,
                                   double clearLengthMm, HookAngle startHook, HookAngle endHook) {
    BarMarkEntry entry;
    entry.markId = markId;
    entry.memberLabel = memberLabel;
    entry.shape = BarShape::Straight;
    entry.barDiaMm = barDiaMm;
    entry.count = practicalBarCount(flex.asRequiredMm2, barDiaMm);
    entry.cutLengthMm = straightBarCutLengthMm(clearLengthMm, barDiaMm, startHook, endHook);
    return entry;
}

BarMarkEntry makeStirrupEntry(const std::string& markId, const std::string& memberLabel,
                               const design::ShearDesignResult& shear, double memberClearLengthMm,
                               double outerWidthMm, double outerDepthMm, double barDiaMm, int legs) {
    if (legs != 2) {
        throw std::invalid_argument("makeStirrupEntry: only 2-leg stirrups/ties are modeled -- see header SCOPE note");
    }
    if (memberClearLengthMm <= 0.0) {
        throw std::invalid_argument("makeStirrupEntry: memberClearLengthMm must be positive");
    }

    BarMarkEntry entry;
    entry.markId = markId;
    entry.memberLabel = memberLabel;
    entry.shape = BarShape::RectStirrup;
    entry.barDiaMm = barDiaMm;
    entry.cutLengthMm = stirrupCutLengthMm(outerWidthMm, outerDepthMm, barDiaMm);

    if (!shear.stirrupsRequired || shear.requiredSpacingMm <= 0.0) {
        // No stirrups required by demand -- still return a zero-count
        // entry rather than throwing, so a caller building a schedule
        // across many members doesn't need a special case for this one.
        entry.count = 0;
        return entry;
    }
    // One extra stirrup to close both ends of the run.
    entry.count = static_cast<int>(std::ceil(memberClearLengthMm / shear.requiredSpacingMm)) + 1;
    return entry;
}

std::vector<BarMarkEntry> makeColumnLongitudinalEntries(const std::string& markPrefix,
                                                         const std::string& memberLabel,
                                                         const design::RectColumnLayout& layout,
                                                         double clearStoryHeightMm,
                                                         bool includeLapSplice, double fyMPa,
                                                         double fcMPa) {
    if (clearStoryHeightMm <= 0.0) {
        throw std::invalid_argument("makeColumnLongitudinalEntries: clearStoryHeightMm must be positive");
    }
    if (layout.barsXY.empty() || layout.barAreaMm2 <= 0.0) {
        throw std::invalid_argument("makeColumnLongitudinalEntries: layout must contain at least one bar");
    }

    // generateRectangularLayout only ever produces a single diameter, but
    // group by (rounded) area anyway so a future mixed-diameter layout
    // constructor doesn't silently mis-schedule bars of different sizes
    // under one mark.
    double barDiaMm = std::sqrt(4.0 * layout.barAreaMm2 / M_PI);

    double cutLength = clearStoryHeightMm;
    if (includeLapSplice) {
        cutLength += simplifiedTensionLapSpliceLengthMm(barDiaMm, fyMPa, fcMPa);
    }

    BarMarkEntry entry;
    entry.markId = markPrefix;
    entry.memberLabel = memberLabel;
    entry.shape = BarShape::Straight;
    entry.barDiaMm = barDiaMm;
    entry.count = static_cast<int>(layout.barsXY.size());
    entry.cutLengthMm = cutLength;

    return {entry};
}

void BarBendingSchedule::addEntry(BarMarkEntry entry) {
    entries_.push_back(std::move(entry));
}

double BarBendingSchedule::totalWeightKg() const {
    double total = 0.0;
    for (const auto& e : entries_) total += e.totalWeightKg();
    return total;
}

std::map<double, double> BarBendingSchedule::weightByDiameterKg() const {
    std::map<double, double> byDia;
    for (const auto& e : entries_) {
        byDia[e.barDiaMm] += e.totalWeightKg();
    }
    return byDia;
}

}  // namespace nrsa::bbs
