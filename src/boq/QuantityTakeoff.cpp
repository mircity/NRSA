#include "boq/QuantityTakeoff.h"

#include <stdexcept>

namespace nrsa::boq {

double rectangularVolumeM3(double bM, double hM, double lengthM) {
    if (bM <= 0.0 || hM <= 0.0 || lengthM <= 0.0) {
        throw std::invalid_argument("rectangularVolumeM3: all dimensions must be positive");
    }
    return bM * hM * lengthM;
}

double rectangularBeamFormworkAreaM2(double bM, double hM, double lengthM) {
    if (bM <= 0.0 || hM <= 0.0 || lengthM <= 0.0) {
        throw std::invalid_argument("rectangularBeamFormworkAreaM2: all dimensions must be positive");
    }
    // Soffit (b) + two sides (h each), top left open.
    return (bM + 2.0 * hM) * lengthM;
}

double rectangularColumnFormworkAreaM2(double bM, double hM, double heightM) {
    if (bM <= 0.0 || hM <= 0.0 || heightM <= 0.0) {
        throw std::invalid_argument("rectangularColumnFormworkAreaM2: all dimensions must be positive");
    }
    return 2.0 * (bM + hM) * heightM;
}

double slabFormworkAreaM2(double bM, double lM) {
    if (bM <= 0.0 || lM <= 0.0) {
        throw std::invalid_argument("slabFormworkAreaM2: both dimensions must be positive");
    }
    return bM * lM;
}

double wallFormworkAreaM2(double lengthM, double heightM) {
    if (lengthM <= 0.0 || heightM <= 0.0) {
        throw std::invalid_argument("wallFormworkAreaM2: both dimensions must be positive");
    }
    return 2.0 * lengthM * heightM;
}

double footingFormworkAreaM2(double planBM, double planLM, double thicknessM) {
    if (planBM <= 0.0 || planLM <= 0.0 || thicknessM <= 0.0) {
        throw std::invalid_argument("footingFormworkAreaM2: all dimensions must be positive");
    }
    return 2.0 * (planBM + planLM) * thicknessM;
}

ConcreteQuantity makeBeamConcreteQuantity(const std::string& memberLabel, double bM, double hM,
                                           double lengthM, double fcMPa) {
    ConcreteQuantity q;
    q.memberLabel = memberLabel;
    q.fcMPa = fcMPa;
    q.volumeM3 = rectangularVolumeM3(bM, hM, lengthM);
    q.formworkAreaM2 = rectangularBeamFormworkAreaM2(bM, hM, lengthM);
    return q;
}

ConcreteQuantity makeColumnConcreteQuantity(const std::string& memberLabel, double bM, double hM,
                                             double heightM, double fcMPa) {
    ConcreteQuantity q;
    q.memberLabel = memberLabel;
    q.fcMPa = fcMPa;
    q.volumeM3 = rectangularVolumeM3(bM, hM, heightM);
    q.formworkAreaM2 = rectangularColumnFormworkAreaM2(bM, hM, heightM);
    return q;
}

ConcreteQuantity makeSlabConcreteQuantity(const std::string& memberLabel, double bM, double lM,
                                           double hMm, double fcMPa) {
    if (hMm <= 0.0) throw std::invalid_argument("makeSlabConcreteQuantity: hMm must be positive");
    ConcreteQuantity q;
    q.memberLabel = memberLabel;
    q.fcMPa = fcMPa;
    q.volumeM3 = rectangularVolumeM3(bM, lM, hMm / 1000.0);
    q.formworkAreaM2 = slabFormworkAreaM2(bM, lM);
    return q;
}

ConcreteQuantity makeWallConcreteQuantity(const std::string& memberLabel, double lengthM,
                                           double heightM, double tMm, double fcMPa) {
    if (tMm <= 0.0) throw std::invalid_argument("makeWallConcreteQuantity: tMm must be positive");
    ConcreteQuantity q;
    q.memberLabel = memberLabel;
    q.fcMPa = fcMPa;
    q.volumeM3 = rectangularVolumeM3(lengthM, heightM, tMm / 1000.0);
    q.formworkAreaM2 = wallFormworkAreaM2(lengthM, heightM);
    return q;
}

ConcreteQuantity makeFootingConcreteQuantity(const std::string& memberLabel, double planBM,
                                              double planLM, double thicknessMm, double fcMPa) {
    if (thicknessMm <= 0.0) throw std::invalid_argument("makeFootingConcreteQuantity: thicknessMm must be positive");
    ConcreteQuantity q;
    q.memberLabel = memberLabel;
    q.fcMPa = fcMPa;
    q.volumeM3 = rectangularVolumeM3(planBM, planLM, thicknessMm / 1000.0);
    q.formworkAreaM2 = footingFormworkAreaM2(planBM, planLM, thicknessMm / 1000.0);
    return q;
}

void QuantityTakeoff::addConcrete(ConcreteQuantity q) {
    concreteItems_.push_back(std::move(q));
}

void QuantityTakeoff::setRebarSchedule(const bbs::BarBendingSchedule& schedule) {
    rebarWeightKg_ = schedule.totalWeightKg();
    rebarWeightByDiaKg_ = schedule.weightByDiameterKg();
}

double QuantityTakeoff::totalConcreteVolumeM3() const {
    double total = 0.0;
    for (const auto& q : concreteItems_) total += q.volumeM3;
    return total;
}

double QuantityTakeoff::totalFormworkAreaM2() const {
    double total = 0.0;
    for (const auto& q : concreteItems_) total += q.formworkAreaM2;
    return total;
}

double QuantityTakeoff::totalRebarWeightKg() const {
    return rebarWeightKg_;
}

std::map<double, double> QuantityTakeoff::concreteVolumeByGradeM3() const {
    std::map<double, double> byGrade;
    for (const auto& q : concreteItems_) byGrade[q.fcMPa] += q.volumeM3;
    return byGrade;
}

double QuantityTakeoff::rebarWeightPerConcreteVolumeKgPerM3() const {
    double volume = totalConcreteVolumeM3();
    if (volume <= 0.0) return 0.0;
    return rebarWeightKg_ / volume;
}

}  // namespace nrsa::boq
