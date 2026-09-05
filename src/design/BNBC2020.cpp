#include "design/BNBC2020.h"

namespace nrsa::design {

std::vector<LoadCombination> generateBasicCombinations(const BnbcLoadCaseIds& ids, int startId) {
    std::vector<LoadCombination> combos;
    int nextId = startId;

    bool hasD = ids.dead >= 0;
    bool hasL = ids.live >= 0;
    bool hasLr = ids.liveRoof >= 0;
    bool hasS = ids.snow >= 0;
    bool hasR = ids.rain >= 0;
    bool hasW = ids.wind >= 0;
    bool hasE = ids.seismic >= 0;
    bool hasRoofLike = hasLr || hasS || hasR;

    // Eq. 5.3.1a: 1.4D
    if (hasD) {
        LoadCombination c(nextId++, "1.4D");
        c.addFactor(ids.dead, 1.4);
        combos.push_back(std::move(c));
    }

    // Eq. 5.3.1b: 1.2D + 1.6L + 0.5(Lr or S or R)
    if (hasD || hasL || hasRoofLike) {
        LoadCombination c(nextId++, "1.2D + 1.6L + 0.5(Lr or S or R)");
        if (hasD) c.addFactor(ids.dead, 1.2);
        if (hasL) c.addFactor(ids.live, 1.6);
        if (hasLr) c.addFactor(ids.liveRoof, 0.5);
        else if (hasS) c.addFactor(ids.snow, 0.5);
        else if (hasR) c.addFactor(ids.rain, 0.5);
        combos.push_back(std::move(c));
    }

    // Eq. 5.3.1c: 1.2D + 1.6(Lr or S or R) + (1.0L or 0.5W) --
    // fundamentally the ROOF-LOAD-governs combination; without any
    // roof-like load (Lr/S/R) present it collapses to a subset of (b)
    // or (d), so require hasRoofLike specifically.
    if (hasRoofLike) {
        LoadCombination c(nextId++, "1.2D + 1.6(Lr or S or R) + (1.0L or 0.5W)");
        if (hasD) c.addFactor(ids.dead, 1.2);
        if (hasLr) c.addFactor(ids.liveRoof, 1.6);
        else if (hasS) c.addFactor(ids.snow, 1.6);
        else if (hasR) c.addFactor(ids.rain, 1.6);
        if (hasL) c.addFactor(ids.live, 1.0);
        else if (hasW) c.addFactor(ids.wind, 0.5);
        combos.push_back(std::move(c));
    }

    // Eq. 5.3.1d: 1.2D + 1.0W + 1.0L + 0.5(Lr or S or R) -- fundamentally
    // a WIND combination; without wind present it reduces to a gravity
    // combo already covered by (b)/(c), so require hasW specifically
    // rather than "any of these load types".
    if (hasW) {
        LoadCombination c(nextId++, "1.2D + 1.0W + 1.0L + 0.5(Lr or S or R)");
        if (hasD) c.addFactor(ids.dead, 1.2);
        c.addFactor(ids.wind, 1.0);
        if (hasL) c.addFactor(ids.live, 1.0);
        if (hasLr) c.addFactor(ids.liveRoof, 0.5);
        else if (hasS) c.addFactor(ids.snow, 0.5);
        else if (hasR) c.addFactor(ids.rain, 0.5);
        combos.push_back(std::move(c));
    }

    // Eq. 5.3.1e: 1.2D + 1.0E + 1.0L + 0.2S -- fundamentally a SEISMIC
    // combination; requires hasE for the same reason (d) requires hasW.
    if (hasE) {
        LoadCombination c(nextId++, "1.2D + 1.0E + 1.0L + 0.2S");
        if (hasD) c.addFactor(ids.dead, 1.2);
        c.addFactor(ids.seismic, 1.0);
        if (hasL) c.addFactor(ids.live, 1.0);
        if (hasS) c.addFactor(ids.snow, 0.2);
        combos.push_back(std::move(c));
    }

    // Eq. 5.3.1f: 0.9D + 1.0W (net uplift/overturning check) -- requires
    // hasW; D is added if present but the combination itself is
    // meaningless without wind.
    if (hasW) {
        LoadCombination c(nextId++, "0.9D + 1.0W");
        if (hasD) c.addFactor(ids.dead, 0.9);
        c.addFactor(ids.wind, 1.0);
        combos.push_back(std::move(c));
    }

    // Eq. 5.3.1g: 0.9D + 1.0E -- requires hasE, same reasoning as (f).
    if (hasE) {
        LoadCombination c(nextId++, "0.9D + 1.0E");
        if (hasD) c.addFactor(ids.dead, 0.9);
        c.addFactor(ids.seismic, 1.0);
        combos.push_back(std::move(c));
    }

    return combos;
}

}  // namespace nrsa::design
