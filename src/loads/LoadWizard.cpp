#include "loads/LoadWizard.h"

#include <stdexcept>

#include "core/Load.h"

namespace nrsa::loads {

double liveLoadKPaForRoomType(RoomType type) {
    // BNBC 2020 Table 6.2.3 values -- see LoadWizard.h's doc comment
    // for the exact clause each one cites and which sub-case within a
    // broader category it represents.
    switch (type) {
        case RoomType::Residential: return 2.00;
        case RoomType::Office: return 2.40;
        case RoomType::Hospital: return 2.00;
        case RoomType::School: return 2.00;
        case RoomType::Parking: return 2.00;
        case RoomType::Commercial: return 4.80;
        case RoomType::Storage: return 6.00;
        case RoomType::Roof: return 1.00;
    }
    return 2.0;
}

PanelLoadResult applyRoomLoadToPanel(Model& model, const std::vector<int>& panelCornerNodeIds,
                                      double panelAreaM2, const DeadLoadComponents& deadLoad,
                                      RoomType roomType, int deadLoadCaseId, int liveLoadCaseId) {
    if (panelCornerNodeIds.empty()) {
        throw std::invalid_argument("applyRoomLoadToPanel: no panel corner nodes given");
    }
    if (panelAreaM2 <= 0.0) {
        throw std::invalid_argument("applyRoomLoadToPanel: panelAreaM2 must be positive");
    }

    PanelLoadResult result;
    std::size_t n = panelCornerNodeIds.size();

    double deadPressure = deadLoad.floorFinishKPa + deadLoad.partitionKPa + deadLoad.ceilingKPa +
                           deadLoad.waterproofingKPa;
    if (deadLoadCaseId >= 0 && deadPressure > 0.0) {
        double totalDead = deadPressure * panelAreaM2;
        double perNode = totalDead / static_cast<double>(n);
        for (int id : panelCornerNodeIds) {
            NodalLoad load;
            load.nodeId = id;
            load.loadCaseId = deadLoadCaseId;
            load.Fz = -perNode;
            model.addNodalLoad(load);
        }
        result.totalDeadForceKN = totalDead;
    }

    if (liveLoadCaseId >= 0) {
        double livePressure = liveLoadKPaForRoomType(roomType);
        double totalLive = livePressure * panelAreaM2;
        double perNode = totalLive / static_cast<double>(n);
        for (int id : panelCornerNodeIds) {
            NodalLoad load;
            load.nodeId = id;
            load.loadCaseId = liveLoadCaseId;
            load.Fz = -perNode;
            model.addNodalLoad(load);
        }
        result.totalLiveForceKN = totalLive;
    }

    return result;
}

double wallLineLoadKNPerM(double wallUnitWeightKNm3, double wallThicknessM, double wallHeightM) {
    if (wallUnitWeightKNm3 < 0.0 || wallThicknessM <= 0.0 || wallHeightM <= 0.0) {
        throw std::invalid_argument("wallLineLoadKNPerM: inputs must be positive");
    }
    return wallUnitWeightKNm3 * wallThicknessM * wallHeightM;
}

}  // namespace nrsa::loads
