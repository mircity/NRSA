#pragma once

#include <vector>

#include "core/Model.h"

namespace nrsa::modeler {

// Roadmap Section 2 ("Universal Building Modeler") describes TWO
// distinct things under one heading: (1) an interactive 3D CAD-style
// drawing interface (Draw/Move/Copy/Mirror/Array/Trim/Snap/Undo-Redo,
// an engineer clicking Grid->Column->Beam->Wall->Slab->Opening->Stair
// ->Foundation into existence) and (2) "Auto Model Generation" — an
// engineer states building parameters (storeys, bay layout, floor
// height, member sizes) and the software generates the full grid,
// columns, beams, and slabs automatically. This module is (2) only.
//
// SCOPE, STATED PLAINLY: (1) is fundamentally a GUI/interaction
// feature — a viewport, mouse input, snapping, undo history — and this
// project has no GUI layer anywhere (there is no src/ui or equivalent;
// everything here is a headless computational engine consumed by
// prototype/NRSA_RCC.html's own browser-side interface). There is
// nothing in a C++ backend that "implements" click-to-draw. So this
// module deliberately covers only the generative, non-interactive
// half of Section 2 -- the half that IS a well-defined, testable,
// headless algorithm, and the one the roadmap itself calls out as the
// potential "killer feature" (see the roadmap's own worked example:
// 10-storey, 4x5 bays, 3.2m floors, 450x450 columns, 300x500 beams --
// runAutoModelGeneration() below is tested against exactly that
// example).
//
// FURTHER SCOPE LIMITS (documented, not hidden):
//   - Uniform bay widths only (a single bayWidthXm repeated baysX
//     times, same for Y) -- non-uniform/irregular bay spacing is a
//     natural next addition, same pattern as every other module here.
//   - Generates Grid + Columns + Beams + Slabs + exterior (perimeter)
//     Walls + self-weight Dead Load. Interior/partition walls,
//     openings in walls, Stairs, and Foundation design are the
//     roadmap's own separate chain steps (Grid -> Column -> Beam ->
//     Wall -> Slab -> Opening -> Stair -> Foundation) and are NOT
//     generated here -- openings need a panel-cutting representation
//     this Element/shell model doesn't have yet, stairs need flight/
//     landing geometry, and foundation SIZING is the Design Engine's
//     job (src/design/RCCFoundation), not a geometry-generation job.
//     Flagging these explicitly rather than letting "Walls and Loads
//     are now covered" read as "the whole chain is covered."
//   - Loads means SELF-WEIGHT ONLY (dead load derived directly from
//     each generated member/slab/wall's own volume x material unit
//     weight) -- floor finish, partition load, live load by room type,
//     etc. all need the room-type/code-table logic that is Section 5's
//     job ("Load Wizard"), not something geometry generation alone can
//     derive.
//   - The base level (z=0) is fully restrained (fixed base) --
//     appropriate for a slab-on-grade/footing-level assumption; a
//     caller modeling basements or pile-supported bases should
//     override the base nodes' restraints afterward.
struct BuildingGenerationParameters {
    int storeys = 1;
    int baysX = 1;
    int baysY = 1;
    double bayWidthXm = 5.0;
    double bayWidthYm = 5.0;
    double floorHeightM = 3.0;
    double columnWidthM = 0.4;
    double columnDepthM = 0.4;
    double beamWidthM = 0.3;
    double beamDepthM = 0.5;
    double slabThicknessM = 0.15;
    int materialId = 0;   // must already exist in the model (this generator does not create materials -- see scope note)
    bool generateSlabs = true;

    // Exterior (perimeter) walls only -- every bay along the four
    // outer boundary edges of the grid, every floor, no openings (see
    // scope note above). Interior partition walls are not generated;
    // a caller wanting those can add Wall elements directly with
    // Element(...,ElementKind::Wall,...) using the node ids in
    // GeneratedModel::nodeIds.
    bool generateExteriorWalls = false;
    double wallThicknessM = 0.2;

    // Self-weight-only dead load (see scope note above). Applied as
    // NodalLoad entries (not DistributedLoad) split evenly across each
    // element's own nodes -- a documented tributary-split
    // simplification, the same one WindAnalysis/SeismicAnalysis use
    // for their own story forces, chosen here specifically to avoid
    // needing each frame element's local-axis sign convention (a
    // vertical column's self-weight is axial in its own local frame,
    // a horizontal beam's is transverse -- resolving that correctly
    // per member is exactly the kind of local-axis-convention risk a
    // global-direction nodal split sidesteps entirely).
    bool generateSelfWeightLoad = false;
    int selfWeightLoadCaseId = -1;  // required if generateSelfWeightLoad is true

    // Starting ids for nodes and for elements (elements share ONE id
    // namespace in Model regardless of kind, so columns/beams/slabs are
    // allocated sequentially from elementIdStart, in that order, rather
    // than each having its own independent counter -- an earlier
    // version of this module gave each category its own start id and
    // defaulted them all to 1, which collided immediately since
    // Model::addElement rejects duplicate ids across ALL kinds, not
    // just within one kind; caught by test_auto_model_generator.cpp).
    // Kept separate from nodeIdStart (nodes DO have their own id
    // namespace, distinct from elements) so a caller combining a
    // generated building with other hand-built content can control
    // exactly where both ranges land.
    int nodeIdStart = 1;
    int elementIdStart = 1;
};

// Where, within [elementIdStart, elementIdStart+N), each generated
// category's ids ended up -- useful for a caller who wants to know the
// exact ranges without re-deriving the counts.
struct GeneratedIdRanges {
    int columnsFirst = 0, columnsCount = 0;
    int beamsFirst = 0, beamsCount = 0;
    int slabsFirst = 0, slabsCount = 0;
    int wallsFirst = 0, wallsCount = 0;
};

// The full grid of generated node ids, indexed [floorIndex][row][col]
// (floorIndex 0 = base/z=0, floorIndex storeys = roof) so a caller can
// look up "the node at grid line (2,3), floor 5" without re-deriving
// the id arithmetic runAutoModelGeneration() uses internally.
struct GeneratedModel {
    std::vector<std::vector<std::vector<int>>> nodeIds;  // [floor][j][i]
    std::vector<int> columnElementIds;
    std::vector<int> beamElementIds;
    std::vector<int> slabElementIds;
    std::vector<int> wallElementIds;
    GeneratedIdRanges idRanges;
    double totalSelfWeightKN = 0.0;  // sum of every generated member/slab/wall's self-weight, whether or not generateSelfWeightLoad actually applied it as a load -- always computed and reported so a caller can sanity-check the building's total weight even without turning the load on
};

// Generates the grid/columns/beams/(slabs)/(exterior walls)/(self-
// weight load) into the given Model per BuildingGenerationParameters
// and returns the id bookkeeping in GeneratedModel. Throws
// std::invalid_argument if storeys/baysX/baysY < 1, any dimension is
// <= 0, materialId does not already exist in the model, or
// generateSelfWeightLoad is true but selfWeightLoadCaseId was left at
// its default -1 (the caller must add that LoadCase to the model
// first, same explicit-loadCaseId convention every load-producing
// module in this project follows -- see e.g. TemperatureAnalysis).
// Column/beam/wall/slab sections are created fresh (via
// model.addSection) every call -- a caller generating many buildings
// in one Model who wants section REUSE should extract the ids from
// the first call and pass a variant of this function that accepts
// pre-built section ids instead (not provided here -- single-building
// generation is this module's scope).
GeneratedModel runAutoModelGeneration(Model& model, const BuildingGenerationParameters& params);

}  // namespace nrsa::modeler
