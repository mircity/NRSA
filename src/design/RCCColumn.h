#pragma once

#include <string>
#include <vector>

namespace nrsa::design {

// ---------------------------------------------------------------------------
// RCCColumn -- rectangular tied-column design under axial load + biaxial
// bending, per ACI 318-19 Chapter 22 / BNBC 2020 Part 6 Chapter 6 (the two
// are numerically identical for this provision set, as with RCCBeam).
//
// Method: full strain-compatibility P-M interaction diagrams are built
// independently about each geometric axis (Whitney rectangular stress
// block, linear strain distribution, ecu=0.003, bilinear elastic-perfectly-
// plastic steel), then combined via Bresler's reciprocal load method:
//
//     1/Pn(biaxial) = 1/Pnx + 1/Pny - 1/Po
//
// where Pnx, Pny are the uniaxial nominal capacities at the SAME
// eccentricity as the actual biaxial demand (ex=Mux/Pu, ey=Muy/Pu), and Po
// is the nominal capacity at zero eccentricity. This is a standard,
// code-permitted approximation (ACI 318-19 Commentary R22.4.2, also in
// MacGregor/Wight and Nilson) -- NOT exact, and known to lose accuracy for
// lightly loaded columns (Pu/Po < ~0.1), where the load-contour method is
// the more accurate alternative; that method is not implemented here.
// designBiaxialColumn() flags that regime via a note rather than silently
// returning a number that looks precise but isn't.
//
// phi varies continuously with the net tensile strain in the extreme
// tension layer of reinforcement, per ACI 318-19 Table 21.2.2 (spiral
// columns are not modeled -- this module assumes tied columns throughout,
// hence Pn,max = 0.80*Po per Section 22.4.2.1 and phi_compression-
// controlled = 0.65 rather than the spiral values).
// ---------------------------------------------------------------------------

// One group of longitudinal bars at a common distance from the reference
// (compression) face, along the bending direction being analyzed. Bars at
// (numerically) the same distance are pre-summed into one layer -- this is
// what a strain-compatibility sweep actually needs, not individual bar
// coordinates.
struct RebarLayer {
    double distFromFaceMm = 0.0;  // distance from the d=0 reference face, along the bending axis
    double totalAreaMm2 = 0.0;    // summed area of all bars at this distance
};

// A rectangular tied-column layout: bars placed around the full perimeter,
// two rows of nFaceBarsAlongB bars along the B-direction faces and
// nFaceBarsAlongH along the H-direction faces (corner bars counted once,
// shared between both). All bars the same diameter (the common case; a
// mixed-diameter layout would need its own constructor, not provided here).
struct RectColumnLayout {
    double bMm = 0.0;   // overall section dimension along local "B" (x) direction
    double hMm = 0.0;   // overall section dimension along local "H" (y) direction
    double barAreaMm2 = 0.0;
    std::vector<std::pair<double, double>> barsXY;  // (x,y) bar-center coords, mm, from the (0,0) corner
};

// Places bars around the perimeter of a b x h tied rectangular column.
// coverMm is clear cover to the TIE, barDiaMm the longitudinal bar
// diameter; bar centers are set at (cover + tie allowance folded into
// coverMm by the caller + barDia/2) from each face -- this function does
// not itself add a separate tie diameter, so pass an effective cover that
// already includes it if that distinction matters to you.
// nFaceBarsAlongB/H each include the two corner bars on that pair of
// faces; both must be >= 2.
RectColumnLayout generateRectangularLayout(double bM, double hM, double coverMm, double barDiaMm,
                                            int nFaceBarsAlongB, int nFaceBarsAlongH);

// Nominal axial capacity at zero eccentricity (concentric compression),
// ACI 318-19 Eq. 22.4.2.2: Po = 0.85*fc'*(Ag-Ast) + fy*Ast. Closed-form,
// independent of the strain-compatibility sweep below -- used both as an
// exact anchor point (Bresler's Po term) and as an independent cross-check
// that the interaction diagram's high-c tail is converging correctly.
double axialCapacityPo(double bMm, double hMm, double astMm2, double fcMPa, double fyMPa);

struct PMDiagramPoint {
    double cMm = 0.0;       // neutral-axis depth from the compression face that produced this point
    double pnKN = 0.0;      // nominal axial capacity
    double mnKNm = 0.0;     // nominal moment capacity about the section centroid
    double etExtreme = 0.0; // net tensile strain in the extreme tension-side layer (+ve = tension)
    double phi = 0.0;       // ACI 318-19 Table 21.2.2 strength-reduction factor at this point (tied column)
    double phiPnKN = 0.0;
    double phiMnKNm = 0.0;
};

// Builds a full nominal P-M interaction diagram for bending about ONE axis
// by sweeping the neutral-axis depth c from near zero (tension-controlled)
// to a multiple of the section depth (asymptotically approaching Po as
// a=beta1*c clips to h and all steel strains approach ecu). widthMm is the
// section dimension perpendicular to the bending direction (constant width
// of the Whitney block); depthMm is the dimension the bending direction
// acts over, and each layer's distFromFaceMm must be measured over that
// same 0..depthMm range from a consistent reference face.
std::vector<PMDiagramPoint> computeUniaxialInteractionDiagram(double widthMm, double depthMm,
                                                                const std::vector<RebarLayer>& layers,
                                                                double fcMPa, double fyMPa,
                                                                int numPoints = 100);

// Nominal uniaxial capacity Pn at a given target eccentricity e = Mn/Pn,
// found by linear interpolation along a (monotonically Pn-increasing,
// e-decreasing) diagram produced by computeUniaxialInteractionDiagram.
// Returns {0,0} and sets outOfRange=true if targetEMm falls entirely
// outside what the diagram spans (e.g. an eccentricity so large the
// section cannot carry it in net compression at all -- pure/near-pure
// bending or net axial tension, neither modeled by this compression-member
// module).
struct UniaxialCapacityAtE {
    double pnKN = 0.0;
    double phi = 0.0;
    bool outOfRange = false;
};
UniaxialCapacityAtE capacityAtEccentricity(const std::vector<PMDiagramPoint>& diagram, double targetEMm);

struct BiaxialDesignResult {
    double poKN = 0.0;
    double pnMaxKN = 0.0;    // 0.80*Po, ACI 318-19 Section 22.4.2.1 tied-column cap
    double exMm = 0.0, eyMm = 0.0;
    double pnxKN = 0.0, pnyKN = 0.0;  // uniaxial nominal capacities at the same eccentricities
    double pnBiaxialKN = 0.0;         // Bresler-combined nominal capacity (capped at pnMaxKN)
    double phi = 0.0;                 // conservative (minimum of the two governing uniaxial cases)
    double phiPnKN = 0.0;
    bool adequate = false;
    bool lowAxialLoadWarning = false;  // Pu/Po < 0.10 -- Bresler accuracy degrades here, see header note
    std::string note;
};

// Full biaxial design check for a rectangular tied column: given the
// factored demand (Pu, Mux, Muy) and a chosen section/rebar layout, reports
// whether phi*Pn(biaxial) >= Pu at that demand's eccentricity ratio.
// PuKN must be > 0 (net compression) -- this module does not handle net
// axial tension or pure-bending members (that's RCCBeam's job).
BiaxialDesignResult designBiaxialColumn(double bM, double hM, double coverMm, double barDiaMm,
                                         int nFaceBarsAlongB, int nFaceBarsAlongH,
                                         double fcMPa, double fyMPa,
                                         double puKN, double muxKNm, double muyKNm);

}  // namespace nrsa::design
