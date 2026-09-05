#pragma once

#include <vector>

namespace nrsa::design {

// Immediate (elastic) settlement of a shallow footing on GRANULAR
// (sand) soil, Schmertmann's strain-influence-factor method
// (Schmertmann 1970, peak-value refinement per Schmertmann, Hartman &
// Brown 1978) using a layered SPT-N profile -- porting into the
// compiled C++ engine the calculation that, until now, existed only in
// prototype/NRSA_RCC.html's geotechnical module (see docs/NRSA_Status.pdf
// and the roadmap-audit note this project's status reports carry: the
// Roadmap.pdf's Core Engine "done" list included Settlement, but the
// only implementation lived in the browser prototype, not here).
//
// PROVENANCE: two formula choices below are confirmed, verbatim,
// against the prototype's own source (grep'd out of its minified JS;
// the surrounding per-layer/summation code is minified past the point
// of reliable line-by-line extraction, so THAT part below is the
// standard textbook Schmertmann formulation, not a literal port):
//   - Es(kPa) = 400*(N+6)              [Bowles 1997 screening
//     correlation -- the prototype's own comment names this exact
//     source]
//   - IzPeak = 0.5 + 0.1*sqrt(netQ/sigmaVoDf)   [Schmertmann 1978
//     peak-value refinement, evaluated with sigmaVoDf = effective
//     overburden stress at the founding depth Df -- again matching the
//     prototype's own variable names in its display template]
//
// SHAPE (square/circular footing, L/B < ~2 -- see scope note below):
//   Iz(z=0)      = 0.1     (at the footing base)
//   Iz(z=B/2)    = IzPeak  (peak, per the 1978 refinement above)
//   Iz(z=2B)     = 0.0     (influence zone bottom)
//   linear between each pair of points.
//
// SCOPE: square/circular shape only (the prototype's own UI text this
// was reverse-engineered from explicitly frames it as sized for a
// square/near-square isolated footing) -- a strip-footing shape
// (Iz: 0.2 at z=0, peak at z=B, zero at z=4B, per the same 1978 paper)
// is a natural next addition for continuous wall footings, same
// "explicit, temporary scope limit" pattern as every other module in
// this project. Time-dependent creep (Schmertmann's C2 factor) is
// NOT applied here -- this returns the immediate/elastic component
// only, matching the prototype's own module split (its separate
// "Consolidation Settlement + Time-Rate" module handles the
// long-term, clay-specific piece; sand's creep component is a smaller,
// often-neglected addition most screening tools -- including,
// apparently, the prototype this replaces -- skip for a first pass).
struct SoilLayer {
    double thicknessM = 0.0;   // layer thickness within the 0..2B influence zone, m
    double sptN = 0.0;         // field (uncorrected) SPT blow count for this layer
};

struct SchmertmannResult {
    double settlementM = 0.0;          // total immediate settlement, m
    double depthCorrectionC1 = 0.0;    // C1
    double izPeak = 0.0;                // computed peak strain influence factor
    double influenceZoneDepthM = 0.0;   // 2B
    std::vector<double> layerContributionsM;  // per-layer settlement contribution, m (same order as input layers)
};

// Es(kPa) = 400*(N+6) -- Bowles (1997) screening correlation, confirmed
// against the prototype's own source comment (see class doc comment).
double soilModulusFromSptN_Bowles1997(double sptN);

// The Schmertmann strain influence factor Iz at depth z (m) below the
// footing base, for a square/circular footing of width B (m), given
// the computed peak value izPeak (use schmertmannIzPeak() below to get
// this from netQ/sigmaVoDf first).
double schmertmannStrainInfluenceFactor(double z, double B, double izPeak);

// IzPeak = 0.5 + 0.1*sqrt(netBearingPressureKpa / effectiveOverburdenAtDfKpa)
// -- Schmertmann 1978 peak-value refinement, confirmed against the
// prototype's own source (see class doc comment). Both arguments in
// kPa; effectiveOverburdenAtDfKpa is clamped to a minimum of 1 kPa to
// avoid a division blowup at zero overburden (surface footing), same
// defensive clamp the prototype's own source uses (Math.max(1, ...)).
double schmertmannIzPeak(double netBearingPressureKpa, double effectiveOverburdenAtDfKpa);

// C1 = 1 - 0.5*(effectiveOverburdenAtDfKpa / netBearingPressureKpa),
// clamped to a minimum of 0.5 -- the standard Schmertmann depth
// (embedment) correction factor.
double schmertmannDepthCorrectionC1(double netBearingPressureKpa, double effectiveOverburdenAtDfKpa);

// Runs the full layered calculation: for a square/circular footing of
// width B (m) carrying net bearing pressure netBearingPressureKpa
// (kPa) at founding depth with effective overburden
// effectiveOverburdenAtDfKpa (kPa), given the soil profile below
// founding level as a list of layers (their thicknesses must sum to
// at least 2*B -- layers beyond 2B are ignored, and it throws
// std::invalid_argument if they sum to less than 2*B, since the
// influence zone would then be under-specified), computes each
// layer's Es from its SPT-N (Bowles 1997), evaluates Iz at each
// layer's mid-depth, and sums C1 * (Iz/Es) * thickness per layer to
// get total settlement (C1 applied once to the total, not per layer,
// matching the standard formula S = C1 * sum(Iz/Es * dz)).
SchmertmannResult runSchmertmannSettlement(double footingWidthM, double netBearingPressureKpa,
                                            double effectiveOverburdenAtDfKpa,
                                            const std::vector<SoilLayer>& layers);

}  // namespace nrsa::design
