#pragma once

#include <string>

namespace nrsa::design {

// ---------------------------------------------------------------------------
// Steel -- structural steel member design/check per AISC 360-16, matching
// the scope already built out in the browser prototype for beam/column steel
// design (see prototype/README.md): doubly-symmetric rolled I-shapes (W/S/M/
// HP) only. Scope, deliberately bounded the same way every other design/
// module in this project states its own limits up front rather than
// hiding them:
//
//   - WShapeSection -- section properties computed PARAMETRICALLY from
//     plate dimensions (d, bf, tf, tw), the same pattern core::Section's own
//     factories (rectangular(), circular(), hollowRectangularTube()) already
//     use -- this project ships no literal AISC Manual shape table, so
//     buildWShape() is the equivalent of looking one up: give it the plate
//     dimensions of a real (or trial) shape and it returns A/I/S/Z/r/J/Cw/rts.
//     Root fillets between flange and web are NOT modeled (same "thin-walled,
//     no fillet" idealization core::Section::hollowRectangularTube already
//     uses) -- typically a 1-3% effect on A/Ix/Zx for a real rolled shape,
//     flagged here rather than silently absorbed into the numbers.
//   - Flexural design (Chapter F2) -- doubly-symmetric COMPACT I-shapes bent
//     about the major axis only: yielding (Mp), inelastic lateral-torsional
//     buckling, and elastic LTB. Non-compact/slender flange or web sections
//     (Chapter F3/F4/F5) are NOT modeled -- checkCompactness() flags this
//     rather than silently applying the compact-section formulas anyway.
//     Minor-axis flexure (Chapter F6) is not modeled.
//   - Shear design (Chapter G2.1, unstiffened webs only) -- tension-field
//     action (Chapter G3, stiffened webs) is not modeled.
//   - Compression design (Chapter E3, flexural buckling only) -- torsional
//     and flexural-torsional buckling (Chapter E4, governing for singly-
//     symmetric/unsymmetric shapes, rarely for a doubly-symmetric W) and
//     local buckling of slender elements (Chapter E7) are not modeled.
//   - Combined axial+flexure interaction (Chapter H1.1) only -- no torsion
//     interaction (H3), no member out-of-plane/stability-analysis-driven
//     amplification (Appendix 8/Chapter C second-order effects are the
//     caller's responsibility, same as RCCColumn takes already-factored
//     Pu/Mu rather than performing its own second-order analysis).
//
// NOT in scope here, matching the roadmap's own stated boundary for this
// module: composite (steel+concrete) member design, and bolted/welded
// connection design (both distinct modules in their own right).
//
// Validation approach, consistent with the rest of this project: round-trip
// design-then-recheck where applicable, closed-form checks at the compact/
// noncompact and elastic/inelastic buckling-regime boundaries Chapter E/F
// themselves define (the F2 provisions are specifically written to be
// EXACTLY continuous at Lb=Lr on the inelastic side, by construction, but
// only NEAR-continuous (~0.1%) on the elastic side -- Lr (F2-6) and the
// elastic Fcr formula (F2-4) are algebraic inverses of each other, but
// AISC publishes both with independently rounded constants (0.078, 1.95,
// 6.76), so they don't cancel out perfectly; the E3 provisions are
// similarly close-but-not-exactly continuous at KL/r's own transition
// point. Both are checked directly, at the tolerance each actually
// supports, in tests/test_design_steel.cpp, rather than assumed exact.
// ---------------------------------------------------------------------------

struct WShapeSection {
    std::string name;
    double dMm = 0.0;    // overall depth
    double bfMm = 0.0;   // flange width
    double tfMm = 0.0;   // flange thickness
    double twMm = 0.0;   // web thickness

    double areaMm2 = 0.0;
    double ixMm4 = 0.0, iyMm4 = 0.0;  // moment of inertia, strong (x) / weak (y) axis
    double sxMm3 = 0.0, syMm3 = 0.0;  // elastic section modulus
    double zxMm3 = 0.0, zyMm3 = 0.0;  // plastic section modulus
    double rxMm = 0.0, ryMm = 0.0;    // radius of gyration
    double jMm4 = 0.0;                // St. Venant torsional constant (open thin-walled, no fillets)
    double cwMm6 = 0.0;               // warping constant
    double rtsMm = 0.0;               // AISC F2-7 effective radius of gyration for LTB
    double hoMm = 0.0;                // distance between flange centroids
};

// Builds a doubly-symmetric I-shape's section properties from plate
// dimensions (all mm). See the module doc comment above for what this is
// (and isn't) equivalent to.
WShapeSection buildWShape(double dMm, double bfMm, double tfMm, double twMm,
                           const std::string& name = "");

struct CompactnessResult {
    double lambdaFlange = 0.0, lambdaPFlange = 0.0;  // bf/2tf vs 0.38*sqrt(E/Fy), Table B4.1b case 10
    double lambdaWeb = 0.0, lambdaPWeb = 0.0;        // h/tw vs 3.76*sqrt(E/Fy), Table B4.1b case 15
    bool flangeCompact = false;
    bool webCompact = false;
    bool compact = false;  // both compact -- only case designFlexuralCapacity's F2 path covers
};

// AISC 360-16 Table B4.1b compactness classification for a doubly-symmetric
// I-shape's flange and web in flexure. This module implements Chapter F2
// (compact sections) only -- see designFlexuralCapacity's own
// sectionNotCompact flag for what happens when this returns compact=false.
CompactnessResult checkCompactness(const WShapeSection& section, double fyMPa);

struct FlexuralCapacityResult {
    double mpKNm = 0.0;   // plastic moment, Fy*Zx
    double lpM = 0.0;     // AISC F2-5, unbraced length limit below which Mn=Mp
    double lrM = 0.0;     // AISC F2-6, unbraced length limit above which elastic LTB governs
    double mnKNm = 0.0;   // governing nominal moment for the given unbraced length Lb
    double phiMnKNm = 0.0;  // 0.90*Mn

    enum class Regime { Yielding, InelasticLTB, ElasticLTB } regime = Regime::Yielding;
    bool sectionNotCompact = false;  // checkCompactness() failed -- Mn below is NOT computed (see note)
    std::string note;
};

// Chapter F2 nominal flexural strength for a doubly-symmetric compact
// I-shape bent about the major axis, given the unbraced length Lb (m) and
// lateral-torsional-buckling modification factor Cb (1.0 -- uniform moment
// -- if the caller has no better estimate; AISC F1-1 gives the general
// non-uniform-moment formula, not computed here since it needs the moment
// diagram, not just section/length). If checkCompactness() finds the
// section non-compact, sectionNotCompact is set true and mnKNm/phiMnKNm are
// left at zero rather than silently applying the compact-section formulas
// to a section they don't apply to.
FlexuralCapacityResult designFlexuralCapacity(const WShapeSection& section, double fyMPa, double eMPa,
                                               double lbM, double cb = 1.0);

struct ShearCapacityResult {
    double awMm2 = 0.0;   // d*tw
    double hOverTw = 0.0;
    double cv1 = 0.0;
    double vnKN = 0.0;
    double phi = 0.0;      // 1.00 (G2.1a) or 0.90 (G2.1b)
    double phiVnKN = 0.0;
};

// Chapter G2.1 nominal shear strength for an unstiffened web (kv=5.34),
// doubly-symmetric I-shape bent about the major axis.
ShearCapacityResult checkShearCapacity(const WShapeSection& section, double fyMPa, double eMPa);

struct CompressionCapacityResult {
    double klrX = 0.0, klrY = 0.0;
    double governingKlr = 0.0;  // larger of the two -- the weaker-axis slenderness governs
    double feMPa = 0.0;         // elastic (Euler) buckling stress at the governing KL/r
    double fcrMPa = 0.0;
    double pnKN = 0.0;
    double phi = 0.90;
    double phiPnKN = 0.0;
    bool elasticBuckling = false;  // true if KL/r > 4.71*sqrt(E/Fy) -- E3-3 governs, not E3-2
};

// Chapter E3 nominal compressive strength (flexural buckling about either
// principal axis, whichever governs) for a doubly-symmetric I-shape, given
// each axis's own effective length KxLx/KyLy (m) -- pass the SAME value for
// both if the member is braced identically about both axes.
CompressionCapacityResult designCompressionCapacity(const WShapeSection& section, double fyMPa, double eMPa,
                                                     double kxLxM, double kyLyM);

struct CombinedInteractionResult {
    double prOverPc = 0.0;
    double ratio = 0.0;  // left-hand side of the governing H1-1a/b equation
    bool adequate = false;
    std::string equation;  // "H1-1a" or "H1-1b"
};

// Chapter H1.1 combined axial + biaxial flexure interaction check: given
// required (already-factored) Pr/Mrx/Mry and available (already phi-
// reduced) Pc/Mcx/Mcy, reports whether the member is adequate under the
// governing interaction equation (H1-1a for Pr/Pc>=0.2, H1-1b otherwise).
CombinedInteractionResult checkCombinedInteraction(double prKN, double pcKN,
                                                    double mrxKNm, double mcxKNm,
                                                    double mryKNm, double mcyKNm);

}  // namespace nrsa::design
