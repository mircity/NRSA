#pragma once

#include <string>

namespace nrsa::design {

// Flexural design result for a singly-reinforced rectangular section
// under a factored moment Mu. Units: mm^2 for areas, mm for lengths
// entering/leaving this struct (the usual rebar-detailing convention),
// even though the REST of this engine works in kN/m/kPa -- the
// conversion happens at this module's boundary, once, rather than
// leaking mixed units into the calling code.
struct FlexuralDesignResult {
    double asRequiredMm2 = 0.0;   // governing required steel area
    double rhoProvided = 0.0;     // asRequiredMm2 / (b*d), before rounding to real bars
    double rhoMin = 0.0;          // ACI 318-19 Section 9.6.1.2
    double rhoMax = 0.0;          // tension-controlled limit (this module's own approximation -- see designFlexure's doc comment)
    bool governedByMinimum = false;   // rhoMin controlled, not the moment demand itself
    bool exceedsMaximum = false;      // demand exceeds what a singly-reinforced section can carry within rhoMax -- needs compression steel or a bigger section
    std::string note;
};

struct ShearDesignResult {
    double vcKN = 0.0;              // concrete shear capacity (unreduced, i.e. Vc not phi*Vc)
    bool stirrupsRequired = false;  // Vu > phi*Vc/2 (ACI 318-19 Section 9.6.3.1 -- stirrups needed at all)
    double requiredSpacingMm = 0.0;  // 0 if stirrups aren't required
    double maxSpacingMm = 0.0;       // ACI 318-19 Section 9.7.6.2.2 spacing cap, for comparison
    bool exceedsMaximumVs = false;   // Vs demand exceeds the 0.66*sqrt(fc')*b*d cap -- section is too small regardless of stirrup spacing
    std::string note;
};

// Singly-reinforced rectangular beam flexural design per ACI 318-19
// Chapter 9 / BNBC 2020 Part 6 Chapter 6 (numerically identical
// provisions): given a factored moment Mu, section width b and
// effective depth d, and material strengths, returns the required
// tension steel area.
//
// phi = 0.9 is assumed throughout (tension-controlled section) rather
// than computed from strain compatibility -- a common, slightly
// conservative simplification for a first design pass; a section that
// comes out needing more than rhoMax steel to carry Mu
// (exceedsMaximum=true) is exactly the case where that assumption
// would need revisiting (a compression-controlled or transition
// section needs its own phi-vs-strain calculation, not implemented
// here).
//
// rhoMax here is this module's own simplified proxy for the
// tension-controlled limit (a strain-compatibility-based c/d ratio,
// not ACI 318-19's exact net-tensile-strain check) -- adequate to flag
// "this section is getting over-reinforced, look closer" but not a
// substitute for the code's own exact Section 21.2.2 provisions when
// precise ductility classification matters (e.g. for seismic design).
FlexuralDesignResult designFlexure(double muKNm, double bM, double dM, double fcMPa, double fyMPa);

// Beam shear design per ACI 318-19 Chapter 9 / BNBC 2020 Part 6 Chapter
// 6: given a factored shear Vu, section width b and effective depth d,
// material strengths, and a chosen stirrup bar (diameter, number of
// legs), returns the required stirrup spacing.
ShearDesignResult designShear(double vuKN, double bM, double dM, double fcMPa, double fyMPa,
                               double stirrupDiaMm, int legs);

}  // namespace nrsa::design
