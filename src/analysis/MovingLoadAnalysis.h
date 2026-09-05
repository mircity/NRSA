#pragma once

#include <vector>

namespace nrsa::analysis {

// One axle/wheel of a moving load train: magnitude (kN) and its fixed
// distance (m) from the FIRST (reference) load in the train, measured
// in the direction of travel. E.g. a 2-axle truck with 100 kN front
// axle and 150 kN rear axle 4 m behind it: {{100, 0.0}, {150, 4.0}}.
struct MovingPointLoad {
    double magnitude = 0.0;  // kN
    double offsetFromFirst = 0.0;  // m, >= 0
};

// Envelope result at ONE cross-section of the span (position x from
// the left support): the largest moment/shear that section ever sees
// as the load train sweeps across the span, and where the train's
// reference load was standing when that maximum occurred.
struct MovingLoadEnvelopePoint {
    double x = 0.0;                 // m from left support
    double maxMoment = 0.0;         // kN*m, largest (can be negative for shear-only sections but this module only returns the governing max magnitude case)
    double trainPositionAtMaxMoment = 0.0;  // m, reference load's position when maxMoment occurred
    double maxShear = 0.0;          // kN
    double trainPositionAtMaxShear = 0.0;
};

// Classical influence-line-based moving load analysis for a SINGLE-SPAN
// SIMPLY SUPPORTED beam (length spanM, m) -- the standard closed-form
// influence line for a simply supported beam:
//   Moment IL at section x due to a UNIT load at position a:
//     eta_M(x, a) = (L-x)*a/L,           a <= x
//                 = x*(L-a)/L,           a >  x
//   Shear IL at section x due to a UNIT load at position a:
//     eta_V(x, a) = -a/L,                a <  x   (load left of section)
//                 = (L-a)/L,             a >= x   (load right of section)
// (the standard sign/shape from any structural analysis text, e.g.
// Hibbeler "Structural Analysis" Ch. 9 -- the shear IL has the classic
// jump of magnitude 1 as the unit load crosses the section.)
//
// For a multi-load TRAIN, superposition applies: the ordinate at
// section x for a given train position is sum over each point load of
// (load magnitude) * eta(x, load's current position), evaluated only
// for loads that currently lie ON the span (0 <= position <= L) --
// a load that has moved off either end simply contributes zero.
//
// The train sweeps across the span (and some approach/exit distance so
// every axle gets a chance to be the critical one) in
// positionStepM increments; at every position and every one of
// sectionCountAlongSpan evenly-spaced sections, moment and shear are
// evaluated and the running maximum (by absolute value) is kept.
//
// SCOPE: single-span simply-supported only -- a continuous multi-span
// girder's influence lines are NOT the simple triangular/bilinear
// shapes used here (they require a separate flexibility/stiffness
// solve per unit-load position, e.g. via Muller-Breslau applied to
// StaticAnalysis rather than a closed form) -- a natural next addition,
// same explicit scope-limit pattern as every other module here.
std::vector<MovingLoadEnvelopePoint> runMovingLoadEnvelope(
    double spanM, const std::vector<MovingPointLoad>& train,
    int sectionCountAlongSpan = 21, double positionStepM = 0.1);

// The two closed-form influence-line ordinate functions themselves,
// exposed for direct unit testing against the textbook shapes.
double momentInfluenceOrdinate(double spanM, double sectionX, double unitLoadPosition);
double shearInfluenceOrdinate(double spanM, double sectionX, double unitLoadPosition);

}  // namespace nrsa::analysis
