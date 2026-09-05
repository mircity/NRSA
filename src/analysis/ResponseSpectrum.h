#pragma once

#include <unordered_map>
#include <vector>

#include "analysis/ModalAnalysis.h"
#include "core/Model.h"

namespace nrsa::analysis {

// A tabulated (period, spectral acceleration) design response
// spectrum, linearly interpolated between points. Spectral
// acceleration values must be supplied in m/s^2 (NOT g's) — the same
// consistent kN-m-s unit system used everywhere else in this project —
// so a caller converting from a code spectrum given in g's must
// multiply by 9.81 before constructing this. Deliberately generic
// (just numbers in, numbers out) rather than hardcoding any specific
// code's spectrum SHAPE (BNBC 2020, ASCE 7, ...) — that shape belongs
// in design/BNBC2020 (not yet built) as a function that PRODUCES the
// (period, Sa) table this class consumes, keeping this analysis-layer
// class usable for any code's spectrum.
class ResponseSpectrum {
public:
    ResponseSpectrum(std::vector<double> periods, std::vector<double> spectralAccelerations);

    // Outside the supplied period range, holds the nearest endpoint's
    // value constant rather than extrapolating a slope — the safer
    // default for a spectrum that was only ever tabulated over the
    // range of interest.
    double accelerationAt(double periodSeconds) const;

private:
    std::vector<double> periods_;
    std::vector<double> sa_;
};

// Per-mode participation data — reported directly (not just used
// internally) because the effective-modal-mass-ratio / cumulative mass
// participation check (does the included mode set capture enough of
// the building's real mass to be a valid basis for design — most codes
// require at least 90% in each direction) is itself a required
// engineering check, not an implementation detail.
struct ModalParticipation {
    int modeIndex;
    double period;
    double participationFactor;      // Gamma_k = L_k (mass-normalized modes: Gamma_k == L_k)
    double effectiveModalMass;       // Gamma_k^2
    double effectiveMassRatio;       // Gamms_k^2 / total mass in the excitation direction
    double cumulativeMassRatio;      // running sum up to and including this mode
};

struct ResponseSpectrumResult {
    std::vector<ModalParticipation> participation;  // one entry per mode considered, sorted by period ascending
    std::unordered_map<int, NodeVector6> displacements;  // combined (SRSS or CQC), per node
    double baseShear = 0.0;  // combined, in the excitation direction
    std::vector<int> skippedElementIds;
};

enum class ModalCombinationMethod { SRSS, CQC };

struct ResponseSpectrumOptions {
    int direction = 0;  // 0=X (Ux), 1=Y (Uy), 2=Z (Uz) — the excitation direction
    ModalCombinationMethod method = ModalCombinationMethod::CQC;
    double dampingRatio = 0.05;  // used by CQC only (Der Kiureghian cross-correlation)
    int numModes = 0;            // 0 = use every mode ModalAnalysis returns
    ModalAnalysis::Options modalOptions{};
};

class ResponseSpectrumAnalysis {
public:
    using Options = ResponseSpectrumOptions;

    ResponseSpectrumAnalysis(Model& model, const ResponseSpectrum& spectrum)
        : model_(model), spectrum_(spectrum) {}

    // Runs ModalAnalysis internally (same coupled frame+shell stiffness
    // and lumped mass StaticAnalysis/ModalAnalysis use), then combines
    // each mode's peak modal response into a single design-level
    // result via the requested combination method.
    //
    // NOT YET DONE: per-node reaction and per-element internal-force
    // recovery combined the same way (only displacements and the
    // overall base shear are computed here) — a natural next addition,
    // following the same per-mode-then-combine pattern already used
    // for displacements and base shear below.
    ResponseSpectrumResult run(Options opts = Options{});

private:
    Model& model_;
    ResponseSpectrum spectrum_;
};

}  // namespace nrsa::analysis
