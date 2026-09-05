#include "analysis/ResponseSpectrum.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nrsa::analysis {

ResponseSpectrum::ResponseSpectrum(std::vector<double> periods, std::vector<double> spectralAccelerations)
    : periods_(std::move(periods)), sa_(std::move(spectralAccelerations)) {
    if (periods_.size() != sa_.size()) {
        throw std::invalid_argument("ResponseSpectrum: periods and accelerations size mismatch");
    }
    if (periods_.size() < 2) {
        throw std::invalid_argument("ResponseSpectrum: needs at least 2 points to interpolate");
    }
    for (std::size_t i = 1; i < periods_.size(); ++i) {
        if (periods_[i] <= periods_[i - 1]) {
            throw std::invalid_argument("ResponseSpectrum: periods must be strictly increasing");
        }
    }
}

double ResponseSpectrum::accelerationAt(double periodSeconds) const {
    if (periodSeconds <= periods_.front()) return sa_.front();
    if (periodSeconds >= periods_.back()) return sa_.back();
    auto it = std::upper_bound(periods_.begin(), periods_.end(), periodSeconds);
    std::size_t hi = static_cast<std::size_t>(it - periods_.begin());
    std::size_t lo = hi - 1;
    double t = (periodSeconds - periods_[lo]) / (periods_[hi] - periods_[lo]);
    return sa_[lo] + t * (sa_[hi] - sa_[lo]);
}

namespace {
// Der Kiureghian (1981) cross-modal correlation coefficient for CQC,
// assuming equal damping ratio zeta for both modes — the standard,
// widely-used form (see e.g. Chopra, "Dynamics of Structures", or the
// ASCE 4 commentary). r = omega_i/omega_j, always taken <= 1 by
// construction (order of arguments doesn't matter — the formula is
// symmetric in i,j once r is defined this way).
double crossCorrelationCQC(double omegaI, double omegaJ, double zeta) {
    double r = std::min(omegaI, omegaJ) / std::max(omegaI, omegaJ);
    double num = 8.0 * zeta * zeta * (1.0 + r) * std::pow(r, 1.5);
    double den = std::pow(1.0 - r * r, 2) + 4.0 * zeta * zeta * r * std::pow(1.0 + r, 2);
    return num / den;
}
}  // namespace

ResponseSpectrumResult ResponseSpectrumAnalysis::run(Options opts) {
    if (opts.direction < 0 || opts.direction > 2) {
        throw std::invalid_argument("ResponseSpectrumAnalysis::run: direction must be 0 (X), 1 (Y), or 2 (Z)");
    }

    ModalAnalysis modal(model_);
    auto modalResult = modal.run(opts.modalOptions);

    ResponseSpectrumResult result;
    result.skippedElementIds = modalResult.skippedElementIds;

    std::size_t nModesTotal = modalResult.modes.size();
    std::size_t nModes = (opts.numModes > 0)
        ? std::min(static_cast<std::size_t>(opts.numModes), nModesTotal)
        : nModesTotal;
    if (nModes == 0) {
        throw std::runtime_error("ResponseSpectrumAnalysis::run: modal analysis returned no modes");
    }

    // ---- Build the excitation influence vector r (1 at every free DOF
    // matching the requested direction's translational component,
    // 0 elsewhere) directly in the SAME reduced free-DOF numbering the
    // mode shapes are already expressed in — avoids expanding to full
    // per-node vectors just to build this.
    int freeDofCount = static_cast<int>(modalResult.modes[0].shape.size());
    std::vector<double> mass(static_cast<std::size_t>(freeDofCount), 0.0);
    std::vector<double> influence(static_cast<std::size_t>(freeDofCount), 0.0);
    double totalMassInDirection = 0.0;
    for (const auto& n : model_.nodes()) {
        auto d = n.dofIndices();
        int gi = d[opts.direction];
        if (gi < 0) continue;
        double m = n.translationalMass();
        mass[static_cast<std::size_t>(gi)] = m;
        influence[static_cast<std::size_t>(gi)] = 1.0;
        totalMassInDirection += m;
    }
    if (totalMassInDirection <= 0.0) {
        throw std::runtime_error(
            "ResponseSpectrumAnalysis::run: no translational mass found in the requested "
            "excitation direction — assign mass before running a response spectrum analysis.");
    }

    // ---- Per-mode participation factor, effective modal mass, and
    // peak modal displacement vector (still in reduced free-DOF form).
    struct ModalPeak {
        double omega;
        double period;
        double gamma;                  // participation factor
        double effectiveMass;
        std::vector<double> peakDisp;  // Gamma_k * Sa(T_k)/omega_k^2 * phi_k, size freeDofCount
    };
    std::vector<ModalPeak> peaks;
    peaks.reserve(nModes);
    double cumulativeMassRatio = 0.0;

    for (std::size_t k = 0; k < nModes; ++k) {
        const auto& mode = modalResult.modes[k];
        double gamma = 0.0;
        for (int i = 0; i < freeDofCount; ++i) {
            gamma += mode.shape(static_cast<std::size_t>(i)) * mass[static_cast<std::size_t>(i)] *
                     influence[static_cast<std::size_t>(i)];
        }
        double effectiveMass = gamma * gamma;
        double massRatio = effectiveMass / totalMassInDirection;
        cumulativeMassRatio += massRatio;

        double omega = std::sqrt(std::max(mode.eigenvalue, 0.0));
        double sa = spectrum_.accelerationAt(mode.period);
        double scale = (omega > 1e-9) ? gamma * sa / (omega * omega) : 0.0;
        std::vector<double> peakDisp(static_cast<std::size_t>(freeDofCount));
        for (int i = 0; i < freeDofCount; ++i) {
            peakDisp[static_cast<std::size_t>(i)] = scale * mode.shape(static_cast<std::size_t>(i));
        }
        peaks.push_back(ModalPeak{omega, mode.period, gamma, effectiveMass, std::move(peakDisp)});

        result.participation.push_back(
            ModalParticipation{static_cast<int>(k), mode.period, gamma, effectiveMass, massRatio, cumulativeMassRatio});
    }

    // ---- Cross-modal correlation matrix (identity for SRSS: rho_ii=1,
    // rho_ij=0 for i!=j — CQC's general form with all off-diagonal
    // terms zeroed reduces to exactly the SRSS combination, so both
    // methods share the same combination formula below).
    std::vector<std::vector<double>> rho(nModes, std::vector<double>(nModes, 0.0));
    for (std::size_t i = 0; i < nModes; ++i) {
        rho[i][i] = 1.0;
        if (opts.method == ModalCombinationMethod::CQC) {
            for (std::size_t j = i + 1; j < nModes; ++j) {
                double r = crossCorrelationCQC(peaks[i].omega, peaks[j].omega, opts.dampingRatio);
                rho[i][j] = r;
                rho[j][i] = r;
            }
        }
    }

    // ---- Combine peak modal displacements per free DOF:
    // combined(i) = sqrt(sum_j sum_k rho_jk * peak_j(i) * peak_k(i)).
    std::vector<double> combined(static_cast<std::size_t>(freeDofCount), 0.0);
    for (int i = 0; i < freeDofCount; ++i) {
        double sum = 0.0;
        for (std::size_t j = 0; j < nModes; ++j) {
            for (std::size_t k = 0; k < nModes; ++k) {
                sum += rho[j][k] * peaks[j].peakDisp[static_cast<std::size_t>(i)] *
                       peaks[k].peakDisp[static_cast<std::size_t>(i)];
            }
        }
        combined[static_cast<std::size_t>(i)] = std::sqrt(std::max(sum, 0.0));
    }

    // ---- Combined base shear: same rho-weighted combination applied
    // to each mode's modal base shear V_k = Gamma_k^2 * Sa(T_k) =
    // effectiveMass_k * Sa(T_k).
    std::vector<double> modalBaseShear(nModes);
    for (std::size_t k = 0; k < nModes; ++k) {
        modalBaseShear[k] = peaks[k].effectiveMass * spectrum_.accelerationAt(peaks[k].period);
    }
    double baseShearSumSq = 0.0;
    for (std::size_t j = 0; j < nModes; ++j) {
        for (std::size_t k = 0; k < nModes; ++k) {
            baseShearSumSq += rho[j][k] * modalBaseShear[j] * modalBaseShear[k];
        }
    }
    result.baseShear = std::sqrt(std::max(baseShearSumSq, 0.0));

    // ---- Expand the combined reduced-DOF displacement vector into a
    // full per-node result (0 at every restrained DOF — same recovery
    // pattern StaticAnalysis/ModalAnalysis::expandModeShape use).
    for (const auto& n : model_.nodes()) {
        NodeVector6 v;
        auto d = n.dofIndices();
        for (int i = 0; i < 6; ++i) {
            if (d[i] >= 0) v.set(static_cast<DOF>(i), combined[static_cast<std::size_t>(d[i])]);
        }
        result.displacements[n.id()] = v;
    }

    return result;
}

}  // namespace nrsa::analysis
