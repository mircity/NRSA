#pragma once

#include <cmath>
#include <string>

namespace nrsa {

enum class MaterialKind { Concrete, ReinforcingSteel, StructuralSteel, Generic };

// Linear-elastic material properties used by every element type (frame,
// shell, solid). Nonlinear material behavior (concrete cracking, steel
// yielding for a pushover) is layered on top of this in
// analysis/NonlinearAnalysis — this class only ever holds the elastic
// backbone properties an element's stiffness matrix needs.
class Material {
public:
    Material(int id, std::string name, MaterialKind kind,
             double E_kPa, double poisson, double unitWeightKNm3)
        : id_(id), name_(std::move(name)), kind_(kind),
          E_(E_kPa), poisson_(poisson), unitWeight_(unitWeightKNm3) {
        // Isotropic shear modulus — callers needing an anisotropic or
        // directly-specified G should construct with setShearModulus()
        // afterward rather than relying on this default.
        G_ = E_ / (2.0 * (1.0 + poisson_));
    }

    int id() const { return id_; }
    const std::string& name() const { return name_; }
    MaterialKind kind() const { return kind_; }

    double elasticModulusKPa() const { return E_; }
    double shearModulusKPa() const { return G_; }
    void setShearModulus(double G_kPa) { G_ = G_kPa; }
    double poissonRatio() const { return poisson_; }
    double unitWeightKNm3() const { return unitWeight_; }

    // Mass density (unit weight / g), kg/m^3-equivalent in kN·s^2/m^4 —
    // the form a lumped-mass matrix needs directly.
    double massDensity() const { return unitWeight_ / 9.81; }

    double compressiveStrengthMPa() const { return fc_MPa_; }
    double yieldStrengthMPa() const { return fy_MPa_; }

    // ---- Factories --------------------------------------------------
    // ACI 318-19 §19.2.2.1: Ec = 4700*sqrt(fc') MPa for normal-weight
    // concrete, fc' in MPa. Poisson's ratio 0.2 per common RC practice
    // (ACI 318 permits 0.17-0.20; BNBC 2020 §6.1 aligns with ACI here).
    static Material concrete(int id, double fcPrimeMPa,
                              const std::string& name = "") {
        double E_MPa = 4700.0 * std::sqrt(fcPrimeMPa);
        Material m(id, name.empty() ? ("C" + std::to_string(static_cast<int>(fcPrimeMPa))) : name,
                   MaterialKind::Concrete, E_MPa * 1000.0, 0.20, 23.6);
        m.fc_MPa_ = fcPrimeMPa;
        return m;
    }

    // Deformed reinforcing bar steel — Es = 200000 MPa per ACI 318-19
    // §20.2.2.2 / BNBC 2020, unless the caller has mill-test data
    // justifying a different value.
    static Material rebar(int id, double fyMPa, const std::string& name = "") {
        Material m(id, name.empty() ? ("Fy" + std::to_string(static_cast<int>(fyMPa))) : name,
                   MaterialKind::ReinforcingSteel, 200000.0 * 1000.0, 0.30, 78.5);
        m.fy_MPa_ = fyMPa;
        return m;
    }

    // Structural steel (sections, plates) — Es = 200000 MPa per AISC
    // 360 / BNBC 2020 steel provisions.
    static Material structuralSteel(int id, double fyMPa, const std::string& name = "") {
        Material m(id, name.empty() ? ("ASTM Fy" + std::to_string(static_cast<int>(fyMPa))) : name,
                   MaterialKind::StructuralSteel, 200000.0 * 1000.0, 0.30, 78.5);
        m.fy_MPa_ = fyMPa;
        return m;
    }

private:
    int id_;
    std::string name_;
    MaterialKind kind_;
    double E_;           // kPa
    double G_;            // kPa
    double poisson_;
    double unitWeight_;   // kN/m^3
    double fc_MPa_ = 0.0;
    double fy_MPa_ = 0.0;
};

}  // namespace nrsa
