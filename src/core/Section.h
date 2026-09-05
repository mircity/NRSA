#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace nrsa {

// Geometric cross-section properties, in meters/m^2/m^4 throughout (an
// element combines this with a Material's E/G to get physical stiffness
// — Section itself never touches material properties). Values default to
// zero so a hand-built Section only needs to set the fields a given
// element formulation actually reads (a shell element ignores J, for
// instance).
class Section {
public:
    Section() = default;
    explicit Section(std::string name) : name_(std::move(name)) {}

    const std::string& name() const { return name_; }
    void setName(std::string n) { name_ = std::move(n); }

    double area = 0.0;                 // A, m^2
    double shearAreaY = 0.0;           // Asy, m^2 (for Timoshenko shear deformation)
    double shearAreaZ = 0.0;           // Asz, m^2
    double momentOfInertiaY = 0.0;     // Iy, m^4 — bending about local y (out-of-plane for a beam)
    double momentOfInertiaZ = 0.0;     // Iz, m^4 — bending about local z (in-plane / strong axis for a beam)
    double torsionalConstant = 0.0;    // J, m^4
    double depth = 0.0;                // overall depth, m — kept for reporting/detailing, not used in stiffness
    double width = 0.0;                // overall width, m
    double thickness = 0.0;            // shell/plate thickness, m (zero for a frame section)

    // ---- Factories ----------------------------------------------------

    // Solid rectangular section, width b x depth h (m). Shear areas use
    // the common 5/6 correction factor for a rectangle (Timoshenko beam
    // theory); torsional constant uses the classic thin/thick rectangle
    // series solution truncated to its usual 3-term accuracy (adequate
    // for RC beam/column aspect ratios; a true St. Venant solve is
    // unnecessary at this precision).
    static Section rectangular(double b, double h, const std::string& name = "") {
        Section s(name);
        s.width = b;
        s.depth = h;
        s.area = b * h;
        s.momentOfInertiaZ = b * h * h * h / 12.0;   // strong axis (bending causing depth to resist)
        s.momentOfInertiaY = h * b * b * b / 12.0;   // weak axis
        s.shearAreaY = (5.0 / 6.0) * s.area;
        s.shearAreaZ = (5.0 / 6.0) * s.area;
        s.torsionalConstant = rectangularTorsionConstant(b, h);
        return s;
    }

    // Solid circular section, diameter d (m).
    static Section circular(double d, const std::string& name = "") {
        Section s(name);
        s.width = s.depth = d;
        double r = d / 2.0;
        s.area = M_PI * r * r;
        s.momentOfInertiaY = s.momentOfInertiaZ = M_PI * d * d * d * d / 64.0;
        s.torsionalConstant = M_PI * d * d * d * d / 32.0;  // polar moment, exact for a solid circle
        s.shearAreaY = s.shearAreaZ = (9.0 / 10.0) * s.area;
        return s;
    }

    // Closed thin-walled rectangular tube — outer plan W x D, uniform
    // wall thickness t (all meters). This is the idealization a lift/
    // stair shear core is modeled as (see design/ShearWall for how a
    // core's plan geometry is turned into W, D, t before calling this).
    // Openings are NOT subtracted; see the docs/ note on core modeling
    // for the same caveat this idealization carries in the browser
    // prototype this engine supersedes.
    static Section hollowRectangularTube(double W, double D, double t,
                                          const std::string& name = "") {
        Section s(name);
        s.width = W;
        s.depth = D;
        s.thickness = t;
        t = std::clamp(t, 0.02, std::min(W, D) / 2.0 - 0.01);
        double Wi = std::max(0.02, W - 2.0 * t);
        double Di = std::max(0.02, D - 2.0 * t);
        s.area = W * D - Wi * Di;
        s.momentOfInertiaZ = (D * W * W * W - Di * Wi * Wi * Wi) / 12.0;
        s.momentOfInertiaY = (W * D * D * D - Wi * Di * Di * Di) / 12.0;
        // Bredt's formula for a closed thin-walled section, evaluated on
        // the wall centerline (median) rectangle rather than the outer
        // or inner face.
        double Wm = W - t, Dm = D - t;
        double Am = Wm * Dm;
        double perimMedian = 2.0 * (Wm + Dm);
        s.torsionalConstant = perimMedian > 0.0
            ? (4.0 * Am * Am * t) / perimMedian
            : s.momentOfInertiaY + s.momentOfInertiaZ;
        s.shearAreaY = s.shearAreaZ = s.area;  // thin tube: full wall area resists shear in either direction
        return s;
    }

    // Flat plate/shell section — thickness only; membrane and
    // plate-bending stiffness are derived from this plus a Material
    // inside ShellElement, not stored here.
    static Section shell(double t, const std::string& name = "") {
        Section s(name);
        s.thickness = t;
        return s;
    }

private:
    std::string name_;

    // Roark's formula for a solid rectangle's torsional constant,
    // b = shorter side, h = longer side:
    //   J = a*b^3*h  (a*b*b*b*h in the b<=h convention below)
    // using the standard series coefficient a(h/b), truncated to the
    // first three terms — within about 0.1% of the exact series for
    // aspect ratios up to 10:1, which covers essentially every RC beam.
    static double rectangularTorsionConstant(double bIn, double hIn) {
        double b = std::min(bIn, hIn);
        double h = std::max(bIn, hIn);
        double ratio = h / b;
        double a = 1.0 / 3.0
            - 0.21 * (1.0 / ratio) * (1.0 - (1.0 / (12.0 * std::pow(ratio, 4))));
        return a * b * b * b * h;
    }
};

}  // namespace nrsa
