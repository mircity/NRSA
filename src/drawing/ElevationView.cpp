#include "drawing/ElevationView.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace nrsa::drawing {

namespace {

// Drops one global coordinate per ViewPlane -- the one shared, tiny
// switch every other function in this file routes through, so the
// "which axis means what" decision lives in exactly one place.
std::array<double, 2> projectPoint(ViewPlane plane, double x, double y, double z) {
    switch (plane) {
        case ViewPlane::ElevationXZ: return {x, z};
        case ViewPlane::ElevationYZ: return {y, z};
        case ViewPlane::PlanXY:      return {x, y};
    }
    return {x, y};  // unreachable, silences -Wreturn-type on some compilers
}

}  // namespace

std::vector<ProjectedNode> projectNodes(const Model& model, ViewPlane plane) {
    std::vector<ProjectedNode> out;
    out.reserve(model.nodes().size());
    for (const auto& n : model.nodes()) {
        auto uv = projectPoint(plane, n.x(), n.y(), n.z());
        out.push_back(ProjectedNode{n.id(), uv[0], uv[1], n.label()});
    }
    return out;
}

std::vector<ProjectedMember> projectElements(const Model& model, ViewPlane plane) {
    std::vector<ProjectedMember> out;
    out.reserve(model.elements().size());
    for (const auto& e : model.elements()) {
        if (e.nodeCount() != 2) {
            // Shell/slab elements (>2 nodes) are out of scope for this
            // phase -- see ElevationView.h header note.
            continue;
        }
        const Node& n1 = model.node(e.nodeId(0));
        const Node& n2 = model.node(e.nodeId(1));
        auto uv1 = projectPoint(plane, n1.x(), n1.y(), n1.z());
        auto uv2 = projectPoint(plane, n2.x(), n2.y(), n2.z());

        ProjectedMember pm;
        pm.elementId = e.id();
        pm.kind = e.kind();
        pm.label = e.label();
        pm.u1 = uv1[0]; pm.v1 = uv1[1];
        pm.u2 = uv2[0]; pm.v2 = uv2[1];
        try {
            const Section& sec = model.section(e.sectionId());
            pm.sectionDepthM = sec.depth;
            pm.sectionName = sec.name();
        } catch (const std::out_of_range&) {
            pm.sectionDepthM = 0.0;
        }
        out.push_back(pm);
    }
    return out;
}

BoundingBoxUV computeBoundingBox(const std::vector<ProjectedNode>& nodes) {
    BoundingBoxUV box;
    if (nodes.empty()) return box;
    box.minU = box.maxU = nodes.front().u;
    box.minV = box.maxV = nodes.front().v;
    for (const auto& n : nodes) {
        box.minU = std::min(box.minU, n.u);
        box.maxU = std::max(box.maxU, n.u);
        box.minV = std::min(box.minV, n.v);
        box.maxV = std::max(box.maxV, n.v);
    }
    return box;
}

namespace {

// meters -> SVG pixel, with V flipped so +up-in-model draws toward the
// top of the image (SVG's native y axis points down).
struct Transform {
    double scale;
    double marginPx;
    double minU, maxV;  // maxV, not minV: V flips, so the model's highest
                         // point becomes the smallest SVG y.

    std::array<double, 2> apply(double u, double v) const {
        double px = marginPx + (u - minU) * scale;
        double py = marginPx + (maxV - v) * scale;
        return {px, py};
    }
};

std::string escapeXml(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            default: out += c;
        }
    }
    return out;
}

}  // namespace

std::string generateElevationSvg(const Model& model, ViewPlane plane,
                                  const ElevationSvgOptions& options) {
    auto nodes = projectNodes(model, plane);
    auto members = projectElements(model, plane);
    BoundingBoxUV box = computeBoundingBox(nodes);

    // Guard against a degenerate (single-node or all-collinear-at-a-point)
    // model, which would otherwise produce a zero-size viewBox.
    double spanU = std::max(box.width(), 0.5);
    double spanV = std::max(box.height(), 0.5);

    Transform tf{options.pixelsPerMeter, options.marginPx, box.minU, box.maxV};
    double widthPx = spanU * options.pixelsPerMeter + 2 * options.marginPx;
    double heightPx = spanV * options.pixelsPerMeter + 2 * options.marginPx;

    std::ostringstream svg;
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 "
        << widthPx << " " << heightPx << "\" width=\"" << widthPx
        << "\" height=\"" << heightPx << "\" font-family=\"sans-serif\">\n";
    svg << "  <rect x=\"0\" y=\"0\" width=\"" << widthPx << "\" height=\"" << heightPx
        << "\" fill=\"white\"/>\n";

    // Members: drawn as a filled rectangle of the section's real depth
    // (see ElevationView.h header note on the depth-vs-width
    // simplification), centered on the projected centerline.
    for (const auto& m : members) {
        auto p1 = tf.apply(m.u1, m.v1);
        auto p2 = tf.apply(m.u2, m.v2);
        double dx = p2[0] - p1[0];
        double dy = p2[1] - p1[1];
        double len = std::sqrt(dx * dx + dy * dy);
        double halfDepthPx = 0.5 * m.sectionDepthM * options.pixelsPerMeter;
        if (halfDepthPx < 1.0) halfDepthPx = 1.0;  // keep zero/unset sections visible

        if (len < 1e-9) {
            continue;  // degenerate (both ends project to the same point)
        }
        // Unit perpendicular to the member axis, in SVG pixel space.
        double nx = -dy / len;
        double ny = dx / len;

        double ax = p1[0] + nx * halfDepthPx, ay = p1[1] + ny * halfDepthPx;
        double bx = p1[0] - nx * halfDepthPx, by = p1[1] - ny * halfDepthPx;
        double cx = p2[0] - nx * halfDepthPx, cy = p2[1] - ny * halfDepthPx;
        double ddx = p2[0] + nx * halfDepthPx, ddy = p2[1] + ny * halfDepthPx;

        const char* fill = (m.kind == ElementKind::Column) ? "#6b7280" : "#9ca3af";
        svg << "  <polygon points=\"" << ax << "," << ay << " " << bx << "," << by << " "
            << cx << "," << cy << " " << ddx << "," << ddy
            << "\" fill=\"" << fill << "\" stroke=\"#111827\" stroke-width=\"1\"/>\n";

        if (options.drawMemberLabels && !m.label.empty()) {
            double midx = (p1[0] + p2[0]) / 2.0;
            double midy = (p1[1] + p2[1]) / 2.0;
            svg << "  <text x=\"" << midx << "\" y=\"" << midy
                << "\" font-size=\"11\" fill=\"#1f2937\" text-anchor=\"middle\">"
                << escapeXml(m.label) << "</text>\n";
        }
    }

    // Nodes: drawn last so joint markers sit on top of member outlines.
    for (const auto& n : nodes) {
        auto p = tf.apply(n.u, n.v);
        svg << "  <circle cx=\"" << p[0] << "\" cy=\"" << p[1] << "\" r=\""
            << options.nodeRadiusPx << "\" fill=\"#111827\"/>\n";
        if (options.drawNodeLabels && !n.label.empty()) {
            svg << "  <text x=\"" << (p[0] + options.nodeRadiusPx + 3) << "\" y=\""
                << (p[1] - options.nodeRadiusPx - 3)
                << "\" font-size=\"10\" fill=\"#374151\">" << escapeXml(n.label)
                << "</text>\n";
        }
    }

    svg << "</svg>\n";
    return svg.str();
}

}  // namespace nrsa::drawing
