#include "xosm/procedural/grouping.hpp"

#include "xosm/geo.hpp"
#include "xosm/procedural/geo_extra.hpp"

#include <algorithm>
#include <numeric>
#include <unordered_map>

namespace xosm::procedural {

namespace {

// Which control points lie on which control lines, by proximity (mirrors control_points.cpp's own
// spacing pass, recomputed here to keep grouping decoupled from extraction).
std::vector<std::vector<std::size_t>> lines_touching_point(const std::vector<ControlLine>& lines,
                                                             const std::vector<ControlPoint>& points,
                                                             const double tolerance_m) {
    std::vector<std::vector<std::size_t>> touching(points.size());
    for (std::size_t pi = 0; pi < points.size(); ++pi) {
        for (std::size_t li = 0; li < lines.size(); ++li) {
            const auto proj = geo::project_to_polyline(lines[li].points, points[pi].point);
            if (proj.distance <= tolerance_m) touching[pi].push_back(li);
        }
    }
    return touching;
}

struct UnionFind {
    std::vector<std::size_t> parent;
    explicit UnionFind(const std::size_t n) : parent(n) { std::iota(parent.begin(), parent.end(), 0); }
    std::size_t find(std::size_t x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    }
    void unite(const std::size_t a, const std::size_t b) {
        const auto ra = find(a), rb = find(b);
        if (ra != rb) parent[ra] = rb;
    }
};

} // namespace

std::vector<ControlLineGroup> group_control_lines(const std::vector<ControlLine>& control_lines,
                                                    const std::vector<ControlPoint>& control_points,
                                                    const GeneratorConfig& config) {
    std::vector<ControlLineGroup> groups;
    if (control_lines.empty()) return groups;

    const double tol = config.snap_distance_m + 1.0;
    const auto touching = lines_touching_point(control_lines, control_points, tol);

    // Component grouping.
    UnionFind uf(control_lines.size());
    for (const auto& lines_here : touching)
        for (std::size_t i = 1; i < lines_here.size(); ++i) uf.unite(lines_here[0], lines_here[i]);

    std::unordered_map<std::size_t, std::vector<std::string>> components;
    for (std::size_t li = 0; li < control_lines.size(); ++li) components[uf.find(li)].push_back(control_lines[li].id);
    int comp_counter = 0;
    for (auto& [root, ids] : components) {
        ControlLineGroup g;
        g.id = "group_component_" + std::to_string(++comp_counter);
        g.kind = ControlLineGroupKind::Component;
        g.control_line_ids = std::move(ids);
        groups.push_back(std::move(g));
    }

    // Crossing grouping: one group per control point that touches >= 2 control lines.
    int crossing_counter = 0;
    for (std::size_t pi = 0; pi < control_points.size(); ++pi) {
        if (touching[pi].size() < 2) continue;
        if (control_points[pi].type != ControlPointType::OsmIntersection &&
            control_points[pi].type != ControlPointType::ProjectedCrossing)
            continue;
        ControlLineGroup g;
        g.id = "group_crossing_" + std::to_string(++crossing_counter);
        g.kind = ControlLineGroupKind::Crossing;
        for (const auto li : touching[pi]) g.control_line_ids.push_back(control_lines[li].id);
        groups.push_back(std::move(g));
    }

    // Parallel grouping (metadata only): same-component corridor pairs with similar heading and a
    // small, roughly constant perpendicular offset over an overlapping span.
    int parallel_counter = 0;
    for (std::size_t i = 0; i < control_lines.size(); ++i) {
        for (std::size_t j = i + 1; j < control_lines.size(); ++j) {
            if (uf.find(i) != uf.find(j)) continue; // Parallel is only meaningful within a component
            const auto& a = control_lines[i];
            const auto& b = control_lines[j];
            const double hdg_a = geo_extra::polyline_heading_near(a.points, true, 1e9);
            const double hdg_b = geo_extra::polyline_heading_near(b.points, true, 1e9);
            double dh = std::abs(geo::norm_angle(hdg_a - hdg_b));
            if (dh > geo::kPi / 2.0) dh = geo::kPi - dh; // ignore direction (anti-parallel still counts)
            if (dh > geo::deg_to_rad(20.0)) continue;

            const auto pa = geo::project_to_polyline(a.points, b.points.front());
            const auto pb = geo::project_to_polyline(a.points, b.points.back());
            const double perp = (pa.distance + pb.distance) / 2.0;
            if (perp < 4.0 || perp > 60.0) continue; // too close (same corridor) or too far to matter

            ControlLineGroup g;
            g.id = "group_parallel_" + std::to_string(++parallel_counter);
            g.kind = ControlLineGroupKind::Parallel;
            g.control_line_ids = {a.id, b.id};
            groups.push_back(std::move(g));
        }
    }

    return groups;
}

} // namespace xosm::procedural
