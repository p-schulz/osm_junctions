#include "xosm/procedural/graph.hpp"

#include "xosm/procedural/geo_extra.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace xosm::procedural {

namespace {

// Extracts the sub-polyline of `points` covering arclength [s_from, s_to], with exact interpolated
// endpoints (control points rarely land exactly on an existing vertex, e.g. a projected crossing).
std::vector<geo::Vec2> sub_polyline(const std::vector<geo::Vec2>& points, const double s_from, const double s_to) {
    std::vector<geo::Vec2> out;
    double acc = 0.0;
    const auto point_at = [&](const double target_s, const std::size_t seg_start) {
        (void)seg_start;
        double a2 = 0.0;
        for (std::size_t i = 1; i < points.size(); ++i) {
            const double seg_len = geo::length(points[i] - points[i - 1]);
            if (a2 + seg_len >= target_s - 1e-9 || i + 1 == points.size()) {
                const double u = seg_len > 1e-9 ? std::clamp((target_s - a2) / seg_len, 0.0, 1.0) : 0.0;
                return points[i - 1] + (points[i] - points[i - 1]) * u;
            }
            a2 += seg_len;
        }
        return points.back();
    };

    out.push_back(point_at(s_from, 0));
    for (std::size_t i = 1; i < points.size(); ++i) {
        const double seg_len = geo::length(points[i] - points[i - 1]);
        const double next_acc = acc + seg_len;
        if (acc > s_from + 1e-6 && acc < s_to - 1e-6) out.push_back(points[i - 1]);
        acc = next_acc;
    }
    out.push_back(point_at(s_to, 0));
    // Drop consecutive duplicates that can appear when s_from/s_to land exactly on a vertex.
    std::vector<geo::Vec2> dedup;
    for (const auto& p : out) {
        if (dedup.empty() || geo::length(p - dedup.back()) > 1e-6) dedup.push_back(p);
    }
    return dedup;
}

struct OnLine {
    std::size_t point_index;
    double s;
};

} // namespace

int control_point_degree(const GeneratedRoadGraph& graph, const std::string& control_point_id) {
    int degree = 0;
    for (const auto& c : graph.connections)
        if (c.from_control_point == control_point_id || c.to_control_point == control_point_id) ++degree;
    return degree;
}

GeneratedRoadGraph build_graph(std::vector<ControlLine> control_lines, std::vector<ControlPoint> control_points,
                                std::vector<ControlLineGroup> groups, const GeneratorConfig& config) {
    GeneratedRoadGraph graph;
    graph.control_lines = control_lines;
    graph.control_points = control_points;
    graph.groups = std::move(groups);

    const double tol = config.snap_distance_m + 1.0;
    int connection_counter = 0;
    std::unordered_set<std::string> seen_pairs; // "line_id|lo_cp|hi_cp" -- duplicate-edge guard

    for (const auto& line : graph.control_lines) {
        std::vector<OnLine> on_line;
        for (std::size_t pi = 0; pi < graph.control_points.size(); ++pi) {
            const auto proj = geo::project_to_polyline(line.points, graph.control_points[pi].point);
            if (proj.distance <= tol) on_line.push_back({pi, proj.s});
        }
        if (on_line.size() < 2) {
            graph.diagnostics.push_back("Control line " + line.id +
                " has fewer than two associated control points and was not connected into the graph.");
            continue;
        }
        std::sort(on_line.begin(), on_line.end(), [](const OnLine& a, const OnLine& b) { return a.s < b.s; });
        // De-duplicate points that snapped to (nearly) the same s (can happen at a merge-chain seam).
        std::vector<OnLine> dedup;
        for (const auto& ol : on_line) {
            if (!dedup.empty() && std::abs(ol.s - dedup.back().s) < 0.5 &&
                dedup.back().point_index == ol.point_index)
                continue;
            dedup.push_back(ol);
        }

        for (std::size_t k = 1; k < dedup.size(); ++k) {
            const auto& from_cp = graph.control_points[dedup[k - 1].point_index];
            const auto& to_cp = graph.control_points[dedup[k].point_index];
            if (from_cp.id == to_cp.id) continue;
            const double seg_len = dedup[k].s - dedup[k - 1].s;
            if (seg_len <= 1e-3) continue;

            std::ostringstream key;
            key << line.id << '|' << std::min(from_cp.id, to_cp.id) << '|' << std::max(from_cp.id, to_cp.id);
            if (!seen_pairs.insert(key.str()).second) {
                graph.diagnostics.push_back("Skipped a duplicate/overlapping edge on control line " + line.id +
                    " between " + from_cp.id + " and " + to_cp.id + ".");
                continue;
            }

            Connection conn;
            conn.id = line.id + "_seg" + std::to_string(++connection_counter);
            conn.from_control_point = from_cp.id;
            conn.to_control_point = to_cp.id;
            conn.control_line_id = line.id;
            conn.geometry = sub_polyline(line.points, dedup[k - 1].s, dedup[k].s);
            conn.tags = line.tags;
            conn.highway = line.highway;
            conn.synthetic = false;
            graph.connections.push_back(std::move(conn));
        }
    }

    // Dangling-stub diagnostics: any control point touched by exactly one connection.
    std::unordered_map<std::string, int> degree;
    for (const auto& c : graph.connections) {
        ++degree[c.from_control_point];
        ++degree[c.to_control_point];
    }
    for (const auto& cp : graph.control_points) {
        const int d = degree.count(cp.id) ? degree[cp.id] : 0;
        if (d == 0) {
            graph.diagnostics.push_back("Control point " + cp.id + " has no incident connections and was dropped from the graph.");
            continue;
        }
        if (d == 1 && cp.type == ControlPointType::EndpointConnector) {
            bool near_boundary = false;
            if (config.bbox_min_lat) {
                // A crude "close to the configured extract bbox" check would need lon/lat; the
                // pipeline stays in projected meters here, so this is left as a general dead-end
                // note -- see docs/procedural_pipeline.md for how to interpret boundary stubs.
                near_boundary = true;
            }
            const std::string reason = near_boundary
                ? "possible map-extract boundary (bbox configured) -- retained as a valid dead-end road stub"
                : "OSM road end with no junction (dead end, driveway, or unmapped continuation) -- retained as a valid dead-end road stub";
            graph.diagnostics.push_back("Control point " + cp.id + " is a dangling boundary stub: " + reason + ".");
            for (auto& c : graph.connections) {
                if (c.from_control_point == cp.id || c.to_control_point == cp.id) c.diagnostic = reason;
            }
        }
    }

    // Geometry-intersection check: connections whose geometry crosses without sharing a control
    // point endpoint. Grade separation (differing OSM `layer`) is accepted silently; anything else
    // is reported, not auto-repaired.
    for (std::size_t i = 0; i < graph.connections.size(); ++i) {
        const auto& a = graph.connections[i];
        for (std::size_t j = i + 1; j < graph.connections.size(); ++j) {
            const auto& b = graph.connections[j];
            const bool shares_endpoint = a.from_control_point == b.from_control_point ||
                                          a.from_control_point == b.to_control_point ||
                                          a.to_control_point == b.from_control_point ||
                                          a.to_control_point == b.to_control_point;
            if (shares_endpoint) continue;
            for (std::size_t si = 1; si < a.geometry.size() && !shares_endpoint; ++si) {
                for (std::size_t sj = 1; sj < b.geometry.size(); ++sj) {
                    geo::Vec2 hit{};
                    if (!geo_extra::segments_cross(a.geometry[si - 1], a.geometry[si], b.geometry[sj - 1],
                                                    b.geometry[sj], &hit))
                        continue;
                    const auto layer_a = tag_value(a.tags, "layer");
                    const auto layer_b = tag_value(b.tags, "layer");
                    if (layer_a && layer_b && *layer_a != *layer_b) continue; // grade separated, fine
                    std::ostringstream msg;
                    msg << "Unresolved at-grade geometry crossing between " << a.id << " and " << b.id
                        << " near (" << hit.x << ", " << hit.y
                        << "); expected a shared OSM node there -- check source data or add layer tags"
                           " if grade-separated.";
                    graph.diagnostics.push_back(msg.str());
                }
            }
        }
    }

    return graph;
}

} // namespace xosm::procedural
