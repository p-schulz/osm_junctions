#include "osm2xodr/procedural/control_points.hpp"

#include "osm2xodr/procedural/geo_extra.hpp"
#include "way_graph.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <unordered_set>

namespace osm2xodr::procedural {

namespace {

int priority(const ControlPointType t) {
    switch (t) {
        case ControlPointType::OsmIntersection: return 4;
        case ControlPointType::ProjectedCrossing: return 3;
        case ControlPointType::CorridorConnector: return 2;
        case ControlPointType::EndpointConnector: return 2;
        case ControlPointType::Sampled: return 1;
    }
    return 0;
}

// Union-find over the candidate points, merging any pair closer than snap_distance_m. Kept
// deliberately O(n^2); extraction runs once per generation and candidate counts are small relative
// to a city-block-scale extract.
std::vector<std::size_t> snap_clusters(const std::vector<geo::Vec2>& pts, const double snap_distance_m) {
    std::vector<std::size_t> parent(pts.size());
    std::iota(parent.begin(), parent.end(), 0);
    std::function<std::size_t(std::size_t)> find = [&](std::size_t x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    };
    for (std::size_t i = 0; i < pts.size(); ++i) {
        for (std::size_t j = i + 1; j < pts.size(); ++j) {
            if (geo::length(pts[i] - pts[j]) <= snap_distance_m) {
                const auto ri = find(i), rj = find(j);
                if (ri != rj) parent[ri] = rj;
            }
        }
    }
    for (std::size_t i = 0; i < pts.size(); ++i) parent[i] = find(i);
    return parent;
}

} // namespace

std::vector<ControlPoint> extract_control_points(const osm::ParseResult& parsed,
                                                   const std::vector<ControlLine>& control_lines,
                                                   const GeneratorConfig& config) {
    std::vector<const osm::RawWay*> all_road_ways;
    for (const auto& way : parsed.roads) {
        const auto hw = tag_value(way.tags, "highway");
        if (hw && osm::is_vehicle_highway_value(*hw)) all_road_ways.push_back(&way);
    }
    const WayGraph graph = build_way_graph(all_road_ways);

    struct Candidate {
        geo::Vec2 point;
        ControlPointType type;
        std::vector<std::int64_t> node_ids;
        std::vector<std::int64_t> way_ids;
    };
    std::vector<Candidate> candidates;
    std::unordered_set<std::int64_t> seen_nodes; // avoid duplicate candidates for the same OSM node

    const auto add_node_candidate = [&](const std::int64_t node, const geo::Vec2& p,
                                         const std::vector<std::int64_t>& way_ids) {
        if (!seen_nodes.insert(node).second) return;
        const int deg = graph.degree(node);
        const ControlPointType type = deg >= 3   ? ControlPointType::OsmIntersection
                                       : deg == 2 ? ControlPointType::CorridorConnector
                                                  : ControlPointType::EndpointConnector;
        candidates.push_back({p, type, {node}, way_ids});
    };

    // 1) Control-line termini (always control points -- every edge of the graph needs endpoints).
    for (const auto& line : control_lines) {
        add_node_candidate(line.start_node_id, line.points.front(), {line.source_way_ids.front()});
        add_node_candidate(line.end_node_id, line.points.back(), {line.source_way_ids.back()});
        // 1b) Real OSM junctions embedded inside the chain (see extract_control_lines).
        for (const auto& [node, p] : line.interior_intersections) add_node_candidate(node, p, line.source_way_ids);
    }

    // 2) Minor-road endpoints near a control line (endpoint touches OSM didn't already resolve
    // above because the minor way isn't part of `control_lines` at all).
    for (const auto* way : all_road_ways) {
        if (highway_is_control_line_class(way->tags, config)) continue; // corridor way, handled above
        for (const bool at_start : {true, false}) {
            const auto& wn = at_start ? way->nodes.front() : way->nodes.back();
            if (seen_nodes.count(wn.ref)) continue;
            const geo::Vec2 p = parsed.projector.project(wn.ll.lat, wn.ll.lon);
            double best_dist = config.projected_crossing_search_m;
            geo::Vec2 best_point{};
            bool found = false;
            for (const auto& line : control_lines) {
                const auto proj = geo::project_to_polyline(line.points, p);
                if (proj.distance < best_dist) {
                    best_dist = proj.distance;
                    best_point = proj.nearest;
                    found = true;
                }
            }
            if (found) {
                seen_nodes.insert(wn.ref);
                candidates.push_back({best_point, ControlPointType::ProjectedCrossing, {wn.ref}, {way->id}});
            }
        }
    }

    // 3) Minor-road geometry crossing a control line at a non-shared-node point.
    for (const auto* way : all_road_ways) {
        if (highway_is_control_line_class(way->tags, config)) continue;
        for (std::size_t i = 1; i < way->nodes.size(); ++i) {
            const geo::Vec2 a = parsed.projector.project(way->nodes[i - 1].ll.lat, way->nodes[i - 1].ll.lon);
            const geo::Vec2 b = parsed.projector.project(way->nodes[i].ll.lat, way->nodes[i].ll.lon);
            for (const auto& line : control_lines) {
                for (std::size_t j = 1; j < line.points.size(); ++j) {
                    geo::Vec2 hit{};
                    if (geo_extra::segments_cross(a, b, line.points[j - 1], line.points[j], &hit)) {
                        candidates.push_back({hit, ControlPointType::ProjectedCrossing, {}, {way->id}});
                    }
                }
            }
        }
    }

    if (candidates.empty()) return {};

    // 4) Snap/merge candidates within snap_distance_m; keep the highest-priority member's type
    // (OsmIntersection wins) and that member's own coordinate rather than a blended centroid, so a
    // real intersection's position is never nudged by a nearby lower-confidence candidate.
    std::vector<geo::Vec2> pts;
    pts.reserve(candidates.size());
    for (const auto& c : candidates) pts.push_back(c.point);
    const auto cluster_of = snap_clusters(pts, config.snap_distance_m);

    std::unordered_map<std::size_t, std::size_t> best_in_cluster; // cluster root -> candidate index
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        const auto root = cluster_of[i];
        const auto it = best_in_cluster.find(root);
        if (it == best_in_cluster.end() || priority(candidates[i].type) > priority(candidates[it->second].type)) {
            best_in_cluster[root] = i;
        }
    }

    std::unordered_map<std::size_t, ControlPoint> merged; // cluster root -> merged point
    int counter = 0;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        const auto root = cluster_of[i];
        auto& cp = merged[root];
        if (cp.id.empty()) {
            const auto& rep = candidates[best_in_cluster[root]];
            cp.id = "cp_" + std::to_string(++counter);
            cp.point = rep.point;
            cp.type = rep.type;
        }
        for (const auto n : candidates[i].node_ids) cp.source_node_ids.push_back(n);
        for (const auto w : candidates[i].way_ids) cp.source_way_ids.push_back(w);
    }

    std::vector<ControlPoint> result;
    result.reserve(merged.size());
    for (auto& [root, cp] : merged) result.push_back(std::move(cp));

    // 5) Spacing: a point that's a terminus of *any* control line is structural and always kept
    // (dropping it would disconnect the graph); only interior points (ProjectedCrossing/Sampled)
    // are candidates for min-spacing pruning or max-spacing insertion.
    std::unordered_set<std::size_t> is_terminus;
    for (const auto& line : control_lines) {
        double best_start = config.snap_distance_m, best_end = config.snap_distance_m;
        std::optional<std::size_t> start_idx, end_idx;
        for (std::size_t i = 0; i < result.size(); ++i) {
            const double ds = geo::length(result[i].point - line.points.front());
            const double de = geo::length(result[i].point - line.points.back());
            if (ds < best_start) { best_start = ds; start_idx = i; }
            if (de < best_end) { best_end = de; end_idx = i; }
        }
        if (start_idx) is_terminus.insert(*start_idx);
        if (end_idx) is_terminus.insert(*end_idx);
    }

    std::unordered_set<std::size_t> to_remove;
    std::vector<ControlPoint> sampled_additions;
    for (const auto& line : control_lines) {
        struct OnLine { std::size_t idx; double s; };
        std::vector<OnLine> on_line;
        for (std::size_t i = 0; i < result.size(); ++i) {
            const auto proj = geo::project_to_polyline(line.points, result[i].point);
            if (proj.distance <= config.snap_distance_m + 0.5) on_line.push_back({i, proj.s});
        }
        std::sort(on_line.begin(), on_line.end(), [](const OnLine& a, const OnLine& b) { return a.s < b.s; });

        for (std::size_t k = 1; k < on_line.size(); ++k) {
            if (to_remove.count(on_line[k - 1].idx) || to_remove.count(on_line[k].idx)) continue;
            const double gap = on_line[k].s - on_line[k - 1].s;
            if (gap >= config.min_control_point_spacing_m) continue;
            const bool a_term = is_terminus.count(on_line[k - 1].idx) != 0;
            const bool b_term = is_terminus.count(on_line[k].idx) != 0;
            if (a_term && b_term) continue; // both structural; nothing safe to drop
            const auto drop_idx = (!a_term && (b_term || priority(result[on_line[k - 1].idx].type) <=
                                                              priority(result[on_line[k].idx].type)))
                                       ? on_line[k - 1].idx
                                       : on_line[k].idx;
            if (!is_terminus.count(drop_idx)) to_remove.insert(drop_idx);
        }

        if (config.sample_long_corridors) {
            for (std::size_t k = 1; k < on_line.size(); ++k) {
                const double gap = on_line[k].s - on_line[k - 1].s;
                if (gap <= config.max_control_point_spacing_m) continue;
                const int n_inserts = static_cast<int>(std::ceil(gap / config.max_control_point_spacing_m)) - 1;
                for (int s_i = 1; s_i <= n_inserts; ++s_i) {
                    const double target_s = on_line[k - 1].s + gap * s_i / (n_inserts + 1);
                    double acc = 0.0;
                    geo::Vec2 p = line.points.back();
                    for (std::size_t seg = 1; seg < line.points.size(); ++seg) {
                        const double seg_len = geo::length(line.points[seg] - line.points[seg - 1]);
                        if (acc + seg_len >= target_s || seg + 1 == line.points.size()) {
                            const double u = seg_len > 1e-9 ? (target_s - acc) / seg_len : 0.0;
                            p = line.points[seg - 1] + (line.points[seg] - line.points[seg - 1]) * std::clamp(u, 0.0, 1.0);
                            break;
                        }
                        acc += seg_len;
                    }
                    ControlPoint cp;
                    cp.id = "cp_" + std::to_string(++counter);
                    cp.point = p;
                    cp.type = ControlPointType::Sampled;
                    cp.source_way_ids = line.source_way_ids;
                    sampled_additions.push_back(std::move(cp));
                }
            }
        }
    }

    std::vector<ControlPoint> final_result;
    final_result.reserve(result.size() + sampled_additions.size());
    for (std::size_t i = 0; i < result.size(); ++i)
        if (!to_remove.count(i)) final_result.push_back(std::move(result[i]));
    for (auto& cp : sampled_additions) final_result.push_back(std::move(cp));
    return final_result;
}

} // namespace osm2xodr::procedural
