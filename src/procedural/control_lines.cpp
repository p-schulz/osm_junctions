#include "xosm/procedural/control_lines.hpp"

#include "xosm/procedural/geo_extra.hpp"
#include "way_graph.hpp"

#include <algorithm>
#include <unordered_set>

namespace xosm::procedural {

bool control_lines_compatible(const Tags& a, const Tags& b) {
    if (tag_value_or(a, "highway", "") != tag_value_or(b, "highway", "")) return false;
    if (tag_value_or(a, "name", "") != tag_value_or(b, "name", "")) return false;
    if (tag_value_or(a, "oneway", "") != tag_value_or(b, "oneway", "")) return false;
    if (tag_value_or(a, "lanes", "") != tag_value_or(b, "lanes", "")) return false;
    if (tag_value_or(a, "lanes:forward", "") != tag_value_or(b, "lanes:forward", "")) return false;
    if (tag_value_or(a, "lanes:backward", "") != tag_value_or(b, "lanes:backward", "")) return false;
    return true;
}

namespace {

bool within_bbox(const geo::LonLat& ll, const GeneratorConfig& config) {
    if (!config.bbox_min_lat) return true;
    return ll.lat >= *config.bbox_min_lat && ll.lat <= *config.bbox_max_lat &&
           ll.lon >= *config.bbox_min_lon && ll.lon <= *config.bbox_max_lon;
}

// Appends `way`'s node refs/points to `chain_nodes`/`chain_points` in the given direction, skipping
// the first node when it duplicates the chain's current last node (the shared junction node).
void append_way(const osm::RawWay& way, const bool reversed, const geo::LocalProjector& proj,
                 std::vector<std::int64_t>& chain_nodes, std::vector<geo::Vec2>& chain_points) {
    const auto append_node = [&](const osm::WayNode& n) {
        if (!chain_nodes.empty() && chain_nodes.back() == n.ref) return;
        chain_nodes.push_back(n.ref);
        chain_points.push_back(proj.project(n.ll.lat, n.ll.lon));
    };
    if (!reversed) {
        for (const auto& n : way.nodes) append_node(n);
    } else {
        for (auto it = way.nodes.rbegin(); it != way.nodes.rend(); ++it) append_node(*it);
    }
}

} // namespace

std::vector<ControlLine> extract_control_lines(const osm::ParseResult& parsed, const GeneratorConfig& config) {
    std::vector<const osm::RawWay*> all_road_ways;
    for (const auto& way : parsed.roads) {
        const auto hw = tag_value(way.tags, "highway");
        if (hw && osm::is_vehicle_highway_value(*hw)) all_road_ways.push_back(&way);
    }
    const WayGraph graph = build_way_graph(all_road_ways);

    std::vector<const osm::RawWay*> corridor_ways;
    for (const auto* way : all_road_ways) {
        if (highway_is_control_line_class(way->tags, config) &&
            within_bbox(way->nodes.front().ll, config) && within_bbox(way->nodes.back().ll, config)) {
            corridor_ways.push_back(way);
        }
    }

    // way pointer -> index within corridor_ways, and reverse lookup for chain-walking via way_graph's
    // endpoint_of (which indexes into `all_road_ways`, not `corridor_ways`).
    std::unordered_map<const osm::RawWay*, std::size_t> all_index_of;
    for (std::size_t i = 0; i < all_road_ways.size(); ++i) all_index_of[all_road_ways[i]] = i;

    std::unordered_set<const osm::RawWay*> used;
    std::vector<ControlLine> lines;

    // Given the chain's current terminal node, look for exactly one *other*, unused corridor way
    // that also has this node as an endpoint, with OSM degree exactly 2 here (a plain pass-through,
    // not a real junction) and tag-compatible with `seed_tags`. Returns nullptr if none qualifies.
    const auto find_continuation = [&](const std::int64_t node, const osm::RawWay* exclude,
                                        const Tags& seed_tags) -> const osm::RawWay* {
        if (graph.degree(node) != 2) return nullptr;
        const auto it = graph.endpoint_of.find(node);
        if (it == graph.endpoint_of.end()) return nullptr;
        const osm::RawWay* found = nullptr;
        for (const auto idx : it->second) {
            const osm::RawWay* candidate = all_road_ways[idx];
            if (candidate == exclude || used.count(candidate)) continue;
            if (!highway_is_control_line_class(candidate->tags, config)) continue;
            if (!control_lines_compatible(seed_tags, candidate->tags)) continue;
            if (found) return nullptr; // ambiguous (shouldn't happen at degree==2, but stay safe)
            found = candidate;
        }
        return found;
    };

    int chain_counter = 0;
    for (const auto* seed : corridor_ways) {
        if (used.count(seed)) continue;
        used.insert(seed);

        std::vector<std::int64_t> chain_nodes;
        std::vector<geo::Vec2> chain_points;
        for (const auto& n : seed->nodes) {
            chain_nodes.push_back(n.ref);
            chain_points.push_back(parsed.projector.project(n.ll.lat, n.ll.lon));
        }
        std::vector<std::int64_t> source_ids{seed->id};

        // Extend forward (from the chain's current back node).
        while (true) {
            const osm::RawWay* next = find_continuation(chain_nodes.back(), nullptr, seed->tags);
            if (!next) break;
            used.insert(next);
            const bool reversed = next->nodes.back().ref == chain_nodes.back();
            append_way(*next, reversed, parsed.projector, chain_nodes, chain_points);
            source_ids.push_back(next->id);
        }
        // Extend backward (from the chain's current front node) by reversing, extending, reversing back.
        while (true) {
            const osm::RawWay* prev = find_continuation(chain_nodes.front(), nullptr, seed->tags);
            if (!prev) break;
            used.insert(prev);
            std::reverse(chain_nodes.begin(), chain_nodes.end());
            std::reverse(chain_points.begin(), chain_points.end());
            const bool reversed = prev->nodes.back().ref == chain_nodes.back();
            append_way(*prev, reversed, parsed.projector, chain_nodes, chain_points);
            std::reverse(chain_nodes.begin(), chain_nodes.end());
            std::reverse(chain_points.begin(), chain_points.end());
            source_ids.insert(source_ids.begin(), prev->id);
        }

        // A node interior to this chain can still be a real OSM junction: e.g. a minor road's
        // endpoint sharing a node with the *middle* of one of this chain's constituent ways, which
        // never shows up as a chain terminus. Force those to survive simplification exactly.
        std::vector<std::size_t> force_keep;
        for (std::size_t i = 1; i + 1 < chain_nodes.size(); ++i)
            if (graph.degree(chain_nodes[i]) >= 3) force_keep.push_back(i);

        ControlLine line;
        line.id = "cl_" + std::to_string(++chain_counter);
        line.points = geo_extra::douglas_peucker(chain_points, config.simplify_tolerance_m, force_keep);
        if (line.points.size() < 2) line.points = {chain_points.front(), chain_points.back()};
        line.source_way_ids = source_ids;
        line.start_node_id = chain_nodes.front();
        line.end_node_id = chain_nodes.back();
        for (const auto i : force_keep) line.interior_intersections.emplace_back(chain_nodes[i], chain_points[i]);
        line.highway = tag_value_or(seed->tags, "highway", "");
        line.name = tag_value_or(seed->tags, "name", "");
        line.tags = seed->tags;
        line.confidence = 1.0;
        line.from_minor_class = false;
        lines.push_back(std::move(line));
    }

    return lines;
}

} // namespace xosm::procedural
