#pragma once

// Internal helper shared by control_lines.cpp and control_points.cpp: a node-level adjacency graph
// built from OSM ways, used to compute real OSM topological degree (the basis for both merge
// decisions and control-point classification). Not part of the public procedural/ API.

#include "xosm/osm_parse.hpp"
#include "xosm/procedural/config.hpp"

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace xosm::procedural {

struct WayGraph {
    std::vector<const osm::RawWay*> ways; // all road ways considered (highway=* vehicle classes only)
    std::unordered_map<std::int64_t, std::unordered_set<std::int64_t>> neighbors; // node -> adjacent node ids
    std::unordered_map<std::int64_t, std::vector<std::size_t>> endpoint_of;       // node -> indices into `ways`
                                                                                    // for which it's the first/last ref

    int degree(const std::int64_t node) const {
        const auto it = neighbors.find(node);
        return it == neighbors.end() ? 0 : static_cast<int>(it->second.size());
    }
};

// `ways` should already be filtered to OSM ways with a recognized highway=* vehicle-road tag
// (railways excluded); see build_context in control_lines.cpp for the actual selection.
inline WayGraph build_way_graph(std::vector<const osm::RawWay*> ways) {
    WayGraph g;
    g.ways = std::move(ways);
    for (std::size_t wi = 0; wi < g.ways.size(); ++wi) {
        const auto& way = *g.ways[wi];
        if (way.nodes.size() < 2) continue;
        for (std::size_t i = 1; i < way.nodes.size(); ++i) {
            const auto a = way.nodes[i - 1].ref;
            const auto b = way.nodes[i].ref;
            if (a == b) continue;
            g.neighbors[a].insert(b);
            g.neighbors[b].insert(a);
        }
        g.endpoint_of[way.nodes.front().ref].push_back(wi);
        if (way.nodes.back().ref != way.nodes.front().ref) g.endpoint_of[way.nodes.back().ref].push_back(wi);
    }
    return g;
}

inline bool highway_is_control_line_class(const Tags& tags, const GeneratorConfig& config) {
    const auto hw = tag_value(tags, "highway");
    return hw && config.control_line_highways.count(*hw) != 0;
}

} // namespace xosm::procedural
