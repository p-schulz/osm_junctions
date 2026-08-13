#include "osm2xodr/procedural/lanes.hpp"

#include "osm2xodr/util.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace osm2xodr::procedural {

int default_lane_count_for_highway(const std::string& highway) {
    static const std::unordered_map<std::string, int> table = {
        {"motorway", 2}, {"motorway_link", 1}, {"trunk", 2}, {"trunk_link", 1},
        {"primary", 2}, {"primary_link", 1}, {"secondary", 1}, {"secondary_link", 1},
        {"tertiary", 1}, {"tertiary_link", 1}, {"unclassified", 1}, {"residential", 1},
        {"living_street", 1}, {"service", 1}, {"road", 1}, {"busway", 1}, {"construction", 1},
    };
    const auto it = table.find(highway);
    return it == table.end() ? 1 : it->second;
}

bool default_oneway_for_highway(const std::string& highway) {
    return highway == "motorway" || highway == "motorway_link" || highway == "trunk_link" ||
           highway == "primary_link" || highway == "secondary_link" || highway == "tertiary_link";
}

namespace {

// Small deterministic hash of a Connection id, used only to seed lane-width jitter when
// config.randomize_lane_variation is explicitly enabled (see lanes.hpp) -- never affects lane
// counts, oneway-ness, or turn restrictions.
std::uint64_t hash_id(const std::string& s, const std::uint64_t seed) {
    std::uint64_t h = 1469598103934665603ULL ^ seed;
    for (const char c : s) {
        h ^= static_cast<unsigned char>(c);
        h *= 1099511628211ULL;
    }
    return h;
}

std::vector<std::vector<std::string>> parse_turn_lanes(const std::string& raw) {
    std::vector<std::vector<std::string>> out;
    for (const auto& slot : util::split_any(raw, "|")) {
        std::vector<std::string> dirs;
        if (slot.empty() || slot == "none") { out.push_back({}); continue; }
        for (const auto& d : util::split_any(slot, ";")) dirs.push_back(d);
        out.push_back(dirs);
    }
    return out;
}

model::LaneSpec make_lane(const int id, const double width, const bool outermost) {
    model::LaneSpec lane;
    lane.id = id;
    lane.type = "driving";
    lane.width = width;
    lane.roadmark_type = outermost ? "solid" : "broken";
    lane.roadmark_weight = "standard";
    lane.roadmark_color = "standard";
    lane.lane_change = outermost ? "none" : "both";
    return lane;
}

void configure_one(Connection& conn, const GeneratorConfig& config) {
    const Tags& tags = conn.tags;
    const std::string highway = conn.highway.empty() ? tag_value_or(tags, "highway", "residential") : conn.highway;

    model::LanePlan plan;
    const auto oneway_tag = tag_value(tags, "oneway");
    plan.oneway = oneway_tag ? util::truthy_osm(*oneway_tag) : default_oneway_for_highway(highway);
    if (oneway_tag && util::falsy_osm(*oneway_tag)) plan.oneway = false;

    const auto lanes_total = util::parse_int(tag_value_or(tags, "lanes", ""));
    const auto lanes_fwd = util::parse_int(tag_value_or(tags, "lanes:forward", ""));
    const auto lanes_bwd = util::parse_int(tag_value_or(tags, "lanes:backward", ""));

    if (plan.oneway) {
        plan.forward_lanes = lanes_fwd.value_or(lanes_total.value_or(default_lane_count_for_highway(highway)));
        plan.backward_lanes = 0;
    } else if (lanes_fwd || lanes_bwd) {
        plan.forward_lanes = lanes_fwd.value_or(std::max(1, lanes_total.value_or(2) - lanes_bwd.value_or(1)));
        plan.backward_lanes = lanes_bwd.value_or(std::max(1, lanes_total.value_or(2) - plan.forward_lanes));
    } else if (lanes_total) {
        plan.forward_lanes = (*lanes_total + 1) / 2; // deterministic: extra lane (if odd) goes forward
        plan.backward_lanes = *lanes_total - plan.forward_lanes;
    } else {
        plan.forward_lanes = default_lane_count_for_highway(highway);
        plan.backward_lanes = default_lane_count_for_highway(highway);
    }
    plan.forward_lanes = std::max(0, plan.forward_lanes);
    plan.backward_lanes = std::max(0, plan.backward_lanes);
    if (plan.forward_lanes == 0 && plan.backward_lanes == 0) plan.forward_lanes = 1;

    double width = config.default_lane_width;
    if (config.randomize_lane_variation) {
        const std::uint64_t h = hash_id(conn.id, config.seed);
        const double jitter = (static_cast<double>(h % 21) - 10.0) / 100.0; // +/-0.10 m, seeded
        width = std::max(config.default_lane_width + jitter, 2.5);
    }

    // forward direction -> right side (negative ids, direction of increasing s), matching the
    // model.hpp / xodr_writer.cpp convention already used by the existing OpenDRIVE writer.
    for (int i = 1; i <= plan.forward_lanes; ++i) plan.right.push_back(make_lane(-i, width, i == plan.forward_lanes));
    for (int i = 1; i <= plan.backward_lanes; ++i) plan.left.push_back(make_lane(i, width, i == plan.backward_lanes));

    const auto turn_fwd_raw = tag_value(tags, plan.oneway ? "turn:lanes" : "turn:lanes:forward");
    if (turn_fwd_raw) {
        const auto decoded = parse_turn_lanes(*turn_fwd_raw);
        if (decoded.size() == plan.right.size()) {
            for (std::size_t i = 0; i < decoded.size(); ++i) plan.right[i].turn_directions = decoded[i];
        }
    }
    if (!plan.oneway) {
        const auto turn_bwd_raw = tag_value(tags, "turn:lanes:backward");
        if (turn_bwd_raw) {
            const auto decoded = parse_turn_lanes(*turn_bwd_raw);
            if (decoded.size() == plan.left.size()) {
                for (std::size_t i = 0; i < decoded.size(); ++i) plan.left[i].turn_directions = decoded[i];
            }
        }
    }

    plan.center_mark = plan.oneway ? "none" : "broken";
    plan.lane_offset = model::compute_lane_offset(plan);
    conn.lanes = plan;
}

} // namespace

void configure_lanes(GeneratedRoadGraph& graph, const GeneratorConfig& config) {
    for (auto& conn : graph.connections) {
        if (conn.synthetic) continue; // junction connectors get their lanes from intersections.cpp
        configure_one(conn, config);
    }
}

} // namespace osm2xodr::procedural
