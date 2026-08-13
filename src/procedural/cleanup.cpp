#include "osm2xodr/procedural/cleanup.hpp"

#include "osm2xodr/procedural/geo_extra.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace osm2xodr::procedural {

namespace {

model::LaneSpec make_bridge_lane(const int id, const double width) {
    model::LaneSpec lane;
    lane.id = id;
    lane.type = "driving";
    lane.width = width;
    lane.roadmark_type = "broken";
    lane.roadmark_weight = "standard";
    lane.roadmark_color = "standard";
    lane.lane_change = "both";
    return lane;
}

} // namespace

void repair_lane_mismatches(GeneratedRoadGraph& graph, const GeneratorConfig& config) {
    std::unordered_map<std::string, std::vector<std::size_t>> incident; // control point id -> connection indices
    for (std::size_t i = 0; i < graph.connections.size(); ++i) {
        if (graph.connections[i].synthetic) continue;
        incident[graph.connections[i].from_control_point].push_back(i);
        incident[graph.connections[i].to_control_point].push_back(i);
    }

    int bridge_counter = 0;
    for (const auto& cp : graph.control_points) {
        if (graph.control_point_junction_id.count(cp.id)) continue; // junctions handle their own lanes
        const auto it = incident.find(cp.id);
        if (it == incident.end() || it->second.size() != 2) continue;

        auto& a = graph.connections[it->second[0]];
        auto& b = graph.connections[it->second[1]];
        const bool a_at_end = a.to_control_point == cp.id;
        const bool b_at_end = b.to_control_point == cp.id;

        // "Incoming"/"outgoing" here just mean the two sides of the seam, in a's direction of travel.
        const auto& a_out_lanes = a_at_end ? a.lanes.left : a.lanes.right;   // leaving `a` toward cp
        const auto& b_in_lanes = b_at_end ? b.lanes.right : b.lanes.left;    // entering `b` from cp
        if (a_out_lanes.size() == b_in_lanes.size()) continue; // no mismatch

        if (a_out_lanes.empty() || b_in_lanes.empty()) {
            graph.warnings.push_back("Lane mismatch at " + cp.id + " between " + a.id + " and " + b.id +
                " could not be bridged (one side has no lanes in the connecting direction); left as-is.");
            continue;
        }

        const double setback = 3.0;
        const geo::Vec2 p0 = geo_extra::trim_polyline_end(a.geometry, a_at_end, setback);
        const geo::Vec2 p3 = geo_extra::trim_polyline_end(b.geometry, b_at_end, setback);
        if (geo::length(p3 - p0) <= 0.5) {
            graph.warnings.push_back("Lane mismatch at " + cp.id + " between " + a.id + " and " + b.id +
                " could not be bridged (segments too short for a taper); left as-is.");
            continue;
        }

        const double hdg0 = geo_extra::polyline_heading_near(a.geometry, false, 8.0);
        const double hdg3 = geo_extra::polyline_heading_near(b.geometry, true, 8.0);
        const auto pairs = geo_extra::pair_lane_slots(static_cast<int>(a_out_lanes.size()),
                                                        static_cast<int>(b_in_lanes.size()),
                                                        config.split_strategy == SplitStrategy::SplitFirst);

        model::LanePlan plan;
        plan.oneway = true;
        plan.forward_lanes = static_cast<int>(pairs.size());
        plan.backward_lanes = 0;
        plan.center_mark = "none";
        for (std::size_t k = 0; k < pairs.size(); ++k) {
            const auto& in_lane = a_out_lanes[pairs[k].first];
            const auto& out_lane = b_in_lanes[pairs[k].second];
            model::LaneSpec lane = make_bridge_lane(-static_cast<int>(k + 1), in_lane.width);
            lane.width_end = out_lane.width;
            lane.link_predecessor_id = in_lane.id;
            lane.link_successor_id = out_lane.id;
            plan.right.push_back(lane);
        }
        plan.lane_offset = model::compute_lane_offset(plan);

        Connection bridge;
        bridge.id = "bridge_" + std::to_string(++bridge_counter);
        bridge.synthetic = true;
        bridge.tags = a.tags;
        bridge.highway = a.highway;
        bridge.lanes = plan;
        bridge.geometry = {p0, p3};
        bridge.explicit_geometry = {geo_extra::hermite_bezier_geometry(p0, hdg0, p3, hdg3)};
        bridge.connector_predecessor_connection_id = a.id;
        bridge.connector_predecessor_contact = a_at_end ? "end" : "start";
        bridge.connector_successor_connection_id = b.id;
        bridge.connector_successor_contact = b_at_end ? "end" : "start";
        graph.connections.push_back(std::move(bridge));

        graph.warnings.push_back("Inserted a lane-count bridge at " + cp.id + " between " + a.id + " (" +
            std::to_string(a_out_lanes.size()) + " lane(s)) and " + b.id + " (" + std::to_string(b_in_lanes.size()) +
            " lane(s)).");
    }
}

std::vector<std::string> validate_map_model(const model::MapModel& model) {
    std::vector<std::string> issues;
    std::unordered_set<std::string> road_ids;
    std::unordered_set<std::string> junction_ids;
    for (const auto& r : model.roads) road_ids.insert(r.id);
    for (const auto& j : model.junctions) junction_ids.insert(j.id);

    const auto check_link = [&](const std::string& road_id, const std::string& xml, const char* which) {
        if (xml.empty()) return;
        const bool is_junction = xml.find("elementType=\"junction\"") != std::string::npos;
        const auto start = xml.find("elementId=\"");
        if (start == std::string::npos) return;
        const auto value_start = start + 11;
        const auto end = xml.find('"', value_start);
        const std::string id = xml.substr(value_start, end - value_start);
        const bool exists = is_junction ? junction_ids.count(id) != 0 : road_ids.count(id) != 0;
        if (!exists) {
            std::ostringstream msg;
            msg << "Road " << road_id << "'s " << which << " references unknown "
                << (is_junction ? "junction " : "road ") << id << ".";
            issues.push_back(msg.str());
        }
    };

    for (const auto& r : model.roads) {
        check_link(r.id, r.predecessor_xml, "predecessor");
        check_link(r.id, r.successor_xml, "successor");

        std::unordered_set<int> ids;
        for (const auto& l : r.lanes.left)
            if (!ids.insert(l.id).second) issues.push_back("Road " + r.id + " has a duplicate lane id " + std::to_string(l.id) + ".");
        for (const auto& l : r.lanes.right)
            if (!ids.insert(l.id).second) issues.push_back("Road " + r.id + " has a duplicate lane id " + std::to_string(l.id) + ".");
    }

    for (const auto& j : model.junctions) {
        for (const auto& c : j.connections) {
            if (!road_ids.count(c.incoming_road)) issues.push_back("Junction " + j.id + " connection " + c.id + " references unknown incoming road " + c.incoming_road + ".");
            if (!road_ids.count(c.connecting_road)) issues.push_back("Junction " + j.id + " connection " + c.id + " references unknown connecting road " + c.connecting_road + ".");
        }
    }

    return issues;
}

} // namespace osm2xodr::procedural
