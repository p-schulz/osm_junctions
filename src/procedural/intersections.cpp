#include "xosm/procedural/intersections.hpp"

#include "xosm/procedural/geo_extra.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace xosm::procedural {

namespace {

constexpr double kHeadingSampleM = 8.0;
constexpr double kThroughAngleDeg = 25.0;
constexpr double kUTurnAngleDeg = 150.0;

double into_junction_heading(const std::vector<geo::Vec2>& pts, const bool at_end) {
    if (at_end) return geo_extra::polyline_heading_near(pts, false, kHeadingSampleM);
    return geo::norm_angle(geo_extra::polyline_heading_near(pts, true, kHeadingSampleM) + geo::kPi);
}

enum class Bucket { Through, Left, Right };

Bucket classify(const double entry_hdg, const double exit_hdg, const bool left_hand_traffic) {
    // entry_hdg points into the junction; the movement continues along exit_hdg. The turn angle is
    // measured from "straight ahead" (entry_hdg extended) to exit_hdg.
    double turn = geo::norm_angle(exit_hdg - entry_hdg);
    if (left_hand_traffic) turn = -turn;
    if (std::abs(turn) <= geo::deg_to_rad(kThroughAngleDeg)) return Bucket::Through;
    return turn > 0.0 ? Bucket::Left : Bucket::Right;
}

bool lane_allows(const std::vector<std::string>& turn_directions, const Bucket bucket) {
    if (turn_directions.empty()) return true; // unrestricted
    for (const auto& d : turn_directions) {
        if (bucket == Bucket::Through && (d == "through" || d == "none" || d.find("straight") != std::string::npos))
            return true;
        if (bucket == Bucket::Left && d.find("left") != std::string::npos) return true;
        if (bucket == Bucket::Right && d.find("right") != std::string::npos) return true;
    }
    return false;
}

struct Leg {
    std::size_t connection_index = 0;
    bool at_end = false;
    geo::Vec2 entry_point{};
    double entry_hdg = 0.0; // direction of travel arriving at the junction
    double exit_hdg = 0.0;  // direction of travel departing the junction
    std::vector<model::LaneSpec> incoming_lanes;  // center-outward
    std::vector<model::LaneSpec> outgoing_lanes;  // center-outward
    std::string predecessor_contact_for_connector; // contact point of THIS leg's road, from the connector's view
};

} // namespace

void generate_intersections(GeneratedRoadGraph& graph, const GeneratorConfig& config) {
    std::unordered_map<std::string, int> degree;
    for (const auto& c : graph.connections) {
        ++degree[c.from_control_point];
        ++degree[c.to_control_point];
    }

    int junction_counter = 0;
    int connector_counter = 0;

    for (const auto& cp : graph.control_points) {
        const int d = degree.count(cp.id) ? degree[cp.id] : 0;
        if (d < config.junction_min_degree) continue;

        std::vector<Leg> legs;
        for (std::size_t ci = 0; ci < graph.connections.size(); ++ci) {
            auto& conn = graph.connections[ci];
            if (conn.synthetic) continue;
            bool at_end;
            if (conn.to_control_point == cp.id) at_end = true;
            else if (conn.from_control_point == cp.id) at_end = false;
            else continue;

            const double entry_hdg_before_trim = into_junction_heading(conn.geometry, at_end);
            const double half_width = model::sum_side_width(conn.lanes.right, at_end) / 2.0 +
                                       model::sum_side_width(conn.lanes.left, at_end) / 2.0;
            const double setback = std::max(config.junction_connector_setback_m, half_width + 1.0);
            const geo::Vec2 trimmed = geo_extra::trim_polyline_end(conn.geometry, at_end, setback);

            Leg leg;
            leg.connection_index = ci;
            leg.at_end = at_end;
            leg.entry_point = trimmed;
            leg.entry_hdg = entry_hdg_before_trim;
            leg.exit_hdg = geo::norm_angle(entry_hdg_before_trim + geo::kPi);
            leg.incoming_lanes = at_end ? conn.lanes.right : conn.lanes.left;
            leg.outgoing_lanes = at_end ? conn.lanes.left : conn.lanes.right;
            leg.predecessor_contact_for_connector = at_end ? "end" : "start";
            legs.push_back(std::move(leg));
        }
        if (legs.size() < 2) continue;

        model::Junction junction;
        junction.id = "j_" + std::to_string(++junction_counter);
        junction.node_ref = cp.source_node_ids.empty() ? 0 : cp.source_node_ids.front();
        junction.point = cp.point;
        graph.control_point_junction_id[cp.id] = junction.id;

        std::vector<std::vector<bool>> incoming_used(legs.size());
        for (std::size_t li = 0; li < legs.size(); ++li) incoming_used[li].assign(legs[li].incoming_lanes.size(), false);

        const auto build_movement = [&](const std::size_t in_i, const std::size_t out_i,
                                         const std::vector<int>& eligible_in_indices) {
            auto& in_leg = legs[in_i];
            auto& out_leg = legs[out_i];
            if (eligible_in_indices.empty() || out_leg.outgoing_lanes.empty()) return;

            const auto pairs = geo_extra::pair_lane_slots(static_cast<int>(eligible_in_indices.size()),
                                                            static_cast<int>(out_leg.outgoing_lanes.size()),
                                                            config.split_strategy == SplitStrategy::SplitFirst);

            const std::string connector_id = junction.id + "_c" + std::to_string(++connector_counter);

            model::LanePlan plan;
            plan.oneway = true;
            plan.forward_lanes = static_cast<int>(pairs.size());
            plan.backward_lanes = 0;
            plan.center_mark = "none";

            model::JunctionConnection jc;
            jc.id = junction.id + "_conn" + std::to_string(connector_counter);
            jc.incoming_road = graph.connections[in_leg.connection_index].id;
            jc.connecting_road = connector_id;
            jc.contact_point = "start";

            for (std::size_t k = 0; k < pairs.size(); ++k) {
                const int in_idx = eligible_in_indices[pairs[k].first];
                const int out_idx = pairs[k].second;
                incoming_used[in_i][in_idx] = true;

                model::LaneSpec lane;
                lane.id = -static_cast<int>(k + 1);
                lane.type = "driving";
                lane.width = in_leg.incoming_lanes[in_idx].width;
                lane.roadmark_type = "none";
                lane.lane_change = "none";
                lane.link_predecessor_id = in_leg.incoming_lanes[in_idx].id;
                lane.link_successor_id = out_leg.outgoing_lanes[out_idx].id;
                plan.right.push_back(lane);
                jc.lane_links.emplace_back(in_leg.incoming_lanes[in_idx].id, lane.id);
            }
            plan.lane_offset = model::compute_lane_offset(plan);

            const auto geom = geo_extra::hermite_bezier_geometry(in_leg.entry_point, in_leg.entry_hdg,
                                                                   out_leg.entry_point, geo::norm_angle(out_leg.exit_hdg));

            const std::string predecessor_conn_id = graph.connections[in_leg.connection_index].id;
            const std::string successor_conn_id = graph.connections[out_leg.connection_index].id;

            Connection connector;
            connector.id = connector_id;
            connector.synthetic = true;
            connector.junction_id = junction.id;
            connector.tags = graph.connections[in_leg.connection_index].tags;
            connector.highway = graph.connections[in_leg.connection_index].highway;
            connector.lanes = plan;
            connector.geometry = {in_leg.entry_point, out_leg.entry_point};
            connector.explicit_geometry = {geom};
            connector.connector_predecessor_connection_id = predecessor_conn_id;
            connector.connector_predecessor_contact = in_leg.predecessor_contact_for_connector;
            connector.connector_successor_connection_id = successor_conn_id;
            connector.connector_successor_contact = out_leg.predecessor_contact_for_connector;
            graph.connections.push_back(std::move(connector));

            junction.connections.push_back(std::move(jc));
        };

        for (std::size_t i = 0; i < legs.size(); ++i) {
            if (legs[i].incoming_lanes.empty()) continue;
            for (std::size_t j = 0; j < legs.size(); ++j) {
                if (i == j || legs[j].outgoing_lanes.empty()) continue;
                const double turn = std::abs(geo::norm_angle(legs[j].exit_hdg - legs[i].entry_hdg));
                if (turn >= geo::deg_to_rad(kUTurnAngleDeg)) continue; // U-turn: skip by default
                const Bucket bucket = classify(legs[i].entry_hdg, legs[j].exit_hdg, config.left_hand_traffic);
                std::vector<int> eligible;
                for (int li = 0; li < static_cast<int>(legs[i].incoming_lanes.size()); ++li)
                    if (lane_allows(legs[i].incoming_lanes[li].turn_directions, bucket)) eligible.push_back(li);
                build_movement(i, j, eligible);
            }
        }

        // Fallback: any incoming lane with no movement at all gets connected to the best-effort
        // available leg (through-preferred by smallest turn angle) so no lane dead-ends silently.
        for (std::size_t i = 0; i < legs.size(); ++i) {
            for (std::size_t li = 0; li < legs[i].incoming_lanes.size(); ++li) {
                if (incoming_used[i][li]) continue;
                std::size_t best_j = legs.size();
                double best_turn = 1e9;
                for (std::size_t j = 0; j < legs.size(); ++j) {
                    if (i == j || legs[j].outgoing_lanes.empty()) continue;
                    const double turn = std::abs(geo::norm_angle(legs[j].exit_hdg - legs[i].entry_hdg));
                    if (turn >= geo::deg_to_rad(kUTurnAngleDeg)) continue;
                    if (turn < best_turn) { best_turn = turn; best_j = j; }
                }
                if (best_j == legs.size()) {
                    graph.warnings.push_back("Junction " + junction.id + ": incoming lane " +
                        std::to_string(legs[i].incoming_lanes[li].id) + " on " +
                        graph.connections[legs[i].connection_index].id +
                        " has no available outgoing movement (all candidates were U-turns); left unconnected.");
                    continue;
                }
                graph.warnings.push_back("Junction " + junction.id + ": incoming lane " +
                    std::to_string(legs[i].incoming_lanes[li].id) + " on " +
                    graph.connections[legs[i].connection_index].id +
                    " had no turn:lanes-eligible movement; connected to the nearest-angle leg anyway to avoid a dead lane.");
                build_movement(i, best_j, {static_cast<int>(li)});
            }
        }

        graph.junctions.push_back(std::move(junction));
    }
}

} // namespace xosm::procedural
