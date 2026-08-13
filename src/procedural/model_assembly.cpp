#include "osm2xodr/procedural/model_assembly.hpp"

#include "osm2xodr/procedural/geo_extra.hpp"
#include "osm2xodr/util.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace osm2xodr::procedural {

namespace {

std::string link_xml(const std::string& element_type, const std::string& element_id, const std::string& contact_point) {
    std::ostringstream ss;
    ss << util::attr("elementType", element_type) << util::attr("elementId", element_id);
    if (!contact_point.empty()) ss << util::attr("contactPoint", contact_point);
    return ss.str();
}

geo::Vec2 bezier_point(const model::GeomPrimitive& g, const double t) {
    const geo::Vec2 p0{0.0, 0.0};
    const geo::Vec2& p1 = g.local_p1;
    const geo::Vec2& p2 = g.local_p2;
    const geo::Vec2& p3 = g.local_p3;
    const double u = 1.0 - t;
    const geo::Vec2 local = p0 * (u * u * u) + p1 * (3 * u * u * t) + p2 * (3 * u * t * t) + p3 * (t * t * t);
    const double c = std::cos(g.hdg), s = std::sin(g.hdg);
    return geo::Vec2{g.x + local.x * c - local.y * s, g.y + local.x * s + local.y * c};
}

struct EndpointRef {
    std::size_t connection_index;
    bool at_end;
};

} // namespace

model::MapModel assemble_map_model(const GeneratedRoadGraph& graph, const osm::ParseResult& parsed,
                                    const GeneratorConfig& config) {
    (void)config;
    model::MapModel model;
    model.projector = parsed.projector;
    model.junctions = graph.junctions;

    std::unordered_map<std::string, std::size_t> connection_index_by_id;
    for (std::size_t i = 0; i < graph.connections.size(); ++i) connection_index_by_id[graph.connections[i].id] = i;

    std::unordered_map<std::string, std::string> control_line_first_way; // control_line id -> "123"
    for (const auto& cl : graph.control_lines)
        if (!cl.source_way_ids.empty()) control_line_first_way[cl.id] = std::to_string(cl.source_way_ids.front());

    // Regular (non-synthetic) connections incident to each control point, for direct road-to-road
    // chaining at a plain (non-junction, non-bridged) degree-2 point.
    std::unordered_map<std::string, std::vector<EndpointRef>> incident;
    for (std::size_t i = 0; i < graph.connections.size(); ++i) {
        const auto& c = graph.connections[i];
        if (c.synthetic) continue;
        incident[c.from_control_point].push_back({i, false});
        incident[c.to_control_point].push_back({i, true});
    }

    // Reverse index: which synthetic (bridge) connection attaches to a given plain connection's end.
    // Junction connectors are excluded here -- a plain road touching a junction always links to the
    // junction id itself (the standard OpenDRIVE convention), not to any one of its connectors.
    struct BridgeRef { std::string bridge_id; bool as_predecessor; };
    std::unordered_map<std::string, BridgeRef> bridge_attachment; // key: "<connection_id>|<contact>"
    for (const auto& c : graph.connections) {
        if (!c.synthetic || !c.junction_id.empty()) continue;
        if (!c.connector_predecessor_connection_id.empty())
            bridge_attachment[c.connector_predecessor_connection_id + "|" + c.connector_predecessor_contact] = {c.id, true};
        if (!c.connector_successor_connection_id.empty())
            bridge_attachment[c.connector_successor_connection_id + "|" + c.connector_successor_contact] = {c.id, false};
    }

    const auto resolve_link = [&](const Connection& conn, const bool at_end) -> std::string {
        const std::string& cp_id = at_end ? conn.to_control_point : conn.from_control_point;
        const std::string contact = at_end ? "end" : "start";

        const auto jit = graph.control_point_junction_id.find(cp_id);
        if (jit != graph.control_point_junction_id.end()) return link_xml("junction", jit->second, "");

        const auto bit = bridge_attachment.find(conn.id + "|" + contact);
        if (bit != bridge_attachment.end()) return link_xml("road", bit->second.bridge_id, bit->second.as_predecessor ? "start" : "end");

        const auto it = incident.find(cp_id);
        if (it == incident.end()) return "";
        for (const auto& ref : it->second) {
            if (graph.connections[ref.connection_index].id == conn.id) continue;
            return link_xml("road", graph.connections[ref.connection_index].id, ref.at_end ? "end" : "start");
        }
        return ""; // dangling stub (degree 1) -- no link, matches conn.diagnostic
    };

    double north = -std::numeric_limits<double>::max();
    double south = std::numeric_limits<double>::max();
    double east = -std::numeric_limits<double>::max();
    double west = std::numeric_limits<double>::max();
    const auto expand = [&](const geo::Vec2& p) {
        north = std::max(north, p.y);
        south = std::min(south, p.y);
        east = std::max(east, p.x);
        west = std::min(west, p.x);
    };

    for (const auto& conn : graph.connections) {
        model::RoadSegment road;
        road.id = conn.id;
        road.tags = conn.tags;
        road.lanes = conn.lanes;
        road.junction_id = conn.junction_id;
        if (const auto it = control_line_first_way.find(conn.control_line_id); it != control_line_first_way.end()) {
            road.source_way_id = std::stoll(it->second);
        }

        if (!conn.explicit_geometry.empty()) {
            road.explicit_geometry = conn.explicit_geometry;
            road.length = 0.0;
            for (const auto& g : road.explicit_geometry) road.length += g.length;
            for (const auto& g : road.explicit_geometry) {
                expand({g.x, g.y});
                for (const double t : {0.25, 0.5, 0.75, 1.0}) expand(bezier_point(g, t));
            }
            road.points = {conn.geometry.front(), conn.geometry.back()};
        } else {
            road.points = conn.geometry;
            road.length = geo::polyline_length(conn.geometry);
            for (const auto& p : conn.geometry) expand(p);
        }

        if (conn.synthetic) {
            road.predecessor_xml = link_xml("road", conn.connector_predecessor_connection_id, conn.connector_predecessor_contact);
            road.successor_xml = link_xml("road", conn.connector_successor_connection_id, conn.connector_successor_contact);
        } else {
            road.predecessor_xml = resolve_link(conn, false);
            road.successor_xml = resolve_link(conn, true);
        }

        model.roads.push_back(std::move(road));
    }

    model.north = north;
    model.south = south;
    model.east = east;
    model.west = west;

    // graph.diagnostics (dangling stubs, rejected crossings, ...) are reported separately by the
    // procedural CLI's own report (see main.cpp's write_report) to avoid double-listing them here.
    model.warnings = parsed.warnings;
    model.warnings.insert(model.warnings.end(), graph.warnings.begin(), graph.warnings.end());

    return model;
}

} // namespace osm2xodr::procedural
