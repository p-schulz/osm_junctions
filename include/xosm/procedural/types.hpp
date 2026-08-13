#pragma once

#include "xosm/geo.hpp"
#include "xosm/model.hpp"
#include "xosm/tags.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// Data model for the OSM-control-line/control-point procedural road-network generator.
//
// Pipeline shape (see pipeline.hpp for the orchestration):
//   osm::parse_osm
//     -> extract_control_lines      (ways -> ControlLine, merged/simplified corridors)
//     -> extract_control_points     (way-graph junctions + projected/endpoint points -> ControlPoint)
//     -> group_control_lines        (spatial/topological grouping -> ControlLineGroup)
//     -> build_graph                (ControlLine x ControlPoint -> Connection edges -> GeneratedRoadGraph)
//     -> configure_lanes            (OSM tags -> model::LanePlan per Connection)
//     -> generate_intersections     (degree>=3 ControlPoints -> model::Junction + connector Connections)
//     -> cleanup_and_validate       (lane-mismatch repair, roadmarks, continuity checks)
//     -> assemble_map_model         (GeneratedRoadGraph -> model::MapModel, for the existing xodr::write_file)
//
// This module never invents control lines/points that aren't grounded in OSM geometry or topology
// (see extract_control_lines/extract_control_points); the only synthetic geometry it ever produces
// is interior junction-connector roads (generate_intersections) and, optionally, lane-count-mismatch
// bridge sections (cleanup_and_validate) -- both clearly marked via Connection::synthetic /
// RoadSegment::junction_id, mirroring how the existing model::MapModel already distinguishes them.
namespace xosm::procedural {

struct ControlLine {
    std::string id;
    std::vector<geo::Vec2> points;                 // simplified centerline, curvature preserved
    std::vector<std::int64_t> source_way_ids;
    std::int64_t start_node_id = 0;                   // OSM node at points.front(), for degree lookup
    std::int64_t end_node_id = 0;                     // OSM node at points.back()
    // Real OSM junction nodes (degree >= 3) that fall *inside* this chain -- e.g. a minor road's
    // endpoint sharing a node with the middle of a merged way, not one of its endpoints. Always
    // survive Douglas-Peucker simplification exactly (see extract_control_lines) so build_graph can
    // split the line here. Vec2 equals the corresponding entry in `points`.
    std::vector<std::pair<std::int64_t, geo::Vec2>> interior_intersections;
    std::string highway;
    std::string name;
    Tags tags;                                       // representative/merged OSM tags
    double confidence = 1.0;                          // 1.0 = directly from OSM geometry/topology
    bool from_minor_class = false;                    // true if not one of config.control_line_highways
};

enum class ControlPointType {
    OsmIntersection,     // real OSM node where road degree != 2, or complex-junction semantics
    ProjectedCrossing,   // a minor road connects to / crosses a control line at a non-OSM-node point
    CorridorConnector,    // endpoint where a control line joins another via topology (rare; degree-2
                          // node kept as a point because two different control lines meet there)
    EndpointConnector,   // truncated/map-boundary terminus of a control line
    Sampled              // inserted along a long corridor because it exceeds max spacing (opt-in)
};

struct ControlPoint {
    std::string id;
    geo::Vec2 point;
    ControlPointType type = ControlPointType::EndpointConnector;
    std::vector<std::int64_t> source_node_ids;
    std::vector<std::int64_t> source_way_ids;
};

enum class ControlLineGroupKind { Component, Crossing, Parallel };

// Replaces JunctionArt's random control-line pairing: groups are derived from real OSM topology
// (shared control points / connected components) or, for Parallel, from geometric proximity between
// corridors that already belong to the same component. Groups are metadata for downstream stages
// (e.g. deciding where a synthetic connector would be legitimate) -- they do not themselves create
// roads.
struct ControlLineGroup {
    std::string id;
    ControlLineGroupKind kind = ControlLineGroupKind::Component;
    std::vector<std::string> control_line_ids;
};

// One edge of the generated graph: either a real OSM-derived corridor segment (control_line_id
// non-empty) or a synthetic connector (synthetic=true, e.g. a junction interior lane or a
// boundary/mismatch repair). Maps directly onto one model::RoadSegment.
struct Connection {
    std::string id;
    std::string from_control_point;
    std::string to_control_point;
    std::string control_line_id;   // empty for synthetic connections
    std::vector<geo::Vec2> geometry;
    std::vector<model::GeomPrimitive> explicit_geometry; // set instead of `geometry` for junction
                                                            // connectors (see intersections.cpp):
                                                            // one ParamPoly3 primitive tracing the
                                                            // interior turn curve exactly.
    Tags tags;
    std::string highway;
    model::LanePlan lanes;
    bool synthetic = false;
    std::string junction_id;       // set for junction-interior connector roads
    std::string diagnostic;        // non-empty: boundary stub / unresolved-crossing note (kept, not fatal)

    // Only set when synthetic=true (a junction-interior connector, see intersections.hpp): which
    // real Connection's road this connector's start/end links to, and which end of that road.
    std::string connector_predecessor_connection_id;
    std::string connector_predecessor_contact; // "start" or "end"
    std::string connector_successor_connection_id;
    std::string connector_successor_contact;
};

struct GeneratedRoadGraph {
    std::vector<ControlLine> control_lines;
    std::vector<ControlPoint> control_points;
    std::vector<ControlLineGroup> groups;
    std::vector<Connection> connections;
    std::vector<model::Junction> junctions;
    std::unordered_map<std::string, std::string> control_point_junction_id; // control point id -> junction id
    std::vector<std::string> diagnostics; // non-fatal pipeline notes (dangling stubs, rejected crossings, ...)
    std::vector<std::string> warnings;    // cleanup/repair warnings (lane mismatches not fully bridgeable, ...)

    std::size_t find_control_point(const std::string& id) const {
        for (std::size_t i = 0; i < control_points.size(); ++i)
            if (control_points[i].id == id) return i;
        return static_cast<std::size_t>(-1);
    }
};

} // namespace xosm::procedural
