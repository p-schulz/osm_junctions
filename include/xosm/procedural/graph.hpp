#pragma once

#include "xosm/procedural/config.hpp"
#include "xosm/procedural/types.hpp"

namespace xosm::procedural {

// Builds the base GeneratedRoadGraph: nodes are `control_points`, edges are the sub-segments of
// each control line between consecutive control points that lie on it (in s-order). Every edge maps
// 1:1 onto a future model::RoadSegment. No lane configuration or junction/intersection structure is
// added yet (see lanes.hpp / intersections.hpp) -- this stage only establishes connectivity:
//  - No duplicate/overlapping edges are created (each control-line sub-segment is emitted once).
//  - A control point touched by exactly one connection is a dangling boundary stub; it's kept (its
//    Connection's `diagnostic` field explains why) rather than silently dropped or joined at random.
//  - Two connections whose geometry crosses without sharing a control point is flagged in
//    `diagnostics` (grade-separated via differing OSM `layer` tags is noted and accepted; an
//    at-grade crossing is reported as a diagnostic, not silently resolved into a junction, since
//    real OSM data should already have a shared node there).
GeneratedRoadGraph build_graph(std::vector<ControlLine> control_lines, std::vector<ControlPoint> control_points,
                                std::vector<ControlLineGroup> groups, const GeneratorConfig& config);

// Degree (incident connection count) of a control point within `graph`.
int control_point_degree(const GeneratedRoadGraph& graph, const std::string& control_point_id);

} // namespace xosm::procedural
