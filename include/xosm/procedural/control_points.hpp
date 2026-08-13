#pragma once

#include "osm2xodr/osm_parse.hpp"
#include "osm2xodr/procedural/config.hpp"
#include "osm2xodr/procedural/types.hpp"

#include <vector>

namespace osm2xodr::procedural {

// Derives ControlPoints from OSM structure -- never from random selection:
//  - OsmIntersection: every ControlLine terminus whose real OSM node degree (across *all* OSM
//    vehicle road ways, not just corridor-class ones) is >= 3, i.e. a genuine junction.
//  - CorridorConnector: a ControlLine terminus with OSM degree == 2 that didn't get merged into its
//    neighbor (tags incompatible, or the neighbor is a non-control-line-class way) -- corridors
//    that continue but change character there.
//  - EndpointConnector: a ControlLine terminus with OSM degree <= 1 (a true dead end, or a
//    truncated map-extract boundary -- see ControlPoint::source_way_ids/diagnostics upstream in
//    build_graph for which).
//  - ProjectedCrossing: where a non-control-line ("minor") road's endpoint lies within
//    config.projected_crossing_search_m of a control line, or where a minor road's geometry
//    crosses a control line at a non-shared-node point.
// Close points are then snapped/merged (config.snap_distance_m), preferring to keep an
// OsmIntersection's own coordinate and id when a cluster contains one.
std::vector<ControlPoint> extract_control_points(const osm::ParseResult& parsed,
                                                   const std::vector<ControlLine>& control_lines,
                                                   const GeneratorConfig& config);

} // namespace osm2xodr::procedural
