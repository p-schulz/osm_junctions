#pragma once

#include "osm2xodr/geo.hpp"
#include "osm2xodr/procedural/types.hpp"

#include <string>

namespace osm2xodr::procedural {

// Writes control_lines.geojson, control_points.geojson, and graph.geojson (LineString/Point
// FeatureCollections, WGS84 lon/lat) into `dir` for inspecting a run's intermediate pipeline state.
// Only called when config.debug_dir is non-empty (see pipeline.cpp).
void write_debug_geojson(const GeneratedRoadGraph& graph, const geo::LocalProjector& projector, const std::string& dir);

} // namespace osm2xodr::procedural
