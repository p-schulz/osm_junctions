#pragma once

#include "osm2xodr/osm_parse.hpp"
#include "osm2xodr/procedural/config.hpp"
#include "osm2xodr/procedural/types.hpp"

#include <vector>

namespace osm2xodr::procedural {

// Extracts ControlLines from OSM road geometry: selects ways whose highway=* value is in
// `config.control_line_highways`, merges adjacent ways into continuous corridors wherever the
// shared node is a plain topological pass-through (OSM degree exactly 2, see way_graph.hpp) and
// their tags are compatible (same highway class, name, oneway, and lane counts), then simplifies
// each merged corridor's polyline with Douglas-Peucker (config.simplify_tolerance_m) while always
// preserving the corridor's own endpoints. No randomness and no node/segment selection outside
// what OSM topology and tags already determine.
std::vector<ControlLine> extract_control_lines(const osm::ParseResult& parsed, const GeneratorConfig& config);

// True if two ways sharing a plain pass-through node (OSM degree 2) may be merged into a single
// ControlLine: same highway value, same name (or both unnamed), same oneway-ness, and equal
// lanes/lanes:forward/lanes:backward tags (missing on both counts as equal) -- merging across a
// real lane-count or directionality change would silently misrepresent the corridor.
bool control_lines_compatible(const Tags& a, const Tags& b);

} // namespace osm2xodr::procedural
