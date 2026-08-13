#pragma once

#include "osm2xodr/model.hpp"
#include "osm2xodr/procedural/config.hpp"
#include "osm2xodr/procedural/types.hpp"

namespace osm2xodr::procedural {

// Runs the full OSM-control-line/control-point procedural pipeline (see types.hpp for the stage
// list) and returns the assembled model::MapModel, ready for xodr::write_file. If `out_graph` is
// non-null, the intermediate GeneratedRoadGraph is copied out too (used by tests and --debug-dir).
model::MapModel run_pipeline(const GeneratorConfig& config, GeneratedRoadGraph* out_graph = nullptr);

} // namespace osm2xodr::procedural
