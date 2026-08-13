#pragma once

#include "osm2xodr/model.hpp"
#include "osm2xodr/osm_parse.hpp"
#include "osm2xodr/procedural/config.hpp"
#include "osm2xodr/procedural/types.hpp"

namespace osm2xodr::procedural {

// Converts the finished GeneratedRoadGraph into a model::MapModel -- the existing OpenDRIVE
// writer's own input contract (see xodr_writer.hpp) -- so xodr::write_file can be reused unchanged.
// Every Connection becomes one model::RoadSegment; a regular connection's predecessor/successor
// link to whichever real element (another road, a junction, or a synthetic connector/bridge road)
// touches it at that end, or is left empty for a dangling boundary stub (see build_graph).
model::MapModel assemble_map_model(const GeneratedRoadGraph& graph, const osm::ParseResult& parsed,
                                    const GeneratorConfig& config);

} // namespace osm2xodr::procedural
