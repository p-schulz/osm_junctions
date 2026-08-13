#pragma once

#include "xosm/model.hpp"
#include "xosm/procedural/config.hpp"
#include "xosm/procedural/types.hpp"

#include <string>
#include <vector>

namespace xosm::procedural {

// Repairs lane-count mismatches at plain (non-junction) degree-2 control points where two
// different control lines meet with different total lane counts on the two sides: inserts a short
// synthetic bridge Connection carrying a split/merge LanePlan (config.split_strategy) between them,
// the same writer-supported mechanism (a plain RoadSegment with tapering lane ids) used for
// junction connectors. When a bridge can't be built cleanly (e.g. the gap is degenerate), the
// mismatch is left in place and a warning is recorded on `graph.warnings` instead.
void repair_lane_mismatches(GeneratedRoadGraph& graph, const GeneratorConfig& config);

// Post-assembly sanity pass over the final model::MapModel: every predecessor/successor "road"/
// "junction" reference resolves to a real id, every junction connection's incoming/connecting roads
// exist, and lane ids within a laneSection are unique. Returns human-readable diagnostics; does not
// mutate the model. Independent of (and in addition to) --validate's libOpenDRIVE XML read-back.
std::vector<std::string> validate_map_model(const model::MapModel& model);

} // namespace xosm::procedural
