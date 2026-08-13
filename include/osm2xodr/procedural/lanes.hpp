#pragma once

#include "osm2xodr/procedural/config.hpp"
#include "osm2xodr/procedural/types.hpp"

namespace osm2xodr::procedural {

// Fills in `model::LanePlan` for every non-synthetic Connection, from OSM tags first
// (lanes/lanes:forward/lanes:backward/oneway/turn:lanes), falling back to deterministic
// per-highway-class defaults only where a tag is missing. Never assigns lanes randomly unless
// config.randomize_lane_variation is set, in which case only lane *width* gets a small
// config.seed-seeded jitter (lane counts/oneway-ness/turn restrictions are always tag- or
// default-driven, never randomized).
void configure_lanes(GeneratedRoadGraph& graph, const GeneratorConfig& config);

// Deterministic per-direction lane-count default for a highway=* class when no lanes/lanes:forward/
// lanes:backward tag is present. `oneway` affects only whether it's meaningful as a single total.
int default_lane_count_for_highway(const std::string& highway);

// True when a highway=* class conventionally defaults to oneway (motorway and *_link classes) if
// OSM supplies no explicit oneway=* tag.
bool default_oneway_for_highway(const std::string& highway);

} // namespace osm2xodr::procedural
