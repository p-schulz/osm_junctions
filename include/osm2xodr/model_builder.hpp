#pragma once

// STALE -- superseded by the OSM-derived control-line/control-point procedural generator in
// src/procedural/ (see docs/procedural_pipeline.md and osm2xodr::procedural::run_pipeline).
// Kept only so the existing `osm2xodr` CLI keeps working during the deprecation period; do not
// extend. Scheduled for full removal.

#include "osm2xodr/model.hpp"
#include "osm2xodr/options.hpp"
#include "osm2xodr/osm_parse.hpp"

namespace osm2xodr::build {

model::MapModel build_model(const osm::ParseResult& parsed, const Options& options);

} // namespace osm2xodr::build
