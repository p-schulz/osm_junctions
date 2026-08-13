#include "osm2xodr/infer.hpp"

// xodr_writer.cpp (reused unchanged -- see xodr_writer.hpp) hard-depends on infer::road_type for
// the OpenDRIVE <type> element. The rest of infer.hpp/infer.cpp is the legacy pipeline's tag-
// inference module (STALE -- see the banners atop model_builder.cpp/infer.cpp) and is deliberately
// not linked into the procedural target, so this provides an independent, minimal definition of
// just that one symbol: a pure, non-heuristic highway=* -> OpenDRIVE road-type classification, not
// a generation strategy.
namespace osm2xodr::infer {

std::string road_type(const Tags& tags) {
    const auto highway = tag_value_or(tags, "highway", "road");
    if (highway == "motorway" || highway == "motorway_link") return "motorway";
    if (highway == "trunk" || highway == "primary" || highway == "secondary" || highway == "tertiary" ||
        highway == "trunk_link" || highway == "primary_link" || highway == "secondary_link" || highway == "tertiary_link")
        return "rural";
    return "town";
}

} // namespace osm2xodr::infer
