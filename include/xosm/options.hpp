#pragma once

#include <optional>
#include <string>
#include <unordered_set>

namespace xosm {

// Input contract shared by the two pieces every generator reuses unchanged: osm::parse_osm (the
// OSM parser) and xodr::write_file (the OpenDRIVE writer). Deliberately minimal -- generation-
// specific configuration (lane widths, snap distances, ...) lives in
// xosm::procedural::GeneratorConfig instead.
struct Options {
    std::string input;
    std::string output;
    std::string name = "xosm-map";
    std::optional<double> origin_lat;
    std::optional<double> origin_lon;
    // OSM highway=* values to exclude entirely from parsing.
    std::unordered_set<std::string> ignore_highways;
};

} // namespace xosm
