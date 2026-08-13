#include "xosm/procedural/cli.hpp"

#include "xosm/util.hpp"

#include <cstdlib>
#include <iostream>

namespace xosm::procedural {

GeneratorConfig parse_args(const int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: xosm <input.osm|osm.pbf> <output.xodr> [options]\n";
        std::cerr << "Try --help for details.\n";
        std::exit(2);
    }

    GeneratorConfig c;
    c.input = argv[1];
    c.output = argv[2];

    auto require_value = [&](int& i, const std::string& opt) -> std::string {
        if (i + 1 >= argc) util::fail("Missing value after " + opt);
        return argv[++i];
    };

    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help") {
            std::cout <<
                "Usage: xosm <input.osm|osm.pbf> <output.xodr> [options]\n\n"
                "Generates an OpenDRIVE road network from OSM-derived control lines/points\n"
                "(see docs/procedural_pipeline.md), reusing the existing OSM parser and XODR writer.\n\n"
                "Options:\n"
                "  --name <name>                    OpenDRIVE header name\n"
                "  --origin-lat <deg> --origin-lon <deg>   Override projection origin\n"
                "  --bbox <minLat> <minLon> <maxLat> <maxLon>  Only use ways within this WGS84 bbox\n"
                "  --highway-classes <a,b,c>         Replace the default control-line highway=* set\n"
                "  --include-highway <value>         Add one highway=* value to the control-line set\n"
                "                                     (repeatable; e.g. --include-highway residential)\n"
                "  --default-lane-width <m>          Default lane width, default 3.5\n"
                "  --snap-distance <m>                Merge control points within this distance, default 3.0\n"
                "  --min-spacing <m>                  Minimum control-point spacing, default 15.0\n"
                "  --max-spacing <m>                  Maximum control-point spacing, default 200.0\n"
                "  --sample-long-corridors            Insert points along corridors exceeding --max-spacing\n"
                "  --simplify-tolerance <m>            Douglas-Peucker tolerance, default 0.5\n"
                "  --projected-crossing-search <m>    Max minor-road-to-control-line search distance, default 12.0\n"
                "  --seed <n>                          Deterministic seed for optional lane-width jitter\n"
                "  --randomize-lane-variation          Enable seeded lane-width jitter (off by default)\n"
                "  --split-strategy <first|last>       Which lane absorbs a junction lane-count mismatch\n"
                "  --junction-setback <m>              Min incident-road trim before a junction, default 6.0\n"
                "  --junction-min-degree <n>           Minimum incident-connection count to form a junction\n"
                "  --left-hand-traffic                 Use left-hand movement classification\n"
                "  --report <file>                     Write conversion report\n"
                "  --validate                          Read generated XODR back with libOpenDRIVE if enabled\n"
                "  --debug-dir <dir>                    Write control_lines/control_points/graph GeoJSON here\n";
            std::exit(0);
        } else if (arg == "--name") {
            c.name = require_value(i, arg);
        } else if (arg == "--origin-lat") {
            c.origin_lat = util::parse_double_prefix(require_value(i, arg));
        } else if (arg == "--origin-lon") {
            c.origin_lon = util::parse_double_prefix(require_value(i, arg));
        } else if (arg == "--bbox") {
            c.bbox_min_lat = util::parse_double_prefix(require_value(i, arg));
            c.bbox_min_lon = util::parse_double_prefix(require_value(i, arg));
            c.bbox_max_lat = util::parse_double_prefix(require_value(i, arg));
            c.bbox_max_lon = util::parse_double_prefix(require_value(i, arg));
        } else if (arg == "--highway-classes") {
            c.control_line_highways.clear();
            for (const auto& v : util::split_any(require_value(i, arg), ",")) c.control_line_highways.insert(v);
        } else if (arg == "--include-highway") {
            c.control_line_highways.insert(require_value(i, arg));
        } else if (arg == "--default-lane-width") {
            c.default_lane_width = util::parse_double_prefix(require_value(i, arg)).value_or(c.default_lane_width);
        } else if (arg == "--snap-distance") {
            c.snap_distance_m = util::parse_double_prefix(require_value(i, arg)).value_or(c.snap_distance_m);
        } else if (arg == "--min-spacing") {
            c.min_control_point_spacing_m = util::parse_double_prefix(require_value(i, arg)).value_or(c.min_control_point_spacing_m);
        } else if (arg == "--max-spacing") {
            c.max_control_point_spacing_m = util::parse_double_prefix(require_value(i, arg)).value_or(c.max_control_point_spacing_m);
        } else if (arg == "--sample-long-corridors") {
            c.sample_long_corridors = true;
        } else if (arg == "--simplify-tolerance") {
            c.simplify_tolerance_m = util::parse_double_prefix(require_value(i, arg)).value_or(c.simplify_tolerance_m);
        } else if (arg == "--projected-crossing-search") {
            c.projected_crossing_search_m = util::parse_double_prefix(require_value(i, arg)).value_or(c.projected_crossing_search_m);
        } else if (arg == "--seed") {
            c.seed = static_cast<std::uint64_t>(util::parse_int(require_value(i, arg)).value_or(0));
        } else if (arg == "--randomize-lane-variation") {
            c.randomize_lane_variation = true;
        } else if (arg == "--split-strategy") {
            const auto v = util::lower(require_value(i, arg));
            if (v == "first") c.split_strategy = SplitStrategy::SplitFirst;
            else if (v == "last") c.split_strategy = SplitStrategy::SplitLast;
            else util::fail("--split-strategy must be 'first' or 'last'");
        } else if (arg == "--junction-setback") {
            c.junction_connector_setback_m = util::parse_double_prefix(require_value(i, arg)).value_or(c.junction_connector_setback_m);
        } else if (arg == "--junction-min-degree") {
            c.junction_min_degree = util::parse_int(require_value(i, arg)).value_or(c.junction_min_degree);
        } else if (arg == "--left-hand-traffic") {
            c.left_hand_traffic = true;
        } else if (arg == "--report") {
            c.report_path = require_value(i, arg);
        } else if (arg == "--validate") {
            c.validate = true;
        } else if (arg == "--debug-dir") {
            c.debug_dir = require_value(i, arg);
        } else {
            util::fail("Unknown option: " + arg);
        }
    }

    if (c.default_lane_width <= 0.1) util::fail("--default-lane-width must be positive");
    if (c.snap_distance_m < 0.0) util::fail("--snap-distance must not be negative");
    if (c.min_control_point_spacing_m < 0.0) util::fail("--min-spacing must not be negative");
    if (c.max_control_point_spacing_m <= c.min_control_point_spacing_m) util::fail("--max-spacing must exceed --min-spacing");
    if (c.simplify_tolerance_m < 0.0) util::fail("--simplify-tolerance must not be negative");
    if (c.junction_min_degree < 2) util::fail("--junction-min-degree must be at least 2");
    return c;
}

} // namespace xosm::procedural
