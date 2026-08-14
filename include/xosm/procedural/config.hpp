#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace xosm::procedural {

enum class SplitStrategy { SplitFirst, SplitLast };

// Configuration for the OSM-control-line/control-point procedural generator (see
// docs/procedural_pipeline.md and pipeline.hpp for the stage-by-stage pipeline this drives).
// Every default here is deterministic; nothing in the pipeline consults a random source unless
// `randomize_lane_variation` is explicitly turned on, and then only seeded by `seed`.
struct GeneratorConfig {
    std::string input;
    std::string output;
    std::string name = "xosm";
    std::optional<double> origin_lat;
    std::optional<double> origin_lon;

    // Optional bounding-box filter (WGS84 degrees) applied to OSM ways before control-line
    // extraction; nullopt means "use everything the parser loaded".
    std::optional<double> bbox_min_lat;
    std::optional<double> bbox_min_lon;
    std::optional<double> bbox_max_lat;
    std::optional<double> bbox_max_lon;

    // highway=* values eligible as control-line material. Defaults to the major-corridor classes;
    // --include-highway adds residential/unclassified/service/etc for local connectivity or test
    // coverage without changing the default behavior for ordinary city extracts.
    std::unordered_set<std::string> control_line_highways = {
        "motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link",
        "secondary", "secondary_link", "tertiary", "tertiary_link"
    };

    double default_lane_width = 3.5;

    double snap_distance_m = 3.0;               // control points closer than this merge into one
    double min_control_point_spacing_m = 15.0;  // non-OSM-intersection points closer than this merge
    double max_control_point_spacing_m = 200.0; // corridors longer than this may get sampled points
    bool sample_long_corridors = false;         // only insert spacing-driven points when explicitly asked

    double simplify_tolerance_m = 0.5; // Douglas-Peucker epsilon for control-line polylines

    double projected_crossing_search_m = 12.0; // max distance from a minor-road endpoint to a
                                                // control line before it's treated as unrelated

    std::uint64_t seed = 0;
    bool randomize_lane_variation = false; // if true, lane width gets a small seeded jitter

    SplitStrategy split_strategy = SplitStrategy::SplitLast;

    double junction_connector_setback_m = 6.0; // min distance an incident road is trimmed back
                                                // from the control point before the junction interior
    int junction_min_degree = 3;

    // Turn radius (meters) driving junction connector curve shape and setback (see
    // intersections::turn_radius_for_highway): a per-highway-class override map (checked first),
    // then a built-in per-class default tier, then this flat fallback for an unmapped class -- the
    // resolved value is always multiplied by junction_turn_radius_scale.
    double junction_turn_radius_m = 8.0;
    std::unordered_map<std::string, double> junction_turn_radius_overrides;
    double junction_turn_radius_scale = 1.0;

    bool left_hand_traffic = false;

    bool validate = false;
    std::string report_path;
    std::string debug_dir; // if non-empty, dump control_lines/control_points/graph as GeoJSON here
};

} // namespace xosm::procedural
