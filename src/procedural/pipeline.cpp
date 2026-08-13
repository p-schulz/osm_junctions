#include "osm2xodr/procedural/pipeline.hpp"

#include "osm2xodr/options.hpp"
#include "osm2xodr/osm_parse.hpp"
#include "osm2xodr/procedural/cleanup.hpp"
#include "osm2xodr/procedural/control_lines.hpp"
#include "osm2xodr/procedural/control_points.hpp"
#include "osm2xodr/procedural/geojson_debug.hpp"
#include "osm2xodr/procedural/graph.hpp"
#include "osm2xodr/procedural/grouping.hpp"
#include "osm2xodr/procedural/intersections.hpp"
#include "osm2xodr/procedural/lanes.hpp"
#include "osm2xodr/procedural/model_assembly.hpp"

namespace osm2xodr::procedural {

model::MapModel run_pipeline(const GeneratorConfig& config, GeneratedRoadGraph* out_graph) {
    Options osm_options;
    osm_options.input = config.input;
    osm_options.origin_lat = config.origin_lat;
    osm_options.origin_lon = config.origin_lon;
    const auto parsed = osm::parse_osm(osm_options);

    const auto control_lines = extract_control_lines(parsed, config);
    const auto control_points = extract_control_points(parsed, control_lines, config);
    const auto groups = group_control_lines(control_lines, control_points, config);

    GeneratedRoadGraph graph = build_graph(control_lines, control_points, groups, config);
    configure_lanes(graph, config);
    generate_intersections(graph, config);
    repair_lane_mismatches(graph, config);

    auto map_model = assemble_map_model(graph, parsed, config);
    const auto issues = validate_map_model(map_model);
    map_model.warnings.insert(map_model.warnings.end(), issues.begin(), issues.end());

    if (!config.debug_dir.empty()) write_debug_geojson(graph, parsed.projector, config.debug_dir);
    if (out_graph) *out_graph = std::move(graph);

    return map_model;
}

} // namespace osm2xodr::procedural
