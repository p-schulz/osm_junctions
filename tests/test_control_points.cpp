#include "fixtures_path.hpp"
#include "test_framework.hpp"

#include "osm2xodr/options.hpp"
#include "osm2xodr/osm_parse.hpp"
#include "osm2xodr/procedural/control_lines.hpp"
#include "osm2xodr/procedural/control_points.hpp"

#include <algorithm>

using namespace osm2xodr;
using namespace osm2xodr::procedural;
using osm2xodr::testing::TestContext;

namespace {

int count_type(const std::vector<ControlPoint>& pts, const ControlPointType t) {
    return static_cast<int>(std::count_if(pts.begin(), pts.end(), [&](const ControlPoint& p) { return p.type == t; }));
}

} // namespace

void run_control_point_tests(TestContext& ctx) {
    ctx.run("extract_control_points: real OSM intersection embedded in a way's interior", [](TestContext& c) {
        Options o;
        o.input = fixture("control_point_intersection.osm");
        const auto parsed = osm::parse_osm(o);
        GeneratorConfig config;
        const auto lines = extract_control_lines(parsed, config);
        const auto points = extract_control_points(parsed, lines, config);
        CHECK_EQ(c, count_type(points, ControlPointType::OsmIntersection), 1);
        // 2 endpoints of the primary way are true dead ends (degree 1) -> EndpointConnector.
        CHECK_EQ(c, count_type(points, ControlPointType::EndpointConnector), 2);
    });

    ctx.run("extract_control_points: minor-road endpoint near a control line -> ProjectedCrossing", [](TestContext& c) {
        Options o;
        o.input = fixture("control_point_projected.osm");
        const auto parsed = osm::parse_osm(o);
        GeneratorConfig config;
        const auto lines = extract_control_lines(parsed, config);
        const auto points = extract_control_points(parsed, lines, config);
        CHECK_EQ(c, count_type(points, ControlPointType::ProjectedCrossing), 1);
    });

    ctx.run("extract_control_points: snapping merges nearby duplicate candidates", [](TestContext& c) {
        Options o;
        o.input = fixture("control_point_intersection.osm");
        const auto parsed = osm::parse_osm(o);
        GeneratorConfig config;
        config.snap_distance_m = 3.0;
        const auto lines = extract_control_lines(parsed, config);
        const auto points_default = extract_control_points(parsed, lines, config);
        // 3 termini of the primary way's chain (2 endpoints) + the interior junction = 3 total
        // candidate nodes; none of them are within snap_distance_m of each other in this fixture, so
        // snapping should not change the count -- this asserts snapping doesn't over-merge distinct points.
        CHECK_EQ(c, points_default.size(), static_cast<std::size_t>(3));
    });

    ctx.run("extract_control_points: min-spacing prunes an interior point close to a terminus", [](TestContext& c) {
        Options o;
        o.input = fixture("control_point_projected.osm");
        const auto parsed = osm::parse_osm(o);
        GeneratorConfig config;
        config.min_control_point_spacing_m = 500.0; // larger than the whole fixture's extent
        const auto lines = extract_control_lines(parsed, config);
        const auto points = extract_control_points(parsed, lines, config);
        // The ProjectedCrossing point is close (in arclength) to one of the two termini, so an
        // aggressive min-spacing should prune it while the (structural) termini remain.
        CHECK_EQ(c, count_type(points, ControlPointType::ProjectedCrossing), 0);
        CHECK_EQ(c, points.size(), static_cast<std::size_t>(2));
    });
}
