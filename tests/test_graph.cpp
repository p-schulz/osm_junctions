#include "fixtures_path.hpp"
#include "test_framework.hpp"

#include "xosm/options.hpp"
#include "xosm/osm_parse.hpp"
#include "xosm/procedural/control_lines.hpp"
#include "xosm/procedural/control_points.hpp"
#include "xosm/procedural/graph.hpp"
#include "xosm/procedural/grouping.hpp"

#include <set>

using namespace xosm;
using namespace xosm::procedural;
using xosm::testing::TestContext;

void run_graph_tests(TestContext& ctx) {
    ctx.run("build_graph: four-way cross yields 5 control points, 4 base edges, no duplicates", [](TestContext& c) {
        Options o;
        o.input = fixture("four_way.osm");
        const auto parsed = osm::parse_osm(o);
        GeneratorConfig config;
        const auto lines = extract_control_lines(parsed, config);
        const auto points = extract_control_points(parsed, lines, config);
        const auto groups = group_control_lines(lines, points, config);
        const auto graph = build_graph(lines, points, groups, config);

        CHECK_EQ(c, graph.control_points.size(), static_cast<std::size_t>(5));
        CHECK_EQ(c, graph.connections.size(), static_cast<std::size_t>(4));

        std::set<std::pair<std::string, std::string>> seen;
        for (const auto& conn : graph.connections) {
            const auto key = conn.from_control_point < conn.to_control_point
                                  ? std::make_pair(conn.from_control_point, conn.to_control_point)
                                  : std::make_pair(conn.to_control_point, conn.from_control_point);
            CHECK(c, seen.insert(key).second); // fails if a duplicate edge exists
        }

        // The shared node (both ways cross there) should have degree 4; the four termini degree 1.
        int degree4_count = 0, degree1_count = 0;
        for (const auto& cp : graph.control_points) {
            const int d = control_point_degree(graph, cp.id);
            if (d == 4) ++degree4_count;
            if (d == 1) ++degree1_count;
        }
        CHECK_EQ(c, degree4_count, 1);
        CHECK_EQ(c, degree1_count, 4);
    });

    ctx.run("build_graph: three-way T yields 3 base edges and one degree-3 control point", [](TestContext& c) {
        Options o;
        o.input = fixture("three_way.osm");
        const auto parsed = osm::parse_osm(o);
        GeneratorConfig config;
        const auto lines = extract_control_lines(parsed, config);
        const auto points = extract_control_points(parsed, lines, config);
        const auto groups = group_control_lines(lines, points, config);
        const auto graph = build_graph(lines, points, groups, config);

        CHECK_EQ(c, graph.connections.size(), static_cast<std::size_t>(3));
        int degree3_count = 0;
        for (const auto& cp : graph.control_points)
            if (control_point_degree(graph, cp.id) == 3) ++degree3_count;
        CHECK_EQ(c, degree3_count, 1);
    });
}
