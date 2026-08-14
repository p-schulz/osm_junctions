#include "fixtures_path.hpp"
#include "test_framework.hpp"

#include "xosm/procedural/geo_extra.hpp"
#include "xosm/procedural/intersections.hpp"
#include "xosm/procedural/pipeline.hpp"

#include <cmath>

using namespace xosm;
using namespace xosm::procedural;
using xosm::testing::TestContext;

void run_intersection_tests(TestContext& ctx) {
    ctx.run("turn_radius_for_highway: built-in per-class tiers", [](TestContext& c) {
        GeneratorConfig config;
        CHECK_EQ(c, turn_radius_for_highway("motorway", config), 15.0);
        CHECK_EQ(c, turn_radius_for_highway("primary", config), 10.0);
        CHECK_EQ(c, turn_radius_for_highway("secondary", config), 8.0);
        CHECK_EQ(c, turn_radius_for_highway("residential", config), 5.0);
    });

    ctx.run("turn_radius_for_highway: unmapped class falls back to junction_turn_radius_m", [](TestContext& c) {
        GeneratorConfig config;
        config.junction_turn_radius_m = 12.5;
        CHECK_EQ(c, turn_radius_for_highway("track", config), 12.5);
    });

    ctx.run("turn_radius_for_highway: override map takes precedence over the built-in table", [](TestContext& c) {
        GeneratorConfig config;
        config.junction_turn_radius_overrides["residential"] = 3.0;
        CHECK_EQ(c, turn_radius_for_highway("residential", config), 3.0);
    });

    ctx.run("turn_radius_for_highway: scale multiplies the resolved radius", [](TestContext& c) {
        GeneratorConfig config;
        config.junction_turn_radius_scale = 2.0;
        CHECK_EQ(c, turn_radius_for_highway("primary", config), 20.0);
    });

    ctx.run("hermite_bezier_geometry: larger radius produces a longer tangent handle", [](TestContext& c) {
        const geo::Vec2 p0{0.0, 0.0};
        const geo::Vec2 p3{10.0, 10.0};
        const double hdg0 = 0.0;           // pointing +x, so the local frame == world frame
        const double hdg3 = geo::kPi / 2.0; // a 90-degree turn
        const auto small = geo_extra::hermite_bezier_geometry(p0, hdg0, p3, hdg3, 4.0);
        const auto big = geo_extra::hermite_bezier_geometry(p0, hdg0, p3, hdg3, 20.0);
        CHECK(c, big.local_p1.x > small.local_p1.x);
    });

    ctx.run("hermite_bezier_geometry: omitting the radius preserves the original chord/3 handle", [](TestContext& c) {
        const geo::Vec2 p0{0.0, 0.0};
        const geo::Vec2 p3{9.0, 0.0};
        const auto g = geo_extra::hermite_bezier_geometry(p0, 0.0, p3, 0.0); // no radius argument
        CHECK(c, std::abs(g.local_p1.x - 3.0) < 1e-9);                       // chord=9, lever=chord/3=3
    });

    ctx.run("generate_intersections: turn radius scale changes connector geometry", [](TestContext& c) {
        const auto total_connector_length = [](const GeneratedRoadGraph& g) {
            double total = 0.0;
            for (const auto& conn : g.connections)
                if (conn.synthetic && !conn.junction_id.empty())
                    for (const auto& geom : conn.explicit_geometry) total += geom.length;
            return total;
        };

        GeneratorConfig config_small;
        config_small.input = fixture("four_way.osm");
        config_small.junction_turn_radius_scale = 0.3;
        GeneratedRoadGraph graph_small;
        run_pipeline(config_small, &graph_small);

        GeneratorConfig config_big;
        config_big.input = fixture("four_way.osm");
        config_big.junction_turn_radius_scale = 3.0;
        GeneratedRoadGraph graph_big;
        run_pipeline(config_big, &graph_big);

        CHECK(c, total_connector_length(graph_big) > total_connector_length(graph_small));
    });
}
