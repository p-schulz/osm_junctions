#include "fixtures_path.hpp"
#include "test_framework.hpp"

#include "osm2xodr/options.hpp"
#include "osm2xodr/osm_parse.hpp"
#include "osm2xodr/procedural/control_lines.hpp"
#include "osm2xodr/procedural/control_points.hpp"
#include "osm2xodr/procedural/graph.hpp"
#include "osm2xodr/procedural/grouping.hpp"
#include "osm2xodr/procedural/lanes.hpp"

using namespace osm2xodr;
using namespace osm2xodr::procedural;
using osm2xodr::testing::TestContext;

namespace {

GeneratedRoadGraph build(const std::string& path, const GeneratorConfig& config) {
    Options o;
    o.input = path;
    const auto parsed = osm::parse_osm(o);
    const auto lines = extract_control_lines(parsed, config);
    const auto points = extract_control_points(parsed, lines, config);
    const auto groups = group_control_lines(lines, points, config);
    auto graph = build_graph(lines, points, groups, config);
    configure_lanes(graph, config);
    return graph;
}

const Connection* find_by_name(const GeneratedRoadGraph& graph, const std::string& name) {
    for (const auto& c : graph.connections)
        if (tag_value_or(c.tags, "name", "") == name) return &c;
    return nullptr;
}

} // namespace

void run_lane_tests(TestContext& ctx) {
    ctx.run("configure_lanes: explicit lanes:forward/backward asymmetry", [](TestContext& c) {
        GeneratorConfig config;
        const auto graph = build(fixture("lanes_asymmetric.osm"), config);
        const auto* conn = find_by_name(graph, "Asymmetric Ave");
        CHECK(c, conn != nullptr);
        if (!conn) return;
        CHECK_EQ(c, conn->lanes.forward_lanes, 2);
        CHECK_EQ(c, conn->lanes.backward_lanes, 1);
        CHECK(c, !conn->lanes.oneway);
        CHECK_EQ(c, conn->lanes.right.size(), static_cast<std::size_t>(2)); // forward -> right
        CHECK_EQ(c, conn->lanes.left.size(), static_cast<std::size_t>(1));  // backward -> left
    });

    ctx.run("configure_lanes: oneway road has zero opposite-direction lanes", [](TestContext& c) {
        GeneratorConfig config;
        const auto graph = build(fixture("lanes_asymmetric.osm"), config);
        const auto* conn = find_by_name(graph, "Oneway Blvd");
        CHECK(c, conn != nullptr);
        if (!conn) return;
        CHECK(c, conn->lanes.oneway);
        CHECK_EQ(c, conn->lanes.forward_lanes, 3);
        CHECK_EQ(c, conn->lanes.backward_lanes, 0);
        CHECK_EQ(c, conn->lanes.right.size(), static_cast<std::size_t>(3));
        CHECK(c, conn->lanes.left.empty());
    });

    ctx.run("default_lane_count_for_highway / default_oneway_for_highway", [](TestContext& c) {
        CHECK_EQ(c, default_lane_count_for_highway("motorway"), 2);
        CHECK_EQ(c, default_lane_count_for_highway("residential"), 1);
        CHECK(c, default_oneway_for_highway("motorway"));
        CHECK(c, !default_oneway_for_highway("residential"));
    });

    ctx.run("configure_lanes: missing tags fall back to deterministic highway-class default", [](TestContext& c) {
        GeneratorConfig config;
        const auto graph = build(fixture("three_way.osm"), config);
        // "Branch Road" (secondary, lanes=2) should derive 1 forward/1 backward from the plain
        // lanes=2 tag deterministically -- same result every run, no lanes:forward/backward tag.
        const auto* conn = find_by_name(graph, "Branch Road");
        CHECK(c, conn != nullptr);
        if (!conn) return;
        CHECK_EQ(c, conn->lanes.forward_lanes, 1);
        CHECK_EQ(c, conn->lanes.backward_lanes, 1);
    });
}
