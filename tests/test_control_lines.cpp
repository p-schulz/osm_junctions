#include "fixtures_path.hpp"
#include "test_framework.hpp"

#include "osm2xodr/options.hpp"
#include "osm2xodr/osm_parse.hpp"
#include "osm2xodr/procedural/control_lines.hpp"

using namespace osm2xodr;
using namespace osm2xodr::procedural;
using osm2xodr::testing::TestContext;

void run_control_line_tests(TestContext& ctx) {
    ctx.run("control_lines_compatible: identical tags merge", [](TestContext& c) {
        Options o;
        o.input = fixture("merge_compatible.osm");
        const auto parsed = osm::parse_osm(o);
        GeneratorConfig config;
        const auto lines = extract_control_lines(parsed, config);
        CHECK_EQ(c, lines.size(), static_cast<std::size_t>(1));
        if (!lines.empty()) {
            CHECK_EQ(c, lines[0].source_way_ids.size(), static_cast<std::size_t>(2));
            // The fixture's 3 nodes are exactly collinear, so Douglas-Peucker simplification
            // legitimately drops the middle one -- assert the merge covers the full span (both
            // ways' worth of length) rather than an exact vertex count.
            CHECK(c, lines[0].points.size() >= 2);
            CHECK(c, geo::length(lines[0].points.back() - lines[0].points.front()) > 200.0);
        }
    });

    ctx.run("control_lines_compatible: differing name/lanes do not merge", [](TestContext& c) {
        Options o;
        o.input = fixture("merge_incompatible.osm");
        const auto parsed = osm::parse_osm(o);
        GeneratorConfig config;
        const auto lines = extract_control_lines(parsed, config);
        CHECK_EQ(c, lines.size(), static_cast<std::size_t>(2));
        for (const auto& l : lines) CHECK_EQ(c, l.source_way_ids.size(), static_cast<std::size_t>(1));
    });

    ctx.run("control_lines_compatible: direct tag comparison", [](TestContext& c) {
        Tags a{{"highway", "secondary"}, {"name", "X"}, {"lanes", "2"}};
        Tags b = a;
        CHECK(c, control_lines_compatible(a, b));
        b["name"] = "Y";
        CHECK(c, !control_lines_compatible(a, b));
    });

    ctx.run("extract_control_lines: minor highway class excluded by default", [](TestContext& c) {
        Options o;
        o.input = fixture("control_point_intersection.osm");
        const auto parsed = osm::parse_osm(o);
        GeneratorConfig config; // default control_line_highways has no "residential"
        const auto lines = extract_control_lines(parsed, config);
        CHECK_EQ(c, lines.size(), static_cast<std::size_t>(1)); // only the primary way
    });

    ctx.run("extract_control_lines: interior real junction survives simplification", [](TestContext& c) {
        Options o;
        o.input = fixture("control_point_intersection.osm");
        const auto parsed = osm::parse_osm(o);
        GeneratorConfig config;
        config.simplify_tolerance_m = 50.0; // aggressive -- would normally drop a near-collinear vertex
        const auto lines = extract_control_lines(parsed, config);
        CHECK_EQ(c, lines.size(), static_cast<std::size_t>(1));
        if (!lines.empty()) CHECK_EQ(c, lines[0].interior_intersections.size(), static_cast<std::size_t>(1));
    });
}
