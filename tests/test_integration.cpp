#include "fixtures_path.hpp"
#include "test_framework.hpp"

#include "osm2xodr/options.hpp"
#include "osm2xodr/procedural/cleanup.hpp"
#include "osm2xodr/procedural/pipeline.hpp"
#include "osm2xodr/xodr_writer.hpp"

#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

using namespace osm2xodr;
using namespace osm2xodr::procedural;
using osm2xodr::testing::TestContext;

namespace {

std::string read_file(const std::string& path) {
    std::ifstream is(path);
    std::ostringstream ss;
    ss << is.rdbuf();
    return ss.str();
}

// Minimal well-formedness check (matching open/close tags) for our own generator's output -- not a
// general XML parser, but sufficient since util::xml_escape guarantees no raw '<'/'>' inside
// attribute values in anything the writer itself emits.
bool xml_tags_balanced(const std::string& xml) {
    std::vector<std::string> stack;
    std::size_t i = 0;
    while (i < xml.size()) {
        const auto lt = xml.find('<', i);
        if (lt == std::string::npos) break;
        const auto gt = xml.find('>', lt);
        if (gt == std::string::npos) return false;
        std::string tag = xml.substr(lt + 1, gt - lt - 1);
        i = gt + 1;
        if (tag.empty()) return false;
        if (tag[0] == '?' || tag[0] == '!') continue;
        const bool self_closing = tag.back() == '/';
        const bool closing = tag[0] == '/';
        if (closing) {
            if (stack.empty()) return false;
            const std::string name = tag.substr(1);
            if (stack.back() != name) return false;
            stack.pop_back();
        } else if (!self_closing) {
            const auto sp = tag.find_first_of(" \t");
            stack.push_back(sp == std::string::npos ? tag : tag.substr(0, sp));
        }
    }
    return stack.empty();
}

int count_occurrences(const std::string& haystack, const std::string& needle) {
    int n = 0;
    std::size_t pos = 0;
    while ((pos = haystack.find(needle, pos)) != std::string::npos) {
        ++n;
        pos += needle.size();
    }
    return n;
}

std::string generate(const std::string& fixture_name, const GeneratorConfig* config_override = nullptr) {
    static std::atomic<int> counter{0};
    GeneratorConfig config = config_override ? *config_override : GeneratorConfig{};
    config.input = fixture(fixture_name);
    config.output = (std::filesystem::temp_directory_path() /
                      ("osm2xodr_test_" + std::to_string(counter++) + ".xodr")).string();
    const auto model = run_pipeline(config);
    Options writer_options;
    writer_options.output = config.output;
    writer_options.name = "test";
    xodr::write_file(model, writer_options);
    const auto content = read_file(config.output);
    std::remove(config.output.c_str());
    return content;
}

} // namespace

void run_integration_tests(TestContext& ctx) {
    ctx.run("integration: three-way OSM fixture -> well-formed XODR with a junction", [](TestContext& c) {
        const auto xml = generate("three_way.osm");
        CHECK(c, !xml.empty());
        CHECK(c, xml_tags_balanced(xml));
        CHECK(c, count_occurrences(xml, "<road ") >= 3);
        CHECK_EQ(c, count_occurrences(xml, "<junction "), 1);
        CHECK(c, count_occurrences(xml, "<laneSection ") >= 3);
        CHECK(c, count_occurrences(xml, "<laneLink ") >= 1);
    });

    ctx.run("integration: four-way OSM fixture -> well-formed XODR, one junction, 12 lane links", [](TestContext& c) {
        const auto xml = generate("four_way.osm");
        CHECK(c, xml_tags_balanced(xml));
        CHECK_EQ(c, count_occurrences(xml, "<road "), 16);
        CHECK_EQ(c, count_occurrences(xml, "<junction "), 1);
        CHECK_EQ(c, count_occurrences(xml, "<laneLink "), 12);
        CHECK(c, count_occurrences(xml, "<paramPoly3 ") >= 12);
    });

    ctx.run("integration: asymmetric-lanes/oneway fixture -> well-formed XODR, no junction", [](TestContext& c) {
        const auto xml = generate("lanes_asymmetric.osm");
        CHECK(c, xml_tags_balanced(xml));
        CHECK_EQ(c, count_occurrences(xml, "<road "), 2);
        CHECK_EQ(c, count_occurrences(xml, "<junction "), 0);
    });

    ctx.run("validate_map_model: all predecessor/successor/junction references resolve", [](TestContext& c) {
        GeneratorConfig config;
        config.input = fixture("four_way.osm");
        GeneratedRoadGraph graph;
        const auto model = run_pipeline(config, &graph);
        const auto issues = validate_map_model(model);
        for (const auto& issue : issues) c.check(false, "unexpected validation issue: " + issue);
        CHECK(c, issues.empty());
    });
}
