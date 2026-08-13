#include "xosm/procedural/cli.hpp"
#include "xosm/procedural/pipeline.hpp"
#include "xosm/util.hpp"
#include "xosm/xodr_writer.hpp"

#ifdef XOSM_WITH_LIBOPENDRIVE
#include <OpenDriveMap.h>
#endif

#include <exception>
#include <fstream>
#include <iostream>

namespace xosm::procedural {

namespace {

void write_report(const model::MapModel& model, const GeneratedRoadGraph& graph, const GeneratorConfig& config) {
    if (config.report_path.empty()) return;
    std::ofstream os(config.report_path);
    if (!os) util::fail("Could not open report file: " + config.report_path);
    os << "xosm conversion report\n=====================================\n\n";
    os << "Input: " << config.input << "\nOutput: " << config.output << "\n\n";
    os << "Control lines: " << graph.control_lines.size() << "\n";
    os << "Control points: " << graph.control_points.size() << "\n";
    os << "Control-line groups: " << graph.groups.size() << "\n";
    os << "Roads written: " << model.roads.size() << "\n";
    os << "Junctions written: " << model.junctions.size() << "\n\n";
    os << "Diagnostics\n-----------\n";
    if (graph.diagnostics.empty()) os << "None\n";
    for (const auto& d : graph.diagnostics) os << "- " << d << "\n";
    os << "\nWarnings\n--------\n";
    if (model.warnings.empty()) os << "None\n";
    for (const auto& w : model.warnings) os << "- " << w << "\n";
}

void validate_with_libopendrive([[maybe_unused]] const std::string& path) {
#ifdef XOSM_WITH_LIBOPENDRIVE
    odr::OpenDriveMap map(path);
    std::cerr << "libOpenDRIVE read-back: " << map.get_roads().size() << " roads parsed\n";
#else
    util::fail("--validate requested, but xosm was built without XOSM_ENABLE_LIBOPENDRIVE_VALIDATION=ON");
#endif
}

int run(const int argc, char** argv) {
    const GeneratorConfig config = parse_args(argc, argv);

    GeneratedRoadGraph graph;
    const model::MapModel model = run_pipeline(config, &graph);

    Options writer_options;
    writer_options.output = config.output;
    writer_options.name = config.name;
    xodr::write_file(model, writer_options);

    write_report(model, graph, config);
    if (config.validate) validate_with_libopendrive(config.output);

    std::cerr << "Wrote " << config.output << " with " << model.roads.size() << " roads and "
              << model.junctions.size() << " junctions (from " << graph.control_lines.size()
              << " control lines and " << graph.control_points.size() << " control points).\n";
    if (!model.warnings.empty()) {
        std::cerr << model.warnings.size() << " warning(s)/diagnostic(s).";
        if (!config.report_path.empty()) std::cerr << " See " << config.report_path << ".";
        std::cerr << "\n";
    }
    return 0;
}

} // namespace

} // namespace xosm::procedural

int main(int argc, char** argv) {
    try {
        return xosm::procedural::run(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
