#include "xosm/procedural/geojson_debug.hpp"

#include "xosm/procedural/geo_extra.hpp"
#include "xosm/util.hpp"

#include <fstream>
#include <iomanip>

namespace xosm::procedural {

namespace {

std::string lonlat_str(const geo::LocalProjector& proj, const geo::Vec2& p) {
    const auto ll = geo_extra::unproject(proj, p);
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(7) << "[" << ll.lon << ", " << ll.lat << "]";
    return ss.str();
}

const char* control_point_type_name(const ControlPointType t) {
    switch (t) {
        case ControlPointType::OsmIntersection: return "osm_intersection";
        case ControlPointType::ProjectedCrossing: return "projected_crossing";
        case ControlPointType::CorridorConnector: return "corridor_connector";
        case ControlPointType::EndpointConnector: return "endpoint_connector";
        case ControlPointType::Sampled: return "sampled";
    }
    return "unknown";
}

void write_control_lines(const GeneratedRoadGraph& graph, const geo::LocalProjector& proj, const std::string& path) {
    std::ofstream os(path);
    os << "{\"type\":\"FeatureCollection\",\"features\":[\n";
    for (std::size_t i = 0; i < graph.control_lines.size(); ++i) {
        const auto& line = graph.control_lines[i];
        os << "  {\"type\":\"Feature\",\"properties\":{\"id\":\"" << line.id << "\",\"highway\":\""
           << line.highway << "\",\"name\":\"" << util::xml_escape(line.name) << "\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[";
        for (std::size_t k = 0; k < line.points.size(); ++k) {
            os << lonlat_str(proj, line.points[k]);
            if (k + 1 < line.points.size()) os << ",";
        }
        os << "]}}" << (i + 1 < graph.control_lines.size() ? "," : "") << "\n";
    }
    os << "]}\n";
}

void write_control_points(const GeneratedRoadGraph& graph, const geo::LocalProjector& proj, const std::string& path) {
    std::ofstream os(path);
    os << "{\"type\":\"FeatureCollection\",\"features\":[\n";
    for (std::size_t i = 0; i < graph.control_points.size(); ++i) {
        const auto& cp = graph.control_points[i];
        os << "  {\"type\":\"Feature\",\"properties\":{\"id\":\"" << cp.id << "\",\"type\":\""
           << control_point_type_name(cp.type) << "\"},\"geometry\":{\"type\":\"Point\",\"coordinates\":"
           << lonlat_str(proj, cp.point) << "}}" << (i + 1 < graph.control_points.size() ? "," : "") << "\n";
    }
    os << "]}\n";
}

void write_graph(const GeneratedRoadGraph& graph, const geo::LocalProjector& proj, const std::string& path) {
    std::ofstream os(path);
    os << "{\"type\":\"FeatureCollection\",\"features\":[\n";
    for (std::size_t i = 0; i < graph.connections.size(); ++i) {
        const auto& c = graph.connections[i];
        os << "  {\"type\":\"Feature\",\"properties\":{\"id\":\"" << c.id << "\",\"synthetic\":"
           << (c.synthetic ? "true" : "false") << ",\"junction_id\":\"" << c.junction_id
           << "\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[";
        if (!c.geometry.empty()) {
            for (std::size_t k = 0; k < c.geometry.size(); ++k) {
                os << lonlat_str(proj, c.geometry[k]);
                if (k + 1 < c.geometry.size()) os << ",";
            }
        }
        os << "]}}" << (i + 1 < graph.connections.size() ? "," : "") << "\n";
    }
    os << "]}\n";
}

} // namespace

void write_debug_geojson(const GeneratedRoadGraph& graph, const geo::LocalProjector& projector, const std::string& dir) {
    write_control_lines(graph, projector, dir + "/control_lines.geojson");
    write_control_points(graph, projector, dir + "/control_points.geojson");
    write_graph(graph, projector, dir + "/graph.geojson");
}

} // namespace xosm::procedural
