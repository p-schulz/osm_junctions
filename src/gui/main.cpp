// xosm-gui: a small imgui viewer around the procedural pipeline (xosm::procedural). A top command
// bar imports an OSM file, runs the pipeline, and can export the result to OpenDRIVE (reusing
// xosm::xodr::write_file unchanged); the main area renders a lane-by-lane 2D preview of the
// resulting GeneratedRoadGraph directly from its projected-meter geometry -- no separate rendering
// model, just the same graph that gets written to .xodr.

#include "xosm/options.hpp"
#include "xosm/procedural/pipeline.hpp"
#include "xosm/procedural/types.hpp"
#include "xosm/xodr_writer.hpp"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "tinyfiledialogs.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

using xosm::geo::Vec2;
using xosm::geo::left_normal;
using xosm::geo::normalize;
using xosm::procedural::Connection;
using xosm::procedural::ControlPoint;
using xosm::procedural::ControlPointType;
using xosm::procedural::GeneratedRoadGraph;
using xosm::procedural::GeneratorConfig;

namespace {

// Pan/zoom state for the 2D preview: `zoom` is screen pixels per world meter; `pan` is the screen
// offset (pixels) of the world origin from the canvas center.
struct Camera {
    ImVec2 pan{0.0f, 0.0f};
    float zoom = 1.0f;
};

ImVec2 world_to_screen(const Vec2& world, const ImVec2& origin, const Camera& cam) {
    return ImVec2(origin.x + cam.pan.x + static_cast<float>(world.x) * cam.zoom,
                  origin.y + cam.pan.y - static_cast<float>(world.y) * cam.zoom);
}

// Standard cubic-Bezier evaluation for a junction connector's ParamPoly3 primitive (model.hpp
// stores its control points in a local frame rotated by hdg, origin (x,y); P0 is implicitly (0,0)).
Vec2 bezier_point(const xosm::model::GeomPrimitive& g, const float t) {
    const Vec2 p0{0.0, 0.0};
    const Vec2& p1 = g.local_p1;
    const Vec2& p2 = g.local_p2;
    const Vec2& p3 = g.local_p3;
    const float u = 1.0f - t;
    const Vec2 local = p0 * (u * u * u) + p1 * (3 * u * u * t) + p2 * (3 * u * t * t) + p3 * (t * t * t);
    const double c = std::cos(g.hdg), s = std::sin(g.hdg);
    return Vec2{g.x + local.x * c - local.y * s, g.y + local.x * s + local.y * c};
}

std::vector<Vec2> connection_polyline(const Connection& conn) {
    if (conn.explicit_geometry.empty()) return conn.geometry;
    std::vector<Vec2> pts;
    constexpr int kSamples = 16;
    for (const auto& g : conn.explicit_geometry) {
        for (int i = 0; i <= kSamples; ++i) pts.push_back(bezier_point(g, static_cast<float>(i) / kSamples));
    }
    return pts;
}

// Unit tangent at polyline vertex `i` (central difference away from the endpoints, so a lane strip
// doesn't kink at interior vertices).
Vec2 tangent_at(const std::vector<Vec2>& pts, const std::size_t i) {
    if (pts.size() < 2) return Vec2{1.0, 0.0};
    if (i == 0) return normalize(pts[1] - pts[0]);
    if (i + 1 == pts.size()) return normalize(pts[i] - pts[i - 1]);
    return normalize(pts[i + 1] - pts[i - 1]);
}

// Per-lane fill color: a family color by connection role (ordinary/junction-connector/bridge/
// dangling stub), a directionality tint (backward/left lanes read slightly bluer than forward/right
// -- oncoming vs. same-direction traffic), and a light/dark alternation by lane index so adjacent
// lanes in the same stack are individually distinguishable rather than reading as one wide fill.
ImU32 lane_fill_color(const Connection& conn, const bool is_left, const int index_from_center) {
    const bool alt = (index_from_center % 2) == 1;
    if (!conn.diagnostic.empty()) return alt ? IM_COL32(205, 75, 75, 255) : IM_COL32(175, 58, 58, 255);
    if (!conn.junction_id.empty()) return alt ? IM_COL32(232, 152, 62, 255) : IM_COL32(203, 128, 42, 255);
    if (conn.synthetic) return alt ? IM_COL32(178, 112, 218, 255) : IM_COL32(153, 92, 193, 255);
    if (is_left) return alt ? IM_COL32(112, 122, 148, 255) : IM_COL32(96, 106, 132, 255); // backward/oncoming
    return alt ? IM_COL32(168, 168, 173, 255) : IM_COL32(147, 147, 152, 255);              // forward
}

ImU32 control_point_color(const ControlPointType t) {
    switch (t) {
        case ControlPointType::OsmIntersection: return IM_COL32(255, 220, 0, 255);
        case ControlPointType::ProjectedCrossing: return IM_COL32(0, 220, 220, 255);
        case ControlPointType::CorridorConnector: return IM_COL32(150, 150, 150, 255);
        case ControlPointType::EndpointConnector: return IM_COL32(230, 60, 60, 255);
        case ControlPointType::Sampled: return IM_COL32(80, 220, 80, 255);
    }
    return IM_COL32(255, 255, 255, 255);
}

const char* control_point_label(const ControlPointType t) {
    switch (t) {
        case ControlPointType::OsmIntersection: return "OSM intersection";
        case ControlPointType::ProjectedCrossing: return "Projected crossing";
        case ControlPointType::CorridorConnector: return "Corridor connector";
        case ControlPointType::EndpointConnector: return "Endpoint / dead end";
        case ControlPointType::Sampled: return "Sampled";
    }
    return "?";
}

// Renders one Connection lane-by-lane: for each side (left = positive ids/backward, right =
// negative ids/forward, both stored center-outward per model.hpp), walks outward from the
// reference line accumulating each lane's own width (interpolated across a width/width_end taper)
// as a filled quad strip offset along the polyline's local left-normal, starting from
// `lanes.lane_offset` -- the same offset the OpenDRIVE writer itself applies between the reference
// line and the lane stack (model::compute_lane_offset).
void draw_connection_lanes(ImDrawList* draw_list, const Connection& conn, const ImVec2& origin, const Camera& cam) {
    const auto centerline = connection_polyline(conn);
    if (centerline.size() < 2) return;

    std::vector<double> s_at(centerline.size(), 0.0);
    for (std::size_t i = 1; i < centerline.size(); ++i)
        s_at[i] = s_at[i - 1] + xosm::geo::length(centerline[i] - centerline[i - 1]);
    const double total_s = s_at.back();

    const auto width_at = [&](const xosm::model::LaneSpec& lane, const double s) {
        if (lane.width_end < 0.0 || total_s <= 1e-6) return lane.width;
        return lane.width + (lane.width_end - lane.width) * (s / total_s);
    };

    std::vector<Vec2> normals(centerline.size());
    for (std::size_t i = 0; i < centerline.size(); ++i) normals[i] = left_normal(tangent_at(centerline, i));

    for (const bool is_left : {true, false}) {
        const auto& lanes = is_left ? conn.lanes.left : conn.lanes.right;
        const double sign = is_left ? 1.0 : -1.0;
        std::vector<double> cum(centerline.size(), static_cast<double>(conn.lanes.lane_offset));
        for (std::size_t li = 0; li < lanes.size(); ++li) {
            const auto& lane = lanes[li];
            std::vector<ImVec2> inner(centerline.size()), outer(centerline.size());
            for (std::size_t i = 0; i < centerline.size(); ++i) {
                const double w = width_at(lane, s_at[i]);
                const double inner_t = cum[i];
                const double outer_t = cum[i] + sign * w;
                inner[i] = world_to_screen(centerline[i] + normals[i] * inner_t, origin, cam);
                outer[i] = world_to_screen(centerline[i] + normals[i] * outer_t, origin, cam);
                cum[i] = outer_t;
            }
            const ImU32 color = lane_fill_color(conn, is_left, static_cast<int>(li));
            for (std::size_t i = 1; i < centerline.size(); ++i)
                draw_list->AddQuadFilled(inner[i - 1], outer[i - 1], outer[i], inner[i], color);
        }
    }

    // A thin reference-line stroke at the lane_offset baseline, for orientation on two-way roads.
    for (std::size_t i = 1; i < centerline.size(); ++i) {
        const Vec2 a = centerline[i - 1] + normals[i - 1] * static_cast<double>(conn.lanes.lane_offset);
        const Vec2 b = centerline[i] + normals[i] * static_cast<double>(conn.lanes.lane_offset);
        draw_list->AddLine(world_to_screen(a, origin, cam), world_to_screen(b, origin, cam),
                            IM_COL32(255, 255, 255, 90), 1.0f);
    }
}

void fit_view(const GeneratedRoadGraph& graph, const ImVec2& canvas_size, Camera& cam) {
    double min_x = 1e18, min_y = 1e18, max_x = -1e18, max_y = -1e18;
    bool any = false;
    for (const auto& conn : graph.connections) {
        for (const auto& p : connection_polyline(conn)) {
            min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
            any = true;
        }
    }
    if (!any || canvas_size.x <= 1.0f || canvas_size.y <= 1.0f) {
        cam = Camera{};
        return;
    }
    const double world_w = std::max(1.0, max_x - min_x);
    const double world_h = std::max(1.0, max_y - min_y);
    const double center_x = (min_x + max_x) / 2.0;
    const double center_y = (min_y + max_y) / 2.0;
    const float zoom = static_cast<float>(std::min(canvas_size.x / world_w, canvas_size.y / world_h) * 0.9);
    cam.zoom = std::max(zoom, 1e-4f);
    cam.pan = ImVec2(static_cast<float>(-center_x * cam.zoom), static_cast<float>(center_y * cam.zoom));
}

void draw_preview(const GeneratedRoadGraph& graph, Camera& cam) {
    ImGui::BeginChild("PreviewCanvas", ImVec2(0, 0), true,
                       ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    const ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
    const ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    const ImVec2 origin(canvas_p0.x + canvas_size.x * 0.5f, canvas_p0.y + canvas_size.y * 0.5f);
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->PushClipRect(canvas_p0, ImVec2(canvas_p0.x + canvas_size.x, canvas_p0.y + canvas_size.y), true);
    draw_list->AddRectFilled(canvas_p0, ImVec2(canvas_p0.x + canvas_size.x, canvas_p0.y + canvas_size.y),
                              IM_COL32(30, 30, 34, 255));

    ImGui::InvisibleButton("canvas_interaction", canvas_size,
                            ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    ImGuiIO& io = ImGui::GetIO();
    if (active && (io.MouseDown[ImGuiMouseButton_Left] || io.MouseDown[ImGuiMouseButton_Right])) {
        cam.pan.x += io.MouseDelta.x;
        cam.pan.y += io.MouseDelta.y;
    }
    if (hovered && io.MouseWheel != 0.0f) {
        const float old_zoom = cam.zoom;
        const float new_zoom = std::clamp(old_zoom * std::pow(1.1f, io.MouseWheel), 1e-4f, 2000.0f);
        const ImVec2 mouse = io.MousePos;
        const ImVec2 before_world((mouse.x - origin.x - cam.pan.x) / old_zoom,
                                   -(mouse.y - origin.y - cam.pan.y) / old_zoom);
        cam.zoom = new_zoom;
        cam.pan.x = mouse.x - origin.x - before_world.x * new_zoom;
        cam.pan.y = mouse.y - origin.y + before_world.y * new_zoom;
    }

    for (const auto& conn : graph.connections) draw_connection_lanes(draw_list, conn, origin, cam);
    for (const auto& cp : graph.control_points) {
        const ImVec2 s = world_to_screen(cp.point, origin, cam);
        if (s.x < canvas_p0.x - 20 || s.x > canvas_p0.x + canvas_size.x + 20 ||
            s.y < canvas_p0.y - 20 || s.y > canvas_p0.y + canvas_size.y + 20)
            continue;
        draw_list->AddCircleFilled(s, cp.type == ControlPointType::OsmIntersection ? 4.5f : 3.0f,
                                    control_point_color(cp.type));
    }

    draw_list->PopClipRect();
    ImGui::EndChild();
}

struct AppState {
    GeneratedRoadGraph graph;
    xosm::model::MapModel model;
    std::string source_name; // basename of the last-imported file, for the export dialog's default name
    Camera camera;
    std::string status = "No file loaded. Use \"Import OSM...\" to begin.";
    bool status_is_error = false;
    bool have_graph = false;
    bool fit_pending = false;
};

void import_osm_file(AppState& app) {
    const char* filters[] = {"*.osm", "*.osm.pbf", "*.pbf"};
    const char* path = tinyfd_openFileDialog("Import OpenStreetMap file", "", 3, filters,
                                              "OpenStreetMap files (.osm, .osm.pbf)", 0);
    if (!path) return; // user cancelled

    GeneratorConfig config;
    config.input = path;
    GeneratedRoadGraph graph;
    try {
        auto model = xosm::procedural::run_pipeline(config, &graph);
        app.graph = std::move(graph);
        app.model = std::move(model);
        app.source_name = std::filesystem::path(path).stem().string();
        app.have_graph = true;
        app.status_is_error = false;
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "Loaded %s: %zu control lines, %zu control points, %zu roads, %zu junctions",
                      path, app.graph.control_lines.size(), app.graph.control_points.size(),
                      app.model.roads.size(), app.model.junctions.size());
        app.status = buf;
        app.fit_pending = true;
    } catch (const std::exception& e) {
        app.have_graph = false;
        app.status_is_error = true;
        app.status = std::string("Failed to import ") + path + ": " + e.what();
    }
}

void export_xodr_file(AppState& app) {
    const char* filters[] = {"*.xodr"};
    const std::string default_path = (app.source_name.empty() ? "network" : app.source_name) + ".xodr";
    const char* path = tinyfd_saveFileDialog("Export OpenDRIVE file", default_path.c_str(), 1, filters,
                                              "OpenDRIVE files (.xodr)");
    if (!path) return; // user cancelled

    xosm::Options writer_options;
    writer_options.output = path;
    writer_options.name = app.source_name.empty() ? "xosm" : app.source_name;
    try {
        xosm::xodr::write_file(app.model, writer_options);
        app.status_is_error = false;
        char buf[512];
        std::snprintf(buf, sizeof(buf), "Exported %s (%zu roads, %zu junctions)", path, app.model.roads.size(),
                      app.model.junctions.size());
        app.status = buf;
    } catch (const std::exception& e) {
        app.status_is_error = true;
        app.status = std::string("Failed to export ") + path + ": " + e.what();
    }
}

void glfw_error_callback(int error, const char* description) {
    std::fprintf(stderr, "GLFW error %d: %s\n", error, description);
}

} // namespace

int main() {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) return 1;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);

    GLFWwindow* window = glfwCreateWindow(1280, 800, "XOSM - road network viewer", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 150");

    AppState app;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::Begin("xosm", nullptr,
                      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                          ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus);

        // --- Command bar -----------------------------------------------------------------
        if (ImGui::Button("Import OSM...")) import_osm_file(app);
        ImGui::SameLine();
        ImGui::BeginDisabled(!app.have_graph);
        if (ImGui::Button("Export .xodr...")) export_xodr_file(app);
        ImGui::SameLine();
        if (ImGui::Button("Fit View")) app.fit_pending = true;
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();
        for (const auto t : {ControlPointType::OsmIntersection, ControlPointType::ProjectedCrossing,
                              ControlPointType::CorridorConnector, ControlPointType::EndpointConnector,
                              ControlPointType::Sampled}) {
            ImGui::ColorButton(control_point_label(t), ImGui::ColorConvertU32ToFloat4(control_point_color(t)),
                                ImGuiColorEditFlags_NoTooltip, ImVec2(10, 10));
            ImGui::SameLine(0.0f, 4.0f);
            ImGui::TextUnformatted(control_point_label(t));
            ImGui::SameLine();
        }
        ImGui::NewLine();
        if (app.status_is_error) ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", app.status.c_str());
        else ImGui::TextUnformatted(app.status.c_str());
        ImGui::Separator();

        // --- 2D preview --------------------------------------------------------------------
        if (app.have_graph) {
            if (app.fit_pending) {
                fit_view(app.graph, ImGui::GetContentRegionAvail(), app.camera);
                app.fit_pending = false;
            }
            draw_preview(app.graph, app.camera);
        } else {
            ImGui::BeginChild("EmptyPreview", ImVec2(0, 0), true);
            const ImVec2 avail = ImGui::GetContentRegionAvail();
            ImGui::SetCursorPos(ImVec2(avail.x * 0.5f - 90.0f, avail.y * 0.5f));
            ImGui::TextDisabled("Import an OSM file to preview its road network");
            ImGui::EndChild();
        }

        ImGui::End();

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.08f, 0.08f, 0.09f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
