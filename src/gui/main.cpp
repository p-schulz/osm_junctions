// osm2xodr-gui: a small imgui viewer around the procedural pipeline (osm2xodr::procedural). A top
// command bar imports an OSM file and runs the pipeline; the main area renders a 2D preview of the
// resulting GeneratedRoadGraph (control-point-derived roads, junction connectors, control points)
// directly from its projected-meter geometry -- no separate rendering model, just the same graph
// osm2xodr-procedural writes to OpenDRIVE.

#include "osm2xodr/procedural/pipeline.hpp"
#include "osm2xodr/procedural/types.hpp"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "tinyfiledialogs.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using osm2xodr::geo::Vec2;
using osm2xodr::procedural::Connection;
using osm2xodr::procedural::ControlPoint;
using osm2xodr::procedural::ControlPointType;
using osm2xodr::procedural::GeneratedRoadGraph;
using osm2xodr::procedural::GeneratorConfig;

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
Vec2 bezier_point(const osm2xodr::model::GeomPrimitive& g, const float t) {
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

ImU32 connection_color(const Connection& conn) {
    if (!conn.diagnostic.empty()) return IM_COL32(230, 60, 60, 255);       // dangling boundary stub
    if (!conn.junction_id.empty()) return IM_COL32(255, 165, 0, 255);      // junction connector
    if (conn.synthetic) return IM_COL32(200, 100, 255, 255);               // lane-count bridge
    return IM_COL32(220, 220, 220, 255);                                  // ordinary road
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

    for (const auto& conn : graph.connections) {
        const auto pts = connection_polyline(conn);
        if (pts.size() < 2) continue;
        const ImU32 color = connection_color(conn);
        const float thickness = conn.junction_id.empty() && !conn.synthetic ? 2.0f : 1.5f;
        for (std::size_t i = 1; i < pts.size(); ++i) {
            draw_list->AddLine(world_to_screen(pts[i - 1], origin, cam), world_to_screen(pts[i], origin, cam),
                                color, thickness);
        }
    }
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
        const auto model = osm2xodr::procedural::run_pipeline(config, &graph);
        app.graph = std::move(graph);
        app.have_graph = true;
        app.status_is_error = false;
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "Loaded %s: %zu control lines, %zu control points, %zu roads, %zu junctions",
                      path, app.graph.control_lines.size(), app.graph.control_points.size(),
                      model.roads.size(), model.junctions.size());
        app.status = buf;
        app.fit_pending = true;
    } catch (const std::exception& e) {
        app.have_graph = false;
        app.status_is_error = true;
        app.status = std::string("Failed to import ") + path + ": " + e.what();
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

    GLFWwindow* window = glfwCreateWindow(1280, 800, "osm2xodr - procedural road network viewer", nullptr, nullptr);
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
        ImGui::Begin("osm2xodr", nullptr,
                      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                          ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus);

        // --- Command bar -----------------------------------------------------------------
        if (ImGui::Button("Import OSM...")) import_osm_file(app);
        ImGui::SameLine();
        ImGui::BeginDisabled(!app.have_graph);
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
