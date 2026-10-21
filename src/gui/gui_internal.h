#pragma once

#include "app_ui_api.h"
#include "cam/feeds.h"
#include "cam/parametric.h"
#include "cam/heightmap_simulation.h"
#include "rendering/heightmap_renderer.h"
#include "result_types.h"

#include "clipper2/clipper.triangulation.h"

#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdl2.h"

#include <SDL.h>
#define GL_GLEXT_PROTOTYPES
#include <SDL_opengl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace tinglecam_gui {

void start_hole_identification(App&,int operation_index);
void poll_hole_detection(App&);
bool holes_section(App&,cam::Operation&);
void add_pre_drilling(App&,int operation_index);
void draw_hole_contours(App&,OrbitScene&,cam::Operation&);
void refresh_surface_contours(App&, const cam::Operation&);
void draw_surface_contours(App&, OrbitScene&, cam::Operation&);
void draw_model_loading(App&);
void remember_recent_job(App&, const std::filesystem::path&);
void depth_defaults(App&, cam::Operation&);
void apply_origin_pick(App&);

extern std::unique_ptr<wave::heightfield::Renderer> heightmap_view;

struct Transform {
  ImVec2 origin;
  float scale = 1.0f;
  ImVec2 operator()(const Clipper2Lib::Point64& point) const {
    return {origin.x + static_cast<float>(wave::to_world(point.x)) * scale,
            origin.y - static_cast<float>(wave::to_world(point.y)) * scale};
  }
};

// ---- Appearance -----------------------------------------------------------
std::string ui_style_blob(const ImGuiStyle& style);
bool ui_style_from_blob(const std::string& blob, ImGuiStyle& style);
bool light_theme(const App& app);
// Canvas colours follow the theme, so a light theme gets a light canvas.
struct CanvasColours {
  ImU32 background = 0;
  ImU32 caption = 0;
  ImU32 message = 0;
  ImU32 outline = 0;  // drawn under axis lines so they read over anything
  std::array<float, 4> background_rgba{};
};
CanvasColours canvas_colours(const App& app);
ImVec4 severity_colour(const App& app, LogSeverity severity);
ImVec4 success_colour(const App& app);
ImVec4 muted_colour(const App& app);

// ---- Property rows: label on the left, control filling the right ----------
bool begin_properties(const char* id);
void end_properties();
void property_label(const char* label, const char* help = nullptr);
void help_marker(const char* text);
bool prop_double(const char* label, double& value, float speed,
                 const char* format, const char* help = nullptr,
                 double minimum = -std::numeric_limits<double>::infinity(),
                 double maximum = std::numeric_limits<double>::infinity());
bool prop_int(const char* label, int& value, float speed, int minimum,
              int maximum, const char* help = nullptr);
bool prop_check(const char* label, bool& value, const char* help = nullptr);
bool prop_text(const char* label, std::string& value, const char* help = nullptr);
// A coordinate stored in the model frame and shown in the program frame.
bool prop_coordinate(const char* label, double& stored, double offset,
                     const char* help = nullptr);
// A value the operation may override. Unset, it shows the inherited figure
// greyed; editing sets it; the reset button clears it.
bool prop_override(const char* label, std::optional<double>& value,
                   std::optional<double> inherited, float speed,
                   const char* format, const char* help = nullptr);
void prop_value(const char* label, const char* format, ...) IM_FMTARGS(2);
template <typename Enum>
bool prop_enum(const char* label, Enum& value, const char* (*name)(Enum),
               const Enum* values, std::size_t count,
               const char* help = nullptr) {
  property_label(label, help);
  ImGui::PushID(label);
  bool changed = false;
  ImGui::SetNextItemWidth(-FLT_MIN);
  if (ImGui::BeginCombo("##value", name(value))) {
    for (std::size_t i = 0; i < count; ++i) {
      const bool selected = values[i] == value;
      if (ImGui::Selectable(name(values[i]), selected)) {
        value = values[i];
        changed = true;
      }
      if (selected) ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
  }
  ImGui::PopID();
  return changed;
}
// A library choice by ID. A missing ID is shown as missing, never as the
// first entry.
template <typename Entries, typename Name>
bool prop_library(const char* label, std::string& id, const Entries& entries,
                  Name name, const char* help = nullptr) {
  property_label(label, help);
  ImGui::PushID(label);
  std::string preview = "Choose...";
  bool found = false;
  for (const auto& entry : entries)
    if (entry.id == id) {
      preview = name(entry);
      found = true;
    }
  if (!found && !id.empty()) preview = "Missing: " + id;
  bool changed = false;
  ImGui::SetNextItemWidth(-FLT_MIN);
  if (!found && !id.empty())
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.45f, 0.4f, 1.0f));
  const bool open = ImGui::BeginCombo("##value", preview.c_str());
  if (!found && !id.empty()) ImGui::PopStyleColor();
  if (open) {
    for (const auto& entry : entries) {
      const bool selected = entry.id == id;
      if (ImGui::Selectable(name(entry), selected)) {
        id = entry.id;
        changed = true;
      }
      if (selected) ImGui::SetItemDefaultFocus();
    }
    if (entries.empty()) ImGui::TextDisabled("Nothing available");
    ImGui::EndCombo();
  }
  ImGui::PopID();
  return changed;
}

// ---- Dialogs and job actions ------------------------------------------------
void confirm(App& app, std::string title, std::string text, std::string accept,
             std::function<void()> action);
void open_file_dialog(App& app, std::string title,
                      std::vector<std::string> extensions, bool saving,
                      std::filesystem::path directory, std::string name,
                      std::function<void(const std::filesystem::path&)> accept);
void draw_dialogs(App& app);
bool job_dirty(const App& app);
// Runs `action` straight away, or after asking when there are unsaved changes.
void unless_unsaved(App& app, const char* doing, std::function<void()> action);
void new_job(App& app);
void open_job(App& app);
void open_recent_job(App& app, const std::filesystem::path& file);
void save_job(App& app, bool choose_file);
void export_gcode(App& app, bool selected_only);
void request_quit(App& app);
void choose_part_model(App& app);
void choose_stock_model(App& app);
void handle_shortcuts(App& app);
std::filesystem::path job_directory(const App& app);

// ---- Planning state -----------------------------------------------------------
void invalidate_plan(App& app);
int first_out_of_date(const App& app, int last);
std::size_t stale_operations(const App& app);
PlanState plan_state(const App& app, std::size_t index);
bool out_of_date(PlanState state);
std::string job_signature(const App& app);
std::string operation_signature(const App& app, std::size_t index,
                                const std::string& job_part);
void add_operation_menu(App& app);
cam::OperationPlan unplanned(const cam::Operation& operation);
void erase_operation(App& app, int index);
void move_operation(App& app, int from, int to);
void align_plan(App& app);
std::vector<std::size_t> selected_operation_indices(const App& app,
                                                     bool fallback_to_focus = true);
bool plannable(const App& app, const cam::Operation& operation);
void plan_out_of_date(App& app);
void depth_defaults(App& app, cam::Operation& operation);
double work_top(const App& app);

// ---- Panels -------------------------------------------------------------------
void setup_panel(App& app);
void operations_panel(App& app);
void operation_panel(App& app);
void plan_buttons(App& app);
void plan_summary(App& app);
void transport_bar(App& app);
void messages_drawer(App& app);
void status_bar(App& app);
void tool_library_window(App& app);
void material_library_window(App& app);
void settings_window(App& app);
void gcode_preview_window(App& app);
bool layout_splitter(App& app, const char* id, bool vertical, float extent,
                     float& delta);

// ---- Views --------------------------------------------------------------------
enum class ViewKind { Setup, Pass, Result };
void draw_setup(App& app);
void draw_canvas(App& app);
void draw_result(App& app);
// Display options and the colour legend, over the corner of a view.
void view_overlay(App& app, ViewKind kind, ImVec2 top_left, ImVec2 size);
// A caption line across the top of a view, kept clear of the overlay buttons.
void view_caption(const App& app, ImDrawList* list, ImVec2 top_left, ImVec2 size, int line,
                  ImU32 colour, const char* text);

Paths64 drawable_region(const Paths64& region, double spacing);
Paths64 fill_mesh(const Paths64& region, std::size_t* failures = nullptr);
void draw_paths(ImDrawList* list, const Paths64& paths, const Transform& to_screen,
                ImU32 colour, float width);
void fill_paths(ImDrawList* list, const Paths64& paths, const Transform& to_screen,
                ImU32 solid, std::size_t* failures = nullptr);
ImU32 engagement_colour(double engagement);
ImU32 feed_colour(double feed, double lowest, double highest);
ImU32 blend(ImU32 low, ImU32 high, float t);
ImU32 limit_colour(cam::FeedLimit limit);
ImU32 move_colour(wave::MoveKind kind);
ImU32 chain_colour(std::size_t chain, std::size_t layer, std::size_t layer_count,
                   int alpha = 255);
std::string clock_text(double seconds);
const char* motion_phase_name(const cam::MotionCursor& cursor);
void fit_view(App& app, ImVec2 canvas);
// The feed range drawn by the Feed colouring, over the passes on show.
void feed_range(const App& app, double& lowest, double& highest);

void append_mesh(std::vector<Facet>& facets, std::vector<ImVec2>& pool,
                 const wave::Mesh& mesh, const Projection& projection,
                 const Vec3& light, const float colour[3], int alpha,
                 bool wireframe = false);
void append_slab(std::vector<Facet>& facets, std::vector<ImVec2>& pool,
                 const Paths64& remaining, const Paths64& lid_mesh,
                 const Paths64& lid_rings, double bottom, double top,
                 const Projection& projection, const Vec3& light,
                 const float wall_colour[3], const float face_colour[3], ImU32 edge,
                 int alpha = 255, bool wireframe = false);
void paint_facets(ImDrawList* list, std::vector<Facet>& facets,
                  const std::vector<ImVec2>& pool);

void orbit_controls(App& app, bool hovered);
bool begin_scene(App& app, OrbitScene& scene, const char* id);
void scene_message(const App& app, OrbitScene& scene, const char* message);
void end_scene(OrbitScene& scene);
void aim_scene(App& app, OrbitScene& scene, const Clipper2Lib::Rect64& extent,
               double lowest, double highest);
void paint_scene(OrbitScene& scene);
void draw_origin_gizmo(const App& app, const OrbitScene& scene);
void aim_camera(App& app, const Vec3& direction);
CubeFace cube_face(int axis, int sign);
ImVec2 cube_centre(ImVec2 top_left, ImVec2 size);
void view_cube(App& app, ImDrawList* list, ImVec2 top_left, ImVec2 size,
               bool hovered);
const char* cube_face_name(int axis, int sign);

// ---- Tools ------------------------------------------------------------------
void import_tools(App& app, const std::filesystem::path& file);
void reload_tools(App& app);

}  // namespace tinglecam_gui
