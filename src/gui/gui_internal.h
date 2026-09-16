#pragma once

#include "cam/config.h"
#include "cam/feeds.h"
#include "cam/job.h"
#include "cam/parametric.h"
#include "cam/post.h"
#include "cam/stock.h"
#include "rendering/prism_renderer.h"
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
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace tinglecam_gui {

struct Transform {
  ImVec2 origin;
  float scale = 1.0f;
  ImVec2 operator()(const Clipper2Lib::Point64& point) const {
    return {origin.x + static_cast<float>(wave::to_world(point.x)) * scale,
            origin.y - static_cast<float>(wave::to_world(point.y)) * scale};
  }
};

Paths64 drawable_region(const Paths64& region, double spacing);
Paths64 drawable_surface(const Paths64& region, double spacing);
Paths64 fill_mesh(const Paths64& region, std::size_t* failures = nullptr);
void draw_paths(ImDrawList* list, const Paths64& paths, const Transform& to_screen,
                ImU32 colour, float width);
void fill_paths(ImDrawList* list, const Paths64& paths, const Transform& to_screen,
                ImU32 solid, std::size_t* failures = nullptr);
ImU32 engagement_colour(const wave::MaterialStep& step, int display);
ImU32 blend(ImU32 low, ImU32 high, float t);
ImU32 limit_colour(cam::FeedLimit limit);
ImU32 move_colour(wave::MoveKind kind);
ImU32 chain_colour(std::size_t chain, std::size_t layer, int alpha = 255);
std::size_t segment_at(const std::vector<cam::FeedSegment>& segments,
                       double distance);
std::string clock_text(double seconds);
const char* motion_phase_name(const cam::MotionCursor& cursor);
void fit_view(App& app, ImVec2 canvas);

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
void append_cutter(std::vector<Facet>& facets, std::vector<ImVec2>& pool,
                   const Vec3& centre, float radius, float bottom, float top,
                   const Projection& projection, const Vec3& light);
void paint_facets(ImDrawList* list, std::vector<Facet>& facets,
                  const std::vector<ImVec2>& pool);
void append_models(const App& app, StackModels which, std::vector<Facet>& facets,
                   std::vector<ImVec2>& pool, const Projection& projection,
                   const Vec3& light, double spacing, int slice_grid);

void orbit_controls(App& app, bool hovered);
bool begin_scene(App& app, OrbitScene& scene, const char* id);
void scene_message(OrbitScene& scene, const char* message);
void end_scene(OrbitScene& scene);
bool scene_button(OrbitScene& scene, const char* label, float from_right,
                  float from_top = 8.0f);
void aim_scene(App& app, OrbitScene& scene, const Clipper2Lib::Rect64& extent,
               double lowest, double highest);
void paint_scene(OrbitScene& scene);
void draw_origin_gizmo(const App& app, const OrbitScene& scene);

void invalidate_plan(App& app);
int first_out_of_date(const App& app, int last);
std::size_t stale_operations(const App& app);
PlanState plan_state(const App& app, std::size_t index);
bool out_of_date(PlanState state);
std::string job_signature(const App& app);
cam::OperationPlan unplanned(const cam::Operation& operation);
cam::Operation make_operation(App& app, cam::OperationMode mode);
void erase_operation(App& app, int index);
void swap_operations(App& app, int a, int b);
void align_plan(App& app);
void origin_panel(App& app);
void job_panel(App& app);
void operations_list_panel(App& app);
void operation_settings_panel(App& app);
void feeds_panel(App& app);
void plan_buttons(App& app);
void plan_panel(App& app);
void output_panel(App& app);
void library_panel(App& app);
void settings_panel(App& app);
void view_panel(App& app);

template <typename Enum>
bool enum_combo(const char* label, Enum& value, const char* (*name)(Enum),
                const Enum* values, std::size_t count) {
  bool changed = false;
  if (ImGui::BeginCombo(label, name(value))) {
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
  return changed;
}

bool origin_field(const char* label, double& stored, double origin);
void text_field(const char* label, std::string& value,
                std::size_t capacity = 128);

}  // namespace tinglecam_gui
