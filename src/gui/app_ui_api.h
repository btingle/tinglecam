#pragma once

#include "cam/config.h"
#include "cam/job.h"
#include "cam/post.h"
#include "gui_types.h"

#include <clipper2/clipper.h>
#include <imgui.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace tinglecam_gui {

class PrismView;

#include "state.h"

void advance_camera(App& app, double seconds);
void aim_camera(App& app, const Vec3& direction);
CubeFace cube_face(int axis, int sign);
ImVec2 cube_centre(ImVec2 top_left, ImVec2 size);
void view_cube(App& app, ImDrawList* list, ImVec2 top_left, ImVec2 size,
               bool hovered);
void tool_builder(App& app);

extern PrismView prism_view;
void release_prism_view();

void bootstrap(App& app);
void apply_ui_scale(App& app);
bool save_settings(App& app);
bool layout_splitter(App& app, const char* id, bool vertical, float extent,
                     float& delta);
void settings_panel(App& app);
void load_meshes(App& app);
void load_job_into(App& app, const std::filesystem::path& file);
void depth_defaults(App& app, cam::Operation& operation);
void insert_operation(App& app, int index, cam::Operation operation);
std::vector<std::size_t> selected_operation_indices(const App& app,
                                                     bool fallback_to_focus = true);
void start_plan(App& app, int first, int last);
void refresh_plan_states(App& app);
void select_operation(App& app, int index);
const cam::LevelPlan* current_level(const App& app);
double level_length(const cam::LevelPlan* level);
void sync_playback_time(App& app);
void refresh_material(App& app);
void advance_playback(App& app, double seconds);
void left_panel(App& app);
void right_panel(App& app);
void draw_setup(App& app);
void draw_canvas(App& app);
void draw_result(App& app);
void import_tools(App& app);
void reload_tools(App& app);
const char* cube_face_name(int axis, int sign);

}  // namespace tinglecam_gui
