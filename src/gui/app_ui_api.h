#pragma once

#include "cam/config.h"
#include "cam/job.h"
#include "cam/post.h"
#include "cam/surface_finishing.h"
#include "cam/holes.h"
#include "gui_types.h"

#include <clipper2/clipper.h>
#include <imgui.h>

#include <algorithm>
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

#include "state.h"

// Startup and appearance.
void bootstrap(App& app);
void load_ui_fonts(App& app);
void apply_ui_theme(App& app);
void apply_ui_scale(App& app);
bool save_settings(App& app);

// The job.
void load_job_into(App& app, const std::filesystem::path& file, bool models = true);
void load_meshes(App& app);
void start_model_loading(App& app, bool reset_setup = false);
void start_job_loading(App& app, const std::filesystem::path& file);
void start_model_loading(App& app, const std::filesystem::path& file, bool part);
bool take_model_load(App& app);
// Fills machine and material from the user's defaults when the job has none.
void adopt_default_setup(App& app);
cam::Operation make_operation(App& app, cam::OperationMode mode);
void insert_operation(App& app, int index, cam::Operation operation);
void mark_job_saved(App& app);
std::string window_title(const App& app);

// Planning.
void start_plan(App& app, int first, int last);
// Takes a finished planning run into the plan on screen. `screenshot` leaves
// the view the command line chose alone.
void take_plan_result(App& app, cam::PartialPlan produced, bool screenshot);
void refresh_plan_states(App& app);
void select_operation(App& app, int index);

// Playback.
const cam::LevelPlan* current_level(const App& app);
double level_length(const cam::LevelPlan* level);
void sync_playback_time(App& app);
void refresh_material(App& app);
void advance_playback(App& app, double seconds);
void advance_camera(App& app, double seconds);

// The whole window, once a frame.
void draw_main_window(App& app);
void release_render_resources();

}  // namespace tinglecam_gui
