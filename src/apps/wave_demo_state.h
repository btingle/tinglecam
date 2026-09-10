#pragma once

#include "wave/analysis.h"
#include "wave/geometry.h"
#include "wave/mesh_slicer.h"
#include "wave/topology.h"

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <imgui.h>

namespace wave_demo {

using wave::Path64;
using wave::Paths64;

struct ViewState {
  ImVec2 pan{0.0f, 0.0f};
  float zoom = 1.0f;
  bool needs_fit = true;
};

struct OrbitState {
  float yaw = 0.75f;
  float pitch = 0.55f;
  float zoom = 1.0f;
};

struct DecisionEntryView {
  wave::WaveSeed seed;
  std::size_t group = 0;
  bool retained = true;
};

struct AppState {
  wave::SvgGeometry svg;
  Paths64 subject_paths;
  Paths64 sliced_stock;
  wave::MeshSlicer subject_slicer;
  wave::MeshSlicer stock_slicer;
  wave::MeshSlice subject_slice;
  wave::MeshSlice stock_slice;
  wave::WaveResult result;
  wave::FrontLineage lineage;
  wave::LayerTopology topology;
  wave::WaveResult standard_result;
  wave::FrontLineage standard_lineage;
  wave::LayerTopology standard_topology;
  wave::WavefrontAnalysis analysis;
  std::optional<wave::Rect64> auto_stock_subject_bounds;
  Paths64 raw_subject_outline;
  Paths64 raw_stock_outline;
  std::array<char, 512> subject_mesh_path{};
  std::array<char, 512> stock_mesh_path{};
  float slice_z = 0.0f;
  float slice_min = 0.0f;
  float slice_max = 1.0f;
  int query_grid = 32;
  float allowance = 150.0f;
  float stepover = 80.0f;
  bool rounded_waves = true;
  float subject_inflation = 0.0f;
  float stock_inflation = 0.0f;
  float lead_in_distance = 40.0f;
  bool lead_in = true;
  int selected_iteration = -1;
  bool show_all_iterations = true;
  bool show_subject = true;
  bool show_stock = true;
  bool show_leading_fronts = true;
  bool show_original_boundaries = true;
  bool show_parent_links = true;
  bool show_event_markers = true;
  bool show_nodes = true;
  bool show_entry_choices = true;
  bool show_remaining_domain = true;
  bool animate = false;
  bool decision_play = false;
  bool analysis_dirty = true;
  int decision_step = 0;
  float decision_speed = 1.0f;
  int max_analysis_layers = 128;
  float tiny_split_area_factor = static_cast<float>(
      wave::kDefaultTinySplitAreaFactor);
  std::string decision_caption;
  std::vector<DecisionEntryView> decision_entries;
  Paths64 decision_domain;
  float animation_speed = 4.0f;
  ViewState view;
  OrbitState orbit;
  std::string error;
  std::string analysis_error;
};

Paths64 offset_boundaries(const Paths64& input, float amount,
                          std::string_view description);
Paths64 allowance_stock(const wave::Rect64& bounds, float allowance);
wave::Rect64 automatic_stock_bounds(const AppState& app);
wave::Rect64 mesh_xy_bounds(const wave::Mesh& mesh);
wave::LayerTopology display_topology(const wave::FrontLineage& lineage);
void recompute(AppState& app);
int decision_frame_count(const AppState& app);
std::string apply_decision_frame(AppState& app);
ImU32 iteration_color(int index, int count, int alpha = 225);
ImU32 node_color(std::size_t index, int alpha = 230);
ImU32 event_color(wave::LineageEventKind kind, int alpha = 255);
const char* event_short_name(wave::LineageEventKind kind);
void update_preview_outlines(AppState& app);
void reslice(AppState& app, bool analyze = true);
void build_analysis(AppState& app);
void update_slice_range(AppState& app, bool center_height);
void load_mesh(AppState& app, bool stock, bool analyze = true);

void draw_canvas(AppState& app);
void draw_mesh_viewer(AppState& app);
void controls(AppState& app, const std::filesystem::path& input);

struct StartupInputs {
  std::filesystem::path svg;
  std::filesystem::path subject_mesh;
  std::filesystem::path stock_mesh;
  std::filesystem::path screenshot;
  std::optional<float> subject_inflation;
  std::optional<float> stock_inflation;
  std::optional<float> lead_in_distance;
  std::optional<bool> lead_in;
  std::optional<float> allowance;
  std::optional<float> stepover;
  std::optional<bool> rounded_waves;
  std::optional<float> slice_z;
  std::optional<int> query_grid;
  std::optional<int> analysis_step;
  std::optional<int> analysis_layers;
  std::optional<float> tiny_split_area_factor;
  bool help = false;
  bool dump_analysis = false;
};

StartupInputs resolve_inputs(int argc, char** argv);
void print_help(const char* executable);
void save_screenshot(const std::filesystem::path& filename, int width,
                     int height);

}  // namespace wave_demo
