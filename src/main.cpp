#include "analysis.h"
#include "geometry.h"
#include "mesh_slicer.h"
#include "topology.h"

#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

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
  // Full-subject XY bounds used by the automatic rectangular stock. Unlike
  // subject_paths, this never changes as a mesh is sliced at different Zs.
  std::optional<wave::Rect64> auto_stock_subject_bounds;
  // The boundaries as sliced, before inflation, so an offset can be seen
  // against what it was applied to.
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
  float tiny_split_area_factor =
      static_cast<float>(wave::kDefaultTinySplitAreaFactor);
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
                          std::string_view description) {
  Paths64 result = Clipper2Lib::Union(input, Clipper2Lib::FillRule::NonZero);
  const std::int64_t delta = wave::to_integer(amount);
  if (delta != 0) {
    result = Clipper2Lib::InflatePaths(
        result, static_cast<double>(delta), Clipper2Lib::JoinType::Round,
        Clipper2Lib::EndType::Polygon);
    result = Clipper2Lib::Union(result, Clipper2Lib::FillRule::NonZero);
  }
  if (result.empty())
    throw std::runtime_error(std::string(description) +
                             " offset removes all geometry");
  return result;
}

Paths64 allowance_stock(const wave::Rect64& bounds, float allowance) {
  if (!(allowance > 0.0f)) throw std::runtime_error("Allowance must be positive");
  const std::int64_t margin = std::max<std::int64_t>(1, wave::to_integer(allowance));
  return Paths64{{
      {bounds.left - margin, bounds.top - margin},
      {bounds.right + margin, bounds.top - margin},
      {bounds.right + margin, bounds.bottom + margin},
      {bounds.left - margin, bounds.bottom + margin},
  }};
}

wave::Rect64 automatic_stock_bounds(const AppState& app) {
  if (app.auto_stock_subject_bounds) return *app.auto_stock_subject_bounds;
  throw std::runtime_error(
      "Automatic stock has no fixed full-subject XY bounds");
}

wave::Rect64 mesh_xy_bounds(const wave::Mesh& mesh) {
  wave::Rect64 result;
  result.left = wave::to_integer(mesh.bounds.lower.x);
  result.top = wave::to_integer(mesh.bounds.lower.y);
  result.right = wave::to_integer(mesh.bounds.upper.x);
  result.bottom = wave::to_integer(mesh.bounds.upper.y);
  return result;
}

wave::LayerTopology display_topology(const wave::FrontLineage& lineage) {
  try {
    return wave::build_layer_topology(lineage);
  } catch (const std::runtime_error&) {
    // A rejected split can put parallel fronts in one node, for which endpoint
    // rails are not unique. The swept bands are still the exact region that
    // node owns and make the more useful diagnostic highlight.
    wave::LayerTopology topology = wave::build_layer_topology(lineage, false);
    for (wave::FrontNode& node : topology.nodes) {
      wave::Paths64 footprint;
      for (const wave::FrontId2D id : node.fronts) {
        if (id.generation >= lineage.generations.size()) continue;
        const wave::FrontGeneration& generation =
            lineage.generations[id.generation];
        if (id.index >= generation.fronts.size()) continue;
        const std::size_t band = generation.fronts[id.index].band;
        if (band >= generation.bands.size()) continue;
        footprint.insert(footprint.end(), generation.bands[band].begin(),
                         generation.bands[band].end());
      }
      node.footprint =
          Clipper2Lib::Union(footprint, Clipper2Lib::FillRule::NonZero);
    }
    return topology;
  }
}

ImU32 iteration_color(int index, int count, int alpha = 225) {
  const float t = count <= 1 ? 0.0f : static_cast<float>(index) / (count - 1);
  const ImVec4 cyan{0.10f, 0.78f, 0.94f, alpha / 255.0f};
  const ImVec4 orange{1.00f, 0.45f, 0.12f, alpha / 255.0f};
  return ImGui::ColorConvertFloat4ToU32(ImVec4(
      cyan.x + (orange.x - cyan.x) * t,
      cyan.y + (orange.y - cyan.y) * t,
      cyan.z + (orange.z - cyan.z) * t,
      alpha / 255.0f));
}

ImU32 node_color(std::size_t index, int alpha = 230) {
  const float hue = std::fmod(static_cast<float>(index) * 0.61803398875f, 1.0f);
  ImVec4 color = ImColor::HSV(hue, 0.60f, 0.95f);
  color.w = alpha / 255.0f;
  return ImGui::ColorConvertFloat4ToU32(color);
}

ImU32 event_color(wave::LineageEventKind kind, int alpha = 255) {
  switch (kind) {
    case wave::LineageEventKind::Split: return IM_COL32(255, 91, 177, alpha);
    case wave::LineageEventKind::Merge: return IM_COL32(72, 229, 139, alpha);
    case wave::LineageEventKind::MergeTermination:
      return IM_COL32(94, 174, 255, alpha);
    case wave::LineageEventKind::Termination: return IM_COL32(255, 92, 73, alpha);
    case wave::LineageEventKind::Birth: return IM_COL32(255, 205, 74, alpha);
  }
  return IM_COL32(255, 255, 255, alpha);
}

const char* event_short_name(wave::LineageEventKind kind) {
  switch (kind) {
    case wave::LineageEventKind::Split: return "S";
    case wave::LineageEventKind::Merge: return "M";
    case wave::LineageEventKind::MergeTermination: return "MT";
    case wave::LineageEventKind::Termination: return "T";
    case wave::LineageEventKind::Birth: return "B";
  }
  return "?";
}

int decision_frame_count(const AppState& app) {
  std::size_t count = 1;
  for (const wave::WavefrontDecisionLayer& layer : app.analysis.layers)
    count += layer.trials.size() + 2;
  return static_cast<int>(std::min<std::size_t>(
      count, static_cast<std::size_t>(std::numeric_limits<int>::max())));
}

void set_decision_layer(AppState& app,
                        const wave::WavefrontDecisionLayer& layer,
                        const std::vector<std::size_t>* retained) {
  app.decision_domain = layer.remaining_domain;
  app.decision_entries.clear();
  app.decision_entries.reserve(layer.entries.size());

  std::vector<std::size_t> entry_nodes;
  for (std::size_t node = 0; node < layer.baseline_topology.nodes.size(); ++node) {
    const wave::FrontNode& item = layer.baseline_topology.nodes[node];
    if (item.kind == wave::FrontNode::Kind::Propagation &&
        item.evaluation_order == 0 && !item.fronts.empty())
      entry_nodes.push_back(node);
  }
  std::vector<std::size_t> entry_groups(layer.entries.size());
  for (std::size_t entry = 0; entry < entry_groups.size(); ++entry)
    entry_groups[entry] = entry;
  for (std::size_t group = 0; group < layer.groups.size(); ++group)
    for (const wave::NodeEntry& member : layer.groups[group].entries) {
      const auto found = std::find(entry_nodes.begin(), entry_nodes.end(),
                                   member.node);
      if (found != entry_nodes.end())
        entry_groups[static_cast<std::size_t>(found - entry_nodes.begin())] =
            group;
    }

  for (std::size_t entry = 0; entry < layer.entries.size(); ++entry) {
    const bool keep = !retained ||
                      std::find(retained->begin(), retained->end(), entry) !=
                          retained->end();
    app.decision_entries.push_back(
        {layer.entries[entry], entry_groups[entry], keep});
  }
}

std::string apply_decision_frame(AppState& app) {
  const int count = decision_frame_count(app);
  app.decision_step = std::clamp(app.decision_step, 0, std::max(0, count - 1));
  if (app.decision_step == 0 || app.analysis.layers.empty()) {
    app.result = app.standard_result;
    app.lineage = app.standard_lineage;
    app.topology = app.standard_topology;
    app.decision_entries.clear();
    app.decision_domain = app.standard_result.domain;
    return "Initial simultaneous analysis";
  }
  std::size_t cursor = static_cast<std::size_t>(app.decision_step - 1);
  for (const wave::WavefrontDecisionLayer& layer : app.analysis.layers) {
    if (cursor == 0) {
      app.result = layer.baseline_wave;
      app.lineage = layer.baseline_lineage;
      app.topology = layer.baseline_topology;
      set_decision_layer(app, layer, nullptr);
      return "Layer " + std::to_string(layer.layer) +
             ": simultaneous baseline (" +
             std::to_string(layer.baseline_topology.nodes.size()) + " nodes)";
    }
    --cursor;
    if (cursor < layer.trials.size()) {
      const wave::WavefrontStrategyTrial& trial = layer.trials[cursor];
      app.result = trial.wave;
      app.lineage = trial.lineage;
      app.topology = trial.topology;
      set_decision_layer(app, layer, &trial.retained_entries);
      return "Layer " + std::to_string(layer.layer) +
             ": longest entry per connected group (" +
             std::to_string(trial.candidate_significant_node_count) + " vs " +
             std::to_string(trial.previous_significant_node_count) +
             " significant; " + std::to_string(trial.candidate_node_count) +
             " vs " + std::to_string(trial.previous_node_count) + " raw, " +
             (trial.accepted ? "accepted" : "rejected") + ")";
    }
    cursor -= layer.trials.size();
    if (cursor == 0) {
      app.result = layer.selected_wave;
      app.lineage = layer.selected_lineage;
      app.topology = layer.selected_topology;
      set_decision_layer(app, layer, &layer.retained_entries);
      return "Layer " + std::to_string(layer.layer) +
             ": selected strategy (" +
             std::to_string(layer.selected_topology.nodes.size()) + " nodes)";
    }
    --cursor;
  }
  app.result = app.standard_result;
  app.lineage = app.standard_lineage;
  app.topology = app.standard_topology;
  app.decision_entries.clear();
  app.decision_domain = app.standard_result.domain;
  return "Initial simultaneous analysis";
}

void recompute(AppState& app) {
  try {
    if (app.subject_paths.empty())
      throw std::runtime_error("The current subject slice is empty");
    const Paths64 subject = offset_boundaries(
        app.subject_paths, app.subject_inflation, "Subject");
    Paths64 stock;
    if (app.stock_slicer.empty())
      stock = allowance_stock(automatic_stock_bounds(app), app.allowance);
    else {
      if (app.sliced_stock.empty()) throw std::runtime_error("The current stock slice is empty");
      stock = app.sliced_stock;
    }
    app.raw_stock_outline = Clipper2Lib::Union(stock, Clipper2Lib::FillRule::NonZero);
    app.raw_subject_outline =
        Clipper2Lib::Union(app.subject_paths, Clipper2Lib::FillRule::NonZero);
    stock = offset_boundaries(stock, app.stock_inflation, "Stock");
    const double softening = app.rounded_waves ? app.stepover : 0.0;
    const double lead_in = app.lead_in ? app.lead_in_distance : 0.0;
    const auto build_standard_fallback = [&] {
      app.standard_result =
          wave::build_wavefront(subject, stock, app.stepover, softening, lead_in);
      app.standard_lineage = wave::build_front_lineage(
          app.standard_result, app.tiny_split_area_factor);
      app.standard_topology = display_topology(app.standard_lineage);
    };
    try {
      app.analysis = wave::analyze_wavefront(subject, stock, app.stepover,
                                             softening, lead_in, 2000,
                                             static_cast<std::size_t>(
                                                 app.max_analysis_layers), {},
                                             wave::kDefaultSignificantNodeAreaFraction,
                                             app.tiny_split_area_factor);
      if (app.analysis.layers.empty()) {
        build_standard_fallback();
      } else {
        // Layer zero is the same all-entry propagation the viewer previously
        // built separately. Reuse it, restoring the input metadata omitted by
        // the isolated-domain rollout, and build footprints only for this
        // primary display graph.
        const wave::WavefrontDecisionLayer& first = app.analysis.layers.front();
        app.standard_result = first.baseline_wave;
        app.standard_result.subject = app.analysis.seed.subject;
        app.standard_result.stock = app.analysis.seed.stock;
        app.standard_result.stock_boundary = app.analysis.seed.stock_boundary;
        app.standard_result.domain = app.analysis.seed.domain;
        app.standard_result.lead_in = app.analysis.seed.lead_in;
        app.standard_result.lead_in_generations =
            app.analysis.seed.lead_in_generations;
        app.standard_lineage = first.baseline_lineage;
        app.standard_topology = display_topology(app.standard_lineage);
      }
      app.analysis_error.clear();
    } catch (const std::exception& error) {
      app.analysis = {};
      app.analysis_error = error.what();
      app.decision_step = 0;
      build_standard_fallback();
    }
    app.decision_caption = apply_decision_frame(app);
    if (app.selected_iteration < 0 && !app.lineage.events.empty())
      app.selected_iteration = static_cast<int>(app.lineage.events.front().generation);
    else
      app.selected_iteration = std::clamp(
          app.selected_iteration, 0,
          std::max(0, static_cast<int>(app.result.generations.size()) - 1));
    app.error.clear();
    app.analysis_dirty = false;
  } catch (const std::exception& error) {
    app.error = error.what();
    // The previous slice's analysis no longer describes what is on screen, and
    // selected_iteration is left negative, so drop both rather than draw stale
    // geometry against indices that no longer exist.
    app.result = {};
    app.lineage = {};
    app.topology = {};
    app.standard_result = {};
    app.standard_lineage = {};
    app.standard_topology = {};
    app.analysis = {};
    app.analysis_error.clear();
    app.decision_step = 0;
    app.decision_caption.clear();
    app.raw_subject_outline.clear();
    app.raw_stock_outline.clear();
    app.selected_iteration = -1;
    app.analysis_dirty = true;
  }
}

void update_preview_outlines(AppState& app) {
  if (app.subject_paths.empty()) return;
  app.raw_subject_outline =
      Clipper2Lib::Union(app.subject_paths, Clipper2Lib::FillRule::NonZero);
  const Paths64 stock = app.stock_slicer.empty()
                            ? allowance_stock(automatic_stock_bounds(app),
                                              app.allowance)
                            : app.sliced_stock;
  app.raw_stock_outline =
      Clipper2Lib::Union(stock, Clipper2Lib::FillRule::NonZero);
}

void reslice(AppState& app, bool analyze);

void build_analysis(AppState& app) {
  if (!app.subject_slicer.empty() || !app.stock_slicer.empty())
    reslice(app, /*analyze=*/false);
  update_preview_outlines(app);
  recompute(app);
}

void update_slice_range(AppState& app, bool center_height) {
  bool valid = false;
  float lower = 0.0f;
  float upper = 0.0f;
  for (const auto* slicer : {&app.subject_slicer, &app.stock_slicer}) {
    if (slicer->empty()) continue;
    const auto& bounds = slicer->mesh().bounds;
    if (!valid) {
      lower = bounds.lower.z;
      upper = bounds.upper.z;
      valid = true;
    } else {
      lower = std::min(lower, bounds.lower.z);
      upper = std::max(upper, bounds.upper.z);
    }
  }
  if (!valid) return;
  app.slice_min = lower;
  app.slice_max = std::max(lower + 1e-5f, upper);
  if (center_height) app.slice_z = (lower + upper) * 0.5f;
  app.slice_z = std::clamp(app.slice_z, app.slice_min, app.slice_max);
}

// Slicing and preview-outline refresh are cheap. Recursive wave analysis is
// optional so interactive edits can be collected for the explicit Build
// button instead of rebuilding on every slider tick.
void reslice(AppState& app, bool analyze = true) {
  try {
    if (!app.subject_slicer.empty()) {
      app.subject_slice = app.subject_slicer.slice(app.slice_z, app.query_grid);
      app.subject_paths = app.subject_slice.polygons;
    } else {
      app.subject_paths = app.svg.paths;
      app.subject_slice = {};
    }
    if (!app.stock_slicer.empty()) {
      app.stock_slice = app.stock_slicer.slice(app.slice_z, app.query_grid);
      app.sliced_stock = app.stock_slice.polygons;
    } else {
      app.stock_slice = {};
      app.sliced_stock.clear();
    }
    app.selected_iteration = -1;
    app.view.needs_fit = true;
    update_preview_outlines(app);
    if (analyze) recompute(app);
  } catch (const std::exception& error) {
    app.error = error.what();
  }
}

void load_mesh(AppState& app, bool stock, bool analyze = true) {
  try {
    auto& path_buffer = stock ? app.stock_mesh_path : app.subject_mesh_path;
    if (path_buffer[0] == '\0') throw std::runtime_error("Enter an OBJ or STL path first");
    wave::Mesh mesh = wave::load_triangle_mesh(path_buffer.data());
    auto& slicer = stock ? app.stock_slicer : app.subject_slicer;
    slicer.set_mesh(std::move(mesh));
    if (!stock) {
      const auto& bounds = slicer.mesh().bounds;
      const float span = std::max(bounds.upper.x - bounds.lower.x,
                                  bounds.upper.y - bounds.lower.y);
      if (span > 0.0f) {
        app.allowance = span * 0.05f;
        app.stepover = span * 0.025f;
        app.lead_in_distance = app.stepover * 0.5f;
      }
      app.auto_stock_subject_bounds = mesh_xy_bounds(slicer.mesh());
    }
    update_slice_range(app, true);
    reslice(app, analyze);
  } catch (const std::exception& error) {
    app.error = error.what();
  }
}

struct CanvasTransform {
  ImVec2 origin;
  float pixels_per_unit = 1.0f;

  ImVec2 point(const Clipper2Lib::Point64& value) const {
    return {origin.x + static_cast<float>(wave::to_world(value.x)) * pixels_per_unit,
            origin.y + static_cast<float>(wave::to_world(value.y)) * pixels_per_unit};
  }
};

void draw_paths(ImDrawList* draw_list, const Paths64& paths,
                const CanvasTransform& transform, ImU32 color, float width) {
  for (const auto& path : paths) {
    if (path.size() < 2) continue;
    for (std::size_t i = 0; i < path.size(); ++i) {
      draw_list->AddLine(transform.point(path[i]),
                         transform.point(path[(i + 1) % path.size()]), color, width);
    }
  }
}

// Dashes are what separate a reference outline from the geometry actually in
// play, which is drawn solid in a colour close by.
void draw_paths_dashed(ImDrawList* draw_list, const Paths64& paths,
                       const CanvasTransform& transform, ImU32 color, float width,
                       float dash_px = 7.0f) {
  for (const auto& path : paths) {
    if (path.size() < 2) continue;
    float phase = 0.0f;
    for (std::size_t i = 0; i < path.size(); ++i) {
      const ImVec2 from = transform.point(path[i]);
      const ImVec2 to = transform.point(path[(i + 1) % path.size()]);
      const float length = std::hypot(to.x - from.x, to.y - from.y);
      if (!(length > 0.0f)) continue;
      for (float at = phase; at < length; at += dash_px * 2.0f) {
        const float start = std::max(at, 0.0f);
        const float stop = std::min(at + dash_px, length);
        if (stop <= start) continue;
        const float t0 = start / length;
        const float t1 = stop / length;
        draw_list->AddLine({from.x + (to.x - from.x) * t0, from.y + (to.y - from.y) * t0},
                           {from.x + (to.x - from.x) * t1, from.y + (to.y - from.y) * t1},
                           color, width);
      }
      // Carry the gap across the vertex so the dashes do not restart on every
      // segment of an offset contour, which would draw it very nearly solid.
      phase = std::fmod(phase - length, dash_px * 2.0f);
      if (phase < 0.0f) phase += dash_px * 2.0f;
    }
  }
}

void draw_path(ImDrawList* draw_list, const Path64& path,
               const CanvasTransform& transform, ImU32 color, float width) {
  draw_paths(draw_list, Paths64{path}, transform, color, width);
}

// Open fronts stop at their endpoints; closed fronts wrap through their first
// point so their complete contour remains visible.
void draw_front(ImDrawList* draw_list, const wave::LeadingFront& front,
                const CanvasTransform& transform, ImU32 color, float width) {
  if (front.path.size() < 2) return;
  const std::size_t segment_count = front.closed ? front.path.size()
                                                  : front.path.size() - 1;
  for (std::size_t i = 0; i < segment_count; ++i) {
    draw_list->AddLine(transform.point(front.path[i]),
                       transform.point(front.path[(i + 1) % front.path.size()]),
                       color, width);
  }
  if (!front.closed) {
    draw_list->AddCircleFilled(transform.point(front.path.front()), width + 1.5f, color);
    draw_list->AddCircleFilled(transform.point(front.path.back()), width + 1.5f, color);
  }
}

// selected_iteration is -1 until a recompute succeeds. Casting that straight to
// size_t yields SIZE_MAX, so every lookup into the lineage goes through here.
std::optional<std::size_t> selected_generation(const AppState& app) {
  if (app.selected_iteration < 0) return std::nullopt;
  const auto index = static_cast<std::size_t>(app.selected_iteration);
  if (index >= app.lineage.generations.size()) return std::nullopt;
  return index;
}

void fit_view(AppState& app, ImVec2 canvas_size) {
  const Paths64& stock = !app.standard_result.stock.empty()
                             ? app.standard_result.stock
                         : !app.result.stock.empty() ? app.result.stock
                                                     : app.raw_stock_outline;
  if (stock.empty()) return;
  const auto bounds = Clipper2Lib::GetBounds(stock);
  const float width = static_cast<float>(wave::to_world(bounds.right - bounds.left));
  const float height = static_cast<float>(wave::to_world(bounds.bottom - bounds.top));
  if (width <= 0.0f || height <= 0.0f) return;
  app.view.zoom = 0.88f * std::min(canvas_size.x / width, canvas_size.y / height);
  const float center_x = static_cast<float>(wave::to_world(bounds.left + bounds.right)) * 0.5f;
  const float center_y = static_cast<float>(wave::to_world(bounds.top + bounds.bottom)) * 0.5f;
  app.view.pan = {-center_x * app.view.zoom, -center_y * app.view.zoom};
  app.view.needs_fit = false;
}

void draw_canvas(AppState& app) {
  ImGui::BeginChild("Canvas", ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  const ImVec2 canvas_min = ImGui::GetCursorScreenPos();
  ImVec2 canvas_size = ImGui::GetContentRegionAvail();
  canvas_size.x = std::max(canvas_size.x, 50.0f);
  canvas_size.y = std::max(canvas_size.y, 50.0f);
  ImGui::InvisibleButton("geometry-canvas", canvas_size,
                         ImGuiButtonFlags_MouseButtonLeft |
                             ImGuiButtonFlags_MouseButtonRight);
  const bool hovered = ImGui::IsItemHovered();
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  const ImVec2 canvas_max{canvas_min.x + canvas_size.x, canvas_min.y + canvas_size.y};
  draw_list->AddRectFilled(canvas_min, canvas_max, IM_COL32(17, 22, 29, 255));
  draw_list->PushClipRect(canvas_min, canvas_max, true);

  if (app.view.needs_fit) fit_view(app, canvas_size);
  const ImVec2 center{canvas_min.x + canvas_size.x * 0.5f,
                      canvas_min.y + canvas_size.y * 0.5f};

  if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
    app.view.pan.x += ImGui::GetIO().MouseDelta.x;
    app.view.pan.y += ImGui::GetIO().MouseDelta.y;
  }
  if (hovered && ImGui::GetIO().MouseWheel != 0.0f) {
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    const float factor = std::pow(1.15f, ImGui::GetIO().MouseWheel);
    const ImVec2 before{(mouse.x - center.x - app.view.pan.x) / app.view.zoom,
                        (mouse.y - center.y - app.view.pan.y) / app.view.zoom};
    app.view.zoom = std::clamp(app.view.zoom * factor, 0.001f, 1000.0f);
    app.view.pan = {mouse.x - center.x - before.x * app.view.zoom,
                    mouse.y - center.y - before.y * app.view.zoom};
  }

  const CanvasTransform transform{{center.x + app.view.pan.x,
                                   center.y + app.view.pan.y},
                                  app.view.zoom};

  // A subtle world-space grid gives scale and makes pan/zoom obvious.
  const float desired_grid_px = 75.0f;
  const float raw_grid = desired_grid_px / app.view.zoom;
  const float grid_power = std::pow(10.0f, std::floor(std::log10(raw_grid)));
  const float grid_unit = (raw_grid / grid_power < 2.0f ? 2.0f
                           : raw_grid / grid_power < 5.0f ? 5.0f : 10.0f) * grid_power;
  const float grid_px = grid_unit * app.view.zoom;
  const float offset_x = std::fmod(center.x + app.view.pan.x - canvas_min.x, grid_px);
  const float offset_y = std::fmod(center.y + app.view.pan.y - canvas_min.y, grid_px);
  for (float x = canvas_min.x + offset_x; x < canvas_max.x; x += grid_px)
    draw_list->AddLine({x, canvas_min.y}, {x, canvas_max.y}, IM_COL32(48, 57, 68, 100));
  for (float y = canvas_min.y + offset_y; y < canvas_max.y; y += grid_px)
    draw_list->AddLine({canvas_min.x, y}, {canvas_max.x, y}, IM_COL32(48, 57, 68, 100));

  if (app.show_stock) {
    const Paths64& stock = !app.standard_result.stock.empty()
                               ? app.standard_result.stock
                           : !app.result.stock.empty() ? app.result.stock
                                                       : app.raw_stock_outline;
    draw_paths(draw_list, stock, transform, IM_COL32(125, 138, 152, 190), 2.0f);
    if (app.show_original_boundaries && app.stock_inflation != 0.0f)
      draw_paths_dashed(draw_list, app.raw_stock_outline, transform,
                        IM_COL32(125, 138, 152, 130), 1.5f);
  }

  if (app.show_remaining_domain && !app.decision_domain.empty())
    draw_paths_dashed(draw_list, app.decision_domain, transform,
                      IM_COL32(101, 188, 255, 135), 2.0f, 9.0f);

  const int count = static_cast<int>(app.result.generations.size());
  if (app.show_all_iterations) {
    for (int i = 0; i < count; ++i) {
      const bool selected = i == app.selected_iteration;
      draw_paths(draw_list, app.result.generations[i], transform,
                 iteration_color(i, count, selected ? 255 : 145),
                 selected ? 3.0f : 1.25f);
    }
  } else if (app.selected_iteration >= 0 && app.selected_iteration < count) {
    draw_paths(draw_list, app.result.generations[app.selected_iteration], transform,
               iteration_color(app.selected_iteration, count), 2.5f);
  }

  if (app.show_subject) {
    const Paths64& subject = !app.standard_result.subject.empty()
                                 ? app.standard_result.subject
                             : !app.result.subject.empty() ? app.result.subject
                                                           : app.raw_subject_outline;
    draw_paths(draw_list, subject, transform, IM_COL32(252, 226, 89, 255), 3.0f);
    if (app.show_original_boundaries && app.subject_inflation != 0.0f)
      draw_paths_dashed(draw_list, app.raw_subject_outline, transform,
                        IM_COL32(252, 226, 89, 150), 1.5f);
  }


  if (app.show_nodes) {
    for (std::size_t node = 0; node < app.topology.nodes.size(); ++node) {
      const wave::FrontNode& topology_node = app.topology.nodes[node];
      if (topology_node.kind == wave::FrontNode::Kind::MergeTermination)
        continue;
      bool selected = false;
      for (const wave::FrontId2D front : topology_node.fronts)
        selected |= static_cast<int>(front.generation) == app.selected_iteration;
      const ImU32 color = node_color(node, selected ? 255 : 115);
      if (!topology_node.footprint.empty()) {
        draw_paths(draw_list, topology_node.footprint, transform, color,
                   selected ? 4.0f : 2.0f);
      } else {
        for (const wave::FrontId2D id : topology_node.fronts) {
          if (id.generation >= app.lineage.generations.size() ||
              id.index >= app.lineage.generations[id.generation].fronts.size())
            continue;
          draw_front(draw_list,
                     app.lineage.generations[id.generation].fronts[id.index],
                     transform, color, selected ? 4.0f : 2.0f);
        }
      }
    }
  }

  const auto canvas_generation = selected_generation(app);
  if (app.show_leading_fronts && canvas_generation) {
    const std::size_t generation = *canvas_generation;
    if (generation > 0) {
      const auto& previous = app.lineage.generations[generation - 1].fronts;
      for (const auto& front : previous)
        draw_front(draw_list, front, transform, IM_COL32(183, 112, 255, 210), 3.0f);
    }
    const auto& current = app.lineage.generations[generation].fronts;
    for (const auto& front : current)
      draw_front(draw_list, front, transform, IM_COL32(90, 245, 174, 255), 4.0f);

    if (app.show_parent_links && generation > 0) {
      const auto& previous = app.lineage.generations[generation - 1].fronts;
      for (std::size_t child = 0; child < current.size(); ++child) {
        const ImVec2 child_position = transform.point(wave::representative_point(current[child]));
        for (const auto parent : current[child].parents) {
          if (parent >= previous.size()) continue;
          const ImVec2 parent_position =
              transform.point(wave::representative_point(previous[parent]));
          draw_list->AddLine(parent_position, child_position,
                             IM_COL32(232, 236, 241, 155), 1.5f);
          draw_list->AddCircleFilled(parent_position, 4.0f,
                                     IM_COL32(183, 112, 255, 255));
          draw_list->AddCircleFilled(child_position, 4.0f,
                                     IM_COL32(90, 245, 174, 255));
        }
      }
    }
  }

  if (app.show_event_markers) {
    for (const auto& event : app.lineage.events) {
      const ImVec2 position = transform.point(event.position);
      const ImU32 color = event_color(event.kind);
      draw_list->AddCircleFilled(position, 9.0f, IM_COL32(14, 18, 24, 235));
      draw_list->AddCircle(position, 9.0f, color, 0, 3.0f);
      draw_list->AddText({position.x - 3.5f, position.y - 7.0f}, color,
                         event_short_name(event.kind));
    }
  }

  if (app.show_entry_choices) {
    for (const DecisionEntryView& entry : app.decision_entries) {
      wave::LeadingFront front;
      front.path = entry.seed.path;
      front.closed = entry.seed.closed;
      const ImU32 color = entry.retained
                              ? node_color(entry.group, 255)
                              : IM_COL32(120, 126, 136, 210);
      draw_front(draw_list, front, transform, color,
                 entry.retained ? 6.0f : 3.0f);
      const ImVec2 marker = transform.point(wave::representative_point(front));
      if (entry.retained)
        draw_list->AddCircleFilled(marker, 5.0f, color);
      else {
        draw_list->AddLine({marker.x - 5.0f, marker.y - 5.0f},
                           {marker.x + 5.0f, marker.y + 5.0f}, color, 2.0f);
        draw_list->AddLine({marker.x - 5.0f, marker.y + 5.0f},
                           {marker.x + 5.0f, marker.y - 5.0f}, color, 2.0f);
      }
    }
  }

  draw_list->PopClipRect();
  ImGui::EndChild();
}

wave::Vec3f subtract(wave::Vec3f a, wave::Vec3f b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

float dot(wave::Vec3f a, wave::Vec3f b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

struct OrbitProjection {
  ImVec2 center;
  wave::Vec3f model_center;
  wave::Vec3f right;
  wave::Vec3f up;
  wave::Vec3f forward;
  float scale = 1.0f;

  ImVec2 point(wave::Vec3f value) const {
    const auto relative = subtract(value, model_center);
    return {center.x + dot(relative, right) * scale,
            center.y - dot(relative, up) * scale};
  }

  float depth(wave::Vec3f value) const {
    return dot(subtract(value, model_center), forward);
  }
};

bool viewer_bounds(const AppState& app, wave::Bounds3f& output) {
  output = {};
  for (const auto* slicer : {&app.subject_slicer, &app.stock_slicer}) {
    if (slicer->empty()) continue;
    const auto& bounds = slicer->mesh().bounds;
    if (!output.valid) output = bounds;
    else {
      output.lower.x = std::min(output.lower.x, bounds.lower.x);
      output.lower.y = std::min(output.lower.y, bounds.lower.y);
      output.lower.z = std::min(output.lower.z, bounds.lower.z);
      output.upper.x = std::max(output.upper.x, bounds.upper.x);
      output.upper.y = std::max(output.upper.y, bounds.upper.y);
      output.upper.z = std::max(output.upper.z, bounds.upper.z);
    }
  }
  return output.valid;
}

void draw_mesh_viewer(AppState& app) {
  ImGui::SeparatorText("3D slice view");
  const ImVec2 panel_min = ImGui::GetCursorScreenPos();
  const ImVec2 panel_size{ImGui::GetContentRegionAvail().x, 230.0f};
  ImGui::InvisibleButton("mesh-orbit-view", panel_size,
                         ImGuiButtonFlags_MouseButtonLeft);
  const bool hovered = ImGui::IsItemHovered();
  if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
    app.orbit.yaw += ImGui::GetIO().MouseDelta.x * 0.009f;
    app.orbit.pitch = std::clamp(app.orbit.pitch - ImGui::GetIO().MouseDelta.y * 0.009f,
                                 -1.45f, 1.45f);
  }
  if (hovered && ImGui::GetIO().MouseWheel != 0.0f)
    app.orbit.zoom = std::clamp(app.orbit.zoom *
                                   std::pow(1.13f, ImGui::GetIO().MouseWheel),
                               0.15f, 12.0f);

  ImDrawList* draw = ImGui::GetWindowDrawList();
  const ImVec2 panel_max{panel_min.x + panel_size.x, panel_min.y + panel_size.y};
  draw->AddRectFilled(panel_min, panel_max, IM_COL32(12, 17, 23, 255), 4.0f);
  draw->AddRect(panel_min, panel_max, IM_COL32(58, 68, 81, 255), 4.0f);
  draw->PushClipRect(panel_min, panel_max, true);

  wave::Bounds3f bounds;
  if (!viewer_bounds(app, bounds)) {
    draw->AddText({panel_min.x + 12.0f, panel_min.y + 12.0f},
                  IM_COL32(145, 155, 166, 255), "Load a subject or stock mesh");
    draw->PopClipRect();
    ImGui::TextDisabled("Left-drag: orbit   Wheel: zoom");
    return;
  }

  const wave::Vec3f center{(bounds.lower.x + bounds.upper.x) * 0.5f,
                           (bounds.lower.y + bounds.upper.y) * 0.5f,
                           (bounds.lower.z + bounds.upper.z) * 0.5f};
  const float extent = std::max({bounds.upper.x - bounds.lower.x,
                                 bounds.upper.y - bounds.lower.y,
                                 bounds.upper.z - bounds.lower.z, 1e-5f});
  const float cy = std::cos(app.orbit.yaw);
  const float sy = std::sin(app.orbit.yaw);
  const float cp = std::cos(app.orbit.pitch);
  const float sp = std::sin(app.orbit.pitch);
  OrbitProjection projection{
      {panel_min.x + panel_size.x * 0.5f, panel_min.y + panel_size.y * 0.5f},
      center, {-sy, cy, 0.0f}, {-sp * cy, -sp * sy, cp},
      {cp * cy, cp * sy, sp},
      std::min(panel_size.x, panel_size.y) * 0.76f / extent * app.orbit.zoom};

  std::array<wave::Vec3f, 4> plane{{
      {bounds.lower.x, bounds.lower.y, app.slice_z},
      {bounds.upper.x, bounds.lower.y, app.slice_z},
      {bounds.upper.x, bounds.upper.y, app.slice_z},
      {bounds.lower.x, bounds.upper.y, app.slice_z},
  }};
  std::array<ImVec2, 4> plane_points{};
  for (std::size_t i = 0; i < plane.size(); ++i) plane_points[i] = projection.point(plane[i]);
  draw->AddConvexPolyFilled(plane_points.data(), 4, IM_COL32(35, 213, 230, 35));

  struct RenderTriangle {
    ImVec2 points[3];
    float depth;
    ImU32 fill;
    ImU32 edge;
  };
  std::vector<RenderTriangle> render_triangles;
  auto append_mesh = [&](const wave::MeshSlicer& slicer, ImU32 fill, ImU32 edge) {
    if (slicer.empty()) return;
    const auto& triangles = slicer.mesh().triangles;
    const std::size_t stride = std::max<std::size_t>(1, triangles.size() / 6000);
    render_triangles.reserve(render_triangles.size() + (triangles.size() + stride - 1) / stride);
    for (std::size_t i = 0; i < triangles.size(); i += stride) {
      const auto& triangle = triangles[i];
      render_triangles.push_back({
          {projection.point(triangle.a), projection.point(triangle.b),
           projection.point(triangle.c)},
          (projection.depth(triangle.a) + projection.depth(triangle.b) +
           projection.depth(triangle.c)) / 3.0f,
          fill, edge});
    }
  };
  append_mesh(app.stock_slicer, IM_COL32(79, 141, 214, 50), IM_COL32(95, 158, 229, 75));
  append_mesh(app.subject_slicer, IM_COL32(246, 205, 78, 78), IM_COL32(255, 221, 104, 105));
  std::sort(render_triangles.begin(), render_triangles.end(),
            [](const RenderTriangle& a, const RenderTriangle& b) {
              return a.depth < b.depth;
            });
  for (const auto& triangle : render_triangles) {
    draw->AddTriangleFilled(triangle.points[0], triangle.points[1], triangle.points[2],
                            triangle.fill);
    draw->AddTriangle(triangle.points[0], triangle.points[1], triangle.points[2],
                      triangle.edge, 1.0f);
  }
  for (int i = 0; i < 4; ++i)
    draw->AddLine(plane_points[i], plane_points[(i + 1) % 4],
                  IM_COL32(52, 226, 238, 230), 2.0f);
  const ImVec2 label = projection.point({bounds.lower.x, bounds.lower.y, app.slice_z});
  draw->AddText({label.x + 5.0f, label.y + 3.0f}, IM_COL32(80, 236, 245, 255), "slice Z");
  draw->PopClipRect();
  ImGui::TextDisabled("Left-drag: orbit   Wheel: zoom");
}

void controls(AppState& app, const std::filesystem::path& input) {
  ImGui::TextUnformatted("Wavefront analysis viewer");
  ImGui::TextDisabled("2D fallback: %s", input.filename().string().c_str());
  draw_mesh_viewer(app);

  ImGui::SeparatorText("Models");
  ImGui::SetNextItemWidth(-1.0f);
  ImGui::InputTextWithHint("##subject-mesh", "Subject .obj or .stl",
                           app.subject_mesh_path.data(), app.subject_mesh_path.size());
  if (ImGui::Button("Load subject mesh")) {
    load_mesh(app, false, /*analyze=*/false);
    app.analysis_dirty = true;
  }
  if (!app.subject_slicer.empty()) {
    ImGui::SameLine();
    if (ImGui::Button("Use SVG subject")) {
      app.subject_slicer = wave::MeshSlicer{};
      app.auto_stock_subject_bounds = app.svg.bounds;
      update_slice_range(app, false);
      reslice(app, /*analyze=*/false);
      update_preview_outlines(app);
      app.analysis_dirty = true;
    }
  }
  ImGui::SetNextItemWidth(-1.0f);
  ImGui::InputTextWithHint("##stock-mesh", "Optional stock .obj or .stl",
                           app.stock_mesh_path.data(), app.stock_mesh_path.size());
  if (ImGui::Button("Load stock mesh")) {
    load_mesh(app, true, /*analyze=*/false);
    app.analysis_dirty = true;
  }
  if (!app.stock_slicer.empty()) {
    ImGui::SameLine();
    if (ImGui::Button("Use allowance stock")) {
      app.stock_slicer = wave::MeshSlicer{};
      update_slice_range(app, false);
      reslice(app, /*analyze=*/false);
      update_preview_outlines(app);
      app.analysis_dirty = true;
    }
  }

  if (!app.subject_slicer.empty() || !app.stock_slicer.empty()) {
    bool slice_changed = ImGui::SliderFloat("Slice Z", &app.slice_z,
                                            app.slice_min, app.slice_max, "%.4f");
    slice_changed |= ImGui::SliderInt("BVH query grid", &app.query_grid, 1, 128);
    if (slice_changed) app.analysis_dirty = true;
  }

  ImGui::SeparatorText("Wave analysis");

  bool changed = false;
  if (app.stock_slicer.empty())
    changed |= ImGui::DragFloat("Allowance", &app.allowance, 1.0f, 0.001f,
                                100000.0f, "%.3f");
  changed |= ImGui::DragFloat("Subject inflation", &app.subject_inflation, 0.1f,
                              -100000.0f, 100000.0f, "%+.3f");
  changed |= ImGui::DragFloat("Stock inflation", &app.stock_inflation, 0.1f,
                              -100000.0f, 100000.0f, "%+.3f");
  changed |= ImGui::DragFloat("Stepover", &app.stepover, 1.0f, 0.001f, 100000.0f, "%.3f");
  changed |= ImGui::Checkbox("Rounded waves", &app.rounded_waves);
  changed |= ImGui::Checkbox("Lead in from the air", &app.lead_in);
  if (app.lead_in)
    changed |= ImGui::DragFloat("Lead-in distance", &app.lead_in_distance,
                                0.1f, 0.0f, 100000.0f, "%.3f");
  changed |= ImGui::SliderInt("Maximum decision layers",
                              &app.max_analysis_layers, 1, 128);
  changed |= ImGui::DragFloat("Tiny split area (stepover^2)",
                              &app.tiny_split_area_factor, 0.01f, 0.0f, 10.0f,
                              "%.3f");
  if (changed) {
    app.view.needs_fit = true;
    app.analysis_dirty = true;
  }

  if (ImGui::Button(app.standard_result.generations.empty()
                        ? "Build analysis"
                        : "Rebuild analysis"))
    build_analysis(app);
  ImGui::SameLine();
  if (ImGui::Button("Fit view")) app.view.needs_fit = true;
  if (app.analysis_dirty) {
    if (app.standard_result.generations.empty())
      ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.30f, 1.0f),
                         "Analysis has not been built.");
    else
      ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.30f, 1.0f),
                         "Options changed; showing the previous build.");
  }

  if (!app.error.empty()) {
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 100, 100, 255));
    ImGui::TextWrapped("%s", app.error.c_str());
    ImGui::PopStyleColor();
    return;
  }

  ImGui::SeparatorText("Decision process");
  const int decision_count = decision_frame_count(app);
  bool decision_changed = false;
  if (ImGui::Button("|<##decision") && app.decision_step > 0) {
    --app.decision_step;
    app.decision_play = false;
    decision_changed = true;
  }
  ImGui::SameLine();
  if (ImGui::Button(app.decision_play ? "Pause##decision"
                                      : "Play##decision"))
    app.decision_play = !app.decision_play;
  ImGui::SameLine();
  if (ImGui::Button(">|##decision") &&
      app.decision_step + 1 < decision_count) {
    ++app.decision_step;
    app.decision_play = false;
    decision_changed = true;
  }
  decision_changed |= ImGui::SliderInt("Analysis step", &app.decision_step,
                                       0, std::max(0, decision_count - 1));
  if (app.decision_play)
    ImGui::SliderFloat("Decision speed", &app.decision_speed, 0.25f, 5.0f,
                       "%.2f step/s");
  if (decision_changed) {
    app.decision_caption = apply_decision_frame(app);
    app.selected_iteration = std::clamp(
        app.selected_iteration, 0,
        std::max(0, static_cast<int>(app.result.generations.size()) - 1));
    app.view.needs_fit = true;
  }
  ImGui::TextWrapped("%s", app.decision_caption.c_str());
  if (!app.analysis_error.empty())
    ImGui::TextColored(ImVec4(1.0f, 0.42f, 0.35f, 1.0f),
                       "Recursive analysis: %s", app.analysis_error.c_str());
  ImGui::TextDisabled("%zu recursive layer%s recorded",
                      app.analysis.layers.size(),
                      app.analysis.layers.size() == 1 ? "" : "s");

  const int count = static_cast<int>(app.result.generations.size());
  if (count > 0) {
    ImGui::SliderInt("Generation", &app.selected_iteration, 0, count - 1);
  }
  ImGui::Checkbox("Show all generations", &app.show_all_iterations);
  ImGui::Checkbox("Show subject", &app.show_subject);
  ImGui::Checkbox(app.stock_slicer.empty() ? "Show allowance box" : "Show stock slice",
                  &app.show_stock);
  if (app.subject_inflation != 0.0f || app.stock_inflation != 0.0f)
    ImGui::Checkbox("Show uninflated boundaries", &app.show_original_boundaries);
  ImGui::Checkbox("Show leading fronts", &app.show_leading_fronts);
  ImGui::Checkbox("Show node boundaries", &app.show_nodes);
  ImGui::Checkbox("Show remaining-domain boundary",
                  &app.show_remaining_domain);
  if (!app.decision_entries.empty())
    ImGui::Checkbox("Show entry choices", &app.show_entry_choices);
  if (app.show_leading_fronts)
    ImGui::Checkbox("Show parent links", &app.show_parent_links);
  ImGui::Checkbox("Show event markers", &app.show_event_markers);
  ImGui::Checkbox("Animate", &app.animate);
  if (app.animate) ImGui::SliderFloat("Speed", &app.animation_speed, 0.25f, 20.0f, "%.2f gen/s");

  ImGui::Separator();
  if (app.subject_slicer.empty())
    ImGui::Text("Subject: SVG (%zu source paths)", app.svg.source_path_count);
  else
    ImGui::Text("Subject: %s", app.subject_slicer.mesh().source.filename().string().c_str());
  ImGui::Text("Union paths: %zu", app.standard_result.subject.size());
  ImGui::Text("Generations: %d", count);
  ImGui::Text("Front nodes: %zu", app.topology.nodes.size());
  ImGui::Text("Events: %zu", app.lineage.events.size());
  ImGui::Text("Stopped on: %s", wave::stop_reason_name(app.result.stop_reason));
  if (count > 0)
    ImGui::Text("Selected contours: %zu",
                app.result.generations[app.selected_iteration].size());

  if (!app.subject_slicer.empty()) {
    const auto& stats = app.subject_slicer.stats();
    ImGui::SeparatorText("Subject slice statistics");
    ImGui::Text("BVH: cuBQL GPU, %.3f ms", stats.build_milliseconds);
    ImGui::Text("Triangles: %zu input, %zu pruned", stats.input_triangles,
                stats.pruned_triangles);
    ImGui::Text("Query: %zu candidates, %.3f ms", app.subject_slice.candidate_triangles,
                app.subject_slice.query_milliseconds);
    ImGui::Text("2D union: %zu contributors, %.3f ms",
                app.subject_slice.contributing_triangles,
                app.subject_slice.polygon_milliseconds);
  }
  if (!app.stock_slicer.empty()) {
    const auto& stats = app.stock_slicer.stats();
    ImGui::SeparatorText("Stock slice statistics");
    ImGui::Text("BVH: cuBQL GPU, %.3f ms", stats.build_milliseconds);
    ImGui::Text("Triangles: %zu input, %zu pruned", stats.input_triangles,
                stats.pruned_triangles);
    ImGui::Text("Query: %zu candidates, %.3f ms", app.stock_slice.candidate_triangles,
                app.stock_slice.query_milliseconds);
  }

  ImGui::SeparatorText("Front lineage");
  if (const auto selected = selected_generation(app)) {
    const std::size_t generation = *selected;
    const auto& current = app.lineage.generations[generation].fronts;
    const std::size_t previous_count = generation == 0
                                           ? 0
                                           : app.lineage.generations[generation - 1].fronts.size();
    ImGui::Text("Transition: %zu -> %zu leading fronts", previous_count,
                current.size());
    for (std::size_t child = 0; child < current.size(); ++child) {
      ImGui::Text("Current %zu: %s, %.1f units, %zu parent%s", child,
                  current[child].closed ? "closed" : "open",
                  wave::front_length(current[child]), current[child].parents.size(),
                  current[child].parents.size() == 1 ? "" : "s");
    }
  }
  if (ImGui::TreeNode("Detected events")) {
    for (std::size_t i = 0; i < app.lineage.events.size(); ++i) {
      const auto& event = app.lineage.events[i];
      ImGui::PushID(static_cast<int>(i));
      if (ImGui::SmallButton("Go"))
        app.selected_iteration = static_cast<int>(event.generation);
      ImGui::SameLine();
      ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(event_color(event.kind)),
                         "generation %zu: %s (%zu -> %zu)", event.generation,
                         wave::lineage_event_kind_name(event.kind),
                         event.previous_fronts.size(), event.current_fronts.size());
      ImGui::PopID();
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Node graph")) {
    for (std::size_t node = 0; node < app.topology.nodes.size(); ++node) {
      const wave::FrontNode& item = app.topology.nodes[node];
      ImGui::PushID(static_cast<int>(node));
      if (!item.fronts.empty()) {
        if (ImGui::SmallButton("Go"))
          app.selected_iteration =
              static_cast<int>(item.fronts.front().generation);
        ImGui::SameLine();
      }
      ImGui::TextColored(
          ImGui::ColorConvertU32ToFloat4(node_color(node)),
          "node %zu: %s, layer %zu, %zu front%s, %zu -> %zu", node,
          item.kind == wave::FrontNode::Kind::MergeTermination
              ? "merge termination"
              : "propagation",
          item.evaluation_order, item.fronts.size(),
          item.fronts.size() == 1 ? "" : "s", item.parents.size(),
          item.children.size());
      ImGui::PopID();
    }
    ImGui::TreePop();
  }

  ImGui::Spacing();
  ImGui::TextColored(ImVec4(0.99f, 0.89f, 0.35f, 1.0f), "Subject");
  ImGui::SameLine();
  ImGui::TextDisabled("  cyan -> orange: early -> late");
  ImGui::TextDisabled("Right-drag: pan   Wheel: zoom");
  ImGui::TextWrapped(
      "Waves are seeded only at the stock boundary. The reached area expands by "
      "Stepover, is clipped to the stock, and then has the subject removed. Each "
      "colored generation is only the newly reached area; subject boundaries never "
      "act as wave sources.");
  ImGui::TextWrapped(
      "Selected leading fronts: violet = previous, green = current. White links "
      "show parentage. Node boundaries have stable distinct colours. Event markers: "
      "S split, M merge, T termination, MT merge-termination, B unexpected birth.");
  ImGui::TextWrapped(
      "During a decision frame, thick coloured fronts are retained entries; grey "
      "fronts marked with an X are muted. Matching colours identify entries in the "
      "same graph-derived remaining volume. The dashed blue outline is the cached "
      "remaining domain for that recursive layer.");
  ImGui::TextWrapped(
      "Rounded waves use round offset joins; disabling them uses sharp miter "
      "joins. Rounded propagation is the normal mode, while mitered propagation "
      "is useful for faster topology diagnostics.");
}

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

StartupInputs resolve_inputs(int argc, char** argv) {
  StartupInputs result;
  for (int i = 1; i < argc; ++i) {
    const std::string_view argument = argv[i];
    auto value_after = [&](std::string_view option) -> std::filesystem::path {
      if (i + 1 >= argc) throw std::runtime_error(std::string(option) + " requires a path");
      return argv[++i];
    };
    auto float_after = [&](std::string_view option) -> float {
      const std::string text = value_after(option).string();
      std::size_t consumed = 0;
      float value = 0.0f;
      try {
        value = std::stof(text, &consumed);
      } catch (const std::exception&) {
        throw std::runtime_error(std::string(option) + " requires a number");
      }
      if (consumed != text.size() || !std::isfinite(value))
        throw std::runtime_error(std::string(option) + " requires a finite number");
      return value;
    };
    auto int_after = [&](std::string_view option) -> int {
      const std::string text = value_after(option).string();
      std::size_t consumed = 0;
      int value = 0;
      try {
        value = std::stoi(text, &consumed);
      } catch (const std::exception&) {
        throw std::runtime_error(std::string(option) + " requires an integer");
      }
      if (consumed != text.size())
        throw std::runtime_error(std::string(option) + " requires an integer");
      return value;
    };
    if (argument == "--help" || argument == "-h") result.help = true;
    else if (argument == "--dump-analysis") result.dump_analysis = true;
    else if (argument == "--svg") result.svg = value_after(argument);
    else if (argument == "--subject-mesh") result.subject_mesh = value_after(argument);
    else if (argument == "--stock-mesh") result.stock_mesh = value_after(argument);
    else if (argument == "--screenshot") result.screenshot = value_after(argument);
    else if (argument == "--subject-inflate") result.subject_inflation = float_after(argument);
    else if (argument == "--stock-inflate") result.stock_inflation = float_after(argument);
    else if (argument == "--lead-in-distance")
      result.lead_in_distance = float_after(argument);
    else if (argument == "--lead-in") result.lead_in = true;
    else if (argument == "--no-lead-in") result.lead_in = false;
    else if (argument == "--allowance") result.allowance = float_after(argument);
    else if (argument == "--stepover") result.stepover = float_after(argument);
    else if (argument == "--round-waves") result.rounded_waves = true;
    else if (argument == "--mitered-waves") result.rounded_waves = false;
    else if (argument == "--slice-z") result.slice_z = float_after(argument);
    else if (argument == "--query-grid") result.query_grid = int_after(argument);
    else if (argument == "--analysis-step")
      result.analysis_step = int_after(argument);
    else if (argument == "--analysis-layers")
      result.analysis_layers = int_after(argument);
    else if (argument == "--tiny-split-area-factor")
      result.tiny_split_area_factor = float_after(argument);
    else if (!argument.starts_with("--") && result.svg.empty()) result.svg = argument;
    else throw std::runtime_error("Unknown command-line argument: " + std::string(argument));
  }
  if (!result.svg.empty()) return result;
  const std::filesystem::path local = "wavetest.svg";
  if (std::filesystem::exists(local)) {
    result.svg = local;
    return result;
  }
  const auto beside_executable = std::filesystem::path(argv[0]).parent_path() / "wavetest.svg";
  result.svg = beside_executable;
  return result;
}

void print_help(const char* executable) {
  std::cout
      << "Usage: " << executable << " [SVG] [options]\n\n"
      << "Mesh/SVG slicing and Clipper2 wavefront topology analysis.\n\n"
      << "Input options:\n"
      << "  --svg PATH              2D fallback subject (default: wavetest.svg)\n"
      << "  --subject-mesh PATH     Load an OBJ or STL subject mesh\n"
      << "  --stock-mesh PATH       Load an OBJ or STL stock mesh\n\n"
      << "Geometry options:\n"
      << "  --slice-z VALUE         Initial mesh slice height\n"
      << "  --query-grid N          GPU query grid resolution, 1..256 (default: 32)\n"
      << "  --subject-inflate VALUE Signed subject boundary offset (default: 0)\n"
      << "  --stock-inflate VALUE   Signed stock boundary offset (default: 0)\n"
      << "  --lead-in-distance VALUE\n"
      << "                           Distance to seed in front of the stock wall\n"
      << "  --lead-in, --no-lead-in Enable or disable that exterior seed distance\n"
      << "                           so the first passes engage gradually (default: on)\n"
      << "  --allowance VALUE       Fallback stock allowance when no stock mesh is loaded\n"
      << "  --stepover VALUE        Wave propagation distance per generation\n"
      << "  --round-waves           Use round offset joins (default)\n"
      << "  --mitered-waves         Use sharp miter offset joins\n\n"
      << "Other options:\n"
      << "  --dump-analysis        Print the recorded decision process and exit\n"
      << "  --analysis-step N      Initially display recorded decision frame N\n"
      << "  --analysis-layers N    Maximum recursive decision layers\n"
      << "  --tiny-split-area-factor VALUE\n"
      << "                         Ignore terminal split branches below VALUE *"
         " stepover^2 (default: 0.5)\n"
      << "  --screenshot PATH       Render one frame to a binary PPM file and exit\n"
      << "  -h, --help              Show this help and exit\n\n"
      << "Positive inflation expands a boundary; negative inflation contracts it.\n";
}

void save_screenshot(const std::filesystem::path& filename, int width, int height) {
  std::vector<unsigned char> pixels(static_cast<std::size_t>(width) * height * 3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
  std::ofstream stream(filename, std::ios::binary);
  if (!stream) throw std::runtime_error("Unable to write screenshot: " + filename.string());
  stream << "P6\n" << width << ' ' << height << "\n255\n";
  const std::size_t row_bytes = static_cast<std::size_t>(width) * 3;
  for (int y = height - 1; y >= 0; --y)
    stream.write(reinterpret_cast<const char*>(pixels.data() + y * row_bytes), row_bytes);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto inputs = resolve_inputs(argc, argv);
    if (inputs.help) {
      print_help(argv[0]);
      return 0;
    }
    const auto& input = inputs.svg;
    AppState app;
    app.svg = wave::load_svg_closed_paths(input);
    app.subject_paths = app.svg.paths;
    app.auto_stock_subject_bounds = app.svg.bounds;
    const double span = std::max(
        wave::to_world(app.svg.bounds.right - app.svg.bounds.left),
        wave::to_world(app.svg.bounds.bottom - app.svg.bounds.top));
    app.allowance = static_cast<float>(span * 0.05);
    app.stepover = static_cast<float>(span * 0.025);
    app.lead_in_distance = app.stepover * 0.5f;
    // Every analysis below would be thrown away: the mesh loads redefine the
    // subject, the command line then redefines the parameters, and the tail of
    // this function analyses the result. So load the meshes, resolve the
    // options, and build the wavefront exactly once, at the bottom.
    if (!inputs.subject_mesh.empty()) {
      std::snprintf(app.subject_mesh_path.data(), app.subject_mesh_path.size(), "%s",
                    inputs.subject_mesh.string().c_str());
      load_mesh(app, false, /*analyze=*/false);
    }
    if (!inputs.stock_mesh.empty()) {
      std::snprintf(app.stock_mesh_path.data(), app.stock_mesh_path.size(), "%s",
                    inputs.stock_mesh.string().c_str());
      load_mesh(app, true, /*analyze=*/false);
    }
    if (inputs.query_grid) {
      if (*inputs.query_grid < 1 || *inputs.query_grid > 256)
        throw std::runtime_error("--query-grid must be between 1 and 256");
      app.query_grid = *inputs.query_grid;
    }
    if (inputs.analysis_layers) {
      if (*inputs.analysis_layers < 1 || *inputs.analysis_layers > 128)
        throw std::runtime_error("--analysis-layers must be between 1 and 128");
      app.max_analysis_layers = *inputs.analysis_layers;
    }
    if (inputs.tiny_split_area_factor) {
      if (*inputs.tiny_split_area_factor < 0.0f)
        throw std::runtime_error(
            "--tiny-split-area-factor must be nonnegative");
      app.tiny_split_area_factor = *inputs.tiny_split_area_factor;
    }
    if (inputs.lead_in) app.lead_in = *inputs.lead_in;
    if (inputs.slice_z) app.slice_z = *inputs.slice_z;
    if (inputs.subject_inflation) app.subject_inflation = *inputs.subject_inflation;
    if (inputs.stock_inflation) app.stock_inflation = *inputs.stock_inflation;
    if (inputs.allowance) {
      if (!(*inputs.allowance > 0.0f))
        throw std::runtime_error("--allowance must be positive");
      app.allowance = *inputs.allowance;
    }
    if (inputs.stepover) {
      if (!(*inputs.stepover > 0.0f))
        throw std::runtime_error("--stepover must be positive");
      app.stepover = *inputs.stepover;
      if (!inputs.lead_in_distance)
        app.lead_in_distance = app.stepover * 0.5f;
    }
    if (inputs.lead_in_distance) {
      if (!(*inputs.lead_in_distance >= 0.0f))
        throw std::runtime_error("--lead-in-distance must not be negative");
      app.lead_in_distance = *inputs.lead_in_distance;
    }
    if (inputs.rounded_waves) app.rounded_waves = *inputs.rounded_waves;
    if (!app.subject_slicer.empty() || !app.stock_slicer.empty())
      reslice(app, /*analyze=*/false);
    else
      update_preview_outlines(app);

    // Interactive startup only prepares the input preview. Headless requests
    // necessarily need a completed analysis before they can dump or render it.
    const bool build_immediately =
        inputs.dump_analysis || !inputs.screenshot.empty() ||
        inputs.analysis_step.has_value();
    if (build_immediately) recompute(app);
    if (inputs.analysis_step) {
      app.decision_step = *inputs.analysis_step;
      app.decision_caption = apply_decision_frame(app);
      app.view.needs_fit = true;
    }

    if (inputs.dump_analysis) {
      if (!app.error.empty())
        throw std::runtime_error(app.error);
      std::cout << "initial nodes " << app.standard_topology.nodes.size()
                << ", recursive layers " << app.analysis.layers.size() << '\n';
      const wave::Rect64 stock_bounds =
          Clipper2Lib::GetBounds(app.standard_result.stock);
      std::cout << "stock bounds " << wave::to_world(stock_bounds.left) << ','
                << wave::to_world(stock_bounds.top) << " -> "
                << wave::to_world(stock_bounds.right) << ','
                << wave::to_world(stock_bounds.bottom) << '\n';
      if (!app.analysis_error.empty())
        std::cout << "analysis error: " << app.analysis_error << '\n';
      for (const wave::WavefrontDecisionLayer& layer : app.analysis.layers) {
        std::cout << "layer " << layer.layer << ": "
                  << layer.entries.size() << " entries in "
                  << layer.groups.size() << " groups, baseline "
                  << layer.baseline_topology.nodes.size() << " nodes, "
                  << layer.remaining_domain.size() << " domain rings, "
                  << layer.cached_reached.size() << " cached-state rings, "
                  << layer.baseline_wave.generations.size()
                  << " generations" << '\n';
        for (const wave::WavefrontStrategyTrial& trial : layer.trials) {
          const std::size_t candidate_entries = std::count_if(
              trial.topology.nodes.begin(), trial.topology.nodes.end(),
              [](const wave::FrontNode& node) {
                return node.kind == wave::FrontNode::Kind::Propagation &&
                       node.evaluation_order == 0;
              });
          std::cout << "  longest entry per connected group: "
                    << trial.candidate_node_count << " vs "
                    << trial.previous_node_count << " raw nodes, "
                    << trial.candidate_significant_node_count << " vs "
                    << trial.previous_significant_node_count
                    << " significant nodes (>= "
                    << trial.minimum_significant_node_area << " mm^2), "
                    << trial.retained_entries.size() << " retained / "
                    << candidate_entries << " resulting entries, "
                    << (trial.accepted ? "accepted" : "rejected") << '\n';
          auto print_areas = [](const char* label,
                                const std::vector<double>& areas) {
            std::vector<double> sorted = areas;
            std::sort(sorted.begin(), sorted.end(), std::greater<double>());
            std::cout << "    " << label << " areas:";
            for (double area : sorted) std::cout << ' ' << area;
            std::cout << " mm^2\n";
          };
          print_areas("baseline", trial.previous_node_areas);
          print_areas("candidate", trial.candidate_node_areas);
        }
        std::cout << "  selected " << layer.selected_topology.nodes.size()
                  << " nodes" << '\n';
        std::vector<std::size_t> order_counts;
        for (const wave::FrontNode& node : layer.selected_topology.nodes) {
          if (node.evaluation_order >= order_counts.size())
            order_counts.resize(node.evaluation_order + 1, 0);
          ++order_counts[node.evaluation_order];
        }
        std::cout << "  node layers";
        for (std::size_t order = 0; order < order_counts.size(); ++order)
          std::cout << ' ' << order << ':' << order_counts[order];
        std::size_t mixed_order_bands = 0;
        for (std::size_t generation = 0;
             generation < layer.selected_lineage.generations.size();
             ++generation) {
          const wave::FrontGeneration& fronts =
              layer.selected_lineage.generations[generation];
          std::vector<std::size_t> band_orders(fronts.bands.size(),
                                                wave::kNoTopologyId);
          for (std::size_t front = 0; front < fronts.fronts.size(); ++front) {
            const std::size_t node =
                layer.selected_topology.front_nodes[generation][front];
            const std::size_t order =
                layer.selected_topology.nodes[node].evaluation_order;
            std::size_t& band_order = band_orders[fronts.fronts[front].band];
            if (band_order == wave::kNoTopologyId)
              band_order = order;
            else if (band_order != order)
              band_order = std::numeric_limits<std::size_t>::max() - 1;
          }
          mixed_order_bands += static_cast<std::size_t>(std::count(
              band_orders.begin(), band_orders.end(),
              std::numeric_limits<std::size_t>::max() - 1));
        }
        std::cout << ", " << mixed_order_bands << " mixed-order bands\n";
      }
      const std::vector<wave::WavefrontStage> committed =
          wave::committed_wavefront_stages(app.analysis);
      std::size_t committed_fronts = 0;
      for (const wave::WavefrontStage& stage : committed) {
        std::size_t fronts = 0;
        for (const wave::FrontGeneration& generation : stage.lineage.generations)
          fronts += generation.fronts.size();
        committed_fronts += fronts;
        std::cout << "committed layer " << stage.analysis_layer << ": "
                  << stage.topology.nodes.size() << " nodes, " << fronts
                  << " fronts, " << stage.lineage.generations.size()
                  << " generations" << '\n';
      }
      std::cout << "committed total: " << committed.size() << " layers, "
                << committed_fronts << " fronts" << '\n';
      return 0;
    }

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0)
      throw std::runtime_error(std::string("SDL initialization failed: ") + SDL_GetError());

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 0);
    SDL_Window* window = SDL_CreateWindow(
        "Wavefront analysis viewer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1500, 900, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!window) throw std::runtime_error(std::string("Window creation failed: ") + SDL_GetError());
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    if (!gl_context) throw std::runtime_error(std::string("OpenGL context failed: ") + SDL_GetError());
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;  // The demo uses a fixed workspace; don't litter the input directory.
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 150");

    bool running = true;
    bool screenshot_pending = !inputs.screenshot.empty();
    double animation_accumulator = 0.0;
    double decision_accumulator = 0.0;
    auto previous_time = std::chrono::steady_clock::now();
    while (running) {
      SDL_Event event;
      while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL2_ProcessEvent(&event);
        if (event.type == SDL_QUIT ||
            (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE &&
             event.window.windowID == SDL_GetWindowID(window))) {
          running = false;
        }
      }

      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - previous_time).count();
      previous_time = now;
      if (app.decision_play) {
        decision_accumulator += elapsed * app.decision_speed;
        while (decision_accumulator >= 1.0 && app.decision_play) {
          if (app.decision_step + 1 >= decision_frame_count(app)) {
            app.decision_play = false;
            decision_accumulator = 0.0;
            break;
          }
          ++app.decision_step;
          app.decision_caption = apply_decision_frame(app);
          app.selected_iteration = std::clamp(
              app.selected_iteration, 0,
              std::max(0, static_cast<int>(app.result.generations.size()) - 1));
          decision_accumulator -= 1.0;
        }
      }
      if (app.animate && !app.result.generations.empty()) {
        animation_accumulator += elapsed * app.animation_speed;
        while (animation_accumulator >= 1.0) {
          app.selected_iteration =
              (app.selected_iteration + 1) % static_cast<int>(app.result.generations.size());
          animation_accumulator -= 1.0;
        }
      }

      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplSDL2_NewFrame();
      ImGui::NewFrame();

      const ImGuiViewport* viewport = ImGui::GetMainViewport();
      ImGui::SetNextWindowPos(viewport->WorkPos);
      ImGui::SetNextWindowSize(viewport->WorkSize);
      ImGui::Begin("Wavefront analysis", nullptr,
                   ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                       ImGuiWindowFlags_NoSavedSettings);
      ImGui::BeginChild("Controls", ImVec2(365.0f, 0.0f), ImGuiChildFlags_Borders);
      controls(app, input);
      ImGui::EndChild();
      ImGui::SameLine();
      draw_canvas(app);
      ImGui::End();

      ImGui::Render();
      int display_width = 0;
      int display_height = 0;
      SDL_GL_GetDrawableSize(window, &display_width, &display_height);
      glViewport(0, 0, display_width, display_height);
      glClearColor(0.035f, 0.047f, 0.063f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT);
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      if (screenshot_pending) {
        save_screenshot(inputs.screenshot, display_width, display_height);
        screenshot_pending = false;
        running = false;
      }
      SDL_GL_SwapWindow(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "clipper2_wave_demo: " << error.what() << '\n';
    return 1;
  }
}
