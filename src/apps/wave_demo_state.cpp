#include "wave_demo_state.h"
namespace wave_demo {
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

ImU32 iteration_color(int index, int count, int alpha) {
  const float t = count <= 1 ? 0.0f : static_cast<float>(index) / (count - 1);
  const ImVec4 cyan{0.10f, 0.78f, 0.94f, alpha / 255.0f};
  const ImVec4 orange{1.00f, 0.45f, 0.12f, alpha / 255.0f};
  return ImGui::ColorConvertFloat4ToU32(ImVec4(
      cyan.x + (orange.x - cyan.x) * t,
      cyan.y + (orange.y - cyan.y) * t,
      cyan.z + (orange.z - cyan.z) * t,
      alpha / 255.0f));
}

ImU32 node_color(std::size_t index, int alpha) {
  const float hue = std::fmod(static_cast<float>(index) * 0.61803398875f, 1.0f);
  ImVec4 color = ImColor::HSV(hue, 0.60f, 0.95f);
  color.w = alpha / 255.0f;
  return ImGui::ColorConvertFloat4ToU32(color);
}

ImU32 event_color(wave::LineageEventKind kind, int alpha) {
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
void reslice(AppState& app, bool analyze) {
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

void load_mesh(AppState& app, bool stock, bool analyze) {
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

}  // namespace wave_demo
