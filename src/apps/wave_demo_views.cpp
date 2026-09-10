#include "wave_demo_state.h"
namespace wave_demo {
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
}  // namespace wave_demo
