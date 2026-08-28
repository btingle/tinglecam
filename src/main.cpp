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
  std::array<char, 512> subject_mesh_path{};
  std::array<char, 512> stock_mesh_path{};
  float slice_z = 0.0f;
  float slice_min = 0.0f;
  float slice_max = 1.0f;
  int query_grid = 32;
  float allowance = 150.0f;
  float engagement = 80.0f;
  float subject_inflation = 0.0f;
  float stock_inflation = 0.0f;
  int selected_iteration = -1;
  bool show_all_iterations = true;
  bool show_subject = true;
  bool show_stock = true;
  bool show_leading_fronts = true;
  bool show_parent_links = true;
  bool show_event_markers = true;
  bool animate = false;
  float animation_speed = 4.0f;
  ViewState view;
  OrbitState orbit;
  std::string error;
};

Paths64 offset_boundaries(const Paths64& input, float amount,
                          std::string_view description) {
  Paths64 result = Clipper2Lib::Union(input, Clipper2Lib::FillRule::NonZero);
  const std::int64_t delta = wave::to_integer(amount);
  if (delta != 0) {
    result = Clipper2Lib::InflatePaths(
        result, static_cast<double>(delta), Clipper2Lib::JoinType::Miter,
        Clipper2Lib::EndType::Polygon);
    result = Clipper2Lib::Union(result, Clipper2Lib::FillRule::NonZero);
  }
  if (result.empty())
    throw std::runtime_error(std::string(description) +
                             " offset removes all geometry");
  return result;
}

Paths64 allowance_stock(const Paths64& subject, float allowance) {
  if (!(allowance > 0.0f)) throw std::runtime_error("Allowance must be positive");
  const auto bounds = Clipper2Lib::GetBounds(subject);
  const std::int64_t margin = std::max<std::int64_t>(1, wave::to_integer(allowance));
  return Paths64{{
      {bounds.left - margin, bounds.top - margin},
      {bounds.right + margin, bounds.top - margin},
      {bounds.right + margin, bounds.bottom + margin},
      {bounds.left - margin, bounds.bottom + margin},
  }};
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

ImU32 event_color(wave::LineageEventKind kind, int alpha = 255) {
  switch (kind) {
    case wave::LineageEventKind::Split: return IM_COL32(255, 91, 177, alpha);
    case wave::LineageEventKind::Merge: return IM_COL32(72, 229, 139, alpha);
    case wave::LineageEventKind::Termination: return IM_COL32(255, 92, 73, alpha);
    case wave::LineageEventKind::Birth: return IM_COL32(255, 205, 74, alpha);
  }
  return IM_COL32(255, 255, 255, alpha);
}

const char* event_short_name(wave::LineageEventKind kind) {
  switch (kind) {
    case wave::LineageEventKind::Split: return "S";
    case wave::LineageEventKind::Merge: return "M";
    case wave::LineageEventKind::Termination: return "T";
    case wave::LineageEventKind::Birth: return "B";
  }
  return "?";
}

void recompute(AppState& app) {
  try {
    if (app.subject_paths.empty())
      throw std::runtime_error("The current subject slice is empty");
    const Paths64 subject = offset_boundaries(
        app.subject_paths, app.subject_inflation, "Subject");
    Paths64 stock;
    if (app.stock_slicer.empty()) stock = allowance_stock(app.subject_paths, app.allowance);
    else {
      if (app.sliced_stock.empty()) throw std::runtime_error("The current stock slice is empty");
      stock = app.sliced_stock;
    }
    stock = offset_boundaries(stock, app.stock_inflation, "Stock");
    app.result = wave::build_wavefront(subject, stock, app.engagement);
    app.lineage = wave::build_front_lineage(app.result, app.engagement);
    if (app.selected_iteration < 0 && !app.lineage.events.empty())
      app.selected_iteration = static_cast<int>(app.lineage.events.front().generation);
    else
      app.selected_iteration = std::clamp(
          app.selected_iteration, 0,
          std::max(0, static_cast<int>(app.result.generations.size()) - 1));
    app.error.clear();
  } catch (const std::exception& error) {
    app.error = error.what();
  }
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

void reslice(AppState& app) {
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
    recompute(app);
  } catch (const std::exception& error) {
    app.error = error.what();
  }
}

void load_mesh(AppState& app, bool stock) {
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
        app.engagement = span * 0.025f;
      }
    }
    update_slice_range(app, true);
    reslice(app);
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

void draw_path(ImDrawList* draw_list, const Path64& path,
               const CanvasTransform& transform, ImU32 color, float width) {
  draw_paths(draw_list, Paths64{path}, transform, color, width);
}

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

void fit_view(AppState& app, ImVec2 canvas_size) {
  const auto bounds = Clipper2Lib::GetBounds(app.result.stock);
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

  if (app.show_stock)
    draw_paths(draw_list, app.result.stock, transform, IM_COL32(125, 138, 152, 190), 2.0f);

  const int count = static_cast<int>(app.result.generations.size());
  if (app.show_all_iterations) {
    for (int i = 0; i < count; ++i) {
      const bool selected = i == app.selected_iteration;
      draw_paths(draw_list, app.result.generations[i], transform,
                 iteration_color(i, count, selected ? 255 : 145),
                 selected ? 3.0f : 1.25f);
    }
  } else if (count > 0) {
    draw_paths(draw_list, app.result.generations[app.selected_iteration], transform,
               iteration_color(app.selected_iteration, count), 2.5f);
  }

  if (app.show_subject)
    draw_paths(draw_list, app.result.subject, transform, IM_COL32(252, 226, 89, 255), 3.0f);

  if (app.show_leading_fronts &&
      app.selected_iteration < static_cast<int>(app.lineage.generations.size())) {
    const std::size_t generation = static_cast<std::size_t>(app.selected_iteration);
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
  ImGui::TextUnformatted("GPU mesh slicer + contour wave");
  ImGui::TextDisabled("2D fallback: %s", input.filename().string().c_str());
  draw_mesh_viewer(app);

  ImGui::SeparatorText("Models");
  ImGui::SetNextItemWidth(-1.0f);
  ImGui::InputTextWithHint("##subject-mesh", "Subject .obj or .stl",
                           app.subject_mesh_path.data(), app.subject_mesh_path.size());
  if (ImGui::Button("Load subject mesh")) load_mesh(app, false);
  if (!app.subject_slicer.empty()) {
    ImGui::SameLine();
    if (ImGui::Button("Use SVG subject")) {
      app.subject_slicer = wave::MeshSlicer{};
      update_slice_range(app, false);
      reslice(app);
    }
  }
  ImGui::SetNextItemWidth(-1.0f);
  ImGui::InputTextWithHint("##stock-mesh", "Optional stock .obj or .stl",
                           app.stock_mesh_path.data(), app.stock_mesh_path.size());
  if (ImGui::Button("Load stock mesh")) load_mesh(app, true);
  if (!app.stock_slicer.empty()) {
    ImGui::SameLine();
    if (ImGui::Button("Use allowance stock")) {
      app.stock_slicer = wave::MeshSlicer{};
      update_slice_range(app, false);
      reslice(app);
    }
  }

  if (!app.subject_slicer.empty() || !app.stock_slicer.empty()) {
    bool slice_changed = ImGui::SliderFloat("Slice Z", &app.slice_z,
                                            app.slice_min, app.slice_max, "%.4f");
    slice_changed |= ImGui::SliderInt("BVH query grid", &app.query_grid, 1, 128);
    if (slice_changed) reslice(app);
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
  changed |= ImGui::DragFloat("Engagement", &app.engagement, 1.0f, 0.001f, 100000.0f, "%.3f");
  if (changed) {
    app.view.needs_fit = true;
    recompute(app);
  }

  if (ImGui::Button("Recompute")) recompute(app);
  ImGui::SameLine();
  if (ImGui::Button("Fit view")) app.view.needs_fit = true;

  if (!app.error.empty()) {
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 100, 100, 255));
    ImGui::TextWrapped("%s", app.error.c_str());
    ImGui::PopStyleColor();
    return;
  }

  const int count = static_cast<int>(app.result.generations.size());
  if (count > 0) {
    ImGui::SliderInt("Generation", &app.selected_iteration, 0, count - 1);
  }
  ImGui::Checkbox("Show all generations", &app.show_all_iterations);
  ImGui::Checkbox("Show subject", &app.show_subject);
  ImGui::Checkbox(app.stock_slicer.empty() ? "Show allowance box" : "Show stock slice",
                  &app.show_stock);
  ImGui::Checkbox("Show leading fronts", &app.show_leading_fronts);
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
  ImGui::Text("Union paths: %zu", app.result.subject.size());
  ImGui::Text("Generations: %d", count);
  ImGui::Text("Stopped on: %s", wave::stop_reason_name(app.result.stop_reason));
  if (count > 0)
    ImGui::Text("Selected contours: %zu",
                app.result.generations[app.selected_iteration].size());

  if (!app.subject_slicer.empty()) {
    const auto& stats = app.subject_slicer.stats();
    ImGui::SeparatorText("Subject slice statistics");
    ImGui::Text("BVH: %s, %.3f ms", stats.gpu_accelerated ? "cuBQL GPU" : "CPU fallback",
                stats.build_milliseconds);
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
    ImGui::Text("BVH: %s, %.3f ms", stats.gpu_accelerated ? "cuBQL GPU" : "CPU fallback",
                stats.build_milliseconds);
    ImGui::Text("Triangles: %zu input, %zu pruned", stats.input_triangles,
                stats.pruned_triangles);
    ImGui::Text("Query: %zu candidates, %.3f ms", app.stock_slice.candidate_triangles,
                app.stock_slice.query_milliseconds);
  }

  ImGui::SeparatorText("Front lineage");
  if (app.selected_iteration < static_cast<int>(app.lineage.generations.size())) {
    const std::size_t generation = static_cast<std::size_t>(app.selected_iteration);
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

  ImGui::Spacing();
  ImGui::TextColored(ImVec4(0.99f, 0.89f, 0.35f, 1.0f), "Subject");
  ImGui::SameLine();
  ImGui::TextDisabled("  cyan -> orange: early -> late");
  ImGui::TextDisabled("Right-drag: pan   Wheel: zoom");
  ImGui::TextWrapped(
      "Waves are seeded only at the stock boundary. The reached area expands by "
      "Engagement, is clipped to the stock, and then has the subject removed. Each "
      "colored generation is only the newly reached area; subject boundaries never "
      "act as wave sources.");
  ImGui::TextWrapped(
      "Selected leading fronts: violet = previous, green = current. White links "
      "show parentage. Event markers: S split, M merge, T termination, B unexpected birth.");
}

struct StartupInputs {
  std::filesystem::path svg;
  std::filesystem::path subject_mesh;
  std::filesystem::path stock_mesh;
  std::filesystem::path screenshot;
  std::optional<float> subject_inflation;
  std::optional<float> stock_inflation;
  std::optional<float> allowance;
  std::optional<float> engagement;
  std::optional<float> slice_z;
  std::optional<int> query_grid;
  bool help = false;
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
    else if (argument == "--svg") result.svg = value_after(argument);
    else if (argument == "--subject-mesh") result.subject_mesh = value_after(argument);
    else if (argument == "--stock-mesh") result.stock_mesh = value_after(argument);
    else if (argument == "--screenshot") result.screenshot = value_after(argument);
    else if (argument == "--subject-inflate") result.subject_inflation = float_after(argument);
    else if (argument == "--stock-inflate") result.stock_inflation = float_after(argument);
    else if (argument == "--allowance") result.allowance = float_after(argument);
    else if (argument == "--engagement") result.engagement = float_after(argument);
    else if (argument == "--slice-z") result.slice_z = float_after(argument);
    else if (argument == "--query-grid") result.query_grid = int_after(argument);
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
      << "GPU downward-prism mesh slicing and Clipper2 wavefront analysis.\n\n"
      << "Input options:\n"
      << "  --svg PATH              2D fallback subject (default: wavetest.svg)\n"
      << "  --subject-mesh PATH     Load an OBJ or STL subject mesh\n"
      << "  --stock-mesh PATH       Load an OBJ or STL stock mesh\n\n"
      << "Geometry options:\n"
      << "  --slice-z VALUE         Initial mesh slice height\n"
      << "  --query-grid N          GPU query grid resolution, 1..256 (default: 32)\n"
      << "  --subject-inflate VALUE Signed subject boundary offset (default: 0)\n"
      << "  --stock-inflate VALUE   Signed stock boundary offset (default: 0)\n"
      << "  --allowance VALUE       Fallback stock allowance when no stock mesh is loaded\n"
      << "  --engagement VALUE      Wave propagation distance per generation\n\n"
      << "Other options:\n"
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
    const double span = std::max(
        wave::to_world(app.svg.bounds.right - app.svg.bounds.left),
        wave::to_world(app.svg.bounds.bottom - app.svg.bounds.top));
    app.allowance = static_cast<float>(span * 0.05);
    app.engagement = static_cast<float>(span * 0.025);
    recompute(app);
    if (!inputs.subject_mesh.empty()) {
      std::snprintf(app.subject_mesh_path.data(), app.subject_mesh_path.size(), "%s",
                    inputs.subject_mesh.string().c_str());
      load_mesh(app, false);
    }
    if (!inputs.stock_mesh.empty()) {
      std::snprintf(app.stock_mesh_path.data(), app.stock_mesh_path.size(), "%s",
                    inputs.stock_mesh.string().c_str());
      load_mesh(app, true);
    }
    if (inputs.query_grid) {
      if (*inputs.query_grid < 1 || *inputs.query_grid > 256)
        throw std::runtime_error("--query-grid must be between 1 and 256");
      app.query_grid = *inputs.query_grid;
    }
    if (inputs.slice_z) app.slice_z = *inputs.slice_z;
    if (inputs.subject_inflation) app.subject_inflation = *inputs.subject_inflation;
    if (inputs.stock_inflation) app.stock_inflation = *inputs.stock_inflation;
    if (inputs.allowance) {
      if (!(*inputs.allowance > 0.0f))
        throw std::runtime_error("--allowance must be positive");
      app.allowance = *inputs.allowance;
    }
    if (inputs.engagement) {
      if (!(*inputs.engagement > 0.0f))
        throw std::runtime_error("--engagement must be positive");
      app.engagement = *inputs.engagement;
    }
    if (!app.subject_slicer.empty() || !app.stock_slicer.empty()) reslice(app);
    else recompute(app);

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0)
      throw std::runtime_error(std::string("SDL initialization failed: ") + SDL_GetError());

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 0);
    SDL_Window* window = SDL_CreateWindow(
        "Clipper2 contour wave", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
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
      ImGui::Begin("Clipper2 wave workspace", nullptr,
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
