#include <clipper2/clipper.h>

#include <SDL.h>
#include <SDL_opengl.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl2.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

using namespace Clipper2Lib;

namespace {

constexpr float kPi = 3.14159265358979323846f;

struct PathItem {
  std::string name;
  PathD points;
  bool closed = true;
  bool visible = true;
  ImU32 colour = IM_COL32(98, 210, 168, 255);
};

struct View {
  ImVec2 pan{0.0f, 0.0f};
  float zoom = 4.0f;
  bool needs_fit = true;
};

struct App {
  std::vector<PathItem> paths;
  int operand_a = -1;
  int operand_b = -1;
  int next_id = 1;
  int selected_row = -1;
  View view;
  ImGuiStyle base_style;
  float ui_scale = 1.0f;

  float rectangle_width = 50.0f;
  float rectangle_height = 50.0f;
  float circle_radius = 25.0f;
  int circle_segments = 48;
  float inflate_amount = 5.0f;
  float arc_tolerance = 0.015f;
  float translate_x = 10.0f;
  float translate_y = 0.0f;
  float rotate_degrees = 15.0f;
  float rotate_pivot_x = 0.0f;
  float rotate_pivot_y = 0.0f;
  int join_type = 2; // Round.
  int end_type = 3;  // Round for open paths (UI index; Clipper2 value is +1).
  bool draw_mode = false;
  bool draw_closed = false;
  PathD in_progress;
  bool trim_mode = false;
  int trim_path = -1;
  int trim_vertex_a = -1;
  int trim_vertex_b = -1;
  char import_path[512] = "paths.obj";
  char export_path[512] = "paths.obj";
  std::string status = "Ready. Add a primitive or draw a path.";
};

bool valid_operand(const App& app, int index);

ImU32 path_colour(std::size_t index, bool active) {
  static constexpr std::array<ImU32, 6> colours = {
      IM_COL32(93, 211, 158, 255), IM_COL32(103, 177, 255, 255),
      IM_COL32(255, 183, 77, 255), IM_COL32(215, 137, 255, 255),
      IM_COL32(255, 111, 130, 255), IM_COL32(83, 214, 218, 255)};
  const ImU32 base = colours[index % colours.size()];
  if (active) return base;
  return IM_COL32(((base >> IM_COL32_R_SHIFT) & 0xff) * 2 / 3,
                  ((base >> IM_COL32_G_SHIFT) & 0xff) * 2 / 3,
                  ((base >> IM_COL32_B_SHIFT) & 0xff) * 2 / 3, 255);
}

std::string next_name(App& app, std::string_view prefix) {
  return std::string(prefix) + " " + std::to_string(app.next_id++);
}

void add_path(App& app, PathD points, bool closed, std::string name) {
  if (points.size() < 2) return;
  while (points.size() > 2 && points.front() == points.back()) points.pop_back();
  app.paths.push_back({std::move(name), std::move(points), closed, true,
                       path_colour(app.paths.size(), true)});
  app.selected_row = static_cast<int>(app.paths.size() - 1);
  app.operand_a = app.selected_row;
  app.view.needs_fit = true;
}

void delete_path(App& app, int index) {
  if (!valid_operand(app, index)) return;
  const std::string name = app.paths[static_cast<std::size_t>(index)].name;
  app.paths.erase(app.paths.begin() + index);
  auto adjust = [index](int& value) {
    if (value == index) value = -1;
    else if (value > index) --value;
  };
  adjust(app.operand_a);
  adjust(app.operand_b);
  adjust(app.selected_row);
  adjust(app.trim_path);
  if (app.trim_path < 0) {
    app.trim_mode = false;
    app.trim_vertex_a = -1;
    app.trim_vertex_b = -1;
  }
  app.view.needs_fit = true;
  app.status = "Deleted " + name + ".";
}

void add_rectangle(App& app) {
  const double half_width = app.rectangle_width * 0.5;
  const double half_height = app.rectangle_height * 0.5;
  add_path(app, {{-half_width, -half_height}, {half_width, -half_height},
                 {half_width, half_height}, {-half_width, half_height}},
           true, next_name(app, "Rectangle"));
  app.status = "Added a closed rectangle.";
}

void add_circle(App& app) {
  const int segments = std::clamp(app.circle_segments, 8, 256);
  PathD circle;
  circle.reserve(static_cast<std::size_t>(segments));
  for (int i = 0; i < segments; ++i) {
    const float angle = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(segments);
    circle.push_back({app.circle_radius * std::cos(angle),
                      app.circle_radius * std::sin(angle)});
  }
  add_path(app, std::move(circle), true, next_name(app, "Circle"));
  app.status = "Added a closed circle.";
}

void add_line(App& app) {
  const double h = app.rectangle_width * 0.5;
  add_path(app, {{-h, 0.0}, {h, 0.0}}, false, next_name(app, "Open line"));
  app.status = "Added an open line.";
}

struct Transform {
  ImVec2 center;
  const View& view;

  ImVec2 screen(PointD point) const {
    return {center.x + view.pan.x + static_cast<float>(point.x) * view.zoom,
            center.y + view.pan.y - static_cast<float>(point.y) * view.zoom};
  }

  PointD world(ImVec2 point) const {
    return {(point.x - center.x - view.pan.x) / view.zoom,
            -(point.y - center.y - view.pan.y) / view.zoom};
  }
};

void fit_view(App& app, ImVec2 size) {
  RectD bounds;
  bool have_bounds = false;
  for (const auto& item : app.paths) {
    if (!item.visible || item.points.empty()) continue;
    const RectD item_bounds = GetBounds(item.points);
    if (!have_bounds) {
      bounds = item_bounds;
      have_bounds = true;
    } else {
      bounds.left = std::min(bounds.left, item_bounds.left);
      bounds.right = std::max(bounds.right, item_bounds.right);
      bounds.top = std::min(bounds.top, item_bounds.top);
      bounds.bottom = std::max(bounds.bottom, item_bounds.bottom);
    }
  }
  if (!have_bounds || bounds.right <= bounds.left || bounds.bottom <= bounds.top) return;
  const float width = static_cast<float>(bounds.right - bounds.left);
  const float height = static_cast<float>(bounds.bottom - bounds.top);
  app.view.zoom = std::clamp(0.78f * std::min(size.x / width, size.y / height), 0.05f, 5000.0f);
  const float x = static_cast<float>((bounds.left + bounds.right) * 0.5);
  const float y = static_cast<float>((bounds.top + bounds.bottom) * 0.5);
  app.view.pan = {-x * app.view.zoom, y * app.view.zoom};
  app.view.needs_fit = false;
}

void draw_path(ImDrawList* draw, const PathD& path, bool closed,
               const Transform& transform, ImU32 colour, float width) {
  if (path.size() < 2) return;
  std::vector<ImVec2> points;
  points.reserve(path.size());
  for (const auto& point : path) points.push_back(transform.screen(point));
  draw->AddPolyline(points.data(), static_cast<int>(points.size()), colour, width,
                    closed ? ImDrawFlags_Closed : 0);
  if (!closed) {
    draw->AddCircleFilled(points.front(), width + 1.5f, colour);
    draw->AddCircleFilled(points.back(), width + 1.5f, colour);
  }
}

void draw_grid(ImDrawList* draw, ImVec2 min, ImVec2 max, const Transform& transform) {
  const float raw_unit = 70.0f / transform.view.zoom;
  const float power = std::pow(10.0f, std::floor(std::log10(std::max(raw_unit, 0.00001f))));
  const float unit = (raw_unit / power < 2.0f ? 2.0f : raw_unit / power < 5.0f ? 5.0f : 10.0f) * power;
  const float step = unit * transform.view.zoom;
  const ImVec2 origin = transform.screen({0.0, 0.0});
  float x = origin.x + std::fmod(min.x - origin.x, step);
  float y = origin.y + std::fmod(min.y - origin.y, step);
  for (; x <= max.x; x += step)
    draw->AddLine({x, min.y}, {x, max.y}, IM_COL32(44, 54, 66, 105), 1.0f);
  for (; y <= max.y; y += step)
    draw->AddLine({min.x, y}, {max.x, y}, IM_COL32(44, 54, 66, 105), 1.0f);
  draw->AddLine({origin.x, min.y}, {origin.x, max.y}, IM_COL32(94, 111, 128, 165), 1.0f);
  draw->AddLine({min.x, origin.y}, {max.x, origin.y}, IM_COL32(94, 111, 128, 165), 1.0f);
  draw->AddText({origin.x + 7.0f, origin.y + 6.0f}, IM_COL32(145, 159, 174, 210), "0, 0");
}

void fit_if_needed(App& app, ImVec2 size) {
  if (app.view.needs_fit) fit_view(app, size);
}

int closest_path(const App& app, ImVec2 mouse, const Transform& transform) {
  int best = -1;
  float best_distance = 13.0f;
  for (std::size_t i = 0; i < app.paths.size(); ++i) {
    if (!app.paths[i].visible) continue;
    for (const auto& point : app.paths[i].points) {
      const ImVec2 screen = transform.screen(point);
      const float distance = std::hypot(screen.x - mouse.x, screen.y - mouse.y);
      if (distance < best_distance) {
        best_distance = distance;
        best = static_cast<int>(i);
      }
    }
  }
  return best;
}

int closest_vertex(const App& app, int path_index, ImVec2 mouse,
                   const Transform& transform) {
  if (!valid_operand(app, path_index)) return -1;
  int best = -1;
  float best_distance = 14.0f;
  const auto& path = app.paths[static_cast<std::size_t>(path_index)];
  for (std::size_t i = 0; i < path.points.size(); ++i) {
    const ImVec2 screen = transform.screen(path.points[i]);
    const float distance = std::hypot(screen.x - mouse.x, screen.y - mouse.y);
    if (distance < best_distance) {
      best_distance = distance;
      best = static_cast<int>(i);
    }
  }
  return best;
}

void draw_canvas(App& app) {
  ImGui::BeginChild("canvas", ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  const ImVec2 min = ImGui::GetCursorScreenPos();
  ImVec2 size = ImGui::GetContentRegionAvail();
  size.x = std::max(size.x, 80.0f);
  size.y = std::max(size.y, 80.0f);
  ImGui::InvisibleButton("path-canvas", size,
                         ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
  const bool hovered = ImGui::IsItemHovered();
  ImDrawList* draw = ImGui::GetWindowDrawList();
  const ImVec2 max{min.x + size.x, min.y + size.y};
  draw->AddRectFilled(min, max, IM_COL32(16, 22, 29, 255), 5.0f);
  draw->PushClipRect(min, max, true);
  const ImVec2 center{min.x + size.x * 0.5f, min.y + size.y * 0.5f};
  fit_if_needed(app, size);
  Transform transform{center, app.view};

  if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
    app.view.pan.x += ImGui::GetIO().MouseDelta.x;
    app.view.pan.y += ImGui::GetIO().MouseDelta.y;
  }
  if (hovered && ImGui::GetIO().MouseWheel != 0.0f) {
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    const PointD before = transform.world(mouse);
    app.view.zoom = std::clamp(app.view.zoom * std::pow(1.15f, ImGui::GetIO().MouseWheel),
                               0.05f, 5000.0f);
    const Transform after{center, app.view};
    const ImVec2 anchored = after.screen(before);
    app.view.pan.x += mouse.x - anchored.x;
    app.view.pan.y += mouse.y - anchored.y;
  }

  draw_grid(draw, min, max, transform);
  for (std::size_t i = 0; i < app.paths.size(); ++i) {
    const auto& item = app.paths[i];
    if (!item.visible) continue;
    const bool active = static_cast<int>(i) == app.operand_a || static_cast<int>(i) == app.operand_b;
    draw_path(draw, item.points, item.closed, transform, path_colour(i, active), active ? 3.0f : 1.8f);
  if (static_cast<int>(i) == app.operand_a || static_cast<int>(i) == app.operand_b) {
      const ImU32 marker = static_cast<int>(i) == app.operand_a ? IM_COL32(255, 222, 105, 255)
                                                                : IM_COL32(111, 195, 255, 255);
      for (const auto& point : item.points)
        draw->AddCircleFilled(transform.screen(point), 3.0f, marker);
    }
    if (app.trim_mode && static_cast<int>(i) == app.trim_path) {
      for (std::size_t vertex = 0; vertex < item.points.size(); ++vertex) {
        const ImVec2 screen = transform.screen(item.points[vertex]);
        const bool first = static_cast<int>(vertex) == app.trim_vertex_a;
        const bool second = static_cast<int>(vertex) == app.trim_vertex_b;
        const ImU32 marker = first ? IM_COL32(255, 222, 105, 255)
                                   : second ? IM_COL32(111, 195, 255, 255)
                                            : IM_COL32(177, 187, 198, 220);
        draw->AddCircle(screen, (first || second) ? 8.0f : 5.0f, marker,
                        0, (first || second) ? 2.5f : 1.5f);
        if (first || second)
          draw->AddText({screen.x + 7.0f, screen.y - 7.0f}, marker,
                        first ? "1" : "2");
      }
    }
  }
  if (!app.in_progress.empty()) {
    draw_path(draw, app.in_progress, app.draw_closed, transform, IM_COL32(255, 239, 144, 255), 3.0f);
    for (const auto& point : app.in_progress)
      draw->AddCircleFilled(transform.screen(point), 4.0f, IM_COL32(255, 239, 144, 255));
  }

  if (hovered && app.draw_mode && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    app.in_progress.push_back(transform.world(ImGui::GetIO().MousePos));
    app.status = "Drawing: click more points, press Enter to finish, Esc to cancel.";
  } else if (hovered && app.trim_mode && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    const int vertex = closest_vertex(app, app.trim_path, ImGui::GetIO().MousePos, transform);
    if (vertex < 0) {
      app.status = "Click a visible vertex on operand A.";
    } else if (app.trim_vertex_a < 0) {
      app.trim_vertex_a = vertex;
      app.status = "Selected trim vertex 1. Click a second vertex.";
    } else if (vertex == app.trim_vertex_a) {
      app.status = "Choose a different second vertex.";
    } else if (app.trim_vertex_b < 0) {
      app.trim_vertex_b = vertex;
      app.status = "Selected trim vertices 1 and 2. Keep the segment between them.";
    } else {
      app.trim_vertex_a = vertex;
      app.trim_vertex_b = -1;
      app.status = "Replaced trim vertex 1. Click a second vertex.";
    }
  } else if (hovered && !app.draw_mode && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    const int hit = closest_path(app, ImGui::GetIO().MousePos, transform);
    if (hit >= 0) {
      app.selected_row = hit;
      app.operand_a = hit;
      app.status = "Selected " + app.paths[hit].name + " as operand A.";
    }
  }
  if (app.draw_mode && ImGui::IsKeyPressed(ImGuiKey_Escape)) {
    app.in_progress.clear();
    app.draw_mode = false;
    app.status = "Cancelled drawing.";
  }
  if (app.trim_mode && ImGui::IsKeyPressed(ImGuiKey_Escape)) {
    app.trim_mode = false;
    app.trim_path = -1;
    app.trim_vertex_a = -1;
    app.trim_vertex_b = -1;
    app.status = "Cancelled vertex selection.";
  }
  if (app.draw_mode && ImGui::IsKeyPressed(ImGuiKey_Enter)) {
    if (app.in_progress.size() >= 2) {
      add_path(app, std::move(app.in_progress), app.draw_closed, next_name(app, "Drawn path"));
      app.status = "Finished drawn path.";
    } else {
      app.status = "A path needs at least two points.";
    }
    app.in_progress.clear();
    app.draw_mode = false;
  }
  draw->PopClipRect();
  ImGui::EndChild();
}

bool valid_operand(const App& app, int index) {
  return index >= 0 && index < static_cast<int>(app.paths.size());
}

void add_boolean_result(App& app, const PathsD& closed, const PathsD& open, std::string_view op) {
  const std::size_t before = app.paths.size();
  for (const auto& path : closed)
    add_path(app, path, true, next_name(app, std::string(op) + " result"));
  for (const auto& path : open)
    add_path(app, path, false, next_name(app, std::string(op) + " result"));
  if (app.paths.size() == before) {
    app.status = std::string(op) + " produced no geometry.";
  } else {
    app.status = std::string(op) + " created " + std::to_string(app.paths.size() - before) + " path(s).";
  }
}

void boolean_op(App& app, ClipType type) {
  if (!valid_operand(app, app.operand_a) || !valid_operand(app, app.operand_b) ||
      app.operand_a == app.operand_b) {
    app.status = "Choose two different paths with the A and B buttons first.";
    return;
  }
  const PathItem& a = app.paths[app.operand_a];
  const PathItem& b = app.paths[app.operand_b];
  const std::string op = type == ClipType::Union ? "Union" :
                         type == ClipType::Intersection ? "Intersection" : "Difference";
  ClipperD clipper(4);
  if (type == ClipType::Union) {
    if (a.closed) clipper.AddSubject(PathsD{a.points});
    else clipper.AddOpenSubject(PathsD{a.points});
    if (b.closed) clipper.AddSubject(PathsD{b.points});
    else clipper.AddOpenSubject(PathsD{b.points});
  } else if (!b.closed) {
    if (type == ClipType::Intersection && a.closed) {
      // Open paths can be subjects, while closed paths are clipping regions.
      clipper.AddOpenSubject(PathsD{b.points});
      clipper.AddClip(PathsD{a.points});
    } else {
      app.status = "Clipper2 requires a closed clipping path for this operation.";
      return;
    }
  } else {
    if (a.closed) clipper.AddSubject(PathsD{a.points});
    else clipper.AddOpenSubject(PathsD{a.points});
    clipper.AddClip(PathsD{b.points});
  }
  PathsD closed;
  PathsD open;
  if (!clipper.Execute(type, FillRule::NonZero, closed, open)) {
    app.status = op + " failed inside Clipper2.";
    return;
  }
  add_boolean_result(app, closed, open, op);
}

void inflate(App& app) {
  if (!valid_operand(app, app.operand_a)) {
    app.status = "Choose a path as operand A first.";
    return;
  }
  const auto& source = app.paths[app.operand_a];
  const JoinType join = static_cast<JoinType>(std::clamp(app.join_type, 0, 3));
  const EndType end = source.closed ? EndType::Polygon
                                    : static_cast<EndType>(std::clamp(app.end_type, 0, 3) + 1);
  const PathsD result = InflatePaths(PathsD{source.points}, app.inflate_amount, join, end,
                                     2.0, 4, app.arc_tolerance);
  if (result.empty()) {
    app.status = "Inflate produced no geometry.";
    return;
  }
  const std::size_t before = app.paths.size();
  for (const auto& path : result)
    add_path(app, path, true, next_name(app, "Inflated result"));
  app.status = "Inflated path into " + std::to_string(app.paths.size() - before) + " closed outline(s).";
}

void translate_path(App& app) {
  if (!valid_operand(app, app.operand_a)) {
    app.status = "Choose a path as operand A first.";
    return;
  }
  const PathItem source = app.paths[app.operand_a];
  PathD result = source.points;
  for (auto& point : result) {
    point.x += app.translate_x;
    point.y += app.translate_y;
  }
  add_path(app, std::move(result), source.closed, next_name(app, "Translated result"));
  app.status = "Translated operand A by (" + std::to_string(app.translate_x) + ", " +
               std::to_string(app.translate_y) + ").";
}

void rotate_path(App& app) {
  if (!valid_operand(app, app.operand_a)) {
    app.status = "Choose a path as operand A first.";
    return;
  }
  const PathItem source = app.paths[app.operand_a];
  const double radians = app.rotate_degrees * 3.14159265358979323846 / 180.0;
  const double cosine = std::cos(radians);
  const double sine = std::sin(radians);
  PathD result = source.points;
  for (auto& point : result) {
    const double x = point.x - app.rotate_pivot_x;
    const double y = point.y - app.rotate_pivot_y;
    point.x = app.rotate_pivot_x + x * cosine - y * sine;
    point.y = app.rotate_pivot_y + x * sine + y * cosine;
  }
  add_path(app, std::move(result), source.closed, next_name(app, "Rotated result"));
  app.status = "Rotated operand A by " + std::to_string(app.rotate_degrees) + " degrees.";
}

void convert_closed(App& app, bool closed) {
  if (!valid_operand(app, app.operand_a)) {
    app.status = "Choose a path as operand A first.";
    return;
  }
  app.paths[app.operand_a].closed = closed;
  app.status = closed ? "Converted operand A to closed." : "Converted operand A to open.";
}

void convert_open_preserve(App& app) {
  if (!valid_operand(app, app.operand_a)) {
    app.status = "Choose a path as operand A first.";
    return;
  }
  auto& path = app.paths[app.operand_a];
  if (path.points.size() >= 2 && path.points.front() != path.points.back())
    path.points.push_back(path.points.front());
  path.closed = false;
  app.status = "Converted operand A to open while preserving its visible closing edge.";
}

void keep_between_vertices(App& app) {
  if (!valid_operand(app, app.trim_path) || app.trim_vertex_a < 0 ||
      app.trim_vertex_b < 0 || app.trim_vertex_a == app.trim_vertex_b) {
    app.status = "Pick two different vertices on operand A first.";
    return;
  }
  const PathItem source = app.paths[static_cast<std::size_t>(app.trim_path)];
  const int first = app.trim_vertex_a;
  const int second = app.trim_vertex_b;
  if (first >= static_cast<int>(source.points.size()) ||
      second >= static_cast<int>(source.points.size())) {
    app.status = "The selected vertices are no longer available.";
    return;
  }

  auto slice = [&](int from, int to, bool wrap) {
    PathD result;
    int index = from;
    result.push_back(source.points[static_cast<std::size_t>(index)]);
    while (index != to) {
      if (wrap) index = (index + 1) % static_cast<int>(source.points.size());
      else ++index;
      result.push_back(source.points[static_cast<std::size_t>(index)]);
    }
    return result;
  };

  if (source.closed) {
    add_path(app, slice(first, second, true), false,
             next_name(app, "Trim result"));
    add_path(app, slice(second, first, true), false,
             next_name(app, "Trim result"));
    app.status = "Created both open curves between the selected vertices.";
  } else {
    const int from = std::min(first, second);
    const int to = std::max(first, second);
    add_path(app, slice(from, to, false), false,
             next_name(app, "Trim result"));
    app.status = "Created the open curve between the selected vertices.";
  }
  app.trim_mode = false;
  app.trim_path = -1;
  app.trim_vertex_a = -1;
  app.trim_vertex_b = -1;
}

std::string clipboard_text(const PathItem& item) {
  std::ostringstream out;
  out << (item.closed ? "closed" : "open") << '\n' << std::setprecision(17);
  for (const auto& point : item.points) out << point.x << ' ' << point.y << '\n';
  return out.str();
}

void copy_path(App& app) {
  if (!valid_operand(app, app.operand_a)) {
    app.status = "Choose a path as operand A first.";
    return;
  }
  const std::string text = clipboard_text(app.paths[app.operand_a]);
  if (SDL_SetClipboardText(text.c_str()) != 0) {
    app.status = std::string("Clipboard error: ") + SDL_GetError();
    return;
  }
  app.status = "Copied operand A to the system clipboard.";
}

void paste_path(App& app) {
  char* raw = SDL_GetClipboardText();
  if (!raw) {
    app.status = "Clipboard is empty.";
    return;
  }
  std::istringstream in(raw);
  SDL_free(raw);
  std::string kind;
  std::getline(in, kind);
  PathD points;
  double x = 0.0;
  double y = 0.0;
  while (in >> x >> y) points.push_back({x, y});
  if (points.size() < 2 || (kind != "open" && kind != "closed")) {
    app.status = "Clipboard does not contain a Path Lab path.";
    return;
  }
  add_path(app, std::move(points), kind == "closed", next_name(app, "Pasted path"));
  app.status = "Pasted a path from the system clipboard.";
}

int parse_obj_index(std::string token, std::size_t count) {
  const std::size_t slash = token.find('/');
  if (slash != std::string::npos) token.resize(slash);
  const int value = std::stoi(token);
  if (value > 0) return value - 1;
  if (value < 0) return static_cast<int>(count) + value;
  return -1;
}

void import_obj(App& app) {
  std::ifstream in(app.import_path);
  if (!in) {
    app.status = std::string("Could not open ") + app.import_path;
    return;
  }
  std::vector<PointD> vertices;
  std::vector<PathD> imported;
  std::vector<bool> imported_closed;
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream row(line);
    std::string tag;
    row >> tag;
    if (tag == "v") {
      double x = 0.0, y = 0.0, ignored = 0.0;
      if (row >> x >> y) {
        row >> ignored;
        vertices.push_back({x, y});
      }
    } else if (tag == "l") {
      PathD path;
      std::string token;
      while (row >> token) {
        try {
          const int index = parse_obj_index(token, vertices.size());
          if (index >= 0 && index < static_cast<int>(vertices.size())) path.push_back(vertices[index]);
        } catch (const std::exception&) {
          path.clear();
          break;
        }
      }
      if (path.size() >= 2) {
        const bool closed = path.front() == path.back();
        if (closed) path.pop_back();
        imported.push_back(std::move(path));
        imported_closed.push_back(closed);
      }
    }
  }
  if (imported.empty()) {
    app.status = "OBJ contained no usable l (polyline) records.";
    return;
  }
  for (std::size_t i = 0; i < imported.size(); ++i)
    add_path(app, std::move(imported[i]), imported_closed[i], next_name(app, "Imported path"));
  app.status = "Imported " + std::to_string(imported.size()) + " path(s) from OBJ.";
}

std::string obj_name(std::string name) {
  for (char& character : name)
    if (character == ' ' || character == '\t') character = '_';
  return name;
}

void export_obj(App& app) {
  std::ofstream out(app.export_path);
  if (!out) {
    app.status = std::string("Could not write ") + app.export_path;
    return;
  }
  out << "# Clipper Path Lab raw curves\n" << std::setprecision(17);
  std::size_t vertex = 1;
  std::size_t count = 0;
  for (const auto& item : app.paths) {
    if (!item.visible || item.points.size() < 2) continue;
    out << "g " << obj_name(item.name) << '\n';
    const std::size_t first = vertex;
    for (const auto& point : item.points) {
      out << "v " << point.x << ' ' << point.y << " 0\n";
      ++vertex;
    }
    out << "l";
    for (std::size_t i = first; i < vertex; ++i) out << ' ' << i;
    if (item.closed) out << ' ' << first;
    out << '\n';
    ++count;
  }
  app.status = "Exported " + std::to_string(count) + " path(s) to OBJ.";
}

bool draw_path_row(App& app, int index) {
  auto& item = app.paths[static_cast<std::size_t>(index)];
  ImGui::PushID(index);
  ImGui::Checkbox("##visible", &item.visible);
  ImGui::SameLine();
  const bool selected = app.selected_row == index;
  if (ImGui::Selectable(item.name.c_str(), selected, ImGuiSelectableFlags_AllowOverlap)) {
    app.selected_row = index;
    app.operand_a = index;
    app.status = "Selected " + item.name + " as A. Use Set B on another row for the clip.";
  }
  ImGui::SameLine();
  if (ImGui::SmallButton(app.operand_a == index ? "A*" : "Set A")) {
    app.operand_a = index;
    app.selected_row = index;
    app.status = item.name + " is now operand A (subject).";
  }
  ImGui::SameLine();
  if (ImGui::SmallButton(app.operand_b == index ? "B*" : "Set B")) {
    app.operand_b = index;
    app.status = item.name + " is now operand B (clip).";
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Delete")) {
    delete_path(app, index);
    ImGui::PopID();
    return true;
  }
  ImGui::SameLine();
  ImGui::TextDisabled("%s · %zu pts", item.closed ? "closed" : "open", item.points.size());
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("A* and B* show the current boolean operands");
  ImGui::PopID();
  return false;
}

void controls(App& app) {
  ImGui::TextUnformatted("CLIPPER PATH LAB");
  ImGui::TextDisabled("Interactive polygon and curve operations");
  ImGui::SeparatorText("Primitives");
  ImGui::DragFloat("Rectangle width", &app.rectangle_width, 0.25f, 1.0f, 10000.0f, "%.2f");
  ImGui::DragFloat("Rectangle height", &app.rectangle_height, 0.25f, 1.0f, 10000.0f, "%.2f");
  ImGui::DragFloat("Circle radius", &app.circle_radius, 0.25f, 1.0f, 10000.0f, "%.2f");
  ImGui::DragInt("Circle segments", &app.circle_segments, 1.0f, 8, 256);
  if (ImGui::Button("Add rectangle", ImVec2(-1, 0))) add_rectangle(app);
  if (ImGui::Button("Add circle", ImVec2(-1, 0))) add_circle(app);
  if (ImGui::Button("Add open line", ImVec2(-1, 0))) add_line(app);
  ImGui::Checkbox("Closed drawn path", &app.draw_closed);
  if (ImGui::Button(app.draw_mode ? "Finish / Enter" : "Draw path", ImVec2(-1, 0))) {
    if (app.draw_mode) {
      if (app.in_progress.size() >= 2)
        add_path(app, std::move(app.in_progress), app.draw_closed, next_name(app, "Drawn path"));
      app.in_progress.clear();
      app.draw_mode = false;
      app.status = "Finished drawn path.";
    } else {
      app.draw_mode = true;
      app.in_progress.clear();
      app.status = "Drawing: click points on the canvas, then press Enter.";
    }
  }
  if (app.draw_mode) ImGui::TextColored(ImVec4(1.0f, 0.86f, 0.38f, 1.0f), "Esc cancels drawing");

  ImGui::SeparatorText("Paths");
  ImGui::TextWrapped("Assign operands explicitly: use Set A for the subject and Set B for the clip. Difference means A - B.");
  if (app.paths.empty()) {
    ImGui::TextDisabled("No paths yet.");
  } else {
    for (int i = 0; i < static_cast<int>(app.paths.size());) {
      if (!draw_path_row(app, i)) ++i;
    }
  }
  ImGui::TextColored(ImVec4(1.0f, 0.86f, 0.35f, 1.0f), "A / subject: %s",
                     valid_operand(app, app.operand_a) ? app.paths[app.operand_a].name.c_str() : "not assigned");
  ImGui::TextColored(ImVec4(0.45f, 0.78f, 1.0f, 1.0f), "B / clip: %s",
                     valid_operand(app, app.operand_b) ? app.paths[app.operand_b].name.c_str() : "not assigned");

  ImGui::SeparatorText("Clipper2 operations");
  if (ImGui::Button("Union", ImVec2(-1, 0))) boolean_op(app, ClipType::Union);
  if (ImGui::Button("Intersect", ImVec2(-1, 0))) boolean_op(app, ClipType::Intersection);
  if (ImGui::Button("Difference  A - B", ImVec2(-1, 0))) boolean_op(app, ClipType::Difference);
  ImGui::DragFloat("Offset / inflate", &app.inflate_amount, 0.1f, -10000.0f, 10000.0f, "%+.2f");
  ImGui::DragFloat("Arc tolerance", &app.arc_tolerance, 0.005f, 0.01f, 5.0f, "%.3f");
  ImGui::TextDisabled("Smaller tolerance = more detailed rounded corners");
  const char* joins[] = {"Square", "Bevel", "Round", "Miter"};
  ImGui::Combo("Join", &app.join_type, joins, 4);
  const char* ends[] = {"Joined", "Butt", "Square", "Round"};
  ImGui::Combo("Open ends", &app.end_type, ends, 4);
  if (ImGui::Button("Inflate A", ImVec2(-1, 0))) inflate(app);

  ImGui::SeparatorText("Transform operations");
  ImGui::DragFloat("Translate X", &app.translate_x, 0.25f, -10000.0f, 10000.0f, "%+.2f");
  ImGui::DragFloat("Translate Y", &app.translate_y, 0.25f, -10000.0f, 10000.0f, "%+.2f");
  if (ImGui::Button("Translate A", ImVec2(-1, 0))) translate_path(app);
  ImGui::DragFloat("Rotate degrees", &app.rotate_degrees, 0.5f, -3600.0f, 3600.0f, "%+.2f");
  ImGui::DragFloat("Pivot X", &app.rotate_pivot_x, 0.25f, -10000.0f, 10000.0f, "%+.2f");
  ImGui::DragFloat("Pivot Y", &app.rotate_pivot_y, 0.25f, -10000.0f, 10000.0f, "%+.2f");
  ImGui::TextDisabled("Rotation is counter-clockwise around the pivot");
  if (ImGui::Button("Rotate A", ImVec2(-1, 0))) rotate_path(app);

  ImGui::SeparatorText("Path tools");
  if (ImGui::Button("Copy A")) copy_path(app);
  ImGui::SameLine();
  if (ImGui::Button("Paste")) paste_path(app);
  if (ImGui::Button("Duplicate A", ImVec2(-1, 0)) && valid_operand(app, app.operand_a)) {
    const auto source = app.paths[app.operand_a];
    add_path(app, source.points, source.closed, next_name(app, "Copy"));
    app.status = "Duplicated operand A.";
  }
  if (ImGui::Button("Convert A to open", ImVec2(-1, 0))) convert_closed(app, false);
  if (ImGui::Button("Convert A to open (preserve)", ImVec2(-1, 0)))
    convert_open_preserve(app);
  if (ImGui::Button("Convert A to closed", ImVec2(-1, 0))) convert_closed(app, true);
  if (ImGui::Button("Pick vertices on A", ImVec2(-1, 0))) {
    if (!valid_operand(app, app.operand_a) || app.paths[app.operand_a].points.size() < 2) {
      app.status = "Choose a path with at least two vertices as operand A first.";
    } else {
      app.trim_mode = true;
      app.trim_path = app.operand_a;
      app.trim_vertex_a = -1;
      app.trim_vertex_b = -1;
      app.status = "Pick two vertices on A; closed paths will produce both arcs.";
    }
  }
  if (app.trim_mode) {
    ImGui::TextColored(ImVec4(1.0f, 0.86f, 0.38f, 1.0f),
                       "Vertex picks: %s, %s",
                       app.trim_vertex_a >= 0 ? "1 selected" : "pick 1",
                       app.trim_vertex_b >= 0 ? "2 selected" : "pick 2");
    if (ImGui::Button("Keep between selected", ImVec2(-1, 0)))
      keep_between_vertices(app);
  }
  if (ImGui::Button("Fit view", ImVec2(-1, 0))) app.view.needs_fit = true;

  ImGui::SeparatorText("OBJ raw curves");
  ImGui::InputTextWithHint("##import", "Import path.obj", app.import_path, sizeof(app.import_path));
  if (ImGui::Button("Import OBJ", ImVec2(-1, 0))) import_obj(app);
  ImGui::InputTextWithHint("##export", "Export path.obj", app.export_path, sizeof(app.export_path));
  if (ImGui::Button("Export visible OBJ", ImVec2(-1, 0))) export_obj(app);

  ImGui::SeparatorText("Interface");
  if (ImGui::SliderFloat("GUI scale", &app.ui_scale, 0.75f, 1.75f, "%.2fx"))
    app.status = "GUI scale updates on the next frame.";

  ImGui::Separator();
  ImGui::TextWrapped("%s", app.status.c_str());
  ImGui::TextDisabled("Right-drag pan  |  wheel zoom  |  click a path selects A");
}

void init_demo(App& app) {
  add_path(app, {{-35.0, -25.0}, {15.0, -25.0}, {15.0, 25.0}, {-35.0, 25.0}},
           true, "Demo rectangle");
  PathD circle;
  for (int i = 0; i < 40; ++i) {
    const double angle = 2.0 * 3.141592653589793 * i / 40.0;
    circle.push_back({25.0 + 18.0 * std::cos(angle), 0.0 + 18.0 * std::sin(angle)});
  }
  add_path(app, std::move(circle), true, "Demo circle");
  app.operand_a = 0;
  app.operand_b = 1;
  app.status = "Demo paths loaded. Select A/B and try Difference.";
}

} // namespace

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
    std::fprintf(stderr, "SDL initialization failed: %s\n", SDL_GetError());
    return 1;
  }
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 0);
  SDL_Window* window = SDL_CreateWindow(
      "Clipper Path Lab", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1420, 860,
      SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
  if (!window) {
    std::fprintf(stderr, "Window creation failed: %s\n", SDL_GetError());
    SDL_Quit();
    return 1;
  }
  SDL_GLContext context = SDL_GL_CreateContext(window);
  if (!context) {
    std::fprintf(stderr, "OpenGL context failed: %s\n", SDL_GetError());
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }
  SDL_GL_MakeCurrent(window, context);
  SDL_GL_SetSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::GetIO().IniFilename = nullptr;
  ImGui_ImplSDL2_InitForOpenGL(window, context);
  ImGui_ImplOpenGL3_Init("#version 150");

  App app;
  app.base_style = ImGui::GetStyle();
  init_demo(app);
  bool running = true;
  while (running) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT ||
          (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE))
        running = false;
    }
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::GetStyle() = app.base_style;
    ImGui::GetStyle().ScaleAllSizes(app.ui_scale);
    ImGui::GetIO().FontGlobalScale = app.ui_scale;
    ImGui::NewFrame();
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::Begin("Clipper Path Lab", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings);
    ImGui::BeginChild("controls", ImVec2(350.0f * app.ui_scale, 0.0f), ImGuiChildFlags_Borders);
    controls(app);
    ImGui::EndChild();
    ImGui::SameLine();
    draw_canvas(app);
    ImGui::End();
    ImGui::Render();
    int width = 0;
    int height = 0;
    SDL_GL_GetDrawableSize(window, &width, &height);
    glViewport(0, 0, width, height);
    glClearColor(0.035f, 0.047f, 0.063f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
