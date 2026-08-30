// TingleCAM: adaptive roughing over the wavefront engine, with tool and machine
// libraries and feeds driven by what the material simulation says is actually
// being removed.

#include "cam/config.h"
#include "cam/feeds.h"
#include "cam/job.h"
#include "cam/post.h"

#include "clipper2/clipper.triangulation.h"

#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdl2.h"

#include <SDL.h>
#include <SDL_opengl.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clipper2Lib::Paths64;
using Clipper2Lib::Point64;

// The control column. Everything in it is laid out against this, so widening
// the panel widens the widgets and their labels together.
constexpr float kSidebarWidth = 520.0f;

// ---------------------------------------------------------------------------
// Background planning
// ---------------------------------------------------------------------------

// Planning a job of any size takes long enough that doing it on the frame
// thread would freeze the window, so it runs on its own thread and reports back
// through here. Everything the worker touches is either atomic or under the
// mutex; the plan itself is handed over once, at the end.
class Planner {
 public:
  ~Planner() { cancel_and_join(); }

  bool running() const { return running_.load(); }
  double fraction() const { return fraction_.load(); }
  std::string stage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stage_;
  }

  void start(cam::JobGeometry& geometry, cam::Job job, cam::ToolLibrary tools,
             cam::MachineLibrary machines, cam::MaterialLibrary materials) {
    cancel_and_join();
    cancelled_.store(false);
    running_.store(true);
    fraction_.store(0.0);
    finished_.store(false);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stage_ = "Starting";
    }
    worker_ = std::thread([this, &geometry, job = std::move(job),
                           tools = std::move(tools), machines = std::move(machines),
                           materials = std::move(materials)]() mutable {
      cam::JobPlan result;
      try {
        result = cam::plan_job(
            geometry, job, tools, machines, materials,
            [this](double fraction, const std::string& stage) {
              fraction_.store(fraction);
              std::lock_guard<std::mutex> lock(mutex_);
              stage_ = stage;
              return !cancelled_.load();
            });
      } catch (const std::exception& error) {
        result = {};
        result.error = error.what();
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        plan_ = std::move(result);
      }
      finished_.store(true);
      running_.store(false);
    });
  }

  void cancel() { cancelled_.store(true); }

  // True once, when a finished plan is ready to be taken.
  bool take(cam::JobPlan& into) {
    if (!finished_.exchange(false)) return false;
    if (worker_.joinable()) worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    into = std::move(plan_);
    return true;
  }

 private:
  void cancel_and_join() {
    cancelled_.store(true);
    if (worker_.joinable()) worker_.join();
    running_.store(false);
  }

  mutable std::mutex mutex_;
  std::thread worker_;
  std::atomic<bool> running_{false};
  std::atomic<bool> finished_{false};
  std::atomic<bool> cancelled_{false};
  std::atomic<double> fraction_{0.0};
  std::string stage_;
  cam::JobPlan plan_;
};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

enum class Colouring { Feed, Limit, Load, Move };
enum class ViewMode { Plan, Result };

struct View {
  ImVec2 pan{0.0f, 0.0f};
  float zoom = 4.0f;
  bool needs_fit = true;
};

// The stacked result view. Orthographic, painter-sorted onto the draw list, the
// same way the wave demo shows its slice, because a depth buffer and shaders
// would be a lot of machinery for a dozen extruded outlines.
struct Orbit {
  float yaw = 0.85f;
  float pitch = 0.62f;
  float zoom = 1.0f;
};

// Playback along one pass. The distance is measured along that pass's toolpath,
// which is what the simulation snapshots are indexed by.
struct Playback {
  bool playing = false;
  double distance = 0.0;
  double speed = 40.0;  // mm of toolpath per second of wall clock
  // Run the cutter at the feeds the plan actually commands rather than at a
  // fixed distance per second. Those feeds came out of what the simulation
  // measured the cutter removing, so this is the job at its real pace: it
  // crawls where the engagement is heavy and opens out where it is not.
  bool real_time = true;
  double time_scale = 1.0;  // multiples of real time
  bool follow = true;   // keep the whole job's passes advancing, not just one
};

struct App {
  cam::ToolLibrary tools;
  cam::MachineLibrary machines;
  cam::MaterialLibrary materials;
  cam::Job job;
  cam::JobGeometry geometry;
  cam::JobPlan plan;
  bool has_plan = false;

  Planner planner;
  std::string banner;
  bool banner_is_error = false;

  std::array<char, 512> part_path{};
  std::array<char, 512> stock_path{};
  int selected_operation = 0;
  int selected_level = 0;
  int selected_plan_operation = 0;
  Colouring colouring = Colouring::Feed;
  ViewMode mode = ViewMode::Plan;
  // Set from the command line so a screenshot can be told which view to take.
  int force_tab = -1;
  bool show_stock = true;
  bool show_part = true;
  bool show_rapids = true;
  bool show_material = true;
  bool show_toolpath = true;
  // The stacked view shows the finished part unless it is asked to follow the
  // cutter, which is the more useful default: most of the time the question is
  // what comes off the machine, not what it looks like part way through.
  bool stack_follows_playback = false;
  // Regions the triangulator refused this frame, so a fill that silently went
  // missing is reported rather than left to be spotted by eye.
  std::size_t fill_failures = 0;
  View view;
  Orbit orbit;
  Playback playback;
  // Remaining stock for the pass being watched, recomputed only when the
  // playback distance actually moves.
  Paths64 material;
  double material_distance = -1.0;
  int material_level = -1;
  int material_operation = -1;

  // Tool Builder
  bool builder_open = false;
  bool builder_is_new = false;
  cam::Tool builder_tool;
  int builder_material = 0;

  std::string gcode;
  cam::PostResult post;
  std::array<char, 512> save_path{};

  std::filesystem::path tools_file;
};

void say(App& app, std::string text, bool error = false) {
  app.banner = std::move(text);
  app.banner_is_error = error;
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

struct Transform {
  ImVec2 origin;
  float scale = 1.0f;
  ImVec2 operator()(const Point64& point) const {
    return {origin.x + static_cast<float>(wave::to_world(point.x)) * scale,
            origin.y - static_cast<float>(wave::to_world(point.y)) * scale};
  }
};

// Vertices far enough apart to be worth keeping. A ring straight out of an
// offset carries a vertex every few hundredths of a millimetre, and nothing
// downstream -- extrusion, triangulation, the fill itself -- gets anything for
// them but work.
Clipper2Lib::Path64 thinned(const Clipper2Lib::Path64& ring, double spacing) {
  Clipper2Lib::Path64 points;
  points.reserve(ring.size());
  for (const auto& vertex : ring) {
    if (!points.empty()) {
      const double dx = wave::to_world(vertex.x - points.back().x);
      const double dy = wave::to_world(vertex.y - points.back().y);
      if (dx * dx + dy * dy < spacing * spacing) continue;
    }
    points.push_back(vertex);
  }
  return points;
}

Paths64 thinned(const Paths64& paths, double spacing) {
  Paths64 out;
  out.reserve(paths.size());
  for (const auto& ring : paths) {
    Clipper2Lib::Path64 kept = thinned(ring, spacing);
    if (kept.size() >= 3) out.push_back(std::move(kept));
  }
  return out;
}

// A region ready to be drawn: decimated to what the view can resolve, then put
// back through Clipper. That second step is not optional. Decimation can leave
// a ring crossing itself on a tight concave feature, and only Clipper turns
// whatever comes out back into simple, correctly oriented, properly nested
// rings, which is what both the triangulator and the difference below need.
Paths64 drawable_region(const Paths64& region, double spacing) {
  return Clipper2Lib::Union(thinned(region, spacing), Clipper2Lib::FillRule::NonZero);
}

// The same region cut into triangles by Clipper2's constrained Delaunay.
//
// Every filled thing in this program goes through here, for two reasons. Holes
// come out as holes, so a pocket needs no painting-over with the background
// colour -- a trick that only ever worked against a flat ground. And a region
// that cannot be triangulated is counted and reported, where an ear clipper
// handed a ring it cannot cut returns arbitrary triangles and says nothing.
Paths64 fill_mesh(const Paths64& region, std::size_t* failures = nullptr) {
  Paths64 mesh;
  if (region.empty()) return mesh;
  const Clipper2Lib::TriangulateResult result = Clipper2Lib::Triangulate(region, mesh);
  if (failures != nullptr && result != Clipper2Lib::TriangulateResult::success &&
      result != Clipper2Lib::TriangulateResult::no_polygons)
    ++*failures;
  return mesh;
}

void draw_paths(ImDrawList* list, const Paths64& paths, const Transform& to_screen,
                ImU32 colour, float width) {
  for (const auto& ring : paths) {
    if (ring.size() < 2) continue;
    for (std::size_t i = 0; i < ring.size(); ++i)
      list->AddLine(to_screen(ring[i]), to_screen(ring[(i + 1) % ring.size()]), colour,
                    width);
  }
}

// Solid material, holes and all, as triangles. A void is absent from the mesh
// rather than painted over in the background colour, so material drawn on top
// of anything else still reads correctly.
void fill_paths(ImDrawList* list, const Paths64& paths, const Transform& to_screen,
                ImU32 solid, std::size_t* failures = nullptr) {
  // One vertex per pixel and a half is all this view can show.
  const double spacing = 1.5 / std::max(to_screen.scale, 1e-6f);
  const Paths64 mesh = fill_mesh(drawable_region(paths, spacing), failures);
  for (const auto& triangle : mesh) {
    if (triangle.size() != 3) continue;
    list->AddTriangleFilled(to_screen(triangle[0]), to_screen(triangle[1]),
                            to_screen(triangle[2]), solid);
  }
}

ImU32 blend(ImU32 low, ImU32 high, float t) {
  const ImVec4 a = ImGui::ColorConvertU32ToFloat4(low);
  const ImVec4 b = ImGui::ColorConvertU32ToFloat4(high);
  t = std::clamp(t, 0.0f, 1.0f);
  return ImGui::ColorConvertFloat4ToU32(
      ImVec4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, 1.0f));
}

ImU32 limit_colour(cam::FeedLimit limit) {
  switch (limit) {
    case cam::FeedLimit::Engagement: return IM_COL32(90, 200, 140, 255);
    case cam::FeedLimit::Rapid: return IM_COL32(80, 95, 115, 255);
    case cam::FeedLimit::Plunge: return IM_COL32(255, 170, 60, 255);
  }
  return IM_COL32(200, 200, 200, 255);
}

ImU32 move_colour(wave::MoveKind kind) {
  switch (kind) {
    case wave::MoveKind::Cut: return IM_COL32(120, 220, 160, 255);
    case wave::MoveKind::Step: return IM_COL32(255, 190, 90, 255);
    case wave::MoveKind::Link: return IM_COL32(110, 180, 255, 255);
    case wave::MoveKind::Retract: return IM_COL32(90, 105, 130, 255);
    case wave::MoveKind::Entry: return IM_COL32(255, 100, 100, 255);
  }
  return IM_COL32(200, 200, 200, 255);
}

const cam::LevelPlan* current_level(const App& app) {
  if (!app.has_plan) return nullptr;
  if (app.selected_plan_operation < 0 ||
      app.selected_plan_operation >= static_cast<int>(app.plan.operations.size()))
    return nullptr;
  const auto& levels = app.plan.operations[app.selected_plan_operation].levels;
  if (app.selected_level < 0 || app.selected_level >= static_cast<int>(levels.size()))
    return nullptr;
  return &levels[app.selected_level];
}

double level_length(const cam::LevelPlan* level) {
  return level ? wave::toolpath_length(level->toolpath) : 0.0;
}

// The remaining stock for the pass being watched. Recomputed only when the
// playback has actually moved, since each call redoes the current move's
// booleans and there is no sense doing that for a still frame.
void refresh_material(App& app) {
  const cam::LevelPlan* level = current_level(app);
  if (!level || level->nothing_to_cut) {
    app.material.clear();
    app.material_distance = -1.0;
    return;
  }
  if (app.material_operation == app.selected_plan_operation &&
      app.material_level == app.selected_level &&
      std::abs(app.material_distance - app.playback.distance) < 1e-9)
    return;
  try {
    app.material = wave::simulation_stock_at(level->simulation, level->toolpath,
                                             app.playback.distance);
  } catch (const std::exception&) {
    app.material = level->simulation.remaining_stock;
  }
  app.material_distance = app.playback.distance;
  app.material_level = app.selected_level;
  app.material_operation = app.selected_plan_operation;
}

// The segment covering a distance along the pass, or the end of the schedule.
// Segments are laid down in path order, so the search is a bisection on where
// each one starts.
std::size_t segment_at(const std::vector<cam::FeedSegment>& segments, double distance) {
  auto it = std::upper_bound(segments.begin(), segments.end(), distance,
                             [](double value, const cam::FeedSegment& segment) {
                               return value < segment.start_distance;
                             });
  if (it == segments.begin()) return 0;
  return static_cast<std::size_t>(std::distance(segments.begin(), it) - 1);
}

// m:ss for anything over a minute, seconds with a decimal below it. A pass is
// usually tens of seconds and a job is minutes, and one format reads badly for
// both.
std::string clock_text(double seconds) {
  char buffer[32];
  if (seconds < 60.0) std::snprintf(buffer, sizeof(buffer), "%.1fs", seconds);
  else std::snprintf(buffer, sizeof(buffer), "%d:%04.1f", static_cast<int>(seconds / 60.0),
                     std::fmod(seconds, 60.0));
  return buffer;
}

// Playback runs along one pass and then falls into the next, so the whole job
// can be watched without driving the slider by hand.
void advance_playback(App& app, double seconds) {
  if (!app.playback.playing || !app.has_plan) return;
  // Preserve the budget's unit across pass boundaries. Machine-time playback
  // carries seconds into the next pass; treating them as millimetres makes the
  // cutter jump whenever a frame crosses a boundary.
  double budget = app.playback.real_time
                      ? seconds * app.playback.time_scale
                      : seconds * app.playback.speed;
  while (budget > 0.0) {
    const cam::LevelPlan* level = current_level(app);
    if (!level) {
      app.playback.playing = false;
      return;
    }
    const double total = level_length(level);
    double left_in_pass = 0.0;
    if (app.playback.real_time) {
      left_in_pass = std::max(
          cam::feed_schedule_seconds(level->feeds) -
              cam::feed_seconds_to(level->feeds, app.playback.distance),
          0.0);
      if (budget < left_in_pass) {
        app.playback.distance = cam::feed_distance_after(
            level->feeds, app.playback.distance, budget);
        return;
      }
    } else {
      left_in_pass = std::max(total - app.playback.distance, 0.0);
      if (budget < left_in_pass) {
        app.playback.distance += budget;
        return;
      }
    }
    budget -= left_in_pass;
    app.playback.distance = total;

    if (!app.playback.follow) {
      app.playback.distance = 0.0;
      return;
    }
    const auto& levels = app.plan.operations[app.selected_plan_operation].levels;
    int next = app.selected_level + 1;
    while (next < static_cast<int>(levels.size()) && levels[next].nothing_to_cut)
      ++next;
    if (next < static_cast<int>(levels.size())) {
      app.selected_level = next;
      app.playback.distance = 0.0;
      refresh_material(app);
      continue;
    }
    int operation = app.selected_plan_operation + 1;
    while (operation < static_cast<int>(app.plan.operations.size()) &&
           app.plan.operations[operation].levels.empty())
      ++operation;
    if (operation < static_cast<int>(app.plan.operations.size())) {
      app.selected_plan_operation = operation;
      app.selected_level = 0;
      app.playback.distance = 0.0;
      refresh_material(app);
      continue;
    }
    app.playback.playing = false;
    return;
  }
}

void fit_view(App& app, ImVec2 canvas) {
  const cam::LevelPlan* level = current_level(app);
  if (!level) return;
  // Everything the pass involves, not just the stock: a lead-in stands outside
  // the stock and a part can reach past it, and framing on the stock alone
  // crops both off the edge of the view.
  Paths64 reference = level->stock_slice;
  reference.insert(reference.end(), level->part_slice.begin(),
                   level->part_slice.end());
  for (const auto& segment : level->feeds.segments)
    reference.push_back(Clipper2Lib::Path64{segment.from, segment.to});
  if (reference.empty()) return;
  const auto bounds = Clipper2Lib::GetBounds(reference);
  const float width = static_cast<float>(wave::to_world(bounds.right - bounds.left));
  const float height = static_cast<float>(wave::to_world(bounds.bottom - bounds.top));
  if (!(width > 0.0f) || !(height > 0.0f)) return;
  app.view.zoom = 0.85f * std::min(canvas.x / width, canvas.y / height);
  const float centre_x =
      static_cast<float>(wave::to_world(bounds.left + bounds.right)) * 0.5f;
  const float centre_y =
      static_cast<float>(wave::to_world(bounds.top + bounds.bottom)) * 0.5f;
  app.view.pan = {-centre_x * app.view.zoom, centre_y * app.view.zoom};
  app.view.needs_fit = false;
}

void draw_canvas(App& app) {
  ImGui::BeginChild("canvas", ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  const ImVec2 top_left = ImGui::GetCursorScreenPos();
  ImVec2 size = ImGui::GetContentRegionAvail();
  size.x = std::max(size.x, 50.0f);
  size.y = std::max(size.y, 50.0f);
  ImGui::InvisibleButton("plot", size,
                         ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
  const bool hovered = ImGui::IsItemHovered();
  ImDrawList* list = ImGui::GetWindowDrawList();
  const ImVec2 bottom_right{top_left.x + size.x, top_left.y + size.y};
  list->AddRectFilled(top_left, bottom_right, IM_COL32(16, 19, 25, 255));
  list->PushClipRect(top_left, bottom_right, true);

  if (app.view.needs_fit) fit_view(app, size);

  if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
    app.view.pan.x += ImGui::GetIO().MouseDelta.x;
    app.view.pan.y += ImGui::GetIO().MouseDelta.y;
  }
  if (hovered && ImGui::GetIO().MouseWheel != 0.0f) {
    const float factor = std::pow(1.12f, ImGui::GetIO().MouseWheel);
    app.view.zoom = std::clamp(app.view.zoom * factor, 0.02f, 4000.0f);
    app.view.pan.x *= factor;
    app.view.pan.y *= factor;
  }

  const Transform to_screen{
      {top_left.x + size.x * 0.5f + app.view.pan.x,
       top_left.y + size.y * 0.5f + app.view.pan.y},
      app.view.zoom};

  app.fill_failures = 0;

  const cam::LevelPlan* level = current_level(app);
  if (level) {
    // What is left of the stock, drawn as the solid it is rather than as an
    // outline. This is the whole point of simulating: the outline of a pocket
    // and the outline of an island look the same until one of them is filled.
    if (app.show_material && !app.material.empty())
      fill_paths(list, app.material, to_screen, IM_COL32(52, 78, 104, 255),
                 &app.fill_failures);
    if (app.show_stock)
      draw_paths(list, level->stock_slice, to_screen, IM_COL32(70, 130, 165, 200), 1.6f);
    if (app.show_part)
      draw_paths(list, level->part_slice, to_screen, IM_COL32(235, 200, 90, 220), 2.0f);

    double lowest = 0.0;
    double highest = 0.0;
    for (const auto& segment : level->feeds.segments) {
      if (segment.rapid) continue;
      highest = std::max(highest, segment.feed);
      lowest = lowest > 0.0 ? std::min(lowest, segment.feed) : segment.feed;
    }
    const double span = std::max(highest - lowest, 1e-6);

    // Where the cutter has got to, in the same units the segments carry.
    const double travelled = app.playback.distance;
    double walked = 0.0;
    for (const auto& segment : level->feeds.segments) {
      if (!app.show_toolpath) break;
      const double segment_start = walked;
      walked += segment.length;
      if (segment.rapid && !app.show_rapids) continue;
      // Anything still ahead of the cutter is drawn faintly, so the part of the
      // pass that has happened reads apart from the part that has not.
      const bool done = segment_start < travelled;
      ImU32 colour = IM_COL32(150, 150, 150, 255);
      float width = 1.7f;
      switch (app.colouring) {
        case Colouring::Feed:
          colour = segment.rapid
                       ? IM_COL32(70, 82, 100, 255)
                       : blend(IM_COL32(60, 110, 230, 255), IM_COL32(120, 235, 140, 255),
                               static_cast<float>((segment.feed - lowest) / span));
          break;
        case Colouring::Limit:
          colour = limit_colour(segment.limit);
          break;
        case Colouring::Load:
          colour = segment.rapid
                       ? IM_COL32(70, 82, 100, 255)
                       : blend(IM_COL32(70, 150, 255, 255), IM_COL32(255, 80, 70, 255),
                               static_cast<float>(segment.engagement));
          break;
        case Colouring::Move:
          colour = move_colour(segment.kind);
          break;
      }
      if (segment.rapid) width = 1.0f;
      if (!done) {
        ImVec4 faded = ImGui::ColorConvertU32ToFloat4(colour);
        faded.w = 0.22f;
        colour = ImGui::ColorConvertFloat4ToU32(faded);
      }
      list->AddLine(to_screen(segment.from), to_screen(segment.to), colour, width);
    }

    // The cutter itself, at its true diameter, so an engagement that looks
    // reasonable as a line can be seen for what it is.
    const cam::Tool* tool = nullptr;
    for (const auto& operation : app.job.operations)
      if (operation.name ==
          app.plan.operations[app.selected_plan_operation].operation_name)
        tool = app.tools.find(operation.tool_id);
    if (tool && app.show_toolpath) {
      const wave::ToolpathCursor cursor =
          wave::toolpath_at(level->toolpath, app.playback.distance);
      const ImVec2 centre = to_screen(cursor.position);
      const float radius =
          static_cast<float>(tool->diameter * 0.5) * app.view.zoom;
      list->AddCircleFilled(centre, std::max(radius, 2.0f),
                            IM_COL32(255, 255, 255, 40));
      list->AddCircle(centre, std::max(radius, 2.0f), IM_COL32(255, 255, 255, 210), 0,
                      1.6f);
      list->AddCircleFilled(centre, 2.5f, IM_COL32(255, 255, 255, 235));
    }
  } else {
    const char* text = app.has_plan ? "This level has nothing to cut"
                                    : "Load a part and a stock model, then plan";
    const ImVec2 extent = ImGui::CalcTextSize(text);
    list->AddText({top_left.x + (size.x - extent.x) * 0.5f,
                   top_left.y + (size.y - extent.y) * 0.5f},
                  IM_COL32(120, 130, 145, 255), text);
  }

  if (app.fill_failures > 0) {
    char note[128];
    std::snprintf(note, sizeof note, "%zu region%s could not be triangulated",
                  app.fill_failures, app.fill_failures == 1 ? "" : "s");
    list->AddText({top_left.x + 12.0f, top_left.y + 10.0f}, IM_COL32(232, 156, 92, 255),
                  note);
  }

  list->PopClipRect();
  ImGui::EndChild();
}

// ---------------------------------------------------------------------------
// The stacked result
// ---------------------------------------------------------------------------

struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Projection {
  ImVec2 centre;
  Vec3 model_centre;
  Vec3 right;
  Vec3 up;
  Vec3 toward;  // from the model toward the eye, so larger depth is nearer
  float scale = 1.0f;

  ImVec2 point(const Vec3& value) const {
    const Vec3 r{value.x - model_centre.x, value.y - model_centre.y,
                 value.z - model_centre.z};
    return {centre.x + (r.x * right.x + r.y * right.y + r.z * right.z) * scale,
            centre.y - (r.x * up.x + r.y * up.y + r.z * up.z) * scale};
  }
  float depth(const Vec3& value) const {
    const Vec3 r{value.x - model_centre.x, value.y - model_centre.y,
                 value.z - model_centre.z};
    return r.x * toward.x + r.y * toward.y + r.z * toward.z;
  }
};

// A convex piece to paint, as a run of points in a shared pool. A frame builds
// tens of thousands of these, so they carry an offset rather than a vector of
// their own.
struct Facet {
  std::uint32_t first = 0;
  std::uint32_t count = 0;
  float depth = 0.0f;
  ImU32 fill = 0;
  ImU32 edge = 0;
};

// A flat colour lit by how far a surface turns away from the light. Every face
// of the stack is either horizontal or vertical, so nothing else separates a
// wall from the lid above it, and without this the whole model reads as one
// silhouette.
ImU32 shaded(const float base[3], float lambert) {
  const float k = 0.42f + 0.58f * std::clamp(lambert, 0.0f, 1.0f);
  return IM_COL32(static_cast<int>(base[0] * k * 255.0f + 0.5f),
                  static_cast<int>(base[1] * k * 255.0f + 0.5f),
                  static_cast<int>(base[2] * k * 255.0f + 0.5f), 255);
}

// One pass's remaining stock, as the slab of material it stands for. Every pass
// is drawn as its own prism and the prisms stack, which is as close to the
// finished part as a set of two-and-a-half-dimensional passes can describe.
//
// `remaining` is what this pass leaves behind and gives the prism its walls;
// `lid_mesh` is the triangulated top surface it exposes, and `lid_rings` the
// same surface as outlines. Everything painted is convex: a wall quad is the
// projection of a vertical rectangle, and the lid arrives already cut into
// triangles, so no ear clipping happens at draw time.
void append_slab(std::vector<Facet>& facets, std::vector<ImVec2>& pool,
                 const Paths64& remaining, const Paths64& lid_mesh,
                 const Paths64& lid_rings, double bottom, double top,
                 const Projection& projection, const Vec3& light,
                 const float wall_colour[3], const float face_colour[3], ImU32 edge) {
  const auto emit = [&](std::initializer_list<Vec3> corners, float depth_bias,
                        ImU32 fill, ImU32 outline) {
    Facet facet;
    facet.first = static_cast<std::uint32_t>(pool.size());
    facet.count = static_cast<std::uint32_t>(corners.size());
    float depth = 0.0f;
    for (const Vec3& corner : corners) {
      pool.push_back(projection.point(corner));
      depth += projection.depth(corner);
    }
    facet.depth = depth / static_cast<float>(corners.size()) + depth_bias;
    facet.fill = fill;
    facet.edge = outline;
    facets.push_back(facet);
  };

  for (const auto& ring : remaining) {
    if (ring.size() < 3) continue;
    for (std::size_t i = 0; i < ring.size(); ++i) {
      const Point64& a = ring[i];
      const Point64& b = ring[(i + 1) % ring.size()];
      // A wall is vertical, so its normal lies in the plane and follows the
      // edge. Taken this way round it points into open space for an outer ring
      // and into the pocket for a hole, which is the face we can see in both.
      const float dx = static_cast<float>(wave::to_world(b.x - a.x));
      const float dy = static_cast<float>(wave::to_world(b.y - a.y));
      const float length = std::sqrt(dx * dx + dy * dy);
      if (length < 1e-6f) continue;
      const Vec3 normal{dy / length, -dx / length, 0.0f};
      if (normal.x * projection.toward.x + normal.y * projection.toward.y <= 0.0f)
        continue;  // turned away from the eye

      const Vec3 a_top{static_cast<float>(wave::to_world(a.x)),
                       static_cast<float>(wave::to_world(a.y)),
                       static_cast<float>(top)};
      const Vec3 b_top{static_cast<float>(wave::to_world(b.x)),
                       static_cast<float>(wave::to_world(b.y)),
                       static_cast<float>(top)};
      const Vec3 a_bottom{a_top.x, a_top.y, static_cast<float>(bottom)};
      const Vec3 b_bottom{b_top.x, b_top.y, static_cast<float>(bottom)};
      emit({a_top, b_top, b_bottom, a_bottom}, 0.0f,
           shaded(wall_colour,
                  normal.x * light.x + normal.y * light.y + normal.z * light.z),
           0);
    }
  }

  const ImU32 face = shaded(face_colour, light.z);
  for (const auto& triangle : lid_mesh) {
    if (triangle.size() != 3) continue;
    emit({Vec3{static_cast<float>(wave::to_world(triangle[0].x)),
               static_cast<float>(wave::to_world(triangle[0].y)),
               static_cast<float>(top)},
          Vec3{static_cast<float>(wave::to_world(triangle[1].x)),
               static_cast<float>(wave::to_world(triangle[1].y)),
               static_cast<float>(top)},
          Vec3{static_cast<float>(wave::to_world(triangle[2].x)),
               static_cast<float>(wave::to_world(triangle[2].y)),
               static_cast<float>(top)}},
         0.002f, face, 0);
  }

  // Outlined from the region rather than its triangles, so the interior edges
  // of the mesh do not show. Where a step faces away from the eye its wall is
  // culled and the outline is the only thing separating the levels either side.
  if (edge == 0) return;
  for (const auto& ring : lid_rings) {
    if (ring.size() < 3) continue;
    Facet outline;
    outline.first = static_cast<std::uint32_t>(pool.size());
    outline.count = static_cast<std::uint32_t>(ring.size());
    float depth = 0.0f;
    for (const Point64& vertex : ring) {
      const Vec3 at{static_cast<float>(wave::to_world(vertex.x)),
                    static_cast<float>(wave::to_world(vertex.y)),
                    static_cast<float>(top)};
      pool.push_back(projection.point(at));
      depth += projection.depth(at);
    }
    outline.depth = depth / static_cast<float>(ring.size()) + 0.004f;
    outline.fill = 0;
    outline.edge = edge;
    facets.push_back(outline);
  }
}

void draw_result(App& app) {
  ImGui::BeginChild("result", ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  const ImVec2 top_left = ImGui::GetCursorScreenPos();
  ImVec2 size = ImGui::GetContentRegionAvail();
  size.x = std::max(size.x, 50.0f);
  size.y = std::max(size.y, 50.0f);
  ImGui::InvisibleButton("stack", size, ImGuiButtonFlags_MouseButtonLeft);
  const bool hovered = ImGui::IsItemHovered();
  ImDrawList* list = ImGui::GetWindowDrawList();
  list->AddRectFilled(top_left, ImVec2(top_left.x + size.x, top_left.y + size.y),
                      IM_COL32(14, 17, 22, 255));
  list->PushClipRect(top_left, ImVec2(top_left.x + size.x, top_left.y + size.y), true);

  if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
    app.orbit.yaw += ImGui::GetIO().MouseDelta.x * 0.009f;
    app.orbit.pitch = std::clamp(app.orbit.pitch - ImGui::GetIO().MouseDelta.y * 0.009f,
                                 0.06f, 1.5f);
  }
  if (hovered && ImGui::GetIO().MouseWheel != 0.0f)
    app.orbit.zoom = std::clamp(app.orbit.zoom * std::pow(1.12f, ImGui::GetIO().MouseWheel),
                                0.1f, 40.0f);

  const auto finish = [&](const char* message) {
    const ImVec2 extent = ImGui::CalcTextSize(message);
    list->AddText({top_left.x + (size.x - extent.x) * 0.5f,
                   top_left.y + (size.y - extent.y) * 0.5f},
                  IM_COL32(120, 130, 145, 255), message);
    list->PopClipRect();
    ImGui::EndChild();
  };

  if (!app.has_plan || app.plan.operations.empty()) {
    finish("Plan the job to see the result");
    return;
  }
  const auto& levels = app.plan.operations[app.selected_plan_operation].levels;
  if (levels.empty()) {
    finish("This operation has no passes");
    return;
  }

  // Bounds over every pass, so orbiting does not change the framing.
  Clipper2Lib::Rect64 extent_2d;
  bool any = false;
  double lowest = 0.0;
  double highest = 0.0;
  for (const auto& level : levels) {
    if (level.stock_slice.empty()) continue;
    const auto bounds = Clipper2Lib::GetBounds(level.stock_slice);
    if (!any) {
      extent_2d = bounds;
      lowest = level.floor_z;
      highest = level.floor_z + level.axial_depth;
      any = true;
    } else {
      extent_2d.left = std::min(extent_2d.left, bounds.left);
      extent_2d.right = std::max(extent_2d.right, bounds.right);
      extent_2d.top = std::min(extent_2d.top, bounds.top);
      extent_2d.bottom = std::max(extent_2d.bottom, bounds.bottom);
      lowest = std::min(lowest, level.floor_z);
      highest = std::max(highest, level.floor_z + level.axial_depth);
    }
  }
  if (!any) {
    finish("Nothing was cut in this operation");
    return;
  }

  const double width = wave::to_world(extent_2d.right - extent_2d.left);
  const double height = wave::to_world(extent_2d.bottom - extent_2d.top);
  const double span = std::max({width, height, highest - lowest, 1e-3});
  const Vec3 centre{
      static_cast<float>(wave::to_world(extent_2d.left + extent_2d.right) * 0.5),
      static_cast<float>(wave::to_world(extent_2d.top + extent_2d.bottom) * 0.5),
      static_cast<float>((lowest + highest) * 0.5)};

  const float cy = std::cos(app.orbit.yaw);
  const float sy = std::sin(app.orbit.yaw);
  const float cp = std::cos(app.orbit.pitch);
  const float sp = std::sin(app.orbit.pitch);
  const Projection projection{
      {top_left.x + size.x * 0.5f, top_left.y + size.y * 0.5f},
      centre,
      {-sy, cy, 0.0f},
      {-sp * cy, -sp * sy, cp},
      {cp * cy, cp * sy, sp},
      static_cast<float>(std::min(size.x, size.y) * 0.72 / span) * app.orbit.zoom};

  // One vertex every thousandth of the part is plenty at any sane zoom, and it
  // keeps the number of quads in the hundreds rather than the tens of thousands.
  const double spacing = std::max(span / 400.0, 0.05);

  // A single light fixed to the eye, so the model stays lit however it is
  // orbited. Every surface here is horizontal or vertical, which is exactly the
  // case a flat colour cannot describe on its own.
  const Vec3 raw{projection.toward.x * 0.85f + projection.up.x * 0.35f +
                     projection.right.x * 0.35f,
                 projection.toward.y * 0.85f + projection.up.y * 0.35f +
                     projection.right.y * 0.35f,
                 projection.toward.z * 0.85f + projection.up.z * 0.35f +
                     projection.right.z * 0.35f};
  const float raw_length =
      std::max(1e-6f, std::sqrt(raw.x * raw.x + raw.y * raw.y + raw.z * raw.z));
  const Vec3 light{raw.x / raw_length, raw.y / raw_length, raw.z / raw_length};

  // What each pass leaves behind, resolved once so a pass can be compared with
  // the one above it to find the top surface it actually exposes.
  std::vector<const Paths64*> leftover(levels.size(), nullptr);
  for (std::size_t i = 0; i < levels.size(); ++i) {
    const cam::LevelPlan& level = levels[i];
    if (level.stock_slice.empty()) continue;
    // Passes above the cutter are finished, the one it is in is part way
    // through, and the ones below it have not been touched.
    const Paths64* remaining = &level.simulation.remaining_stock;
    if (level.nothing_to_cut) {
      remaining = &level.stock_slice;
    } else if (app.stack_follows_playback) {
      if (static_cast<int>(i) > app.selected_level) remaining = &level.stock_slice;
      else if (static_cast<int>(i) == app.selected_level && !app.material.empty())
        remaining = &app.material;
    }
    if (!remaining->empty()) leftover[i] = remaining;
  }

  // Thinned once each, before anything is compared: the rings arrive carrying
  // thousands of vertices and every boolean below would otherwise pay for them
  // on every frame. Two passes that leave the same stock thin to the same rings,
  // so the difference between them still comes out exactly empty.
  std::vector<Paths64> outline(levels.size());
  for (std::size_t i = 0; i < levels.size(); ++i)
    if (leftover[i] != nullptr) outline[i] = drawable_region(*leftover[i], spacing);

  std::vector<Facet> facets;
  std::vector<ImVec2> pool;
  std::size_t untriangulated = 0;
  for (std::size_t i = 0; i < levels.size(); ++i) {
    if (leftover[i] == nullptr) continue;
    const cam::LevelPlan& level = levels[i];

    const Paths64& walls = outline[i];
    if (walls.empty()) continue;

    // Only the ring of material this pass exposes gets a top: the rest of it is
    // roofed by the pass above. A run of passes that all leave the same stock
    // then reads as one wall rather than a stack of coincident lids.
    const Paths64 lid_rings =
        i == 0 || outline[i - 1].empty()
            ? walls
            : Clipper2Lib::Difference(walls, outline[i - 1],
                                      Clipper2Lib::FillRule::NonZero);
    const Paths64 lid_mesh = fill_mesh(lid_rings, &untriangulated);

    // Tinted by depth so the steps read apart even where two of them meet on a
    // face turned away from the light, and the pass being watched is picked out.
    const float t = levels.size() < 2
                        ? 0.0f
                        : static_cast<float>(i) / (levels.size() - 1);
    const bool active =
        app.stack_follows_playback && static_cast<int>(i) == app.selected_level;
    const float face_colour[3] = {active ? 0.44f : 0.54f - 0.12f * t,
                                  active ? 0.62f : 0.61f - 0.12f * t,
                                  active ? 0.78f : 0.70f - 0.12f * t};
    const float wall_colour[3] = {0.44f, 0.50f, 0.58f};
    const ImU32 edge = active ? IM_COL32(170, 215, 250, 200)
                              : IM_COL32(196, 214, 236, 90);
    append_slab(facets, pool, walls, lid_mesh, lid_rings, level.floor_z,
                level.floor_z + level.axial_depth, projection, light, wall_colour,
                face_colour, edge);
  }
  std::sort(facets.begin(), facets.end(),
            [](const Facet& a, const Facet& b) { return a.depth < b.depth; });

  // A wall is one quad per edge of the ring, and an antialiased fill insets
  // every one of them by half a pixel, so the wall would come out ruled with
  // hairlines where the quads meet.
  const ImDrawListFlags saved_flags = list->Flags;
  list->Flags &= ~ImDrawListFlags_AntiAliasedFill;
  for (const Facet& facet : facets) {
    if (facet.count < 3) continue;
    const ImVec2* points = pool.data() + facet.first;
    if (facet.fill != 0) {
      // Only ever a triangle from the mesh or a quad from a wall, both convex.
      if (facet.count == 3)
        list->AddTriangleFilled(points[0], points[1], points[2], facet.fill);
      else
        list->AddQuadFilled(points[0], points[1], points[2], points[3], facet.fill);
    }
    if (facet.edge != 0)
      list->AddPolyline(points, static_cast<int>(facet.count), facet.edge,
                        ImDrawFlags_Closed, 1.0f);
  }
  list->Flags = saved_flags;

  char caption[200];
  std::snprintf(caption, sizeof caption,
                "%zu passes stacked, Z %.2f to %.2f mm   Left-drag: orbit   Wheel: zoom",
                levels.size(), lowest, highest);
  list->AddText({top_left.x + 12.0f, top_left.y + 10.0f}, IM_COL32(130, 142, 158, 255),
                caption);
  if (untriangulated > 0) {
    std::snprintf(caption, sizeof caption,
                  "%zu pass%s could not be triangulated and is not drawn",
                  untriangulated, untriangulated == 1 ? "" : "es");
    list->AddText({top_left.x + 12.0f, top_left.y + 28.0f}, IM_COL32(232, 156, 92, 255),
                  caption);
  }

  list->PopClipRect();
  ImGui::EndChild();
}

// ---------------------------------------------------------------------------
// Tool Builder
// ---------------------------------------------------------------------------

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

void text_field(const char* label, std::string& value, std::size_t capacity = 128) {
  std::vector<char> buffer(std::max(capacity, value.size() + 1), '\0');
  std::snprintf(buffer.data(), buffer.size(), "%s", value.c_str());
  if (ImGui::InputText(label, buffer.data(), buffer.size())) value = buffer.data();
}

void tool_builder(App& app) {
  if (!app.builder_open) return;
  ImGui::SetNextWindowSize(ImVec2(620, 560), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Tool builder", &app.builder_open)) {
    ImGui::End();
    return;
  }
  cam::Tool& tool = app.builder_tool;
  text_field("Name", tool.name);
  ImGui::DragScalar("Diameter (mm)", ImGuiDataType_Double, &tool.diameter, 0.05f);
  ImGui::DragScalar("Flute length (mm)", ImGuiDataType_Double,
                    &tool.flute_length, 0.5f);
  ImGui::Checkbox("Centre cutting", &tool.center_cutting);
  ImGui::InputInt("Tool number", &tool.tool_number);
  ImGui::InputInt("Length offset", &tool.length_offset);
  text_field("Notes", tool.notes, 512);

  ImGui::SeparatorText("Material cutting data");
  int remove = -1;
  for (std::size_t i = 0; i < tool.cutting_data.size(); ++i) {
    cam::CuttingData& data = tool.cutting_data[i];
    ImGui::PushID(static_cast<int>(i));
    const cam::Material* material = app.materials.find(data.material_id);
    const std::string title =
        material ? material->name : data.material_id + " (unknown)";
    if (ImGui::CollapsingHeader(title.c_str(),
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::DragScalar("Spindle (rpm)", ImGuiDataType_Double,
                        &data.spindle_rpm, 100.0f);
      ImGui::DragScalar("Max radial (of diameter)", ImGuiDataType_Double,
                        &data.max_ae_over_d, 0.01f);
      ImGui::DragScalar("Max axial (of diameter)", ImGuiDataType_Double,
                        &data.max_ap_over_d, 0.05f);
      ImGui::DragScalar("Plunge feed (mm/min)", ImGuiDataType_Double,
                        &data.plunge_feed, 5.0f);
      static const cam::CoolantNeed kCoolants[]{
          cam::CoolantNeed::Required, cam::CoolantNeed::Recommended,
          cam::CoolantNeed::Dry, cam::CoolantNeed::AirBlast};
      enum_combo("Coolant", data.coolant, cam::coolant_need_name,
                 kCoolants, std::size(kCoolants));
      if (ImGui::Button("Remove this material"))
        remove = static_cast<int>(i);
    }
    ImGui::PopID();
  }
  if (remove >= 0)
    tool.cutting_data.erase(tool.cutting_data.begin() + remove);

  std::vector<const char*> names;
  for (const auto& material : app.materials.materials)
    names.push_back(material.name.c_str());
  if (!names.empty()) {
    app.builder_material =
        std::clamp(app.builder_material, 0, static_cast<int>(names.size()) - 1);
    ImGui::SetNextItemWidth(240.0f);
    ImGui::Combo("##add-material", &app.builder_material, names.data(),
                 static_cast<int>(names.size()));
    ImGui::SameLine();
    if (ImGui::Button("Add material")) {
      const auto& material = app.materials.materials[app.builder_material];
      if (tool.data_for(material.id)) {
        say(app, material.name + " is already on this tool", true);
      } else {
        cam::CuttingData data;
        data.material_id = material.id;
        tool.cutting_data.push_back(data);
      }
    }
  }

  ImGui::Separator();
  const auto problems = cam::validate(tool);
  for (const auto& problem : problems)
    ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.4f, 1.0f),
                       "%s", problem.c_str());
  ImGui::BeginDisabled(!problems.empty());
  if (ImGui::Button("Save tool")) {
    if (app.builder_is_new || tool.id.empty())
      tool.id = app.tools.unique_id_for(tool.name);
    app.tools.upsert(tool);
    try {
      cam::save_tools(app.tools_file, app.tools);
      say(app, "Saved " + tool.name);
      app.builder_open = false;
    } catch (const std::exception& error) {
      say(app, error.what(), true);
    }
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Cancel")) app.builder_open = false;
  ImGui::End();
}

// ---------------------------------------------------------------------------
// Panels
// ---------------------------------------------------------------------------

void depth_defaults(App& app, cam::Operation& operation) {
  double low = 0.0;
  double high = 0.0;
  if (!app.geometry.z_range(low, high)) return;
  operation.top_z = high;
  operation.bottom_z = low;
}

void load_meshes(App& app) {
  try {
    if (app.part_path[0] != '\0') {
      app.geometry.load_part(app.part_path.data());
      app.job.part_mesh = app.part_path.data();
    }
    if (app.stock_path[0] != '\0') {
      app.geometry.load_stock(app.stock_path.data());
      app.job.stock_mesh = app.stock_path.data();
    }
    for (auto& operation : app.job.operations) depth_defaults(app, operation);
    double low = 0.0;
    double high = 0.0;
    if (app.geometry.z_range(low, high)) {
      // Put the safe planes above the work now that there is work to be above.
      // Left at their defaults they would sit inside a part whose top is not at
      // zero, which is most parts.
      if (app.job.retract_z < high + 0.5) app.job.retract_z = high + 2.0;
      if (app.job.clearance_z < high + 1.0) app.job.clearance_z = high + 10.0;
      say(app, "Loaded. Z runs " + std::to_string(low) + " to " + std::to_string(high));
    }
    app.has_plan = false;
  } catch (const std::exception& error) {
    say(app, error.what(), true);
  }
}

void job_panel(App& app) {
  ImGui::SeparatorText("Job");
  text_field("Name", app.job.name);
  ImGui::InputText("Part model", app.part_path.data(), app.part_path.size());
  ImGui::InputText("Stock model", app.stock_path.data(), app.stock_path.size());
  if (ImGui::Button("Load models")) load_meshes(app);
  ImGui::SameLine();
  if (ImGui::Button("Fit view")) app.view.needs_fit = true;

  ImGui::DragScalar("Origin X", ImGuiDataType_Double, &app.job.origin_x, 0.1f);
  ImGui::DragScalar("Origin Y", ImGuiDataType_Double, &app.job.origin_y, 0.1f);
  ImGui::DragScalar("Origin Z", ImGuiDataType_Double, &app.job.origin_z, 0.1f);
  ImGui::DragScalar("Clearance Z", ImGuiDataType_Double, &app.job.clearance_z, 0.1f);
  ImGui::DragScalar("Retract Z", ImGuiDataType_Double, &app.job.retract_z, 0.1f);
  text_field("Work offset", app.job.work_offset, 16);
}

void operations_panel(App& app) {
  ImGui::SeparatorText("Operations");
  if (app.job.operations.empty()) {
    cam::Operation operation;
    if (!app.tools.tools.empty()) operation.tool_id = app.tools.tools.front().id;
    if (!app.machines.machines.empty())
      operation.machine_id = app.machines.machines.front().id;
    if (!app.materials.materials.empty())
      operation.material_id = app.materials.materials.front().id;
    depth_defaults(app, operation);
    app.job.operations.push_back(operation);
  }

  std::vector<const char*> names;
  for (const auto& operation : app.job.operations) names.push_back(operation.name.c_str());
  app.selected_operation =
      std::clamp(app.selected_operation, 0, static_cast<int>(names.size()) - 1);
  ImGui::SetNextItemWidth(200.0f);
  ImGui::Combo("##operation", &app.selected_operation, names.data(),
               static_cast<int>(names.size()));
  ImGui::SameLine();
  if (ImGui::Button("Add")) {
    cam::Operation copy = app.job.operations[app.selected_operation];
    copy.name += " copy";
    app.job.operations.push_back(copy);
    app.selected_operation = static_cast<int>(app.job.operations.size()) - 1;
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(app.job.operations.size() < 2);
  if (ImGui::Button("Remove")) {
    app.job.operations.erase(app.job.operations.begin() + app.selected_operation);
    app.selected_operation = 0;
  }
  ImGui::EndDisabled();

  cam::Operation& operation = app.job.operations[app.selected_operation];
  text_field("Operation name", operation.name);

  const auto pick = [&](const char* label, std::string& id, const auto& entries,
                        auto name_of) {
    int index = -1;
    std::vector<const char*> labels;
    for (std::size_t i = 0; i < entries.size(); ++i) {
      labels.push_back(name_of(entries[i]));
      if (entries[i].id == id) index = static_cast<int>(i);
    }
    if (labels.empty()) {
      ImGui::TextDisabled("%s: nothing available", label);
      return;
    }
    if (index < 0) index = 0;
    if (ImGui::Combo(label, &index, labels.data(), static_cast<int>(labels.size())))
      id = entries[index].id;
    else if (id.empty())
      id = entries[index].id;
  };

  pick("Tool", operation.tool_id, app.tools.tools,
       [](const cam::Tool& tool) { return tool.name.c_str(); });
  pick("Machine", operation.machine_id, app.machines.machines,
       [](const cam::Machine& machine) { return machine.name.c_str(); });
  pick("Material", operation.material_id, app.materials.materials,
       [](const cam::Material& material) { return material.name.c_str(); });

  const cam::Tool* tool = app.tools.find(operation.tool_id);
  const cam::Machine* machine = app.machines.find(operation.machine_id);
  if (tool && !tool->data_for(operation.material_id))
    ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.4f, 1.0f),
                       "This tool has no cutting data for that material");
  if (machine && machine->specifications_unverified)
    ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.35f, 1.0f),
                       "This machine profile carries unverified limits");

  ImGui::DragScalar("Stepover (of diameter)", ImGuiDataType_Double,
                    &operation.stepover_ratio, 0.005f);
  ImGui::DragScalar("Stepdown (mm)", ImGuiDataType_Double, &operation.stepdown, 0.1f);
  if (tool)
    ImGui::TextDisabled("= %.3f mm wide, %zu passes", operation.stepover_ratio * tool->diameter,
                        cam::level_floors(operation).size());
  ImGui::DragScalar("Radial stock to leave", ImGuiDataType_Double,
                    &operation.radial_stock_to_leave, 0.02f);
  ImGui::DragScalar("Axial stock to leave", ImGuiDataType_Double,
                    &operation.axial_stock_to_leave, 0.02f);
  ImGui::DragScalar("Top Z", ImGuiDataType_Double, &operation.top_z, 0.1f);
  ImGui::DragScalar("Bottom Z", ImGuiDataType_Double, &operation.bottom_z, 0.1f);
  ImGui::Checkbox("Lead in from the air", &operation.lead_in);
  ImGui::SetItemTooltip(
      "Seeds the wave outside the stock so the cutter arrives at the wall along "
      "a path rather than at a standstill. Without it the first pass runs down "
      "the stock wall itself and buries half the cutter.");
  ImGui::BeginDisabled(!operation.lead_in);
  ImGui::DragScalar("Stock oversize (mm)", ImGuiDataType_Double,
                    &operation.stock_deviation, 0.01f);
  ImGui::SetItemTooltip(
      "How much larger than its model the blank may be. The run-in is placed so "
      "the first pass to touch the stock aims for a full stepover less this, "
      "which leaves that much headroom for an oversize blank.");
  if (tool) {
    const double bite = operation.stepover_ratio * tool->diameter;
    const double first =
        std::max(bite - std::clamp(operation.stock_deviation, 0.0, bite * 0.9), 0.0);
    ImGui::TextDisabled("= target first bite %.3f mm, then %.3f mm", first, bite);
  }
  ImGui::EndDisabled();

  ImGui::DragScalar("Simulation/feed pitch (mm)", ImGuiDataType_Double,
                    &operation.path_pitch, 0.01f);
  ImGui::SetItemTooltip(
      "The complete at-depth path is resampled at this travelled-distance "
      "interval before simulation and feed assignment.");
}

void feeds_panel(App& app) {
  if (app.job.operations.empty()) return;
  cam::Operation& operation = app.job.operations[app.selected_operation];
  const cam::Machine* machine = app.machines.find(operation.machine_id);
  const cam::MaterialFeedRange* library =
      machine ? machine->feeds_for(operation.material_id) : nullptr;
  ImGui::SeparatorText("Engagement feeds");
  ImGui::TextWrapped(
      "Feed is interpolated directly between these endpoints using the "
      "simulation's engagement measurement.");
  ImGui::Checkbox("Enter manually", &operation.manual_feed_range);
  if (!operation.manual_feed_range && library) {
    ImGui::Text("Full engagement  %.0f mm/min", library->full_engagement);
    ImGui::Text("No engagement    %.0f mm/min", library->no_engagement);
  } else if (!operation.manual_feed_range) {
    ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.4f, 1.0f),
                       "No library range for this machine and material");
  }
  ImGui::BeginDisabled(!operation.manual_feed_range);
  ImGui::DragScalar("Full engagement (mm/min)", ImGuiDataType_Double,
                    &operation.full_engagement_feed, 10.0f);
  ImGui::DragScalar("No engagement (mm/min)", ImGuiDataType_Double,
                    &operation.no_engagement_feed, 10.0f);
  ImGui::EndDisabled();
}

void plan_panel(App& app) {
  ImGui::SeparatorText("Plan");
  if (app.planner.running()) {
    ImGui::ProgressBar(static_cast<float>(app.planner.fraction()), ImVec2(-1, 0));
    ImGui::TextWrapped("%s", app.planner.stage().c_str());
    if (ImGui::Button("Cancel")) app.planner.cancel();
    return;
  }

  const bool ready = app.geometry.has_part() && app.geometry.has_stock();
  ImGui::BeginDisabled(!ready);
  if (ImGui::Button("Plan job", ImVec2(-1, 0))) {
    app.gcode.clear();
    app.post = {};
    app.planner.start(app.geometry, app.job, app.tools, app.machines, app.materials);
  }
  ImGui::EndDisabled();
  if (!ready) ImGui::TextDisabled("Load a part and a stock model first");

  if (!app.has_plan) return;

  const cam::PlanStats& stats = app.plan.stats;
  ImGui::Separator();
  ImGui::Text("Passes        %zu of %zu cut", stats.cut_levels, stats.levels);
  ImGui::Text("Cutting       %.0f mm in %.1f min", stats.cutting_distance,
              stats.cutting_minutes);
  ImGui::Text("Travel        %.0f mm in %.1f min", stats.rapid_distance,
              stats.rapid_minutes);
  ImGui::Text("Removed       %.0f mm3", stats.removed_volume);
  ImGui::Text("Peak load     %.2f of a full slot", stats.maximum_load);
  ImGui::Text("Left standing %.0f mm2", stats.excess_area);
  ImGui::SetItemTooltip(
      "Stock still there when the operation finishes that is not part of the "
      "finished shape. Roughing always leaves some: the radial allowance is "
      "exactly this. A figure well above it is material the passes missed.");
  if (stats.gouge_area > 0.0)
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Gouges        %.3f mm2",
                       stats.gouge_area);
  if (stats.unsafe_plunges > 0)
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Unsafe plunges %zu",
                       stats.unsafe_plunges);
  if (stats.unresolved_entries > 0)
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Entries into solid %zu",
                       stats.unresolved_entries);
  if (stats.lifts_over_subject > 0)
    ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.35f, 1.0f), "Rapids over the part %zu",
                       stats.lifts_over_subject);
  if (stats.unreachable_fronts > 0)
    ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.35f, 1.0f), "Fronts never reached %zu",
                       stats.unreachable_fronts);

  for (const auto& warning : app.plan.warnings)
    ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.35f, 1.0f), "! %s", warning.c_str());

  const cam::LevelPlan* level = current_level(app);
  if (level)
    ImGui::Text("Spindle %.0f rpm", level->feeds.stats.spindle_rpm);
}

void output_panel(App& app) {
  if (!app.has_plan) return;
  ImGui::SeparatorText("Output");
  if (ImGui::Button("Generate G-code", ImVec2(-1, 0))) {
    app.post = cam::post_job(app.plan, app.job, app.tools, app.machines);
    app.gcode = app.post.text;
    if (!app.post.ok()) say(app, app.post.error, true);
    else say(app, "Generated " + std::to_string(app.post.line_count) + " lines");
  }
  if (app.gcode.empty()) return;

  ImGui::Text("%zu lines, %zu cutting moves, %zu rapids", app.post.line_count,
              app.post.cutting_moves, app.post.rapid_moves);
  ImGui::Text("Estimated %.1f minutes", app.post.estimated_minutes);
  for (const auto& warning : app.post.warnings)
    ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.35f, 1.0f), "! %s", warning.c_str());

  ImGui::InputText("File", app.save_path.data(), app.save_path.size());
  if (ImGui::Button("Save")) {
    try {
      cam::write_file(app.save_path.data(), app.gcode);
      say(app, std::string("Wrote ") + app.save_path.data());
    } catch (const std::exception& error) {
      say(app, error.what(), true);
    }
  }
  if (ImGui::CollapsingHeader("Preview")) {
    ImGui::BeginChild("gcode", ImVec2(0, 220), ImGuiChildFlags_Borders,
                      ImGuiWindowFlags_HorizontalScrollbar);
    ImGui::TextUnformatted(app.gcode.c_str(),
                           app.gcode.c_str() + std::min<std::size_t>(app.gcode.size(), 60000));
    ImGui::EndChild();
  }
}

void library_panel(App& app) {
  ImGui::SeparatorText("Tools");
  for (const auto& tool : app.tools.tools) {
    ImGui::PushID(tool.id.c_str());
    ImGui::Text("%d  %s", tool.tool_number, tool.name.c_str());
    ImGui::SameLine(ImGui::GetContentRegionAvail().x - 90.0f);
    if (ImGui::SmallButton("Edit")) {
      app.builder_tool = tool;
      app.builder_is_new = false;
      app.builder_open = true;
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Copy")) {
      app.builder_tool = tool;
      app.builder_tool.name += " copy";
      app.builder_is_new = true;
      app.builder_open = true;
    }
    ImGui::PopID();
  }
  if (ImGui::Button("New tool")) {
    app.builder_tool = cam::Tool{};
    app.builder_tool.name = "New tool";
    app.builder_tool.tool_number = static_cast<int>(app.tools.tools.size()) + 1;
    app.builder_tool.length_offset = app.builder_tool.tool_number;
    app.builder_is_new = true;
    app.builder_open = true;
  }
  ImGui::SameLine();
  ImGui::TextDisabled("%zu machines, %zu materials", app.machines.machines.size(),
                      app.materials.materials.size());
}

void view_panel(App& app) {
  if (!app.has_plan) return;
  ImGui::SeparatorText("View");
  std::vector<const char*> operations;
  for (const auto& operation : app.plan.operations)
    operations.push_back(operation.operation_name.c_str());
  if (!operations.empty()) {
    app.selected_plan_operation =
        std::clamp(app.selected_plan_operation, 0, static_cast<int>(operations.size()) - 1);
    if (ImGui::Combo("Operation", &app.selected_plan_operation, operations.data(),
                     static_cast<int>(operations.size())))
      app.selected_level = 0;
  }
  const auto& levels = app.plan.operations[app.selected_plan_operation].levels;
  if (!levels.empty()) {
    int last = static_cast<int>(levels.size()) - 1;
    app.selected_level = std::clamp(app.selected_level, 0, last);
    ImGui::SliderInt("Pass", &app.selected_level, 0, last);
    ImGui::TextDisabled("Depth %.3f mm, cutting %.3f mm",
                        levels[app.selected_level].floor_z,
                        levels[app.selected_level].axial_depth);
  }
  static const char* kColourings[]{"Feed rate", "Limiting reason", "Engagement",
                                   "Move type"};
  int colouring = static_cast<int>(app.colouring);
  if (ImGui::Combo("Colour by", &colouring, kColourings, 4))
    app.colouring = static_cast<Colouring>(colouring);
  ImGui::Checkbox("Stock", &app.show_stock);
  ImGui::SameLine();
  ImGui::Checkbox("Part", &app.show_part);
  ImGui::SameLine();
  ImGui::Checkbox("Travel", &app.show_rapids);
  ImGui::Checkbox("Material##show", &app.show_material);
  ImGui::SameLine();
  ImGui::Checkbox("Toolpath", &app.show_toolpath);
  ImGui::Checkbox("Stack follows the cutter", &app.stack_follows_playback);
  ImGui::SetItemTooltip(
      "Off, the stacked view shows the finished part. On, it shows how far the "
      "job has got: passes above the cutter finished, the one it is in part way "
      "through, and the ones below it untouched.");

  ImGui::SeparatorText("Playback");
  const cam::LevelPlan* level = current_level(app);
  const double total = level_length(level);
  if (ImGui::Button(app.playback.playing ? "Pause" : "Play"))
    app.playback.playing = !app.playback.playing;
  ImGui::SameLine();
  if (ImGui::Button("Restart")) {
    app.playback.distance = 0.0;
    app.playback.playing = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("End")) {
    app.playback.distance = total;
    app.playback.playing = false;
  }
  ImGui::SameLine();
  ImGui::Checkbox("Run on", &app.playback.follow);
  ImGui::SetItemTooltip("Fall into the next pass instead of looping this one.");

  float where = static_cast<float>(app.playback.distance);
  ImGui::SetNextItemWidth(-160.0f);
  if (ImGui::SliderFloat("Along pass", &where, 0.0f, static_cast<float>(total), "%.1f mm"))
    app.playback.distance = where;

  ImGui::Checkbox("Machine time", &app.playback.real_time);
  ImGui::SetItemTooltip(
      "Run the cutter at the feeds the plan commands. Those came out of what "
      "the simulation measured it removing, so the pass crawls where the "
      "engagement is heavy and opens out where there is nothing in the way. "
      "Off, it runs at a fixed distance per second instead.");
  if (app.playback.real_time) {
    float scale = static_cast<float>(app.playback.time_scale);
    ImGui::SetNextItemWidth(-160.0f);
    if (ImGui::SliderFloat("Rate", &scale, 0.1f, 20.0f, "%.2fx",
                           ImGuiSliderFlags_Logarithmic))
      app.playback.time_scale = scale;
    if (level && !level->nothing_to_cut) {
      const double elapsed =
          cam::feed_seconds_to(level->feeds, app.playback.distance);
      const double whole = cam::feed_schedule_seconds(level->feeds);
      ImGui::Text("%s of %s in this pass, %.1f min in the job",
                  clock_text(elapsed).c_str(), clock_text(whole).c_str(),
                  app.plan.stats.cutting_minutes + app.plan.stats.rapid_minutes);
      const cam::FeedSegment* segment = nullptr;
      if (!level->feeds.segments.empty())
        segment = &level->feeds.segments[segment_at(level->feeds.segments,
                                                    app.playback.distance)];
      if (segment != nullptr)
        ImGui::Text("Now %.0f mm/min, %.0f%% engaged, %s", segment->rate,
                    segment->engagement * 100.0, cam::feed_limit_name(segment->limit));
    }
  } else {
    float speed = static_cast<float>(app.playback.speed);
    ImGui::SetNextItemWidth(-160.0f);
    if (ImGui::SliderFloat("Speed", &speed, 2.0f, 400.0f, "%.0f mm/s"))
      app.playback.speed = speed;
  }

  if (level && !level->nothing_to_cut) {
    const double area = wave::material_area(app.material);
    const double started = level->simulation.stats.initial_stock_area;
    ImGui::Text("Stock left %.1f mm2 of %.1f", area, started);
    if (started > 0.0)
      ImGui::ProgressBar(static_cast<float>(1.0 - area / started), ImVec2(-1, 12),
                         "removed");
  }
}

void sidebar(App& app) {
  if (!app.banner.empty()) {
    const ImVec4 colour = app.banner_is_error ? ImVec4(1.0f, 0.45f, 0.4f, 1.0f)
                                              : ImVec4(0.55f, 0.85f, 0.65f, 1.0f);
    ImGui::TextColored(colour, "%s", app.banner.c_str());
    ImGui::Separator();
  }
  job_panel(app);
  operations_panel(app);
  plan_panel(app);
  view_panel(app);
  output_panel(app);
  if (ImGui::CollapsingHeader("Engagement feeds")) feeds_panel(app);
  if (ImGui::CollapsingHeader("Libraries")) library_panel(app);
}

// ---------------------------------------------------------------------------

void bootstrap(App& app) {
  const std::filesystem::path data = cam::data_directory();
  const std::filesystem::path config = cam::config_directory();
  app.tools_file = config / "tools.json";

  try {
    app.machines = cam::load_machines(data / "machines");
    app.materials = cam::load_materials(data / "materials");
  } catch (const std::exception& error) {
    say(app, error.what(), true);
  }
  if (app.machines.machines.empty())
    say(app, "No machine profiles found under " + (data / "machines").string(), true);

  try {
    app.tools = cam::load_tools(app.tools_file);
    if (app.tools.tools.empty()) {
      // First run: start from the shipped tools rather than an empty crib, and
      // write them into the user's own file so editing them is normal.
      app.tools = cam::load_tools(data / "starter-tools.json");
      if (!app.tools.tools.empty()) cam::save_tools(app.tools_file, app.tools);
    }
  } catch (const std::exception& error) {
    say(app, error.what(), true);
  }
  std::snprintf(app.save_path.data(), app.save_path.size(), "%s",
                (std::filesystem::current_path() / "tinglecam.nc").string().c_str());
}

}  // namespace

int main(int argc, char** argv) {
  App app;
  bootstrap(app);

  std::filesystem::path screenshot;
  std::filesystem::path gcode_out;
  std::string view_name;
  double progress_fraction = -1.0;
  double stepdown = 0.0;
  double stepover = 0.0;
  double path_pitch = 0.0;
  double full_feed = 0.0;
  double no_feed = 0.0;
  std::string tool_id;
  std::string machine_id;
  std::string material_id;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    const auto value = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(argument + " needs a value");
      return argv[++i];
    };
    try {
      if (argument == "--part") std::snprintf(app.part_path.data(), app.part_path.size(), "%s", value().c_str());
      else if (argument == "--stock") std::snprintf(app.stock_path.data(), app.stock_path.size(), "%s", value().c_str());
      else if (argument == "--screenshot") screenshot = value();
      else if (argument == "--view") view_name = value();
      else if (argument == "--at") progress_fraction = std::stod(value());
      else if (argument == "--follow") app.stack_follows_playback = true;
      else if (argument == "--gcode") gcode_out = value();
      else if (argument == "--tool") tool_id = value();
      else if (argument == "--machine") machine_id = value();
      else if (argument == "--material") material_id = value();
      else if (argument == "--stepdown") stepdown = std::stod(value());
      else if (argument == "--stepover") stepover = std::stod(value());
      else if (argument == "--path-pitch") path_pitch = std::stod(value());
      else if (argument == "--full-feed") full_feed = std::stod(value());
      else if (argument == "--no-feed") no_feed = std::stod(value());
      else if (argument == "--help" || argument == "-h") {
        std::printf(
            "TingleCAM -- adaptive roughing over the wavefront engine\n\n"
            "  --part PATH        Part model (OBJ or STL)\n"
            "  --stock PATH       Stock model (OBJ or STL)\n"
            "  --tool ID          Tool from the library\n"
            "  --machine ID       Machine profile\n"
            "  --material ID      Workpiece material\n"
            "  --stepover RATIO   Radial engagement as a fraction of diameter\n"
            "  --stepdown MM      Axial depth per pass\n"
            "  --path-pitch MM    Simulation/feed sampling interval (default 0.2)\n"
            "  --full-feed RATE   Manual feed at full engagement, mm/min\n"
            "  --no-feed RATE     Manual feed at zero engagement, mm/min\n"
            "  --gcode PATH       Plan and post without opening a window, then exit\n"
            "  --screenshot PATH  Render one frame to a binary PPM and exit\n"
            "  --view pass|result Which view a screenshot should capture\n"
            "  --at FRACTION      How far through the pass to place the cutter, 0 to 1\n"
            "  -h, --help         Show this help and exit\n\n"
            "Tools live in %s\n"
            "Machines and materials are read from %s\n",
            app.tools_file.string().c_str(), cam::data_directory().string().c_str());
        return EXIT_SUCCESS;
      } else {
        std::fprintf(stderr, "Unknown option: %s\n", argument.c_str());
        return EXIT_FAILURE;
      }
    } catch (const std::exception& error) {
      std::fprintf(stderr, "%s\n", error.what());
      return EXIT_FAILURE;
    }
  }
  if (app.part_path[0] != '\0' || app.stock_path[0] != '\0') load_meshes(app);

  // Anything the command line set has to reach the operation the panels would
  // otherwise have created, so make it here and apply them to it.
  if (!app.job.operations.empty() || app.geometry.has_part()) {
    if (app.job.operations.empty()) {
      cam::Operation operation;
      if (!app.tools.tools.empty()) operation.tool_id = app.tools.tools.front().id;
      if (!app.machines.machines.empty())
        operation.machine_id = app.machines.machines.front().id;
      if (!app.materials.materials.empty())
        operation.material_id = app.materials.materials.front().id;
      depth_defaults(app, operation);
      app.job.operations.push_back(operation);
    }
    cam::Operation& operation = app.job.operations.front();
    if (!tool_id.empty()) operation.tool_id = tool_id;
    if (!machine_id.empty()) operation.machine_id = machine_id;
    if (!material_id.empty()) operation.material_id = material_id;
    if (stepdown > 0.0) operation.stepdown = stepdown;
    if (stepover > 0.0) operation.stepover_ratio = stepover;
    if (path_pitch > 0.0) operation.path_pitch = path_pitch;
    if (full_feed > 0.0 || no_feed > 0.0) {
      operation.manual_feed_range = true;
      if (full_feed > 0.0) operation.full_engagement_feed = full_feed;
      if (no_feed > 0.0) operation.no_engagement_feed = no_feed;
    }
  }

  // Planning and posting with no window at all, which is how this gets checked
  // without a display and how it fits into anything scripted.
  if (!gcode_out.empty()) {
    if (!app.geometry.has_part() || !app.geometry.has_stock()) {
      std::fprintf(stderr, "--gcode needs both --part and --stock\n");
      return EXIT_FAILURE;
    }
    const cam::JobPlan plan = cam::plan_job(
        app.geometry, app.job, app.tools, app.machines, app.materials,
        [](double fraction, const std::string& stage) {
          std::fprintf(stderr, "\r%3.0f%%  %-56s", fraction * 100.0, stage.c_str());
          std::fflush(stderr);
          return true;
        });
    std::fprintf(stderr, "\r%-64s\n", "");
    for (const auto& warning : plan.warnings)
      std::fprintf(stderr, "warning: %s\n", warning.c_str());
    if (!plan.ok()) {
      std::fprintf(stderr, "%s\n", plan.error.c_str());
      return EXIT_FAILURE;
    }
    const cam::PostResult post = cam::post_job(plan, app.job, app.tools, app.machines);
    for (const auto& warning : post.warnings)
      std::fprintf(stderr, "warning: %s\n", warning.c_str());
    if (!post.ok()) {
      std::fprintf(stderr, "%s\n", post.error.c_str());
      return EXIT_FAILURE;
    }
    cam::write_file(gcode_out, post.text);
    // Everything the simulation flags, printed whether or not it fired, so a
    // clean run says so rather than being read off an absence of output.
    std::printf(
        "Safety: %zu unsafe plunge(s), %zu entry(s) into solid, %zu rapid(s) over "
        "the part, %zu front(s) never reached, %.3f mm2 gouged, %.0f mm2 left "
        "standing\n",
        plan.stats.unsafe_plunges, plan.stats.unresolved_entries,
        plan.stats.lifts_over_subject, plan.stats.unreachable_fronts,
        plan.stats.gouge_area, plan.stats.excess_area);
    std::printf(
        "%zu passes cut of %zu, %.0f mm of cutting in %.1f min, peak load %.2f\n"
        "%zu lines written to %s\n",
        plan.stats.cut_levels, plan.stats.levels, plan.stats.cutting_distance,
        plan.stats.cutting_minutes + plan.stats.rapid_minutes, plan.stats.maximum_load,
        post.line_count, gcode_out.string().c_str());
    return EXIT_SUCCESS;
  }

  bool screenshot_pending = !screenshot.empty();

  // A screenshot draws one frame and exits, so it has no business opening a
  // window on the desktop: under a compositor that can lose its shared memory,
  // a run that cannot reach the display hangs holding the connection instead of
  // failing. An explicit SDL_VIDEODRIVER still wins.
  if (screenshot_pending && SDL_getenv("SDL_VIDEODRIVER") == nullptr)
    SDL_setenv("SDL_VIDEODRIVER", "offscreen", 1);

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
    std::fprintf(stderr, "SDL initialisation failed: %s\n", SDL_GetError());
    return EXIT_FAILURE;
  }
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_Window* window = SDL_CreateWindow(
      "TingleCAM", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1500, 940,
      SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
  if (!window) {
    std::fprintf(stderr, "Window creation failed: %s\n", SDL_GetError());
    return EXIT_FAILURE;
  }
  SDL_GLContext context = SDL_GL_CreateContext(window);
  SDL_GL_MakeCurrent(window, context);
  SDL_GL_SetSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGui::GetIO().IniFilename = nullptr;
  ImGui::GetStyle().FrameRounding = 3.0f;
  ImGui_ImplSDL2_InitForOpenGL(window, context);
  ImGui_ImplOpenGL3_Init("#version 150");

  bool running = true;
  // Declared above SDL initialisation, which needs to know.
  int screenshot_delay = 0;
  auto previous_frame = std::chrono::steady_clock::now();
  if (screenshot_pending && app.geometry.has_part() && app.geometry.has_stock())
    app.planner.start(app.geometry, app.job, app.tools, app.machines, app.materials);
  if (!view_name.empty()) app.force_tab = view_name == "result" ? 1 : 0;
  while (running) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT ||
          (event.type == SDL_WINDOWEVENT &&
           event.window.event == SDL_WINDOWEVENT_CLOSE &&
           event.window.windowID == SDL_GetWindowID(window)))
        running = false;
    }

    if (app.planner.take(app.plan)) {
      app.has_plan = app.plan.ok();
      app.selected_level = 0;
      app.selected_plan_operation = 0;
      app.view.needs_fit = true;
      app.playback.distance = 0.0;
      app.material.clear();
      app.material_distance = -1.0;
      app.material_level = -1;
      if (progress_fraction >= 0.0) {
        const cam::LevelPlan* level = current_level(app);
        app.playback.distance =
            std::clamp(progress_fraction, 0.0, 1.0) * level_length(level);
      }
      if (!app.plan.ok()) say(app, app.plan.error, true);
      else say(app, "Planned " + std::to_string(app.plan.stats.cut_levels) + " passes");
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - previous_frame).count();
    previous_frame = now;
    advance_playback(app, std::min(elapsed, 0.1));
    if (app.has_plan) refresh_material(app);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::Begin("TingleCAM", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);
    // Wide enough for "Stepover (of diameter)" and "Feed slew (mm/min per mm)"
    // to be read whole. ImGui puts a label to the right of its widget and gives
    // it whatever is left over, so the panel and the item width have to be set
    // together: at the default two thirds, the labels lost their tails.
    ImGui::BeginChild("sidebar", ImVec2(kSidebarWidth, 0.0f), ImGuiChildFlags_Borders);
    ImGui::PushItemWidth(kSidebarWidth * 0.42f);
    sidebar(app);
    ImGui::PopItemWidth();
    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginChild("views", ImVec2(0, 0));
    if (ImGui::BeginTabBar("view-tabs")) {
      const ImGuiTabItemFlags plan_flag =
          app.force_tab == 0 ? ImGuiTabItemFlags_SetSelected : 0;
      const ImGuiTabItemFlags result_flag =
          app.force_tab == 1 ? ImGuiTabItemFlags_SetSelected : 0;
      if (ImGui::BeginTabItem("Pass", nullptr, plan_flag)) {
        app.mode = ViewMode::Plan;
        draw_canvas(app);
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Stacked result", nullptr, result_flag)) {
        app.mode = ViewMode::Result;
        draw_result(app);
        ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
    }
    ImGui::EndChild();
    ImGui::End();
    tool_builder(app);

    ImGui::Render();
    int width = 0;
    int height = 0;
    SDL_GL_GetDrawableSize(window, &width, &height);
    glViewport(0, 0, width, height);
    glClearColor(0.03f, 0.04f, 0.05f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    if (screenshot_pending && app.planner.running()) {
      // Wait for the plan, so the frame that gets captured has something in it.
    } else if (screenshot_pending && ++screenshot_delay < 3) {
      // One more frame, so the panels have laid out against the finished plan.
    } else if (screenshot_pending) {
      std::vector<unsigned char> pixels(static_cast<std::size_t>(width) * height * 3);
      glPixelStorei(GL_PACK_ALIGNMENT, 1);
      glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
      std::ofstream file(screenshot, std::ios::binary);
      file << "P6\n" << width << ' ' << height << "\n255\n";
      for (int row = height - 1; row >= 0; --row)
        file.write(reinterpret_cast<const char*>(
                       pixels.data() + static_cast<std::size_t>(row) * width * 3),
                   static_cast<std::streamsize>(width) * 3);
      screenshot_pending = false;
      running = false;
    }

    SDL_GL_SwapWindow(window);
  }

  app.planner.cancel();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return EXIT_SUCCESS;
}
