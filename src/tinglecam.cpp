// TingleCAM: adaptive roughing over the wavefront engine, with tool and machine
// libraries and feeds driven by what the material simulation says is actually
// being removed.

#include "cam/config.h"
#include "cam/feeds.h"
#include "cam/job.h"
#include "cam/post.h"
#include "cam/stock.h"
#include "prism_renderer.h"

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
#include <vector>
#include <unordered_map>

namespace {

using Clipper2Lib::Paths64;
using Clipper2Lib::Point64;

// The control column. Everything in it is laid out against this, so widening
// the panel widens the widgets and their labels together.
constexpr float kLeftPanelWidth = 440.0f;
constexpr float kRightPanelWidth = 480.0f;

// ---------------------------------------------------------------------------
// Background planning
// ---------------------------------------------------------------------------

// Planning a job of any size takes long enough that doing it on the frame
// thread would freeze the window. It runs on its own thread and reports back
// through here. Everything the worker touches is either atomic or under the
// mutex; the result itself is handed over once, at the end.
//
// `Result` carries an `error` string, which is where a thrown exception lands.
template <typename Result>
class Background {
 public:
  ~Background() { cancel_and_join(); }

  bool running() const { return running_.load(); }
  double fraction() const { return fraction_.load(); }
  std::string stage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stage_;
  }

  void start(std::function<Result(const cam::ProgressCallback&)> task) {
    cancel_and_join();
    cancelled_.store(false);
    running_.store(true);
    fraction_.store(0.0);
    finished_.store(false);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stage_ = "Starting";
    }
    worker_ = std::thread([this, task = std::move(task)]() mutable {
      Result result;
      try {
        result = task(report());
      } catch (const std::exception& error) {
        result = {};
        result.error = error.what();
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        result_ = std::move(result);
      }
      finished_.store(true);
      running_.store(false);
    });
  }

  void cancel() { cancelled_.store(true); }

  // True once, when a finished result is ready to be taken.
  bool take(Result& into) {
    if (!finished_.exchange(false)) return false;
    if (worker_.joinable()) worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    into = std::move(result_);
    return true;
  }

 private:
  cam::ProgressCallback report() {
    return [this](double fraction, const std::string& stage) {
      fraction_.store(fraction);
      std::lock_guard<std::mutex> lock(mutex_);
      stage_ = stage;
      return !cancelled_.load();
    };
  }

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
  Result result_;
};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

enum class Colouring { Feed, Limit, Load, Move };

// Which body the plan view draws. The stock encloses the part, so showing both
// means showing the stock through, not showing it on top.
enum class ModelShow { Stock, Part, Both };

const char* model_show_name(ModelShow value) {
  switch (value) {
    case ModelShow::Stock: return "Stock";
    case ModelShow::Part: return "Part";
    case ModelShow::Both: return "Both";
  }
  return "Both";
}

// And which the stacked result superimposes on the passes. A fourth state the
// plan view has no use for: there the model is the subject, here it is an
// overlay on something else, so it has to be possible to have neither.
enum class StackModels { None, Part, Stock, Both };

const char* stack_models_name(StackModels value) {
  switch (value) {
    case StackModels::None: return "No model";
    case StackModels::Part: return "Part";
    case StackModels::Stock: return "Stock";
    case StackModels::Both: return "Both";
  }
  return "No model";
}

enum class NodeDebugColouring { None, Node, ZGroup };

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
  // Where the framed centre sits on screen, in pixels from the middle of the
  // canvas. Held in screen space rather than as a point in the model, because
  // the gesture is screen space: a drag should move what is under the pointer
  // by exactly that many pixels whatever the orbit and zoom happen to be.
  ImVec2 pan{0.0f, 0.0f};
};

// Playback along one pass. The distance is measured along that pass's toolpath,
// which is what the simulation snapshots are indexed by.
struct Playback {
  bool playing = false;
  double distance = 0.0;
  // Elapsed commanded-machine time within this pass.  This is distinct from
  // distance because the latter stands still during retracts and plunges.
  double motion_seconds = 0.0;
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

  Background<cam::JobPlan> planner;
  std::string banner;
  bool banner_is_error = false;

  std::array<char, 512> part_path{};
  std::array<char, 512> stock_path{};
  int selected_operation = 0;
  int selected_level = 0;
  int selected_plan_operation = 0;
  Colouring colouring = Colouring::Feed;
  // Set from the command line so a screenshot can be told which view to take.
  int force_tab = -1;
  bool show_stock = true;
  bool show_part = true;
  // What the plan view shows. Separate from the pass view's own toggles above:
  // that view draws outlines and can afford both at once, while a solid needs
  // to be told which of two nested bodies is the subject.
  ModelShow models = ModelShow::Both;
  // What the stacked result draws over the passes. Off to begin with: the
  // stack is the thing being looked at, and a model over it is a question
  // asked of that stack rather than the default way to read it.
  StackModels stack_models = StackModels::None;
  bool show_rapids = true;
  bool show_material = true;
  bool show_toolpath = true;
  NodeDebugColouring node_debug_colouring = NodeDebugColouring::None;
  // Which corner, edge or centre of the blank the origin picker is aimed at,
  // one choice per axis: 0 low, 1 middle, 2 high. A pick on its own moves
  // nothing -- it is applied on the button, so a stray click cannot silently
  // shift where the program thinks the work is.
  std::array<int, 3> origin_pick{1, 1, 2};
  bool show_entry_front_chains = false;
  bool show_boundary_contacts = false;
  // Follow the cutter by default so the stacked view and playback describe the
  // same instant. It can still be switched off to inspect the finished stack.
  bool stack_follows_playback = true;
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
  int stock_cache_operation = -1, stock_cache_level = -1;
  std::vector<cam::StockSlab> stock_cache;

  // Tool Builder
  bool builder_open = false;
  bool builder_is_new = false;
  cam::Tool builder_tool;

  std::string gcode;
  cam::PostResult post;
  std::array<char, 512> save_path{};

  std::filesystem::path tools_file;
  std::string tools_import_file;
  std::string job_file = "tinglecam-job.json";
};

void say(App& app, std::string text, bool error = false) {
  app.banner = std::move(text);
  app.banner_is_error = error;
}

void log_warnings(const char* source, const std::vector<std::string>& warnings) {
  for (const auto& warning : warnings)
    std::fprintf(stderr, "%s warning: %s\n", source, warning.c_str());
}

void show_warnings(const char* id, const std::vector<std::string>& warnings) {
  if (warnings.empty()) return;
  ImGui::PushID(id);
  if (ImGui::SmallButton("Copy warnings")) {
    std::string text;
    for (const auto& warning : warnings) text += warning + '\n';
    ImGui::SetClipboardText(text.c_str());
  }
  ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.72f, 0.35f, 1.0f));
  for (const auto& warning : warnings)
    ImGui::TextWrapped("! %s", warning.c_str());
  ImGui::PopStyleColor();
  ImGui::PopID();
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

// A difference between two independently sampled versions of a nominally
// coincident curve can still contain a few integer-grid crumbs. They are far
// below the detail this view retains, but a mixture of tiny outer and hole
// rings can make the triangulator reject the whole otherwise-empty surface.
Paths64 drawable_surface(const Paths64& region, double spacing) {
  const double integer_scale2 = wave::kIntegerScale * wave::kIntegerScale;
  const double minimum_area = spacing * spacing * 0.25 * integer_scale2;
  Paths64 visible;
  visible.reserve(region.size());
  for (const auto& ring : region)
    if (std::abs(Clipper2Lib::Area(ring)) >= minimum_area)
      visible.push_back(ring);
  return drawable_region(visible, spacing);
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

ImU32 chain_colour(std::size_t chain, std::size_t layer,
                   std::size_t layer_count, int alpha) {
  const float hue = std::fmod(static_cast<float>(chain) * 0.61803398875f, 1.0f);
  const float depth = layer_count <= 1
                          ? 0.0f
                          : static_cast<float>(layer) /
                                static_cast<float>(layer_count - 1);
  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;
  ImGui::ColorConvertHSVtoRGB(hue, 0.72f, 1.0f - depth * 0.28f, r, g, b);
  return IM_COL32(static_cast<int>(r * 255.0f),
                  static_cast<int>(g * 255.0f),
                  static_cast<int>(b * 255.0f), alpha);
}

const cam::LevelPlan* current_level(const App& app) {
  if (!app.has_plan) return nullptr;
  if (app.selected_plan_operation < 0 ||
      app.selected_plan_operation >= static_cast<int>(app.plan.operations.size()))
    return nullptr;
  const auto& planned_operation =
      app.plan.operations[app.selected_plan_operation];
  const auto& levels = planned_operation.levels;
  if (app.selected_level < 0 || app.selected_level >= static_cast<int>(levels.size()))
    return nullptr;
  return &levels[app.selected_level];
}

double level_length(const cam::LevelPlan* level) {
  return level ? wave::toolpath_length(level->toolpath) : 0.0;
}

void sync_playback_time(App& app) {
  const cam::LevelPlan* level = current_level(app);
  app.playback.motion_seconds =
      level ? cam::feed_seconds_to(level->feeds, app.playback.distance) : 0.0;
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

const char* motion_phase_name(const cam::MotionCursor& cursor) {
  if (cursor.to_z > cursor.from_z) return "Rapid retract";
  if (cursor.to_z < cursor.from_z)
    return cursor.rapid ? "Rapid descent" : "Plunge";
  if (cursor.rapid) return "Rapid traverse";
  return wave::move_kind_name(cursor.kind);
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
              app.playback.motion_seconds,
          0.0);
      if (budget < left_in_pass) {
        app.playback.motion_seconds += budget;
        app.playback.distance = cam::feed_distance_at_seconds(
            level->feeds, app.playback.motion_seconds);
        return;
      }
    } else {
      left_in_pass = std::max(total - app.playback.distance, 0.0);
      if (budget < left_in_pass) {
        app.playback.distance += budget;
        sync_playback_time(app);
        return;
      }
    }
    budget -= left_in_pass;
    app.playback.distance = total;
    app.playback.motion_seconds = cam::feed_schedule_seconds(level->feeds);

    if (!app.playback.follow) {
      app.playback.distance = 0.0;
      app.playback.motion_seconds = 0.0;
      return;
    }
    const auto& levels = app.plan.operations[app.selected_plan_operation].levels;
    int next = app.selected_level + 1;
    while (next < static_cast<int>(levels.size()) && levels[next].nothing_to_cut)
      ++next;
    if (next < static_cast<int>(levels.size())) {
      app.selected_level = next;
      app.playback.distance = 0.0;
      app.playback.motion_seconds = 0.0;
      refresh_material(app);
      continue;
    }
    int operation = app.selected_plan_operation + 1;
    while (operation < static_cast<int>(app.plan.operations.size())) {
      const auto& next_levels = app.plan.operations[operation].levels;
      int first_cut = 0;
      while (first_cut < static_cast<int>(next_levels.size()) &&
             next_levels[first_cut].nothing_to_cut)
        ++first_cut;
      if (first_cut < static_cast<int>(next_levels.size())) {
        app.selected_plan_operation = operation;
        app.selected_operation = operation;
        app.selected_level = first_cut;
        app.playback.distance = 0.0;
        app.playback.motion_seconds = 0.0;
        refresh_material(app);
        break;
      }
      ++operation;
    }
    if (operation >= static_cast<int>(app.plan.operations.size())) {
      app.playback.playing = false;
      return;
    }
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
  // The pass opens where the previous one finished, which can be well outside
  // anything this one touches.
  for (const auto& phase : level->feeds.motion)
    if (phase.arrival)
      reference.push_back(Clipper2Lib::Path64{phase.from, phase.to});
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

// The same marker for the flat pass view, which is one depth seen from above:
// two axes rather than three, since the third points at the reader.
void draw_origin_marker(const App& app, ImDrawList* list,
                        const Transform& to_screen) {
  const ImVec2 centre = to_screen(Point64(
      wave::to_integer(-app.job.origin_x), wave::to_integer(-app.job.origin_y)));
  static const ImVec2 kSteps[2] = {{46.0f, 0.0f}, {0.0f, -46.0f}};
  static const ImU32 kColours[2] = {IM_COL32(232, 96, 92, 255),
                                    IM_COL32(126, 205, 122, 255)};
  static const char* const kNames[2] = {"X", "Y"};
  for (int axis = 0; axis < 2; ++axis) {
    const ImVec2 end{centre.x + kSteps[axis].x, centre.y + kSteps[axis].y};
    list->AddLine(centre, end, IM_COL32(12, 14, 18, 200), 3.5f);
    list->AddLine(centre, end, kColours[axis], 1.7f);
    list->AddText({end.x + 3.0f, end.y - 6.0f}, kColours[axis], kNames[axis]);
  }
  list->AddCircleFilled(centre, 3.5f, IM_COL32(12, 14, 18, 220));
  list->AddCircleFilled(centre, 2.0f, IM_COL32(238, 240, 245, 255));
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

    const auto& operation = app.plan.operations[app.selected_plan_operation];
    const std::size_t layer_index = static_cast<std::size_t>(app.selected_level);
    if (layer_index < operation.inter_layer.layers.size()) {
      const wave::LayerTopology& topology =
          operation.inter_layer.layers[layer_index];
      if (app.node_debug_colouring != NodeDebugColouring::None &&
          layer_index < operation.inter_layer.node_chain_ids.size()) {
        const auto& ids = operation.inter_layer.node_chain_ids[layer_index];
        for (std::size_t node = 0; node < topology.nodes.size() && node < ids.size();
             ++node) {
          const bool joined = ids[node] != wave::kNoTopologyId;
          const ImU32 edge = app.node_debug_colouring ==
                                     NodeDebugColouring::Node
                                 ? chain_colour(node, 0, 1, 245)
                                 : (joined
                                        ? chain_colour(ids[node], layer_index,
                                                       operation.levels.size(), 245)
                                        : IM_COL32(145, 150, 165, 150));
          // Use the same fill path as simulated stock. Pre-blending into an
          // opaque dark colour avoids alpha accumulating on shared triangle
          // edges while leaving the toolpath legible above the node area.
          const ImU32 fill = blend(IM_COL32(16, 19, 25, 255), edge, 0.18f);
          fill_paths(list, topology.nodes[node].footprint, to_screen, fill,
                     &app.fill_failures);
          draw_paths(list, topology.nodes[node].footprint, to_screen, edge, 2.2f);
        }
      }
      if (app.show_entry_front_chains &&
          layer_index < operation.inter_layer.entry_front_chain_ids.size()) {
        const auto& ids = operation.inter_layer.entry_front_chain_ids[layer_index];
        for (std::size_t entry = 0;
             entry < topology.entry_fronts.size() && entry < ids.size(); ++entry) {
          const std::size_t id = topology.entry_fronts[entry];
          if (id >= level->lineage.entry_fronts.size()) continue;
          const auto& path = level->lineage.entry_fronts[id].path;
          std::vector<ImVec2> points;
          points.reserve(path.size());
          for (const Point64& point : path) points.push_back(to_screen(point));
          if (points.size() >= 2)
            list->AddPolyline(points.data(), static_cast<int>(points.size()),
                              ids[entry] == wave::kNoTopologyId
                                  ? IM_COL32(145, 150, 165, 150)
                                  : chain_colour(ids[entry], layer_index,
                                                 operation.levels.size(), 255),
                              ImDrawFlags_None, 4.0f);
        }
      }
      if (app.show_boundary_contacts) {
        for (const auto& generation : level->lineage.generations)
          for (const auto& front : generation.fronts)
            if (!front.closed)
              for (const auto& contact : front.boundary_contacts)
                if (contact.valid)
                  list->AddCircleFilled(to_screen(contact.point), 3.5f,
                                        IM_COL32(255, 110, 210, 245));
      }
    }

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

    // The rapid in from the previous pass. It covers no path distance, so it
    // cannot be placed by `travelled` like the rest; whether it has happened is
    // a question about elapsed time.
    if (app.show_toolpath && app.show_rapids)
      for (const auto& phase : level->feeds.motion) {
        if (!phase.arrival) continue;
        const double duration = phase.length / (std::max(phase.rate, 1.0) / 60.0);
        const ImU32 colour = app.playback.motion_seconds >= duration
                                 ? IM_COL32(70, 82, 100, 255)
                                 : IM_COL32(70, 82, 100, 56);
        list->AddLine(to_screen(phase.from), to_screen(phase.to), colour, 1.0f);
      }

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
      const wave::Point64 position = level->feeds.motion.empty()
                                         ? wave::toolpath_at(
                                               level->toolpath,
                                               app.playback.distance)
                                               .position
                                         : cam::feed_motion_at_seconds(
                                               level->feeds,
                                               app.playback.motion_seconds)
                                               .position;
      const ImVec2 centre = to_screen(position);
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

  draw_origin_marker(app, list, to_screen);

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

#include "prism_view.h"
PrismView prism_view;

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

ImU32 shaded_alpha(const float base[3], float lambert, int alpha) {
  const float k = 0.42f + 0.58f * std::clamp(lambert, 0.0f, 1.0f);
  return IM_COL32(static_cast<int>(base[0] * k * 255.0f + 0.5f),
                  static_cast<int>(base[1] * k * 255.0f + 0.5f),
                  static_cast<int>(base[2] * k * 255.0f + 0.5f), alpha);
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
                 const float wall_colour[3], const float face_colour[3], ImU32 edge,
                 int alpha = 255) {
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
           shaded_alpha(wall_colour,
                        normal.x * light.x + normal.y * light.y +
                            normal.z * light.z,
                        alpha),
           0);
    }
  }

  const ImU32 face = shaded_alpha(face_colour, light.z, alpha);
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

// A loaded model as the triangles it was given, rather than as the stack of
// prisms a slicer sees it as. Sampling the mesh at a fixed set of depths put a
// staircase on every curved face and littered the flat ones with the hairline
// slivers two independently decimated slices differ by -- neither of which is
// in the model. Nothing in this view is a plan, so nothing here has any reason
// to look at the models through a slicer.
//
// Faces turned away from the eye are dropped. Every model loaded here is a
// closed solid wound outward, so those are exactly the faces the rest of the
// solid stands in front of.
void append_mesh(std::vector<Facet>& facets, std::vector<ImVec2>& pool,
                 const wave::Mesh& mesh, const Projection& projection,
                 const Vec3& light, const float colour[3], int alpha) {
  facets.reserve(facets.size() + mesh.triangles.size() / 2);
  pool.reserve(pool.size() + mesh.triangles.size() * 3 / 2);
  for (const wave::Triangle3f& triangle : mesh.triangles) {
    const Vec3 corners[3]{{triangle.a.x, triangle.a.y, triangle.a.z},
                          {triangle.b.x, triangle.b.y, triangle.b.z},
                          {triangle.c.x, triangle.c.y, triangle.c.z}};
    const Vec3 u{corners[1].x - corners[0].x, corners[1].y - corners[0].y,
                 corners[1].z - corners[0].z};
    const Vec3 v{corners[2].x - corners[0].x, corners[2].y - corners[0].y,
                 corners[2].z - corners[0].z};
    Vec3 normal{u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z,
                u.x * v.y - u.y * v.x};
    const float length = std::sqrt(normal.x * normal.x + normal.y * normal.y +
                                   normal.z * normal.z);
    if (length < 1e-12f) continue;  // degenerate, and it has no normal to shade
    normal = {normal.x / length, normal.y / length, normal.z / length};
    if (normal.x * projection.toward.x + normal.y * projection.toward.y +
            normal.z * projection.toward.z <= 0.0f)
      continue;

    Facet facet;
    facet.first = static_cast<std::uint32_t>(pool.size());
    facet.count = 3;
    float depth = 0.0f;
    for (const Vec3& corner : corners) {
      pool.push_back(projection.point(corner));
      depth += projection.depth(corner);
    }
    facet.depth = depth / 3.0f;
    facet.fill = shaded_alpha(colour,
                              normal.x * light.x + normal.y * light.y +
                                  normal.z * light.z,
                              alpha);
    facet.edge = 0;
    facets.push_back(facet);
  }
}

// One entry front, as a vertical ribbon along the curve it actually occupies.
// A front is a stretch of wall the wave starts from, not a volume: giving it
// thickness put faces inside the stock for the depth sort to fight over, and
// said something about the material that is not what a front is.
//
// A ribbon has no inside, so both faces are lit and neither is culled. Where a
// lead-in stands the front off the stock, the ribbon stands off with it, which
// is where the pass view draws the same curve.
void append_ribbon(std::vector<Facet>& facets, std::vector<ImVec2>& pool,
                   const wave::Path64& path, bool closed, double bottom, double top,
                   const Projection& projection, const Vec3& light,
                   const float colour[3], int alpha) {
  if (path.size() < 2) return;
  const std::size_t segments = closed ? path.size() : path.size() - 1;
  for (std::size_t i = 0; i < segments; ++i) {
    const Point64& a = path[i];
    const Point64& b = path[(i + 1) % path.size()];
    const float dx = static_cast<float>(wave::to_world(b.x - a.x));
    const float dy = static_cast<float>(wave::to_world(b.y - a.y));
    const float length = std::sqrt(dx * dx + dy * dy);
    if (length < 1e-6f) continue;
    const float nx = dy / length;
    const float ny = -dx / length;
    const float lambert = std::abs(nx * light.x + ny * light.y);

    const Vec3 a_top{static_cast<float>(wave::to_world(a.x)),
                     static_cast<float>(wave::to_world(a.y)),
                     static_cast<float>(top)};
    const Vec3 b_top{static_cast<float>(wave::to_world(b.x)),
                     static_cast<float>(wave::to_world(b.y)),
                     static_cast<float>(top)};
    const Vec3 a_bottom{a_top.x, a_top.y, static_cast<float>(bottom)};
    const Vec3 b_bottom{b_top.x, b_top.y, static_cast<float>(bottom)};

    Facet facet;
    facet.first = static_cast<std::uint32_t>(pool.size());
    facet.count = 4;
    float depth = 0.0f;
    for (const Vec3& corner : {a_top, b_top, b_bottom, a_bottom}) {
      pool.push_back(projection.point(corner));
      depth += projection.depth(corner);
    }
    // Biased towards the eye so a front lying flat on the stock wall, which is
    // what one with no lead-in does, is drawn over it rather than into it.
    facet.depth = depth * 0.25f + 0.05f;
    facet.fill = shaded_alpha(colour, lambert, alpha);
    facet.edge = 0;
    facets.push_back(facet);
  }
}

// The active cutter in the stacked result. It is intentionally translucent:
// the point of this view is to see both where the tool is and the stock it is
// passing through. Side quads, a top cap, and the two silhouette rails make it
// read as a cylinder under the same orthographic projection as the slabs.
void append_cutter(std::vector<Facet>& facets, std::vector<ImVec2>& pool,
                   const Vec3& centre, float radius, float bottom, float top,
                   const Projection& projection, const Vec3& light) {
  constexpr std::size_t kSides = 40;
  constexpr float kPi = 3.14159265358979323846f;
  const float colour[3] = {0.48f, 0.86f, 1.0f};

  const auto point = [&](std::size_t side, float z) {
    const float angle = 2.0f * kPi * static_cast<float>(side) /
                        static_cast<float>(kSides);
    return Vec3{centre.x + radius * std::cos(angle),
                centre.y + radius * std::sin(angle), z};
  };
  const auto emit = [&](std::initializer_list<Vec3> corners, float depth_bias,
                        ImU32 fill, ImU32 edge) {
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
    facet.edge = edge;
    facets.push_back(facet);
  };

  for (std::size_t side = 0; side < kSides; ++side) {
    const std::size_t next = (side + 1) % kSides;
    const float angle = 2.0f * kPi *
                        (static_cast<float>(side) + 0.5f) /
                        static_cast<float>(kSides);
    const Vec3 normal{std::cos(angle), std::sin(angle), 0.0f};
    const float facing = normal.x * projection.toward.x +
                         normal.y * projection.toward.y;
    const int alpha = facing > 0.0f ? 54 : 18;
    emit({point(side, top), point(next, top), point(next, bottom),
          point(side, bottom)},
         0.0f,
         shaded_alpha(colour,
                      normal.x * light.x + normal.y * light.y,
                      alpha),
         0);

    // The visible top is a triangle fan. Separate triangles do not overlap, so
    // their alpha remains uniform while still fitting the convex-facet renderer.
    emit({Vec3{centre.x, centre.y, top}, point(side, top), point(next, top)},
         0.003f, shaded_alpha(colour, light.z, 42), 0);
  }

  const ImU32 outline = IM_COL32(165, 226, 250, 175);
  for (const float z : {bottom, top}) {
    Facet ring;
    ring.first = static_cast<std::uint32_t>(pool.size());
    ring.count = kSides;
    float depth = 0.0f;
    for (std::size_t side = 0; side < kSides; ++side) {
      const Vec3 at = point(side, z);
      pool.push_back(projection.point(at));
      depth += projection.depth(at);
    }
    ring.depth = depth / static_cast<float>(kSides) + 0.006f;
    ring.edge = outline;
    facets.push_back(ring);
  }

  const float view_angle =
      std::atan2(projection.toward.y, projection.toward.x);
  for (const float angle : {view_angle - kPi * 0.5f,
                            view_angle + kPi * 0.5f}) {
    const Vec3 lower{centre.x + radius * std::cos(angle),
                     centre.y + radius * std::sin(angle), bottom};
    const Vec3 upper{lower.x, lower.y, top};
    emit({lower, upper}, 0.007f, 0, outline);
  }
}

// The parts of an orbited stack view that say nothing about what is being
// drawn: the canvas, the mouse, the projection, the light, and the painter's
// sort. Three views share them and differ only in the slabs they contribute.
struct OrbitScene {
  ImVec2 top_left;
  ImVec2 size;
  ImDrawList* list = nullptr;
  bool hovered = false;
  Projection projection;
  Vec3 light;
  double spacing = 0.05;
  std::vector<Facet> facets;
  std::vector<ImVec2> pool;
};

// Left-drag orbits, right-drag pans, wheel zooms. One reader for every 3D view,
// so the gesture means the same thing in all of them and a view added later
// cannot quietly disagree.
//
// The pan is in pixels and is not divided by the zoom: the drag moves what is
// under the pointer by the distance the pointer moved, which is what makes it
// feel like dragging the model rather than steering a camera at it.
void orbit_controls(App& app, bool hovered) {
  if (!hovered) return;
  const ImGuiIO& io = ImGui::GetIO();
  if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
    app.orbit.yaw += io.MouseDelta.x * 0.009f;
    app.orbit.pitch =
        std::clamp(app.orbit.pitch - io.MouseDelta.y * 0.009f, 0.06f, 1.5f);
  }
  if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
    app.orbit.pan.x += io.MouseDelta.x;
    app.orbit.pan.y += io.MouseDelta.y;
  }
  if (io.MouseWheel != 0.0f) {
    const float before = app.orbit.zoom;
    app.orbit.zoom =
        std::clamp(before * std::pow(1.12f, io.MouseWheel), 0.1f, 40.0f);
    // Scaled by however much the zoom actually moved, clamp included, which
    // makes the wheel scale the picture about the middle of the canvas. Left
    // alone, the view expands about a centre that panning has pushed off the
    // edge, and zooming in after panning throws the model out of shot. The 2D
    // pass view has always done this; these now agree with it.
    const float factor = before > 0.0f ? app.orbit.zoom / before : 1.0f;
    app.orbit.pan.x *= factor;
    app.orbit.pan.y *= factor;
  }
}

// Opens the child window and reads the orbit controls. Returns false when the
// view has nothing to show, having already drawn `empty_message` and closed up.
bool begin_scene(App& app, OrbitScene& scene, const char* id) {
  ImGui::BeginChild(id, ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  scene.top_left = ImGui::GetCursorScreenPos();
  scene.size = ImGui::GetContentRegionAvail();
  scene.size.x = std::max(scene.size.x, 50.0f);
  scene.size.y = std::max(scene.size.y, 50.0f);
  // A view may put a control over its own canvas, so the canvas has to let a
  // later item take the mouse where the two overlap.
  ImGui::SetNextItemAllowOverlap();
  ImGui::InvisibleButton(id, scene.size,
                         ImGuiButtonFlags_MouseButtonLeft |
                             ImGuiButtonFlags_MouseButtonRight);
  scene.hovered = ImGui::IsItemHovered();
  scene.list = ImGui::GetWindowDrawList();
  const ImVec2 bottom_right{scene.top_left.x + scene.size.x,
                            scene.top_left.y + scene.size.y};
  scene.list->AddRectFilled(scene.top_left, bottom_right, IM_COL32(14, 17, 22, 255));
  scene.list->PushClipRect(scene.top_left, bottom_right, true);
  orbit_controls(app, scene.hovered);
  return true;
}

void scene_message(OrbitScene& scene, const char* message) {
  const ImVec2 extent = ImGui::CalcTextSize(message);
  scene.list->AddText({scene.top_left.x + (scene.size.x - extent.x) * 0.5f,
                       scene.top_left.y + (scene.size.y - extent.y) * 0.5f},
                      IM_COL32(120, 130, 145, 255), message);
}

void end_scene(OrbitScene& scene) {
  scene.list->PopClipRect();
  ImGui::EndChild();
}

// A control in the corner of a view, declared after the canvas so it takes the
// mouse where the two overlap and orbiting does not start under the pointer.
// Returns true when it was pressed.
bool scene_button(OrbitScene& scene, const char* label, float from_right) {
  const ImVec2 saved = ImGui::GetCursorScreenPos();
  const float width = ImGui::CalcTextSize(label).x +
                      ImGui::GetStyle().FramePadding.x * 2.0f;
  ImGui::SetCursorScreenPos({scene.top_left.x + scene.size.x - width - from_right,
                             scene.top_left.y + 8.0f});
  const bool pressed = ImGui::Button(label);
  if (ImGui::IsItemHovered()) scene.hovered = false;
  // Back inside the canvas rather than to where the canvas left it: that was
  // one item's spacing past the bottom, and putting the cursor there grows the
  // parent, which ImGui rightly complains about.
  (void)saved;
  ImGui::SetCursorScreenPos(scene.top_left);
  return pressed;
}

// Frames the view on a fixed world box. Nothing that moves may contribute to
// it, or the model swims about under the thing being watched.
void aim_scene(App& app, OrbitScene& scene, const Clipper2Lib::Rect64& extent,
               double lowest, double highest) {
  const double width = wave::to_world(extent.right - extent.left);
  const double height = wave::to_world(extent.bottom - extent.top);
  const double span = std::max({width, height, highest - lowest, 1e-3});
  const Vec3 centre{
      static_cast<float>(wave::to_world(extent.left + extent.right) * 0.5),
      static_cast<float>(wave::to_world(extent.top + extent.bottom) * 0.5),
      static_cast<float>((lowest + highest) * 0.5)};
  const float cy = std::cos(app.orbit.yaw);
  const float sy = std::sin(app.orbit.yaw);
  const float cp = std::cos(app.orbit.pitch);
  const float sp = std::sin(app.orbit.pitch);
  scene.projection = Projection{
      {scene.top_left.x + scene.size.x * 0.5f + app.orbit.pan.x,
       scene.top_left.y + scene.size.y * 0.5f + app.orbit.pan.y},
      centre,
      {-sy, cy, 0.0f},
      {-sp * cy, -sp * sy, cp},
      {cp * cy, cp * sy, sp},
      static_cast<float>(std::min(scene.size.x, scene.size.y) * 0.72 / span) *
          app.orbit.zoom};

  // One vertex every thousandth of the part is plenty at any sane zoom, and it
  // keeps the number of quads in the hundreds rather than the tens of thousands.
  scene.spacing = std::max(span / 400.0, 0.05);

  // A single light fixed to the eye, so the model stays lit however it is
  // orbited. Every surface here is horizontal or vertical, which is exactly the
  // case a flat colour cannot describe on its own.
  const Vec3 raw{scene.projection.toward.x * 0.85f + scene.projection.up.x * 0.35f +
                     scene.projection.right.x * 0.35f,
                 scene.projection.toward.y * 0.85f + scene.projection.up.y * 0.35f +
                     scene.projection.right.y * 0.35f,
                 scene.projection.toward.z * 0.85f + scene.projection.up.z * 0.35f +
                     scene.projection.right.z * 0.35f};
  const float raw_length =
      std::max(1e-6f, std::sqrt(raw.x * raw.x + raw.y * raw.y + raw.z * raw.z));
  scene.light = Vec3{raw.x / raw_length, raw.y / raw_length, raw.z / raw_length};
}

// Painter's algorithm, then straight into the draw list.
// Depth sorts one body of facets and paints it. A body at a time rather than
// everything at once, so a caller with two of them can decide that one stands
// in front of the other outright instead of interleaving the two sorts.
void paint_facets(ImDrawList* list, std::vector<Facet>& facets,
                  const std::vector<ImVec2>& pool) {
  std::sort(facets.begin(), facets.end(),
            [](const Facet& a, const Facet& b) { return a.depth < b.depth; });
  // A wall is one quad per edge of the ring, and an antialiased fill insets
  // every one of them by half a pixel, so the wall would come out ruled with
  // hairlines where the quads meet.
  const ImDrawListFlags saved_flags = list->Flags;
  list->Flags &= ~ImDrawListFlags_AntiAliasedFill;
  for (const Facet& facet : facets) {
    if (facet.count < 2) continue;
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
                        facet.count > 2 ? ImDrawFlags_Closed : ImDrawFlags_None,
                        1.0f);
  }
  list->Flags = saved_flags;
}

void paint_scene(OrbitScene& scene) {
  paint_facets(scene.list, scene.facets, scene.pool);
}

// A triad at the program origin, drawn over the scene rather than into it.
// It is a statement about the coordinates, not a thing standing in the work, so
// it should not be able to go behind the work and disappear -- the one moment
// it is wanted is the moment the origin has been put somewhere unexpected,
// which is very often inside the blank.
//
// Sized in pixels, not millimetres. A fixed world length would vanish on a
// large part and swallow a small one.
void draw_origin_gizmo(const App& app, const OrbitScene& scene) {
  const Projection& projection = scene.projection;
  if (!(projection.scale > 0.0f)) return;
  // Where the program's zero sits, in the model's own coordinates. The job
  // carries the offset added to every posted coordinate, which is the negative
  // of the point it is measured from.
  const Vec3 at{static_cast<float>(-app.job.origin_x),
                static_cast<float>(-app.job.origin_y),
                static_cast<float>(-app.job.origin_z)};
  const float reach = 46.0f / projection.scale;
  const ImVec2 centre = projection.point(at);

  static const Vec3 kDirections[3] = {
      {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
  static const ImU32 kColours[3] = {IM_COL32(232, 96, 92, 255),
                                    IM_COL32(126, 205, 122, 255),
                                    IM_COL32(104, 158, 246, 255)};
  static const char* const kNames[3] = {"X", "Y", "Z"};
  for (int axis = 0; axis < 3; ++axis) {
    const Vec3& step = kDirections[axis];
    const Vec3 tip{at.x + step.x * reach, at.y + step.y * reach,
                   at.z + step.z * reach};
    const ImVec2 end = projection.point(tip);
    // Outlined, because an axis crossing the part it is drawn over is otherwise
    // the same brightness as what it crosses.
    scene.list->AddLine(centre, end, IM_COL32(12, 14, 18, 200), 3.5f);
    scene.list->AddLine(centre, end, kColours[axis], 1.7f);
    scene.list->AddText({end.x + 3.0f, end.y - 6.0f}, kColours[axis],
                        kNames[axis]);
  }
  scene.list->AddCircleFilled(centre, 3.5f, IM_COL32(12, 14, 18, 220));
  scene.list->AddCircleFilled(centre, 2.0f, IM_COL32(238, 240, 245, 255));
}

// ---------------------------------------------------------------------------
// Stage 1: the models, before there is a plan to look at.

// The Z range the operation will actually cut, so the depth range being set can
// be seen against the models rather than read off two numbers.
void draw_setup(App& app) {
  OrbitScene scene;
  begin_scene(app, scene, "setup");
  if (!app.geometry.has_part() || !app.geometry.has_stock()) {
    scene_message(scene, "Load a part and a stock model to inspect them");
    end_scene(scene);
    return;
  }
  double low = 0.0;
  double high = 0.0;
  if (!app.geometry.work_range(low, high)) {
    scene_message(scene, "The models have no depth to cut");
    end_scene(scene);
    return;
  }

  const cam::Operation* operation =
      app.job.operations.empty() ? nullptr
                                 : &app.job.operations[std::clamp(
                                       app.selected_operation, 0,
                                       static_cast<int>(app.job.operations.size()) - 1)];

  const bool draw_stock = app.models != ModelShow::Part;
  const bool draw_part = app.models != ModelShow::Stock;
  // A rectangular blank has no mesh behind it, so it is drawn as the single
  // prism it is defined to be rather than pretending to a model it never had.
  const bool boxed = app.geometry.box_stock();
  const wave::Paths64 box =
      boxed ? app.geometry.stock_outline(low, operation ? operation->slice_grid : 32)
            : wave::Paths64{};

  Clipper2Lib::Rect64 extent;
  bool any = false;
  const auto cover = [&](std::int64_t left, std::int64_t top, std::int64_t right,
                         std::int64_t bottom) {
    if (!any) {
      extent = Clipper2Lib::Rect64(left, top, right, bottom);
      any = true;
      return;
    }
    extent.left = std::min(extent.left, left);
    extent.right = std::max(extent.right, right);
    extent.top = std::min(extent.top, top);
    extent.bottom = std::max(extent.bottom, bottom);
  };
  const auto cover_mesh = [&](const wave::Mesh& mesh) {
    if (!mesh.bounds.valid) return;
    cover(wave::to_integer(mesh.bounds.lower.x), wave::to_integer(mesh.bounds.lower.y),
          wave::to_integer(mesh.bounds.upper.x), wave::to_integer(mesh.bounds.upper.y));
  };
  if (draw_part) cover_mesh(app.geometry.part().mesh());
  if (draw_stock) {
    if (boxed) {
      if (!box.empty()) {
        const auto bounds = Clipper2Lib::GetBounds(box);
        cover(bounds.left, bounds.top, bounds.right, bounds.bottom);
      }
    } else {
      cover_mesh(app.geometry.stock().mesh());
    }
  }
  if (!any) {
    scene_message(scene, "Neither model has anything to show");
    end_scene(scene);
    return;
  }
  aim_scene(app, scene, extent, low, high);

  // Both bodies whole, and the stock made see-through when both are on. Cutting
  // a part-shaped hole in the stock instead gave every wall of the part an
  // exact coincident twin on the hole's inside for the depth sort to pick
  // between; letting the two solids simply overlap has no coincidence in it.
  if (draw_part) {
    const float colour[3]{0.42f, 0.60f, 0.80f};
    append_mesh(scene.facets, scene.pool, app.geometry.part().mesh(),
                scene.projection, scene.light, colour, 255);
  }
  if (draw_stock) {
    const float colour[3]{0.34f, 0.38f, 0.45f};
    const int alpha = draw_part ? 90 : 255;
    if (boxed) {
      const wave::Paths64 rings = drawable_region(box, scene.spacing);
      if (!rings.empty()) {
        const float face[3]{0.40f, 0.44f, 0.52f};
        append_slab(scene.facets, scene.pool, rings, fill_mesh(rings, nullptr),
                    rings, low, high, scene.projection, scene.light, colour,
                    face, IM_COL32(160, 180, 205, 70), alpha);
      }
    } else {
      append_mesh(scene.facets, scene.pool, app.geometry.stock().mesh(),
                  scene.projection, scene.light, colour, alpha);
    }
  }
  paint_scene(scene);
  draw_origin_gizmo(app, scene);

  // One button cycling the three, rather than two checkboxes that can both be
  // off and leave the view empty.
  if (scene_button(scene, model_show_name(app.models), 10.0f))
    app.models = app.models == ModelShow::Both      ? ModelShow::Stock
                 : app.models == ModelShow::Stock   ? ModelShow::Part
                                                    : ModelShow::Both;

  char caption[220];
  std::snprintf(caption, sizeof caption,
                "Part and stock, Z %.2f to %.2f mm   Drag: orbit   Right-drag: pan   Wheel: "
                "zoom",
                low, high);
  scene.list->AddText({scene.top_left.x + 12.0f, scene.top_left.y + 10.0f},
                      IM_COL32(130, 142, 158, 255), caption);
  if (operation) {
    std::snprintf(caption, sizeof caption,
                  "This operation cuts depth %.2f to %.2f mm (Z %.2f to %.2f)",
                  operation->depth_from, operation->depth_to,
                  cam::z_at_depth(high, operation->depth_to),
                  cam::z_at_depth(high, operation->depth_from));
    scene.list->AddText({scene.top_left.x + 12.0f, scene.top_left.y + 28.0f},
                        IM_COL32(120, 165, 210, 255), caption);
  }
  end_scene(scene);
}

// ---------------------------------------------------------------------------
// The loaded models, as a translucent sheet over a stack that has already been
// painted. Into a facet list of their own, depth sorted on their own, and drawn
// after: they are an overlay, not another body in the scene.
//
// Sorting them together with the passes was the obvious thing and is wrong. A
// simulated pass wall and the part surface it was cut to sit in the same place
// to within a rounding, so the sort has no real answer for which is in front
// and picks per facet on centroid depth -- the two then interleave, and the
// overlay comes out speckled and crawls as the view turns. Nothing in the
// question the overlay answers needs the model to be occluded by the stack, so
// the fix is to stop asking: it goes on top, and being see-through is what lets
// the stack be read through it.
//
// The part is the more opaque of the two: it is the shape being cut towards,
// and the blank is mostly a boundary.
//
// Warm, where every pass in the stack is cool. The plan view's blue would be
// the consistent choice and is the wrong one here: it is within a shade of the
// colour the passes are already drawn in, and the whole question this overlay
// answers is which of the two a given surface belongs to.
void append_models(const App& app, StackModels which, std::vector<Facet>& facets,
                   std::vector<ImVec2>& pool, const Projection& projection,
                   const Vec3& light, double spacing, int slice_grid) {
  if (which == StackModels::None) return;
  const bool want_part = which != StackModels::Stock;
  const bool want_stock = which != StackModels::Part;

  if (want_part && app.geometry.has_part()) {
    const float colour[3]{0.82f, 0.62f, 0.34f};
    append_mesh(facets, pool, app.geometry.part().mesh(), projection, light,
                colour, 95);
  }
  if (!want_stock || !app.geometry.has_stock()) return;
  const float colour[3]{0.34f, 0.38f, 0.45f};
  if (!app.geometry.box_stock()) {
    append_mesh(facets, pool, app.geometry.stock().mesh(), projection, light,
                colour, 55);
    return;
  }
  // A rectangular blank has no mesh behind it, so it is the prism it is defined
  // to be, exactly as the plan view draws it.
  double low = 0.0;
  double high = 0.0;
  if (!app.geometry.work_range(low, high)) return;
  const Paths64 rings =
      drawable_region(app.geometry.stock_outline(low, slice_grid), spacing);
  if (rings.empty()) return;
  const float face[3]{0.40f, 0.44f, 0.52f};
  append_slab(facets, pool, rings, fill_mesh(rings, nullptr), rings, low, high,
              projection, light, colour, face, IM_COL32(160, 180, 205, 60), 55);
}

void draw_result(App& app) {
  ImGui::BeginChild("result", ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  const ImVec2 top_left = ImGui::GetCursorScreenPos();
  ImVec2 size = ImGui::GetContentRegionAvail();
  size.x = std::max(size.x, 50.0f);
  size.y = std::max(size.y, 50.0f);
  // The overlay control sits over this canvas, so the canvas has to let a later
  // item take the mouse where the two meet, or orbiting starts under it.
  ImGui::SetNextItemAllowOverlap();
  ImGui::InvisibleButton("stack", size,
                         ImGuiButtonFlags_MouseButtonLeft |
                             ImGuiButtonFlags_MouseButtonRight);
  const bool hovered = ImGui::IsItemHovered();
  ImDrawList* list = ImGui::GetWindowDrawList();
  list->AddRectFilled(top_left, ImVec2(top_left.x + size.x, top_left.y + size.y),
                      IM_COL32(14, 17, 22, 255));
  list->PushClipRect(top_left, ImVec2(top_left.x + size.x, top_left.y + size.y), true);

  orbit_controls(app, hovered);

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
  const auto& planned_operation =
      app.plan.operations[app.selected_plan_operation];
  const auto& levels = planned_operation.levels;
  if (levels.empty()) {
    finish("This operation has no passes");
    return;
  }

  const int active_level = std::clamp(
      app.selected_level, 0, static_cast<int>(levels.size()) - 1);
  const cam::LevelPlan& active = levels[active_level];
  const cam::Tool* active_tool = app.tools.find(planned_operation.tool_id);
  std::optional<cam::MotionCursor> cutter_cursor;
  if (active_tool != nullptr && !active.toolpath.moves.empty()) {
    if (!active.feeds.motion.empty()) {
      cutter_cursor = cam::feed_motion_at_seconds(
          active.feeds, app.playback.motion_seconds);
    } else {
      const wave::ToolpathCursor planar =
          wave::toolpath_at(active.toolpath, app.playback.distance);
      cutter_cursor = cam::MotionCursor{planar.position, active.floor_z,
                                        app.playback.distance, planar.kind,
                                        planar.kind == wave::MoveKind::Retract};
    }
  }

  // Bounds over the whole operation: every pass's stock and every pass's
  // toolpath, plus room above for the flute. Deliberately nothing that moves.
  // Framing the live cutter kept the tool centred and swam the model around it
  // on every frame, which made the stack impossible to read.
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
  if (planned_operation.composed_stock) {
    const auto include = [&](const cam::StockSlab& slab) {
      if (slab.region->empty()) return;
      const auto b = Clipper2Lib::GetBounds(*slab.region);
      if (!any) { extent_2d = b; lowest=slab.bottom; highest=slab.top; any=true; }
      else {
        extent_2d.left=std::min(extent_2d.left,b.left); extent_2d.right=std::max(extent_2d.right,b.right);
        extent_2d.top=std::min(extent_2d.top,b.top); extent_2d.bottom=std::max(extent_2d.bottom,b.bottom);
        lowest=std::min(lowest,slab.bottom); highest=std::max(highest,slab.top);
      }
    };
    for (const auto& slab : planned_operation.preview_context) include(slab);
    for (const auto& level : levels) for (const auto& slab : level.local_stock) include(slab);
  }
  const double material_lowest = lowest;
  double material_highest = highest;
  if (!planned_operation.subdivisions.empty() &&
      planned_operation.subdivisions.front().level_count > 0) {
    const std::size_t first =
        planned_operation.subdivisions.front().first_level;
    if (first < levels.size())
      material_highest = levels[first].floor_z + levels[first].axial_depth;
  }
  // Everywhere the cutter is ever commanded to be, with room for its flute,
  // taken once from the plan rather than from where it happens to stand this
  // frame. A lead-in and a retract both reach outside the stock, so the stack
  // alone is not enough to keep the tool in shot.
  if (any && active_tool != nullptr) {
    const std::int64_t radius = wave::to_integer(active_tool->diameter * 0.5);
    for (const auto& level : levels) {
      for (const cam::MotionPhase& phase : level.feeds.motion) {
        for (const Point64& point : {phase.from, phase.to}) {
          extent_2d.left = std::min(extent_2d.left, point.x - radius);
          extent_2d.right = std::max(extent_2d.right, point.x + radius);
          extent_2d.top = std::min(extent_2d.top, point.y - radius);
          extent_2d.bottom = std::max(extent_2d.bottom, point.y + radius);
        }
        lowest = std::min({lowest, phase.from_z, phase.to_z});
        highest = std::max({highest,
                            std::max(phase.from_z, phase.to_z) +
                                active_tool->flute_length});
      }
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
      {top_left.x + size.x * 0.5f + app.orbit.pan.x,
       top_left.y + size.y * 0.5f + app.orbit.pan.y},
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

  // Posting and playback keep subdivision groups in execution order. The
  // stack is a physical section, so interleave those groups by their actual Z
  // floors before building its slabs.
  std::vector<std::size_t> display_levels(levels.size());
  for (std::size_t i = 0; i < levels.size(); ++i) display_levels[i] = i;
  std::stable_sort(display_levels.begin(), display_levels.end(),
                   [&](std::size_t a, std::size_t b) {
                     return levels[a].floor_z > levels[b].floor_z;
                   });

  // Future subdivision layers deliberately alias the latest subdivision being
  // evaluated. Once subdivision 2 begins, subdivision 3's virtual layers stop
  // aliasing subdivision 1 and alias subdivision 2 instead. Only the selected
  // source layer ever needs a live, partially simulated stock object.
  const auto stock_for = [&](std::size_t target_index) -> const Paths64* {
    const cam::LevelPlan& target = levels[target_index];
    std::size_t source_index = target_index;
    if (app.stack_follows_playback)
      source_index = cam::subdivision_preview_source(
          planned_operation, active_level, target_index);
    const cam::LevelPlan& source = levels[source_index];
    const Paths64* remaining = &source.simulation.remaining_stock;
    if (source.nothing_to_cut) {
      remaining = &source.incoming_stock;
    } else if (app.stack_follows_playback &&
               source.subdivision == active.subdivision) {
      if (source.subdivision_level > active.subdivision_level) {
        remaining = &source.simulation.initial_stock;
      } else if (source.subdivision_level == active.subdivision_level) {
        remaining = &app.material;
      }
    }
    return remaining->empty() ? nullptr : remaining;
  };

  std::vector<const Paths64*> leftover(display_levels.size(), nullptr);
  for (std::size_t display = 0; display < display_levels.size(); ++display) {
    const std::size_t target = display_levels[display];
    if (!levels[target].stock_slice.empty()) leftover[display] = stock_for(target);
  }

  std::vector<Facet> facets;
  std::vector<ImVec2> pool;
  std::size_t untriangulated = 0;
  std::vector<cam::StockSlab> shader_stock;
  if (!planned_operation.composed_stock) {
    for (std::size_t i=0;i<display_levels.size();++i) {
      if (!leftover[i]) continue;
      const auto& level=levels[display_levels[i]];
      const double top=level.explicit_motion ? level.floor_z+level.axial_depth
          : (i==0 ? material_highest : levels[display_levels[i-1]].floor_z);
      if (top>level.floor_z) shader_stock.push_back({level.floor_z,top,
          std::make_shared<const Paths64>(*leftover[i])});
    }
  }
  if (planned_operation.composed_stock) {
    const int before = app.stack_follows_playback ? active_level : static_cast<int>(levels.size());
    if (app.stock_cache_operation != app.selected_plan_operation || app.stock_cache_level != before) {
      app.stock_cache = cam::preview_before_level(planned_operation, before);
      app.stock_cache_operation = app.selected_plan_operation;
      app.stock_cache_level = before;
    }
    auto stock = app.stock_cache;
    if (app.stack_follows_playback && !active.nothing_to_cut) {
      const auto removed = Clipper2Lib::Difference(active.incoming_stock, app.material, Clipper2Lib::FillRule::NonZero);
      stock = cam::stock_outside(stock, removed, active.floor_z, active.floor_z+active.axial_depth);
    }
    shader_stock=std::move(stock);
  }
  try {
    const GLuint texture=prism_view.render(shader_stock,projection,light,top_left,size,float(span*4));
    list->AddImage(static_cast<ImTextureID>(texture),top_left,
        ImVec2(top_left.x+size.x,top_left.y+size.y),ImVec2(0,1),ImVec2(1,0));
    if (prism_view.failed_layers())
      list->AddText(top_left,IM_COL32(255,150,80,255),"Some stock layers could not be triangulated; details in terminal.");
  } catch (const std::exception& error) {
    list->AddText(top_left,IM_COL32(255,90,90,255),error.what());
    std::fprintf(stderr,"Prism renderer: %s\n",error.what());
  }
  if (cutter_cursor && active_tool != nullptr) {
    append_cutter(
        facets, pool,
        Vec3{static_cast<float>(wave::to_world(cutter_cursor->position.x)),
             static_cast<float>(wave::to_world(cutter_cursor->position.y)),
             static_cast<float>(cutter_cursor->z)},
        static_cast<float>(active_tool->diameter * 0.5),
        static_cast<float>(cutter_cursor->z),
        static_cast<float>(cutter_cursor->z + active_tool->flute_length),
        projection, light);
  }
  paint_facets(list, facets, pool);

  // And the models over the finished stack, sorted among themselves alone.
  if (app.stack_models != StackModels::None) {
    std::vector<Facet> model_facets;
    std::vector<ImVec2> model_pool;
    append_models(app, app.stack_models, model_facets, model_pool, projection,
                  light, spacing,
                  app.job.operations.empty()
                      ? 32
                      : app.job.operations[std::clamp(
                                app.selected_operation, 0,
                                static_cast<int>(app.job.operations.size()) - 1)]
                            .slice_grid);
    paint_facets(list, model_facets, model_pool);
  }
  if (active.explicit_motion) {
    const auto project = [&](const Point64& xy, double z) {
      return projection.point({static_cast<float>(wave::to_world(xy.x)),
                               static_cast<float>(wave::to_world(xy.y)),
                               static_cast<float>(z)});
    };
    // Dimension-driven jobs may have no mesh. Keep their requested boundaries
    // visible even after the last material has been removed from an area.
    for (const auto& level : levels) {
      if (level.floor_z != active.floor_z) continue;
      for (const auto& ring : level.stock_slice)
        for (std::size_t i = 0; i < ring.size(); ++i)
          list->AddLine(project(ring[i], level.floor_z),
                        project(ring[(i+1)%ring.size()], level.floor_z),
                        IM_COL32(200, 175, 90, 180), 1.2f);
    }
    if (app.show_toolpath)
      for (const auto& phase : active.feeds.motion) {
        if (phase.rapid && !app.show_rapids) continue;
        list->AddLine(project(phase.from, phase.from_z), project(phase.to, phase.to_z),
                      move_colour(phase.kind), phase.kind == wave::MoveKind::Entry ? 1.8f : 1.0f);
      }
  }
  // This view builds its own projection and canvas rather than going through
  // begin_scene, so the gizmo and the corner control are handed one made up on
  // the spot. Nothing else is ever read out of it.
  {
    OrbitScene overlay;
    overlay.list = list;
    overlay.projection = projection;
    overlay.top_left = top_left;
    overlay.size = size;
    draw_origin_gizmo(app, overlay);
    if (scene_button(overlay, stack_models_name(app.stack_models), 10.0f))
      app.stack_models = app.stack_models == StackModels::None ? StackModels::Part
                         : app.stack_models == StackModels::Part
                             ? StackModels::Stock
                         : app.stack_models == StackModels::Stock
                             ? StackModels::Both
                             : StackModels::None;
  }

  char caption[200];
  if (planned_operation.subdivisions.size() > 1) {
    std::snprintf(
        caption, sizeof caption,
        "%zu layers from %zu subdivisions, Z %.2f to %.2f mm   Drag: orbit   "
        "Right-drag: pan   Wheel: zoom",
        levels.size(), planned_operation.subdivisions.size(), material_lowest,
        material_highest);
  } else {
    std::snprintf(
        caption, sizeof caption,
        "%zu passes stacked, Z %.2f to %.2f mm   Drag: orbit   Right-drag: pan   "
        "Wheel: zoom",
        levels.size(), material_lowest, material_highest);
  }
  list->AddText({top_left.x + 12.0f, top_left.y + 10.0f}, IM_COL32(130, 142, 158, 255),
                caption);
  if (untriangulated > 0) {
    std::snprintf(caption, sizeof caption,
                  "%zu layer%s could not be triangulated and is not drawn",
                  untriangulated, untriangulated == 1 ? "" : "s");
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

void invalidate_plan(App& app);

// Publish a library change only once it has been written successfully.
void commit_tools(App& app, cam::ToolLibrary library) {
  cam::save_tools(app.tools_file, library);
  app.tools = std::move(library);
  invalidate_plan(app);
}

void reload_tools(App& app) {
  if (!std::filesystem::exists(app.tools_file))
    throw std::runtime_error("Cannot open " + app.tools_file.string());
  auto library = cam::load_tools(app.tools_file);
  app.tools = std::move(library);
  invalidate_plan(app);
  say(app, "Reloaded " + app.tools_file.string());
}

void import_tools(App& app) {
  if (!std::filesystem::exists(app.tools_import_file))
    throw std::runtime_error("Cannot open " + app.tools_import_file);
  const auto imported = cam::load_tools(app.tools_import_file);
  auto library = app.tools;
  for (const auto& tool : imported.tools) library.upsert(tool);
  commit_tools(app, std::move(library));
  say(app, "Imported " + std::to_string(imported.tools.size()) + " tools into " +
               app.tools_file.string());
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
  ImGui::InputDouble("Diameter (mm)", &tool.diameter, 0, 0, "%.15g");
  ImGui::InputDouble("Flute length (mm)", &tool.flute_length, 0, 0, "%.15g");
  ImGui::InputInt("Number of flutes", &tool.flutes);
  static constexpr cam::ToolType types[]{cam::ToolType::FlatEndMill,
      cam::ToolType::BallEndMill, cam::ToolType::Drill, cam::ToolType::VBit};
  enum_combo("Type", tool.type, cam::tool_type_name, types, std::size(types));

  ImGui::Separator();
  cam::Tool candidate = tool;
  if (app.builder_is_new || candidate.id.empty())
    candidate.id = app.tools.unique_id_for(candidate.name);
  ImGui::TextDisabled("ID: %s", candidate.id.c_str());
  const auto problems = cam::validate(candidate);
  for (const auto& problem : problems)
    ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.4f, 1.0f),
                       "%s", problem.c_str());
  ImGui::BeginDisabled(!problems.empty() || app.planner.running());
  if (ImGui::Button("Save tool")) {
    try {
      auto library = app.tools;
      library.upsert(candidate);
      commit_tools(app, std::move(library));
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

// Defined below with the origin panel, which is where the box it places the
// origin against is worked out.
void apply_origin_pick(App& app);

void depth_defaults(App& app, cam::Operation& operation) {
  double low = 0.0;
  double high = 0.0;
  if (!app.geometry.work_range(low, high)) return;
  // The whole of the work, top to bottom. Depths measured from its top, so the
  // near end is zero whatever the models' own coordinates say.
  operation.depth_from = 0.0;
  operation.depth_to = high - low;
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
    } else {
      // No model named, so a box around the part is the only blank on offer.
      // Any model loaded earlier goes, or it would go on standing in for one
      // the job no longer names.
      app.geometry.clear_stock();
      app.job.stock_mesh.clear();
      app.job.box_stock = true;
    }
    app.geometry.set_box_stock(app.job.box_stock, app.job.box_stock_margin);
    for (auto& operation : app.job.operations) depth_defaults(app, operation);
    // The blank has changed, so the picked corner of it is somewhere new.
    apply_origin_pick(app);
    double low = 0.0;
    double high = 0.0;
    if (app.geometry.work_range(low, high))
      say(app, "Loaded. Z runs " + std::to_string(low) + " to " + std::to_string(high));
    app.has_plan = false;
  } catch (const std::exception& error) {
    say(app, error.what(), true);
  }
}

void load_job_into(App& app, const std::filesystem::path& file) {
  cam::Job job = cam::load_job(file);
  cam::JobGeometry geometry;
  const auto resolve = [&](std::filesystem::path& path) {
    if (!path.empty() && path.is_relative())
      path = std::filesystem::absolute(file).parent_path() / path;
  };
  resolve(job.part_mesh);
  resolve(job.stock_mesh);
  if (!job.part_mesh.empty()) geometry.load_part(job.part_mesh);
  if (!job.stock_mesh.empty()) geometry.load_stock(job.stock_mesh);
  geometry.set_box_stock(job.box_stock, job.box_stock_margin);
  app.geometry = std::move(geometry);
  app.job = std::move(job);
  app.job_file = file.string();
  std::snprintf(app.part_path.data(), app.part_path.size(), "%s", app.job.part_mesh.string().c_str());
  std::snprintf(app.stock_path.data(), app.stock_path.size(), "%s", app.job.stock_mesh.string().c_str());
  app.selected_operation = 0;
  invalidate_plan(app);
  say(app, "Loaded " + file.string());
}

// The box the program origin is placed against: the blank, because the blank is
// what sits on the machine and what the operator touches off. A rectangular
// blank has no mesh behind it, so its footprint comes from the outline it is
// defined by, and its Z from the part it was built around.
struct JobBounds {
  double low[3]{};
  double high[3]{};
  bool valid = false;
  const char* what = "stock";

  double size(int axis) const { return high[axis] - low[axis]; }
  // Where a low/middle/high pick on one axis puts the origin, in model
  // coordinates.
  double pick(int axis, int choice) const {
    if (choice == 0) return low[axis];
    if (choice == 2) return high[axis];
    return (low[axis] + high[axis]) * 0.5;
  }
};

JobBounds job_bounds(const App& app) {
  JobBounds result;
  const auto take = [&](const wave::Bounds3f& box, const char* what) {
    if (!box.valid) return false;
    result.low[0] = box.lower.x;
    result.low[1] = box.lower.y;
    result.low[2] = box.lower.z;
    result.high[0] = box.upper.x;
    result.high[1] = box.upper.y;
    result.high[2] = box.upper.z;
    result.what = what;
    result.valid = true;
    return true;
  };
  if (app.geometry.box_stock()) {
    // The blank stood in for: the part's own Z, and a footprint this asks the
    // geometry for rather than recomputing, so the box drawn and the box the
    // origin is placed against cannot drift apart.
    double low = 0.0;
    double high = 0.0;
    if (!app.geometry.has_part() || !app.geometry.work_range(low, high))
      return result;
    const wave::Paths64& outline = app.geometry.stock_outline(low, 32);
    if (outline.empty()) return result;
    const Clipper2Lib::Rect64 box = Clipper2Lib::GetBounds(outline);
    result.low[0] = wave::to_world(box.left);
    result.low[1] = wave::to_world(box.top);
    result.low[2] = low;
    result.high[0] = wave::to_world(box.right);
    result.high[1] = wave::to_world(box.bottom);
    result.high[2] = high;
    result.what = "box stock";
    result.valid = true;
    return result;
  }
  if (app.geometry.has_stock() &&
      take(app.geometry.stock().mesh().bounds, "stock"))
    return result;
  if (app.geometry.has_part()) take(app.geometry.part().mesh().bounds, "part");
  return result;
}

// Put the origin where the picks say, on the blank as it stands now. Called
// when a model loads as well as from the button, so the origin the gizmo shows
// is the one the picks describe from the outset rather than wherever the mesh
// happens to carry its own zero.
void apply_origin_pick(App& app) {
  const JobBounds bounds = job_bounds(app);
  if (!bounds.valid) return;
  app.job.origin_x = -bounds.pick(0, app.origin_pick[0]);
  app.job.origin_y = -bounds.pick(1, app.origin_pick[1]);
  app.job.origin_z = -bounds.pick(2, app.origin_pick[2]);
}

// Placing the program origin. Everything here is in the model's own
// coordinates, measured from the bottom-back-left corner of the blank, because
// that is the corner an operator can find with an edge finder. What the job
// carries is the other half of the same statement -- the offset added to every
// posted coordinate, which is the negative of the point picked here -- and none
// of the planning ever sees this side of it.
void origin_panel(App& app) {
  ImGui::SeparatorText("Origin");
  const JobBounds bounds = job_bounds(app);
  if (!bounds.valid) {
    ImGui::TextDisabled("Load a model to place the program origin");
    return;
  }

  static const char* const kLabels[3][3] = {
      {"Left", "Centre *", "Right"},
      {"Back", "Centre *", "Front"},
      {"Bottom", "Centre", "Top *"},
  };
  static const char* const kAxes[3] = {"-/+X", "-/+Y", "-/+Z"};

  if (ImGui::BeginTable("origin_pick", 4, ImGuiTableFlags_SizingStretchSame)) {
    ImGui::TableSetupColumn("l", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("m", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("h", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("a", ImGuiTableColumnFlags_WidthFixed);
    for (int axis = 0; axis < 3; ++axis) {
      ImGui::TableNextRow();
      for (int choice = 0; choice < 3; ++choice) {
        ImGui::TableSetColumnIndex(choice);
        const bool picked = app.origin_pick[axis] == choice;
        if (picked)
          ImGui::PushStyleColor(ImGuiCol_Button,
                                ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        ImGui::PushID(axis * 3 + choice);
        if (ImGui::Button(kLabels[axis][choice], ImVec2(-FLT_MIN, 0.0f)))
          app.origin_pick[axis] = choice;
        ImGui::PopID();
        if (picked) ImGui::PopStyleColor();
      }
      ImGui::TableSetColumnIndex(3);
      ImGui::AlignTextToFramePadding();
      ImGui::TextDisabled("%s", kAxes[axis]);
    }
    ImGui::EndTable();
  }
  ImGui::TextDisabled("* the usual choice");

  if (ImGui::Button("Apply")) apply_origin_pick(app);
  ImGui::SetItemTooltip(
      "Move the origin to the picked place on the blank. No re-plan: the "
      "origin is only where the posted numbers are measured from, and the plan "
      "is the same motion whichever corner that is.");

  // The three figures are the origin itself, stated where it can be checked
  // against the work rather than in the sign the post wants it in.
  double offset[3] = {
      -app.job.origin_x - bounds.low[0],
      -app.job.origin_y - bounds.low[1],
      -app.job.origin_z - bounds.low[2],
  };
  static const char* const kFields[3] = {"X", "Y", "Z"};
  bool edited = false;
  if (ImGui::BeginTable("origin_value", 3, ImGuiTableFlags_SizingStretchSame)) {
    for (int axis = 0; axis < 3; ++axis) {
      ImGui::TableNextColumn();
      ImGui::PushID(axis);
      // Labelled on the left. ImGui puts a widget's label on its right, which
      // a full-width field in a table cell pushes off the end of the cell.
      ImGui::AlignTextToFramePadding();
      ImGui::TextUnformatted(kFields[axis]);
      ImGui::SameLine();
      ImGui::SetNextItemWidth(-FLT_MIN);
      if (ImGui::InputDouble("##value", &offset[axis], 0.0, 0.0, "%.3f"))
        edited = true;
      ImGui::PopID();
    }
    ImGui::EndTable();
  }
  if (edited) {
    app.job.origin_x = -(offset[0] + bounds.low[0]);
    app.job.origin_y = -(offset[1] + bounds.low[1]);
    app.job.origin_z = -(offset[2] + bounds.low[2]);
  }
  ImGui::TextDisabled("From the bottom-back-left corner of the %s",
                      bounds.what);
  ImGui::TextDisabled("Bounds %.1f x %.1f x %.1f mm", bounds.size(0),
                      bounds.size(1), bounds.size(2));
}

void job_panel(App& app) {
  ImGui::SeparatorText("Job");
  text_field("Name", app.job.name);
  text_field("Job JSON", app.job_file, 1024);
  ImGui::BeginDisabled(app.planner.running());
  if (ImGui::Button("Save job")) {
    try {
      cam::Job saved = app.job;
      if (!saved.part_mesh.empty()) saved.part_mesh = std::filesystem::absolute(saved.part_mesh);
      if (!saved.stock_mesh.empty()) saved.stock_mesh = std::filesystem::absolute(saved.stock_mesh);
      cam::save_job(app.job_file, saved);
      say(app, "Saved " + app.job_file);
    } catch (const std::exception& error) { say(app, error.what(), true); }
  }
  ImGui::SameLine();
  if (ImGui::Button("Load job")) {
    try { load_job_into(app, app.job_file); }
    catch (const std::exception& error) { say(app, error.what(), true); }
  }
  ImGui::EndDisabled();
  ImGui::InputText("Part model", app.part_path.data(), app.part_path.size());
  ImGui::InputText("Stock model", app.stock_path.data(), app.stock_path.size());
  // A named model always wins, so the fallback is only offered without one.
  ImGui::BeginDisabled(app.stock_path[0] != '\0');
  if (ImGui::Checkbox("Box stock around the part", &app.job.box_stock))
    app.geometry.set_box_stock(app.job.box_stock, app.job.box_stock_margin);
  ImGui::SetItemTooltip(
      "Stand a rectangular blank in for a stock model: the part's bounding box "
      "grown in X and Y. Z stays at the part's own range.");
  if (app.job.box_stock) {
    if (ImGui::DragScalar("Box margin (mm)", ImGuiDataType_Double,
                          &app.job.box_stock_margin, 0.1f))
      app.geometry.set_box_stock(app.job.box_stock,
                                 std::max(app.job.box_stock_margin, 0.0));
  }
  ImGui::EndDisabled();
  if (ImGui::Button("Load models")) load_meshes(app);
  ImGui::SameLine();
  if (ImGui::Button("Fit view")) {
    app.view.needs_fit = true;
    // The 3D views frame themselves, so there is nothing to fit there -- but a
    // view panned off into the corner is exactly as lost as one badly zoomed,
    // and this is the button to reach for.
    app.orbit.pan = ImVec2(0.0f, 0.0f);
  }

  origin_panel(app);

  ImGui::DragScalar("Rapid plane (above highest)", ImGuiDataType_Double,
                    &app.job.rapid_clearance_height, 0.1f);
  ImGui::SetItemTooltip(
      "How far the safe plane clears the tallest thing in the way, part or "
      "blank, not the distance of each retract: a retract travels from its "
      "pass depth to that one plane. Measured over the work rather than up "
      "from the origin, so moving the origin cannot move it. Re-plan after "
      "changing it -- it is what every lift is costed against.");
  text_field("Work offset", app.job.work_offset, 16);
}

cam::Operation make_operation(App& app, cam::OperationMode mode) {
  cam::Operation operation;
  operation.mode = mode;
  switch (mode) {
    case cam::OperationMode::Roughing: operation.name = "Rough"; break;
    case cam::OperationMode::ContourFinishing: operation.name = "Contour finish"; break;
    case cam::OperationMode::Pocket: operation.name = "Pocket"; break;
    case cam::OperationMode::PlaneClearing: operation.name = "Plane clearing"; break;
  }
  if (!app.job.operations.empty()) {
    const cam::Operation& selected = app.job.operations[std::clamp(
        app.selected_operation, 0,
        static_cast<int>(app.job.operations.size()) - 1)];
    operation.tool_id = selected.tool_id;
    operation.machine_id = selected.machine_id;
    operation.material_id = selected.material_id;
    operation.spindle_rpm = selected.spindle_rpm;
    operation.plunge_feed = selected.plunge_feed;
    operation.coolant = selected.coolant;
    operation.manual_feed_range = selected.manual_feed_range;
    operation.full_engagement_feed = selected.full_engagement_feed;
    operation.no_engagement_feed = selected.no_engagement_feed;
    operation.depth_from = selected.depth_from;
    operation.depth_to = selected.depth_to;
  } else {
    if (!app.tools.tools.empty()) operation.tool_id = app.tools.tools.front().id;
    if (!app.machines.machines.empty())
      operation.machine_id = app.machines.machines.front().id;
    if (!app.materials.materials.empty())
      operation.material_id = app.materials.materials.front().id;
    depth_defaults(app, operation);
  }
  if (mode == cam::OperationMode::ContourFinishing) {
    operation.radial_stock_to_leave = 0.0;
    operation.rough_boundary_cleanup = false;
  }

  const std::string base = operation.name;
  int suffix = 2;
  const auto name_exists = [&](const std::string& name) {
    return std::any_of(app.job.operations.begin(), app.job.operations.end(),
                       [&](const cam::Operation& candidate) {
                         return candidate.name == name;
                       });
  };
  while (name_exists(operation.name))
    operation.name = base + " " + std::to_string(suffix++);
  return operation;
}

void select_operation(App& app, int index) {
  if (app.job.operations.empty()) {
    app.selected_operation = 0;
    return;
  }
  index = std::clamp(index, 0,
                     static_cast<int>(app.job.operations.size()) - 1);
  app.selected_operation = index;
  if (!app.has_plan || app.plan.operations.empty()) return;

  app.selected_plan_operation = std::clamp(
      index, 0, static_cast<int>(app.plan.operations.size()) - 1);
  const auto& levels = app.plan.operations[app.selected_plan_operation].levels;
  app.selected_level = 0;
  while (app.selected_level < static_cast<int>(levels.size()) &&
         levels[app.selected_level].nothing_to_cut)
    ++app.selected_level;
  if (app.selected_level >= static_cast<int>(levels.size()))
    app.selected_level = 0;
  app.playback.playing = false;
  app.playback.distance = 0.0;
  app.playback.motion_seconds = 0.0;
  app.material.clear();
  app.material_distance = -1.0;
  app.material_level = -1;
  app.material_operation = -1;
  app.stock_cache_operation = -1;
  app.view.needs_fit = true;
}

void invalidate_plan(App& app) {
  app.has_plan = false;
  app.playback.playing = false;
  app.gcode.clear();
  app.post = {};
  app.material.clear();
  app.material_distance = -1.0;
  app.material_level = -1;
  app.material_operation = -1;
  app.stock_cache_operation = -1;
}

void feeds_panel(App& app);

void operations_list_panel(App& app) {
  ImGui::SeparatorText("Ordered operations");
  if (app.job.operations.empty()) {
    app.job.operations.push_back(
        make_operation(app, cam::OperationMode::Roughing));
  }

  app.selected_operation =
      std::clamp(app.selected_operation, 0,
                 static_cast<int>(app.job.operations.size()) - 1);
  if (ImGui::BeginListBox("##operation-order", ImVec2(-FLT_MIN, 190.0f))) {
    for (std::size_t i = 0; i < app.job.operations.size(); ++i) {
      const cam::Operation& operation = app.job.operations[i];
      const std::string label = std::to_string(i + 1) + ". " + operation.name +
                                "  [" + cam::operation_mode_name(operation.mode) +
                                "]##" + std::to_string(i);
      if (ImGui::Selectable(label.c_str(),
                            static_cast<int>(i) == app.selected_operation))
        select_operation(app, static_cast<int>(i));
    }
    ImGui::EndListBox();
  }

  if (ImGui::Button("Add roughing")) {
    invalidate_plan(app);
    app.job.operations.push_back(
        make_operation(app, cam::OperationMode::Roughing));
    select_operation(app, static_cast<int>(app.job.operations.size()) - 1);
  }
  ImGui::SameLine();
  if (ImGui::Button("Add contour finish")) {
    invalidate_plan(app);
    app.job.operations.push_back(
        make_operation(app, cam::OperationMode::ContourFinishing));
    select_operation(app, static_cast<int>(app.job.operations.size()) - 1);
  }
  if (ImGui::Button("Add pocket")) {
    invalidate_plan(app);
    app.job.operations.push_back(make_operation(app, cam::OperationMode::Pocket));
    select_operation(app, static_cast<int>(app.job.operations.size()) - 1);
  }
  ImGui::SameLine();
  if (ImGui::Button("Add plane clearing")) {
    invalidate_plan(app);
    app.job.operations.push_back(make_operation(app, cam::OperationMode::PlaneClearing));
    select_operation(app, static_cast<int>(app.job.operations.size()) - 1);
  }
  if (ImGui::Button("Duplicate")) {
    invalidate_plan(app);
    cam::Operation copy = app.job.operations[app.selected_operation];
    copy.name += " copy";
    app.job.operations.insert(app.job.operations.begin() + app.selected_operation + 1,
                              std::move(copy));
    select_operation(app, app.selected_operation + 1);
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(app.selected_operation == 0);
  if (ImGui::Button("Move up")) {
    invalidate_plan(app);
    std::swap(app.job.operations[app.selected_operation],
              app.job.operations[app.selected_operation - 1]);
    select_operation(app, app.selected_operation - 1);
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(
      app.selected_operation + 1 >= static_cast<int>(app.job.operations.size()));
  if (ImGui::Button("Move down")) {
    invalidate_plan(app);
    std::swap(app.job.operations[app.selected_operation],
              app.job.operations[app.selected_operation + 1]);
    select_operation(app, app.selected_operation + 1);
  }
  ImGui::EndDisabled();

  ImGui::BeginDisabled(app.job.operations.size() < 2);
  if (ImGui::Button("Remove")) {
    invalidate_plan(app);
    app.job.operations.erase(app.job.operations.begin() + app.selected_operation);
    const int next = std::min(app.selected_operation,
                              static_cast<int>(app.job.operations.size()) - 1);
    app.selected_operation = -1;
    select_operation(app, next);
  }
  ImGui::EndDisabled();
  ImGui::TextWrapped(
      "Operations run from top to bottom. Simulation stock is handed to each "
      "operation from every operation above it.");
}

void operation_settings_panel(App& app) {
  ImGui::SeparatorText("Selected operation");
  if (app.job.operations.empty()) {
    ImGui::TextDisabled("Add an operation first");
    return;
  }

  cam::Operation& operation = app.job.operations[app.selected_operation];
  text_field("Operation name", operation.name);
  static constexpr cam::OperationMode kModes[]{
      cam::OperationMode::Roughing,
      cam::OperationMode::ContourFinishing,
      cam::OperationMode::Pocket, cam::OperationMode::PlaneClearing,
  };
  enum_combo("Mode", operation.mode, cam::operation_mode_name, kModes,
             std::size(kModes));
  if (operation.mode == cam::OperationMode::ContourFinishing) {
    static constexpr cam::ContourDirectionPolicy kDirections[]{
        cam::ContourDirectionPolicy::Either,
        cam::ContourDirectionPolicy::Climb,
        cam::ContourDirectionPolicy::Conventional,
    };
    enum_combo("Contour direction", operation.contour_direction,
               cam::contour_direction_name, kDirections,
               std::size(kDirections));
    ImGui::Checkbox("Clip contours to nominal stock",
                    &operation.clip_contours_to_stock);
  } else if (operation.mode == cam::OperationMode::Roughing) {
    ImGui::Checkbox("Boundary cleanup contour",
                    &operation.rough_boundary_cleanup);
  }

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
  if (machine && machine->specifications_unverified)
    ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.35f, 1.0f),
                       "This machine profile carries unverified limits");

  ImGui::InputDouble("Stepover (of diameter)", &operation.stepover_ratio, 0, 0, "%.6g");
  ImGui::InputDouble("Stepdown (mm)", &operation.stepdown, 0, 0, "%.6g");
  if (cam::is_parametric(operation.mode)) {
    if (operation.mode == cam::OperationMode::Pocket) {
      auto& p = operation.pocket;
      static constexpr cam::PocketShape shapes[]{cam::PocketShape::Circle,
          cam::PocketShape::Rectangle, cam::PocketShape::RoundedRectangle};
      enum_combo("Pocket shape", p.shape, cam::pocket_shape_name, shapes, std::size(shapes));
      ImGui::InputDouble("Center X (mm)", &p.center_x, 0, 0, "%.6g");
      ImGui::InputDouble("Center Y (mm)", &p.center_y, 0, 0, "%.6g");
      ImGui::InputDouble("Top Z (mm)", &p.top_z, 0, 0, "%.6g");
      if (p.shape != cam::PocketShape::Circle) {
        ImGui::InputDouble("Width X (mm)", &p.width, 0, 0, "%.6g");
        ImGui::InputDouble("Height Y (mm)", &p.height, 0, 0, "%.6g");
      }
      if (p.shape != cam::PocketShape::Rectangle)
        ImGui::InputDouble(p.shape == cam::PocketShape::Circle ? "Radius (mm)" :
            "Corner radius (mm)", &p.radius, 0, 0, "%.6g");
      ImGui::InputDouble("Pocket depth (mm)", &p.depth, 0, 0, "%.6g");
      ImGui::InputDouble("Helix angle (degrees)", &p.helix_angle_degrees, 0, 0, "%.6g");
      ImGui::TextWrapped("Floor Z: %.6g mm. Every level starts with a helical entry.", p.top_z-p.depth);
      if (p.shape == cam::PocketShape::Rectangle)
        ImGui::TextWrapped("Rectangle corners retain the tool radius.");
    } else {
      auto& p = operation.plane;
      ImGui::InputDouble("Top Z (mm)", &p.top_z, 0, 0, "%.6g");
      ImGui::InputDouble("Clearing depth (mm)", &p.depth, 0, 0, "%.6g");
      ImGui::TextWrapped("Floor Z: %.6g mm. Rectangles share these depths. "
                         "The cutter stays inside each area, leaving rounded corners.", p.top_z-p.depth);
      int remove = -1;
      for (std::size_t i = 0; i < p.areas.size(); ++i) {
        ImGui::PushID(static_cast<int>(i));
        ImGui::SeparatorText(("Area " + std::to_string(i+1)).c_str());
        auto& area = p.areas[i];
        ImGui::InputDouble("Center X (mm)", &area.center_x, 0, 0, "%.6g");
        ImGui::InputDouble("Center Y (mm)", &area.center_y, 0, 0, "%.6g");
        ImGui::InputDouble("Width X (mm)", &area.width, 0, 0, "%.6g");
        ImGui::InputDouble("Height Y (mm)", &area.height, 0, 0, "%.6g");
        if (ImGui::Button("Remove area")) remove = static_cast<int>(i);
        ImGui::PopID();
      }
      if (remove >= 0) p.areas.erase(p.areas.begin()+remove);
      if (ImGui::Button("Add area")) p.areas.push_back(cam::ClearingArea{});
    }
    ImGui::TextWrapped("Coordinates are in the job frame; the program origin offset "
                       "is applied when posting. No part mesh is required.");
    ImGui::InputDouble("Simulation/feed pitch (mm)", &operation.path_pitch, 0, 0, "%.6g");
    feeds_panel(app);
    return;
  }
  const std::size_t base_layers =
      cam::level_floors(operation, operation.stepdown,
                        cam::highest_model_z(app.geometry))
          .size();
  if (operation.mode == cam::OperationMode::Roughing) {
    if (ImGui::DragInt("Finishing subdivisions",
                       &operation.finishing_subdivisions, 0.05f, 1, 64))
      operation.finishing_subdivisions =
          std::clamp(operation.finishing_subdivisions, 1, 64);
    ImGui::SetItemTooltip(
        "Runs this roughing operation repeatedly at evenly interleaved depth "
        "offsets. Each pass receives the stock left by the preceding pass.");
    const double minimum_cleanup_tolerance = 0.0;
    const double maximum_cleanup_tolerance =
        tool ? tool->diameter * 0.5 : 10.0;
    ImGui::BeginDisabled(!operation.rough_boundary_cleanup);
    if (ImGui::SliderScalar(
            "Cleanup tolerance (mm)", ImGuiDataType_Double,
            &operation.cleanup_tolerance, &minimum_cleanup_tolerance,
            &maximum_cleanup_tolerance, "%.3f"))
      operation.cleanup_tolerance = std::clamp(
          operation.cleanup_tolerance, minimum_cleanup_tolerance,
          maximum_cleanup_tolerance);
    ImGui::SetItemTooltip(
        "Numerical allowance excluded from the cutter radius when deciding "
        "whether a finishing-subdivision cleanup contour is close enough to "
        "remaining stock. The effective distance is cutter radius minus this "
        "value; setting it to the cutter radius keeps the strict wave-only "
        "rule.");
    ImGui::EndDisabled();
  }
  if (tool) {
    const std::string preview_layers =
        operation.mode == cam::OperationMode::Roughing &&
                operation.finishing_subdivisions > 1
            ? ", " +
                  std::to_string(base_layers *
                                 operation.finishing_subdivisions) +
                  " preview layers"
            : "";
    ImGui::TextWrapped(
        "= %.3f mm wide, %zu base layers%s",
        operation.stepover_ratio * tool->diameter, base_layers,
        preview_layers.c_str());
  }
  ImGui::DragScalar("Radial stock to leave", ImGuiDataType_Double,
                    &operation.radial_stock_to_leave, 0.02f);
  ImGui::DragScalar("Axial stock to leave", ImGuiDataType_Double,
                    &operation.axial_stock_to_leave, 0.02f);
  ImGui::DragScalar("Depth from (mm)", ImGuiDataType_Double,
                    &operation.depth_from, 0.1f);
  ImGui::DragScalar("Depth to (mm)", ImGuiDataType_Double, &operation.depth_to,
                    0.1f);
  ImGui::SetItemTooltip(
      "Measured down from the top of the work -- the taller of the part and the "
      "blank -- so a blank standing proud of the part is faced off rather than "
      "left standing. Where the origin sits does not enter into it.");
  if (operation.mode == cam::OperationMode::Roughing) {
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
      const double first = std::max(
          bite - std::clamp(operation.stock_deviation, 0.0, bite * 0.9), 0.0);
      ImGui::TextDisabled("= target first bite %.3f mm, then %.3f mm", first,
                          bite);
    }
    ImGui::EndDisabled();
  }

  ImGui::DragScalar("Simulation/feed pitch (mm)", ImGuiDataType_Double,
                    &operation.path_pitch, 0.01f);
  ImGui::SetItemTooltip(
      "The complete at-depth path is resampled at this travelled-distance "
      "interval before simulation and feed assignment.");
  if (operation.mode == cam::OperationMode::Roughing) {
    ImGui::DragScalar("Entry-front Z match (mm)", ImGuiDataType_Double,
                      &operation.entry_front_match_tolerance, 0.01f);
    ImGui::SetItemTooltip(
        "The complete lower-layer entry curve must fit in this envelope around "
        "the upper curve. Node-area Z matching remains exact.");
    ImGui::DragScalar("Tiny split area (stepover^2)", ImGuiDataType_Double,
                      &operation.tiny_split_area_factor, 0.01f);
    ImGui::SetItemTooltip(
        "A terminal split branch smaller than this multiple of stepover squared "
        "stays in the wave but does not create a separate node. Zero disables "
        "the filter.");
  }

  feeds_panel(app);
}

void feeds_panel(App& app) {
  if (app.job.operations.empty()) return;
  cam::Operation& operation = app.job.operations[app.selected_operation];
  const cam::Machine* machine = app.machines.find(operation.machine_id);
  const cam::MaterialFeedRange* library =
      machine ? machine->feeds_for(operation.material_id) : nullptr;
  ImGui::SeparatorText("Operation cutting settings");
  ImGui::InputDouble("Spindle (rpm)", &operation.spindle_rpm, 0, 0, "%.6g");
  ImGui::InputDouble("Plunge / ramp feed (mm/min)", &operation.plunge_feed, 0, 0, "%.6g");
  ImGui::Checkbox("Coolant", &operation.coolant);
  if (ImGui::CollapsingHeader("Machine tool assignment")) {
    ImGui::InputInt("Tool number (0 = auto)", &operation.tool_number);
    ImGui::InputInt("H offset (0 = tool)", &operation.length_offset);
  }
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

void start_plan(App& app) {
  app.gcode.clear();
  app.post = {};
  cam::JobGeometry& geometry = app.geometry;
  cam::Job job = app.job;
  cam::ToolLibrary tools = app.tools;
  cam::MachineLibrary machines = app.machines;
  cam::MaterialLibrary materials = app.materials;
  app.planner.start([&geometry, job = std::move(job), tools = std::move(tools),
                     machines = std::move(machines),
                     materials = std::move(materials)](
                        const cam::ProgressCallback& progress) {
    return cam::plan_job(geometry, job, tools, machines, materials, progress);
  });
}

void plan_panel(App& app) {
  ImGui::SeparatorText("Plan");
  if (app.planner.running()) {
    ImGui::ProgressBar(static_cast<float>(app.planner.fraction()), ImVec2(-1, 0));
    ImGui::TextWrapped("%s", app.planner.stage().c_str());
    if (ImGui::Button("Cancel")) app.planner.cancel();
    return;
  }

  const bool ready = !app.job.operations.empty() && std::all_of(
      app.job.operations.begin(), app.job.operations.end(), [&](const cam::Operation& op) {
        return cam::is_parametric(op.mode) || (app.geometry.has_part() && app.geometry.has_stock());
      });
  ImGui::BeginDisabled(!ready);
  if (ImGui::Button("Plan job", ImVec2(-1, 0))) start_plan(app);
  ImGui::EndDisabled();
  if (!ready) ImGui::TextDisabled("Mesh operations require a part and stock; pocket/plane operations use their dimensions");

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

  show_warnings("plan-warnings", app.plan.warnings);

  const cam::LevelPlan* level = current_level(app);
  if (level)
    ImGui::Text("Spindle %.0f rpm", level->feeds.stats.spindle_rpm);
}

void output_panel(App& app) {
  if (!app.has_plan) return;
  ImGui::SeparatorText("Output");
  if (ImGui::Button("Generate G-code", ImVec2(-1, 0))) {
    app.post = cam::post_job(app.plan, app.job, app.tools, app.machines);
    log_warnings("G-code", app.post.warnings);
    app.gcode = app.post.text;
    if (!app.post.ok()) say(app, app.post.error, true);
    else say(app, "Generated " + std::to_string(app.post.line_count) + " lines");
  }
  if (app.gcode.empty()) return;

  ImGui::Text("%zu lines, %zu cutting moves, %zu rapids", app.post.line_count,
              app.post.cutting_moves, app.post.rapid_moves);
  ImGui::Text("Estimated %.1f minutes", app.post.estimated_minutes);
  show_warnings("post-warnings", app.post.warnings);

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
  ImGui::TextWrapped("Library file: %s", app.tools_file.string().c_str());
  if (ImGui::SmallButton("Copy library path"))
    ImGui::SetClipboardText(app.tools_file.string().c_str());
  ImGui::SameLine();
  ImGui::BeginDisabled(app.builder_open || app.planner.running());
  if (ImGui::SmallButton("Reload from disk")) {
    try {
      reload_tools(app);
    } catch (const std::exception& error) {
      say(app, error.what(), true);
    }
  }
  ImGui::TextWrapped("Starter tools are copied only on first run. To apply edits "
                     "to another JSON file, import it below. Matching IDs are "
                     "replaced; other tools are kept.");
  text_field("Import JSON", app.tools_import_file, 1024);
  if (ImGui::Button("Import tools")) {
    try {
      import_tools(app);
    } catch (const std::exception& error) {
      say(app, error.what(), true);
    }
  }
  ImGui::EndDisabled();
  ImGui::Separator();
  for (const auto& tool : app.tools.tools) {
    ImGui::PushID(tool.id.c_str());
    ImGui::Text("%s", tool.name.c_str());
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
    app.builder_is_new = true;
    app.builder_open = true;
  }
  ImGui::SameLine();
  ImGui::TextDisabled("%zu machines, %zu materials", app.machines.machines.size(),
                      app.materials.materials.size());
}

void view_panel(App& app) {
  ImGui::SeparatorText("Playback / Display");
  if (!app.has_plan || app.plan.operations.empty()) {
    ImGui::TextDisabled("Plan the job to enable playback and result display.");
    return;
  }
  app.selected_plan_operation = std::clamp(
      app.selected_plan_operation, 0,
      static_cast<int>(app.plan.operations.size()) - 1);
  ImGui::Text("Operation: %s",
              app.plan.operations[app.selected_plan_operation]
                  .operation_name.c_str());
  const auto& levels = app.plan.operations[app.selected_plan_operation].levels;
  if (!levels.empty()) {
    int last = static_cast<int>(levels.size()) - 1;
    app.selected_level = std::clamp(app.selected_level, 0, last);
    if (ImGui::SliderInt("Pass", &app.selected_level, 0, last)) {
      app.playback.distance = 0.0;
      app.playback.motion_seconds = 0.0;
    }
    ImGui::TextDisabled("Depth %.3f mm, cutting %.3f mm",
                        levels[app.selected_level].floor_z,
                        levels[app.selected_level].axial_depth);
    const cam::LevelPlan& selected = levels[app.selected_level];
    const auto& subdivisions =
        app.plan.operations[app.selected_plan_operation].subdivisions;
    if (subdivisions.size() > 1 &&
        selected.subdivision < subdivisions.size()) {
      ImGui::TextDisabled(
          "Subdivision %zu/%zu, layer %zu/%zu", selected.subdivision + 1,
          subdivisions.size(), selected.subdivision_level + 1,
          subdivisions[selected.subdivision].level_count);
    }
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
  if (ImGui::TreeNode("Debug view")) {
    const auto& topology =
        app.plan.operations[app.selected_plan_operation].inter_layer;
    if (!levels.empty()) {
      const cam::LevelPlan& selected = levels[app.selected_level];
      ImGui::TextDisabled(
          "%zu recursive wave layer%s, %zu/%zu trial%s accepted",
          selected.wave_stages.size(),
          selected.wave_stages.size() == 1 ? "" : "s",
          selected.accepted_analysis_trials, selected.analysis_trials,
          selected.analysis_trials == 1 ? "" : "s");
      ImGui::TextDisabled(
          "%zu current node%s, %zu current entr%s; overlays use the combined "
          "committed result.",
          selected.topology.nodes.size(),
          selected.topology.nodes.size() == 1 ? "" : "s",
          selected.lineage.entry_fronts.size(),
          selected.lineage.entry_fronts.size() == 1 ? "y" : "ies");
    }
    ImGui::TextDisabled("%zu node Z-group%s, %zu entry-front Z-group%s",
                        topology.node_chains.size(),
                        topology.node_chains.size() == 1 ? "" : "s",
                        topology.entry_front_chains.size(),
                        topology.entry_front_chains.size() == 1 ? "" : "s");
    static const char* kNodeColourings[]{"Off", "By node", "By Z-group"};
    int node_colouring = static_cast<int>(app.node_debug_colouring);
    if (ImGui::Combo("Node boundaries", &node_colouring, kNodeColourings,
                     std::size(kNodeColourings)))
      app.node_debug_colouring =
          static_cast<NodeDebugColouring>(node_colouring);
    ImGui::Checkbox("Entry-front Z-groups", &app.show_entry_front_chains);
    ImGui::Checkbox("Boundary contacts", &app.show_boundary_contacts);
    ImGui::TextDisabled("Closed node areas use the simulated-stock fill path.");
    ImGui::TreePop();
  }
  ImGui::Checkbox("Stock follows the cutter", &app.stack_follows_playback);
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
    app.playback.motion_seconds = 0.0;
    app.playback.playing = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("End")) {
    app.playback.distance = total;
    app.playback.motion_seconds =
        level ? cam::feed_schedule_seconds(level->feeds) : 0.0;
    app.playback.playing = false;
  }
  ImGui::SameLine();
  ImGui::Checkbox("Run on", &app.playback.follow);
  ImGui::SetItemTooltip("Fall into the next pass instead of looping this one.");

  float where = static_cast<float>(app.playback.distance);
  ImGui::SetNextItemWidth(-160.0f);
  if (ImGui::SliderFloat("Along pass", &where, 0.0f,
                         static_cast<float>(total), "%.1f mm")) {
    app.playback.distance = where;
    sync_playback_time(app);
  }

  if (ImGui::Checkbox("Machine time", &app.playback.real_time))
    sync_playback_time(app);
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
      const double elapsed = app.playback.motion_seconds;
      const double whole = cam::feed_schedule_seconds(level->feeds);
      ImGui::Text("%s of %s in this pass, %.1f min in the job",
                  clock_text(elapsed).c_str(), clock_text(whole).c_str(),
                  app.plan.stats.cutting_minutes + app.plan.stats.rapid_minutes);
      const cam::MotionCursor motion = cam::feed_motion_at_seconds(
          level->feeds, app.playback.motion_seconds);
      const double motion_total = cam::feed_motion_length(level->feeds);
      ImGui::Text("Machine XYZ travel %.1f of %.1f mm",
                  motion.motion_distance, motion_total);
      ImGui::Text("%s %.1f of %.1f mm at %.0f mm/min (%.2fx view)",
                  motion_phase_name(motion), motion.phase_distance,
                  motion.phase_length, motion.rate, app.playback.time_scale);
      ImGui::TextDisabled("Tool Z %.3f mm; rapid plane %.3f mm",
                          motion.z, app.plan.rapid_plane);
      const cam::FeedSegment* segment = nullptr;
      if (!level->feeds.segments.empty())
        segment = &level->feeds.segments[segment_at(level->feeds.segments,
                                                    app.playback.distance)];
      const bool positioning = motion.rapid || motion.from_z != motion.to_z;
      if (segment != nullptr && !positioning)
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

void ensure_initial_operation(App& app) {
  if (app.job.operations.empty())
    app.job.operations.push_back(
        make_operation(app, cam::OperationMode::Roughing));
}

void left_panel(App& app) {
  ensure_initial_operation(app);
  const float available = ImGui::GetContentRegionAvail().y;
  const float operations_height = std::clamp(available * 0.46f, 330.0f, 420.0f);
  const float program_height = std::max(available - operations_height - 6.0f,
                                        140.0f);

  ImGui::BeginChild("program-controls", ImVec2(0, program_height),
                    ImGuiChildFlags_Borders);
  if (!app.banner.empty()) {
    const ImVec4 colour = app.banner_is_error ? ImVec4(1.0f, 0.45f, 0.4f, 1.0f)
                                              : ImVec4(0.55f, 0.85f, 0.65f, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Text, colour);
    ImGui::TextWrapped("%s", app.banner.c_str());
    ImGui::PopStyleColor();
    ImGui::Separator();
  }
  if (ImGui::BeginTabBar("program-tabs")) {
    if (ImGui::BeginTabItem("Job / Origin")) {
      if (ImGui::BeginChild("job-origin-scroll", ImVec2(0, 0),
                            ImGuiChildFlags_None,
                            ImGuiWindowFlags_AlwaysVerticalScrollbar)) {
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.48f);
        job_panel(app);
        plan_panel(app);
        output_panel(app);
        ImGui::PopItemWidth();
      }
      ImGui::EndChild();
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Libraries")) {
      if (ImGui::BeginChild("libraries-scroll", ImVec2(0, 0),
                            ImGuiChildFlags_None,
                            ImGuiWindowFlags_AlwaysVerticalScrollbar))
        library_panel(app);
      ImGui::EndChild();
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }
  ImGui::EndChild();

  ImGui::BeginChild("operation-list", ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_AlwaysVerticalScrollbar);
  operations_list_panel(app);
  ImGui::EndChild();
}

void right_panel(App& app) {
  ensure_initial_operation(app);
  const float available = ImGui::GetContentRegionAvail().y;
  const float settings_height = std::max(available * 0.58f, 220.0f);

  ImGui::BeginChild("operation-settings", ImVec2(0, settings_height),
                    ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_AlwaysVerticalScrollbar);
  ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.43f);
  const std::string settings_before = cam::to_json_text(app.job);
  operation_settings_panel(app);
  if (app.has_plan && settings_before != cam::to_json_text(app.job))
    invalidate_plan(app);
  ImGui::PopItemWidth();
  ImGui::EndChild();

  ImGui::BeginChild("playback-display", ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_AlwaysVerticalScrollbar);
  view_panel(app);
  ImGui::EndChild();
}

// ---------------------------------------------------------------------------

void bootstrap(App& app) {
  const std::filesystem::path data = cam::data_directory();
  const std::filesystem::path config = cam::config_directory();
  app.tools_file = config / "tools.json";
  app.tools_import_file = (data / "starter-tools.json").string();

  try {
    app.machines = cam::load_machines(data / "machines");
    app.materials = cam::load_materials(data / "materials");
  } catch (const std::exception& error) {
    say(app, error.what(), true);
  }
  if (app.machines.machines.empty())
    say(app, "No machine profiles found under " + (data / "machines").string(), true);

  try {
    if (std::filesystem::exists(app.tools_file)) {
      app.tools = cam::load_tools(app.tools_file);
    } else {
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
  std::filesystem::path job_in;
  std::string view_name;
  double progress_fraction = -1.0;
  double stepdown = 0.0;
  int finishing_subdivisions = 0;
  double stepover = 0.0;
  double path_pitch = 0.0;
  double entry_front_tolerance = 0.0;
  std::optional<double> tiny_split_area_factor;
  double rapid_clearance = 0.0;
  double full_feed = 0.0;
  double no_feed = 0.0;
  bool profile_planning = false;
  std::string tool_id;
  std::string machine_id;
  std::string material_id;
  std::string mode_name;
  std::string direction_name;
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
      else if (argument == "--models") {
        const std::string which = value();
        app.models = which == "stock"  ? ModelShow::Stock
                     : which == "part" ? ModelShow::Part
                                       : ModelShow::Both;
      }
      else if (argument == "--box-stock") {
        app.job.box_stock = true;
        app.job.box_stock_margin = std::stod(value());
      }
      else if (argument == "--gcode") gcode_out = value();
      else if (argument == "--job") job_in = value();
      else if (argument == "--tool") tool_id = value();
      else if (argument == "--machine") machine_id = value();
      else if (argument == "--material") material_id = value();
      else if (argument == "--stepdown") stepdown = std::stod(value());
      else if (argument == "--finishing-subdivisions")
        finishing_subdivisions = std::stoi(value());
      else if (argument == "--stepover") stepover = std::stod(value());
      else if (argument == "--path-pitch") path_pitch = std::stod(value());
      else if (argument == "--entry-front-tolerance")
        entry_front_tolerance = std::stod(value());
      else if (argument == "--tiny-split-area-factor")
        tiny_split_area_factor = std::stod(value());
      else if (argument == "--rapid-clearance") rapid_clearance = std::stod(value());
      else if (argument == "--mode") mode_name = value();
      else if (argument == "--contour-direction") direction_name = value();
      else if (argument == "--full-feed") full_feed = std::stod(value());
      else if (argument == "--no-feed") no_feed = std::stod(value());
      else if (argument == "--profile-planning") profile_planning = true;
      else if (argument == "--help" || argument == "-h") {
        std::printf(
            "TingleCAM -- wavefront roughing and contour finishing\n\n"
            "  --part PATH        Part model (OBJ or STL)\n"
            "  --stock PATH       Stock model (OBJ or STL)\n"
            "  --job PATH         Load a job JSON, including pocket/plane dimensions\n"
            "  --tool ID          Tool from the library\n"
            "  --machine ID       Machine profile\n"
            "  --material ID      Workpiece material\n"
            "  --stepover RATIO   Radial engagement as a fraction of diameter\n"
            "  --stepdown MM      Axial depth per pass\n"
            "  --finishing-subdivisions N\n"
            "                     Interleaved stock-aware roughing passes (default 1)\n"
            "  --path-pitch MM    Simulation/feed sampling interval (default 1.0)\n"
            "  --mode rough|finish|pocket|plane\n"
            "                     Operation mode (pocket/plane need no mesh)\n"
            "  --contour-direction either|climb|conventional\n"
            "                     Permitted contour traversal direction\n"
            "  --entry-front-tolerance MM\n"
            "                     Whole-curve Z association envelope (default 3)\n"
            "  --tiny-split-area-factor VALUE\n"
            "                     Ignore terminal split branches below VALUE *\n"
            "                     stepover^2 (default 0.5; zero disables)\n"
            "  --rapid-clearance MM\n"
            "                     Safe rapid height above the tallest of part and\n"
            "                     blank (default 10)\n"
            "  --full-feed RATE   Manual feed at full engagement, mm/min\n"
            "  --no-feed RATE     Manual feed at zero engagement, mm/min\n"
            "  --gcode PATH       Plan and post without opening a window, then exit\n"
            "  --profile-planning Print planning phase timings and scale counters\n"
            "  --screenshot PATH  Render one frame to a binary PPM and exit\n"
            "  --box-stock MM     Use the part's bounding box, grown by MM, as stock\n"
            "  --models stock|part|both   What the plan view shows\n"
            "  --view models|pass|result\n"
            "                     Which view a screenshot should capture\n"
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
  if (!view_name.empty() && view_name != "models" && view_name != "pass" &&
      view_name != "result") {
    std::fprintf(stderr, "--view must be models, pass, or result\n");
    return EXIT_FAILURE;
  }
  if (!job_in.empty()) {
    try { load_job_into(app, job_in); }
    catch (const std::exception& error) {
      std::fprintf(stderr, "%s\n", error.what());
      return EXIT_FAILURE;
    }
  } else if (app.part_path[0] != '\0' || app.stock_path[0] != '\0') load_meshes(app);
  app.geometry.set_box_stock(app.job.box_stock, app.job.box_stock_margin);

  // Anything the command line set has to reach the operation the panels would
  // otherwise have created, so make it here and apply them to it.
  if (!app.job.operations.empty() || app.geometry.has_part() || !mode_name.empty()) {
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
    if (finishing_subdivisions > 0)
      operation.finishing_subdivisions = finishing_subdivisions;
    if (stepover > 0.0) operation.stepover_ratio = stepover;
    if (path_pitch > 0.0) operation.path_pitch = path_pitch;
    if (entry_front_tolerance > 0.0)
      operation.entry_front_match_tolerance = entry_front_tolerance;
    if (tiny_split_area_factor)
      operation.tiny_split_area_factor = *tiny_split_area_factor;
    if (rapid_clearance > 0.0)
      app.job.rapid_clearance_height = rapid_clearance;
    if (!mode_name.empty()) {
      if (mode_name == "rough") operation.mode = cam::OperationMode::Roughing;
      else if (mode_name == "finish")
        operation.mode = cam::OperationMode::ContourFinishing;
      else if (mode_name == "pocket") operation.mode = cam::OperationMode::Pocket;
      else if (mode_name == "plane") operation.mode = cam::OperationMode::PlaneClearing;
      else {
        std::fprintf(stderr, "--mode must be rough, finish, pocket or plane\n");
        return EXIT_FAILURE;
      }
    }
    if (!direction_name.empty()) {
      if (direction_name == "either")
        operation.contour_direction = cam::ContourDirectionPolicy::Either;
      else if (direction_name == "climb")
        operation.contour_direction = cam::ContourDirectionPolicy::Climb;
      else if (direction_name == "conventional")
        operation.contour_direction = cam::ContourDirectionPolicy::Conventional;
      else {
        std::fprintf(stderr,
                     "--contour-direction must be either, climb, or conventional\n");
        return EXIT_FAILURE;
      }
    }
    if (full_feed > 0.0 || no_feed > 0.0) {
      operation.manual_feed_range = true;
      if (full_feed > 0.0) operation.full_engagement_feed = full_feed;
      if (no_feed > 0.0) operation.no_engagement_feed = no_feed;
    }
  }

  // Planning and posting with no window at all, which is how this gets checked
  // without a display and how it fits into anything scripted.
  if (!gcode_out.empty()) {
    if (app.job.operations.empty()) {
      std::fprintf(stderr, "--gcode needs --job, --mode, or model inputs\n");
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
    if (profile_planning) {
      const cam::PlanningMetrics& m = plan.metrics;
      std::fprintf(
          stderr,
          "planning_profile_ms total=%.3f slicing=%.3f preparation=%.3f "
          "wavefront=%.3f lineage=%.3f inter_layer=%.3f toolpath=%.3f "
          "cleanup=%.3f stitching=%.3f resampling=%.3f simulation=%.3f "
          "feeds=%.3f\n",
          m.total_milliseconds, m.slicing_milliseconds,
          m.preparation_milliseconds, m.wavefront_milliseconds,
          m.lineage_milliseconds, m.inter_layer_milliseconds,
          m.toolpath_milliseconds, m.cleanup_milliseconds,
          m.stitching_milliseconds, m.resampling_milliseconds,
          m.simulation_milliseconds, m.feeds_milliseconds);
      std::fprintf(
          stderr,
          "planning_profile_counts wave_generations=%zu fronts=%zu "
          "analysis_layers=%zu analysis_trials=%zu analysis_accepted=%zu "
          "toolpath_moves=%zu simulation_intervals=%zu "
          "empty_simulation_intervals=%zu gouge_bounds_rejections=%zu\n",
          m.wave_generations, m.fronts, m.analysis_layers, m.analysis_trials,
          m.accepted_analysis_trials, m.toolpath_moves,
          m.simulation_intervals, m.empty_simulation_intervals,
          m.gouge_bounds_rejections);
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
        "%zu node Z-chain(s), %zu entry-front Z-chain(s)\n"
        "%zu lines written to %s\n",
        plan.stats.cut_levels, plan.stats.levels, plan.stats.cutting_distance,
        plan.stats.cutting_minutes + plan.stats.rapid_minutes, plan.stats.maximum_load,
        plan.operations.empty() ? 0 :
            plan.operations.front().inter_layer.node_chains.size(),
        plan.operations.empty() ? 0 :
            plan.operations.front().inter_layer.entry_front_chains.size(),
        post.line_count, gcode_out.string().c_str());
    if (!plan.operations.empty()) {
      const auto& topology = plan.operations.front().inter_layer;
      std::size_t unjoined_entries = 0;
      for (const auto& layer : topology.entry_front_chain_ids)
        unjoined_entries += static_cast<std::size_t>(std::count(
            layer.begin(), layer.end(), wave::kNoTopologyId));
      for (std::size_t chain = 0; chain < topology.entry_front_chains.size();
           ++chain) {
        const auto& members = topology.entry_front_chains[chain].members;
        if (!members.empty())
          std::printf("  entry chain %zu: layers %zu-%zu (%zu fronts)\n", chain,
                      members.front().layer, members.back().layer,
                      members.size());
      }
      if (unjoined_entries != 0)
        std::printf("  %zu entry front(s) remain unassociated\n",
                    unjoined_entries);
    }
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
  if (screenshot_pending && !app.job.operations.empty())
    start_plan(app);
  if (!view_name.empty()) {
    if (view_name == "result") app.force_tab = 2;
    else if (view_name == "models") app.force_tab = 0;
    else app.force_tab = 1;
  }
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
      app.stock_cache_operation = -1;
      log_warnings("Planning", app.plan.warnings);
      app.has_plan = app.plan.ok();
      select_operation(app, app.selected_operation);
      app.view.needs_fit = true;
      app.playback.distance = 0.0;
      app.playback.motion_seconds = 0.0;
      app.material.clear();
      app.material_distance = -1.0;
      app.material_level = -1;
      if (progress_fraction >= 0.0) {
        const cam::LevelPlan* level = current_level(app);
        app.playback.distance =
            std::clamp(progress_fraction, 0.0, 1.0) * level_length(level);
        sync_playback_time(app);
      }
      if (!app.plan.ok()) {
        std::fprintf(stderr, "Planning failed:\n%s\n", app.plan.error.c_str());
        say(app, app.plan.error, true);
      } else if (screenshot_pending) {
        // The view was chosen on the command line; leave it alone.
      } else {
        app.force_tab = 2;
        say(app, "Planned " + std::to_string(app.plan.stats.cut_levels) + " passes");
      }
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
    ImGui::BeginChild("left-panel", ImVec2(kLeftPanelWidth, 0.0f));
    ImGui::PushItemWidth(kLeftPanelWidth * 0.48f);
    left_panel(app);
    ImGui::PopItemWidth();
    ImGui::EndChild();
    ImGui::SameLine();
    const float centre_width = std::max(
        200.0f, ImGui::GetContentRegionAvail().x - kRightPanelWidth -
                    ImGui::GetStyle().ItemSpacing.x);
    ImGui::BeginChild("views", ImVec2(centre_width, 0));
    if (ImGui::BeginTabBar("view-tabs")) {
      const auto flag = [&](int index) -> ImGuiTabItemFlags {
        return app.force_tab == index ? ImGuiTabItemFlags_SetSelected : 0;
      };
      if (ImGui::BeginTabItem("Plan", nullptr, flag(0))) {
        draw_setup(app);
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Pass", nullptr, flag(1))) {
        draw_canvas(app);
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Stacked result", nullptr, flag(2))) {
        draw_result(app);
        ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
    }
    // One frame of forcing is enough; leaving it set would pin the tab against
    // the mouse. A screenshot keeps asking, since the view it wants is chosen
    // before there is a plan to show in it.
    if (!screenshot_pending) app.force_tab = -1;
    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginChild("right-panel", ImVec2(0, 0));
    ImGui::PushItemWidth(kRightPanelWidth * 0.44f);
    right_panel(app);
    ImGui::PopItemWidth();
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
      // Wait for the plan so the captured frame has something in it.
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
  prism_view.release();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return EXIT_SUCCESS;
}
