#pragma once

using Clipper2Lib::Paths64;
using Clipper2Lib::Point64;

// A quarter turn, which is as far as the camera's elevation goes either way.
constexpr float kQuarterTurn = 1.57079632679489661923f;

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
  cam::PlanningProgress progress() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return progress_;
  }
  std::string stage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stage_;
  }

  void start(std::function<Result(const cam::ProgressCallback&)> task) {
    cancel_and_join();
    cancelled_.store(false);
    running_.store(true);
    finished_.store(false);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      progress_ = {};
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

  // Cancel and wait. The worker reads state the App owns, so anything about to
  // disturb that state has to be able to stop it first rather than merely ask.
  void stop() { cancel_and_join(); }

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
    return [this](cam::PlanningProgress progress, const std::string& stage) {
      std::lock_guard<std::mutex> lock(mutex_);
      progress_ = progress;
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
  cam::PlanningProgress progress_;
  std::string stage_;
  Result result_;
};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

enum class Colouring { Feed, Limit, Load, Move };

// Which models a 3D view draws. The stock encloses the part, so showing both
// means showing the stock through, not showing it on top.
enum class ModelShow { None, Part, Stock, Both };

inline const char* model_show_name(ModelShow value) {
  switch (value) {
    case ModelShow::None: return "None";
    case ModelShow::Part: return "Part";
    case ModelShow::Stock: return "Stock";
    case ModelShow::Both: return "Part and stock";
  }
  return "None";
}

// The operation modes, in the order they are offered.
constexpr cam::OperationMode kOperationModes[]{
    cam::OperationMode::Roughing,
    cam::OperationMode::ContourFinishing,
    cam::OperationMode::Pocket,
    cam::OperationMode::Hole,
    cam::OperationMode::PlaneClearing,
    cam::OperationMode::SurfaceFinishing,
};

// `cam::operation_mode_name` is the job file's key; this is the label.
inline const char* operation_mode_label(cam::OperationMode value) {
  switch (value) {
    case cam::OperationMode::Roughing: return "Roughing";
    case cam::OperationMode::ContourFinishing: return "Contour finishing";
    case cam::OperationMode::Hole: return "Hole";
    case cam::OperationMode::Pocket: return "Pocket";
    case cam::OperationMode::PlaneClearing: return "Plane clearing";
    case cam::OperationMode::SurfaceFinishing: return "Surface finishing (STEP)";
  }
  return "Unknown";
}

// How an operation stands against the plan on screen. A plan is kept when
// settings change: it is still the best picture of the job, and an edit only
// changes which operations it still describes.
enum class PlanState {
  None,        // this operation has no result at all
  Planned,     // this operation's passes came from exactly these settings
  Failed,      // planned, and the planner could not do it
  Edited,      // its own settings have changed since it was planned
  Downstream,  // unchanged, but something ahead of it has been planned since
};

// What one operation's plan was built from, and when. The signature notices an
// edit; the serial orders the operations against each other.
struct PlannedRecord {
  std::string signature;
  std::uint64_t serial = 0;
};

// The error an unplanned operation's entry carries, so everything that skips a
// failed operation skips it too.
constexpr const char* kUnplanned = "Not planned yet";

enum class NodeDebugColouring { None, Node, ZGroup };

struct View {
  ImVec2 pan{0.0f, 0.0f};
  float zoom = 4.0f;
  bool needs_fit = true;
};

// The orbited 3D views. Orthographic.
struct Orbit {
  float yaw = 0.85f;
  // Elevation runs the whole quarter turn either way.
  float pitch = 0.62f;
  float zoom = 1.0f;
  // A view-cube snap turns the camera over a few frames.
  bool turning = false;
  float turn = 0.0f;  // 0 to 1 across the movement
  float from_yaw = 0.0f, from_pitch = 0.0f;
  float to_yaw = 0.0f, to_pitch = 0.0f;
  // A left press that landed on the view cube orbits nothing.
  bool cube_pressed = false;
  // Screen-space pan of the framed centre, in pixels.
  ImVec2 pan{0.0f, 0.0f};
};

// Playback along one pass. Distance is along that pass's toolpath.
struct Playback {
  bool playing = false;
  double distance = 0.0;
  // Commanded-machine time within this pass. Distinct from distance, which
  // stands still during retracts and plunges.
  double motion_seconds = 0.0;
  double speed = 40.0;  // mm of toolpath per second of wall clock
  // Run at the commanded feeds rather than a fixed distance per second.
  bool real_time = true;
  double time_scale = 1.0;  // multiples of real time
  bool follow = true;   // carry on into the next pass
};

enum class LogSeverity {
  Info,
  Warning,
  Error,
};

struct LogMessage {
  LogSeverity severity = LogSeverity::Info;
  std::string source;
  std::string text;
};

// A modal question with one action. `action` runs on confirm.
struct Confirmation {
  bool pending = false;
  std::string title;
  std::string text;
  std::string accept = "OK";
  std::function<void()> action;
};

// The in-app file browser. One at a time; `accept` receives the chosen path.
struct FileDialog {
  bool pending = false;
  bool saving = false;
  std::string title;
  std::vector<std::string> extensions;  // lower case, with the dot
  std::filesystem::path directory;
  std::array<char, 512> name{};
  std::array<char, 1024> location{};
  std::string selected;
  std::string error;
  std::function<void(const std::filesystem::path&)> accept;
};

struct LoadedModels {
  cam::Job job;
  cam::JobGeometry geometry;
  std::filesystem::path job_file;
  std::vector<std::string> warnings;
  bool opened_job = false, reset_setup = false;
  bool replace_part = true, replace_stock = true;
  std::string error;
};

struct App {
  cam::ToolLibrary tools;
  cam::MachineLibrary machines;
  cam::MaterialLibrary materials;
  cam::UserSettings settings = cam::default_user_settings();
  cam::Job job;
  cam::JobGeometry geometry;
  // One entry per operation in the job, always: an operation with no result
  // carries an entry saying so.
  cam::JobPlan plan;
  bool has_plan = false;
  // What each entry was planned from. Dirtiness is the difference between
  // this and the job now, worked out once a frame.
  std::vector<PlannedRecord> planned;
  std::uint64_t plan_serial = 0;
  std::vector<PlanState> plan_states;

  Background<cam::PartialPlan> planner;
  Background<LoadedModels> model_loader;
  bool model_loading = false, model_load_cancelled = false;
  int model_rotation_target = 0;
  std::string model_load_label, model_load_error;
  Background<cam::HoleDetection> hole_detector;
  std::string hole_detection_id;
  cam::HoleDetection hole_detection;
  Background<cam::SurfaceContourPreview> surface_previewer;
  cam::SurfaceContourPreview surface_preview;
  cam::SurfaceContourOrder surface_order;
  std::string surface_preview_key, surface_order_key;
  std::vector<std::size_t> surface_order_roots;
  // Cached screen depth of the part for visible-surface contour picking.
  std::shared_ptr<const cam::CadModel> surface_depth_model;
  Projection surface_depth_projection{};
  ImVec2 surface_depth_origin{}, surface_depth_size{};
  int surface_depth_width = 0, surface_depth_height = 0;
  std::vector<float> surface_depth;
  // The range in flight, and what its operations said when it was asked for.
  std::size_t planning_first = 0;
  std::vector<std::string> planning_signatures;
  std::vector<LogMessage> messages;
  int message_filter = 0;  // 0 all, 1 info, 2 warnings, 3 errors
  bool show_messages = false;
  std::string graphics_renderer;
  bool software_graphics = false;

  // Where the job lives; empty until it is first saved or opened.
  std::filesystem::path job_file;
  // The job as last saved or opened, for noticing unsaved changes.
  std::string saved_job;
  int selected_operation = 0;
  // Separate from the focused row: the operations the multi-operation
  // planning and export actions use.
  std::vector<char> operation_selection;
  int selected_level = 0;
  int selected_plan_operation = 0;
  Colouring colouring = Colouring::Feed;
  // Set from the command line so a screenshot can be told which view to take.
  int force_tab = -1;
  // Keep forcing the tab every frame, which a screenshot needs.
  bool force_tab_hold = false;
  // Rendering one frame to a file: nothing is saved.
  bool screenshot_mode = false;
  bool show_stock = true;
  bool show_part = true;
  bool show_material = true;
  bool show_cutting_paths = true;
  bool show_rapid_travel = true;
  // Draw every pass of the operation rather than only the current one.
  bool show_all_passes = false;
  ModelShow setup_models = ModelShow::Both;
  ModelShow result_models = ModelShow::None;
  bool wireframe = false;
  NodeDebugColouring node_debug_colouring = NodeDebugColouring::None;
  // Which corner, edge or centre of the blank the origin picker is aimed at:
  // 0 low, 1 middle, 2 high per axis.
  std::array<int, 3> origin_pick{1, 1, 2};
  bool show_entry_front_chains = false;
  bool show_boundary_contacts = false;
  // Follow the cutter, so the stacked view and playback describe one instant.
  bool stack_follows_playback = true;
  std::size_t fill_failures = 0;
  View view;
  Orbit orbit;
  Playback playback;
  // Remaining stock for the pass being watched, recomputed only when the
  // playback position actually moves.
  std::shared_ptr<const wave::Heightmap> material_heightmap;
  double material_motion_seconds = -1;
  double material_motion_distance = 0;
  bool material_section_dirty = false;
  Paths64 material;
  double material_distance = -1.0;
  int material_level = -1;
  int material_operation = -1;

  // Windows
  bool show_tool_library = false;
  bool show_material_library = false;
  bool show_settings = false;
  bool show_gcode_preview = false;
  bool builder_open = false;
  bool builder_is_new = false;
  cam::Tool builder_tool;

  std::string gcode;
  cam::PostResult post;
  cam::ExportFlavor export_flavor = cam::ExportFlavor::MachineDefault;
  std::vector<int> posted_operations;
  std::filesystem::path gcode_file;

  Confirmation confirmation;
  FileDialog file_dialog;
  bool quit_requested = false;
  bool quit = false;

  std::filesystem::path tools_file;
  std::filesystem::path materials_file;
  std::filesystem::path settings_file;
  bool panel_layout_dragging = false;
  ImGuiStyle unscaled_style;
  bool unscaled_style_captured = false;
  ImFont* theme_sans_font = nullptr;
  ImFont* theme_mono_font = nullptr;
  ImFont* theme_serif_font = nullptr;
  bool theme_fonts_loaded = false;
};

inline float ui_size(const App& app, float value) {
  return value * static_cast<float>(app.settings.ui_scale);
}

inline void log_message(App& app, LogSeverity severity, std::string source,
                        std::string text) {
  app.messages.push_back({severity, std::move(source), std::move(text)});
}

inline void say(App& app, std::string text, bool error = false) {
  log_message(app, error ? LogSeverity::Error : LogSeverity::Info,
              "Application", std::move(text));
}

inline void log_warnings(App& app, const char* source,
                         const std::vector<std::string>& warnings) {
  for (const auto& warning : warnings)
    std::fprintf(stderr, "%s warning: %s\n", source, warning.c_str());
  for (const auto& warning : warnings)
    app.messages.push_back({LogSeverity::Warning, source, warning});
}
