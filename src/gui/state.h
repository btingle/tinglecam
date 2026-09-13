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

inline const char* model_show_name(ModelShow value) {
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

inline const char* stack_models_name(StackModels value) {
  switch (value) {
    case StackModels::None: return "No model";
    case StackModels::Part: return "Part";
    case StackModels::Stock: return "Stock";
    case StackModels::Both: return "Both";
  }
  return "No model";
}

// The operation modes, in the order they are offered. One list, so the Add
// picker, the Mode combo and anything else that has to enumerate them cannot
// come to differ.
constexpr cam::OperationMode kOperationModes[]{
    cam::OperationMode::Roughing,
    cam::OperationMode::ContourFinishing,
    cam::OperationMode::Pocket,
    cam::OperationMode::PlaneClearing,
};

// `cam::operation_mode_name` is the name the job file carries, which is the
// wrong thing to put in front of an operator: `plane_clearing` is a key, not a
// label. The file keeps its name and the window gets this one.
inline const char* operation_mode_label(cam::OperationMode value) {
  switch (value) {
    case cam::OperationMode::Roughing: return "Roughing";
    case cam::OperationMode::ContourFinishing: return "Contour finishing";
    case cam::OperationMode::Pocket: return "Pocket";
    case cam::OperationMode::PlaneClearing: return "Plane clearing";
  }
  return "Unknown";
}

// How an operation stands against the plan on screen. A plan is kept when
// settings change rather than thrown away: what came out of the last run is
// still the best picture of the job there is, and the only thing an edit
// actually changes is which operations that picture still describes. Saying so
// costs one marker per row; discarding it costs the operator the preview they
// were editing against.
enum class PlanState {
  None,        // this operation has no result at all
  Planned,     // this operation's passes came from exactly these settings
  Failed,      // planned, and the planner could not do it
  Edited,      // its own settings have changed since it was planned
  Downstream,  // unchanged, but something ahead of it has been planned since
};

// What one operation's plan was built from, and when. The signature notices an
// edit; the serial orders the operations against each other, which is what
// tells an untouched operation that the one above it has been re-planned since
// -- and therefore that the stock it inherited is no longer what it was handed.
struct PlannedRecord {
  std::string signature;
  std::uint64_t serial = 0;
};

// The error an operation's entry in the plan carries when there is no result
// for it: one added since the plan was made, or one no re-plan has reached.
// Written into the entry rather than left blank so that everything already
// skipping a failed operation -- the post included -- skips this too.
constexpr const char* kUnplanned = "Not planned yet";

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
  // Azimuth roams: there is no edge to the turntable, and a view that stops
  // short of coming round again is a view that has to be dragged back.
  float yaw = 0.85f;
  // Elevation runs the whole quarter turn either way, so straight down and
  // straight up are both reachable and both exact. Nothing needs it kept off
  // the poles: the basis stays orthonormal there, the painter's sort still
  // separates the slabs by height, and the raycaster already handles a ray
  // parallel to a prism's faces.
  float pitch = 0.62f;
  float zoom = 1.0f;
  // A view-cube snap turns the camera over a few frames rather than teleporting
  // it. Which way the model went is most of what the snap has to say, and a cut
  // between two orientations does not say it.
  bool turning = false;
  float turn = 0.0f;  // 0 to 1 across the movement
  float from_yaw = 0.0f, from_pitch = 0.0f;
  float to_yaw = 0.0f, to_pitch = 0.0f;
  // A left press that landed on the view cube. Held so the drag it may turn
  // into orbits nothing: pressing the cube is asking for a view, not steering.
  bool cube_pressed = false;
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

struct App {
  cam::ToolLibrary tools;
  cam::MachineLibrary machines;
  cam::MaterialLibrary materials;
  cam::UserSettings settings = cam::default_user_settings();
  cam::Job job;
  cam::JobGeometry geometry;
  // One entry per operation in the job, always: an operation with no result
  // carries an entry saying so. Keeping the two lists in step is what lets a
  // re-plan replace one operation's passes and leave the rest standing.
  cam::JobPlan plan;
  bool has_plan = false;
  // What each of those entries was planned from. Dirtiness is the difference
  // between this and the job now, worked out once a frame, rather than a flag
  // every edit path has to remember to set -- and there is an edit path for
  // every field in the settings panel.
  std::vector<PlannedRecord> planned;
  std::uint64_t plan_serial = 0;
  std::vector<PlanState> plan_states;

  Background<cam::PartialPlan> planner;
  // The range in flight, and what its operations said when it was asked for.
  // Taken at the start rather than read back at the end: the settings panel
  // stays live while the planner runs, and a plan describes the settings it
  // was given, not the ones it happens to land among.
  std::size_t planning_first = 0;
  std::vector<std::string> planning_signatures;
  std::string banner;
  bool banner_is_error = false;
  std::vector<LogMessage> messages;
  int message_filter = 0;  // 0 all, 1 info, 2 warnings, 3 errors

  std::array<char, 512> part_path{};
  std::array<char, 512> stock_path{};
  int selected_operation = 0;
  // Separate from the focused row: these are the operations used by the
  // multi-operation planning and G-code actions.
  std::vector<char> operation_selection;
  // What the Add button will add. One picker and one button rather than a
  // button per mode: the modes are a list that grows, and a row of buttons is
  // not.
  cam::OperationMode add_mode = cam::OperationMode::Roughing;
  cam::OperationMode settings_mode = cam::OperationMode::Roughing;
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
  // The plan view drawn as edges rather than surfaces. What it is for is
  // seeing one body through another -- a pocket volume standing inside the
  // blank, or the part inside it -- which no amount of translucency in a
  // painter-sorted solid does honestly.
  bool wireframe = false;
  // What the stacked result draws over the passes. Off to begin with: the
  // stack is the thing being looked at, and a model over it is a question
  // asked of that stack rather than the default way to read it.
  StackModels stack_models = StackModels::None;
  // Cutting motion and rapid travel are separate questions, and both views
  // answer them the same way. A stack read for where the tool went wants the
  // cuts alone; one read for how much of the job is lifting wants the rapids.
  bool show_rapid_travel = true;
  bool show_material = true;
  bool show_cutting_paths = true;
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
  std::vector<cam::StockSlab> stock_display_cache;
  double stock_display_distance = -1.0;

  // Tool Builder
  bool builder_open = false;
  bool builder_is_new = false;
  cam::Tool builder_tool;

  std::string gcode;
  cam::PostResult post;
  std::vector<int> posted_operations;
  std::array<char, 512> save_path{};

  std::filesystem::path tools_file;
  std::filesystem::path settings_file;
  std::string tools_import_file;
  std::string job_file = "tinglecam-job.json";
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

inline void say(App& app, std::string text, bool error = false) {
  app.banner = std::move(text);
  app.banner_is_error = error;
  app.messages.push_back({error ? LogSeverity::Error : LogSeverity::Info,
                          "Application", app.banner});
}

inline void log_warnings(App& app, const char* source,
                         const std::vector<std::string>& warnings) {
  for (const auto& warning : warnings)
    std::fprintf(stderr, "%s warning: %s\n", source, warning.c_str());
  for (const auto& warning : warnings)
    app.messages.push_back({LogSeverity::Warning, source, warning});
}

inline void show_warnings(const char* id, const std::vector<std::string>& warnings) {
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
