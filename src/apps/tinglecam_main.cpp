#include "../gui/gui_internal.h"
#include "cam/profiling.h"
#include "rendering/graphics_startup.h"
#include "wave/mesh_slicer_cuda.h"

using namespace tinglecam_gui;

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
  std::vector<double> depth_breaks;
  std::optional<double> cleanup_stepover_limit;
  bool fixed_cleanup_stepover = false;
  double stepover = 0.0;
  double path_pitch = 0.0;
  double simulation_pitch = 0.0, surface_stepover = 0.0, surface_tolerance = 0.0;
  std::vector<std::size_t> selected_faces;
  bool cad_info = false, flow_u = false, flip_normals = false;
  double entry_front_tolerance = 0.0;
  std::optional<double> tiny_split_area_factor;
  double rapid_clearance = 0.0;
  std::optional<double> box_stock_uniform;
  std::optional<double> box_margin_minus_x;
  std::optional<double> box_margin_plus_x;
  std::optional<double> box_margin_minus_y;
  std::optional<double> box_margin_plus_y;
  std::optional<double> box_margin_minus_z;
  std::optional<double> box_margin_plus_z;
  std::optional<double> spindle_rpm, chip_load, plunge_feed;
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
      if (argument == "--part") app.job.part_mesh = value();
      else if (argument == "--stock") app.job.stock_mesh = value();
      else if (argument == "--screenshot") screenshot = value();
      else if (argument == "--view") view_name = value();
      else if (argument == "--at") progress_fraction = std::stod(value());
      else if (argument == "--follow") app.stack_follows_playback = true;
      else if (argument == "--models") {
        const std::string which = value();
        app.setup_models = which == "stock"  ? ModelShow::Stock
                           : which == "part" ? ModelShow::Part
                                             : ModelShow::Both;
      }
      else if (argument == "--box-stock") {
        box_stock_uniform = std::stod(value());
      }
      else if (argument == "--box-margin-minus-x")
        box_margin_minus_x = std::stod(value());
      else if (argument == "--box-margin-plus-x")
        box_margin_plus_x = std::stod(value());
      else if (argument == "--box-margin-minus-y")
        box_margin_minus_y = std::stod(value());
      else if (argument == "--box-margin-plus-y")
        box_margin_plus_y = std::stod(value());
      else if (argument == "--box-margin-minus-z")
        box_margin_minus_z = std::stod(value());
      else if (argument == "--box-margin-plus-z")
        box_margin_plus_z = std::stod(value());
      else if (argument == "--gcode") gcode_out = value();
      else if (argument == "--job") job_in = value();
      else if (argument == "--export-flavor") {
        const auto flavor=value();
        if(flavor=="machine-default") app.export_flavor=cam::ExportFlavor::MachineDefault;
        else if(flavor=="roland-mdx40a") app.export_flavor=cam::ExportFlavor::RolandMdx40a;
        else throw std::runtime_error("--export-flavor must be machine-default or roland-mdx40a");
      }
      else if (argument == "--tool") tool_id = value();
      else if (argument == "--machine") machine_id = value();
      else if (argument == "--material") material_id = value();
      else if (argument == "--stepdown") stepdown = std::stod(value());
      else if (argument == "--finishing-subdivisions")
        finishing_subdivisions = std::stoi(value());
      else if (argument == "--depth-break") depth_breaks.push_back(std::stod(value()));
      else if (argument == "--cleanup-stepover-limit")
        cleanup_stepover_limit = std::stod(value());
      else if (argument == "--fixed-cleanup-stepover") fixed_cleanup_stepover = true;
      else if (argument == "--stepover") stepover = std::stod(value());
      else if (argument == "--simulation-pitch") simulation_pitch = std::stod(value());
      else if (argument == "--surface-stepover") surface_stepover = std::stod(value());
      else if (argument == "--surface-tolerance") surface_tolerance = std::stod(value());
      else if (argument == "--face") {
        const int face = std::stoi(value());
        if (face < 1) throw std::invalid_argument("Face numbers start at 1");
        selected_faces.push_back(std::size_t(face-1));
      }
      else if (argument == "--cad-info") cad_info = true;
      else if (argument == "--flow-u") flow_u = true;
      else if (argument == "--reverse-normals") flip_normals = true;
      else if (argument == "--path-pitch") path_pitch = std::stod(value());
      else if (argument == "--entry-front-tolerance")
        entry_front_tolerance = std::stod(value());
      else if (argument == "--tiny-split-area-factor")
        tiny_split_area_factor = std::stod(value());
      else if (argument == "--rapid-clearance") rapid_clearance = std::stod(value());
      else if (argument == "--mode") mode_name = value();
      else if (argument == "--contour-direction") direction_name = value();
      else if (argument == "--spindle-rpm") spindle_rpm = std::stod(value());
      else if (argument == "--chip-load") chip_load = std::stod(value());
      else if (argument == "--plunge-feed") plunge_feed = std::stod(value());
      else if (argument == "--profile-planning") profile_planning = true;
      else if (argument == "--help" || argument == "-h") {
        std::printf(
            "TingleCAM -- wavefront roughing and contour finishing\n\n"
            "  --part PATH        Part model (OBJ, STL or STEP)\n"
            "  --stock PATH       Stock model (OBJ, STL or STEP)\n"
            "  --job PATH         Load a job JSON, including pocket/plane dimensions\n"
            "  --tool ID          Tool from the library\n"
            "  --machine ID       Machine profile for the job\n"
            "  --material ID      Workpiece material for the job\n"
            "  --stepover RATIO   Radial engagement as a fraction of diameter\n"
            "  --stepdown MM      Axial depth per pass\n"
            "  --finishing-subdivisions N\n"
            "                     Coarse descent, then N-1 upward cuts per terrace\n"
            "  --depth-break MM  Intermediate coarse floor; repeat for multiple depths\n"
            "  --cleanup-stepover-limit RATIO\n"
            "                     Adaptive cleanup width ceiling (default 0.5 diameter)\n"
            "  --fixed-cleanup-stepover  Keep the base width on cleanup layers\n"
            "  --path-pitch MM    Engagement and feed sampling interval (default 1.0)\n"
            "  --simulation-pitch MM  XY stock-grid spacing for the job (default 0.02)\n"
            "  --cad-info         Print imported STEP face IDs and source identity\n"
            "  --face N           Select CAD face N (1-based, repeatable)\n"
            "  --surface-stepover MM / --surface-tolerance MM\n"
            "                     Physical stepover / XYZ chord tolerance\n"
            "  --flow-u / --reverse-normals  Surface finishing direction controls\n"
            "  --mode rough|finish|pocket|plane|surface|hole\n"
            "                     Operation mode (pocket/plane/manual holes need no mesh)\n"
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
            "  --spindle-rpm RPM / --chip-load MM / --plunge-feed MM_PER_MIN\n"
            "                     Override the tool's cutting data for the material\n"
            "  --gcode PATH       Plan and post without opening a window, then exit\n"
            "  --export-flavor machine-default|roland-mdx40a\n"
            "                     Output dialect for GUI and headless export\n"
            "  --profile-planning Print planning phase timings and scale counters\n"
            "  --screenshot PATH  Render one frame to a binary PPM and exit\n"
            "  --box-stock MM     Use the part's bounding box, grown by MM in XY,\n"
            "                     as stock (directional margins are configurable in the UI)\n"
            "  --box-margin-minus-x MM / --box-margin-plus-x MM\n"
            "  --box-margin-minus-y MM / --box-margin-plus-y MM\n"
            "  --box-margin-minus-z MM / --box-margin-plus-z MM\n"
            "                     Directional box-stock margins\n"
            "  --models stock|part|both   What the setup view shows\n"
            "  --view setup|pass|result\n"
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
  if (!view_name.empty() && view_name != "setup" && view_name != "pass" &&
      view_name != "result") {
    std::fprintf(stderr, "--view must be setup, pass, or result\n");
    return EXIT_FAILURE;
  }
  // Interactive startup reads job metadata now and loads geometry after the
  // window is visible, so long STEP imports can show their progress.
  const bool defer_models = gcode_out.empty() && !cad_info;
  if (!job_in.empty()) {
    try { load_job_into(app, job_in, !defer_models); }
    catch (const std::exception& error) {
      std::fprintf(stderr, "%s\n", error.what());
      return EXIT_FAILURE;
    }
  } else if (!defer_models && (!app.job.part_mesh.empty() || !app.job.stock_mesh.empty()))
    load_meshes(app);
  if (!machine_id.empty()) app.job.machine_id = machine_id;
  if (!material_id.empty()) app.job.material_id = material_id;
  if (simulation_pitch > 0.0) app.job.simulation_pitch = simulation_pitch;
  if (rapid_clearance > 0.0) app.job.rapid_clearance_height = rapid_clearance;
  if (box_stock_uniform) {
    const double margin = *box_stock_uniform;
    app.job.box_stock = true;
    app.job.box_stock_margins = {margin, margin, margin, margin, 0.0, 0.0};
  }
  const auto apply_box_margin = [&](const std::optional<double>& value,
                                    double& target) {
    if (!value) return;
    app.job.box_stock = true;
    target = *value;
  };
  apply_box_margin(box_margin_minus_x, app.job.box_stock_margins.minus_x);
  apply_box_margin(box_margin_plus_x, app.job.box_stock_margins.plus_x);
  apply_box_margin(box_margin_minus_y, app.job.box_stock_margins.minus_y);
  apply_box_margin(box_margin_plus_y, app.job.box_stock_margins.plus_y);
  apply_box_margin(box_margin_minus_z, app.job.box_stock_margins.minus_z);
  apply_box_margin(box_margin_plus_z, app.job.box_stock_margins.plus_z);
  app.geometry.set_box_stock(app.job.box_stock, app.job.box_stock_margins);

  if (cad_info) {
    const auto& cad=app.geometry.part_cad();
    if(!cad) {std::fprintf(stderr,"--cad-info requires a STEP part\n");return EXIT_FAILURE;}
    std::printf("STEP fingerprint %s, %zu faces, %zu triangles\n",cad->fingerprint().c_str(),cad->faces().size(),cad->mesh().triangles.size());
    for(const auto& face:cad->faces()) {
      const auto p=cad->evaluate(face.id,.5,.5);
      std::printf("Face %zu: %s, area %.6g mm2, midpoint %.4f %.4f %.4f, normal %.4f %.4f %.4f%s\n",
          face.id+1,face.type.c_str(),face.area,p.point.x,p.point.y,p.point.z,p.normal.x,p.normal.y,p.normal.z,p.inside?"":" (trimmed midpoint)");
    }
    return EXIT_SUCCESS;
  }
  cam::OperationMode requested_mode = cam::OperationMode::Roughing;
  if (!mode_name.empty()) {
    if (mode_name == "rough") requested_mode = cam::OperationMode::Roughing;
    else if (mode_name == "finish")
      requested_mode = cam::OperationMode::ContourFinishing;
    else if (mode_name == "pocket")
      requested_mode = cam::OperationMode::Pocket;
    else if (mode_name == "plane")
      requested_mode = cam::OperationMode::PlaneClearing;
    else if(mode_name=="hole" || mode_name=="holes") requested_mode=cam::OperationMode::Hole;
    else if (mode_name == "surface") requested_mode = cam::OperationMode::SurfaceFinishing;
    else {
      std::fprintf(stderr, "--mode must be rough, finish, pocket, plane, surface or hole\n");
      return EXIT_FAILURE;
    }
  }

  // Command-line options apply to an existing job operation, or to a new
  // operation only when --mode explicitly asks for one. Loading models alone
  // should leave the same empty operation list as a fresh GUI job.
  if (!app.job.operations.empty() || !mode_name.empty()) {
    if (app.job.operations.empty()) {
      cam::Operation operation = make_operation(app, requested_mode);
      // Through the helper, so the plan's entry for it exists from the start
      // and the three lists never have to be repaired.
      insert_operation(app, 0, std::move(operation));
    }
    cam::Operation& operation = app.job.operations.front();
    if (!tool_id.empty()) operation.tool_id = tool_id;
    if (stepdown > 0.0) operation.stepdown = stepdown;
    if (finishing_subdivisions > 0)
      operation.finishing_subdivisions = finishing_subdivisions;
    if (!depth_breaks.empty()) operation.depth_breaks = depth_breaks;
    if (cleanup_stepover_limit) operation.cleanup_stepover_limit = *cleanup_stepover_limit;
    if (fixed_cleanup_stepover) operation.adaptive_cleanup_stepover = false;
    if (stepover > 0.0) operation.stepover_ratio = stepover;
    if (path_pitch > 0.0) operation.path_pitch = path_pitch;
    if (surface_stepover > 0.0) operation.surface.stepover = surface_stepover;
    if (surface_tolerance > 0.0) operation.surface.chord_tolerance = surface_tolerance;
    if (flow_u) operation.surface.along_v = false;
    if (flip_normals) operation.surface.flip_normals = true;
    if (!selected_faces.empty()) {
      operation.surface.faces=selected_faces;
      if(app.geometry.part_cad()) operation.surface.model_fingerprint=app.geometry.part_cad()->fingerprint();
    }
    if (entry_front_tolerance > 0.0)
      operation.entry_front_match_tolerance = entry_front_tolerance;
    if (tiny_split_area_factor)
      operation.tiny_split_area_factor = *tiny_split_area_factor;
    if (!mode_name.empty()) operation.mode = requested_mode;
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
    if (spindle_rpm) operation.cutting.spindle_rpm = spindle_rpm;
    if (chip_load) operation.cutting.chip_load = chip_load;
    if (plunge_feed) operation.cutting.plunge_feed = plunge_feed;
  }

  // Planning and posting with no window at all, which is how this gets checked
  // without a display and how it fits into anything scripted.
  if (!gcode_out.empty()) {
    if (app.job.operations.empty()) {
      std::fprintf(stderr, "--gcode needs --job or --mode with model inputs\n");
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
      cam::report_plan_memory(plan);
      const cam::PlanningMetrics& m = plan.metrics;
      if (m.surface_samples) {
        std::fprintf(
            stderr,
            "surface_profile sampling_work_ms=%.3f linking_ms=%.3f "
            "samples=%zu seam_joins=%zu stay_down_links=%zu retracts=%zu\n",
            m.surface_sampling_milliseconds, m.surface_linking_milliseconds,
            m.surface_samples, m.surface_seam_joins, m.surface_stay_down_links,
            m.surface_retracts);
      }

      std::fprintf(
          stderr,
          "planning_profile_ms total=%.3f slicing=%.3f preparation=%.3f "
          "wavefront=%.3f lineage=%.3f inter_layer=%.3f toolpath=%.3f "
          "cleanup=%.3f stitching=%.3f resampling=%.3f "
          "feeds=%.3f parallel_cleanup=%.3f cleanup_work=%.3f xyz_simulation=%.3f\n",
          m.total_milliseconds, m.slicing_milliseconds,
          m.preparation_milliseconds, m.wavefront_milliseconds,
          m.lineage_milliseconds, m.inter_layer_milliseconds,
          m.toolpath_milliseconds, m.cleanup_milliseconds,
          m.stitching_milliseconds, m.resampling_milliseconds,
          m.feeds_milliseconds,
          m.parallel_cleanup_milliseconds, m.cleanup_work_milliseconds, m.xyz_simulation_milliseconds);
      std::fprintf(
          stderr,
          "planning_profile_counts wave_generations=%zu fronts=%zu "
          "analysis_layers=%zu analysis_trials=%zu analysis_accepted=%zu "
          "toolpath_moves=%zu\n",
          m.wave_generations, m.fronts, m.analysis_layers, m.analysis_trials,
          m.accepted_analysis_trials, m.toolpath_moves);
    }
    if (profile_planning) {
      for (std::size_t index = 0; index < plan.operations.size(); ++index) {
        const auto& metrics = plan.operations[index].metrics;
        std::fprintf(stderr,
                     "operation_profile index=%zu total_ms=%.3f toolpath_ms=%.3f xyz_ms=%.3f\n",
                     index, metrics.total_milliseconds, metrics.toolpath_milliseconds,
                     metrics.xyz_simulation_milliseconds);
      }
    }
    const cam::PostResult post = cam::post_job(plan, app.job, app.tools, app.machines, app.export_flavor);
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
        "Safety: %zu roughing plunge(s) onto stock, %zu roughing entry "
        "point(s) skipped, %zu rapid(s) over the part, %zu front(s) never "
        "reached, %.3f mm2 gouged\n",
        plan.stats.unsafe_plunges, plan.stats.skipped_entries,
        plan.stats.lifts_over_subject, plan.stats.unreachable_fronts,
        plan.stats.gouge_area);
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

  const auto graphics_defaults = wave::graphics::apply_wsl_defaults(
      wave::graphics::running_under_wsl(), wave::cuda_slicer::current_device_name);
  SDL_Window* window = nullptr;
  SDL_GLContext context = nullptr;
  const auto close_graphics = [&] {
    if (context) SDL_GL_DeleteContext(context);
    if (window) SDL_DestroyWindow(window);
    context = nullptr;
    window = nullptr;
    SDL_Quit();
  };
  const auto open_graphics = [&] {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) return false;
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    window = SDL_CreateWindow(
        "TingleCAM", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1500, 940,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!window) return false;
    context = SDL_GL_CreateContext(window);
    return context && SDL_GL_MakeCurrent(window, context) == 0;
  };
  bool graphics_open = open_graphics();
  if (!graphics_open && graphics_defaults.applied()) {
    const std::string reason = SDL_GetError();
    close_graphics();
    wave::graphics::undo_defaults(graphics_defaults);
    std::fprintf(stderr,
        "WSL hardware OpenGL startup failed (%s); retrying the original driver.\n",
        reason.c_str());
    graphics_open = open_graphics();
  }
  if (!graphics_open) {
    std::fprintf(stderr, "Graphics initialisation failed: %s\n", SDL_GetError());
    close_graphics();
    return EXIT_FAILURE;
  }
  SDL_GL_SetSwapInterval(1);
  if (const auto* renderer = glGetString(GL_RENDERER))
    app.graphics_renderer = reinterpret_cast<const char*>(renderer);
  app.software_graphics = wave::graphics::software_renderer(app.graphics_renderer);
  std::fprintf(stderr, "OpenGL renderer: %s\n", app.graphics_renderer.c_str());
  app.messages.push_back({LogSeverity::Info, "Graphics", app.graphics_renderer});
  if (app.software_graphics)
    app.messages.push_back({LogSeverity::Warning, "Graphics",
        "Software OpenGL rendering is active; 3D playback may be slow."});

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::GetIO().IniFilename = nullptr;
  load_ui_fonts(app);
  apply_ui_theme(app);
  ImGui_ImplSDL2_InitForOpenGL(window, context);
  ImGui_ImplOpenGL3_Init("#version 150");

  bool running = true;
  int screenshot_delay = 0;
  auto previous_frame = std::chrono::steady_clock::now();
  app.screenshot_mode = screenshot_pending;
  if (defer_models && (!app.job.part_mesh.empty() || !app.job.stock_mesh.empty()))
    start_model_loading(app, job_in.empty());
  bool startup_loading = app.model_loading;
  bool screenshot_plan_pending = screenshot_pending;
  if (!view_name.empty()) {
    app.force_tab = view_name == "result" ? 2 : view_name == "setup" ? 0 : 1;
    app.force_tab_hold = screenshot_pending;
  }
  std::string title;
  while (running) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT ||
          (event.type == SDL_WINDOWEVENT &&
           event.window.event == SDL_WINDOWEVENT_CLOSE &&
           event.window.windowID == SDL_GetWindowID(window))) {
        if (screenshot_pending) running = false;
        else request_quit(app);
      }
    }

    if (take_model_load(app) && startup_loading) {
      startup_loading = false;
      if (!selected_faces.empty() && !app.job.operations.empty() && app.geometry.part_cad())
        app.job.operations.front().surface.model_fingerprint = app.geometry.part_cad()->fingerprint();
    }
    if (screenshot_plan_pending && !app.model_loading) {
      screenshot_plan_pending = false;
      if (app.model_load_error.empty() && !app.job.operations.empty())
        start_plan(app, 0, static_cast<int>(app.job.operations.size()) - 1);
    }
    cam::PartialPlan produced;
    if (app.planner.take(produced)) {
      take_plan_result(app, std::move(produced), screenshot_pending);
      if (progress_fraction >= 0.0) {
        const cam::LevelPlan* level = current_level(app);
        app.playback.distance =
            std::clamp(progress_fraction, 0.0, 1.0) * level_length(level);
        sync_playback_time(app);
      }
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - previous_frame).count();
    previous_frame = now;
    advance_playback(app, std::min(elapsed, 0.1));
    advance_camera(app, std::min(elapsed, 0.1));
    if (app.has_plan) refresh_material(app);
    // Once a frame, before anything reads it, so panels cannot disagree.
    refresh_plan_states(app);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();
    // A press that landed on the view cube blocks orbiting until it is let go.
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left) &&
        !ImGui::IsMouseReleased(ImGuiMouseButton_Left))
      app.orbit.cube_pressed = false;
    draw_main_window(app);
    if (app.quit) running = false;
    if (const std::string wanted = window_title(app); wanted != title) {
      title = wanted;
      SDL_SetWindowTitle(window, title.c_str());
    }

    ImGui::Render();
    int width = 0;
    int height = 0;
    SDL_GL_GetDrawableSize(window, &width, &height);
    glViewport(0, 0, width, height);
    glClearColor(0.03f, 0.04f, 0.05f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    if (screenshot_pending && (app.model_loading || app.planner.running() || app.surface_previewer.running())) {
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
  app.model_loader.cancel();
  release_render_resources();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return EXIT_SUCCESS;
}
