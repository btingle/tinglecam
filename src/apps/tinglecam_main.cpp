#include "../gui/gui_internal.h"

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

  cam::OperationMode requested_mode = cam::OperationMode::Roughing;
  if (!mode_name.empty()) {
    if (mode_name == "rough") requested_mode = cam::OperationMode::Roughing;
    else if (mode_name == "finish")
      requested_mode = cam::OperationMode::ContourFinishing;
    else if (mode_name == "pocket")
      requested_mode = cam::OperationMode::Pocket;
    else if (mode_name == "plane")
      requested_mode = cam::OperationMode::PlaneClearing;
    else {
      std::fprintf(stderr, "--mode must be rough, finish, pocket or plane\n");
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
  apply_ui_scale(app);
  ImGui_ImplSDL2_InitForOpenGL(window, context);
  ImGui_ImplOpenGL3_Init("#version 150");

  bool running = true;
  // Declared above SDL initialisation, which needs to know.
  int screenshot_delay = 0;
  auto previous_frame = std::chrono::steady_clock::now();
  if (screenshot_pending && !app.job.operations.empty())
    start_plan(app, 0, static_cast<int>(app.job.operations.size()) - 1);
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

    cam::PartialPlan produced;
    if (app.planner.take(produced)) {
      app.stock_cache_operation = -1;
      // A run that could not be made -- cancelled, or overtaken by an edit to
      // the list -- changes nothing. The plan on screen is still the plan.
      if (!produced.error.empty()) {
        if (produced.error != "Cancelled") {
          std::fprintf(stderr, "Planning failed:\n%s\n", produced.error.c_str());
          say(app, produced.error, true);
        } else {
          say(app, "Planning cancelled");
        }
      } else {
        // Each planned operation takes the place of its own entry and nothing
        // else. The signature recorded is the one taken when the run was asked
        // for, so an operation edited while it ran comes back marked edited
        // rather than pretending the plan describes the new settings.
        const std::uint64_t serial = ++app.plan_serial;
        std::size_t planned_count = 0;
        for (std::size_t i = 0; i < produced.operations.size(); ++i) {
          const std::size_t at = produced.first + i;
          if (at >= app.plan.operations.size()) break;
          app.plan.operations[at] = std::move(produced.operations[i]);
          app.planned[at] = {i < app.planning_signatures.size()
                                 ? app.planning_signatures[i]
                                 : std::string(),
                             serial};
          if (app.plan.operations[at].ok()) ++planned_count;
        }
        app.plan.rapid_plane = produced.rapid_plane;
        cam::summarise_job(app.plan);
        log_warnings("Planning", app.plan.warnings);
        refresh_plan_states(app);
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
        if (planned_count == 0) {
          std::fprintf(stderr, "Planning failed:\n%s\n", app.plan.error.c_str());
          say(app, app.plan.error.empty()
                       ? "No operation in that range could be planned"
                       : app.plan.error,
              true);
        } else if (screenshot_pending) {
          // The view was chosen on the command line; leave it alone.
        } else {
          app.force_tab = 2;
          say(app, "Planned " + std::to_string(app.plan.stats.cut_levels) +
                       " passes");
        }
      }
      app.planning_signatures.clear();
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - previous_frame).count();
    previous_frame = now;
    advance_playback(app, std::min(elapsed, 0.1));
    advance_camera(app, std::min(elapsed, 0.1));
    if (app.has_plan) refresh_material(app);
    // Once a frame, before anything reads it: three panels ask which
    // operations the plan still describes, and they must not be able to
    // disagree within one frame.
    refresh_plan_states(app);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    // A press that landed on the view cube blocks orbiting until it is let go.
    // The release is normally consumed by the cube itself, but the view it was
    // in may have been switched away from in the meantime, and a press nobody
    // is left to release would block orbiting for good.
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left) &&
        !ImGui::IsMouseReleased(ImGuiMouseButton_Left))
      app.orbit.cube_pressed = false;

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::Begin("TingleCAM", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);
    const float content_width = ImGui::GetContentRegionAvail().x;
    const float content_height = ImGui::GetContentRegionAvail().y;
    const float splitter_width = ui_size(app, 6.0f);
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    const float usable_width = std::max(
        1.0f, content_width - 2.0f * splitter_width - 4.0f * spacing);
    const float minimum_left = ui_size(app, 260.0f);
    const float minimum_centre = ui_size(app, 200.0f);
    const float minimum_right = ui_size(app, 280.0f);
    const float maximum_left =
        std::max(minimum_left, usable_width - minimum_centre - minimum_right);
    float left_panel_width = std::clamp(
        ui_size(app, static_cast<float>(app.settings.panel_layout.left_width)),
        minimum_left, maximum_left);
    const float maximum_right =
        std::max(minimum_right, usable_width - minimum_centre - left_panel_width);
    float right_panel_width = std::clamp(
        ui_size(app, static_cast<float>(app.settings.panel_layout.right_width)),
        minimum_right, maximum_right);
    float centre_width = std::max(
        1.0f, usable_width - left_panel_width - right_panel_width);

    ImGui::BeginChild("left-panel", ImVec2(left_panel_width, content_height));
    ImGui::PushItemWidth(left_panel_width * 0.48f);
    left_panel(app);
    ImGui::PopItemWidth();
    ImGui::EndChild();
    ImGui::SameLine();
    float delta = 0.0f;
    layout_splitter(app, "left-column-splitter", true, content_height, delta);
    if (delta != 0.0f) {
      app.settings.panel_layout.left_width = std::clamp(
          app.settings.panel_layout.left_width +
              delta / static_cast<float>(app.settings.ui_scale),
          260.0, 1200.0);
    }
    ImGui::SameLine();
    ImGui::BeginChild("views", ImVec2(centre_width, content_height));
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
    delta = 0.0f;
    layout_splitter(app, "right-column-splitter", true, content_height, delta);
    if (delta != 0.0f) {
      app.settings.panel_layout.right_width = std::clamp(
          app.settings.panel_layout.right_width -
              delta / static_cast<float>(app.settings.ui_scale),
          280.0, 1400.0);
    }
    ImGui::SameLine();
    ImGui::BeginChild("right-panel", ImVec2(right_panel_width, content_height));
    ImGui::PushItemWidth(right_panel_width * 0.44f);
    right_panel(app);
    ImGui::PopItemWidth();
    ImGui::EndChild();
    ImGui::End();
    if (app.panel_layout_dragging &&
        !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
      save_settings(app);
      app.panel_layout_dragging = false;
    }
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
  release_prism_view();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return EXIT_SUCCESS;
}
