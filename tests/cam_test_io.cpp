#include "cam_test_support.h"
namespace cam_test {
void check_user_settings() {
  std::cout << "user settings\n";
  const auto directory = std::filesystem::temp_directory_path() /
      ("tinglecam-settings-" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()));
  const auto file = directory / "settings.json";
  cam::UserSettings settings = cam::default_user_settings();
  settings.ui_scale = 1.35;
  settings.stock_render_scale = .75;
  settings.surface_render_scale = .5;
  settings.ui_theme = cam::UiTheme::Ember;
  settings.custom_theme = "Workshop";
  settings.custom_themes = {{"Workshop", cam::UiFont::Mono, "style-bytes"}};
  settings.panel_layout.left_width = 512.0;
  settings.panel_layout.right_width = 604.0;
  settings.panel_layout.left_setup_fraction = 0.61;
  settings.panel_layout.messages_height = 243.0;
  settings.show_debug = true;
  settings.default_machine = "makera-z1";
  settings.default_material = "hdpe";
  settings.recent_jobs = {"/tmp/a.json", "/tmp/b.json"};
  settings.planning_defaults = {0.1, 0.04, 0.3};
  cam::operation_default(settings, cam::OperationMode::Pocket)
      .pockets.front().width = 47.0;
  cam::operation_default(settings, cam::OperationMode::PlaneClearing)
      .plane.areas.front().depth = 2.5;
  auto& rough_defaults = cam::operation_default(settings, cam::OperationMode::Roughing);
  rough_defaults.depth_breaks = {1.5, 3.0};
  rough_defaults.finishing_subdivisions = 3;
  rough_defaults.cleanup_stepover_limit = 0.6;
  rough_defaults.adaptive_cleanup_stepover = false;
  cam::save_user_settings(file, settings);
  const cam::UserSettings loaded = cam::load_user_settings(file);
  const auto& loaded_rough = cam::operation_default(loaded, cam::OperationMode::Roughing);
  require(loaded_rough.depth_breaks == rough_defaults.depth_breaks &&
              loaded_rough.finishing_subdivisions == 3 &&
              !loaded_rough.adaptive_cleanup_stepover,
          "adaptive roughing defaults survive settings persistence");
  require_near(loaded_rough.cleanup_stepover_limit, 0.6, 1e-12,
               "default cleanup ceiling survives settings persistence");
  require_near(loaded.ui_scale, 1.35, 1e-12,
               "UI scale survives settings persistence");
  require(loaded.planning_defaults.simulation_pitch == 0.1 &&
              loaded.planning_defaults.wave_simplify_tolerance == 0.04 &&
              loaded.planning_defaults.engagement_sampling == 0.3,
          "planning defaults survive settings persistence");
  require_near(loaded.stock_render_scale,.75,1e-12,"Stock render resolution survives settings persistence");
  require_near(loaded.surface_render_scale,.5,1e-12,"Surface render resolution survives settings persistence");
  require(loaded.ui_theme == cam::UiTheme::Ember,
          "UI theme survives settings persistence");
  require(loaded.custom_theme == "Workshop" &&
              loaded.custom_themes.size() == 1 &&
              loaded.custom_themes.front().font == cam::UiFont::Mono &&
              loaded.custom_themes.front().style_blob == "style-bytes",
          "custom theme library survives settings persistence");
  require_near(loaded.panel_layout.left_width, 512.0, 1e-12,
               "left panel width survives settings persistence");
  require_near(loaded.panel_layout.right_width, 604.0, 1e-12,
               "right panel width survives settings persistence");
  require_near(loaded.panel_layout.left_setup_fraction, 0.61, 1e-12,
               "left panel split survives settings persistence");
  require_near(loaded.panel_layout.messages_height, 243.0, 1e-12,
               "message drawer height survives settings persistence");
  require(loaded.show_debug && loaded.default_machine == "makera-z1" &&
              loaded.default_material == "hdpe" &&
              loaded.recent_jobs == settings.recent_jobs,
          "job defaults and recent jobs survive settings persistence");
  require_near(
      cam::operation_default(loaded, cam::OperationMode::Pocket)
          .pockets.front().width,
      47.0, 1e-12, "pocket defaults survive settings persistence");
  require_near(cam::operation_default(loaded,
                                       cam::OperationMode::PlaneClearing)
                    .plane.areas.front().depth,
                2.5, 1e-12, "plane defaults survive settings persistence");
  cam::write_file(file, R"({"ui_scale":1.2})");
  const cam::UserSettings legacy = cam::load_user_settings(file);
  require_near(legacy.ui_scale, 1.2, 1e-12,
               "settings may omit optional operation defaults");
  require(legacy.stock_render_scale==0 && legacy.surface_render_scale==1,
          "Existing settings retain automatic stock and native surface resolution");
  for(const auto invalid:{R"({"stock_render_scale":-0.1})", R"({"stock_render_scale":1.1})",
                          R"({"surface_render_scale":0})"}) {
    bool rejected=false;
    try {cam::write_file(file,invalid);(void)cam::load_user_settings(file);}
    catch(const std::exception&) {rejected=true;}
    require(rejected,"Invalid render resolution is rejected");
  }
  bool rejected = false;
  try {
    cam::write_file(file, R"({"ui_scale":4.0})");
    (void)cam::load_user_settings(file);
  } catch (const std::exception&) {
    rejected = true;
  }
  require(rejected, "invalid UI scale is rejected");
  std::error_code ignored;
  std::filesystem::remove_all(directory, ignored);
}

void check_operation_stock_handoff() {
  std::cout << "operation stock handoff\n";
  const wave::Paths64 nominal{box(0.0, 0.0, 10.0, 10.0)};

  auto stock=wave::Heightmap::from_polygons(nominal,-5,0,.05);
  const wave::MillingCutter cutter{wave::CutterShape::Flat,1,20};
  cam::OperationPlan shallow;
  stock.cut({1,0,-2},{1,10,-2},cutter);
  shallow.stock_after=std::make_shared<const wave::Heightmap>(stock);
  cam::OperationPlan deep;
  stock.cut({9,0,-4},{9,10,-4},cutter);
  deep.stock_after=std::make_shared<const wave::Heightmap>(stock);
  const cam::PriorOperations operations{&shallow,&deep};
  const auto above=cam::stock_after_operations_at_z(nominal,-.5,operations);
  require_near(area(above),60,.001,"Both top-down sweeps remove all material above their floors");
  const auto below=cam::stock_after_operations_at_z(nominal,-3,operations);
  require_near(area(below),80,.001,"Stock remains below the shallow floor");
  require(inside(point(1,5),below)&&!inside(point(9,5),below),
          "Downstream stock follows the actual heightmap columns");
  const auto shallow_conversions=shallow.stock_after->slice_conversions();
  const auto deep_conversions=deep.stock_after->slice_conversions();
  cam::stock_after_operations_at_z(nominal,-.5,operations);
  require(shallow.stock_after->slice_conversions()==shallow_conversions&&deep.stock_after->slice_conversions()==deep_conversions,
          "Repeated layer queries reuse only the latest composed stock polygon");
  cam::OperationPlan failed=shallow;failed.error="deliberate failure";
  require_near(area(cam::stock_after_operations_at_z(nominal,-1,{&failed})),100,.001,
               "A failed operation cannot mutate downstream stock");
}

void check_schedule_timing() {
  std::cout << "schedule timing\n";
  wave::Toolpath toolpath;
  wave::ToolpathMove retract;
  retract.kind = wave::MoveKind::Retract;
  retract.path = {point(0.0), point(10.0)};
  retract.length = 10.0;
  toolpath.moves.push_back(retract);
  wave::ToolpathMove cut;
  cut.kind = wave::MoveKind::Cut;
  cut.path = {point(10.0), point(20.0)};
  cut.length = 10.0;
  toolpath.moves.push_back(cut);
  const auto schedule = cam::plan_feeds(toolpath, test_machine(), test_feeds());
  require(schedule.segments.size() == 2, "rapid and cutting movement are scheduled");
  const double rapid_seconds = 10.0 / 6000.0 * 60.0;
  // Provisional rates: the stock simulation assigns the measured ones.
  const double cut_seconds = 10.0 / 2000.0 * 60.0;
  require_near(cam::feed_schedule_seconds(schedule),
               rapid_seconds + cut_seconds, 1e-9,
               "machine time uses the final scheduled rates");
  require_near(cam::feed_seconds_to(schedule, 10.0), rapid_seconds, 1e-9,
               "distance converts to time across a rapid");

  const auto vertical = cam::plan_feeds(toolpath, test_machine(), test_feeds(), -2.0, 10.0);
  const double descent_seconds = 12.0 / 6000.0 * 60.0;
  const double lift_seconds = 12.0 / 6000.0 * 60.0;
  require(vertical.motion.size() == 4,
          "machine motion includes traverse, plunge, cut, and final lift");
  require_near(cam::feed_schedule_seconds(vertical),
               rapid_seconds + descent_seconds + cut_seconds + lift_seconds,
               1e-9, "machine time includes vertical motion");
  require_near(cam::feed_motion_length(vertical), 44.0, 1e-9,
               "XYZ travel has a counter independent of planar path distance");
  require_near(vertical.stats.rapid_distance, 34.0, 1e-9,
               "travel totals include the traverse, plunge, and final lift");
  require_near(vertical.stats.rapid_minutes,
               rapid_seconds / 60.0 + descent_seconds / 60.0 +
                   lift_seconds / 60.0,
               1e-9, "travel time uses each vertical phase's commanded rate");
  require_near(cam::feed_seconds_to(vertical, 10.0),
               rapid_seconds + descent_seconds, 1e-9,
               "the cutting start is reached only after the rapid descent");

  const cam::MotionCursor halfway_down = cam::feed_motion_at_seconds(
      vertical, rapid_seconds + descent_seconds * 0.5);
  require_near(halfway_down.z, 4.0, 1e-9,
               "playback interpolates the cutter down to the pass");
  require_near(halfway_down.path_distance, 10.0, 1e-9,
               "planar simulation distance pauses during a plunge");
  require_near(halfway_down.phase_distance, 6.0, 1e-9,
               "the active vertical phase reports its travelled clearance");
  require_near(halfway_down.motion_distance, 16.0, 1e-9,
               "the XYZ counter includes rapid and vertical travel");
  require(halfway_down.rapid,
          "a retract landing is offered as a rapid descent for the stock simulation to check");
  require_near(halfway_down.rate, 6000.0, 1e-9,
               "the playback cursor exposes the commanded rapid rate");

  const cam::MotionCursor halfway_up = cam::feed_motion_at_seconds(
      vertical,
      rapid_seconds + descent_seconds + cut_seconds + lift_seconds * 0.5);
  require_near(halfway_up.z, 4.0, 1e-9,
               "playback interpolates the final rapid lift");
  require_near(halfway_up.path_distance, 20.0, 1e-9,
               "planar simulation distance pauses during a retract");

  wave::Toolpath between_paths;
  wave::ToolpathMove first_cut;
  first_cut.kind = wave::MoveKind::Cut;
  first_cut.path = {point(0.0), point(1.0)};
  first_cut.length = 1.0;
  between_paths.moves.push_back(first_cut);
  wave::ToolpathMove lift_and_cross;
  lift_and_cross.kind = wave::MoveKind::Retract;
  lift_and_cross.path = {point(1.0), point(3.0)};
  lift_and_cross.length = 2.0;
  between_paths.moves.push_back(lift_and_cross);
  wave::ToolpathMove second_cut = first_cut;
  second_cut.path = {point(3.0), point(4.0)};
  between_paths.moves.push_back(second_cut);
  const auto transition = cam::plan_feeds(between_paths, test_machine(), test_feeds(), -2.0, 10.0);
  require(transition.motion.size() == 7,
          "a disconnected transition has plunge, cut, lift, traverse, "
          "plunge, cut, and final lift phases");
  if (transition.motion.size() == 7) {
    require(transition.motion[2].rapid &&
                transition.motion[2].from_z == -2.0 &&
                transition.motion[2].to_z == 10.0,
            "an internal retract first rises vertically");
    require(transition.motion[3].rapid &&
                transition.motion[3].from_z == 10.0 &&
                transition.motion[3].to_z == 10.0,
            "the disconnected XY move stays on the clearance plane");
    require(transition.motion[4].rapid &&
                transition.motion[4].from_z == 10.0 &&
                transition.motion[4].to_z == -2.0,
            "a cleared cutter descends rapidly only after reaching the next path");
  }

  // A machine whose Z axis is slower than the others. Only the vertical phases
  // change: a lift is commanded as a rise, a traverse and a descent, never as
  // one diagonal, so each one is timed at the axis that actually moves it.
  {
    cam::Machine slow_z = test_machine();
    slow_z.rapid_feed_z = 1500.0;
    const auto slowed = cam::plan_feeds(between_paths, slow_z, test_feeds(),
                                        -2.0, 10.0);
    require(slowed.motion.size() == transition.motion.size(),
            "a slower Z axis changes the timing, not the moves");
    double vertical = 0.0;
    double lateral = 0.0;
    for (const cam::MotionPhase& phase : slowed.motion) {
      if (!phase.rapid) continue;
      if (phase.from_z != phase.to_z) {
        vertical += phase.length;
        require_near(phase.rate, 1500.0, 1e-9,
                     "a vertical rapid runs at the Z rate");
      } else {
        lateral += phase.length;
        require_near(phase.rate, 6000.0, 1e-9,
                     "a lateral rapid still runs at the lateral rate");
      }
    }
    require(vertical > 0.0 && lateral > 0.0,
            "this pass has both a vertical and a lateral rapid to compare");
    // The vertical distance is unchanged and now takes four times as long.
    require_near(cam::feed_schedule_seconds(slowed) -
                     cam::feed_schedule_seconds(transition),
                 vertical / 1500.0 * 60.0 - vertical / 6000.0 * 60.0, 1e-9,
                 "the whole of the difference is the vertical, reprised at the "
                 "Z rate");
    require_near(slowed.stats.rapid_distance, transition.stats.rapid_distance,
                 1e-9, "and the cutter travels no further for it");
  }

  // A profile that names no Z rate is a machine whose Z keeps up, which is what
  // every profile said before there were two rates.
  {
    cam::Machine unstated = test_machine();
    unstated.rapid_feed_z = 0.0;
    const auto same = cam::plan_feeds(between_paths, unstated, test_feeds(),
                                      -2.0, 10.0);
    require_near(cam::feed_schedule_seconds(same),
                 cam::feed_schedule_seconds(transition), 1e-9,
                 "an unstated Z rate is the lateral rate");
  }

  // The next pass down starts wherever its own geometry does, which is not
  // where the previous one left the cutter. That leg is the plan's to own.
  const auto arrived = cam::plan_feeds(between_paths, test_machine(), test_feeds(), -2.0, 10.0, cam::PassLink{point(0.0, 5.0), 10.0, false});
  require(arrived.motion.size() == transition.motion.size() + 1,
          "arriving from a previous pass adds one phase");
  if (!arrived.motion.empty()) {
    const cam::MotionPhase& opening = arrived.motion.front();
    require(opening.arrival && opening.rapid,
            "the pass opens with the rapid in from the previous one");
    require(opening.from == point(0.0, 5.0) &&
                opening.to == transition.motion.front().from,
            "the arrival runs from where the cutter was left to where the "
            "pass begins");
    require(opening.from_z == 10.0 && opening.to_z == 10.0,
            "the arrival stays on the clearance plane");
    require_near(opening.path_distance_begin, opening.path_distance_end, 1e-12,
                 "arriving covers no planar path distance");
    require_near(cam::feed_schedule_seconds(arrived) -
                     cam::feed_schedule_seconds(transition),
                 5.0 / 6000.0 * 60.0, 1e-9,
                 "the arrival is timed at the machine rapid");
    require_near(arrived.stats.rapid_distance - transition.stats.rapid_distance,
                 5.0, 1e-9, "the arrival counts as travel");
  }
  require(cam::feed_motion_at_seconds(arrived, 0.0).position == point(0.0, 5.0),
          "playback starts the pass where the previous one finished");

  const auto unresolved_schedule = cam::plan_feeds(
      straight_cut(1), test_machine(), test_feeds(), -2.0, 10.0);
  require(!unresolved_schedule.motion.empty() &&
              !unresolved_schedule.motion.front().rapid,
          "a cut without a verified cleared retract still uses a plunge");
  if (!unresolved_schedule.motion.empty())
    require_near(unresolved_schedule.motion.front().rate, 120.0, 1e-9,
                 "an unresolved entry retains the configured plunge feed");
}

}  // namespace cam_test
