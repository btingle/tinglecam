#include "cam_test_support.h"
namespace cam_test {
void check_operation_stock_handoff() {
  std::cout << "operation stock handoff\n";
  const wave::Paths64 nominal{box(0.0, 0.0, 10.0, 10.0)};

  cam::OperationPlan shallow;
  cam::LevelPlan shallow_level;
  shallow_level.floor_z = -2.0;
  shallow_level.axial_depth = 1.0;
  shallow_level.simulation.removed_material =
      wave::Paths64{box(0.0, 0.0, 2.0, 10.0)};
  shallow.levels.push_back(shallow_level);

  cam::OperationPlan deep;
  cam::LevelPlan deep_level;
  deep_level.floor_z = -4.0;
  deep_level.axial_depth = 2.0;
  deep_level.simulation.removed_material =
      wave::Paths64{box(8.0, 0.0, 10.0, 10.0)};
  deep.levels.push_back(deep_level);

  const cam::PriorOperations operations{&shallow, &deep};
  const wave::Paths64 above =
      cam::stock_after_operations_at_z(nominal, -0.5, operations);
  require_near(area(above), 100.0, 0.001,
               "cuts outside a level's axial slab do not affect that Z");

  const wave::Paths64 shallow_stock =
      cam::stock_after_operations_at_z(nominal, -1.5, operations);
  require_near(area(shallow_stock), 80.0, 0.001,
               "a later operation receives material removed in the shallow slab");
  require(!inside(point(1.0, 5.0), shallow_stock) &&
              inside(point(9.0, 5.0), shallow_stock),
          "the shallow handoff removes only the shallow operation's sweep");

  const wave::Paths64 deep_stock =
      cam::stock_after_operations_at_z(nominal, -3.0, operations);
  require_near(area(deep_stock), 80.0, 0.001,
               "the deeper slab receives its own prior swept material");
  require(inside(point(1.0, 5.0), deep_stock) &&
              !inside(point(9.0, 5.0), deep_stock),
          "stock handoff respects the axial range of each planned level");

  const wave::Paths64 boundary_stock =
      cam::stock_after_operations_at_z(nominal, -2.0, operations);
  require_near(area(boundary_stock), 60.0, 0.001,
               "shared slab boundaries include both physical cutter sweeps");

  cam::OperationPlan failed = shallow;
  failed.error = "deliberate failure";
  const wave::Paths64 after_failure =
      cam::stock_after_operations_at_z(nominal, -1.5, {&failed});
  require_near(area(after_failure), 100.0, 0.001,
               "a failed operation cannot mutate downstream stock");
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
  wave::SimulationResult simulation;
  simulation.moves.resize(2);
  wave::MaterialStep step;
  step.from = point(10.0);
  step.to = point(20.0);
  step.move_distance_end = 10.0;
  step.load = 1.0;
  simulation.moves[1].steps.push_back(step);
  const auto schedule = cam::plan_feeds(
      toolpath, simulation, test_machine(), 500.0, 2000.0, 12000.0, 120.0);
  require(schedule.segments.size() == 2, "rapid and cutting movement are scheduled");
  const double rapid_seconds = 10.0 / 6000.0 * 60.0;
  const double cut_seconds = 10.0 / 500.0 * 60.0;
  require_near(cam::feed_schedule_seconds(schedule),
               rapid_seconds + cut_seconds, 1e-9,
               "machine time uses the final scheduled rates");
  require_near(cam::feed_seconds_to(schedule, 10.0), rapid_seconds, 1e-9,
               "distance converts to time across a rapid");

  const auto vertical = cam::plan_feeds(
      toolpath, simulation, test_machine(), 500.0, 2000.0, 12000.0,
      120.0, -2.0, 10.0);
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
          "a simulation-cleared retract landing returns to depth at rapid");
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
  wave::SimulationResult between_simulation;
  between_simulation.moves.resize(3);
  wave::ToolpathMove first_cut;
  first_cut.kind = wave::MoveKind::Cut;
  first_cut.path = {point(0.0), point(1.0)};
  first_cut.length = 1.0;
  between_paths.moves.push_back(first_cut);
  wave::MaterialStep first_step;
  first_step.from = point(0.0);
  first_step.to = point(1.0);
  first_step.move_distance_end = 1.0;
  between_simulation.moves[0].steps.push_back(first_step);
  wave::ToolpathMove lift_and_cross;
  lift_and_cross.kind = wave::MoveKind::Retract;
  lift_and_cross.path = {point(1.0), point(3.0)};
  lift_and_cross.length = 2.0;
  between_paths.moves.push_back(lift_and_cross);
  wave::ToolpathMove second_cut = first_cut;
  second_cut.path = {point(3.0), point(4.0)};
  between_paths.moves.push_back(second_cut);
  wave::MaterialStep second_step = first_step;
  second_step.from = point(3.0);
  second_step.to = point(4.0);
  between_simulation.moves[2].steps.push_back(second_step);
  const auto transition = cam::plan_feeds(
      between_paths, between_simulation, test_machine(), 500.0, 2000.0,
      12000.0, 120.0, -2.0, 10.0);
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
    const auto slowed = cam::plan_feeds(between_paths, between_simulation,
                                        slow_z, 500.0, 2000.0, 12000.0, 120.0,
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
    const auto same = cam::plan_feeds(between_paths, between_simulation,
                                      unstated, 500.0, 2000.0, 12000.0, 120.0,
                                      -2.0, 10.0);
    require_near(cam::feed_schedule_seconds(same),
                 cam::feed_schedule_seconds(transition), 1e-9,
                 "an unstated Z rate is the lateral rate");
  }

  // The next pass down starts wherever its own geometry does, which is not
  // where the previous one left the cutter. That leg is the plan's to own.
  const auto arrived = cam::plan_feeds(
      between_paths, between_simulation, test_machine(), 500.0, 2000.0,
      12000.0, 120.0, -2.0, 10.0, cam::PassLink{point(0.0, 5.0), 10.0, false});
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

  const SyntheticCut unresolved = straight_cut({1.0});
  const auto unresolved_schedule = cam::plan_feeds(
      unresolved.toolpath, unresolved.simulation, test_machine(), 500.0,
      2000.0, 12000.0, 120.0, -2.0, 10.0);
  require(!unresolved_schedule.motion.empty() &&
              !unresolved_schedule.motion.front().rapid,
          "a cut without a verified cleared retract still uses a plunge");
  if (!unresolved_schedule.motion.empty())
    require_near(unresolved_schedule.motion.front().rate, 120.0, 1e-9,
                 "an unresolved entry retains the configured plunge feed");
}

}  // namespace cam_test
