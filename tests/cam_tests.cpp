#include "cam/config.h"
#include "cam/feeds.h"
#include "cam/job.h"
#include "cam/post.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {

int failures = 0;

void require(bool condition, const std::string& message) {
  if (condition) return;
  std::cerr << "  FAILED: " << message << '\n';
  ++failures;
}

void require_near(double actual, double expected, double tolerance,
                  const std::string& message) {
  if (std::abs(actual - expected) <= tolerance) return;
  std::cerr << "  FAILED: " << message << " (got " << actual
            << ", wanted " << expected << ")\n";
  ++failures;
}

cam::Machine test_machine() {
  cam::Machine machine;
  machine.id = "test";
  machine.name = "Test machine";
  machine.controller = cam::Controller::LinuxCnc;
  machine.rapid_feed = 6000.0;
  machine.min_rpm = 1000.0;
  machine.max_rpm = 24000.0;
  machine.rpm_resolution = 100.0;
  machine.feed_ranges.push_back({"plastic", 500.0, 2000.0});
  return machine;
}

cam::Tool test_tool() {
  cam::Tool tool;
  tool.id = "test-tool";
  tool.name = "6 mm test tool";
  tool.diameter = 6.0;
  tool.flute_length = 16.0;
  cam::CuttingData data;
  data.material_id = "plastic";
  data.spindle_rpm = 12000.0;
  data.max_ae_over_d = 1.0;
  data.max_ap_over_d = 2.0;
  data.plunge_feed = 120.0;
  tool.cutting_data.push_back(data);
  return tool;
}

wave::Point64 point(double x, double y = 0.0) {
  return wave::Point64(wave::to_integer(x), wave::to_integer(y));
}

struct SyntheticCut {
  wave::Toolpath toolpath;
  wave::SimulationResult simulation;
};

SyntheticCut straight_cut(const std::vector<double>& loads) {
  SyntheticCut cut;
  wave::ToolpathMove move;
  move.kind = wave::MoveKind::Cut;
  move.path = {point(0.0), point(static_cast<double>(loads.size()))};
  move.length = static_cast<double>(loads.size());
  cut.toolpath.moves.push_back(move);
  cut.simulation.moves.resize(1);
  for (std::size_t index = 0; index < loads.size(); ++index) {
    wave::MaterialStep step;
    step.from = point(static_cast<double>(index));
    step.to = point(static_cast<double>(index + 1));
    step.move_distance_begin = static_cast<double>(index);
    step.move_distance_end = static_cast<double>(index + 1);
    step.load = loads[index];
    step.removed_area = loads[index] * 6.0;
    cut.simulation.moves[0].steps.push_back(step);
  }
  return cut;
}

void check_engagement_feeds() {
  std::cout << "engagement feed mapping\n";
  for (const auto [load, expected] :
       std::vector<std::pair<double, double>>{{1.0, 500.0},
                                              {0.5, 1250.0},
                                              {0.0, 2000.0},
                                              {1.5, 500.0}}) {
    const SyntheticCut cut = straight_cut({load});
    const auto schedule = cam::plan_feeds(
        cut.toolpath, cut.simulation, test_machine(), 500.0, 2000.0,
        12000.0, 120.0);
    require(schedule.segments.size() == 1, "one sample produces one feed segment");
    if (!schedule.segments.empty())
      require_near(schedule.segments.front().feed, expected, 1e-9,
                   "feed is the clamped linear engagement interpolation");
  }
}

void check_feed_compatible_merge() {
  std::cout << "post-feed line merging\n";
  const SyntheticCut equal = straight_cut({0.5, 0.5, 0.5, 0.5});
  const auto merged = cam::plan_feeds(
      equal.toolpath, equal.simulation, test_machine(), 500.0, 2000.0,
      12000.0, 120.0);
  require(merged.segments.size() == 1,
          "collinear segments at one feed merge after assignment");
  if (!merged.segments.empty())
    require_near(merged.segments.front().length, 4.0, 1e-9,
                 "merging preserves travelled distance");

  const SyntheticCut different = straight_cut({0.5, 0.5, 0.2, 0.2});
  const auto split = cam::plan_feeds(
      different.toolpath, different.simulation, test_machine(), 500.0, 2000.0,
      12000.0, 120.0);
  require(split.segments.size() == 2,
          "a feed change prevents a collinear merge");
}

void check_path_resampling() {
  std::cout << "path pitch\n";
  wave::Toolpath raw;
  wave::ToolpathMove first;
  first.kind = wave::MoveKind::Cut;
  first.path = {point(0.0), point(0.03), point(0.07), point(0.35)};
  first.length = 0.35;
  raw.moves.push_back(first);
  wave::ToolpathMove second;
  second.kind = wave::MoveKind::Step;
  second.path = {point(0.35), point(1.0)};
  second.length = 0.65;
  raw.moves.push_back(second);

  const wave::Toolpath sampled = wave::resample_toolpath(raw, 0.2);
  require(!sampled.moves.empty(), "resampling retains a cutting path");
  require(sampled.moves.front().path.front() == point(0.0),
          "resampling retains the path start");
  require(sampled.moves.back().path.back() ==
              point(1.0),
          "resampling retains the path endpoint");
  bool retained_micro_vertex = false;
  for (const auto& move : sampled.moves)
    for (const auto& point : move.path)
      if (point.x == wave::to_integer(0.03) ||
          point.x == wave::to_integer(0.07) ||
          point.x == wave::to_integer(0.35))
        retained_micro_vertex = true;
  require(!retained_micro_vertex,
          "original micro-vertices are not forced into the normalized path");
  require_near(wave::toolpath_length(sampled), 1.0, 1e-6,
               "uniform sampling preserves a straight path's length");
  for (const auto& move : sampled.moves)
    for (std::size_t i = 1; i < move.path.size(); ++i)
      require_near(
          std::hypot(wave::to_world(move.path[i].x - move.path[i - 1].x),
                     wave::to_world(move.path[i].y - move.path[i - 1].y)),
          0.2, 1e-6, "pitch carries across the original move boundary");
}

void check_level_floors() {
  std::cout << "depth levels\n";
  cam::Operation operation;
  operation.top_z = 0.0;
  operation.bottom_z = -5.0;
  operation.stepdown = 2.0;
  const auto floors = cam::level_floors(operation);
  require(floors.size() == 3, "five millimetres in two millimetre steps is three passes");
  if (floors.size() == 3) {
    require_near(floors[0], -2.0, 1e-9, "first pass takes a full step");
    require_near(floors[2], -5.0, 1e-9, "last pass lands on the floor");
  }
  operation.axial_stock_to_leave = 0.5;
  require_near(cam::level_floors(operation).back(), -4.5, 1e-9,
               "axial stock allowance raises the final floor");
}

void check_serialisation() {
  std::cout << "serialisation\n";
  const cam::Tool tool = test_tool();
  const cam::Tool tool_back =
      cam::tool_from_json_text(cam::to_json_text(tool));
  require(tool_back.id == tool.id, "tool identity survives a round trip");
  require(tool_back.cutting_data.size() == 1, "tool cutting data survives");
  if (!tool_back.cutting_data.empty())
    require_near(tool_back.cutting_data.front().spindle_rpm, 12000.0, 1e-9,
                 "tool spindle speed survives");

  const cam::Machine machine = test_machine();
  const cam::Machine machine_back =
      cam::machine_from_json_text(cam::to_json_text(machine));
  require(machine_back.controller == machine.controller,
          "machine controller survives");
  require(machine_back.feed_ranges.size() == 1,
          "machine feed ranges survive");

  cam::Job job;
  cam::Operation operation;
  operation.path_pitch = 0.15;
  operation.manual_feed_range = true;
  operation.full_engagement_feed = 450.0;
  operation.no_engagement_feed = 1900.0;
  job.operations.push_back(operation);
  const cam::Job job_back =
      cam::job_from_json_text(cam::to_json_text(job));
  require(job_back.operations.size() == 1, "job operations survive");
  if (!job_back.operations.empty()) {
    require_near(job_back.operations[0].path_pitch, 0.15, 1e-9,
                 "path pitch survives");
    require(job_back.operations[0].manual_feed_range,
            "manual feed selection survives");
    require_near(job_back.operations[0].no_engagement_feed, 1900.0, 1e-9,
                 "manual feed endpoint survives");
  }

  const cam::Tool sparse = cam::tool_from_json_text(
      R"({"id":"x","name":"X","diameter":8.0,"unheard_of":42})");
  require_near(sparse.diameter, 8.0, 1e-9,
               "recognized fields load while unknown fields are ignored");
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
}

void check_validation() {
  std::cout << "validation\n";
  require(cam::validate(test_tool()).empty(), "the test tool validates");
  cam::Tool bad = test_tool();
  bad.diameter = 0.0;
  require(!cam::validate(bad).empty(), "zero tool diameter is rejected");
  bad = test_tool();
  bad.cutting_data.push_back(bad.cutting_data.front());
  require(!cam::validate(bad).empty(), "duplicate tool material data is rejected");
  require(cam::validate(test_machine()).empty(), "the test machine validates");
}

void check_shipped_data(const std::filesystem::path& data) {
  std::cout << "shipped profiles\n";
  const auto materials = cam::load_materials(data / "materials");
  const auto machines = cam::load_machines(data / "machines");
  const auto tools = cam::load_tools(data / "starter-tools.json");
  require(materials.materials.size() >= 6, "material profiles load");
  require(machines.machines.size() >= 3, "machine profiles load");
  require(tools.tools.size() >= 4, "starter tools load");
  for (const auto& material : materials.materials)
    for (const auto& problem : cam::validate(material))
      require(false, material.id + ": " + problem);
  for (const auto& machine : machines.machines) {
    for (const auto& problem : cam::validate(machine))
      require(false, machine.id + ": " + problem);
    for (const auto& range : machine.feed_ranges)
      require(materials.find(range.material_id) != nullptr,
              machine.id + " names an unknown material feed range");
  }
  for (const auto& tool : tools.tools) {
    for (const auto& problem : cam::validate(tool))
      require(false, tool.id + ": " + problem);
    for (const auto& entry : tool.cutting_data)
      require(materials.find(entry.material_id) != nullptr,
              tool.id + " names an unknown material");
  }
  const cam::Machine* makera = machines.find("makera-z1");
  require(makera != nullptr, "the Makera profile is present");
  if (makera) {
    const cam::MaterialFeedRange* plastic = makera->feeds_for("hdpe");
    require(plastic != nullptr, "the Makera has a plastic feed range");
    if (plastic) {
      require_near(plastic->full_engagement, 500.0, 1e-9,
                   "Makera plastic full-engagement feed");
      require_near(plastic->no_engagement, 2000.0, 1e-9,
                   "Makera plastic no-engagement feed");
    }
  }
}

void check_post_output() {
  std::cout << "post output\n";
  cam::ToolLibrary tools;
  cam::Tool tool = test_tool();
  tool.tool_number = 3;
  tools.upsert(tool);
  cam::MachineLibrary machines;
  cam::Machine machine = test_machine();
  machines.upsert(machine);
  cam::Job job;
  cam::Operation operation;
  operation.name = "Rough";
  operation.tool_id = tool.id;
  operation.machine_id = machine.id;
  operation.material_id = "plastic";
  job.operations.push_back(operation);

  const SyntheticCut cut = straight_cut({0.5, 0.5});
  cam::LevelPlan level;
  level.floor_z = -1.5;
  level.axial_depth = 1.5;
  level.toolpath = cut.toolpath;
  level.simulation = cut.simulation;
  level.feeds = cam::plan_feeds(
      cut.toolpath, cut.simulation, machine, 500.0, 2000.0, 12000.0, 120.0);
  cam::OperationPlan planned_operation;
  planned_operation.operation_name = "Rough";
  planned_operation.tool_id = tool.id;
  planned_operation.spindle_rpm = 12000.0;
  planned_operation.levels.push_back(level);
  cam::JobPlan plan;
  plan.operations.push_back(planned_operation);

  const cam::PostResult result = cam::post_job(plan, job, tools, machines);
  require(result.ok(), "the post succeeds: " + result.error);
  require(result.text.find("M3 S12000") != std::string::npos,
          "the configured spindle speed is posted");
  require(result.text.find("F1250") != std::string::npos,
          "the engagement-derived feed is posted");
  require(result.text.find("M30") != std::string::npos,
          "the program terminates");
}

}  // namespace

int main(int argc, char** argv) {
  try {
    check_engagement_feeds();
    check_feed_compatible_merge();
    check_path_resampling();
    check_level_floors();
    check_serialisation();
    check_schedule_timing();
    check_validation();
    check_post_output();
    if (argc > 1) check_shipped_data(argv[1]);
  } catch (const std::exception& error) {
    std::cerr << "Test threw: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  if (failures > 0) {
    std::cerr << failures << " check(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "All checks passed.\n";
  return EXIT_SUCCESS;
}
