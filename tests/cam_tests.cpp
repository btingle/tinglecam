#include "cam/config.h"
#include "cam/feeds.h"
#include "cam/job.h"
#include "cam/post.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

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
  std::cerr << "  FAILED: " << message << " (got " << actual << ", wanted "
            << expected << ")\n";
  ++failures;
}

// ---------------------------------------------------------------------------

// A machine with no meaningful limits, so a test can isolate one stage of the
// feed pipeline instead of discovering which of twelve happened to bind.
cam::Machine unlimited_machine() {
  cam::Machine machine;
  machine.id = "test";
  machine.name = "Unlimited";
  machine.controller = cam::Controller::LinuxCnc;
  machine.x = {0.0, 1000.0, 20000.0, 100000.0, 0.0};
  machine.y = machine.x;
  machine.z = machine.x;
  machine.max_cutting_feed = 100000.0;
  machine.block_rate = 100000.0;
  machine.min_rpm = 1.0;
  machine.max_rpm = 100000.0;
  machine.power_curve = {{1.0, 100.0}, {100000.0, 100.0}};
  machine.drivetrain_efficiency = 1.0;
  machine.rpm_resolution = 1.0;
  machine.max_cutting_force = 1e9;
  machine.max_mrr = 1e9;
  machine.decimals = 3;
  return machine;
}

cam::Tool six_millimetre_cutter() {
  cam::Tool tool;
  tool.id = "test-6mm";
  tool.name = "6 mm test";
  tool.diameter = 6.0;
  tool.flutes = 3;
  tool.flute_length = 16.0;
  tool.overall_length = 50.0;
  tool.shank_diameter = 6.0;
  tool.stickout = 18.0;
  tool.substrate = cam::Substrate::Carbide;
  return tool;
}

cam::CuttingData aluminium_data() {
  cam::CuttingData data;
  data.material_id = "aluminium-6082";
  data.surface_speed = 250.0;
  data.feed_per_tooth = 0.05;
  data.min_chip_thickness = 0.0;
  data.max_ae_over_d = 1.0;
  data.max_ap_over_d = 2.0;
  return data;
}

cam::Material aluminium() {
  cam::Material material;
  material.id = "aluminium-6082";
  material.name = "Aluminium";
  material.specific_cutting_energy = 0.7;
  return material;
}

// One straight cut of a given length at a given engagement, sampled into a
// given number of steps. Standing in for the engine so the feed tests do not
// depend on what the planner happens to produce today.
struct SyntheticCut {
  wave::Toolpath toolpath;
  wave::SimulationResult simulation;
};

SyntheticCut straight_cut(double length_mm, double load, std::size_t steps) {
  SyntheticCut cut;
  wave::ToolpathMove move;
  move.kind = wave::MoveKind::Cut;
  move.path.push_back(wave::Point64(std::int64_t{0}, std::int64_t{0}));
  move.path.push_back(wave::Point64(wave::to_integer(length_mm), std::int64_t{0}));
  move.length = length_mm;
  cut.toolpath.moves.push_back(move);
  cut.toolpath.stats.cut_count = 1;
  cut.toolpath.stats.cut_length = length_mm;

  cut.simulation.moves.resize(1);
  cut.simulation.tool_radius = 3.0;
  for (std::size_t i = 0; i < steps; ++i) {
    wave::MaterialStep step;
    const double from = length_mm * static_cast<double>(i) / steps;
    const double to = length_mm * static_cast<double>(i + 1) / steps;
    step.from = wave::Point64(wave::to_integer(from), std::int64_t{0});
    step.to = wave::Point64(wave::to_integer(to), std::int64_t{0});
    step.load = load;
    step.removed_area = load * 6.0 * (to - from);
    cut.simulation.moves[0].steps.push_back(step);
  }
  return cut;
}

// ---------------------------------------------------------------------------

void check_chip_thinning() {
  std::cout << "chip thinning\n";
  require_near(cam::chip_thinning_factor(1.0), 1.0, 1e-9, "slotting is not thinned");
  require_near(cam::chip_thinning_factor(0.5), 1.0, 1e-9,
               "half engagement is the reference");
  // The textbook case: a tenth of the diameter needs two thirds more feed to
  // leave the same chip behind.
  require_near(cam::chip_thinning_factor(0.1), 1.0 / 0.6, 1e-9,
               "a tenth of diameter thins to 0.6");
  require(cam::chip_thinning_factor(0.25) > 1.15,
          "a quarter of diameter should ask for meaningfully more feed");
  require(cam::chip_thinning_factor(0.05) > cam::chip_thinning_factor(0.2),
          "thinning grows as engagement falls");
}

void check_base_feed() {
  std::cout << "base feed\n";
  const cam::Tool tool = six_millimetre_cutter();
  const cam::CuttingData data = aluminium_data();
  const cam::Material material = aluminium();
  cam::Machine machine = unlimited_machine();

  cam::FeedPolicy policy;
  policy.corner_compensation = false;
  policy.enable_power = false;
  policy.enable_force = false;
  policy.enable_deflection = false;
  policy.enable_acceleration = false;
  policy.feed_slew_limit = 0.0;
  policy.lookahead_distance = 0.0;
  policy.min_feed = 0.0;

  const SyntheticCut cut = straight_cut(20.0, 1.0, 4);
  cam::CutContext context;
  context.axial_depth = 1.0;
  const cam::FeedSchedule schedule =
      cam::plan_feeds(cut.toolpath, cut.simulation, tool, machine, material, data,
                      context, policy);

  const double expected_rpm = 1000.0 * 250.0 / (3.14159265358979 * 6.0);
  require_near(schedule.stats.spindle_rpm, expected_rpm, 1.0,
               "spindle speed follows from surface speed and diameter");
  require(!schedule.segments.empty(), "a cut should produce segments");

  // Slotting: no thinning, so the feed is exactly the book chip load.
  const double expected_feed = 0.05 * 3.0 * schedule.stats.spindle_rpm;
  for (const auto& segment : schedule.segments)
    require_near(segment.feed, expected_feed, 1.0,
                 "slotting feed is chip load times teeth times rpm");

  // A quarter engagement asks for the thinning factor more.
  const SyntheticCut light = straight_cut(20.0, 0.25, 4);
  const cam::FeedSchedule thinned =
      cam::plan_feeds(light.toolpath, light.simulation, tool, machine, material, data,
                      context, policy);
  require_near(thinned.segments.front().feed,
               expected_feed * cam::chip_thinning_factor(0.25), 1.0,
               "light engagement is fed faster by the thinning factor");
}

void check_power_limit() {
  std::cout << "spindle power\n";
  const cam::Tool tool = six_millimetre_cutter();
  const cam::CuttingData data = aluminium_data();
  const cam::Material material = aluminium();

  cam::Machine machine = unlimited_machine();
  // A spindle far too small for a full-width six millimetre slot.
  machine.power_curve = {{1.0, 0.05}, {100000.0, 0.05}};
  machine.max_mrr = 1e9;

  cam::FeedPolicy policy;
  policy.corner_compensation = false;
  policy.enable_force = false;
  policy.enable_deflection = false;
  policy.enable_acceleration = false;
  policy.feed_slew_limit = 0.0;
  policy.lookahead_distance = 0.0;
  policy.min_feed = 0.0;
  policy.power_margin = 1.0;

  const SyntheticCut cut = straight_cut(20.0, 1.0, 4);
  cam::CutContext context;
  context.axial_depth = 3.0;
  const cam::FeedSchedule schedule =
      cam::plan_feeds(cut.toolpath, cut.simulation, tool, machine, material, data,
                      context, policy);

  require(schedule.segments.front().limit == cam::FeedLimit::SpindlePower,
          "a 50 watt spindle should be what limits a full-width slot");
  // 50 W at 0.7 J/mm^3 is about 4300 mm^3/min, over a 3 by 6 mm section.
  const double expected = 0.05 * 1000.0 * 60.0 / (3.0 * 6.0 * 0.7);
  require_near(schedule.segments.front().feed, expected, expected * 0.02,
               "the power-limited feed follows from removal rate");
  require(schedule.stats.maximum_power_kw <= 0.051,
          "the planned feed should not exceed the spindle's power");
}

void check_acceleration_limit() {
  std::cout << "acceleration\n";
  const cam::Tool tool = six_millimetre_cutter();
  const cam::CuttingData data = aluminium_data();
  const cam::Material material = aluminium();

  cam::Machine machine = unlimited_machine();
  machine.x.max_acceleration = 200.0;
  machine.y.max_acceleration = 200.0;
  machine.z.max_acceleration = 200.0;

  cam::FeedPolicy policy;
  policy.corner_compensation = false;
  policy.enable_power = false;
  policy.enable_force = false;
  policy.enable_deflection = false;
  policy.feed_slew_limit = 0.0;
  policy.lookahead_distance = 0.0;
  policy.min_feed = 0.0;

  // A very short cut cannot be got up to speed, however light it is.
  const SyntheticCut brief = straight_cut(0.4, 0.1, 1);
  cam::CutContext context;
  const cam::FeedSchedule schedule =
      cam::plan_feeds(brief.toolpath, brief.simulation, tool, machine, material, data,
                      context, policy);
  require(schedule.segments.front().limit == cam::FeedLimit::Acceleration,
          "a short move should be limited by what the machine can reach");
  require_near(schedule.segments.front().feed, std::sqrt(200.0 * 0.4) * 60.0, 1.0,
               "the reachable feed follows from acceleration over the run");

  // The same total distance in one straight run is not limited, because the
  // machine never has to stop in the middle of it.
  const SyntheticCut ample = straight_cut(60.0, 0.1, 60);
  const cam::FeedSchedule fast =
      cam::plan_feeds(ample.toolpath, ample.simulation, tool, machine, material, data,
                      context, policy);
  require(fast.segments.front().limit != cam::FeedLimit::Acceleration,
          "a long straight run should not be acceleration limited");
}

void check_block_rate() {
  std::cout << "block rate\n";
  const cam::Tool tool = six_millimetre_cutter();
  const cam::CuttingData data = aluminium_data();
  const cam::Material material = aluminium();

  cam::Machine machine = unlimited_machine();
  machine.block_rate = 50.0;

  cam::FeedPolicy policy;
  policy.corner_compensation = false;
  policy.enable_power = false;
  policy.enable_force = false;
  policy.enable_deflection = false;
  policy.enable_acceleration = false;
  policy.feed_slew_limit = 0.0;
  policy.lookahead_distance = 0.0;
  policy.min_feed = 0.0;

  const SyntheticCut cut = straight_cut(10.0, 0.15, 100);
  cam::CutContext context;

  // With merging switched off, a hundred blocks over ten millimetres is a tenth
  // of a millimetre each, and fifty of those a second is 300 mm/min whatever
  // the axes could otherwise do.
  policy.path_tolerance = 0.0;
  const cam::FeedSchedule raw =
      cam::plan_feeds(cut.toolpath, cut.simulation, tool, machine, material, data,
                      context, policy);
  require(raw.segments.front().limit == cam::FeedLimit::BlockRate,
          "short blocks on a slow controller should bind on block rate");
  require_near(raw.segments.front().feed, 0.1 * 50.0 * 60.0, 1.0,
               "the block-rate feed is segment length times block rate");

  // Merging is what rescues it: the blocks are collinear, so they collapse into
  // far fewer and much longer ones and the controller stops being the limit.
  // Without this a path would be throttled by its own sampling resolution.
  policy.path_tolerance = 0.01;
  const cam::FeedSchedule merged =
      cam::plan_feeds(cut.toolpath, cut.simulation, tool, machine, material, data,
                      context, policy);
  require(merged.segments.size() < raw.segments.size() / 4,
          "collinear blocks should collapse into far fewer");
  require(merged.segments.front().limit != cam::FeedLimit::BlockRate,
          "merged blocks should no longer bind on block rate");
  double merged_length = 0.0;
  for (const auto& segment : merged.segments) merged_length += segment.length;
  require_near(merged_length, 10.0, 0.01,
               "merging must not change how far the tool travels");
}

void check_deflection_model() {
  std::cout << "deflection\n";
  cam::Tool tool = six_millimetre_cutter();
  const double at_18 = cam::tip_deflection(tool, 100.0);
  tool.stickout = 36.0;
  const double at_36 = cam::tip_deflection(tool, 100.0);
  require_near(at_36 / at_18, 8.0, 0.01,
               "doubling the stickout should deflect eight times as much");

  cam::Tool steel = six_millimetre_cutter();
  steel.substrate = cam::Substrate::HighSpeedSteel;
  require(cam::tip_deflection(steel, 100.0) > at_18 * 2.0,
          "a steel cutter should be markedly less stiff than carbide");

  // A ball nose engaged at its tip is nothing like six millimetres across.
  cam::Tool ball = six_millimetre_cutter();
  ball.type = cam::ToolType::BallEndmill;
  ball.corner_radius = 3.0;
  require(cam::effective_diameter(ball, 0.5) < 4.0,
          "a ball nose barely engaged presents a small effective diameter");
  require_near(cam::effective_diameter(ball, 3.0), 6.0, 1e-6,
              "at full radius depth a ball nose is its nominal diameter");
}

void check_level_floors() {
  std::cout << "depth levels\n";
  cam::Operation operation;
  operation.top_z = 0.0;
  operation.bottom_z = -5.0;
  operation.stepdown = 2.0;
  operation.axial_stock_to_leave = 0.0;
  const auto floors = cam::level_floors(operation);
  require(floors.size() == 3, "five millimetres in two millimetre steps is three passes");
  if (floors.size() == 3) {
    require_near(floors[0], -2.0, 1e-9, "first pass takes a full step");
    require_near(floors[1], -4.0, 1e-9, "second pass takes a full step");
    require_near(floors[2], -5.0, 1e-9, "the last pass lands exactly on the floor");
  }

  operation.axial_stock_to_leave = 0.5;
  const auto with_allowance = cam::level_floors(operation);
  require(!with_allowance.empty() &&
              std::abs(with_allowance.back() + 4.5) < 1e-9,
          "an axial allowance stops the roughing short of the floor");

  operation.top_z = -5.0;
  operation.bottom_z = -5.0;
  require(cam::level_floors(operation).empty(),
          "an empty depth range produces no passes");
}

void check_serialisation() {
  std::cout << "serialisation\n";
  cam::Tool tool = six_millimetre_cutter();
  tool.cutting_data.push_back(aluminium_data());
  tool.coating = cam::Coating::AlTiN;
  tool.type = cam::ToolType::BullNose;
  tool.corner_radius = 0.5;
  const cam::Tool round_tripped = cam::tool_from_json_text(cam::to_json_text(tool));
  require(round_tripped.id == tool.id, "tool identity survives a round trip");
  require(round_tripped.coating == tool.coating, "coating survives a round trip");
  require(round_tripped.type == tool.type, "type survives a round trip");
  require_near(round_tripped.corner_radius, 0.5, 1e-9, "corner radius survives");
  require(round_tripped.cutting_data.size() == 1, "cutting data survives");
  if (!round_tripped.cutting_data.empty())
    require_near(round_tripped.cutting_data[0].feed_per_tooth, 0.05, 1e-9,
                 "feed per tooth survives a round trip");

  const cam::Machine machine = unlimited_machine();
  const cam::Machine machine_back =
      cam::machine_from_json_text(cam::to_json_text(machine));
  require(machine_back.controller == machine.controller, "controller survives");
  require_near(machine_back.x.max_acceleration, machine.x.max_acceleration, 1e-9,
               "axis limits survive");
  require(machine_back.power_curve.size() == machine.power_curve.size(),
          "the power curve survives");

  cam::Job job;
  job.name = "Round trip";
  job.clearance_z = 12.5;
  cam::Operation operation;
  operation.name = "Rough";
  operation.tool_id = "test-6mm";
  operation.stepdown = 1.25;
  operation.feeds.power_margin = 0.65;
  job.operations.push_back(operation);
  const cam::Job job_back = cam::job_from_json_text(cam::to_json_text(job));
  require(job_back.name == job.name, "job name survives");
  require_near(job_back.clearance_z, 12.5, 1e-9, "clearance survives");
  require(job_back.operations.size() == 1, "operations survive");
  if (!job_back.operations.empty()) {
    require_near(job_back.operations[0].stepdown, 1.25, 1e-9, "stepdown survives");
    require_near(job_back.operations[0].feeds.power_margin, 0.65, 1e-9,
                 "the feed policy survives");
  }

  // An unknown field is ignored and a missing one keeps its default, so a file
  // from another version still loads.
  const cam::Tool sparse = cam::tool_from_json_text(
      R"({"id":"x","name":"X","diameter":8.0,"unheard_of":42})");
  require(sparse.diameter == 8.0, "a sparse tool reads what is there");
  require(sparse.flutes == 2, "a missing field keeps its default");
}

void check_validation() {
  std::cout << "validation\n";
  cam::Tool tool;
  tool.id = "x";
  tool.name = "X";
  tool.cutting_data.push_back(aluminium_data());
  require(cam::validate(tool).empty(), "a plain tool should validate");

  cam::Tool bad = tool;
  bad.diameter = 0.0;
  require(!cam::validate(bad).empty(), "a zero diameter should be rejected");

  bad = tool;
  bad.type = cam::ToolType::BallEndmill;
  require(!cam::validate(bad).empty(),
          "a ball nose without the matching corner radius should be rejected");

  bad = tool;
  bad.cutting_data[0].min_chip_thickness = 0.2;
  require(!cam::validate(bad).empty(),
          "a chip floor above the book feed should be rejected");

  bad = tool;
  bad.cutting_data.push_back(aluminium_data());
  require(!cam::validate(bad).empty(), "duplicate cutting data should be rejected");
}

void check_shipped_data(const std::filesystem::path& data) {
  std::cout << "shipped profiles\n";
  const auto machines = cam::load_machines(data / "machines");
  require(machines.machines.size() >= 3, "the machine profiles should load");
  for (const auto& machine : machines.machines) {
    const auto problems = cam::validate(machine);
    for (const auto& problem : problems)
      require(false, machine.id + ": " + problem);
    require(cam::spindle_power(machine, machine.max_rpm) > 0.0,
            machine.id + " should have power at its top speed");
    require(cam::spindle_torque(machine, machine.max_rpm) > 0.0,
            machine.id + " should have torque at its top speed");
  }
  require(machines.find("makera-z1") != nullptr, "the Makera profile should be present");
  const cam::Machine* makera = machines.find("makera-z1");
  if (makera)
    require(makera->specifications_unverified,
            "the Makera profile must be marked unverified until its numbers are checked");

  const auto materials = cam::load_materials(data / "materials");
  require(materials.materials.size() >= 6, "the material profiles should load");
  for (const auto& material : materials.materials)
    for (const auto& problem : cam::validate(material))
      require(false, material.id + ": " + problem);

  const auto tools = cam::load_tools(data / "starter-tools.json");
  require(tools.tools.size() >= 4, "the starter tools should load");
  for (const auto& tool : tools.tools) {
    for (const auto& problem : cam::validate(tool))
      require(false, tool.id + ": " + problem);
    for (const auto& entry : tool.cutting_data)
      require(materials.find(entry.material_id) != nullptr,
              tool.id + " names an unknown material: " + entry.material_id);
  }

  // A low-power spindle in steel must come out power limited, which is the
  // whole reason the material carries a specific cutting energy.
  const cam::Machine* router = machines.find("grbl-router");
  const cam::Tool* steel_cutter = tools.find("6mm-4f-steel");
  const cam::Material* steel = materials.find("steel-1018");
  if (router && steel_cutter && steel) {
    const cam::CuttingData* data_for = steel_cutter->data_for("steel-1018");
    require(data_for != nullptr, "the steel cutter should carry steel data");
    if (data_for) {
      const SyntheticCut cut = straight_cut(30.0, 0.4, 30);
      cam::CutContext context;
      context.axial_depth = 3.0;
      const auto schedule =
          cam::plan_feeds(cut.toolpath, cut.simulation, *steel_cutter, *router,
                          *steel, *data_for, context, cam::FeedPolicy{});
      require(schedule.stats.maximum_power_kw <=
                  cam::spindle_power(*router, schedule.stats.spindle_rpm) *
                      router->drivetrain_efficiency * 1.001,
              "a planned cut must stay inside the spindle's power");
      require(!schedule.segments.empty(), "the steel cut should produce segments");
    }
  }
}

void check_post_output() {
  std::cout << "post output\n";
  cam::ToolLibrary tools;
  cam::Tool tool = six_millimetre_cutter();
  tool.tool_number = 3;
  tool.cutting_data.push_back(aluminium_data());
  tools.upsert(tool);

  cam::MachineLibrary machines;
  cam::Machine machine = unlimited_machine();
  machine.id = "post-test";
  machine.name = "Post test";
  machine.tool_change = cam::ToolChange::Manual;
  machines.upsert(machine);

  cam::Job job;
  job.name = "Post test job";
  job.clearance_z = 15.0;
  job.retract_z = 3.0;
  cam::Operation operation;
  operation.name = "Rough";
  operation.tool_id = tool.id;
  operation.machine_id = machine.id;
  operation.material_id = "aluminium-6082";
  job.operations.push_back(operation);

  // A plan assembled by hand, so the post is tested without needing a mesh.
  cam::JobPlan plan;
  cam::OperationPlan op;
  op.operation_name = "Rough";
  op.tool_id = tool.id;
  op.spindle_rpm = 12000.0;
  cam::LevelPlan level;
  level.floor_z = -1.5;
  level.axial_depth = 1.5;
  const SyntheticCut cut = straight_cut(20.0, 0.5, 4);
  level.toolpath = cut.toolpath;
  level.simulation = cut.simulation;
  level.feeds = cam::plan_feeds(cut.toolpath, cut.simulation, tool, machine,
                                aluminium(), aluminium_data(), cam::CutContext{},
                                cam::FeedPolicy{});
  op.levels.push_back(level);
  plan.operations.push_back(op);

  const cam::PostResult result = cam::post_job(plan, job, tools, machines);
  require(result.ok(), "the post should succeed: " + result.error);
  require(result.text.find("G21") != std::string::npos, "millimetre mode is emitted");
  require(result.text.find("G90") != std::string::npos, "absolute mode is emitted");
  require(result.text.find("M3 S12000") != std::string::npos,
          "the spindle is started at the planned speed");
  require(result.text.find("M30") != std::string::npos, "the program ends");
  require(result.text.find("Z-1.5") != std::string::npos,
          "the cut happens at the level's depth");
  require(result.text.find("nan") == std::string::npos &&
              result.text.find("inf") == std::string::npos,
          "no coordinate or feed comes out as a non-number");
  require(result.cutting_moves > 0, "cutting moves are counted");

  // A job whose safe planes sit inside the work must have them lifted rather
  // than posted as written: a plunge to a Z above where the tool already is
  // would drive it up through the part.
  cam::Job unsafe = job;
  unsafe.clearance_z = 0.0;
  unsafe.retract_z = -3.0;  // below the -1.5 floor of the pass above
  const cam::PostResult lifted = cam::post_job(plan, unsafe, tools, machines);
  require(lifted.ok(), "a job with low planes should still post");
  require(lifted.text.find("Z-3") == std::string::npos,
          "the retract plane must not be left inside the cut");
  bool warned_about_planes = false;
  for (const auto& warning : lifted.warnings)
    if (warning.find("clearance") != std::string::npos) warned_about_planes = true;
  require(warned_about_planes, "raising the safe planes should be reported");

  // A machine whose profile is not trusted has to say so in the file itself.
  cam::MachineLibrary flagged = machines;
  flagged.machines[0].specifications_unverified = true;
  const cam::PostResult warned = cam::post_job(plan, job, tools, flagged);
  require(warned.text.find("unverified") != std::string::npos,
          "an unverified profile is called out in the program");
  require(!warned.warnings.empty(), "an unverified profile warns the operator");
}

}  // namespace

int main(int argc, char** argv) {
  try {
    check_chip_thinning();
    check_base_feed();
    check_power_limit();
    check_acceleration_limit();
    check_block_rate();
    check_deflection_model();
    check_level_floors();
    check_serialisation();
    check_validation();
    check_post_output();
    if (argc > 1) check_shipped_data(argv[1]);
    else std::cout << "shipped profiles: skipped, no data directory given\n";
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
