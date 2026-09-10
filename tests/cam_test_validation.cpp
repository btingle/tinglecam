#include "cam_test_support.h"
namespace cam_test {
void check_validation() {
  std::cout << "validation\n";
  require(cam::validate(test_tool()).empty(), "the test tool validates");
  cam::Tool bad = test_tool();
  bad.diameter = 0.0;
  require(!cam::validate(bad).empty(), "zero tool diameter is rejected");
  bad = test_tool();
  bad.flutes = 0;
  require(!cam::validate(bad).empty(), "zero flute count is rejected");
  require(cam::validate(test_machine()).empty(), "the test machine validates");
}

void check_planning_error_details() {
  std::cout << "planning error details\n";
  cam::Job job;
  cam::Operation broken;
  broken.name = "Broken roughing";
  broken.tool_id = "missing-tool";
  job.operations.push_back(broken);

  const cam::JobPlan plan = cam::plan_job(
      cam::JobGeometry{}, job, cam::ToolLibrary{}, cam::MachineLibrary{},
      cam::MaterialLibrary{}, {});
  require(!plan.ok(), "an operation with an unknown tool must fail planning");
  require(plan.error.find("No operation could be planned") != std::string::npos,
          "the job-level planning failure keeps its summary");
  require(plan.error.find("Broken roughing") != std::string::npos,
          "the job-level failure names the operation");
  require(plan.error.find("Unknown tool: missing-tool") != std::string::npos,
          "the job-level failure includes the concrete cause");
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
  tools.upsert(tool);
  cam::MachineLibrary machines;
  cam::Machine machine = test_machine();
  machines.upsert(machine);
  cam::Job job;
  cam::Operation operation;
  operation.name = "Rough";
  operation.tool_number = 3;
  operation.plunge_feed = 120.0;
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
  require(result.text.find("F120") != std::string::npos,
          "a cut without a cleared landing is posted at plunge feed");
  require(result.text.find("M30") != std::string::npos,
          "the program terminates");

  wave::Toolpath safe_path;
  wave::ToolpathMove landing;
  landing.kind = wave::MoveKind::Retract;
  landing.path = {point(0.0), point(0.0)};
  safe_path.moves.push_back(landing);
  wave::ToolpathMove safe_cut;
  safe_cut.kind = wave::MoveKind::Cut;
  safe_cut.path = {point(0.0), point(1.0)};
  safe_cut.length = 1.0;
  safe_path.moves.push_back(safe_cut);
  wave::SimulationResult safe_simulation;
  safe_simulation.moves.resize(2);
  wave::MaterialStep safe_step;
  safe_step.from = point(0.0);
  safe_step.to = point(1.0);
  safe_step.move_distance_end = 1.0;
  safe_step.load = 1.0;
  safe_simulation.moves[1].steps.push_back(safe_step);
  level.toolpath = safe_path;
  level.simulation = safe_simulation;
  level.feeds = cam::plan_feeds(
      safe_path, safe_simulation, machine, 500.0, 2000.0, 12000.0,
      120.0, level.floor_z, job.rapid_clearance_height);
  plan.operations.front().levels.front() = level;
  const cam::PostResult rapid_entry =
      cam::post_job(plan, job, tools, machines);
  require(rapid_entry.ok(), "the safe-entry post succeeds: " + rapid_entry.error);
  require(rapid_entry.text.find("F120") == std::string::npos,
          "a simulation-cleared retract landing returns to depth with G0");
}

}  // namespace cam_test
