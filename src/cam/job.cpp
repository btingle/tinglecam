#include "cam/job.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cam {
namespace {

using Clipper2Lib::FillRule;
using Clipper2Lib::InflatePaths;
using Clipper2Lib::JoinType;
using Clipper2Lib::EndType;
using Clipper2Lib::Union;

wave::Paths64 offset_outward(const wave::Paths64& input, double amount) {
  wave::Paths64 result = Union(input, FillRule::NonZero);
  const std::int64_t delta = wave::to_integer(amount);
  if (delta == 0) return result;
  result = InflatePaths(result, static_cast<double>(delta), JoinType::Round,
                        EndType::Polygon);
  return Union(result, FillRule::NonZero);
}

// A subject far outside the stock, used when nothing of the part reaches this
// depth. The wave needs something to propagate against, and a marker parked
// beyond the work makes the whole level clear away.
wave::Paths64 absent_subject(const wave::Paths64& stock) {
  const auto bounds = Clipper2Lib::GetBounds(stock);
  const std::int64_t span =
      std::max<std::int64_t>(1000, std::max(bounds.right - bounds.left,
                                            bounds.bottom - bounds.top));
  const std::int64_t left = bounds.right + span;
  const std::int64_t top = bounds.bottom + span;
  return wave::Paths64{{{left, top},
                        {left + span, top},
                        {left + span, top + span},
                        {left, top + span}}};
}

void accumulate(PlanStats& into, const PlanStats& from) {
  into.levels += from.levels;
  into.cut_levels += from.cut_levels;
  into.cutting_distance += from.cutting_distance;
  into.rapid_distance += from.rapid_distance;
  into.cutting_minutes += from.cutting_minutes;
  into.rapid_minutes += from.rapid_minutes;
  into.tool_change_minutes += from.tool_change_minutes;
  into.removed_volume += from.removed_volume;
  into.gouge_area += from.gouge_area;
  into.unsafe_plunges += from.unsafe_plunges;
  into.unresolved_entries += from.unresolved_entries;
  into.maximum_load = std::max(into.maximum_load, from.maximum_load);
  into.maximum_power_kw = std::max(into.maximum_power_kw, from.maximum_power_kw);
  into.maximum_force = std::max(into.maximum_force, from.maximum_force);
  into.maximum_deflection = std::max(into.maximum_deflection, from.maximum_deflection);
}

}  // namespace

const char* operation_type_name(OperationType value) {
  switch (value) {
    case OperationType::AdaptiveRough: return "adaptive-rough";
    case OperationType::Contour: return "contour";
    case OperationType::Face: return "face";
  }
  return "unknown";
}

std::optional<OperationType> operation_type_from_name(const std::string& text) {
  if (text == "adaptive-rough") return OperationType::AdaptiveRough;
  if (text == "contour") return OperationType::Contour;
  if (text == "face") return OperationType::Face;
  return std::nullopt;
}

void JobGeometry::load_part(const std::filesystem::path& file) {
  part_.set_mesh(wave::load_triangle_mesh(file));
}

void JobGeometry::load_stock(const std::filesystem::path& file) {
  stock_.set_mesh(wave::load_triangle_mesh(file));
}

bool JobGeometry::z_range(double& low, double& high) const {
  // The overlap, not the union. A depth where only one of the two models exists
  // has either no stock to remove or no part to protect, and offering it as a
  // default only produces passes that find nothing to cut.
  bool any = false;
  double lowest = 0.0;
  double highest = 0.0;
  for (const wave::MeshSlicer* slicer : {&part_, &stock_}) {
    if (slicer->empty()) continue;
    const auto& bounds = slicer->mesh().bounds;
    if (!bounds.valid) continue;
    if (!any) {
      lowest = bounds.lower.z;
      highest = bounds.upper.z;
      any = true;
      continue;
    }
    lowest = std::max<double>(lowest, bounds.lower.z);
    highest = std::min<double>(highest, bounds.upper.z);
  }
  if (!any || !(highest > lowest)) return false;
  low = lowest;
  high = highest;
  return true;
}

std::vector<double> level_floors(const Operation& operation) {
  std::vector<double> floors;
  const double bottom = operation.bottom_z + operation.axial_stock_to_leave;
  if (!(operation.top_z > bottom)) return floors;
  const double step = std::max(operation.stepdown, 1e-6);
  double z = operation.top_z;
  // A guard rather than a limit: at a sane stepdown no real part gets close.
  for (int guard = 0; guard < 20000; ++guard) {
    z -= step;
    if (z <= bottom + 1e-9) {
      floors.push_back(bottom);
      break;
    }
    floors.push_back(z);
  }
  return floors;
}

OperationPlan plan_operation(const JobGeometry& geometry,
                             const Operation& operation,
                             const Tool& tool,
                             const Machine& machine,
                             const Material& material,
                             const ProgressCallback& progress,
                             double progress_from,
                             double progress_to) {
  OperationPlan plan;
  plan.operation_name = operation.name;
  plan.tool_id = operation.tool_id;

  if (!geometry.has_part()) {
    plan.error = "No part mesh is loaded";
    return plan;
  }
  if (!geometry.has_stock()) {
    plan.error = "No stock mesh is loaded";
    return plan;
  }
  const CuttingData* data = tool.data_for(operation.material_id);
  if (!data) {
    plan.error = "Tool " + tool.name + " has no cutting data for " +
                 operation.material_id;
    return plan;
  }

  const double radius = tool.diameter * 0.5;

  // Engagement, capped by what the tool says it will take. Exceeding the book
  // figure silently is how a plan looks fine and cuts badly, so the ceiling is
  // applied here and reported rather than left to the operator to remember.
  double stepover_ratio = operation.stepover_ratio;
  if (data->max_ae_over_d > 0.0 && stepover_ratio > data->max_ae_over_d) {
    plan.warnings.push_back(
        "Stepover reduced to the tool's ceiling of " +
        std::to_string(data->max_ae_over_d) + " of diameter");
    stepover_ratio = data->max_ae_over_d;
  }
  const double stepover = std::max(stepover_ratio * tool.diameter, 1e-4);

  double stepdown = operation.stepdown;
  const double axial_ceiling =
      std::min(tool.flute_length, data->max_ap_over_d > 0.0
                                      ? data->max_ap_over_d * tool.diameter
                                      : tool.flute_length);
  if (axial_ceiling > 0.0 && stepdown > axial_ceiling) {
    plan.warnings.push_back("Stepdown reduced to " + std::to_string(axial_ceiling) +
                            " mm, the deepest this tool takes in one pass");
    stepdown = axial_ceiling;
  }

  Operation effective = operation;
  effective.stepdown = stepdown;
  const std::vector<double> floors = level_floors(effective);
  if (floors.empty()) {
    plan.error = "The depth range is empty: the top is not above the floor";
    return plan;
  }

  plan.levels.reserve(floors.size());
  double previous_z = operation.top_z;
  const double span = std::max(progress_to - progress_from, 0.0);

  for (std::size_t index = 0; index < floors.size(); ++index) {
    const double floor_z = floors[index];
    if (progress) {
      const double fraction =
          progress_from + span * (static_cast<double>(index) / floors.size());
      if (!progress(fraction, operation.name + ": depth " +
                                  std::to_string(floor_z) + " mm")) {
        plan.error = "Cancelled";
        return plan;
      }
    }

    LevelPlan level;
    level.floor_z = floor_z;
    level.axial_depth = previous_z - floor_z;
    previous_z = floor_z;

    try {
      // The slicer returns the downward envelope: everything at or above this
      // depth, projected. That is exactly the material that has to survive the
      // pass, so no separate rest model is needed for roughing.
      const auto part_slice =
          geometry.part().slice(static_cast<float>(floor_z), operation.slice_grid);
      const auto stock_slice =
          geometry.stock().slice(static_cast<float>(floor_z), operation.slice_grid);
      level.part_slice = Union(part_slice.polygons, FillRule::NonZero);
      level.stock_slice = Union(stock_slice.polygons, FillRule::NonZero);

      if (level.stock_slice.empty()) {
        level.nothing_to_cut = true;
        plan.levels.push_back(std::move(level));
        continue;
      }

      // The cutter follows a path whose centre must stay a radius clear of the
      // part, plus whatever is being left for a finishing pass.
      const wave::Paths64 keep_out =
          level.part_slice.empty()
              ? absent_subject(level.stock_slice)
              : offset_outward(level.part_slice,
                               radius + operation.radial_stock_to_leave);

      const double lead_in = operation.lead_in ? radius : 0.0;
      level.wave = wave::build_wavefront(keep_out, level.stock_slice, stepover,
                                         stepover, lead_in);
      level.lineage = wave::build_front_lineage(level.wave);
      level.toolpath = wave::build_toolpath(level.wave, level.lineage, radius);

      const wave::Paths64 final_subject =
          level.part_slice.empty() ? keep_out : level.part_slice;
      level.simulation = wave::simulate_material(level.stock_slice, final_subject,
                                                 level.toolpath, radius,
                                                 operation.simulation_stock_inflation);

      CutContext context;
      context.axial_depth = level.axial_depth;
      context.direction = operation.direction;
      level.feeds = plan_feeds(level.toolpath, level.simulation, tool, machine,
                               material, *data, context, operation.feeds);

      if (level.toolpath.moves.empty()) level.nothing_to_cut = true;
    } catch (const std::exception& error) {
      plan.warnings.push_back("Depth " + std::to_string(floor_z) + " mm: " +
                              error.what());
      level.nothing_to_cut = true;
    }

    plan.levels.push_back(std::move(level));
  }

  // ---- Roll up ----------------------------------------------------------
  plan.stats.levels = plan.levels.size();
  for (const LevelPlan& level : plan.levels) {
    if (level.nothing_to_cut) continue;
    ++plan.stats.cut_levels;
    if (plan.spindle_rpm <= 0.0) plan.spindle_rpm = level.feeds.stats.spindle_rpm;
    plan.stats.cutting_distance += level.feeds.stats.cutting_distance;
    plan.stats.rapid_distance += level.feeds.stats.rapid_distance;
    plan.stats.cutting_minutes += level.feeds.stats.cutting_minutes;
    plan.stats.rapid_minutes += level.feeds.stats.rapid_minutes;
    plan.stats.maximum_load =
        std::max(plan.stats.maximum_load, level.simulation.stats.maximum_load);
    plan.stats.maximum_power_kw =
        std::max(plan.stats.maximum_power_kw, level.feeds.stats.maximum_power_kw);
    plan.stats.maximum_force =
        std::max(plan.stats.maximum_force, level.feeds.stats.maximum_force);
    plan.stats.maximum_deflection =
        std::max(plan.stats.maximum_deflection, level.feeds.stats.maximum_deflection);
    plan.stats.removed_volume +=
        level.simulation.stats.removed_area * level.axial_depth;
    plan.stats.gouge_area += level.simulation.stats.gouge_area;
    plan.stats.unsafe_plunges += level.simulation.stats.unsafe_plunges;
    plan.stats.unresolved_entries += level.simulation.stats.unresolved_entries;

    for (const std::string& warning : level.feeds.warnings)
      if (std::find(plan.warnings.begin(), plan.warnings.end(), warning) ==
          plan.warnings.end())
        plan.warnings.push_back(warning);
  }

  if (plan.stats.cut_levels == 0)
    plan.warnings.push_back("Nothing was left to cut at any depth");
  if (plan.stats.unsafe_plunges > 0)
    plan.warnings.push_back(
        std::to_string(plan.stats.unsafe_plunges) +
        " plunge(s) come down where the cutter still overlaps stock");
  if (plan.stats.gouge_area > 0.0)
    plan.warnings.push_back("The cutter reaches into the finished part; increase "
                            "the radial stock to leave");
  if (!tool.center_cutting && plan.stats.unresolved_entries > 0)
    plan.warnings.push_back("This tool is not centre cutting and the path needs "
                            "to enter solid material");

  return plan;
}

JobPlan plan_job(const JobGeometry& geometry,
                 const Job& job,
                 const ToolLibrary& tools,
                 const MachineLibrary& machines,
                 const MaterialLibrary& materials,
                 const ProgressCallback& progress) {
  JobPlan plan;
  if (job.operations.empty()) {
    plan.error = "The job has no operations";
    return plan;
  }

  const double share = 1.0 / static_cast<double>(job.operations.size());
  std::string previous_tool;
  for (std::size_t index = 0; index < job.operations.size(); ++index) {
    const Operation& operation = job.operations[index];
    OperationPlan result;
    result.operation_name = operation.name;
    result.tool_id = operation.tool_id;

    const Tool* tool = tools.find(operation.tool_id);
    const Machine* machine = machines.find(operation.machine_id);
    const Material* material = materials.find(operation.material_id);
    if (!tool) result.error = "Unknown tool: " + operation.tool_id;
    else if (!machine) result.error = "Unknown machine: " + operation.machine_id;
    else if (!material) result.error = "Unknown material: " + operation.material_id;

    if (result.ok()) {
      result = plan_operation(geometry, operation, *tool, *machine, *material,
                              progress, share * index, share * (index + 1));
      if (result.error == "Cancelled") {
        plan.error = "Cancelled";
        return plan;
      }
      if (!previous_tool.empty() && previous_tool != operation.tool_id)
        result.stats.tool_change_minutes +=
            machine->tool_change == ToolChange::Automatic ? 0.25 : 1.0;
      previous_tool = operation.tool_id;
    }

    accumulate(plan.stats, result.stats);
    for (const std::string& warning : result.warnings)
      plan.warnings.push_back(operation.name + ": " + warning);
    if (!result.ok()) plan.warnings.push_back(operation.name + ": " + result.error);
    plan.operations.push_back(std::move(result));
  }

  if (progress) progress(1.0, "Done");

  const bool any = std::any_of(plan.operations.begin(), plan.operations.end(),
                               [](const OperationPlan& op) { return op.ok(); });
  if (!any) plan.error = "No operation could be planned";
  return plan;
}

}  // namespace cam
