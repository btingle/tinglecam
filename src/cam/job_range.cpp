#include "job_internal.h"
#include "cam/profiling.h"
#include "cam/surface_finishing.h"
#include "cam/heightmap_simulation.h"
namespace cam {
using namespace detail;
double job_rapid_plane(const Job& job, const JobGeometry& geometry) {
  double highest = highest_model_z(geometry);
  for (const auto& operation : job.operations) {
    if (operation.mode == OperationMode::Pocket) {
      for (const auto& pocket : operation.pockets)
        highest = std::max(highest, pocket.top_z);
    }
    if (operation.mode == OperationMode::PlaneClearing) {
      for (const auto& area : operation.plane.areas)
        highest = std::max(highest, area.top_z);
    }
  }
  return job_rapid_plane(job, highest);
}

PartialPlan plan_operation_range(const JobGeometry& geometry,
                                 const Job& job,
                                 const ToolLibrary& tools,
                                 const MachineLibrary& machines,
                                 const MaterialLibrary& materials,
                                 const ProgressCallback& progress,
                                 std::size_t first, std::size_t last,
                                 const PriorOperations& earlier) {
  PartialPlan produced;
  produced.first = first;
  if (job.operations.empty() || first > last ||
      last >= job.operations.size()) {
    produced.error = "That range of operations is not in the job";
    return produced;
  }
  if (earlier.size() != first) {
    produced.error = "The operations above the range are not all accounted for";
    return produced;
  }
  if (!(job.rapid_clearance_height > 0) ||
      !std::isfinite(job.rapid_clearance_height)) {
    produced.error = "Rapid clearance must be finite and greater than zero";
    return produced;
  }
  produced.rapid_plane = job_rapid_plane(job, geometry);

  // Reserved up front and never grown past it, because `prior` below holds
  // pointers into it: an operation is planned against the ones before it, and
  // for everything after the first of the range that means the ones this run
  // has just produced.
  const std::size_t count = last - first + 1;
  produced.operations.reserve(count);
  PriorOperations prior = earlier;
  prior.reserve(last + 1);

  // The tool the cutter is holding when the range begins. Taken from the last
  // operation above it that actually produced a plan, exactly as running the
  // job from the top would leave it: an operation that failed changed no tool.
  std::string previous_tool;
  for (std::size_t index = first; index-- > 0;)
    if (earlier[index] != nullptr && earlier[index]->ok()) {
      previous_tool = earlier[index]->tool_id;
      break;
    }

  const double share = 1.0 / static_cast<double>(count);
  for (std::size_t index = first; index <= last; ++index) {
    const Operation& operation = job.operations[index];
    const double from = share * static_cast<double>(index - first);
    const double to = share * static_cast<double>(index - first + 1);
    OperationPlan result;
    result.planned = true;
    result.operation_name = operation.name;
    result.tool_id = operation.tool_id;
    result.machine_id = operation.machine_id;
    result.material_id = operation.material_id;

    const Tool* tool = tools.find(operation.tool_id);
    const Machine* machine = machines.find(operation.machine_id);
    const Material* material = materials.find(operation.material_id);
    if (!tool) result.error = "Unknown tool: " + operation.tool_id;
    else if (!machine) result.error = "Unknown machine: " + operation.machine_id;
    else if (!material) result.error = "Unknown material: " + operation.material_id;

    if (result.ok()) {
      const auto operation_started=PlanningClock::now();
      const auto stage_progress = [&](PlanningProgress p, const std::string& stage) {
        if (!progress) return true;
        const double local = std::clamp((p.fraction-from)/std::max(to-from,1e-9),0.0,1.0);
        PlanningProgress scaled=p;
        scaled.fraction=from+(to-from)*0.85*local;
        scaled.active_until=std::max(scaled.fraction,from+(to-from)*0.85*
            std::clamp((p.active_until-from)/std::max(to-from,1e-9),0.0,1.0));
        return progress(scaled,stage);
      };
      const auto surface_progress = [&](PlanningProgress p,const std::string& stage) {
        return stage_progress(from+(to-from)*p.fraction,stage);
      };
      const auto simulation_progress = [&](PlanningProgress p,const std::string& stage) {
        return !progress || progress(from+(to-from)*(0.85+0.15*p.fraction),stage);
      };
      if (operation.mode == OperationMode::SurfaceFinishing)
        result = plan_surface_finishing(geometry, operation, *tool, *machine, produced.rapid_plane, surface_progress);
      else if (is_parametric(operation.mode))
        result = plan_parametric_operation(operation, *tool, *machine,
            produced.rapid_plane, prior, stage_progress, from, to, &geometry);
      else
        result = plan_subdivided_operation(
            geometry, operation, *tool, *machine, *material, stage_progress,
            from, to, produced.rapid_plane, prior);
      if(result.ok()) {
        const auto simulation_started=PlanningClock::now();
        simulate_operation_xyz(result,geometry,job,operation,*tool,*machine,prior,simulation_progress);
        result.metrics.xyz_simulation_milliseconds=elapsed_milliseconds(simulation_started);
      }
      result.metrics.total_milliseconds=elapsed_milliseconds(operation_started);
      result.planned = true;
      if (result.error == "Cancelled") {
        produced.operations.clear();
        produced.error = "Cancelled";
        return produced;
      }
      if (result.ok()) {
        if (!previous_tool.empty() && previous_tool != operation.tool_id)
          result.stats.tool_change_minutes +=
              machine->tool_change == ToolChange::Automatic ? 0.25 : 1.0;
        previous_tool = operation.tool_id;
      }
    }

    produced.operations.push_back(std::move(result));
    prior.push_back(&produced.operations.back());
  }
  release_planning_scratch();
  return produced;
}

void summarise_job(JobPlan& plan) {
  plan.stats = {};
  plan.metrics = {};
  plan.warnings.clear();
  plan.error.clear();
  for (const OperationPlan& operation : plan.operations) {
    if (!operation.planned) continue;
    accumulate(plan.stats, operation.stats);
    accumulate(plan.metrics, operation.metrics);
    for (const std::string& warning : operation.warnings)
      plan.warnings.push_back(operation.operation_name + ": " + warning);
  }
  finish_job(plan);
}
}  // namespace cam
