#include "job_internal.h"
#include "cam/profiling.h"
#include "cam/holes.h"
#include "cam/surface_finishing.h"
#include "cam/heightmap_simulation.h"
namespace cam {
using namespace detail;

const ToolAssignment* find_tool_assignment(const std::vector<ToolAssignment>& table,
                                           const std::string& tool_id) {
  for (const auto& entry : table)
    if (entry.tool_id == tool_id) return &entry;
  return nullptr;
}

std::vector<ToolAssignment> resolved_tool_table(const Job& job) {
  std::vector<ToolAssignment> table;
  int highest = 0;
  for (const auto& entry : job.tools) {
    if (entry.tool_id.empty() || find_tool_assignment(table, entry.tool_id)) continue;
    table.push_back(entry);
    highest = std::max(highest, entry.number);
  }
  const auto add_tool=[&](const std::string& id) {
    if(!id.empty() && !find_tool_assignment(table,id)) table.push_back({id,++highest,0});
  };
  for(const auto& operation:job.operations) {
    add_tool(operation.tool_id);
    if(operation.mode==OperationMode::Hole)
      for(const auto& h:operation.holes.systems) for(const auto& l:h.levels) add_tool(l.tool_id);
  }
  for (auto& entry : table) {
    if (entry.number <= 0) entry.number = ++highest;
    if (entry.length_offset <= 0) entry.length_offset = entry.number;
  }
  return table;
}
double job_rapid_plane(const Job& job, const JobGeometry& geometry) {
  double highest = highest_model_z(geometry);
  for(const auto& f:geometry.fixtures())
    highest=std::max(highest,double(f.geometry.mesh().bounds.upper.z)+f.model.clearance);
  for (const auto& operation : job.operations) {
    if(operation.mode==OperationMode::Hole)
      for(const auto& h:operation.holes.systems) for(const auto& l:h.levels)
        highest=std::max(highest,l.top_z);
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

PartialPlan plan_operation_range(const JobGeometry& base_geometry,
                                 const Job& job,
                                 const ToolLibrary& tools,
                                 const MachineLibrary& machines,
                                 const MaterialLibrary& materials,
                                 const ProgressCallback& progress,
                                 std::size_t first, std::size_t last,
                                 const PriorOperations& earlier) {
  PartialPlan produced;
  produced.first = first;
  std::map<std::string,JobGeometry> setup_geometry;
  std::map<std::string,Job> setup_jobs;
  const JobGeometry& geometry=base_geometry;
  if (job.operations.empty() || first > last ||
      last >= job.operations.size()) {
    produced.error = "That range of operations is not in the job";
    return produced;
  }
  Job validation=job;
  validation.operations.resize(last+1);
  if(!job.setups.empty()) {
    if(const auto* last_setup=find_setup(job,job.operations[last].setup_id))
      validation.setups.resize(std::size_t(last_setup-job.setups.data())+1);
  }
  const auto problems=validate_setups(validation);
  if(!problems.empty()) {produced.error=problems.front();return produced;}
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
  if (!std::isfinite(job.plunge_clearance_height) || job.plunge_clearance_height < 0) {
    produced.error = "Plunge clearance must be finite and nonnegative";
    return produced;
  }


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
      for(const auto& level:earlier[index]->levels)
        if(!level.nothing_to_cut && !level.feeds.motion.empty() && !level.tool_id.empty()) previous_tool=level.tool_id;
      break;
    }

  const double share = 1.0 / static_cast<double>(count);
  for (std::size_t index = first; index <= last; ++index) {
    Operation operation=job.operations[index];
    std::string source_error;
    try {operation=resolve_hole_source(job,index);}
    catch(const std::exception& e) {source_error=e.what();}
    const double from = share * static_cast<double>(index - first);
    const double to = share * static_cast<double>(index - first + 1);
    OperationPlan result;
    result.planned = true;
    result.operation_name = operation.name;
    result.tool_id = operation.tool_id;

    const JobGeometry* active_geometry=&base_geometry;
    const Job* active_job=&job;
    PriorOperations setup_prior;
    double rapid_plane=produced.rapid_plane;
    try {
      if(!job.setups.empty()) {
        auto found=setup_jobs.find(operation.setup_id);
        if(found==setup_jobs.end()) {
          auto context=job_for_setup(job,operation.setup_id);
          auto view=base_geometry.context_view(context, [&](double,const std::string& stage) {
            return !progress || progress(from,stage);
          });
          if(!context.stock_from_operation.empty()) {
            std::size_t source=0;
            while(source<index && job.operations[source].id!=context.stock_from_operation) ++source;
            if(source>=prior.size() || !prior[source] || !prior[source]->planned ||
               !prior[source]->ok() || !prior[source]->stock_after)
              throw std::runtime_error("Inherited stock source has no valid result; plan its source operation first");
            const auto& previous=prior[source]->stock_after;
            view.set_inherited_stock(std::make_shared<const wave::Heightmap>(previous->transformed(context.stock_transfer)));
          }
          setup_geometry.emplace(operation.setup_id,std::move(view));
          found=setup_jobs.emplace(operation.setup_id,std::move(context)).first;
        }
        active_job=&found->second;
        active_geometry=&setup_geometry.at(operation.setup_id);
        for(std::size_t previous=0;previous<index;++previous)
          if(job.operations[previous].setup_id==operation.setup_id) setup_prior.push_back(prior[previous]);
        rapid_plane=job_rapid_plane(*active_job,*active_geometry);
      } else setup_prior=prior;
      if(!std::isfinite(active_job->rapid_clearance_height) || active_job->rapid_clearance_height<=0)
        throw std::runtime_error("Rapid clearance must be finite and greater than zero");
      if(!std::isfinite(active_job->plunge_clearance_height) || active_job->plunge_clearance_height<0)
        throw std::runtime_error("Plunge clearance must be finite and nonnegative");
    } catch(const std::exception& e) {source_error=e.what();}
    const Machine* machine=machines.find(active_job->machine_id);
    const Material* material=materials.find(active_job->material_id);
    produced.rapid_plane=std::max(produced.rapid_plane,rapid_plane);
    result.error=source_error;
    const Tool* tool = tools.find(operation.tool_id);
    CuttingFeeds feeds;
    if (result.ok() && !machine) result.error = active_job->machine_id.empty()
        ? "Choose a machine for the job" : "Unknown machine: " + active_job->machine_id;
    else if (result.ok() && !material) result.error = active_job->material_id.empty()
        ? "Choose a material for the job" : "Unknown material: " + active_job->material_id;
    else if (result.ok() && !tool) result.error = operation.tool_id.empty()
        ? "Choose a tool" : "Unknown tool: " + operation.tool_id;
    else if(result.ok()) {
      try {
        feeds = resolve_cutting_feeds(*tool, *machine, active_job->material_id,
                                      operation.cutting);
      } catch (const std::exception& error) {
        result.error = error.what();
      }
    }

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
      const bool area_roughing=operation.mode==OperationMode::Roughing && !operation.roughing_areas.empty();
      if(area_roughing)
        result=plan_roughing_areas(*active_geometry,*active_job,operation,*tool,*machine,feeds,
                                  rapid_plane,setup_prior,progress,from,to,tools);
      else if(operation.mode==OperationMode::Hole)
        result=plan_hole_operation(operation,*tool,*machine,feeds,rapid_plane,setup_prior,
                                   stage_progress,from,to,active_geometry,&tools,active_job->material_id);
      else if (operation.mode == OperationMode::SurfaceFinishing)
        result = plan_surface_finishing(*active_geometry, operation, *tool, *machine, feeds, rapid_plane, surface_progress);
      else if (is_parametric(operation.mode))
        result = plan_parametric_operation(operation, *tool, *machine, feeds,
            rapid_plane, setup_prior, stage_progress, from, to, active_geometry);
      else
        result = plan_subdivided_operation(
            *active_geometry, operation, *tool, *machine, feeds, stage_progress,
            from, to, rapid_plane, setup_prior, active_job->planning);
      if(result.ok() && !area_roughing) {
        const auto simulation_started=PlanningClock::now();
        simulate_operation_xyz(result,*active_geometry,*active_job,operation,*tool,*machine,feeds,setup_prior,simulation_progress,&tools);
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
        for(const auto& level:result.levels) {
          if(level.nothing_to_cut || level.feeds.motion.empty()) continue;
          const auto id=level.tool_id.empty()?operation.tool_id:level.tool_id;
          if(!previous_tool.empty() && previous_tool!=id)
            result.stats.tool_change_minutes+=machine->tool_change==ToolChange::Automatic?.25:1.;
          previous_tool=id;
        }
      }
    }

    result.operation_id=operation.id; result.setup_id=operation.setup_id; result.rapid_plane=rapid_plane;
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
  plan.debug_messages.clear();
  plan.error.clear();
  for (const OperationPlan& operation : plan.operations) {
    if (!operation.planned) continue;
    accumulate(plan.stats, operation.stats);
    accumulate(plan.metrics, operation.metrics);
    for (const std::string& warning : operation.warnings)
      plan.warnings.push_back(operation.operation_name + ": " + warning);
    for (const auto& message : operation.debug_messages)
      plan.debug_messages.push_back(operation.operation_name + ": " + message);
  }
  finish_job(plan);
}
}  // namespace cam
