#include "job_internal.h"
#include "cam/heightmap_simulation.h"

namespace cam {
wave::Paths64 roughing_area_boundary(const RoughingArea& area, double inset) {
  if(!std::isfinite(area.center_x) || !std::isfinite(area.center_y) ||
     !std::isfinite(area.width) || !std::isfinite(area.height) ||
     !std::isfinite(inset) || inset<0 || area.width<=2*inset || area.height<=2*inset)
    throw std::invalid_argument("Roughing area must have finite coordinates and be wider and taller than the cutter");
  const auto coordinate=[](double value) {
    constexpr double limit=double(std::numeric_limits<std::int64_t>::max())/wave::kIntegerScale/4;
    if(!std::isfinite(value) || std::abs(value)>limit)
      throw std::invalid_argument("Roughing area coordinates are out of range");
    return wave::to_integer(value);
  };
  const auto x0=coordinate(area.center_x-area.width/2+inset),x1=coordinate(area.center_x+area.width/2-inset);
  const auto y0=coordinate(area.center_y-area.height/2+inset),y1=coordinate(area.center_y+area.height/2-inset);
  if(x1<=x0 || y1<=y0) throw std::invalid_argument("Roughing area has no room for the cutter");
  return {{{x0,y0},{x1,y0},{x1,y1},{x0,y1}}};
}

namespace detail {
OperationPlan plan_roughing_areas(const JobGeometry& geometry, const Job& job,
    const Operation& operation, const Tool& tool, const Machine& machine,
    const CuttingFeeds& feeds, double rapid_plane, const PriorOperations& prior,
    const ProgressCallback& progress, double from, double to, const ToolLibrary& tools) {
  OperationPlan result;result.operation_name=operation.name;result.tool_id=operation.tool_id;
  // Validate every area before doing any cutting or simulation.
  try {for(const auto& area:operation.roughing_areas) roughing_area_boundary(area,tool.diameter/2);}
  catch(const std::exception& e) {result.error=e.what();return result;}
  const double share=(to-from)/operation.roughing_areas.size();
  for(std::size_t i=0;i<operation.roughing_areas.size();++i) {
    const auto& area=operation.roughing_areas[i];
    const auto name=area.name.empty()?"Area "+std::to_string(i+1):area.name;
    Operation pass=operation;pass.roughing_areas={area};pass.name=operation.name+" / "+name;
    const double start=from+share*i,seam=start+.85*share,end=start+share;
    PriorOperations history=prior;
    if(result.stock_after) history.push_back(&result);
    auto planned=plan_subdivided_operation(geometry,pass,tool,machine,feeds,progress,start,seam,
                                           rapid_plane,history);
    if(planned.ok()) {
      const auto simulation_start=PlanningClock::now();
      simulate_operation_xyz(planned,geometry,job,pass,tool,machine,feeds,history,
          [&](PlanningProgress p,const std::string& label) {
        return !progress || progress(seam+(end-seam)*p.fraction,pass.name+": "+label);
      },&tools);
      planned.metrics.xyz_simulation_milliseconds=elapsed_milliseconds(simulation_start);
    }
    if(!planned.ok()) {
      result.error=planned.error=="Cancelled"?planned.error:name+": "+planned.error;
      return result;
    }
    if(!result.stock_before) result.stock_before=planned.stock_before;
    result.stock_after=planned.stock_after;
    result.spindle_rpm=std::max(result.spindle_rpm,planned.spindle_rpm);
    const auto level_offset=result.levels.size(),subdivision_offset=result.subdivisions.size();
    for(auto& level:planned.levels) {
      level.roughing_area=int(i);level.roughing_area_name=name;
      level.subdivision+=subdivision_offset;
      result.levels.push_back(std::move(level));
    }
    for(auto subdivision:planned.subdivisions) {
      subdivision.first_level+=level_offset;result.subdivisions.push_back(subdivision);
    }
    append_inter_layer(result.inter_layer,std::move(planned.inter_layer),level_offset);
    accumulate(result.stats,planned.stats);accumulate(result.metrics,planned.metrics);
    for(auto& warning:planned.warnings) result.warnings.push_back(name+": "+warning);
    for(auto& message:planned.debug_messages) result.debug_messages.push_back(name+": "+message);
  }
  return result;
}
} // namespace detail
} // namespace cam
