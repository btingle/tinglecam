#include "cam/config.h"
#include "cam/job.h"
#include "cam/post.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
cam::Tool tool() {
  cam::Tool result; result.id="test"; result.name="Test mill"; result.diameter=2;
  return result;
}
cam::CuttingFeeds feeds() { return {10000,.01,2,100,2000}; }
cam::OperationPlan plan_waves(const cam::JobGeometry& geometry, const cam::Operation& op,
                        const cam::PriorOperations& prior={}) {
  return cam::plan_wavefronts(geometry,op,tool(),{},feeds(),{},0,1,prior);
}
void expanded(const wave::Paths64& before, const wave::Paths64& after, double amount) {
  const auto a=Clipper2Lib::GetBounds(before), b=Clipper2Lib::GetBounds(after);
  const auto delta=wave::to_integer(amount);
  check(b.left==a.left-delta && b.right==a.right+delta &&
        b.top==a.top-delta && b.bottom==a.bottom+delta,
        "Dilation must expand every boundary by the entered distance");
}

void check_outside_rim(const std::filesystem::path& part) {
  auto cutter=tool();
  cutter.diameter=6.35;
  cutter.flute_length=20;
  cutter.cutting={{"plastic",10000,.01,100}};
  cam::Machine machine; machine.id="test"; machine.max_rpm=20000;
  const cam::ToolLibrary tools{{cutter}};
  const cam::MachineLibrary machines{{machine}};
  const cam::MaterialLibrary materials{{{"plastic","Plastic"}}};
  cam::Job job;
  job.machine_id="test"; job.material_id="plastic";
  job.simulation_pitch=.05;
  cam::Operation op;
  op.tool_id=cutter.id; op.depth_to=3; op.stepdown=1;
  op.stepover_ratio=.15; op.path_pitch=.2;
  op.radial_stock_to_leave=.2;
  op.finishing_subdivisions=2;
  job.operations={op};
  cam::JobGeometry geometry;
  geometry.load_part(part);

  // Both rims are narrower than the cutter radius. The second is also
  // narrower than radial allowance + cleanup tolerance, so borrowing the
  // upward-cleanup reach would still miss it.
  for (const auto [rim, lead_in] : {std::pair{1.,true}, std::pair{1.,false},
                                  std::pair{.25,true}, std::pair{.25,false}}) {
    job.operations.front().lead_in=lead_in;
    geometry.set_box_stock(true,{rim,rim,rim,rim,0,0});
    const auto planned=cam::plan_job(geometry,job,tools,machines,materials,{});
    if (!planned.ok()) throw std::runtime_error("Outside-rim roughing: "+planned.error);
    const auto& rough=planned.operations.front();
    std::size_t coarse_levels=0;
    for (const auto& level : rough.levels) {
      if (level.finishing_index!=0) continue;
      ++coarse_levels;
      double removed=0;
      for (const auto& phase : level.feeds.motion) removed+=phase.removed_volume;
      check(removed>0,"Every downward level must remove the outside rim before cleanup");
      check(!level.stock_checkpoints.empty(),"Rim cuts must retain simulated stock");
      const auto allowance=Clipper2Lib::InflatePaths(level.part_slice,
          wave::to_integer(op.radial_stock_to_leave+.04),
          Clipper2Lib::JoinType::Round,Clipper2Lib::EndType::Polygon);
      const auto waste=Clipper2Lib::Difference(level.stock_slice,allowance,
          Clipper2Lib::FillRule::NonZero);
      const auto after=level.stock_checkpoints.back().stock->slice(
          level.floor_z+.5*level.axial_depth);
      check(wave::material_area(Clipper2Lib::Intersect(after,waste,
          Clipper2Lib::FillRule::NonZero))<.01,
          "The coarse pass must clear its slab outside the radial allowance");
      const auto protected_stock=Clipper2Lib::InflatePaths(level.part_slice,
          wave::to_integer(op.radial_stock_to_leave-.05),
          Clipper2Lib::JoinType::Round,Clipper2Lib::EndType::Polygon);
      check(wave::material_area(Clipper2Lib::Difference(protected_stock,after,
          Clipper2Lib::FillRule::NonZero))<.01,
          "Exterior cuts must retain the part and its radial stock allowance");
    }
    check(coarse_levels==3,"Rim regression must exercise three downward levels");
    check(rough.stats.unsafe_plunges==0 && rough.stats.unreachable_fronts==0,
          "Exterior roughing must retain safe entries and reachable fronts");
    check(rough.stats.gouge_area<.001,"Exterior roughing must preserve the part");
  }

  geometry.set_box_stock(true,{.1,.1,.1,.1,0,0});
  const auto inside_allowance=cam::plan_job(geometry,job,tools,machines,materials,{});
  check(inside_allowance.ok(),"Stock within the radial allowance must plan");
  check(inside_allowance.operations.front().stats.cut_levels==0,
        "Exterior reach must not cut a rim inside the radial stock allowance");

  // A projected blank plus planning dilation must also discover the exterior
  // paths, although planning dilation does not add simulated material.
  geometry.set_box_stock(true,{0,0,0,0,0,0});
  op.stock_boundary_dilation=1;
  auto dilated=cam::plan_wavefronts(geometry,op,cutter,machine,feeds(),{});
  cam::plan_toolpaths(dilated,op,cutter,machine,feeds(),{});
  check(dilated.ok(),"Projected stock with dilation must plan exterior paths");
  for (const auto& level : dilated.levels)
    check(level.toolpath.stats.cut_count>0,
          "Planning dilation must produce downward cuts around projected stock");
}
}
int main() {
  const auto dir=std::filesystem::temp_directory_path()/"tinglecam-stock-dilation-tests";
  try {
    std::filesystem::create_directories(dir);
    const auto part=dir/"part.obj",stock=dir/"stock.obj";
    {
      std::ofstream out(part);
      out<<"v -2 -2 0\nv 2 -2 0\nv 2 2 0\nv -2 2 0\n"
            "v -2 -2 4\nv 2 -2 4\nv 2 2 4\nv -2 2 4\nf 1 4 3 2\nf 5 6 7 8\n";
    }
    check_outside_rim(part);
    cam::JobGeometry geometry;
    geometry.load_part(part); geometry.set_box_stock(true,{8,8,8,8,0,0});
    cam::Operation op; op.tool_id="test"; op.depth_to=2; op.stepdown=1;
    op.stepover_ratio=.5; op.path_pitch=.5;
    const auto plain=plan_waves(geometry,op);
    check(plain.ok() && plain.levels.size()==2,"Baseline roughing plans both levels");
    op.stock_boundary_dilation=1;
    const auto grown=plan_waves(geometry,op);
    check(grown.ok(),"Dilation must be valid above the first-bite headroom limit");
    for(std::size_t i=0;i<grown.levels.size();++i) {
      const auto& level=grown.levels[i];
      expanded(plain.levels[i].stock_slice,level.stock_slice,1);
      expanded(plain.levels[i].incoming_stock,level.incoming_stock,1);
      const auto b=Clipper2Lib::GetBounds(level.stock_slice);
      check(b.Width()==22000 && b.Height()==22000,"20 x 20 with 1 mm dilation must become 22 x 22");
      check(std::abs(wave::material_area(level.stock_slice)-484)<1e-6,
            "Square stock must retain square corners");
      check(level.floor_z==plain.levels[i].floor_z,"XY dilation must preserve Z floors");
    }
    const auto nominal=Clipper2Lib::GetBounds(geometry.stock_outline(3,32));
    check(nominal.Width()==20000,"Operation dilation must not compound in the cached source slices");

    op.finishing_subdivisions=2;
    cam::Job cleanup_job;
    cleanup_job.machine_id="test"; cleanup_job.material_id="plastic";
    cleanup_job.simulation_pitch=.25; cleanup_job.operations={op};
    auto cutting_tool=tool(); cutting_tool.cutting={{"plastic",10000,.01,100}};
    cam::Machine machine; machine.id="test"; machine.name="Test machine"; machine.max_rpm=20000;
    const cam::ToolLibrary tools{{cutting_tool}};
    const cam::MachineLibrary machines{{machine}};
    const cam::MaterialLibrary materials{{{"plastic","Plastic"}}};
    const auto full=cam::plan_job(geometry,cleanup_job,tools,machines,materials,{});
    if(!full.ok()) throw std::runtime_error("Dilated roughing integration: "+full.error);
    const auto& cleanup=full.operations.front();
    check(cleanup.levels.size()==4,"Dilated roughing with cleanup must plan");
    check(!cam::post_job(full,cleanup_job,tools,machines).text.empty(),
          "Dilated planning must survive material simulation and G-code export");
    for(const auto& level:cleanup.levels) {
      const auto b=Clipper2Lib::GetBounds(level.stock_slice);
      check(b.Width()==22000 && b.Height()==22000,"Cleanup must apply dilation once per original slice");
    }

    cam::OperationPlan prior;
    auto material=wave::Heightmap::box({{-10,-10,0},{10,10,4},true},.25);
    material.cut({-5,-10,0},{-5,10,0},{wave::CutterShape::Flat,1,20});
    prior.stock_after=std::make_shared<const wave::Heightmap>(material);
    op.stock_boundary_dilation=0;
    const auto remaining=plan_waves(geometry,op,{&prior});
    op.stock_boundary_dilation=.25;
    const auto dilated_remaining=plan_waves(geometry,op,{&prior});
    check(remaining.ok() && dilated_remaining.ok(),"Prior-operation stock must support dilation");
    for(std::size_t i=0;i<remaining.levels.size();++i) {
      const auto expected=Clipper2Lib::InflatePaths(remaining.levels[i].incoming_stock,250,
          Clipper2Lib::JoinType::Miter,Clipper2Lib::EndType::Polygon);
      check(wave::paths_equal_canonical(expected,dilated_remaining.levels[i].incoming_stock),
            "Dilation must apply to actual incoming stock after prior operations");
    }

    { std::ofstream out(stock);
      out<<"v -10 -10 -1\nv 10 -10 -1\nv 10 10 -1\nv -10 10 -1\n"
            "v -10 -10 0\nv 10 -10 4\nv 10 10 4\nv -10 10 0\nf 1 4 3 2\nf 5 6 7 8\n";
    }
    geometry.load_stock(stock); geometry.set_box_stock(false,{});
    op.stock_boundary_dilation=0; const auto sloped=plan_waves(geometry,op);
    op.stock_boundary_dilation=1; const auto sloped_grown=plan_waves(geometry,op);
    check(sloped.ok() && sloped_grown.ok(),"Sloped stock must plan");
    check(sloped.levels[0].stock_slice!=sloped.levels[1].stock_slice,
          "Fixture must provide different per-Z stock outlines");
    for(std::size_t i=0;i<sloped.levels.size();++i)
      expanded(sloped.levels[i].stock_slice,sloped_grown.levels[i].stock_slice,1);

    op.mode=cam::OperationMode::ContourFinishing;
    const auto finish=plan_waves(geometry,op);
    check(finish.ok() && finish.levels[0].stock_slice==sloped.levels[0].stock_slice,
          "Dilation must affect roughing only");
    op.mode=cam::OperationMode::Roughing;
    for(double invalid:{-1.,std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN(),1e300}) {
      op.stock_boundary_dilation=invalid;
      check(!plan_waves(geometry,op).ok(),"Invalid dilation must fail before planning");
    }

    op.stock_boundary_dilation=1;
    cam::Job job; job.operations={op};
    check(cam::job_from_json_text(cam::to_json_text(job)).operations.front().stock_boundary_dilation==1,
          "Job persistence must preserve dilation");
    check(cam::job_from_json_text("{\"operations\":[{}]}").operations.front().stock_boundary_dilation==0,
          "Older jobs must default to zero dilation");
    auto settings=cam::default_user_settings();
    cam::operation_default(settings,cam::OperationMode::Roughing).stock_boundary_dilation=1.25;
    cam::save_user_settings(dir/"settings.json",settings);
    check(cam::operation_default(cam::load_user_settings(dir/"settings.json"),
              cam::OperationMode::Roughing).stock_boundary_dilation==1.25,
          "Operation defaults must persist dilation");
    cam::save_job(dir/"job.json",job);
    std::vector<std::string> warnings;
    (void)cam::load_job(dir/"job.json",&warnings);
    check(warnings.empty(),"Saved dilation must be recognized when jobs are loaded");
    std::filesystem::remove_all(dir);
    std::cout<<"Stock dilation preserves per-Z geometry, cleanup, operation scope and persistence.\n";
  }catch(const std::exception& e) {
    std::filesystem::remove_all(dir); std::cerr<<e.what()<<'\n';return 1;
  }
}
