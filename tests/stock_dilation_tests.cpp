#include "cam/config.h"
#include "cam/job.h"
#include "cam/post.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

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
