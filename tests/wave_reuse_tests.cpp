#include "cam/job.h"
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool condition,const char* text) {if(!condition)throw std::runtime_error(text);}
cam::OperationPlan plan(const cam::JobGeometry& geometry,std::size_t& work) {
  cam::Operation operation;
  operation.mode=cam::OperationMode::Roughing;
  operation.depth_from=0;operation.depth_to=4;operation.stepdown=1;
  operation.stepover_ratio=.4;operation.path_pitch=.2;
  cam::Tool tool;tool.id="reuse-test";tool.name="Reuse test";tool.diameter=1;
  cam::CuttingFeeds feeds{10000,.01,2,100,1000};
  return cam::plan_wavefronts(geometry,operation,tool,{},feeds,
      [&](cam::PlanningProgress progress,const std::string& stage) {
        if(stage.find(": wavefronts")!=std::string::npos)
          work=std::max(work,progress.total_levels);
        return true;
      });
}
}
int main() {
 const auto dir=std::filesystem::temp_directory_path()/"tinglecam-wave-reuse-tests";
 try {
  std::filesystem::create_directories(dir);
  const auto part=dir/"part.obj",stock=dir/"stock.obj";
  {
    std::ofstream out(part);
    out<<"v 0 0 0\nv 4 0 0\nv 4 4 0\nv 0 4 0\n"
          "v 0 0 6\nv 4 0 6\nv 4 4 6\nv 0 4 6\nf 1 4 3 2\nf 5 6 7 8\n";
  }
  cam::JobGeometry geometry;
  geometry.load_part(part);
  geometry.set_box_stock(true,{3,3,3,3,0,0});
  std::size_t work=0;
  auto repeated=plan(geometry,work);
  check(repeated.ok() && repeated.levels.size()==4,"Repeated-outline roughing must plan all depths");
  check(work==1,"Identical XY inputs must perform only one wavefront analysis");
  const auto& first=repeated.levels.front();
  check(!first.wave.generations.empty(),"Reused wave must contain actual cutting fronts");
  for(std::size_t i=0;i<repeated.levels.size();++i) {
    const auto& level=repeated.levels[i];
    check(level.floor_z==5-double(i),"Wave reuse must preserve each level's own depth");
    check(level.wave.generations==first.wave.generations &&
          level.wave_stages.size()==first.wave_stages.size() &&
          level.topology.nodes.size()==first.topology.nodes.size(),
          "Reused wavefronts must preserve geometry and topology");
  }
  const auto saved=first.wave.generations;
  repeated.levels[1].wave.generations.clear();
  check(first.wave.generations==saved,"Reused levels must own independent mutable geometry");
  {
    std::ofstream out(stock);
    out<<"v -3 -3 -1\nv 7 -3 -1\nv 7 7 -1\nv -3 7 -1\n"
          "v -3 -3 0\nv 7 -3 6\nv 7 7 6\nv -3 7 0\nf 1 4 3 2\nf 5 6 7 8\n";
  }
  geometry.load_stock(stock);geometry.set_box_stock(false,{});
  work=0;
  const auto varying=plan(geometry,work);
  check(varying.ok() && work==4,"Different incoming-stock outlines must remain independent analyses");
  std::filesystem::remove_all(dir);
  std::cout<<"Wavefront reuse preserves per-depth geometry and rejects differing stock.\n";
 }catch(const std::exception& e) {
  std::filesystem::remove_all(dir);std::cerr<<e.what()<<'\n';return 1;
 }
}
