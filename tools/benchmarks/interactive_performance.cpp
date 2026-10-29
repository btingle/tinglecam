// Repeatable import/index/timeline probes. Optional model paths stay local.
#include "cam/cad.h"
#include "cam/feeds.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
using Clock=std::chrono::steady_clock;
double elapsed(Clock::time_point start) {
 return std::chrono::duration<double,std::milli>(Clock::now()-start).count();
}
}
int main(int argc,char** argv) {
 try {
  if(argc>2)throw std::runtime_error("Usage: interactive_performance [MODEL]");
  if(argc==2) {
   auto start=Clock::now(),stage_start=start;
   std::string last;
   const auto cad=cam::step_filename(argv[1]) ? cam::CadModel::load(argv[1],.01,
    [&](double,const std::string& stage) {
     if(stage!=last) {
      if(!last.empty())std::cout<<"import_stage=\""<<last<<"\" ms="<<elapsed(stage_start)<<std::endl;
      last=stage;stage_start=Clock::now();
     }
     return true;
    }) : nullptr;
   auto mesh=cad?cad->mesh():wave::load_triangle_mesh(argv[1]);
   std::cout<<"import_ms="<<elapsed(start)<<" triangles="<<mesh.triangles.size()<<std::endl;
   wave::MeshSlicer slicer(wave::MeshSlicerBackend::CPU);
   slicer.set_mesh(std::move(mesh));
   std::cout<<"mesh_index_ms="<<slicer.stats().build_milliseconds
    <<" retained="<<slicer.stats().retained_triangles<<std::endl;
  }
  wave::Mesh overlap;
  for(int i=0;i<16000;++i) {
   const float z=float(i)/16000;
   overlap.triangles.push_back({{0,0,z},{1,0,1-z},{0,1,z}});
  }
  overlap.bounds={{0,0,0},{1,1,1},true};
  wave::MeshSlicer slicer(wave::MeshSlicerBackend::CPU);
  slicer.set_mesh(std::move(overlap));
  std::cout<<"overlap_triangles=16000 index_ms="<<slicer.stats().build_milliseconds<<std::endl;
  double ray_ms;
  const auto hit=slicer.vertical_raycast({{.25f,.25f,2}},-1,ray_ms).front();
  if(!std::isfinite(hit) || std::abs(hit-(.25f+.5f*15999/16000))>1e-5)
   throw std::runtime_error("Overlap top-envelope ray changed");
  cam::FeedSchedule schedule;
  for(int i=0;i<100000;++i) {
   cam::MotionPhase phase;
   phase.from={i*1000,0};phase.to={(i+1)*1000,0};phase.length=1;phase.rate=600;
   phase.path_distance_begin=i;phase.path_distance_end=i+1;
   schedule.motion.push_back(phase);
  }
  const auto query=[&] {
   auto start=Clock::now();double sum=0;
   for(int i=0;i<2000;++i)
    sum+=cam::feed_motion_at_seconds(schedule,(i*7919%100000)*.1).motion_distance;
   std::cout<<"lookup_2000_ms="<<elapsed(start)<<" checksum="<<sum<<std::endl;
   return sum;
  };
  const double reference=query();
  cam::index_feed_motion(schedule);
  if(std::abs(query()-reference)>1e-5)throw std::runtime_error("Timeline indexing changed lookup");
 } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
