#include "cam/config.h"
#include "cam/heightmap_simulation.h"
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
using namespace wave;
using namespace Clipper2Lib;
void check(bool condition,const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
Path64 box(double x0,double y0,double x1,double y1) {
  return {{to_integer(x0),to_integer(y0)},{to_integer(x1),to_integer(y0)},
          {to_integer(x1),to_integer(y1)},{to_integer(x0),to_integer(y1)}};
}
WavefrontStage entry_stage(bool reachable) {
  WavefrontStage stage;auto& wave=stage.wave;
  wave.stock={box(0,0,20,20)};wave.stock_boundary=wave.stock.front();
  wave.subject={box(-20,-20,40,-3.3)};
  wave.machining_limit={box(-1,-3.5,21,22)};
  wave.stepover=.9;wave.lead_in=3.2;wave.lead_in_generations=4;
  auto& lineage=stage.lineage;lineage.stepover=.9;lineage.generations.resize(2);
  for(int g=0;g<2;++g) {
    const double y=g==0?-3.2:(reachable?-2.3:5.);
    auto& generation=lineage.generations[g];
    LeadingFront front;front.path={Point64(to_integer(5),to_integer(y)),Point64(to_integer(15),to_integer(y))};
    if(g==0)front.children={0};else front.parents={0};
    generation.fronts.push_back(front);
    generation.bands={{box(4,y-(g==0?.001:.9),16,y)}};
    wave.generations.push_back(generation.bands.front());
  }
  stage.topology.front_nodes={{0},{0}};
  stage.topology.nodes.resize(1);stage.topology.nodes[0].fronts={{0,0},{1,0}};
  return stage;
}
void landings_and_rejected_entries() {
  const auto stage=entry_stage(true);
  const auto path=build_toolpath(std::vector{stage},3);
  check(path.stats.cut_count==1 && path.stats.skipped_entry_count==0,
        "A radius-clear landing between an inset area and fixture must remain usable");
  const auto landing=path.moves.front().path.front();
  check(to_world(landing.y)<=-3. && to_world(landing.y)>=-3.3,
        "The landing clears physical stock without shrinking fixture clearance twice");
  auto stock=Heightmap::from_polygons(stage.wave.stock,0,4,.1);
  const MillingCutter cutter{CutterShape::Flat,3,20};
  check(stock.clear({to_world(landing.x),to_world(landing.y),10},
                    {to_world(landing.x),to_world(landing.y),0},cutter),
        "The admitted landing must allow a full-radius descent in air");
  const auto rejected=build_toolpath(std::vector{entry_stage(false)},3);
  check(rejected.stats.cut_count==0 && rejected.stats.skipped_entry_count==1,
        "A disconnected band must still reject its entry");
  check(rejected.moves.empty() && rejected.stats.retract_count==0,
        "Rejected entries must leave no phantom landing or rapid rendezvous");
}
void cleanup_requires_roughing() {
  for (bool partial : {false,true}) {
    cam::OperationPlan plan;plan.levels.resize(1);auto& level=plan.levels.front();
    level.floor_z=0;level.axial_depth=1;
    level.part_slice={box(6,6,14,14)};
    level.incoming_stock=level.stock_slice={box(0,0,20,20)};
    if (partial) level.wave_stages.push_back(entry_stage(true));
    level.wave_stages.push_back(entry_stage(false));
    // A rejected entry may nevertheless have a complete theoretical wave.
    level.wave.reached_generations={{box(-10,-10,30,30)}};
    cam::Operation operation;operation.tool_id="mill";operation.stepover_ratio=.15;
    operation.rough_boundary_cleanup=true;
    cam::Tool tool;tool.id="mill";tool.name="Mill";tool.diameter=6;tool.flute_length=20;
    const cam::CuttingFeeds feeds{10000,.01,2,100,2000};
    cam::plan_toolpaths(plan,operation,tool,{},feeds,{});
    check(plan.ok(),"An omitted entry is reported without failing the operation");
    check(level.toolpath.stats.skipped_entry_count==1,
          "The unfinished roughing entry must remain visible");
    if (!partial) {
      check(level.toolpath.moves.empty() && level.nothing_to_cut,
            "A rejected pass must emit neither cleanup nor rapid waypoints");
    } else {
      check(level.toolpath.stats.cut_count>0,"An accepted roughing cut is retained");
      for (const auto& move : level.toolpath.moves) for (const auto& p : move.path)
        check(p.y<0,"Partially successful roughing must not authorize boundary cleanup");
    }
  }
}

void write_box(const std::filesystem::path& file,double x0,double x1) {
  std::ostringstream out;
  for(auto p:{Vec3d{x0,-.5,0},{x1,-.5,0},{x1,.5,0},{x0,.5,0},
              {x0,-.5,3},{x1,-.5,3},{x1,.5,3},{x0,.5,3}})
    out<<"v "<<p.x<<' '<<p.y<<' '<<p.z<<'\n';
  out<<"f 1 4 3 2\nf 5 6 7 8\nf 1 2 6 5\nf 2 3 7 6\nf 3 4 8 7\nf 4 1 5 8\n";
  cam::write_file(file,out.str());
}
cam::MotionPhase phase(Vec3d from,Vec3d to,bool rapid) {
  cam::MotionPhase p;p.from={to_integer(from.x),to_integer(from.y)};
  p.to={to_integer(to.x),to_integer(to.y)};p.from_z=from.z;p.to_z=to.z;
  p.length=distance(from,to);p.rate=rapid?3000:1000;p.rapid=rapid;
  p.kind=rapid?MoveKind::Retract:MoveKind::Cut;return p;
}
void subdivision_arrival() {
  cam::OperationPlan previous;previous.levels.resize(1);
  previous.levels[0].feeds.motion={phase({-12,-10,0},{-12,-10,30},true)};
  cam::OperationPlan current;current.levels.resize(1);auto& level=current.levels[0];
  const auto stage=entry_stage(true);
  level.floor_z=0;level.axial_depth=1;
  level.incoming_stock=level.stock_slice=stage.wave.stock;
  level.wave_stages={stage};
  cam::Operation operation;operation.tool_id="mill";operation.stepover_ratio=.15;
  operation.rough_boundary_cleanup=false;
  cam::Tool tool;tool.id="mill";tool.name="Mill";tool.diameter=6;tool.flute_length=20;
  const cam::CuttingFeeds feeds{10000,.01,2,100,2000};
  cam::plan_toolpaths(current,operation,tool,{},feeds,{},0,1,30,{},&previous);
  check(current.ok() && !level.feeds.motion.empty(),"A later subdivision must emit its real cut");
  check(level.feeds.motion.front().arrival &&
        level.feeds.motion.front().from==previous.levels[0].feeds.motion.back().to,
        "A subdivision must arrive from the actual previous group's resting position");
  std::size_t arrivals=0;
  for(const auto& motion:level.feeds.motion)if(motion.arrival)++arrivals;
  check(arrivals==1,"The subdivision must carry exactly one opening arrival");
}
void fixture_links(const std::filesystem::path& directory) {
  cam::Tool tool;tool.id="mill";tool.name="Mill";tool.diameter=.5;
  tool.flute_length=20;tool.cutting={{"test",10000,.01,100}};
  cam::Machine machine;machine.id="test";machine.max_rpm=20000;
  machine.rapid_feed=3000;machine.rapid_feed_z=1000;
  const auto feeds=cam::resolve_cutting_feeds(tool,machine,"test",{});
  const auto stock=std::make_shared<const Heightmap>(Heightmap::box({{-20,-20,0},{-10,-10,4},true},.1));
  for(int mode=0;mode<3;++mode) {
    const auto file=directory/("fixture-"+std::to_string(mode)+".obj");
    write_box(file,mode==0?10:.8,mode==0?11:1.2);
    cam::Job job;job.rapid_clearance_height=17;job.planning.simulation_pitch=.1;
    cam::SetupContext::ProtectedModel fixture;fixture.mesh=file;fixture.clearance=.1;
    job.fixtures={fixture};cam::JobGeometry geometry;geometry.configure(job);
    cam::Operation operation;operation.tool_id=tool.id;operation.mode=cam::OperationMode::Roughing;
    cam::OperationPlan before;before.stock_after=stock;
    cam::OperationPlan plan;plan.levels.resize(2);
    auto& a=plan.levels[0];auto& b=plan.levels[1];
    a.floor_z=2;b.floor_z=1;a.axial_depth=b.axial_depth=1;
    a.feeds.motion={phase({-.5,0,2},{0,0,2},false),phase({0,0,2},{0,0,20},true)};
    b.feeds.motion={phase({0,0,20},{2,0,20},true),phase({2,0,20},{2,0,1},true),
                    phase({2,0,1},{2.5,0,1},false),phase({2.5,0,1},{2.5,0,20},true)};
    if(mode==2)a.machining_limit=b.machining_limit={box(-1,-.1,3,.1)};
    cam::simulate_operation_xyz(plan,geometry,job,operation,tool,machine,feeds,{&before});
    check(plan.ok(),"Fixture-aware linking must retain a valid simulation: "+plan.error);
    check(a.stays_down_after==(mode!=2),std::string(mode==2?
          "A fixture blocking the complete allowed corridor must retain a retract":
          "Fixtures must permit clear direct or routed inter-level links")+" mode="+std::to_string(mode));
    for(const auto& level:plan.levels)for(const auto& p:level.feeds.motion)
      check(geometry.fixture_clear(cam::motion_point(p,0),cam::motion_point(p,1),cam::milling_cutter(tool)),
            "The optimizer must not introduce a fixture collision");
  }
}
}
int main() {
  const auto directory=std::filesystem::temp_directory_path()/cam::new_document_id();
  std::filesystem::create_directories(directory);int result=0;
  try {landings_and_rejected_entries();cleanup_requires_roughing();fixture_links(directory);
    subdivision_arrival();
    std::cout<<"Physical landing clearance, rejected entries, cleanup gating and fixture-aware links passed.\n";
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';result=1;}
  std::filesystem::remove_all(directory);return result;
}
