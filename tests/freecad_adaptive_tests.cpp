#include "cam/adaptive_roughing.h"
#include "cam/job_internal.h"
#include "../third_party/freecad_adaptive/bridge.h"
#include <iostream>
#include <stdexcept>

namespace {
void check(bool value,const char* message) {if(!value) throw std::runtime_error(message);}
wave::Path64 box(double x0,double y0,double x1,double y1) {
  return {{wave::to_integer(x0),wave::to_integer(y0)},
          {wave::to_integer(x1),wave::to_integer(y0)},
          {wave::to_integer(x1),wave::to_integer(y1)},
          {wave::to_integer(x0),wave::to_integer(y1)}};
}
cam::LevelPlan pocket() {
  cam::LevelPlan level;level.floor_z=0;level.axial_depth=2;level.radial_width=.8;
  level.incoming_stock=level.stock_slice={box(0,0,20,20)};
  level.machining_limit={box(3,3,17,17)};
  return level;
}
}
int main() {
  try {
    // Chunk endpoints may differ after smoothing. They are connected using
    // FreeCAD's declared motion, without introducing a clearance-plane visit.
    wave::Toolpath chunks;
    chunks.moves.push_back({wave::MoveKind::Cut,{{-1000,0},{0,0}},{},1});
    freecad_adaptive::Region region;
    region.paths={{0,{{.001,0},{1,0}}},{1,{{1.001,0},{2,0}}},
                  {0,{{2.001,0},{3,0}}},{2,{{3,0}}},
                  {2,{{3,0},{4,0}}},{0,{{4.001,0},{5,0}}}};
    cam::detail::append_adaptive_paths(chunks,region);
    check(chunks.moves.size()==6,"No-op engine links must disappear");
    std::size_t retracts=0;
    for(std::size_t i=1;i<chunks.moves.size();++i) {
      const auto& move=chunks.moves[i];
      check(move.path.front()==chunks.moves[i-1].path.back(),"Adaptive chunks must remain continuous");
      if(move.kind==wave::MoveKind::Retract) {
        ++retracts;
        check(move.path.front()!=move.path.back(),"A retract must actually change XY position");
      }
    }
    check(retracts==1,"Only FreeCAD's actual uncleared traverse may retract");
    const cam::CuttingFeeds chunk_feeds{10000,.01,2,100,1000};
    const auto schedule=cam::plan_feeds(chunks,{},chunk_feeds,0,10,{},true);
    std::size_t lifts=0;
    for(const auto& phase:schedule.motion) if(phase.to_z>phase.from_z) ++lifts;
    check(lifts==2,"Feed scheduling may lift only for the real traverse and pass end");

    cam::Operation operation;operation.radial_stock_to_leave=.2;
    auto level=pocket();
    cam::detail::generate_adaptive_roughing(level,operation,1,.8,{});
    check(level.adaptive_roughing && !level.toolpath.moves.empty(),"FreeCAD must emit real cutting paths");
    check(!level.adaptive_helices.empty(),"A fully enclosed area must retain a helix entry");
    for(const auto& move:level.toolpath.moves) if(move.kind!=wave::MoveKind::Retract)
      for(const auto& p:move.path)
        check(p.x>=3000 && p.x<=17000 && p.y>=3000 && p.y<=17000,"Adaptive paths must respect centre limits");
    cam::OperationPlan plan;plan.levels.push_back(level);
    cam::Tool tool;tool.id="test";tool.name="Test";tool.diameter=2;tool.flute_length=20;
    const cam::CuttingFeeds feeds{10000,.01,2,100,1000};
    cam::plan_toolpaths(plan,operation,tool,{},feeds,{},0,1,10);
    check(plan.ok() && !plan.levels[0].feeds.motion.empty(),"Helical roughing must produce a feed schedule");
    bool helix=false;
    for(const auto& p:plan.levels[0].feeds.motion)
      if(p.arc && p.to_z<p.from_z) {
        helix=true;
        check(!p.rapid && p.kind==wave::MoveKind::Entry,"Helix descent must be cutting motion");
        check(p.to_z>=0 && p.from_z<=2,"Helix stays within its axial slab");
      }
    check(helix,"Helix metadata must become actual XYZ arcs");
    auto fixtures=pocket();fixtures.fixture_keep_out=Clipper2Lib::InflatePaths(
        wave::Paths64{box(9,9,11,11)},1000,Clipper2Lib::JoinType::Round,Clipper2Lib::EndType::Polygon);
    cam::detail::generate_adaptive_roughing(fixtures,operation,1,.8,{});
    check(!fixtures.toolpath.moves.empty(),"Fixture avoidance must retain usable cuts");
    cam::OperationPlan fixture_plan;fixture_plan.levels.push_back(fixtures);
    auto coarse_sampling=operation;coarse_sampling.path_pitch=5;
    cam::plan_toolpaths(fixture_plan,coarse_sampling,tool,{},feeds,{},0,1,10);
    check(fixture_plan.ok() && !fixture_plan.levels[0].nothing_to_cut,
          "Coarse resampling must retain adaptive fixture paths");
    for(const auto& move:fixture_plan.levels[0].toolpath.moves) if(move.kind!=wave::MoveKind::Retract)
      for(std::size_t i=1;i<move.path.size();++i) {
        Clipper2Lib::Clipper64 clip;clip.AddOpenSubject({{move.path[i-1],move.path[i]}});
        clip.AddClip(Clipper2Lib::InflatePaths(fixtures.fixture_keep_out,-2,
            Clipper2Lib::JoinType::Round,Clipper2Lib::EndType::Polygon));
        wave::Paths64 closed,open;clip.Execute(Clipper2Lib::ClipType::Intersection,
            Clipper2Lib::FillRule::NonZero,closed,open);
        check(open.empty(),"Adaptive cutting and clear links cannot cross fixture keepouts");
      }
    bool cancelled=false;
    try {auto next=pocket();cam::detail::generate_adaptive_roughing(next,operation,1,.8,
          [](double){return false;});}
    catch(const std::runtime_error& e) {cancelled=std::string(e.what())=="Cancelled";}
    check(cancelled,"Cancellation must discard a partial FreeCAD solve");
    std::cout<<"Adaptive paths, helix motion, bounds, fixtures and cancellation passed.\n";
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
