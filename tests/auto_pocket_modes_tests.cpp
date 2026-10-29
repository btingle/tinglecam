#include "cam/auto_pockets.h"
#include "cam/config.h"
#include "cam/parametric.h"
#include "cam/post.h"
#include <clipper2/clipper.h>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <stdexcept>

namespace {
constexpr double pi=std::numbers::pi;
void check(bool value,const std::string& text) {if(!value) throw std::runtime_error(text);}
void near(double a,double b,const std::string& text,double tolerance=1e-7) {check(std::abs(a-b)<tolerance,text);}
wave::Point64 p(double x,double y) {return {wave::to_integer(x),wave::to_integer(y)};}
wave::Path64 rectangle(double x,double y,double w,double h) {
 return {p(x-w/2,y-h/2),p(x+w/2,y-h/2),p(x+w/2,y+h/2),p(x-w/2,y+h/2)};
}
struct Fixture {
 std::filesystem::path file=std::filesystem::temp_directory_path()/
  ("tinglecam-pocket-modes-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".obj");
 Fixture() {
  using namespace Clipper2Lib;
  wave::Paths64 holes{Ellipse(p(10,10),2000.,0,128),Ellipse(p(20,10),1587.5,0,128),
   Ellipse(p(35,10),5000.,0,128),rectangle(50,10,4,6),
   {p(56,8),p(60,8),p(60,10),p(58,10),p(58,12),p(56,12)}};
  // A 1 micron seam spur reproduces the jig's false concavity rejection.
  holes[0].insert(holes[0].begin()+1,p(12.001,10.001));
  auto outer=rectangle(32,10,64,20);std::ofstream out(file);std::size_t vertex=0;
  const auto face=[&](const wave::Paths64& region,double z) {
   wave::Paths64 triangles;
   check(Triangulate(region,triangles)==TriangulateResult::success,"Fixture triangulation failed");
   for(const auto& t:triangles) {
    for(auto q:t) out<<"v "<<wave::to_world(q.x)<<' '<<wave::to_world(q.y)<<' '<<z<<'\n';
    out<<"f "<<vertex+1<<' '<<vertex+2<<' '<<vertex+3<<'\n';vertex+=3;
   }
  };
  face(Difference({outer},holes,FillRule::NonZero),10);
  // No floor under the second circle: its ray reaches the stock bottom.
  face(Difference({outer},{holes[1]},FillRule::NonZero),0);
  for(std::size_t i=0;i<holes.size();++i) if(i!=1) face({holes[i]},4);
  check(bool(out),"Unable to write fixture");
 }
 ~Fixture(){std::error_code ec;std::filesystem::remove(file,ec);}
};
cam::Tool cutter(){cam::Tool t;t.id="t";t.name="3.175 mm cutter";t.diameter=3.175;t.flute_length=25;return t;}
cam::Machine machine(){cam::Machine m;m.id="m";m.name="Test";m.rapid_feed=3000;m.rapid_feed_z=500;
 m.min_rpm=1000;m.max_rpm=24000;m.feed_ranges.push_back({"material",500,1500});return m;}
cam::Operation operation(){cam::Operation o;o.name="Holes";o.mode=cam::OperationMode::Pocket;o.auto_identify_pockets=true;
 o.tool_id="t";o.machine_id="m";o.material_id="material";o.stepdown=2.5;return o;}
cam::OperationPlan plan(const cam::JobGeometry& g,const cam::Operation& o,const cam::Tool& t=cutter()) {
 return cam::plan_parametric_operation(o,t,machine(),30,{}, {},0,1,&g);
}
void continuity(const cam::OperationPlan& plan) {
 for(const auto& level:plan.levels) {
  const auto& moves=level.feeds.motion;
  for(std::size_t i=1;i<moves.size();++i) {
   check(moves[i-1].to==moves[i].from,"Motion must remain continuous in XY");
   near(moves[i-1].to_z,moves[i].from_z,"Motion must remain continuous in Z");
  }
 }
}
void test_modes(const cam::JobGeometry& g) {
 auto op=operation();auto detected=cam::identify_pockets(g,op);
 check(detected.size()==4,"Four convex holes must be detected; the real concave hole must remain rejected");
 auto layered=plan(g,op);check(layered.ok(),layered.error);
 check(layered.identified_pockets.size()==3,"Layered mode needs orbital clearance");
 op.pocket_auto.strategy=cam::PocketAutoStrategy::ContinuousHelix;
 auto helix=plan(g,op);check(helix.ok(),helix.error);continuity(helix);
 check(helix.identified_pockets.size()==1 && helix.levels.size()==1,"Continuous helix must select only the small circular hole");
 const auto& level=helix.levels.front();near(level.floor_z,4,"Helix must reach the real floor: "+std::to_string(level.floor_z)+" centre "+std::to_string(helix.identified_pockets.front().center_x));
 near(level.axial_depth,6,"Preview pass spans full pocket depth");
 int descending=0,cleanup=0;bool partial=false;double elapsed=0;
 for(const auto& phase:level.feeds.motion) {
  if(phase.arc) {
   if(phase.from_z>phase.to_z) {
    ++descending;near((phase.from_z-phase.to_z)*2*pi/phase.arc_sweep_radians,op.stepdown,"Exactly one stepdown per revolution");
    if(phase.arc_sweep_radians<2*pi-1e-8) {
     partial=true;const double duration=phase.length/phase.rate*60;
     auto cursor=cam::feed_motion_at_seconds(level.feeds,elapsed+duration/2);
     const double angle=std::atan2(wave::to_world(phase.from.y-phase.arc_center.y),wave::to_world(phase.from.x-phase.arc_center.x))+
                         phase.arc_sweep_radians/2;
     near(wave::to_world(cursor.position.x),10+std::hypot(wave::to_world(phase.from.x-phase.arc_center.x),
          wave::to_world(phase.from.y-phase.arc_center.y))*std::cos(angle),"Partial-turn preview follows its sweep",.002);
    }
   } else {
    ++cleanup;near(phase.to_z,4,"Cleanup must remain at the floor");near(phase.arc_sweep_radians,2*pi,"Full cleanup revolution");
   }
  }
  elapsed+=phase.length/std::max(phase.rate,1.)*60+phase.dwell_seconds;
 }
 check(descending==3 && partial && cleanup==1,"Two full descents, one partial descent, one cleanup turn");
 check(helix.stats.removed_volume>0,"Continuous helix must update material stock");
 op.pocket_auto.strategy=cam::PocketAutoStrategy::Drill;op.pocket_auto.peck_depth=2.5;
 op.pocket_auto.chip_break_retract=.3;op.pocket_auto.bottom_dwell_seconds=.25;
 for(auto cycle:{cam::DrillingCycle::Feed,cam::DrillingCycle::Peck,cam::DrillingCycle::ChipBreak}) {
  op.pocket_auto.drilling_cycle=cycle;auto drilled=plan(g,op);check(drilled.ok(),drilled.error);continuity(drilled);
  check(drilled.identified_pockets.size()==4 && drilled.levels.size()==4,"Drilling must accept nominal same-diameter holes");
  check(drilled.stats.removed_volume>0,"Drilling must remove the central disk from stock");
  int dwells=0,pecks=0;
  for(const auto& pass:drilled.levels) {
   double deepest=10;
   for(const auto& phase:pass.feeds.motion) {
    check(!phase.arc,"Drilling must have no orbital motions");
    if(!phase.rapid) check(phase.from==phase.to,"Drilling must feed only in Z");
    if(phase.dwell_seconds>0) {++dwells;near(phase.to_z,pass.floor_z,"Dwell belongs at the bottom");}
    if(phase.rapid && phase.to_z<phase.from_z)
     check(phase.to_z>=deepest-1e-8,"Rapid re-entry must never pass the previously drilled depth");
    if(!phase.rapid) deepest=std::min(deepest,phase.to_z);
    if(phase.rapid && phase.from_z<10 && phase.to_z>phase.from_z && phase.to_z<30) {
     ++pecks;
     if(cycle==cam::DrillingCycle::Peck) near(phase.to_z,11,"Peck cycle must retract to R clearance");
     else near(phase.to_z-phase.from_z,.3,"Chip break must use configured short retract");
    }
   }
   near(deepest,pass.floor_z,"Drilling reaches full depth");
  }
  check(dwells==4,"One bottom dwell per hole");check((cycle==cam::DrillingCycle::Feed)==(pecks==0),"Only peck cycles perform intermediate retracts");
  auto no_dwell=op;no_dwell.pocket_auto.bottom_dwell_seconds=0;auto baseline=plan(g,no_dwell);
  near((drilled.stats.cutting_minutes-baseline.stats.cutting_minutes)*60,1,"Dwell must contribute to time estimates");
 }
 cam::Tool drill=cutter();drill.type=cam::ToolType::Drill;
 check(plan(g,op,drill).ok(),"Drill tool type must work for centre drilling");
 auto invalid=op;invalid.pocket_auto.peck_depth=0;check(!plan(g,invalid).ok(),"Reject zero peck depth");
 cam::Job job;job.operations={op};job.origin_x=12;job.origin_y=-7;job.origin_z=3;
 auto full=cam::plan_job(g,job,{{cutter()}},{{machine()}},{{{"material","Material"}}},{});check(full.ok(),full.error);
 auto posted=cam::post_job(full,job,{{cutter()}},{{machine()}});check(posted.ok(),posted.error);
 check(posted.text.find("G4 P0.250")!=std::string::npos,"Post must emit bottom dwell without unit conversion");
 check(posted.text.find("G83")==std::string::npos,"Peck paths must expand to controller-independent explicit motions");
 for(auto controller:{cam::Controller::Fanuc,cam::Controller::Marlin,cam::Controller::Smoothieware,cam::Controller::Haas}) {
  auto dialect=machine();dialect.controller=controller;dialect.output_inches=true;
  auto code=cam::post_job(full,job,{{cutter()}},{{dialect}});check(code.ok(),code.error);
  const std::string dwell=controller==cam::Controller::Smoothieware ? "G4 S0.250" :
      controller==cam::Controller::Haas ? "G4 P0.250" : "G4 P250";
  check(code.text.find(dwell)!=std::string::npos,"Controller dwell units must be independent of inch/mm coordinates");
 }
 auto spiral_op=operation();spiral_op.pocket_auto.strategy=cam::PocketAutoStrategy::ContinuousHelix;
 cam::Job spiral_job;spiral_job.operations={spiral_op};
 auto spiral_plan=cam::plan_job(g,spiral_job,{{cutter()}},{{machine()}},{{{"material","Material"}}},{});
 auto spiral_code=cam::post_job(spiral_plan,spiral_job,{{cutter()}},{{machine()}});
 check(spiral_code.ok() && spiral_code.text.find("G3")!=std::string::npos,
       "Continuous helices must post as XYZ arcs");
 const auto serialized=cam::to_json_text(job);const auto loaded=cam::job_from_json_text(serialized);
 check(cam::to_json_text(loaded)==serialized,"Strategy and drilling settings must round-trip");
 auto legacy=cam::job_from_json_text(R"({"operations":[{"mode":"pocket","auto_identify_pockets":true,"pocket_auto":{"check_depth":1}}]})");
 check(legacy.operations.front().pocket_auto.strategy==cam::PocketAutoStrategy::Layered,"Old jobs retain layered strategy");
}
void model_regression(const std::filesystem::path& file) {
 cam::JobGeometry g;g.load_part(file);g.set_box_stock(true,2);auto op=operation();auto holes=cam::identify_pockets(g,op);
 check(holes.size()==24,"Jig model must identify all 24 holes");int four=0,small=0;
 for(const auto& hole:holes) {
  if(std::abs(hole.width-4)<.005)++four;
  if(std::abs(hole.width-3.175)<.005)++small;
 }
 check(four==8 && small==16,"Jig dimensions must remain eight 4 mm holes and sixteen 3.175 mm holes");
 for(auto strategy:{cam::PocketAutoStrategy::Layered,cam::PocketAutoStrategy::ContinuousHelix,cam::PocketAutoStrategy::Drill}) {
  op.pocket_auto.strategy=strategy;auto result=plan(g,op);check(result.ok(),result.error);
  check(result.identified_pockets.size()==(strategy==cam::PocketAutoStrategy::Drill?24:8),"Unexpected jig strategy tool-fit counts");
  continuity(result);
 }
 std::cout<<"Jig: detected 24 (8 x 4 mm, 16 x 3.175 mm); layered/helix 8, drilling 24.\n";
}
}
int main(int argc,char** argv) {
 try {Fixture fixture;cam::JobGeometry g;g.load_part(fixture.file);g.set_box_stock(true,2);test_modes(g);
  if(argc>1)model_regression(argv[1]);
  std::cout<<"Pocket strategies, pecks, dwell, pitch, stock, post and persistence passed.\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
