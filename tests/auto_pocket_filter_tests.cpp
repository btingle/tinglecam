#include "cam/auto_pockets.h"
#include "cam/parametric.h"
#include <clipper2/clipper.h>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool value, const std::string& message) {
  if (!value) throw std::runtime_error(message);
}
wave::Path64 rectangle(double x, double y, double width, double height) {
  return {{wave::to_integer(x-width/2),wave::to_integer(y-height/2)},
          {wave::to_integer(x+width/2),wave::to_integer(y-height/2)},
          {wave::to_integer(x+width/2),wave::to_integer(y+height/2)},
          {wave::to_integer(x-width/2),wave::to_integer(y+height/2)}};
}
struct Fixture {
  std::filesystem::path path = std::filesystem::temp_directory_path() /
      ("tinglecam-pocket-filter-" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()) + ".obj");
  Fixture() {
    using namespace Clipper2Lib;
    const auto outer=rectangle(32.5,10,65,20);
    wave::Paths64 holes{rectangle(10,10,8,8),rectangle(25,10,2,2),
                       rectangle(40,10,8,8),rectangle(55,10,3.21,3.21)};
    std::ofstream out(path);
    std::size_t vertex=0;
    const auto face=[&](const wave::Paths64& region,double z) {
      wave::Paths64 triangles;
      check(Triangulate(region,triangles)==TriangulateResult::success,"Fixture triangulation failed");
      for(const auto& t:triangles) {
        for(const auto& p:t) out<<"v "<<wave::to_world(p.x)<<' '<<wave::to_world(p.y)<<' '<<z<<'\n';
        out<<"f "<<vertex+1<<' '<<vertex+2<<' '<<vertex+3<<'\n';vertex+=3;
      }
    };
    face(Difference({outer},holes,FillRule::NonZero),10);
    face({outer},0);
    for(std::size_t i=0;i<holes.size();++i) face({holes[i]},i==2?2:8);
    check(bool(out),"Unable to write test mesh");
  }
  ~Fixture() {std::error_code ignored;std::filesystem::remove(path,ignored);}
};
bool warning_contains(const cam::OperationPlan& plan,const std::string& text) {
  for(const auto& warning:plan.warnings) if(warning.find(text)!=std::string::npos) return true;
  return false;
}
}
int main() {
  try {
    Fixture fixture;
    cam::JobGeometry geometry;geometry.load_part(fixture.path);geometry.set_box_stock(true,2);
    cam::Tool tool;tool.id="tool";tool.name="Test cutter";tool.diameter=3.175;tool.flute_length=4;
    cam::Machine machine;machine.id="machine";machine.name="Test machine";machine.rapid_feed=3000;machine.rapid_feed_z=500;
    machine.min_rpm=1000;machine.max_rpm=24000;machine.feed_ranges.push_back({"material",500,1500});
    cam::Operation op;op.mode=cam::OperationMode::Pocket;op.auto_identify_pockets=true;
    op.tool_id=tool.id;op.machine_id=machine.id;op.material_id="material";
    op.pocket_auto.check_depth=1;op.stepdown=1;op.spindle_rpm=12000;op.plunge_feed=100;
    check(cam::identify_pockets(geometry,op).size()==4,"Fixture must detect all four pockets before tool filtering");
    const auto plan=[&](const cam::Operation& operation,const cam::Tool& cutter) {
      return cam::plan_parametric_operation(operation,cutter,machine,20,{}, {},0,1,&geometry);
    };
    auto mixed=plan(op,tool);
    check(mixed.ok(),mixed.error);
    check(mixed.identified_pockets.size()==1 && !mixed.levels.empty(),"Only the machinable pocket must be retained");
    check(std::abs(mixed.identified_pockets.front().center_x-10)<.001,"Wrong pocket retained");
    check(warning_contains(mixed,"detected 4; planned 1; excluded 3"),"Missing exclusion totals");
    check(warning_contains(mixed,"1 too small") && warning_contains(mixed,"1 insufficient helix clearance") &&
          warning_contains(mixed,"1 depth exceeds"),"Missing counts by exclusion reason");
    for(const auto& level:mixed.levels)
      for(const auto& phase:level.feeds.motion) if(!phase.rapid)
        check(wave::to_world(phase.to.x)<15,"Excluded pockets must have no cutting motion");
    auto margin=op;margin.pocket_radial_margin=2;
    auto offset=plan(margin,tool);
    check(offset.ok() && offset.identified_pockets.size()==1,"Margin-erased pockets must not abort planning");
    check(warning_contains(offset,"2 no single pocket remains"),"Margin exclusion count missing");
    auto oversized=tool;oversized.diameter=20;
    auto empty=plan(op,oversized);
    check(empty.ok() && empty.levels.empty() && empty.identified_pockets.empty(),"All excluded must be a successful no-op");
    check(empty.composed_stock && !empty.preview_context.empty(),"No-op must preserve incoming stock for later operations");
    check(warning_contains(empty,"planned 0; excluded 4") && warning_contains(empty,"4 too small"),"All-excluded log missing");
    auto smaller=tool;smaller.diameter=1;smaller.flute_length=12;
    auto retried=plan(op,smaller);
    check(retried.ok() && retried.identified_pockets.size()==4,
          "Changing to a suitable tool must restore previously excluded pockets");
    check(!warning_contains(retried,"excluded"),"Replanning must not retain stale exclusions");
    auto invalid=op;invalid.stepdown=-1;
    check(!plan(invalid,tool).ok(),"Invalid operation settings must still fail");
    auto manual=op;manual.auto_identify_pockets=false;
    manual.pockets.front().width=2;manual.pockets.front().height=2;manual.pockets.front().radius=0;
    check(!plan(manual,tool).ok(),"Manual pockets must still report an incompatible tool as an error");
    cam::Job job;job.operations={op};
    auto full=cam::plan_job(geometry,job,{{tool}},{{machine}},{{{"material","Material"}}},{});
    check(full.ok(),full.error);
    bool logged=false;
    for(const auto& warning:full.warnings) if(warning.find("excluded 3")!=std::string::npos) logged=true;
    check(logged,"Exclusion summary must reach the job warnings used by GUI and CLI logs");
    auto no_op_job=cam::plan_job(geometry,job,{{oversized}},{{machine}},{{{"material","Material"}}},{});
    check(no_op_job.ok() && no_op_job.operations.front().levels.empty(),
          "A job with only excluded pockets must not fail");
    std::cout<<"Automatic pocket filtering, exclusion logs, no-op stock and manual validation passed.\n";
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
