#include "cam/stock.h"
#include "cam/parametric.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace cam;
using namespace Clipper2Lib;
void check(bool value,const char* message) { if (!value) throw std::runtime_error(message); }
wave::Paths64 rect(double x,double y,double w,double h) {
  return {{{wave::to_integer(x),wave::to_integer(y)}, {wave::to_integer(x+w),wave::to_integer(y)},
           {wave::to_integer(x+w),wave::to_integer(y+h)}, {wave::to_integer(x),wave::to_integer(y+h)}}};
}
double volume(const std::vector<StockSlab>& stock) {
  double v=0; for (const auto& s : stock) v+=std::abs(Area(*s.region))*(s.top-s.bottom)/std::pow(double(wave::to_integer(1)),2); return v;
}
void near(double a,double b) { check(std::abs(a-b)<1e-5,"Volume conservation"); }
void aliases() {
  const auto shape=std::make_shared<const wave::Paths64>(rect(0,0,20,20));
  std::vector<StockSlab> stock{{0,10,shape}};
  const auto hole=rect(5,5,10,10);
  auto outside=stock_outside(stock,hole,2,8);
  auto inside=stock_inside(stock,hole,2,8);
  check(outside.size()==3,"Two boundary aliases plus clipped middle");
  check(outside.front().region==shape && outside.back().region==shape,"Height aliases share original polygons");
  near(volume(outside)+volume(inside),volume(stock));
  OperationPlan op; op.composed_stock=true; op.preview_context=outside;
  LevelPlan first; first.floor_z=5; first.axial_depth=3; first.stock_slice=hole;
  first.local_stock=stock_inside(stock,hole,5,8); first.simulation.removed_material=hole;
  LevelPlan second=first; second.floor_z=2; second.local_stock=stock_inside(stock,hole,2,5);
  op.levels={first,second};
  near(volume(preview_before_level(op,0)),4000);
  near(volume(preview_before_level(op,1)),3700);
  near(volume(preview_before_level(op,2)),3400);
  // A duplicate later area starts empty. It must not hide the first area's
  // initial material or add duplicate geometry during playback.
  auto duplicate=first; duplicate.local_stock.clear(); op.levels.insert(op.levels.begin()+1,duplicate);
  near(volume(preview_before_level(op,0)),4000);
  near(volume(preview_before_level(op,3)),3400);
}
void integration() {
  const auto file=std::filesystem::temp_directory_path()/"tinglecam-stock-test.obj";
  { std::ofstream out(file); out << "v 0 0 0\nv 20 0 0\nv 20 20 0\nv 0 20 0\nv 0 0 10\nv 20 0 10\nv 20 20 10\nv 0 20 10\n"
    "f 1 3 2\nf 1 4 3\nf 5 6 7\nf 5 7 8\nf 1 2 6\nf 1 6 5\nf 2 3 7\nf 2 7 6\nf 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n"; }
  JobGeometry geometry; geometry.load_part(file); geometry.set_box_stock(true,0); std::filesystem::remove(file);
  Tool tool; tool.id="tool"; tool.name="Tool"; tool.diameter=2; tool.flute_length=20; tool.flutes=2;
  Machine machine; machine.id="machine"; machine.name="Machine"; machine.rapid_feed=3000; machine.rapid_feed_z=500;
  machine.min_rpm=1000; machine.max_rpm=24000; machine.feed_ranges.push_back({"material",500,1500});
  Operation op; op.mode=OperationMode::Pocket; op.material_id="material"; op.pockets.front().shape=PocketShape::RoundedRectangle; op.pockets.front().radius=0;
  op.pockets.front().center_x=10; op.pockets.front().center_y=10; op.pockets.front().width=12; op.pockets.front().height=12;
  op.pockets.front().top_z=9; op.pockets.front().depth=6; op.stepdown=6;
  OperationPlan rough; LevelPlan old; old.floor_z=6; old.axial_depth=4; old.simulation.removed_material=rect(0,0,10,20); rough.levels.push_back(old);
  auto result=plan_parametric_operation(op,tool,machine,15,{&rough},{},0,1,&geometry);
  check(result.ok(),result.error.c_str()); check(result.composed_stock,"Loaded stock enables composition");
  check(result.levels.size()==1 && result.levels[0].local_stock.size()==2,"One new pass retains two previous Z sections");
  near(volume(preview_before_level(result,0)),3200);
  near(volume(preview_before_level(result,1)),3200-result.stats.removed_volume);
  check(result.stats.removed_volume>600 && result.stats.removed_volume<648.1,"Removal reflects prior roughing, not synthetic footprint");
  const auto later=active_stock(geometry,{&rough,&result},32);
  near(volume(later),3200-result.stats.removed_volume);
  Operation plane = op;
  plane.mode=OperationMode::PlaneClearing;
  plane.plane.areas={{7,10,10,16,8.5,7},{12,10,12,16,8.5,7}};
  const auto facing=plan_parametric_operation(plane,tool,machine,15,{&rough,&result},{},0,1,&geometry);
  check(facing.ok(),facing.error.c_str());
  near(volume(preview_before_level(facing,0)),volume(later));
  near(volume(preview_before_level(facing,facing.levels.size())),
       volume(later)-facing.stats.removed_volume);
  double previous_volume=volume(later);
  for (std::size_t i=0;i<facing.levels.size();++i) {
    const auto& level=facing.levels[i];
    const auto before=preview_before_level(facing,i);
    near(volume(before),previous_volume);
    const auto half=stock_outside(before,rect(0,0,10,20),level.floor_z,level.floor_z+level.axial_depth);
    check(volume(half)<=volume(before),"Partial playback never adds material");
    previous_volume=volume(preview_before_level(facing,i+1));
    check(previous_volume<=volume(before)+1e-6,"Overlapping passes monotonically remove stock");
  }
  op.pockets.front().center_x=100;
  const auto air=plan_parametric_operation(op,tool,machine,15,{&rough},{},0,1,&geometry);
  check(air.ok() && air.levels[0].nothing_to_cut,"Pocket outside active stock cuts nothing");
  near(volume(preview_before_level(air,1)),3200);
  op.pockets.front().center_x=10; op.pockets.front().top_z=15;
  op.pockets.front().depth=2; op.stepdown=1;
  const auto above=plan_parametric_operation(op,tool,machine,20,{},{},0,1,&geometry);
  check(above.ok() && above.stats.removed_volume==0,"Box stock is bounded in Z");
}
// A partial re-plan has to produce exactly what planning the whole job
// produces for the operations it covers. The point of it is that the ones it
// does not cover are not recomputed, so what it reads their leftover stock out
// of is the plan already in hand -- and if that handoff differed in any way
// from the one a full run makes, the saving would be bought with a different
// answer.
void partial_replan() {
  const auto file=std::filesystem::temp_directory_path()/"tinglecam-partial-test.obj";
  { std::ofstream out(file); out << "v 0 0 0\nv 20 0 0\nv 20 20 0\nv 0 20 0\nv 0 0 10\nv 20 0 10\nv 20 20 10\nv 0 20 10\n"
    "f 1 3 2\nf 1 4 3\nf 5 6 7\nf 5 7 8\nf 1 2 6\nf 1 6 5\nf 2 3 7\nf 2 7 6\nf 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n"; }
  JobGeometry geometry; geometry.load_part(file); geometry.set_box_stock(true,0);
  std::filesystem::remove(file);
  Tool tool; tool.id="tool"; tool.name="Tool"; tool.diameter=2; tool.flute_length=20; tool.flutes=2;
  Machine machine; machine.id="machine"; machine.name="Machine"; machine.rapid_feed=3000;
  machine.rapid_feed_z=500; machine.min_rpm=1000; machine.max_rpm=24000;
  machine.feed_ranges.push_back({"material",500,1500});
  ToolLibrary tools{{tool}};
  MachineLibrary machines{{machine}};
  MaterialLibrary materials{{{"material","Test material"}}};

  Job job;
  Operation first; first.name="First"; first.mode=OperationMode::PlaneClearing;
  first.tool_id="tool"; first.machine_id="machine"; first.material_id="material";
  first.plane.areas={{5,10,8,16,10,3}}; first.stepdown=3;
  Operation second=first; second.name="Second"; second.mode=OperationMode::Pocket;
  second.pockets.front().shape=PocketShape::RoundedRectangle; second.pockets.front().center_x=10;
  second.pockets.front().center_y=10; second.pockets.front().width=12; second.pockets.front().height=12;
  second.pockets.front().top_z=9; second.pockets.front().depth=5; second.stepdown=5;
  job.operations={first,second};

  const JobPlan whole=plan_job(geometry,job,tools,machines,materials,{});
  check(whole.ok(),whole.error.c_str());
  check(whole.operations.size()==2,"A full plan covers every operation");
  for (const auto& op : whole.operations)
    check(op.planned && op.ok(),"A full plan marks every operation planned");

  // The second operation alone, reading the first out of the plan just made.
  const PriorOperations earlier{&whole.operations[0]};
  const PartialPlan again=plan_operation_range(geometry,job,tools,machines,
                                               materials,{},1,1,earlier);
  check(again.error.empty(),again.error.c_str());
  check(again.first==1 && again.operations.size()==1,"Only the asked-for range comes back");
  check(again.operations[0].planned && again.operations[0].ok(),
        again.operations[0].error.c_str());
  near(again.operations[0].stats.removed_volume,
       whole.operations[1].stats.removed_volume);
  check(again.operations[0].levels.size()==whole.operations[1].levels.size(),
        "A re-planned operation cuts the same passes as a full run");
  near(again.rapid_plane,whole.rapid_plane);

  // Planning the first on its own inherits nothing, so it must reproduce the
  // first operation of a full run rather than the second's leftovers.
  const PartialPlan head=plan_operation_range(geometry,job,tools,machines,
                                              materials,{},0,0,{});
  check(head.error.empty(),head.error.c_str());
  near(head.operations[0].stats.removed_volume,
       whole.operations[0].stats.removed_volume);

  // A range whose predecessors are not accounted for is refused rather than
  // planned against nothing, which would quietly cut air as though it were
  // stock the operation above had already taken.
  const PartialPlan gap=plan_operation_range(geometry,job,tools,machines,
                                             materials,{},1,1,{});
  check(!gap.error.empty(),"A range missing its predecessors must be refused");
  const PartialPlan outside=plan_operation_range(geometry,job,tools,machines,
                                                 materials,{},0,9,{});
  check(!outside.error.empty(),"A range beyond the job must be refused");

  // Splicing one operation's result into a plan and re-totalling it gives the
  // totals of the plan as it now stands, not of the run that touched part of it.
  JobPlan spliced=plan_job(geometry,job,tools,machines,materials,{});
  spliced.operations[1]=again.operations[0];
  summarise_job(spliced);
  near(spliced.stats.removed_volume,whole.stats.removed_volume);
  check(spliced.ok(),"A spliced plan is still a plan");

  // An operation with no result is a gap, not a failure to report.
  JobPlan partial=plan_job(geometry,job,tools,machines,materials,{});
  partial.operations[1]=OperationPlan{};
  partial.operations[1].operation_name="Second";
  partial.operations[1].error="Not planned yet";
  summarise_job(partial);
  check(partial.ok(),"A half-planned job is still usable");
  for (const auto& warning : partial.warnings)
    check(warning.find("Not planned yet")==std::string::npos,
          "An operation nothing has been asked of is not warned about");
  near(partial.stats.removed_volume,whole.operations[0].stats.removed_volume);
}
int main() { try { aliases(); integration(); partial_replan(); } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1;} }
