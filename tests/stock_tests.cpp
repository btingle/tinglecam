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
void top_down_stock() {
  const auto footprint=rect(0,0,20,20);
  auto stock=wave::Heightmap::from_polygons(footprint,0,10,.1);
  const auto original=stock;
  const wave::MillingCutter cutter{wave::CutterShape::Flat,2,20};
  const auto first=stock.cut({0,5,6},{20,5,6},cutter);
  const auto middle=stock;
  const auto second=stock.cut({0,15,3},{20,15,3},cutter);
  near(original.volume()-stock.volume(),first.volume+second.volume);
  check(stock.top(100,50)==6&&stock.top(100,150)==3,"Top-down cuts retain their distinct floors");
  check(stock.top(100,100)==10,"Uncut stock retains its original top");
  near(original.volume(),4000);
  check(middle.volume()>stock.volume(),"Snapshots preserve earlier stock");
  OperationPlan op;op.stock_before=std::make_shared<const wave::Heightmap>(original);
  op.stock_after=std::make_shared<const wave::Heightmap>(stock);
  const auto slabs=heightmap_slabs(stock,{3,6});
  check(!slabs.empty(),"Heightmaps derive polygon boundaries for existing planners");
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

  // Rebuilding the head must start a fresh grid at the newly requested pitch,
  // while a downstream-only rebuild must keep the predecessor's stock grid.
  Job finer = job;
  finer.operations[0].simulation_pitch = .025;
  const auto fine_head = plan_operation_range(geometry, finer, tools, machines,
                                               materials, {}, 0, 0, {});
  check(fine_head.error.empty() && fine_head.operations[0].ok(),
        "First-operation pitch change replans successfully");
  const auto& fine_grid = *fine_head.operations[0].stock_after;
  near(fine_grid.pitch(), .025);
  check(fine_grid.columns() == whole.operations[0].stock_after->columns() * 4 &&
            fine_grid.rows() == whole.operations[0].stock_after->rows() * 4,
        "Replanning at one quarter pitch creates sixteen times as many cells");
  near(whole.operations[0].stock_after->pitch(), .1);
  const auto fine_tail = plan_operation_range(
      geometry, finer, tools, machines, materials, {}, 1, 1,
      {&fine_head.operations[0]});
  check(fine_tail.error.empty() && fine_tail.operations[0].ok(),
        "Downstream operation replans on the new stock grid");
  near(fine_tail.operations[0].stock_before->pitch(), .025);
  near(fine_tail.operations[0].stock_after->pitch(), .025);
  Job coarser = job;
  coarser.operations[0].simulation_pitch = .2;
  const auto coarse_head = plan_operation_range(
      geometry, coarser, tools, machines, materials, {}, 0, 0, {});
  check(coarse_head.error.empty() && coarse_head.operations[0].ok(),
        "Grid pitch can also be made coarser after a fine plan");
  near(coarse_head.operations[0].stock_after->pitch(), .2);
  check(coarse_head.operations[0].stock_after->columns() * 2 ==
            whole.operations[0].stock_after->columns(),
        "A later rebuild is not stuck with the initial grid dimensions");

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
int main() { try { top_down_stock(); partial_replan(); } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1;} }
