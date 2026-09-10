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
  Operation op; op.mode=OperationMode::Pocket; op.material_id="material"; op.pocket.shape=PocketShape::Rectangle;
  op.pocket.center_x=10; op.pocket.center_y=10; op.pocket.width=12; op.pocket.height=12;
  op.pocket.top_z=9; op.pocket.depth=6; op.stepdown=6;
  OperationPlan rough; LevelPlan old; old.floor_z=6; old.axial_depth=4; old.simulation.removed_material=rect(0,0,10,20); rough.levels.push_back(old);
  auto result=plan_parametric_operation(op,tool,machine,15,{rough},{},0,1,&geometry);
  check(result.ok(),result.error.c_str()); check(result.composed_stock,"Loaded stock enables composition");
  check(result.levels.size()==1 && result.levels[0].local_stock.size()==2,"One new pass retains two previous Z sections");
  near(volume(preview_before_level(result,0)),3200);
  near(volume(preview_before_level(result,1)),3200-result.stats.removed_volume);
  check(result.stats.removed_volume>600 && result.stats.removed_volume<648.1,"Removal reflects prior roughing, not synthetic footprint");
  const auto later=active_stock(geometry,{rough,result},32);
  near(volume(later),3200-result.stats.removed_volume);
  Operation plane = op;
  plane.mode=OperationMode::PlaneClearing;
  plane.plane.top_z=8.5; plane.plane.depth=7; plane.stepdown=2.5;
  plane.plane.areas={{7,10,10,16},{12,10,12,16}};
  const auto facing=plan_parametric_operation(plane,tool,machine,15,{rough,result},{},0,1,&geometry);
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
  op.pocket.center_x=100;
  const auto air=plan_parametric_operation(op,tool,machine,15,{rough},{},0,1,&geometry);
  check(air.ok() && air.levels[0].nothing_to_cut,"Pocket outside active stock cuts nothing");
  near(volume(preview_before_level(air,1)),3200);
  op.pocket.center_x=10; op.pocket.top_z=15; op.pocket.depth=2; op.stepdown=1;
  const auto above=plan_parametric_operation(op,tool,machine,20,{},{},0,1,&geometry);
  check(above.ok() && above.stats.removed_volume==0,"Box stock is bounded in Z");
}
int main() { try { aliases(); integration(); } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1;} }
