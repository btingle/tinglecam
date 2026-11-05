#include "cam/config.h"
#include "cam/heightmap_simulation.h"
#include "cam/post.h"
#include "wave/analysis.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
void check(bool b,const std::string& message) {if(!b) throw std::runtime_error(message);}
void near(double a,double b,double e,const char* message) {
  check(std::abs(a-b)<=e,std::string(message)+": "+std::to_string(a)+" / "+std::to_string(b));
}
void part(const std::filesystem::path& path) {
  std::ostringstream out;int index=0;
  for(double y:{6.,30.}) for(double x:{6.,30.}) {
    for(auto p:{wave::Vec3d{x,y,0},{x+4,y,0},{x+4,y+4,0},{x,y+4,0},
                {x,y,4},{x+4,y,4},{x+4,y+4,4},{x,y+4,4}})
      out<<"v "<<p.x<<' '<<p.y<<' '<<p.z<<'\n';
    for(auto face:{std::array{1,4,3,2},std::array{5,6,7,8},std::array{1,2,6,5},
                   std::array{2,3,7,6},std::array{3,4,8,7},std::array{4,1,5,8}}) {
      out<<'f';for(auto v:face) out<<' '<<v+index;out<<'\n';
    }
    index+=8;
  }
  cam::write_file(path,out.str());
}
cam::Tool cutter() {
  cam::Tool t;t.id="mill";t.name="Mill";t.diameter=2;t.flute_length=20;
  t.cutting={{"plastic",10000,.01,100}};return t;
}
cam::Machine machine() {cam::Machine m;m.id="machine";m.max_rpm=20000;return m;}
const cam::ToolLibrary tools{{cutter()}};
const cam::MachineLibrary machines{{machine()}};
const cam::MaterialLibrary materials{{{"plastic","Plastic"}}};
cam::Job job(const std::filesystem::path& mesh) {
  cam::Job j;j.part_mesh=mesh;j.box_stock=true;j.box_stock_margins={6,6,6,6,0,0};
  j.machine_id="machine";j.material_id="plastic";j.planning.simulation_pitch=.25;
  cam::Operation op;op.tool_id="mill";op.depth_to=3;op.stepdown=1;op.stepover_ratio=.4;
  op.path_pitch=.5;op.radial_stock_to_leave=.2;op.finishing_subdivisions=2;
  op.roughing_areas={{"NW",7,7,26,26},{"NE",33,7,26,26},
                    {"SE",33,33,26,26},{"SW",7,33,26,26}};
  j.operations={op};return j;
}
void valid(const cam::JobPlan& plan) {
  check(plan.ok(),"Job failed: "+plan.error);
  for(const auto& op:plan.operations) check(op.ok(),"Operation failed: "+op.error);
}
void bounded_motion(const cam::OperationPlan& op) {
  for(const auto& level:op.levels) {
    check(!level.machining_limit.empty(),"Area passes retain their centre limit");
    const auto b=Clipper2Lib::GetBounds(level.machining_limit);
    for(const auto& phase:level.feeds.motion) if(!phase.rapid) {
      const int pieces=phase.arc?32:1;
      for(int i=0;i<=pieces;++i) {
        const auto p=cam::motion_point(phase,double(i)/pieces);
        check(p.x>=wave::to_world(b.left)-.002 && p.x<=wave::to_world(b.right)+.002 &&
              p.y>=wave::to_world(b.top)-.002 && p.y<=wave::to_world(b.bottom)+.002,
              "Cutting and at-depth links stay within their area");
      }
    }
    near(wave::material_area(level.part_slice),64,1e-5,"All four parts remain protected in every area");
  }
}
void sequence(const std::filesystem::path& mesh) {
  auto j=job(mesh);cam::JobGeometry geometry;geometry.configure(j);
  const auto plan=cam::plan_job(geometry,j,tools,machines,materials,{});valid(plan);
  const auto& op=plan.operations.front();bounded_motion(op);
  check(op.levels.size()==24,"Four areas each complete three coarse depths and three cleanups");
  for(int area=0;area<4;++area) {
    for(int local=0;local<6;++local) {
      const auto& level=op.levels[area*6+local];
      check(level.roughing_area==area && level.roughing_area_name==j.operations[0].roughing_areas[area].name,
            "Passes retain area identity in execution order");
      check((local<3)==(level.finishing_index==0),"An area completes coarse depths before terrace cleanup");
    }
    if(area>0) {
      const auto& before=op.levels[area*6-1];const auto& after=op.levels[area*6];
      near(before.stock_checkpoints.back().stock->volume(),after.stock_checkpoints.front().stock->volume(),
           1e-6,"Next area starts with the previous area's simulated stock");
      check(!before.stays_down_after,"Area boundaries retain a safe-plane retract");
      check(std::any_of(after.feeds.motion.begin(),after.feeds.motion.end(),[](const auto& phase) {
        return phase.rapid && phase.from!=phase.to;
      }),"Area changes include the actual XY rapid from the previous area");
    }
  }
  check(op.stats.removed_volume>100 && op.stats.unsafe_plunges==0,"Area roughing removes stock with the existing entry rules");
  check(op.stats.gouge_area<.001,"Area roughing preserves the protected parts");
  const auto posted=cam::post_job(plan,j,tools,machines);
  check(posted.ok() && posted.text.find("Area 1: NW")<posted.text.find("Area 2: NE") &&
        posted.text.find("Area 2: NE")<posted.text.find("Area 3: SE") &&
        posted.text.find("Area 3: SE")<posted.text.find("Area 4: SW"),
        "Area-first motion and named area boundaries export in order");
  // Each area's simulation must leave every cell outside its full-cutter box intact.
  for(int a=0;a<4;++a) {
    const auto& before=*op.levels[a*6].stock_checkpoints.front().stock;
    const auto& after=*op.levels[a*6+5].stock_checkpoints.back().stock;
    const auto box=Clipper2Lib::GetBounds(cam::roughing_area_boundary(j.operations[0].roughing_areas[a]));
    for(int r=0;r<before.rows();++r) for(int c=0;c<before.columns();++c)
      if(before.x(c)<wave::to_world(box.left)-before.pitch() || before.x(c)>wave::to_world(box.right)+before.pitch() ||
         before.y(r)<wave::to_world(box.top)-before.pitch() || before.y(r)>wave::to_world(box.bottom)+before.pitch()) {
        const auto x=before.top(c,r),y=after.top(c,r);
        check((std::isnan(x)&&std::isnan(y)) || x==y,"An area cannot cut stock beyond its bounding box");
      }
  }
  const auto cancelled=cam::plan_operation_range(geometry,j,tools,machines,materials,
      [](cam::PlanningProgress p,const std::string&) {return p.fraction<.3;},0,0,{});
  check(cancelled.error=="Cancelled" && cancelled.operations.empty(),
        "Cancelling a later area discards the incomplete operation normally");
  const auto text=cam::to_json_text(j);const auto loaded=cam::job_from_json_text(text);
  check(cam::to_json_text(loaded)==text,"Ordered area names and bounds round-trip in job files");
  // The same ordered single-area operations must hand on the same stock.
  auto separate=j;separate.operations.clear();
  for(const auto& area:j.operations[0].roughing_areas) {
    auto one=j.operations[0];one.roughing_areas={area};separate.operations.push_back(one);
  }
  const auto reference=cam::plan_job(geometry,separate,tools,machines,materials,{});valid(reference);
  near(op.stock_after->volume(),reference.operations.back().stock_after->volume(),1e-5,
       "One multi-area operation matches the same areas sequenced as separate operations");
  auto repeated=j;repeated.operations[0].roughing_areas={j.operations[0].roughing_areas[0],j.operations[0].roughing_areas[0]};
  const auto overlap=cam::plan_job(geometry,repeated,tools,machines,materials,{});valid(overlap);
  const auto& overlapping=overlap.operations[0];
  near(overlapping.levels[5].stock_checkpoints.back().stock->volume(),
       overlapping.levels[6].stock_checkpoints.front().stock->volume(),1e-6,
       "Overlapping areas reuse the actual remaining stock");
}
void existing_entries(const std::filesystem::path& mesh) {
  auto j=job(mesh);auto& op=j.operations[0];op.depth_to=1;op.finishing_subdivisions=1;
  op.roughing_areas={{"Interior",14,14,18,18}};
  cam::JobGeometry geometry;geometry.configure(j);
  const auto enclosed=cam::plan_job(geometry,j,tools,machines,materials,{});valid(enclosed);
  check(enclosed.operations[0].stats.cut_levels==0,"Artificial area edges do not create entry fronts");
  cam::OperationPlan opening;opening.planned=true;
  auto stock=wave::Heightmap::box({{0,0,0},{40,40,4},true},.25);
  stock.cut({13,13,0},{13,13,0},{wave::CutterShape::Flat,3,20});
  opening.stock_after=std::make_shared<const wave::Heightmap>(stock);
  j.operations.insert(j.operations.begin(),cam::Operation{});
  const auto planned=cam::plan_operation_range(geometry,j,tools,machines,materials,{},1,1,{&opening});
  check(planned.error.empty() && planned.operations[0].ok(),"Existing pocket entry plans with an area limit");
  const auto& rough=planned.operations[0];bounded_motion(rough);
  check(rough.stats.removed_volume>0 && rough.stats.unsafe_plunges==0,
        "An earlier opening remains a usable roughing entry without changing entry strategy");
  auto inherited_job=j;inherited_job.operations={j.operations.back()};
  inherited_job.operations[0].roughing_areas={{"Flipped",14,26,18,18}};
  const auto inherited=std::make_shared<const wave::Heightmap>(
      stock.transformed({{{1,-2,-3}},{0,40,4}}));
  geometry.set_inherited_stock(inherited);
  const auto transferred=cam::plan_job(geometry,inherited_job,tools,machines,materials,{});valid(transferred);
  check(transferred.operations[0].stats.removed_volume>0 && transferred.operations[0].stats.unsafe_plunges==0,
        "Area limits also use openings in directly inherited flipped stock");
  near(transferred.operations[0].stock_before->volume(),inherited->volume(),1e-6,
       "Area planning starts from the inherited heightmap without rebuilding the original blank");
  auto invalid=j;invalid.operations[1].roughing_areas[0].width=1;
  const auto rejected=cam::plan_operation_range(geometry,invalid,tools,machines,materials,{},1,1,{&opening});
  check(!rejected.operations[0].ok(),"An area narrower than the cutter is rejected");
}
}
int main() {
  const auto directory=std::filesystem::temp_directory_path()/cam::new_document_id();
  std::filesystem::create_directories(directory);
  int result=0;
  try {const auto mesh=directory/"parts.obj";part(mesh);sequence(mesh);existing_entries(mesh);
       std::cout<<"Area-first roughing, domain limits, shared stock and existing entries passed.\n";}
  catch(const std::exception& e) {std::cerr<<e.what()<<'\n';result=1;}
  std::filesystem::remove_all(directory);return result;
}
