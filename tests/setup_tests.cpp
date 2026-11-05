#include "cam/config.h"
#include "cam/heightmap_simulation.h"
#include "cam/post.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace {
void check(bool value,const std::string& message) {if(!value) throw std::runtime_error(message);}
void near(double a,double b,const char* message) {check(std::abs(a-b)<1e-5,message);}
void write_box(const std::filesystem::path& path,double x0,double y0,double z0,double x1,double y1,double z1) {
  std::ostringstream s;
  for(auto p:{wave::Vec3d{x0,y0,z0},{x1,y0,z0},{x1,y1,z0},{x0,y1,z0},
              {x0,y0,z1},{x1,y0,z1},{x1,y1,z1},{x0,y1,z1}})
    s<<"v "<<p.x<<' '<<p.y<<' '<<p.z<<'\n';
  s<<"f 1 4 3 2\nf 5 6 7 8\nf 1 2 6 5\nf 2 3 7 6\nf 3 4 8 7\nf 4 1 5 8\n";
  cam::write_file(path,s.str());
}
void heightmap_transfer() {
  wave::Heightmap source(-1,-2,3,2,.25);
  for(int r=0;r<source.rows();++r) for(int c=0;c<source.columns();++c)
    source.set(c,r,-3+.02*c,1+.03*r+.01*c);
  source.set(2,3,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN());
  std::vector<cam::ModelOrientation> rotations(1);
  for(std::size_t i=0;i<rotations.size();++i) for(int axis=0;axis<3;++axis) {
    auto next=rotations[i];next.quarter_turn(axis,1);
    if(std::find(rotations.begin(),rotations.end(),next)==rotations.end()) rotations.push_back(next);
  }
  for(const auto rotation:rotations) {
    cam::ModelTransform placement{rotation,{7,11,-2}};
    if(!placement.preserves_columns()) {
      bool rejected=false;try {(void)source.transformed(placement);}catch(const std::invalid_argument&){rejected=true;}
      check(rejected,"A sideways heightmap must require explicit conversion");continue;
    }
    const auto moved=source.transformed(placement);
    near(source.volume(),moved.volume(),"Direct stock transfer preserves volume");
    for(int r=0;r<source.rows();++r) for(int c=0;c<source.columns();++c) {
      const auto p=placement.apply(wave::Vec3d{source.x(c),source.y(r),0});
      const int col=int(std::floor((p.x-moved.lower_x())/moved.pitch()));
      const int row=int(std::floor((p.y-moved.lower_y())/moved.pitch()));
      const double low=source.bottom(c,r),high=source.top(c,r);
      if(std::isnan(high)) {check(std::isnan(moved.top(col,row)),"Transfer preserves empty cells");continue;}
      near(moved.bottom(col,row),(rotation.axes[2]>0?low:-high)-2,"Transfer remaps lower heights");
      near(moved.top(col,row),(rotation.axes[2]>0?high:-low)-2,"Transfer remaps upper heights");
    }
    auto edited=moved;edited.set(0,0,0,1);
    check(source.top(0,0)==1,"Downstream edits cannot mutate the source result");
  }
  bool rejected=false;
  try {(void)source.transformed({{},{1e100,0,0}});}catch(const std::invalid_argument&) {rejected=true;}
  check(rejected,"Out-of-range stock placement must fail without corrupting its source");
  auto translated=source.transformed({{}, {1,2,0}});
  check(translated.shared_tiles(source)>0,"XY translations share copy-on-write height storage");
}
cam::Tool tool() {
  cam::Tool t;t.id="test";t.name="Test mill";t.diameter=1;t.flute_length=20;
  t.cutting={{"plastic",10000,.01,100}};return t;
}
cam::Machine machine() {cam::Machine m;m.id="test";m.max_rpm=20000;return m;}
cam::Operation facing(const std::string& setup,double top,double depth) {
  cam::Operation op;op.id=cam::new_document_id();op.setup_id=setup;op.tool_id="test";
  op.mode=cam::OperationMode::PlaneClearing;op.name="Face";op.stepdown=.5;
  op.plane.areas={{6,4,14,10,top,depth}};return op;
}
void check_plan(const cam::JobPlan& plan) {
  std::string error=plan.error;
  for(const auto& op:plan.operations) if(!op.ok()) error+='\n'+op.error;
  check(plan.ok(),"Setup planning failed: "+error);
  for(const auto& op:plan.operations) check(op.ok(),"Operation failed: "+error);
}
void setups(const std::filesystem::path& directory) {
  write_box(directory/"part.obj",4,3,0,8,5,2);
  write_box(directory/"stock.obj",0,0,0,12,8,4);
  cam::Job job;job.name="Two sided";job.machine_id="test";job.material_id="plastic";
  job.part_mesh=directory/"part.obj";job.stock_mesh=directory/"stock.obj";job.planning.simulation_pitch=.25;
  cam::ensure_setup_ids(job);
  auto& first=job.setups.front();first.name="Top";
  job.operations.push_back(facing(first.id,4,1));
  cam::Setup second;static_cast<cam::SetupContext&>(second)=first;
  second.id=cam::new_document_id();second.name="Bottom";
  second.part_from_setup=first.id;second.part_mesh.clear();
  second.part_orientation={{1,-2,-3}};second.part_offset_y=8;second.part_offset_z=4;
  second.stock_from_operation=job.operations.front().id;
  second.stock_transfer={{{1,-2,-3}},{0,8,4}};
  second.work_offset="G55";second.origin_x=20;second.origin_z=-4;
  job.setups.push_back(second);
  job.operations.push_back(facing(second.id,4,.5));
  check(cam::validate_setups(job).empty(),"Two-sided inheritance must validate");
  cam::JobGeometry geometry;geometry.configure(job.setups.front());
  auto second_view=geometry.context_view(cam::resolve_setup_context(job,second.id));
  check(geometry.part().source_identity()==second_view.part().source_identity(),"Setups share their immutable part source");
  const cam::ToolLibrary tools{{tool()}};const cam::MachineLibrary machines{{machine()}};
  const cam::MaterialLibrary materials{{{"plastic","Plastic"}}};
  const auto plan=cam::plan_job(geometry,job,tools,machines,materials,{});check_plan(plan);
  near(plan.operations[0].stock_after->highest(),3,"First setup faces stock down to Z3");
  near(plan.operations[1].stock_before->pitch(),.25,"Inherited stock uses the job's grid pitch");
  near(plan.operations[1].stock_before->lowest(),1,"Flipped setup inherits the machined side as its lower surface");
  near(plan.operations[1].stock_before->highest(),4,"Flipped setup exposes the original bottom");
  near(plan.operations[1].stock_before->volume(),plan.operations[0].stock_after->volume(),"Setup transfer preserves remaining material");
  check(plan.operations[1].stock_after->volume()<plan.operations[1].stock_before->volume(),"Second setup removes stock from the other side");
  near(plan.operations[0].stock_after->highest(),3,"Second setup leaves the source snapshot unchanged");
  const auto posted=cam::post_job(plan,job,tools,machines);
  check(posted.ok() && posted.text.find("G55")!=std::string::npos && posted.text.find("Setup: Bottom")!=std::string::npos &&
      posted.text.find("M0")!=std::string::npos,"Export switches contexts with a pause and work offset");
  const auto only_bottom=cam::post_operation(plan,job,tools,machines,1);
  check(only_bottom.ok() && only_bottom.text.find("G55")!=std::string::npos,"Selected export resolves context without exporting the stock-source operation");
  const auto text=cam::to_json_text(job);
  const auto loaded=cam::job_from_json_text(text);
  check(cam::to_json_text(loaded)==text,"Setup contexts, placements and stable result references round-trip exactly");
  const auto original_id=job.operations.front().id;
  job.operations.front().name="Renamed source";
  check(job.setups[1].stock_from_operation==original_id,"Renaming never changes stock references");
  cam::PriorOperations earlier{&plan.operations.front()};
  const auto partial=cam::plan_operation_range(geometry,job,tools,machines,materials,{},1,1,earlier);
  check(partial.error.empty() && partial.operations.front().ok(),"Partial replanning can inherit a stock result from another setup");
  near(partial.operations.front().stock_before->volume(),plan.operations[1].stock_before->volume(),"Partial and full plans inherit the same stock");
  auto failed=plan.operations.front();failed.error="failed";
  const auto missing=cam::plan_operation_range(geometry,job,tools,machines,materials,{},1,1,{&failed});
  check(!missing.operations.front().ok() && missing.operations.front().error.find("stock source")!=std::string::npos,
      "A failed stock source must never revert to the original blank");
  auto with_extra=job;
  with_extra.operations.insert(with_extra.operations.begin()+1,facing(job.setups.front().id,3,.25));
  const auto extra=cam::plan_job(geometry,with_extra,tools,machines,materials,{});check_plan(extra);
  near(extra.operations[1].stock_after->highest(),2.75,"A later operation changes its own stock result");
  near(extra.operations[2].stock_before->lowest(),1,"An explicit stock reference keeps its selected stage, not the last operation in the source setup");
  near(extra.operations[0].stock_after->highest(),3,"Subsequent operations preserve referenced stock snapshots");
  job.setups[1].stock_transfer.orientation.quarter_turn(0,1);
  check(!cam::validate_setups(job).empty(),"Unsupported inherited stock orientation is explicit");
  job.setups[1].stock_transfer=second.stock_transfer;
  job.setups[1].part_from_setup=job.setups[1].id;
  check(!cam::validate_setups(job).empty(),"Cyclic inheritance is rejected");
}
void raised_rim_roughing(const std::filesystem::path& directory) {
  // A ball pass leaves a thin curved lip at the original bottom. After a
  // flip it is suspended above the next roughing floor, outside the part.
  auto source=wave::Heightmap::box({{0,0,0},{12,8,4},true},.125);
  source.cut({2,-2,0},{2,10,0},{wave::CutterShape::Ball,2,20});
  auto flipped=std::make_shared<const wave::Heightmap>(
      source.transformed({{{1,-2,-3}},{0,8,4}}));
  check(wave::material_area(flipped->slice(1))<wave::material_area(flipped->footprint())-1,
        "The flipped ball-mill lip lies above, rather than across, the roughing floor");
  write_box(directory/"rim-part.obj",4,3,2,8,5,4);
  cam::JobGeometry geometry;geometry.load_part(directory/"rim-part.obj");
  geometry.set_inherited_stock(flipped);
  cam::Operation op;op.tool_id="test";op.depth_to=3;op.stepdown=3;
  op.path_pitch=.25;op.stepover_ratio=.25;op.radial_stock_to_leave=.1;
  const cam::CuttingFeeds feeds{10000,.01,1,100,1000};
  auto cutter=tool();cutter.type=cam::ToolType::BallEndMill;
  cam::OperationPlan previous;previous.stock_after=flipped;
  for(const cam::PriorOperations prior:{cam::PriorOperations{},cam::PriorOperations{&previous}}) {
    const auto planned=cam::plan_operation(geometry,op,cutter,machine(),feeds,{},0,1,10,prior);
    check(planned.ok() && planned.levels.size()==1,"Flipped-rim roughing must plan its coarse floor");
    const auto& level=planned.levels.front();
    check(wave::material_area(Clipper2Lib::Difference(flipped->footprint(),level.incoming_stock,
        Clipper2Lib::FillRule::NonZero))<1e-6,
        "Roughing must include the ball-mill lip above its floor, both inherited and after a prior operation");
    const wave::Paths64 raised_band{{{1500,1000},{2500,1000},{2500,7000},{1500,7000}}};
    check(wave::material_area(Clipper2Lib::Difference(raised_band,level.swept,
        Clipper2Lib::FillRule::NonZero))<.001,
        "Roughing paths must sweep the suspended lip rather than treating it as cleared air");
    auto remaining=*flipped;
    const auto shape=cam::milling_cutter(cutter);
    for(const auto& phase:level.feeds.motion) if(!phase.rapid && !phase.arc)
      remaining.cut(cam::motion_point(phase,0),cam::motion_point(phase,1),shape);
    check(!(remaining.top(16,32)>remaining.bottom(16,32)),
        "Executing the roughing paths removes the flipped lip above the floor");
  }
}
void fixtures(const std::filesystem::path& directory) {
  write_box(directory/"clamp.obj",8.5,2,0,12,6,5);
  cam::Job job;job.part_mesh=directory/"part.obj";job.stock_mesh=directory/"stock.obj";
  job.machine_id="test";job.material_id="plastic";job.planning.simulation_pitch=.25;
  cam::SetupContext::ProtectedModel fixture;fixture.mesh=directory/"clamp.obj";fixture.clearance=.2;
  job.fixtures={fixture};cam::ensure_setup_ids(job);
  cam::Operation op;op.id=cam::new_document_id();op.setup_id=job.setups.front().id;
  op.tool_id="test";op.depth_to=3;op.stepdown=1;op.path_pitch=.25;op.radial_stock_to_leave=.1;
  op.rough_boundary_cleanup=true;job.operations={op};
  cam::JobGeometry geometry;geometry.configure(job.setups.front());
  const cam::ToolLibrary tools{{tool()}};const cam::MachineLibrary machines{{machine()}};
  const cam::MaterialLibrary materials{{{"plastic","Plastic"}}};
  const auto plan=cam::plan_job(geometry,job,tools,machines,materials,{});check_plan(plan);
  near(plan.operations.front().stock_before->volume(),384,"Fixtures do not add material to the stock model");
  check(plan.operations.front().stats.removed_volume>0,"Fixture-aware roughing still cuts accessible stock");
  check(plan.operations.front().rapid_plane>5.2,"Safe plane clears the fixture as well as part and stock");
  const auto cutter=cam::milling_cutter(tool());
  for(const auto& level:plan.operations.front().levels) for(const auto& phase:level.feeds.motion)
    check(geometry.fixture_clear(cam::motion_point(phase,0),cam::motion_point(phase,1),cutter),
        "Roughing and boundary cleanup avoid fixture clearance");
  const auto at=plan.operations.front().stock_after;
  const int col=int((10-at->lower_x())/at->pitch()),row=int((4-at->lower_y())/at->pitch());
  near(at->top(col,row),4,"Material beneath a clamp remains stock");
  const auto encoded=cam::to_json_text(job);
  check(cam::to_json_text(cam::job_from_json_text(encoded))==encoded,"Fixture roles, clearance and placement persist");
  const auto cached=geometry.fixture_slice_count();
  for(int i=0;i<1000;++i) {
    const double z=double(i)/100;
    check(geometry.fixture_clear({-2,0,z},{2,0,z+.003},cutter),
          "Continuously varying finishing heights remain clear outside fixture geometry");
  }
  check(geometry.fixture_slice_count()==cached,
        "Finishing sweep checks must not build or retain a global fixture outline for every height");
  const auto forbidden=geometry.fixture_keep_out(1,32,.5);
  const wave::Paths64 part{{{4000,3000},{8000,3000},{8000,5000},{4000,5000}}};
  const auto contours=cam::derive_offset_contours(part,.6,nullptr,&forbidden);
  check(!contours.empty(),"Part finishing targets survive fixture clipping");
  for(const auto& contour:contours) for(const auto p:contour.path)
    check(p.x<8300,"Boundary finishing targets the part, never the clamp boundary");

  // Finishing is allowed to enter fixture clearance. Record where it does so,
  // keep the resulting stock usable, and distinguish clear portions in playback.
  cam::Operation finishing = op;finishing.mode=cam::OperationMode::SurfaceFinishing;
  const auto feeds=cam::resolve_cutting_feeds(tool(),machine(),"plastic",{});
  const auto simulate=[&](cam::MotionPhase motion) {
    cam::OperationPlan result;result.planned=true;result.tool_id="test";
    result.levels.emplace_back();result.levels.front().floor_z=motion.to_z;
    result.levels.front().axial_depth=1;result.levels.front().feeds.motion={motion};
    cam::simulate_operation_xyz(result,geometry,job,finishing,tool(),machine(),feeds,
                                {&plan.operations.front()});
    check(result.ok() && bool(result.stock_after),"Fixture contact retains a usable simulated operation: "+result.error);
    check(!result.warnings.empty() && result.warnings.back().find("contact fixture")!=std::string::npos,
          "Fixture contact is reported as a warning rather than a planning error");
    return result;
  };
  cam::MotionPhase line;line.from={5000,4000};line.to={15000,4000};
  line.from_z=line.to_z=4.5;line.length=10;line.rate=1000;line.rapid=true;
  const auto crossing=simulate(line);
  const auto& schedule=crossing.levels.front().feeds;
  check(schedule.motion.size()==1,"A rapid fixture crossing retains its commanded motion");
  const auto& contacts=schedule.motion.front().fixture_contacts;
  check(contacts.size()==1 && contacts.front().begin>0 && contacts.front().end<1,
        "A crossing with clear endpoints records only its colliding interval");
  check(std::abs(contacts.front().from.x-7.794)<.05 &&
        std::abs(contacts.front().to.x-12.706)<.05,
        "Contact locations include the cutter radius and fixture clearance");
  near(crossing.stock_after->volume(),at->volume(),"Fixture geometry never becomes removable stock");
  const auto duration=cam::feed_schedule_seconds(schedule);
  for(double t:{0.,.1,.3,.5,.7,.9,1.,2.}) {
    const auto cursor=cam::feed_motion_at_seconds(schedule,t*duration);
    check(cursor.fixture_contact==(t>=.3 && t<=.7),
          "Playback turns red inside contact and clears before and after it");
  }
  line.rapid=false;line.kind=wave::MoveKind::Cut;
  (void)simulate(line); // Cutting motion, as well as rapids, must continue.
  cam::MotionPhase arc;arc.arc=true;arc.from={13000,4000};arc.to={7000,4000};
  arc.arc_center={10000,4000};arc.from_z=arc.to_z=4.5;
  arc.length=3*std::acos(-1.);arc.rate=1000;arc.rapid=true;
  const auto around=simulate(arc);
  const auto& arc_schedule=around.levels.front().feeds;
  const auto arc_duration=cam::feed_schedule_seconds(arc_schedule);
  check(cam::feed_motion_at_seconds(arc_schedule,.25*arc_duration).fixture_contact &&
        !cam::feed_motion_at_seconds(arc_schedule,.5*arc_duration).fixture_contact &&
        cam::feed_motion_at_seconds(arc_schedule,.75*arc_duration).fixture_contact,
        "Arc playback records separated contacts and clears between them");
  for(double t:{.25,.5,.75}) {
    const auto cursor=cam::feed_motion_at_seconds(arc_schedule,t*arc_duration);
    const auto expected=cam::motion_point(arc,t);
    check(std::abs(wave::to_world(cursor.position.x)-expected.x)<.002 &&
          std::abs(wave::to_world(cursor.position.y)-expected.y)<.002,
          "Partial-arc playback follows the same trajectory as contact detection");
  }
  cam::MotionPhase vertical;vertical.from=vertical.to={10000,4000};
  vertical.from_z=8;vertical.to_z=4.5;vertical.length=3.5;vertical.rate=1000;vertical.rapid=true;
  const auto descent=simulate(vertical);
  const auto& descent_schedule=descent.levels.front().feeds;
  check(!cam::feed_motion_at_seconds(descent_schedule,0).fixture_contact &&
        cam::feed_motion_at_seconds(descent_schedule,cam::feed_schedule_seconds(descent_schedule)).fixture_contact,
        "Split descents preserve contact at the correct height");
  auto posted_plan=plan;posted_plan.operations={crossing};posted_plan.operations.front().operation_id=op.id;
  posted_plan.operations.front().setup_id=op.setup_id;posted_plan.operations.front().rapid_plane=plan.operations.front().rapid_plane;
  check(cam::post_job(posted_plan,job,tools,machines).ok(),"Fixture contact does not prevent export");
}
void empty_roughing_debug(const std::filesystem::path& directory) {
  cam::Job job;job.part_mesh=job.stock_mesh=directory/"stock.obj";
  job.machine_id="test";job.material_id="plastic";job.planning.simulation_pitch=.25;
  cam::Operation op;op.tool_id="test";op.depth_to=1;op.stepdown=.5;
  op.radial_stock_to_leave=0;op.rough_boundary_cleanup=false;job.operations={op};
  cam::ensure_setup_ids(job);
  cam::JobGeometry geometry;geometry.configure(job.setups.front());
  const auto plan=cam::plan_job(geometry,job,{{tool()}},{{machine()}},{{{"plastic","Plastic"}}},{});
  check_plan(plan);
  const auto nothing=[](const std::string& s) { return s.find("Nothing was left to cut")!=std::string::npos; };
  check(std::none_of(plan.warnings.begin(),plan.warnings.end(),nothing),
        "Empty roughing does not produce normal warnings");
  check(std::any_of(plan.debug_messages.begin(),plan.debug_messages.end(),nothing),
        "Empty roughing retains an opt-in debug diagnostic at job level");
}
}
int main() {
  const auto directory=std::filesystem::temp_directory_path()/cam::new_document_id();
  std::filesystem::create_directories(directory);
  try {heightmap_transfer();setups(directory);raised_rim_roughing(directory);fixtures(directory);empty_roughing_debug(directory);std::filesystem::remove_all(directory);}
  catch(const std::exception& e) {std::cerr<<e.what()<<'\n';std::filesystem::remove_all(directory);return 1;}
  std::cout<<"Setup inheritance, transformed resources and fixture roughing checks passed.\n";
}
