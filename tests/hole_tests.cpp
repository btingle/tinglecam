#include "cam/holes.h"
#include "cam/config.h"
#include "cam/heightmap_simulation.h"
#include "cam/post.h"
#include "cam/job_internal.h"
#include "cam/hole_config.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
namespace {
void check(bool value,const std::string& message) {if(!value) throw std::runtime_error(message);}
void near(double a,double b,double e,const char* message) {check(std::abs(a-b)<=e,std::string(message)+": "+std::to_string(a)+" / "+std::to_string(b));}
struct Fixture {
  std::filesystem::path directory=std::filesystem::temp_directory_path()/
      ("tinglecam-hole-tests-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  Fixture() {std::filesystem::create_directory(directory);}
  ~Fixture() {std::error_code ec;std::filesystem::remove_all(directory,ec);}
  std::filesystem::path model(bool through=false,int sides=128,double ry_scale=1,bool taper=false) {
    auto path=directory/(std::to_string(through)+"-"+std::to_string(sides)+"-"+std::to_string(ry_scale)+"-"+std::to_string(taper)+".obj");
    std::ofstream out(path);out<<std::setprecision(10);int vertex=1;
    const auto point=[&](double radius,int i,double z) {
      const double a=2*std::numbers::pi*i/sides;
      return wave::Vec3d{12+radius*std::cos(a),-8+radius*ry_scale*std::sin(a),z};
    };
    const auto triangle=[&](wave::Vec3d a,wave::Vec3d b,wave::Vec3d c) {
      for(auto p:{a,b,c}) out<<"v "<<p.x<<' '<<p.y<<' '<<p.z<<'\n';
      out<<"f "<<vertex<<' '<<vertex+1<<' '<<vertex+2<<'\n';vertex+=3;
    };
    const auto ring=[&](double inside,double outside,double z) {
      for(int i=0;i<sides;++i) {
        const auto a=point(inside,i,z),b=point(outside,i,z),c=point(outside,i+1,z),d=point(inside,i+1,z);
        triangle(a,b,c);if(inside>0) triangle(a,c,d);
      }
    };
    ring(6,20,10);
    if(taper) {
      for(int i=0;i<sides;++i) {
        triangle(point(6,i,10),point(4,i,7),point(4,i+1,7));
        triangle(point(6,i,10),point(4,i+1,7),point(6,i+1,10));
      }
      ring(0,20,0);
    } else {ring(4,6,7);ring(2,4,3);ring(through?2:0,20,0);}
    return path;
  }
};
cam::Tool endmill() {
  cam::Tool t;t.id="mill";t.name="2 mm end mill";t.diameter=2;t.flute_length=25;
  t.cutting={{"material",10000,.02,120}};return t;
}
cam::Tool drill(double diameter=4,const char* id="drill") {
  auto t=endmill();t.id=id;t.name="Drill";t.type=cam::ToolType::Drill;t.diameter=diameter;
  t.point_angle_degrees=90;t.cutting={{"material",5000,.02,60}};return t;
}
cam::Machine machine() {
  cam::Machine m;m.id="machine";m.name="Test";m.rapid_feed=3000;m.rapid_feed_z=500;
  m.min_rpm=1000;m.max_rpm=20000;m.tool_change=cam::ToolChange::Automatic;return m;
}
cam::Operation hole() {
  cam::Operation op;op.mode=cam::OperationMode::Hole;op.name="Holes";op.tool_id="mill";
  op.stepdown=2;op.path_pitch=.5;op.holes.helix_angle_degrees=10;return op;
}
cam::Job job(cam::Operation op) {
  cam::Job j;j.material_id="material";j.machine_id="machine";j.simulation_pitch=.25;
  j.operations={std::move(op)};return j;
}
cam::JobPlan plan(const cam::JobGeometry& g,const cam::Job& j) {
  auto p=cam::plan_job(g,j,{{endmill(),drill(),drill(1,"pilot")}},{{machine()}},
      {{{"material","Material"}}},{});
  check(p.ok(),p.error);
  for(const auto& op:p.operations) check(op.ok(),op.error);
  return p;
}
void detection(Fixture& files) {
  cam::JobGeometry g;g.load_part(files.model());
  cam::HoleParameters p;p.automatic=true;
  const auto result=cam::identify_holes(g,p);
  check(result.error.empty(),result.error);check(result.systems.size()==1,"Find one circular opening");
  const auto& h=result.systems.front();
  near(h.center_x,12,.002,"Circle centre X");near(h.center_y,-8,.002,"Circle centre Y");
  check(h.levels.size()==3,"Find two nested counterbores and blind bore");
  for(int i=0;i<3;++i) near(h.levels[i].radius,6-2*i,.003,"Fitted counterbore radius");
  near(h.levels[0].bottom_z,7,.001,"Outer shoulder");near(h.levels[1].bottom_z,3,.001,"Inner shoulder");
  near(h.levels[2].bottom_z,0,.001,"Blind floor");check(!h.levels[2].through,"Blind bore is not through");
  cam::apply_hole_detection(p,result);p.systems[0].levels[1].selected=false;
  p.systems[0].levels[0].strategy=cam::HoleStrategy::Pocket;
  cam::apply_hole_detection(p,result);
  check(!p.systems[0].levels[1].selected && p.systems[0].levels[0].strategy==cam::HoleStrategy::Pocket,
        "Re-identification preserves choices by geometry");
  p.counterbores=false;auto single=cam::identify_holes(g,p);
  check(single.systems.size()==1 && single.systems[0].levels.size()==1,"Counterbore discovery can be disabled");
  p.counterbores=true;p.inspection_depth=4;auto lower=cam::identify_holes(g,p);
  check(lower.systems.size()==1 && lower.systems[0].levels.size()==2,"Inspection can begin below the top counterbore");
  near(lower.systems[0].levels[0].top_z,6,.001,"Inspection start depth");
  p.inspection_depth=0;
  g.load_part(files.model(true));auto through=cam::identify_holes(g,p);
  check(through.systems.size()==1 && through.systems[0].levels.size()==3 && through.systems[0].levels[2].through,
        "Terminate a nested through hole on ray misses");
  g.load_part(files.model(false,6));check(cam::identify_holes(g,p).systems.empty(),
      "Circle tolerance covers polygon edges, not just vertices on a circle");
  g.load_part(files.model(false,128,.96));check(cam::identify_holes(g,p).systems.empty(),"Reject a noncircular opening");
  p.circle_tolerance=.2;check(!cam::identify_holes(g,p).systems.empty(),"User circle tolerance controls acceptance");
  p.circle_tolerance=.03;g.load_part(files.model(false,128,1,true));
  const auto tapered=cam::identify_holes(g,p);
  check(tapered.systems.empty() && !tapered.warnings.empty(),"Tapers do not become many false counterbore levels");
  const auto cancelled=cam::identify_holes(g,p,[](auto,const auto&) {return false;});
  check(cancelled.error=="Cancelled","Independent discovery is cancellable");
}
void drilling_and_motion(Fixture& files) {
  cam::JobGeometry g;g.load_part(files.model());g.set_box_stock(true,{});
  auto op=hole();op.tool_id="drill";op.holes.strategy=cam::HoleStrategy::Drill;
  auto& h=op.holes.systems.front();h.center_x=12;h.center_y=-8;
  h.levels={{}};auto& l=h.levels[0];l.radius=2;l.top_z=10;l.bottom_z=3;
  op.holes.drilling.peck_depth=2;
  auto tip=plan(g,job(op));
  double low=100;int descents=0;
  for(const auto& phase:tip.operations[0].levels[0].feeds.motion) {
    low=std::min(low,phase.to_z);
    if(!phase.rapid && phase.to_z<phase.from_z) ++descents;
  }
  near(low,3,1e-7,"Tip depth stops at requested Z");check(descents>=4,"Drilling pecks descend to progressively deeper floors");
  op.holes.drilling.depth_reference=cam::DrillDepth::FullDiameter;
  auto full=plan(g,job(op));near(full.operations[0].levels[0].floor_z,1,1e-7,"Full diameter depth includes conical tip length");
  op.holes.drilling.depth_adjustment=-1;
  auto adjusted=plan(g,job(op));near(adjusted.operations[0].levels[0].floor_z,2,1e-7,
      "Drilling depth can be adjusted independently of hole dimensions");
  const auto cutter=cam::milling_cutter(drill());
  near(wave::swept_cutter_height(1.5,0,{0,0,10},{0,0,3},cutter),4.5,1e-8,"Simulation cuts the conical drill point");
  auto flat=drill();flat.point_angle_degrees=180;near(cam::drill_tip_height(flat),0,1e-8,"Flat drills have no tip extension");
  flat=drill();flat.tip_flat_diameter=1;near(cam::drill_tip_height(flat),1.5,1e-8,"Tip flat shortens the cone");
  near(wave::swept_cutter_height(.25,0,{0,0,3},{0,0,3},cam::milling_cutter(flat)),3,1e-8,"Tip flat is represented in stock simulation");
  // Helical finishing intentionally accepts a large uncut core.
  op=hole();op.holes.systems[0].center_x=12;op.holes.systems[0].center_y=-8;
  op.holes.systems[0].levels[0]={};op.holes.systems[0].levels[0].radius=6;
  op.holes.systems[0].levels[0].top_z=10;op.holes.systems[0].levels[0].bottom_z=7;
  auto helix=plan(g,job(op));
  check(std::any_of(helix.operations[0].levels[0].feeds.motion.begin(),helix.operations[0].levels[0].feeds.motion.end(),
      [](const auto& m) {return m.arc && m.to_z<m.from_z;}),"Helical finishing retains native helical moves without core constraints");
}
void retraced_footprints() {
  using wave::MoveKind;
  const auto point=[](double x,double y) {
    return wave::Point64{wave::to_integer(x),wave::to_integer(y)};
  };
  const auto footprint=[&](const wave::Paths64& paths) {
    cam::LevelPlan level;
    for(const auto& path:paths) {
      wave::ToolpathMove move;move.kind=MoveKind::Cut;move.path=path;
      level.toolpath.moves.push_back(std::move(move));
    }
    return cam::detail::swept_footprint(level,.5);
  };
  const auto a=point(0,0),b=point(4,0),c=point(4,4),d=point(0,4),e=point(-4,0);
  const auto expected=footprint({{a,b,c,d,a},{a,e}});
  // Reverse traversal, repeated loops, and a branch after retracing must all
  // cover the same material without inventing a connection between runs.
  const auto actual=footprint({{a,b,c,d,a,d,c,b,a,e,a},{e,a,b,c,d,a}});
  near(wave::material_area(Clipper2Lib::Xor(expected,actual,Clipper2Lib::FillRule::NonZero)),
       0,.02,"Retraced paths preserve the swept footprint");
  const wave::Paths64 core{{point(1,1),point(3,1),point(3,3),point(1,3)}};
  near(wave::material_area(Clipper2Lib::Intersect(actual,core,Clipper2Lib::FillRule::NonZero)),
       0,1e-8,"Closed retraced paths preserve the uncut central core");
  near(wave::material_area(footprint({{a,a,a}})),
       wave::material_area(footprint({{a}})),1e-8,"Stationary cutter retains its footprint");

  // The failing job's first hole: a 3.175 mm cutter in a 4 mm bore, descending
  // 12 mm at 3 degrees. Its 101 descending turns share one XY projection.
  auto op=hole();op.stepdown=1;op.holes.helix_angle_degrees=3;
  op.holes.radial_margin=.05;
  auto& system=op.holes.systems.front();system.center_x=-70;system.center_y=-87.99991535257209;
  auto& level=system.levels.front();level.radius=2.0000071910686055;level.top_z=6;level.bottom_z=-6;
  auto mill=endmill();mill.diameter=3.175;
  const auto rates=cam::resolve_cutting_feeds(mill,machine(),"material",{});
  const auto deep=cam::plan_hole_operation(op,mill,machine(),rates,16,{}, {},0,1);
  check(deep.ok(),deep.error);
  const auto& planned=deep.levels.front();
  check(std::count_if(planned.feeds.motion.begin(),planned.feeds.motion.end(),
        [](const auto& phase) {return phase.arc;})==102,"Keep every helical turn in the XYZ motion");
  near(planned.floor_z,-6,1e-8,"Deep helix reaches the requested floor");
  level.bottom_z=5.99;
  const auto shallow=cam::plan_hole_operation(op,mill,machine(),rates,16,{}, {},0,1);
  check(shallow.ok(),shallow.error);
  near(wave::material_area(Clipper2Lib::Xor(planned.swept,shallow.levels.front().swept,
       Clipper2Lib::FillRule::NonZero)),0,1e-8,"Helix footprint is independent of repeated turns");
  near(wave::material_area(planned.swept),std::numbers::pi*1.95*1.95,.08,
       "Small helix sweeps the full bore inside the radial margin");
}
void independent_drill_diameter() {
  auto op=hole();op.tool_id="drill";op.holes.strategy=cam::HoleStrategy::Drill;
  auto& system=op.holes.systems.front();system.center_x=12;system.center_y=-8;
  auto& level=system.levels.front();level.top_z=10;level.bottom_z=3;
  const auto rates=cam::resolve_cutting_feeds(drill(),machine(),"material",{});
  for(double radius:{1.0,3.0}) {
    level.radius=radius; // The selected drill has a radius of 2 mm.
    auto j=job(op);
    const auto planned=plan({},j);
    const auto posted=cam::post_job(planned,j,{{drill()}},{{machine()}});
    check(posted.ok(),posted.error);
    check(!posted.text.empty(),"Different drill and bore diameters export G-code");
    op.holes.source_id="source";
    const auto pilot=cam::plan_hole_operation(op,drill(),machine(),rates,20,{}, {},0,1);
    check(pilot.ok(),pilot.error);
    for(const auto& phase:pilot.levels.front().feeds.motion) {
      near(wave::to_world(phase.to.x),12,1e-8,"Pilot drill stays on the hole centre X");
      near(wave::to_world(phase.to.y),-8,1e-8,"Pilot drill stays on the hole centre Y");
    }
    op.holes.source_id.clear();
  }
}
void partial_peck_retracts() {
  auto op=hole();op.tool_id="drill";op.holes.strategy=cam::HoleStrategy::Drill;
  auto& level=op.holes.systems.front().levels.front();
  level.radius=2;level.top_z=10;level.bottom_z=3;
  auto& drilling=op.holes.drilling;drilling.peck_depth=2;
  const auto rates=cam::resolve_cutting_feeds(drill(),machine(),"material",{});
  const auto plan_drill=[&] {
    return cam::plan_hole_operation(op,drill(),machine(),rates,20,{}, {},0,1);
  };
  for(double distance:{0.0,.05,.5,5.0}) {
    drilling.partial_peck_retract=distance>0;drilling.peck_retract_distance=distance;
    const auto planned=plan_drill();check(planned.ok(),planned.error);
    const auto& motion=planned.levels.front().feeds.motion;
    int pecks=0;double drilled_to=10;
    for(std::size_t i=0;i<motion.size();++i) {
      const auto& phase=motion[i];
      if(phase.rapid && phase.to_z<phase.from_z)
        check(phase.to_z>=drilled_to,"Peck return cannot rapid into uncut material");
      if(phase.rapid || phase.to_z>=phase.from_z) continue;
      drilled_to=phase.to_z;++pecks;
      near(phase.to_z,std::max(3.0,10-2.0*pecks),1e-8,"Pecks retain their programmed depths");
      check(i+1<motion.size() && motion[i+1].rapid && motion[i+1].to_z>phase.to_z,
            "Every peck retracts upward");
      const double target=pecks==4?20:distance>0?std::min(10.0,phase.to_z+distance):11;
      near(motion[i+1].to_z,target,1e-8,"Partial/full peck and final clearance heights");
    }
    check(pecks==4,"Partial retract preserves every peck, including the final short peck");
    const auto j=job(op);
    const auto saved=cam::job_from_json_text(cam::to_json_text(j));
    check(cam::to_json_text(saved)==cam::to_json_text(j),"Peck retract settings round-trip");
    const auto simulated=plan({},j);
    const auto posted=cam::post_job(simulated,j,{{drill()}},{{machine()}});
    check(posted.ok() && !posted.text.empty(),"Peck retracts simulate and export G-code");
  }
  drilling.partial_peck_retract=true;
  for(double invalid:{0.0,-.5,std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::quiet_NaN()}) {
    drilling.peck_retract_distance=invalid;
    check(!plan_drill().ok(),"Reject invalid active partial retract distances");
  }
  drilling.peck_depth=0;drilling.peck_retract_distance=.5;
  const auto single=plan_drill();check(single.ok(),single.error);
  check(std::count_if(single.levels.front().feeds.motion.begin(),single.levels.front().feeds.motion.end(),
        [](const auto& phase) {return !phase.rapid && phase.to_z<phase.from_z;})==1,
        "Zero peck depth still drills in one feed move");
  const auto legacy=cam::hole_parameters_from_json(nlohmann::json{
      {"drilling",{{"peck_depth",2},{"peck_clearance",1}}}});
  check(!legacy.drilling.partial_peck_retract,"Existing jobs retain full retract peck drilling");
}
void helical_radial_margin() {
  auto op=hole();op.holes.strategy=cam::HoleStrategy::Helical;
  auto& system=op.holes.systems.front();system.center_x=0;system.center_y=0;
  auto& level=system.levels.front();level.radius=2;level.top_z=2;level.bottom_z=0;
  for(double margin:{0.0,.25}) {
    op.holes.radial_margin=margin;
    auto j=job(op);j.simulation_pitch=.025;
    const auto planned=plan({},j);
    const auto& operation=planned.operations.front();
    int arcs=0;
    for(const auto& phase:operation.levels.front().feeds.motion) if(phase.arc) {
      ++arcs;
      for(double t:{0.0,.25,.5,.75,1.0}) {
        const auto p=cam::motion_point(phase,t);
        near(std::hypot(p.x,p.y)+endmill().diameter/2,level.radius-margin,.002,
             "Helical cutter envelope includes the radial margin on every turn");
      }
    }
    check(arcs>1,"Check descending helix and finishing circle");
    check(bool(operation.stock_after),"Helix has simulated stock");
    const auto& stock=*operation.stock_after;
    const auto height=[&](double x) {
      return stock.top(int(std::floor((x-stock.lower_x())/stock.pitch())),
                       int(std::floor((0-stock.lower_y())/stock.pitch())));
    };
    near(height(1.9),margin>0?2:0,1e-6,"Simulation preserves the radial allowance at the wall");
    near(height(1.5),0,1e-6,"Simulation clears inside the margin-adjusted wall");
  }
}
void strategies_links_and_storage(Fixture& files) {
  cam::JobGeometry g;g.load_part(files.model());g.set_box_stock(true,{});
  auto op=hole();op.holes.automatic=true;op.holes.id="source";
  auto identified=cam::identify_holes(g,op.holes);cam::apply_hole_detection(op.holes,identified);
  op.holes.strategy=cam::HoleStrategy::Adaptive;op.holes.adaptive_radius=3;
  check(cam::effective_hole_strategy(op.holes,op.holes.systems[0].levels[0])==cam::HoleStrategy::Pocket &&
      cam::effective_hole_strategy(op.holes,op.holes.systems[0].levels[2])==cam::HoleStrategy::Helical,
      "Adaptive strategy applies to individual counterbore radii");
  auto j=job(op);auto mixed=plan(g,j);
  check(mixed.operations[0].levels.size()>3,"Adaptive pockets use layer-wise clearing");
  op.holes.systems[0].levels[2].strategy=cam::HoleStrategy::Drill;
  op.holes.systems[0].levels[2].tool_id="drill";j=job(op);mixed=plan(g,j);
  check(mixed.operations[0].levels.back().tool_id=="drill","Counterbore levels may change tools");
  const auto posted=cam::post_job(mixed,j,{{endmill(),drill()}},{{machine()}});
  check(posted.error.empty(),posted.error);
  check(posted.text.find("T1 M6")!=std::string::npos && posted.text.find("T2 M6")!=std::string::npos &&
      posted.text.find("S5000")!=std::string::npos && posted.text.find("G3")!=std::string::npos,
      "Post emits tool changes, drilling spindle speed and native milling arcs");
  auto batched_op=op;batched_op.holes.automatic=false;
  auto second=batched_op.holes.systems[0];second.center_x=-12;
  batched_op.holes.systems.push_back(second);
  const auto batched=plan({},job(batched_op));
  std::string last;int changes=0;
  for(const auto& level:batched.operations[0].levels) if(!level.feeds.motion.empty()) {
    if(!last.empty() && level.tool_id!=last) ++changes;
    last=level.tool_id;
  }
  check(changes==1,"Ready levels batch by tool across holes while keeping their depth dependencies");
  cam::Operation pilot=hole();pilot.name="Pilot";pilot.tool_id="pilot";
  pilot.holes.source_id="source";pilot.holes.strategy=cam::HoleStrategy::Drill;
  pilot.holes.drilling.peck_depth=2;pilot.holes.drilling.partial_peck_retract=true;
  pilot.holes.drilling.peck_retract_distance=.2;
  op.holes.systems[0].levels[2].strategy.reset();op.holes.systems[0].levels[2].tool_id.clear();
  j=job(op);j.operations.insert(j.operations.begin(),pilot);
  auto inherited=cam::resolve_hole_source(j,0);
  check(inherited.holes.drilling.partial_peck_retract && inherited.holes.drilling.peck_retract_distance==.2,
        "Linked pilots retain their independent partial retract settings");
  check(inherited.holes.systems.size()==1 && inherited.holes.systems[0].levels.size()==1,
        "Linked pre-drilling inherits selected milling systems once");
  near(inherited.holes.systems[0].levels[0].top_z,10,.001,"Pilot drilling starts above the whole system");
  auto predrilled=plan(g,j);check(predrilled.operations[0].tool_id=="pilot","Pre-drilling is a normal editable operation");
  auto saved=cam::job_from_json_text(cam::to_json_text(j));
  check(cam::to_json_text(saved)==cam::to_json_text(j),"Hole systems, tool overrides and pilot links round-trip");
  auto original=cam::to_json_text(j);saved.operations[1].holes.circle_tolerance*=2;
  auto stale=cam::plan_job(g,saved,{{endmill(),drill(),drill(1,"pilot")}},{{machine()}},{{{"material","Material"}}},{});
  check(!stale.operations[0].ok() && !stale.operations[1].ok(),"Changing discovery settings invalidates parent and linked pilot plans");
  const auto tool_file=files.directory/"tools.json";cam::save_tools(tool_file,{{drill()}});
  const auto tools=cam::load_tools(tool_file);near(tools.tools[0].point_angle_degrees,90,1e-8,"Drill angle round-trips");
}
}
int main() {
  try {Fixture files;helical_radial_margin();partial_peck_retracts();independent_drill_diameter();retraced_footprints();detection(files);drilling_and_motion(files);strategies_links_and_storage(files);
    std::cout<<"Hole discovery, nested levels, drilling, strategies, tool changes and storage passed\n";return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
