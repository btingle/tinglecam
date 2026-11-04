#include "cam/holes.h"
#include <clipper2/clipper.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>
namespace cam {
const char* hole_strategy_name(HoleStrategy s) {
  switch(s) {
    case HoleStrategy::Drill:return "drill";
    case HoleStrategy::Helical:return "helical";
    case HoleStrategy::Pocket:return "pocket";
    case HoleStrategy::Adaptive:return "adaptive";
  }
  return "unknown";
}
std::optional<HoleStrategy> hole_strategy_from_name(const std::string& s) {
  for(auto v:{HoleStrategy::Drill,HoleStrategy::Helical,HoleStrategy::Pocket,HoleStrategy::Adaptive})
    if(s==hole_strategy_name(v)) return v;
  return {};
}
const char* drill_depth_name(DrillDepth d) {return d==DrillDepth::Tip?"tip":"full_diameter";}
std::optional<DrillDepth> drill_depth_from_name(const std::string& s) {
  if(s=="tip") return DrillDepth::Tip;
  if(s=="full_diameter") return DrillDepth::FullDiameter;
  return {};
}
HoleStrategy effective_hole_strategy(const HoleParameters& p,const HoleLevel& l) {
  const auto s=l.strategy.value_or(p.strategy);
  return s==HoleStrategy::Adaptive?(l.radius>p.adaptive_radius?HoleStrategy::Pocket:HoleStrategy::Helical):s;
}
namespace {
void require(bool b,const char* message) {if(!b) throw std::runtime_error(message);}
struct Circle { double x=0,y=0,r=0,error=0; };
std::optional<Circle> fit_circle(const wave::Path64& path,double tolerance) {
  if(path.size()<3) return {};
  // Translate first: normal equations must not lose precision for large work
  // coordinates. Segment-length weights avoid bias from uneven tessellation.
  const double ox=wave::to_world(path.front().x),oy=wave::to_world(path.front().y);
  double a[3][4]{};
  for(std::size_t i=0;i<path.size();++i) {
    const auto& p=path[i];const auto& q=path[(i+1)%path.size()];
    const auto& prev=path[(i+path.size()-1)%path.size()];
    const double x=wave::to_world(p.x)-ox,y=wave::to_world(p.y)-oy;
    const double weight=std::hypot(double(p.x-q.x),double(p.y-q.y))+
                        std::hypot(double(p.x-prev.x),double(p.y-prev.y));
    const double row[]{2*x,2*y,1};
    for(int j=0;j<3;++j) {
      for(int k=0;k<3;++k) a[j][k]+=weight*row[j]*row[k];
      a[j][3]+=weight*row[j]*(x*x+y*y);
    }
  }
  for(int col=0;col<3;++col) {
    int pivot=col;for(int row=col+1;row<3;++row) if(std::abs(a[row][col])>std::abs(a[pivot][col])) pivot=row;
    if(std::abs(a[pivot][col])<1e-12) return {};
    for(int k=col;k<4;++k) std::swap(a[pivot][k],a[col][k]);
    const double d=a[col][col];for(int k=col;k<4;++k) a[col][k]/=d;
    for(int row=0;row<3;++row) if(row!=col) {
      const double f=a[row][col];for(int k=col;k<4;++k) a[row][k]-=f*a[col][k];
    }
  }
  Circle c;c.x=ox+a[0][3];c.y=oy+a[1][3];
  const double rr=a[2][3]+a[0][3]*a[0][3]+a[1][3]*a[1][3];
  if(!(rr>0)) return {};
  c.r=std::sqrt(rr);
  for(std::size_t i=0;i<path.size();++i) {
    const double x=wave::to_world(path[i].x)-c.x,y=wave::to_world(path[i].y)-c.y;
    const auto& p=path[(i+1)%path.size()];
    const double dx=wave::to_world(p.x-path[i].x),dy=wave::to_world(p.y-path[i].y);
    const double t=dx*dx+dy*dy>0?std::clamp(-(x*dx+y*dy)/(dx*dx+dy*dy),0.,1.):0.;
    c.error=std::max({c.error,std::abs(std::hypot(x,y)-c.r),
                     std::abs(std::hypot(x+t*dx,y+t*dy)-c.r)});
  }
  if(c.error>tolerance || c.r<=tolerance) return {};
  return c;
}
std::vector<Circle> circles_at(const JobGeometry& g,double z,const HoleParameters& p,
                                 const std::optional<Circle>& local={}) {
  Clipper2Lib::Clipper64 clipper;
  // The downward material envelope excludes hidden cavities and preserves
  // every shoulder of a top-accessible counterbore system.
  auto slice=g.part_outline(z,32);
  if(local) {
    const auto r=wave::to_integer(local->r+p.circle_tolerance*2);
    const auto x=wave::to_integer(local->x),y=wave::to_integer(local->y);
    slice=Clipper2Lib::Intersect(slice,{{{x-r,y-r},{x+r,y-r},{x+r,y+r},{x-r,y+r}}},
                                Clipper2Lib::FillRule::NonZero);
  }
  clipper.AddSubject(slice);Clipper2Lib::PolyTree64 tree;wave::Paths64 open;
  if(!clipper.Execute(Clipper2Lib::ClipType::Union,Clipper2Lib::FillRule::NonZero,tree,open)) return {};
  std::vector<Circle> found;
  for(std::size_t i=0;i<tree.Count();++i) {
    const auto* outer=tree.Child(i);
    for(std::size_t j=0;j<outer->Count();++j) {
      const auto* hole=outer->Child(j);
      if(hole->Count()!=0) continue;
      if(auto c=fit_circle(hole->Polygon(),p.circle_tolerance)) found.push_back(*c);
    }
  }
  return found;
}
}
std::string hole_detection_signature(const JobGeometry& g,const HoleParameters& p) {
  if(!g.has_part()) return {};
  const auto& mesh=g.part().mesh();
  std::ostringstream key;key<<std::setprecision(17)<<"holes-v1:";
  if(g.part_cad()) key<<g.part_cad()->fingerprint();
  else {
    key<<mesh.source.lexically_normal().string()<<':'<<mesh.triangles.size();
    std::error_code ec;
    const auto size=std::filesystem::file_size(mesh.source,ec);if(!ec) key<<':'<<size;
    const auto time=std::filesystem::last_write_time(mesh.source,ec);if(!ec) key<<':'<<time.time_since_epoch().count();
    key<<':'<<mesh.bounds.lower.x<<':'<<mesh.bounds.lower.y<<':'<<mesh.bounds.lower.z
       <<':'<<mesh.bounds.upper.x<<':'<<mesh.bounds.upper.y<<':'<<mesh.bounds.upper.z;
  }
  key<<':'<<mesh.bounds.lower.x<<':'<<mesh.bounds.lower.y<<':'<<mesh.bounds.lower.z
     <<':'<<mesh.bounds.upper.x<<':'<<mesh.bounds.upper.y<<':'<<mesh.bounds.upper.z;
  if (g.part_orientation() != ModelOrientation{})
    for (int axis : g.part_orientation().axes) key << ':' << axis;
  key<<':'<<p.inspection_depth<<':'<<p.circle_tolerance<<':'<<p.counterbores
     <<':'<<p.probe_inset<<':'<<p.slice_epsilon;
  return key.str();
}
HoleDetection identify_holes(const JobGeometry& g,const HoleParameters& p,const ProgressCallback& progress) {
  HoleDetection result;
  try {
    require(g.has_part(),"Load a part before identifying holes");
    require(std::isfinite(p.circle_tolerance) && p.circle_tolerance>0,"Circle tolerance must be positive");
    require(std::isfinite(p.probe_inset) && p.probe_inset>0,"Probe inset must be positive");
    require(std::isfinite(p.slice_epsilon) && p.slice_epsilon>=.001,"Slice offset must be at least 0.001 mm");
    const auto& b=g.part().mesh().bounds;
    require(std::isfinite(p.inspection_depth) && p.inspection_depth>=0 &&
        p.inspection_depth<double(b.upper.z-b.lower.z),"Inspection depth must be inside the part's Z range");
    result.signature=hole_detection_signature(g,p);
    const auto report=[&](double f,const std::string& stage) {
      if(progress && !progress(f,stage)) throw std::runtime_error("Cancelled");
    };
    report(0,"Slicing for circular openings");
    const double inspect=b.upper.z-p.inspection_depth;
    const auto roots=circles_at(g,inspect-p.slice_epsilon,p);
    for(std::size_t i=0;i<roots.size();++i) {
      HoleSystem system;system.levels.clear();system.center_x=roots[i].x;system.center_y=roots[i].y;
      Circle circle=roots[i];double top=inspect,z=inspect-p.slice_epsilon;
      while(z>b.lower.z) {
        report(double(i)/std::max(std::size_t{1},roots.size()),"Probing hole "+std::to_string(i+1));
        const double inset=std::min(circle.r*.25,std::max(p.probe_inset,circle.error*2+.002));
        std::vector<wave::Vec3f> origins;
        for(int a=0;a<12;++a) {
          const double angle=2*std::numbers::pi*a/12;
          origins.push_back({float(circle.x+(circle.r-inset)*std::cos(angle)),
              float(circle.y+(circle.r-inset)*std::sin(angle)),float(z)});
        }
        double ms=0;const auto hits=g.part().vertical_raycast(origins,b.lower.z-p.slice_epsilon,ms);
        double floor=-std::numeric_limits<double>::infinity(),lowest=std::numeric_limits<double>::infinity();
        int finite=0;
        for(float h:hits) if(std::isfinite(h)) {floor=std::max(floor,double(h));lowest=std::min(lowest,double(h));++finite;}
        HoleLevel level;level.radius=circle.r;level.top_z=top;level.fit_error=circle.error;
        level.through=finite==0;level.bottom_z=level.through?double(b.lower.z):floor;
        if(!(level.bottom_z<z-p.slice_epsilon*.1)) {
          result.warnings.push_back("Hole "+std::to_string(i+1)+": sloping or chamfered wall; inspect its detected extent");break;
        }
        if(finite && (finite!=12 || floor-lowest>std::max(p.circle_tolerance,2*p.slice_epsilon))) {
          result.warnings.push_back("Hole "+std::to_string(i+1)+": floor probes disagree; feature omitted");break;
        }
        if(finite && floor+2*p.slice_epsilon<z) {
          bool cylindrical=false;
          for(const auto& c:circles_at(g,(top+floor)*.5,p,circle))
            if(std::hypot(c.x-circle.x,c.y-circle.y)<=p.circle_tolerance &&
                std::abs(c.r-circle.r)<=std::max(.003,p.circle_tolerance*.25)) cylindrical=true;
          if(!cylindrical) {
            result.warnings.push_back("Hole "+std::to_string(i+1)+": tapered/chamfered level omitted");break;
          }
        }
        require(system.levels.size()<20000,"Too many nested hole levels");
        system.levels.push_back(level);
        if(level.through || !p.counterbores) break;
        const auto children=circles_at(g,floor-p.slice_epsilon,p,circle);
        std::optional<Circle> next;
        for(const auto& child:children)
          if(std::hypot(child.x-system.center_x,child.y-system.center_y)<=p.circle_tolerance &&
              child.r<circle.r-std::max(.002,2*p.circle_tolerance)) {
            if(next) {result.warnings.push_back("Ambiguous nested openings; counterbore search stopped");next.reset();break;}
            next=child;
          }
        if(!next) break;
        // Distinct cylindrical levels must persist below the shoulder; this
        // rejects a chamfer/taper instead of inventing a staircase of bores.
        const double verify_z=std::max(double(b.lower.z)+p.slice_epsilon,floor-3*p.slice_epsilon);
        bool persistent=false;
        for(const auto& c:circles_at(g,verify_z,p,circle))
          if(std::hypot(c.x-next->x,c.y-next->y)<=p.circle_tolerance &&
              std::abs(c.r-next->r)<=std::max(.002,p.circle_tolerance)) persistent=true;
        if(!persistent) {result.warnings.push_back("Taper/chamfer below a shoulder; recursion stopped");break;}
        circle=*next;top=floor;z=floor-p.slice_epsilon;
      }
      if(!system.levels.empty()) result.systems.push_back(std::move(system));
    }
    std::sort(result.systems.begin(),result.systems.end(),[](const auto& a,const auto& b) {
      return a.center_x!=b.center_x?a.center_x<b.center_x:a.center_y<b.center_y;
    });
    report(1,"Hole identification complete");
  } catch(const std::exception& e) {result.error=e.what();result.systems.clear();}
  return result;
}
void apply_hole_detection(HoleParameters& p,const HoleDetection& detection) {
  if(!detection.error.empty()) throw std::runtime_error(detection.error);
  auto systems=detection.systems;
  if(!p.detection_signature.empty()) for(auto& system:systems) {
    system.selected=false;
    for(const auto& old:p.systems) {
      if(std::hypot(old.center_x-system.center_x,old.center_y-system.center_y)>p.circle_tolerance) continue;
      system.selected=old.selected;
      for(auto& level:system.levels) {
        level.selected=false;
        for(const auto& previous:old.levels)
          if(std::abs(previous.radius-level.radius)<=p.circle_tolerance &&
              std::abs(previous.top_z-level.top_z)<=p.circle_tolerance &&
              std::abs(previous.bottom_z-level.bottom_z)<=p.circle_tolerance) {
            level.selected=previous.selected;level.strategy=previous.strategy;level.tool_id=previous.tool_id;break;
          }
      }
      break;
    }
  }
  p.systems=std::move(systems);p.detection_signature=detection.signature;
}
Operation resolve_hole_source(const Job& job,std::size_t index) {
  Operation operation=job.operations.at(index);
  if(operation.mode!=OperationMode::Hole || operation.holes.source_id.empty()) return operation;
  const Operation* source=nullptr;
  for(const auto& candidate:job.operations)
    if(candidate.mode==OperationMode::Hole && candidate.holes.id==operation.holes.source_id) {
      require(!source,"Two hole operations share the same link identity");source=&candidate;
    }
  require(source && source!=&job.operations[index],"The linked hole operation is missing");
  require(source->holes.source_id.empty(),"A pilot operation cannot link to another pilot operation");
  const auto source_index=std::size_t(source-job.operations.data());
  require(index<source_index,"Place the pre-drilling operation before its milling operation");
  operation.holes.systems.clear();
  operation.holes.automatic=source->holes.automatic;
  operation.holes.detection_signature=source->holes.detection_signature;
  operation.holes.inspection_depth=source->holes.inspection_depth;
  operation.holes.circle_tolerance=source->holes.circle_tolerance;
  operation.holes.counterbores=source->holes.counterbores;
  operation.holes.probe_inset=source->holes.probe_inset;
  operation.holes.slice_epsilon=source->holes.slice_epsilon;
  operation.holes.strategy=HoleStrategy::Drill;
  for(const auto& h:source->holes.systems) if(h.selected) {
    HoleSystem pilot=h;pilot.levels.clear();
    for(const auto& l:h.levels) if(l.selected && effective_hole_strategy(source->holes,l)!=HoleStrategy::Drill) {
      HoleLevel bore=l;
      for(const auto& upper:h.levels) bore.top_z=std::max(bore.top_z,upper.top_z);
      bore.tool_id.clear();bore.strategy=HoleStrategy::Drill;
      if(pilot.levels.empty() || bore.bottom_z<pilot.levels.front().bottom_z) pilot.levels={bore};
    }
    if(!pilot.levels.empty()) operation.holes.systems.push_back(std::move(pilot));
  }
  return operation;
}
}
