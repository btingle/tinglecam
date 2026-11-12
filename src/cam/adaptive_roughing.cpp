#include "cam/adaptive_roughing.h"
#include "cam/job_internal.h"
#include "bridge.h"
#include <numbers>

namespace cam::detail {
namespace {
freecad_adaptive::Paths convert(const wave::Paths64& paths) {
  freecad_adaptive::Paths result;
  for(const auto& path:paths) {
    freecad_adaptive::Path out;
    for(auto p:path) out.emplace_back(wave::to_world(p.x),wave::to_world(p.y));
    if(out.size()>2) result.push_back(std::move(out));
  }
  return result;
}
wave::Point64 point(freecad_adaptive::Point p) {
  return {wave::to_integer(p.first),wave::to_integer(p.second)};
}
wave::Paths64 inflate(const wave::Paths64& paths,double amount) {
  return Clipper2Lib::InflatePaths(paths,wave::to_integer(amount),
      Clipper2Lib::JoinType::Round,Clipper2Lib::EndType::Polygon);
}
}
void generate_adaptive_roughing(LevelPlan& level,const Operation& operation,
    double radius,double stepover,const LevelProgress& report,double simulation_pitch) {
  using namespace Clipper2Lib;
  if(report && !report(0)) throw std::runtime_error("Cancelled");
  level.adaptive_roughing=true;
  // The solver takes physical boundaries and offsets them by the cutter radius.
  // Supply air explicitly so it cannot expand through fixtures or area limits.
  auto domain=inflate(level.stock_slice,3*radius+stepover);
  if(!level.machining_limit.empty()) {
    const auto physical=InflatePaths(level.machining_limit,wave::to_integer(radius),
        JoinType::Miter,EndType::Polygon);
    domain=Intersect(domain,physical,FillRule::NonZero);
  }
  auto protected_region=inflate(level.part_slice,operation.radial_stock_to_leave+.01);
  protected_region=Union(protected_region,inflate(level.fixture_keep_out,-radius+.02),FillRule::NonZero);
  auto boundary=Difference(domain,protected_region,FillRule::NonZero);
  auto cleared=Difference(domain,level.incoming_stock,FillRule::NonZero);
  bool cancelled=false;
  auto regions=freecad_adaptive::generate(convert(level.incoming_stock),convert(boundary),
      convert(cleared),2*radius,stepover,operation.rough_boundary_cleanup,[&] {
        cancelled=report && !report(.5);return cancelled;
      });
  if(cancelled || (report && !report(.8))) throw std::runtime_error("Cancelled");
  auto& path=level.toolpath;
  const auto append=[&](wave::MoveKind kind,wave::Path64 points) {
    if(points.empty()) return;
    double length=0;
    for(std::size_t i=1;i<points.size();++i) length+=distance_mm(points[i-1],points[i]);
    path.moves.push_back({kind,std::move(points),{},length});
  };
  for(const auto& region:regions) {
    if(region.failed) {++path.stats.skipped_entry_count;continue;}
    if(region.incomplete) level.adaptive_warnings.push_back("FreeCAD adaptive left an incomplete region");
    if(region.paths.empty()) continue;
    const auto start=point(region.start),center=point(region.center);
    const auto from=path.moves.empty()?start:path.moves.back().path.back();
    append(wave::MoveKind::Retract,{from,start});
    const double helix_radius=distance_mm(start,center);
    if(helix_radius>.01) {
      level.adaptive_helices.push_back({start,center});
      wave::Path64 circle{start};
      const double angle=std::atan2(double(start.y-center.y),double(start.x-center.x));
      const int samples=std::max(48,int(std::ceil(2*std::numbers::pi*helix_radius/.1)));
      for(int i=1;i<samples;++i) {
        const double a=angle+2*std::numbers::pi*i/samples;
        circle.emplace_back(center.x+wave::to_integer(helix_radius*std::cos(a)),
                            center.y+wave::to_integer(helix_radius*std::sin(a)));
      }
      circle.push_back(start);
      append(wave::MoveKind::Entry,std::move(circle));
    }
    for(const auto& [motion,vertices]:region.paths) {
      wave::Path64 points;
      for(auto p:vertices) if(points.empty() || points.back()!=point(p)) points.push_back(point(p));
      if(points.empty()) continue;
      auto previous=path.moves.back().path.back();
      if(previous!=points.front()) append(wave::MoveKind::Retract,{previous,points.front()});
      append(motion==0?wave::MoveKind::Cut:motion==1?wave::MoveKind::Link:
             wave::MoveKind::Retract,std::move(points));
    }
  }
  // FreeCAD's landing proof uses an exact cutter. The heightmap also reserves
  // a cell diagonal. Prefer a nearby landing with that headroom, then follow
  // an already cleared route to the solver's original start point.
  auto available=cleared;
  LevelPlan prefix;
  std::vector<wave::ToolpathMove> routed;
  bool feed_entry=false;
  const auto protected_centres=Union(inflate(level.part_slice,radius+operation.radial_stock_to_leave),
                                     level.fixture_keep_out,FillRule::NonZero);
  // FreeCAD smooths its clear links. Fall back to the existing clearance
  // plane if that smoothing crosses a protected cutter-centre boundary.
  for(auto& move:path.moves) if(move.kind==wave::MoveKind::Link) {
    Clipper64 clip;clip.AddOpenSubject({move.path});clip.AddClip(inflate(protected_centres,-.002));
    wave::Paths64 closed,open;clip.Execute(ClipType::Intersection,FillRule::NonZero,closed,open);
    if(!open.empty()) move.kind=wave::MoveKind::Retract;
  }
  const auto inside=[](const wave::Paths64& rings,const wave::Point64& p) {
    int winding=0;
    for(const auto& ring:rings) {
      const auto where=PointInPolygon(p,ring);
      if(where==PointInPolygonResult::IsOn) return true;
      if(where==PointInPolygonResult::IsInside) winding+=Area(ring)>0?1:-1;
    }
    return winding!=0;
  };
  for(auto& source_move:path.moves) {
    auto move=std::move(source_move);
    if(move.kind==wave::MoveKind::Retract) {
      if(report && !report(.9)) throw std::runtime_error("Cancelled");
      if(!prefix.toolpath.moves.empty()) {
        available=Union(available,swept_footprint(prefix,radius),FillRule::NonZero);
        prefix.toolpath.moves.clear();
      }
      const auto target=move.path.back();
      const bool helix=std::any_of(level.adaptive_helices.begin(),level.adaptive_helices.end(),
          [&](const auto& entry) {return entry.first==target;});
      if(!helix) {
        auto safe=Difference(inflate(available,-radius-simulation_pitch*std::sqrt(.5)-.01),
                             protected_centres,FillRule::NonZero);
        auto travelable=Difference(inflate(available,-radius),protected_centres,FillRule::NonZero);
        if(!level.machining_limit.empty()) {
          safe=Intersect(safe,level.machining_limit,FillRule::NonZero);
          travelable=Intersect(travelable,level.machining_limit,FillRule::NonZero);
        }
        if(!inside(safe,target)) {
          std::vector<std::pair<double,wave::Point64>> candidates;
          for(const auto& ring:safe) for(std::size_t i=0;i<ring.size();++i) {
            const auto a=ring[i],b=ring[(i+1)%ring.size()];
            const double dx=double(b.x-a.x),dy=double(b.y-a.y),square=dx*dx+dy*dy;
            const double t=square>0?std::clamp(((target.x-a.x)*dx+(target.y-a.y)*dy)/square,0.,1.):0.;
            const wave::Point64 p{a.x+std::llround(t*dx),a.y+std::llround(t*dy)};
            candidates.emplace_back(distance_mm(p,target),p);
          }
          std::sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b){return a.first<b.first;});
          const wave::Router router(travelable,2.0);
          std::optional<wave::Path64> route;
          for(const auto& candidate:candidates) {
            if((route=router.route(candidate.second,target))) break;
          }
          if(route) {
            const auto landing=route->front();
            const auto from=routed.empty()?landing:routed.back().path.back();
            routed.push_back({wave::MoveKind::Retract,{from,landing},{},distance_mm(from,landing)});
            double length=0;
            for(std::size_t i=1;i<route->size();++i) length+=distance_mm((*route)[i-1],(*route)[i]);
            routed.push_back({wave::MoveKind::Link,*route,{},length});
            continue;
          }
          // FreeCAD commands a feed descent after a clearance-height link.
          // Preserve that convention when a conservative rapid landing cannot
          // be proved, allowing the unchanged simulator to split the approach.
          feed_entry=true;
        }
      }
    } else {
      if(feed_entry && move.path.size()>1) {move.kind=wave::MoveKind::Entry;feed_entry=false;}
      prefix.toolpath.moves.push_back(move);
    }
    routed.push_back(std::move(move));
  }
  path.moves=std::move(routed);
  wave::Paths64 cutting;
  for(const auto& move:path.moves)
    if(move.kind!=wave::MoveKind::Retract && move.path.size()>1) cutting.push_back(move.path);
  const auto keep_out=Union(inflate(level.part_slice,radius+operation.radial_stock_to_leave),
                            level.fixture_keep_out,FillRule::NonZero);
  const auto reject_crossings=[&](ClipType type,const wave::Paths64& region,const char* label) {
    Clipper64 clip;clip.AddOpenSubject(cutting);clip.AddClip(region);
    wave::Paths64 closed,open;clip.Execute(type,FillRule::NonZero,closed,open);
    for(const auto& segment:open) for(std::size_t i=1;i<segment.size();++i)
      if(distance_mm(segment[i-1],segment[i])>.002) {
        throw std::runtime_error(std::string("FreeCAD adaptive path crosses ")+label+" at "+
            std::to_string(wave::to_world(segment[i].x))+","+
            std::to_string(wave::to_world(segment[i].y))+" by "+
            std::to_string(distance_mm(segment[i-1],segment[i]))+" mm");
      }
  };
  reject_crossings(ClipType::Intersection,inflate(keep_out,-.002),"a protected model or fixture");
  if(!level.machining_limit.empty())
    reject_crossings(ClipType::Difference,inflate(level.machining_limit,.002),"the machining limit");
  // Routing only sees space actually swept by accepted paths, plus existing air.
  const auto swept=swept_footprint(level,radius);
  path.cleared=inflate(Union(cleared,swept,FillRule::NonZero),-radius);
  path.cleared=Difference(path.cleared,Union(inflate(level.part_slice,
      radius+operation.radial_stock_to_leave),level.fixture_keep_out,FillRule::NonZero),FillRule::NonZero);
  if(!level.machining_limit.empty()) path.cleared=Intersect(path.cleared,level.machining_limit,FillRule::NonZero);
  if(report && !report(1)) throw std::runtime_error("Cancelled");
}

void add_adaptive_helices(LevelPlan& level,const CuttingFeeds& feeds) {
  if(level.adaptive_helices.empty()) return;
  std::vector<MotionPhase> motion;
  for(const auto& phase:level.feeds.motion) {
    const auto entry=std::find_if(level.adaptive_helices.begin(),level.adaptive_helices.end(),
        [&](const auto& helix) {return phase.from==helix.first;});
    if(phase.arc || phase.from!=phase.to || phase.to_z>=phase.from_z ||
       entry==level.adaptive_helices.end()) {motion.push_back(phase);continue;}
    const double top=std::max(level.floor_z+level.axial_depth,phase.to_z);
    auto approach=phase;approach.to_z=std::min(top,phase.from_z);
    approach.length=std::abs(approach.from_z-approach.to_z);
    if(approach.length>0) motion.push_back(approach);
    const double r=distance_mm(entry->first,entry->second);
    const double height=approach.to_z-level.floor_z;
    const double pitch=std::min(level.axial_depth,2*std::numbers::pi*r*std::tan(3*std::numbers::pi/180));
    const double count=std::ceil(height/std::max(pitch,.001));
    if(count>100000) throw std::runtime_error("Adaptive helix requires too many turns");
    const int turns=std::max(1,int(count));
    for(int i=0;i<turns;++i) {
      auto helix=phase;helix.arc=true;helix.clockwise=false;helix.arc_center=entry->second;
      helix.rapid=false;helix.kind=wave::MoveKind::Entry;helix.rate=feeds.plunge_feed;
      helix.arrival=false;
      helix.from_z=approach.to_z-height*i/turns;
      helix.to_z=approach.to_z-height*(i+1)/turns;
      helix.length=std::hypot(2*std::numbers::pi*r,helix.to_z-helix.from_z);
      motion.push_back(helix);
    }
  }
  level.feeds.motion=std::move(motion);
  level.feeds.authoritative_motion=true;
}
}
