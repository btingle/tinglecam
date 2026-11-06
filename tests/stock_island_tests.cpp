#include "planning/toolpath.h"
#include "simulation/heightmap.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdexcept>

namespace {
using namespace wave;
using namespace Clipper2Lib;
void check(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
Path64 box(double x0, double y0, double x1, double y1) {
  return {{to_integer(x0),to_integer(y0)}, {to_integer(x1),to_integer(y0)},
          {to_integer(x1),to_integer(y1)}, {to_integer(x0),to_integer(y1)}};
}
void contacts(const CommittedWavefront& committed) {
  for (const auto& generation : committed.lineage.generations)
    for (const auto& front : generation.fronts) {
      for (const auto& contact : front.boundary_contacts) if (contact.valid) {
        check(contact.component < committed.lineage.boundary_components.size(),
              "Combined boundary contacts must remain valid");
        const auto& ring = committed.lineage.boundary_components[contact.component];
        check(std::any_of(committed.stages.begin(),committed.stages.end(),[&](const auto& stage) {
          for (const auto& g : stage.lineage.generations)
            for (const auto& f : g.fronts) if (f.path == front.path)
              for (const auto& c : f.boundary_contacts)
                if (c.valid && c.point == contact.point &&
                    stage.lineage.boundary_components[c.component] == ring) return true;
          return false;
        }), "Combining islands must retain the actual endpoint boundary");
      }
    }
}
void overlapping_air() {
  const Paths64 stock{box(-25,-22,-3,22),box(3,-22,25,22)};
  const Paths64 parts{box(-18,-12,-10,12),box(10,-12,18,12)};
  // Treat this extra protected object as a fixture. It must be present even
  // though it lies in the approach domain, outside either stock island.
  const Paths64 protected_geometry = Union(parts,Paths64{box(-2,14,2,18)},FillRule::NonZero);
  const auto subject = InflatePaths(protected_geometry,3200,JoinType::Round,EndType::Polygon);
  const auto analyses = analyze_stock_wavefronts(subject,stock,1.2,1.2,3.2,
      2000,128,{},kDefaultSignificantNodeAreaFraction,kDefaultTinySplitAreaFactor,3);
  check(analyses.size()==2,"Physical stock islands remain separate despite overlapping air");
  check(material_area(Intersect(analyses[0].seed.domain,analyses[1].seed.domain,
        FillRule::NonZero))>0,"Regression must exercise overlapping expanded domains");
  const auto committed = commit_wavefront_analyses(analyses);
  check(!committed.stages.empty(),"Independent stock analyses must produce cuts");
  std::size_t count=0;
  for (const auto& stage : committed.stages) {
    check(paths_equal_canonical(stage.wave.stock,stock),"Routing must see both physical stock islands");
    check(paths_equal_canonical(stage.wave.subject,subject),"Every island must protect every part and fixture");
    for (const auto& generation : stage.lineage.generations)
      for (const auto& front : generation.fronts) {
        const auto b=GetBounds(front.path);
        check(!(b.left < -3000 && b.right > 3000),
              "A propagated front must not bridge distinct stock islands");
        ++count;
      }
  }
  check(count>10,"Regression must exercise multiple real propagation fronts");
  contacts(committed);
  const auto path=build_toolpath(committed.stages,3);
  check(path.stats.cut_count>0 && path.stats.entry_count==0 && path.stats.unreachable_fronts==0,
        "Independent contours must remain reachable with the existing entry rules");
  auto stock_map=Heightmap::from_polygons(stock,0,3,.125);
  const MillingCutter cutter{CutterShape::Flat,3,20};
  for (const auto& move : path.moves) {
    if (move.kind==MoveKind::Retract || move.path.size()<2) continue;
    for (std::size_t i=1;i<move.path.size();++i) {
      const auto a=move.path[i-1],b=move.path[i];
      stock_map.cut({to_world(a.x),to_world(a.y),0},
                    {to_world(b.x),to_world(b.y),0},cutter);
    }
  }
  const auto after=stock_map.slice(1);
  // A terminating wave has no front on its final cap. Boundary finishing
  // handles the last radial stepover; this checks for holes beyond that band.
  const auto waste=Difference(stock,InflatePaths(parts,1600,JoinType::Round,EndType::Polygon),FillRule::NonZero);
  const double remaining=material_area(Intersect(after,waste,FillRule::NonZero));
  check(remaining<1,"Independent wave contours must leave no gaps beyond one stepover of the allowance");
  check(material_area(Difference(parts,after,FillRule::NonZero))<.01,
        "Independent contours must preserve both parts");
}
void holes_and_connected_stock() {
  auto hole=box(-12,-12,12,12);std::reverse(hole.begin(),hole.end());
  const Paths64 stock{box(-20,-20,20,20),hole,box(-4,-4,4,4)};
  const Paths64 subject{box(100,100,101,101)};
  const auto islands=analyze_stock_wavefronts(subject,stock,2,0,0,200,32);
  check(islands.size()==2,"An island nested inside a stock hole remains independent");
  Paths64 domains;
  for(const auto& island:islands)
    domains.insert(domains.end(),island.seed.domain.begin(),island.seed.domain.end());
  check(paths_equal_canonical(Union(domains,FillRule::NonZero),stock),
        "Separating islands must preserve holes and occupied stock exactly");
  const Paths64 plate{box(-20,-20,20,20)};
  const Paths64 parts{box(-12,-4,-8,4),box(8,-4,12,4)};
  const auto single=analyze_stock_wavefronts(parts,plate,2,2,3.2,200,32,{},
      kDefaultSignificantNodeAreaFraction,kDefaultTinySplitAreaFactor,3);
  const auto reference=commit_wavefront_analysis(analyze_wavefront(parts,plate,2,2,3.2,200,32,{},
      kDefaultSignificantNodeAreaFraction,kDefaultTinySplitAreaFactor,3));
  const auto result=commit_wavefront_analyses(single);
  check(single.size()==1 && result.wave.generations==reference.wave.generations,
        "Multiple parts on connected stock must retain existing propagation");
}

void simplified_ribbons(const std::filesystem::path& fixture) {
  nlohmann::json data;
  std::ifstream(fixture) >> data;
  const auto paths = [](const auto& input) {
    Paths64 out;
    for (const auto& ring : input) {
      Path64 path;
      for (const auto& point : ring)
        path.emplace_back(point[0].template get<std::int64_t>(),
                          point[1].template get<std::int64_t>());
      out.push_back(std::move(path));
    }
    return out;
  };
  const auto subject=paths(data.at("subject"));
  const auto stock=paths(data.at("stock"));
  const auto limit=paths(data.at("limit"));
  const double step=data.at("stepover"),radius=data.at("radius"),lead=data.at("lead_in");
  const auto plan = [&](double tolerance) {
    return commit_wavefront_analyses(analyze_stock_wavefronts(subject,stock,
        step,step,lead,2000,128,{},kDefaultSignificantNodeAreaFraction,
        kDefaultTinySplitAreaFactor,radius,limit,tolerance));
  };
  const auto exact=plan(0);
  const auto reference=build_toolpath(exact.stages,radius);
  check(reference.stats.skipped_entry_count==0,"Exact reference must reach the inner region");
  const auto simplified=plan(.01);
  for(const auto& stage:simplified.stages)
    for(std::size_t g=1;g<stage.lineage.generations.size();++g)
      for(const auto& front:stage.lineage.generations[g].fronts)
        check(!front.parents.empty(),
              "Simplifying a moving ribbon must not create detached propagation sources");
  const auto routed=build_toolpath(simplified.stages,radius);
  check(routed.stats.skipped_entry_count==0 && routed.stats.unreachable_fronts==0,
        "Simplified fronts must all remain accessible with normal entry rules");
  check(routed.stats.cut_count==reference.stats.cut_count,
        "Simplification must preserve complete contours rather than add fragment passes");
  const auto missing=Intersect(Difference(reference.cleared,routed.cleared,FillRule::NonZero),
                               stock,FillRule::NonZero);
  check(material_area(missing)<.1,
        "Simplification must preserve access to the entire inner region within rounding error");
  contacts(simplified);
}
}
int main(int argc,char** argv) {
  try {
    overlapping_air();holes_and_connected_stock();
    if(argc>1)simplified_ribbons(argv[1]);
    std::cout<<"Independent stock contours, coverage, routing, holes and boundary provenance passed.\n";
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
