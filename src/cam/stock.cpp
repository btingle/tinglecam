#include "cam/stock.h"
#include <algorithm>
#include <cmath>

namespace cam {
namespace {
std::vector<StockSlab> clip(const std::vector<StockSlab>& stock,
    const wave::Paths64& footprint, double bottom, double top, bool inside) {
  if (footprint.empty() || top <= bottom) return inside ? std::vector<StockSlab>{} : stock;
  std::vector<StockSlab> out;
  for (const auto& slab : stock) {
    const double lo = std::max(bottom, slab.bottom);
    const double hi = std::min(top, slab.top);
    if (hi <= lo) { if (!inside) out.push_back(slab); continue; }
    if (!inside && slab.bottom < lo) out.push_back({slab.bottom, lo, slab.region});
    auto region = inside ? Clipper2Lib::Intersect(*slab.region, footprint, Clipper2Lib::FillRule::NonZero)
                         : Clipper2Lib::Difference(*slab.region, footprint, Clipper2Lib::FillRule::NonZero);
    if (!region.empty()) {
      auto shared = region == *slab.region ? slab.region
          : std::make_shared<const wave::Paths64>(std::move(region));
      out.push_back({lo, hi, std::move(shared)});
    }
    if (!inside && hi < slab.top) out.push_back({hi, slab.top, slab.region});
  }
  return out;
}
}
std::vector<StockSlab> stock_inside(const std::vector<StockSlab>& s,
    const wave::Paths64& p, double b, double t) { return clip(s,p,b,t,true); }
std::vector<StockSlab> stock_outside(const std::vector<StockSlab>& s,
    const wave::Paths64& p, double b, double t) { return clip(s,p,b,t,false); }
std::vector<StockSlab> preview_before_level(const OperationPlan& operation, std::size_t active) {
  auto stock = operation.preview_context;
  for (std::size_t i=0;i<operation.levels.size();++i) {
    const auto& level = operation.levels[i];
    auto local = level.local_stock;
    // Earliest area owns overlaps initially; subsequent removals are applied
    // in execution order. Future areas must not reveal cuts prematurely.
    for (std::size_t j=0;j<i;++j) {
      const auto& earlier = operation.levels[j];
      local = stock_outside(local, earlier.stock_slice, earlier.floor_z,
                            earlier.floor_z+earlier.axial_depth);
    }
    stock.insert(stock.end(),local.begin(),local.end());
  }
  for (std::size_t i=0;i<std::min(active,operation.levels.size());++i) {
    const auto& level = operation.levels[i];
    stock = stock_outside(stock,level.simulation.removed_material,
                          level.floor_z,level.floor_z+level.axial_depth);
  }
  return stock;
}
std::vector<StockSlab> active_stock(const JobGeometry& geometry,
    const std::vector<OperationPlan>& previous, int grid,
    const std::vector<double>& extra_boundaries) {
  if (!geometry.has_stock()) return {};
  for (std::size_t i=previous.size(); i>0; --i) {
    const auto& op = previous[i-1];
    if (!op.ok() || !op.composed_stock) continue;
    auto stock = preview_before_level(op,op.levels.size());
    for (std::size_t j=i;j<previous.size();++j) if (previous[j].ok())
      for (const auto& level : previous[j].levels)
        stock = stock_outside(stock,level.simulation.removed_material,
                              level.floor_z,level.floor_z+level.axial_depth);
    return stock;
  }

  const auto& mesh = geometry.box_stock() ? geometry.part().mesh() : geometry.stock().mesh();
  const double bottom = mesh.bounds.lower.z, top = mesh.bounds.upper.z;
  std::vector<double> cuts{bottom,top};
  // Use the slicer's existing slab approximation at machining boundaries.
  // Sampling every mesh vertex height can require millions of slices and does
  // not turn a slab model into an exact solid for sloping surfaces.
  for (double z : extra_boundaries)
    if (z > bottom && z < top) cuts.push_back(z);
  for (const auto& op : previous) if (op.ok()) for (const auto& level : op.levels) {
    for (double z : {level.floor_z,level.floor_z+level.axial_depth})
      if (z > bottom && z < top) cuts.push_back(z);
  }
  std::sort(cuts.begin(),cuts.end());
  cuts.erase(std::unique(cuts.begin(),cuts.end(), [](double a,double b){return std::abs(a-b)<1e-8;}),cuts.end());
  std::vector<StockSlab> result;
  for (std::size_t i=1;i<cuts.size();++i) {
    auto region = stock_after_operations_at_z(geometry.stock_outline(cuts[i-1],grid),
                                               (cuts[i-1]+cuts[i])/2,previous);
    if (!region.empty()) result.push_back({cuts[i-1],cuts[i],std::make_shared<const wave::Paths64>(std::move(region))});
  }
  return result;
}
}
