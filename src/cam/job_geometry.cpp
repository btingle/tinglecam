#include "job_internal.h"
#include <bit>
namespace cam {
using namespace detail;
using namespace detail;
const char* operation_mode_name(OperationMode value) {
  switch (value) {
    case OperationMode::Roughing: return "roughing";
    case OperationMode::ContourFinishing: return "contour_finishing";
    case OperationMode::Pocket: return "pocket";
    case OperationMode::PlaneClearing: return "plane_clearing";
  }
  return "unknown";
}

void JobGeometry::load_part(const std::filesystem::path& file) {
  part_.set_mesh(wave::load_triangle_mesh(file));
  part_slices_.clear();
  box_.clear();
}

void JobGeometry::load_stock(const std::filesystem::path& file) {
  stock_.set_mesh(wave::load_triangle_mesh(file));
  stock_slices_.clear();
}

void JobGeometry::clear_stock() {
  stock_.clear();
  stock_slices_.clear();
}

namespace {

double nonnegative_margin(double value) {
  return std::isfinite(value) ? std::max(value, 0.0) : 0.0;
}

BoxStockMargins normalize_box_margins(const BoxStockMargins& margins) {
  return {nonnegative_margin(margins.minus_x),
          nonnegative_margin(margins.plus_x),
          nonnegative_margin(margins.minus_y),
          nonnegative_margin(margins.plus_y),
          nonnegative_margin(margins.minus_z),
          nonnegative_margin(margins.plus_z)};
}

bool same_box_margins(const BoxStockMargins& a, const BoxStockMargins& b) {
  return a.minus_x == b.minus_x && a.plus_x == b.plus_x &&
         a.minus_y == b.minus_y && a.plus_y == b.plus_y &&
         a.minus_z == b.minus_z && a.plus_z == b.plus_z;
}

}  // namespace

void JobGeometry::set_box_stock(bool enabled, const BoxStockMargins& margins) {
  const BoxStockMargins normalized = normalize_box_margins(margins);
  if (box_stock_ == enabled && same_box_margins(box_margins_, normalized)) return;
  box_stock_ = enabled;
  box_margins_ = normalized;
  box_.clear();
  stock_slices_.clear();
}

void JobGeometry::set_box_stock(bool enabled, double margin) {
  set_box_stock(enabled, BoxStockMargins{margin, margin, margin, margin, 0.0, 0.0});
}

void JobGeometry::forget_slices() {
  part_slices_.clear();
  stock_slices_.clear();
}

const wave::Paths64& JobGeometry::outline(const wave::MeshSlicer& slicer,
                                          std::map<SliceKey, wave::Paths64>& cache,
                                          double z, int grid) const {
  // The float the slicer takes is the key, not the double asked for, so two
  // depths that reach the same slice share one entry rather than cutting twice.
  const SliceKey key{
      std::bit_cast<std::uint32_t>(static_cast<float>(z)),
      grid};
  {
    std::lock_guard<std::mutex> lock(slices_);
    const auto found = cache.find(key);
    if (found != cache.end()) return found->second;
  }
  // Sliced outside the lock: it is the expensive half, and a second thread
  // arriving at the same depth may as well cut it twice as wait. The map is
  // what needs protecting, not the work.
  auto slice = slicer.slice(static_cast<float>(z), grid);
  auto region = Union(slice.polygons, FillRule::NonZero);
  std::lock_guard<std::mutex> lock(slices_);
  return cache.emplace(key, std::move(region)).first->second;
}

const wave::Paths64& JobGeometry::part_outline(double z, int grid) const {
  return outline(part_, part_slices_, z, grid);
}

const wave::Paths64& JobGeometry::stock_outline(double z, int grid) const {
  if (!box_stock()) return outline(stock_, stock_slices_, z, grid);
  // One rectangle, the same at every depth, so the depth is not part of the
  // question. An empty part leaves it empty and the level reports nothing to
  // cut, which is what a missing model should look like.
  std::lock_guard<std::mutex> lock(slices_);
  if (box_.empty() && !part_.empty()) {
    const auto& bounds = part_.mesh().bounds;
    if (bounds.valid) {
      const std::int64_t left =
          wave::to_integer(bounds.lower.x - box_margins_.minus_x);
      const std::int64_t right =
          wave::to_integer(bounds.upper.x + box_margins_.plus_x);
      const std::int64_t bottom =
          wave::to_integer(bounds.lower.y - box_margins_.minus_y);
      const std::int64_t top =
          wave::to_integer(bounds.upper.y + box_margins_.plus_y);
      box_ = wave::Paths64{wave::Path64{{left, bottom},
                                        {right, bottom},
                                        {right, top},
                                        {left, top}}};
    }
  }
  return box_;
}

bool JobGeometry::work_range(double& low, double& high) const {
  // The union, not the overlap. Either model on its own is a reason to be at a
  // depth: blank above the part is material to face off, part below the blank
  // is a hole to descend into. Taking the overlap quietly clipped both away,
  // which is how a blank standing proud of its part came to be left standing.
  if (box_stock() && !part_.empty() && part_.mesh().bounds.valid) {
    const auto& bounds = part_.mesh().bounds;
    low = bounds.lower.z - box_margins_.minus_z;
    high = bounds.upper.z + box_margins_.plus_z;
    return high > low;
  }

  bool any = false;
  double lowest = 0.0;
  double highest = 0.0;
  for (const wave::MeshSlicer* slicer : {&part_, &stock_}) {
    if (slicer->empty()) continue;
    const auto& bounds = slicer->mesh().bounds;
    if (!bounds.valid) continue;
    if (!any) {
      lowest = bounds.lower.z;
      highest = bounds.upper.z;
      any = true;
      continue;
    }
    lowest = std::min<double>(lowest, bounds.lower.z);
    highest = std::max<double>(highest, bounds.upper.z);
  }
  if (!any || !(highest > lowest)) return false;
  low = lowest;
  high = highest;
  return true;
}

double JobGeometry::stock_bottom() const {
  if (!stock_.empty() && stock_.mesh().bounds.valid)
    return stock_.mesh().bounds.lower.z;
  if (box_stock_ && !part_.empty() && part_.mesh().bounds.valid)
    return part_.mesh().bounds.lower.z - box_margins_.minus_z;
  return std::numeric_limits<double>::quiet_NaN();
}

std::vector<double> level_floors(const Operation& operation, double stepdown,
                                 double work_top) {
  std::vector<double> floors;
  if (!std::isfinite(work_top) || !std::isfinite(operation.depth_from) ||
      !std::isfinite(operation.depth_to) || !std::isfinite(stepdown) ||
      !(stepdown > 0.0)) return floors;
  const double top = z_at_depth(work_top, operation.depth_from);
  const double bottom = z_at_depth(work_top, operation.depth_to);
  if (!(top > bottom)) return floors;
  const double step = std::max(stepdown, 1e-6);
  std::vector<double> boundaries;
  if (operation.mode == OperationMode::Roughing) {
    for (double depth : operation.depth_breaks) {
      if (!std::isfinite(depth) || !(depth > operation.depth_from &&
                                   depth < operation.depth_to)) return {};
      boundaries.push_back(z_at_depth(work_top, depth));
    }
  }
  boundaries.push_back(bottom);
  std::sort(boundaries.begin(), boundaries.end(), std::greater<double>());
  boundaries.erase(std::unique(boundaries.begin(), boundaries.end(),
      [](double a, double b) { return std::abs(a-b) < 1e-9; }), boundaries.end());
  double z = top;
  // A guard rather than a limit: at a sane stepdown no real part gets close.
  for (double boundary : boundaries) {
    while (z > boundary + 1e-9) {
      if (floors.size() >= 20000) return {}; // Never return a truncated descent.
      const double next = z - step;
      z = next <= boundary + 1e-9 ? boundary : next;
      floors.push_back(z);
    }
  }
  return floors;
}

}  // namespace cam
