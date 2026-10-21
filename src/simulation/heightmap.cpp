#include "simulation/heightmap.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace wave {
namespace {
constexpr double infinity = std::numeric_limits<double>::infinity();
bool finite(double x) { return std::isfinite(x); }
struct Span {
  double x;
  int winding;
};
void validate_motion(Vec3d a, Vec3d b, const MillingCutter& tool) {
  for (double value :
       {a.x, a.y, a.z, b.x, b.y, b.z, tool.radius, tool.flute_length})
    if (!finite(value))
      throw std::invalid_argument(
          "Simulation motion and cutter must be finite");
  if(tool.shape==CutterShape::Drill &&
      (!finite(tool.point_angle_degrees) || tool.point_angle_degrees<=0 || tool.point_angle_degrees>180 ||
       !finite(tool.tip_flat_radius) || tool.tip_flat_radius<0 || tool.tip_flat_radius>tool.radius))
    throw std::invalid_argument("Invalid drill point geometry");
  if (tool.radius <= 0 || tool.flute_length <= 0)
    throw std::invalid_argument(
        "Simulation cutter dimensions must be positive");
}
int first_cell(double value, int count) {
  return int(std::clamp(std::floor(value), 0.0, double(count)));
}
int last_cell(double value, int count) {
  return int(std::clamp(std::floor(value), -1.0, double(count - 1)));
}
template <class Function>
void raster(const Paths64& paths, const Heightmap& map, Function function) {
  // Scan conversion with NonZero winding; cost scales with rows and boundary
  // edges, not with every cell times every polygon.
  for (int row = 0; row < map.rows(); ++row) {
    const double y = map.y(row) * kIntegerScale;
    std::vector<Span> crossings;
    for (const auto& path : paths)
      for (std::size_t i = 0, j = path.size() - 1; i < path.size(); j = i++) {
        const auto a = path[j], b = path[i];
        if ((a.y <= y && b.y > y) || (b.y <= y && a.y > y))
          crossings.push_back(
              {a.x + (y - a.y) * double(b.x - a.x) / double(b.y - a.y),
               b.y > a.y ? 1 : -1});
      }
    std::sort(crossings.begin(), crossings.end(),
              [](auto a, auto b) { return a.x < b.x; });
    int winding = 0;
    for (std::size_t i = 0; i + 1 < crossings.size(); ++i) {
      winding += crossings[i].winding;
      if (!winding) continue;
      const int first = std::max(
          0, int(std::ceil((crossings[i].x / kIntegerScale - map.lower_x()) /
                               map.pitch() -
                           0.5)));
      const int last = std::min(
          map.columns() - 1,
          int(std::floor((crossings[i + 1].x / kIntegerScale - map.lower_x()) /
                             map.pitch() -
                         0.5)));
      for (int column = first; column <= last; ++column) function(column, row);
    }
  }
}
}  // namespace

double distance(Vec3d a, Vec3d b) {
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}
Vec3d interpolate(Vec3d a, Vec3d b, double t) {
  return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
}

float* Heightmap::Heights::writable() {
  if (!samples) {
    auto dense = std::make_shared<std::array<float, tile_width * tile_width>>();
    for (std::size_t i = 0; i < dense->size(); ++i) (*dense)[i] = at(i);
    samples = std::move(dense);
    pattern.reset();
  } else if (!samples.unique())
    samples =
        std::make_shared<std::array<float, tile_width * tile_width>>(*samples);
  return samples->data();
}
void Heightmap::Heights::compact() {
  if (!samples) return;
  const auto same = [](float a, float b) {
    return a == b || (std::isnan(a) && std::isnan(b));
  };
  const auto& values = *samples;
  bool uniform_values = true, rows = true, cols = true;
  for (std::size_t i = 0; i < values.size(); ++i) {
    uniform_values &= same(values[i], values[0]);
    rows &= same(values[i], values[i % tile_width]);
    cols &= same(values[i], values[(i / tile_width) * tile_width]);
    if (!uniform_values && !rows && !cols) return;
  }
  if (uniform_values) {
    uniform = values[0];
    samples.reset();
    return;
  }
  if (rows || cols) {
    pattern = std::make_shared<std::array<float, tile_width>>();
    columns = !rows;
    for (int i = 0; i < tile_width; ++i)
      (*pattern)[i] = values[columns ? i * tile_width : i];
    samples.reset();
  }
}
void Heightmap::Tile::changed_top(float before, float after) {
  if (std::isfinite(before) && before == maximum_top) --maximum_count;
  if (std::isfinite(after)) {
    if (after > maximum_top) {
      maximum_top = after;
      maximum_count = 1;
    } else if (after == maximum_top)
      ++maximum_count;
  }
  if (maximum_count == 0) {
    maximum_top = -infinity;
    if (top.samples)
      for (float value : *top.samples) {
        if (!std::isfinite(value)) continue;
        if (value > maximum_top) {
          maximum_top = value;
          maximum_count = 1;
        } else if (value == maximum_top)
          ++maximum_count;
      }
    else if (std::isfinite(top.uniform)) {
      maximum_top = top.uniform;
      maximum_count = tile_width * tile_width;
    }
  }
}
Heightmap::Heightmap(double x, double y, double width, double height,
                     double pitch)
    : x_(x), y_(y), pitch_(pitch) {
  if (!finite(x) || !finite(y) || !finite(width) || !finite(height) ||
      !finite(pitch) || pitch < 0.001 || width <= 0 || height <= 0)
    throw std::invalid_argument(
        "Heightmap requires finite positive bounds and pitch >= 0.001 mm");
  const double columns = std::ceil(width / pitch),
               rows = std::ceil(height / pitch);
  if (columns * rows > 128000000)
    throw std::runtime_error(
        "Heightmap exceeds 128 million cells; increase simulation pitch");
  columns_ = int(columns);
  rows_ = int(rows);
  tile_columns_ = (columns_ + tile_width - 1) / tile_width;
  tile_count_ =
      std::size_t(tile_columns_) * ((rows_ + tile_width - 1) / tile_width);
  auto tile = std::make_shared<Tile>();
  auto page = std::make_shared<Page>();
  page->tiles.fill(tile);
  pages_.resize((tile_count_ + page_width - 1) / page_width, page);
}
std::size_t Heightmap::tile_index(int c, int r) const {
  return (r / tile_width) * tile_columns_ + c / tile_width;
}
std::size_t Heightmap::cell_index(int c, int r) const {
  return (r % tile_width) * tile_width + c % tile_width;
}
const std::shared_ptr<Heightmap::Tile>& Heightmap::tile_at(
    std::size_t index) const {
  return pages_[index / page_width]->tiles[index % page_width];
}
Heightmap::Tile& Heightmap::writable_tile(std::size_t index) {
  auto& page = pages_[index / page_width];
  if (!page.unique()) page = std::make_shared<Page>(*page);
  auto& tile = page->tiles[index % page_width];
  if (!tile.unique()) tile = std::make_shared<Tile>(*tile);
  return *tile;
}
void Heightmap::invalidate_slices() {
  if (!slice_cache_.unique())
    slice_cache_ = std::make_shared<SliceCache>();
  else
    slice_cache_->polygons.clear();
}
void Heightmap::compact() {
  for (auto& page : pages_) {
    if (!page.unique()) continue;
    for (auto& tile : page->tiles)
      if (tile.unique()) {
        tile->top.compact();
        tile->bottom.compact();
      }
  }
}
void Heightmap::compact_history() { compact(); }
double Heightmap::top(int c, int r) const {
  return tile_at(tile_index(c, r))->top.at(cell_index(c, r));
}
double Heightmap::bottom(int c, int r) const {
  return tile_at(tile_index(c, r))->bottom.at(cell_index(c, r));
}
void Heightmap::set(int c, int r, double low, double high) {
  invalidate_slices();
  auto& tile = writable_tile(tile_index(c, r));
  const auto i = cell_index(c, r);
  const float before = tile.top.at(i), oldlow = tile.bottom.at(i),
              after = float(high), newlow = float(low);
  if (finite(before) && before > oldlow)
    volume_ -= (double(before) - oldlow) * pitch_ * pitch_;
  if (finite(after) && after > newlow)
    volume_ += (double(after) - newlow) * pitch_ * pitch_;
  tile.bottom.writable()[i] = newlow;
  tile.top.writable()[i] = after;
  tile.changed_top(before, after);
  if (finite(newlow)) lowest_ = std::min(lowest_, double(newlow));
  if (oldlow == lowest_ && newlow > oldlow) {
    lowest_ = infinity;
    for (int row = 0; row < rows_; ++row)
      for (int col = 0; col < columns_; ++col) {
        const double z = bottom(col, row);
        if (finite(z)) lowest_ = std::min(lowest_, z);
      }
  }
}
Heightmap Heightmap::from_polygons(const Paths64& paths, double low,
                                   double high, double pitch) {
  const auto bounds = Clipper2Lib::GetBounds(paths);
  if (paths.empty() || bounds.Width() <= 0 || bounds.Height() <= 0 ||
      !(high > low))
    return {};
  Heightmap result(to_world(bounds.left), to_world(bounds.top),
                   to_world(bounds.Width()), to_world(bounds.Height()), pitch);
  if (paths.size() == 1 && paths.front().size() == 4 &&
      std::abs(Clipper2Lib::Area(paths)) ==
          double(bounds.Width()) * bounds.Height()) {
    bool rectangle = true;
    for (const auto& p : paths.front())
      rectangle &= (p.x == bounds.left || p.x == bounds.right) &&
                   (p.y == bounds.top || p.y == bounds.bottom);
    if (rectangle) {
      result.initialize_box(float(low), float(high));
      // Preserve the centre-sampled polygon boundary in a partial edge cell.
      // Filling every rounded-up box cell would silently enlarge stock.
      const double nan = std::numeric_limits<double>::quiet_NaN();
      if (result.x(result.columns_ - 1) > to_world(bounds.right))
        for (int r = 0; r < result.rows_; ++r)
          result.set(result.columns_ - 1, r, nan, nan);
      if (result.y(result.rows_ - 1) > to_world(bounds.bottom))
        for (int c = 0; c < result.columns_; ++c)
          result.set(c, result.rows_ - 1, nan, nan);
      return result;
    }
  }
  raster(paths, result, [&](int c, int r) { result.set(c, r, low, high); });
  result.compact();
  return result;
}
void Heightmap::initialize_box(float low, float high) {
  auto tile = std::make_shared<Tile>();
  tile->top.uniform = high;
  tile->bottom.uniform = low;
  tile->maximum_top = high;
  tile->maximum_count = tile_width * tile_width;
  auto page = std::make_shared<Page>();
  page->tiles.fill(tile);
  std::fill(pages_.begin(), pages_.end(), page);
  lowest_ = low;
  volume_ = (double(high) - low) * pitch_ * pitch_ * columns_ * rows_;
}
Heightmap Heightmap::box(const Bounds3f& bounds, double pitch) {
  if (!bounds.valid) return {};
  Heightmap result(bounds.lower.x, bounds.lower.y,
                   bounds.upper.x - bounds.lower.x,
                   bounds.upper.y - bounds.lower.y, pitch);
  result.initialize_box(bounds.lower.z, bounds.upper.z);
  return result;
}
Heightmap Heightmap::from_mesh(const Mesh& mesh, double pitch) {
  if (!mesh.bounds.valid) return {};
  Heightmap result(mesh.bounds.lower.x, mesh.bounds.lower.y,
                   mesh.bounds.upper.x - mesh.bounds.lower.x,
                   mesh.bounds.upper.y - mesh.bounds.lower.y, pitch);
  // Rasterize XY triangles and retain their top/bottom envelope. Interior
  // geometry is deliberately outside this experimental representation.
  for (const auto& triangle : mesh.triangles) {
    const auto a = triangle.a, b = triangle.b, c = triangle.c;
    const double determinant =
        (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if (std::abs(determinant) < 1e-14) continue;
    int c0 = std::max(
        0, int(std::floor((std::min({a.x, b.x, c.x}) - result.x_) / pitch)));
    int c1 = std::min(
        result.columns_ - 1,
        int(std::ceil((std::max({a.x, b.x, c.x}) - result.x_) / pitch)));
    int r0 = std::max(
        0, int(std::floor((std::min({a.y, b.y, c.y}) - result.y_) / pitch)));
    int r1 = std::min(
        result.rows_ - 1,
        int(std::ceil((std::max({a.y, b.y, c.y}) - result.y_) / pitch)));
    for (int r = r0; r <= r1; ++r)
      for (int col = c0; col <= c1; ++col) {
        const double x = result.x(col) - a.x, y = result.y(r) - a.y;
        const double u = (x * (c.y - a.y) - y * (c.x - a.x)) / determinant;
        const double v = ((b.x - a.x) * y - (b.y - a.y) * x) / determinant;
        if (u < -1e-7 || v < -1e-7 || u + v > 1 + 1e-7) continue;
        const double z = a.z + u * (b.z - a.z) + v * (c.z - a.z);
        const double lo = result.bottom(col, r), hi = result.top(col, r);
        result.set(col, r, finite(lo) ? std::min(lo, z) : z,
                   finite(hi) ? std::max(hi, z) : z);
      }
  }
  result.compact();
  return result;
}
void Heightmap::fill(const Paths64& paths, double low, double high) {
  raster(paths, *this, [&](int c, int r) {
    const double oldlow = bottom(c, r), oldhigh = top(c, r);
    set(c, r, finite(oldlow) ? std::min(oldlow, low) : low,
        finite(oldhigh) ? std::max(oldhigh, high) : high);
  });
}
namespace {
struct SweepKernel {
  Vec3d a;
  double dx, dy, dz, length2, inverse_length2, radius, r2, critical;
  bool ball, drill;
  double cone_slope, tip_flat;
  SweepKernel(Vec3d from, Vec3d to, const MillingCutter& tool)
      : a(from),
        dx(to.x - from.x),
        dy(to.y - from.y),
        dz(to.z - from.z),
        length2(dx * dx + dy * dy),
        inverse_length2(length2 > 1e-24 ? 1 / length2 : 0),
        radius(tool.radius),
        r2(radius * radius),
        critical(length2 > 1e-24 ? dz / std::sqrt(length2 * (length2 + dz * dz))
                                 : 0),
        ball(tool.shape == CutterShape::Ball), drill(tool.shape==CutterShape::Drill),
        cone_slope(!drill || tool.point_angle_degrees>=180?0:
                   1/std::tan(tool.point_angle_degrees*std::acos(-1.0)/360)),
        tip_flat(tool.tip_flat_radius) {}
  double height(double x, double y) const {
    const double qx = x - a.x, qy = y - a.y;
    if (length2 < 1e-24) {
      const double rho2 = qx * qx + qy * qy;
      if (rho2 > r2) return infinity;
      return a.z + std::min(0.0, dz) +
             (ball ? radius - std::sqrt(std::max(0.0, r2 - rho2)) :
              drill?std::max(0.0,std::sqrt(rho2)-tip_flat)*cone_slope:0);
    }
    const double projection = (qx * dx + qy * dy) * inverse_length2;
    if (!ball && !drill && dz == 0) {
      const double t = std::clamp(projection, 0.0, 1.0), ex = qx - t * dx,
                   ey = qy - t * dy;
      return ex * ex + ey * ey <= r2 ? a.z : infinity;
    }
    const double perpendicular2 =
        std::max(0.0, qx * qx + qy * qy - projection * projection * length2);
    if (perpendicular2 > r2) return infinity;
    const double h = std::sqrt(std::max(0.0, r2 - perpendicular2));
    const double half = h * std::sqrt(inverse_length2);
    const double lo = std::max(0.0, projection - half),
                 hi = std::min(1.0, projection + half);
    if (lo > hi) return infinity;
    if(drill) {
      // The swept conical profile is convex along a straight segment. Drilling
      // itself takes the analytic vertical branch above; this handles clearance
      // queries and uncommon sloped motions with the same tip geometry.
      const auto at=[&](double t) {
        const double rho=std::hypot(qx-t*dx,qy-t*dy);
        return a.z+t*dz+std::max(0.0,rho-tip_flat)*cone_slope;
      };
      double left=lo,right=hi;
      for(int i=0;i<48;++i) {
        const double l=(2*left+right)/3,r=(left+2*right)/3;
        if(at(l)<at(r)) right=r;else left=l;
      }
      return std::min({at(lo),at(hi),at((left+right)*.5)});
    }
    const double t = ball ? std::clamp(projection - critical * h, lo, hi)
                          : (dz >= 0 ? lo : hi);
    const double ex = qx - t * dx, ey = qy - t * dy;
    return a.z + t * dz +
           (ball ? radius - std::sqrt(std::max(0.0, r2 - ex * ex - ey * ey))
                 : 0);
  }
};
}  // namespace
template <class Function>
bool Heightmap::visit_sweep(Vec3d a, Vec3d b, double radius,
                            Function function) const {
  if (empty()) return true;
  const int first =
      first_cell((std::min(a.y, b.y) - radius - y_) / pitch_, rows_);
  const int last =
      last_cell((std::max(a.y, b.y) + radius - y_) / pitch_, rows_);
  for (int ty = first / tile_width; ty <= last / tile_width && first <= last;
       ++ty) {
    const int r0 = std::max(first, ty * tile_width),
              r1 = std::min(last, (ty + 1) * tile_width - 1);
    double lo = 0, hi = 1;
    const double dy = b.y - a.y;
    if (std::abs(dy) > 1e-20) {
      double t0 = (y(r0) - radius - a.y) / dy, t1 = (y(r1) + radius - a.y) / dy;
      if (t0 > t1) std::swap(t0, t1);
      lo = std::max(0.0, t0);
      hi = std::min(1.0, t1);
      if (lo > hi) continue;
    }
    const double x0 = a.x + (b.x - a.x) * lo, x1 = a.x + (b.x - a.x) * hi;
    const int first_c =
        first_cell((std::min(x0, x1) - radius - x_) / pitch_, columns_);
    const int last_c =
        last_cell((std::max(x0, x1) + radius - x_) / pitch_, columns_);
    for (int tx = first_c / tile_width;
         tx <= last_c / tile_width && first_c <= last_c; ++tx)
      if (!function(std::size_t(ty) * tile_columns_ + tx,
                    std::max(first_c, tx * tile_width),
                    std::min(last_c, (tx + 1) * tile_width - 1), r0, r1))
        return false;
  }
  return true;
}
bool Heightmap::shank_clear(Vec3d a, Vec3d b, const MillingCutter& tool) const {
  validate_motion(a, b, tool);
  a.z += tool.flute_length;
  b.z += tool.flute_length;
  const SweepKernel kernel(a, b,
                           {CutterShape::Flat, tool.radius, tool.flute_length});
  const double floor = std::min(a.z, b.z) + 1e-5;
  return visit_sweep(a, b, tool.radius,
                     [&](std::size_t index, int c0, int c1, int r0, int r1) {
                       const auto& tile = *tile_at(index);
                       if (tile.maximum_top <= floor) return true;
                       for (int r = r0; r <= r1; ++r)
                         for (int c = c0; c <= c1; ++c) {
                           const auto i = cell_index(c, r);
                           const double top = tile.top.at(i);
                           if (finite(top) && top > tile.bottom.at(i) &&
                               top > kernel.height(x(c), y(r)) + 1e-5)
                             return false;
                         }
                       return true;
                     });
}
double Heightmap::volume() const { return volume_; }
double Heightmap::highest() const {
  double value = -infinity;
  for (std::size_t i = 0; i < tile_count_; ++i) {
    const int col = int(i % tile_columns_) * tile_width,
              row = int(i / tile_columns_) * tile_width;
    const auto& tile = *tile_at(i);
    if (col + tile_width <= columns_ && row + tile_width <= rows_)
      value = std::max(value, double(tile.maximum_top));
    else
      for (int r = row; r < std::min(rows_, row + tile_width); ++r)
        for (int c = col; c < std::min(columns_, col + tile_width); ++c) {
          const double z = tile.top.at(cell_index(c, r));
          if (finite(z)) value = std::max(value, z);
        }
  }
  return value;
}
double Heightmap::lowest() const { return lowest_; }
Paths64 Heightmap::outlines(double z, bool all) const {
  Paths64 rectangles;
  std::unordered_map<std::uint64_t, std::size_t> previous, current;
  const auto occupied = [&](float top, float bottom) {
    return finite(top) && top > bottom &&
           (all || (top > z + 1e-7 && bottom <= z + 1e-7));
  };
  for (int r = 0; r < rows_;) {
    int repeat = std::min(tile_width - r % tile_width, rows_ - r);
    for (int tx = 0; tx < tile_columns_ && repeat > 1; ++tx) {
      const auto& tile =
          *tile_at(std::size_t(r / tile_width) * tile_columns_ + tx);
      for (const auto* h : {&tile.top, &tile.bottom})
        if (h->samples || (h->pattern && h->columns)) {
          repeat = 1;
          break;
        }
    }
    current.clear();
    int first = -1;
    const auto emit = [&](int end) {
      if (first < 0) return;
      const auto key = (std::uint64_t(first) << 32) | std::uint32_t(end);
      const auto y1 = to_integer(y_ + (r + repeat) * pitch_);
      if (auto found = previous.find(key); found != previous.end()) {
        auto& rectangle = rectangles[found->second];
        rectangle[2].y = rectangle[3].y = y1;
        current[key] = found->second;
      } else {
        const auto x0 = to_integer(x_ + first * pitch_),
                   x1 = to_integer(x_ + end * pitch_),
                   y0 = to_integer(y_ + r * pitch_);
        current[key] = rectangles.size();
        rectangles.push_back({{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}});
      }
      first = -1;
    };
    for (int c = 0; c < columns_;) {
      const auto& tile = *tile_at(tile_index(c, r));
      const int end = std::min(columns_, (c / tile_width + 1) * tile_width);
      const bool constant =
          !tile.top.samples && (!tile.top.pattern || tile.top.columns) &&
          !tile.bottom.samples && (!tile.bottom.pattern || tile.bottom.columns);
      if (constant) {
        const auto i = cell_index(c, r);
        if (occupied(tile.top.at(i), tile.bottom.at(i))) {
          if (first < 0) first = c;
        } else
          emit(c);
        c = end;
      } else
        for (; c < end; ++c) {
          const auto i = cell_index(c, r);
          if (occupied(tile.top.at(i), tile.bottom.at(i))) {
            if (first < 0) first = c;
          } else
            emit(c);
        }
    }
    emit(columns_);
    previous.swap(current);
    r += repeat;
  }
  return Clipper2Lib::Union(rectangles, Clipper2Lib::FillRule::NonZero);
}
std::shared_ptr<const Paths64> Heightmap::slice_shared(double z) const {
  if (!std::isfinite(z))
    throw std::invalid_argument("Stock slice Z must be finite");
  std::shared_ptr<SliceCache::Entry> entry;
  {
    std::lock_guard lock(slice_cache_->mutex);
    auto [found, inserted] = slice_cache_->polygons.try_emplace(z);
    if (inserted) found->second = std::make_shared<SliceCache::Entry>();
    entry = found->second;
  }
  // Coalesce concurrent requests for the same layer while allowing different
  // layers to polygonize independently. call_once also retries on failure.
  std::call_once(entry->ready, [&] {
    entry->polygon = std::make_shared<const Paths64>(outlines(z, false));
    std::lock_guard lock(slice_cache_->mutex);
    ++slice_cache_->conversions;
  });
  return entry->polygon;
}
std::size_t Heightmap::slice_conversions() const {
  std::lock_guard lock(slice_cache_->mutex);
  return slice_cache_->conversions;
}
Paths64 Heightmap::slice(double z) const { return *slice_shared(z); }
Paths64 Heightmap::footprint() const { return outlines(0, true); }

double swept_cutter_height(double x, double y, Vec3d a, Vec3d b,
                           const MillingCutter& tool) {
  return SweepKernel(a, b, tool).height(x, y);
}
RemovalSample Heightmap::cut(Vec3d a, Vec3d b, const MillingCutter& tool) {
  validate_motion(a, b, tool);
  invalidate_slices();
  RemovalSample result;
  const SweepKernel kernel(a, b, tool);
  const double minimum = std::min(a.z, b.z);
  visit_sweep(a, b, tool.radius,
              [&](std::size_t index, int c0, int c1, int r0, int r1) {
                const auto* original = tile_at(index).get();
                if (original->maximum_top <= minimum) return true;
                Tile* write = nullptr;
                float* heights = nullptr;
                for (int r = r0; r <= r1; ++r)
                  for (int c = c0; c <= c1; ++c) {
                    const auto i = cell_index(c, r);
                    const double before = original->top.at(i),
                                 low = original->bottom.at(i);
                    if (!finite(before) || before <= low || before <= minimum)
                      continue;
                    const double desired = std::max(
                        low, std::min(before, kernel.height(x(c), y(r))));
                    const double after =
                        std::clamp(double(float(desired)), low, before);
                    if (after >= before) continue;
                    if (!write) {
                      write = &writable_tile(index);
                      heights = write->top.writable();
                    }
                    heights[i] = float(after);
                    write->changed_top(float(before), float(after));
                    result.volume += (before - after) * pitch_ * pitch_;
                    result.maximum_depth =
                        std::max(result.maximum_depth, before - after);
                    ++result.cells;
                  }
                if (write && write->maximum_count == tile_width * tile_width) {
                  write->top.uniform = write->maximum_top;
                  write->top.samples.reset();
                  write->top.pattern.reset();
                }
                return true;
              });
  volume_ -= result.volume;
  return result;
}
bool Heightmap::clear(Vec3d a, Vec3d b, const MillingCutter& tool,
                      double tolerance, bool conservative) const {
  validate_motion(a, b, tool);
  if (!finite(tolerance) || tolerance < 0)
    throw std::invalid_argument("Invalid clearance tolerance");
  const double diagonal = conservative ? pitch_ * std::sqrt(0.5) : 0.0;
  auto expanded = tool;
  expanded.radius += diagonal;
  if(tool.shape==CutterShape::Drill) expanded.tip_flat_radius+=diagonal;
  if (tool.shape == CutterShape::Ball) {
    a.z -= diagonal;
    b.z -= diagonal;
  }
  const SweepKernel kernel(a, b, expanded);
  const double minimum = std::min(a.z, b.z) + tolerance;
  return visit_sweep(a, b, expanded.radius,
                     [&](std::size_t index, int c0, int c1, int r0, int r1) {
                       const auto& tile = *tile_at(index);
                       if (tile.maximum_top <= minimum) return true;
                       for (int r = r0; r <= r1; ++r)
                         for (int c = c0; c <= c1; ++c) {
                           const auto i = cell_index(c, r);
                           const double top = tile.top.at(i);
                           if (finite(top) && top > tile.bottom.at(i) &&
                               top > kernel.height(x(c), y(r)) + tolerance)
                             return false;
                         }
                       return true;
                     });
}
double Heightmap::plunge_contact_z(double px, double py,
                                  const MillingCutter& tool) const {
  const Vec3d at{px, py, 0};
  validate_motion(at, at, tool);
  const double diagonal = pitch_ * std::sqrt(0.5);
  auto expanded = tool;
  expanded.radius += diagonal;
  if(tool.shape==CutterShape::Drill) expanded.tip_flat_radius+=diagonal;
  const Vec3d tip{px, py, tool.shape == CutterShape::Ball ? -diagonal : 0};
  const SweepKernel profile(tip, tip, expanded);
  double contact = -infinity;
  visit_sweep(at, at, expanded.radius,
              [&](std::size_t index, int c0, int c1, int r0, int r1) {
                const auto& tile = *tile_at(index);
                if (tile.maximum_top + diagonal <= contact) return true;
                for (int r = r0; r <= r1; ++r)
                  for (int c = c0; c <= c1; ++c) {
                    const auto i = cell_index(c, r);
                    const double top = tile.top.at(i);
                    if (finite(top) && top > tile.bottom.at(i))
                      contact = std::max(contact, top - profile.height(x(c), y(r)));
                  }
                return true;
              });
  return contact;
}

int Heightmap::display_stride(std::size_t maximum_cells) const {
  return std::max(1, int(std::ceil(std::sqrt(
                         double(columns_) * rows_ /
                         std::max(std::size_t{1}, maximum_cells)))));
}
Mesh Heightmap::display_mesh(std::size_t maximum_cells) const {
  Mesh mesh;
  if (empty()) return mesh;
  const int stride = display_stride(maximum_cells);
  const auto add = [&](Vec3f a, Vec3f b, Vec3f c) {
    mesh.triangles.push_back({a, b, c});
  };
  for (int r = 0; r < rows_; r += stride)
    for (int c = 0; c < columns_; c += stride) {
      if (!finite(top(c, r)) || top(c, r) <= bottom(c, r)) continue;
      const float x0 = float(x_ + c * pitch_),
                  x1 = float(x_ + std::min(columns_, c + stride) * pitch_);
      const float y0 = float(y_ + r * pitch_),
                  y1 = float(y_ + std::min(rows_, r + stride) * pitch_);
      const float z = float(top(c, r)), lo = float(bottom(c, r));
      Vec3f a{x0, y0, z}, b{x1, y0, z}, d{x0, y1, z}, e{x1, y1, z};
      add(a, b, e);
      add(a, e, d);
      const auto wall = [&](Vec3f p, Vec3f q, int nc, int nr) {
        const double nz = nc >= 0 && nr >= 0 && nc < columns_ && nr < rows_
                              ? top(nc, nr)
                              : lo;
        const float floor = float(finite(nz) ? std::max(double(lo), nz) : lo);
        if (floor >= z) return;
        Vec3f p0{p.x, p.y, floor}, q0{q.x, q.y, floor};
        add(p, q, q0);
        add(p, q0, p0);
      };
      wall(b, a, c, r - stride);
      wall(e, b, c + stride, r);
      wall(d, e, c, r + stride);
      wall(a, d, c - stride, r);
    }
  mesh.bounds = {{float(x_), float(y_), float(lowest())},
                 {float(x_ + columns_ * pitch_), float(y_ + rows_ * pitch_),
                  float(highest())},
                 true};
  return mesh;
}
std::size_t Heightmap::shared_tiles(const Heightmap& other) const {
  std::size_t count = 0;
  for (std::size_t i = 0; i < std::min(tile_count_, other.tile_count_); ++i)
    count += tile_at(i) == other.tile_at(i);
  return count;
}
Heightmap::MemoryUsage Heightmap::memory_usage(
    const std::vector<const Heightmap*>& maps) {
  MemoryUsage usage;
  std::unordered_set<const Heightmap*> seen_maps;
  std::unordered_set<const Page*> pages;
  std::unordered_set<const Tile*> tiles;
  std::unordered_set<const void*> arrays;
  for (const auto* map : maps)
    if (map && seen_maps.insert(map).second) {
      ++usage.maps;
      usage.bytes += sizeof(Heightmap) +
                     map->pages_.capacity() * sizeof(std::shared_ptr<Page>);
      for (const auto& page : map->pages_)
        if (pages.insert(page.get()).second) {
          ++usage.pages;
          usage.bytes += sizeof(Page);
          for (const auto& tile : page->tiles)
            if (tile && tiles.insert(tile.get()).second) {
              ++usage.tiles;
              usage.bytes += sizeof(Tile);
              for (const auto* heights : {&tile->top, &tile->bottom})
                if (heights->samples &&
                    arrays.insert(heights->samples.get()).second) {
                  ++usage.height_arrays;
                  usage.bytes += sizeof(*heights->samples);
                } else if (heights->pattern &&
                           arrays.insert(heights->pattern.get()).second)
                  usage.bytes += sizeof(*heights->pattern);
            }
        }
    }
  return usage;
}
}  // namespace wave
