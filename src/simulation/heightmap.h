#pragma once

#include <array>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <vector>
#include <utility>

#include "wave/mesh_slicer.h"

namespace wave {

enum class CutterShape { Flat, Ball, Drill };
struct MillingCutter {
  CutterShape shape = CutterShape::Flat;
  double radius = 3;
  double flute_length = 20;
  double point_angle_degrees = 118;
  double tip_flat_radius = 0;
};
struct RemovalSample {
  double volume = 0;
  double maximum_depth = 0;
  std::size_t cells = 0;
};

// A fixed world-space XY grid with floating point top/bottom heights. Copies
// share immutable tiles until changed, so operation and playback snapshots do
// not duplicate the entire stock. Interior cavities are intentionally omitted.
class Heightmap {
 public:
  static constexpr int tile_width = 32;
  struct Heights {
    float uniform = std::numeric_limits<float>::quiet_NaN();
    std::shared_ptr<std::array<float, tile_width * tile_width>> samples;
    std::shared_ptr<std::array<float, tile_width>> pattern;
    bool columns = false;
    float at(std::size_t i) const {
      return samples
                 ? (*samples)[i]
                 : (pattern
                        ? (*pattern)[columns ? i / tile_width : i % tile_width]
                        : uniform);
    }
    float* writable();
    void compact();
  };
  struct Tile {
    Heights top, bottom;
    float maximum_top = -std::numeric_limits<float>::infinity();
    int maximum_count = 0;
    void changed_top(float before, float after);
  };
  static constexpr int page_width = 256;
  struct Page {
    std::array<std::shared_ptr<Tile>, page_width> tiles;
  };
  struct SliceCache {
    struct Entry {
      std::once_flag ready;
      std::shared_ptr<const Paths64> polygon;
    };
    std::mutex mutex;
    std::map<std::pair<double, bool>, std::shared_ptr<Entry>> polygons;
    std::size_t conversions = 0;
  };
  struct MemoryUsage {
    std::size_t maps = 0, pages = 0, tiles = 0, height_arrays = 0, bytes = 0;
  };
  Heightmap() = default;
  Heightmap(double x, double y, double width, double height, double pitch);
  static Heightmap from_polygons(const Paths64& polygons, double bottom,
                                 double top, double pitch);
  static Heightmap from_mesh(const Mesh& mesh, double pitch);
  static Heightmap box(const Bounds3f& bounds, double pitch);
  // Exact grid remapping; never meshes or resamples remaining material.
  Heightmap transformed(const ModelTransform&) const;
  bool empty() const { return columns_ == 0 || rows_ == 0; }
  int columns() const { return columns_; }
  int rows() const { return rows_; }
  double pitch() const { return pitch_; }
  double x(int column) const { return x_ + (column + 0.5) * pitch_; }
  double y(int row) const { return y_ + (row + 0.5) * pitch_; }
  double lower_x() const { return x_; }
  double lower_y() const { return y_; }
  double top(int column, int row) const;
  double bottom(int column, int row) const;
  void set(int column, int row, double bottom, double top);
  void fill(const Paths64&, double bottom, double top);
  bool shank_clear(Vec3d from, Vec3d to, const MillingCutter&) const;
  void compact_history();
  double volume() const;
  double highest() const;
  double lowest() const;
  // Occupied grid cells at a depth, unioned into ordinary Clipper polygons.
  Paths64 slice(double z) const;
  std::shared_ptr<const Paths64> slice_shared(double z) const;
  // XY projection of all material above Z, including columns whose underside
  // is above the plane. Roughing needs this envelope rather than a section.
  Paths64 project_above(double z) const;
  std::shared_ptr<const Paths64> project_above_shared(double z) const;
  std::size_t slice_conversions() const;
  static MemoryUsage memory_usage(const std::vector<const Heightmap*>&);
  Paths64 footprint() const;
  RemovalSample cut(Vec3d from, Vec3d to, const MillingCutter& cutter);
  // Conservative clearance grows the cutter by half a cell diagonal, so a
  // touching sweep is never called clear. Sampled clearance asks only whether
  // cutting the sweep would remove anything, cell centre by cell centre.
  bool clear(Vec3d from, Vec3d to, const MillingCutter& cutter,
             double tolerance = 0.001, bool conservative = true) const;
  // Tip Z at first contact during a vertical descent. Uses the same expanded
  // flat/ball footprint as conservative clearance. Negative infinity means
  // that there is no remaining stock under the cutter.
  double plunge_contact_z(double x, double y, const MillingCutter&) const;
  int display_stride(std::size_t maximum_cells = 150000) const;
  Mesh display_mesh(std::size_t maximum_cells = 150000) const;
  // Immutable page snapshots for incremental render uploads. Holding these
  // references participates in the same copy-on-write rules as stock history.
  std::vector<std::shared_ptr<const Page>> tile_pages() const {
    return {pages_.begin(), pages_.end()};
  }
  std::size_t shared_tiles(const Heightmap& other) const;

 private:
  std::size_t tile_index(int column, int row) const;
  std::size_t cell_index(int column, int row) const;
  const std::shared_ptr<Tile>& tile_at(std::size_t) const;
  Tile& writable_tile(std::size_t);
  void invalidate_slices();
  void compact();
  void initialize_box(float bottom, float top);
  std::vector<std::shared_ptr<Page>> pages_;
  std::shared_ptr<SliceCache> slice_cache_ = std::make_shared<SliceCache>();
  std::size_t tile_count_ = 0;
  double volume_ = 0;
  double lowest_ = std::numeric_limits<double>::infinity();
  double x_ = 0, y_ = 0, pitch_ = 0.05;
  int columns_ = 0, rows_ = 0, tile_columns_ = 0;
  enum class Outline { Section, Above, Footprint };
  Paths64 outlines(double z, Outline) const;
  std::shared_ptr<const Paths64> outline_shared(double z, bool above) const;
  template <class Function>
  bool visit_sweep(Vec3d, Vec3d, double, Function) const;
};

// Lower envelope of the continuously swept flat/ball cutter at an XY sample.
// Infinity means that the sample lies outside the swept cutter footprint.
double swept_cutter_height(double x, double y, Vec3d from, Vec3d to,
                           const MillingCutter& cutter);
double distance(Vec3d a, Vec3d b);
Vec3d interpolate(Vec3d a, Vec3d b, double t);

}  // namespace wave
