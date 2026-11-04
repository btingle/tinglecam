#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>
#include <thread>

#include "simulation/heightmap.h"
using namespace wave;
void check(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
void near(double a, double b, double e, const char* message) {
  if (std::abs(a - b) > e)
    throw std::runtime_error(std::string(message) + ": " + std::to_string(a) +
                             " vs " + std::to_string(b));
}
int main() {
  try {
    // Independent dense sampling of the cutter pose verifies the analytic swept
    // lower envelope, including diagonal ramps, vertical moves and end caps.
    std::mt19937 rng(61);
    std::uniform_real_distribution<double> d(-4, 4);
    for (auto shape : {CutterShape::Flat, CutterShape::Ball})
      for (int k = 0; k < 200; ++k) {
        Vec3d a{d(rng), d(rng), d(rng)}, b{d(rng), d(rng), d(rng)};
        if (k % 10 == 0) {
          b.x = a.x;
          b.y = a.y;
        }
        const double x = d(rng), y = d(rng);
        MillingCutter tool{shape, 2, 20};
        double reference = std::numeric_limits<double>::infinity();
        for (int i = 0; i <= 20000; ++i) {
          const auto p = interpolate(a, b, i / 20000.0);
          const double rho = std::hypot(x - p.x, y - p.y);
          if (rho <= tool.radius)
            reference = std::min(
                reference,
                p.z + (shape == CutterShape::Ball
                           ? 2 - std::sqrt(std::max(0.0, 4 - rho * rho))
                           : 0));
        }
        const double exact = swept_cutter_height(x, y, a, b, tool);
        if (std::isfinite(reference)) {
          check(exact <= reference + 1e-8,
                "Analytic envelope bounds sampled poses");
          near(exact, reference, 0.006,
               "Continuous envelope agrees with dense independent reference");
        } else
          check(!std::isfinite(exact), "Disjoint sweep stays empty");
      }
    Heightmap stock = Heightmap::box({{0, 0, 0}, {10, 10, 10}, true}, 0.1);
    const Heightmap original = stock;
    const double initial = stock.volume();
    MillingCutter flat{CutterShape::Flat, 1, 20};
    auto removed = stock.cut({0, 5, 5}, {10, 5, 5}, flat);
    near(removed.volume, 100, 0.2, "A full-width slot has the expected volume");
    near(initial - stock.volume(), removed.volume, 1e-5,
         "Removal volume conserves stock");
    near(original.volume(), 1000, 1e-5, "Snapshots remain immutable");
    check(stock.shared_tiles(original) > 0, "Unchanged tiles are shared");
    check(stock.shared_tiles(original) < original.shared_tiles(original),
          "Changed tiles are copied");
    near(stock.cut({10, 5, 5}, {0, 5, 5}, flat).volume, 0, 1e-8,
         "Reversing an executed sweep removes no stock");
    check(stock.clear({1, 5, 6}, {9, 5, 6},
                      MillingCutter{CutterShape::Flat, 0.8, 20}),
          "Narrower cutter clears the slot");
    check(!stock.clear({0, 2, 5}, {10, 2, 5}, flat),
          "Uncut material blocks traverse");
    check(!stock.shank_clear({5, 2, -15}, {5, 2, -15}, flat),
          "Stock above flute length blocks shaft");
    auto ball = Heightmap::box({{-3, -3, 0}, {3, 3, 4}, true}, 0.025);
    MillingCutter nose{CutterShape::Ball, 2, 20};
    const double ball_initial = ball.volume();
    const auto plunge = ball.cut({0, 0, 5}, {0, 0, 0}, nose);
    const double expected =
        4 * std::numbers::pi * 4 -
        std::numbers::pi * 8 / 3;  // cylinder down to hemisphere envelope
    near(ball_initial - ball.volume(), plunge.volume, 1e-8,
         "Ball removal conserves stored stock volume");
    near(plunge.volume, expected, 0.04,
         "Ball plunge leaves a hemispherical floor");
    check(ball.top(120, 120) < 0.001, "Ball tip reaches the centre floor");
    check(ball.top(190, 120) > 1, "Ball edge leaves higher stock");
    auto ramp = Heightmap::box({{0, 0, 0}, {10, 10, 10}, true}, 0.1);
    ramp.cut({0, 5, 8}, {10, 5, 2}, flat);
    check(ramp.top(20, 50) > ramp.top(80, 50) + 2,
          "Ramp removal uses actual XYZ");
    check(!ramp.slice(6).empty() && !ramp.slice(1).empty(),
          "Grid supplies planner slice polygons");
    auto raised=Heightmap::box({{0,0,3},{2,2,4},true},.25);
    check(raised.slice(1).empty(),"A plane below a suspended sheet remains empty");
    near(material_area(raised.project_above(1)),4,1e-9,"Above-plane projection includes a suspended sheet");
    check(raised.project_above(4).empty(),"Above-plane projection excludes material ending at the floor");
    const auto section=raised.slice_shared(1),envelope=raised.project_above_shared(1);
    check(section!=envelope && section->empty() && material_area(*envelope)==4 &&
          raised.project_above_shared(1)==envelope,
          "Cross-section and above-plane projection cache independently");
    auto lowered=raised;lowered.cut({-1,1,0},{3,1,0},{CutterShape::Flat,3,20});
    check(lowered.project_above(1).empty() && raised.project_above_shared(1)==envelope,
          "Cuts invalidate their projection without changing an inherited snapshot");
    check(!ramp.display_mesh(1000).triangles.empty(),
          "Grid supplies display triangles");
    auto fine_preview = Heightmap::box({{0, 0, 0}, {10, 10, 10}, true}, .025);
    check(original.display_stride(2500) == 2 &&
              fine_preview.display_stride(2500) == 8,
          "A fixed preview budget can hide changes in simulation pitch");
    near(original.pitch() * original.display_stride(2500),
         fine_preview.pitch() * fine_preview.display_stride(2500), 1e-8,
         "Coarse and fine grids can display at identical cell spacing");
    check(fine_preview.display_stride(40000) == 2,
          "Increasing preview detail resolves finer simulated cells");
    bool front_wall = false;
    for (const auto& triangle : original.display_mesh(20000).triangles) {
      const auto a = triangle.a, b = triangle.b, c = triangle.c;
      if (a.y == 0 && b.y == 0 && c.y == 0) {
        const double ny = (b.z - a.z) * (c.x - a.x) - (b.x - a.x) * (c.z - a.z);
        check(ny < 0, "Stock display walls face outwards");
        front_wall = true;
      }
    }
    check(front_wall, "Stock display includes its outside walls");
    bool invalid_motion = false;
    try {
      stock.cut({0, 0, std::numeric_limits<double>::quiet_NaN()}, {1, 1, 0},
                flat);
    } catch (const std::invalid_argument&) {
      invalid_motion = true;
    }
    check(invalid_motion, "Nonfinite stock motion is rejected");
    // 100 million cells and a hundred snapshots must scale with changed
    // tiles, rather than duplicating the tile directory or flat stock arrays.
    auto fine = Heightmap::box({{0, 0, 0}, {100, 100, 10}, true}, .01);
    std::vector<Heightmap> snapshots;
    snapshots.reserve(100);
    for (int i = 0; i < 100; ++i) {
      snapshots.push_back(fine);
      fine.set(i * 64, i * 32, 0, 5);
    }
    std::vector<const Heightmap*> maps{&fine};
    for (const auto& snapshot : snapshots) maps.push_back(&snapshot);
    check(Heightmap::memory_usage(maps).bytes < 16 * 1024 * 1024,
          "Fine-grid snapshot directories share storage");
    near(snapshots.front().volume(), 100000, 1e-5,
         "Large shared stock stays immutable");
    auto patterned = Heightmap::box({{0, 0, 0}, {10, 10, 10}, true}, .025);
    patterned.cut({5, -3, 2}, {5, 13, 2}, nose);
    std::vector<double> row;
    for (int c = 0; c < patterned.columns(); ++c)
      row.push_back(patterned.top(c, 100));
    const auto dense = Heightmap::memory_usage({&patterned}).bytes;
    patterned.compact_history();
    for (int c = 0; c < patterned.columns(); ++c)
      near(patterned.top(c, 100), row[c], 0,
           "Lossless pattern compaction preserves heights");
    check(Heightmap::memory_usage({&patterned}).bytes < dense / 2,
          "Repeated row heights compact without reducing grid resolution");
    std::array<std::shared_ptr<const Paths64>, 8> layer_queries;
    {
      std::vector<std::jthread> workers;
      for (std::size_t i = 0; i < layer_queries.size(); ++i)
        workers.emplace_back([&, i] {
          layer_queries[i] = patterned.slice_shared(4);
        });
    }
    const auto cached = layer_queries.front();
    for (const auto& query : layer_queries)
      check(query == cached, "Concurrent layer queries share one polygon");
    check(patterned.slice_shared(4) == cached &&
              patterned.slice_conversions() == 1,
          "A layer polygon is converted once per immutable stock version");
    auto edited = patterned;
    edited.cut({0, 2, 0}, {10, 2, 0}, flat);
    edited.slice_shared(4);
    check(patterned.slice_shared(4) == cached &&
              patterned.slice_conversions() == 1,
          "Stock edits preserve earlier polygon caches");
    const auto edge = to_integer(1.02), zero = to_integer(0);
    const Paths64 rectangle{
        {{zero, zero}, {edge, zero}, {edge, edge}, {zero, edge}}};
    const auto partial = Heightmap::from_polygons(rectangle, 0, 1, .1);
    near(partial.volume(), 1, 1e-8,
         "Fast rectangle initialization keeps centre-sampled edge cells");
    check(!std::isfinite(partial.top(10, 5)) &&
              !std::isfinite(partial.top(5, 10)),
          "Partial cells with centres outside the polygon stay empty");
    // Differential check of tile rejection and diagonal bounds against a
    // complete cell traversal with the independent public envelope query.
    for (auto shape : {CutterShape::Ball, CutterShape::Flat}) {
      auto grid = Heightmap::box({{-4, -4, -2}, {4, 4, 8}, true}, .125);
      MillingCutter tool{shape, .8, 20};
      for (int k = 0; k < 20; ++k) {
        Vec3d a{d(rng) * 4, d(rng) * 4, d(rng)},
            b{d(rng) * 4, d(rng) * 4, d(rng)};
        std::vector<float> expected;
        for (int r = 0; r < grid.rows(); ++r)
          for (int c = 0; c < grid.columns(); ++c)
            expected.push_back(float(std::max(
                grid.bottom(c, r),
                std::min(
                    grid.top(c, r),
                    swept_cutter_height(grid.x(c), grid.y(r), a, b, tool)))));
        grid.cut(a, b, tool);
        std::size_t i = 0;
        for (int r = 0; r < grid.rows(); ++r)
          for (int c = 0; c < grid.columns(); ++c)
            near(grid.top(c, r), expected[i++], 1e-6,
                 "Tile traversal matches complete swept-volume sampling");
      }
    }
    bool rejected = false;
    try {
      Heightmap huge(0, 0, 1e6, 1e6, .001);
    } catch (const std::runtime_error&) {
      rejected = true;
    }
    check(rejected, "Oversized grids fail with an actionable error");
    std::cout
        << "Heightmap sweep, volume, clearance and snapshot checks passed.\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
