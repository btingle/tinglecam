#include "routing.h"

#include <clipper2/clipper.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>

namespace {

using Clipper2Lib::FillRule;
using wave::Path64;
using wave::Paths64;
using wave::Point64;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

Point64 mm(double x, double y) {
  return Point64(static_cast<std::int64_t>(x * wave::kIntegerScale),
                 static_cast<std::int64_t>(y * wave::kIntegerScale));
}

Path64 box(double x0, double y0, double x1, double y1) {
  return Path64{mm(x0, y0), mm(x1, y0), mm(x1, y1), mm(x0, y1)};
}

double length_mm(const Path64& path) {
  double total = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i)
    total += std::hypot(static_cast<double>(path[i].x - path[i - 1].x),
                        static_cast<double>(path[i].y - path[i - 1].y));
  return total / wave::kIntegerScale;
}

// A hundred-millimetre room with a wall standing in it, open only over the top.
Paths64 room_with_wall(double wall_top) {
  return Clipper2Lib::Difference(Paths64{box(0.0, 0.0, 100.0, 100.0)},
                                 Paths64{box(50.0, 0.0, 55.0, wall_top)},
                                 FillRule::NonZero);
}

// The slack the planner grants a route: the same allowance two separately
// rounded descriptions of one wall need before they agree.
constexpr double kSlack = 4.0;

void check_geodesic() {
  const wave::Router router(room_with_wall(80.0), kSlack);
  require(router.usable(), "a room with a wall in it should be routable");
  const auto route = router.route(mm(25.0, 40.0), mm(75.0, 40.0));
  require(route.has_value(), "there is a way around the wall");
  require(route->size() == 4, "the way around turns at both corners of the wall");

  // Straight over the top of the wall and down: 47.170 + 5 + 44.721.
  const double expected = std::hypot(25.0, 40.0) + 5.0 + std::hypot(20.0, 40.0);
  require(std::abs(length_mm(*route) - expected) < 0.05,
          "the route around the wall is the shortest one, not a walk of the wall");

  // What makes this worth having: the walk this replaces followed the boundary
  // the whole way, and the boundary is far longer than the way past it.
  const double boundary_walk = 40.0 + 25.0 + 100.0 + 20.0 + 40.0;
  require(length_mm(*route) < boundary_walk * 0.6,
          "a routed detour should beat following the outline around");
}

void check_clear_line_is_left_alone() {
  const wave::Router router(room_with_wall(80.0), kSlack);
  const auto route = router.route(mm(10.0, 10.0), mm(10.0, 90.0));
  require(route.has_value(), "an unobstructed pair is connected");
  require(route->size() == 2,
          "an unobstructed pair is one segment, with nothing inserted into it");
  require(std::abs(length_mm(*route) - 80.0) < 1e-6,
          "an unobstructed route is the straight line");
}

void check_separated_rooms() {
  // The same wall, run the whole way across. There is no way round it, and
  // saying so is what lets the caller fall back to lifting.
  const Paths64 split = Clipper2Lib::Difference(
      Paths64{box(0.0, 0.0, 100.0, 100.0)}, Paths64{box(50.0, -10.0, 55.0, 110.0)},
      FillRule::NonZero);
  const wave::Router router(split, kSlack);
  require(!router.route(mm(25.0, 40.0), mm(75.0, 40.0)).has_value(),
          "two rooms with no door between them are not connected");
  require(router.route(mm(25.0, 40.0), mm(25.0, 80.0)).has_value(),
          "a pair on one side of it still is");
}

void check_route_stays_inside() {
  // An island in the middle: the route has to go round it, and every point of
  // what comes back has to be somewhere the tool may be.
  const Paths64 free = Clipper2Lib::Difference(
      Paths64{box(0.0, 0.0, 100.0, 100.0)}, Paths64{box(40.0, 40.0, 60.0, 60.0)},
      FillRule::NonZero);
  const wave::Router router(free, kSlack);
  const auto route = router.route(mm(20.0, 50.0), mm(80.0, 50.0));
  require(route.has_value(), "an island can be walked around");
  require(route->size() > 2, "the straight line through the island is not it");

  Clipper2Lib::Clipper64 clipper;
  clipper.AddOpenSubject(Paths64{*route});
  clipper.AddClip(Clipper2Lib::InflatePaths(free, 3.0 * kSlack,
                                            Clipper2Lib::JoinType::Miter,
                                            Clipper2Lib::EndType::Polygon));
  Paths64 closed;
  Paths64 open;
  require(clipper.Execute(Clipper2Lib::ClipType::Intersection, FillRule::NonZero,
                          closed, open),
          "clipping the route against the region should succeed");
  double inside = 0.0;
  for (const Path64& piece : open) inside += length_mm(piece);
  require(std::abs(inside - length_mm(*route)) < 0.01,
          "every millimetre of a route lies in the region it was routed through");
}

// A boundary carrying the structure a real one carries. The region is built the
// way the planner builds a cleared region -- a shape offset out and back on the
// integer grid -- so its boundary wanders by a unit or two from vertex to
// vertex and runs near-collinear for long stretches.
//
// This is the case that has to survive, and the way it fails is worth naming:
// not a bad route, but a report that a perfectly good one does not exist. A
// straight line between two points on such a boundary can leave the region
// through a vertex, which touches rather than crosses, so the arithmetic that
// admits it sees nothing wrong; the check at the end then throws out the whole
// route for the sake of the one bad segment in it, and the planner lifts.
void check_noisy_boundary() {
  Path64 lobed;
  const int steps = 720;
  for (int i = 0; i < steps; ++i) {
    const double angle = 2.0 * 3.14159265358979 * i / steps;
    const double radius = 18.0 + 2.0 * std::sin(5.0 * angle);
    lobed.push_back(mm(50.0 + radius * std::cos(angle),
                       50.0 + radius * std::sin(angle)));
  }
  Paths64 solid = Clipper2Lib::InflatePaths(Paths64{lobed}, 2000.0,
                                            Clipper2Lib::JoinType::Round,
                                            Clipper2Lib::EndType::Polygon);
  solid = Clipper2Lib::InflatePaths(solid, -2000.0, Clipper2Lib::JoinType::Round,
                                    Clipper2Lib::EndType::Polygon);
  const Paths64 free = Clipper2Lib::Difference(
      Paths64{box(0.0, 0.0, 100.0, 100.0)}, solid, FillRule::NonZero);

  const wave::Router router(free, kSlack);
  require(router.usable(), "a region with a wandering boundary is still a region");
  const auto route = router.route(mm(50.0, 3.0), mm(50.0, 97.0));
  require(route.has_value(),
          "a boundary that wanders by a unit or two must not hide the way "
          "around the obstacle it bounds");
  require(length_mm(*route) > 94.0,
          "the straight line runs through the obstacle, so the route is longer");
  require(length_mm(*route) < 130.0,
          "and it is the way around, not a tour of the region");
}

void check_empty_region() {
  const wave::Router router(Paths64{}, kSlack);
  require(!router.usable(), "there is nothing to route through in an empty region");
  require(!router.route(mm(0.0, 0.0), mm(1.0, 1.0)).has_value(),
          "an unusable router connects nothing");
}

}  // namespace

int main() {
  try {
    check_geodesic();
    check_clear_line_is_left_alone();
    check_separated_rooms();
    check_route_stays_inside();
    check_noisy_boundary();
    check_empty_region();
    std::cout << "routing_tests: shortest paths around obstacles\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "routing_tests: " << error.what() << '\n';
    return 1;
  }
}
