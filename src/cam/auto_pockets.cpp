#include "cam/auto_pockets.h"

#include <clipper2/clipper.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace cam {
namespace {

using namespace Clipper2Lib;

struct Candidate {
  Path64 boundary;
  Point64 centre;
};

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

bool positive(double value) { return std::isfinite(value) && value > 0.0; }

std::vector<Candidate> holes_at(const wave::Paths64& region) {
  if (region.empty()) return {};
  Clipper64 clipper;
  clipper.AddSubject(region);
  PolyTree64 tree;
  Paths64 open;
  if (!clipper.Execute(ClipType::Union, FillRule::NonZero, tree, open))
    return {};

  std::vector<Candidate> result;
  for (std::size_t outer_index = 0; outer_index < tree.Count(); ++outer_index) {
    const PolyPath64* outer = tree.Child(outer_index);
    // A child of a material component is a hole. Nested children are islands,
    // not the simple pocket shape this pass promises to handle.
    for (std::size_t hole_index = 0; hole_index < outer->Count(); ++hole_index) {
      const PolyPath64* hole = outer->Child(hole_index);
      if (hole->Polygon().size() < 3 || hole->Count() != 0) continue;
      result.push_back(Candidate{hole->Polygon(), Point64(0, 0)});
    }
  }
  return result;
}

Point64 centroid(const Path64& path) {
  long double twice_area = 0.0;
  long double x = 0.0;
  long double y = 0.0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const Point64& a = path[i];
    const Point64& b = path[(i + 1) % path.size()];
    const long double cross = static_cast<long double>(a.x) * b.y -
                              static_cast<long double>(b.x) * a.y;
    twice_area += cross;
    x += (static_cast<long double>(a.x) + b.x) * cross;
    y += (static_cast<long double>(a.y) + b.y) * cross;
  }
  if (std::abs(twice_area) < 1e-12L) {
    long double average_x = 0.0;
    long double average_y = 0.0;
    for (const Point64& point : path) {
      average_x += point.x;
      average_y += point.y;
    }
    return {static_cast<int64_t>(std::llround(average_x / path.size())),
            static_cast<int64_t>(std::llround(average_y / path.size()))};
  }
  return {static_cast<int64_t>(std::llround(x / (3.0L * twice_area))),
          static_cast<int64_t>(std::llround(y / (3.0L * twice_area)))};
}

bool convex(const Path64& path) {
  if (path.size() < 3) return false;
  const int winding = Area(path) >= 0.0 ? 1 : -1;
  long double largest_turn = 0.0;
  std::vector<long double> turns;
  turns.reserve(path.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    const Point64& a = path[i];
    const Point64& b = path[(i + 1) % path.size()];
    const Point64& c = path[(i + 2) % path.size()];
    const long double cross =
        static_cast<long double>(b.x - a.x) * (c.y - b.y) -
        static_cast<long double>(b.y - a.y) * (c.x - b.x);
    turns.push_back(cross);
    largest_turn = std::max(largest_turn, std::abs(cross));
  }
  if (!(largest_turn > 0.0)) return false;
  // A downward-envelope slice can leave one tiny reverse turn where adjacent
  // mesh facets meet. Allow that quantization noise, but retain a strict
  // relative limit so genuinely concave contours are still rejected.
  constexpr long double concavity_tolerance = 0.01L;
  for (const long double turn : turns) {
    if (std::abs(turn) < 0.5L) continue;
    const int sign = turn > 0 ? 1 : -1;
    if (sign != winding && std::abs(turn) > largest_turn * concavity_tolerance)
      return false;
  }
  return true;
}

Rect64 bounds(const Path64& path) {
  Rect64 result{path.front().x, path.front().y,
                path.front().x, path.front().y};
  for (const Point64& point : path) {
    result.left = std::min(result.left, point.x);
    result.right = std::max(result.right, point.x);
    result.top = std::min(result.top, point.y);
    result.bottom = std::max(result.bottom, point.y);
  }
  return result;
}

Path64 exclusion_rectangle(const PocketExclusionArea& exclusion) {
  const auto integer = [](double value) {
    return wave::to_integer(value);
  };
  const double left = exclusion.center_x - exclusion.width * 0.5;
  const double right = exclusion.center_x + exclusion.width * 0.5;
  const double bottom = exclusion.center_y - exclusion.height * 0.5;
  const double top = exclusion.center_y + exclusion.height * 0.5;
  return {{integer(left), integer(bottom)}, {integer(right), integer(bottom)},
          {integer(right), integer(top)}, {integer(left), integer(top)}};
}

bool excluded(const Path64& candidate,
              const std::vector<PocketExclusionArea>& exclusions) {
  for (const PocketExclusionArea& exclusion : exclusions) {
    if (!exclusion.enabled) continue;
    const Paths64 overlap = Intersect(Paths64{candidate},
                                      Paths64{exclusion_rectangle(exclusion)},
                                      FillRule::NonZero);
    if (std::abs(Area(overlap)) > 0.0) return true;
  }
  return false;
}

double area_millimetres(const Paths64& path) {
  return std::abs(Area(path)) /
         (wave::kIntegerScale * wave::kIntegerScale);
}

const Candidate* matching_hole(const Candidate& original,
                               const std::vector<Candidate>& lower) {
  const double original_area = area_millimetres(Paths64{original.boundary});
  if (!(original_area > 0.0)) return nullptr;
  const Candidate* match = nullptr;
  double best_difference = std::numeric_limits<double>::infinity();
  for (const Candidate& candidate : lower) {
    if (PointInPolygon(original.centre, candidate.boundary) ==
        PointInPolygonResult::IsOutside)
      continue;
    const double intersection = area_millimetres(
        Intersect(Paths64{original.boundary}, Paths64{candidate.boundary},
                  FillRule::NonZero));
    if (intersection <= 0.0) continue;
    const double union_area = area_millimetres(
        Union(Paths64{original.boundary}, Paths64{candidate.boundary},
              FillRule::NonZero));
    if (!(union_area > 0.0)) continue;
    const double difference = 1.0 - intersection / union_area;
    if (difference < best_difference) {
      best_difference = difference;
      match = &candidate;
    }
  }
  // This tolerance is deliberately fairly tight. A pocket whose walls change
  // topology or appreciably change footprint is left for a future strategy.
  return best_difference <= 0.03 ? match : nullptr;
}

}  // namespace

std::vector<OperationPlan::IdentifiedPocket> identify_pockets(
    const JobGeometry& geometry, const Operation& operation) {
  require(geometry.has_part() && geometry.has_stock(),
          "Auto pocket identification requires part and stock geometry");
  const auto& part_bounds = geometry.part().mesh().bounds;
  require(part_bounds.valid, "Part mesh has no valid bounds");
  require(std::isfinite(operation.pocket_auto.check_depth) &&
              operation.pocket_auto.check_depth > 0.0,
          "Pocket inspection depth must be finite and positive");
  require(operation.pocket_auto.check_depth <
              static_cast<double>(part_bounds.upper.z - part_bounds.lower.z),
          "Pocket inspection depth must be inside the part's Z range");
  for (const auto& exclusion : operation.pocket_auto.exclusions) {
    if (!exclusion.enabled) continue;
    require(std::isfinite(exclusion.center_x) &&
                std::isfinite(exclusion.center_y) && positive(exclusion.width) &&
                positive(exclusion.height),
            "Pocket exclusion area must have finite position and positive size");
  }
  const double top_z = part_bounds.upper.z;
  const double check_z = top_z - operation.pocket_auto.check_depth;
  const auto& inspected = geometry.part_outline(check_z, operation.slice_grid);
  std::vector<Candidate> candidates = holes_at(inspected);
  const double stock_bottom = geometry.stock_bottom();
  require(std::isfinite(stock_bottom), "Stock has no valid lower bound");

  // Require the opening to be present at the top as well. This prevents a
  // buried cavity from becoming an unsafe plunge target merely because its
  // lower envelope happens to contain a convex hole.
  const double epsilon = 1e-4;
  const auto top_candidates = holes_at(
      geometry.part_outline(top_z - epsilon, operation.slice_grid));
  std::vector<Candidate> top_openings;
  for (Candidate& candidate : candidates) {
    candidate.centre = centroid(candidate.boundary);
    if (matching_hole(candidate, top_candidates)) top_openings.push_back(candidate);
  }
  candidates = std::move(top_openings);

  std::vector<wave::Vec3f> origins;
  origins.reserve(candidates.size() * 9);
  for (Candidate& candidate : candidates) {
    if (!convex(candidate.boundary) || excluded(candidate.boundary,
                                                 operation.pocket_auto.exclusions))
      continue;
    candidate.centre = centroid(candidate.boundary);
    origins.push_back({static_cast<float>(wave::to_world(candidate.centre.x)),
                       static_cast<float>(wave::to_world(candidate.centre.y)),
                       static_cast<float>(check_z + epsilon)});
    // These points remain strictly inside a convex contour and make the
    // maximum-depth estimate robust to a sloping or uneven pocket floor.
    const std::size_t samples = std::min<std::size_t>(candidate.boundary.size(), 8);
    for (std::size_t i = 0; i < samples; ++i) {
      const Point64& vertex = candidate.boundary[i];
      origins.push_back({static_cast<float>(
                             wave::to_world(candidate.centre.x * 3 / 4 + vertex.x / 4)),
                         static_cast<float>(
                             wave::to_world(candidate.centre.y * 3 / 4 + vertex.y / 4)),
                         static_cast<float>(check_z + epsilon)});
    }
  }
  double ray_milliseconds = 0.0;
  const std::vector<float> hits = geometry.part().vertical_raycast(
      origins, static_cast<float>(stock_bottom), ray_milliseconds);

  std::vector<OperationPlan::IdentifiedPocket> result;
  std::size_t origin_index = 0;
  for (const Candidate& candidate : candidates) {
    if (!convex(candidate.boundary) || excluded(candidate.boundary,
                                                operation.pocket_auto.exclusions))
      continue;
    double bottom_z = std::numeric_limits<double>::infinity();
    const std::size_t samples = 1 +
        std::min<std::size_t>(candidate.boundary.size(), 8);
    for (std::size_t sample = 0; sample < samples; ++sample) {
      const float hit = hits[origin_index++];
      bottom_z = std::min(bottom_z,
                          std::isfinite(hit) ? double(hit) : stock_bottom);
    }
    if (!(bottom_z < check_z - epsilon)) continue;

    // A through-hole has no mesh floor, so compare it at the lowest slice of
    // the part. A blind pocket is compared just above its deepest ray hit.
    const double test_z = std::isfinite(bottom_z) && bottom_z > part_bounds.lower.z
                              ? bottom_z + epsilon
                              : part_bounds.lower.z + epsilon;
    if (!(test_z < check_z)) continue;
    const auto lower_candidates = holes_at(
        geometry.part_outline(test_z, operation.slice_grid));
    if (!matching_hole(candidate, lower_candidates)) continue;

    const Rect64 extent = bounds(candidate.boundary);
    OperationPlan::IdentifiedPocket pocket;
    pocket.boundary = {candidate.boundary};
    pocket.center_x = wave::to_world(candidate.centre.x);
    pocket.center_y = wave::to_world(candidate.centre.y);
    pocket.width = wave::to_world(extent.right - extent.left);
    pocket.height = wave::to_world(extent.bottom - extent.top);
    pocket.top_z = top_z;
    pocket.bottom_z = bottom_z;
    result.push_back(std::move(pocket));
  }
  return result;
}

}  // namespace cam
