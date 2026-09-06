#include "cam/contours.h"

#include <clipper2/clipper.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace cam {
namespace {

using namespace Clipper2Lib;

wave::Path64 closed_variant(const wave::Path64& ring, std::size_t start,
                            bool reverse) {
  wave::Path64 path;
  path.reserve(ring.size() + 1);
  for (std::size_t step = 0; step < ring.size(); ++step) {
    const std::size_t index = reverse
                                  ? (start + ring.size() - step) % ring.size()
                                  : (start + step) % ring.size();
    path.push_back(ring[index]);
  }
  path.push_back(path.front());
  return path;
}

bool permit_forward(ContourDirectionPolicy policy) {
  return policy != ContourDirectionPolicy::Conventional;
}

bool permit_reverse(ContourDirectionPolicy policy) {
  return policy != ContourDirectionPolicy::Climb;
}

double distance_integer(const wave::Point64& a, const wave::Point64& b) {
  return std::hypot(static_cast<double>(b.x - a.x),
                    static_cast<double>(b.y - a.y));
}

wave::Point64 project_segment(const wave::Point64& point,
                              const wave::Point64& a,
                              const wave::Point64& b) {
  const long double dx = static_cast<long double>(b.x - a.x);
  const long double dy = static_cast<long double>(b.y - a.y);
  const long double length_squared = dx * dx + dy * dy;
  long double t = 0.0L;
  if (length_squared > 0.0L) {
    t = ((static_cast<long double>(point.x - a.x) * dx) +
         (static_cast<long double>(point.y - a.y) * dy)) /
        length_squared;
    t = std::clamp(t, 0.0L, 1.0L);
  }
  return wave::Point64(
      static_cast<std::int64_t>(std::llround(a.x + dx * t)),
      static_cast<std::int64_t>(std::llround(a.y + dy * t)));
}

bool segment_avoids_region(const wave::Point64& from,
                           const wave::Point64& to,
                           const wave::Paths64& forbidden) {
  if (forbidden.empty()) return true;
  Clipper64 clipper;
  clipper.AddOpenSubject(wave::Paths64{wave::Path64{from, to}});
  clipper.AddClip(forbidden);
  wave::Paths64 closed;
  wave::Paths64 open;
  if (!clipper.Execute(ClipType::Intersection, FillRule::NonZero, closed, open))
    return false;
  double intersection = 0.0;
  for (const auto& piece : open)
    for (std::size_t i = 1; i < piece.size(); ++i)
      intersection += distance_integer(piece[i - 1], piece[i]);
  // Touching the finishing contour at its endpoint is legal; travelling along
  // or through the protected region is not.
  return intersection <= 1.0;
}

}  // namespace

wave::Paths64 stock_exterior_tolerance(const wave::Paths64& stock,
                                       double tolerance) {
  if (tolerance < 0.0 || !std::isfinite(tolerance))
    throw std::invalid_argument("Stock tolerance must be finite and nonnegative");
  const wave::Paths64 material = Union(stock, FillRule::NonZero);
  const std::int64_t distance = wave::to_integer(tolerance);
  if (material.empty() || distance == 0) return {};
  const wave::Paths64 contact = Union(
      InflatePaths(material, static_cast<double>(distance), JoinType::Round,
                   EndType::Polygon),
      FillRule::NonZero);
  return Difference(contact, material, FillRule::NonZero);
}

std::vector<DerivedContour> derive_offset_contours(
    const wave::Paths64& sliced_boundaries, double outward_offset,
    const wave::Paths64* stock_clip) {
  if (sliced_boundaries.empty()) return {};
  wave::Paths64 polygons = Union(sliced_boundaries, FillRule::NonZero);
  const std::int64_t delta = wave::to_integer(outward_offset);
  if (delta != 0)
    polygons = Union(InflatePaths(polygons, static_cast<double>(delta),
                                  JoinType::Round, EndType::Polygon),
                     FillRule::NonZero);
  std::vector<DerivedContour> result;
  for (const wave::Path64& ring : polygons) {
    if (ring.size() < 2) continue;
    if (!stock_clip) {
      result.push_back({ring, true});
      continue;
    }
    wave::Path64 open = ring;
    open.push_back(ring.front());
    Clipper64 clipper;
    clipper.AddOpenSubject(wave::Paths64{open});
    clipper.AddClip(*stock_clip);
    wave::Paths64 closed_solution;
    wave::Paths64 open_solution;
    if (!clipper.Execute(ClipType::Intersection, FillRule::NonZero,
                         closed_solution, open_solution))
      throw std::runtime_error("Contour clipping failed");
    for (wave::Path64& piece : open_solution) {
      if (piece.size() < 2) continue;
      const bool closed = piece.front() == piece.back();
      if (closed) piece.pop_back();
      if (piece.size() >= 2) result.push_back({std::move(piece), closed});
    }
  }
  return result;
}

std::vector<wave::TraversalTask> contour_traversal_tasks(
    const std::vector<DerivedContour>& contours,
    ContourDirectionPolicy direction,
    std::size_t maximum_closed_starts) {
  maximum_closed_starts = std::max<std::size_t>(1, maximum_closed_starts);
  std::vector<wave::TraversalTask> tasks;
  tasks.reserve(contours.size());
  for (std::size_t index = 0; index < contours.size(); ++index) {
    const DerivedContour& contour = contours[index];
    if (contour.path.size() < 2) continue;
    wave::TraversalTask task;
    task.source = index;
    if (!contour.closed) {
      if (permit_forward(direction))
        task.variants.push_back({contour.path, std::nullopt});
      if (permit_reverse(direction)) {
        wave::Path64 reverse = contour.path;
        std::reverse(reverse.begin(), reverse.end());
        task.variants.push_back({std::move(reverse), std::nullopt});
      }
    } else {
      const std::size_t starts = std::min(maximum_closed_starts, contour.path.size());
      for (std::size_t sample = 0; sample < starts; ++sample) {
        const std::size_t start = sample * contour.path.size() / starts;
        if (permit_forward(direction))
          task.variants.push_back(
              {closed_variant(contour.path, start, false), std::nullopt});
        if (permit_reverse(direction))
          task.variants.push_back(
              {closed_variant(contour.path, start, true), std::nullopt});
      }
    }
    if (!task.variants.empty()) tasks.push_back(std::move(task));
  }
  return tasks;
}

void add_safe_stock_approaches(
    std::vector<wave::TraversalTask>& tasks, const wave::Paths64& stock,
    const wave::Paths64& protected_centres, double cutter_radius,
    double extra_clearance) {
  if (stock.empty()) throw std::invalid_argument("Stock approach region is empty");
  if (!(cutter_radius >= 0.0) || !(extra_clearance >= 0.0))
    throw std::invalid_argument("Stock approach clearance must not be negative");
  const double clearance = static_cast<double>(std::max<std::int64_t>(
      1, wave::to_integer(cutter_radius + extra_clearance)));
  const wave::Paths64 outside_boundary = InflatePaths(
      Union(stock, FillRule::NonZero), clearance, JoinType::Round,
      EndType::Polygon);
  for (wave::TraversalTask& task : tasks) {
    std::vector<wave::TraversalVariant> legal;
    legal.reserve(task.variants.size());
    for (wave::TraversalVariant variant : task.variants) {
      if (variant.path.empty()) continue;
      const wave::Point64 start = variant.path.front();
      struct Candidate {
        wave::Point64 point;
        double distance = 0.0;
      };
      std::vector<Candidate> candidates;
      for (const wave::Path64& ring : outside_boundary) {
        candidates.reserve(candidates.size() + ring.size());
        for (std::size_t i = 0; i < ring.size(); ++i) {
          const wave::Point64 candidate =
              project_segment(start, ring[i], ring[(i + 1) % ring.size()]);
          candidates.push_back(
              {candidate, distance_integer(candidate, start)});
        }
      }
      // Exact intersection is the expensive part. Distance is a lower-cost
      // complete ordering, so the first legal candidate in this stable order
      // is the same nearest legal candidate the exhaustive scan selected.
      std::stable_sort(candidates.begin(), candidates.end(),
                       [](const Candidate& a, const Candidate& b) {
                         return a.distance < b.distance;
                       });
      std::optional<wave::Point64> best;
      for (const Candidate& candidate : candidates) {
        if (!segment_avoids_region(candidate.point, start, protected_centres))
          continue;
        best = candidate.point;
        break;
      }
      if (!best) continue;
      variant.safe_approach = *best == start ? wave::Path64{start}
                                             : wave::Path64{*best, start};
      legal.push_back(std::move(variant));
    }
    if (legal.empty())
      throw std::runtime_error(
          "No straight cutter-clear approach exists for a finishing contour");
    task.variants = std::move(legal);
  }
}

const char* contour_direction_name(ContourDirectionPolicy value) {
  switch (value) {
    case ContourDirectionPolicy::Either: return "either";
    case ContourDirectionPolicy::Climb: return "climb";
    case ContourDirectionPolicy::Conventional: return "conventional";
  }
  return "unknown";
}

}  // namespace cam
