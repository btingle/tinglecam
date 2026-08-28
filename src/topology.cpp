#include "topology.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace wave {
namespace {

using namespace Clipper2Lib;

bool point_in_nonzero_paths(const Point64& point, const Paths64& paths) {
  int winding = 0;
  for (const auto& path : paths) {
    const auto relation = PointInPolygon(point, path);
    if (relation == PointInPolygonResult::IsOn) return true;
    if (relation == PointInPolygonResult::IsInside)
      winding += Area(path) >= 0.0 ? 1 : -1;
  }
  return winding != 0;
}

bool point_on_segment(const Point64& point, const Point64& a, const Point64& b) {
  if (!IsCollinear(a, point, b)) return false;
  return point.x >= std::min(a.x, b.x) && point.x <= std::max(a.x, b.x) &&
         point.y >= std::min(a.y, b.y) && point.y <= std::max(a.y, b.y);
}

bool segment_on_paths(const Point64& a, const Point64& b, const Paths64& paths) {
  for (const auto& path : paths) {
    for (std::size_t i = 0; i < path.size(); ++i) {
      const Point64& edge_a = path[i];
      const Point64& edge_b = path[(i + 1) % path.size()];
      if (point_on_segment(a, edge_a, edge_b) &&
          point_on_segment(b, edge_a, edge_b))
        return true;
    }
  }
  return false;
}

std::pair<Point64, Point64> normal_probes(const Point64& a, const Point64& b,
                                         std::int64_t distance) {
  const long double dx = static_cast<long double>(b.x - a.x);
  const long double dy = static_cast<long double>(b.y - a.y);
  const long double length = std::hypotl(dx, dy);
  const long double midpoint_x = (static_cast<long double>(a.x) + b.x) * 0.5L;
  const long double midpoint_y = (static_cast<long double>(a.y) + b.y) * 0.5L;
  if (length == 0.0L) {
    const Point64 midpoint(static_cast<std::int64_t>(std::llround(midpoint_x)),
                           static_cast<std::int64_t>(std::llround(midpoint_y)));
    return {midpoint, midpoint};
  }
  const long double offset_x = -dy / length * distance;
  const long double offset_y = dx / length * distance;
  return {
      Point64(static_cast<std::int64_t>(std::llround(midpoint_x + offset_x)),
              static_cast<std::int64_t>(std::llround(midpoint_y + offset_y))),
      Point64(static_cast<std::int64_t>(std::llround(midpoint_x - offset_x)),
              static_cast<std::int64_t>(std::llround(midpoint_y - offset_y))),
  };
}

// Reclassifies each maximal run of non-leading segments that is flanked by
// leading segments and is shorter than the noise floor. Returns how many
// segments were reclassified.
std::size_t absorb_quantization_gaps(std::vector<bool>& leading,
                                     const std::vector<double>& segment_length,
                                     double noise_floor) {
  const std::size_t count = leading.size();
  std::vector<std::size_t> absorbed;
  for (std::size_t i = 0; i < count; ++i) {
    // Starting only where a gap follows a leading segment visits each maximal
    // gap run once and guarantees it is flanked on the left.
    if (leading[i] || !leading[(i + count - 1) % count]) continue;
    double total = 0.0;
    std::size_t cursor = i;
    std::size_t run_length = 0;
    while (run_length < count && !leading[cursor]) {
      total += segment_length[cursor];
      cursor = (cursor + 1) % count;
      ++run_length;
    }
    if (run_length >= count || total > noise_floor) continue;
    for (std::size_t k = 0; k < run_length; ++k) absorbed.push_back((i + k) % count);
  }
  // Applied after the scan so reclassification cannot merge two distinct runs.
  for (const auto index : absorbed) leading[index] = true;
  return absorbed.size();
}

std::vector<LeadingFront> extract_leading_fronts(const Paths64& wave_band,
                                                 const Paths64& unreached,
                                                 const Paths64& static_boundaries) {
  std::vector<LeadingFront> fronts;
  constexpr std::int64_t probe_distance = 2;
  // Offset output is quantized to the integer grid, so a wavefront can carry
  // single-unit staircase steps whose normal points along the front rather than
  // across it. Those steps fail the probe test and would otherwise cut one
  // wavefront into two that share an endpoint. Measured over the sample meshes,
  // such artifacts reach 8 integer units while the shortest genuine gap between
  // fronts is 878, so this floor sits with wide margin between the two.
  constexpr double gap_noise_floor = 32.0;

  for (const auto& ring : wave_band) {
    const std::size_t count = ring.size();
    if (count < 2) continue;
    std::vector<bool> leading(count, false);
    std::vector<double> segment_length(count, 0.0);
    std::size_t leading_count = 0;
    for (std::size_t i = 0; i < count; ++i) {
      const Point64& a = ring[i];
      const Point64& b = ring[(i + 1) % count];
      segment_length[i] = std::hypot(static_cast<double>(b.x - a.x),
                                     static_cast<double>(b.y - a.y));
      if (a == b || segment_on_paths(a, b, static_boundaries)) continue;
      const auto [plus, minus] = normal_probes(a, b, probe_distance);
      if (point_in_nonzero_paths(plus, unreached) ||
          point_in_nonzero_paths(minus, unreached)) {
        leading[i] = true;
        ++leading_count;
      }
    }

    if (leading_count == 0) continue;
    // Runs the closed-ring check afterwards so a ring whose only interruption
    // was quantization noise stays a closed front instead of becoming an open
    // curve that appears to start and end at the same point.
    leading_count +=
        absorb_quantization_gaps(leading, segment_length, gap_noise_floor);

    if (leading_count == count) {
      fronts.push_back({ring, true, {}, {}});
      continue;
    }

    std::size_t first_gap = 0;
    while (first_gap < count && leading[first_gap]) ++first_gap;
    std::size_t cursor = (first_gap + 1) % count;
    std::size_t visited = 0;
    while (visited < count) {
      if (!leading[cursor]) {
        cursor = (cursor + 1) % count;
        ++visited;
        continue;
      }
      Path64 path;
      path.push_back(ring[cursor]);
      while (visited < count && leading[cursor]) {
        path.push_back(ring[(cursor + 1) % count]);
        cursor = (cursor + 1) % count;
        ++visited;
      }
      if (path.size() >= 2) fronts.push_back({std::move(path), false, {}, {}});
    }
  }
  return fronts;
}

Paths64 front_reach(const LeadingFront& front, double distance,
                    double softening) {
  if (front.path.size() < 2) return {};
  const EndType end_type = front.closed ? EndType::Joined : EndType::Square;
  if (softening <= 0.0)
    return InflatePaths(Paths64{front.path}, distance, JoinType::Miter, end_type);
  if (softening >= distance)
    return InflatePaths(Paths64{front.path}, distance, JoinType::Round, end_type);
  Paths64 result = InflatePaths(Paths64{front.path}, distance - softening,
                                JoinType::Miter, end_type);
  return InflatePaths(result, softening, JoinType::Round, EndType::Polygon);
}

double open_intersection_length(const LeadingFront& front, const Paths64& clips) {
  if (front.path.size() < 2 || clips.empty()) return 0.0;
  Path64 open_path = front.path;
  if (front.closed) open_path.push_back(front.path.front());
  Clipper64 clipper;
  clipper.AddOpenSubject(Paths64{open_path});
  clipper.AddClip(clips);
  Paths64 closed_solution;
  Paths64 open_solution;
  if (!clipper.Execute(ClipType::Intersection, FillRule::NonZero,
                       closed_solution, open_solution))
    return 0.0;
  double length = 0.0;
  for (const auto& path : open_solution) {
    for (std::size_t i = 1; i < path.size(); ++i) {
      const long double dx = static_cast<long double>(path[i].x - path[i - 1].x);
      const long double dy = static_cast<long double>(path[i].y - path[i - 1].y);
      length += static_cast<double>(std::hypotl(dx, dy));
    }
  }
  return length;
}

Point64 average_points(const std::vector<Point64>& points) {
  if (points.empty()) return Point64();
  long double x = 0.0L;
  long double y = 0.0L;
  for (const auto& point : points) {
    x += point.x;
    y += point.y;
  }
  return Point64(static_cast<std::int64_t>(std::llround(x / points.size())),
                 static_cast<std::int64_t>(std::llround(y / points.size())));
}

}  // namespace

FrontLineage build_front_lineage(const WaveResult& wave) {
  if (!(wave.engagement > 0.0)) throw std::invalid_argument("Engagement must be positive");
  if (wave.softening < 0.0 || wave.softening > wave.engagement)
    throw std::invalid_argument("Softening must be between zero and engagement");
  if (wave.generations.size() != wave.reached_generations.size())
    throw std::invalid_argument("Wave bands and cumulative reached sets do not match");
  FrontLineage result;
  if (wave.generations.empty()) return result;

  const Paths64 stock = wave.stock.empty() ? Paths64{wave.stock_boundary} : wave.stock;
  const Paths64 free_space = Difference(stock, wave.subject, FillRule::NonZero);
  Paths64 static_boundaries = wave.subject;
  static_boundaries.insert(static_boundaries.end(), stock.begin(), stock.end());
  result.generations.reserve(wave.generations.size());

  for (std::size_t generation = 0; generation < wave.generations.size(); ++generation) {
    Paths64 unreached = Difference(free_space, wave.reached_generations[generation],
                                   FillRule::NonZero);
    unreached = Union(unreached, FillRule::NonZero);
    result.generations.push_back({extract_leading_fronts(
        wave.generations[generation], unreached, static_boundaries)});
  }

  const double engagement_integer = static_cast<double>(
      std::max<std::int64_t>(1, to_integer(wave.engagement)));
  const double softening_integer = static_cast<double>(
      std::clamp<std::int64_t>(to_integer(wave.softening), 0,
                               static_cast<std::int64_t>(engagement_integer)));
  const double tolerance = std::clamp(engagement_integer * 0.001, 4.0, 100.0);

  for (std::size_t generation = 1; generation < result.generations.size(); ++generation) {
    auto& previous = result.generations[generation - 1].fronts;
    auto& current = result.generations[generation].fronts;
    Paths64 band_guard = InflatePaths(wave.generations[generation], tolerance,
                                      JoinType::Miter, EndType::Polygon);
    for (std::size_t parent = 0; parent < previous.size(); ++parent) {
      Paths64 candidate = front_reach(previous[parent], engagement_integer + tolerance,
                                      softening_integer);
      candidate = Intersect(candidate, band_guard, FillRule::NonZero);
      if (candidate.empty()) continue;
      for (std::size_t child = 0; child < current.size(); ++child) {
        if (open_intersection_length(current[child], candidate) <= tolerance) continue;
        previous[parent].children.push_back(child);
        current[child].parents.push_back(parent);
      }
    }

    for (std::size_t parent = 0; parent < previous.size(); ++parent) {
      const auto& children = previous[parent].children;
      if (children.size() > 1) {
        std::vector<Point64> points;
        points.push_back(representative_point(previous[parent]));
        for (const auto child : children)
          points.push_back(representative_point(current[child]));
        result.events.push_back({LineageEventKind::Split, generation,
                                 {parent}, children, average_points(points)});
      } else if (children.empty()) {
        result.events.push_back({LineageEventKind::Termination, generation,
                                 {parent}, {}, representative_point(previous[parent])});
      }
    }
    for (std::size_t child = 0; child < current.size(); ++child) {
      const auto& parents = current[child].parents;
      if (parents.size() > 1) {
        std::vector<Point64> points;
        points.push_back(representative_point(current[child]));
        for (const auto parent : parents)
          points.push_back(representative_point(previous[parent]));
        result.events.push_back({LineageEventKind::Merge, generation,
                                 parents, {child}, average_points(points)});
      } else if (parents.empty()) {
        result.events.push_back({LineageEventKind::Birth, generation,
                                 {}, {child}, representative_point(current[child])});
      }
    }
  }
  return result;
}

Point64 representative_point(const LeadingFront& front) {
  if (front.path.empty()) return Point64();
  if (front.path.size() == 1) return front.path.front();
  long double total_length = 0.0L;
  const std::size_t segment_count = front.closed ? front.path.size() : front.path.size() - 1;
  for (std::size_t i = 0; i < segment_count; ++i) {
    const auto& a = front.path[i];
    const auto& b = front.path[(i + 1) % front.path.size()];
    total_length += std::hypotl(static_cast<long double>(b.x - a.x),
                               static_cast<long double>(b.y - a.y));
  }
  long double target = total_length * 0.5L;
  for (std::size_t i = 0; i < segment_count; ++i) {
    const auto& a = front.path[i];
    const auto& b = front.path[(i + 1) % front.path.size()];
    const long double length = std::hypotl(static_cast<long double>(b.x - a.x),
                                          static_cast<long double>(b.y - a.y));
    if (target <= length && length > 0.0L) {
      const long double t = target / length;
      return Point64(static_cast<std::int64_t>(std::llround(a.x + (b.x - a.x) * t)),
                     static_cast<std::int64_t>(std::llround(a.y + (b.y - a.y) * t)));
    }
    target -= length;
  }
  return front.path.back();
}

double front_length(const LeadingFront& front) {
  double length = 0.0;
  if (front.path.size() < 2) return length;
  const std::size_t segment_count = front.closed ? front.path.size() : front.path.size() - 1;
  for (std::size_t i = 0; i < segment_count; ++i) {
    const auto& a = front.path[i];
    const auto& b = front.path[(i + 1) % front.path.size()];
    length += std::hypot(to_world(b.x - a.x), to_world(b.y - a.y));
  }
  return length;
}

const char* lineage_event_kind_name(LineageEventKind kind) {
  switch (kind) {
    case LineageEventKind::Split: return "split";
    case LineageEventKind::Merge: return "merge";
    case LineageEventKind::Termination: return "termination";
    case LineageEventKind::Birth: return "birth (unexpected)";
  }
  return "unknown";
}

}  // namespace wave
