#include "topology_detail.h"

#include <clipper2/clipper.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace wave::detail {
using namespace Clipper2Lib;

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

// Splits a wave band into connected pieces. Clipper hands back outers and holes
// as a flat list, which loses which hole belongs to which outer -- and a hole
// treated as a standalone ring describes the island it surrounds, not band
// material.
void collect_band_components(const PolyPath64& node, std::vector<Paths64>& out) {
  for (std::size_t i = 0; i < node.Count(); ++i) {
    const PolyPath64* outer = node.Child(i);
    Paths64 component;
    component.push_back(outer->Polygon());
    for (std::size_t j = 0; j < outer->Count(); ++j) {
      const PolyPath64* hole = outer->Child(j);
      component.push_back(hole->Polygon());
      // An island sitting inside the hole starts a component of its own.
      collect_band_components(*hole, out);
    }
    out.push_back(std::move(component));
  }
}

std::vector<Paths64> band_components(const Paths64& wave_band) {
  Clipper64 clipper;
  clipper.AddSubject(wave_band);
  PolyTree64 tree;
  Paths64 open_solution;
  if (!clipper.Execute(ClipType::Union, FillRule::NonZero, tree, open_solution))
    return {};
  std::vector<Paths64> components;
  collect_band_components(tree, components);
  return components;
}

constexpr std::int64_t probe_distance = 2;
// Offset output is quantized to the integer grid, so a wavefront can carry
// single-unit staircase steps whose normal points along the front rather than
// across it. Those steps fail the probe test and would otherwise cut one
// wavefront into two that share an endpoint. Measured over the sample meshes,
// such artifacts reach 8 integer units while the shortest genuine gap between
// fronts is 878, so this floor sits with wide margin between the two.
constexpr double gap_noise_floor = 32.0;

// Splits every edge of a ring so that none is longer than `max_segment`,
// leaving the ring's own vertices in place.
//
// The leading test below takes one probe pair per edge, so an edge is only
// ever wholly leading or wholly not. That resolution is inherited from the
// offsetter, which emits a straight wall as one edge however long it is: a
// hundred-millimetre wall fronting unreached material along most of its length
// is thrown away entire whenever its midpoint happens to sit against the part.
// The generation after it still advances through the stretches that were open,
// and the fronts that come out of them are left with no parent, because the
// parent they grew from was never extracted. A parentless front is ready from
// the outset, so the toolpath takes it in proximity order with nothing having
// cleared a way in, and has nothing to enter it with but a plunge.
//
// Densifying first gives the classification a resolution that does not depend
// on how the offset happened to be vertexed. Probe points end up at most one
// pitch apart, so no leading or blocked run longer than a pitch can fall
// between them.
Path64 densify_ring(const Path64& ring, double max_segment) {
  if (ring.size() < 2 || !(max_segment > 0.0)) return ring;
  Path64 result;
  result.reserve(ring.size());
  for (std::size_t i = 0; i < ring.size(); ++i) {
    const Point64& a = ring[i];
    const Point64& b = ring[(i + 1) % ring.size()];
    result.push_back(a);
    const double dx = static_cast<double>(b.x - a.x);
    const double dy = static_cast<double>(b.y - a.y);
    const double length = std::hypot(dx, dy);
    if (!(length > max_segment)) continue;
    const auto pieces =
        static_cast<std::size_t>(std::ceil(length / max_segment));
    for (std::size_t piece = 1; piece < pieces; ++piece) {
      const double ratio = static_cast<double>(piece) / pieces;
      const Point64 point(
          static_cast<std::int64_t>(std::llround(a.x + dx * ratio)),
          static_cast<std::int64_t>(std::llround(a.y + dy * ratio)));
      // Rounding can put a piece boundary back on the point before it. Keeping
      // the duplicate would only add a zero-length edge the classifier skips.
      if (point != result.back()) result.push_back(point);
    }
  }
  return result;
}

void extract_ring_fronts(const Path64& raw_ring, std::size_t band,
                         const Paths64& unreached,
                         const Paths64& static_boundaries,
                         double probe_pitch,
                         std::vector<LeadingFront>& fronts) {
  const Path64 ring = densify_ring(raw_ring, probe_pitch);
  const std::size_t count = ring.size();
  if (count < 2) return;
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

  if (leading_count == 0) return;
  // Runs the closed-ring check afterwards so a ring whose only interruption
  // was quantization noise stays a closed front instead of becoming an open
  // curve that appears to start and end at the same point.
  leading_count +=
      absorb_quantization_gaps(leading, segment_length, gap_noise_floor);

  if (leading_count == count) {
    LeadingFront front;
    front.path = ring;
    front.closed = true;
    front.band = band;
    fronts.push_back(std::move(front));
    return;
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
    if (path.size() >= 2) {
      LeadingFront front;
      front.path = std::move(path);
      front.band = band;
      fronts.push_back(std::move(front));
    }
  }
}

// How finely the leading test samples a ring. A front is responsible for a
// stepover of material, so a run shorter than a quarter of one bounds a child
// band well under the half-stepover-squared area that `reject_tiny_splits`
// already dismisses as noise: at this pitch nothing the lineage would keep can
// be missed. Blockages shorter than the pitch may go unsplit, which costs
// nothing -- the children on either side of one still lie within the single
// parent's reach and are matched to it. The floor keeps a piece longer than
// the quantization gaps `absorb_quantization_gaps` is there to close, so
// densifying can never manufacture a gap short enough to be absorbed.
double front_probe_pitch(double stepover) {
  return std::max(static_cast<double>(to_integer(stepover)) * 0.25,
                  gap_noise_floor);
}

std::vector<LeadingFront> extract_leading_fronts(const std::vector<Paths64>& bands,
                                                 const Paths64& unreached,
                                                 const Paths64& static_boundaries,
                                                 double probe_pitch) {
  std::vector<LeadingFront> fronts;
  for (std::size_t band = 0; band < bands.size(); ++band)
    for (const auto& ring : bands[band])
      extract_ring_fronts(ring, band, unreached, static_boundaries, probe_pitch,
                          fronts);
  return fronts;
}

std::vector<std::vector<double>> front_responsibility_areas(
    const FrontLineage& lineage) {
  std::vector<std::vector<double>> result(lineage.generations.size());
  for (std::size_t generation = 0; generation < lineage.generations.size();
       ++generation) {
    const FrontGeneration& item = lineage.generations[generation];
    result[generation].assign(item.fronts.size(), 0.0);
    std::vector<std::vector<std::size_t>> owners(item.bands.size());
    for (std::size_t front = 0; front < item.fronts.size(); ++front)
      if (item.fronts[front].band < owners.size())
        owners[item.fronts[front].band].push_back(front);

    for (std::size_t band = 0; band < item.bands.size(); ++band) {
      if (owners[band].empty()) continue;
      const double area = std::abs(Area(item.bands[band])) /
                          (kIntegerScale * kIntegerScale);
      double total_length = 0.0;
      for (const std::size_t front : owners[band])
        total_length += front_length(item.fronts[front]);
      if (total_length > 0.0) {
        for (const std::size_t front : owners[band])
          result[generation][front] +=
              area * front_length(item.fronts[front]) / total_length;
      } else {
        const double share = area / owners[band].size();
        for (const std::size_t front : owners[band])
          result[generation][front] += share;
      }
    }
  }
  return result;
}

// Returns infinity when the branch reaches another split or merge: a narrow
// opening into a later substantial region is real topology, not a tiny spur.
// Only an unambiguous 1:1 chain ending in termination can be dismissed.
double terminal_branch_area(
    const FrontLineage& lineage,
    const std::vector<std::vector<double>>& areas,
    std::size_t generation, std::size_t front) {
  double result = 0.0;
  while (generation < lineage.generations.size() &&
         front < lineage.generations[generation].fronts.size()) {
    const LeadingFront& item = lineage.generations[generation].fronts[front];
    if (item.parents.size() != 1 || generation >= areas.size() ||
        front >= areas[generation].size())
      return std::numeric_limits<double>::infinity();
    result += areas[generation][front];
    if (item.children.empty()) return result;
    if (item.children.size() != 1 ||
        generation + 1 >= lineage.generations.size())
      return std::numeric_limits<double>::infinity();
    const std::size_t child = item.children.front();
    if (child >= lineage.generations[generation + 1].fronts.size() ||
        lineage.generations[generation + 1].fronts[child].parents.size() != 1)
      return std::numeric_limits<double>::infinity();
    ++generation;
    front = child;
  }
  return std::numeric_limits<double>::infinity();
}

void reject_tiny_splits(FrontLineage& lineage, double area_factor) {
  if (!(area_factor > 0.0) || lineage.events.empty()) return;
  const double threshold = area_factor * lineage.stepover * lineage.stepover;
  const std::vector<std::vector<double>> areas =
      front_responsibility_areas(lineage);
  std::vector<LineageEvent> accepted;
  accepted.reserve(lineage.events.size());
  for (LineageEvent& event : lineage.events) {
    bool reject = false;
    if (event.kind == LineageEventKind::Split &&
        event.generation < lineage.generations.size() &&
        event.current_fronts.size() >= 2) {
      std::size_t substantial = 0;
      std::size_t tiny = 0;
      for (const std::size_t child : event.current_fronts) {
        const double area = terminal_branch_area(
            lineage, areas, event.generation, child);
        if (area < threshold)
          ++tiny;
        else
          ++substantial;
      }
      // Keep a real fork whenever two branches survive the local scale. With
      // only one substantial branch, the others are terminal ripples and the
      // parent remains one topological propagation node.
      reject = tiny > 0 && substantial <= 1;
    }
    if (reject)
      lineage.rejected_splits.push_back(std::move(event));
    else
      accepted.push_back(std::move(event));
  }
  lineage.events = std::move(accepted);
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

// Whether two pieces of the newly swept band are the same terminal contact.
// This is deliberately evaluated here, beside ordinary lineage matching: once
// the geometry has been reduced to a graph, a pair of fronts which consumed
// one another is otherwise indistinguishable from two unrelated terminations.
bool terminal_reaches_touch(const Paths64& lhs, const Paths64& rhs,
                            double tolerance) {
  if (lhs.empty() || rhs.empty()) return false;
  const Rect64 a = GetBounds(lhs);
  const Rect64 b = GetBounds(rhs);
  if (a.right + tolerance < b.left || b.right + tolerance < a.left ||
      a.bottom + tolerance < b.top || b.bottom + tolerance < a.top)
    return false;
  if (!Intersect(lhs, rhs, FillRule::NonZero).empty()) return true;
  const Paths64 guard = InflatePaths(lhs, tolerance, JoinType::Round,
                                     EndType::Polygon);
  return !Intersect(guard, rhs, FillRule::NonZero).empty();
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

struct SegmentProjection {
  Point64 point;
  long double distance = std::numeric_limits<long double>::max();
  long double along = 0.0L;
};

SegmentProjection project_to_segment(const Point64& point, const Point64& a,
                                     const Point64& b) {
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
  const long double x = static_cast<long double>(a.x) + dx * t;
  const long double y = static_cast<long double>(a.y) + dy * t;
  SegmentProjection result;
  result.point = Point64(static_cast<std::int64_t>(std::llround(x)),
                         static_cast<std::int64_t>(std::llround(y)));
  result.distance = std::hypotl(static_cast<long double>(point.x) - x,
                               static_cast<long double>(point.y) - y);
  result.along = std::sqrt(length_squared) * t;
  return result;
}

// How far a front endpoint may be from a static boundary ring and still be
// called a contact -- and, because such a contact is recorded as the projected
// point on the ring rather than the endpoint itself, equally how far anything
// built along those rings may be from the propagation bands.
//
BoundaryContact classify_boundary_contact(const Point64& endpoint,
                                          const Paths64& boundaries,
                                          double tolerance) {
  BoundaryContact best;
  long double best_distance = std::numeric_limits<long double>::max();
  for (std::size_t component = 0; component < boundaries.size(); ++component) {
    const Path64& ring = boundaries[component];
    if (ring.size() < 2) continue;
    long double walked = 0.0L;
    for (std::size_t i = 0; i < ring.size(); ++i) {
      const Point64& a = ring[i];
      const Point64& b = ring[(i + 1) % ring.size()];
      const SegmentProjection projection = project_to_segment(endpoint, a, b);
      if (projection.distance < best_distance) {
        best_distance = projection.distance;
        best.component = component;
        best.point = projection.point;
        best.arclength = to_world(static_cast<std::int64_t>(
            std::llround(walked + projection.along)));
        best.error = to_world(static_cast<std::int64_t>(
            std::llround(projection.distance)));
      }
      walked += std::hypotl(static_cast<long double>(b.x - a.x),
                            static_cast<long double>(b.y - a.y));
    }
  }
  best.valid = best.component != kNoTopologyId && best_distance <= tolerance;
  if (!best.valid) best.component = kNoTopologyId;
  return best;
}

void classify_front_contacts(std::vector<LeadingFront>& fronts,
                             const Paths64& boundaries, double tolerance) {
  for (LeadingFront& front : fronts) {
    if (front.closed || front.path.size() < 2) continue;
    front.boundary_contacts[0] =
        classify_boundary_contact(front.path.front(), boundaries, tolerance);
    front.boundary_contacts[1] =
        classify_boundary_contact(front.path.back(), boundaries, tolerance);
  }
}

double point_distance(const Point64& a, const Point64& b) {
  return std::hypot(static_cast<double>(b.x - a.x),
                    static_cast<double>(b.y - a.y));
}

double path_length_integer(const Path64& path, bool closed) {
  if (path.size() < 2) return 0.0;
  double length = 0.0;
  const std::size_t segments = closed ? path.size() : path.size() - 1;
  for (std::size_t i = 0; i < segments; ++i)
    length += point_distance(path[i], path[(i + 1) % path.size()]);
  return length;
}

bool whole_open_path_inside(const Path64& path, const Paths64& region) {
  if (path.size() < 2 || region.empty()) return false;
  Clipper64 clipper;
  clipper.AddOpenSubject(Paths64{path});
  clipper.AddClip(region);
  Paths64 closed_solution;
  Paths64 open_solution;
  if (!clipper.Execute(ClipType::Intersection, FillRule::NonZero,
                       closed_solution, open_solution))
    return false;
  double inside = 0.0;
  for (const Path64& piece : open_solution)
    inside += path_length_integer(piece);
  const double total = path_length_integer(path);
  // One integer grid unit is representational slack, not a machining
  // tolerance. The semantic envelope is exclusively the caller's value.
  return inside + 1.0 >= total;
}

bool area_contains_with_tolerance(const Paths64& upper, const Paths64& lower,
                                  double tolerance) {
  if (upper.empty() || lower.empty()) return false;
  Paths64 envelope = upper;
  if (tolerance > 0.0)
    envelope = InflatePaths(
        Union(upper, FillRule::NonZero),
        static_cast<double>(std::max<std::int64_t>(1, to_integer(tolerance))),
        JoinType::Round, EndType::Polygon);
  return Difference(lower, envelope, FillRule::NonZero).empty();
}

double absolute_area(const Paths64& paths) {
  double area = 0.0;
  for (const Path64& path : paths) area += Area(path);
  return std::abs(area);
}

void append_path(Path64& destination, const Path64& source) {
  if (source.empty()) return;
  std::size_t begin = 0;
  if (!destination.empty() && destination.back() == source.front()) begin = 1;
  destination.insert(destination.end(), source.begin() + begin, source.end());
}

std::size_t boundary_segment(const Path64& ring, const Point64& point) {
  for (std::size_t i = 0; i < ring.size(); ++i)
    if (point_on_segment(point, ring[i], ring[(i + 1) % ring.size()])) return i;
  std::size_t best = 0;
  long double distance = std::numeric_limits<long double>::max();
  for (std::size_t i = 0; i < ring.size(); ++i) {
    const SegmentProjection projection =
        project_to_segment(point, ring[i], ring[(i + 1) % ring.size()]);
    if (projection.distance < distance) {
      distance = projection.distance;
      best = i;
    }
  }
  return best;
}

Path64 boundary_walk(const FrontLineage& lineage,
                     const BoundaryContact& from,
                     const BoundaryContact& to, bool forward) {
  if (!from.valid || !to.valid || from.component != to.component ||
      from.component >= lineage.boundary_components.size())
    throw std::invalid_argument(
        "Boundary walk contacts do not share a valid component");
  const Path64& ring = lineage.boundary_components[from.component];
  if (ring.size() < 2)
    throw std::runtime_error("Boundary component is degenerate");
  const std::size_t from_segment = boundary_segment(ring, from.point);
  const std::size_t to_segment = boundary_segment(ring, to.point);
  Path64 path{from.point};
  std::size_t segment = from_segment;
  for (std::size_t guard = 0; guard <= ring.size(); ++guard) {
    if (segment == to_segment) {
      if (path.back() != to.point) path.push_back(to.point);
      return path;
    }
    const std::size_t vertex = forward ? (segment + 1) % ring.size() : segment;
    if (path.back() != ring[vertex]) path.push_back(ring[vertex]);
    segment = forward ? (segment + 1) % ring.size()
                      : (segment + ring.size() - 1) % ring.size();
  }
  throw std::runtime_error("Boundary walk did not close around its component");
}

OrientedFront orient_after(const LeadingFront& front,
                           const OrientedFront* previous) {
  OrientedFront result{front.path, front.boundary_contacts};
  if (previous == nullptr || result.path.size() < 2) return result;
  const double direct = point_distance(previous->path.front(), result.path.front()) +
                        point_distance(previous->path.back(), result.path.back());
  const double reverse = point_distance(previous->path.front(), result.path.back()) +
                         point_distance(previous->path.back(), result.path.front());
  if (reverse < direct) {
    std::reverse(result.path.begin(), result.path.end());
    std::swap(result.contacts[0], result.contacts[1]);
  }
  return result;
}

Paths64 member_bands(const FrontLineage& lineage, const FrontNode& node) {
  Paths64 bands;
  for (const FrontId2D id : node.fronts) {
    const LeadingFront& front =
        lineage.generations[id.generation].fronts[id.index];
    if (front.band >= lineage.generations[id.generation].bands.size())
      throw std::runtime_error("Wavefront refers to a missing generation band");
    const Paths64& band = lineage.generations[id.generation].bands[front.band];
    bands.insert(bands.end(), band.begin(), band.end());
  }
  return Union(bands, FillRule::NonZero);
}

Paths64 one_closed_component(const Paths64& raw, const char* context) {
  const Paths64 united = Union(raw, FillRule::NonZero);
  const std::vector<Paths64> components = band_components(united);
  if (components.size() != 1 || components.front().empty() ||
      absolute_area(components.front()) <= 0.0) {
    std::ostringstream message;
    message << context << " did not form one closed, non-empty boundary";
    throw std::runtime_error(message.str());
  }
  return components.front();
}

Path64 endpoint_connection(const FrontLineage& lineage,
                           const OrientedFront& from,
                           const OrientedFront& to, std::size_t side,
                           const Paths64& swept) {
  const Point64& a = side == 0 ? from.path.front() : from.path.back();
  const Point64& b = side == 0 ? to.path.front() : to.path.back();
  const BoundaryContact& a_contact = from.contacts[side];
  const BoundaryContact& b_contact = to.contacts[side];
  Path64 connection;
  if (a_contact.valid && b_contact.valid) {
    if (a_contact.component != b_contact.component)
      throw std::runtime_error(
          "A node endpoint changed known boundary component without a split or merge");
    const Path64 forward = boundary_walk(lineage, a_contact, b_contact, true);
    const Path64 reverse = boundary_walk(lineage, a_contact, b_contact, false);
    const Paths64 guard = InflatePaths(swept, boundary_slack(lineage.stepover),
                                       JoinType::Round, EndType::Polygon);
    const bool forward_legal = whole_open_path_inside(forward, guard);
    const bool reverse_legal = whole_open_path_inside(reverse, guard);
    if (!forward_legal && !reverse_legal)
      throw std::runtime_error(
          "A node boundary rail leaves its propagation bands");
    const Path64& boundary =
        forward_legal && reverse_legal
            ? (path_length_integer(forward) <= path_length_integer(reverse)
                   ? forward
                   : reverse)
            : (forward_legal ? forward : reverse);
    connection = {a};
    if (connection.back() != a_contact.point)
      connection.push_back(a_contact.point);
    append_path(connection, boundary);
    if (connection.back() != b) connection.push_back(b);
    return connection;
  }

  // A boundary attachment may begin or end within a 1->1 run. In that case
  // there is no static contour to follow; the endpoint trajectory itself is
  // the node side. It is accepted only when the local propagation bands prove
  // that the straight bridge belongs to the node.
  connection = {a, b};
  const Paths64 guard = InflatePaths(swept, boundary_slack(lineage.stepover),
                                     JoinType::Round, EndType::Polygon);
  if (!whole_open_path_inside(connection, guard))
    throw std::runtime_error(
        "An attaching or detaching node endpoint leaves its propagation bands");
  return connection;
}

Paths64 terminal_open_footprint(const FrontLineage& lineage,
                                const std::vector<OrientedFront>& fronts,
                                const std::array<Path64, 2>& rails,
                                const Paths64& swept) {
  const OrientedFront& last = fronts.back();
  const BoundaryContact& end = last.contacts[1];
  const BoundaryContact& start = last.contacts[0];
  if (!end.valid || !start.valid || end.component != start.component)
    throw std::runtime_error(
        "A terminal node's last open front does not close on one boundary component");

  std::array<Paths64, 2> candidates;
  std::array<double, 2> cap_lengths{};
  std::array<bool, 2> valid{};
  for (std::size_t direction = 0; direction < 2; ++direction) {
    Path64 boundary = fronts.front().path;
    append_path(boundary, rails[1]);
    Path64 cap{last.path.back()};
    if (cap.back() != end.point) cap.push_back(end.point);
    const Path64 boundary_cap =
        boundary_walk(lineage, end, start, direction == 0);
    cap_lengths[direction] = path_length_integer(boundary_cap);
    append_path(cap, boundary_cap);
    if (cap.back() != last.path.front()) cap.push_back(last.path.front());
    append_path(boundary, cap);
    Path64 return_rail = rails[0];
    std::reverse(return_rail.begin(), return_rail.end());
    append_path(boundary, return_rail);
    try {
      candidates[direction] = one_closed_component(
          Paths64{boundary}, "Terminal node cap candidate");
      valid[direction] = true;
    } catch (const std::runtime_error&) {
      // The complementary walk can self-intersect around a concave boundary.
      // It is simply not a candidate; the terminal still succeeds if the other
      // direction forms the required single closed component.
    }
  }
  std::array<double, 2> mismatch{};
  for (std::size_t direction = 0; direction < 2; ++direction) {
    if (!valid[direction]) {
      mismatch[direction] = std::numeric_limits<double>::infinity();
      continue;
    }
    const double outside = absolute_area(
        Difference(candidates[direction], swept, FillRule::NonZero));
    const double missing = absolute_area(
        Difference(swept, candidates[direction], FillRule::NonZero));
    mismatch[direction] = outside + missing;
  }
  if (!valid[0] && !valid[1])
    throw std::runtime_error(
        "Neither terminal boundary direction forms one closed component");
  std::size_t selected = valid[0] ? 0 : 1;
  if (mismatch[1] < mismatch[0] ||
      (mismatch[1] == mismatch[0] && cap_lengths[1] < cap_lengths[0]))
    selected = 1;
  return candidates[selected];
}


std::vector<InterLayerChain> assemble_chains(
    const std::vector<std::size_t>& layer_sizes,
    const std::vector<std::vector<std::size_t>>& next,
    std::vector<std::vector<std::size_t>>& chain_ids) {
  chain_ids.resize(layer_sizes.size());
  std::vector<std::vector<std::size_t>> previous(layer_sizes.size());
  for (std::size_t layer = 0; layer < layer_sizes.size(); ++layer) {
    chain_ids[layer].assign(layer_sizes[layer], kNoTopologyId);
    previous[layer].assign(layer_sizes[layer], kNoTopologyId);
  }
  for (std::size_t layer = 0; layer + 1 < next.size(); ++layer)
    for (std::size_t index = 0; index < next[layer].size(); ++index)
      if (next[layer][index] != kNoTopologyId)
        previous[layer + 1][next[layer][index]] = index;

  std::vector<InterLayerChain> chains;
  for (std::size_t layer = 0; layer < layer_sizes.size(); ++layer) {
    for (std::size_t index = 0; index < layer_sizes[layer]; ++index) {
      if (previous[layer][index] != kNoTopologyId ||
          next[layer][index] == kNoTopologyId)
        continue;
      InterLayerChain chain;
      std::size_t at_layer = layer;
      std::size_t at_index = index;
      while (true) {
        chain.members.push_back({at_layer, at_index});
        if (at_layer + 1 >= next.size() ||
            next[at_layer][at_index] == kNoTopologyId)
          break;
        at_index = next[at_layer][at_index];
        ++at_layer;
      }
      if (chain.members.size() < 2) continue;
      const std::size_t id = chains.size();
      for (const LayerItemId& member : chain.members)
        chain_ids[member.layer][member.index] = id;
      chains.push_back(std::move(chain));
    }
  }
  return chains;
}

}  // namespace wave::detail
