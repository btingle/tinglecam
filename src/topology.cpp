#include "topology.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <sstream>
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

void extract_ring_fronts(const Path64& ring, std::size_t band,
                         const Paths64& unreached,
                         const Paths64& static_boundaries,
                         std::vector<LeadingFront>& fronts) {
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

std::vector<LeadingFront> extract_leading_fronts(const std::vector<Paths64>& bands,
                                                 const Paths64& unreached,
                                                 const Paths64& static_boundaries) {
  std::vector<LeadingFront> fronts;
  for (std::size_t band = 0; band < bands.size(); ++band)
    for (const auto& ring : bands[band])
      extract_ring_fronts(ring, band, unreached, static_boundaries, fronts);
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
// The rings and the bands are separately rounded descriptions of the same
// walls: the rings come from a boolean on the machinable region, the bands from
// offsetting. They agree to a couple of grid units, not exactly, and a rail is
// deliberately laid on the ring. Measuring one against the other any more
// tightly than a contact was accepted in the first place rejects correct
// geometry over a micron of rounding.
double boundary_slack(double stepover) {
  return std::clamp(static_cast<double>(to_integer(stepover)) * 0.001, 4.0,
                    100.0);
}

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

double path_length_integer(const Path64& path, bool closed = false) {
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

bool exact_area_contains(const Paths64& upper, const Paths64& lower) {
  if (upper.empty() || lower.empty()) return false;
  return Difference(lower, upper, FillRule::NonZero).empty();
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

struct OrientedFront {
  Path64 path;
  std::array<BoundaryContact, 2> contacts;
};

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

class DisjointSet {
 public:
  explicit DisjointSet(std::size_t count) : parent_(count), rank_(count, 0) {
    std::iota(parent_.begin(), parent_.end(), 0);
  }
  std::size_t find(std::size_t value) {
    if (parent_[value] != value) parent_[value] = find(parent_[value]);
    return parent_[value];
  }
  void join(std::size_t a, std::size_t b) {
    a = find(a);
    b = find(b);
    if (a == b) return;
    if (rank_[a] < rank_[b]) std::swap(a, b);
    parent_[b] = a;
    if (rank_[a] == rank_[b]) ++rank_[a];
  }

 private:
  std::vector<std::size_t> parent_;
  std::vector<unsigned char> rank_;
};

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

}  // namespace

FrontLineage build_front_lineage(const WaveResult& wave) {
  if (!(wave.stepover > 0.0)) throw std::invalid_argument("Stepover must be positive");
  if (wave.softening < 0.0 || wave.softening > wave.stepover)
    throw std::invalid_argument("Softening must be between zero and stepover");
  if (wave.generations.size() != wave.reached_generations.size())
    throw std::invalid_argument("Wave bands and cumulative reached sets do not match");
  FrontLineage result;
  result.stepover = wave.stepover;
  if (wave.generations.empty()) return result;

  const Paths64 stock = wave.stock.empty() ? Paths64{wave.stock_boundary} : wave.stock;
  // The wave runs through the domain, which a lead-in extends past the stock
  // wall into open air. Bounding fronts by the stock instead would leave every
  // run-in band with nothing unreached ahead of it and no front to hand on, and
  // would call the stock wall a static boundary when the wave is meant to cross
  // it. Without a lead-in the two are the same region.
  const Paths64& region = wave.domain.empty() ? stock : wave.domain;
  const Paths64 free_space = Difference(region, wave.subject, FillRule::NonZero);
  // Contact identity belongs to the final machinable region, not to whichever
  // input polygon happened to contribute an edge. Using concatenated subject
  // and domain rings gives one physical boundary two IDs where inputs touch.
  const Paths64 static_boundaries = free_space;
  result.boundary_components = static_boundaries;
  result.generations.reserve(wave.generations.size());

  for (std::size_t generation = 0; generation < wave.generations.size(); ++generation) {
    Paths64 unreached = Difference(free_space, wave.reached_generations[generation],
                                   FillRule::NonZero);
    unreached = Union(unreached, FillRule::NonZero);
    std::vector<Paths64> bands = band_components(wave.generations[generation]);
    auto fronts = extract_leading_fronts(bands, unreached, static_boundaries);
    const double contact_tolerance = boundary_slack(wave.stepover);
    classify_front_contacts(fronts, static_boundaries, contact_tolerance);
    result.generations.push_back({std::move(fronts), std::move(bands)});
  }

  // Entry identity belongs to the wave source itself. Deriving it from the
  // stock wall splits one source whenever the protected subject happens to
  // occlude that wall in several places. Generation zero is the connected
  // source boundary from which propagation actually starts (at the outside of
  // the run-in when one exists), so its fronts are the stable selectable entry
  // fronts.
  if (!result.generations.empty())
    result.entry_fronts = result.generations.front().fronts;

  const double stepover_integer = static_cast<double>(
      std::max<std::int64_t>(1, to_integer(wave.stepover)));
  const double softening_integer = static_cast<double>(
      std::clamp<std::int64_t>(to_integer(wave.softening), 0,
                               static_cast<std::int64_t>(stepover_integer)));
  const double tolerance = std::clamp(stepover_integer * 0.001, 4.0, 100.0);

  for (std::size_t generation = 1; generation < result.generations.size(); ++generation) {
    auto& previous = result.generations[generation - 1].fronts;
    auto& current = result.generations[generation].fronts;
    Paths64 band_guard = InflatePaths(wave.generations[generation], tolerance,
                                      JoinType::Miter, EndType::Polygon);
    for (std::size_t parent = 0; parent < previous.size(); ++parent) {
      Paths64 candidate = front_reach(previous[parent], stepover_integer + tolerance,
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

LayerTopology build_layer_topology(const FrontLineage& lineage) {
  LayerTopology result;
  result.front_nodes.resize(lineage.generations.size());
  std::vector<std::size_t> offsets(lineage.generations.size() + 1, 0);
  std::size_t total = 0;
  for (std::size_t generation = 0; generation < lineage.generations.size(); ++generation) {
    offsets[generation] = total;
    result.front_nodes[generation].assign(
        lineage.generations[generation].fronts.size(), kNoTopologyId);
    total += lineage.generations[generation].fronts.size();
  }
  offsets.back() = total;
  if (total == 0) return result;

  DisjointSet sets(total);
  for (std::size_t generation = 0; generation + 1 < lineage.generations.size();
       ++generation) {
    const auto& parents = lineage.generations[generation].fronts;
    const auto& children = lineage.generations[generation + 1].fronts;
    for (std::size_t parent = 0; parent < parents.size(); ++parent) {
      if (parents[parent].children.size() != 1) continue;
      const std::size_t child = parents[parent].children.front();
      if (child >= children.size() || children[child].parents.size() != 1) continue;
      sets.join(offsets[generation] + parent,
                offsets[generation + 1] + child);
    }
  }

  std::vector<std::size_t> root_to_node(total, kNoTopologyId);
  for (std::size_t generation = 0; generation < lineage.generations.size(); ++generation) {
    for (std::size_t index = 0;
         index < lineage.generations[generation].fronts.size(); ++index) {
      const std::size_t root = sets.find(offsets[generation] + index);
      if (root_to_node[root] == kNoTopologyId) {
        root_to_node[root] = result.nodes.size();
        result.nodes.push_back({});
      }
      const std::size_t node = root_to_node[root];
      result.front_nodes[generation][index] = node;
      result.nodes[node].fronts.push_back({generation, index});
    }
  }

  const double half_pitch = std::max<double>(1.0, to_integer(lineage.stepover) * 0.5);
  for (FrontNode& node : result.nodes) {
    const FrontId2D last_id = *std::max_element(
        node.fronts.begin(), node.fronts.end(),
        [](const FrontId2D& a, const FrontId2D& b) {
          return a.generation < b.generation;
        });
    const LeadingFront& last_front =
        lineage.generations[last_id.generation].fronts[last_id.index];
    const bool terminal = last_front.children.empty();
    if (node.fronts.size() == 1 && !terminal) {
      // A singleton has no preceding/following fronts with which to enclose an
      // area, so a non-terminal singleton alone receives the half-pitch local
      // responsibility band.
      const FrontId2D id = node.fronts.front();
      const LeadingFront& front =
          lineage.generations[id.generation].fronts[id.index];
      if (front.path.size() >= 2 &&
          front.band < lineage.generations[id.generation].bands.size()) {
        const EndType end = front.closed ? EndType::Joined : EndType::Round;
        const Paths64 envelope = InflatePaths(Paths64{front.path}, half_pitch,
                                               JoinType::Round, end);
        const Paths64 footprint = Intersect(
            envelope, lineage.generations[id.generation].bands[front.band],
            FillRule::NonZero);
        node.footprint = one_closed_component(
            footprint, "Non-terminal singleton node footprint");
      }
      continue;
    }

    const Paths64 swept = member_bands(lineage, node);
    std::vector<OrientedFront> open_fronts;
    std::vector<Path64> closed_fronts;
    bool has_open = false;
    bool has_closed = false;
    for (const FrontId2D id : node.fronts) {
      const LeadingFront& front =
          lineage.generations[id.generation].fronts[id.index];
      if (front.closed) {
        has_closed = true;
        closed_fronts.push_back(front.path);
      } else {
        has_open = true;
      }
    }

    // Detaching from a static boundary turns an open front into a closed one;
    // reattachment does the reverse. Such a run has no continuous pair of
    // endpoint rails. Its connected swept band is the geometric record of the
    // attachment transition, so use that boundary and require it to be one
    // closed component rather than silently inventing endpoints.
    if (has_open && has_closed) {
      node.footprint = one_closed_component(
          swept, "Attaching or detaching node footprint");
      continue;
    }

    if (has_open)
      for (const FrontId2D id : node.fronts) {
        const LeadingFront& front =
            lineage.generations[id.generation].fronts[id.index];
        open_fronts.push_back(orient_after(
            front, open_fronts.empty() ? nullptr : &open_fronts.back()));
      }

    if (open_fronts.empty()) {
      if (closed_fronts.empty())
        throw std::runtime_error("A node contains no usable front boundary");
      const Paths64 first = one_closed_component(
          Paths64{closed_fronts.front()}, "Closed node's first front");
      if (terminal) {
        node.footprint = first;
      } else {
        const Paths64 last = one_closed_component(
            Paths64{closed_fronts.back()}, "Closed node's last front");
        const bool first_contains_last = Difference(
            last, first, FillRule::NonZero).empty();
        const bool last_contains_first = Difference(
            first, last, FillRule::NonZero).empty();
        if (!first_contains_last && !last_contains_first)
          throw std::runtime_error(
              "A closed node's first and last fronts are not nested");
        node.footprint = one_closed_component(
            first_contains_last ? Difference(first, last, FillRule::NonZero)
                                : Difference(last, first, FillRule::NonZero),
            "Closed non-terminal node footprint");
      }
      continue;
    }

    std::array<Path64, 2> rails{
        Path64{open_fronts.front().path.front()},
        Path64{open_fronts.front().path.back()},
    };
    for (std::size_t i = 1; i < open_fronts.size(); ++i)
      for (std::size_t side = 0; side < 2; ++side)
        append_path(rails[side], endpoint_connection(
                                     lineage, open_fronts[i - 1],
                                     open_fronts[i], side, swept));

    if (terminal) {
      const auto& contacts = open_fronts.back().contacts;
      if (contacts[0].valid && contacts[1].valid &&
          contacts[0].component == contacts[1].component)
        node.footprint = terminal_open_footprint(lineage, open_fronts,
                                                 rails, swept);
      else
        node.footprint = one_closed_component(
            swept, "Terminal node footprint without a static-boundary cap");
      continue;
    }

    Path64 boundary = open_fronts.front().path;
    append_path(boundary, rails[1]);
    Path64 last_reverse = open_fronts.back().path;
    std::reverse(last_reverse.begin(), last_reverse.end());
    append_path(boundary, last_reverse);
    Path64 first_return = rails[0];
    std::reverse(first_return.begin(), first_return.end());
    append_path(boundary, first_return);
    node.footprint = one_closed_component(
        Paths64{boundary}, "Open non-terminal node footprint");
  }

  for (const FrontNode& node : result.nodes) {
    if (node.footprint.empty())
      throw std::runtime_error("A node footprint has no closed boundary");
  }

  // The node graph is already in generation order, so a single forward pass
  // computes its topological evaluation depth.
  for (std::size_t generation = 0; generation + 1 < lineage.generations.size();
       ++generation) {
    for (std::size_t parent = 0;
         parent < lineage.generations[generation].fronts.size(); ++parent) {
      const std::size_t parent_node = result.front_nodes[generation][parent];
      for (const std::size_t child :
           lineage.generations[generation].fronts[parent].children) {
        if (child >= result.front_nodes[generation + 1].size()) continue;
        const std::size_t child_node = result.front_nodes[generation + 1][child];
        if (child_node == parent_node) continue;
        result.nodes[child_node].evaluation_order =
            std::max(result.nodes[child_node].evaluation_order,
                     result.nodes[parent_node].evaluation_order + 1);
      }
    }
  }

  result.entry_fronts.resize(lineage.entry_fronts.size());
  std::iota(result.entry_fronts.begin(), result.entry_fronts.end(), 0);
  return result;
}

InterLayerTopology build_inter_layer_topology(
    const std::vector<const FrontLineage*>& lineages,
    double entry_front_match_tolerance) {
  if (!(entry_front_match_tolerance > 0.0) ||
      !std::isfinite(entry_front_match_tolerance))
    throw std::invalid_argument("Entry-front match tolerance must be positive");
  InterLayerTopology result;
  result.layers.reserve(lineages.size());
  for (const FrontLineage* lineage : lineages) {
    if (!lineage) throw std::invalid_argument("Inter-layer lineage is null");
    result.layers.push_back(build_layer_topology(*lineage));
  }

  std::vector<std::size_t> node_sizes;
  std::vector<std::vector<std::size_t>> node_next(result.layers.size());
  for (const LayerTopology& layer : result.layers)
    node_sizes.push_back(layer.nodes.size());
  for (std::size_t layer = 0; layer < result.layers.size(); ++layer)
    node_next[layer].assign(result.layers[layer].nodes.size(), kNoTopologyId);

  for (std::size_t layer = 0; layer + 1 < result.layers.size(); ++layer) {
    const auto& upper = result.layers[layer].nodes;
    const auto& lower = result.layers[layer + 1].nodes;
    std::vector<std::size_t> upper_count(upper.size(), 0);
    std::vector<std::size_t> lower_count(lower.size(), 0);
    std::vector<std::pair<std::size_t, std::size_t>> candidates;
    for (std::size_t a = 0; a < upper.size(); ++a)
      for (std::size_t b = 0; b < lower.size(); ++b)
        if (upper[a].evaluation_order == lower[b].evaluation_order &&
            exact_area_contains(upper[a].footprint, lower[b].footprint)) {
          ++upper_count[a];
          ++lower_count[b];
          candidates.push_back({a, b});
        }
    for (const auto& [a, b] : candidates)
      if (upper_count[a] == 1 && lower_count[b] == 1) node_next[layer][a] = b;
  }
  result.node_chains =
      assemble_chains(node_sizes, node_next, result.node_chain_ids);

  std::vector<std::size_t> entry_sizes;
  std::vector<std::vector<std::size_t>> entry_next(result.layers.size());
  for (const LayerTopology& layer : result.layers)
    entry_sizes.push_back(layer.entry_fronts.size());
  for (std::size_t layer = 0; layer < result.layers.size(); ++layer)
    entry_next[layer].assign(result.layers[layer].entry_fronts.size(), kNoTopologyId);
  const double tolerance = static_cast<double>(
      std::max<std::int64_t>(1, to_integer(entry_front_match_tolerance)));
  for (std::size_t layer = 0; layer + 1 < result.layers.size(); ++layer) {
    const auto& upper_layer = result.layers[layer];
    const auto& lower_layer = result.layers[layer + 1];
    std::vector<std::size_t> upper_count(upper_layer.entry_fronts.size(), 0);
    std::vector<std::size_t> lower_count(lower_layer.entry_fronts.size(), 0);
    std::vector<std::pair<std::size_t, std::size_t>> candidates;
    for (std::size_t a = 0; a < upper_layer.entry_fronts.size(); ++a) {
      const std::size_t upper_id = upper_layer.entry_fronts[a];
      const LeadingFront& upper = lineages[layer]->entry_fronts[upper_id];
      const Paths64 envelope = InflatePaths(Paths64{upper.path}, tolerance,
                                             JoinType::Round,
                                             upper.closed ? EndType::Joined
                                                          : EndType::Round);
      for (std::size_t b = 0; b < lower_layer.entry_fronts.size(); ++b) {
        const std::size_t lower_id = lower_layer.entry_fronts[b];
        const LeadingFront& lower = lineages[layer + 1]->entry_fronts[lower_id];
        Path64 lower_path = lower.path;
        if (lower.closed && !lower_path.empty())
          lower_path.push_back(lower_path.front());
        if (!whole_open_path_inside(lower_path, envelope)) continue;
        ++upper_count[a];
        ++lower_count[b];
        candidates.push_back({a, b});
      }
    }
    for (const auto& [a, b] : candidates)
      if (upper_count[a] == 1 && lower_count[b] == 1) entry_next[layer][a] = b;
  }
  result.entry_front_chains =
      assemble_chains(entry_sizes, entry_next, result.entry_front_chain_ids);

  std::vector<std::vector<bool>> scheduled(result.layers.size());
  for (std::size_t layer = 0; layer < result.layers.size(); ++layer)
    scheduled[layer].assign(result.layers[layer].nodes.size(), false);
  for (std::size_t layer = 0; layer < result.layers.size(); ++layer) {
    std::vector<std::size_t> order(result.layers[layer].nodes.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
      return result.layers[layer].nodes[a].evaluation_order <
             result.layers[layer].nodes[b].evaluation_order;
    });
    for (const std::size_t node : order) {
      if (scheduled[layer][node]) continue;
      const std::size_t chain = result.node_chain_ids[layer][node];
      if (chain == kNoTopologyId) {
        result.node_schedule.push_back({layer, node});
        scheduled[layer][node] = true;
        continue;
      }
      const auto& members = result.node_chains[chain].members;
      auto current = std::find_if(
          members.begin(), members.end(), [&](const LayerItemId& member) {
            return member.layer == layer && member.index == node;
          });
      for (; current != members.end(); ++current) {
        if (scheduled[current->layer][current->index]) continue;
        result.node_schedule.push_back(*current);
        scheduled[current->layer][current->index] = true;
      }
    }
  }
  return result;
}

Path64 boundary_arc(const FrontLineage& lineage,
                    const BoundaryContact& from,
                    const BoundaryContact& to) {
  Path64 forward = boundary_walk(lineage, from, to, true);
  Path64 reverse = boundary_walk(lineage, from, to, false);
  return path_length_integer(forward) <= path_length_integer(reverse)
             ? forward
             : reverse;
}

Paths64 entry_front_territory(const FrontLineage& lineage, std::size_t entry) {
  if (lineage.generations.empty() ||
      entry >= lineage.generations.front().fronts.size())
    return {};
  // Breadth-first down the generations. A generation is only ever reached from
  // the one above it, so a single forward sweep with a per-generation frontier
  // visits every descendant without revisiting any.
  Paths64 territory;
  std::vector<std::size_t> frontier{entry};
  for (std::size_t generation = 0;
       generation < lineage.generations.size() && !frontier.empty();
       ++generation) {
    const FrontGeneration& level = lineage.generations[generation];
    std::vector<bool> band_taken(level.bands.size(), false);
    std::vector<bool> next_seen(
        generation + 1 < lineage.generations.size()
            ? lineage.generations[generation + 1].fronts.size()
            : 0,
        false);
    std::vector<std::size_t> next;
    for (const std::size_t index : frontier) {
      if (index >= level.fronts.size()) continue;
      const LeadingFront& front = level.fronts[index];
      if (front.band < level.bands.size() && !band_taken[front.band]) {
        band_taken[front.band] = true;
        const Paths64& band = level.bands[front.band];
        territory.insert(territory.end(), band.begin(), band.end());
      }
      for (const std::size_t child : front.children)
        if (child < next_seen.size() && !next_seen[child]) {
          next_seen[child] = true;
          next.push_back(child);
        }
    }
    frontier = std::move(next);
  }
  return Union(territory, FillRule::NonZero);
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
