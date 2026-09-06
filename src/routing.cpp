#include "routing.h"

#include <clipper2/clipper.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace wave {
namespace {

using namespace Clipper2Lib;

double length_of(const Point64& a, const Point64& b) {
  return std::hypot(static_cast<double>(b.x - a.x),
                    static_cast<double>(b.y - a.y));
}

// Which side of the directed line a->b the point c falls on. Coordinates are
// micrometres, so a part a metre across still leaves the products four orders
// of magnitude inside a signed 64-bit range.
std::int64_t side(const Point64& a, const Point64& b, const Point64& c) {
  const std::int64_t value = (b.x - a.x) * (c.y - a.y) -
                             (b.y - a.y) * (c.x - a.x);
  return value > 0 ? 1 : (value < 0 ? -1 : 0);
}

// True only when the two segments cross at a point interior to both. Touching
// at an endpoint does not count, and neither does running along each other: a
// route is entitled to touch the corners it turns at and to graze a wall, and
// treating either as a blockage would reject every useful edge in the graph.
bool crosses(const Point64& a, const Point64& b, const Point64& c,
             const Point64& d) {
  const std::int64_t d1 = side(a, b, c);
  const std::int64_t d2 = side(a, b, d);
  const std::int64_t d3 = side(c, d, a);
  const std::int64_t d4 = side(c, d, b);
  return d1 * d2 < 0 && d3 * d4 < 0;
}

// Non-zero containment across a whole set of rings, so a point sitting in a
// hole reads as outside rather than as inside the ring that bounds it.
bool inside(const Paths64& paths, const Point64& point) {
  int winding = 0;
  for (const Path64& ring : paths) {
    if (ring.size() < 3) continue;
    const PointInPolygonResult result = PointInPolygon(point, ring);
    if (result == PointInPolygonResult::IsOn) return true;
    if (result == PointInPolygonResult::IsInside)
      winding += Area(ring) >= 0.0 ? 1 : -1;
  }
  return winding != 0;
}

Point64 along(const Point64& a, const Point64& b, double t) {
  return Point64(a.x + static_cast<std::int64_t>((b.x - a.x) * t),
                 a.y + static_cast<std::int64_t>((b.y - a.y) * t));
}

double open_length(const Paths64& paths) {
  double total = 0.0;
  for (const Path64& path : paths)
    for (std::size_t i = 1; i < path.size(); ++i)
      total += length_of(path[i - 1], path[i]);
  return total;
}

double path_length(const Path64& path) {
  double total = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i)
    total += length_of(path[i - 1], path[i]);
  return total;
}

// A closed ring covering the disc of `radius` about `centre`. Coarse on
// purpose: it is intersected with the caller's region before it is used for
// anything, so its only job is to bound how far from the anchor that region is
// restored. Eight sides through the radius rather than around it: the narrowest
// direction still reaches 92% of it, which is well clear of the inset this has
// to bridge, and every unit of overshoot is region restored for nothing.
Path64 disc_around(const Point64& centre, double radius) {
  constexpr std::size_t kSides = 8;
  constexpr double kTwoPi = 6.283185307179586;
  Path64 ring;
  ring.reserve(kSides);
  for (std::size_t i = 0; i < kSides; ++i) {
    const double angle = kTwoPi * static_cast<double>(i) /
                         static_cast<double>(kSides);
    ring.push_back(Point64(
        centre.x + std::llround(std::cos(angle) * radius),
        centre.y + std::llround(std::sin(angle) * radius)));
  }
  return ring;
}

}  // namespace

Paths64 routable_region(const Paths64& region, double slack,
                        const std::vector<Point64>& anchors) {
  if (region.empty() || !(slack > 0.0)) return region;
  // What a route may spend before it is asked for one: the Router pads by the
  // slack to build its graph and again to probe against, so this is how far
  // outside the region a route it offers can sit.
  const double inset = 2.0 * slack;
  const Paths64 eroded =
      InflatePaths(region, -inset, JoinType::Miter, EndType::Polygon);
  if (anchors.empty()) return eroded;

  // Half again the inset: far enough that an anchor on the region's own
  // boundary reaches the eroded interior with room to spare rather than meeting
  // it at a single point, and no farther, because inside this radius a route is
  // held to `region` rather than to the eroded one, and holding it to the
  // eroded one is the whole of what the erosion was buying. Widening it does
  // not buy anything back: at twice the inset the circular test part gouges
  // 0.576 mm2 against 0.567 here, and lifts the same 70-odd times either way.
  const double reach = 1.5 * inset;
  // Deduplicated first. A caller naming the two ends of every contour variant
  // names most points twice over, and each repeat is another ring for the union
  // below to reconcile against every other.
  std::vector<Point64> unique(anchors);
  std::sort(unique.begin(), unique.end(),
            [](const Point64& a, const Point64& b) {
              return a.x != b.x ? a.x < b.x : a.y < b.y;
            });
  unique.erase(std::unique(unique.begin(), unique.end()), unique.end());
  Paths64 discs;
  discs.reserve(unique.size());
  for (const Point64& anchor : unique) discs.push_back(disc_around(anchor, reach));
  const Paths64 restored =
      Intersect(Union(discs, FillRule::NonZero), region, FillRule::NonZero);
  return Union(eroded, restored, FillRule::NonZero);
}

Router::Router(const Paths64& region, double slack)
    : slack_(std::max(slack, 2.0)) {
  if (region.empty()) return;
  // Normalised before anything is measured off it. A caller's region is a union
  // of whatever has been cleared so far and may well arrive with its rings
  // overlapping and wound every which way, and offsetting that directly does
  // not offset it: on a two-band test case it turned the cut bands into islands
  // marooned inside the stock, with no way out to the air they are joined to.
  // Orientation matters twice over here anyway -- the corner rule reads the
  // filled side off the winding, and containment counts it.
  region_ = Union(region, FillRule::NonZero);
  if (region_.empty()) return;
  const double pad = slack_;
  padded_ = InflatePaths(region_, pad, JoinType::Miter, EndType::Polygon);
  if (padded_.empty()) {
    region_.clear();
    return;
  }
  probe_ = InflatePaths(padded_, pad, JoinType::Miter, EndType::Polygon);

  // The graph is built on a simplified outline, and this is not an economy: it
  // is what makes the graph mean anything. An offset boundary zigzags by a unit
  // or two from vertex to vertex, so on the raw article left and right turns
  // very nearly alternate -- on the sample parts, 322 of one against 294 of the
  // other around a single island. Every straight line between two points on
  // such a boundary clips one of those zigzags, so tested against it nothing can
  // see anything and the graph falls apart into dozens of pieces.
  //
  // Simplifying by the slack removes exactly the detail the slack already says
  // is not to be trusted, and the corners and the obstacles are taken from the
  // same simplified outline so a route along the boundary is legal by
  // construction rather than by tolerance.
  // Simplified by the pad, which is the scale below which the padding has
  // already said the boundary is not to be trusted. Anything finer is detail
  // this cannot honour, and anything coarser walks the outline out past the
  // region every route is finally measured against.
  const Paths64 outline = SimplifyPaths(padded_, pad);
  for (const Path64& ring : outline) {
    if (ring.size() < 2) continue;
    for (std::size_t i = 0; i < ring.size(); ++i) {
      edge_from_.push_back(ring[i]);
      edge_to_.push_back(ring[(i + 1) % ring.size()]);
    }
  }

  // Every vertex of the outline is a corner the graph may turn at, and the
  // temptation to keep only the ones a shortest path can actually turn at --
  // the reflex ones, where the free space wraps around an obstacle -- should be
  // resisted. That reduction is sound for exact geometry and treacherous here.
  // Two neighbouring reflex corners are joined by the boundary edge between
  // them, but a boundary this one is offset output, and a run of it alternates
  // reflex and convex almost vertex by vertex, so neighbouring reflex corners
  // are rarely neighbours on the ring and the chord between them clips whatever
  // sits in between. The chain around an obstacle then depends on which
  // vertices a given simplification happened to leave: on one sample region it
  // held at an epsilon of 45 units, broke at 60, held again from 75 to 120, and
  // broke past 180. Keeping every vertex makes each one reachable from its
  // neighbour along their shared edge, so the graph is connected wherever the
  // region is, and the answer stops depending on a tuning constant.
  //
  // Nothing is lost by it: the turning points of a true shortest path are still
  // in the set, so the route found is still the shortest one.
  std::vector<std::pair<std::size_t, std::size_t>> ring_spans;
  for (const Path64& ring : outline) {
    const std::size_t begin = corners_.size();
    for (const Point64& vertex : ring) corners_.push_back(vertex);
    ring_spans.push_back({begin, corners_.size()});
  }
  outline_ = outline;

  const std::size_t count = corners_.size();
  corner_visible_.assign(count * count, 2);
  for (std::size_t i = 0; i < count; ++i)
    corner_visible_[i * count + i] = 1;

  // Two vertices with a boundary edge between them are joined whatever the
  // visibility test makes of it, because that edge is a legal route by
  // construction: it is the region's own edge. Asking instead is asking a
  // question about a segment lying exactly on a boundary, which is the one
  // question this arithmetic cannot answer, and the answer decides whether the
  // graph is connected at all. Left to the test, whether a route exists came
  // and went with the simplification epsilon -- found at 30, lost at 60, found
  // again at 75 through 120 -- for a region whose shape never changed. Stated
  // rather than asked, it stays found.
  for (const auto& [begin, end] : ring_spans) {
    const std::size_t size = end - begin;
    if (size < 2) continue;
    for (std::size_t i = 0; i < size; ++i) {
      const std::size_t a = begin + i;
      const std::size_t b = begin + (i + 1) % size;
      corner_visible_[a * count + b] = 1;
      corner_visible_[b * count + a] = 1;
    }
  }
}

bool Router::visible(const Point64& a, const Point64& b) const {
  if (a == b) return true;
  for (std::size_t i = 0; i < edge_from_.size(); ++i)
    if (crosses(a, b, edge_from_[i], edge_to_[i])) return false;
  // Crossing nothing leaves two cases apart: a segment inside the region and
  // one wholly outside it, in a hole or beyond the boundary. A point on it
  // tells them apart, and having crossed no edge the whole segment is on that
  // side of the question.
  //
  // Those points are asked of a region padded past the one the graph is built
  // on, and they have to be. The most important segment in the whole graph is
  // the one joining two neighbouring corners, which is a boundary edge itself,
  // so a point on it lies exactly on the boundary -- where integer rounding
  // drops it a fraction of a unit onto whichever side it pleases. Asked of the
  // boundary it sits on, every such segment is a coin toss, and the graph comes
  // apart along the very chain that holds it together: sixteen of thirty-seven
  // neighbouring pairs, on the first part this was run against. Asked of a
  // region padded past it, the point is comfortably inside, while a segment
  // through material still misses by its whole width.
  //
  // More than one point, because crossing nothing is not quite the guarantee it
  // reads as: a segment that leaves the region through a vertex touches rather
  // than crosses, and an offset boundary carrying near-collinear vertices in
  // quantity offers plenty of those to slip out through. Sampling along the
  // segment catches the excursions worth catching at a cost that stays
  // arithmetic.
  const double span = length_of(a, b);
  const std::size_t samples = std::clamp<std::size_t>(
      static_cast<std::size_t>(span / std::max(slack_, 1.0)), 1, 16);
  for (std::size_t i = 0; i < samples; ++i) {
    const double t = (static_cast<double>(i) + 0.5) /
                     static_cast<double>(samples);
    if (!inside(probe_, along(a, b, t))) return false;
  }
  return true;
}

std::optional<Path64> Router::route(const Point64& from,
                                    const Point64& to) const {
  if (!usable()) return std::nullopt;
  if (from == to) return Path64{from, to};
  if (visible(from, to)) return Path64{from, to};

  const std::size_t count = corners_.size();
  if (count == 0) return std::nullopt;
  // The graph for this query: every corner, plus the two ends. Their visibility
  // is the only part that cannot be shared between queries.
  std::vector<std::uint8_t> from_visible(count, 0);
  std::vector<std::uint8_t> to_visible(count, 0);
  for (std::size_t i = 0; i < count; ++i) {
    from_visible[i] = visible(from, corners_[i]) ? 1 : 0;
    to_visible[i] = visible(to, corners_[i]) ? 1 : 0;
  }

  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> best(count, infinity);
  std::vector<std::size_t> previous(count, count);
  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  for (std::size_t i = 0; i < count; ++i)
    if (from_visible[i]) {
      best[i] = length_of(from, corners_[i]);
      queue.push({best[i], i});
    }

  double finish = infinity;
  std::size_t last = count;
  std::vector<bool> settled(count, false);
  while (!queue.empty()) {
    const auto [cost, node] = queue.top();
    queue.pop();
    if (settled[node]) continue;
    settled[node] = true;
    // Nothing still in the queue can beat what is already in hand, because
    // every remaining route reaches its end through a node at least this far.
    if (cost >= finish) break;
    if (to_visible[node]) {
      const double total = cost + length_of(corners_[node], to);
      if (total < finish) {
        finish = total;
        last = node;
      }
    }
    for (std::size_t next = 0; next < count; ++next) {
      if (settled[next]) continue;
      const double candidate = cost + length_of(corners_[node], corners_[next]);
      if (candidate >= best[next]) continue;
      std::uint8_t& edge = corner_visible_[node * count + next];
      if (edge == 2) {
        edge = visible(corners_[node], corners_[next]) ? 1 : 0;
        corner_visible_[next * count + node] = edge;
      }
      if (!edge) continue;
      best[next] = candidate;
      previous[next] = node;
      queue.push({candidate, next});
    }
  }
  if (last == count) return std::nullopt;

  Path64 walk;
  for (std::size_t node = last; node != count; node = previous[node])
    walk.push_back(corners_[node]);
  std::reverse(walk.begin(), walk.end());
  walk.insert(walk.begin(), from);
  walk.push_back(to);

  // The graph is an accelerator, not the authority. Its visibility test is
  // arithmetic on segments and admits a case the region itself would not -- a
  // route threading a pinch narrower than a unit, say -- so what it proposes is
  // measured against the region before it is handed back.
  //
  // Measured against the same padded region the probe uses, for the same
  // reason: a route wraps an obstacle by running along the graph's own
  // boundary, Clipper drops the stretch of an open path lying exactly on the
  // edge of what clips it, and checking against that boundary would reject
  // every route that does the one thing a route around an obstacle has to do.
  // What the check is for survives the padding -- a route through a hole or
  // across a pinch misses by its whole width, not by a rounding.
  Clipper64 clipper;
  clipper.AddOpenSubject(Paths64{walk});
  clipper.AddClip(probe_);
  Paths64 closed;
  Paths64 open;
  if (!clipper.Execute(ClipType::Intersection, FillRule::NonZero, closed, open))
    return std::nullopt;
  // One unit of rounding is available at each joint, so the allowance counts
  // them rather than assuming the whole walk rounds the same way once.
  if (open_length(open) + slack_ * static_cast<double>(walk.size()) <
      path_length(walk))
    return std::nullopt;
  return walk;
}

}  // namespace wave
