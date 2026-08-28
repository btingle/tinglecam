#include "toolpath.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace wave {
namespace {

using namespace Clipper2Lib;

double distance_world(const Point64& a, const Point64& b) {
  return std::hypot(to_world(b.x - a.x), to_world(b.y - a.y));
}

double path_length_world(const Path64& path) {
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i)
    length += distance_world(path[i - 1], path[i]);
  return length;
}

std::size_t nearest_vertex(const Path64& path, const Point64& point) {
  std::size_t best = 0;
  double best_distance = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double dx = static_cast<double>(path[i].x - point.x);
    const double dy = static_cast<double>(path[i].y - point.y);
    const double distance = dx * dx + dy * dy;
    if (distance < best_distance) {
      best_distance = distance;
      best = i;
    }
  }
  return best;
}

double nearest_vertex_distance(const Path64& path, const Point64& point) {
  if (path.empty()) return std::numeric_limits<double>::max();
  return distance_world(path[nearest_vertex(path, point)], point);
}

Point64 lerp_point(const Point64& a, const Point64& b, double t) {
  return Point64(
      static_cast<std::int64_t>(std::llround(a.x + (b.x - a.x) * t)),
      static_cast<std::int64_t>(std::llround(a.y + (b.y - a.y) * t)));
}

// The point `distance` world units along an open polyline.
Point64 point_along(const Path64& path, double distance) {
  if (path.empty()) return Point64();
  double remaining = std::max(0.0, distance);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double segment = distance_world(path[i - 1], path[i]);
    if (remaining <= segment || i + 1 == path.size()) {
      const double t = segment > 0.0 ? std::clamp(remaining / segment, 0.0, 1.0) : 0.0;
      return lerp_point(path[i - 1], path[i], t);
    }
    remaining -= segment;
  }
  return path.back();
}

// Restates a closed ring as an explicit loop that starts and ends at `start`.
Path64 close_ring_from(const Path64& ring, std::size_t start) {
  Path64 path;
  path.reserve(ring.size() + 1);
  for (std::size_t i = 0; i < ring.size(); ++i)
    path.push_back(ring[(start + i) % ring.size()]);
  path.push_back(path.front());
  return path;
}

// The area a connecting move must stay out of: everything that is not yet
// cleared, which is uncut material, the subject, and the space beyond the
// stock all at once. Eroding by `tolerance` is what makes the test usable --
// links run between the ends of open fronts, which lie exactly on the stock
// wall, and a move sliding along that wall must not read as leaving the part.
Paths64 forbidden_region(const Paths64& safe, const Rect64& world,
                         double tolerance) {
  const Path64 box{
      Point64(world.left, world.top), Point64(world.right, world.top),
      Point64(world.right, world.bottom), Point64(world.left, world.bottom)};
  Paths64 forbidden = Difference(Paths64{box}, safe, FillRule::NonZero);
  if (tolerance > 0.0)
    forbidden = InflatePaths(forbidden, -tolerance, JoinType::Miter,
                             EndType::Polygon);
  return forbidden;
}

// True when the straight segment from `a` to `b` grazes the forbidden region by
// no more than `tolerance`.
bool segment_avoids(const Point64& a, const Point64& b, const Paths64& forbidden,
                    double tolerance) {
  if (forbidden.empty()) return true;
  Clipper64 clipper;
  clipper.AddOpenSubject(Paths64{Path64{a, b}});
  clipper.AddClip(forbidden);
  Paths64 closed_solution;
  Paths64 open_solution;
  if (!clipper.Execute(ClipType::Intersection, FillRule::NonZero,
                       closed_solution, open_solution))
    return false;
  double blocked = 0.0;
  for (const auto& piece : open_solution)
    for (std::size_t i = 1; i < piece.size(); ++i)
      blocked += std::hypot(static_cast<double>(piece[i].x - piece[i - 1].x),
                            static_cast<double>(piece[i].y - piece[i - 1].y));
  return blocked <= tolerance;
}

// Where a point lands when dropped onto a closed ring.
struct RingProjection {
  std::size_t segment = 0;
  double t = 0.0;
  double distance = std::numeric_limits<double>::max();
  Point64 point;
};

RingProjection project_onto_ring(const Path64& ring, const Point64& point) {
  RingProjection best;
  for (std::size_t i = 0; i < ring.size(); ++i) {
    const Point64& a = ring[i];
    const Point64& b = ring[(i + 1) % ring.size()];
    const double dx = static_cast<double>(b.x - a.x);
    const double dy = static_cast<double>(b.y - a.y);
    const double square_length = dx * dx + dy * dy;
    double t = 0.0;
    if (square_length > 0.0)
      t = std::clamp((static_cast<double>(point.x - a.x) * dx +
                      static_cast<double>(point.y - a.y) * dy) /
                         square_length,
                     0.0, 1.0);
    const Point64 candidate = lerp_point(a, b, t);
    const double distance = distance_world(candidate, point);
    if (distance < best.distance) best = {i, t, distance, candidate};
  }
  return best;
}

std::vector<double> ring_arc_lengths(const Path64& ring) {
  std::vector<double> cumulative(ring.size() + 1, 0.0);
  for (std::size_t i = 0; i < ring.size(); ++i)
    cumulative[i + 1] =
        cumulative[i] + distance_world(ring[i], ring[(i + 1) % ring.size()]);
  return cumulative;
}

double ring_arc_of(const std::vector<double>& cumulative, const RingProjection& p) {
  return cumulative[p.segment] +
         p.t * (cumulative[p.segment + 1] - cumulative[p.segment]);
}

// The vertices met walking `span` units forward around the ring from `from`,
// bracketed by the two projected endpoints.
Path64 walk_ring(const Path64& ring, const std::vector<double>& cumulative,
                 const RingProjection& from, const RingProjection& to, bool forward,
                 double span) {
  const std::size_t count = ring.size();
  Path64 path;
  path.push_back(from.point);
  double travelled =
      forward ? cumulative[from.segment + 1] - ring_arc_of(cumulative, from)
              : ring_arc_of(cumulative, from) - cumulative[from.segment];
  std::size_t vertex = forward ? (from.segment + 1) % count : from.segment;
  for (std::size_t guard = 0; guard <= count && travelled < span; ++guard) {
    path.push_back(ring[vertex]);
    const std::size_t next =
        forward ? (vertex + 1) % count : (vertex + count - 1) % count;
    travelled += distance_world(ring[vertex], ring[next]);
    vertex = next;
  }
  path.push_back(to.point);
  return path;
}

// Drops vertices closer together than `world_spacing`, so the shortcut pass
// that follows does a bounded amount of work on a contour that may carry
// hundreds of offset vertices.
Path64 decimate(const Path64& path, double world_spacing, std::size_t limit) {
  if (path.size() <= 2) return path;
  const double total = path_length_world(path);
  double spacing = std::max(world_spacing, total / static_cast<double>(limit));
  Path64 out;
  out.push_back(path.front());
  for (std::size_t i = 1; i + 1 < path.size(); ++i)
    if (distance_world(out.back(), path[i]) >= spacing) out.push_back(path[i]);
  out.push_back(path.back());
  return out;
}

// Pulls the walk straight wherever a shortcut stays clear, longest jump first.
Path64 shorten(const Path64& path, const Paths64& forbidden, double tolerance) {
  if (path.size() <= 2) return path;
  Path64 out;
  out.push_back(path.front());
  std::size_t i = 0;
  while (i + 1 < path.size()) {
    std::size_t next = i + 1;
    for (std::size_t j = path.size() - 1; j > i + 1; --j) {
      if (segment_avoids(path[i], path[j], forbidden, tolerance)) {
        next = j;
        break;
      }
    }
    out.push_back(path[next]);
    i = next;
  }
  return out;
}

// Routes from `from` to `to` around whatever blocks the straight segment, by
// following the boundary of the region the tool may travel through. Returns
// nothing when no ring carries both ends or every walk exceeds `budget`.
std::optional<Path64> route_around(const Point64& from, const Point64& to,
                                   const Paths64& travel, const Paths64& forbidden,
                                   double tolerance, double budget) {
  // `tolerance` is in integer units because it is compared against clipper
  // output, while the walk is measured in world units; the shortcut spacing has
  // to be converted or it swallows the whole contour.
  const double world_spacing = to_world(static_cast<std::int64_t>(tolerance)) * 4.0;
  std::optional<Path64> best;
  double best_length = budget;
  for (const auto& ring : travel) {
    if (ring.size() < 3) continue;
    const RingProjection start = project_onto_ring(ring, from);
    const RingProjection finish = project_onto_ring(ring, to);
    const auto cumulative = ring_arc_lengths(ring);
    const double total = cumulative.back();
    if (total <= 0.0) continue;
    double forward_span = ring_arc_of(cumulative, finish) - ring_arc_of(cumulative, start);
    if (forward_span < 0.0) forward_span += total;
    for (const bool forward : {true, false}) {
      const double span = forward ? forward_span : total - forward_span;
      // The walk cannot beat what is already in hand even before the ends are
      // joined on, so skip it rather than build it.
      if (span + start.distance + finish.distance >= best_length) continue;
      Path64 walk = walk_ring(ring, cumulative, start, finish, forward, span);
      walk.insert(walk.begin(), from);
      walk.push_back(to);
      walk = shorten(decimate(walk, world_spacing, 48), forbidden, tolerance);
      bool clear = true;
      for (std::size_t i = 1; i < walk.size() && clear; ++i)
        clear = segment_avoids(walk[i - 1], walk[i], forbidden, tolerance);
      if (!clear) continue;
      const double length = path_length_world(walk);
      if (length < best_length) {
        best_length = length;
        best = std::move(walk);
      }
    }
  }
  return best;
}

// A point at least `clearance` inside cleared material, as close to `target` as
// one can be found. This is where a lift may set the tool back down without the
// descent itself having to cut.
std::optional<Point64> find_plunge_point(const Paths64& cleared, const Point64& target,
                                         double integer_clearance) {
  for (const double scale : {1.0, 0.5, 0.25}) {
    const Paths64 zone = InflatePaths(cleared, -integer_clearance * scale,
                                      JoinType::Round, EndType::Polygon);
    std::optional<Point64> best;
    double best_distance = std::numeric_limits<double>::max();
    for (const auto& ring : zone) {
      if (ring.size() < 3) continue;
      const RingProjection projection = project_onto_ring(ring, target);
      if (projection.distance < best_distance) {
        best_distance = projection.distance;
        best = projection.point;
      }
    }
    if (best) return best;
  }
  return std::nullopt;
}

struct Node {
  std::size_t generation = 0;
  std::size_t index = 0;
  std::size_t pending_parents = 0;
  bool cut = false;
};

}  // namespace

Toolpath build_toolpath(const WaveResult& wave, const FrontLineage& lineage,
                        const ToolpathOptions& options) {
  Toolpath toolpath;
  if (lineage.generations.empty()) return toolpath;
  if (lineage.generations.size() != wave.generations.size())
    throw std::invalid_argument("Lineage and wave generations do not match");

  // Link tests are run against offset output on the integer grid, so they need
  // the same order of slack the lineage uses when matching fronts.
  // How far a connecting move may stray outside cleared material. The fronts it
  // joins are offset output on the integer grid, so a move sliding along a wall
  // sits tens of units off the wall's true position and would otherwise read as
  // a gouge. Classification is flat over 32..1400 integer units on the sample
  // meshes -- below that quantization noise forces spurious retracts, and the
  // genuine pocket-to-pocket jumps stay retracts across the whole range -- so
  // this sits mid-plateau and scales with the stepover it must never let a link
  // cross.
  // How far inside cleared material a lift must set the tool down. Half a
  // stepover puts the plunge along the middle of the band the parent front
  // opened rather than against either of its edges.
  const double plunge_clearance =
      static_cast<double>(to_integer(wave.stepover)) * 0.5;
  const double tolerance = std::clamp(
      static_cast<double>(to_integer(wave.stepover)) * 0.05, 32.0, 500.0);

  std::vector<std::size_t> generation_offset(lineage.generations.size() + 1, 0);
  std::vector<Node> nodes;
  for (std::size_t generation = 0; generation < lineage.generations.size(); ++generation) {
    generation_offset[generation] = nodes.size();
    const auto& fronts = lineage.generations[generation].fronts;
    for (std::size_t index = 0; index < fronts.size(); ++index)
      nodes.push_back({generation, index, fronts[index].parents.size(), false});
  }
  generation_offset.back() = nodes.size();
  if (nodes.empty()) return toolpath;

  // A band ring is only credited as cleared once every front sharing it has
  // been cut, so a link is never routed through material a sibling front still
  // has to remove.
  std::vector<std::vector<std::size_t>> band_pending(lineage.generations.size());
  for (std::size_t generation = 0; generation < lineage.generations.size(); ++generation) {
    band_pending[generation].assign(lineage.generations[generation].bands.size(), 0);
    for (const auto& front : lineage.generations[generation].fronts)
      if (front.band < band_pending[generation].size())
        ++band_pending[generation][front.band];
  }

  std::vector<std::size_t> ready;
  for (std::size_t node = 0; node < nodes.size(); ++node)
    if (nodes[node].pending_parents == 0) ready.push_back(node);

  Rect64 world = GetBounds(wave.stock);
  const std::int64_t margin = std::max<std::int64_t>(
      16, to_integer(wave.stepover) * 4);
  world.left -= margin;
  world.top -= margin;
  world.right += margin;
  world.bottom += margin;

  Paths64 cleared;
  // Rebuilt whenever `cleared` grows, which is once per front rather than once
  // per link.
  Paths64 cleared_forbidden = forbidden_region(cleared, world, tolerance);
  Point64 position;
  bool has_position = false;

  while (!ready.empty()) {
    // Greedy nearest-ready. Because a front's children are the fronts closest
    // to it, this walks each branch to its end before backtracking, which keeps
    // links local without needing an explicit depth-first traversal.
    std::size_t choice = 0;
    if (has_position) {
      double best = std::numeric_limits<double>::max();
      for (std::size_t i = 0; i < ready.size(); ++i) {
        const auto& node = nodes[ready[i]];
        const auto& front = lineage.generations[node.generation].fronts[node.index];
        const double distance = nearest_vertex_distance(front.path, position);
        if (distance < best) {
          best = distance;
          choice = i;
        }
      }
    }
    const std::size_t node_index = ready[choice];
    ready.erase(ready.begin() + static_cast<std::ptrdiff_t>(choice));
    Node& node = nodes[node_index];
    const auto& front = lineage.generations[node.generation].fronts[node.index];
    node.cut = true;

    if (front.path.size() >= 2) {
      Path64 cut;
      if (front.closed) {
        // One winding for every loop, so the cutter stays on the same side of
        // the material throughout. Which winding is climb is a machining
        // decision that belongs with the post stage.
        Path64 ring = front.path;
        if (Area(ring) < 0.0) std::reverse(ring.begin(), ring.end());
        cut = close_ring_from(ring, has_position ? nearest_vertex(ring, position) : 0);
      } else {
        cut = front.path;
        if (has_position &&
            distance_world(cut.back(), position) < distance_world(cut.front(), position))
          std::reverse(cut.begin(), cut.end());
      }

      const Point64 entry = cut.front();
      if (!has_position) {
        // Nothing is cleared yet, so the opening descent is always into solid
        // material and has to be ramped.
        toolpath.moves.push_back({MoveKind::Entry, Path64{entry, entry}, {}, 0.0});
        ++toolpath.stats.entry_count;
      } else if (position != entry) {
        const Paths64& band =
            front.band < lineage.generations[node.generation].bands.size()
                ? lineage.generations[node.generation].bands[front.band]
                : cleared;
        const Paths64 travel = Union(cleared, band, FillRule::NonZero);
        const Paths64 travel_forbidden = forbidden_region(travel, world, tolerance);

        const auto emit = [&](Path64 path, bool engaged) {
          const double length = path_length_world(path);
          if (engaged) {
            ++toolpath.stats.step_count;
            toolpath.stats.step_length += length;
          } else {
            ++toolpath.stats.link_count;
            toolpath.stats.link_length += length;
          }
          toolpath.moves.push_back(
              {engaged ? MoveKind::Step : MoveKind::Link, std::move(path), {}, length});
        };
        const auto stays_clear = [&](const Path64& path, const Paths64& forbidden) {
          for (std::size_t i = 1; i < path.size(); ++i)
            if (!segment_avoids(path[i - 1], path[i], forbidden, tolerance)) return false;
          return true;
        };

        // Free travel first: a move that never leaves cleared material costs
        // nothing in engagement, so it is always preferable to an engaged step.
        if (segment_avoids(position, entry, cleared_forbidden, tolerance)) {
          emit(Path64{position, entry}, false);
        } else if (segment_avoids(position, entry, travel_forbidden, tolerance)) {
          emit(Path64{position, entry}, true);
        } else if (const auto route = route_around(
                       position, entry, travel, travel_forbidden, tolerance,
                       std::max(options.detour_limit * distance_world(position, entry),
                                2.0 * wave.stepover))) {
          // Following a contour around the obstruction beats lifting over it.
          emit(*route, !stays_clear(*route, cleared_forbidden));
          ++toolpath.stats.detour_count;
        } else if (const auto plunge = find_plunge_point(cleared, entry, plunge_clearance);
                   plunge && stays_clear(Path64{*plunge, entry}, travel_forbidden)) {
          // Set down where material is already gone, then cut in from there, so
          // the descent itself never has to remove anything.
          const Path64 rapid{position, *plunge};
          const double length = path_length_world(rapid);
          toolpath.moves.push_back({MoveKind::Retract, rapid, {}, length});
          ++toolpath.stats.retract_count;
          toolpath.stats.retract_length += length;
          if (*plunge != entry)
            emit(Path64{*plunge, entry}, !stays_clear(Path64{*plunge, entry}, cleared_forbidden));
        } else {
          const Path64 rapid{position, entry};
          const double length = path_length_world(rapid);
          toolpath.moves.push_back({MoveKind::Entry, rapid, {}, length});
          ++toolpath.stats.entry_count;
          toolpath.stats.entry_length += length;
        }
      }

      const double cut_length = path_length_world(cut);
      position = cut.back();
      has_position = true;
      toolpath.moves.push_back({MoveKind::Cut, std::move(cut),
                                {node.generation, node.index}, cut_length});
      ++toolpath.stats.cut_count;
      toolpath.stats.cut_length += cut_length;
    }

    auto& pending = band_pending[node.generation];
    if (front.band < pending.size() && --pending[front.band] == 0) {
      cleared = Union(cleared, lineage.generations[node.generation].bands[front.band],
                      FillRule::NonZero);
      cleared_forbidden = forbidden_region(cleared, world, tolerance);
    }

    if (node.generation + 1 >= lineage.generations.size()) continue;
    for (const auto child : front.children) {
      const std::size_t child_node = generation_offset[node.generation + 1] + child;
      if (child_node >= nodes.size()) continue;
      if (nodes[child_node].pending_parents > 0 &&
          --nodes[child_node].pending_parents == 0)
        ready.push_back(child_node);
    }
  }

  for (const auto& node : nodes)
    if (!node.cut) ++toolpath.stats.unreachable_fronts;
  return toolpath;
}

double toolpath_length(const Toolpath& toolpath) {
  double total = 0.0;
  for (const auto& move : toolpath.moves) total += move.length;
  return total;
}

double toolpath_move_start(const Toolpath& toolpath, std::size_t move) {
  double total = 0.0;
  for (std::size_t i = 0; i < move && i < toolpath.moves.size(); ++i)
    total += toolpath.moves[i].length;
  return total;
}

ToolpathCursor toolpath_at(const Toolpath& toolpath, double distance) {
  ToolpathCursor cursor;
  if (toolpath.moves.empty()) return cursor;
  distance = std::max(0.0, distance);
  double travelled = 0.0;
  for (std::size_t i = 0; i < toolpath.moves.size(); ++i) {
    const auto& move = toolpath.moves[i];
    const bool last = i + 1 == toolpath.moves.size();
    if (distance <= travelled + move.length || last) {
      cursor.move = i;
      cursor.kind = move.kind;
      cursor.move_distance = std::clamp(distance - travelled, 0.0, move.length);
      cursor.position = move.path.empty() ? Point64()
                                          : point_along(move.path, cursor.move_distance);
      return cursor;
    }
    travelled += move.length;
  }
  return cursor;
}

const char* move_kind_name(MoveKind kind) {
  switch (kind) {
    case MoveKind::Cut: return "cut";
    case MoveKind::Step: return "step";
    case MoveKind::Link: return "link";
    case MoveKind::Retract: return "retract";
    case MoveKind::Entry: return "entry";
  }
  return "unknown";
}

}  // namespace wave
