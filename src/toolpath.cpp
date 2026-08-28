#include "toolpath.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
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

struct Node {
  std::size_t generation = 0;
  std::size_t index = 0;
  std::size_t pending_parents = 0;
  bool cut = false;
};

}  // namespace

Toolpath build_toolpath(const WaveResult& wave, const FrontLineage& lineage) {
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
        toolpath.moves.push_back({MoveKind::Retract, Path64{entry, entry}, {}, 0.0});
        ++toolpath.stats.retract_count;
      } else if (position != entry) {
        const Path64 link{position, entry};
        const double length = path_length_world(link);
        // Free travel first: a move that never leaves cleared material costs
        // nothing in engagement, so it is always preferable to an engaged step.
        if (segment_avoids(position, entry, cleared_forbidden, tolerance)) {
          toolpath.moves.push_back({MoveKind::Link, link, {}, length});
          ++toolpath.stats.link_count;
          toolpath.stats.link_length += length;
        } else if (front.band < lineage.generations[node.generation].bands.size() &&
                   segment_avoids(
                       position, entry,
                       forbidden_region(
                           Union(cleared,
                                 lineage.generations[node.generation].bands[front.band],
                                 FillRule::NonZero),
                           world, tolerance),
                       tolerance)) {
          toolpath.moves.push_back({MoveKind::Step, link, {}, length});
          ++toolpath.stats.step_count;
          toolpath.stats.step_length += length;
        } else {
          toolpath.moves.push_back({MoveKind::Retract, link, {}, length});
          ++toolpath.stats.retract_count;
          toolpath.stats.retract_length += length;
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
  }
  return "unknown";
}

}  // namespace wave
