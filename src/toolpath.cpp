#include "toolpath.h"

#include "routing.h"

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
  // Eroding is what lets a move graze along a wall. It has a known cost: an
  // obstacle thinner than twice the tolerance disappears from the test, so a
  // rib of the part narrower than that does not block a route. Widening the
  // tolerance makes that worse, which is the reason it stays small.
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
  // Clipper will not clip a zero-length open path, and a segment that goes
  // nowhere cannot pass through anything. This matters: the connectors joining
  // a routed walk to the ring it follows are degenerate whenever the endpoint
  // already lies on that ring, which is the usual case.
  if (a == b) return true;
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

bool stays_clear(const Path64& path, const Paths64& forbidden, double tolerance) {
  for (std::size_t i = 1; i < path.size(); ++i)
    if (!segment_avoids(path[i - 1], path[i], forbidden, tolerance)) return false;
  return true;
}

// Non-zero containment across a whole set of rings, so a point sitting in a
// hole reads as outside the way the fills everywhere else in this file do.
bool inside_paths(const Paths64& paths, const Point64& point) {
  int winding = 0;
  for (const auto& ring : paths) {
    if (ring.size() < 3) continue;
    const PointInPolygonResult result = PointInPolygon(point, ring);
    if (result == PointInPolygonResult::IsOn) return true;
    if (result == PointInPolygonResult::IsInside)
      winding += Area(ring) >= 0.0 ? 1 : -1;
  }
  return winding != 0;
}

// A point at least `clearance` inside cleared material, as close to `target` as
// one can be found. This is where a lift may set the tool back down without the
// descent itself having to cut.
std::optional<Point64> find_plunge_point(const Paths64& cleared, const Point64& target,
                                         double integer_clearance) {
  for (const double scale : {1.0, 0.5, 0.25}) {
    const Paths64 zone = InflatePaths(cleared, -integer_clearance * scale,
                                      JoinType::Round, EndType::Polygon);
    // Somewhere the tool already stands clear needs no landing of its own.
    // Projecting onto the boundary regardless would set it down on the nearest
    // edge of the clearance zone and then feed it across to a spot it could
    // have descended straight onto, which is the stray approach that made the
    // opening move of a lead-in path cut anything at all.
    if (scale == 1.0 && inside_paths(zone, target)) return target;
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
  bool processed = false;
};

bool cutter_sweep_intersects_stock(const LeadingFront& front,
                                   const Paths64& stock,
                                   double tool_radius) {
  if (front.path.size() < 2 || stock.empty() || !(tool_radius > 0.0))
    return false;
  const Paths64 sweep = InflatePaths(
      Paths64{front.path},
      static_cast<double>(std::max<std::int64_t>(1, to_integer(tool_radius))),
      JoinType::Round, front.closed ? EndType::Joined : EndType::Round);
  if (sweep.empty()) return false;
  // A tangent has no intersection polygon and therefore remains an air pass;
  // any positive-area overlap is real engagement, however small.
  return !Intersect(sweep, stock, FillRule::NonZero).empty();
}


}  // namespace

static void append_toolpath(Toolpath& toolpath, const WaveResult& wave,
                            const FrontLineage& lineage, double tool_radius,
                            double floor_z, double rapid_plane,
                            double rapid_feed, double traversal_feed,
                            double rapid_feed_z,
                            const LayerTopology* existing_topology) {
  if (lineage.generations.empty()) return;
  if (lineage.generations.size() != wave.generations.size())
    throw std::invalid_argument("Lineage and wave generations do not match");
  const double vertical_feed = rapid_feed_z > 0.0 ? rapid_feed_z : rapid_feed;
  if (!(rapid_feed > 0.0) || !(traversal_feed > 0.0) || rapid_feed_z < 0.0 ||
      !std::isfinite(rapid_feed_z) ||
      !std::isfinite(floor_z) || !std::isfinite(rapid_plane))
    throw std::invalid_argument("Toolpath routing feeds and heights are invalid");
  LayerTopology built_topology;
  if (!existing_topology) built_topology = build_layer_topology(lineage);
  const LayerTopology& layer_topology =
      existing_topology ? *existing_topology : built_topology;
  if (layer_topology.front_nodes.size() != lineage.generations.size())
    throw std::invalid_argument("Layer topology and lineage do not match");

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
  // How far inside cleared material a lift must set the tool down. What the
  // descent sweeps is a disc of the tool's radius, so that radius is the
  // clearance, and anything less is a plunge that cuts on the way in. Half a
  // stepover is the floor rather than the figure: it puts the landing along the
  // middle of the band the parent front opened rather than against an edge, and
  // it is what this used to use outright, back when the radius was not known
  // here and every plan came out the same for every cutter.
  const double plunge_clearance =
      std::max(static_cast<double>(
                   to_integer(tool_radius > 0.0 ? tool_radius
                                                : wave.stepover * 0.5)),
               static_cast<double>(to_integer(wave.stepover)) * 0.5) +
      tolerance;
  const double effective_tool_radius =
      tool_radius > 0.0 ? tool_radius : wave.stepover * 0.5;

  std::vector<std::size_t> generation_offset(lineage.generations.size() + 1, 0);
  std::vector<Node> nodes;
  for (std::size_t generation = 0; generation < lineage.generations.size(); ++generation) {
    generation_offset[generation] = nodes.size();
    const auto& fronts = lineage.generations[generation].fronts;
    for (std::size_t index = 0; index < fronts.size(); ++index)
      nodes.push_back({generation, index, fronts[index].parents.size(), false});
  }
  generation_offset.back() = nodes.size();
  if (nodes.empty()) return;

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

  // Wide enough to hold the fronts, which a lead-in puts outside the stock, and
  // then the travel space around them. The air outside the stock is bounded by
  // this box, so it also has to leave a frame that survives being eroded by the
  // plunge clearance from both sides -- otherwise a wide cutter finds no landing
  // beside the stock and settles for one of the relaxed fallbacks instead.
  Rect64 world = GetBounds(wave.stock);
  const std::int64_t margin = std::max<std::int64_t>(
      16, to_integer(wave.stepover) * 4 + to_integer(wave.lead_in) +
              static_cast<std::int64_t>(plunge_clearance * 2.0 + tolerance * 2.0));
  world.left -= margin;
  world.top -= margin;
  world.right += margin;
  world.bottom += margin;

  const Path64 world_box{
      Point64(world.left, world.top), Point64(world.right, world.top),
      Point64(world.right, world.bottom), Point64(world.left, world.bottom)};
  const Paths64 stock_area =
      wave.stock.empty() ? Paths64{wave.stock_boundary} : wave.stock;

  // Lead-in propagation contains fronts wholly in the air, and those emit
  // nothing. Behind them, a front whose cutter reaches the stock cuts -- the
  // first one included, because the caller sized the run-in so that the first
  // touching front is the one taking the designed bite of a stepover less the
  // stock allowance.
  //
  // The seed ring is the exception. It is a one-unit ring on the edge of the
  // approach region and not a pass at all, and where the subject notches that
  // region the ring runs along the stock wall itself -- so it would cut, at
  // very nearly a full slot. Without a lead-in there is no approach and the
  // seed is the stock wall, which is the pass the operator asked for by
  // turning the run-in off.
  //
  // What stood here instead was a rule skipping the first touching front on
  // every branch, as incidental contact not worth a whole contour. That held
  // when the lead-in was rounded up to a whole number of stepovers and first
  // contact landed wherever the rounding put it; it does not now that the
  // run-in is honoured exactly. It happened to come to the same thing wherever
  // a seed piece already lay against the wall, which is most part geometry --
  // and on a plain blank with none of the part at that depth, where no seed
  // piece touches, it threw the designed first bite away and handed it to the
  // front a stepover further in, which took nearly a full slot.
  std::vector<bool> touches_stock(nodes.size(), false);
  std::vector<bool> emits_cut(nodes.size(), false);
  std::vector<std::vector<bool>> band_emits_cut(lineage.generations.size());
  for (std::size_t generation = 0; generation < lineage.generations.size();
       ++generation) {
    band_emits_cut[generation].assign(
        lineage.generations[generation].bands.size(), false);
    for (std::size_t index = 0;
         index < lineage.generations[generation].fronts.size(); ++index) {
      const std::size_t node = generation_offset[generation] + index;
      const LeadingFront& front =
          lineage.generations[generation].fronts[index];
      touches_stock[node] = cutter_sweep_intersects_stock(
          front, stock_area, effective_tool_radius);
      emits_cut[node] = touches_stock[node] &&
                        (wave.lead_in_generations == 0 || generation > 0);
      if (emits_cut[node] && front.band < band_emits_cut[generation].size())
        band_emits_cut[generation][front.band] = true;
    }
  }

  // The stock stands in air, so the space beside it is already empty: the tool
  // can be lowered there without cutting, and can travel there at depth. This
  // is what gives the opening move somewhere to plunge -- it lands beside the
  // stock and feeds in horizontally instead of ramping into solid material --
  // and it lets later transitions run round the outside of the stock rather
  // than lift over it.
  // Held off the stock wall by the same slack the clearance tests allow, or a
  // move grazing along the wall would read as travelling in the air outside it
  // and be reported as free when it is cutting.
  const Paths64 air = Difference(
      Paths64{world_box},
      InflatePaths(Union(stock_area, wave.subject, FillRule::NonZero),
                   tolerance * 2.0, JoinType::Miter, EndType::Polygon),
      FillRule::NonZero);

  Paths64 cleared = toolpath.cleared.empty()
                        ? air
                        : Union(toolpath.cleared, air, FillRule::NonZero);
  // Rebuilt whenever `cleared` grows, which is once per front rather than once
  // per link.
  Paths64 cleared_forbidden = forbidden_region(cleared, world, tolerance);
  Point64 position;
  bool has_position = false;
  for (auto move = toolpath.moves.rbegin(); move != toolpath.moves.rend();
       ++move) {
    if (move->path.empty()) continue;
    position = move->path.back();
    has_position = true;
    break;
  }
  std::optional<FrontId> previous_front;

  struct LiftPlan { Path64 path; };

  // A disconnected move is deliberately simple: straight up to the single
  // safe plane, direct XY there, then down. A 2D slice is not used to invent a
  // controller trajectory or a lower obstacle-skimming rapid.
  const auto plan_lift = [&](const Point64& origin, const Point64& target) {
    return LiftPlan{Path64{origin, target}};
  };

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
    node.processed = true;

    if (emits_cut[node_index]) {
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
        bool reverse_cut =
            has_position && distance_world(cut.back(), position) <
                                distance_world(cut.front(), position);
        // Inside one topology node, choose the target end by the shared
        // boundary walk rather than by its straight-line proximity. This is
        // the common C-shaped transition the endpoint provenance was retained
        // to resolve.
        if (previous_front && previous_front->generation <
                                  layer_topology.front_nodes.size() &&
            node.generation < layer_topology.front_nodes.size()) {
          const std::size_t previous_node =
              layer_topology.front_nodes[previous_front->generation]
                                        [previous_front->index];
          const std::size_t target_node =
              layer_topology.front_nodes[node.generation][node.index];
          const LeadingFront& previous =
              lineage.generations[previous_front->generation]
                  .fronts[previous_front->index];
          if (previous_node == target_node && !previous.closed &&
              !previous.path.empty()) {
            const std::size_t previous_side =
                distance_world(position, previous.path.front()) <=
                        distance_world(position, previous.path.back())
                    ? 0
                    : 1;
            double best_boundary = std::numeric_limits<double>::infinity();
            for (std::size_t target_side = 0; target_side < 2; ++target_side) {
              const BoundaryContact& from =
                  previous.boundary_contacts[previous_side];
              const BoundaryContact& to = front.boundary_contacts[target_side];
              if (!from.valid || !to.valid || from.component != to.component)
                continue;
              const double length =
                  path_length_world(boundary_arc(lineage, from, to));
              if (length < best_boundary) {
                best_boundary = length;
                reverse_cut = target_side == 1;
              }
            }
          }
        }
        if (reverse_cut) std::reverse(cut.begin(), cut.end());
      }

      const Point64 entry = cut.front();
      if (!has_position) {
        has_position = true;
        // The air beside the stock is always somewhere the tool can come down,
        // so the opening move is a plunge in free space followed by an ordinary
        // feed in. Only a part that leaves no such landing forces a ramp.
        if (const auto plunge = find_plunge_point(cleared, entry, plunge_clearance)) {
          position = *plunge;
          toolpath.moves.push_back({MoveKind::Retract, Path64{position, position}, {}, 0.0});
          ++toolpath.stats.retract_count;
        } else {
          position = entry;
          toolpath.moves.push_back({MoveKind::Entry, Path64{entry, entry}, {}, 0.0});
          ++toolpath.stats.entry_count;
        }
      }
      if (position != entry) {
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

        std::optional<Path64> preferred_boundary;
        if (previous_front && !front.closed) {
          const std::size_t previous_node =
              layer_topology.front_nodes[previous_front->generation]
                                        [previous_front->index];
          const std::size_t target_node =
              layer_topology.front_nodes[node.generation][node.index];
          const LeadingFront& previous =
              lineage.generations[previous_front->generation]
                  .fronts[previous_front->index];
          if (previous_node == target_node && !previous.closed) {
            const std::size_t previous_side =
                distance_world(position, previous.path.front()) <=
                        distance_world(position, previous.path.back())
                    ? 0
                    : 1;
            const std::size_t target_side =
                distance_world(entry, front.path.front()) <=
                        distance_world(entry, front.path.back())
                    ? 0
                    : 1;
            const BoundaryContact& from =
                previous.boundary_contacts[previous_side];
            const BoundaryContact& to = front.boundary_contacts[target_side];
            if (from.valid && to.valid) {
              if (from.component != to.component)
                throw std::runtime_error(
                    "Wavefront topology error: adjacent fronts in one node "
                    "name different boundary components");
              Path64 path = boundary_arc(lineage, from, to);
              if (path.front() != position) path.insert(path.begin(), position);
              if (path.back() != entry) path.push_back(entry);
              preferred_boundary = std::move(path);
            }
          }
        }

        // Free travel first: a move that never leaves cleared material costs
        // nothing in engagement, so it is always preferable to an engaged step.
        if (preferred_boundary) {
          if (!stays_clear(*preferred_boundary, travel_forbidden, tolerance))
            throw std::runtime_error(
                "Wavefront topology error: shared-boundary transition leaves "
                "the legal propagation band");
          emit(*preferred_boundary,
               !stays_clear(*preferred_boundary, cleared_forbidden, tolerance));
        } else if (segment_avoids(position, entry, cleared_forbidden, tolerance)) {
          emit(Path64{position, entry}, false);
        } else if (segment_avoids(position, entry, travel_forbidden, tolerance)) {
          emit(Path64{position, entry}, true);
        } else {
          // A multiplier against the blocked straight line is not a meaningful
          // cost: it can admit arbitrarily shaped detours and change decisions
          // when a slice moves by a fraction of a unit. Build both realizable
          // alternatives and compare all of the XY motion each one requires.
          //
          // The travel region is clipped to a finite box so Clipper can hold
          // the air outside the stock, and that box is an implementation
          // boundary rather than a physical one. A shortest path cannot be led
          // astray by it: a route turns only at corners the free space wraps
          // around, and every corner of the box is convex seen from inside, so
          // no route ever runs out to one.
          const Router router(travel, tolerance);
          const auto detour = router.route(position, entry);
          const double detour_distance =
              detour ? path_length_world(*detour)
                     : std::numeric_limits<double>::infinity();
          const double detour_minutes = detour_distance / traversal_feed;

          std::optional<Point64> plunge =
              find_plunge_point(cleared, entry, plunge_clearance);
          Path64 approach;
          std::optional<LiftPlan> lift;
          double lift_minutes = std::numeric_limits<double>::infinity();
          if (plunge) {
            approach = Path64{*plunge, entry};
            if (!stays_clear(approach, travel_forbidden, tolerance)) {
              plunge.reset();
              approach.clear();
            } else {
              lift = plan_lift(position, *plunge);
              const double vertical =
                  2.0 * std::abs(rapid_plane - floor_z);
              lift_minutes =
                  path_length_world(lift->path) / rapid_feed +
                  vertical / vertical_feed +
                  path_length_world(approach) / traversal_feed;
            }
          }

          if (detour && detour_minutes < lift_minutes) {
            emit(*detour, !stays_clear(*detour, cleared_forbidden, tolerance));
            ++toolpath.stats.detour_count;
          } else if (plunge && lift) {
            // Set down where material is already gone, then cut in from there,
            // so the descent itself never has to remove anything.
            const double rapid_length = path_length_world(lift->path);
            toolpath.moves.push_back(
                {MoveKind::Retract, std::move(lift->path), {}, rapid_length});
            ++toolpath.stats.retract_count;
            toolpath.stats.retract_length += rapid_length;
            if (*plunge != entry) {
              const bool engaged =
                  !stays_clear(approach, cleared_forbidden, tolerance);
              emit(std::move(approach), engaged);
            }
          } else if (detour) {
            // No cleared landing exists, so the valid at-depth route avoids an
            // entry into solid material even when it is long.
            emit(*detour, !stays_clear(*detour, cleared_forbidden, tolerance));
            ++toolpath.stats.detour_count;
          } else {
            const LiftPlan rapid = plan_lift(position, entry);
            const double length = path_length_world(rapid.path);
            toolpath.moves.push_back({MoveKind::Entry, rapid.path, {}, length});
            ++toolpath.stats.entry_count;
            toolpath.stats.entry_length += length;
          }
        }
      }

      const double cut_length = path_length_world(cut);
      position = cut.back();
      has_position = true;
      toolpath.moves.push_back({MoveKind::Cut, std::move(cut),
                                {node.generation, node.index}, cut_length});
      ++toolpath.stats.cut_count;
      toolpath.stats.cut_length += cut_length;
      previous_front = FrontId{node.generation, node.index};
    }

    auto& pending = band_pending[node.generation];
    if (front.band < pending.size() && --pending[front.band] == 0) {
      const Paths64& band =
          lineage.generations[node.generation].bands[front.band];
      // A band represented only by skipped lead-in/first-contact fronts is not
      // credited as machined stock. Retain its truly exterior portion for
      // entry routing; the +1 child supplies the band that actually cuts in.
      const Paths64 cleared_band =
          band_emits_cut[node.generation][front.band]
              ? band
              : Difference(band, stock_area, FillRule::NonZero);
      cleared = Union(cleared, cleared_band, FillRule::NonZero);
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
    if (!node.processed) ++toolpath.stats.unreachable_fronts;
  toolpath.cleared = std::move(cleared);
}

Toolpath build_toolpath(const WaveResult& wave, const FrontLineage& lineage,
                        double tool_radius, double floor_z,
                        double rapid_plane, double rapid_feed,
                        double traversal_feed, double rapid_feed_z,
                        const LayerTopology* existing_topology) {
  Toolpath toolpath;
  append_toolpath(toolpath, wave, lineage, tool_radius, floor_z, rapid_plane,
                  rapid_feed, traversal_feed, rapid_feed_z,
                  existing_topology);
  return toolpath;
}

Toolpath build_toolpath(const std::vector<WavefrontStage>& stages,
                        double tool_radius, double floor_z,
                        double rapid_plane, double rapid_feed,
                        double traversal_feed, double rapid_feed_z) {
  Toolpath toolpath;
  for (const WavefrontStage& stage : stages)
    append_toolpath(toolpath, stage.wave, stage.lineage, tool_radius, floor_z,
                    rapid_plane, rapid_feed, traversal_feed, rapid_feed_z,
                    &stage.topology);
  return toolpath;
}

Toolpath resample_toolpath(const Toolpath& source, double pitch) {
  if (pitch < 1.0 / kIntegerScale || !std::isfinite(pitch))
    throw std::invalid_argument(
        "Path pitch must be at least the 0.001 mm geometry resolution");

  const auto engaged = [](MoveKind kind) {
    return kind == MoveKind::Cut || kind == MoveKind::Step ||
           kind == MoveKind::Link;
  };
  struct Span {
    Point64 from;
    Point64 to;
    double begin = 0.0;
    double end = 0.0;
    MoveKind kind = MoveKind::Cut;
    FrontId front;
  };
  const auto same_source = [](const ToolpathMove& move, const Span& span) {
    if (move.kind != span.kind) return false;
    return move.kind != MoveKind::Cut ||
           (move.front.generation == span.front.generation &&
            move.front.index == span.front.index);
  };

  Toolpath result;
  result.cleared = source.cleared;
  result.stats.detour_count = source.stats.detour_count;
  result.stats.lifts_over_subject = source.stats.lifts_over_subject;
  result.stats.unreachable_fronts = source.stats.unreachable_fronts;

  std::size_t first = 0;
  while (first < source.moves.size()) {
    if (!engaged(source.moves[first].kind)) {
      result.moves.push_back(source.moves[first]);
      ++first;
      continue;
    }

    std::size_t last = first + 1;
    while (last < source.moves.size() && engaged(source.moves[last].kind) &&
           !source.moves[last - 1].path.empty() &&
           !source.moves[last].path.empty() &&
           source.moves[last - 1].path.back() ==
               source.moves[last].path.front())
      ++last;

    std::vector<Span> spans;
    double total = 0.0;
    for (std::size_t move_index = first; move_index < last; ++move_index) {
      const ToolpathMove& move = source.moves[move_index];
      for (std::size_t point = 1; point < move.path.size(); ++point) {
        const double length =
            distance_world(move.path[point - 1], move.path[point]);
        if (!(length > 0.0)) continue;
        spans.push_back({move.path[point - 1], move.path[point], total,
                         total + length, move.kind, move.front});
        total += length;
      }
    }
    if (spans.empty()) {
      for (std::size_t move_index = first; move_index < last; ++move_index)
        result.moves.push_back(source.moves[move_index]);
      first = last;
      continue;
    }

    const auto point_at = [&](double distance) {
      distance = std::clamp(distance, 0.0, total);
      const auto found = std::lower_bound(
          spans.begin(), spans.end(), distance,
          [](const Span& span, double value) { return span.end < value; });
      const Span& span = found == spans.end() ? spans.back() : *found;
      const double length = span.end - span.begin;
      const double ratio =
          length > 0.0 ? std::clamp((distance - span.begin) / length, 0.0, 1.0)
                       : 0.0;
      return lerp_point(span.from, span.to, ratio);
    };

    std::vector<double> samples{0.0};
    for (double distance = pitch; distance < total - 1e-9; distance += pitch)
      samples.push_back(distance);
    samples.push_back(total);

    std::size_t span_index = 0;
    for (std::size_t sample = 1; sample < samples.size(); ++sample) {
      const double midpoint = (samples[sample - 1] + samples[sample]) * 0.5;
      while (span_index + 1 < spans.size() &&
             midpoint > spans[span_index].end)
        ++span_index;
      const Span& span = spans[span_index];
      const Point64 from = point_at(samples[sample - 1]);
      const Point64 to = point_at(samples[sample]);
      if (from == to) continue;
      if (!result.moves.empty() && same_source(result.moves.back(), span) &&
          result.moves.back().path.back() == from) {
        result.moves.back().path.push_back(to);
        result.moves.back().length += distance_world(from, to);
      } else {
        ToolpathMove move;
        move.kind = span.kind;
        move.front = span.front;
        move.path = {from, to};
        move.length = distance_world(from, to);
        result.moves.push_back(std::move(move));
      }
    }
    first = last;
  }

  for (const ToolpathMove& move : result.moves) {
    switch (move.kind) {
      case MoveKind::Cut:
        ++result.stats.cut_count;
        result.stats.cut_length += move.length;
        break;
      case MoveKind::Step:
        ++result.stats.step_count;
        result.stats.step_length += move.length;
        break;
      case MoveKind::Link:
        ++result.stats.link_count;
        result.stats.link_length += move.length;
        break;
      case MoveKind::Retract:
        ++result.stats.retract_count;
        result.stats.retract_length += move.length;
        break;
      case MoveKind::Entry:
        ++result.stats.entry_count;
        result.stats.entry_length += move.length;
        break;
    }
  }
  return result;
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
