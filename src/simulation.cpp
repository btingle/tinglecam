#include "simulation.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace wave {
namespace {

using namespace Clipper2Lib;

bool engaged(MoveKind kind) {
  return kind == MoveKind::Cut || kind == MoveKind::Step ||
         kind == MoveKind::Link;
}

Paths64 normalized(const Paths64& paths) {
  return Union(paths, FillRule::NonZero);
}

// Clipper Boolean operations already return non-overlapping, correctly
// oriented rings. Summing those signed rings accounts for holes directly; a
// second union would only rerun the scanline engine over the same solution.
double boolean_result_area(const Paths64& paths) {
  return std::abs(Area(paths)) / (kIntegerScale * kIntegerScale);
}

bool bounds_overlap(const Rect64& a, const Rect64& b) {
  return a.left <= b.right && b.left <= a.right && a.top <= b.bottom &&
         b.top <= a.bottom;
}

Paths64 tool_sweep(const Point64& from, const Point64& to, double radius) {
  if (from == to)
    return Paths64{Ellipse(from, radius, radius, 48)};
  return InflatePaths(Paths64{Path64{from, to}}, radius, JoinType::Round,
                      EndType::Round);
}

Point64 interpolate(const Point64& from, const Point64& to, double ratio) {
  ratio = std::clamp(ratio, 0.0, 1.0);
  return Point64(
      static_cast<std::int64_t>(std::llround(
          from.x + static_cast<long double>(to.x - from.x) * ratio)),
      static_cast<std::int64_t>(std::llround(
          from.y + static_cast<long double>(to.y - from.y) * ratio)));
}

double segment_length(const Point64& from, const Point64& to) {
  return std::hypot(to_world(to.x - from.x), to_world(to.y - from.y));
}

}  // namespace

double material_area(const Paths64& paths) {
  if (paths.empty()) return 0.0;
  const Paths64 united = normalized(paths);
  return std::abs(Area(united)) / (kIntegerScale * kIntegerScale);
}

SimulationResult simulate_material(const Paths64& raw_stock,
                                   const Paths64& raw_final_subject,
                                   const Toolpath& toolpath,
                                   double tool_radius,
                                   double simulation_stock_inflation) {
  if (raw_stock.empty()) throw std::invalid_argument("Simulation stock is empty");
  if (raw_final_subject.empty())
    throw std::invalid_argument("Final subject is empty");
  if (!(tool_radius > 0.0) || !std::isfinite(tool_radius))
    throw std::invalid_argument("Tool radius must be positive and finite");
  if (!std::isfinite(simulation_stock_inflation))
    throw std::invalid_argument("Simulation stock inflation must be finite");

  SimulationResult result;
  result.tool_radius = tool_radius;
  result.stock_inflation = simulation_stock_inflation;
  result.initial_stock = normalized(raw_stock);
  result.final_subject = normalized(raw_final_subject);

  const std::int64_t inflation = to_integer(simulation_stock_inflation);
  if (inflation != 0) {
    result.initial_stock = InflatePaths(
        result.initial_stock, static_cast<double>(inflation), JoinType::Miter,
        EndType::Polygon);
    result.initial_stock = normalized(result.initial_stock);
  }
  if (result.initial_stock.empty())
    throw std::invalid_argument("Simulation stock inflation removes all stock");

  const double radius_integer = static_cast<double>(to_integer(tool_radius));
  if (!(radius_integer > 0.0))
    throw std::invalid_argument("Tool radius is below the integer geometry resolution");

  Paths64 remaining = result.initial_stock;
  Paths64 gouges;
  const Rect64 final_subject_bounds = GetBounds(result.final_subject);
  result.moves.resize(toolpath.moves.size());
  result.remaining_after_move.reserve(toolpath.moves.size());

  for (std::size_t move_index = 0; move_index < toolpath.moves.size(); ++move_index) {
    const ToolpathMove& move = toolpath.moves[move_index];
    SimulatedMove& simulated = result.moves[move_index];

    if (engaged(move.kind)) {
      double move_distance = 0.0;
      for (std::size_t segment = 1; segment < move.path.size(); ++segment) {
        const Point64& segment_from = move.path[segment - 1];
        const Point64& segment_to = move.path[segment];
        const double length = segment_length(segment_from, segment_to);
        if (!(length > 0.0)) continue;

        // The caller normalizes cutting paths to the configured simulation
        // pitch. Evaluate those intervals exactly rather than imposing a
        // second, unrelated subdivision here.
        const std::size_t pieces = 1;
        for (std::size_t piece = 0; piece < pieces; ++piece) {
          const double begin_ratio = static_cast<double>(piece) / pieces;
          const double end_ratio = static_cast<double>(piece + 1) / pieces;
          MaterialStep step;
          step.from = interpolate(segment_from, segment_to, begin_ratio);
          step.to = interpolate(segment_from, segment_to, end_ratio);
          const double step_length = segment_length(step.from, step.to);
          if (!(step_length > 0.0)) continue;
          step.move_distance_begin = move_distance + length * begin_ratio;
          step.move_distance_end = move_distance + length * end_ratio;
          step.sweep = tool_sweep(step.from, step.to, radius_integer);

          const Paths64 removed =
              Intersect(remaining, step.sweep, FillRule::NonZero);
          Paths64 gouge;
          if (removed.empty()) {
            ++result.stats.empty_cutting_steps;
          } else if (bounds_overlap(GetBounds(removed), final_subject_bounds)) {
            gouge = Intersect(removed, result.final_subject,
                              FillRule::NonZero);
          } else {
            ++result.stats.gouge_bounds_rejections;
          }
          step.removed_area = boolean_result_area(removed);
          step.gouge_area = boolean_result_area(gouge);
          step.load = step.removed_area /
                      (2.0 * tool_radius * step_length);
          result.stats.maximum_load =
              std::max(result.stats.maximum_load, step.load);
          if (step.gouge_area > 0.0) {
            ++result.stats.gouging_steps;
            gouges.insert(gouges.end(), gouge.begin(), gouge.end());
          }
          ++result.stats.cutting_steps;
          if (!removed.empty())
            remaining = Difference(remaining, step.sweep, FillRule::NonZero);
          simulated.steps.push_back(std::move(step));
        }
        move_distance += length;
      }
    } else if (move.kind == MoveKind::Retract && !move.path.empty()) {
      const Point64& destination = move.path.back();
      const Paths64 footprint{
          Ellipse(destination, radius_integer, radius_integer, 48)};
      simulated.plunge_collision =
          Intersect(remaining, footprint, FillRule::NonZero);
      simulated.plunge_collision_area = material_area(simulated.plunge_collision);
      if (simulated.plunge_collision_area > 0.0)
        ++result.stats.unsafe_plunges;
    } else if (move.kind == MoveKind::Entry) {
      simulated.entry_unresolved = true;
      ++result.stats.unresolved_entries;
    }

    result.remaining_after_move.push_back(remaining);
  }

  result.remaining_stock = remaining;
  result.removed_material = Difference(result.initial_stock, remaining,
                                       FillRule::NonZero);
  result.gouged_subject = normalized(gouges);
  result.excess_material = Difference(remaining, result.final_subject,
                                      FillRule::NonZero);
  result.stats.initial_stock_area = boolean_result_area(result.initial_stock);
  result.stats.removed_area = boolean_result_area(result.removed_material);
  result.stats.remaining_area = boolean_result_area(result.remaining_stock);
  result.stats.excess_area = boolean_result_area(result.excess_material);
  result.stats.gouge_area = material_area(result.gouged_subject);
  return result;
}

Paths64 simulation_stock_at(const SimulationResult& simulation,
                            const Toolpath& toolpath,
                            double distance) {
  if (simulation.initial_stock.empty() || toolpath.moves.empty())
    return simulation.initial_stock;
  if (simulation.moves.size() != toolpath.moves.size() ||
      simulation.remaining_after_move.size() != toolpath.moves.size())
    throw std::invalid_argument("Simulation and toolpath moves do not match");

  const ToolpathCursor cursor = toolpath_at(toolpath, distance);
  Paths64 remaining = cursor.move == 0
                          ? simulation.initial_stock
                          : simulation.remaining_after_move[cursor.move - 1];
  const SimulatedMove& move = simulation.moves[cursor.move];
  const double radius_integer = static_cast<double>(to_integer(simulation.tool_radius));
  for (const auto& step : move.steps) {
    if (cursor.move_distance <= step.move_distance_begin) break;
    if (cursor.move_distance >= step.move_distance_end) {
      remaining = Difference(remaining, step.sweep, FillRule::NonZero);
      continue;
    }
    const double span = step.move_distance_end - step.move_distance_begin;
    const double ratio = span > 0.0
                             ? (cursor.move_distance - step.move_distance_begin) / span
                             : 0.0;
    const Point64 partial_to = interpolate(step.from, step.to, ratio);
    remaining = Difference(remaining,
                           tool_sweep(step.from, partial_to, radius_integer),
                           FillRule::NonZero);
    break;
  }
  return normalized(remaining);
}

}  // namespace wave
