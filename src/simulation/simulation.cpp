#include "simulation/simulation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "simulation/heightmap.h"

namespace wave {
namespace {
bool engaged(MoveKind kind) {
  return kind == MoveKind::Cut || kind == MoveKind::Step ||
         kind == MoveKind::Link;
}
Vec3d point(Point64 p) { return {to_world(p.x), to_world(p.y), 0}; }
double length(Point64 a, Point64 b) { return distance(point(a), point(b)); }
}  // namespace
const char* simulation_method_name(SimulationMethod method) {
  switch (method) {
    case SimulationMethod::Contact:
      return "contact";
    case SimulationMethod::Compare:
      return "compare";
    case SimulationMethod::ReferenceArea:
      return "reference_area";
    case SimulationMethod::Heightmap:
      return "heightmap";
    default:
      return "local_area";
  }
}
double material_area(const Paths64& paths) {
  return std::abs(Clipper2Lib::Area(
             Clipper2Lib::Union(paths, Clipper2Lib::FillRule::NonZero))) /
         (kIntegerScale * kIntegerScale);
}
SimulationResult simulate_material(
    const Paths64& raw_stock, const Paths64& subject, const Toolpath& path,
    double radius, double inflation, SimulationMethod method,
    const std::function<bool(double)>& progress, bool retain_playback) {
  if (!(radius > 0) || !std::isfinite(radius) || !std::isfinite(inflation))
    throw std::invalid_argument("Invalid simulation cutter or inflation");
  SimulationResult result;
  result.tool_radius = radius;
  result.stock_inflation = inflation;
  result.method = method;
  result.initial_stock =
      inflation == 0
          ? Clipper2Lib::Union(raw_stock, Clipper2Lib::FillRule::NonZero)
          : Clipper2Lib::InflatePaths(raw_stock, to_integer(inflation),
                                      Clipper2Lib::JoinType::Miter,
                                      Clipper2Lib::EndType::Polygon);
  result.final_subject =
      Clipper2Lib::Union(subject, Clipper2Lib::FillRule::NonZero);
  const auto bounds = Clipper2Lib::GetBounds(result.initial_stock);
  const double pitch =
      std::max({0.005, std::min(0.05, radius / 40),
                std::sqrt(to_world(bounds.Width()) * to_world(bounds.Height()) /
                          2000000.0)});
  auto stock = Heightmap::from_polygons(result.initial_stock, 0, 1, pitch);
  if(retain_playback) result.heightmap_before = std::make_shared<const Heightmap>(stock);
  result.stats.initial_stock_area = stock.volume();
  MillingCutter tool{CutterShape::Flat, radius, 1};
  result.moves.resize(path.moves.size());
  Paths64 gouges, routing_sweeps;
  std::size_t total = 0, completed = 0;
  for (const auto& move : path.moves)
    total += std::max(std::size_t{1}, move.path.size() - 1);
  const auto report = [&] {
    if (progress &&
        !progress(double(completed) / std::max(std::size_t{1}, total)))
      throw std::runtime_error("Cancelled");
  };
  for (std::size_t index = 0; index < path.moves.size(); ++index) {
    report();
    const auto& move = path.moves[index];
    auto& output = result.moves[index];
    double along = 0;
    if (engaged(move.kind))
      for (std::size_t i = 1; i < move.path.size(); ++i) {
        const auto from = move.path[i - 1], to = move.path[i];
        const double span = length(from, to);
        if (span <= 0) continue;
        if (completed++ % 64 == 0) report();
        auto removal = stock.cut(point(from), point(to), tool);
        MaterialStep step;
        step.from = from;
        step.to = to;
        step.move_distance_begin = along;
        step.move_distance_end = along + span;
        step.removed_area = removal.volume;
        step.load = step.area_load = step.contact_load =
            removal.volume / (2 * radius * span);
        result.stats.maximum_load =
            std::max(result.stats.maximum_load, step.load);
        ++result.stats.cutting_steps;
        if (removal.volume <= 0) ++result.stats.empty_cutting_steps;
        // Nominal part protection retains the original polygon geometry, even
        // though stock removal and engagement are now sampled on the XY grid.
        auto sweep = Clipper2Lib::InflatePaths(
            Paths64{{from, to}}, to_integer(radius),
            Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Round);
        routing_sweeps.insert(routing_sweeps.end(), sweep.begin(), sweep.end());
        if (completed % 64 == 0)
          routing_sweeps = Clipper2Lib::Union(routing_sweeps,
                                              Clipper2Lib::FillRule::NonZero);
        auto hit = Clipper2Lib::Intersect(sweep, result.final_subject,
                                          Clipper2Lib::FillRule::NonZero);
        step.gouge_area = material_area(hit);
        if (step.gouge_area > 0) {
          gouges.insert(gouges.end(), hit.begin(), hit.end());
          ++result.stats.gouging_steps;
        }
        output.steps.push_back(step);
        along += span;
      }
    else if (move.kind == MoveKind::Retract && !move.path.empty()) {
      const auto at = point(move.path.back());
      // The planar adapter samples its columns at centres. Exact nominal
      // target collision remains independently tested above.
      auto probe = tool;
      probe.radius = std::max(radius - pitch * std::sqrt(0.5), radius / 2);
      if (!stock.clear(at, at, probe)) {
        output.plunge_collision = Clipper2Lib::Intersect(
            stock.footprint(),
            Paths64{Clipper2Lib::Ellipse(move.path.back(), to_integer(radius),
                                         to_integer(radius), 48)},
            Clipper2Lib::FillRule::NonZero);
        output.plunge_collision_area =
            std::max(pitch * pitch, material_area(output.plunge_collision));
        ++result.stats.unsafe_plunges;
      }
    } else if (move.kind == MoveKind::Entry) {
      output.entry_unresolved = true;
      ++result.stats.unresolved_entries;
    }
    if (!engaged(move.kind))
      completed += std::max(std::size_t{1}, move.path.size() - 1);
    if (retain_playback && (index + 1) % 32 == 0)
      result.checkpoints.push_back(
          {index + 1, {}, std::make_shared<const Heightmap>(stock)});
  }
  result.stats.stock_batches = (result.stats.cutting_steps + 63) / 64;
  if(retain_playback) result.heightmap_after = std::make_shared<const Heightmap>(stock);
  result.remaining_stock = stock.footprint();
  // Smooth planar sweep evidence belongs to wavefront routing and subdivision
  // geometry. Do not inject the grid's staircase boundaries into those
  // algorithms. Stock snapshots, engagement and XYZ replay use the heightmap.
  result.removed_material = Clipper2Lib::Intersect(
      result.initial_stock,
      Clipper2Lib::Union(routing_sweeps, Clipper2Lib::FillRule::NonZero),
      Clipper2Lib::FillRule::NonZero);
  result.gouged_subject =
      Clipper2Lib::Union(gouges, Clipper2Lib::FillRule::NonZero);
  result.excess_material =
      Clipper2Lib::Difference(result.remaining_stock, result.final_subject,
                              Clipper2Lib::FillRule::NonZero);
  result.stats.remaining_area = stock.volume();
  result.stats.removed_area =
      result.stats.initial_stock_area - result.stats.remaining_area;
  result.stats.gouge_area = material_area(result.gouged_subject);
  result.stats.excess_area = material_area(result.excess_material);
  if (progress && !progress(1)) throw std::runtime_error("Cancelled");
  return result;
}
Paths64 simulation_stock_at(const SimulationResult& simulation,
                            const Toolpath& path, double target) {
  if (!simulation.heightmap_before) return simulation.remaining_stock;
  Heightmap stock = *simulation.heightmap_before;
  std::size_t start = 0;
  for (const auto& checkpoint : simulation.checkpoints) {
    double d = 0;
    for (std::size_t i = 0; i < checkpoint.moves_completed; ++i)
      d += path.moves[i].length;
    if (d > target) break;
    if (checkpoint.heightmap) {
      stock = *checkpoint.heightmap;
      start = checkpoint.moves_completed;
    }
  }
  double travelled = 0;
  for (std::size_t i = 0; i < start; ++i) travelled += path.moves[i].length;
  MillingCutter tool{CutterShape::Flat, simulation.tool_radius, 1};
  for (std::size_t index = start;
       index < path.moves.size() && travelled < target; ++index) {
    const auto& move = path.moves[index];
    if (engaged(move.kind))
      for (std::size_t i = 1; i < move.path.size() && travelled < target; ++i) {
        auto a = point(move.path[i - 1]), b = point(move.path[i]);
        double span = distance(a, b);
        if (span > 0) {
          stock.cut(
              a,
              interpolate(a, b,
                          std::clamp((target - travelled) / span, 0.0, 1.0)),
              tool);
          travelled += span;
        }
      }
    else
      travelled += move.length;
  }
  return stock.footprint();
}
Paths64 advance_simulation_stock(const SimulationResult& simulation,
                                 const Toolpath& path, const Paths64&, double,
                                 double to) {
  return simulation_stock_at(simulation, path, to);
}
}  // namespace wave
