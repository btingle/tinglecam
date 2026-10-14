#include "simulation/simulation.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <stdexcept>

namespace wave {
namespace {

using namespace Clipper2Lib;

constexpr std::size_t kCheckpointMoveStride = 4;
constexpr std::size_t kPlaybackSweepBatchSize = 64;

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

void append_sweep(Paths64& sweeps, const Point64& from, const Point64& to,
                  double radius) {
  Paths64 sweep = tool_sweep(from, to, radius);
  sweeps.insert(sweeps.end(), std::make_move_iterator(sweep.begin()),
                std::make_move_iterator(sweep.end()));
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

// Point classification stays in floating point: rounding a contact point onto
// the already-cut boundary would create false engagement on a repeated pass.
bool contains(double x, double y, const Paths64& region) {
  int winding = 0;
  for (const auto& ring : region) {
    if (ring.empty()) continue;
    auto a = ring.back();
    for (const auto& b : ring) {
      const double side = (double(b.x) - a.x) * (y - a.y) -
                          (x - a.x) * (double(b.y) - a.y);
      if (a.y <= y && b.y > y && side > 0) ++winding;
      if (a.y > y && b.y <= y && side < 0) --winding;
      a = b;
    }
  }
  return winding != 0;
}

// Integrate the projection of stock-contacting leading arcs onto the feed
// direction. Arc length/angle alone is not the removed width. Dividing the
// projected width by the diameter gives the instantaneous area-based load.
double contact_at(double x, double y, double ux, double uy, double radius,
                  const Paths64& stock) {
  constexpr double half_pi = 1.57079632679489661923;
  std::vector<double> angles{-half_pi, half_pi};
  for (const auto& ring : stock) {
    if (ring.empty()) continue;
    auto a = ring.back();
    for (const auto& b : ring) {
      if (std::max(a.x,b.x) >= x-radius && std::min(a.x,b.x) <= x+radius &&
          std::max(a.y,b.y) >= y-radius && std::min(a.y,b.y) <= y+radius) {
        const double dx = double(b.x)-a.x, dy = double(b.y)-a.y;
        const double ax = a.x-x, ay = a.y-y;
        const double aa = dx*dx+dy*dy;
        const double ab = ax*dx+ay*dy;
        const double discriminant = ab*ab-aa*(ax*ax+ay*ay-radius*radius);
        if (aa > 0 && discriminant >= 0) {
          const double root = std::sqrt(discriminant);
          for (double t : {(-ab-root)/aa, (-ab+root)/aa}) {
            if (t < 0 || t > 1) continue;
            const double px = ax+t*dx, py = ay+t*dy;
            const double angle = std::atan2(ux*py-uy*px, ux*px+uy*py);
            if (angle > -half_pi && angle < half_pi) angles.push_back(angle);
          }
        }
      }
      a = b;
    }
  }
  std::sort(angles.begin(), angles.end());
  double load = 0;
  for (std::size_t i=1; i<angles.size(); ++i) {
    const double mid = (angles[i-1]+angles[i])*0.5;
    const double forward = radius*std::cos(mid), side = radius*std::sin(mid);
    if (contains(x+ux*forward-uy*side, y+uy*forward+ux*side, stock))
      load += 0.5*(std::sin(angles[i])-std::sin(angles[i-1]));
  }
  return std::clamp(load, 0.0, 1.0);
}

double contact_load(const Point64& from, const Point64& to, double radius,
                    const Paths64& stock) {
  const double dx = double(to.x)-from.x, dy = double(to.y)-from.y;
  const double length = std::hypot(dx,dy);
  // Fixed quadrature within the existing interval, not adaptive path sampling.
  constexpr double nodes[]{0.1127016653792583, 0.5, 0.8872983346207417};
  constexpr double weights[]{5.0/18, 8.0/18, 5.0/18};
  double load = 0;
  for (int i=0; i<3; ++i)
    load += weights[i]*contact_at(from.x+nodes[i]*dx, from.y+nodes[i]*dy,
                                  dx/length,dy/length,radius,stock);
  return load;
}

}  // namespace

const char* simulation_method_name(SimulationMethod method) {
  switch (method) {
    case SimulationMethod::LocalArea: return "local_area";
    case SimulationMethod::Contact: return "contact";
    case SimulationMethod::Compare: return "compare";
    case SimulationMethod::ReferenceArea: return "reference_area";
  }
  return "unknown";
}

double material_area(const Paths64& paths) {
  if (paths.empty()) return 0.0;
  const Paths64 united = normalized(paths);
  return std::abs(Area(united)) / (kIntegerScale * kIntegerScale);
}

SimulationResult simulate_material(const Paths64& raw_stock,
                                   const Paths64& raw_final_subject,
                                   const Toolpath& toolpath,
                                   double tool_radius,
                                   double simulation_stock_inflation,
                                   SimulationMethod method,
                                   const std::function<bool(double)>& progress) {
  if (raw_stock.empty()) throw std::invalid_argument("Simulation stock is empty");
  if (raw_final_subject.empty())
    throw std::invalid_argument("Final subject is empty");
  if (!(tool_radius > 0.0) || !std::isfinite(tool_radius))
    throw std::invalid_argument("Tool radius must be positive and finite");
  if (!std::isfinite(simulation_stock_inflation))
    throw std::invalid_argument("Simulation stock inflation must be finite");

  SimulationResult result;
  result.method = method;
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
  const bool local_mode = method != SimulationMethod::ReferenceArea;
  const bool use_contact = method == SimulationMethod::Contact ||
                           method == SimulationMethod::Compare;
  Paths64 local, pending_sweeps;
  std::size_t batch_end = 0;
  const auto flush = [&] {
    if (!pending_sweeps.empty()) {
      // All capsule sweeps have positive winding. Clipper can subtract their
      // union directly, committing one global update for the whole batch.
      remaining = Difference(remaining, pending_sweeps, FillRule::NonZero);
      pending_sweeps.clear();
    }
  };
  Point64 last_cut_to;
  bool continuing_cut = false;
  double sampled_distance = 0;
  const Rect64 final_subject_bounds = GetBounds(result.final_subject);
  result.moves.resize(toolpath.moves.size());
  result.checkpoints.reserve(toolpath.moves.size() / kCheckpointMoveStride);

  std::size_t total_intervals = toolpath.moves.size();
  for (const auto& move : toolpath.moves)
    if (engaged(move.kind) && !move.path.empty())
      total_intervals += move.path.size() - 1;
  std::size_t completed_intervals = 0;
  const auto report_progress = [&] {
    if (progress && !progress(total_intervals == 0 ? 1.0 :
          static_cast<double>(completed_intervals) / total_intervals))
      throw std::runtime_error("Cancelled");
  };
  report_progress();

  for (std::size_t move_index = 0; move_index < toolpath.moves.size(); ++move_index) {
    const ToolpathMove& move = toolpath.moves[move_index];
    SimulatedMove& simulated = result.moves[move_index];

    if (engaged(move.kind)) {
      double move_distance = 0.0;
      batch_end = 0;
      for (std::size_t segment = 1; segment < move.path.size(); ++segment) {
        if ((++completed_intervals % 64) == 0) report_progress();
        const Point64& from = move.path[segment-1];
        const Point64& to = move.path[segment];
        const double length = segment_length(from,to);
        if (!(length > 0)) continue;
        if (local_mode && segment >= batch_end) {
          flush();
          Rect64 bounds(from.x,from.y,from.x,from.y);
          double distance = 0;
          batch_end = segment;
          // Bound both the geometry working set and the length of a batch.
          // Preserve every input interval, including corners and reversals.
          do {
            const auto& p = move.path[batch_end];
            bounds.left = std::min(bounds.left,p.x);
            bounds.right = std::max(bounds.right,p.x);
            bounds.top = std::min(bounds.top,p.y);
            bounds.bottom = std::max(bounds.bottom,p.y);
            distance += segment_length(move.path[batch_end-1],p);
            ++batch_end;
          } while (batch_end < move.path.size() && batch_end-segment < 64 &&
                   distance < 4*tool_radius);
          const auto margin = to_integer(tool_radius+1.0);
          bounds.left -= margin; bounds.right += margin;
          bounds.top -= margin; bounds.bottom += margin;
          local = Clipper2Lib::RectClip(bounds,remaining);
          ++result.stats.stock_batches;
        }
        auto& stock = local_mode ? local : remaining;
        MaterialStep step;
        step.from = from; step.to = to;
        step.move_distance_begin = move_distance;
        step.move_distance_end = move_distance+length;
        const Paths64 sweep = tool_sweep(from,to,radius_integer);
        Clipper64 cut;
        cut.AddSubject(stock);
        cut.AddClip(sweep);
        Paths64 removed, after, gouge;
        if (method == SimulationMethod::Contact) {
          cut.Execute(ClipType::Difference,FillRule::NonZero,after);
          step.removed_area = std::max(0.0,boolean_result_area(stock)-boolean_result_area(after));
          // Gouge detection always uses the swept interval, independently of
          // the contact estimate and its three quadrature locations.
          if (bounds_overlap(GetBounds(sweep),final_subject_bounds)) {
            cut.Execute(ClipType::Intersection,FillRule::NonZero,removed);
            gouge = Intersect(removed,result.final_subject,FillRule::NonZero);
          } else if (step.removed_area > 0) ++result.stats.gouge_bounds_rejections;
        } else {
          cut.Execute(ClipType::Intersection,FillRule::NonZero,removed);
          step.removed_area = boolean_result_area(removed);
          if (!removed.empty()) {
            cut.Execute(ClipType::Difference,FillRule::NonZero,after);
            if (bounds_overlap(GetBounds(removed),final_subject_bounds))
              gouge = Intersect(removed,result.final_subject,FillRule::NonZero);
            else ++result.stats.gouge_bounds_rejections;
          }
        }
        const bool cutting = step.removed_area > 0;
        step.area_load = step.removed_area/(2*tool_radius*length);
        step.contact_load = step.area_load;
        if (use_contact && cutting) {
          // A discontinuous entry may remove the starting disk as well as the
          // forward sweep. A circumference query cannot measure that disk.
          if (continuing_cut && last_cut_to == from && step.area_load <= 1) {
            // Sweeps use the radius rounded to the integer grid, while feeds
            // normalize by the configured diameter (also for area loads).
            step.contact_load = contact_load(from,to,radius_integer,stock) *
                                radius_integer/(kIntegerScale*tool_radius);
            // Preserve positive removal if a thin feature falls between the
            // quadrature locations. This does not make the estimate exact.
            if (step.contact_load <= 1e-12) {
              step.contact_load = step.area_load;
              ++result.stats.contact_fallbacks;
            }
          } else ++result.stats.contact_fallbacks;
        }
        step.load = method == SimulationMethod::Contact ? step.contact_load : step.area_load;
        const double disagreement = std::abs(std::clamp(step.contact_load,0.0,1.0) -
                                            std::clamp(step.area_load,0.0,1.0));
        result.stats.contact_mean_abs_difference += disagreement*length;
        result.stats.contact_max_abs_difference =
            std::max(result.stats.contact_max_abs_difference,disagreement);
        sampled_distance += length;
        step.gouge_area = boolean_result_area(gouge);
        result.stats.maximum_load = std::max(result.stats.maximum_load,step.load);
        if (step.gouge_area > 0) {
          ++result.stats.gouging_steps;
          gouges.insert(gouges.end(),gouge.begin(),gouge.end());
        }
        ++result.stats.cutting_steps;
        if (!cutting) ++result.stats.empty_cutting_steps;
        if (cutting) {
          stock = std::move(after);
        }
        // Commit the geometric sweep regardless of either load estimate, so
        // both methods leave identical stock for subsequent batches/layers.
        if (local_mode) pending_sweeps.insert(pending_sweeps.end(),sweep.begin(),sweep.end());
        simulated.steps.push_back(step);
        move_distance += length;
        continuing_cut = true; last_cut_to = to;
      }
      flush();
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

    ++completed_intervals;
    report_progress();
    if (!engaged(move.kind)) continuing_cut = false;
    if ((move_index + 1) % kCheckpointMoveStride == 0)
      result.checkpoints.push_back({move_index + 1, remaining});
  }

  result.remaining_stock = remaining;
  if (sampled_distance > 0) result.stats.contact_mean_abs_difference /= sampled_distance;
  result.removed_material = Difference(result.initial_stock, remaining,
                                       FillRule::NonZero);
  result.gouged_subject = local_mode
      ? Intersect(result.removed_material,result.final_subject,FillRule::NonZero)
      : normalized(gouges);
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
  if (simulation.moves.size() != toolpath.moves.size())
    throw std::invalid_argument("Simulation and toolpath moves do not match");

  if (distance >= toolpath_length(toolpath)) return simulation.remaining_stock;
  const ToolpathCursor cursor = toolpath_at(toolpath, distance);
  const double radius_integer = static_cast<double>(to_integer(simulation.tool_radius));
  Paths64 remaining = simulation.initial_stock;
  std::size_t first_move = 0;
  for (auto checkpoint = simulation.checkpoints.rbegin();
       checkpoint != simulation.checkpoints.rend(); ++checkpoint) {
    if (checkpoint->moves_completed > cursor.move) continue;
    remaining = checkpoint->remaining_stock;
    first_move = checkpoint->moves_completed;
    break;
  }

  Paths64 completed_sweeps;
  for (std::size_t move_index = first_move; move_index < cursor.move;
       ++move_index)
    for (const MaterialStep& step : simulation.moves[move_index].steps) {
      Paths64 sweep = tool_sweep(step.from, step.to, radius_integer);
      completed_sweeps.insert(completed_sweeps.end(),
                              std::make_move_iterator(sweep.begin()),
                              std::make_move_iterator(sweep.end()));
    }
  if (!completed_sweeps.empty())
    remaining = Difference(remaining, completed_sweeps, FillRule::NonZero);

  const SimulatedMove& move = simulation.moves[cursor.move];
  Paths64 current_sweeps;
  std::size_t current_sweep_count = 0;
  const auto flush_current_sweeps = [&] {
    if (current_sweeps.empty()) return;
    remaining = Difference(remaining, current_sweeps, FillRule::NonZero);
    current_sweeps.clear();
    current_sweep_count = 0;
  };
  for (const auto& step : move.steps) {
    if (cursor.move_distance <= step.move_distance_begin) break;
    if (cursor.move_distance >= step.move_distance_end) {
      append_sweep(current_sweeps, step.from, step.to, radius_integer);
      if (++current_sweep_count == kPlaybackSweepBatchSize)
        flush_current_sweeps();
      continue;
    }
    const double span = step.move_distance_end - step.move_distance_begin;
    const double ratio = span > 0.0
                             ? (cursor.move_distance - step.move_distance_begin) / span
                             : 0.0;
    const Point64 partial_to = interpolate(step.from, step.to, ratio);
    append_sweep(current_sweeps, step.from, partial_to, radius_integer);
    break;
  }
  flush_current_sweeps();
  return remaining;
}

Paths64 advance_simulation_stock(const SimulationResult& simulation,
                                 const Toolpath& toolpath,
                                 const Paths64& remaining_stock,
                                 double from_distance,
                                 double to_distance) {
  if (simulation.moves.size() != toolpath.moves.size())
    throw std::invalid_argument("Simulation and toolpath moves do not match");
  if (remaining_stock.empty() || toolpath.moves.empty()) return remaining_stock;

  const double total = toolpath_length(toolpath);
  from_distance = std::clamp(from_distance, 0.0, total);
  to_distance = std::clamp(to_distance, 0.0, total);
  if (to_distance <= from_distance) return remaining_stock;

  const double radius_integer = static_cast<double>(to_integer(simulation.tool_radius));
  Paths64 remaining = remaining_stock;
  Paths64 pending_sweeps;
  std::size_t sweep_count = 0;
  const auto flush = [&] {
    if (pending_sweeps.empty()) return;
    remaining = Difference(remaining, pending_sweeps, FillRule::NonZero);
    pending_sweeps.clear();
    sweep_count = 0;
  };
  const auto append = [&](const Point64& from, const Point64& to) {
    append_sweep(pending_sweeps, from, to, radius_integer);
    if (++sweep_count == kPlaybackSweepBatchSize) flush();
  };

  double move_start = 0.0;
  for (std::size_t move_index = 0; move_index < toolpath.moves.size(); ++move_index) {
    const ToolpathMove& path_move = toolpath.moves[move_index];
    const double move_end = move_start + path_move.length;
    if (move_end <= from_distance) {
      move_start = move_end;
      continue;
    }
    if (move_start >= to_distance) break;

    for (const MaterialStep& step : simulation.moves[move_index].steps) {
      const double step_begin = move_start + step.move_distance_begin;
      const double step_end = move_start + step.move_distance_end;
      const double begin = std::max(from_distance, step_begin);
      const double end = std::min(to_distance, step_end);
      if (end <= begin) continue;
      const double span = step.move_distance_end - step.move_distance_begin;
      if (!(span > 0.0)) continue;
      const Point64 from = interpolate(step.from, step.to,
          (begin - step_begin) / span);
      const Point64 to = interpolate(step.from, step.to,
          (end - step_begin) / span);
      if (from != to) append(from, to);
    }
    move_start = move_end;
  }
  flush();
  return remaining;
}

}  // namespace wave
