#include "cam/heightmap_simulation.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include "cam/parametric.h"

namespace cam {
wave::Vec3d motion_point(const MotionPhase& phase, double t) {
  wave::Vec3d p{wave::to_world(phase.from.x) +
                    (wave::to_world(phase.to.x - phase.from.x)) * t,
                wave::to_world(phase.from.y) +
                    (wave::to_world(phase.to.y - phase.from.y)) * t,
                phase.from_z + (phase.to_z - phase.from_z) * t};
  if (phase.arc) {
    const double cx = wave::to_world(phase.arc_center.x),
                 cy = wave::to_world(phase.arc_center.y);
    const double x = wave::to_world(phase.from.x) - cx,
                 y = wave::to_world(phase.from.y) - cy;
    const double start = std::atan2(y, x);
    double sweep = (phase.clockwise ? -1 : 1) * 2 * std::numbers::pi;
    if (phase.from != phase.to) {
      sweep = std::atan2(wave::to_world(phase.to.y) - cy,
                         wave::to_world(phase.to.x) - cx) -
              start;
      if (phase.clockwise && sweep >= 0) sweep -= 2 * std::numbers::pi;
      if (!phase.clockwise && sweep <= 0) sweep += 2 * std::numbers::pi;
    }
    const double angle = start + sweep * t, r = std::hypot(x, y);
    p.x = cx + r * std::cos(angle);
    p.y = cy + r * std::sin(angle);
  }
  return p;
}
namespace {
template <class F>
void sweep_phase(const MotionPhase& phase, double fraction, double pitch,
                 F function, double begin = 0) {
  // Straight sweeps are analytic. Circular/helical moves are chorded at an
  // XY error much smaller than the stock grid, with exact interpolated Z.
  int pieces = 1;
  if (phase.arc) {
    const double r =
        std::hypot(wave::to_world(phase.from.x - phase.arc_center.x),
                   wave::to_world(phase.from.y - phase.arc_center.y));
    const double chord =
        std::sqrt(std::max(1e-12, 8 * r * std::min(0.001, pitch / 20)));
    pieces = std::max(8, int(std::ceil(phase.length / chord)));
    if (pieces > 1000000)
      throw std::runtime_error("Simulation arc requires too many samples");
  }
  const int first = std::max(0, int(std::floor(begin * pieces)));
  auto a = motion_point(phase, double(first) / pieces);
  for (int i = first + 1; i <= pieces && double(i - 1) / pieces < fraction;
       ++i) {
    auto b = motion_point(phase, double(i) / pieces);
    if (double(i) / pieces > fraction)
      b = wave::interpolate(a, b, fraction * pieces - (i - 1));
    function(a, b);
    a = b;
  }
}
wave::Heightmap initial_stock(const JobGeometry& geometry, const Job& job,
                              double pitch) {
  if (geometry.has_stock()) {
    if (!geometry.box_stock())
      return wave::Heightmap::from_mesh(geometry.stock().mesh(), pitch);
    double low, high;
    geometry.work_range(low, high);
    const auto bounds = Clipper2Lib::GetBounds(geometry.stock_outline(low, 32));
    return wave::Heightmap::from_polygons({{{bounds.left, bounds.top},
                                            {bounds.right, bounds.top},
                                            {bounds.right, bounds.bottom},
                                            {bounds.left, bounds.bottom}}},
                                          low, high, pitch);
  }
  struct Prism {
    wave::Paths64 shape;
    double bottom, top;
  };
  std::vector<Prism> prisms;
  wave::Paths64 all;
  for (const auto& op : job.operations) {
    if (op.mode == OperationMode::Pocket)
      for (const auto& p : op.pockets)
        prisms.push_back({pocket_boundary(p), p.top_z - p.depth, p.top_z});
    if (op.mode == OperationMode::PlaneClearing)
      for (const auto& a : op.plane.areas) {
        const auto x = wave::to_integer(a.center_x),
                   y = wave::to_integer(a.center_y);
        const auto w = wave::to_integer(a.width / 2),
                   h = wave::to_integer(a.height / 2);
        prisms.push_back(
            {{{{x - w, y - h}, {x + w, y - h}, {x + w, y + h}, {x - w, y + h}}},
             a.top_z - a.depth,
             a.top_z});
      }
  }
  for (const auto& p : prisms)
    all.insert(all.end(), p.shape.begin(), p.shape.end());
  if (all.empty())
    throw std::runtime_error("No stock is available for XYZ simulation");
  const auto bounds = Clipper2Lib::GetBounds(all);
  wave::Heightmap stock(wave::to_world(bounds.left), wave::to_world(bounds.top),
                        wave::to_world(bounds.Width()),
                        wave::to_world(bounds.Height()), pitch);
  for (const auto& p : prisms) stock.fill(p.shape, p.bottom, p.top);
  return stock;
}
}  // namespace
void advance_stock_motion(wave::Heightmap& stock, const LevelPlan& level,
                          const wave::MillingCutter& tool, double from,
                          double to) {
  if (to <= from) return;
  std::size_t phase = 0;
  double distance = 0;
  if (!level.motion_distances.empty()) {
    const auto i = std::upper_bound(level.motion_distances.begin(),
                                    level.motion_distances.end(), from);
    phase = i == level.motion_distances.begin()
                ? 0
                : std::size_t(i - level.motion_distances.begin() - 1);
    if (phase >= level.feeds.motion.size()) return;
    distance = level.motion_distances[phase];
  }
  for (; phase < level.feeds.motion.size() && distance < to; ++phase) {
    const auto& p = level.feeds.motion[phase];
    const double lo =
        p.length > 0 ? std::clamp((from - distance) / p.length, 0.0, 1.0) : 0;
    const double hi =
        p.length > 0 ? std::clamp((to - distance) / p.length, 0.0, 1.0) : 1;
    if (!p.rapid && hi > lo) {
      if (!p.arc)
        stock.cut(motion_point(p, lo), motion_point(p, hi), tool);
      else
        sweep_phase(
            p, hi, stock.pitch(),
            [&](auto a, auto b) { stock.cut(a, b, tool); }, lo);
    }
    distance += p.length;
  }
}
wave::Heightmap stock_at_motion(const LevelPlan& level,
                                const wave::MillingCutter& tool,
                                double target) {
  if (level.stock_checkpoints.empty()) return {};
  wave::Heightmap stock = *level.stock_checkpoints.front().stock;
  const auto found = std::upper_bound(
      level.stock_checkpoints.begin(), level.stock_checkpoints.end(), target,
      [](double d, const StockCheckpoint& checkpoint) {
        return d < checkpoint.motion_distance;
      });
  const auto& checkpoint = found == level.stock_checkpoints.begin()
                               ? level.stock_checkpoints.front()
                               : *std::prev(found);
  stock = *checkpoint.stock;
  std::size_t phase = checkpoint.phase;
  double travelled = checkpoint.motion_distance;
  for (; phase < level.feeds.motion.size() && travelled < target; ++phase) {
    const auto& p = level.feeds.motion[phase];
    if (!p.rapid)
      sweep_phase(
          p,
          p.length > 0 ? std::clamp((target - travelled) / p.length, 0.0, 1.0)
                       : 1,
          stock.pitch(), [&](auto a, auto b) { stock.cut(a, b, tool); });
    travelled += p.length;
  }
  return stock;
}
void simulate_operation_xyz(OperationPlan& plan, const JobGeometry& geometry,
                            const Job& job, const Operation& operation,
                            const Tool& tool, const Machine& machine,
                            const PriorOperations& prior,
                            const ProgressCallback& progress) {
  try {
    if (!std::isfinite(operation.simulation_pitch) ||
        operation.simulation_pitch < 0.001)
      throw std::runtime_error("Simulation pitch must be at least 0.001 mm");
    std::shared_ptr<const wave::Heightmap> incoming;
    for (auto i = prior.rbegin(); i != prior.rend(); ++i)
      if (*i && (*i)->ok() && (*i)->stock_after) {
        incoming = (*i)->stock_after;
        break;
      }
    auto stock = incoming
                     ? *incoming
                     : initial_stock(geometry, job, operation.simulation_pitch);
    plan.stock_before = std::make_shared<const wave::Heightmap>(stock);
    if (incoming && std::abs(stock.pitch() - operation.simulation_pitch) > 1e-8)
      plan.warnings.push_back("Using the first operation's simulation pitch (" +
                              std::to_string(stock.pitch()) +
                              " mm) to preserve stock history");
    const wave::MillingCutter cutter{tool.type == ToolType::BallEndMill
                                         ? wave::CutterShape::Ball
                                         : wave::CutterShape::Flat,
                                     tool.diameter / 2, tool.flute_length};
    if (tool.type != ToolType::FlatEndMill &&
        tool.type != ToolType::BallEndMill)
      throw std::runtime_error(
          "Heightmap simulation currently supports flat and ball end mills");
    double slow = operation.full_engagement_feed,
           fast = operation.no_engagement_feed;
    if (!operation.manual_feed_range)
      if (const auto* feeds = machine.feeds_for(operation.material_id)) {
        slow = feeds->full_engagement;
        fast = feeds->no_engagement;
      }
    if (!std::isfinite(slow) || !std::isfinite(fast) || slow <= 0 ||
        fast < slow || !std::isfinite(operation.plunge_feed) ||
        operation.plunge_feed <= 0)
      throw std::runtime_error("Invalid cutting feed range or plunge feed");
    plan.stats.cutting_distance = plan.stats.rapid_distance = 0;
    plan.stats.cutting_minutes = plan.stats.rapid_minutes = 0;
    plan.stats.removed_volume = plan.stats.maximum_load = 0;
    plan.stats.levels = plan.levels.size();
    plan.stats.cut_levels = 0;
    std::size_t conservative_descents = 0;
    for (std::size_t l = 0; l < plan.levels.size(); ++l) {
      auto& level = plan.levels[l];
      if (progress &&
          !progress(double(l) / std::max(std::size_t{1}, plan.levels.size()),
                    "Simulating XYZ stock"))
        throw std::runtime_error("Cancelled");
      level.stock_checkpoints.clear();
      level.stock_checkpoints.push_back(
          {0, std::make_shared<const wave::Heightmap>(stock)});
      level.simulation.heightmap_before = level.stock_checkpoints.front().stock;
      level.feeds.authoritative_motion = true;
      auto& stats = level.feeds.stats;
      stats = {};
      stats.spindle_rpm = plan.spindle_rpm;
      double removed = 0, motion_distance = 0;
      // Keep feed estimates local even when the planar feed pass previously
      // merged a long straight contour into one commanded move.
      std::vector<MotionPhase> phases;
      std::size_t phase_count = 0;
      for (const auto& phase : level.feeds.motion)
        phase_count +=
            phase.rapid || phase.arc ||
                    operation.mode == OperationMode::SurfaceFinishing
                ? 1
                : std::max(
                      std::size_t{1},
                      std::size_t(std::ceil(
                          phase.length / std::max(.05, operation.path_pitch))));
      phases.reserve(phase_count);
      for (const auto& phase : level.feeds.motion) {
        const int n =
            phase.rapid || phase.arc ||
                    operation.mode == OperationMode::SurfaceFinishing
                ? 1
                : std::max(
                      1, int(std::ceil(phase.length /
                                       std::max(0.05, operation.path_pitch))));
        if (n > 1000000)
          throw std::runtime_error("Simulation path has too many feed samples");
        for (int i = 0; i < n; ++i) {
          auto p = phase;
          const double a = double(i) / n, b = double(i + 1) / n;
          const auto from = motion_point(phase, a), to = motion_point(phase, b);
          p.from = {wave::to_integer(from.x), wave::to_integer(from.y)};
          p.to = {wave::to_integer(to.x), wave::to_integer(to.y)};
          p.from_z = from.z;
          p.to_z = to.z;
          p.length = phase.length / n;
          p.path_distance_begin =
              phase.path_distance_begin +
              (phase.path_distance_end - phase.path_distance_begin) * a;
          p.path_distance_end =
              phase.path_distance_begin +
              (phase.path_distance_end - phase.path_distance_begin) * b;
          phases.push_back(p);
        }
      }
      level.feeds.motion = std::move(phases);
      level.motion_distances.clear();
      level.motion_distances.reserve(level.feeds.motion.size() + 1);
      double cumulative = 0;
      for (const auto& phase : level.feeds.motion) {
        level.motion_distances.push_back(cumulative);
        cumulative += phase.length;
      }
      level.motion_distances.push_back(cumulative);
      // Bounded checkpoints plus forward replay avoid retaining a new grid
      // history every 64 tiny feed intervals on long, high-resolution jobs.
      const std::size_t checkpoint_stride =
          std::max(std::size_t{64}, (level.feeds.motion.size() + 7) / 8);
      for (std::size_t i = 0; i < level.feeds.motion.size(); ++i) {
        auto& phase = level.feeds.motion[i];
        double volume = 0, depth = 0;
        if (i % 64 == 0 && progress &&
            !progress(double(l) / std::max(std::size_t{1}, plan.levels.size()),
                      "Simulating XYZ stock"))
          throw std::runtime_error("Cancelled");
        const bool lift = phase.from == phase.to && phase.to_z >= phase.from_z;
        if (phase.rapid && !lift) {
          bool clear = true;
          sweep_phase(phase, 1, stock.pitch(), [&](auto a, auto b) {
            clear &= stock.clear(a, b, cutter);
          });
          if (!clear) {
            // A grid cannot prove a touching descent is in air. Execute it at
            // plunge feed, rather than preserve an unproven G0 landing.
            if (phase.from != phase.to)
              throw std::runtime_error(
                  "XYZ stock simulation found a rapid traverse through "
                  "remaining stock");
            phase.rapid = false;
            phase.kind = wave::MoveKind::Entry;
            ++conservative_descents;
          }
        }
        if (!phase.rapid)
          sweep_phase(phase, 1, stock.pitch(), [&](auto a, auto b) {
            if (!stock.shank_clear(a, b, cutter))
              throw std::runtime_error(
                  "Remaining stock reaches above the cutter's flute length");
            const auto sample = stock.cut(a, b, cutter);
            volume += sample.volume;
            depth = std::max(depth, sample.maximum_depth);
          });
        const double reference = tool.diameter *
                                 std::max(level.axial_depth, 0.001) *
                                 std::max(phase.length, 1e-9);
        const double load = volume / reference;
        phase.engagement = load;
        phase.removed_volume = volume;
        if (!phase.rapid)
          phase.rate =
              phase.kind == wave::MoveKind::Entry
                  ? operation.plunge_feed
                  : slow + (fast - slow) * (1 - std::clamp(load, 0.0, 1.0));
        if (!(phase.rate > 0) || !std::isfinite(phase.rate))
          throw std::runtime_error("Invalid commanded motion feed");
        if (phase.rapid) {
          stats.rapid_distance += phase.length;
          stats.rapid_minutes += phase.length / phase.rate;
        } else {
          stats.cutting_distance += phase.length;
          stats.cutting_minutes += phase.length / phase.rate;
          stats.minimum_feed = stats.minimum_feed
                                   ? std::min(stats.minimum_feed, phase.rate)
                                   : phase.rate;
          stats.maximum_feed = std::max(stats.maximum_feed, phase.rate);
        }
        stats.maximum_engagement = std::max(stats.maximum_engagement, load);
        removed += volume;
        motion_distance += phase.length;
        if ((i + 1) % checkpoint_stride == 0) {
          stock.compact_history();
          level.stock_checkpoints.push_back(
              {i + 1, std::make_shared<const wave::Heightmap>(stock),
               motion_distance});
        }
      }
      // The 2D feed overlay must describe the same rates as the XYZ post.
      level.feeds.segments.clear();
      std::size_t segment_count = 0;
      for (const auto& phase : level.feeds.motion)
        if (phase.path_distance_end > phase.path_distance_begin)
          segment_count +=
              phase.arc
                  ? std::max(32, int(std::ceil(phase.length /
                                               std::max(stock.pitch(), .1))))
                  : 1;
      level.feeds.segments.reserve(segment_count);
      std::size_t move_index = 0;
      double move_end = level.toolpath.moves.empty()
                            ? 0
                            : level.toolpath.moves.front().length;
      for (const auto& phase : level.feeds.motion) {
        while (move_index + 1 < level.toolpath.moves.size() &&
               move_end < phase.path_distance_begin - 1e-8)
          move_end += level.toolpath.moves[++move_index].length;
        const int pieces =
            phase.arc
                ? std::max(32, int(std::ceil(phase.length /
                                             std::max(stock.pitch(), 0.1))))
                : 1;
        for (int j = 0; j < pieces; ++j) {
          const auto a = motion_point(phase, double(j) / pieces),
                     b = motion_point(phase, double(j + 1) / pieces);
          FeedSegment segment;
          segment.from = {wave::to_integer(a.x), wave::to_integer(a.y)};
          segment.to = {wave::to_integer(b.x), wave::to_integer(b.y)};
          segment.move_index = move_index;
          segment.kind = phase.kind;
          segment.rapid = phase.rapid;
          segment.safe_retract = phase.rapid;
          segment.length =
              (phase.path_distance_end - phase.path_distance_begin) / pieces;
          segment.start_distance =
              phase.path_distance_begin + segment.length * j;
          segment.rate = segment.feed = phase.rate;
          segment.engagement = phase.engagement;
          segment.limit = phase.rapid ? FeedLimit::Rapid
                                      : (phase.kind == wave::MoveKind::Entry
                                             ? FeedLimit::Plunge
                                             : FeedLimit::Engagement);
          if (segment.length > 0) level.feeds.segments.push_back(segment);
        }
      }
      stock.compact_history();
      level.stock_checkpoints.push_back(
          {level.feeds.motion.size(),
           std::make_shared<const wave::Heightmap>(stock), motion_distance});
      level.simulation.heightmap_after = level.stock_checkpoints.back().stock;
      // Routing has finished and the XYZ phases now own engagement. Retaining
      // the preliminary planar samples duplicates the same path/load history.
      std::vector<wave::SimulatedMove>().swap(level.simulation.moves);
      std::vector<wave::SimulationCheckpoint>().swap(
          level.simulation.checkpoints);
      if (removed > 0) ++plan.stats.cut_levels;
      plan.stats.removed_volume += removed;
      plan.stats.maximum_load =
          std::max(plan.stats.maximum_load, stats.maximum_engagement);
      plan.stats.cutting_distance += stats.cutting_distance;
      plan.stats.rapid_distance += stats.rapid_distance;
      plan.stats.cutting_minutes += stats.cutting_minutes;
      plan.stats.rapid_minutes += stats.rapid_minutes;
    }
    plan.stock_after = std::make_shared<const wave::Heightmap>(stock);
    if (conservative_descents)
      plan.warnings.push_back(std::to_string(conservative_descents) +
                              " descents use plunge feed because grid "
                              "clearance could not be established");
  } catch (const std::exception& error) {
    plan.error = error.what();
    plan.stock_after.reset();
  }
}
}  // namespace cam
