#include "cam/heightmap_simulation.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <type_traits>

#include "cam/parametric.h"
#include "job_internal.h"
#include "planning/routing.h"

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
    if constexpr (std::is_invocable_v<F, wave::Vec3d, wave::Vec3d, double, double>)
      function(a, b, double(i - 1) / pieces, std::min(fraction, double(i) / pieces));
    else
      function(a, b);
    a = b;
  }
}
void record_fixture_contacts(MotionPhase& phase, const JobGeometry& geometry,
                             const wave::MillingCutter& cutter, double pitch) {
  phase.fixture_contacts.clear();
  if (geometry.fixtures().empty()) return;
  // Only refine colliding sweeps. Clear moves still cost one indexed query.
  // Localize contact conservatively to 0.05 mm of tool travel, independently
  // of the stock grid. Keep fractions so arcs and vertical moves play back too.
  const auto visit = [&](auto&& self, wave::Vec3d a, wave::Vec3d b,
                         double begin, double end) -> void {
    if (geometry.fixture_clear(a, b, cutter)) return;
    if (phase.length * (end - begin) > 0.05) {
      const double middle = (begin + end) / 2;
      const auto p = motion_point(phase, middle);
      self(self, a, p, begin, middle);
      self(self, p, b, middle, end);
    } else if (!phase.fixture_contacts.empty() &&
               phase.fixture_contacts.back().end == begin) {
      phase.fixture_contacts.back().end = end;
      phase.fixture_contacts.back().to = b;
    } else {
      phase.fixture_contacts.push_back({begin, end, a, b});
    }
  };
  sweep_phase(phase, 1, pitch, [&](auto a, auto b, double begin, double end) {
    visit(visit, a, b, begin, end);
  });
}
wave::Heightmap initial_stock(const JobGeometry& geometry, const Job& job,
                              double pitch) {
  if(geometry.inherited_stock()) return *geometry.inherited_stock();
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
        prisms.push_back({pocket_boundary(p, op.pocket_radial_margin),
                          p.top_z - p.depth, p.top_z});
    if(op.mode==OperationMode::Hole && op.holes.source_id.empty())
      for(const auto& h:op.holes.systems) for(const auto& l:h.levels) {
        const auto shape=Clipper2Lib::Ellipse(wave::Point64(wave::to_integer(h.center_x),wave::to_integer(h.center_y)),
            wave::to_integer(l.radius),wave::to_integer(l.radius),96);
        prisms.push_back({{shape},l.bottom_z,l.top_z});
      }
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

// The stock an operation starts from: the newest simulated stock before it, or
// the blank.
wave::Heightmap stock_before_operation(const JobGeometry& geometry, const Job& job,
                                       const PriorOperations& prior, double pitch) {
  for (auto i = prior.rbegin(); i != prior.rend(); ++i)
    if (*i && (*i)->ok() && (*i)->stock_after) return *(*i)->stock_after;
  return initial_stock(geometry, job, pitch);
}
}  // namespace

namespace detail {
bool link_layers(LevelPlan& previous, LevelPlan& level,
                 const wave::Heightmap& stock, const wave::MillingCutter& cutter,
                 double travel, double vertical_rate, double rapid_plane,
                 double slack) {
  auto& before = previous.feeds.motion;
  auto& after = level.feeds.motion;
  const auto last_cut = std::find_if(before.rbegin(), before.rend(),
                                     [](const auto& p) { return !p.rapid; });
  const auto first_cut = std::find_if(after.begin(), after.end(),
                                      [](const auto& p) { return !p.rapid; });
  if (last_cut == before.rend() || first_cut == after.end() ||
      last_cut->to_z != previous.floor_z || first_cut->from_z != level.floor_z)
    return false;
  const auto from = last_cut->to, to = first_cut->from;
  const double low = std::min(previous.floor_z, level.floor_z);
  const double high = std::max(previous.floor_z, level.floor_z);
  if (!(high < rapid_plane)) return false;
  const auto at = [](const wave::Point64& p, double z) {
    return wave::Vec3d{wave::to_world(p.x), wave::to_world(p.y), z};
  };
  const auto clear = [&](const wave::Point64& a, double za,
                         const wave::Point64& b, double zb) {
    for(const auto* limit:{&previous.machining_limit,&level.machining_limit}) if(!limit->empty()) {
      const auto bounds=Clipper2Lib::GetBounds(*limit);
      for(const auto p:{a,b}) if(p.x<bounds.left || p.x>bounds.right || p.y<bounds.top || p.y>bounds.bottom)
        return false;
    }
    return stock.clear(at(a, za), at(b, zb), cutter, 0.001, false);
  };
  // Lift first when moving up; descend last when moving down.
  const auto lower_point = previous.floor_z < level.floor_z ? from : to;
  if (!clear(lower_point, low, lower_point, high)) return false;
  std::optional<wave::Path64> route = wave::Path64{from, to};
  if (!clear(from, high, to, high)) {
    const auto obstacles = stock.slice(high + 0.002);
    if (obstacles.empty()) return false;
    auto bounds = Clipper2Lib::GetBounds(obstacles);
    bounds.left = std::min({bounds.left, from.x, to.x});
    bounds.right = std::max({bounds.right, from.x, to.x});
    bounds.top = std::min({bounds.top, from.y, to.y});
    bounds.bottom = std::max({bounds.bottom, from.y, to.y});
    const auto margin = wave::to_integer(cutter.radius * 4 + 10);
    bounds.left -= margin; bounds.right += margin;
    bounds.top -= margin; bounds.bottom += margin;
    const wave::Paths64 world{{{bounds.left, bounds.top}, {bounds.right, bounds.top},
                               {bounds.right, bounds.bottom}, {bounds.left, bounds.bottom}}};
    auto free = Clipper2Lib::Difference(
        world, detail::offset_outward(obstacles, cutter.radius),
        Clipper2Lib::FillRule::NonZero);
    for(const auto* limit:{&previous.machining_limit,&level.machining_limit}) if(!limit->empty())
      free=Clipper2Lib::Intersect(free,*limit,Clipper2Lib::FillRule::NonZero);
    const wave::Router router(wave::routable_region(free, slack, {from, to}), slack);
    route = router.route(from, to);
    if (!route || route->empty()) return false;
    for (std::size_t i = 1; i < route->size(); ++i)
      if (!clear((*route)[i - 1], high, (*route)[i], high)) return false;
  }
  std::vector<MotionPhase> seam;
  const double path_distance = first_cut->path_distance_begin;
  const auto add = [&](const wave::Point64& a, const wave::Point64& b, double za,
                       double zb) {
    MotionPhase phase;
    phase.from = a; phase.to = b; phase.from_z = za; phase.to_z = zb;
    phase.path_distance_begin = phase.path_distance_end = path_distance;
    phase.length = std::hypot(detail::distance_mm(a, b), zb - za);
    phase.rapid = za != zb;
    phase.rate = phase.rapid ? vertical_rate : travel;
    phase.kind = wave::MoveKind::Link;
    phase.inter_layer = true;
    if (phase.length > 0) seam.push_back(phase);
  };
  add(from, from, previous.floor_z, high);
  for (std::size_t i = 1; i < route->size(); ++i)
    add((*route)[i - 1], (*route)[i], high, high);
  add(to, to, high, level.floor_z);
  const auto departure = last_cut.base();
  double old_minutes = 0, new_minutes = 0;
  for (auto i = departure; i != before.end(); ++i) old_minutes += i->length / i->rate;
  for (auto i = after.begin(); i != first_cut; ++i) old_minutes += i->length / i->rate;
  for (const auto& phase : seam) new_minutes += phase.length / phase.rate;
  if (!(new_minutes < old_minutes)) return false;
  before.erase(departure, before.end());
  after.erase(after.begin(), first_cut);
  if (!seam.empty()) seam.front().arrival = true;
  after.insert(after.begin(), seam.begin(), seam.end());
  previous.stays_down_after = previous.feeds.ends_at_depth = true;
  level.feeds.begins_at_depth = true;
  return true;
}
}  // namespace detail

namespace {

bool vertical_entry(const MotionPhase& phase) {
  return !phase.arc && phase.from == phase.to && phase.to_z < phase.from_z &&
         (phase.rapid || phase.kind == wave::MoveKind::Entry);
}

// Rebuilds everything derived from a level's commanded motion: distances,
// travel and timing totals, and the 2D segments the pass view draws.
void finish_level_motion(LevelPlan& level, const CuttingFeeds& feeds,
                         double pitch) {
  index_feed_motion(level.feeds);
  level.motion_distances.clear();
  level.motion_distances.reserve(level.feeds.motion.size() + 1);
  double total = 0;
  for (const auto& phase : level.feeds.motion) {
    level.motion_distances.push_back(total);
    total += phase.length;
  }
  level.motion_distances.push_back(total);
  for (auto& checkpoint : level.stock_checkpoints) {
    checkpoint.phase = std::min(checkpoint.phase, level.feeds.motion.size());
    checkpoint.motion_distance = std::min(checkpoint.motion_distance, total);
  }
  auto& stats = level.feeds.stats;
  const double rpm = stats.spindle_rpm, engagement = stats.maximum_engagement;
  stats = {};
  stats.spindle_rpm = rpm;
  stats.maximum_engagement = engagement;
  double engaged = 0, weighted = 0;
  for (const auto& phase : level.feeds.motion) {
    if (phase.rapid) {
      stats.rapid_distance += phase.length;
      stats.rapid_minutes += phase.length / phase.rate;
    } else {
      stats.cutting_distance += phase.length;
      stats.cutting_minutes += phase.length / phase.rate;
      stats.minimum_feed = stats.minimum_feed ? std::min(stats.minimum_feed, phase.rate)
                                              : phase.rate;
      stats.maximum_feed = std::max(stats.maximum_feed, phase.rate);
      if (phase.from_z == phase.to_z && phase.removed_volume > 0) {
        engaged += phase.length;
        weighted += phase.engagement*phase.length;
      }
    }
  }
  stats.mean_engagement = engaged > 0 ? weighted/engaged : 0;
  // The 2D feed overlay must describe the same rates as the XYZ post.
  level.feeds.segments.clear();
  std::size_t move_index = 0;
  double move_end = level.toolpath.moves.empty() ? 0 : level.toolpath.moves.front().length;
  for (const auto& phase : level.feeds.motion) {
    if (!(phase.path_distance_end > phase.path_distance_begin)) continue;
    while (move_index + 1 < level.toolpath.moves.size() &&
           move_end < phase.path_distance_begin - 1e-8)
      move_end += level.toolpath.moves[++move_index].length;
    const int pieces = phase.arc
        ? std::max(32, int(std::ceil(phase.length / std::max(pitch, 0.1)))) : 1;
    FeedLimit limit = FeedLimit::Engagement;
    if (phase.rapid) limit = FeedLimit::Rapid;
    else if (phase.kind == wave::MoveKind::Entry) limit = FeedLimit::Plunge;
    else feeds.feed_at(phase.engagement, &limit);
    for (int j = 0; j < pieces; ++j) {
      const auto a = motion_point(phase, double(j) / pieces),
                 b = motion_point(phase, double(j + 1) / pieces);
      FeedSegment segment;
      segment.from = {wave::to_integer(a.x), wave::to_integer(a.y)};
      segment.to = {wave::to_integer(b.x), wave::to_integer(b.y)};
      segment.move_index = move_index;
      segment.kind = phase.kind;
      segment.rapid = phase.rapid;
      segment.length = (phase.path_distance_end - phase.path_distance_begin) / pieces;
      segment.start_distance = phase.path_distance_begin + segment.length * j;
      segment.rate = segment.feed = phase.rate;
      segment.engagement = phase.engagement;
      segment.limit = limit;
      if (segment.length > 0) level.feeds.segments.push_back(segment);
    }
  }
}
}  // namespace

wave::MillingCutter milling_cutter(const Tool& tool) {
  return {tool.type == ToolType::BallEndMill ? wave::CutterShape::Ball :
          tool.type == ToolType::Drill ? wave::CutterShape::Drill : wave::CutterShape::Flat,
          tool.diameter / 2, tool.flute_length,tool.point_angle_degrees,tool.tip_flat_diameter/2};
}

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
                            const Tool& base_tool, const Machine& machine,
                            const CuttingFeeds& base_feeds,
                            const PriorOperations& prior,
                            const ProgressCallback& progress,const ToolLibrary* tools) {
  const Tool& tool=base_tool;const auto& feeds=base_feeds;
  try {
    if (!std::isfinite(job.simulation_pitch) || job.simulation_pitch < 0.001)
      throw std::runtime_error("Stock grid pitch must be at least 0.001 mm");
    if (!std::isfinite(job.plunge_clearance_height) || job.plunge_clearance_height < 0)
      throw std::runtime_error("Plunge clearance must be finite and nonnegative");
    if (tool.type != ToolType::FlatEndMill && tool.type != ToolType::BallEndMill && tool.type!=ToolType::Drill)
      throw std::runtime_error("Stock simulation supports end mills and drills");
    auto stock = stock_before_operation(geometry, job, prior, job.simulation_pitch);
    plan.stock_before = std::make_shared<const wave::Heightmap>(stock);
    if (std::abs(stock.pitch() - job.simulation_pitch) > 1e-8) {
      if(geometry.inherited_stock())
        plan.debug_messages.push_back("Inherited stock keeps its source grid pitch of " +
                                std::to_string(stock.pitch()) + " mm without resampling");
      else
        plan.warnings.push_back("Earlier operations were simulated at " +
                                std::to_string(stock.pitch()) +
                                " mm; plan them again to use the new grid pitch");
    }
    const bool roughing = operation.mode == OperationMode::Roughing;
    const double vertical_rate = std::max(rapid_feed_z(machine), 1.0);
    // Keep even a zero-clearance approach above contact after the post rounds
    // its coordinates, including inch output and an arbitrary work origin.
    const double coordinate_step = (machine.output_inches ? 25.4 : 1.0) *
                                   std::pow(10.0, -machine.decimals);
    const double plunge_clearance = std::max(job.plunge_clearance_height, coordinate_step);
    const double slack = detail::link_slack(operation.stepover_ratio * tool.diameter);
    const double rapid_plane = job_rapid_plane(job, geometry);
    plan.spindle_rpm = feeds.spindle_rpm;
    plan.stats.removed_volume = plan.stats.maximum_load = 0;
    plan.stats.levels = plan.levels.size();
    plan.stats.cut_levels = 0;
    plan.stats.unsafe_plunges = 0;
    std::size_t plunged_descents = 0;
    LevelPlan* previous = nullptr;
    for (std::size_t l = 0; l < plan.levels.size(); ++l) {
      auto& level = plan.levels[l];
      const Tool* selected=&base_tool;
      if(!level.tool_id.empty() && level.tool_id!=base_tool.id) {
        selected=tools?tools->find(level.tool_id):nullptr;
        if(!selected) throw std::runtime_error("Unknown per-level tool");
      }
      const Tool& tool=*selected;
      const auto feeds=selected==&base_tool?base_feeds:
          resolve_cutting_feeds(tool,machine,job.material_id,operation.cutting);
      const auto cutter=milling_cutter(tool);
      const double travel=std::max(feeds.travel_feed(),1.0);
      if (progress &&
          !progress(double(l) / std::max(std::size_t{1}, plan.levels.size()),
                    "Simulating stock"))
        throw std::runtime_error("Cancelled");
      if (geometry.fixtures().empty() && roughing && previous && !level.nothing_to_cut && !level.feeds.motion.empty() &&
          detail::link_layers(*previous, level, stock, cutter, travel, vertical_rate,
                      rapid_plane, slack))
        finish_level_motion(*previous, feeds, stock.pitch());
      level.stock_checkpoints.clear();
      level.stock_checkpoints.push_back(
          {0, std::make_shared<const wave::Heightmap>(stock)});
      level.feeds.authoritative_motion = true;
      level.feeds.stats = {};
      level.feeds.stats.spindle_rpm = feeds.spindle_rpm;
      double removed = 0, motion_distance = 0, maximum_engagement = 0;
      // Keep feed estimates local even where a long straight contour was
      // planned as one move.
      std::vector<MotionPhase> phases;
      const auto pieces = [&](const MotionPhase& phase) {
        return phase.rapid || phase.arc || vertical_entry(phase) ||
                       operation.mode == OperationMode::SurfaceFinishing
                   ? 1
                   : std::max(1, int(std::ceil(phase.length /
                                               std::max(0.05, operation.path_pitch))));
      };
      std::size_t phase_count = 0;
      for (const auto& phase : level.feeds.motion) phase_count += pieces(phase);
      if (phase_count > 50000000)
        throw std::runtime_error("Simulation path has too many feed samples");
      phases.reserve(phase_count);
      for (const auto& phase : level.feeds.motion) {
        const int n = pieces(phase);
        for (int i = 0; i < n; ++i) {
          auto p = phase;
          const double a = double(i) / n, b = double(i + 1) / n;
          const auto from = motion_point(phase, a), to = motion_point(phase, b);
          if (n > 1) {
            p.from = {wave::to_integer(from.x), wave::to_integer(from.y)};
            p.to = {wave::to_integer(to.x), wave::to_integer(to.y)};
            p.from_z = from.z;
            p.to_z = to.z;
          }
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
      // A descent is split against the stock as it exists when that motion
      // executes, after all earlier cuts in this level and earlier operations.
      const std::size_t maximum_phases = level.feeds.motion.size() +
          std::count_if(level.feeds.motion.begin(), level.feeds.motion.end(), vertical_entry);
      if (maximum_phases > 50000000)
        throw std::runtime_error("Simulation path has too many feed samples");
      std::vector<MotionPhase> simulated;
      simulated.reserve(maximum_phases);
      // At most eight intermediate snapshots, even after splitting entries.
      const std::size_t checkpoint_stride =
          std::max(std::size_t{64}, (maximum_phases + 7) / 8);
      auto execute = [&](MotionPhase phase) {
        double volume = 0, depth = 0;
        if (simulated.size() % 64 == 0 && progress &&
            !progress(double(l) / std::max(std::size_t{1}, plan.levels.size()),
                      "Simulating stock"))
          throw std::runtime_error("Cancelled");
        record_fixture_contacts(phase, geometry, cutter, stock.pitch());
        const bool lift = phase.from == phase.to && phase.to_z >= phase.from_z;
        if (phase.rapid && !lift) {
          bool clear = true;
          sweep_phase(phase, 1, stock.pitch(), [&](auto a, auto b) {
            clear &= stock.clear(a, b, cutter);
          });
          if (!clear) {
            // A rapid descent that lands on stock comes down at plunge feed.
            if (phase.from != phase.to)
              throw std::runtime_error(
                  "A rapid traverse passes through remaining stock");
            phase.rapid = false;
            phase.kind = wave::MoveKind::Entry;
            if (roughing) ++plan.stats.unsafe_plunges;
            else ++plunged_descents;
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
        // Against the deeper of the pass's slab and what this move actually
        // cut, so a near-empty slab cannot inflate the engagement.
        const double reference = tool.diameter *
                                 std::max({level.axial_depth, depth, 0.001}) *
                                 std::max(phase.length, 1e-9);
        const double load = volume / reference;
        phase.engagement = load;
        phase.removed_volume = volume;
        if (!phase.rapid)
          phase.rate = phase.kind == wave::MoveKind::Entry
                           ? std::min(feeds.plunge_feed, feeds.maximum_feed)
                           : feeds.feed_at(load);
        if (!(phase.rate > 0) || !std::isfinite(phase.rate))
          throw std::runtime_error("Invalid commanded motion feed");
        // Plunges and other vertical moves are measured against a slot they
        // are not cutting; only moves at one height say anything about load.
        if (!phase.rapid && phase.kind != wave::MoveKind::Entry &&
            phase.from_z == phase.to_z)
          maximum_engagement = std::max(maximum_engagement, load);
        removed += volume;
        motion_distance += phase.length;
        simulated.push_back(std::move(phase));
        if (simulated.size() % checkpoint_stride == 0) {
          stock.compact_history();
          level.stock_checkpoints.push_back(
              {simulated.size(), std::make_shared<const wave::Heightmap>(stock),
               motion_distance});
        }
      };
      for (const auto& candidate : level.feeds.motion) {
        if (!vertical_entry(candidate)) {
          execute(candidate);
          continue;
        }
        const auto from = motion_point(candidate, 0), to = motion_point(candidate, 1);
        const double approach_z = stock.plunge_contact_z(from.x, from.y, cutter) + plunge_clearance;
        if (candidate.rapid && approach_z > candidate.to_z &&
            !stock.clear(from, to, cutter)) {
          if (roughing) ++plan.stats.unsafe_plunges;
          else ++plunged_descents;
        }
        auto approach = candidate;
        approach.to_z = std::clamp(approach_z, candidate.to_z, candidate.from_z);
        approach.length = approach.from_z - approach.to_z;
        approach.rapid = true;
        approach.kind = wave::MoveKind::Link;
        approach.rate = vertical_rate;
        if (approach.length > 1e-10) execute(approach);
        auto plunge = candidate;
        plunge.from_z = approach.to_z;
        plunge.length = plunge.from_z - plunge.to_z;
        plunge.rapid = false;
        plunge.kind = wave::MoveKind::Entry;
        if (approach.length > 1e-10) plunge.arrival = false;
        if (plunge.length > 1e-10) execute(plunge);
      }
      level.feeds.motion = std::move(simulated);
      stock.compact_history();
      level.stock_checkpoints.push_back(
          {level.feeds.motion.size(),
           std::make_shared<const wave::Heightmap>(stock), motion_distance});
      level.feeds.stats.maximum_engagement = maximum_engagement;
      finish_level_motion(level, feeds, stock.pitch());
      if (removed > 0) ++plan.stats.cut_levels;
      plan.stats.removed_volume += removed;
      plan.stats.maximum_load = std::max(plan.stats.maximum_load, maximum_engagement);
      if (!level.nothing_to_cut && !level.feeds.motion.empty()) previous = &level;
    }
    plan.stats.cutting_distance = plan.stats.rapid_distance = 0;
    plan.stats.cutting_minutes = plan.stats.rapid_minutes = 0;
    for (const auto& level : plan.levels) {
      plan.stats.cutting_distance += level.feeds.stats.cutting_distance;
      plan.stats.rapid_distance += level.feeds.stats.rapid_distance;
      plan.stats.cutting_minutes += level.feeds.stats.cutting_minutes;
      plan.stats.rapid_minutes += level.feeds.stats.rapid_minutes;
    }
    plan.stock_after = std::make_shared<const wave::Heightmap>(stock);
    std::size_t contact_moves = 0;
    const FixtureContact* first_contact = nullptr;
    for (const auto& level : plan.levels)
      for (const auto& phase : level.feeds.motion)
        if (!phase.fixture_contacts.empty()) {
          ++contact_moves;
          if (!first_contact) first_contact = &phase.fixture_contacts.front();
        }
    if (first_contact) {
      const auto p = first_contact->from;
      plan.warnings.push_back(std::to_string(contact_moves) +
          " motion(s) contact fixture or keep-away clearance; tool shown red during contact in playback. "
          "First contact near (" + std::to_string(p.x) + ", " + std::to_string(p.y) +
          ", " + std::to_string(p.z) + ") mm in setup coordinates");
    }
    if (plan.stats.unsafe_plunges > 0)
      plan.warnings.push_back(std::to_string(plan.stats.unsafe_plunges) +
                              " roughing descent(s) land on stock and plunge");
    if (plunged_descents)
      plan.warnings.push_back(std::to_string(plunged_descents) +
                              " descent(s) land on stock and use plunge feed");
  } catch (const std::exception& error) {
    plan.error = error.what();
    plan.stock_after.reset();
  }
}
}  // namespace cam
