#include "job_internal.h"
namespace cam { namespace detail {

using PlanningClock = std::chrono::steady_clock;

double elapsed_milliseconds(PlanningClock::time_point start) {
  return std::chrono::duration<double, std::milli>(PlanningClock::now() - start)
      .count();
}

using Clipper2Lib::FillRule;
using Clipper2Lib::InflatePaths;
using Clipper2Lib::JoinType;
using Clipper2Lib::EndType;
using Clipper2Lib::Union;

wave::Paths64 offset_outward(const wave::Paths64& input, double amount) {
  wave::Paths64 result = Union(input, FillRule::NonZero);
  const std::int64_t delta = wave::to_integer(amount);
  if (delta == 0) return result;
  result = InflatePaths(result, static_cast<double>(delta), JoinType::Round,
                        EndType::Polygon);
  return Union(result, FillRule::NonZero);
}

// A subject far outside the stock, used when nothing of the part reaches this
// depth. The wave needs something to propagate against, and a marker parked
// beyond the work makes the whole level clear away.
wave::Paths64 absent_subject(const wave::Paths64& stock) {
  const auto bounds = Clipper2Lib::GetBounds(stock);
  const std::int64_t span =
      std::max<std::int64_t>(1000, std::max(bounds.right - bounds.left,
                                            bounds.bottom - bounds.top));
  const std::int64_t left = bounds.right + span;
  const std::int64_t top = bounds.bottom + span;
  return wave::Paths64{{{left, top},
                        {left + span, top},
                        {left + span, top + span},
                        {left, top + span}}};
}

void accumulate(PlanStats& into, const PlanStats& from) {
  into.levels += from.levels;
  into.cut_levels += from.cut_levels;
  into.cutting_distance += from.cutting_distance;
  into.rapid_distance += from.rapid_distance;
  into.cutting_minutes += from.cutting_minutes;
  into.rapid_minutes += from.rapid_minutes;
  into.tool_change_minutes += from.tool_change_minutes;
  into.removed_volume += from.removed_volume;
  into.gouge_area += from.gouge_area;
  into.unsafe_plunges += from.unsafe_plunges;
  into.unresolved_entries += from.unresolved_entries;
  into.skipped_entries += from.skipped_entries;
  into.lifts_over_subject += from.lifts_over_subject;
  into.unreachable_fronts += from.unreachable_fronts;
  into.excess_area += from.excess_area;
  into.maximum_load = std::max(into.maximum_load, from.maximum_load);
}

void accumulate(PlanningMetrics& into, const PlanningMetrics& from) {
  into.slicing_milliseconds += from.slicing_milliseconds;
  into.preparation_milliseconds += from.preparation_milliseconds;
  into.wavefront_milliseconds += from.wavefront_milliseconds;
  into.lineage_milliseconds += from.lineage_milliseconds;
  into.inter_layer_milliseconds += from.inter_layer_milliseconds;
  into.toolpath_milliseconds += from.toolpath_milliseconds;
  into.cleanup_milliseconds += from.cleanup_milliseconds;
  into.stitching_milliseconds += from.stitching_milliseconds;
  into.resampling_milliseconds += from.resampling_milliseconds;
  into.simulation_milliseconds += from.simulation_milliseconds;
  into.feeds_milliseconds += from.feeds_milliseconds;
  into.total_milliseconds += from.total_milliseconds;
  into.wave_generations += from.wave_generations;
  into.fronts += from.fronts;
  into.analysis_layers += from.analysis_layers;
  into.analysis_trials += from.analysis_trials;
  into.accepted_analysis_trials += from.accepted_analysis_trials;
  into.toolpath_moves += from.toolpath_moves;
  into.simulation_intervals += from.simulation_intervals;
  into.empty_simulation_intervals += from.empty_simulation_intervals;
  into.gouge_bounds_rejections += from.gouge_bounds_rejections;
}

// Everything both planning stages need out of the operation, the tool and the
// machine. Resolving it once keeps the two stages from disagreeing about the
// engagement they were capped to.
double distance_mm(const wave::Point64& a, const wave::Point64& b) {
  return std::hypot(wave::to_world(b.x - a.x), wave::to_world(b.y - a.y));
}

// How far a move at depth may stray outside the region it travels through.
// The same figure the toolpath builder holds its own links to, so a link, a
// finishing connector and a transition between depths are all judged alike.
double link_slack(double stepover) {
  return std::clamp(static_cast<double>(wave::to_integer(stepover)) * 0.05,
                    32.0, 500.0);
}

double subdivision_cleanup_reach(const Operation& operation, double radius,
                                 bool finishing_subdivision) {
  return finishing_subdivision
             ? std::clamp(radius - operation.cleanup_tolerance, 0.0, radius)
             : 0.0;
}

// Everywhere the cutter's centre may be at one depth when nothing has been
// removed there yet: the space beside the stock, minus the part it must not
// touch. The stock stands in air, so that space is empty whether or not this
// pass emptied it -- which is the whole of what a finishing pass has to travel
// through. Its contours are clipped to the stock, so they begin and end on the
// stock wall, and two of them can finish and start a few millimetres apart
// along it with only the wall in between.
wave::Paths64 open_space(const wave::Paths64& stock_slice,
                         const wave::Paths64& keep_out) {
  if (stock_slice.empty()) return {};
  Clipper2Lib::Rect64 world = Clipper2Lib::GetBounds(stock_slice);
  // Room for the tool to stand clear of the stock, and for a route to go round
  // the outside of it rather than being pinned against the edge of the box.
  const std::int64_t margin =
      std::max<std::int64_t>(16, (world.right - world.left) / 4);
  world.left -= margin;
  world.top -= margin;
  world.right += margin;
  world.bottom += margin;
  const wave::Path64 box{
      wave::Point64(world.left, world.top), wave::Point64(world.right, world.top),
      wave::Point64(world.right, world.bottom),
      wave::Point64(world.left, world.bottom)};
  return Clipper2Lib::Difference(
      wave::Paths64{box},
      Clipper2Lib::Union(stock_slice, keep_out, Clipper2Lib::FillRule::NonZero),
      Clipper2Lib::FillRule::NonZero);
}

// `warnings`, when given, receives the notes about engagement that had to be
// reduced. The second stage passes nullptr so a plan carried the rest of the
// way does not say everything twice.
bool resolve_setup(const Operation& operation, const Tool& tool,
                   const Machine& machine, const Material& material,
                   std::string& error, std::vector<std::string>* warnings,
                   OperationSetup& setup) {
  const auto problems = validate(tool);
  if (!problems.empty()) { error = problems.front(); return false; }
  if (!(operation.stepover_ratio > 0.0 && operation.stepover_ratio <= 1.0) ||
      !(operation.stepdown > 0.0) || !std::isfinite(operation.stepdown) ||
      !(operation.spindle_rpm > 0.0) || !std::isfinite(operation.spindle_rpm) ||
      !(operation.plunge_feed > 0.0) || !std::isfinite(operation.plunge_feed)) {
    error = "Stepover must be in (0, 1]; stepdown, spindle and plunge feed must be finite and positive";
    return false;
  }
  const MaterialFeedRange* library_feeds =
      machine.feeds_for(operation.material_id);
  if (!operation.manual_feed_range && !library_feeds) {
    error = "Machine " + machine.name + " has no feed range for " +
            material.name;
    return false;
  }
  setup.full_engagement_feed = operation.manual_feed_range
                                   ? operation.full_engagement_feed
                                   : library_feeds->full_engagement;
  setup.no_engagement_feed = operation.manual_feed_range
                                 ? operation.no_engagement_feed
                                 : library_feeds->no_engagement;
  if (!(setup.full_engagement_feed > 0.0) ||
      setup.no_engagement_feed < setup.full_engagement_feed ||
      !std::isfinite(setup.full_engagement_feed) ||
      !std::isfinite(setup.no_engagement_feed)) {
    error =
        "Feed range must be positive and run from a slower full-engagement "
        "feed to an equal or faster no-engagement feed";
    return false;
  }
  if (operation.path_pitch < 1.0 / wave::kIntegerScale ||
      !std::isfinite(operation.path_pitch)) {
    error = "Path pitch must be at least 0.001 mm";
    return false;
  }
  if (operation.finishing_subdivisions < 1 ||
      operation.finishing_subdivisions > 64) {
    error = "Finishing subdivisions must be between 1 and 64";
    return false;
  }
  if (!(operation.cleanup_stepover_limit > 0.0 &&
        operation.cleanup_stepover_limit <= 1.0)) {
    error = "Cleanup stepover limit must be in (0, 1]";
    return false;
  }
  if (operation.mode == OperationMode::Roughing) {
    if (operation.depth_breaks.size() > 1024) {
      error = "At most 1024 depth breaks are supported";
      return false;
    }
    for (double depth : operation.depth_breaks) {
      if (!std::isfinite(depth) || !(depth > operation.depth_from &&
                                   depth < operation.depth_to)) {
        error = "Depth breaks must be finite and inside the operation depth range";
        return false;
      }
    }
  }
  if (!(operation.entry_front_match_tolerance > 0.0) ||
      !std::isfinite(operation.entry_front_match_tolerance)) {
    error = "Entry-front Z match tolerance must be positive";
    return false;
  }
  if (operation.tiny_split_area_factor < 0.0 ||
      !std::isfinite(operation.tiny_split_area_factor)) {
    error = "Tiny split area factor must be nonnegative";
    return false;
  }
  if (operation.radial_stock_to_leave < 0.0 ||
      !std::isfinite(operation.radial_stock_to_leave)) {
    error = "Radial stock to leave must be finite and nonnegative";
    return false;
  }
  if (!std::isfinite(operation.pocket_radial_margin)) {
    error = "Pocket radial finishing margin must be finite";
    return false;
  }
  if (!std::isfinite(operation.axial_stock_to_leave)) {
    error = "Axial stock to leave must be finite";
    return false;
  }
  if (operation.cleanup_tolerance < 0.0 ||
      !std::isfinite(operation.cleanup_tolerance)) {
    error = "Boundary cleanup tolerance must be finite and nonnegative";
    return false;
  }

  setup.radius = tool.diameter * 0.5;

  setup.stepover = operation.stepover_ratio * tool.diameter;
  const double stepdown = operation.stepdown;

  // Held below the stepover: at or past it the first engaging front misses the
  // stock altogether and the one behind it arrives at more than a full bite,
  // which is the opposite of what the allowance is for.
  setup.first_bite_allowance = std::max(operation.stock_deviation, 0.0);
  if (setup.first_bite_allowance > setup.stepover * 0.9) {
    setup.first_bite_allowance = setup.stepover * 0.9;
    if (warnings)
      warnings->push_back(
          "Stock oversize reduced to " +
          std::to_string(setup.first_bite_allowance) +
          " mm, which is as much as this stepover leaves room for");
  }

  setup.stepdown = stepdown;
  return true;
}

// The floors the operation's depth range lands on. Separate from the setup
// above because depths are measured from the top of the work, and only a caller
// with the geometry in hand knows where that is.
bool resolve_floors(const Operation& operation, const OperationSetup& setup,
                    double work_top, std::string& error,
                    std::vector<double>& floors) {
  floors = level_floors(operation, setup.stepdown, work_top);
  if (floors.empty()) {
    error = "The depth range is empty: it has to start above where it ends";
    return false;
  }
  return true;
}

// A job is usable when any one operation planned. The rest are reported as
// warnings rather than hidden, and a job with none at all carries every reason.
void finish_job(JobPlan& plan) {
  const bool tried = std::any_of(plan.operations.begin(), plan.operations.end(),
                                 [](const OperationPlan& op) { return op.planned; });
  const bool any = std::any_of(
      plan.operations.begin(), plan.operations.end(),
      [](const OperationPlan& op) { return op.planned && op.ok(); });
  if (!tried) {
    plan.error = "No operation has been planned";
  } else if (!any) {
    std::ostringstream error;
    error << "No operation could be planned:";
    for (const OperationPlan& operation : plan.operations) {
      // An operation nothing has yet been asked of is not a failure to
      // report. The list of operations says it has no plan; this is the
      // account of the ones that were tried.
      if (!operation.planned || operation.ok()) continue;
      error << "\n  " << operation.operation_name << ": ";
      std::string indent = operation.error;
      std::size_t newline = 0;
      while ((newline = indent.find('\n', newline)) != std::string::npos) {
        indent.replace(newline, 1, "\n    ");
        newline += 5;
      }
      error << indent;
    }
    plan.error = error.str();
  } else {
    // A usable job may still contain a failed optional operation. Keep that
    // failure visible without making the successfully planned operations
    // unpostable.
    for (const OperationPlan& operation : plan.operations)
      if (operation.planned && !operation.ok())
        plan.warnings.push_back(operation.operation_name + ": " +
                                operation.error);
  }
}

wave::Paths64 stock_after_operation_at_z(
    const wave::Paths64& nominal_stock, double z,
    const OperationPlan& operation) {
  if (nominal_stock.empty() || !operation.ok()) return nominal_stock;
  wave::Paths64 removed;
  for (const LevelPlan& level : operation.levels) {
    if (level.nothing_to_cut || level.simulation.removed_material.empty())
      continue;
    const double slab_bottom = level.floor_z;
    const double slab_top = level.floor_z + level.axial_depth;
    constexpr double kZEpsilon = 1e-9;
    if (z < slab_bottom - kZEpsilon || z > slab_top + kZEpsilon) continue;
    removed.insert(removed.end(), level.simulation.removed_material.begin(),
                   level.simulation.removed_material.end());
  }
  if (removed.empty()) return nominal_stock;
  return Clipper2Lib::Difference(
      nominal_stock,
      Clipper2Lib::Union(removed, Clipper2Lib::FillRule::NonZero),
      Clipper2Lib::FillRule::NonZero);
}

void append_inter_layer(wave::InterLayerTopology& into,
                        wave::InterLayerTopology from,
                        std::size_t layer_offset) {
  const std::size_t node_offset = into.node_chains.size();
  const std::size_t entry_offset = into.entry_front_chains.size();
  for (wave::InterLayerChain& chain : from.node_chains) {
    for (wave::LayerItemId& member : chain.members)
      member.layer += layer_offset;
    into.node_chains.push_back(std::move(chain));
  }
  for (wave::InterLayerChain& chain : from.entry_front_chains) {
    for (wave::LayerItemId& member : chain.members)
      member.layer += layer_offset;
    into.entry_front_chains.push_back(std::move(chain));
  }
  for (auto& ids : from.node_chain_ids) {
    for (std::size_t& id : ids)
      if (id != wave::kNoTopologyId) id += node_offset;
    into.node_chain_ids.push_back(std::move(ids));
  }
  for (auto& ids : from.entry_front_chain_ids) {
    for (std::size_t& id : ids)
      if (id != wave::kNoTopologyId) id += entry_offset;
    into.entry_front_chain_ids.push_back(std::move(ids));
  }
  for (wave::LayerItemId item : from.node_schedule) {
    item.layer += layer_offset;
    into.node_schedule.push_back(item);
  }
  into.layers.insert(into.layers.end(),
                     std::make_move_iterator(from.layers.begin()),
                     std::make_move_iterator(from.layers.end()));
}

// The way from where one pass finishes to where the next one starts. Lifting to
// the rapid plane and coming back down always works and always costs the full
// height of the job twice over -- on a fourteen-pass part that is most of a
// metre of vertical rapid, spent to cross gaps that are often a few
// millimetres wide.
//
// Where the pass that just finished cleared a way across at its own floor, the
// cutter can walk it and step down a single pass depth instead. That way is
// only ever offered when it is quicker, and it is offered by extending the
// earlier pass rather than by inventing motion between the two: the transition
// runs at that pass's depth, through what that pass removed, so it is that
// pass's toolpath that should carry it, and it is that pass's simulation that
// then checks it like any other move at that depth.
void stitch_passes(OperationPlan& plan, const OperationSetup& setup,
                   const Machine& machine, double rapid_plane) {
  const double rapid = std::max(rapid_feed(machine), 1.0);
  const double rapid_z = std::max(rapid_feed_z(machine), 1.0);
  const double travel = std::max(setup.no_engagement_feed, 1.0);
  // The allowance the toolpath builder grants a link at depth, so a transition
  // is held to the same standard as the links inside the pass it extends.
  const double slack = link_slack(setup.stepover);

  LevelPlan* previous = nullptr;
  for (LevelPlan& level : plan.levels) {
    if (level.nothing_to_cut || level.toolpath.moves.empty() ||
        level.toolpath.moves.front().path.empty())
      continue;
    LevelPlan* const earlier = previous;
    previous = &level;
    if (earlier == nullptr || earlier->toolpath.moves.back().path.empty() ||
        earlier->toolpath.cleared.empty())
      continue;

    const wave::Point64 from = earlier->toolpath.moves.back().path.back();
    const wave::Point64 to = level.toolpath.moves.front().path.front();
    // Pulled in by what the router may spend before it is asked for a route.
    // Its slack forgives the disagreement between two roundings of one wall,
    // which is what lets a route run along that wall at all; spent against the
    // part instead it is a cut into the finished surface. Taking it off the
    // region first means a route that uses all of it still only reaches the
    // edge of what the pass actually cleared. It is not free -- a corridor
    // narrower than this stops offering a way through, and the transition falls
    // back to lifting -- but a corridor that narrow is one stepover wide at
    // most, and worth refusing.
    //
    // The two ends are exempt from it. They are where the passes actually
    // finish and start, they lie on the boundary being pulled in, and a router
    // asked to leave from a point outside itself answers nothing at all.
    const wave::Paths64 room =
        wave::routable_region(earlier->toolpath.cleared, slack, {from, to});
    if (room.empty()) continue;
    const wave::Router router(room, slack);
    const auto route = router.route(from, to);
    if (!route) continue;

    double crossing = 0.0;
    for (std::size_t i = 1; i < route->size(); ++i)
      crossing += distance_mm((*route)[i - 1], (*route)[i]);
    const double descent = std::abs(earlier->floor_z - level.floor_z);
    // Up, across the plane, and back down against across at depth and one step
    // down. Both are what the machine is actually commanded to do, so the two
    // are comparable without a fudge factor between them.
    const double lift = (std::abs(rapid_plane - earlier->floor_z) +
                         std::abs(rapid_plane - level.floor_z)) / rapid_z +
                        distance_mm(from, to) / rapid;
    const double stay = crossing / travel + descent / rapid_z;
    if (stay >= lift && !earlier->stay_down_preferred) continue;

    // Kept aside rather than appended here. It joins the pass after the pass
    // has been resampled, because resampling runs consecutive engaged moves
    // together and takes its samples across the whole run: fused with the cuts
    // it extends, a sample interval straddling the join comes out labelled a
    // cut while lying partly on a transition that crosses open air.
    earlier->transition = *route;
    earlier->stays_down_after = true;
  }
}

}  // namespace detail
}  // namespace cam
