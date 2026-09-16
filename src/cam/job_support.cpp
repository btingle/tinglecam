#include "job_internal.h"
#include "cam/stock.h"
#include <unordered_set>
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
  into.parallel_cleanup_milliseconds += from.parallel_cleanup_milliseconds;
  into.cleanup_work_milliseconds += from.cleanup_work_milliseconds;
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

void link_roughing_levels(OperationPlan& plan, const JobGeometry& geometry,
    const Operation& operation, const OperationSetup& setup,
    const Machine& machine, double rapid_plane, const ProgressCallback& progress,
    double progress_from, double progress_to) {
  const bool trace = std::getenv("TINGLECAM_TRACE_LINKS");
  std::size_t active_layer = 0;
  auto stock = plan.preview_context;
  const double travel = std::max(setup.no_engagement_feed, 1.0);
  const double vertical_rate = std::max(rapid_feed_z(machine), 1.0);
  const double slack = link_slack(setup.stepover);
  // Only the integer geometry's one-unit rounding allowance is admitted by
  // the final swept-volume check, never the router's much larger slack.
  const double sweep_radius = std::max(0.0, double(wave::to_integer(setup.radius)) - 1.0);
  std::map<double, wave::Paths64> protected_slices;
  const auto obstacles = [&](double floor, const Clipper2Lib::Rect64* bounds = nullptr) {
    wave::Paths64 blocked;
    const auto append = [&](const wave::Paths64& region) {
      for (const auto& ring : region) {
        if (bounds) {
          const auto b = Clipper2Lib::GetBounds(ring);
          if (b.right < bounds->left || b.left > bounds->right ||
              b.bottom < bounds->top || b.top > bounds->bottom) continue;
        }
        blocked.push_back(ring);
      }
    };
    std::unordered_set<const wave::Paths64*> seen;
    for (const auto& slab : stock) {
      // Project all remaining stock above the tip. This conservatively checks
      // the cutter column, including material above both transition heights.
      if (slab.top <= floor + 1e-9) continue;
      if (seen.insert(slab.region.get()).second) append(*slab.region);
    }
    if (geometry.has_part()) {
      auto [entry, inserted] = protected_slices.try_emplace(floor);
      if (inserted) {
        const double z = std::nextafter(
            static_cast<float>(floor - operation.axial_stock_to_leave),
            std::numeric_limits<float>::infinity());
        // Remaining stock already contains the radial allowance. Re-inflating
        // the nominal part would recreate allowance material that earlier cuts
        // removed (including differences between rounded offset polygons), and
        // falsely obstruct travel out of those cuts. Protect the actual part
        // independently; every remaining stock slab is still checked above.
        entry->second = geometry.part_outline(z, operation.slice_grid);
      }
      append(entry->second);
    }
    // Collision queries need only rings whose bounds meet the cutter sweep.
    // Intersect applies NonZero winding to all clips together, so a preliminary
    // union of the entire stock column is unnecessary. Retain enclosing rings
    // and holes; only wholly disjoint bounds may be discarded. Routing still
    // needs the complete, united obstacle boundary.
    return bounds ? blocked : Union(blocked, FillRule::NonZero);
  };
  // Stock subtraction approximates round caps with polygons. A new sweep at a
  // different angle can meet tiny remnants inside a cutter capsule that has
  // already physically passed through. Prove those remnants were swept; never
  // discard collisions merely because their area is small.
  const auto previously_swept = [&](const wave::Path64& ring, const StockSlab& slab) {
    const auto bounds = Clipper2Lib::GetBounds(ring);
    const long double radius = setup.radius * wave::kIntegerScale;
    const long double radius2 = radius * radius;
    for (std::size_t prior = 0; prior < active_layer; ++prior) {
      const auto& cut = plan.levels[prior];
      if (cut.nothing_to_cut || cut.floor_z > slab.bottom + 1e-9 ||
          cut.floor_z + cut.axial_depth < slab.top - 1e-9) continue;
      for (const auto& move : cut.simulation.moves) for (const auto& step : move.steps) {
        // Convex capsule bounds must contain the whole collision polygon.
        if (bounds.left < std::min(step.from.x, step.to.x) - radius ||
            bounds.right > std::max(step.from.x, step.to.x) + radius ||
            bounds.top < std::min(step.from.y, step.to.y) - radius ||
            bounds.bottom > std::max(step.from.y, step.to.y) + radius) continue;
        const long double dx = step.to.x - step.from.x;
        const long double dy = step.to.y - step.from.y;
        const long double length2 = dx * dx + dy * dy;
        bool contained = true;
        for (const auto& point : ring) {
          const long double px = point.x - step.from.x;
          const long double py = point.y - step.from.y;
          const long double t = length2 > 0 ?
              std::clamp((px * dx + py * dy) / length2, 0.0L, 1.0L) : 0;
          const long double ex = px - t * dx, ey = py - t * dy;
          if (ex * ex + ey * ey >= radius2) { contained = false; break; }
        }
        // A capsule is convex: all vertices inside this SAME capsule proves
        // the entire ring interior lies inside an already-executed cut.
        if (contained) return true;
      }
    }
    return false;
  };
  const auto clear = [&](const wave::Path64& path, double floor) {
    const auto sweep = path.front() == path.back() && path.size() <= 2
        ? wave::Paths64{Clipper2Lib::Ellipse(path.front(), sweep_radius, sweep_radius, 48)}
        : InflatePaths(wave::Paths64{path}, sweep_radius, JoinType::Round, EndType::Round);
    const auto bounds = Clipper2Lib::GetBounds(sweep);
    const auto blocked = obstacles(floor, &bounds);
    if (blocked.empty()) return true;
    const auto collision = Clipper2Lib::Intersect(sweep, blocked, FillRule::NonZero);
    if (trace) std::fprintf(stderr, "link_clear layer=%zu floor=%.3f from=%.3f,%.3f to=%.3f,%.3f collision_mm2=%.9f\n",
        active_layer, floor, wave::to_world(path.front().x), wave::to_world(path.front().y),
        wave::to_world(path.back().x), wave::to_world(path.back().y),
        std::abs(Clipper2Lib::Area(collision)) / (wave::kIntegerScale * wave::kIntegerScale));
    if (collision.empty()) return true;
    // Finished-part protection is independent of stock-removal evidence.
    if (geometry.has_part()) {
      const auto protected_hit = Clipper2Lib::Intersect(sweep, protected_slices.at(floor), FillRule::NonZero);
      if (!protected_hit.empty()) {
        if (trace) std::fprintf(stderr, "link_protected_hit layer=%zu area=%.9f\n", active_layer,
            std::abs(Clipper2Lib::Area(protected_hit)) / (wave::kIntegerScale * wave::kIntegerScale));
        return false;
      }
    }
    for (const auto& slab : stock) {
      if (slab.top <= floor + 1e-9) continue;
      const auto hits = Clipper2Lib::Intersect(sweep, *slab.region, FillRule::NonZero);
      for (const auto& ring : hits)
        if (!previously_swept(ring, slab)) {
          if (trace) std::fprintf(stderr, "link_unswept layer=%zu slab=%.3f,%.3f area=%.9f\n", active_layer, slab.bottom, slab.top,
              std::abs(Clipper2Lib::Area(ring)) / (wave::kIntegerScale * wave::kIntegerScale));
          return false;
        }
    }
    if (trace) std::fprintf(stderr, "link_proven_swept layer=%zu floor=%.3f\n", active_layer, floor);
    return true;
  };
  const auto account = [](FeedStats& stats, const MotionPhase& phase) {
    if (phase.rapid || phase.from_z != phase.to_z) {
      stats.rapid_distance += phase.length;
      stats.rapid_minutes += phase.length / phase.rate;
    } else {
      stats.cutting_distance += phase.length;
      stats.cutting_minutes += phase.length / phase.rate;
      stats.minimum_feed = stats.minimum_feed > 0
          ? std::min(stats.minimum_feed, phase.rate) : phase.rate;
      stats.maximum_feed = std::max(stats.maximum_feed, phase.rate);
    }
  };

  LevelPlan* previous = nullptr;
  for (std::size_t index = 0; index < plan.levels.size(); ++index) {
    if (progress && !progress(progress_from + (progress_to - progress_from) *
                               double(index) / plan.levels.size(),
                               operation.name + ": linking layers")) {
      plan.error = "Cancelled";
      return;
    }
    active_layer = index;
    auto& level = plan.levels[index];
    if (trace) std::fprintf(stderr, "link_level layer=%zu z=%.3f subdivision=%d empty=%d\n", index, level.floor_z, level.finishing_index, level.nothing_to_cut);
    level.feeds.authoritative_motion = true;
    if (level.nothing_to_cut || level.feeds.motion.empty()) continue;
    if (previous) {
      auto& before = previous->feeds;
      auto& after = level.feeds;
      // Preserve every cutting move. Only replace the preceding departure
      // and following arrival, which must consist entirely of rapid motion.
      const auto last_cut = std::find_if(before.motion.rbegin(), before.motion.rend(),
                                         [](const auto& p) { return !p.rapid; });
      const auto first_cut = std::find_if(after.motion.begin(), after.motion.end(),
                                          [](const auto& p) { return !p.rapid; });
      if (last_cut != before.motion.rend() && first_cut != after.motion.end() &&
          last_cut->to_z == previous->floor_z && first_cut->from_z == level.floor_z) {
        const auto from = last_cut->to, to = first_cut->from;
        const double low = std::min(previous->floor_z, level.floor_z);
        const double high = std::max(previous->floor_z, level.floor_z);
        // Lift first when moving up; descend last when moving down. Proving
        // the lower endpoint's entire column clear proves the vertical leg.
        const auto lower_point = previous->floor_z < level.floor_z ? from : to;
        if (high < rapid_plane && sweep_radius > 0 && clear({lower_point}, low)) {
          std::optional<wave::Path64> route = wave::Path64{from, to};
          const bool direct_clear = clear(*route, high);
          if (!direct_clear) {
            const auto upper_obstacles = obstacles(high);
            auto bounds = Clipper2Lib::GetBounds(upper_obstacles);
            bounds.left = std::min({bounds.left, from.x, to.x});
            bounds.right = std::max({bounds.right, from.x, to.x});
            bounds.top = std::min({bounds.top, from.y, to.y});
            bounds.bottom = std::max({bounds.bottom, from.y, to.y});
            const auto margin = wave::to_integer(setup.radius * 4 + 10);
            bounds.left -= margin; bounds.right += margin;
            bounds.top -= margin; bounds.bottom += margin;
            const wave::Paths64 world{{{bounds.left, bounds.top}, {bounds.right, bounds.top},
                                       {bounds.right, bounds.bottom}, {bounds.left, bounds.bottom}}};
            const auto free = Clipper2Lib::Difference(world,
                offset_outward(upper_obstacles, setup.radius), FillRule::NonZero);
            const wave::Router router(wave::routable_region(free, slack, {from, to}), slack);
            route = router.route(from, to);
          }
          if (trace && !route) std::fprintf(stderr, "link_no_route layer=%zu\n", index);
          if (route && !route->empty() && (direct_clear || clear(*route, high))) {
            std::vector<MotionPhase> seam;
            const double path_distance = first_cut->path_distance_begin;
            const auto add = [&](const wave::Point64& a, const wave::Point64& b,
                                 double za, double zb) {
              MotionPhase phase;
              phase.from = a; phase.to = b; phase.from_z = za; phase.to_z = zb;
              phase.path_distance_begin = phase.path_distance_end = path_distance;
              phase.length = std::hypot(distance_mm(a, b), zb - za);
              phase.rapid = za != zb;
              phase.rate = phase.rapid ? vertical_rate : travel;
              phase.kind = wave::MoveKind::Link;
              phase.inter_layer = true;
              if (phase.length > 0) seam.push_back(phase);
            };
            add(from, from, previous->floor_z, high);
            for (std::size_t i = 1; i < route->size(); ++i)
              add((*route)[i-1], (*route)[i], high, high);
            add(to, to, high, level.floor_z);
            const auto departure = last_cut.base();
            double old_minutes = 0, new_minutes = 0;
            for (auto i = departure; i != before.motion.end(); ++i) old_minutes += i->length / i->rate;
            for (auto i = after.motion.begin(); i != first_cut; ++i) old_minutes += i->length / i->rate;
            for (const auto& phase : seam) new_minutes += phase.length / phase.rate;
            if (trace) std::fprintf(stderr, "link_cost layer=%zu old_seconds=%.6f new_seconds=%.6f\n", index, old_minutes*60, new_minutes*60);
            if (new_minutes < old_minutes) {
              before.motion.erase(departure, before.motion.end());
              after.motion.erase(after.motion.begin(), first_cut);
              if (!seam.empty()) seam.front().arrival = true;
              after.motion.insert(after.motion.begin(), seam.begin(), seam.end());
              previous->stays_down_after = before.ends_at_depth = true;
              after.begins_at_depth = true;
            }
          }
        }
      }
    }
    stock = stock_outside(stock, level.simulation.removed_material,
                           level.floor_z, level.floor_z + level.axial_depth);
    previous = &level;
  }
  plan.stats.cutting_distance = plan.stats.rapid_distance = 0;
  plan.stats.cutting_minutes = plan.stats.rapid_minutes = 0;
  for (auto& level : plan.levels) {
    if (level.nothing_to_cut) continue;
    // Planar segments can be merged at slightly different unrounded feeds.
    // The XYZ timeline is authoritative now, so derive travel and timing
    // directly from it rather than adjusting those older segment totals.
    auto& stats = level.feeds.stats;
    stats.cutting_distance = stats.rapid_distance = 0;
    stats.cutting_minutes = stats.rapid_minutes = 0;
    for (const auto& phase : level.feeds.motion) account(stats, phase);
    plan.stats.cutting_distance += level.feeds.stats.cutting_distance;
    plan.stats.rapid_distance += level.feeds.stats.rapid_distance;
    plan.stats.cutting_minutes += level.feeds.stats.cutting_minutes;
    plan.stats.rapid_minutes += level.feeds.stats.rapid_minutes;
  }
  if (progress && !progress(progress_to, operation.name + ": linked layers"))
    plan.error = "Cancelled";
}

}  // namespace detail
}  // namespace cam
