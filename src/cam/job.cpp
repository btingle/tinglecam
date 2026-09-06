#include "cam/job.h"

#include "routing.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iterator>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace cam {
namespace {

using PlanningClock = std::chrono::steady_clock;

double elapsed_milliseconds(PlanningClock::time_point start) {
  return std::chrono::duration<double, std::milli>(PlanningClock::now() - start)
      .count();
}

template <typename Function>
void parallel_for_levels(const std::vector<std::size_t>& levels,
                         Function&& function) {
  if (levels.empty()) return;
  std::atomic<std::size_t> next{0};
  const std::size_t workers = std::min<std::size_t>(4, levels.size());
  std::vector<std::thread> threads;
  threads.reserve(workers);
  for (std::size_t worker = 0; worker < workers; ++worker) {
    threads.emplace_back([&] {
      for (;;) {
        const std::size_t work = next.fetch_add(1, std::memory_order_relaxed);
        if (work >= levels.size()) return;
        function(levels[work]);
      }
    });
  }
  for (std::thread& thread : threads) thread.join();
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

struct OperationSetup {
  const CuttingData* data = nullptr;
  double full_engagement_feed = 0.0;
  double no_engagement_feed = 0.0;
  double radius = 0.0;
  double stepover = 0.0;
  double first_bite_allowance = 0.0;
  // After the tool's axial ceiling has been applied to what the operation asked
  // for. The floors are not resolved here: they need the top of the work, which
  // only a caller holding the geometry can supply.
  double stepdown = 0.0;
};

// `warnings`, when given, receives the notes about engagement that had to be
// reduced. The second stage passes nullptr so a plan carried the rest of the
// way does not say everything twice.
bool resolve_setup(const Operation& operation, const Tool& tool,
                   const Machine& machine, const Material& material,
                   std::string& error, std::vector<std::string>* warnings,
                   OperationSetup& setup) {
  setup.data = tool.data_for(operation.material_id);
  if (!setup.data) {
    error = "Tool " + tool.name + " has no cutting data for " +
            operation.material_id;
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
  if (operation.cleanup_tolerance < 0.0 ||
      !std::isfinite(operation.cleanup_tolerance)) {
    error = "Boundary cleanup tolerance must be finite and nonnegative";
    return false;
  }

  setup.radius = tool.diameter * 0.5;

  // Engagement, capped by what the tool says it will take. Exceeding the book
  // figure silently is how a plan looks fine and cuts badly, so the ceiling is
  // applied here and reported rather than left to the operator to remember.
  double stepover_ratio = operation.stepover_ratio;
  if (setup.data->max_ae_over_d > 0.0 &&
      stepover_ratio > setup.data->max_ae_over_d) {
    if (warnings)
      warnings->push_back(
          "Stepover reduced to the tool's ceiling of " +
          std::to_string(setup.data->max_ae_over_d) + " of diameter");
    stepover_ratio = setup.data->max_ae_over_d;
  }
  setup.stepover = std::max(stepover_ratio * tool.diameter, 1e-4);

  double stepdown = operation.stepdown;
  const double axial_ceiling =
      std::min(tool.flute_length, setup.data->max_ap_over_d > 0.0
                                      ? setup.data->max_ap_over_d * tool.diameter
                                      : tool.flute_length);
  if (axial_ceiling > 0.0 && stepdown > axial_ceiling) {
    if (warnings)
      warnings->push_back("Stepdown reduced to " + std::to_string(axial_ceiling) +
                          " mm, the deepest this tool takes in one pass");
    stepdown = axial_ceiling;
  }

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
  const bool any = std::any_of(plan.operations.begin(), plan.operations.end(),
                               [](const OperationPlan& op) { return op.ok(); });
  if (!any) {
    std::ostringstream error;
    error << "No operation could be planned:";
    for (const OperationPlan& operation : plan.operations) {
      if (operation.ok()) continue;
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
      if (!operation.ok())
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
    const wave::Paths64 room = Clipper2Lib::InflatePaths(
        earlier->toolpath.cleared, -2.0 * slack, Clipper2Lib::JoinType::Miter,
        Clipper2Lib::EndType::Polygon);
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
    if (stay >= lift) continue;

    // Kept aside rather than appended here. It joins the pass after the pass
    // has been resampled, because resampling runs consecutive engaged moves
    // together and takes its samples across the whole run: fused with the cuts
    // it extends, a sample interval straddling the join comes out labelled a
    // cut while lying partly on a transition that crosses open air.
    earlier->transition = *route;
    earlier->stays_down_after = true;
  }
}

}  // namespace

const char* operation_mode_name(OperationMode value) {
  switch (value) {
    case OperationMode::Roughing: return "roughing";
    case OperationMode::ContourFinishing: return "contour_finishing";
  }
  return "unknown";
}

void JobGeometry::load_part(const std::filesystem::path& file) {
  part_.set_mesh(wave::load_triangle_mesh(file));
  part_slices_.clear();
  box_.clear();
}

void JobGeometry::load_stock(const std::filesystem::path& file) {
  stock_.set_mesh(wave::load_triangle_mesh(file));
  stock_slices_.clear();
}

void JobGeometry::clear_stock() {
  stock_.clear();
  stock_slices_.clear();
}

void JobGeometry::set_box_stock(bool enabled, double margin) {
  if (box_stock_ == enabled && box_margin_ == margin) return;
  box_stock_ = enabled;
  box_margin_ = margin;
  box_.clear();
  stock_slices_.clear();
}

void JobGeometry::forget_slices() {
  part_slices_.clear();
  stock_slices_.clear();
}

const wave::Paths64& JobGeometry::outline(const wave::MeshSlicer& slicer,
                                          std::map<SliceKey, wave::Paths64>& cache,
                                          double z, int grid) const {
  // The float the slicer takes is the key, not the double asked for, so two
  // depths that reach the same slice share one entry rather than cutting twice.
  const SliceKey key{
      static_cast<std::int64_t>(
          std::llround(static_cast<double>(static_cast<float>(z)) *
                       wave::kIntegerScale)),
      grid};
  const auto found = cache.find(key);
  if (found != cache.end()) return found->second;
  const auto slice = slicer.slice(static_cast<float>(z), grid);
  return cache.emplace(key, Union(slice.polygons, FillRule::NonZero))
      .first->second;
}

const wave::Paths64& JobGeometry::part_outline(double z, int grid) const {
  return outline(part_, part_slices_, z, grid);
}

const wave::Paths64& JobGeometry::stock_outline(double z, int grid) const {
  if (!box_stock()) return outline(stock_, stock_slices_, z, grid);
  // One rectangle, the same at every depth, so the depth is not part of the
  // question. An empty part leaves it empty and the level reports nothing to
  // cut, which is what a missing model should look like.
  if (box_.empty() && !part_.empty()) {
    const auto& bounds = part_.mesh().bounds;
    if (bounds.valid) {
      const double margin = std::max(box_margin_, 0.0);
      const std::int64_t left = wave::to_integer(bounds.lower.x - margin);
      const std::int64_t right = wave::to_integer(bounds.upper.x + margin);
      const std::int64_t bottom = wave::to_integer(bounds.lower.y - margin);
      const std::int64_t top = wave::to_integer(bounds.upper.y + margin);
      box_ = wave::Paths64{wave::Path64{{left, bottom},
                                        {right, bottom},
                                        {right, top},
                                        {left, top}}};
    }
  }
  return box_;
}

bool JobGeometry::work_range(double& low, double& high) const {
  // The union, not the overlap. Either model on its own is a reason to be at a
  // depth: blank above the part is material to face off, part below the blank
  // is a hole to descend into. Taking the overlap quietly clipped both away,
  // which is how a blank standing proud of its part came to be left standing.
  bool any = false;
  double lowest = 0.0;
  double highest = 0.0;
  for (const wave::MeshSlicer* slicer : {&part_, &stock_}) {
    if (slicer->empty()) continue;
    const auto& bounds = slicer->mesh().bounds;
    if (!bounds.valid) continue;
    if (!any) {
      lowest = bounds.lower.z;
      highest = bounds.upper.z;
      any = true;
      continue;
    }
    lowest = std::min<double>(lowest, bounds.lower.z);
    highest = std::max<double>(highest, bounds.upper.z);
  }
  if (!any || !(highest > lowest)) return false;
  low = lowest;
  high = highest;
  return true;
}

std::vector<double> level_floors(const Operation& operation, double stepdown,
                                 double work_top) {
  std::vector<double> floors;
  if (!std::isfinite(work_top)) return floors;
  const double top = z_at_depth(work_top, operation.depth_from);
  const double bottom =
      z_at_depth(work_top, operation.depth_to) + operation.axial_stock_to_leave;
  if (!(top > bottom)) return floors;
  const double step = std::max(stepdown, 1e-6);
  double z = top;
  // A guard rather than a limit: at a sane stepdown no real part gets close.
  for (int guard = 0; guard < 20000; ++guard) {
    z -= step;
    if (z <= bottom + 1e-9) {
      floors.push_back(bottom);
      break;
    }
    floors.push_back(z);
  }
  return floors;
}

OperationPlan plan_wavefronts(const JobGeometry& geometry,
                              const Operation& operation,
                              const Tool& tool,
                              const Machine& machine,
                              const Material& material,
                              const ProgressCallback& progress,
                              double progress_from,
                              double progress_to,
                              const std::vector<OperationPlan>* prior_operations,
                              const OperationPlan* prior_subdivisions) {
  OperationPlan plan;
  plan.operation_name = operation.name;
  plan.tool_id = operation.tool_id;
  plan.machine_id = operation.machine_id;
  plan.material_id = operation.material_id;

  if (!geometry.has_part()) {
    plan.error = "No part mesh is loaded";
    return plan;
  }
  if (!geometry.has_stock()) {
    plan.error = "No stock mesh is loaded";
    return plan;
  }
  OperationSetup setup;
  if (!resolve_setup(operation, tool, machine, material, plan.error,
                     &plan.warnings, setup))
    return plan;
  const double radius = setup.radius;
  const double stepover = setup.stepover;
  const double exterior_stock_reach = subdivision_cleanup_reach(
      operation, radius, prior_subdivisions != nullptr);
  const double first_bite_allowance = setup.first_bite_allowance;
  const double work_top = highest_model_z(geometry);
  std::vector<double> floors;
  if (!resolve_floors(operation, setup, work_top, plan.error, floors))
    return plan;

  plan.levels.resize(floors.size());
  std::vector<wave::Paths64> keep_outs(floors.size());
  std::vector<std::size_t> wave_levels;
  wave_levels.reserve(floors.size());
  std::vector<std::exception_ptr> wave_errors(floors.size());
  std::vector<std::string> level_failures;
  double previous_z = z_at_depth(work_top, operation.depth_from);
  const double span = std::max(progress_to - progress_from, 0.0);

  for (std::size_t index = 0; index < floors.size(); ++index) {
    const double floor_z = floors[index];
    if (progress) {
      const double fraction =
          progress_from + span * (static_cast<double>(index) / floors.size());
      if (!progress(fraction, operation.name + ": depth " +
                                  std::to_string(floor_z) + " mm")) {
        plan.error = "Cancelled";
        return plan;
      }
    }

    LevelPlan& level = plan.levels[index];
    level.floor_z = floor_z;
    level.axial_depth = previous_z - floor_z;
    previous_z = floor_z;

    try {
      // The slicer returns the downward envelope: everything at or above this
      // depth, projected. That is exactly the material that has to survive the
      // pass, so no separate rest model is needed for roughing.
      const auto slice_start = PlanningClock::now();
      level.part_slice = geometry.part_outline(floor_z, operation.slice_grid);
      level.stock_slice = geometry.stock_outline(floor_z, operation.slice_grid);
      level.incoming_stock =
          prior_operations == nullptr
              ? level.stock_slice
              : stock_after_operations_at_z(level.stock_slice, floor_z,
                                             *prior_operations);
      if (prior_subdivisions != nullptr)
        level.incoming_stock = stock_after_operation_at_z(
            level.incoming_stock, floor_z, *prior_subdivisions);
      plan.metrics.slicing_milliseconds += elapsed_milliseconds(slice_start);

      if (level.stock_slice.empty() || level.incoming_stock.empty()) {
        level.nothing_to_cut = true;
        continue;
      }

      // The cutter follows a path whose centre must stay a radius clear of the
      // part, plus whatever is being left for a finishing pass.
      const auto preparation_start = PlanningClock::now();
      keep_outs[index] =
          level.part_slice.empty()
              ? absent_subject(level.incoming_stock)
              : offset_outward(level.part_slice,
                               radius + operation.radial_stock_to_leave);
      plan.metrics.preparation_milliseconds +=
          elapsed_milliseconds(preparation_start);

      // Finishing does not propagate a wave, so it has no entry fronts to
      // offer and nothing to discover here. Its contours are derived from the
      // slices this stage already took, which happens with the rest of the
      // toolpath work.
      //
      // Except where the part does not reach this depth at all. There is no
      // surface to finish there because there is no part there -- but there is
      // stock, and all of it is waste. Skipping the depth leaves that waste
      // standing and the passes below it cutting into a blank that is still the
      // wrong shape, so it is cleared instead, by the same wave a roughing pass
      // would use against the same absent subject.
      if (operation.mode == OperationMode::ContourFinishing &&
          !level.part_slice.empty()) {
        continue;
      }

      // The run-in is seeded a tool radius plus the stock allowance in front
      // of the wall. The front one stepover behind that is the first to touch
      // the material, and it bites a stepover less the allowance -- so a blank
      // up to that much oversize still takes no more than a full stepover on
      // its first pass, and one exactly on size takes very nearly a full one.
      // Without this the run-in landed a front on the wall itself and the pass
      // before it grazed off whatever fraction of a stepover was left over.
      wave_levels.push_back(index);
    } catch (const std::exception& error) {
      const std::string detail =
          "Depth " + std::to_string(floor_z) +
          " mm, wavefront construction: " + error.what();
      level_failures.push_back(detail);
      plan.warnings.push_back(detail);
      level.nothing_to_cut = true;
    }
  }

  const double lead_in =
      operation.lead_in ? radius + first_bite_allowance : 0.0;
  const auto wavefront_start = PlanningClock::now();
  parallel_for_levels(wave_levels, [&](std::size_t index) {
    LevelPlan& level = plan.levels[index];
    try {
      wave::WavefrontAnalysis analysis = wave::analyze_wavefront(
          keep_outs[index], level.incoming_stock, stepover, stepover, lead_in,
          2000, 128, {}, wave::kDefaultSignificantNodeAreaFraction,
          operation.tiny_split_area_factor, exterior_stock_reach);
      wave::CommittedWavefront committed =
          wave::commit_wavefront_analysis(analysis);
      level.wave_stages = std::move(committed.stages);
      level.wave = std::move(committed.wave);
      level.lineage = std::move(committed.lineage);
      level.topology = std::move(committed.topology);
      for (const wave::WavefrontDecisionLayer& decision : analysis.layers)
        for (const wave::WavefrontStrategyTrial& trial : decision.trials) {
          ++level.analysis_trials;
          if (trial.accepted) ++level.accepted_analysis_trials;
        }
    } catch (...) {
      wave_errors[index] = std::current_exception();
    }
  });
  plan.metrics.wavefront_milliseconds +=
      elapsed_milliseconds(wavefront_start);

  for (const std::size_t index : wave_levels) {
    LevelPlan& level = plan.levels[index];
    if (wave_errors[index]) {
      try {
        std::rethrow_exception(wave_errors[index]);
      } catch (const std::exception& error) {
        const std::string detail =
            "Depth " + std::to_string(level.floor_z) +
            " mm, wavefront construction: " + error.what();
        level_failures.push_back(detail);
        plan.warnings.push_back(detail);
      }
      level.nothing_to_cut = true;
      continue;
    }
    plan.metrics.analysis_layers += level.wave_stages.size();
    plan.metrics.analysis_trials += level.analysis_trials;
    plan.metrics.accepted_analysis_trials +=
        level.accepted_analysis_trials;
    for (const wave::WavefrontStage& stage : level.wave_stages) {
      plan.metrics.wave_generations += stage.lineage.generations.size();
      for (const wave::FrontGeneration& generation : stage.lineage.generations)
        plan.metrics.fronts += generation.fronts.size();
    }
  }

  // Inter-layer associations need every slice at once, so wavefront planning
  // is deliberately complete before any layer is sequenced into tool motion.
  // A known boundary-component mismatch is an internal topology failure and
  // aborts the operation instead of quietly emitting an arbitrary link.
  try {
    std::vector<const wave::FrontLineage*> lineages;
    std::vector<const wave::LayerTopology*> topologies;
    lineages.reserve(plan.levels.size());
    topologies.reserve(plan.levels.size());
    for (const LevelPlan& level : plan.levels) {
      lineages.push_back(&level.lineage);
      topologies.push_back(&level.topology);
    }
    const auto inter_layer_start = PlanningClock::now();
    plan.inter_layer = wave::build_inter_layer_topology(
        lineages, operation.entry_front_match_tolerance, topologies);
    plan.metrics.inter_layer_milliseconds +=
        elapsed_milliseconds(inter_layer_start);
  } catch (const std::exception& error) {
    plan.error = "Inter-layer topology: " + std::string(error.what());
    return plan;
  }

  return plan;
}

double highest_model_z(const JobGeometry& geometry) {
  double highest = -std::numeric_limits<double>::infinity();
  for (const wave::MeshSlicer* slicer : {&geometry.part(), &geometry.stock()}) {
    if (slicer->empty()) continue;
    const wave::Bounds3f& bounds = slicer->mesh().bounds;
    if (bounds.valid) highest = std::max<double>(highest, bounds.upper.z);
  }
  // A rectangular blank has no mesh, and stands only as tall as the part it was
  // built around, so the part above already covers it.
  return highest;
}

double job_rapid_plane(const Job& job, double highest) {
  // A clearance above the work, not a height above the origin. Where the origin
  // sits is a matter of where it is convenient to measure from, and it moves;
  // what a traverse has to clear is the tallest thing in the way, and that does
  // not. Tying the plane to the origin meant moving the origin moved the plane,
  // which moved what every lift cost, which meant re-planning the job to say
  // where its zero was.
  //
  // The tallest thing in the way is the taller of the part and the blank. Not
  // the top of the cut: an operation that stops short still has to traverse
  // over whatever it is leaving above itself.
  if (!std::isfinite(highest)) return job.rapid_clearance_height;
  return highest + job.rapid_clearance_height;
}

void plan_toolpaths(OperationPlan& plan,
                    const Operation& operation,
                    const Tool& tool,
                    const Machine& machine,
                    const Material& material,
                    const ProgressCallback& progress,
                    double progress_from,
                    double progress_to,
                    double rapid_plane,
                    bool finishing_subdivision) {
  if (!plan.ok()) return;
  if (!std::isfinite(rapid_plane)) {
    plan.error = "Rapid plane must be finite";
    return;
  }
  // The meshes are already sliced into the plan, so this stage wants the
  // engagement figures back and nothing else. Its warnings were raised when the
  // wavefronts were built, so they are not raised again.
  OperationSetup setup;
  if (!resolve_setup(operation, tool, machine, material, plan.error, nullptr,
                     setup))
    return;
  const CuttingData* data = setup.data;
  const double radius = setup.radius;
  const double cleanup_reach =
      subdivision_cleanup_reach(operation, radius, finishing_subdivision);
  const double full_engagement_feed = setup.full_engagement_feed;
  const double no_engagement_feed = setup.no_engagement_feed;
  const double span = std::max(progress_to - progress_from, 0.0);
  std::vector<std::string> level_failures;

  // Toolpaths first, for every depth, and only then the seams between them.
  // Deciding how the cutter gets from one pass to the next takes both passes in
  // hand at once -- where this one finishes, and where the next one starts --
  // so nothing downstream of that decision can run until every pass has been
  // built. Simulation and feeds come last, after the seams have been settled,
  // which is what keeps a transition an ordinary part of the pass it belongs to
  // rather than motion that nothing ever checks.
  const double build_share = 0.7;
  std::vector<std::size_t> toolpath_levels;
  toolpath_levels.reserve(plan.levels.size());
  for (std::size_t index = 0; index < plan.levels.size(); ++index) {
    LevelPlan& level = plan.levels[index];
    if (level.nothing_to_cut) continue;
    if (level.incoming_stock.empty()) {
      // This is a valid result of chaining, not a planning failure: an earlier
      // operation already removed every bit of stock in this slab.
      level.nothing_to_cut = true;
      continue;
    }
    if (progress) {
      const double fraction =
          progress_from +
          span * build_share * (static_cast<double>(index) / plan.levels.size());
      if (!progress(fraction, operation.name + ": toolpath at depth " +
                                  std::to_string(level.floor_z) + " mm")) {
        plan.error = "Cancelled";
        return;
      }
    }
    toolpath_levels.push_back(index);
  }

  std::vector<std::exception_ptr> toolpath_errors(plan.levels.size());
  const auto toolpath_start = PlanningClock::now();
  parallel_for_levels(toolpath_levels, [&](std::size_t index) {
    LevelPlan& level = plan.levels[index];
    try {
      // A depth the part does not reach is cleared rather than finished, in
      // either mode: see the wavefront stage, which is where the wave for it
      // was built.
      const bool clearing = level.part_slice.empty();
      if (operation.mode == OperationMode::ContourFinishing && !clearing) {
        const wave::Paths64 keep_out =
            level.part_slice.empty()
                ? absent_subject(level.stock_slice)
                : offset_outward(level.part_slice,
                                 radius + operation.radial_stock_to_leave);
        const auto contours = derive_offset_contours(
            level.part_slice, radius + operation.radial_stock_to_leave,
            operation.clip_contours_to_stock ? &level.stock_slice : nullptr);
        auto tasks = contour_traversal_tasks(contours,
                                             operation.contour_direction);
        add_safe_stock_approaches(tasks, level.stock_slice, keep_out, radius);
        const wave::Paths64 travelable =
            open_space(level.stock_slice, keep_out);
        wave::TraversalSettings traversal;
        traversal.allow_stay_down = true;
        traversal.stay_down_region = travelable;
        traversal.stay_down_slack = link_slack(setup.stepover);
        traversal.cutting_feed = no_engagement_feed;
        traversal.rapid_feed = rapid_feed(machine);
        traversal.rapid_feed_z = rapid_feed_z(machine);
        traversal.floor_z = level.floor_z;
        traversal.rapid_plane = rapid_plane;
        level.toolpath = wave::plan_traversal(tasks, traversal).toolpath;
        level.toolpath.cleared = travelable;
      }
      if (operation.mode == OperationMode::Roughing || clearing) {
        level.toolpath = wave::build_toolpath(
            level.wave_stages, radius, level.floor_z, rapid_plane,
            rapid_feed(machine), no_engagement_feed, rapid_feed_z(machine));
      }
    } catch (...) {
      toolpath_errors[index] = std::current_exception();
    }
  });
  plan.metrics.toolpath_milliseconds += elapsed_milliseconds(toolpath_start);

  std::vector<std::size_t> cleanup_levels;
  cleanup_levels.reserve(toolpath_levels.size());
  for (const std::size_t index : toolpath_levels) {
    LevelPlan& level = plan.levels[index];
    if (toolpath_errors[index]) continue;
    const bool clearing = level.part_slice.empty();
    if ((operation.mode == OperationMode::Roughing || clearing) &&
        operation.rough_boundary_cleanup && !level.part_slice.empty() &&
        (cleanup_reach > 0.0 ||
         !level.wave.reached_generations.empty()))
      cleanup_levels.push_back(index);
  }

  std::vector<std::exception_ptr> cleanup_errors(plan.levels.size());
  const auto cleanup_start = PlanningClock::now();
  parallel_for_levels(cleanup_levels, [&](std::size_t index) {
    LevelPlan& level = plan.levels[index];
    try {
      // A centreline inside stock is safe only where the wave opened a route to
      // it. The optional tolerance shell also admits a centreline just outside
      // stock, where the cut approaches the edge from air. Keeping those cases
      // separate admits thin subdivision remnants without authorizing a
      // full-width cleanup through an unopened pocket.
      wave::Paths64 clip;
      if (!level.wave.reached_generations.empty()) {
        // The contour lies exactly on the reached region's inner boundary,
        // both being the same slice offset by the same distance, so allow the
        // small disagreement introduced by their independent rounding.
        clip = Clipper2Lib::InflatePaths(
            level.wave.reached_generations.back(),
            wave::boundary_slack(setup.stepover), Clipper2Lib::JoinType::Miter,
            Clipper2Lib::EndType::Polygon);
      }
      if (operation.clip_contours_to_stock) {
        const wave::Paths64 reached_stock = Clipper2Lib::Intersect(
            clip, level.incoming_stock, Clipper2Lib::FillRule::NonZero);
        clip = cleanup_reach > 0.0
                   ? Clipper2Lib::Union(
                         reached_stock,
                         stock_exterior_tolerance(level.incoming_stock,
                                                  cleanup_reach),
                         Clipper2Lib::FillRule::NonZero)
                   : reached_stock;
      }
      const auto contours = derive_offset_contours(
          level.part_slice, radius + operation.radial_stock_to_leave, &clip);
      auto tasks =
          contour_traversal_tasks(contours, operation.contour_direction);
      add_safe_stock_approaches(tasks, level.incoming_stock, level.wave.subject,
                                radius);
      wave::TraversalSettings traversal;
      traversal.allow_stay_down = true;
      // What the pass has cleared, which is the space beside the stock as
      // well as the material the wave took out. The reached region on its
      // own stops at the stock wall, so a cleanup contour ending on that
      // wall could not be joined to the next one round the outside.
      traversal.stay_down_region = level.toolpath.cleared;
      traversal.stay_down_slack = link_slack(setup.stepover);
      traversal.cutting_feed = no_engagement_feed;
      traversal.rapid_feed = rapid_feed(machine);
      traversal.rapid_feed_z = rapid_feed_z(machine);
      traversal.floor_z = level.floor_z;
      traversal.rapid_plane = rapid_plane;
      if (!level.toolpath.moves.empty() &&
          !level.toolpath.moves.back().path.empty())
        traversal.initial_position = level.toolpath.moves.back().path.back();
      traversal.prefer_stay_down_when_legal = true;
      wave::Toolpath cleanup = wave::plan_traversal(tasks, traversal).toolpath;
      level.toolpath.moves.insert(level.toolpath.moves.end(),
                                  cleanup.moves.begin(), cleanup.moves.end());
    } catch (...) {
      cleanup_errors[index] = std::current_exception();
    }
  });
  plan.metrics.cleanup_milliseconds += elapsed_milliseconds(cleanup_start);

  for (const std::size_t index : toolpath_levels) {
    LevelPlan& level = plan.levels[index];
    if (toolpath_errors[index]) {
      try {
        std::rethrow_exception(toolpath_errors[index]);
      } catch (const std::exception& error) {
        const std::string detail =
            "Depth " + std::to_string(level.floor_z) +
            " mm, toolpath: " + error.what();
        level_failures.push_back(detail);
        plan.warnings.push_back(detail);
      }
      level.nothing_to_cut = true;
      continue;
    }
    if (cleanup_errors[index]) {
      try {
        std::rethrow_exception(cleanup_errors[index]);
      } catch (const std::exception& error) {
        plan.warnings.push_back(
            "Depth " + std::to_string(level.floor_z) +
            " mm, optional boundary cleanup skipped: " + error.what());
      }
    }
    // A subdivision may leave less than one wavefront stepover against the
    // boundary. In that case the propagated wave has no cutting contour, but
    // the optional exterior tolerance shell can still authorize edge cleanup
    // from air.
    // Decide that the whole level is empty only after cleanup had that chance.
    if (level.toolpath.moves.empty()) {
      level.nothing_to_cut = true;
      continue;
    }
    plan.metrics.toolpath_moves += level.toolpath.moves.size();
  }

  const auto stitching_start = PlanningClock::now();
  stitch_passes(plan, setup, machine, rapid_plane);
  plan.metrics.stitching_milliseconds +=
      elapsed_milliseconds(stitching_start);

  // ---- Resampling --------------------------------------------------------
  // Normalize every path before simulation. This remains ordered so progress
  // callbacks run only on the coordinator thread and cancellation never races
  // a geometry worker.
  std::vector<std::size_t> simulated_levels;
  simulated_levels.reserve(plan.levels.size());
  for (std::size_t index = 0; index < plan.levels.size(); ++index) {
    LevelPlan& level = plan.levels[index];
    if (level.nothing_to_cut) continue;
    if (progress) {
      const double fraction =
          progress_from + span * build_share +
          span * (1.0 - build_share) *
              (static_cast<double>(index) / plan.levels.size());
      if (!progress(fraction, operation.name + ": feeds at depth " +
                                  std::to_string(level.floor_z) + " mm")) {
        plan.error = "Cancelled";
        return;
      }
    }
    try {
      const auto resampling_start = PlanningClock::now();
      level.toolpath =
          wave::resample_toolpath(level.toolpath, operation.path_pitch);
      if (level.stays_down_after && level.transition.size() >= 2) {
        // Sampled the same way everything else at depth is, so the simulation
        // sees it in the same steps and can say what it passes over.
        wave::Toolpath crossing;
        double length = 0.0;
        for (std::size_t i = 1; i < level.transition.size(); ++i)
          length += distance_mm(level.transition[i - 1], level.transition[i]);
        crossing.moves.push_back(
            {wave::MoveKind::Link, level.transition, {}, length});
        crossing = wave::resample_toolpath(crossing, operation.path_pitch);
        for (wave::ToolpathMove& move : crossing.moves) {
          level.toolpath.stats.link_length += move.length;
          ++level.toolpath.stats.link_count;
          level.toolpath.moves.push_back(std::move(move));
        }
      }
      plan.metrics.resampling_milliseconds +=
          elapsed_milliseconds(resampling_start);
      simulated_levels.push_back(index);
    } catch (const std::exception& error) {
      const std::string detail =
          "Depth " + std::to_string(level.floor_z) +
          " mm, simulation or feeds: " + error.what();
      level_failures.push_back(detail);
      plan.warnings.push_back(detail);
      level.nothing_to_cut = true;
    }
  }

  // ---- Simulation --------------------------------------------------------
  // Depths have independent stock slices and toolpaths. A small fixed bound
  // captures the available parallelism without multiplying the transient
  // Clipper working set by the number of levels in a deep job.
  std::vector<std::exception_ptr> simulation_errors(plan.levels.size());
  const auto simulation_start = PlanningClock::now();
  if (!simulated_levels.empty()) {
    std::atomic<std::size_t> next{0};
    const std::size_t workers =
        std::min<std::size_t>(4, simulated_levels.size());
    std::vector<std::thread> threads;
    threads.reserve(workers);
    for (std::size_t worker = 0; worker < workers; ++worker) {
      threads.emplace_back([&] {
        for (;;) {
          const std::size_t work = next.fetch_add(1, std::memory_order_relaxed);
          if (work >= simulated_levels.size()) return;
          const std::size_t index = simulated_levels[work];
          LevelPlan& level = plan.levels[index];
          try {
            const wave::Paths64& final_subject =
                level.part_slice.empty() ? level.wave.subject : level.part_slice;
            level.simulation = wave::simulate_material(
                level.incoming_stock, final_subject, level.toolpath, radius,
                operation.simulation_stock_inflation);
          } catch (...) {
            simulation_errors[index] = std::current_exception();
          }
        }
      });
    }
    for (std::thread& thread : threads) thread.join();
  }
  plan.metrics.simulation_milliseconds +=
      elapsed_milliseconds(simulation_start);

  for (const std::size_t index : simulated_levels) {
    LevelPlan& level = plan.levels[index];
    if (simulation_errors[index]) {
      try {
        std::rethrow_exception(simulation_errors[index]);
      } catch (const std::exception& error) {
        const std::string detail =
            "Depth " + std::to_string(level.floor_z) +
            " mm, simulation or feeds: " + error.what();
        level_failures.push_back(detail);
        plan.warnings.push_back(detail);
      }
      level.nothing_to_cut = true;
      continue;
    }
    plan.metrics.simulation_intervals +=
        level.simulation.stats.cutting_steps;
    plan.metrics.empty_simulation_intervals +=
        level.simulation.stats.empty_cutting_steps;
    plan.metrics.gouge_bounds_rejections +=
        level.simulation.stats.gouge_bounds_rejections;
  }

  // ---- Feeds -------------------------------------------------------------
  // Feed schedules carry the previous pass position and height, so linking is
  // intentionally restored in level order after all independent simulations.
  PassLink link;
  link.from_z = rapid_plane;
  for (const std::size_t index : simulated_levels) {
    LevelPlan& level = plan.levels[index];
    if (level.nothing_to_cut) continue;
    try {
      link.stay_down_after = level.stays_down_after;
      const auto feeds_start = PlanningClock::now();
      level.feeds = plan_feeds(level.toolpath, level.simulation, machine,
                               full_engagement_feed, no_engagement_feed,
                               data->spindle_rpm, data->plunge_feed,
                               level.floor_z, rapid_plane, link);
      plan.metrics.feeds_milliseconds += elapsed_milliseconds(feeds_start);
      if (level.toolpath.moves.empty()) level.nothing_to_cut = true;
      if (!level.nothing_to_cut && !level.feeds.motion.empty()) {
        link.from = level.feeds.motion.back().to;
        link.from_z = level.feeds.ends_at_depth ? level.floor_z : rapid_plane;
      }
    } catch (const std::exception& error) {
      const std::string detail =
          "Depth " + std::to_string(level.floor_z) +
          " mm, simulation or feeds: " + error.what();
      level_failures.push_back(detail);
      plan.warnings.push_back(detail);
      level.nothing_to_cut = true;
    }
  }

  // ---- Roll up ----------------------------------------------------------
  plan.stats.levels = plan.levels.size();
  for (const LevelPlan& level : plan.levels) {
    if (level.nothing_to_cut) continue;
    ++plan.stats.cut_levels;
    if (plan.spindle_rpm <= 0.0) plan.spindle_rpm = level.feeds.stats.spindle_rpm;
    plan.stats.cutting_distance += level.feeds.stats.cutting_distance;
    plan.stats.rapid_distance += level.feeds.stats.rapid_distance;
    plan.stats.cutting_minutes += level.feeds.stats.cutting_minutes;
    plan.stats.rapid_minutes += level.feeds.stats.rapid_minutes;
    plan.stats.maximum_load =
        std::max(plan.stats.maximum_load, level.simulation.stats.maximum_load);
    plan.stats.removed_volume +=
        level.simulation.stats.removed_area * level.axial_depth;
    plan.stats.gouge_area += level.simulation.stats.gouge_area;
    plan.stats.unsafe_plunges += level.simulation.stats.unsafe_plunges;
    plan.stats.unresolved_entries += level.simulation.stats.unresolved_entries;
    plan.stats.lifts_over_subject += level.toolpath.stats.lifts_over_subject;
    plan.stats.unreachable_fronts += level.toolpath.stats.unreachable_fronts;
    plan.stats.excess_area += level.simulation.stats.excess_area;
  }

  if (plan.stats.cut_levels == 0) {
    if (!level_failures.empty()) {
      std::ostringstream error;
      error << "Every candidate cutting level failed:";
      for (const std::string& detail : level_failures)
        error << "\n  - " << detail;
      plan.error = error.str();
    } else {
      plan.warnings.push_back("Nothing was left to cut at any depth");
    }
  }
  if (plan.stats.unsafe_plunges > 0)
    plan.warnings.push_back(
        std::to_string(plan.stats.unsafe_plunges) +
        " plunge(s) come down where the cutter still overlaps stock");
  if (plan.stats.gouge_area > 0.0)
    plan.warnings.push_back("The cutter reaches into the finished part; increase "
                            "the radial stock to leave");
  // Every unresolved entry is posted as a straight descent into uncut stock,
  // because the ramp that ought to replace it is not built yet. That is worth
  // saying whatever the tool is, and worth saying twice for a tool that cannot
  // cut on its centre at all.
  if (plan.stats.unresolved_entries > 0)
    plan.warnings.push_back(
        std::to_string(plan.stats.unresolved_entries) +
        " entry move(s) descend straight into material that has not been "
        "cleared; the planner has no ramp to offer in their place");
  if (!tool.center_cutting && plan.stats.unresolved_entries > 0)
    plan.warnings.push_back("This tool is not centre cutting and the path needs "
                            "to enter solid material");
  if (plan.stats.lifts_over_subject > 0)
    plan.warnings.push_back(
        std::to_string(plan.stats.lifts_over_subject) +
        " rapid(s) pass over the part. One slice cannot say how high it stands, "
        "so check the clearance plane clears the whole model");
  if (plan.stats.unreachable_fronts > 0)
    plan.warnings.push_back(
        std::to_string(plan.stats.unreachable_fronts) +
        " front(s) were never reached, so the stock they bound is left standing");

}

OperationPlan plan_operation(const JobGeometry& geometry,
                             const Operation& operation,
                             const Tool& tool,
                             const Machine& machine,
                             const Material& material,
                             const ProgressCallback& progress,
                             double progress_from,
                             double progress_to,
                             double rapid_plane,
                             const std::vector<OperationPlan>* prior_operations,
                             const OperationPlan* prior_subdivisions) {
  const auto planning_start = PlanningClock::now();
  // Wavefronts are the cheaper half; give the toolpaths the larger share of the
  // reported progress so the bar does not stall in the second stage.
  const double seam = progress_from + (progress_to - progress_from) * 0.4;
  OperationPlan plan = plan_wavefronts(geometry, operation, tool, machine,
                                       material, progress, progress_from, seam,
                                       prior_operations, prior_subdivisions);
  plan_toolpaths(plan, operation, tool, machine, material, progress, seam,
                 progress_to, rapid_plane, prior_subdivisions != nullptr);
  plan.metrics.total_milliseconds = elapsed_milliseconds(planning_start);
  return plan;
}

wave::Paths64 stock_after_operations_at_z(
    const wave::Paths64& nominal_stock, double z,
    const std::vector<OperationPlan>& operations) {
  if (nominal_stock.empty()) return {};

  wave::Paths64 remaining =
      Clipper2Lib::Union(nominal_stock, Clipper2Lib::FillRule::NonZero);
  for (const OperationPlan& operation : operations) {
    remaining = stock_after_operation_at_z(remaining, z, operation);
    if (remaining.empty()) break;
  }
  return remaining;
}

std::size_t subdivision_preview_source(const OperationPlan& operation,
                                       std::size_t active_level,
                                       std::size_t target_level) {
  if (active_level >= operation.levels.size() ||
      target_level >= operation.levels.size())
    return target_level;
  const LevelPlan& active = operation.levels[active_level];
  const LevelPlan& target = operation.levels[target_level];
  if (target.subdivision <= active.subdivision ||
      active.subdivision >= operation.subdivisions.size())
    return target_level;
  const OperationSubdivision& source =
      operation.subdivisions[active.subdivision];
  if (source.level_count == 0) return target_level;
  return source.first_level +
         std::min(target.subdivision_level, source.level_count - 1);
}

OperationPlan plan_subdivided_operation(
    const JobGeometry& geometry, const Operation& operation,
    const Tool& tool, const Machine& machine, const Material& material,
    const ProgressCallback& progress, double progress_from,
    double progress_to, double rapid_plane,
    const std::vector<OperationPlan>* prior_operations) {
  OperationPlan result;
  result.operation_name = operation.name;
  result.tool_id = operation.tool_id;
  result.machine_id = operation.machine_id;
  result.material_id = operation.material_id;

  OperationSetup setup;
  if (!resolve_setup(operation, tool, machine, material, result.error, nullptr,
                     setup))
    return result;
  const int count = operation.mode == OperationMode::Roughing
                        ? operation.finishing_subdivisions
                        : 1;
  const double span = progress_to - progress_from;
  for (int subdivision = 0; subdivision < count; ++subdivision) {
    // The base floors are one full step below depth_from. Raising each
    // following group by stepdown/N produces, for N=4, starts at 1.0, 0.75,
    // 0.5 and 0.25 stepdowns while preserving the base layer count.
    const double shift =
        setup.stepdown * static_cast<double>(subdivision) / count;
    Operation pass = operation;
    pass.finishing_subdivisions = 1;
    pass.depth_from -= shift;
    pass.depth_to -= shift;
    if (count > 1)
      pass.name = operation.name + " subdivision " +
                  std::to_string(subdivision + 1) + "/" +
                  std::to_string(count);

    const double pass_from =
        progress_from + span * static_cast<double>(subdivision) / count;
    const double pass_to =
        progress_from + span * static_cast<double>(subdivision + 1) / count;
    OperationPlan planned = plan_operation(
        geometry, pass, tool, machine, material, progress, pass_from, pass_to,
        rapid_plane, prior_operations,
        result.levels.empty() ? nullptr : &result);
    if (!planned.ok()) {
      if (planned.error == "Cancelled") {
        result.error = "Cancelled";
        return result;
      }
      result.error = count == 1
                         ? planned.error
                         : "Subdivision " + std::to_string(subdivision + 1) +
                               "/" + std::to_string(count) + ": " +
                               planned.error;
      return result;
    }

    const std::size_t first = result.levels.size();
    result.subdivisions.push_back(
        {first, planned.levels.size(), shift});
    append_inter_layer(result.inter_layer, std::move(planned.inter_layer),
                       first);
    for (std::size_t local = 0; local < planned.levels.size(); ++local) {
      planned.levels[local].subdivision =
          static_cast<std::size_t>(subdivision);
      planned.levels[local].subdivision_level = local;
      result.levels.push_back(std::move(planned.levels[local]));
    }
    accumulate(result.stats, planned.stats);
    accumulate(result.metrics, planned.metrics);
    result.spindle_rpm = std::max(result.spindle_rpm, planned.spindle_rpm);
    for (const std::string& warning : planned.warnings)
      result.warnings.push_back(
          count == 1
              ? warning
              : "Subdivision " + std::to_string(subdivision + 1) + "/" +
                    std::to_string(count) + ": " + warning);
  }
  return result;
}

JobPlan plan_job(const JobGeometry& geometry,
                 const Job& job,
                 const ToolLibrary& tools,
                 const MachineLibrary& machines,
                 const MaterialLibrary& materials,
                 const ProgressCallback& progress) {
  JobPlan plan;
  if (job.operations.empty()) {
    plan.error = "The job has no operations";
    return plan;
  }

  plan.rapid_plane = job_rapid_plane(job, highest_model_z(geometry));
  const double share = 1.0 / static_cast<double>(job.operations.size());
  std::string previous_tool;
  for (std::size_t index = 0; index < job.operations.size(); ++index) {
    const Operation& operation = job.operations[index];
    OperationPlan result;
    result.operation_name = operation.name;
    result.tool_id = operation.tool_id;
    result.machine_id = operation.machine_id;
    result.material_id = operation.material_id;

    const Tool* tool = tools.find(operation.tool_id);
    const Machine* machine = machines.find(operation.machine_id);
    const Material* material = materials.find(operation.material_id);
    if (!tool) result.error = "Unknown tool: " + operation.tool_id;
    else if (!machine) result.error = "Unknown machine: " + operation.machine_id;
    else if (!material) result.error = "Unknown material: " + operation.material_id;

    if (result.ok()) {
      result = plan_subdivided_operation(
          geometry, operation, *tool, *machine, *material, progress,
          share * index, share * (index + 1), plan.rapid_plane,
          &plan.operations);
      if (result.error == "Cancelled") {
        plan.error = "Cancelled";
        return plan;
      }
      if (result.ok()) {
        if (!previous_tool.empty() && previous_tool != operation.tool_id)
          result.stats.tool_change_minutes +=
              machine->tool_change == ToolChange::Automatic ? 0.25 : 1.0;
        previous_tool = operation.tool_id;
      }
    }

    accumulate(plan.stats, result.stats);
    accumulate(plan.metrics, result.metrics);
    for (const std::string& warning : result.warnings)
      plan.warnings.push_back(operation.name + ": " + warning);
    plan.operations.push_back(std::move(result));
  }

  if (progress) progress(1.0, "Done");

  finish_job(plan);
  return plan;
}

}  // namespace cam
