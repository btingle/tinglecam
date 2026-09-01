#include "cam/job.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace cam {
namespace {

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

// Everything both planning stages need out of the operation, the tool and the
// machine. Resolving it once keeps the two stages from disagreeing about the
// engagement they were capped to.
struct OperationSetup {
  const CuttingData* data = nullptr;
  double full_engagement_feed = 0.0;
  double no_engagement_feed = 0.0;
  double radius = 0.0;
  double stepover = 0.0;
  double first_bite_allowance = 0.0;
  std::vector<double> floors;
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
  if (!(operation.entry_front_match_tolerance > 0.0) ||
      !std::isfinite(operation.entry_front_match_tolerance)) {
    error = "Entry-front Z match tolerance must be positive";
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

  Operation effective = operation;
  effective.stepdown = stepdown;
  setup.floors = level_floors(effective);
  if (setup.floors.empty()) {
    error = "The depth range is empty: the top is not above the floor";
    return false;
  }
  return true;
}

double absolute_area_mm2(const wave::Paths64& paths) {
  double signed_area = 0.0;
  for (const wave::Path64& path : paths) signed_area += Clipper2Lib::Area(path);
  return std::abs(signed_area) / (wave::kIntegerScale * wave::kIntegerScale);
}

// The muting the wave is given for one depth: every anchor the operation
// carries. Anchors that name no seed piece at this depth simply match nothing,
// which is what happens where an entry does not reach this far down.
wave::EntryMute layer_mute(const Operation& operation) {
  wave::EntryMute mute;
  for (const std::vector<wave::Point64>& entry : operation.muted_entries)
    mute.anchors.insert(mute.anchors.end(), entry.begin(), entry.end());
  mute.tolerance = static_cast<double>(wave::to_integer(
      std::max(operation.entry_front_match_tolerance, 0.001)));
  return mute;
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

bool JobGeometry::z_range(double& low, double& high) const {
  // The overlap, not the union. A depth where only one of the two models exists
  // has either no stock to remove or no part to protect, and offering it as a
  // default only produces passes that find nothing to cut.
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
    lowest = std::max<double>(lowest, bounds.lower.z);
    highest = std::min<double>(highest, bounds.upper.z);
  }
  if (!any || !(highest > lowest)) return false;
  low = lowest;
  high = highest;
  return true;
}

std::vector<double> level_floors(const Operation& operation) {
  std::vector<double> floors;
  const double bottom = operation.bottom_z + operation.axial_stock_to_leave;
  if (!(operation.top_z > bottom)) return floors;
  const double step = std::max(operation.stepdown, 1e-6);
  double z = operation.top_z;
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
                              double progress_to) {
  OperationPlan plan;
  plan.operation_name = operation.name;
  plan.tool_id = operation.tool_id;
  plan.stage = PlanStage::Wavefronts;

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
  const double first_bite_allowance = setup.first_bite_allowance;
  const std::vector<double>& floors = setup.floors;
  const wave::EntryMute mute = layer_mute(operation);

  plan.levels.reserve(floors.size());
  std::vector<std::string> level_failures;
  double previous_z = operation.top_z;
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

    LevelPlan level;
    level.floor_z = floor_z;
    level.axial_depth = previous_z - floor_z;
    previous_z = floor_z;

    try {
      // The slicer returns the downward envelope: everything at or above this
      // depth, projected. That is exactly the material that has to survive the
      // pass, so no separate rest model is needed for roughing.
      level.part_slice = geometry.part_outline(floor_z, operation.slice_grid);
      level.stock_slice = geometry.stock_outline(floor_z, operation.slice_grid);

      if (level.stock_slice.empty()) {
        level.nothing_to_cut = true;
        plan.levels.push_back(std::move(level));
        continue;
      }

      // The cutter follows a path whose centre must stay a radius clear of the
      // part, plus whatever is being left for a finishing pass.
      const wave::Paths64 keep_out =
          level.part_slice.empty()
              ? absent_subject(level.stock_slice)
              : offset_outward(level.part_slice,
                               radius + operation.radial_stock_to_leave);

      // Finishing does not propagate a wave, so it has no entry fronts to
      // offer and nothing to discover here. Its contours are derived from the
      // slices this stage already took, which happens with the rest of the
      // toolpath work.
      if (operation.mode == OperationMode::ContourFinishing) {
        plan.levels.push_back(std::move(level));
        continue;
      }

      // The run-in is seeded a tool radius plus the stock allowance in front
      // of the wall. The front one stepover behind that is the first to touch
      // the material, and it bites a stepover less the allowance -- so a blank
      // up to that much oversize still takes no more than a full stepover on
      // its first pass, and one exactly on size takes very nearly a full one.
      // Without this the run-in landed a front on the wall itself and the pass
      // before it grazed off whatever fraction of a stepover was left over.
      const double lead_in = operation.lead_in ? radius + first_bite_allowance : 0.0;
      level.wave = wave::build_wavefront(keep_out, level.stock_slice, stepover,
                                         stepover, lead_in, 2000, mute);
      level.lineage = wave::build_front_lineage(level.wave);
    } catch (const std::exception& error) {
      const std::string detail =
          "Depth " + std::to_string(floor_z) +
          " mm, wavefront construction: " + error.what();
      level_failures.push_back(detail);
      plan.warnings.push_back(detail);
      level.nothing_to_cut = true;
    }

    plan.levels.push_back(std::move(level));
  }

  for (const LevelPlan& level : plan.levels)
    plan.stats.muted_entries += level.wave.muted_entries;
  if (plan.stats.muted_entries > 0)
    plan.warnings.push_back(
        std::to_string(plan.stats.muted_entries) +
        " entry front(s) were declined, so the wave never started there");

  // Inter-layer associations need every slice at once, so wavefront discovery
  // is deliberately complete before any layer is sequenced into tool motion.
  // A known boundary-component mismatch is an internal topology failure and
  // aborts the operation instead of quietly emitting an arbitrary link.
  try {
    std::vector<const wave::FrontLineage*> lineages;
    lineages.reserve(plan.levels.size());
    for (const LevelPlan& level : plan.levels) lineages.push_back(&level.lineage);
    plan.inter_layer = wave::build_inter_layer_topology(
        lineages, operation.entry_front_match_tolerance);
  } catch (const std::exception& error) {
    plan.error = "Inter-layer topology: " + std::string(error.what());
    return plan;
  }

  return plan;
}

void plan_toolpaths(OperationPlan& plan,
                    const Operation& operation,
                    const Tool& tool,
                    const Machine& machine,
                    const Material& material,
                    const ProgressCallback& progress,
                    double progress_from,
                    double progress_to,
                    double rapid_clearance_height) {
  if (!plan.ok() || plan.has_toolpaths()) return;
  if (!(rapid_clearance_height > 0.0) ||
      !std::isfinite(rapid_clearance_height)) {
    plan.error = "Rapid clearance height must be positive";
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
  const double full_engagement_feed = setup.full_engagement_feed;
  const double no_engagement_feed = setup.no_engagement_feed;
  const double span = std::max(progress_to - progress_from, 0.0);
  std::vector<std::string> level_failures;
  std::size_t level_index = 0;

  // Where the previous pass lifted off, so the next one can rapid across to its
  // own start instead of appearing there. Passes with nothing to cut are
  // skipped by the post and by playback alike, so they do not move the cutter
  // and do not update this.
  std::optional<wave::Point64> cutter_left_at;
  for (LevelPlan& level : plan.levels) {
    const std::size_t index = level_index++;
    if (level.nothing_to_cut) continue;
    if (progress) {
      const double fraction =
          progress_from +
          span * (static_cast<double>(index) / plan.levels.size());
      if (!progress(fraction, operation.name + ": toolpath at depth " +
                                  std::to_string(level.floor_z) + " mm")) {
        plan.error = "Cancelled";
        return;
      }
    }
    try {
      if (operation.mode == OperationMode::ContourFinishing) {
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
        wave::TraversalSettings traversal;
        traversal.allow_stay_down = false;
        traversal.cutting_feed = no_engagement_feed;
        traversal.rapid_feed = rapid_feed(machine);
        traversal.floor_z = level.floor_z;
        traversal.rapid_clearance_height = rapid_clearance_height;
        level.toolpath = wave::plan_traversal(tasks, traversal).toolpath;
        if (level.toolpath.moves.empty()) {
          level.nothing_to_cut = true;
          continue;
        }
      }
      if (operation.mode == OperationMode::Roughing) {
        level.toolpath = wave::build_toolpath(
            level.wave, level.lineage, radius, level.floor_z,
            rapid_clearance_height, rapid_feed(machine), no_engagement_feed);
        if (operation.rough_boundary_cleanup && !level.part_slice.empty()) {
          const auto contours = derive_offset_contours(
              level.part_slice, radius + operation.radial_stock_to_leave,
              operation.clip_contours_to_stock ? &level.stock_slice : nullptr);
          auto tasks = contour_traversal_tasks(contours,
                                               operation.contour_direction);
          add_safe_stock_approaches(tasks, level.stock_slice,
                                    level.wave.subject, radius);
          wave::TraversalSettings traversal;
          traversal.allow_stay_down = true;
          if (!level.wave.reached_generations.empty())
            traversal.stay_down_region = level.wave.reached_generations.back();
          traversal.cutting_feed = no_engagement_feed;
          traversal.rapid_feed = rapid_feed(machine);
          traversal.floor_z = level.floor_z;
          traversal.rapid_clearance_height = rapid_clearance_height;
          if (!level.toolpath.moves.empty() &&
              !level.toolpath.moves.back().path.empty())
            traversal.initial_position = level.toolpath.moves.back().path.back();
          traversal.prefer_stay_down_when_legal = true;
          wave::Toolpath cleanup =
              wave::plan_traversal(tasks, traversal).toolpath;
          level.toolpath.moves.insert(level.toolpath.moves.end(),
                                      cleanup.moves.begin(), cleanup.moves.end());
        }
      }
      level.toolpath =
          wave::resample_toolpath(level.toolpath, operation.path_pitch);
      const wave::Paths64 final_subject =
          level.part_slice.empty() ? level.wave.subject : level.part_slice;
      level.simulation = wave::simulate_material(
          level.stock_slice, final_subject, level.toolpath, radius,
          operation.simulation_stock_inflation);
      level.feeds = plan_feeds(level.toolpath, level.simulation, machine,
                               full_engagement_feed, no_engagement_feed,
                               data->spindle_rpm, data->plunge_feed,
                               level.floor_z, rapid_clearance_height,
                               cutter_left_at);
      if (level.toolpath.moves.empty()) level.nothing_to_cut = true;
      if (!level.nothing_to_cut && !level.feeds.motion.empty())
        cutter_left_at = level.feeds.motion.back().to;
    } catch (const std::exception& error) {
      const std::string detail =
          "Depth " + std::to_string(level.floor_z) +
          " mm, toolpath or simulation: " + error.what();
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

  plan.stage = PlanStage::Toolpaths;
}

OperationPlan plan_operation(const JobGeometry& geometry,
                             const Operation& operation,
                             const Tool& tool,
                             const Machine& machine,
                             const Material& material,
                             const ProgressCallback& progress,
                             double progress_from,
                             double progress_to,
                             double rapid_clearance_height) {
  // Wavefronts are the cheaper half; give the toolpaths the larger share of the
  // reported progress so the bar does not stall in the second stage.
  const double seam = progress_from + (progress_to - progress_from) * 0.4;
  OperationPlan plan = plan_wavefronts(geometry, operation, tool, machine,
                                       material, progress, progress_from, seam);
  plan_toolpaths(plan, operation, tool, machine, material, progress, seam,
                 progress_to, rapid_clearance_height);
  return plan;
}

std::vector<EntryOption> entry_options(const OperationPlan& plan,
                                       const Operation& operation) {
  std::vector<EntryOption> options;
  const wave::InterLayerTopology& topology = plan.inter_layer;
  // A chain gathers one physical entry's appearances down the Z stack, so it is
  // the thing to offer. A front that matched no chain is its own option rather
  // than being dropped, or a one-layer entry could never be declined.
  std::vector<std::vector<bool>> claimed(topology.layers.size());
  for (std::size_t layer = 0; layer < topology.layers.size(); ++layer)
    claimed[layer].assign(topology.layers[layer].entry_fronts.size(), false);

  auto add_member = [&](EntryOption& option, const wave::LayerItemId& member) {
    if (member.layer >= plan.levels.size()) return;
    if (member.layer < claimed.size() && member.index < claimed[member.layer].size())
      claimed[member.layer][member.index] = true;
    const LevelPlan& level = plan.levels[member.layer];
    const wave::LayerTopology& layer = topology.layers[member.layer];
    if (member.index >= layer.entry_fronts.size()) return;
    const std::size_t front = layer.entry_fronts[member.index];
    if (front >= level.lineage.entry_fronts.size()) return;
    option.members.push_back(member);
    option.anchors.push_back(
        wave::representative_point(level.lineage.entry_fronts[front]));
    option.territory_area +=
        absolute_area_mm2(wave::entry_front_territory(level.lineage, front));
    const double top = level.floor_z + level.axial_depth;
    if (option.members.size() == 1) {
      option.top_z = top;
      option.bottom_z = level.floor_z;
    } else {
      option.top_z = std::max(option.top_z, top);
      option.bottom_z = std::min(option.bottom_z, level.floor_z);
    }
  };

  for (const wave::InterLayerChain& chain : topology.entry_front_chains) {
    EntryOption option;
    for (const wave::LayerItemId& member : chain.members) add_member(option, member);
    if (!option.members.empty()) options.push_back(std::move(option));
  }
  for (std::size_t layer = 0; layer < claimed.size(); ++layer)
    for (std::size_t index = 0; index < claimed[layer].size(); ++index) {
      if (claimed[layer][index]) continue;
      EntryOption option;
      add_member(option, {layer, index});
      if (!option.members.empty()) options.push_back(std::move(option));
    }

  // Muting is recorded as points rather than indices, so which options are off
  // is answered by asking where those points landed.
  const double tolerance = std::max(operation.entry_front_match_tolerance, 0.001);
  for (EntryOption& option : options)
    for (const wave::Point64& anchor : option.anchors)
      for (const std::vector<wave::Point64>& group : operation.muted_entries)
        for (const wave::Point64& muted : group)
          if (std::hypot(wave::to_world(anchor.x - muted.x),
                         wave::to_world(anchor.y - muted.y)) <= tolerance)
            option.muted = true;

  // Ordered by where each entry sits, not by how much it reaches: territory
  // changes every time another entry is declined, and a list that reshuffles
  // under the pointer is a list that gets mis-clicked.
  std::sort(options.begin(), options.end(),
            [](const EntryOption& a, const EntryOption& b) {
              if (a.anchors.empty() || b.anchors.empty())
                return a.anchors.size() > b.anchors.size();
              if (a.anchors.front().x != b.anchors.front().x)
                return a.anchors.front().x < b.anchors.front().x;
              return a.anchors.front().y < b.anchors.front().y;
            });
  return options;
}

void merge_entry_options(std::vector<EntryOption>& known,
                         std::vector<EntryOption> fresh,
                         double match_tolerance) {
  const double tolerance = std::max(match_tolerance, 0.001);
  const auto same = [&](const EntryOption& a, const EntryOption& b) {
    for (const wave::Point64& one : a.anchors)
      for (const wave::Point64& other : b.anchors)
        if (std::hypot(wave::to_world(one.x - other.x),
                       wave::to_world(one.y - other.y)) <= tolerance)
          return true;
    return false;
  };
  std::vector<bool> claimed(fresh.size(), false);
  for (EntryOption& entry : known) {
    bool found = false;
    for (std::size_t i = 0; i < fresh.size(); ++i) {
      if (claimed[i] || !same(entry, fresh[i])) continue;
      claimed[i] = true;
      found = true;
      // Keep the anchors that were offered rather than the ones just read: the
      // anchors are what muting is recorded as, and they must not drift each
      // time a neighbour is declined and the front grows.
      entry.members = fresh[i].members;
      entry.territory_area = fresh[i].territory_area;
      entry.top_z = fresh[i].top_z;
      entry.bottom_z = fresh[i].bottom_z;
      entry.muted = false;
      break;
    }
    // Absent from a plan it was seeded into means it was declined, which is the
    // only reason a front that exists can fail to appear.
    if (!found) entry.muted = true;
  }
  for (std::size_t i = 0; i < fresh.size(); ++i)
    if (!claimed[i]) known.push_back(std::move(fresh[i]));
}

JobPlan plan_job(const JobGeometry& geometry,
                 const Job& job,
                 const ToolLibrary& tools,
                 const MachineLibrary& machines,
                 const MaterialLibrary& materials,
                 const ProgressCallback& progress,
                 PlanStage upto) {
  JobPlan plan;
  if (job.operations.empty()) {
    plan.error = "The job has no operations";
    return plan;
  }

  const double share = 1.0 / static_cast<double>(job.operations.size());
  std::string previous_tool;
  for (std::size_t index = 0; index < job.operations.size(); ++index) {
    const Operation& operation = job.operations[index];
    OperationPlan result;
    result.operation_name = operation.name;
    result.tool_id = operation.tool_id;

    const Tool* tool = tools.find(operation.tool_id);
    const Machine* machine = machines.find(operation.machine_id);
    const Material* material = materials.find(operation.material_id);
    if (!tool) result.error = "Unknown tool: " + operation.tool_id;
    else if (!machine) result.error = "Unknown machine: " + operation.machine_id;
    else if (!material) result.error = "Unknown material: " + operation.material_id;

    if (result.ok()) {
      result = upto == PlanStage::Wavefronts
                   ? plan_wavefronts(geometry, operation, *tool, *machine,
                                     *material, progress, share * index,
                                     share * (index + 1))
                   : plan_operation(geometry, operation, *tool, *machine,
                                    *material, progress, share * index,
                                    share * (index + 1),
                                    job.rapid_clearance_height);
      if (result.error == "Cancelled") {
        plan.error = "Cancelled";
        return plan;
      }
      if (!previous_tool.empty() && previous_tool != operation.tool_id)
        result.stats.tool_change_minutes +=
            machine->tool_change == ToolChange::Automatic ? 0.25 : 1.0;
      previous_tool = operation.tool_id;
    }

    accumulate(plan.stats, result.stats);
    for (const std::string& warning : result.warnings)
      plan.warnings.push_back(operation.name + ": " + warning);
    plan.operations.push_back(std::move(result));
  }

  if (progress) progress(1.0, "Done");

  finish_job(plan);
  return plan;
}

void plan_job_toolpaths(JobPlan& plan,
                        const Job& job,
                        const ToolLibrary& tools,
                        const MachineLibrary& machines,
                        const MaterialLibrary& materials,
                        const ProgressCallback& progress) {
  const double share = plan.operations.empty()
                           ? 1.0
                           : 1.0 / static_cast<double>(plan.operations.size());
  plan.stats = {};
  plan.warnings.clear();
  plan.error.clear();
  std::string previous_tool;
  for (std::size_t index = 0; index < plan.operations.size(); ++index) {
    OperationPlan& result = plan.operations[index];
    const Operation* operation = nullptr;
    for (const Operation& candidate : job.operations)
      if (candidate.name == result.operation_name) operation = &candidate;
    const Tool* tool = operation ? tools.find(operation->tool_id) : nullptr;
    const Machine* machine = operation ? machines.find(operation->machine_id) : nullptr;
    const Material* material =
        operation ? materials.find(operation->material_id) : nullptr;
    if (result.ok() && operation && tool && machine && material) {
      plan_toolpaths(result, *operation, *tool, *machine, *material, progress,
                     share * index, share * (index + 1),
                     job.rapid_clearance_height);
      if (result.error == "Cancelled") {
        plan.error = "Cancelled";
        return;
      }
      if (!previous_tool.empty() && previous_tool != operation->tool_id)
        result.stats.tool_change_minutes +=
            machine->tool_change == ToolChange::Automatic ? 0.25 : 1.0;
      previous_tool = operation->tool_id;
    }
    accumulate(plan.stats, result.stats);
    for (const std::string& warning : result.warnings)
      plan.warnings.push_back(result.operation_name + ": " + warning);
  }
  if (progress) progress(1.0, "Done");
  finish_job(plan);
}


}  // namespace cam
