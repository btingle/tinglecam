#include "job_internal.h"
#include "cam/stock.h"
namespace cam {
using namespace detail;
OperationPlan plan_wavefronts(const JobGeometry& geometry,
                              const Operation& operation,
                              const Tool& tool,
                              const Machine& machine,
                              const Material& material,
                              const ProgressCallback& progress,
                              double progress_from,
                              double progress_to,
                              const PriorOperations& prior_operations,
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
    level.radial_width = stepover;
    previous_z = floor_z;

    try {
      // The slicer returns the downward envelope: everything at or above this
      // depth, projected. That is exactly the material that has to survive the
      // pass, so no separate rest model is needed for roughing. Taken the axial
      // allowance below the floor, it also carries what the part has just under
      // the cutter, so the pass stops above it. The blank is sliced at the
      // floor: what is left is part, not stock.
      const auto slice_start = PlanningClock::now();
      const double protected_z = floor_z - operation.axial_stock_to_leave;
      // A horizontal face coincident with the cutter floor is support, not
      // an obstacle above it. Query the next representable slicer height so
      // an explicit floor can actually clear down onto that face. The exact
      // float cache key keeps this distinct from an inclusive slice there.
      const double slice_z = operation.mode == OperationMode::Roughing
          ? std::nextafter(static_cast<float>(protected_z),
                           std::numeric_limits<float>::infinity())
          : protected_z;
      level.part_slice = geometry.part_outline(slice_z, operation.slice_grid);
      level.stock_slice = geometry.stock_outline(floor_z, operation.slice_grid);
      level.incoming_stock =
          prior_operations.empty()
              ? level.stock_slice
              : stock_after_operations_at_z(level.stock_slice, floor_z,
                                            prior_operations);
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
  double low = 0.0;
  double high = 0.0;
  if (geometry.work_range(low, high)) return high;
  return -std::numeric_limits<double>::infinity();
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

// Where an operation leaves the cutter standing when it finishes: the end of
// the last pass in it that cut anything. Absent when none did.
std::optional<wave::Point64> resting_position(const OperationPlan& plan) {
  for (auto level = plan.levels.rbegin(); level != plan.levels.rend(); ++level) {
    if (level->nothing_to_cut || level->feeds.motion.empty()) continue;
    return level->feeds.motion.back().to;
  }
  return std::nullopt;
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
                    const PriorOperations& prior_operations,
                    const OperationPlan* prior_subdivisions) {
  const bool finishing_subdivision = prior_subdivisions != nullptr;
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
        // Earlier operations remove stock without belonging to this wave. Use
        // only those operations here; `incoming_stock` may also include cuts
        // from an earlier subdivision of this same operation, whose geometry
        // must remain subject to the existing subdivision routing rules.
        const wave::Paths64 prior_stock = stock_after_operations_at_z(
            level.stock_slice, level.floor_z, prior_operations);
        const wave::Paths64 prior_removed = Clipper2Lib::Difference(
            level.stock_slice, prior_stock,
            Clipper2Lib::FillRule::NonZero);
        // The removed-stock outline is a swept boundary, not a guarantee that
        // a new cutter centre can ride exactly on its edge. Keep the same small
        // slack used by at-depth links inside the inherited opening; this
        // avoids turning rounding at a prior pocket wall into a gouge.
        const wave::Paths64 initial_cleared = Clipper2Lib::InflatePaths(
            prior_removed, -link_slack(setup.stepover),
            Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
        level.toolpath = wave::build_toolpath(
            level.wave_stages, radius, level.floor_z, rapid_plane,
            rapid_feed(machine), no_engagement_feed, rapid_feed_z(machine),
            initial_cleared);
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
  // A subdivision group is planned on its own but does not run on its own: it
  // picks up where the group before it put the cutter down. Starting its first
  // pass with nowhere to come from leaves that pass with no opening rapid at
  // all, so the cutter arrives at it out of nothing -- which is what the
  // preview draws, and what the travel totals leave out. The group before
  // always finishes at the rapid plane, so only the position has to be
  // carried; the height is the one already set above.
  if (prior_subdivisions != nullptr)
    link.from = resting_position(*prior_subdivisions);
  for (const std::size_t index : simulated_levels) {
    LevelPlan& level = plan.levels[index];
    if (level.nothing_to_cut) continue;
    try {
      link.stay_down_after = level.stays_down_after;
      const auto feeds_start = PlanningClock::now();
      level.feeds = plan_feeds(level.toolpath, level.simulation, machine,
                               full_engagement_feed, no_engagement_feed,
                               operation.spindle_rpm, operation.plunge_feed,
                               level.floor_z, rapid_plane, link,
                               operation.mode != OperationMode::Roughing);
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
    plan.stats.skipped_entries += level.toolpath.stats.skipped_entry_count;
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
  if (plan.stats.skipped_entries > 0)
    plan.warnings.push_back(
        std::to_string(plan.stats.skipped_entries) +
        " roughing entry point(s) skipped; no constant-engagement route or "
        "cleared landing was available");
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
                             const PriorOperations& prior_operations,
                             const OperationPlan* prior_subdivisions) {
  const auto planning_start = PlanningClock::now();
  // Wavefronts are the cheaper half; give the toolpaths the larger share of the
  // reported progress so the bar does not stall in the second stage.
  const double seam = progress_from + (progress_to - progress_from) * 0.4;
  OperationPlan plan = plan_wavefronts(geometry, operation, tool, machine,
                                       material, progress, progress_from, seam,
                                       prior_operations, prior_subdivisions);
  plan_toolpaths(plan, operation, tool, machine, material, progress, seam,
                 progress_to, rapid_plane, prior_operations, prior_subdivisions);
  plan.metrics.total_milliseconds = elapsed_milliseconds(planning_start);
  return plan;
}

wave::Paths64 stock_after_operations_at_z(
    const wave::Paths64& nominal_stock, double z,
    const PriorOperations& operations) {
  if (nominal_stock.empty()) return {};

  wave::Paths64 remaining =
      Clipper2Lib::Union(nominal_stock, Clipper2Lib::FillRule::NonZero);
  for (const OperationPlan* operation : operations) {
    if (operation == nullptr) continue;
    remaining = stock_after_operation_at_z(remaining, z, *operation);
    if (remaining.empty()) break;
  }
  return remaining;
}

wave::Paths64 stock_after_planned_operation_at_z(
    const wave::Paths64& nominal_stock, double z, const OperationPlan& operation) {
  return stock_after_operation_at_z(nominal_stock, z, operation);
}

OperationPlan plan_subdivided_operation(
    const JobGeometry& geometry, const Operation& operation,
    const Tool& tool, const Machine& machine, const Material& material,
    const ProgressCallback& progress, double progress_from,
    double progress_to, double rapid_plane,
    const PriorOperations& prior_operations) {
  OperationPlan result;
  result.operation_name = operation.name;
  result.tool_id = operation.tool_id;
  result.machine_id = operation.machine_id;
  result.material_id = operation.material_id;

  OperationSetup setup;
  std::vector<std::string> setup_warnings;
  if (!resolve_setup(operation, tool, machine, material, result.error, &setup_warnings,
                     setup))
    return result;
  result.warnings = setup_warnings;
  const int count = operation.mode == OperationMode::Roughing
                        ? operation.finishing_subdivisions
                        : 1;
  std::vector<double> floors;
  const double work_top = highest_model_z(geometry);
  if (!resolve_floors(operation, setup, work_top, result.error, floors))
    return result;
  const std::size_t groups = 1 + floors.size() * (count - 1);
  const std::size_t total_levels = floors.size() * count;
  const double span = progress_to - progress_from;
  for (std::size_t group = 0; group < groups; ++group) {
    Operation pass = operation;
    pass.finishing_subdivisions = 1;
    std::size_t coarse = 0;
    int finishing = 0;
    double shift = 0.0;
    if (group > 0) {
      coarse = floors.size() - 1 - (group - 1) / (count - 1);
      finishing = 1 + (group - 1) % (count - 1);
      const double upper = coarse == 0
                               ? z_at_depth(work_top, operation.depth_from)
                               : floors[coarse - 1];
      const double coarse_axial = upper - floors[coarse];
      shift = coarse_axial * finishing / count;
      const double cleanup_floor = floors[coarse] + shift;
      // The upward travel distance is not axial engagement. Material can
      // stand anywhere from this floor to the top of the coarse terrace.
      const double cleanup_axial = upper - cleanup_floor;
      pass.depth_from = work_top - upper;
      pass.depth_to = work_top - cleanup_floor;
      pass.depth_breaks.clear();
      if (operation.adaptive_cleanup_stepover) {
        const double ceiling = std::max(operation.stepover_ratio,
                                        operation.cleanup_stepover_limit);
        pass.stepover_ratio = std::min(ceiling,
            operation.stepover_ratio * coarse_axial / cleanup_axial);
      }
      pass.name += " terrace " + std::to_string(coarse + 1) +
                   " cleanup " + std::to_string(finishing) + "/" +
                   std::to_string(count - 1);
    }

    const double pass_from = progress_from +
        span * result.levels.size() / total_levels;
    const std::size_t group_levels = group == 0 ? floors.size() : 1;
    const double pass_to = progress_from +
        span * (result.levels.size() + group_levels) / total_levels;
    OperationPlan planned = plan_operation(
        geometry, pass, tool, machine, material, progress, pass_from, pass_to,
        rapid_plane, prior_operations,
        result.levels.empty() ? nullptr : &result);
    if (!planned.ok()) {
      result.error = planned.error == "Cancelled" ? planned.error
          : pass.name + ": " + planned.error;
      return result;
    }

    const std::size_t first = result.levels.size();
    result.subdivisions.push_back(
        {first, planned.levels.size(), shift});
    append_inter_layer(result.inter_layer, std::move(planned.inter_layer),
                       first);
    for (std::size_t local = 0; local < planned.levels.size(); ++local) {
      planned.levels[local].subdivision = group;
      planned.levels[local].coarse_level = group == 0 ? local : coarse;
      planned.levels[local].finishing_index = finishing;
      planned.levels[local].subdivision_level = local;
      result.levels.push_back(std::move(planned.levels[local]));
    }
    accumulate(result.stats, planned.stats);
    accumulate(result.metrics, planned.metrics);
    result.spindle_rpm = std::max(result.spindle_rpm, planned.spindle_rpm);
    for (const std::string& warning : planned.warnings) {
      // Engagement limits apply to the whole operation. Keep layer-specific
      // diagnostics, but don't repeat identical setup notes for every group.
      if (std::find(setup_warnings.begin(), setup_warnings.end(), warning) !=
          setup_warnings.end())
        continue;
      result.warnings.push_back(
          group == 0 ? warning : pass.name + ": " + warning);
    }
  }
  if (operation.mode == OperationMode::Roughing) {
    // Compose physical slabs independently of execution order. In particular,
    // future upward cuts must not appear in the stock during coarse descent.
    std::vector<double> boundaries;
    for (const auto& level : result.levels) {
      boundaries.push_back(level.floor_z);
      boundaries.push_back(level.floor_z + level.axial_depth);
    }
    if (progress && !progress(progress_to, operation.name + ": stock preview")) {
      result.error = "Cancelled";
      return result;
    }
    result.preview_context = active_stock(geometry, prior_operations,
                                           operation.slice_grid, boundaries);
    result.composed_stock = true;
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
  if (!(job.rapid_clearance_height > 0) || !std::isfinite(job.rapid_clearance_height)) {
    plan.error = "Rapid clearance must be finite and greater than zero";
    return plan;
  }

  PartialPlan whole =
      plan_operation_range(geometry, job, tools, machines, materials, progress,
                           0, job.operations.size() - 1, {});
  if (!whole.error.empty()) {
    plan.error = std::move(whole.error);
    return plan;
  }
  plan.rapid_plane = whole.rapid_plane;
  plan.operations = std::move(whole.operations);

  if (progress) progress(1.0, "Done");

  summarise_job(plan);
  return plan;
}
}  // namespace cam
