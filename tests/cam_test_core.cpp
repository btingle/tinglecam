#include "cam_test_support.h"
#include "cam/stock.h"
#include "cam/job_internal.h"
namespace cam_test {

int failures = 0;

void require(bool condition, const std::string& message) {
  if (condition) return;
  std::cerr << "  FAILED: " << message << '\n';
  ++failures;
}

void require_near(double actual, double expected, double tolerance,
                  const std::string& message) {
  if (std::abs(actual - expected) <= tolerance) return;
  std::cerr << "  FAILED: " << message << " (got " << actual
            << ", wanted " << expected << ")\n";
  ++failures;
}

void check_parallel_progress() {
  const auto coordinator = std::this_thread::get_id();
  for (const std::size_t workers : {1, 2}) {
    std::vector<int> visits(4, 0);
    double last = 0.2;
    bool saw_active = false, saw_partial = false;
    cam::PlanningProgress final;
    const bool ok = cam::detail::parallel_for_levels(
        {0, 1, 2, 3}, [&](std::size_t index, const cam::detail::LevelProgress& report) {
          ++visits[index];
          for (int step = 0; step < 100; ++step) {
            report(step / 100.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }, [&](cam::PlanningProgress p, const std::string&) {
          require(std::this_thread::get_id() == coordinator,
                  "parallel progress runs on coordinator only");
          require(p.fraction >= last && p.fraction <= p.active_until + 1e-12 &&
                  p.active_until <= 0.8 + 1e-12,
                  "completed and active progress are monotone and bounded");
          require(p.active_workers <= workers &&
                  p.active_workers + p.completed_levels <= p.total_levels,
                  "worker counts match assigned layer work");
          saw_active |= p.active_workers > 0;
          saw_partial |= p.fraction > 0.2 + 0.6 * p.completed_levels / 4 + 1e-6;
          last = p.fraction;
          final = p;
          return true;
        }, 0.2, 0.8, "test", workers);
    require(ok && saw_active && saw_partial, "serial and parallel workers report fine progress");
    require(final.completed_levels == 4 && final.active_workers == 0,
            "final progress completes every layer");
    require_near(final.fraction, 0.8, 1e-12, "parallel stage reaches its boundary");
    for (int count : visits) require(count == 1, "each layer executes exactly once");

    std::atomic<int> started{0}, running{0};
    const bool cancelled = cam::detail::parallel_for_levels(
        {0, 1, 2, 3}, [&](std::size_t, const cam::detail::LevelProgress& report) {
          ++started;
          ++running;
          while (report(0.25))
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          --running;
        }, [](cam::PlanningProgress p, const std::string&) {
          return p.active_workers == 0;
        }, 0.0, 1.0, "cancel", workers);
    require(!cancelled && running == 0 && started <= static_cast<int>(workers),
            "cancellation joins active workers and skips queued layers");
  }
  bool rethrown = false;
  try {
    cam::detail::parallel_for_levels({0, 1},
        [](std::size_t, const cam::detail::LevelProgress&) {
          throw std::runtime_error("worker failure");
        }, {}, 0, 1, "throw", 2);
  } catch (const std::runtime_error&) { rethrown = true; }
  require(rethrown, "worker exceptions are rethrown after joining");
}

cam::Machine test_machine() {
  cam::Machine machine;
  machine.id = "test";
  machine.name = "Test machine";
  machine.controller = cam::Controller::LinuxCnc;
  machine.rapid_feed = 6000.0;
  machine.min_rpm = 1000.0;
  machine.max_rpm = 24000.0;
  machine.rpm_resolution = 100.0;
  machine.feed_ranges.push_back({"plastic", 500.0, 2000.0});
  return machine;
}

cam::Tool test_tool() {
  cam::Tool tool;
  tool.id = "test-tool";
  tool.name = "6 mm test tool";
  tool.diameter = 6.0;
  tool.flute_length = 16.0;
  return tool;
}

wave::Point64 point(double x, double y) {
  return wave::Point64(wave::to_integer(x), wave::to_integer(y));
}

wave::Path64 box(double x0, double y0, double x1, double y1) {
  return {point(x0, y0), point(x1, y0), point(x1, y1), point(x0, y1)};
}

// Whether a point stands in a region that may have holes. A one-unit square is
// intersected with the region rather than each ring being tested in turn, so
// holes and nested outers are handled by the same fill rule the region was
// built with.
bool inside(const wave::Point64& at, const wave::Paths64& region) {
  const wave::Path64 speck{{at.x, at.y},
                           {at.x + 1, at.y},
                           {at.x + 1, at.y + 1},
                           {at.x, at.y + 1}};
  return !Clipper2Lib::Intersect(wave::Paths64{speck}, region,
                                 Clipper2Lib::FillRule::NonZero)
              .empty();
}

double area(const wave::Paths64& paths) {
  double signed_area = 0.0;
  for (const auto& path : paths) signed_area += Clipper2Lib::Area(path);
  return std::abs(signed_area) /
         (wave::kIntegerScale * wave::kIntegerScale);
}

SyntheticCut straight_cut(const std::vector<double>& loads) {
  SyntheticCut cut;
  wave::ToolpathMove move;
  move.kind = wave::MoveKind::Cut;
  move.path = {point(0.0), point(static_cast<double>(loads.size()))};
  move.length = static_cast<double>(loads.size());
  cut.toolpath.moves.push_back(move);
  cut.simulation.moves.resize(1);
  for (std::size_t index = 0; index < loads.size(); ++index) {
    wave::MaterialStep step;
    step.from = point(static_cast<double>(index));
    step.to = point(static_cast<double>(index + 1));
    step.move_distance_begin = static_cast<double>(index);
    step.move_distance_end = static_cast<double>(index + 1);
    step.load = loads[index];
    step.removed_area = loads[index] * 6.0;
    cut.simulation.moves[0].steps.push_back(step);
  }
  return cut;
}

void check_engagement_feeds() {
  std::cout << "engagement feed mapping\n";
  for (const auto [load, expected] :
       std::vector<std::pair<double, double>>{{1.0, 500.0},
                                              {0.5, 1250.0},
                                              {0.0, 2000.0},
                                              {1.5, 500.0}}) {
    const SyntheticCut cut = straight_cut({load});
    const auto schedule = cam::plan_feeds(
        cut.toolpath, cut.simulation, test_machine(), 500.0, 2000.0,
        12000.0, 120.0);
    require(schedule.segments.size() == 1, "one sample produces one feed segment");
    if (!schedule.segments.empty())
      require_near(schedule.segments.front().feed, expected, 1e-9,
                   "feed is the clamped linear engagement interpolation");
  }
}

void check_feed_compatible_merge() {
  std::cout << "post-feed line merging\n";
  const SyntheticCut equal = straight_cut({0.5, 0.5, 0.5, 0.5});
  const auto merged = cam::plan_feeds(
      equal.toolpath, equal.simulation, test_machine(), 500.0, 2000.0,
      12000.0, 120.0);
  require(merged.segments.size() == 1,
          "collinear segments at one feed merge after assignment");
  if (!merged.segments.empty())
    require_near(merged.segments.front().length, 4.0, 1e-9,
                 "merging preserves travelled distance");

  const SyntheticCut different = straight_cut({0.5, 0.5, 0.2, 0.2});
  const auto split = cam::plan_feeds(
      different.toolpath, different.simulation, test_machine(), 500.0, 2000.0,
      12000.0, 120.0);
  require(split.segments.size() == 2,
          "a feed change prevents a collinear merge");
}

void check_path_resampling() {
  std::cout << "path pitch\n";
  wave::Toolpath raw;
  wave::ToolpathMove first;
  first.kind = wave::MoveKind::Cut;
  first.path = {point(0.0), point(0.03), point(0.07), point(0.35)};
  first.length = 0.35;
  raw.moves.push_back(first);
  wave::ToolpathMove second;
  second.kind = wave::MoveKind::Step;
  second.path = {point(0.35), point(1.0)};
  second.length = 0.65;
  raw.moves.push_back(second);

  const wave::Toolpath sampled = wave::resample_toolpath(raw, 0.2);
  require(!sampled.moves.empty(), "resampling retains a cutting path");
  require(sampled.moves.front().path.front() == point(0.0),
          "resampling retains the path start");
  require(sampled.moves.back().path.back() ==
              point(1.0),
          "resampling retains the path endpoint");
  bool retained_micro_vertex = false;
  for (const auto& move : sampled.moves)
    for (const auto& point : move.path)
      if (point.x == wave::to_integer(0.03) ||
          point.x == wave::to_integer(0.07) ||
          point.x == wave::to_integer(0.35))
        retained_micro_vertex = true;
  require(!retained_micro_vertex,
          "original micro-vertices are not forced into the normalized path");
  require_near(wave::toolpath_length(sampled), 1.0, 1e-6,
               "uniform sampling preserves a straight path's length");
  for (const auto& move : sampled.moves)
    for (std::size_t i = 1; i < move.path.size(); ++i)
      require_near(
          std::hypot(wave::to_world(move.path[i].x - move.path[i - 1].x),
                     wave::to_world(move.path[i].y - move.path[i - 1].y)),
          0.2, 1e-6, "pitch carries across the original move boundary");
}

void check_level_floors() {
  std::cout << "depth levels\n";
  cam::Operation operation;
  operation.depth_from = 0.0;
  operation.depth_to = 5.0;
  operation.stepdown = 2.0;
  const auto floors = cam::level_floors(operation, operation.stepdown, 0.0);
  require(floors.size() == 3, "five millimetres in two millimetre steps is three passes");
  if (floors.size() == 3) {
    require_near(floors[0], -2.0, 1e-9, "first pass takes a full step");
    require_near(floors[2], -5.0, 1e-9, "last pass lands on the floor");
  }
  // The axial allowance leaves the depth range alone; check_axial_allowance
  // measures what it does instead.
  operation.axial_stock_to_leave = 0.5;
  require_near(cam::level_floors(operation, operation.stepdown, 0.0).back(), -5.0,
               1e-9, "an axial allowance does not move the depth range");
  // Depths are below the top of the work, so the same operation follows it.
  require_near(cam::level_floors(operation, operation.stepdown, 12.0).back(),
               7.0, 1e-9, "and the range hangs off wherever the work tops out");
  operation.depth_from = 1.0;
  operation.depth_to = 8.0;
  operation.stepdown = 3.0;
  operation.depth_breaks = {6.5, 3.5, 3.5};
  require(cam::level_floors(operation, 3.0, 10.0) ==
              std::vector<double>({6.5, 3.5, 2.0}),
          "unsorted duplicate breaks restart the grid without duplicate floors");
  operation.depth_breaks = {8.5};
  require(cam::level_floors(operation, 3.0, 10.0).empty(),
          "out-of-range breaks cannot silently alter the depth range");
}

// A prism along Y with an L-shaped XZ section: the low half of the footprint
// tops out at Z 6, the high half at Z 10. The face at Z 6 is the floor an
// axial allowance has to protect.
void write_stepped_part(const std::filesystem::path& file) {
  std::ofstream out(file);
  const double x[6] = {0, 40, 40, 20, 20, 0};
  const double z[6] = {0, 0, 6, 6, 10, 10};
  for (int y = 0; y < 2; ++y)
    for (int i = 0; i < 6; ++i)
      out << "v " << x[i] << ' ' << (y ? 40 : 0) << ' ' << z[i] << '\n';
  for (int i = 1; i < 5; ++i) out << "f 1 " << (i + 1) << ' ' << (i + 2) << '\n';
  for (int i = 1; i < 5; ++i) out << "f 7 " << (i + 8) << ' ' << (i + 7) << '\n';
  for (int i = 0; i < 6; ++i) {
    const int a = i + 1;
    const int b = (i + 1) % 6 + 1;
    out << "f " << a << ' ' << b << ' ' << (b + 6) << '\n';
    out << "f " << a << ' ' << (b + 6) << ' ' << (a + 6) << '\n';
  }
}

void check_adaptive_roughing() {
  std::cout << "coarse descent and upward roughing cleanup\n";
  const auto file = std::filesystem::temp_directory_path() /
                    "tinglecam-adaptive-roughing.obj";
  write_stepped_part(file);
  cam::JobGeometry geometry;
  geometry.load_part(file);
  geometry.set_box_stock(true, cam::BoxStockMargins{4, 4, 4, 4, 1, 1});
  std::filesystem::remove(file);
  const auto tool = test_tool();
  const auto machine = test_machine();
  cam::ToolLibrary tools;
  tools.upsert(tool);
  cam::MachineLibrary machines;
  machines.upsert(machine);
  cam::MaterialLibrary materials;
  materials.materials.push_back({"plastic", "Plastic"});
  cam::Operation op;
  op.tool_id = tool.id;
  op.machine_id = machine.id;
  op.material_id = "plastic";
  op.depth_to = 6;
  op.stepdown = 3;
  op.finishing_subdivisions = 3;
  op.stepover_ratio = 0.2;
  op.cleanup_stepover_limit = 0.5;
  op.path_pitch = 0.5;
  cam::Job job;
  const auto plan = [&](const cam::Operation& operation,
                        const cam::ProgressCallback& progress = {}) {
    job.operations = {operation};
    return cam::plan_job(geometry, job, tools, machines, materials, progress);
  };
  const double top = cam::highest_model_z(geometry);
  const auto check_layers = [&](const cam::JobPlan& result,
                                const std::vector<double>& depths,
                                const std::vector<double>& axial,
                                const std::vector<double>& widths) {
    require(result.ok(), "adaptive roughing plans: " + result.error);
    if (!result.ok() || result.operations.empty()) return;
    const auto& rough = result.operations.front();
    require(rough.levels.size() == depths.size(), "all scheduled layers survive planning");
    if (rough.levels.size() != depths.size()) return;
    cam::OperationPlan prefix;
    for (std::size_t i = 0; i < depths.size(); ++i) {
      const auto& level = rough.levels[i];
      require_near(top - level.floor_z, depths[i], 1e-8, "layer execution depth");
      require_near(level.axial_depth, axial[i], 1e-8, "remaining terrace axial bound");
      require_near(level.radial_width, widths[i] * tool.diameter, 1e-8,
                   "cleanup radial width respects axial ratio and ceiling");
      if (level.finishing_index != 0) {
        const auto inherited = cam::stock_after_planned_operation_at_z(
            level.stock_slice, level.floor_z, prefix);
        require(wave::paths_equal_canonical(inherited, level.incoming_stock),
                "each upward cut receives every preceding cut's stock");
        const auto& coarse = rough.levels[level.coarse_level];
        require_near(level.floor_z + level.axial_depth,
                     coarse.floor_z + coarse.axial_depth, 1e-8,
                     "cleanup never extends above its coarse terrace");
      }
      prefix.levels.push_back(level);
    }
    require(rough.stats.unsafe_plunges == 0 && rough.stats.unresolved_entries == 0,
            "upward planning preserves safe resolved entries");
    require(cam::post_job(result, job, tools, machines).ok(),
            "the combined downward/upward operation posts");

    const auto volume = [](const std::vector<cam::StockSlab>& stock) {
      double sum = 0;
      for (const auto& slab : stock) sum += area(*slab.region) * (slab.top - slab.bottom);
      return sum;
    };
    double before = volume(cam::preview_before_level(rough, 0));
    require_near(before, 48.0 * 48.0 * 12.0, 1e-3,
                 "preview begins with the entire blank including Z margins");
    for (std::size_t i = 0; i < rough.levels.size(); ++i) {
      const double after = volume(cam::preview_before_level(rough, i + 1));
      const auto& level = rough.levels[i];
      require_near(before - after,
                   area(level.simulation.removed_material) * level.axial_depth,
                   0.05, "preview removes only the next executed slab sweep");
      before = after;
    }
    const auto final_stock = cam::preview_before_level(rough, rough.levels.size());
    for (const auto& slab : final_stock) {
      const auto expected = cam::stock_after_planned_operation_at_z(
          geometry.stock_outline(slab.bottom, op.slice_grid),
          (slab.bottom + slab.top) * 0.5, rough);
      require(wave::paths_equal_canonical(*slab.region, expected),
              "final preview and downstream stock handoff agree");
    }
  };
  const char* worker_env = std::getenv("TINGLECAM_PLANNING_WORKERS");
  const std::optional<std::string> saved_workers = worker_env
      ? std::optional<std::string>(worker_env) : std::nullopt;
  setenv("TINGLECAM_PLANNING_WORKERS", "4", 1);
  double last_progress = 0;
  bool cleanup_batch_seen = false;
  const auto first = plan(op, [&](cam::PlanningProgress progress, const std::string& stage) {
    cleanup_batch_seen |= stage.find("cleanup subdivision") != std::string::npos &&
                          progress.total_levels == 2;
    require(progress + 1e-9 >= last_progress && progress <= 1.0,
            "progress follows the whole operation monotonically");
    last_progress = progress;
    return true;
  });
  check_layers(first, {3, 6, 5, 4, 2, 1}, {3, 3, 2, 1, 2, 1},
               {.2, .2, .3, .5, .3, .5});
  setenv("TINGLECAM_PLANNING_WORKERS", "1", 1);
  const auto serial = plan(op);
  if (saved_workers) setenv("TINGLECAM_PLANNING_WORKERS", saved_workers->c_str(), 1);
  else unsetenv("TINGLECAM_PLANNING_WORKERS");
  require(serial.ok() && first.ok() &&
              cam::post_job(serial, job, tools, machines).text ==
              cam::post_job(first, job, tools, machines).text,
          "parallel cleanup retains serial geometry, engagement, execution order and links");
  require(cleanup_batch_seen, "cleanup groups expose both independent terraces to workers");


  op.depth_breaks = {4.5, 2.0};
  const auto broken = plan(op);
  check_layers(broken, {2, 4.5, 6, 5.5, 5, 11.0/3, 17.0/6, 4.0/3, 2.0/3},
               {2, 2.5, 1.5, 1, .5, 5.0/3, 5.0/6, 4.0/3, 2.0/3},
               {.2, .2, .2, .3, .5, .3, .5, .3, .5});

  op.adaptive_cleanup_stepover = false;
  const auto fixed = plan(op);
  require(fixed.ok(), "fixed-width cleanup remains available");
  if (fixed.ok()) for (const auto& level : fixed.operations.front().levels)
    require_near(level.radial_width, 1.2, 1e-9, "disabled adaptation keeps base width");
  op.finishing_subdivisions = 1;
  const auto coarse_only = plan(op);
  require(coarse_only.ok() && coarse_only.operations.front().levels.size() == 3,
          "depth breaks also work without finishing subdivisions");

  // The flat is at Z6, five millimetres below this blank's Z11 top.
  // Leave 0.25 mm and ensure the explicit break really machines the face.
  op.depth_breaks = {4.75};
  op.axial_stock_to_leave = 0.25;
  // Prime the inclusive slice first: the above-face slice must have its own
  // cache entry even though both heights round to the same 0.001 mm grid.
  const auto inclusive = geometry.part_outline(6.0, op.slice_grid);
  const auto flat = plan(op);
  require(flat.ok(), "an explicit flat depth with axial allowance plans");
  bool flat_cleared = false;
  if (flat.ok()) for (const auto& level : flat.operations.front().levels) {
    const auto removed_on_face = Clipper2Lib::Intersect(
        level.simulation.removed_material, wave::Paths64{box(26, 10, 34, 30)},
        Clipper2Lib::FillRule::NonZero);
    if (area(removed_on_face) > 1.0) {
      require(level.floor_z >= 6.25 - 1e-8, "the flat's axial allowance is preserved");
      if (std::abs(level.floor_z - 6.25) < 1e-8) flat_cleared = true;
    }
  }
  require(flat_cleared, "a depth break clears the horizontal face exactly to its allowance");
  require(wave::paths_equal_canonical(inclusive, geometry.part_outline(6.0, op.slice_grid)),
          "roughing does not change inclusive slices used by other operations");

  op.depth_breaks = {op.depth_to};
  require(!plan(op).ok(), "an endpoint is rejected as an intermediate break");
  op.depth_breaks.clear();
  op.cleanup_stepover_limit = std::numeric_limits<double>::quiet_NaN();
  require(!plan(op).ok(), "non-finite cleanup width is rejected");
  op.cleanup_stepover_limit = 0.5;
  op.finishing_subdivisions = 3;
  const auto cancelled = plan(op, [](double progress, const std::string&) {
    return progress < 0.5;
  });
  require(!cancelled.ok(), "cancellation during upward cleanup stops the operation");
}

void check_roughing_links() {
  std::cout << "stock-validated roughing layer links\n";
  cam::Operation operation;
  cam::detail::OperationSetup setup;
  setup.radius = 1; setup.stepover = .5; setup.no_engagement_feed = 2000;
  const auto machine = test_machine();
  const auto phase = [](wave::Point64 a, wave::Point64 b, double za, double zb, bool rapid) {
    cam::MotionPhase p;
    p.from=a; p.to=b; p.from_z=za; p.to_z=zb; p.rapid=rapid;
    p.rate=rapid ? 6000 : 1000;
    p.length=std::hypot(wave::to_world(b.x-a.x), wave::to_world(b.y-a.y));
    p.length=std::hypot(p.length,zb-za);
    p.kind=rapid ? wave::MoveKind::Retract : wave::MoveKind::Cut;
    return p;
  };
  const auto make_plan = [&](double first_z, double second_z) {
    cam::OperationPlan plan;
    cam::LevelPlan a,b;
    a.floor_z=first_z; a.axial_depth=3;
    b.floor_z=second_z; b.axial_depth=1;
    a.feeds.motion={phase(point(-1),point(-1),10,first_z,true),
                    phase(point(-1),point(0),first_z,first_z,false),
                    phase(point(0),point(0),first_z,10,true)};
    b.feeds.motion={phase(point(0),point(10),10,10,true),
                    phase(point(10),point(10),10,second_z,true),
                    phase(point(10),point(11),second_z,second_z,false),
                    phase(point(11),point(11),second_z,10,true)};
    plan.levels={a,b};
    for (auto& level : plan.levels) for (const auto& p : level.feeds.motion) {
      if (p.rapid) {
        level.feeds.stats.rapid_distance+=p.length;
        level.feeds.stats.rapid_minutes+=p.length/p.rate;
      } else {
        level.feeds.stats.cutting_distance+=p.length;
        level.feeds.stats.cutting_minutes+=p.length/p.rate;
      }
    }
    return plan;
  };
  const auto link = [&](cam::OperationPlan& plan) {
    cam::detail::link_roughing_levels(plan, cam::JobGeometry{}, operation, setup,
                                     machine,10,{},0,1);
    const cam::MotionPhase* previous = nullptr;
    double minutes=0, distance=0;
    for (const auto& level : plan.levels) for (const auto& p : level.feeds.motion) {
      if (previous) {
        require(previous->to == p.from, "linked motion is continuous in XY");
        require_near(previous->to_z,p.from_z,1e-9,"linked motion is continuous in Z");
      }
      minutes+=p.length/p.rate; distance+=p.length; previous=&p;
    }
    require_near(plan.stats.cutting_minutes+plan.stats.rapid_minutes,minutes,1e-9,
                 "linked estimate equals the commanded timeline");
    require_near(plan.stats.cutting_distance+plan.stats.rapid_distance,distance,1e-9,
                 "linked travel totals equal the commanded timeline");
  };
  auto rising=make_plan(-3,-2);
  rising.preview_context={{-3,0,std::make_shared<const wave::Paths64>(wave::Paths64{box(-5,-5,15,5)})}};
  rising.levels[0].simulation.removed_material={box(-2,-2,13,2)};
  link(rising);
  require(rising.levels[0].stays_down_after && rising.levels[1].feeds.begins_at_depth,
          "accumulated removal permits upward linking");
  require(rising.levels[1].feeds.motion.front().from_z==-3 &&
              rising.levels[1].feeds.motion.front().to_z==-2 &&
              rising.levels[1].feeds.motion.front().inter_layer,
          "upward cleanup rises before traversing");
  {
    auto tool=test_tool(); tool.diameter=2;
    operation.tool_id=tool.id; operation.machine_id=machine.id;
    operation.material_id="plastic";
    cam::Job job; job.operations={operation};
    cam::JobPlan posted_plan; posted_plan.rapid_plane=10;
    rising.tool_id=tool.id; rising.machine_id=machine.id; rising.planned=true;
    rising.spindle_rpm=12000; posted_plan.operations={rising};
    const auto posted=cam::post_job(posted_plan,job,cam::ToolLibrary{{tool}},
                                    cam::MachineLibrary{{machine}});
    require(posted.ok(),"stock-validated XYZ links post: "+posted.error);
    const auto next=posted.text.find("Depth -2");
    const auto up=posted.text.find("Z-2",next);
    const auto across=posted.text.find("X10",next);
    const auto clearance=posted.text.find("Z10",next);
    require(next!=std::string::npos && up<across && across<clearance,
            "posted cleanup rises and traverses before its final clearance retract");
    require(posted.text.find("X-1")<posted.text.find("Z-3"),
            "the first roughing entry establishes XY before descending");
  }
  // A polygonal stock subtraction can leave a tiny remnant inside the exact
  // circular tool footprint. Only a matching executed capsule in this Z slab
  // may explain it away; area alone must never authorize a link.
  auto remnant = make_plan(-3, -2);
  const auto speck = std::make_shared<const wave::Paths64>(
      wave::Paths64{box(0.996, 0.010, 0.998, 0.011)});
  remnant.preview_context = {{-3, 0, speck}};
  wave::SimulatedMove swept;
  wave::MaterialStep step;
  step.from = point(-1); step.to = point(0);
  swept.steps.push_back(step);
  remnant.levels[0].simulation.moves.push_back(swept);
  auto unexplained = remnant;
  unexplained.levels[0].simulation.moves.clear();
  link(unexplained);
  require(!unexplained.levels[0].stays_down_after,
          "small collision area alone does not authorize upward travel");
  link(remnant);
  require(remnant.levels[0].stays_down_after,
          "an executed capsule proves a numerical remnant already cleared");
  auto overhead_remnant = make_plan(-3, -2);
  overhead_remnant.preview_context = {{1, 2, speck}};
  overhead_remnant.levels[0].simulation.moves.push_back(swept);
  link(overhead_remnant);
  require(!overhead_remnant.levels[0].stays_down_after,
          "a footprint cleared below cannot excuse overhead stock");

  const auto part_file = std::filesystem::temp_directory_path() / "tinglecam-link-part.obj";
  { std::ofstream mesh(part_file);
    mesh << "v -0.5 1.5 0\nv 0.5 1.5 0\nv 0.5 2.5 0\nv -0.5 2.5 0\n"
            "f 1 2 3\nf 1 3 4\n";
  }
  cam::JobGeometry part_geometry;
  part_geometry.load_part(part_file);
  std::filesystem::remove(part_file);
  auto allowance_operation = operation;
  allowance_operation.radial_stock_to_leave = 1.0;
  auto cleared_allowance = make_plan(-3, -2);
  cam::detail::link_roughing_levels(cleared_allowance, part_geometry,
      allowance_operation, setup, machine, 10, {}, 0, 1);
  require(cleared_allowance.levels[0].stays_down_after,
          "nominal allowance is not recreated where no stock remains");
  auto uncut_allowance = make_plan(-3, -2);
  uncut_allowance.preview_context = {{-3, 1, std::make_shared<const wave::Paths64>(
      wave::Paths64{box(-0.5, 0.5, 0.5, 1.5)})}};
  cam::detail::link_roughing_levels(uncut_allowance, part_geometry,
      allowance_operation, setup, machine, 10, {}, 0, 1);
  require(!uncut_allowance.levels[0].stays_down_after,
          "remaining allowance is protected as actual stock");
  auto protected_part = make_plan(-3, -2);
  for (auto& level : protected_part.levels) for (auto& motion : level.feeds.motion) {
    motion.from.y += 2000; motion.to.y += 2000;
  }
  auto claimed_cut = swept;
  for (auto& sample : claimed_cut.steps) {
    sample.from.y += 2000; sample.to.y += 2000;
  }
  protected_part.levels[0].simulation.moves.push_back(claimed_cut);
  cam::detail::link_roughing_levels(protected_part, part_geometry,
      allowance_operation, setup, machine, 10, {}, 0, 1);
  require(!protected_part.levels[0].stays_down_after,
          "finished part protection cannot be waived by claimed earlier removal");

  auto falling=make_plan(-2,-3);
  link(falling);
  require(falling.levels[0].stays_down_after,"a clear descent can stay down");
  require(falling.levels[1].feeds.motion.front().from_z==-2 &&
              falling.levels[1].feeds.motion.front().to_z==-2,
          "descending links traverse before descending");
  auto blocked=make_plan(-2,-3);
  blocked.preview_context={{-3,-2.5,std::make_shared<const wave::Paths64>(wave::Paths64{box(9,-2,12,2)})}};
  link(blocked);
  require(!blocked.levels[0].stays_down_after,
          "stock below the traversal blocks an unsafe vertical descent");
  auto roof=make_plan(-3,-2);
  roof.preview_context={{1,2,std::make_shared<const wave::Paths64>(wave::Paths64{box(-2,-2,2,2)})}};
  link(roof);
  require(!roof.levels[0].stays_down_after,
          "overhead stock blocks a cutter column that looks clear at the tip");
  auto detour=make_plan(-3,-2);
  detour.preview_context={{-4,0,std::make_shared<const wave::Paths64>(wave::Paths64{box(4,-1,6,1)})}};
  link(detour);
  require(detour.levels[0].stays_down_after,
          "the router finds a safe detour around an intervening island");
  auto hollow=make_plan(-3,-2);
  auto hole=box(-2,-2,13,2);
  std::reverse(hole.begin(),hole.end());
  const auto shell=std::make_shared<const wave::Paths64>(
      wave::Paths64{box(-100,-100,100,100),hole,box(200,200,210,210)});
  hollow.preview_context={{-4,-1,shell},{-1,0,shell}};
  link(hollow);
  require(hollow.levels[0].stays_down_after,
          "collision filtering preserves holes through repeated stock slabs");
  auto covered_hole=make_plan(-3,-2);
  covered_hole.preview_context=hollow.preview_context;
  covered_hole.preview_context.push_back(
      {1,2,std::make_shared<const wave::Paths64>(wave::Paths64{box(-50,-50,50,50)})});
  link(covered_hole);
  require(!covered_hole.levels[0].stays_down_after,
          "an enclosing overhead region blocks a hole in lower stock");
  auto skipped=make_plan(-3,-2);
  cam::LevelPlan empty; empty.nothing_to_cut=true;
  skipped.levels.insert(skipped.levels.begin()+1,empty);
  link(skipped);
  require(skipped.levels[0].stays_down_after && skipped.levels[2].feeds.begins_at_depth,
          "empty layers do not break the executed layer chain");
  auto cancelled=make_plan(-3,-2);
  cam::detail::link_roughing_levels(cancelled,cam::JobGeometry{},operation,setup,
      machine,10,[](double,const std::string&){return false;},0,1);
  require(cancelled.error=="Cancelled","linking remains cancellable");
}

void check_axial_allowance() {
  std::cout << "axial stock allowance\n";
  const auto file = std::filesystem::temp_directory_path() /
                    "tinglecam-axial-allowance.obj";
  write_stepped_part(file);

  cam::Tool tool = test_tool();
  tool.diameter = 6.0;
  tool.flute_length = 30.0;
  const cam::Machine machine = test_machine();
  cam::ToolLibrary tools;
  tools.upsert(tool);
  cam::MachineLibrary machines;
  machines.upsert(machine);
  cam::MaterialLibrary materials;
  materials.materials.push_back({"plastic", "Plastic"});

  // Floors land at 9.25, 8.25, ..., so one sits a quarter of a millimetre
  // above the face -- the case the allowance exists to stop.
  const auto lowest_pass_onto_the_face = [&](double allowance) {
    cam::JobGeometry geometry;
    geometry.load_part(file);
    geometry.set_box_stock(true, 4.0);
    cam::Operation operation;
    operation.tool_id = tool.id;
    operation.machine_id = machine.id;
    operation.material_id = "plastic";
    operation.stepover_ratio = 0.4;
    operation.stepdown = 1.0;
    operation.depth_from = 0.75;
    operation.depth_to = 6.0;
    operation.radial_stock_to_leave = 0.0;
    operation.axial_stock_to_leave = allowance;
    cam::Job job;
    job.operations = {operation};
    const auto plan =
        cam::plan_job(geometry, job, tools, machines, materials, {});
    require(plan.ok(), "stepped part plans: " + plan.error);
    if (!plan.ok() || plan.operations.empty())
      return std::pair<bool, double>{false, 0.0};

    // Well inside the low half, so a pass that reached the face is unmistakable.
    const wave::Paths64 face{box(24.0, 8.0, 36.0, 32.0)};
    double lowest = 0.0;
    for (const auto& level : plan.operations.front().levels) {
      if (level.nothing_to_cut) continue;
      const auto onto = Clipper2Lib::Intersect(
          level.simulation.removed_material, face, Clipper2Lib::FillRule::NonZero);
      if (area(onto) < 1.0) continue;
      if (lowest == 0.0 || level.floor_z < lowest) lowest = level.floor_z;
    }
    return std::pair<bool, double>{true, lowest};
  };

  const auto bare_result = lowest_pass_onto_the_face(0.0);
  require(bare_result.first, "the zero-allowance stepped part plans");
  const double bare = bare_result.second;
  require(bare > 6.0 && bare < 6.5,
          "with no allowance a pass comes down within a fraction of the face");
  const auto left_result = lowest_pass_onto_the_face(0.5);
  require(left_result.first, "the positive-allowance stepped part plans");
  const double left = left_result.second;
  require(left >= 6.5 - 1e-9,
          "an axial allowance holds every pass at least that far above the face");
  require(left > bare,
          "and the allowance is what moved it, not the depth range");
  const auto overcut_result = lowest_pass_onto_the_face(-0.5);
  require(overcut_result.first,
          "a negative axial allowance is accepted by planning");
  std::filesystem::remove(file);
}

wave::FrontLineage single_front_lineage(const wave::Path64& front,
                                        const wave::Path64& band) {
  wave::FrontLineage lineage;
  lineage.stepover = 2.0;
  wave::LeadingFront leading;
  leading.path = front;
  leading.band = 0;
  lineage.generations.push_back({{leading}, {wave::Paths64{band}}});
  lineage.entry_fronts.push_back(leading);
  return lineage;
}

// A node whose two fronts both end on one static boundary ring, with the
// propagation bands inset from that ring by `band_inset` millimetres. The
// rails run along the ring, so the inset is exactly the disagreement between
// the two descriptions of the same wall that the rail check has to survive.
wave::FrontLineage railed_node(double band_inset) {
  const double lo = band_inset;
  const double hi = 10.0 - band_inset;
  wave::FrontLineage lineage;
  lineage.stepover = 2.0;
  lineage.boundary_components = {box(0, 0, 10, 10)};
  lineage.generations.resize(3);
  lineage.generations[0].bands = {wave::Paths64{box(lo, 0, hi, 2)}};
  lineage.generations[1].bands = {wave::Paths64{box(lo, 2, hi, 4)}};
  lineage.generations[2].bands = {wave::Paths64{box(lo, 4, 5, 6)},
                                  wave::Paths64{box(5, 4, hi, 6)}};

  auto contact = [](wave::Point64 at) {
    wave::BoundaryContact result;
    result.valid = true;
    result.component = 0;
    result.point = at;
    return result;
  };

  wave::LeadingFront first;
  first.path = {point(0, 1), point(10, 1)};
  first.children = {0};
  first.boundary_contacts[0] = contact(point(0, 1));
  first.boundary_contacts[1] = contact(point(10, 1));
  lineage.generations[0].fronts = {first};

  // Two children keep the node non-terminal, so its footprint is closed by the
  // two rails rather than by a terminal cap.
  wave::LeadingFront second;
  second.path = {point(0, 3), point(10, 3)};
  second.parents = {0};
  second.children = {0, 1};
  second.boundary_contacts[0] = contact(point(0, 3));
  second.boundary_contacts[1] = contact(point(10, 3));
  lineage.generations[1].fronts = {second};

  wave::LeadingFront left;
  left.path = {point(0, 5), point(5, 5)};
  left.parents = {0};
  left.band = 0;
  wave::LeadingFront right;
  right.path = {point(5, 5), point(10, 5)};
  right.parents = {0};
  right.band = 1;
  lineage.generations[2].fronts = {left, right};
  return lineage;
}

// The boundary rings and the propagation bands are separately rounded
// descriptions of the same walls, and a rail is deliberately laid on the ring.
// A rail may therefore miss the bands by the same slack a contact was accepted
// with -- and by no more than that.
void check_boundary_rail_slack() {
  std::cout << "boundary rail slack\n";
  const wave::LayerTopology rounded =
      wave::build_layer_topology(railed_node(0.003));
  const std::size_t node = rounded.front_nodes[0][0];
  require_near(area(rounded.nodes[node].footprint), 20.0, 1e-6,
               "a rail a few microns off its bands still closes its node");

  bool rejected = false;
  try {
    (void)wave::build_layer_topology(railed_node(0.5));
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  require(rejected,
          "the slack is a rounding allowance, not a licence to leave the bands");
}

// Two stock walls the subject keeps apart, so the wave has two entry fronts and
// declining one is a question with a visible answer.
void check_entry_front_selection() {
  std::cout << "entry front selection\n";
  const wave::Paths64 stock{box(0, 0, 40, 10)};
  // A bar across the middle, leaving a corridor at each end and no way round.
  const wave::Paths64 subject{box(12, -1, 28, 11)};
  const auto all = wave::build_wavefront(subject, stock, 1.0);
  const auto lineage = wave::build_front_lineage(all);
  require(lineage.entry_fronts.size() == 2,
          "a subject spanning the stock leaves two separate entries");
  if (lineage.entry_fronts.size() != 2) return;

  // Decline one, and check the wave never starts there rather than merely being
  // told not to cut there. The corridors are mirror images, so what the wave
  // can still reach answers that exactly.
  const double both = area(all.reached_generations.back());
  require(both > 0.0, "the wave reaches material from the two entries");
  wave::EntryMute mute;
  mute.anchors.push_back(wave::representative_point(lineage.entry_fronts[0]));
  mute.tolerance = wave::to_integer(1.0);
  const auto one = wave::build_wavefront(subject, stock, 1.0, 0.0, 0.0, 2000, mute);
  require(one.muted_entries == 1, "the declined seed piece is dropped");
  const auto reduced = wave::build_front_lineage(one);
  require(reduced.entry_fronts.size() == 1,
          "only the entry that was kept propagates");
  require_near(area(one.reached_generations.back()), both * 0.5, 1.0,
               "the corridor behind the declined entry is out of reach, "
               "since the subject blocks the way round");

  // The entry that was kept is exactly where it was. Nothing about a front
  // depends on which of its neighbours were declined, which is what lets a
  // choice be offered without propagating anything.
  const wave::LeadingFront& before = lineage.entry_fronts[1];
  const wave::LeadingFront& after = reduced.entry_fronts.front();
  require(before.path == after.path,
          "declining one entry leaves the others exactly where they were");

  // An anchor that names nothing is not an error and mutes nothing: that is how
  // a selection outlives a change of depth that moves the fronts.
  wave::EntryMute stray;
  stray.anchors.push_back(point(500.0, 500.0));
  stray.tolerance = wave::to_integer(1.0);
  const auto untouched =
      wave::build_wavefront(subject, stock, 1.0, 0.0, 0.0, 2000, stray);
  require(untouched.muted_entries == 0,
          "an anchor matching no entry declines nothing");
  require(wave::build_front_lineage(untouched).entry_fronts.size() == 2,
          "and leaves every entry propagating");
}

// The blank that stands in for a stock model when there is none.
void check_box_stock(const std::filesystem::path& part_mesh) {
  std::cout << "box stock\n";
  cam::JobGeometry geometry;
  require(!geometry.has_stock(), "nothing loaded is not a blank");
  geometry.load_part(part_mesh);
  require(!geometry.has_stock(), "a part alone is still not a blank");

  geometry.set_box_stock(true, 4.0);
  require(geometry.has_stock(), "the box stands in for a stock model");
  require(geometry.box_stock(), "and reports that it is what is standing in");

  const auto& bounds = geometry.part().mesh().bounds;
  require(bounds.valid, "the part has bounds to grow");
  if (!bounds.valid) return;
  const wave::Paths64& box = geometry.stock_outline(0.0, 32);
  require(box.size() == 1 && box.front().size() == 4,
          "the blank is one rectangle");
  if (box.size() != 1) return;
  const auto rect = Clipper2Lib::GetBounds(box);
  require_near(wave::to_world(rect.left), bounds.lower.x - 4.0, 1e-3,
               "the blank is grown by the margin in X");
  require_near(wave::to_world(rect.right), bounds.upper.x + 4.0, 1e-3,
               "on both sides");
  require_near(wave::to_world(rect.bottom - rect.top),
               static_cast<double>(bounds.upper.y - bounds.lower.y) + 8.0, 1e-3,
               "and in Y");

  const cam::BoxStockMargins directional{1.0, 3.0, 2.0, 4.0, 6.0, 7.0};
  geometry.set_box_stock(true, directional);
  const auto directional_box =
      Clipper2Lib::GetBounds(geometry.stock_outline(0.0, 32));
  require_near(wave::to_world(directional_box.left), bounds.lower.x - 1.0,
               1e-3, "the negative X margin is independent");
  require_near(wave::to_world(directional_box.right), bounds.upper.x + 3.0,
               1e-3, "the positive X margin is independent");
  require_near(wave::to_world(directional_box.top), bounds.lower.y - 2.0,
               1e-3, "the negative Y margin is independent");
  require_near(wave::to_world(directional_box.bottom), bounds.upper.y + 4.0,
               1e-3, "the positive Y margin is independent");
  double directional_low = 0.0;
  double directional_high = 0.0;
  require(geometry.work_range(directional_low, directional_high),
          "directional Z margins preserve a work range");
  require_near(directional_low, bounds.lower.z - 6.0, 1e-3,
               "the negative Z margin extends the work range");
  require_near(directional_high, bounds.upper.z + 7.0, 1e-3,
               "the positive Z margin extends the work range");

  // Every depth gets the same rectangle: it is a prism, not a sliced solid.
  double low = 0.0;
  double high = 0.0;
  require(geometry.work_range(low, high), "a box blank still has a depth range");
  require(geometry.stock_outline(low, 32) == box &&
              geometry.stock_outline(high, 32) == box,
          "the blank is the same at every depth");

  const double wider = 9.0;
  geometry.set_box_stock(true, wider);
  const auto grown = Clipper2Lib::GetBounds(geometry.stock_outline(0.0, 32));
  require_near(wave::to_world(grown.left), bounds.lower.x - wider, 1e-3,
               "changing the margin re-cuts the blank rather than reusing it");

  // A real model always wins, so turning the fallback on cannot quietly
  // replace a blank the job actually names.
  geometry.load_stock(part_mesh);
  require(geometry.has_stock() && !geometry.box_stock(),
          "a loaded stock model wins over the box");
  geometry.clear_stock();
  require(geometry.box_stock(),
          "and dropping it hands the job back to the box");
}

void check_engagement_limits(const std::filesystem::path& part_mesh) {
  std::cout << "engagement limit diagnostics\n";
  cam::Tool tool = test_tool();
  tool.diameter = 3.175;
  tool.flute_length = 25;
  const cam::Machine machine = test_machine();
  cam::JobGeometry geometry;
  geometry.load_part(part_mesh);
  geometry.set_box_stock(true, 1.0);
  cam::Operation operation;
  operation.tool_id = tool.id;
  operation.machine_id = machine.id;
  operation.material_id = "plastic";
  operation.stepover_ratio = 0.4;
  operation.stepdown = 2;
  operation.depth_to = 3;
  operation.finishing_subdivisions = 4;
  cam::Job job;
  job.operations = {operation};
  cam::ToolLibrary tools;
  tools.upsert(tool);
  cam::MachineLibrary machines;
  machines.upsert(machine);
  cam::MaterialLibrary materials;
  materials.materials.push_back({"plastic", "Plastic"});
  const auto plan = cam::plan_job(geometry, job, tools, machines, materials, {});
  require(plan.ok(), "limited engagement plans: " + plan.error);
  for (const auto& warning : plan.warnings)
    require(warning.find("Stepover reduced") == std::string::npos &&
                warning.find("Stepdown reduced") == std::string::npos,
            "tools no longer impose material strategy limits");
  if (!plan.operations.empty()) {
    require_near(plan.operations.front().levels.front().axial_depth, 2.0, 1e-9,
                 "requested stepdown is used without a tool-material cap");
    for (const auto& level : plan.operations.front().levels)
      require(level.axial_depth <= 2.0 + 1e-9,
              "requested stepdown bounds each planned layer");
  }
}

// A real roughing-to-roughing-to-finishing plan exercises both operation
// ordering and the inherited-stock planning and simulation handoff.
void check_multi_operation_plan(const std::filesystem::path& data,
                                const std::filesystem::path& part_mesh) {
  std::cout << "multi-operation planning\n";
  const auto materials = cam::load_materials(data / "materials");
  const auto machines = cam::load_machines(data / "machines");
  const auto tools = cam::load_tools(data / "starter-tools.json");
  const cam::Tool* tool = tools.tools.empty() ? nullptr : &tools.tools.front();
  if (!tool) return;
  const std::string material_id = "aluminium-6082";
  const cam::Machine* machine = nullptr;
  for (const auto& candidate : machines.machines)
    if (candidate.feeds_for(material_id)) machine = &candidate;
  if (!machine || !materials.find(material_id)) return;

  cam::JobGeometry geometry;
  geometry.load_part(part_mesh);
  geometry.set_box_stock(true, 4.0);
  double low = 0.0;
  double high = 0.0;
  require(geometry.work_range(low, high),
          "the multi-operation fixture spans a depth range");
  if (!(high > low)) return;

  cam::Operation rough;
  rough.name = "Rough first";
  rough.tool_id = tool->id;
  rough.machine_id = machine->id;
  rough.material_id = material_id;
  rough.depth_from = 0.0;
  rough.depth_to = std::min(high - low, 2.0);
  rough.stepdown = rough.depth_to;
  rough.stepover_ratio = 0.7;
  rough.path_pitch = 0.5;
  rough.finishing_subdivisions = 4;

  cam::Operation rest = rough;
  rest.name = "Rest rough second";
  rest.finishing_subdivisions = 1;

  cam::Operation finish = rest;
  finish.name = "Finish third";
  finish.mode = cam::OperationMode::ContourFinishing;
  finish.stepdown = rough.stepdown * 0.5;
  finish.radial_stock_to_leave = 0.0;

  cam::Job job;
  job.operations = {rough, rest, finish};
  const cam::JobPlan plan =
      cam::plan_job(geometry, job, tools, machines, materials, {});
  require(plan.ok(), "roughing followed by contour finishing plans: " + plan.error);
  require(plan.operations.size() == 3,
          "the ordered operation list is preserved in the plan");
  if (!plan.ok() || plan.operations.size() != 3) return;
  require(plan.operations[0].operation_name == rough.name &&
              plan.operations[1].operation_name == rest.name &&
              plan.operations[2].operation_name == finish.name,
          "operation execution order matches the job list");

  const cam::OperationPlan& subdivided = plan.operations[0];
  require(subdivided.subdivisions.size() == 4,
          "four finishing subdivisions remain one planned operation");
  if (subdivided.subdivisions.size() == 4) {
    const std::size_t base_count = subdivided.subdivisions.front().level_count;
    require(subdivided.levels.size() == base_count * 4,
            "each subdivision allocates the base number of preview layers");
    for (std::size_t pass = 0; pass < 4; ++pass) {
      const cam::OperationSubdivision& range = subdivided.subdivisions[pass];
      require(range.level_count == base_count,
              "every subdivision keeps its own complete layer record");
      if (range.level_count == 0) continue;
      const cam::LevelPlan& first = subdivided.levels[range.first_level];
      require(first.subdivision == pass && first.subdivision_level == 0,
              "flattened execution levels retain subdivision identity");
      const double expected_depth =
          rough.depth_from + rough.stepdown * (1.0 - pass / 4.0);
      require_near(high - first.floor_z, expected_depth, 1e-8,
                   "subdivision first floors are interleaved in execution order");
    }
    bool inherited_between_subdivisions = false;
    for (std::size_t pass = 1; pass < 4; ++pass) {
      const cam::OperationSubdivision& range = subdivided.subdivisions[pass];
      for (std::size_t local = 0; local < range.level_count; ++local) {
        const cam::LevelPlan& level =
            subdivided.levels[range.first_level + local];
        if (wave::material_area(level.incoming_stock) + 0.001 <
            wave::material_area(level.stock_slice))
          inherited_between_subdivisions = true;
      }
    }
    require(inherited_between_subdivisions,
            "later subdivisions inherit stock from earlier subdivisions");

    // A subdivision group is planned on its own but runs on from the one before
    // it, so its first cutting pass has to open with a rapid in from wherever
    // that group left the cutter. Planning each group with nowhere to come from
    // left that rapid out: the post still emitted the motion, but the schedule
    // did not carry it, so the preview drew the toolpath appearing at the new
    // group out of nothing and the travel totals were short by it.
    const cam::LevelPlan* previous_cut = nullptr;
    std::size_t openings_checked = 0;
    for (const cam::LevelPlan& level : subdivided.levels) {
      if (level.nothing_to_cut || level.feeds.motion.empty()) continue;
      if (previous_cut != nullptr &&
          previous_cut->subdivision != level.subdivision) {
        std::size_t arrivals = 0;
        for (const cam::MotionPhase& phase : level.feeds.motion)
          if (phase.arrival) ++arrivals;
        require(arrivals == 1,
                "a subdivision opens with one rapid in from the group before it");
        require(level.feeds.motion.front().arrival,
                "and making it is the first thing that pass does");
        require(level.feeds.motion.front().from ==
                    previous_cut->feeds.motion.back().to,
                "starting where the group before it left the cutter standing");
        ++openings_checked;
      }
      previous_cut = &level;
    }
    // Not every group necessarily cuts -- an earlier one can leave a depth with
    // nothing on it -- so the count is not fixed at three. What matters is that
    // a boundary was reached at all, or the checks above passed by never
    // running.
    require(openings_checked >= 1,
            "at least one subdivision boundary was there to be checked");

    require(subdivided.composed_stock && !subdivided.preview_context.empty(),
            "subdivision playback uses physical stock slabs");
  }

  bool roughing_inherited_less_stock = false;
  bool roughing_compared = false;
  for (const cam::LevelPlan& level : plan.operations[1].levels) {
    if (level.incoming_stock.empty() || level.wave.stock.empty()) continue;
    roughing_compared = true;
    require(wave::paths_equal_canonical(level.wave.stock,
                                        level.incoming_stock),
            "downstream roughing wave propagates through inherited stock");
    if (wave::material_area(level.incoming_stock) + 0.001 <
        wave::material_area(level.stock_slice))
      roughing_inherited_less_stock = true;
  }
  require(roughing_compared,
          "the second roughing operation has an inherited planning level");
  require(roughing_inherited_less_stock,
          "the second roughing operation inherits the first rougher's removal");
  require(plan.operations[1].stats.cut_levels == 0,
          "an identical second roughing operation does not recut cleared air");

  bool inherited_less_stock = false;
  bool compared = false;
  for (const cam::LevelPlan& level : plan.operations[2].levels) {
    if (level.nothing_to_cut || level.simulation.initial_stock.empty()) continue;
    compared = true;
    require(wave::paths_equal_canonical(level.incoming_stock,
                                        level.simulation.initial_stock),
            "downstream simulation starts with its recorded incoming stock");
    const double nominal = wave::material_area(level.stock_slice);
    const double incoming = wave::material_area(level.incoming_stock);
    require(incoming <= nominal + 0.001,
            "stock handoff never recreates material outside the nominal blank");
    if (incoming + 0.001 < nominal) inherited_less_stock = true;
  }
  require(compared, "the finishing operation has a simulated cutting level");
  require(inherited_less_stock,
          "at least one finishing level inherits material removed by roughing");
}

void check_lsjig_subdivision_cleanup(const std::filesystem::path& data,
                                     const std::filesystem::path& part_mesh) {
  std::cout << "LsJig subdivision cleanup\n";
  const cam::ToolLibrary tools = cam::load_tools(data / "starter-tools.json");
  const cam::MachineLibrary machines = cam::load_machines(data / "machines");
  const cam::MaterialLibrary materials = cam::load_materials(data / "materials");
  const cam::Tool* tool = tools.find("6mm-3f-alu");
  const cam::Machine* machine = machines.find("grbl-router");
  const cam::Material* material = materials.find("aluminium-6082");
  require(tool && machine && material,
          "the LsJig cleanup fixture profiles are available");
  if (!tool || !machine || !material) return;

  cam::JobGeometry geometry;
  geometry.load_part(part_mesh);
  geometry.set_box_stock(true, 5.0);
  double low = 0.0;
  double high = 0.0;
  require(geometry.work_range(low, high),
          "the LsJig cleanup fixture spans a depth range");
  if (!(high > low)) return;

  cam::Operation operation;
  operation.tool_id = tool->id;
  operation.machine_id = machine->id;
  operation.material_id = material->id;
  operation.depth_to = high - low;
  operation.finishing_subdivisions = 4;
  operation.cleanup_tolerance = tool->diameter * 0.5;
  cam::Job job;
  job.box_stock = true;
  job.box_stock_margin = 5.0;
  job.operations = {operation};

  const cam::JobPlan strict_plan =
      cam::plan_job(geometry, job, tools, machines, materials, {});
  require(strict_plan.ok(),
          "the strict LsJig subdivision fixture plans: " + strict_plan.error);
  if (!strict_plan.ok() || strict_plan.operations.empty()) return;
  std::size_t strict_cutting_layers = 0;
  for (const cam::LevelPlan& level : strict_plan.operations.front().levels)
    if (!level.nothing_to_cut && level.simulation.stats.removed_area > 0.0)
      ++strict_cutting_layers;
  require(strict_plan.operations.front().levels.size() == 68 &&
              strict_cutting_layers > 0,
          "strict cleanup retains all 68 scheduled layers and cuts stock");
  const cam::OperationPlan& strict_rough = strict_plan.operations.front();
  std::size_t strict_later_cutting = 0;
  std::size_t strict_later_reached = 0;
  for (std::size_t subdivision = 1;
       subdivision < strict_rough.subdivisions.size(); ++subdivision) {
    const cam::OperationSubdivision& range =
        strict_rough.subdivisions[subdivision];
    for (std::size_t local = 0; local < range.level_count; ++local) {
      const cam::LevelPlan& level =
          strict_rough.levels[range.first_level + local];
      if (!level.wave.reached_generations.empty()) ++strict_later_reached;
      if (!level.nothing_to_cut && level.simulation.stats.removed_area > 0.0)
        ++strict_later_cutting;
    }
  }

  job.operations.front().cleanup_tolerance = 0.15;
  const cam::JobPlan plan =
      cam::plan_job(geometry, job, tools, machines, materials, {});
  require(plan.ok(), "the LsJig subdivision cleanup plans: " + plan.error);
  if (!plan.ok() || plan.operations.empty()) return;

  const cam::OperationPlan& rough = plan.operations.front();
  require(rough.subdivisions.size() == 52,
          "the LsJig cleanup follows one coarse stack with 51 upward layers");
  if (rough.subdivisions.size() != 52) return;
  std::size_t upward_links = 0;
  const cam::MotionPhase* preceding_motion = nullptr;
  double commanded_minutes = 0;
  for (const auto& level : rough.levels) {
    if (level.nothing_to_cut) continue;
    if (level.finishing_index > 0 && level.stays_down_after) ++upward_links;
    require(level.feeds.authoritative_motion,
            "roughing posts the same linked timeline used for playback");
    for (const auto& motion : level.feeds.motion) {
      if (preceding_motion) {
        require(preceding_motion->to == motion.from,
                "LsJig linked motion has no missing XY moves");
        require_near(preceding_motion->to_z,motion.from_z,1e-8,
                     "LsJig linked motion has no missing Z moves");
      }
      commanded_minutes += motion.length / motion.rate;
      preceding_motion = &motion;
    }
  }
  require(upward_links > 0,"LsJig cleanup crosses subdivision boundaries without retracting");
  require_near(commanded_minutes,rough.stats.cutting_minutes+rough.stats.rapid_minutes,
               1e-7,"LsJig timing includes precisely the linked commanded moves");
  std::size_t later_layers = 0;
  std::size_t cutting_layers = 0;
  std::size_t reached_layers = 0;
  for (std::size_t subdivision = 1; subdivision < rough.subdivisions.size();
       ++subdivision) {
    const cam::OperationSubdivision& range = rough.subdivisions[subdivision];
    for (std::size_t local = 0; local < range.level_count; ++local) {
      ++later_layers;
      const cam::LevelPlan& level = rough.levels[range.first_level + local];
      if (!level.wave.reached_generations.empty()) ++reached_layers;
      if (!level.nothing_to_cut && level.simulation.stats.removed_area > 0.0)
        ++cutting_layers;
    }
  }
  require(later_layers == 51,
          "the LsJig fixture has 51 layers after its first subdivision");
  require(reached_layers > strict_later_reached,
          "exterior cutter reach seeds more later LsJig roughing layers");
  require(cutting_layers > strict_later_cutting,
          "exterior cutter reach removes stock on more later LsJig layers");
  require(rough.stats.unsafe_plunges == 0 &&
              rough.stats.unresolved_entries == 0 &&
              rough.stats.lifts_over_subject == 0 &&
              rough.stats.unreachable_fronts == 0,
          "exterior-domain roughing introduces no unsafe or unresolved motion");
  require_near(rough.stats.gouge_area, strict_rough.stats.gouge_area, 1e-6,
               "exterior-domain roughing introduces no additional gouge area");
}

// Z associations preserve the wavefront topology between adjacent layers.
}  // namespace cam_test
