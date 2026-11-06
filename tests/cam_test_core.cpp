#include "cam_test_support.h"
#include "cam/heightmap_simulation.h"
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
  return machine;
}

// 12000 rpm x 2 flutes x 0.0208333 mm is 500 mm/min at full engagement.
cam::Tool test_tool() {
  cam::Tool tool;
  tool.id = "test-tool";
  tool.name = "6 mm test tool";
  tool.diameter = 6.0;
  tool.flute_length = 16.0;
  tool.flutes = 2;
  tool.cutting.push_back({"plastic", 12000.0, 500.0 / 24000.0, 120.0});
  return tool;
}

cam::CuttingFeeds test_feeds() {
  return cam::resolve_cutting_feeds(test_tool(), test_machine(), "plastic", {});
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

wave::Toolpath straight_cut(std::size_t pieces) {
  wave::Toolpath toolpath;
  wave::ToolpathMove move;
  move.kind = wave::MoveKind::Cut;
  for (std::size_t index = 0; index <= pieces; ++index)
    move.path.push_back(point(static_cast<double>(index)));
  move.length = static_cast<double>(pieces);
  toolpath.moves.push_back(move);
  return toolpath;
}

void check_engagement_feeds() {
  std::cout << "chip-thinning engagement feeds\n";
  const cam::CuttingFeeds feeds = test_feeds();
  require_near(feeds.full_feed(), 500.0, 1e-6, "full feed is rpm x flutes x chip load");
  for (const auto [engagement, expected] :
       std::vector<std::pair<double, double>>{{1.5, 500.0},
                                              {1.0, 500.0},
                                              {0.5, 500.0},
                                              {0.25, 500.0 / std::sqrt(0.75)},
                                              {0.0, 2000.0}})
    require_near(feeds.feed_at(engagement), expected, 1e-6,
                 "lighter radial engagement feeds faster for the same chip");
  cam::CuttingFeeds capped = feeds;
  capped.maximum_feed = 900.0;
  cam::FeedLimit limit = cam::FeedLimit::Engagement;
  require_near(capped.feed_at(0.0, &limit), 900.0, 1e-9, "the machine maximum caps the feed");
  require(limit == cam::FeedLimit::Machine, "and says that it did");

  cam::Tool bare = test_tool();
  bare.cutting.clear();
  bool missing = false;
  try { cam::resolve_cutting_feeds(bare, test_machine(), "plastic", {}); }
  catch (const std::runtime_error&) { missing = true; }
  require(missing, "a tool without cutting data for the material cannot be planned");
  cam::CuttingOverrides overrides;
  overrides.spindle_rpm = 30000.0;
  overrides.chip_load = 0.01;
  overrides.plunge_feed = 50.0;
  const auto overridden = cam::resolve_cutting_feeds(bare, test_machine(), "plastic", overrides);
  require_near(overridden.spindle_rpm, 24000.0, 1e-9, "spindle overrides clamp to the machine");
  require_near(overridden.full_feed(), 24000.0 * 2 * 0.01, 1e-9,
               "overrides stand in for missing cutting data");
}

void check_feed_compatible_merge() {
  std::cout << "post-feed line merging\n";
  const auto merged = cam::plan_feeds(straight_cut(4), test_machine(), test_feeds());
  require(merged.segments.size() == 1,
          "collinear segments at one feed merge after assignment");
  if (!merged.segments.empty())
    require_near(merged.segments.front().length, 4.0, 1e-9,
                 "merging preserves travelled distance");
  wave::Toolpath bent = straight_cut(2);
  bent.moves.front().path.push_back(point(2.0, 1.0));
  bent.moves.front().length += 1.0;
  require(cam::plan_feeds(bent, test_machine(), test_feeds()).segments.size() == 2,
          "a change of direction prevents a merge");
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

  raw.moves.resize(1);
  raw.moves.front().path={point(0,0),point(.35,0),point(.35,.65)};
  raw.moves.front().length=1;
  const auto bent=wave::resample_toolpath(raw,.2);
  bool kept_corner=false;
  for (const auto& move:bent.moves)
    for (const auto& p:move.path) kept_corner |= p==point(.35,0);
  require(kept_corner,"resampling must retain a corner on a safe route");
  require_near(wave::toolpath_length(bent),1,1e-6,
               "resampling must not shortcut a corner through an obstacle");
  raw.moves.front().kind=wave::MoveKind::Link;
  raw.moves.front().path={point(0,0),point(.35,0),point(.7,.02)};
  raw.moves.front().length=.35+std::hypot(.35,.02);
  const auto shallow=wave::resample_toolpath(raw,.5);
  kept_corner=false;
  for(const auto& move:shallow.moves)
    for(const auto& p:move.path) kept_corner |= p==point(.35,0);
  require(kept_corner,"sampling must retain shallow bends on clearance-boundary links");
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

// The lowest and highest stock top over a rectangle of the grid.
std::pair<double, double> face_heights(const wave::Heightmap& stock, double x0, double y0,
                                       double x1, double y1) {
  double low = std::numeric_limits<double>::infinity();
  double high = -low;
  for (int r = 0; r < stock.rows(); ++r)
    for (int c = 0; c < stock.columns(); ++c) {
      if (stock.x(c) < x0 || stock.x(c) > x1 || stock.y(r) < y0 || stock.y(r) > y1) continue;
      const double top = stock.top(c, r);
      if (!std::isfinite(top)) continue;
      low = std::min(low, top);
      high = std::max(high, top);
    }
  return {low, high};
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
  op.depth_to = 6;
  op.stepdown = 3;
  op.finishing_subdivisions = 3;
  op.stepover_ratio = 0.2;
  op.cleanup_stepover_limit = 0.5;
  op.path_pitch = 0.5;
  cam::Job job;
  job.machine_id = machine.id;
  job.material_id = "plastic";
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
    for (std::size_t i = 0; i < depths.size(); ++i) {
      const auto& level = rough.levels[i];
      require_near(top - level.floor_z, depths[i], 1e-8, "layer execution depth");
      require_near(level.axial_depth, axial[i], 1e-8, "remaining terrace axial bound");
      require_near(level.radial_width, widths[i] * tool.diameter, 1e-8,
                   "cleanup radial width respects axial ratio and ceiling");
      if (level.finishing_index != 0) {
        require(Clipper2Lib::Difference(level.incoming_stock, level.stock_slice,
                                        Clipper2Lib::FillRule::NonZero).empty() &&
                    wave::material_area(level.incoming_stock) + 0.01 <
                        wave::material_area(level.stock_slice),
                "each upward cut receives the stock the cuts before it left");
        const auto& coarse = rough.levels[level.coarse_level];
        require_near(level.floor_z + level.axial_depth,
                     coarse.floor_z + coarse.axial_depth, 1e-8,
                     "cleanup never extends above its coarse terrace");
      }
    }
    require(rough.stats.unsafe_plunges == 0, "upward planning plunges onto no stock");
    require(cam::post_job(result, job, tools, machines).ok(),
            "the combined downward/upward operation posts");

    require(rough.stock_before && rough.stock_after,"A roughing operation retains XYZ stock snapshots");
    if (!rough.stock_before || !rough.stock_after) return;
    double before=rough.stock_before->volume();
    require_near(before,48.0*48.0*12.0,1e-3,"Preview begins with the complete blank");
    for (const auto& level:rough.levels) {
      require(!level.stock_checkpoints.empty(),"Each level retains playback checkpoints");
      if(level.stock_checkpoints.empty()) continue;
      const auto after=level.stock_checkpoints.back().stock->volume();
      double removed=0;
      for(const auto& phase:level.feeds.motion) removed+=phase.removed_volume;
      require_near(before-after,removed,1e-3,"XYZ playback removes the executed cutter volume");
      before=after;
    }
    require_near(rough.stock_before->volume()-before,rough.stats.removed_volume,1e-3,
                 "Operation removal agrees with its heightmap history");
    for(double z:{1.5,3.5,6.5,10.5}) {
      const auto expected=cam::stock_after_operations_at_z(
          geometry.stock_outline(z,op.slice_grid),z,{&rough});
      const auto actual=Clipper2Lib::Intersect(geometry.stock_outline(z,op.slice_grid),
          rough.stock_after->slice(z),Clipper2Lib::FillRule::NonZero);
      require(wave::paths_equal_canonical(actual,expected),
              "Downstream stock queries use the final XYZ snapshot");
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
  if (flat.ok()) {
    const auto [low, high] = face_heights(*flat.operations.front().stock_after, 26, 10, 34, 30);
    require(low >= 6.25 - 1e-5, "the flat's axial allowance is preserved");
    require(high <= 6.25 + 1e-5,
            "a depth break clears the horizontal face exactly to its allowance");
  }
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
  const wave::MillingCutter cutter{wave::CutterShape::Flat, 1.0, 10.0};
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
    return plan;
  };
  // A blank from Z -10 to the floor everywhere, with `walls` standing up to
  // Z 0 where they are given.
  const auto stock = [](const wave::Paths64& walls, double wall_low = -3, double wall_high = 0) {
    wave::Heightmap map(-20, -20, 50, 40, 0.05);
    map.fill({box(-20, -20, 30, 20)}, -10, -4);
    if (!walls.empty()) map.fill(walls, wall_low, wall_high);
    return map;
  };
  const auto link = [&](cam::OperationPlan& plan, const wave::Heightmap& map) {
    const bool linked = cam::detail::link_layers(plan.levels[0], plan.levels[1], map, cutter,
                                                 2000.0, 6000.0, 10.0, 25.0, {});
    const cam::MotionPhase* previous = nullptr;
    for (const auto& level : plan.levels) for (const auto& p : level.feeds.motion) {
      if (previous) {
        require(previous->to == p.from, "linked motion is continuous in XY");
        require_near(previous->to_z, p.from_z, 1e-9, "linked motion is continuous in Z");
      }
      previous = &p;
    }
    return linked;
  };
  // Walls around a channel the first layer has cleared to its floor.
  const auto channel_walls = [](double x0, double y0, double x1, double y1) {
    return Clipper2Lib::Difference(wave::Paths64{box(-5, -5, 15, 5)},
                                   wave::Paths64{box(x0, y0, x1, y1)},
                                   Clipper2Lib::FillRule::NonZero);
  };
  auto rising = make_plan(-3, -2);
  require(link(rising, stock(channel_walls(-2, -2, 13, 2))) &&
              rising.levels[0].stays_down_after && rising.levels[1].feeds.begins_at_depth,
          "cleared stock permits upward linking");
  require(rising.levels[1].feeds.motion.front().from_z == -3 &&
              rising.levels[1].feeds.motion.front().to_z == -2 &&
              rising.levels[1].feeds.motion.front().inter_layer,
          "upward cleanup rises before traversing");
  {
    auto tool = test_tool(); tool.diameter = 2;
    cam::Operation operation;
    operation.tool_id = tool.id;
    cam::Job job;
    job.machine_id = test_machine().id;
    job.material_id = "plastic";
    job.operations = {operation};
    cam::JobPlan posted_plan; posted_plan.rapid_plane = 10;
    rising.tool_id = tool.id; rising.planned = true;
    rising.spindle_rpm = 12000; posted_plan.operations = {rising};
    const auto posted = cam::post_job(posted_plan, job, cam::ToolLibrary{{tool}},
                                      cam::MachineLibrary{{test_machine()}});
    require(posted.ok(), "stock-validated XYZ links post: " + posted.error);
    const auto next = posted.text.find("Depth -2");
    const auto up = posted.text.find("Z-2", next);
    const auto across = posted.text.find("X10", next);
    const auto clearance = posted.text.find("Z10", next);
    require(next != std::string::npos && up < across && across < clearance,
            "posted cleanup rises and traverses before its final clearance retract");
    require(posted.text.find("X-1") < posted.text.find("Z-3"),
            "the first roughing entry establishes XY before descending");
  }
  auto uncleared = make_plan(-3, -2);
  require(!link(uncleared, stock({box(-5, -5, 15, 5)})),
          "uncut stock above the floor blocks the link");
  auto falling = make_plan(-2, -3);
  require(link(falling, stock({})) && falling.levels[0].stays_down_after,
          "a clear descent can stay down");
  require(falling.levels[1].feeds.motion.front().from_z == -2 &&
              falling.levels[1].feeds.motion.front().to_z == -2,
          "descending links traverse before descending");
  auto blocked = make_plan(-2, -3);
  require(!link(blocked, stock({box(9, -2, 12, 2)}, -3, -2.5)),
          "stock below the traversal blocks an unsafe vertical descent");
  auto roof = make_plan(-3, -2);
  require(!link(roof, stock({box(-2, -2, 2, 2)}, 1, 2)),
          "overhead stock blocks a cutter column that looks clear at the tip");
  auto detour = make_plan(-3, -2);
  require(link(detour, stock({box(4, -1, 6, 1)}, -4, 0)),
          "the router finds a safe detour around an intervening island");
  auto wide = make_plan(-3, -2);
  require(!link(wide, stock({box(4, -18, 6, 18)}, -4, 0)),
          "a detour slower than lifting is not taken");
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
    geometry.set_box_stock(true, cam::BoxStockMargins{4, 4, 4, 4, 0, 0});
    cam::Operation operation;
    operation.tool_id = tool.id;
    operation.stepover_ratio = 0.4;
    operation.stepdown = 1.0;
    operation.depth_from = 0.75;
    operation.depth_to = 6.0;
    operation.radial_stock_to_leave = 0.0;
    operation.axial_stock_to_leave = allowance;
    cam::Job job;
    job.machine_id = machine.id;
    job.material_id = "plastic";
    job.operations = {operation};
    const auto plan =
        cam::plan_job(geometry, job, tools, machines, materials, {});
    require(plan.ok(), "stepped part plans: " + plan.error);
    if (!plan.ok() || plan.operations.empty())
      return std::pair<bool, double>{false, 0.0};

    // Well inside the low half, so a pass that reached the face is unmistakable.
    const auto& after = plan.operations.front().stock_after;
    if (!after) return std::pair<bool, double>{false, 0.0};
    return std::pair<bool, double>{true, face_heights(*after, 24, 8, 36, 32).first};
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

  geometry.set_box_stock(true, cam::BoxStockMargins{4, 4, 4, 4, 0, 0});
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
  geometry.set_box_stock(true, cam::BoxStockMargins{wider, wider, wider, wider, 0, 0});
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
  geometry.set_box_stock(true, cam::BoxStockMargins{1, 1, 1, 1, 0, 0});
  cam::Operation operation;
  operation.tool_id = tool.id;
  operation.stepover_ratio = 0.4;
  operation.stepdown = 2;
  operation.depth_to = 3;
  operation.finishing_subdivisions = 4;
  cam::Job job;
  job.machine_id = machine.id;
  job.material_id = "plastic";
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
  const cam::Machine* machine = machines.machines.empty() ? nullptr : &machines.machines.front();
  require(machine && materials.find(material_id) && tool->cutting_for(material_id),
          "the shipped data has a machine and aluminium cutting data");
  if (!machine || !materials.find(material_id)) return;

  cam::JobGeometry geometry;
  geometry.load_part(part_mesh);
  geometry.set_box_stock(true, cam::BoxStockMargins{4, 4, 4, 4, 0, 0});
  double low = 0.0;
  double high = 0.0;
  require(geometry.work_range(low, high),
          "the multi-operation fixture spans a depth range");
  if (!(high > low)) return;

  cam::Operation rough;
  rough.name = "Rough first";
  rough.tool_id = tool->id;
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
  job.machine_id = machine->id;
  job.material_id = material_id;
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
      }
      previous_cut = &level;
    }
    // Earlier groups can clear every remaining terrace. The dedicated
    // roughing-entry regression guarantees an arrival is exercised without
    // depending on simplification leaving remnants in this model.

    require(subdivided.stock_before && subdivided.stock_after,
            "subdivision playback uses the simulated stock");
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
  // Exterior contact exposes tiny boundary remnants in the sampled inherited
  // stock. An identical rougher may skim those, but must not take bulk stock
  // or spend its clearance allowance on the part.
  const auto& repeated=plan.operations[1].stats;
  const double rounding_volume=wave::to_world(2)*rough.depth_to*repeated.cutting_distance;
  require(repeated.removed_volume<=rounding_volume+1e-9,
          "an identical second rougher removes only geometry-rounding remnants");
  require(plan.operations[0].stats.gouge_area<1e-6 && repeated.gouge_area<1e-6,
          "initial and repeated exterior roughing preserve part clearance");

  bool inherited_less_stock = false;
  bool compared = false;
  for (const cam::LevelPlan& level : plan.operations[2].levels) {
    if (level.nothing_to_cut || level.incoming_stock.empty()) continue;
    compared = true;
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

double level_removed(const cam::LevelPlan& level) {
  double removed = 0.0;
  for (const auto& phase : level.feeds.motion) removed += phase.removed_volume;
  return removed;
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
  geometry.set_box_stock(true, cam::BoxStockMargins{5, 5, 5, 5, 0, 0});
  double low = 0.0;
  double high = 0.0;
  require(geometry.work_range(low, high),
          "the LsJig cleanup fixture spans a depth range");
  if (!(high > low)) return;

  cam::Operation operation;
  operation.tool_id = tool->id;
  operation.depth_to = high - low;
  operation.finishing_subdivisions = 4;
  operation.cleanup_tolerance = tool->diameter * 0.5;
  cam::Job job;
  job.machine_id = machine->id;
  job.material_id = material->id;
  job.box_stock = true;
  job.box_stock_margins = cam::BoxStockMargins{5, 5, 5, 5, 0, 0};
  job.operations = {operation};

  const cam::JobPlan strict_plan =
      cam::plan_job(geometry, job, tools, machines, materials, {});
  require(strict_plan.ok(),
          "the strict LsJig subdivision fixture plans: " + strict_plan.error);
  if (!strict_plan.ok() || strict_plan.operations.empty()) return;
  std::size_t strict_cutting_layers = 0;
  for (const cam::LevelPlan& level : strict_plan.operations.front().levels)
    if (!level.nothing_to_cut && level_removed(level) > 0.0)
      ++strict_cutting_layers;
  require(strict_plan.operations.front().levels.size() == 68 &&
              strict_cutting_layers > 0,
          "strict cleanup retains all 68 scheduled layers and cuts stock");
  const cam::OperationPlan& strict_rough = strict_plan.operations.front();
  std::size_t strict_later_reached = 0;
  for (std::size_t subdivision = 1;
       subdivision < strict_rough.subdivisions.size(); ++subdivision) {
    const cam::OperationSubdivision& range =
        strict_rough.subdivisions[subdivision];
    for (std::size_t local = 0; local < range.level_count; ++local) {
      const cam::LevelPlan& level =
          strict_rough.levels[range.first_level + local];
      if (!level.wave.reached_generations.empty()) ++strict_later_reached;
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
      if (!level.nothing_to_cut && level_removed(level) > 0.0)
        ++cutting_layers;
    }
  }
  require(later_layers == 51,
          "the LsJig fixture has 51 layers after its first subdivision");
  require(reached_layers > strict_later_reached,
          "exterior cutter reach seeds more later LsJig roughing layers");
  require(cutting_layers > 0,
          "upward layers with completed roughing still remove stock with exterior reach");
  require(rough.stats.unsafe_plunges == 0 &&
              rough.stats.lifts_over_subject == 0 &&
              rough.stats.unreachable_fronts == 0,
          "exterior-domain roughing introduces no unsafe or unresolved motion");
  require_near(rough.stats.gouge_area, strict_rough.stats.gouge_area, 1e-6,
               "exterior-domain roughing introduces no additional gouge area");
}

// Z associations preserve the wavefront topology between adjacent layers.
}  // namespace cam_test
