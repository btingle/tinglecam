#include "cam_test_support.h"
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
    if (!plan.ok() || plan.operations.empty()) return 0.0;

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
    return lowest;
  };

  const double bare = lowest_pass_onto_the_face(0.0);
  require(bare > 6.0 && bare < 6.5,
          "with no allowance a pass comes down within a fraction of the face");
  const double left = lowest_pass_onto_the_face(0.5);
  require(left >= 6.5 - 1e-9,
          "an axial allowance holds every pass at least that far above the face");
  require(left > bare,
          "and the allowance is what moved it, not the depth range");
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

    if (base_count > 0) {
      const std::size_t active =
          subdivided.subdivisions[1].first_level;
      const std::size_t future =
          subdivided.subdivisions[3].first_level + base_count - 1;
      const std::size_t expected =
          subdivided.subdivisions[1].first_level + base_count - 1;
      require(cam::subdivision_preview_source(subdivided, active, future) ==
                  expected,
              "future preview layers alias the active subdivision at the same depth");
      const std::size_t completed =
          subdivided.subdivisions[0].first_level + base_count - 1;
      require(cam::subdivision_preview_source(subdivided, active, completed) ==
                  completed,
              "completed subdivision preview layers retain their own stock");
    }
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
              strict_cutting_layers == 45,
          "a cutter-radius cleanup tolerance retains the checkpoint's 45/68 "
          "layers");
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
  require(rough.subdivisions.size() == 4,
          "the LsJig cleanup fixture retains four subdivisions");
  if (rough.subdivisions.size() != 4) return;
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
