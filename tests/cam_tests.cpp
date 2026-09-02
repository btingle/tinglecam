#include "cam/config.h"
#include "cam/feeds.h"
#include "cam/job.h"
#include "cam/post.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {

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
  cam::CuttingData data;
  data.material_id = "plastic";
  data.spindle_rpm = 12000.0;
  data.max_ae_over_d = 1.0;
  data.max_ap_over_d = 2.0;
  data.plunge_feed = 120.0;
  tool.cutting_data.push_back(data);
  return tool;
}

wave::Point64 point(double x, double y = 0.0) {
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

struct SyntheticCut {
  wave::Toolpath toolpath;
  wave::SimulationResult simulation;
};

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
  operation.axial_stock_to_leave = 0.5;
  require_near(cam::level_floors(operation, operation.stepdown, 0.0).back(), -4.5,
               1e-9, "axial stock allowance raises the final floor");
  // Depths are below the top of the work, so the same operation follows it.
  require_near(cam::level_floors(operation, operation.stepdown, 12.0).back(),
               7.5, 1e-9, "and the range hangs off wherever the work tops out");
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

// The entry fronts are found without planning anything, and finding them does
// not depend on which of them have been declined. That invariant is what lets a
// choice be made and unmade for the price of a click.
void check_entry_discovery(const std::filesystem::path& data,
                           const std::filesystem::path& part_mesh) {
  std::cout << "entry discovery\n";
  const auto materials = cam::load_materials(data / "materials");
  const auto machines = cam::load_machines(data / "machines");
  const auto tools = cam::load_tools(data / "starter-tools.json");
  const cam::Tool* tool = tools.tools.empty() ? nullptr : &tools.tools.front();
  require(tool != nullptr, "a tool to cut with");
  if (!tool || tool->cutting_data.empty()) return;
  const std::string material_id = tool->cutting_data.front().material_id;
  const cam::Machine* machine = nullptr;
  for (const auto& candidate : machines.machines)
    if (candidate.feeds_for(material_id)) machine = &candidate;
  const cam::Material* material = materials.find(material_id);
  require(machine != nullptr && material != nullptr,
          "a machine and a material to cut it in");
  if (!machine || !material) return;

  cam::JobGeometry geometry;
  geometry.load_part(part_mesh);
  geometry.set_box_stock(true, 4.0);
  double low = 0.0;
  double high = 0.0;
  require(geometry.work_range(low, high), "the part has a depth range");

  cam::Operation operation;
  operation.tool_id = tool->id;
  operation.machine_id = machine->id;
  operation.material_id = material_id;
  operation.depth_from = 0.0;
  // Coarse on purpose: this is about which fronts are found, not about how
  // finely the part is cut, and a few thick passes say it just as well.
  operation.depth_to = (high - low) * 0.5;
  operation.stepdown = std::max((high - low) * 0.25, 0.5);

  const cam::EntryDiscovery found =
      cam::discover_entries(geometry, operation, *tool, *machine, *material);
  require(found.ok(), "discovery succeeds: " + found.error);
  require(!found.entries.empty(), "a blank around a part has somewhere to start");
  if (found.entries.empty()) return;
  for (const cam::EntryOption& entry : found.entries) {
    require(!entry.curves.empty(), "an entry on offer is somewhere in particular");
    require(entry.anchors.size() == entry.curves.size(),
            "every appearance has the anchor that names it");
    require(!entry.muted, "nothing is declined until it is declined");
  }

  // Decline one, and ask again. The same fronts have to come back in the same
  // order: a seed piece that is dropped takes nothing with it.
  operation.muted_entries.push_back(found.entries.front().anchors);
  const cam::EntryDiscovery again =
      cam::discover_entries(geometry, operation, *tool, *machine, *material);
  require(again.ok(), "discovery succeeds with an entry declined");
  require(again.entries.size() == found.entries.size(),
          "declining an entry does not remove it from the list");
  if (again.entries.size() != found.entries.size()) return;
  for (std::size_t i = 0; i < again.entries.size(); ++i) {
    require(again.entries[i].anchors == found.entries[i].anchors,
            "and moves none of the others");
    require(again.entries[i].muted == (i == 0),
            "the declined entry, and only it, comes back marked");
  }
}

// Declining the entry that is the only way into a depth leaves that depth with
// nothing to cut -- and nothing may be emitted there. The boundary cleanup
// contours are derived from the part rather than from the wave, so before they
// were clipped to what the wave reached they were still emitted at those
// depths: a finishing pass around a pocket through solid stock that nothing had
// roughed.
void check_declined_entry(const std::filesystem::path& data,
                          const std::filesystem::path& objs) {
  std::cout << "declined entry\n";
  const auto materials = cam::load_materials(data / "materials");
  const auto machines = cam::load_machines(data / "machines");
  const auto tools = cam::load_tools(data / "starter-tools.json");
  const cam::Tool* tool = tools.tools.empty() ? nullptr : &tools.tools.front();
  if (!tool || tool->cutting_data.empty()) return;
  const std::string material_id = tool->cutting_data.front().material_id;
  const cam::Machine* machine = nullptr;
  for (const auto& candidate : machines.machines)
    if (candidate.feeds_for(material_id)) machine = &candidate;
  const cam::Material* material = materials.find(material_id);
  if (!machine || !material) return;

  cam::JobGeometry geometry;
  geometry.load_part(objs / "final_obj" / "Rs.obj");
  geometry.load_stock(objs / "stock_obj" / "Rs.obj");
  double low = 0.0;
  double high = 0.0;
  require(geometry.work_range(low, high), "the part and its blank span a depth");

  cam::Operation operation;
  operation.tool_id = tool->id;
  operation.machine_id = machine->id;
  operation.material_id = material_id;
  operation.depth_from = 0.0;
  operation.depth_to = high - low;
  operation.stepdown = std::max((high - low) * 0.2, 0.5);

  const cam::EntryDiscovery found =
      cam::discover_entries(geometry, operation, *tool, *machine, *material);
  require(found.ok() && found.entries.size() >= 2,
          "this blank offers more than one way in");
  if (!found.ok() || found.entries.size() < 2) return;

  // Decline the one that reaches deepest, so the depths only it opens are left
  // with no entry at all rather than merely a longer way round.
  const cam::EntryOption* deepest = &found.entries.front();
  for (const cam::EntryOption& entry : found.entries)
    if (entry.bottom_z < deepest->bottom_z) deepest = &entry;
  operation.muted_entries.push_back(deepest->anchors);

  cam::Job job;
  job.operations.push_back(operation);
  const cam::JobPlan plan =
      cam::plan_job(geometry, job, tools, machines, materials, {});
  require(plan.ok(), "the job plans with an entry declined: " + plan.error);
  if (!plan.ok() || plan.operations.empty()) return;

  std::size_t starved = 0;
  std::size_t emitted = 0;
  std::size_t outside = 0;
  std::size_t checked = 0;
  for (const cam::LevelPlan& level : plan.operations.front().levels) {
    if (level.stock_slice.empty()) continue;
    if (level.wave.reached_generations.empty()) {
      ++starved;
      if (!level.toolpath.moves.empty()) ++emitted;
      continue;
    }
    const wave::Paths64 reached = Clipper2Lib::InflatePaths(
        level.wave.reached_generations.back(),
        wave::boundary_slack(level.wave.stepover), Clipper2Lib::JoinType::Miter,
        Clipper2Lib::EndType::Polygon);
    for (const wave::ToolpathMove& move : level.toolpath.moves) {
      if (move.kind != wave::MoveKind::Cut) continue;
      for (const wave::Point64& at : move.path) {
        ++checked;
        if (!inside(at, reached)) ++outside;
      }
    }
  }
  require(starved > 0,
          "declining the deepest entry leaves depths the wave cannot start at");
  require(emitted == 0,
          std::to_string(emitted) +
              " depth(s) with no entry still emit a cutting pass");
  require(checked > 0, "the depths that keep an entry are still cut");
  require(outside == 0,
          std::to_string(outside) + " of " + std::to_string(checked) +
              " cut points stand in stock the wave never reached");
}

void check_inter_layer_topology() {
  std::cout << "inter-layer topology\n";
  wave::FrontLineage upper = single_front_lineage(
      {point(0.0, 5.0), point(10.0, 5.0)}, box(0.0, 4.0, 10.0, 6.0));
  wave::FrontLineage lower_inside = single_front_lineage(
      {point(1.0, 5.1), point(9.0, 5.1)}, box(1.0, 4.2, 9.0, 5.8));
  auto joined = wave::build_inter_layer_topology(
      {&upper, &lower_inside}, 0.2);
  require(joined.node_chains.size() == 1,
          "an exactly contained area forms a Z chain");
  require(joined.entry_front_chains.size() == 1,
          "a whole entry curve inside the tolerance envelope forms a Z chain");

  wave::FrontLineage lower_outside = single_front_lineage(
      {point(-0.001, 5.1), point(9.0, 5.1)},
      box(-0.001, 4.2, 9.0, 5.8));
  auto exact = wave::build_inter_layer_topology(
      {&upper, &lower_outside}, 0.2);
  require(exact.node_chains.empty(),
          "area matching does not inflate the upper footprint by even one grid unit");
  require(exact.entry_front_chains.size() == 1,
          "entry-front matching remains independently tolerant");

  wave::FrontLineage split;
  split.stepover = 0.5;
  split.generations.resize(3);
  split.generations[0].bands = {wave::Paths64{box(0, 0, 10, 2)}};
  split.generations[1].bands = {wave::Paths64{box(0, 2, 10, 4)}};
  split.generations[2].bands = {wave::Paths64{box(0, 4, 5, 6)},
                                wave::Paths64{box(5, 4, 10, 6)}};
  wave::LeadingFront root;
  root.path = {point(0, 1), point(10, 1)};
  root.children = {0};
  split.generations[0].fronts = {root};
  wave::LeadingFront fork;
  fork.path = {point(0, 3), point(10, 3)};
  fork.parents = {0};
  fork.children = {0, 1};
  split.generations[1].fronts = {fork};
  wave::LeadingFront left;
  left.path = {point(0, 5), point(5, 5)};
  left.parents = {0};
  left.band = 0;
  wave::LeadingFront right;
  right.path = {point(5, 5), point(10, 5)};
  right.parents = {0};
  right.band = 1;
  split.generations[2].fronts = {left, right};
  const wave::LayerTopology nodes = wave::build_layer_topology(split);
  require(nodes.nodes.size() == 3,
          "a split front stays upstream while each outgoing branch starts a node");
  require(nodes.front_nodes[0][0] == nodes.front_nodes[1][0],
          "the split front belongs to its singular parent's node");
  require(nodes.front_nodes[1][0] != nodes.front_nodes[2][0] &&
              nodes.front_nodes[2][0] != nodes.front_nodes[2][1],
          "the split front does not collapse its outgoing branches together");
  const std::size_t upstream_node = nodes.front_nodes[0][0];
  const std::size_t terminal_node = nodes.front_nodes[2][0];
  require_near(area(nodes.nodes[upstream_node].footprint), 20.0, 1e-6,
               "a stable open node is bounded by its first and last fronts");
  require_near(area(nodes.nodes[terminal_node].footprint), 10.0, 1e-6,
               "a singleton terminal owns its full closed terminal band");

  wave::FrontLineage inconsistent;
  inconsistent.stepover = 2.0;
  inconsistent.boundary_components = {box(0, 0, 10, 10),
                                      box(20, 0, 30, 10)};
  inconsistent.generations.resize(2);
  wave::LeadingFront before;
  before.path = {point(0, 2), point(0, 8)};
  before.children = {0};
  before.boundary_contacts[0].valid = true;
  before.boundary_contacts[0].component = 0;
  wave::LeadingFront after = before;
  after.children.clear();
  after.parents = {0};
  after.boundary_contacts[0].component = 1;
  inconsistent.generations[0].fronts = {before};
  inconsistent.generations[1].fronts = {after};
  bool rejected = false;
  try {
    (void)wave::build_layer_topology(inconsistent);
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  require(rejected,
          "a known boundary-component change across 1-to-1 lineage is an error");

  wave::FrontLineage detaching;
  detaching.stepover = 2.0;
  detaching.boundary_components = {box(0, 0, 10, 4)};
  detaching.generations.resize(2);
  detaching.generations[0].bands = {wave::Paths64{box(0, 0, 10, 2)}};
  detaching.generations[1].bands = {wave::Paths64{box(0, 2, 10, 4)}};
  wave::LeadingFront attached;
  attached.path = {point(0, 1), point(10, 1)};
  attached.children = {0};
  attached.boundary_contacts[0] = {true, 0, point(0, 1)};
  attached.boundary_contacts[1] = {true, 0, point(10, 1)};
  wave::LeadingFront detached;
  detached.path = {point(1, 3), point(9, 3)};
  detached.parents = {0};
  detaching.generations[0].fronts = {attached};
  detaching.generations[1].fronts = {detached};
  const wave::LayerTopology detached_node =
      wave::build_layer_topology(detaching);
  require(detached_node.nodes.size() == 1 &&
              !detached_node.nodes.front().footprint.empty(),
          "a validated boundary detachment remains one closed node");

  wave::FrontLineage disconnected = detaching;
  disconnected.generations[0].fronts[0].closed = true;
  disconnected.generations[0].fronts[0].path = box(0, 0, 2, 2);
  disconnected.generations[1].bands = {
      wave::Paths64{box(20, 20, 22, 22)}};
  bool rejected_open_boundary = false;
  try {
    (void)wave::build_layer_topology(disconnected);
  } catch (const std::runtime_error&) {
    rejected_open_boundary = true;
  }
  require(rejected_open_boundary,
          "an attaching node whose swept boundary is disconnected is an error");

  wave::FrontLineage terminal_cap;
  terminal_cap.stepover = 2.0;
  terminal_cap.boundary_components = {box(0, 0, 100, 100)};
  terminal_cap.generations.resize(1);
  wave::LeadingFront terminal_front;
  terminal_front.path = {point(0, 40), point(40, 0)};
  terminal_front.boundary_contacts[0] = {true, 0, point(0, 40)};
  terminal_front.boundary_contacts[1] = {true, 0, point(40, 0)};
  terminal_cap.generations[0].fronts = {terminal_front};
  terminal_cap.generations[0].bands = {
      wave::Paths64{{point(0, 40), point(40, 0), point(0, 0)}}};
  const wave::LayerTopology capped =
      wave::build_layer_topology(terminal_cap);
  require_near(area(capped.nodes.front().footprint), 800.0, 1e-6,
               "a terminal cap chooses the local boundary arc, not its complement");
}

void check_general_traversal_and_contours() {
  std::cout << "general traversal and contours\n";
  const std::vector<cam::DerivedContour> contours{
      {{point(0), point(1)}, false},
      {{point(10), point(11)}, false},
  };
  wave::TraversalSettings settings;
  settings.allow_stay_down = false;
  settings.cutting_feed = 10.0;
  settings.rapid_feed = 100.0;
  settings.floor_z = -2.0;
  settings.rapid_plane = 10.0;
  const auto routed = wave::plan_traversal(
      cam::contour_traversal_tasks(contours,
                                   cam::ContourDirectionPolicy::Either),
      settings);
  require(routed.order.size() == 2, "both generic traversal tasks are ordered");
  require(routed.toolpath.stats.cut_count == 2,
          "each traversal task emits one cut");
  require(routed.toolpath.stats.retract_count == 2,
          "disconnected tasks use the safe rapid plane");
  require_near(routed.estimated_minutes, 0.65, 1e-9,
               "routing cost includes initial descent and full retract height");

  // Two contours a short hop apart with a wall between them, which is what a
  // pair clipped to the same stock looks like: they finish and start on that
  // wall a few millimetres from each other. Lifting the whole height of the job
  // to cross it is the wrong answer by an order of magnitude, and until the
  // connector could go round something it was the only answer available.
  {
    const std::vector<cam::DerivedContour> across{
        {{point(-8.0, -8.0), point(-1.0, 0.0)}, false},
        {{point(1.0, 0.0), point(8.0, -8.0)}, false},
    };
    // Everywhere but a wall standing between the two contours, open only over
    // the top of it. It has to reach past both contours: allowed to reverse
    // one, the ordering would otherwise simply connect the pair of ends that
    // can already see each other.
    const wave::Paths64 room = Clipper2Lib::Difference(
        wave::Paths64{box(-20, -20, 20, 20)},
        wave::Paths64{box(-0.5, -20, 0.5, 0.5)}, Clipper2Lib::FillRule::NonZero);
    wave::TraversalSettings blocked;
    blocked.allow_stay_down = true;
    blocked.stay_down_region = room;
    blocked.cutting_feed = 1000.0;
    blocked.rapid_feed = 1000.0;
    blocked.floor_z = -2.0;
    blocked.rapid_plane = 200.0;

    const auto lifted = wave::plan_traversal(
        cam::contour_traversal_tasks(across,
                                     cam::ContourDirectionPolicy::Either),
        blocked);
    require(lifted.toolpath.stats.retract_count == 2,
            "with no allowance to route within, a blocked connector lifts");

    blocked.stay_down_slack = 40.0;
    const auto joined = wave::plan_traversal(
        cam::contour_traversal_tasks(across,
                                     cam::ContourDirectionPolicy::Either),
        blocked);
    require(joined.toolpath.stats.link_count == 1,
            "given one, it goes round the wall instead");
    require(joined.toolpath.stats.retract_count == 1,
            "and the only retract left is the descent that opens the pass");
    for (const auto& move : joined.toolpath.moves)
      if (move.kind == wave::MoveKind::Link)
        require(move.path.size() > 2 && move.length < 12.0,
                "the way round is a route with corners in it, and a short one");
  }

  const auto offset = cam::derive_offset_contours(
      wave::Paths64{box(0, 0, 10, 10)}, 1.0, nullptr);
  require(offset.size() == 1 && offset.front().closed,
          "unclipped offset geometry remains a closed contour");
  const wave::Paths64 half_stock{box(-2, -2, 5, 12)};
  const auto clipped = cam::derive_offset_contours(
      wave::Paths64{box(0, 0, 10, 10)}, 1.0, &half_stock);
  require(!clipped.empty(), "stock clipping retains the reachable contour section");
  for (const auto& contour : clipped)
    require(!contour.closed, "a stock-clipped contour may be open");

  auto approached = cam::contour_traversal_tasks(
      offset, cam::ContourDirectionPolicy::Either, 4);
  const std::size_t contour_point_count =
      approached.empty() || approached.front().variants.empty()
          ? 0
          : approached.front().variants.front().path.size();
  const wave::Paths64 stock{box(-2, -2, 12, 12)};
  const wave::Paths64 protected_centres{box(0, 0, 10, 10)};
  cam::add_safe_stock_approaches(approached, stock, protected_centres, 1.0);
  require(!approached.empty() && !approached.front().variants.empty() &&
              approached.front().variants.front().path.size() ==
                  contour_point_count &&
              approached.front().variants.front().safe_approach.has_value(),
          "finishing keeps its contour separate from its safe approach");

  wave::TraversalTask first;
  first.variants.push_back(
      {{point(0), point(1)}, wave::Path64{point(5, 5), point(0)}});
  wave::TraversalTask second;
  second.variants.push_back(
      {{point(2), point(3)}, wave::Path64{point(5, 5), point(2)}});
  wave::TraversalSettings cleanup;
  cleanup.allow_stay_down = true;
  cleanup.prefer_stay_down_when_legal = true;
  cleanup.stay_down_region = {box(-2, -2, 4, 2)};
  cleanup.initial_position = point(-1);
  cleanup.cutting_feed = 1000.0;
  cleanup.rapid_feed = 2000.0;
  cleanup.floor_z = -1.0;
  cleanup.rapid_plane = 10.0;
  const auto direct_cleanup = wave::plan_traversal({first, second}, cleanup);
  require(direct_cleanup.toolpath.stats.retract_count == 0,
          "legal cleanup connections stay down instead of using safe approaches");
  for (const auto& move : direct_cleanup.toolpath.moves)
    for (const auto& p : move.path)
      require(p != point(5, 5),
              "a safe-entry landing is not a mandatory cleanup waypoint");

  wave::TraversalSettings disconnected = cleanup;
  disconnected.allow_stay_down = false;
  disconnected.initial_position.reset();
  const auto safe_entry = wave::plan_traversal({first}, disconnected);
  require(safe_entry.toolpath.moves.size() == 3 &&
              safe_entry.toolpath.moves[0].kind == wave::MoveKind::Retract &&
              safe_entry.toolpath.moves[0].path.front() == point(5, 5) &&
              safe_entry.toolpath.moves[1].path.front() == point(5, 5) &&
              safe_entry.toolpath.moves[1].path.back() == point(0),
          "a disconnected contour still enters through its safe approach");
}

void check_serialisation() {
  std::cout << "serialisation\n";
  const cam::Tool tool = test_tool();
  const cam::Tool tool_back =
      cam::tool_from_json_text(cam::to_json_text(tool));
  require(tool_back.id == tool.id, "tool identity survives a round trip");
  require(tool_back.cutting_data.size() == 1, "tool cutting data survives");
  if (!tool_back.cutting_data.empty())
    require_near(tool_back.cutting_data.front().spindle_rpm, 12000.0, 1e-9,
                 "tool spindle speed survives");

  const cam::Machine machine = test_machine();
  const cam::Machine machine_back =
      cam::machine_from_json_text(cam::to_json_text(machine));
  require(machine_back.controller == machine.controller,
          "machine controller survives");
  require(machine_back.feed_ranges.size() == 1,
          "machine feed ranges survive");

  cam::Job job;
  cam::Operation operation;
  operation.path_pitch = 0.15;
  operation.mode = cam::OperationMode::ContourFinishing;
  operation.entry_front_match_tolerance = 0.12;
  operation.contour_direction = cam::ContourDirectionPolicy::Climb;
  operation.manual_feed_range = true;
  operation.full_engagement_feed = 450.0;
  operation.no_engagement_feed = 1900.0;
  // Two declined entries, one of them spanning two depths. The grouping is the
  // point: it is the only record that a declined entry exists at all.
  operation.muted_entries = {{point(3.5, -2.25), point(3.5, -2.5)},
                             {point(-9.125, 4.0)}};
  job.operations.push_back(operation);
  job.rapid_clearance_height = 12.0;
  const cam::Job job_back =
      cam::job_from_json_text(cam::to_json_text(job));
  require(job_back.operations.size() == 1, "job operations survive");
  if (!job_back.operations.empty()) {
    require_near(job_back.operations[0].path_pitch, 0.15, 1e-9,
                 "path pitch survives");
    require(job_back.operations[0].mode == cam::OperationMode::ContourFinishing,
            "operation mode survives");
    require_near(job_back.operations[0].entry_front_match_tolerance, 0.12, 1e-9,
                 "entry-front tolerance survives");
    const auto& muted = job_back.operations[0].muted_entries;
    require(muted.size() == 2 && muted[0].size() == 2 && muted[1].size() == 1,
            "declined entry fronts survive with their grouping intact");
    if (muted.size() == 2 && muted[0].size() == 2 && muted[1].size() == 1) {
      require(muted[0][0] == point(3.5, -2.25) && muted[0][1] == point(3.5, -2.5),
              "declined anchors survive to the grid unit");
      require(muted[1][0] == point(-9.125, 4.0),
              "a negative anchor survives the millimetre round trip");
    }
    require(job_back.operations[0].contour_direction ==
                cam::ContourDirectionPolicy::Climb,
            "contour direction survives");
    require(job_back.operations[0].manual_feed_range,
            "manual feed selection survives");
    require_near(job_back.operations[0].no_engagement_feed, 1900.0, 1e-9,
                 "manual feed endpoint survives");
  }
  require_near(job_back.rapid_clearance_height, 12.0, 1e-9,
               "the single safe rapid height survives");

  const cam::Tool sparse = cam::tool_from_json_text(
      R"({"id":"x","name":"X","diameter":8.0,"unheard_of":42})");
  require_near(sparse.diameter, 8.0, 1e-9,
               "recognized fields load while unknown fields are ignored");
}

void check_schedule_timing() {
  std::cout << "schedule timing\n";
  wave::Toolpath toolpath;
  wave::ToolpathMove retract;
  retract.kind = wave::MoveKind::Retract;
  retract.path = {point(0.0), point(10.0)};
  retract.length = 10.0;
  toolpath.moves.push_back(retract);
  wave::ToolpathMove cut;
  cut.kind = wave::MoveKind::Cut;
  cut.path = {point(10.0), point(20.0)};
  cut.length = 10.0;
  toolpath.moves.push_back(cut);
  wave::SimulationResult simulation;
  simulation.moves.resize(2);
  wave::MaterialStep step;
  step.from = point(10.0);
  step.to = point(20.0);
  step.move_distance_end = 10.0;
  step.load = 1.0;
  simulation.moves[1].steps.push_back(step);
  const auto schedule = cam::plan_feeds(
      toolpath, simulation, test_machine(), 500.0, 2000.0, 12000.0, 120.0);
  require(schedule.segments.size() == 2, "rapid and cutting movement are scheduled");
  const double rapid_seconds = 10.0 / 6000.0 * 60.0;
  const double cut_seconds = 10.0 / 500.0 * 60.0;
  require_near(cam::feed_schedule_seconds(schedule),
               rapid_seconds + cut_seconds, 1e-9,
               "machine time uses the final scheduled rates");
  require_near(cam::feed_seconds_to(schedule, 10.0), rapid_seconds, 1e-9,
               "distance converts to time across a rapid");

  const auto vertical = cam::plan_feeds(
      toolpath, simulation, test_machine(), 500.0, 2000.0, 12000.0,
      120.0, -2.0, 10.0);
  const double descent_seconds = 12.0 / 6000.0 * 60.0;
  const double lift_seconds = 12.0 / 6000.0 * 60.0;
  require(vertical.motion.size() == 4,
          "machine motion includes traverse, plunge, cut, and final lift");
  require_near(cam::feed_schedule_seconds(vertical),
               rapid_seconds + descent_seconds + cut_seconds + lift_seconds,
               1e-9, "machine time includes vertical motion");
  require_near(cam::feed_motion_length(vertical), 44.0, 1e-9,
               "XYZ travel has a counter independent of planar path distance");
  require_near(vertical.stats.rapid_distance, 34.0, 1e-9,
               "travel totals include the traverse, plunge, and final lift");
  require_near(vertical.stats.rapid_minutes,
               rapid_seconds / 60.0 + descent_seconds / 60.0 +
                   lift_seconds / 60.0,
               1e-9, "travel time uses each vertical phase's commanded rate");
  require_near(cam::feed_seconds_to(vertical, 10.0),
               rapid_seconds + descent_seconds, 1e-9,
               "the cutting start is reached only after the rapid descent");

  const cam::MotionCursor halfway_down = cam::feed_motion_at_seconds(
      vertical, rapid_seconds + descent_seconds * 0.5);
  require_near(halfway_down.z, 4.0, 1e-9,
               "playback interpolates the cutter down to the pass");
  require_near(halfway_down.path_distance, 10.0, 1e-9,
               "planar simulation distance pauses during a plunge");
  require_near(halfway_down.phase_distance, 6.0, 1e-9,
               "the active vertical phase reports its travelled clearance");
  require_near(halfway_down.motion_distance, 16.0, 1e-9,
               "the XYZ counter includes rapid and vertical travel");
  require(halfway_down.rapid,
          "a simulation-cleared retract landing returns to depth at rapid");
  require_near(halfway_down.rate, 6000.0, 1e-9,
               "the playback cursor exposes the commanded rapid rate");

  const cam::MotionCursor halfway_up = cam::feed_motion_at_seconds(
      vertical,
      rapid_seconds + descent_seconds + cut_seconds + lift_seconds * 0.5);
  require_near(halfway_up.z, 4.0, 1e-9,
               "playback interpolates the final rapid lift");
  require_near(halfway_up.path_distance, 20.0, 1e-9,
               "planar simulation distance pauses during a retract");

  wave::Toolpath between_paths;
  wave::SimulationResult between_simulation;
  between_simulation.moves.resize(3);
  wave::ToolpathMove first_cut;
  first_cut.kind = wave::MoveKind::Cut;
  first_cut.path = {point(0.0), point(1.0)};
  first_cut.length = 1.0;
  between_paths.moves.push_back(first_cut);
  wave::MaterialStep first_step;
  first_step.from = point(0.0);
  first_step.to = point(1.0);
  first_step.move_distance_end = 1.0;
  between_simulation.moves[0].steps.push_back(first_step);
  wave::ToolpathMove lift_and_cross;
  lift_and_cross.kind = wave::MoveKind::Retract;
  lift_and_cross.path = {point(1.0), point(3.0)};
  lift_and_cross.length = 2.0;
  between_paths.moves.push_back(lift_and_cross);
  wave::ToolpathMove second_cut = first_cut;
  second_cut.path = {point(3.0), point(4.0)};
  between_paths.moves.push_back(second_cut);
  wave::MaterialStep second_step = first_step;
  second_step.from = point(3.0);
  second_step.to = point(4.0);
  between_simulation.moves[2].steps.push_back(second_step);
  const auto transition = cam::plan_feeds(
      between_paths, between_simulation, test_machine(), 500.0, 2000.0,
      12000.0, 120.0, -2.0, 10.0);
  require(transition.motion.size() == 7,
          "a disconnected transition has plunge, cut, lift, traverse, "
          "plunge, cut, and final lift phases");
  if (transition.motion.size() == 7) {
    require(transition.motion[2].rapid &&
                transition.motion[2].from_z == -2.0 &&
                transition.motion[2].to_z == 10.0,
            "an internal retract first rises vertically");
    require(transition.motion[3].rapid &&
                transition.motion[3].from_z == 10.0 &&
                transition.motion[3].to_z == 10.0,
            "the disconnected XY move stays on the clearance plane");
    require(transition.motion[4].rapid &&
                transition.motion[4].from_z == 10.0 &&
                transition.motion[4].to_z == -2.0,
            "a cleared cutter descends rapidly only after reaching the next path");
  }

  // A machine whose Z axis is slower than the others. Only the vertical phases
  // change: a lift is commanded as a rise, a traverse and a descent, never as
  // one diagonal, so each one is timed at the axis that actually moves it.
  {
    cam::Machine slow_z = test_machine();
    slow_z.rapid_feed_z = 1500.0;
    const auto slowed = cam::plan_feeds(between_paths, between_simulation,
                                        slow_z, 500.0, 2000.0, 12000.0, 120.0,
                                        -2.0, 10.0);
    require(slowed.motion.size() == transition.motion.size(),
            "a slower Z axis changes the timing, not the moves");
    double vertical = 0.0;
    double lateral = 0.0;
    for (const cam::MotionPhase& phase : slowed.motion) {
      if (!phase.rapid) continue;
      if (phase.from_z != phase.to_z) {
        vertical += phase.length;
        require_near(phase.rate, 1500.0, 1e-9,
                     "a vertical rapid runs at the Z rate");
      } else {
        lateral += phase.length;
        require_near(phase.rate, 6000.0, 1e-9,
                     "a lateral rapid still runs at the lateral rate");
      }
    }
    require(vertical > 0.0 && lateral > 0.0,
            "this pass has both a vertical and a lateral rapid to compare");
    // The vertical distance is unchanged and now takes four times as long.
    require_near(cam::feed_schedule_seconds(slowed) -
                     cam::feed_schedule_seconds(transition),
                 vertical / 1500.0 * 60.0 - vertical / 6000.0 * 60.0, 1e-9,
                 "the whole of the difference is the vertical, reprised at the "
                 "Z rate");
    require_near(slowed.stats.rapid_distance, transition.stats.rapid_distance,
                 1e-9, "and the cutter travels no further for it");
  }

  // A profile that names no Z rate is a machine whose Z keeps up, which is what
  // every profile said before there were two rates.
  {
    cam::Machine unstated = test_machine();
    unstated.rapid_feed_z = 0.0;
    const auto same = cam::plan_feeds(between_paths, between_simulation,
                                      unstated, 500.0, 2000.0, 12000.0, 120.0,
                                      -2.0, 10.0);
    require_near(cam::feed_schedule_seconds(same),
                 cam::feed_schedule_seconds(transition), 1e-9,
                 "an unstated Z rate is the lateral rate");
  }

  // The next pass down starts wherever its own geometry does, which is not
  // where the previous one left the cutter. That leg is the plan's to own.
  const auto arrived = cam::plan_feeds(
      between_paths, between_simulation, test_machine(), 500.0, 2000.0,
      12000.0, 120.0, -2.0, 10.0, cam::PassLink{point(0.0, 5.0), 10.0, false});
  require(arrived.motion.size() == transition.motion.size() + 1,
          "arriving from a previous pass adds one phase");
  if (!arrived.motion.empty()) {
    const cam::MotionPhase& opening = arrived.motion.front();
    require(opening.arrival && opening.rapid,
            "the pass opens with the rapid in from the previous one");
    require(opening.from == point(0.0, 5.0) &&
                opening.to == transition.motion.front().from,
            "the arrival runs from where the cutter was left to where the "
            "pass begins");
    require(opening.from_z == 10.0 && opening.to_z == 10.0,
            "the arrival stays on the clearance plane");
    require_near(opening.path_distance_begin, opening.path_distance_end, 1e-12,
                 "arriving covers no planar path distance");
    require_near(cam::feed_schedule_seconds(arrived) -
                     cam::feed_schedule_seconds(transition),
                 5.0 / 6000.0 * 60.0, 1e-9,
                 "the arrival is timed at the machine rapid");
    require_near(arrived.stats.rapid_distance - transition.stats.rapid_distance,
                 5.0, 1e-9, "the arrival counts as travel");
  }
  require(cam::feed_motion_at_seconds(arrived, 0.0).position == point(0.0, 5.0),
          "playback starts the pass where the previous one finished");

  const SyntheticCut unresolved = straight_cut({1.0});
  const auto unresolved_schedule = cam::plan_feeds(
      unresolved.toolpath, unresolved.simulation, test_machine(), 500.0,
      2000.0, 12000.0, 120.0, -2.0, 10.0);
  require(!unresolved_schedule.motion.empty() &&
              !unresolved_schedule.motion.front().rapid,
          "a cut without a verified cleared retract still uses a plunge");
  if (!unresolved_schedule.motion.empty())
    require_near(unresolved_schedule.motion.front().rate, 120.0, 1e-9,
                 "an unresolved entry retains the configured plunge feed");
}

void check_validation() {
  std::cout << "validation\n";
  require(cam::validate(test_tool()).empty(), "the test tool validates");
  cam::Tool bad = test_tool();
  bad.diameter = 0.0;
  require(!cam::validate(bad).empty(), "zero tool diameter is rejected");
  bad = test_tool();
  bad.cutting_data.push_back(bad.cutting_data.front());
  require(!cam::validate(bad).empty(), "duplicate tool material data is rejected");
  require(cam::validate(test_machine()).empty(), "the test machine validates");
}

void check_planning_error_details() {
  std::cout << "planning error details\n";
  cam::Job job;
  cam::Operation broken;
  broken.name = "Broken roughing";
  broken.tool_id = "missing-tool";
  job.operations.push_back(broken);

  const cam::JobPlan plan = cam::plan_job(
      cam::JobGeometry{}, job, cam::ToolLibrary{}, cam::MachineLibrary{},
      cam::MaterialLibrary{}, {});
  require(!plan.ok(), "an operation with an unknown tool must fail planning");
  require(plan.error.find("No operation could be planned") != std::string::npos,
          "the job-level planning failure keeps its summary");
  require(plan.error.find("Broken roughing") != std::string::npos,
          "the job-level failure names the operation");
  require(plan.error.find("Unknown tool: missing-tool") != std::string::npos,
          "the job-level failure includes the concrete cause");
}

void check_shipped_data(const std::filesystem::path& data) {
  std::cout << "shipped profiles\n";
  const auto materials = cam::load_materials(data / "materials");
  const auto machines = cam::load_machines(data / "machines");
  const auto tools = cam::load_tools(data / "starter-tools.json");
  require(materials.materials.size() >= 6, "material profiles load");
  require(machines.machines.size() >= 3, "machine profiles load");
  require(tools.tools.size() >= 4, "starter tools load");
  for (const auto& material : materials.materials)
    for (const auto& problem : cam::validate(material))
      require(false, material.id + ": " + problem);
  for (const auto& machine : machines.machines) {
    for (const auto& problem : cam::validate(machine))
      require(false, machine.id + ": " + problem);
    for (const auto& range : machine.feed_ranges)
      require(materials.find(range.material_id) != nullptr,
              machine.id + " names an unknown material feed range");
  }
  for (const auto& tool : tools.tools) {
    for (const auto& problem : cam::validate(tool))
      require(false, tool.id + ": " + problem);
    for (const auto& entry : tool.cutting_data)
      require(materials.find(entry.material_id) != nullptr,
              tool.id + " names an unknown material");
  }
  const cam::Machine* makera = machines.find("makera-z1");
  require(makera != nullptr, "the Makera profile is present");
  if (makera) {
    const cam::MaterialFeedRange* plastic = makera->feeds_for("hdpe");
    require(plastic != nullptr, "the Makera has a plastic feed range");
    if (plastic) {
      require_near(plastic->full_engagement, 500.0, 1e-9,
                   "Makera plastic full-engagement feed");
      require_near(plastic->no_engagement, 2000.0, 1e-9,
                   "Makera plastic no-engagement feed");
    }
  }
}

void check_post_output() {
  std::cout << "post output\n";
  cam::ToolLibrary tools;
  cam::Tool tool = test_tool();
  tool.tool_number = 3;
  tools.upsert(tool);
  cam::MachineLibrary machines;
  cam::Machine machine = test_machine();
  machines.upsert(machine);
  cam::Job job;
  cam::Operation operation;
  operation.name = "Rough";
  operation.tool_id = tool.id;
  operation.machine_id = machine.id;
  operation.material_id = "plastic";
  job.operations.push_back(operation);

  const SyntheticCut cut = straight_cut({0.5, 0.5});
  cam::LevelPlan level;
  level.floor_z = -1.5;
  level.axial_depth = 1.5;
  level.toolpath = cut.toolpath;
  level.simulation = cut.simulation;
  level.feeds = cam::plan_feeds(
      cut.toolpath, cut.simulation, machine, 500.0, 2000.0, 12000.0, 120.0);
  cam::OperationPlan planned_operation;
  planned_operation.operation_name = "Rough";
  planned_operation.tool_id = tool.id;
  planned_operation.spindle_rpm = 12000.0;
  planned_operation.levels.push_back(level);
  cam::JobPlan plan;
  plan.operations.push_back(planned_operation);

  const cam::PostResult result = cam::post_job(plan, job, tools, machines);
  require(result.ok(), "the post succeeds: " + result.error);
  require(result.text.find("M3 S12000") != std::string::npos,
          "the configured spindle speed is posted");
  require(result.text.find("F1250") != std::string::npos,
          "the engagement-derived feed is posted");
  require(result.text.find("F120") != std::string::npos,
          "a cut without a cleared landing is posted at plunge feed");
  require(result.text.find("M30") != std::string::npos,
          "the program terminates");

  wave::Toolpath safe_path;
  wave::ToolpathMove landing;
  landing.kind = wave::MoveKind::Retract;
  landing.path = {point(0.0), point(0.0)};
  safe_path.moves.push_back(landing);
  wave::ToolpathMove safe_cut;
  safe_cut.kind = wave::MoveKind::Cut;
  safe_cut.path = {point(0.0), point(1.0)};
  safe_cut.length = 1.0;
  safe_path.moves.push_back(safe_cut);
  wave::SimulationResult safe_simulation;
  safe_simulation.moves.resize(2);
  wave::MaterialStep safe_step;
  safe_step.from = point(0.0);
  safe_step.to = point(1.0);
  safe_step.move_distance_end = 1.0;
  safe_step.load = 1.0;
  safe_simulation.moves[1].steps.push_back(safe_step);
  level.toolpath = safe_path;
  level.simulation = safe_simulation;
  level.feeds = cam::plan_feeds(
      safe_path, safe_simulation, machine, 500.0, 2000.0, 12000.0,
      120.0, level.floor_z, job.rapid_clearance_height);
  plan.operations.front().levels.front() = level;
  const cam::PostResult rapid_entry =
      cam::post_job(plan, job, tools, machines);
  require(rapid_entry.ok(), "the safe-entry post succeeds: " + rapid_entry.error);
  require(rapid_entry.text.find("F120") == std::string::npos,
          "a simulation-cleared retract landing returns to depth with G0");
}

}  // namespace

int main(int argc, char** argv) {
  try {
    check_engagement_feeds();
    check_feed_compatible_merge();
    check_path_resampling();
    check_level_floors();
    check_boundary_rail_slack();
    check_entry_front_selection();
    check_inter_layer_topology();
    check_general_traversal_and_contours();
    check_serialisation();
    check_schedule_timing();
    check_validation();
    check_planning_error_details();
    check_post_output();
    if (argc > 1) check_shipped_data(argv[1]);
    if (argc > 2) check_box_stock(argv[2]);
    if (argc > 2) check_entry_discovery(argv[1], argv[2]);
    if (argc > 3) check_declined_entry(argv[1], argv[3]);
  } catch (const std::exception& error) {
    std::cerr << "Test threw: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  if (failures > 0) {
    std::cerr << failures << " check(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "All checks passed.\n";
  return EXIT_SUCCESS;
}
