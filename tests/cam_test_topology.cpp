#include "cam_test_support.h"
namespace cam_test {
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

  wave::FrontLineage lower_near = single_front_lineage(
      {point(-0.025, 5.1), point(9.0, 5.1)},
      box(-0.025, 4.2, 9.0, 5.8));
  auto tolerant = wave::build_inter_layer_topology(
      {&upper, &lower_near}, 0.2);
  require(tolerant.node_chains.size() == 1,
          "a 0.025 mm area discrepancy retains its node Z association");
  require(tolerant.entry_front_chains.size() == 1,
          "entry-front matching remains independently tolerant");

  wave::FrontLineage lower_far = single_front_lineage(
      {point(-0.075, 5.1), point(9.0, 5.1)},
      box(-0.075, 4.2, 9.0, 5.8));
  auto outside_tolerance = wave::build_inter_layer_topology(
      {&upper, &lower_far}, 0.2);
  require(outside_tolerance.node_chains.empty(),
          "a discrepancy over 0.05 mm still breaks the node Z association");
  require(outside_tolerance.entry_front_chains.size() == 1,
          "the entry-front envelope remains independent of node tolerance");

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

  const wave::Paths64 edge_stock{box(0, 0, 11, 10)};
  const wave::Paths64 strict_edge_access =
      cam::stock_exterior_tolerance(edge_stock, 2.0);
  require(cam::derive_offset_contours(
              wave::Paths64{box(0, 0, 10, 10)}, 3.2,
              &strict_edge_access)
              .empty(),
          "cleanup outside the selected stock tolerance is rejected");
  const wave::Paths64 edge_access =
      cam::stock_exterior_tolerance(edge_stock, 3.0);
  const auto edge_cleanup = cam::derive_offset_contours(
      wave::Paths64{box(0, 0, 10, 10)}, 3.2, &edge_access);
  require(!edge_cleanup.empty(),
          "an air-side cutter centre may clean stock with its cutting edge");
  for (const auto& contour : edge_cleanup)
    for (const wave::Point64& centre : contour.path)
      require(!inside(centre, edge_stock),
              "air-side cleanup never places its cutter centre in stock");
  const wave::Paths64 unopened_stock{box(-10, -10, 20, 20)};
  const wave::Paths64 unopened_access =
      cam::stock_exterior_tolerance(unopened_stock, 3.0);
  require(cam::derive_offset_contours(
              wave::Paths64{box(0, 0, 10, 10)}, 3.2, &unopened_access)
              .empty(),
          "air-side eligibility does not authorize a contour buried in stock");

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
  require_near(cam::Operation{}.path_pitch, 1.0, 1e-9,
               "new operations default to a 1 mm simulation pitch");
  require(cam::Operation{}.finishing_subdivisions == 1,
          "roughing defaults to one finishing subdivision");
  require_near(cam::Operation{}.cleanup_tolerance, 0.15, 1e-9,
               "roughing defaults to a small cleanup tolerance");
  const cam::Tool tool = test_tool();
  const cam::Tool tool_back =
      cam::tool_from_json_text(cam::to_json_text(tool));
  require(tool_back.id == tool.id, "tool identity survives a round trip");
  require(tool_back.flutes == tool.flutes && tool_back.type == tool.type,
          "flute count and tool type survive a round trip");

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
  operation.simulation_method = wave::SimulationMethod::Compare;
  operation.finishing_subdivisions = 4;
  operation.depth_breaks = {1.25, 3.5};
  operation.adaptive_cleanup_stepover = false;
  operation.cleanup_stepover_limit = 0.65;
  operation.cleanup_tolerance = 0.2;
  operation.mode = cam::OperationMode::ContourFinishing;
  operation.entry_front_match_tolerance = 0.12;
  operation.contour_direction = cam::ContourDirectionPolicy::Climb;
  operation.manual_feed_range = true;
  operation.full_engagement_feed = 450.0;
  operation.no_engagement_feed = 1900.0;
  job.operations.push_back(operation);
  job.rapid_clearance_height = 12.0;
  const cam::Job job_back =
      cam::job_from_json_text(cam::to_json_text(job));
  const std::string job_text = cam::to_json_text(job);
  require(job_text.find("levels") == std::string::npos &&
              job_text.find("triangles") == std::string::npos,
          "job files contain reconstruction data, not models or plan results");
  require(job_back.operations.size() == 1, "job operations survive");
  if (!job_back.operations.empty()) {
    require(job_back.operations[0].simulation_method == wave::SimulationMethod::Compare,
            "engagement method survives serialization");
    require_near(job_back.operations[0].path_pitch, 0.15, 1e-9,
                 "path pitch survives");
    require(job_back.operations[0].finishing_subdivisions == 4,
            "finishing subdivisions survive");
    require(job_back.operations[0].depth_breaks == operation.depth_breaks &&
                !job_back.operations[0].adaptive_cleanup_stepover,
            "depth breaks and cleanup adaptation survive");
    require_near(job_back.operations[0].cleanup_stepover_limit, 0.65, 1e-9,
                 "cleanup width ceiling survives");
    require_near(job_back.operations[0].cleanup_tolerance, 0.2, 1e-9,
                 "cleanup tolerance survives");
    require(job_back.operations[0].mode == cam::OperationMode::ContourFinishing,
            "operation mode survives");
    require_near(job_back.operations[0].entry_front_match_tolerance, 0.12, 1e-9,
                 "entry-front tolerance survives");
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

}  // namespace cam_test
