#include "analysis.h"
#include "geometry.h"
#include "mesh_slicer.h"
#include "toolpath.h"
#include "topology.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

// Two wave bands sweeping left to right across plain rectangular stock. When
// `notched` is set, the second band's leading edge carries a five-unit step
// standing in for the quantization staircase that Clipper's offset output
// leaves on a real wavefront. The step is far too small to separate two
// wavefronts, so both variants must report a single leading front.
wave::WaveResult two_band_sweep(bool notched) {
  wave::WaveResult wave_result;
  wave_result.stock = wave::Paths64{{{0, 0}, {60000, 0}, {60000, 20000}, {0, 20000}}};
  wave_result.stock_boundary = wave_result.stock.front();
  wave_result.stepover = 10.0;
  wave_result.softening = 0.0;
  const wave::Path64 first{{0, 0}, {10000, 0}, {10000, 20000}, {0, 20000}};
  wave::Path64 second{{10000, 0}, {20000, 0}, {20000, 10000}};
  if (notched) {
    second.push_back({19995, 10000});
    second.push_back({19995, 10001});
    second.push_back({20000, 10001});
  }
  second.push_back({20000, 20000});
  second.push_back({10000, 20000});
  wave_result.generations = {wave::Paths64{first}, wave::Paths64{second}};
  wave_result.reached_generations = {
      wave::Paths64{first},
      Clipper2Lib::Union(wave::Paths64{first, second},
                         Clipper2Lib::FillRule::NonZero)};
  return wave_result;
}

// One wave band sweeping left to right, its leading edge a single straight
// edge twenty units tall that stands against the part over the middle eight of
// them. Offsetting emits a wall like this as one edge however long it is, and
// the leading test takes one probe pair per edge, so the whole edge used to be
// judged by the one spot that is blocked and discarded entire. The generation
// behind it still advances through the stretches that were open, above the
// part and below it, and those fronts are then left with no parent -- which is
// what put a plunge into uncleared material at the head of a pass, since a
// parentless front is ready before anything has cleared a way in.
wave::WaveResult blocked_midpoint_sweep() {
  wave::WaveResult result;
  result.stock = wave::Paths64{{{0, 0}, {60000, 0}, {60000, 20000}, {0, 20000}}};
  result.stock_boundary = result.stock.front();
  result.subject = wave::Paths64{
      {{10000, 6000}, {20000, 6000}, {20000, 14000}, {10000, 14000}}};
  result.stepover = 10.0;
  result.softening = 0.0;
  const wave::Path64 first{{0, 0}, {10000, 0}, {10000, 20000}, {0, 20000}};
  const wave::Path64 lower{
      {10000, 0}, {20000, 0}, {20000, 6000}, {10000, 6000}};
  const wave::Path64 upper{
      {10000, 14000}, {20000, 14000}, {20000, 20000}, {10000, 20000}};
  result.generations = {wave::Paths64{first}, wave::Paths64{lower, upper}};
  result.reached_generations = {
      wave::Paths64{first},
      Clipper2Lib::Union(wave::Paths64{first, lower, upper},
                         Clipper2Lib::FillRule::NonZero)};
  return result;
}

// Two sources sweep a rectangular channel from opposite ends. Their last
// fronts consume the middle band together and leave no surviving front, which
// is a merge into an empty terminal node rather than two independent deaths.
wave::WaveResult opposing_terminal_sweep() {
  wave::WaveResult result;
  const wave::Path64 stock{
      {0, 0}, {20000, 0}, {20000, 10000}, {0, 10000}};
  const wave::Path64 left_seed{
      {0, 0}, {1000, 0}, {1000, 10000}, {0, 10000}};
  const wave::Path64 right_seed{
      {19000, 0}, {20000, 0}, {20000, 10000}, {19000, 10000}};
  const wave::Path64 left_step{
      {1000, 0}, {6000, 0}, {6000, 10000}, {1000, 10000}};
  const wave::Path64 right_step{
      {14000, 0}, {19000, 0}, {19000, 10000}, {14000, 10000}};
  const wave::Path64 terminal_band{
      {6000, 0}, {14000, 0}, {14000, 10000}, {6000, 10000}};
  result.stock = {stock};
  result.stock_boundary = stock;
  result.domain = {stock};
  result.stepover = 5.0;
  result.generations = {{left_seed, right_seed},
                        {left_step, right_step},
                        {terminal_band}};
  result.reached_generations = {
      {left_seed, right_seed},
      Clipper2Lib::Union({left_seed, right_seed, left_step, right_step},
                         Clipper2Lib::FillRule::NonZero),
      {stock},
  };
  return result;
}

// Two fronts also terminate on the same transition, but in disconnected
// regions. Coincident timing alone must not turn them into a merge.
wave::WaveResult independent_terminal_sweeps() {
  wave::WaveResult result;
  const wave::Path64 lower{
      {0, 0}, {10000, 0}, {10000, 5000}, {0, 5000}};
  const wave::Path64 upper{
      {0, 30000}, {10000, 30000}, {10000, 35000}, {0, 35000}};
  const wave::Path64 lower_seed{
      {0, 0}, {1000, 0}, {1000, 5000}, {0, 5000}};
  const wave::Path64 upper_seed{
      {0, 30000}, {1000, 30000}, {1000, 35000}, {0, 35000}};
  const wave::Path64 lower_finish{
      {1000, 0}, {10000, 0}, {10000, 5000}, {1000, 5000}};
  const wave::Path64 upper_finish{
      {1000, 30000}, {10000, 30000}, {10000, 35000}, {1000, 35000}};
  result.stock = {lower, upper};
  result.stock_boundary = lower;
  result.domain = result.stock;
  result.stepover = 10.0;
  result.generations = {{lower_seed, upper_seed},
                        {lower_finish, upper_finish}};
  result.reached_generations = {{lower_seed, upper_seed}, {lower, upper}};
  return result;
}

// Two disconnected cuts can each meet the air outside the stock within the
// planner's clearance tolerance. The finite box used to represent that
// unbounded air is not itself a machining contour and must not become a long
// at-depth detour between them.
std::pair<wave::WaveResult, wave::FrontLineage> outer_box_detour_case() {
  wave::WaveResult wave_result;
  const wave::Path64 left_stock{
      {0, 0}, {10000, 0}, {10000, 10000}, {0, 10000}};
  const wave::Path64 right_stock{
      {20000, 0}, {30000, 0}, {30000, 10000}, {20000, 10000}};
  const wave::Path64 left_band{{0, 0}, {1500, 0}, {1500, 10000}, {0, 10000}};
  const wave::Path64 right_band{
      {28500, 0}, {30000, 0}, {30000, 10000}, {28500, 10000}};
  wave_result.stock = {left_stock, right_stock};
  wave_result.stock_boundary = left_stock;
  wave_result.stepover = 1.5;
  wave_result.softening = 0.0;
  wave_result.generations = {wave::Paths64{left_band, right_band}};
  wave_result.reached_generations = wave_result.generations;

  wave::FrontLineage lineage;
  lineage.generations.push_back({
      {
          {{{1500, 1000}, {1500, 9000}}, false, 0, {}, {}},
          {{{28500, 1000}, {28500, 9000}}, false, 1, {}, {}},
      },
      {wave::Paths64{left_band}, wave::Paths64{right_band}},
  });
  return {std::move(wave_result), std::move(lineage)};
}

// The contour route around the right wall is valid but slightly longer than
// lifting to the cleared exterior and approaching the second cut from there.
// This distinguishes direct candidate comparison from the former multiplier.
std::pair<wave::WaveResult, wave::FrontLineage> shorter_rapid_case() {
  wave::WaveResult wave_result;
  const wave::Path64 stock{{0, 0}, {10000, 0}, {10000, 10000}, {0, 10000}};
  const wave::Path64 top_band{{0, 0}, {10000, 0}, {10000, 1500}, {0, 1500}};
  const wave::Path64 bottom_band{
      {0, 8500}, {10000, 8500}, {10000, 10000}, {0, 10000}};
  wave_result.stock = {stock};
  wave_result.stock_boundary = stock;
  wave_result.stepover = 1.5;
  wave_result.generations = {wave::Paths64{top_band, bottom_band}};
  wave_result.reached_generations = wave_result.generations;

  wave::FrontLineage lineage;
  lineage.generations.push_back({
      {
          {{{1000, 1500}, {9000, 1500}}, false, 0, {}, {}},
          {{{1000, 8500}, {9000, 8500}}, false, 1, {}, {}},
      },
      {wave::Paths64{top_band}, wave::Paths64{bottom_band}},
  });
  return {std::move(wave_result), std::move(lineage)};
}

// Total length of toolpath motion that runs inside the subject. The subject is
// eroded first because cuts legitimately hug its boundary; only a real
// incursion past that slack counts. Retracts are exempt: they happen above the
// work.
double gouge_length(const wave::Toolpath& toolpath, const wave::Paths64& subject,
                    double erosion) {
  const wave::Paths64 forbidden = Clipper2Lib::InflatePaths(
      subject, -erosion, Clipper2Lib::JoinType::Miter,
      Clipper2Lib::EndType::Polygon);
  if (forbidden.empty()) return 0.0;
  double total = 0.0;
  for (const auto& move : toolpath.moves) {
    // Lifts happen above the work, so only motion at depth can gouge.
    if (move.kind == wave::MoveKind::Retract || move.kind == wave::MoveKind::Entry ||
        move.path.size() < 2)
      continue;
    Clipper2Lib::Clipper64 clipper;
    clipper.AddOpenSubject(wave::Paths64{move.path});
    clipper.AddClip(forbidden);
    wave::Paths64 closed_solution;
    wave::Paths64 open_solution;
    if (!clipper.Execute(Clipper2Lib::ClipType::Intersection,
                         Clipper2Lib::FillRule::NonZero, closed_solution,
                         open_solution))
      continue;
    for (const auto& piece : open_solution)
      for (std::size_t i = 1; i < piece.size(); ++i)
        total += std::hypot(wave::to_world(piece[i].x - piece[i - 1].x),
                            wave::to_world(piece[i].y - piece[i - 1].y));
  }
  return total;
}

wave::Path64 test_box(double x0, double y0, double x1, double y1) {
  return wave::Path64{{wave::to_integer(x0), wave::to_integer(y0)},
                      {wave::to_integer(x1), wave::to_integer(y0)},
                      {wave::to_integer(x1), wave::to_integer(y1)},
                      {wave::to_integer(x0), wave::to_integer(y1)}};
}

bool region_covers(const wave::Paths64& region, double x, double y) {
  const wave::Paths64 probe{test_box(x - 0.05, y - 0.05, x + 0.05, y + 0.05)};
  return !Clipper2Lib::Intersect(probe, region, Clipper2Lib::FillRule::NonZero)
              .empty();
}

std::size_t front_count(const wave::FrontLineage& lineage) {
  std::size_t result = 0;
  for (const wave::FrontGeneration& generation : lineage.generations)
    result += generation.fronts.size();
  return result;
}

void check_committed_fronts(const wave::WavefrontAnalysis& analysis) {
  const auto committed = wave::committed_wavefront_stages(analysis);
  require(committed.size() == analysis.layers.size(),
          "every recursive decision should produce one committed stage");
  if (committed.size() != analysis.layers.size()) return;
  for (std::size_t layer = 0; layer < committed.size(); ++layer) {
    const auto& decision = analysis.layers[layer];
    const auto& stage = committed[layer];
    std::vector<const wave::LeadingFront*> expected;
    for (std::size_t generation = 0;
         generation < decision.selected_lineage.generations.size();
         ++generation)
      for (std::size_t front = 0;
           front < decision.selected_lineage.generations[generation].fronts.size();
           ++front) {
        const std::size_t node =
            decision.selected_topology.front_nodes[generation][front];
        if (decision.selected_topology.nodes[node].kind ==
                wave::FrontNode::Kind::Propagation &&
            decision.selected_topology.nodes[node].evaluation_order == 0)
          expected.push_back(
              &decision.selected_lineage.generations[generation].fronts[front]);
      }
    std::vector<const wave::LeadingFront*> actual;
    for (const wave::FrontGeneration& generation : stage.lineage.generations)
      for (const wave::LeadingFront& front : generation.fronts)
        actual.push_back(&front);
    require(actual.size() == expected.size(),
            "committing a layer must neither omit nor duplicate root fronts");
    for (std::size_t front = 0;
         front < std::min(actual.size(), expected.size()); ++front)
      require(actual[front]->path == expected[front]->path &&
                  actual[front]->closed == expected[front]->closed,
              "a committed front must retain the selected geometry exactly");
  }

  const wave::CommittedWavefront final =
      wave::commit_wavefront_analysis(analysis);
  require(front_count(final.lineage) == [&] {
            std::size_t count = 0;
            for (const wave::WavefrontStage& stage : committed)
              count += front_count(stage.lineage);
            return count;
          }(),
          "the combined result should contain every committed front once");
  std::size_t node_offset = 0;
  std::size_t expected_entries = 0;
  wave::Paths64 expected_reached;
  for (const wave::WavefrontStage& stage : committed) {
    expected_entries += stage.lineage.entry_fronts.size();
    for (std::size_t node = 0; node < stage.topology.nodes.size(); ++node) {
      require(node_offset + node < final.topology.nodes.size(),
              "the combined result should contain every committed node");
      if (node_offset + node >= final.topology.nodes.size()) continue;
      const wave::FrontNode& combined = final.topology.nodes[node_offset + node];
      require(combined.evaluation_order == stage.analysis_layer,
              "a final node should retain its recursive analysis layer");
      require(!combined.footprint.empty(),
              "a final node should have a highlightable footprint");
      expected_reached.insert(expected_reached.end(),
                              stage.topology.nodes[node].footprint.begin(),
                              stage.topology.nodes[node].footprint.end());
    }
    node_offset += stage.topology.nodes.size();
  }
  require(final.topology.nodes.size() == node_offset,
          "the combined result should contain no obsolete baseline nodes");
  expected_reached = Clipper2Lib::Union(
      expected_reached, Clipper2Lib::FillRule::NonZero);
  require(!final.wave.reached_generations.empty(),
          "the final committed wave should retain reached geometry");
  if (!final.wave.reached_generations.empty()) {
    const wave::Paths64& actual = final.wave.reached_generations.back();
    require(Clipper2Lib::Difference(expected_reached, actual,
                                    Clipper2Lib::FillRule::NonZero).empty(),
            "committing recursive layers must retain terminal swept bands");
  }
  require(final.lineage.entry_fronts.size() == expected_entries &&
              final.topology.entry_fronts.size() == expected_entries,
          "the combined result should expose every committed node entry");
  const std::vector<const wave::FrontLineage*> lineages{&final.lineage,
                                                         &final.lineage};
  const std::vector<const wave::LayerTopology*> topologies{&final.topology,
                                                            &final.topology};
  const wave::InterLayerTopology inter_layer =
      wave::build_inter_layer_topology(lineages, 1.0, topologies);
  require(inter_layer.layers.size() == 2 &&
              inter_layer.layers.front().nodes.size() ==
                  final.topology.nodes.size(),
          "Z grouping should consume the supplied committed topology");
  for (std::size_t node = 0; node < final.topology.nodes.size(); ++node)
    require(inter_layer.layers.front().nodes[node].evaluation_order ==
                final.topology.nodes[node].evaluation_order,
            "Z grouping should retain recursive node evaluation order");
}

void check_lsjig_reconstruction(const std::filesystem::path& mesh_path) {
  wave::MeshSlicer slicer;
  slicer.set_mesh(wave::load_triangle_mesh(mesh_path));
  const wave::Bounds3f& bounds = slicer.mesh().bounds;
  const wave::Paths64 stock{test_box(bounds.lower.x - 5.0,
                                     bounds.lower.y - 5.0,
                                     bounds.upper.x + 5.0,
                                     bounds.upper.y + 5.0)};
  const auto subject_at = [&](float z) {
    const wave::Paths64 raw = slicer.slice(z, 32).polygons;
    return Clipper2Lib::InflatePaths(
        Clipper2Lib::Union(raw, Clipper2Lib::FillRule::NonZero),
        static_cast<double>(wave::to_integer(3.2)),
        Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
  };
  const wave::Paths64 subject = subject_at(5.0f);
  const wave::WavefrontAnalysis analysis = wave::analyze_wavefront(
      subject, stock, 2.4, 2.4, 3.2);
  check_committed_fronts(analysis);
  const auto committed = wave::committed_wavefront_stages(analysis);
  require(committed.size() == 2,
          "LsJig1 at the representative CAM depth has two recursive layers");
  std::size_t fronts = 0;
  std::size_t generations = 0;
  for (const wave::WavefrontStage& stage : committed) {
    fronts += front_count(stage.lineage);
    generations += stage.lineage.generations.size();
  }
  require(fronts == 26 && generations == 17,
          "LsJig1 committed geometry should match the CAM analysis totals");
  const wave::Toolpath toolpath = wave::build_toolpath(committed, 3.0);
  require(toolpath.stats.unreachable_fronts == 0,
          "LsJig1 committed stages should retain every dependency");

  // At this height the one-entry strategy creates a terminal split branch of
  // only a few square millimetres. Rejecting that event should collapse the
  // branch into its continuing node without deleting any wavefront geometry.
  const wave::Paths64 tie_subject = subject_at(15.0f);
  const wave::WavefrontAnalysis scored = wave::analyze_wavefront(
      tie_subject, stock, 2.4, 2.4, 3.2);
  require(scored.layers.size() >= 2 &&
              scored.layers[1].trials.size() == 1,
          "LsJig1 Z15 should expose the raw node-count tie");
  if (scored.layers.size() >= 2 && !scored.layers[1].trials.empty()) {
    const wave::WavefrontStrategyTrial& trial =
        scored.layers[1].trials.front();
    require(trial.candidate_node_count == 4 &&
                trial.previous_node_count == 6,
            "the microscopic LsJig1 split should not create candidate nodes");
    require(trial.candidate_significant_node_count == 2 &&
                trial.previous_significant_node_count == 4 && trial.accepted,
            "the split-filtered LsJig1 strategy should win selection");
    require(trial.minimum_significant_node_area > 9.0 &&
                trial.minimum_significant_node_area < 10.5,
            "the LsJig1 significance threshold should use 0.5% of its largest node");
  }

  const wave::WavefrontAnalysis area_only = wave::analyze_wavefront(
      tie_subject, stock, 2.4, 2.4, 3.2, 2000, 128, {},
      wave::kDefaultSignificantNodeAreaFraction, 0.0);
  require(area_only.layers.size() >= 2 &&
              area_only.layers[1].trials.size() == 1,
          "unfiltered LsJig1 comparison should expose the same trial");
  if (area_only.layers.size() >= 2 &&
      !area_only.layers[1].trials.empty()) {
    const wave::WavefrontStrategyTrial& filtered =
        scored.layers[1].trials.front();
    const wave::WavefrontStrategyTrial& unfiltered =
        area_only.layers[1].trials.front();
    require(unfiltered.candidate_node_count == 6 &&
                unfiltered.previous_node_count == 6 && unfiltered.accepted,
            "area scoring should still resolve the unfiltered raw node tie");
    require(front_count(filtered.lineage) == front_count(unfiltered.lineage),
            "rejecting a tiny split must retain every propagated front");
    const auto split_count = [](const wave::FrontLineage& lineage) {
      return std::count_if(
          lineage.events.begin(), lineage.events.end(),
          [](const wave::LineageEvent& event) {
            return event.kind == wave::LineageEventKind::Split;
          });
    };
    require(split_count(filtered.lineage) + 1 ==
                split_count(unfiltered.lineage),
            "the terminal microscopic branch should lose exactly one split event");
    require(filtered.lineage.rejected_splits.size() == 1 &&
                unfiltered.lineage.rejected_splits.empty(),
            "rejected split contraction should be explicit in the lineage");
  }

  const wave::WavefrontAnalysis raw_counted = wave::analyze_wavefront(
      tie_subject, stock, 2.4, 2.4, 3.2, 2000, 128, {}, 0.0, 0.0);
  require(raw_counted.layers.size() >= 2 &&
              raw_counted.layers[1].trials.size() == 1,
          "raw LsJig1 comparison should expose the same trial");
  if (raw_counted.layers.size() >= 2 &&
      !raw_counted.layers[1].trials.empty()) {
    const wave::WavefrontStrategyTrial& trial =
        raw_counted.layers[1].trials.front();
    require(trial.candidate_significant_node_count == 6 &&
                trial.previous_significant_node_count == 6 && !trial.accepted,
            "a zero area fraction should preserve raw node-count behavior");
  }
}

// A square part centred in square stock, so every stock face is open, against
// the same part pushed out to meet the right face, so that face is occluded.
void check_lead_in() {
  const wave::Paths64 stock{test_box(-50, -50, 50, 50)};
  const wave::Paths64 centred{test_box(-20, -20, 20, 20)};
  const double stepover = 5.0;

  const auto plain = wave::build_wavefront(centred, stock, stepover, stepover);
  require(plain.lead_in_generations == 0, "no lead-in was asked for");
  require(plain.lead_in == 0.0, "no lead-in was asked for");
  require(!region_covers(plain.domain, -52.0, 0.0),
          "without a lead-in the wave must not run outside the stock");

  // Ten units of lead-in at a five unit stepover is two whole generations, and
  // each one is a generation the plain wave does not have.
  const auto led = wave::build_wavefront(centred, stock, stepover, stepover, 10.0);
  require(led.lead_in_generations == 2, "ten units of lead-in is two stepovers");
  require(std::abs(led.lead_in - 10.0) < 1e-9, "a whole-step lead-in is unchanged");
  require(led.generations.size() == plain.generations.size() + 2,
          "a two generation lead-in should add two generations");
  require(region_covers(led.domain, -52.0, 0.0),
          "the lead-in should open the air in front of an exposed stock face");
  require(!region_covers(led.domain, -62.0, 0.0),
          "the lead-in should reach no further than it was asked to");

  // A lead-in that is not a whole number of stepovers is used exactly as given
  // rather than rounded up. Where the first bite lands is the caller's decision
  // -- it sizes the run-in to put a chosen fraction of a stepover on the first
  // pass that touches the stock -- and rounding here would take that away.
  const auto part_step = wave::build_wavefront(centred, stock, stepover, stepover, 7.0);
  require(std::abs(part_step.lead_in - 7.0) < 1e-9,
          "a lead-in is seeded at exactly the distance asked for");
  require(part_step.lead_in_generations == 2,
          "seven units at a five unit stepover leaves two fronts outside the wall");
  require(region_covers(part_step.domain, -56.0, 0.0),
          "a part-step lead-in should still open the whole distance");
  require(!region_covers(part_step.domain, -58.0, 0.0),
          "a part-step lead-in should reach no further than it was asked to");

  // Every front the lead-in adds stands clear of the stock, which is the whole
  // point of it: the cutter takes up its bite over those passes instead of
  // arriving at full width on the first one.
  const auto lineage = wave::build_front_lineage(led);
  bool saw_outside = false;
  for (std::size_t generation = 0; generation < led.lead_in_generations; ++generation)
    for (const auto& front : lineage.generations[generation].fronts)
      if (!front.path.empty()) {
        require(!region_covers(stock,
                               wave::to_world(wave::representative_point(front).x),
                               wave::to_world(wave::representative_point(front).y)),
                "a lead-in front must not lie in the stock");
        saw_outside = true;
      }
  require(saw_outside, "a lead-in should produce fronts of its own");

  // Fronts stand at ten, five and zero units out from the wall. A 3 mm cutter
  // on the first two reaches nothing, so those cut nothing; on the third it
  // takes its first bite, and that is where cutting starts. Not the one after:
  // the caller sizes the run-in to place the first touching front where it
  // wants the first bite, and passing over that front hands the bite to the one
  // a whole stepover deeper, which arrives at very nearly a full slot.
  const auto led_toolpath = wave::build_toolpath(led, lineage, 3.0);
  std::size_t first_cut = std::numeric_limits<std::size_t>::max();
  for (const auto& move : led_toolpath.moves) {
    if (move.kind != wave::MoveKind::Cut) continue;
    first_cut = std::min(first_cut, move.front.generation);
  }
  require(first_cut == 2,
          "cutting starts at the first front whose cutter reaches the stock");

  // The subject reaches the right face, so that face has no wavefront on it and
  // must get no run-in. The left face is untouched and must still get one.
  const wave::Paths64 flush{test_box(-20, -20, 50, 20)};
  const auto occluded = wave::build_wavefront(flush, stock, stepover, stepover, 10.0);
  require(region_covers(occluded.domain, -52.0, 0.0),
          "an exposed face should still lead in");
  require(!region_covers(occluded.domain, 52.0, 0.0),
          "stock the subject occludes must not be led into");
}

void check_exterior_stock_reach() {
  const wave::Paths64 subject{test_box(-10, -10, 10, 10)};
  const wave::Paths64 stock{test_box(8, -2, 9.8, 2)};

  const auto strict =
      wave::seed_wavefront(subject, stock, 1.0, 1.0, 0.0, {}, 0.15);
  require(strict.generations.empty(),
          "stock farther inside the keepout than the exterior reach stays "
          "ineligible");

  const auto reached =
      wave::seed_wavefront(subject, stock, 1.0, 1.0, 0.0, {}, 0.25);
  require(!reached.generations.empty(),
          "exterior cutter reach should seed stock just inside the keepout");
  require(wave::paths_equal_canonical(reached.stock, stock),
          "exterior reach must not inflate the physical stock record");
  require(region_covers(reached.domain, 10.03, 0.0),
          "the propagation domain should include nearby exterior air");
  require(!region_covers(reached.stock, 10.03, 0.0),
          "nearby exterior air must not be reported as physical stock");

  const auto lineage = wave::build_front_lineage(reached);
  const auto air_only = wave::build_toolpath(reached, lineage, 0.1);
  require(air_only.stats.cut_count == 0,
          "a front touching only the expanded domain must remain an air pass");
  const auto contacting = wave::build_toolpath(reached, lineage, 0.3);
  require(contacting.stats.cut_count > 0,
          "the same exterior front should cut when the physical cutter reaches "
          "stock");
}

// The classification of a leading edge must not depend on how the offsetter
// happened to vertex it. Whatever the sampling misses, every front the wave
// grows has to descend from the front it grew out of: the toolpath sequences
// fronts by that descent, and one with no parent is scheduled with nothing
// cleared ahead of it and entered with a plunge.
void check_long_leading_edge_is_split() {
  const auto lineage = wave::build_front_lineage(blocked_midpoint_sweep());
  require(lineage.generations.size() == 2,
          "the blocked sweep should keep both of its generations");
  require(!lineage.generations.front().fronts.empty(),
          "an edge fronting open material along most of its length must yield "
          "a front even when its midpoint stands against the part");
  require(lineage.generations.front().fronts.size() == 2,
          "the part should divide that edge into the two open stretches");
  for (const auto& front : lineage.generations[1].fronts)
    require(!front.parents.empty(),
            "a front the wave advanced into must descend from the one it grew "
            "out of");
  for (const auto& event : lineage.events)
    require(event.kind != wave::LineageEventKind::Birth,
            "a wave that only advances must report no front born from nothing");
}

// A plunge lands in material that is already gone, so it has to stand at least
// one tool radius clear of everything that is not. Before the radius reached
// the planner this was half a stepover regardless of the cutter, which put the
// opening descent inside the stock for any tool wider than the step.
void check_plunge_clearance() {
  const wave::Paths64 stock{test_box(-50, -50, 50, 50)};
  const wave::Paths64 subject{test_box(-20, -20, 20, 20)};
  const double stepover = 5.0;
  const double tool_radius = 12.0;
  const auto wave_result = wave::build_wavefront(subject, stock, stepover, stepover);
  const auto lineage = wave::build_front_lineage(wave_result);
  const auto toolpath = wave::build_toolpath(wave_result, lineage, tool_radius);

  const wave::Paths64 swept = Clipper2Lib::InflatePaths(
      stock, static_cast<double>(wave::to_integer(tool_radius)),
      Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
  bool checked = false;
  for (const auto& move : toolpath.moves) {
    if (move.kind != wave::MoveKind::Retract || move.path.empty()) continue;
    const wave::Point64& landing = move.path.back();
    require(!region_covers(swept, wave::to_world(landing.x), wave::to_world(landing.y)),
            "the opening plunge must clear the stock by a whole tool radius");
    checked = true;
    break;
  }
  require(checked, "the opening move should be a plunge in free air");
}

}  // namespace

int main(int argc, char** argv) {
  try {
    require(argc >= 2, "expected SVG filename argument");
    const auto svg = wave::load_svg_closed_paths(argv[1]);
    require(svg.source_path_count == 5, "wavetest.svg should contain five closed paths");
    require(std::abs(wave::to_world(svg.bounds.left) + 1500.0) < 0.01,
            "left transformed bound should be -1500");
    require(std::abs(wave::to_world(svg.bounds.right) - 1500.0) < 0.01,
            "right transformed bound should be 1500");
    require(std::abs(wave::to_world(svg.bounds.top) + 1500.0) < 0.01,
            "top transformed bound should be -1500");
    require(std::abs(wave::to_world(svg.bounds.bottom) - 1500.0) < 0.01,
            "bottom transformed bound should be 1500");

    const auto result = wave::build_wavefront(svg.paths, 150.0, 75.0);
    require(result.subject.size() == 5, "the five disjoint subjects should survive union");
    require(result.generations.size() > 2, "wavefront should produce several generations");
    require(result.reached_generations.size() == result.generations.size(),
            "every wave band should retain its cumulative reached set");
    require(result.stop_reason == wave::StopReason::Unchanged,
            "seeded propagation should finish at a fixed point");
    require(!wave::paths_equal_canonical(result.generations.front(),
                                         result.generations.back()),
            "first and last generations should differ");
    for (std::size_t i = 1; i < result.generations.size(); ++i) {
      require(!wave::paths_equal_canonical(result.generations[i - 1],
                                           result.generations[i]),
              "retained consecutive generations must differ");
    }

    const wave::Path64 square{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const wave::Path64 rotated{{10, 10}, {0, 10}, {0, 0}, {10, 0}};
    const wave::Path64 reversed{{0, 0}, {0, 10}, {10, 10}, {10, 0}};
    require(wave::paths_equal_canonical({square}, {rotated}),
            "canonical comparison should ignore starting vertex");
    require(wave::paths_equal_canonical({square}, {reversed}),
            "canonical comparison should ignore winding");

    check_lead_in();
    check_exterior_stock_reach();
    check_plunge_clearance();
    check_long_leading_edge_is_split();

    const auto lineage = wave::build_front_lineage(result);
    require(lineage.generations.size() == result.generations.size(),
            "lineage should cover every wave generation");
    require(!lineage.generations.front().fronts.empty(),
            "the stock seed should have a leading front");
    require(lineage.events.size() == 5, "wavetest should produce five lineage events");
    const std::vector<std::size_t> expected_generations{11, 18, 25, 31, 38};
    const std::vector<wave::LineageEventKind> expected_kinds{
        wave::LineageEventKind::Split,
        wave::LineageEventKind::Merge,
        wave::LineageEventKind::Split,
        wave::LineageEventKind::Merge,
        wave::LineageEventKind::Termination,
    };
    for (std::size_t i = 0; i < lineage.events.size(); ++i) {
      require(lineage.events[i].generation == expected_generations[i],
              "lineage event occurred in an unexpected generation");
      require(lineage.events[i].kind == expected_kinds[i],
              "lineage event has an unexpected kind");
    }

    const auto analysis = wave::analyze_wavefront(
        svg.paths, result.stock, 75.0, 0.0, 0.0, 2000, 8);
    require(!analysis.layers.empty(),
            "recursive wave analysis should evaluate its root layer");
    require(!analysis.layers.front().entries.empty() &&
                !analysis.layers.front().selected_topology.nodes.empty(),
            "a decision layer should retain entries and a selected node graph");
    require(analysis.layers.size() >= 2,
            "the sample should expose a later recursive decision layer");
    if (analysis.layers.size() >= 2) {
      const auto& decision = analysis.layers[1];
      require(decision.entries.size() == 2 && decision.groups.size() == 1,
              "the later sample entries should share one graph-derived volume");
      require(decision.trials.size() == 1 &&
                  decision.trials.front().retained_entries.size() == 1,
              "the connected volume should receive one longest-entry trial");
      const std::size_t candidate_roots = std::count_if(
          decision.trials.front().topology.nodes.begin(),
          decision.trials.front().topology.nodes.end(),
          [](const wave::FrontNode& node) {
            return node.kind == wave::FrontNode::Kind::Propagation &&
                   node.evaluation_order == 0;
          });
      require(candidate_roots == 1,
              "later-layer muting should activate only the retained entry front");
      require(decision.trials.front().accepted,
              "the sample's one-entry strategy should win on node count");
    }
    const auto committed = wave::committed_wavefront_stages(analysis);
    check_committed_fronts(analysis);
    require(committed.size() == analysis.layers.size(),
            "every recursive decision should produce one committed stage");
    std::size_t committed_fronts = 0;
    for (const wave::WavefrontStage& stage : committed) {
      require(stage.topology.front_nodes.size() == stage.lineage.generations.size(),
              "committed topology should match its filtered lineage");
      for (std::size_t generation = 0;
           generation < stage.lineage.generations.size(); ++generation) {
        require(stage.topology.front_nodes[generation].size() ==
                    stage.lineage.generations[generation].fronts.size(),
                "every committed front should retain a topology node");
        committed_fronts += stage.lineage.generations[generation].fronts.size();
      }
    }
    const auto committed_toolpath = wave::build_toolpath(committed);
    require(committed_toolpath.stats.unreachable_fronts == 0,
            "committed stages should preserve their internal dependencies");
    require(committed_toolpath.stats.cut_count == committed_fronts,
            "the staged sequencer should cut every committed front once");
    require(committed_toolpath.stats.retract_count >= 1,
            "staged sequencing still needs its initial descent");

    for (const bool notched : {false, true}) {
      const auto sweep = wave::build_front_lineage(two_band_sweep(notched));
      require(sweep.generations.size() == 2, "the sweep should cover both bands");
      require(sweep.generations[0].fronts.size() == 1,
              "the first band should expose one leading front");
      require(sweep.generations[1].fronts.size() == 1,
              "a quantization step must not cut the leading front in two");
      require(sweep.events.empty(),
              "a quantization step must not raise a split or birth event");
      const auto sweep_path = wave::build_toolpath(two_band_sweep(notched), sweep);
      require(sweep_path.stats.cut_count == 2, "both bands should be cut");
      require(sweep_path.stats.entry_count == 0 && sweep_path.stats.retract_count == 1,
              "the opening move plunges in the air beside the stock, not into it");
      require(sweep_path.stats.step_count == 1,
              "the move onto the second band is a stepover, not free travel");
      require(sweep_path.stats.link_count == 1,
              "the feed in from the air beside the stock cuts nothing");

    }

    {
      const wave::FrontLineage collision =
          wave::build_front_lineage(opposing_terminal_sweep());
      require(collision.events.size() == 1,
              "opposing terminal fronts should produce one event");
      require(collision.events.front().kind ==
                  wave::LineageEventKind::MergeTermination,
              "opposing terminal fronts should merge into termination");
      require(collision.events.front().previous_fronts.size() == 2 &&
                  collision.events.front().current_fronts.empty(),
              "a merge termination has parents and no surviving front");
      const wave::LayerTopology nodes =
          wave::build_layer_topology(collision);
      require(nodes.nodes.size() == 3,
              "two propagation nodes should feed one empty terminal node");
      const wave::FrontNode& terminal = nodes.nodes.back();
      require(terminal.kind ==
                  wave::FrontNode::Kind::MergeTermination &&
                  terminal.fronts.empty() && terminal.footprint.empty() &&
                  terminal.parents.size() == 2 && terminal.children.empty(),
              "the terminal merge node should be an empty graph sink");
      const auto groups = wave::group_node_entries(nodes, 0);
      require(groups.size() == 1 && groups.front().entries.size() == 2,
              "the empty terminal node should graph-group its two entries");
    }
    {
      const wave::FrontLineage independent =
          wave::build_front_lineage(independent_terminal_sweeps());
      require(independent.events.size() == 2,
              "disconnected terminal fronts should remain separate events");
      require(std::all_of(independent.events.begin(), independent.events.end(),
                          [](const wave::LineageEvent& event) {
                            return event.kind ==
                                   wave::LineageEventKind::Termination;
                          }),
              "simultaneous but geometrically separate deaths are not a merge");
    }
    {
      const wave::Paths64 remaining{
          {{1000, 0}, {19000, 0}, {19000, 10000}, {1000, 10000}}};
      const wave::WaveSeed left{{{1000, 0}, {1000, 10000}}, false};
      const wave::WaveSeed right{{{19000, 10000}, {19000, 0}}, false};
      const auto both = wave::build_wavefront_from_seeds(
          remaining, {left, right}, 5.0, 0.0);
      const auto left_only = wave::build_wavefront_from_seeds(
          remaining, {left}, 5.0, 0.0);
      const auto both_lineage = wave::build_front_lineage(both);
      const auto left_lineage = wave::build_front_lineage(left_only);
      const auto both_nodes = wave::build_layer_topology(both_lineage);
      const auto left_nodes = wave::build_layer_topology(left_lineage);
      require(both_nodes.nodes.size() > left_nodes.nodes.size(),
              "one explicit later-layer seed should simplify the opposing sweep");
    }


    {
      auto [outer_wave, outer_lineage] = outer_box_detour_case();
      const auto outer_path = wave::build_toolpath(outer_wave, outer_lineage);
      require(outer_path.stats.retract_count == 2,
              "the artificial outer box must not replace a required rapid");
      for (const auto& move : outer_path.moves)
        require(move.kind != wave::MoveKind::Step || move.length < 5.0,
                "a step must not follow the artificial outer box");
    }

    {
      auto [choice_wave, choice_lineage] = shorter_rapid_case();
      // With no vertical penalty this synthetic lift is genuinely faster than
      // the slightly longer at-depth detour. Real jobs pass their full safe
      // height, which can correctly reverse this choice.
      const auto choice_path = wave::build_toolpath(
          choice_wave, choice_lineage, 0.0, 0.0, 0.0, 2000.0, 1000.0);
      require(choice_path.stats.retract_count == 2,
              "a shorter rapid must beat a longer valid detour");
      require(choice_path.stats.detour_count == 0,
              "distance selection must reject the longer detour");
      const auto high_clearance = wave::build_toolpath(
          choice_wave, choice_lineage, 0.0, -5.0, 10.0, 2000.0, 1000.0);
      require(high_clearance.stats.detour_count == 1,
              "safe-plane vertical time can make the at-depth route faster");
    }

    const auto toolpath = wave::build_toolpath(result, lineage);
    std::size_t front_count = 0;
    for (const auto& generation : lineage.generations)
      front_count += generation.fronts.size();
    require(toolpath.stats.unreachable_fronts == 0,
            "lineage precedence should leave no front unreachable");
    require(toolpath.stats.cut_count == front_count,
            "every leading front should be cut exactly once");
    require(toolpath.moves.front().kind == wave::MoveKind::Retract,
            "the opening move lands in the air beside the stock");
    // The free space around wavetest's islands is connected and the stock stands
    // in air, so every descent can land somewhere already empty. A ramp here
    // means a blocked segment stopped being routed around.
    require(toolpath.stats.entry_count == 0,
            "no descent on wavetest should have to ramp into solid material");
    require(toolpath.stats.retract_count >= 1,
            "wavetest needs at least its opening descent");
    require(toolpath.stats.lifts_over_subject == 0,
            "a lift must not pass over the subject where a way round exists");
    require(toolpath.stats.retract_count + toolpath.stats.entry_count <=
                toolpath.stats.cut_count,
            "a lift only ever happens on the way to a cut");
    for (std::size_t i = 1; i < toolpath.moves.size(); ++i)
      require(!(toolpath.moves[i].kind == wave::MoveKind::Cut &&
                toolpath.moves[i - 1].kind == wave::MoveKind::Cut),
              "consecutive cuts must be joined by a connecting move");
    // A lift that sets down in cleared material owes the tool a move at depth
    // before it can start cutting, or the descent itself does the cutting.
    for (std::size_t i = 0; i + 1 < toolpath.moves.size(); ++i)
      require(!(toolpath.moves[i].kind == wave::MoveKind::Retract &&
                toolpath.moves[i + 1].kind == wave::MoveKind::Cut) ||
                  toolpath.moves[i].path.back() == toolpath.moves[i + 1].path.front(),
              "a retract must hand the tool to the cut it serves");

    // A front may only be entered once the parents that opened the way to it
    // have been cut, or the tool takes a full-width bite instead of a stepover.
    std::vector<std::vector<bool>> cut(lineage.generations.size());
    for (std::size_t i = 0; i < lineage.generations.size(); ++i)
      cut[i].assign(lineage.generations[i].fronts.size(), false);
    for (const auto& move : toolpath.moves) {
      if (move.kind != wave::MoveKind::Cut) continue;
      const auto& front = lineage.generations[move.front.generation].fronts[move.front.index];
      for (const auto parent : front.parents)
        require(cut[move.front.generation - 1][parent],
                "a front was cut before one of its parents");
      cut[move.front.generation][move.front.index] = true;
    }

    require(gouge_length(toolpath, result.subject, 500.0) == 0.0,
            "no cutting or linking move may pass through the subject");

    // The playback cursor has to walk the path monotonically and land exactly on
    // both ends, or the animated tool will jump or stop short of the last cut.
    const double total = wave::toolpath_length(toolpath);
    require(total > 0.0, "the toolpath should have positive length");
    require(wave::toolpath_at(toolpath, 0.0).move == 0,
            "the cursor starts on the opening move");
    require(wave::toolpath_at(toolpath, -50.0).move == 0,
            "a negative distance clamps to the start");
    const auto finish = wave::toolpath_at(toolpath, total * 2.0);
    require(finish.move + 1 == toolpath.moves.size(),
            "a distance past the end clamps to the final move");
    require(finish.position == toolpath.moves.back().path.back(),
            "the cursor should finish on the last point of the last move");
    std::size_t previous_move = 0;
    for (int sample = 0; sample <= 400; ++sample) {
      const auto cursor = wave::toolpath_at(toolpath, total * sample / 400.0);
      require(cursor.move >= previous_move, "the cursor must not walk backwards");
      require(cursor.move < toolpath.moves.size(), "the cursor must stay in range");
      const auto& move = toolpath.moves[cursor.move];
      require(cursor.move_distance >= 0.0 && cursor.move_distance <= move.length + 1e-6,
              "the cursor must sit within the move it reports");
      previous_move = cursor.move;
    }
    require(previous_move + 1 == toolpath.moves.size(),
            "sampling the whole length should reach the final move");

    if (argc > 2) check_lsjig_reconstruction(argv[2]);

    std::cout << "Loaded " << svg.source_path_count << " paths and generated "
              << result.generations.size() << " non-empty wavefronts and "
              << lineage.events.size() << " lineage events.\n";
    std::cout << "Toolpath: cut " << toolpath.stats.cut_length << " in "
              << toolpath.stats.cut_count << " moves, step "
              << toolpath.stats.step_length << " in " << toolpath.stats.step_count
              << ", link " << toolpath.stats.link_length << " in "
              << toolpath.stats.link_count << ", retract "
              << toolpath.stats.retract_length << " in "
              << toolpath.stats.retract_count << ".\n";
    for (const auto& event : lineage.events)
      std::cout << "  event at " << event.generation << ": "
                << wave::lineage_event_kind_name(event.kind) << '\n';
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::cerr << "Test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
