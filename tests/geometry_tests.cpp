#include "geometry.h"
#include "toolpath.h"
#include "topology.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
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

  // These two generated contours are farther from the stock than a 3 mm
  // cutter can reach. The next contour is the first to touch stock, and is
  // retained only as entry geometry: actual cutting starts at its +1 child.
  const auto led_toolpath = wave::build_toolpath(led, lineage, 3.0);
  bool saw_productive_cut = false;
  for (const auto& move : led_toolpath.moves) {
    if (move.kind != wave::MoveKind::Cut) continue;
    require(move.front.generation > led.lead_in_generations,
            "lead-in cutting must start one generation after first contact");
    saw_productive_cut = true;
  }
  require(saw_productive_cut,
          "suppressing the first contact must retain its productive children");

  // The subject reaches the right face, so that face has no wavefront on it and
  // must get no run-in. The left face is untouched and must still get one.
  const wave::Paths64 flush{test_box(-20, -20, 50, 20)};
  const auto occluded = wave::build_wavefront(flush, stock, stepover, stepover, 10.0);
  require(region_covers(occluded.domain, -52.0, 0.0),
          "an exposed face should still lead in");
  require(!region_covers(occluded.domain, 52.0, 0.0),
          "stock the subject occludes must not be led into");
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
    require(argc == 2, "expected SVG filename argument");
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
    check_plunge_clearance();

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
