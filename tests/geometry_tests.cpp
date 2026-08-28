#include "geometry.h"
#include "toolpath.h"
#include "topology.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

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
      require(sweep_path.stats.entry_count == 1 && sweep_path.stats.retract_count == 0,
              "two adjacent bands need only the opening ramp");
      require(sweep_path.stats.step_count == 1,
              "the move onto the second band is a stepover, not free travel");
    }

    const auto toolpath = wave::build_toolpath(result, lineage);
    std::size_t front_count = 0;
    for (const auto& generation : lineage.generations)
      front_count += generation.fronts.size();
    require(toolpath.stats.unreachable_fronts == 0,
            "lineage precedence should leave no front unreachable");
    require(toolpath.stats.cut_count == front_count,
            "every leading front should be cut exactly once");
    require(toolpath.moves.front().kind == wave::MoveKind::Entry,
            "nothing is cleared at the start, so the first move ramps in");
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
