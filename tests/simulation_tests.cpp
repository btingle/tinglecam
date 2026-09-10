#include "simulation/simulation.h"
#include "wave/topology.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void require_near(double actual, double expected, double tolerance,
                  const char* message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(std::string(message) + ": got " +
                             std::to_string(actual) + ", expected " +
                             std::to_string(expected));
  }
}

wave::Path64 box(std::int64_t left, std::int64_t top,
                 std::int64_t right, std::int64_t bottom) {
  return {{left, top}, {right, top}, {right, bottom}, {left, bottom}};
}

wave::ToolpathMove move(wave::MoveKind kind, wave::Path64 path) {
  wave::ToolpathMove result;
  result.kind = kind;
  result.path = std::move(path);
  for (std::size_t i = 1; i < result.path.size(); ++i)
    result.length += std::hypot(
        wave::to_world(result.path[i].x - result.path[i - 1].x),
        wave::to_world(result.path[i].y - result.path[i - 1].y));
  return result;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const wave::Paths64 stock{box(0, 0, 10000, 10000)};
    const wave::Paths64 subject{box(4000, 4000, 6000, 6000)};

    wave::Toolpath toolpath;
    toolpath.moves.push_back(
        move(wave::MoveKind::Cut, {{0, 5000}, {10000, 5000}}));
    toolpath.moves.push_back(
        move(wave::MoveKind::Retract, {{10000, 5000}, {5000, 8000}}));
    toolpath.moves.push_back(
        move(wave::MoveKind::Entry, {{5000, 8000}, {5000, 9000}}));

    const auto simulation =
        wave::simulate_material(stock, subject, toolpath, 1.0);
    require(simulation.moves.size() == toolpath.moves.size(),
            "simulation moves must align with toolpath moves");
    require(simulation.stats.cutting_steps == 1,
            "simulation should evaluate the normalized input interval directly");
    require_near(simulation.stats.removed_area, 20.0, 0.02,
                 "a boundary-to-boundary slot should remove width times length");
    double load_area = 0.0;
    for (const auto& step : simulation.moves[0].steps)
      load_area += step.load * 2.0 * simulation.tool_radius *
                   (step.move_distance_end - step.move_distance_begin);
    require_near(load_area, simulation.stats.removed_area, 0.02,
                 "per-step load must account for the newly removed area");
    require_near(simulation.stats.gouge_area, 4.0, 0.02,
                 "the removed portion of final subject must be reported as a gouge");
    require(simulation.stats.gouging_steps > 0,
            "intersecting cutter steps must be marked as gouging");
    require(simulation.stats.unsafe_plunges == 1,
            "a retract landing in uncut stock must be unsafe");
    require(simulation.moves[1].plunge_collision_area > 3.0,
            "the unsafe plunge should retain its collision geometry");
    require(simulation.stats.unresolved_entries == 1 &&
                simulation.moves[2].entry_unresolved,
            "entries without Z geometry must be explicitly unresolved");

    const double half_length = toolpath.moves[0].length * 0.5;
    const auto halfway = wave::simulation_stock_at(simulation, toolpath, half_length);
    const double halfway_area = wave::material_area(halfway);
    require(halfway_area < simulation.stats.initial_stock_area &&
                halfway_area > simulation.stats.remaining_area,
            "animated stock must shrink progressively through a cut");
    const auto finished = wave::simulation_stock_at(
        simulation, toolpath, wave::toolpath_length(toolpath) * 2.0);
    require(wave::paths_equal_canonical(finished, simulation.remaining_stock),
            "animated stock at the end must equal final simulated stock");

    wave::Toolpath safe_plunge;
    safe_plunge.moves.push_back(
        move(wave::MoveKind::Link, {{0, 5000}, {10000, 5000}}));
    safe_plunge.moves.push_back(
        move(wave::MoveKind::Retract, {{10000, 5000}, {5000, 5000}}));
    const auto safe = wave::simulate_material(stock, subject, safe_plunge, 1.0);
    require(safe.stats.unsafe_plunges == 0,
            "a retract may plunge into the area cleared by an earlier link");
    require(safe.stats.removed_area > 0.0,
            "links are at depth and must remove unexpected simulation stock");

    const auto expanded =
        wave::simulate_material(stock, subject, wave::Toolpath{}, 1.0, 1.0);
    require_near(expanded.stats.initial_stock_area, 144.0, 0.02,
                 "simulation stock inflation must be applied independently");
    require_near(wave::material_area(expanded.final_subject), 4.0, 0.001,
                 "simulation must retain the raw uninflated final subject");

    bool rejected_radius = false;
    try {
      static_cast<void>(
          wave::simulate_material(stock, subject, toolpath, 0.0));
    } catch (const std::invalid_argument&) {
      rejected_radius = true;
    }
    require(rejected_radius, "zero tool radius must be rejected");

    require(argc == 2, "expected representative SVG filename argument");
    const auto svg = wave::load_svg_closed_paths(argv[1]);
    const auto representative_wave =
        wave::build_wavefront(svg.paths, 150.0, 75.0);
    const auto representative_lineage =
        wave::build_front_lineage(representative_wave);
    const auto representative_path =
        wave::build_toolpath(representative_wave, representative_lineage);
    const auto representative_simulation = wave::simulate_material(
        representative_wave.stock, svg.paths, representative_path, 37.5);
    require(representative_simulation.moves.size() ==
                representative_path.moves.size(),
            "representative simulation must cover the complete planned path");
    require(representative_simulation.stats.removed_area > 0.0,
            "representative toolpath should remove stock");
    require(representative_simulation.stats.cutting_steps > 0,
            "representative toolpath should produce load samples");

    std::cout << "Simulated " << simulation.stats.cutting_steps
              << " cutting step, " << simulation.stats.gouging_steps
              << " gouge, and " << simulation.stats.unsafe_plunges
              << " unsafe plunge. Representative path used "
              << representative_simulation.stats.cutting_steps
              << " load samples.\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::cerr << "Test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
