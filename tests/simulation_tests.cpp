#include "simulation/simulation.h"
#include "wave/topology.h"

#include <cmath>
#include <algorithm>
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

void check_methods() {
  using namespace Clipper2Lib;
  using wave::SimulationMethod;
  const wave::Paths64 stock{box(0,0,20000,10000)};
  const wave::Paths64 subject{box(8000,4000,12000,6000)};
  wave::Toolpath path;
  path.moves.push_back(move(wave::MoveKind::Cut,{{-2000,-600},{22000,-600}}));
  path.moves.push_back(move(wave::MoveKind::Cut,{{22000,-600},{-2000,-600}}));
  path.moves.push_back(move(wave::MoveKind::Retract,{{-2000,-600},{-2000,5000}}));
  path.moves.push_back(move(wave::MoveKind::Cut,{{-2000,5000},{22000,5000}}));
  path.moves.push_back(move(wave::MoveKind::Retract,{{22000,5000},{10000,5000}}));
  path.moves.push_back(move(wave::MoveKind::Cut,{{10000,5000},{10000,12000},{5000,12000},{5000,-2000}}));
  path = wave::resample_toolpath(path,0.2);
  const auto reference = wave::simulate_material(stock,subject,path,1,0,SimulationMethod::ReferenceArea);
  double last_progress = 0.0;
  std::size_t reports = 0;
  const auto local = wave::simulate_material(stock,subject,path,1,0,SimulationMethod::LocalArea,
      [&](double fraction) {
        require(fraction >= last_progress && fraction <= 1.0,
                "simulation progress counts intervals monotonically");
        last_progress = fraction;
        ++reports;
        return true;
      });
  require(last_progress == 1.0 && reports > path.moves.size() + 1,
          "simulation reports progress inside long moves and reaches completion");
  bool cancelled = false;
  try {
    wave::simulate_material(stock,subject,path,1,0,SimulationMethod::LocalArea,
        [](double fraction) { return fraction < 0.1; });
  } catch (const std::runtime_error& error) {
    cancelled = std::string(error.what()) == "Cancelled";
  }
  require(cancelled, "simulation cooperatively cancels between interval batches");
  const auto contact = wave::simulate_material(stock,subject,path,1,0,SimulationMethod::Contact);
  const auto compare = wave::simulate_material(stock,subject,path,1,0,SimulationMethod::Compare);
  require(local.stats.stock_batches > 1 && local.stats.stock_batches < local.stats.cutting_steps/4,
          "local simulation batches many input intervals per global update");
  for (const auto* result : {&local,&contact,&compare}) {
    require(result->stats.cutting_steps==reference.stats.cutting_steps,
            "simulation modes retain every input interval");
    require_near(wave::material_area(Xor(result->remaining_stock,reference.remaining_stock,FillRule::NonZero)),
                 0,0.02,"batching agrees with reference stock within integer clipping tolerance");
    require_near(result->stats.gouge_area,reference.stats.gouge_area,0.02,
                 "all methods retain swept-interval gouge detection");
    require(result->stats.unsafe_plunges==reference.stats.unsafe_plunges,
            "pending stock cuts are committed before checking a plunge");
    require(wave::paths_equal_canonical(result->remaining_stock,local.remaining_stock),
            "contact estimates never change final stock geometry");
    for (std::size_t m=0;m<result->moves.size();++m) {
      require(result->moves[m].steps.size()==reference.moves[m].steps.size(),"sample layout stays comparable");
      for (std::size_t i=0;i<result->moves[m].steps.size();++i) {
        const auto& step=result->moves[m].steps[i];
        require(std::isfinite(step.load) && step.load>=0,"engagement is finite and nonnegative");
        require_near(step.area_load,reference.moves[m].steps[i].area_load,0.025,
                     "local material accounting agrees with reference samples");
        if (m==0 && step.to.x>step.from.x && step.from.x>2000 && step.to.x<18000)
          require_near(step.load,0.2,0.01,"steady side cutting measures radial engagement / diameter");
        if (m==0 && step.to.x<step.from.x && step.to.x>2000 && step.from.x<18000)
          require_near(step.load,0,0.01,"reverse travel through a previous cut remains air");
      }
    }
    const auto final_preview=wave::simulation_stock_at(*result,path,wave::toolpath_length(path));
    require(wave::paths_equal_canonical(final_preview,result->remaining_stock),"completed preview matches batched stock");
  }
  require(compare.stats.contact_mean_abs_difference<0.02,"contact and area agree on simple cuts");
  for (std::size_t m=0; m<local.moves.size(); ++m)
    for (std::size_t i=0; i<local.moves[m].steps.size(); ++i)
      require(compare.moves[m].steps[i].load == local.moves[m].steps[i].load,
              "comparison retains area engagement for feed assignment");
  // Holes, islands, reversals, and a small stock island between contact sample
  // locations exercise winding and the independent swept-volume safety check.
  auto hole=box(4000,2000,16000,8000); std::reverse(hole.begin(),hole.end());
  auto hollow=stock; hollow.push_back(hole); hollow.push_back(box(9990,4800,10010,5200));
  const auto hollow_ref=wave::simulate_material(hollow,subject,path,1,0,SimulationMethod::ReferenceArea);
  const auto hollow_contact=wave::simulate_material(hollow,subject,path,1,0,SimulationMethod::Contact);
  require(hollow_contact.stats.gouge_area > 0.0075,
          "the complete thin island is reported as gouged");
  require_near(hollow_contact.stats.gouge_area,hollow_ref.stats.gouge_area,0.001,
               "contact sampling cannot hide gouging a thin island");
  require_near(wave::material_area(Xor(hollow_contact.remaining_stock,hollow_ref.remaining_stock,FillRule::NonZero)),
               0,0.02,"local clipping preserves holes and islands");
  wave::Toolpath fractional;
  fractional.moves.push_back(move(wave::MoveKind::Cut,{{-3000,-953},{23000,-953}}));
  fractional=wave::resample_toolpath(fractional,0.2);
  const auto fractional_contact=wave::simulate_material(stock,subject,fractional,1.5875,0,SimulationMethod::Contact);
  for (const auto& step : fractional_contact.moves.front().steps)
    if (step.from.x>3000 && step.to.x<17000)
      require_near(step.load,0.2,1e-6,
                   "contact uses the configured diameter after geometric radius rounding");
}

}  // namespace

int main(int argc, char** argv) {
  try {
    check_methods();
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
