#include "geometry.h"
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
  wave_result.engagement = 10.0;
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
    }

    std::cout << "Loaded " << svg.source_path_count << " paths and generated "
              << result.generations.size() << " non-empty wavefronts and "
              << lineage.events.size() << " lineage events.\n";
    for (const auto& event : lineage.events)
      std::cout << "  event at " << event.generation << ": "
                << wave::lineage_event_kind_name(event.kind) << '\n';
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::cerr << "Test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
