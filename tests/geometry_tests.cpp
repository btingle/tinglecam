#include "geometry.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
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

    const auto result = wave::build_wavefront(svg.paths, 150.0, 80.0);
    require(result.subject.size() == 5, "the five disjoint subjects should survive union");
    require(result.generations.size() > 2, "wavefront should produce several generations");
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

    std::cout << "Loaded " << svg.source_path_count << " paths and generated "
              << result.generations.size() << " non-empty wavefronts.\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::cerr << "Test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
