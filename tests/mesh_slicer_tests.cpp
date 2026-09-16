#include "wave/mesh_slicer.h"

#include <clipper2/clipper.h>

#include <cmath>
#include <iostream>
#include <future>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

wave::Mesh test_mesh() {
  wave::Mesh mesh;
  mesh.source = "synthetic-prisms";
  mesh.triangles = {
      {{0.0f, 0.0f, 10.0f}, {10.0f, 0.0f, 10.0f}, {0.0f, 10.0f, 10.0f}},
      {{1.0f, 1.0f, 5.0f}, {3.0f, 1.0f, 5.0f}, {1.0f, 3.0f, 5.0f}},
      {{12.0f, 0.0f, 0.0f}, {18.0f, 0.0f, 8.0f}, {12.0f, 6.0f, 8.0f}},
  };
  mesh.bounds = {{0.0f, 0.0f, 0.0f}, {18.0f, 10.0f, 10.0f}, true};
  return mesh;
}

}  // namespace

int main() {
  try {
    wave::MeshSlicer slicer;
    slicer.set_mesh(test_mesh());
    require(slicer.stats().input_triangles == 3, "input triangle count mismatch");
    require(slicer.stats().pruned_triangles == 1, "contained prism was not pruned");
    require(slicer.stats().retained_triangles == 2, "retained triangle count mismatch");

    const auto high = slicer.slice(9.0f, 8);
    require(high.candidate_triangles == 1, "high slice should query one prism");
    require(high.polygons.size() == 1, "high slice should contain one polygon");
    require(std::abs(std::abs(Clipper2Lib::Area(high.polygons.front())) - 50000000.0) < 1.0,
            "flat prism slice area mismatch");

    const auto middle = slicer.slice(4.0f, 8);
    require(middle.candidate_triangles == 2, "middle slice should query two retained prisms");
    require(!middle.polygons.empty(), "middle slice should produce polygons");

    // Concurrent layers reuse one GPU handle. Each query must retain its own
    // candidate count and polygons through the shared-buffer download.
    std::vector<std::future<void>> queries;
    for (int worker = 0; worker < 6; ++worker) {
      queries.push_back(std::async(std::launch::async, [&, worker] {
        for (int i = 0; i < 16; ++i) {
          const bool upper = (worker + i) % 2 == 0;
          const auto actual = slicer.slice(upper ? 9.0f : 4.0f, 8);
          const auto& expected = upper ? high : middle;
          require(actual.candidate_triangles == expected.candidate_triangles,
                  "concurrent slices must not exchange candidate buffers");
          require(Clipper2Lib::Xor(actual.polygons, expected.polygons,
                                  Clipper2Lib::FillRule::NonZero).empty(),
                  "concurrent slices must agree with serial geometry");
        }
      }));
    }
    for (auto& query : queries) query.get();

    const auto empty = slicer.slice(11.0f, 8);
    require(empty.polygons.empty(), "slice above the mesh should be empty");
    std::cout << (slicer.stats().gpu_accelerated ? "GPU" : "CPU")
              << " mesh slicer: 3 triangles, 1 contained prism pruned\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "mesh_slicer_tests: " << error.what() << '\n';
    return 1;
  }
}
