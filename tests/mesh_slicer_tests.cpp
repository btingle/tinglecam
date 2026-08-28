#include "mesh_slicer.h"

#include <clipper2/clipper.h>

#include <cmath>
#include <iostream>
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
