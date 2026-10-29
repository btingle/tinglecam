#include "wave/mesh_slicer.h"

#include <clipper2/clipper.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <future>
#include <random>
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

void check_backend(wave::MeshSlicerBackend backend, bool expect_cpu = false) {
  wave::MeshSlicer slicer(backend);
  slicer.set_mesh(test_mesh());
  require(slicer.stats().input_triangles == 3, "input triangle count mismatch");
  require(slicer.stats().pruned_triangles == 1, "contained prism was not pruned");
  require(slicer.stats().retained_triangles == 2, "retained triangle count mismatch");
  if (backend == wave::MeshSlicerBackend::CPU || expect_cpu)
    require(!slicer.stats().gpu_accelerated, "CPU backend should be selected");
  if (backend == wave::MeshSlicerBackend::CUDA)
    require(slicer.stats().gpu_accelerated, "explicit CUDA must not silently fall back");
  if (backend == wave::MeshSlicerBackend::Default && !expect_cpu) {
    const char* override = std::getenv("TINGLECAM_SLICER");
    const bool force_cpu = override && std::string(override) == "cpu";
    require(slicer.stats().gpu_accelerated == (!force_cpu && wave::cuda_slicer_available()),
            "automatic selection should prefer usable CUDA, otherwise CPU");
  }

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
  double milliseconds = 0;
  const auto hits = slicer.vertical_raycast(
      {{1, 1, 12}, {13, 1, 12}, {30, 30, 12}, {1, 1, 9}}, -1, milliseconds);
  require(std::abs(hits[0] - 10) < 1e-5f, "downward ray should hit the top prism");
  require(std::abs(hits[1] - 8.0f / 3) < 1e-5f, "downward ray should hit the sloped prism");
  require(!std::isfinite(hits[2]), "outside ray must miss");
  require(!std::isfinite(hits[3]), "hidden lower prism must stay pruned for ray queries");
  require(slicer.vertical_raycast({}, -1, milliseconds).empty(), "empty ray batch");
  std::cout << (slicer.stats().gpu_accelerated ? "GPU" : "CPU")
            << " mesh slicer: 3 triangles, 1 contained prism pruned\n";
  slicer.clear();
  require(slicer.empty(), "clear must release the mesh");
  slicer.set_mesh(test_mesh());
  wave::MeshSlicer moved = std::move(slicer);
  require(!moved.slice(4).polygons.empty(), "moved slicer must retain a live mesh index");
}

void compare_backends() {
  auto mesh = test_mesh();
  // Duplicate triangles exercise stable tie-breaking; separated random slopes
  // exercise deeper trees and axis-parallel ray traversal on the host.
  mesh.triangles.push_back(mesh.triangles.front());
  std::mt19937 rng(92471);
  std::uniform_real_distribution<float> height(0, 10);
  for (int y = 0; y < 20; ++y)
    for (int x = 0; x < 20; ++x) {
      const float a = float(x) + 30, b = float(y);
      mesh.triangles.push_back({{a, b, height(rng)}, {a + .9f, b, height(rng)},
                                {a, b + .9f, height(rng)}});
    }
  mesh.bounds.upper = {50, 20, 10};
  wave::MeshSlicer cpu(wave::MeshSlicerBackend::CPU), gpu(wave::MeshSlicerBackend::CUDA);
  cpu.set_mesh(mesh); gpu.set_mesh(mesh);
  require(cpu.stats().pruned_triangles == gpu.stats().pruned_triangles,
          "CPU and CUDA must agree on containment pruning");
  for (float z : {-1.0f, 0.0f, 1.25f, 5.0f, 9.99f, 10.0f, 11.0f}) {
    const auto expected = gpu.slice(z, 32);
    for (int grid : {1, 8, 32, 256}) {
      const auto actual = cpu.slice(z, grid);
      require(actual.candidate_triangles == expected.candidate_triangles,
              "CPU candidates must agree regardless of slice grid");
      require(Clipper2Lib::Xor(actual.polygons, expected.polygons,
                              Clipper2Lib::FillRule::NonZero).empty(),
              "CPU and CUDA slice geometry must agree on the integer grid");
    }
  }
  std::uniform_real_distribution<float> x(-1, 51), y(-1, 21), z(0, 12);
  std::vector<wave::Vec3f> origins;
  for (int i = 0; i < 10000; ++i) origins.push_back({x(rng), y(rng), z(rng)});
  double milliseconds = 0;
  const auto expected = gpu.vertical_raycast(origins, -1, milliseconds);
  const auto actual = cpu.vertical_raycast(origins, -1, milliseconds);
  for (std::size_t i = 0; i < actual.size(); ++i) {
    require(std::isfinite(actual[i]) == std::isfinite(expected[i]),
            "CPU and CUDA downward ray hit masks must agree");
    if (std::isfinite(actual[i]))
      require(std::abs(actual[i] - expected[i]) < 1e-4f,
              "CPU and CUDA downward ray depths must agree");
  }
}

void check_overlapping_surfaces() {
  wave::Mesh mesh;
  constexpr int count=2048;
  for (int i=0;i<count;++i) {
    const float z=float(i)/count;
    mesh.triangles.push_back({{0,0,z},{1,0,1-z},{0,1,z}});
  }
  mesh.bounds={{0,0,0},{1,1,1},true};
  for (const auto backend : {wave::MeshSlicerBackend::CPU,wave::MeshSlicerBackend::CUDA}) {
    if (backend==wave::MeshSlicerBackend::CUDA && !wave::cuda_slicer_available()) continue;
    wave::MeshSlicer slicer(backend);
    slicer.set_mesh(mesh);
    const auto slice=slicer.slice(.5f);
    require(std::abs(Clipper2Lib::Area(slice.polygons)-500000)<1,
            "Bounded pruning must preserve the full projected surface envelope");
    std::vector<wave::Vec3f> origins;
    for (float x : {.1f,.25f,.5f,.75f}) origins.push_back({x,.1f,2});
    double ms;
    const auto hits=slicer.vertical_raycast(origins,-1,ms);
    for (std::size_t i=0;i<origins.size();++i) {
      float expected=-1;
      for (int j=0;j<count;++j) {
        const float z=float(j)/count;
        expected=std::max(expected,z+origins[i].x*(1-2*z));
      }
      require(std::isfinite(hits[i]) && std::abs(hits[i]-expected)<1e-5,
              "Bounded pruning must preserve top-envelope ray heights");
    }
  }
}

int main(int argc, char** argv) {
  try {
    const bool expect_cpu = argc == 2 && std::string(argv[1]) == "--expect-cpu";
    check_overlapping_surfaces();
    check_backend(wave::MeshSlicerBackend::CPU);
    check_backend(wave::MeshSlicerBackend::Default, expect_cpu);
    if (wave::cuda_slicer_available()) {
      require(!expect_cpu, "hidden CUDA devices should be unavailable");
      check_backend(wave::MeshSlicerBackend::CUDA);
      compare_backends();
    } else {
      bool rejected = false;
      try {
        wave::MeshSlicer forced(wave::MeshSlicerBackend::CUDA);
        forced.set_mesh(test_mesh());
      } catch (const std::runtime_error&) { rejected = true; }
      require(rejected, "explicit CUDA should fail when acceleration is unavailable");
      std::cout << "CUDA unavailable: automatic CPU fallback verified\n";
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "mesh_slicer_tests: " << error.what() << '\n';
    return 1;
  }
}
