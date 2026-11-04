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

void check_projected_sweeps() {
  wave::Mesh slope;
  slope.triangles={{{0,0,0},{12,0,12},{0,12,0}}};
  slope.bounds={{0,0,0},{12,12,12},true};
  wave::MeshSlicer slicer(wave::MeshSlicerBackend::CPU);slicer.set_mesh(slope);
  const auto point=[&](double x,double y,double z,double radius) {
    return slicer.projected_sweep_clear({x,y,z},{x,y,z},z,radius);
  };
  require(point(4,2,6,.9),"Sweeps must clip sloping fixture triangles at the requested height");
  require(!point(5,2,6,1),"Exact tangency with fixture clearance must be rejected");
  require(!point(7,2,6,.1),"A centre inside the projected surface is blocked");
  require(point(7,7,6,.9),"Broad-phase box overlap alone does not prove contact");
  require(!point(7,6,6,.9),"Disc clearance must include diagonal fixture edges");
  require(slicer.projected_sweep_clear({-2,10,6},{14,10,6},6,.1),"Sweeps outside the clipped triangle are clear");
  require(!slicer.projected_sweep_clear({-2,4,6},{14,4,6},6,.1),"A crossing between clear endpoints must be blocked");
  require(slicer.projected_sweep_clear({-2,4,13},{14,4,13},13,1),"A sweep above the fixture is clear");

  wave::Mesh wall;wall.triangles={{{0,0,0},{0,12,0},{0,0,12}}};
  wall.bounds={{0,0,0},{0,12,12},true};
  wave::MeshSlicer vertical(wave::MeshSlicerBackend::CPU);vertical.set_mesh(wall);
  require(!vertical.projected_sweep_clear({.5,3,2},{.5,3,2},2,1),"Thin vertical fixtures remain obstacles");
  require(vertical.projected_sweep_clear({2,3,2},{2,3,2},2,1),"Thin fixtures do not block unrelated nearby space");

  // An independent polygon oracle checks all cube orientations and translated
  // placements. Skip only the small boundary band affected by integer rounding.
  std::vector<wave::ModelOrientation> orientations(1);
  for(std::size_t i=0;i<orientations.size();++i) for(int axis=0;axis<3;++axis) {
    auto next=orientations[i];next.quarter_turn(axis,1);
    if(std::find(orientations.begin(),orientations.end(),next)==orientations.end()) orientations.push_back(next);
  }
  std::mt19937 random(4271);
  std::uniform_real_distribution<double> coordinate(-15,15),height(-13,13);
  std::size_t checked=0,blocked=0;
  for(const auto orientation:orientations) {
    const wave::ModelTransform placement{orientation,{21,-13,7}};
    const auto placed=slicer.placed(placement);
    for(int i=0;i<60;++i) {
      const wave::Vec3d a{21+coordinate(random),-13+coordinate(random),0};
      const wave::Vec3d b{21+coordinate(random),-13+coordinate(random),0};
      const float z=float(7+height(random));
      const auto region=placed.slice(z,32).polygons;
      const auto reference=[&](double radius) {
        const auto obstacles=Clipper2Lib::InflatePaths(region,wave::to_integer(radius),
            Clipper2Lib::JoinType::Round,Clipper2Lib::EndType::Polygon);
        Clipper2Lib::Clipper64 clipper;
        clipper.AddOpenSubject({{{wave::to_integer(a.x),wave::to_integer(a.y)},
                                 {wave::to_integer(b.x),wave::to_integer(b.y)}}});
        clipper.AddClip(obstacles);
        wave::Paths64 closed,open;
        require(clipper.Execute(Clipper2Lib::ClipType::Intersection,Clipper2Lib::FillRule::NonZero,closed,open),
                "Fixture reference clipping succeeds");
        return open.empty();
      };
      const bool small=reference(.48),large=reference(.52);
      if(small!=large) continue;
      // A zero-area projection has no polygon oracle; covered separately above.
      if(region.empty() && !placed.projected_sweep_clear(a,b,z,.5)) continue;
      const bool clear=placed.projected_sweep_clear(a,b,z,.5);
      require(clear==small,"Indexed fixture sweeps must match the independent outline oracle");
      ++checked;if(!clear) ++blocked;
    }
  }
  require(checked>1000 && blocked>40,"Fixture oracle exercises both blocked and clear sweeps across placements");
  wave::MeshSlicer concurrent(wave::MeshSlicerBackend::CPU);concurrent.set_mesh(slope);
  std::vector<std::future<bool>> queries;
  for(int i=0;i<8;++i) queries.push_back(std::async(std::launch::async,[&] {
    return !concurrent.projected_sweep_clear({-2,4,6},{14,4,6},6,.1);
  }));
  for(auto& query:queries) require(query.get(),"Shared fixture sweep acceleration supports concurrent queries");
}

int main(int argc, char** argv) {
  try {
    const bool expect_cpu = argc == 2 && std::string(argv[1]) == "--expect-cpu";
    check_projected_sweeps();
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
