// Compare real application slice/raycast APIs. All reported durations are host
// wall time, including GPU allocation, transfers, synchronization and sorting.
#include "cam/cad.h"
#include "wave/mesh_slicer.h"

#include <clipper2/clipper.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
void check(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
struct Measurements {
  double build = 0, slices = 0, polygons = 0, concurrent = 0, probes = 0, dense = 0;
  std::size_t retained = 0;
  std::vector<wave::MeshSlice> results;
  std::vector<float> hits;
};

Measurements measure(const wave::Mesh& mesh, wave::MeshSlicerBackend backend,
                      const std::vector<float>& depths,
                      const std::vector<wave::Vec3f>& rays) {
  Measurements m;
  wave::MeshSlicer slicer(backend);
  slicer.set_mesh(mesh);
  m.build = slicer.stats().build_milliseconds;
  m.retained = slicer.stats().retained_triangles;
  auto start = Clock::now();
  for (float z : depths) m.results.push_back(slicer.slice(z));
  m.slices = elapsed(start);
  for (const auto& slice : m.results) m.polygons += slice.polygon_milliseconds;
  // Same depths with the application's usual maximum of eight planning workers.
  start = Clock::now();
  std::vector<std::future<void>> workers;
  for (std::size_t worker = 0; worker < 8; ++worker)
    workers.push_back(std::async(std::launch::async, [&, worker] {
      for (std::size_t i = worker; i < depths.size(); i += 8) {
        const auto actual = slicer.slice(depths[i]);
        check(actual.polygons == m.results[i].polygons, "Concurrent slice changed geometry");
      }
    }));
  for (auto& worker : workers) worker.get();
  m.concurrent = elapsed(start);
  // Repeated 12-ray batches represent counterbore shoulder probing; the dense
  // batch measures throughput separately from latency.
  double unused = 0;
  start = Clock::now();
  for (int batch = 0; batch < 100; ++batch) {
    const auto begin = rays.begin() + batch * 12;
    const auto hits = slicer.vertical_raycast({begin, begin + 12}, mesh.bounds.lower.z - 1, unused);
    check(hits.size() == 12, "Short probe result");
  }
  m.probes = elapsed(start);
  start = Clock::now();
  m.hits = slicer.vertical_raycast(rays, mesh.bounds.lower.z - 1, unused);
  m.dense = elapsed(start);
  return m;
}

void compare(const Measurements& cpu, const Measurements& gpu) {
  if (cpu.retained != gpu.retained)
    std::cerr << "Containment counts differ: CPU retained " << cpu.retained
              << ", CUDA retained " << gpu.retained << '\n';
  for (std::size_t i = 0; i < cpu.results.size(); ++i) {
    if (cpu.results[i].polygons == gpu.results[i].polygons) continue;
    const auto difference = Clipper2Lib::Xor(cpu.results[i].polygons, gpu.results[i].polygons,
                                            Clipper2Lib::FillRule::NonZero);
    check(difference.empty(), "Slice polygons differ at depth index " + std::to_string(i) +
          " CPU candidates=" + std::to_string(cpu.results[i].candidate_triangles) +
          " CUDA candidates=" + std::to_string(gpu.results[i].candidate_triangles) +
          " XOR area mm2=" + std::to_string(Clipper2Lib::Area(difference) / 1e6));
  }
  for (std::size_t i = 0; i < cpu.hits.size(); ++i) {
    const float a = cpu.hits[i], b = gpu.hits[i];
    check(std::isfinite(a) == std::isfinite(b), "Ray hit masks differ at " + std::to_string(i));
    if (std::isfinite(a)) check(std::abs(a - b) < .001f, "Ray depths differ by >= 0.001 mm");
  }
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 2 || argc > 4) throw std::runtime_error("Usage: mesh_slicer_benchmark MODEL [RUNS=3] [SLICES=32]");
    const int runs = argc > 2 ? std::stoi(argv[2]) : 3;
    const int slices = argc > 3 ? std::stoi(argv[3]) : 32;
    check(runs > 0 && slices > 1, "Positive run count and at least two slices required");
    const auto start = Clock::now();
    const std::filesystem::path path = argv[1];
    const auto cad = cam::step_filename(path) ? cam::CadModel::load(path) : nullptr;
    const auto mesh = cad ? cad->mesh() : wave::load_triangle_mesh(path);
    const double import = elapsed(start);
    std::vector<float> depths;
    for (int i = 0; i < slices; ++i)
      depths.push_back(mesh.bounds.lower.z + (mesh.bounds.upper.z - mesh.bounds.lower.z) * i / (slices - 1));
    std::vector<wave::Vec3f> rays;
    for (int y = 0; y < 256; ++y)
      for (int x = 0; x < 256; ++x)
        rays.push_back({mesh.bounds.lower.x + (mesh.bounds.upper.x - mesh.bounds.lower.x) * (x + .5f) / 256,
                        mesh.bounds.lower.y + (mesh.bounds.upper.y - mesh.bounds.lower.y) * (y + .5f) / 256,
                        mesh.bounds.upper.z + 1});
    std::cerr << "model=" << path << " triangles=" << mesh.triangles.size()
              << " import_ms=" << import << " slices=" << slices
              << " rays=" << rays.size() << '\n';
    std::cout << "run,backend,triangles,retained,build_ms,slices_ms,polygons_ms,concurrent_slices_ms,100x12_rays_ms,65536_rays_ms\n";
    const bool cuda_available = wave::cuda_slicer_available();
    if (!cuda_available) std::cerr << "CUDA unavailable; measuring CPU only.\n";
    for (int run = 1; run <= runs; ++run) {
      Measurements cpu, gpu;
      if (!cuda_available) {
        cpu = measure(mesh, wave::MeshSlicerBackend::CPU, depths, rays);
      } else if (run % 2) {
        gpu = measure(mesh, wave::MeshSlicerBackend::CUDA, depths, rays);
        cpu = measure(mesh, wave::MeshSlicerBackend::CPU, depths, rays);
      } else {
        cpu = measure(mesh, wave::MeshSlicerBackend::CPU, depths, rays);
        gpu = measure(mesh, wave::MeshSlicerBackend::CUDA, depths, rays);
      }
      for (const auto& item : {std::pair{"cuda", &gpu}, std::pair{"cpu", &cpu}}) {
        if (!cuda_available && item.second == &gpu) continue;
        const auto& m = *item.second;
        std::cout << std::fixed << std::setprecision(3) << run << ',' << item.first << ','
                  << mesh.triangles.size() << ',' << m.retained << ',' << m.build << ','
                  << m.slices << ',' << m.polygons << ',' << m.concurrent << ','
                  << m.probes << ',' << m.dense << std::endl;
      }
      if (cuda_available) compare(cpu, gpu);
    }
    if (cuda_available) std::cerr << "All slice polygons and ray hits agree.\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
