#include "mesh_slicer_cpu.h"

#include <cuBQL/builder/cpu.h>
#include <cuBQL/traversal/fixedBoxQuery.h>
#include <cuBQL/traversal/rayQueries.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <thread>

namespace wave::cpu_slicer {
namespace {
using Clock = std::chrono::steady_clock;
using cuBQL::box3f;
using cuBQL::vec3f;

double elapsed(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

// Keep the CUDA predicates and tolerances, including deterministic duplicate
// removal. This experiment changes execution backend, not slice semantics.
float cross_xy(Vec3f a, Vec3f b, Vec3f p) {
  return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
}
bool point_in_xy(const Triangle3f& triangle, Vec3f p) {
  const float a = cross_xy(triangle.a, triangle.b, p);
  const float b = cross_xy(triangle.b, triangle.c, p);
  const float c = cross_xy(triangle.c, triangle.a, p);
  constexpr float epsilon = 1e-5f;
  return (a >= -epsilon && b >= -epsilon && c >= -epsilon) ||
         (a <= epsilon && b <= epsilon && c <= epsilon);
}
float height_at(const Triangle3f& triangle, Vec3f p) {
  const float area = cross_xy(triangle.a, triangle.b, triangle.c);
  const float wb = ((p.x - triangle.a.x) * (triangle.c.y - triangle.a.y) -
                    (p.y - triangle.a.y) * (triangle.c.x - triangle.a.x)) / area;
  const float wc = ((triangle.b.x - triangle.a.x) * (p.y - triangle.a.y) -
                    (triangle.b.y - triangle.a.y) * (p.x - triangle.a.x)) / area;
  return triangle.a.z + wb * (triangle.b.z - triangle.a.z) +
         wc * (triangle.c.z - triangle.a.z);
}
bool contained(const Triangle3f& inner, const Triangle3f& outer) {
  for (Vec3f p : {inner.a, inner.b, inner.c}) {
    if (!point_in_xy(outer, p) || p.z > height_at(outer, p) + 1e-4f)
      return false;
  }
  return true;
}
vec3f packed(Vec3f p) { return {p.x, p.y, p.z}; }

template<class Function>
void parallel_for(std::size_t count, Function function) {
  unsigned limit = std::min(8u, std::max(1u, std::thread::hardware_concurrency()));
  if (const char* value = std::getenv("TINGLECAM_SLICER_THREADS")) {
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (end == value || *end || parsed < 1 || parsed > 256)
      throw std::invalid_argument("TINGLECAM_SLICER_THREADS must be between 1 and 256");
    limit = static_cast<unsigned>(parsed);
  }
  const auto workers = std::min<std::size_t>(limit, std::max<std::size_t>(1, count / 1024));
  auto range = [&](std::size_t worker) {
    for (std::size_t i = count * worker / workers;
         i < count * (worker + 1) / workers; ++i) function(i);
  };
  std::vector<std::jthread> threads;
  for (std::size_t worker = 1; worker < workers; ++worker)
    threads.emplace_back(range, worker);
  range(0);
}
}  // namespace

struct Handle {
  const std::vector<Triangle3f>& triangles;
  Bounds3f bounds;
  std::vector<box3f> boxes;
  cuBQL::bvh3f bvh;
  Handle(const std::vector<Triangle3f>& input, Bounds3f bounds)
      : triangles(input), bounds(bounds) {}
  ~Handle() { cuBQL::cpu::freeBVH(bvh); }
};

Handle* create(const std::vector<Triangle3f>& triangles, const Bounds3f& bounds,
               MeshBuildStats& stats) {
  if (triangles.empty()) throw std::invalid_argument("Cannot build a BVH for an empty mesh");
  const auto start = Clock::now();
  auto handle = std::make_unique<Handle>(triangles, bounds);
  handle->boxes.reserve(triangles.size());
  for (const auto& t : triangles)
    handle->boxes.push_back(box3f().extend(packed(t.a)).extend(packed(t.b)).extend(packed(t.c)));
  cuBQL::cpuBuilder(handle->bvh, handle->boxes.data(),
                    static_cast<std::uint32_t>(triangles.size()), cuBQL::BuildConfig(4));
  std::vector<std::uint8_t> pruned(triangles.size());
  parallel_for(triangles.size(), [&](std::size_t id) {
    const auto& box = handle->boxes[id];
    const box3f search{{box.lower.x, box.lower.y, bounds.lower.z},
                       {box.upper.x, box.upper.y, bounds.upper.z}};
    cuBQL::fixedBoxQuery::forEachPrim([&](std::uint32_t other) {
      if (other != id && contained(triangles[id], triangles[other]) &&
          (!contained(triangles[other], triangles[id]) || other < id)) {
        pruned[id] = 1;
        return CUBQL_TERMINATE_TRAVERSAL;
      }
      return CUBQL_CONTINUE_TRAVERSAL;
    }, handle->bvh, search);
  });
  for (std::size_t id = 0; id < triangles.size(); ++id)
    if (pruned[id]) handle->boxes[id] = box3f();
  cuBQL::cpu::freeBVH(handle->bvh);
  cuBQL::cpuBuilder(handle->bvh, handle->boxes.data(),
                    static_cast<std::uint32_t>(triangles.size()), cuBQL::BuildConfig(4));
  stats.pruned_triangles = std::count(pruned.begin(), pruned.end(), 1);
  stats.retained_triangles = triangles.size() - stats.pruned_triangles;
  stats.gpu_accelerated = false;
  stats.build_milliseconds = elapsed(start);
  return handle.release();
}

void destroy(Handle* handle) { delete handle; }

std::vector<std::uint32_t> query(const Handle* handle, float z,
                                 double& milliseconds) {
  const auto start = Clock::now();
  std::vector<std::uint32_t> result;
  if (z <= handle->bounds.upper.z) {
    const box3f search{{handle->bounds.lower.x, handle->bounds.lower.y, z},
                       packed(handle->bounds.upper)};
    cuBQL::fixedBoxQuery::forEachPrim([&](std::uint32_t id) {
      if (handle->boxes[id].overlaps(search)) result.push_back(id);
      return CUBQL_CONTINUE_TRAVERSAL;
    }, handle->bvh, search);
    std::sort(result.begin(), result.end());
  }
  milliseconds = elapsed(start);
  return result;
}

std::vector<float> vertical_raycast(const Handle* handle,
                                    const std::vector<Vec3f>& origins,
                                    float lower_z, double& milliseconds) {
  const auto start = Clock::now();
  std::vector<float> result(origins.size(), std::numeric_limits<float>::quiet_NaN());
  parallel_for(origins.size(), [&](std::size_t i) {
    const auto origin = origins[i];
    if (origin.z <= lower_z) return;
    cuBQL::ray3f ray(packed(origin), {0, 0, -1}, 1e-5f, origin.z - lower_z);
    float nearest = ray.tMax;
    cuBQL::shrinkingRayQuery::forEachPrim([&](std::uint32_t id) {
      const auto& triangle = handle->triangles[id];
      if (point_in_xy(triangle, origin)) {
        const float height = height_at(triangle, origin);
        if (height < origin.z - 1e-5f && height >= lower_z - 1e-5f)
          nearest = std::min(nearest, origin.z - height);
      }
      return nearest;
    }, handle->bvh, ray);
    if (nearest < origin.z - lower_z - 1e-5f) result[i] = origin.z - nearest;
  });
  milliseconds = elapsed(start);
  return result;
}
}  // namespace wave::cpu_slicer
