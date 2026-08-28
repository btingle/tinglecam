#include "mesh_slicer_cuda.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace wave::cuda_slicer {
namespace {

double signed_area_xy(const Triangle3f& triangle) {
  return static_cast<double>(triangle.b.x - triangle.a.x) *
             (triangle.c.y - triangle.a.y) -
         static_cast<double>(triangle.b.y - triangle.a.y) *
             (triangle.c.x - triangle.a.x);
}

double height_at(const Triangle3f& triangle, Vec3f point) {
  const double area = signed_area_xy(triangle);
  const double wb = ((point.x - triangle.a.x) * (triangle.c.y - triangle.a.y) -
                     (point.y - triangle.a.y) * (triangle.c.x - triangle.a.x)) / area;
  const double wc = ((triangle.b.x - triangle.a.x) * (point.y - triangle.a.y) -
                     (triangle.b.y - triangle.a.y) * (point.x - triangle.a.x)) / area;
  return triangle.a.z + wb * (triangle.b.z - triangle.a.z) +
         wc * (triangle.c.z - triangle.a.z);
}

bool point_in_xy(const Triangle3f& triangle, Vec3f point) {
  const auto cross = [](Vec3f a, Vec3f b, Vec3f p) {
    return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
  };
  const double c0 = cross(triangle.a, triangle.b, point);
  const double c1 = cross(triangle.b, triangle.c, point);
  const double c2 = cross(triangle.c, triangle.a, point);
  constexpr double epsilon = 1e-6;
  return (c0 >= -epsilon && c1 >= -epsilon && c2 >= -epsilon) ||
         (c0 <= epsilon && c1 <= epsilon && c2 <= epsilon);
}

bool contained(const Triangle3f& inner, const Triangle3f& outer) {
  for (const Vec3f point : {inner.a, inner.b, inner.c}) {
    if (!point_in_xy(outer, point) || point.z > height_at(outer, point) + 1e-5) return false;
  }
  return true;
}

}  // namespace

struct Handle {
  std::vector<Triangle3f> triangles;
  std::vector<bool> pruned;
};

Handle* create(const std::vector<Triangle3f>& triangles, const Bounds3f&,
               MeshBuildStats& stats) {
  const auto start = std::chrono::steady_clock::now();
  auto* handle = new Handle{triangles, std::vector<bool>(triangles.size(), false)};
  for (std::size_t i = 0; i < triangles.size(); ++i) {
    for (std::size_t j = 0; j < triangles.size(); ++j) {
      if (i == j) continue;
      if (contained(triangles[i], triangles[j]) &&
          (!contained(triangles[j], triangles[i]) || j < i)) {
        handle->pruned[i] = true;
        break;
      }
    }
  }
  stats.pruned_triangles = std::count(handle->pruned.begin(), handle->pruned.end(), true);
  stats.retained_triangles = triangles.size() - stats.pruned_triangles;
  stats.gpu_accelerated = false;
  stats.build_milliseconds = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  return handle;
}

void destroy(Handle* handle) { delete handle; }

std::vector<std::uint32_t> query(Handle* handle, float z, int, double& milliseconds) {
  if (!handle) throw std::runtime_error("Mesh slicer is not initialized");
  const auto start = std::chrono::steady_clock::now();
  std::vector<std::uint32_t> result;
  for (std::size_t i = 0; i < handle->triangles.size(); ++i) {
    const auto& t = handle->triangles[i];
    if (!handle->pruned[i] && std::max({t.a.z, t.b.z, t.c.z}) >= z)
      result.push_back(static_cast<std::uint32_t>(i));
  }
  milliseconds = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  return result;
}

}  // namespace wave::cuda_slicer
