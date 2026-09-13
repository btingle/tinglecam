#include "mesh_slicer_cuda.h"

#include <cuBQL/bvh.h>
#include <cuBQL/traversal/fixedBoxQuery.h>
#include <cuBQL/traversal/rayQueries.h>

#include <cuda_runtime.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace wave::cuda_slicer {
namespace {

using cuBQL::box3f;
using cuBQL::bvh3f;
using cuBQL::vec3f;

struct GpuTriangle {
  vec3f a;
  vec3f b;
  vec3f c;
};

void check(cudaError_t result, const char* operation) {
  if (result != cudaSuccess)
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(result));
}

__device__ float cross_xy(vec3f a, vec3f b, vec3f p) {
  return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
}

__device__ bool point_in_xy(const GpuTriangle& triangle, vec3f point) {
  const float c0 = cross_xy(triangle.a, triangle.b, point);
  const float c1 = cross_xy(triangle.b, triangle.c, point);
  const float c2 = cross_xy(triangle.c, triangle.a, point);
  constexpr float epsilon = 1e-5f;
  return (c0 >= -epsilon && c1 >= -epsilon && c2 >= -epsilon) ||
         (c0 <= epsilon && c1 <= epsilon && c2 <= epsilon);
}

__device__ float height_at(const GpuTriangle& triangle, vec3f point) {
  const float area = cross_xy(triangle.a, triangle.b, triangle.c);
  const float wb = ((point.x - triangle.a.x) * (triangle.c.y - triangle.a.y) -
                    (point.y - triangle.a.y) * (triangle.c.x - triangle.a.x)) / area;
  const float wc = ((triangle.b.x - triangle.a.x) * (point.y - triangle.a.y) -
                    (triangle.b.y - triangle.a.y) * (point.x - triangle.a.x)) / area;
  return triangle.a.z + wb * (triangle.b.z - triangle.a.z) +
         wc * (triangle.c.z - triangle.a.z);
}

__device__ bool contained(const GpuTriangle& inner, const GpuTriangle& outer) {
  const vec3f points[3] = {inner.a, inner.b, inner.c};
  for (const auto point : points) {
    if (!point_in_xy(outer, point)) return false;
    if (point.z > height_at(outer, point) + 1e-4f) return false;
  }
  return true;
}

__global__ void make_bounds(box3f* boxes, const GpuTriangle* triangles,
                            std::uint32_t count) {
  const std::uint32_t id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id >= count) return;
  const auto triangle = triangles[id];
  boxes[id] = box3f().extend(triangle.a).extend(triangle.b).extend(triangle.c);
}

__global__ void prune_contained(std::uint32_t* pruned,
                                const GpuTriangle* triangles,
                                const box3f* boxes, std::uint32_t count,
                                bvh3f bvh, float global_lower_z,
                                float global_upper_z) {
  const std::uint32_t id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id >= count) return;
  const box3f primitive_bounds = boxes[id];
  const box3f query_box{
      vec3f(primitive_bounds.lower.x, primitive_bounds.lower.y, global_lower_z),
      vec3f(primitive_bounds.upper.x, primitive_bounds.upper.y, global_upper_z)};
  const GpuTriangle inner = triangles[id];
  auto candidate = [&](std::uint32_t other) -> int {
    if (other == id || !contained(inner, triangles[other]))
      return CUBQL_CONTINUE_TRAVERSAL;
    // Equal prisms are deterministically represented by their lowest ID.
    if (!contained(triangles[other], inner) || other < id) {
      pruned[id] = 1;
      return CUBQL_TERMINATE_TRAVERSAL;
    }
    return CUBQL_CONTINUE_TRAVERSAL;
  };
  cuBQL::fixedBoxQuery::forEachPrim(candidate, bvh, query_box);
}

__global__ void count_pruned(const std::uint32_t* pruned, std::uint32_t count,
                             std::uint32_t* result) {
  const std::uint32_t id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id < count && pruned[id]) atomicAdd(result, 1u);
}

__global__ void invalidate_pruned(box3f* boxes, const std::uint32_t* pruned,
                                  std::uint32_t count) {
  const std::uint32_t id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id >= count || !pruned[id]) return;
  boxes[id].lower = vec3f(1.0f);
  boxes[id].upper = vec3f(-1.0f);
}

__global__ void query_tiles(std::uint32_t* flags, std::uint32_t* output,
                            std::uint32_t* output_count,
                            const std::uint32_t* pruned,
                            const box3f* boxes, std::uint32_t triangle_count, bvh3f bvh,
                            vec3f lower, vec3f upper, float slice_z,
                            int grid_resolution) {
  const int tile = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int tile_count = grid_resolution * grid_resolution;
  if (tile >= tile_count) return;
  const int x = tile % grid_resolution;
  const int y = tile / grid_resolution;
  const float x0 = lower.x + (upper.x - lower.x) * x / grid_resolution;
  const float x1 = lower.x + (upper.x - lower.x) * (x + 1) / grid_resolution;
  const float y0 = lower.y + (upper.y - lower.y) * y / grid_resolution;
  const float y1 = lower.y + (upper.y - lower.y) * (y + 1) / grid_resolution;
  const box3f query_box{vec3f(x0, y0, slice_z), vec3f(x1, y1, upper.z)};
  auto candidate = [&](std::uint32_t id) -> int {
    if (id >= triangle_count || pruned[id] || !boxes[id].overlaps(query_box))
      return CUBQL_CONTINUE_TRAVERSAL;
    if (atomicCAS(flags + id, 0u, 1u) == 0u) {
      const std::uint32_t slot = atomicAdd(output_count, 1u);
      output[slot] = id;
    }
    return CUBQL_CONTINUE_TRAVERSAL;
  };
  cuBQL::fixedBoxQuery::forEachPrim(candidate, bvh, query_box);
}

__device__ float downward_hit(const GpuTriangle& triangle, vec3f origin,
                              float lower_z) {
  if (!point_in_xy(triangle, origin)) return CUBQL_INF;
  const float height = height_at(triangle, origin);
  constexpr float epsilon = 1e-5f;
  if (height >= origin.z - epsilon || height < lower_z - epsilon)
    return CUBQL_INF;
  return origin.z - height;
}

__global__ void vertical_rays(float* output, const vec3f* origins,
                              std::uint32_t count, const GpuTriangle* triangles,
                              bvh3f bvh, float lower_z) {
  const std::uint32_t id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id >= count) return;
  const vec3f origin = origins[id];
  cuBQL::ray3f ray(origin, vec3f(0.0f, 0.0f, -1.0f), 1e-5f,
                   origin.z - lower_z);
  float nearest = ray.tMax;
  auto candidate = [&](std::uint32_t primitive) -> float {
    nearest = fminf(nearest,
                    downward_hit(triangles[primitive], origin, lower_z));
    return nearest;
  };
  cuBQL::shrinkingRayQuery::forEachPrim(candidate, bvh, ray);
  output[id] = nearest < origin.z - lower_z - 1e-5f
                   ? origin.z - nearest
                   : CUBQL_INF;
}

float elapsed_ms(cudaEvent_t start, cudaEvent_t stop) {
  check(cudaEventRecord(stop), "record CUDA timing event");
  check(cudaEventSynchronize(stop), "wait for CUDA timing event");
  float milliseconds = 0.0f;
  check(cudaEventElapsedTime(&milliseconds, start, stop), "read CUDA timing event");
  return milliseconds;
}

}  // namespace

struct Handle {
  bvh3f bvh;
  GpuTriangle* triangles = nullptr;
  box3f* boxes = nullptr;
  std::uint32_t* pruned = nullptr;
  std::uint32_t* flags = nullptr;
  std::uint32_t* output = nullptr;
  std::uint32_t* output_count = nullptr;
  std::uint32_t triangle_count = 0;
  Vec3f lower;
  Vec3f upper;

  ~Handle() {
    cuBQL::cuda::free(bvh);
    cudaFree(triangles);
    cudaFree(boxes);
    cudaFree(pruned);
    cudaFree(flags);
    cudaFree(output);
    cudaFree(output_count);
  }
};

Handle* create(const std::vector<Triangle3f>& triangles, const Bounds3f& bounds,
               MeshBuildStats& stats) {
  if (triangles.empty()) throw std::invalid_argument("Cannot build a BVH for an empty mesh");
  int device_count = 0;
  check(cudaGetDeviceCount(&device_count), "enumerate CUDA devices");
  if (device_count == 0) throw std::runtime_error("No CUDA device is available");

  const auto wall_start = std::chrono::steady_clock::now();
  auto handle = std::make_unique<Handle>();
  handle->triangle_count = static_cast<std::uint32_t>(triangles.size());
  handle->lower = bounds.lower;
  handle->upper = bounds.upper;
  std::vector<GpuTriangle> packed;
  packed.reserve(triangles.size());
  for (const auto& triangle : triangles) {
    packed.push_back({vec3f(triangle.a.x, triangle.a.y, triangle.a.z),
                      vec3f(triangle.b.x, triangle.b.y, triangle.b.z),
                      vec3f(triangle.c.x, triangle.c.y, triangle.c.z)});
  }
  const std::size_t count = triangles.size();
  check(cudaMalloc(&handle->triangles, count * sizeof(GpuTriangle)), "allocate GPU triangles");
  check(cudaMalloc(&handle->boxes, count * sizeof(box3f)), "allocate GPU bounds");
  check(cudaMalloc(&handle->pruned, count * sizeof(std::uint32_t)), "allocate prune flags");
  check(cudaMalloc(&handle->flags, count * sizeof(std::uint32_t)), "allocate query flags");
  check(cudaMalloc(&handle->output, count * sizeof(std::uint32_t)), "allocate query output");
  check(cudaMalloc(&handle->output_count, sizeof(std::uint32_t)), "allocate query count");
  check(cudaMemcpy(handle->triangles, packed.data(), count * sizeof(GpuTriangle),
                   cudaMemcpyHostToDevice), "upload triangles");
  check(cudaMemset(handle->pruned, 0, count * sizeof(std::uint32_t)), "clear prune flags");

  constexpr int block_size = 256;
  const int blocks = static_cast<int>((count + block_size - 1) / block_size);
  make_bounds<<<blocks, block_size>>>(handle->boxes, handle->triangles,
                                      handle->triangle_count);
  check(cudaGetLastError(), "launch BVH bounds kernel");
  cuBQL::gpuBuilder(handle->bvh, handle->boxes, handle->triangle_count,
                    cuBQL::BuildConfig(4));
  prune_contained<<<blocks, block_size>>>(
      handle->pruned, handle->triangles, handle->boxes, handle->triangle_count,
      handle->bvh, bounds.lower.z, bounds.upper.z);
  check(cudaGetLastError(), "launch prism containment kernel");
  std::uint32_t* device_pruned_count = nullptr;
  check(cudaMalloc(&device_pruned_count, sizeof(std::uint32_t)), "allocate prune count");
  check(cudaMemset(device_pruned_count, 0, sizeof(std::uint32_t)), "clear prune count");
  count_pruned<<<blocks, block_size>>>(handle->pruned, handle->triangle_count,
                                       device_pruned_count);
  check(cudaGetLastError(), "launch prune count kernel");
  std::uint32_t pruned_count = 0;
  check(cudaMemcpy(&pruned_count, device_pruned_count, sizeof(std::uint32_t),
                   cudaMemcpyDeviceToHost), "download prune count");
  cudaFree(device_pruned_count);

  // Rebuild over only retained primitives. cuBQL treats inverted boxes as
  // invalid and preserves original primitive IDs for every valid box.
  invalidate_pruned<<<blocks, block_size>>>(handle->boxes, handle->pruned,
                                             handle->triangle_count);
  check(cudaGetLastError(), "launch pruned-bound invalidation kernel");
  cuBQL::cuda::free(handle->bvh);
  cuBQL::gpuBuilder(handle->bvh, handle->boxes, handle->triangle_count,
                    cuBQL::BuildConfig(4));

  stats.pruned_triangles = pruned_count;
  stats.retained_triangles = count - pruned_count;
  stats.gpu_accelerated = true;
  stats.build_milliseconds = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - wall_start).count();
  return handle.release();
}

void destroy(Handle* handle) {
  delete handle;
}

std::vector<std::uint32_t> query(Handle* handle, float z, int grid_resolution,
                                 double& milliseconds) {
  if (!handle) throw std::runtime_error("Mesh slicer is not initialized");
  grid_resolution = std::clamp(grid_resolution, 1, 256);
  if (z > handle->upper.z) {
    milliseconds = 0.0;
    return {};
  }
  const std::size_t bytes = handle->triangle_count * sizeof(std::uint32_t);
  check(cudaMemset(handle->flags, 0, bytes), "clear slice candidate flags");
  check(cudaMemset(handle->output_count, 0, sizeof(std::uint32_t)), "clear slice candidate count");
  cudaEvent_t start = nullptr;
  cudaEvent_t stop = nullptr;
  check(cudaEventCreate(&start), "create CUDA timing event");
  check(cudaEventCreate(&stop), "create CUDA timing event");
  check(cudaEventRecord(start), "record CUDA timing event");
  const int tile_count = grid_resolution * grid_resolution;
  constexpr int block_size = 256;
  query_tiles<<<(tile_count + block_size - 1) / block_size, block_size>>>(
      handle->flags, handle->output, handle->output_count, handle->pruned,
      handle->boxes, handle->triangle_count, handle->bvh,
      vec3f(handle->lower.x, handle->lower.y, handle->lower.z),
      vec3f(handle->upper.x, handle->upper.y, handle->upper.z), z,
      grid_resolution);
  check(cudaGetLastError(), "launch tiled slice query");
  milliseconds = elapsed_ms(start, stop);
  cudaEventDestroy(start);
  cudaEventDestroy(stop);
  std::uint32_t count = 0;
  check(cudaMemcpy(&count, handle->output_count, sizeof(std::uint32_t),
                   cudaMemcpyDeviceToHost), "download slice candidate count");
  std::vector<std::uint32_t> result(count);
  if (count)
    check(cudaMemcpy(result.data(), handle->output, count * sizeof(std::uint32_t),
                     cudaMemcpyDeviceToHost), "download slice candidates");
  std::sort(result.begin(), result.end());
  return result;
}

std::vector<float> vertical_raycast(Handle* handle,
                                    const std::vector<Vec3f>& origins,
                                    float lower_z, double& milliseconds) {
  if (!handle) throw std::runtime_error("Mesh slicer is not initialized");
  if (origins.empty()) {
    milliseconds = 0.0;
    return {};
  }
  const std::size_t origin_bytes = origins.size() * sizeof(vec3f);
  const std::size_t result_bytes = origins.size() * sizeof(float);
  vec3f* device_origins = nullptr;
  float* device_results = nullptr;
  check(cudaMalloc(&device_origins, origin_bytes), "allocate ray origins");
  check(cudaMalloc(&device_results, result_bytes), "allocate ray results");
  check(cudaMemcpy(device_origins, origins.data(), origin_bytes,
                   cudaMemcpyHostToDevice), "upload ray origins");
  cudaEvent_t start = nullptr;
  cudaEvent_t stop = nullptr;
  try {
    check(cudaEventCreate(&start), "create CUDA timing event");
    check(cudaEventCreate(&stop), "create CUDA timing event");
    check(cudaEventRecord(start), "record CUDA timing event");
    constexpr int block_size = 256;
    const int blocks = static_cast<int>((origins.size() + block_size - 1) /
                                        block_size);
    vertical_rays<<<blocks, block_size>>>(
        device_results, device_origins, static_cast<std::uint32_t>(origins.size()),
        handle->triangles, handle->bvh, lower_z);
    check(cudaGetLastError(), "launch vertical ray query");
    milliseconds = elapsed_ms(start, stop);
    std::vector<float> results(origins.size());
    check(cudaMemcpy(results.data(), device_results, result_bytes,
                     cudaMemcpyDeviceToHost), "download ray results");
    cudaEventDestroy(start);
    cudaEventDestroy(stop);
    cudaFree(device_origins);
    cudaFree(device_results);
    return results;
  } catch (...) {
    if (start) cudaEventDestroy(start);
    if (stop) cudaEventDestroy(stop);
    cudaFree(device_origins);
    cudaFree(device_results);
    throw;
  }
}

}  // namespace wave::cuda_slicer
