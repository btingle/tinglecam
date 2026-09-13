#pragma once

#include "mesh_slicer.h"

#include <cstdint>
#include <vector>

namespace wave::cuda_slicer {

struct Handle;

Handle* create(const std::vector<Triangle3f>& triangles, const Bounds3f& bounds,
               MeshBuildStats& stats);
void destroy(Handle* handle);
std::vector<std::uint32_t> query(Handle* handle, float z, int grid_resolution,
                                 double& milliseconds);
// Casts downward from each origin and returns the first mesh hit as a Z value.
// A NaN means that the ray reached the supplied lower bound without hitting
// retained geometry. The query is intentionally GPU-only, just like slicing.
std::vector<float> vertical_raycast(Handle* handle,
                                    const std::vector<Vec3f>& origins,
                                    float lower_z, double& milliseconds);

}  // namespace wave::cuda_slicer
