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

}  // namespace wave::cuda_slicer
