#pragma once

#include "mesh_slicer.h"

#include <cstdint>
#include <vector>

namespace wave::cpu_slicer {
struct Handle;
// The mesh must outlive the handle. Queries are read-only and can run concurrently.
Handle* create(const std::vector<Triangle3f>& triangles, const Bounds3f& bounds,
               MeshBuildStats& stats);
void destroy(Handle* handle);
std::vector<std::uint32_t> query(const Handle* handle, float z,
                                 double& milliseconds);
std::vector<float> vertical_raycast(const Handle* handle,
                                    const std::vector<Vec3f>& origins,
                                    float lower_z, double& milliseconds);
}  // namespace wave::cpu_slicer
