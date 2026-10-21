#include "mesh_slicer_cuda.h"

#include <stdexcept>

namespace wave::cuda_slicer {
struct Handle {};

bool available() { return false; }
std::string current_device_name() { return {}; }

Handle* create(const std::vector<Triangle3f>&, const Bounds3f&, MeshBuildStats&) {
  throw std::runtime_error("CUDA mesh acceleration was not built; use the automatic or CPU slicer");
}
void destroy(Handle* handle) { delete handle; }

std::vector<std::uint32_t> query(Handle*, float, int, double&) {
  throw std::runtime_error("CUDA mesh slicer is not initialized");
}
std::vector<float> vertical_raycast(Handle*, const std::vector<Vec3f>&, float, double&) {
  throw std::runtime_error("CUDA mesh slicer is not initialized");
}
}  // namespace wave::cuda_slicer
