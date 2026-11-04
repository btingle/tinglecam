#pragma once

#include "geometry.h"
#include "model_transform.h"

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace wave {

struct Vec3f {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Vec3d {
  double x = 0, y = 0, z = 0;
};

struct Triangle3f {
  Vec3f a;
  Vec3f b;
  Vec3f c;
};

struct Bounds3f {
  Vec3f lower;
  Vec3f upper;
  bool valid = false;
};

struct Mesh {
  std::vector<Triangle3f> triangles;
  Bounds3f bounds;
  std::filesystem::path source;
};

struct MeshBuildStats {
  std::size_t input_triangles = 0;
  std::size_t retained_triangles = 0;
  std::size_t pruned_triangles = 0;
  double build_milliseconds = 0.0;
  bool gpu_accelerated = false;
};

struct MeshSlice {
  Paths64 polygons;
  std::size_t candidate_triangles = 0;
  std::size_t contributing_triangles = 0;
  double query_milliseconds = 0.0;
  double polygon_milliseconds = 0.0;
};

// Loads polygonal OBJ, binary STL, or ASCII STL geometry, preserving all
// non-degenerate 3D faces for display. Height queries ignore vertical faces.
Mesh load_triangle_mesh(const std::filesystem::path& filename);

enum class MeshSlicerBackend { Default, CPU, CUDA };
// Includes build support, runtime/driver availability and kernel compatibility.
bool cuda_slicer_available();

// Persistent downward-prism slicer. The BVH and containment pruning are built
// once; each slice only queries candidates and reconstructs their 2D union.
class MeshSlicer {
 public:
  // Default prefers CUDA and falls back to CPU when unavailable. Override with
  // TINGLECAM_SLICER=auto|cpu|cuda; explicit CUDA requests fail rather than fall back.
  explicit MeshSlicer(MeshSlicerBackend backend = MeshSlicerBackend::Default);
  ~MeshSlicer();
  MeshSlicer(MeshSlicer&&) noexcept;
  MeshSlicer& operator=(MeshSlicer&&) noexcept;
  MeshSlicer(const MeshSlicer&) = default;
  MeshSlicer& operator=(const MeshSlicer&) = default;

  // Read-only placement of the shared source. Rotated query structures are
  // cached per source orientation; translations never rebuild a BVH.
  MeshSlicer placed(const ModelTransform&) const;
  const Mesh& source_mesh() const;
  const void* source_identity() const;

  void set_mesh(Mesh mesh);
  // Back to holding nothing. `set_mesh` rejects an empty mesh, so this is how a
  // slicer is emptied once it has been given one.
  void clear();
  MeshSlice slice(float z, int grid_resolution = 32) const;
  // Downward ray query against the retained mesh on the chosen backend. Misses are
  // returned as NaN; callers decide whether the lower bound is the stock floor.
  std::vector<float> vertical_raycast(const std::vector<Vec3f>& origins,
                                      float lower_z,
                                      double& milliseconds) const;
  // Test a swept XY disc against the downward envelope above minimum_z.
  // Uses a lazily built, shared BVH and local triangle clipping: no global
  // polygon union, Z quantization, or per-height cache is needed.
  bool projected_sweep_clear(Vec3d from, Vec3d to, double minimum_z,
                             double radius) const;
  const Mesh& mesh() const;
  // Keeps the immutable placed display geometry alive across model replacement.
  std::shared_ptr<const Mesh> shared_mesh() const;
  const MeshBuildStats& stats() const;
  bool empty() const;

 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace wave
