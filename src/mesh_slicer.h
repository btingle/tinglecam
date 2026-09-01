#pragma once

#include "geometry.h"

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

// Loads triangulated OBJ, binary STL, or ASCII STL geometry.
Mesh load_triangle_mesh(const std::filesystem::path& filename);

// Persistent downward-prism slicer. The BVH and containment pruning are built
// once; each slice only queries candidates and reconstructs their 2D union.
class MeshSlicer {
 public:
  MeshSlicer();
  ~MeshSlicer();
  MeshSlicer(MeshSlicer&&) noexcept;
  MeshSlicer& operator=(MeshSlicer&&) noexcept;
  MeshSlicer(const MeshSlicer&) = delete;
  MeshSlicer& operator=(const MeshSlicer&) = delete;

  void set_mesh(Mesh mesh);
  // Back to holding nothing. `set_mesh` rejects an empty mesh, so this is how a
  // slicer is emptied once it has been given one.
  void clear();
  MeshSlice slice(float z, int grid_resolution = 32) const;
  const Mesh& mesh() const;
  const MeshBuildStats& stats() const;
  bool empty() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace wave
