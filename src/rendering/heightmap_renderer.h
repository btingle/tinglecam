#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "simulation/heightmap.h"

namespace wave::heightfield {
struct Camera {
  Vec3f centre, right, up, toward, light;
  float scale = 1, reach = 100, pan_x = 0, pan_y = 0;
  std::array<float, 4> background{14.f / 255, 17.f / 255, 22.f / 255, 1};
};
struct MeshDraw {
  const Mesh* mesh = nullptr;
  Vec3f offset{};
  std::array<float, 4> colour{.42f, .60f, .80f, 1};
  bool wireframe = false;
};
struct LineVertex {
  Vec3f position;
  std::array<float, 4> colour;
};
// Independent line segments, two vertices each. Shared immutable geometry lets
// the renderer retain its GPU buffer across camera and playback changes.
struct Lines {
  std::vector<LineVertex> vertices;
};
struct Cutter {
  Vec3f tip{};
  float radius = 0, length = 0;
  bool ball = false;
  float point_height = 0, tip_flat_radius = 0;
};
struct Stats {
  std::size_t changed_tiles = 0, uploaded_bytes = 0, resident_bytes = 0;
  std::size_t tiles = 0, hierarchy_nodes = 0;
  std::size_t line_upload_bytes = 0;
  bool reused_frame = false;
};
// OpenGL 3.2 / GLSL 1.50; no CUDA or compute-shader dependency. Construct,
// render, read back, and destroy on the thread owning the current GL context.
// Display caps interpolate the stored samples with discontinuity-aware
// bilinear patches; simulation geometry is never modified by reconstruction.
class Renderer {
 public:
  Renderer();
  ~Renderer();
  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;
  unsigned render(const Heightmap&, const Camera&, int width, int height,
                  const std::vector<MeshDraw>& meshes = {},
                  const Cutter& cutter = {},
                  const std::shared_ptr<const Lines>& lines = {});
  Stats stats() const;
  std::vector<std::uint8_t> read_colour() const;
  std::vector<float> read_depth() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace wave::heightfield
