#pragma once
#include "wave/mesh_slicer.h"
#include <memory>
#include <vector>
#include <cstdint>

namespace wave::prism {
struct Geometry { std::vector<Triangle3f> triangles; }; // CCW XY, normalized Z [0,1]
struct Layer { std::shared_ptr<const Geometry> geometry; float bottom, top; };
struct Camera {
  Vec3f centre, right, up, toward, light;
  float scale=1, reach=100, pan_x=0, pan_y=0;
};
struct Stats { std::size_t layer_builds=0, top_builds=0, top_refits=0; };
struct Renderer;
Renderer* create();
void destroy(Renderer* renderer);
void update(Renderer* renderer,const std::vector<Layer>& layers);
Stats stats(const Renderer* renderer);
// CUDA/GL interop keeps rendered pixels on the GPU. Null means the current
// OpenGL context cannot share resources with the CUDA device.
struct Interop;
Interop* register_buffer(unsigned int gl_buffer);
void unregister_buffer(Interop* interop);
void render(Renderer*,Interop*,const Camera&,int width,int height);
// Also useful for headless tests or presentation on a non-CUDA GL device.
std::vector<std::uint8_t> render_host(Renderer*,const Camera&,int width,int height);
}
