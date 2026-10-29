#include "heightmap_renderer.h"

#include <SDL.h>
#include <SDL_opengl.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

#include "heightmap_shaders.h"

namespace wave::heightfield {
namespace {
bool same_vector(Vec3f a, Vec3f b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}
bool same_camera(const Camera& a, const Camera& b) {
  return same_vector(a.centre,b.centre) && same_vector(a.right,b.right) &&
         same_vector(a.up,b.up) && same_vector(a.toward,b.toward) &&
         same_vector(a.light,b.light) && a.scale == b.scale && a.reach == b.reach &&
         a.pan_x == b.pan_x && a.pan_y == b.pan_y && a.background == b.background;
}
bool same_cutter(const Cutter& a, const Cutter& b) {
  return same_vector(a.tip,b.tip) && a.radius == b.radius &&
         a.length == b.length && a.ball == b.ball &&
         a.point_height==b.point_height && a.tip_flat_radius==b.tip_flat_radius;
}
// Resolve post-1.1 entry points through SDL, including on Windows. Nothing in
// this backend requires NVIDIA, CUDA/GL sharing, or OpenGL compute support.
#define GL_FUNCTIONS(X)                                        \
  X(PFNGLACTIVETEXTUREPROC, ActiveTexture)                     \
  X(PFNGLGENBUFFERSPROC, GenBuffers)                           \
  X(PFNGLDELETEBUFFERSPROC, DeleteBuffers)                     \
  X(PFNGLBINDBUFFERPROC, BindBuffer)                           \
  X(PFNGLBUFFERDATAPROC, BufferData)                           \
  X(PFNGLBUFFERSUBDATAPROC, BufferSubData)                     \
  X(PFNGLTEXBUFFERPROC, TexBuffer)                             \
  X(PFNGLCREATESHADERPROC, CreateShader)                       \
  X(PFNGLSHADERSOURCEPROC, ShaderSource)                       \
  X(PFNGLCOMPILESHADERPROC, CompileShader)                     \
  X(PFNGLGETSHADERIVPROC, GetShaderiv)                         \
  X(PFNGLGETSHADERINFOLOGPROC, GetShaderInfoLog)               \
  X(PFNGLDELETESHADERPROC, DeleteShader)                       \
  X(PFNGLCREATEPROGRAMPROC, CreateProgram)                     \
  X(PFNGLATTACHSHADERPROC, AttachShader)                       \
  X(PFNGLBINDATTRIBLOCATIONPROC, BindAttribLocation)           \
  X(PFNGLBINDFRAGDATALOCATIONPROC, BindFragDataLocation)       \
  X(PFNGLLINKPROGRAMPROC, LinkProgram)                         \
  X(PFNGLGETPROGRAMIVPROC, GetProgramiv)                       \
  X(PFNGLGETPROGRAMINFOLOGPROC, GetProgramInfoLog)             \
  X(PFNGLDELETEPROGRAMPROC, DeleteProgram)                     \
  X(PFNGLUSEPROGRAMPROC, UseProgram)                           \
  X(PFNGLGETUNIFORMLOCATIONPROC, GetUniformLocation)           \
  X(PFNGLUNIFORM1IPROC, Uniform1i)                             \
  X(PFNGLUNIFORM1FPROC, Uniform1f)                             \
  X(PFNGLUNIFORM2FPROC, Uniform2f)                             \
  X(PFNGLUNIFORM2IPROC, Uniform2i)                             \
  X(PFNGLUNIFORM3FPROC, Uniform3f)                             \
  X(PFNGLUNIFORM4FVPROC, Uniform4fv)                           \
  X(PFNGLGENVERTEXARRAYSPROC, GenVertexArrays)                 \
  X(PFNGLDELETEVERTEXARRAYSPROC, DeleteVertexArrays)           \
  X(PFNGLBINDVERTEXARRAYPROC, BindVertexArray)                 \
  X(PFNGLENABLEVERTEXATTRIBARRAYPROC, EnableVertexAttribArray) \
  X(PFNGLVERTEXATTRIBPOINTERPROC, VertexAttribPointer)         \
  X(PFNGLGENFRAMEBUFFERSPROC, GenFramebuffers)                 \
  X(PFNGLDELETEFRAMEBUFFERSPROC, DeleteFramebuffers)           \
  X(PFNGLBINDFRAMEBUFFERPROC, BindFramebuffer)                 \
  X(PFNGLFRAMEBUFFERTEXTURE2DPROC, FramebufferTexture2D)       \
  X(PFNGLCHECKFRAMEBUFFERSTATUSPROC, CheckFramebufferStatus)   \
  X(PFNGLCLEARBUFFERFVPROC, ClearBufferfv)                     \
  X(PFNGLBLENDFUNCSEPARATEPROC, BlendFuncSeparate)             \
  X(PFNGLBLENDEQUATIONSEPARATEPROC, BlendEquationSeparate)
struct Gl {
#define DECLARE(type, name) type name = nullptr;
  GL_FUNCTIONS(DECLARE)
#undef DECLARE
  Gl() {
#define LOAD(type, name)                                            \
  name = reinterpret_cast<type>(SDL_GL_GetProcAddress("gl" #name)); \
  if (!name) throw std::runtime_error("Heightmap renderer requires gl" #name);
    GL_FUNCTIONS(LOAD)
#undef LOAD
  }
};
// Restore state even when an allocation/shader error interrupts rendering.
struct State {
  Gl& gl;
  GLint program, vao, array_buffer, unpack_buffer, pack_buffer, draw_fbo,
      read_fbo;
  GLint viewport[4], active, textures[4], texture_2d, depth_func, polygon[2];
  GLint texture_buffer, cull_mode, front_face, pack_alignment, pack_row_length,
      pack_skip_rows, pack_skip_pixels;
  GLint blend_src_rgb, blend_dst_rgb, blend_src_alpha, blend_dst_alpha;
  GLint blend_eq_rgb, blend_eq_alpha;
  GLboolean depth_mask, colour_mask[4];
  bool depth, blend, cull, scissor;
  explicit State(Gl& g) : gl(g) {
    glGetIntegerv(GL_CURRENT_PROGRAM, &program);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &vao);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &array_buffer);
    glGetIntegerv(GL_PIXEL_UNPACK_BUFFER_BINDING, &unpack_buffer);
    glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &pack_buffer);
    glGetIntegerv(GL_TEXTURE_BUFFER_BINDING, &texture_buffer);
    glGetIntegerv(GL_CULL_FACE_MODE, &cull_mode);
    glGetIntegerv(GL_FRONT_FACE, &front_face);
    glGetIntegerv(GL_PACK_ALIGNMENT, &pack_alignment);
    glGetIntegerv(GL_PACK_ROW_LENGTH, &pack_row_length);
    glGetIntegerv(GL_PACK_SKIP_ROWS, &pack_skip_rows);
    glGetIntegerv(GL_PACK_SKIP_PIXELS, &pack_skip_pixels);
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &draw_fbo);
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &read_fbo);
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &active);
    for (int i = 0; i < 4; ++i) {
      gl.ActiveTexture(GL_TEXTURE0 + i);
      glGetIntegerv(GL_TEXTURE_BINDING_BUFFER, &textures[i]);
    }
    gl.ActiveTexture(GL_TEXTURE0);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture_2d);
    glGetIntegerv(GL_DEPTH_FUNC, &depth_func);
    glGetIntegerv(GL_POLYGON_MODE, polygon);
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depth_mask);
    glGetBooleanv(GL_COLOR_WRITEMASK, colour_mask);
    depth = glIsEnabled(GL_DEPTH_TEST);
    blend = glIsEnabled(GL_BLEND);
    cull = glIsEnabled(GL_CULL_FACE);
    scissor = glIsEnabled(GL_SCISSOR_TEST);
    glGetIntegerv(GL_BLEND_SRC_RGB, &blend_src_rgb);
    glGetIntegerv(GL_BLEND_DST_RGB, &blend_dst_rgb);
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &blend_src_alpha);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &blend_dst_alpha);
    glGetIntegerv(GL_BLEND_EQUATION_RGB, &blend_eq_rgb);
    glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &blend_eq_alpha);
  }
  ~State() {
    gl.UseProgram(program);
    gl.BindVertexArray(vao);
    gl.BindBuffer(GL_ARRAY_BUFFER, array_buffer);
    gl.BindBuffer(GL_PIXEL_UNPACK_BUFFER, unpack_buffer);
    gl.BindBuffer(GL_PIXEL_PACK_BUFFER, pack_buffer);
    gl.BindBuffer(GL_TEXTURE_BUFFER, texture_buffer);
    glCullFace(cull_mode);
    glFrontFace(front_face);
    glPixelStorei(GL_PACK_ALIGNMENT, pack_alignment);
    glPixelStorei(GL_PACK_ROW_LENGTH, pack_row_length);
    glPixelStorei(GL_PACK_SKIP_ROWS, pack_skip_rows);
    glPixelStorei(GL_PACK_SKIP_PIXELS, pack_skip_pixels);
    gl.BindFramebuffer(GL_DRAW_FRAMEBUFFER, draw_fbo);
    gl.BindFramebuffer(GL_READ_FRAMEBUFFER, read_fbo);
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    for (int i = 0; i < 4; ++i) {
      gl.ActiveTexture(GL_TEXTURE0 + i);
      glBindTexture(GL_TEXTURE_BUFFER, textures[i]);
    }
    gl.ActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture_2d);
    gl.ActiveTexture(active);
    glDepthFunc(depth_func);
    glDepthMask(depth_mask);
    glColorMask(colour_mask[0], colour_mask[1], colour_mask[2], colour_mask[3]);
    glPolygonMode(GL_FRONT_AND_BACK, polygon[0]);
    gl.BlendFuncSeparate(blend_src_rgb, blend_dst_rgb, blend_src_alpha,
                         blend_dst_alpha);
    gl.BlendEquationSeparate(blend_eq_rgb, blend_eq_alpha);
    for (auto [cap, enabled] : {std::pair{GL_DEPTH_TEST, depth},
                                {GL_BLEND, blend},
                                {GL_CULL_FACE, cull},
                                {GL_SCISSOR_TEST, scissor}})
      if (enabled)
        glEnable(cap);
      else
        glDisable(cap);
  }
};
using F4 = std::array<float, 4>;
using I4 = std::array<int, 4>;
struct Buffer {
  GLuint buffer = 0, texture = 0;
  std::size_t capacity = 0;
  GLenum format;
  std::size_t element_size;
  Buffer(GLenum f, std::size_t s) : format(f), element_size(s) {}
  void release(Gl& gl) {
    gl.DeleteBuffers(1, &buffer);
    glDeleteTextures(1, &texture);
    buffer = texture = 0;
    capacity = 0;
  }
  bool reserve(Gl& gl, std::size_t count, std::size_t limit) {
    if (count <= capacity) return false;
    if (count > limit)
      throw std::runtime_error(
          "Full-resolution heightmap exceeds this OpenGL device's "
          "texture-buffer limit");
    capacity = std::min(
        limit, std::max(count, std::max(std::size_t{64}, capacity * 2)));
    if (!buffer) {
      gl.GenBuffers(1, &buffer);
      glGenTextures(1, &texture);
    }
    gl.BindBuffer(GL_TEXTURE_BUFFER, buffer);
    gl.BufferData(GL_TEXTURE_BUFFER, capacity * element_size, nullptr,
                  GL_DYNAMIC_DRAW);
    glBindTexture(GL_TEXTURE_BUFFER, texture);
    gl.TexBuffer(GL_TEXTURE_BUFFER, format, buffer);
    return true;
  }
  void upload(Gl& gl, std::size_t offset, std::size_t count, const void* data) {
    if (!count) return;
    gl.BindBuffer(GL_TEXTURE_BUFFER, buffer);
    gl.BufferSubData(GL_TEXTURE_BUFFER, offset * element_size,
                     count * element_size, data);
  }
};
struct Slot {
  int offset = 0, size = 0;
};
struct TileSlots {
  Slot top, bottom;
};
struct Vertex {
  Vec3f position, normal;
};
std::vector<Vertex> mesh_vertices(const Mesh& mesh) {
  std::vector<Vertex> vertices;
  vertices.reserve(mesh.triangles.size() * 3);
  for (const auto& t : mesh.triangles) {
    const Vec3f a{t.b.x - t.a.x, t.b.y - t.a.y, t.b.z - t.a.z};
    const Vec3f b{t.c.x - t.a.x, t.c.y - t.a.y, t.c.z - t.a.z};
    Vec3f n{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
    const float length = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
    if (length < 1e-12f) continue;
    n = {n.x / length, n.y / length, n.z / length};
    for (auto p : {t.a, t.b, t.c}) vertices.push_back({p, n});
  }
  return vertices;
}
Mesh cutter_mesh(const Cutter& cutter) {
  Mesh mesh;
  constexpr int sides = 40, rings = 10;
  constexpr float pi = 3.14159265358979323846f;
  const float bottom = cutter.ball ? cutter.radius : cutter.point_height;
  auto point = [&](int side, float z) {
    const float a = 2 * pi * side / sides;
    return Vec3f{cutter.radius * std::cos(a), cutter.radius * std::sin(a), z};
  };
  for (int i = 0; i < sides; ++i) {
    auto a = point(i, bottom), b = point(i + 1, bottom),
         c = point(i + 1, cutter.length), d = point(i, cutter.length);
    mesh.triangles.push_back({a, b, c});
    mesh.triangles.push_back({a, c, d});
    mesh.triangles.push_back({{0, 0, cutter.length}, d, c});
    if(!cutter.ball && cutter.tip_flat_radius<=0)
      mesh.triangles.push_back({{0,0,0},b,a});
    else if (!cutter.ball) {
      const float t=2*pi*i/sides,u=2*pi*(i+1)/sides;
      const Vec3f p{cutter.tip_flat_radius*std::cos(t),cutter.tip_flat_radius*std::sin(t),0},
                  q{cutter.tip_flat_radius*std::cos(u),cutter.tip_flat_radius*std::sin(u),0};
      mesh.triangles.push_back({p,b,a});mesh.triangles.push_back({p,q,b});
      mesh.triangles.push_back({{0,0,0},q,p});
    }
  }
  if (cutter.ball)
    for (int ring = 0; ring < rings; ++ring)
      for (int i = 0; i < sides; ++i) {
        auto p = [&](int r, int side) {
          const float phi = -pi / 2 + pi / 2 * r / rings,
                      theta = 2 * pi * side / sides;
          return Vec3f{cutter.radius * std::cos(phi) * std::cos(theta),
                       cutter.radius * std::cos(phi) * std::sin(theta),
                       cutter.radius * (1 + std::sin(phi))};
        };
        auto a = p(ring, i), b = p(ring, i + 1), c = p(ring + 1, i + 1),
             d = p(ring + 1, i);
        mesh.triangles.push_back({a, b, c});
        mesh.triangles.push_back({a, c, d});
      }
  return mesh;
}
}  // namespace

namespace {
constexpr const char* line_vertex = R"GLSL(#version 150
in vec3 position;
in vec4 normal;
uniform vec3 centre, right, up;
uniform vec2 viewport, pan;
uniform float scale;
out vec4 line_colour;
void main() {
  vec3 p = position-centre;
  vec2 pixel = pan + scale*vec2(dot(p,right),-dot(p,up));
  gl_Position = vec4(2.0*pixel.x/viewport.x-1.0,1.0-2.0*pixel.y/viewport.y,0.0,1.0);
  line_colour = normal;
})GLSL";
constexpr const char* line_fragment = R"GLSL(#version 150
in vec4 line_colour;
out vec4 colour;
void main() { colour = line_colour; }
)GLSL";
}

struct Renderer::Impl {
  Gl gl;
  Buffer bounds{GL_RGBA32F, sizeof(F4)}, tree{GL_RGBA32I, sizeof(I4)},
      descriptors{GL_RGBA32I, sizeof(I4)}, samples{GL_R32F, sizeof(float)};
  GLuint program = 0, mesh_program = 0, vao = 0, mesh_vao = 0, mesh_vbo = 0;
  GLuint line_program = 0, line_vao = 0, line_vbo = 0;
  std::shared_ptr<const Lines> line_source;
  GLuint fbo = 0, image = 0, depth = 0;
  int width = 0, height = 0, columns = 0, rows = 0, tile_columns = 0;
  std::size_t texture_limit = 0;
  double origin_x = 0, origin_y = 0, pitch = 0;
  std::vector<F4> node_bounds;
  std::vector<I4> nodes, desc;
  std::vector<std::array<float, 2>> tile_heights;
  std::vector<int> parents, leaves;
  std::vector<float> data;
  std::map<int, std::vector<int>> free_slots;
  std::vector<TileSlots> slots;
  std::vector<std::shared_ptr<const Heightmap::Page>> pages;
  Stats counters;
  Camera drawn_camera;
  Cutter drawn_cutter;
  bool frame_valid = false;
  bool initialized = false;
  Impl() {
    GLint n;
    glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, &n);
    texture_limit = std::max(n, 0);
  }
  ~Impl() {
    bounds.release(gl);
    tree.release(gl);
    descriptors.release(gl);
    samples.release(gl);
    gl.DeleteProgram(program);
    gl.DeleteProgram(mesh_program);
    gl.DeleteProgram(line_program);
    gl.DeleteVertexArrays(1, &line_vao);
    gl.DeleteBuffers(1, &line_vbo);
    gl.DeleteVertexArrays(1, &vao);
    gl.DeleteVertexArrays(1, &mesh_vao);
    gl.DeleteBuffers(1, &mesh_vbo);
    gl.DeleteFramebuffers(1, &fbo);
    glDeleteTextures(1, &image);
    glDeleteTextures(1, &depth);
  }
  GLuint shader(GLenum kind, const char* source) {
    GLuint s = gl.CreateShader(kind);
    gl.ShaderSource(s, 1, &source, nullptr);
    gl.CompileShader(s);
    GLint ok;
    gl.GetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
      GLint n;
      gl.GetShaderiv(s, GL_INFO_LOG_LENGTH, &n);
      std::string log(std::max(n, 1), 0);
      gl.GetShaderInfoLog(s, n, nullptr, log.data());
      gl.DeleteShader(s);
      throw std::runtime_error("Heightmap shader: " + log);
    }
    return s;
  }
  GLuint link(const char* vs, const char* fs) {
    GLuint a = shader(GL_VERTEX_SHADER, vs), b = 0, p = 0;
    try {
      b = shader(GL_FRAGMENT_SHADER, fs);
      p = gl.CreateProgram();
      gl.AttachShader(p, a);
      gl.AttachShader(p, b);
      gl.BindAttribLocation(p, 0, "position");
      gl.BindAttribLocation(p, 1, "normal");
      gl.BindFragDataLocation(p, 0, "colour");
      gl.LinkProgram(p);
      GLint ok;
      gl.GetProgramiv(p, GL_LINK_STATUS, &ok);
      if (!ok) {
        GLint n;
        gl.GetProgramiv(p, GL_INFO_LOG_LENGTH, &n);
        std::string log(std::max(n, 1), 0);
        gl.GetProgramInfoLog(p, n, nullptr, log.data());
        throw std::runtime_error("Heightmap program: " + log);
      }
    } catch (...) {
      gl.DeleteShader(a);
      gl.DeleteShader(b);
      gl.DeleteProgram(p);
      throw;
    }
    gl.DeleteShader(a);
    gl.DeleteShader(b);
    return p;
  }
  void init() {
    if (initialized) return;
    program = link(fullscreen_vertex,
                   (std::string(heightmap_fragment) + surface_lighting).c_str());
    mesh_program = link(mesh_vertex,
                        (std::string(mesh_fragment) + surface_lighting).c_str());
    line_program = link(line_vertex, line_fragment);
    gl.GenVertexArrays(1, &line_vao);
    gl.GenBuffers(1, &line_vbo);
    gl.BindVertexArray(line_vao);
    gl.BindBuffer(GL_ARRAY_BUFFER, line_vbo);
    gl.EnableVertexAttribArray(0);
    gl.EnableVertexAttribArray(1);
    gl.VertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(LineVertex), nullptr);
    gl.VertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(LineVertex),
                           reinterpret_cast<void*>(offsetof(LineVertex, colour)));
    gl.GenVertexArrays(1, &vao);
    gl.GenVertexArrays(1, &mesh_vao);
    gl.GenBuffers(1, &mesh_vbo);
    gl.BindVertexArray(mesh_vao);
    gl.BindBuffer(GL_ARRAY_BUFFER, mesh_vbo);
    gl.EnableVertexAttribArray(0);
    gl.EnableVertexAttribArray(1);
    gl.VertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
    gl.VertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                           reinterpret_cast<void*>(sizeof(Vec3f)));
    gl.GenFramebuffers(1, &fbo);
    glGenTextures(1, &image);
    glGenTextures(1, &depth);
    initialized = true;
  }
  int build(int x0, int y0, int x1, int y1, int parent) {
    const int n = nodes.size();
    nodes.push_back({-1, -1, -1, 0});
    parents.push_back(parent);
    node_bounds.push_back(
        {float(x0 * 32 * pitch), float(y0 * 32 * pitch), 1, 0});
    node_bounds.push_back({float(std::min(columns, x1 * 32) * pitch),
                           float(std::min(rows, y1 * 32) * pitch), 0, 0});
    if (x1 - x0 == 1 && y1 - y0 == 1) {
      const int tile = y0 * tile_columns + x0;
      nodes[n][2] = tile;
      leaves[tile] = n;
    } else if (x1 - x0 >= y1 - y0) {
      const int m = (x0 + x1) / 2;
      const int a = build(x0, y0, m, y1, n), b = build(m, y0, x1, y1, n);
      nodes[n][0] = a;
      nodes[n][1] = b;
    } else {
      const int m = (y0 + y1) / 2;
      const int a = build(x0, y0, x1, m, n), b = build(x0, m, x1, y1, n);
      nodes[n][0] = a;
      nodes[n][1] = b;
    }
    return n;
  }
  Slot allocate(int n) {
    auto& free = free_slots[n];
    if (!free.empty()) {
      int offset = free.back();
      free.pop_back();
      return {offset, n};
    }
    Slot s{int(data.size()), n};
    data.resize(data.size() + n);
    return s;
  }
  int store(const Heightmap::Heights& h, Slot& slot) {
    int mode = h.samples ? 3 : h.pattern ? (h.columns ? 2 : 1) : 0;
    int n = mode == 3 ? 1024 : mode ? 32 : 1;
    if (slot.size != n) {
      if (slot.size) free_slots[slot.size].push_back(slot.offset);
      slot = allocate(n);
    }
    if (h.samples)
      std::copy(h.samples->begin(), h.samples->end(),
                data.begin() + slot.offset);
    else if (h.pattern)
      std::copy(h.pattern->begin(), h.pattern->end(),
                data.begin() + slot.offset);
    else
      data[slot.offset] = h.uniform;
    return mode;
  }
  void update(const Heightmap& stock) {
    counters.changed_tiles = counters.uploaded_bytes = 0;
    counters.reused_frame = false;
    bool reset = columns != stock.columns() || rows != stock.rows() ||
                 pitch != stock.pitch() || origin_x != stock.lower_x() ||
                 origin_y != stock.lower_y();
    columns = stock.columns();
    rows = stock.rows();
    pitch = stock.pitch();
    origin_x = stock.lower_x();
    origin_y = stock.lower_y();
    auto incoming = stock.tile_pages();
    if (reset) {
      frame_valid = false;
      pages.clear();
      nodes.clear();
      node_bounds.clear();
      parents.clear();
      leaves.clear();
      data.clear();
      free_slots.clear();
      tile_columns = (columns + 31) / 32;
      const int tile_rows = (rows + 31) / 32;
      slots.assign(std::size_t(tile_columns) * tile_rows, {});
      desc.assign(slots.size(), {});
      tile_heights.assign(slots.size(), {1, 0});
      leaves.resize(slots.size());
      if (!slots.empty()) build(0, 0, tile_columns, tile_rows, -1);
    }
    std::vector<int> changed;
    for (std::size_t p = 0; p < incoming.size(); ++p) {
      if (p < pages.size() && incoming[p] == pages[p]) continue;
      for (int k = 0; k < Heightmap::page_width; ++k) {
        const auto i = p * Heightmap::page_width + k;
        if (i >= slots.size()) break;
        const auto& tile = incoming[p]->tiles[k];
        if (p < pages.size() && tile == pages[p]->tiles[k]) continue;
        changed.push_back(i);
        auto& d = desc[i];
        auto& s = slots[i];
        d = {store(tile->top, s.top), s.top.offset,
             store(tile->bottom, s.bottom), s.bottom.offset};
        float bottom = std::numeric_limits<float>::infinity();
        const auto& h = tile->bottom;
        if (h.samples)
          for (float v : *h.samples)
            if (std::isfinite(v)) bottom = std::min(bottom, v);
        if (h.pattern)
          for (float v : *h.pattern)
            if (std::isfinite(v)) bottom = std::min(bottom, v);
        if (!h.samples && !h.pattern && std::isfinite(h.uniform))
          bottom = h.uniform;
        tile_heights[i] = bottom < tile->maximum_top
            ? std::array<float, 2>{bottom, tile->maximum_top}
            : std::array<float, 2>{1, 0};
      }
    }
    // Interpolation reaches half a cell into neighbouring tiles. Their height
    // ranges must participate in culling, including after a neighbour changes.
    std::vector<int> affected;
    const int tile_rows = (rows + 31) / 32;
    for (int tile : changed)
      for (int y = std::max(0, tile / tile_columns - 1);
           y <= std::min(tile_rows - 1, tile / tile_columns + 1); ++y)
        for (int x = std::max(0, tile % tile_columns - 1);
             x <= std::min(tile_columns - 1, tile % tile_columns + 1); ++x)
          affected.push_back(y * tile_columns + x);
    std::sort(affected.begin(), affected.end());
    affected.erase(std::unique(affected.begin(), affected.end()), affected.end());
    for (int tile : affected) {
      auto& lo = node_bounds[leaves[tile] * 2];
      auto& hi = node_bounds[leaves[tile] * 2 + 1];
      lo[2] = tile_heights[tile][0];
      hi[2] = tile_heights[tile][1];
      bool flat = desc[tile][0] == 0 && desc[tile][2] == 0;
      for (int y = std::max(0, tile / tile_columns - 1);
           y <= std::min(tile_rows - 1, tile / tile_columns + 1); ++y)
        for (int x = std::max(0, tile % tile_columns - 1);
             x <= std::min(tile_columns - 1, tile % tile_columns + 1); ++x) {
          const int other = y * tile_columns + x;
          const auto& range = tile_heights[other];
          if (range[1] > range[0]) {
            if (hi[2] <= lo[2]) {
              lo[2] = range[0];
              hi[2] = range[1];
            } else {
              lo[2] = std::min(lo[2], range[0]);
              hi[2] = std::max(hi[2], range[1]);
            }
          }
          flat &= desc[other][0] == 0 && desc[other][2] == 0 &&
                  range == tile_heights[tile];
        }
      // Empty tiles only need the half-cell halo of occupied border samples,
      // rather than a whole tile of expensive empty-space ray traversal.
      const int x0=(tile%tile_columns)*32,y0=(tile/tile_columns)*32;
      const int x1=std::min(x0+32,columns),y1=std::min(y0+32,rows);
      lo[0]=float(x0*pitch);lo[1]=float(y0*pitch);
      hi[0]=float(x1*pitch);hi[1]=float(y1*pitch);
      if (!(tile_heights[tile][1]>tile_heights[tile][0])) {
        lo={float(x1*pitch),float(y1*pitch),1,0};
        hi={float(x0*pitch),float(y0*pitch),0,0};
        for(int y=std::max(0,y0-1);y<=std::min(rows-1,y1);++y)
          for(int x=std::max(0,x0-1);x<=std::min(columns-1,x1);++x) {
            if(x>=x0 && x<x1 && y>=y0 && y<y1) continue;
            const int other=(y/32)*tile_columns+x/32;
            const auto& neighbour=incoming[other/Heightmap::page_width]
                ->tiles[other%Heightmap::page_width];
            const auto index=(y%32)*32+x%32;
            const float low=neighbour->bottom.at(index),high=neighbour->top.at(index);
            if(!(high>low)) continue;
            const float lx=float(std::max(double(x0),x-.5)*pitch),
                        ly=float(std::max(double(y0),y-.5)*pitch),
                        hx=float(std::min(double(x1),x+1.5)*pitch),
                        hy=float(std::min(double(y1),y+1.5)*pitch);
            if(hi[2]<=lo[2]) {lo={lx,ly,low,0};hi={hx,hy,high,0};}
            else {
              lo[0]=std::min(lo[0],lx);lo[1]=std::min(lo[1],ly);lo[2]=std::min(lo[2],low);
              hi[0]=std::max(hi[0],hx);hi[1]=std::max(hi[1],hy);hi[2]=std::max(hi[2],high);
            }
          }
        flat=false;
      }
      nodes[leaves[tile]][3] = flat ? 1 : 0;
    }
    std::vector<int> dirty;
    for (int i : affected)
      for (int n = leaves[i]; n >= 0; n = parents[n]) dirty.push_back(n);
    std::sort(dirty.begin(), dirty.end(), std::greater<int>());
    dirty.erase(std::unique(dirty.begin(), dirty.end()), dirty.end());
    for (int n : dirty)
      if (nodes[n][2] < 0) {
        const int a = nodes[n][0], b = nodes[n][1];
        auto& lo = node_bounds[n * 2];
        auto& hi = node_bounds[n * 2 + 1];
        const bool va = node_bounds[a * 2 + 1][2] > node_bounds[a * 2][2],
                   vb = node_bounds[b * 2 + 1][2] > node_bounds[b * 2][2];
        for(int axis=0;axis<2;++axis) {
          lo[axis]=va?node_bounds[a*2][axis]:vb?node_bounds[b*2][axis]:0;
          hi[axis]=va?node_bounds[a*2+1][axis]:vb?node_bounds[b*2+1][axis]:0;
          if(va && vb) {
            lo[axis]=std::min(lo[axis],node_bounds[b*2][axis]);
            hi[axis]=std::max(hi[axis],node_bounds[b*2+1][axis]);
          }
        }
        lo[2] = va ? node_bounds[a * 2][2] : vb ? node_bounds[b * 2][2] : 1;
        hi[2] = va   ? node_bounds[a * 2 + 1][2]
                : vb ? node_bounds[b * 2 + 1][2]
                     : 0;
        if (va && vb) {
          lo[2] = std::min(lo[2], node_bounds[b * 2][2]);
          hi[2] = std::max(hi[2], node_bounds[b * 2 + 1][2]);
        }
      }
    // Merge adjacent dirty ranges: playback uploads changed slots/ancestors,
    // without one driver call per tile, while camera changes upload nothing.
    auto full = [&](Buffer& b, const auto& v) {
      b.upload(gl, 0, v.size(), v.data());
      counters.uploaded_bytes += v.size() * sizeof(v[0]);
    };
    auto ranges = [&](Buffer& buffer, const auto& values,
                      std::vector<std::pair<std::size_t, std::size_t>> spans) {
      if (spans.empty()) return;
      std::sort(spans.begin(), spans.end());
      std::size_t begin = spans[0].first, end = spans[0].second;
      auto send = [&] {
        buffer.upload(gl, begin, end - begin, values.data() + begin);
        counters.uploaded_bytes += (end - begin) * sizeof(values[0]);
      };
      for (std::size_t i = 1; i < spans.size(); ++i) {
        if (spans[i].first <= end)
          end = std::max(end, spans[i].second);
        else {
          send();
          begin = spans[i].first;
          end = spans[i].second;
        }
      }
      send();
    };
    gl.ActiveTexture(GL_TEXTURE0);
    bool grow = samples.reserve(gl, std::max(std::size_t{1}, data.size()),
                                texture_limit);
    if (reset || grow)
      full(samples, data);
    else {
      std::vector<std::pair<std::size_t, std::size_t>> spans;
      for (int i : changed)
        for (const auto s : {slots[i].top, slots[i].bottom})
          spans.emplace_back(s.offset, s.offset + s.size);
      ranges(samples, data, std::move(spans));
    }
    grow = descriptors.reserve(gl, std::max(std::size_t{1}, desc.size()),
                               texture_limit);
    if (reset || grow)
      full(descriptors, desc);
    else {
      std::vector<std::pair<std::size_t, std::size_t>> spans;
      for (int i : changed) spans.emplace_back(i, i + 1);
      ranges(descriptors, desc, std::move(spans));
    }
    grow =
        tree.reserve(gl, std::max(std::size_t{1}, nodes.size()), texture_limit);
    if (reset || grow) full(tree, nodes);
    else {
      std::vector<std::pair<std::size_t, std::size_t>> spans;
      for (int i : affected) spans.emplace_back(leaves[i], leaves[i] + 1);
      ranges(tree, nodes, std::move(spans));
    }
    grow = bounds.reserve(gl, std::max(std::size_t{1}, node_bounds.size()),
                          texture_limit);
    if (reset || grow)
      full(bounds, node_bounds);
    else {
      std::vector<std::pair<std::size_t, std::size_t>> spans;
      for (int n : dirty) spans.emplace_back(n * 2, n * 2 + 2);
      ranges(bounds, node_bounds, std::move(spans));
    }
    pages = std::move(incoming);
    counters.changed_tiles = changed.size();
    counters.tiles = slots.size();
    counters.hierarchy_nodes = nodes.size();
    counters.resident_bytes =
        samples.capacity * sizeof(float) + descriptors.capacity * sizeof(I4) +
        tree.capacity * sizeof(I4) + bounds.capacity * sizeof(F4) +
        std::size_t(width) * height * 8;
  }
  bool same_stock(const Heightmap& stock) const {
    return columns == stock.columns() && rows == stock.rows() &&
           pitch == stock.pitch() && origin_x == stock.lower_x() &&
           origin_y == stock.lower_y() && pages == stock.tile_pages();
  }
  void target(int w, int h) {
    gl.BindFramebuffer(GL_FRAMEBUFFER, fbo);
    if (w == width && h == height) return;
    frame_valid = false;
    gl.BindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    gl.ActiveTexture(GL_TEXTURE0);
    for (auto [texture, format, type, internal] :
         {std::array<GLenum, 4>{image, GL_RGBA, GL_UNSIGNED_BYTE, GL_RGBA8},
          std::array<GLenum, 4>{depth, GL_DEPTH_COMPONENT, GL_FLOAT,
                                GL_DEPTH_COMPONENT32F}}) {
      glBindTexture(GL_TEXTURE_2D, texture);
      glTexImage2D(GL_TEXTURE_2D, 0, internal, w, h, 0, format, type, nullptr);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      gl.FramebufferTexture2D(
          GL_FRAMEBUFFER,
          texture == image ? GL_COLOR_ATTACHMENT0 : GL_DEPTH_ATTACHMENT,
          GL_TEXTURE_2D, texture, 0);
    }
    if (gl.CheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
      throw std::runtime_error("Heightmap framebuffer is incomplete");
    width = w;
    height = h;
  }
  void camera(GLuint p, const Camera& c) {
    for (auto [name, value] : {std::pair{"centre", c.centre},
                               {"right", c.right},
                               {"up", c.up},
                               {"toward", c.toward},
                               {"light", c.light}})
      gl.Uniform3f(gl.GetUniformLocation(p, name), value.x, value.y, value.z);
    gl.Uniform2f(gl.GetUniformLocation(p, "pan"), c.pan_x, c.pan_y);
    gl.Uniform2f(gl.GetUniformLocation(p, "viewport"), width, height);
    gl.Uniform1f(gl.GetUniformLocation(p, "scale"), c.scale);
    gl.Uniform1f(gl.GetUniformLocation(p, "reach"), c.reach);
  }
  void lines(const std::shared_ptr<const Lines>& source, const Camera& c) {
    gl.BindVertexArray(line_vao);
    gl.BindBuffer(GL_ARRAY_BUFFER, line_vbo);
    if (line_source != source) {
      const std::size_t bytes = source ? source->vertices.size()*sizeof(LineVertex) : 0;
      gl.BufferData(GL_ARRAY_BUFFER, bytes, bytes ? source->vertices.data() : nullptr, GL_STATIC_DRAW);
      counters.line_upload_bytes = bytes;
      line_source = source;
    }
    if (!source || source->vertices.empty()) return;
    gl.UseProgram(line_program);
    camera(line_program, c);
    // Paths are an overlay, matching the existing preview's visibility.
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDepthMask(GL_FALSE);
    glDrawArrays(GL_LINES, 0, source->vertices.size());
  }
  void mesh(const MeshDraw& draw, const Camera& c) {
    if (!draw.mesh || draw.mesh->triangles.empty()) return;
    auto vertices = mesh_vertices(*draw.mesh);
    gl.UseProgram(mesh_program);
    camera(mesh_program, c);
    gl.Uniform3f(gl.GetUniformLocation(mesh_program, "offset"), draw.offset.x,
                 draw.offset.y, draw.offset.z);
    gl.Uniform4fv(gl.GetUniformLocation(mesh_program, "base_colour"), 1,
                  draw.colour.data());
    gl.BindVertexArray(mesh_vao);
    gl.BindBuffer(GL_ARRAY_BUFFER, mesh_vbo);
    gl.BufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex),
                  vertices.data(), GL_STREAM_DRAW);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    if (draw.colour[3] < 1) {
      glEnable(GL_BLEND);
      gl.BlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE,
                           GL_ONE_MINUS_SRC_ALPHA);
      gl.BlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
    } else
      glDisable(GL_BLEND);
    glDepthMask(draw.colour[3] >= 1);
    glPolygonMode(GL_FRONT_AND_BACK, draw.wireframe ? GL_LINE : GL_FILL);
    glDrawArrays(GL_TRIANGLES, 0, vertices.size());
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }
};
Renderer::Renderer() : impl_(std::make_unique<Impl>()) {}
Renderer::~Renderer() = default;
unsigned Renderer::render(const Heightmap& stock, const Camera& camera,
                          int width, int height,
                          const std::vector<MeshDraw>& meshes,
                          const Cutter& cutter,
                          const std::shared_ptr<const Lines>& lines) {
  if (width <= 0 || height <= 0 || !std::isfinite(camera.scale) ||
      camera.scale <= 0 || !std::isfinite(camera.reach) || camera.reach <= 0)
    throw std::invalid_argument("Invalid heightmap viewport or camera");
  auto& r = *impl_;
  r.counters.line_upload_bytes = 0;
  if (r.frame_valid && r.width == width && r.height == height && meshes.empty() && r.line_source == lines &&
      same_camera(r.drawn_camera,camera) && same_cutter(r.drawn_cutter,cutter) &&
      r.same_stock(stock)) {
    r.counters.changed_tiles = r.counters.uploaded_bytes = 0;
    r.counters.reused_frame = true;
    return r.image;
  }
  State saved(r.gl);
  r.init();
  r.target(width, height);
  r.update(stock);
  // Mutable mesh pointers have no revision identifier, so frames with mesh
  // overlays always redraw. Stock pages and the cutter/camera inputs have
  // complete change detection and can safely reuse the rendered texture.
  if (r.frame_valid && meshes.empty() && r.line_source == lines && r.counters.changed_tiles == 0 &&
      same_camera(r.drawn_camera,camera) && same_cutter(r.drawn_cutter,cutter)) {
    r.counters.reused_frame = true;
    return r.image;
  }
  r.frame_valid = false;
  glViewport(0, 0, width, height);
  glDisable(GL_SCISSOR_TEST);
  glDisable(GL_BLEND);
  glDisable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glDepthMask(GL_TRUE);
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  const float clear_depth = 1;
  r.gl.ClearBufferfv(GL_DEPTH, 0, &clear_depth);
  r.gl.ClearBufferfv(GL_COLOR, 0, camera.background.data());
  r.gl.UseProgram(r.program);
  r.camera(r.program, camera);
  const char* names[]{"bounds_data", "tree_data", "tile_data", "heights"};
  Buffer* buffers[]{&r.bounds, &r.tree, &r.descriptors, &r.samples};
  for (int i = 0; i < 4; ++i) {
    r.gl.ActiveTexture(GL_TEXTURE0 + i);
    glBindTexture(GL_TEXTURE_BUFFER, buffers[i]->texture);
    r.gl.Uniform1i(r.gl.GetUniformLocation(r.program, names[i]), i);
  }
  r.gl.Uniform2f(r.gl.GetUniformLocation(r.program, "grid_origin"), r.origin_x,
                 r.origin_y);
  r.gl.Uniform2i(r.gl.GetUniformLocation(r.program, "grid_size"), r.columns,
                 r.rows);
  r.gl.Uniform1f(r.gl.GetUniformLocation(r.program, "pitch"), r.pitch);
  r.gl.Uniform1i(r.gl.GetUniformLocation(r.program, "empty_grid"),
                 r.nodes.empty());
  r.gl.BindVertexArray(r.vao);
  glDrawArrays(GL_TRIANGLES, 0, 3);
  for (const auto& mesh : meshes) r.mesh(mesh, camera);
  if (cutter.radius > 0 && (cutter.ball?cutter.length>=cutter.radius:cutter.length>0)) {
    auto shape = cutter_mesh(cutter);
    r.mesh({&shape, cutter.tip, {.48f, .86f, 1.f, .38f}, false}, camera);
  }
  r.lines(lines, camera);
  if (auto error = glGetError(); error != GL_NO_ERROR)
    throw std::runtime_error("OpenGL heightmap rendering failed (" +
                             std::to_string(error) + ")");
  r.drawn_camera = camera;
  r.drawn_cutter = cutter;
  r.frame_valid = meshes.empty();
  return r.image;
}
Stats Renderer::stats() const { return impl_->counters; }
std::vector<std::uint8_t> Renderer::read_colour() const {
  auto& r = *impl_;
  State saved(r.gl);
  r.gl.BindFramebuffer(GL_READ_FRAMEBUFFER, r.fbo);
  r.gl.BindBuffer(GL_PIXEL_PACK_BUFFER, 0);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ROW_LENGTH, 0);
  glPixelStorei(GL_PACK_SKIP_ROWS, 0);
  glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
  std::vector<std::uint8_t> pixels(std::size_t(r.width) * r.height * 4);
  glReadPixels(0, 0, r.width, r.height, GL_RGBA, GL_UNSIGNED_BYTE,
               pixels.data());
  return pixels;
}
std::vector<float> Renderer::read_depth() const {
  auto& r = *impl_;
  State saved(r.gl);
  r.gl.BindFramebuffer(GL_READ_FRAMEBUFFER, r.fbo);
  r.gl.BindBuffer(GL_PIXEL_PACK_BUFFER, 0);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ROW_LENGTH, 0);
  glPixelStorei(GL_PACK_SKIP_ROWS, 0);
  glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
  std::vector<float> pixels(std::size_t(r.width) * r.height);
  glReadPixels(0, 0, r.width, r.height, GL_DEPTH_COMPONENT, GL_FLOAT,
               pixels.data());
  return pixels;
}
}  // namespace wave::heightfield
