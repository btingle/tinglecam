#include <SDL.h>
#include <SDL_opengl.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "rendering/heightmap_renderer.h"

using namespace wave;
using namespace wave::heightfield;
void check(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
void near(double a, double b, double e, const char* message) {
  if (std::abs(a - b) > e)
    throw std::runtime_error(std::string(message) + ": " + std::to_string(a) +
                             " / " + std::to_string(b));
}
// Independent brute-force reference: intersect every occupied stock column,
// with no renderer hierarchy, compression, DDA, shader, or mesh involved.
double reference(const Heightmap& stock, Vec3d origin, Vec3d direction,
                 double limit) {
  double hit = limit;
  for (int row = 0; row < stock.rows(); ++row)
    for (int col = 0; col < stock.columns(); ++col) {
      const double low = stock.bottom(col, row), high = stock.top(col, row);
      if (!std::isfinite(low) || !std::isfinite(high) || high <= low) continue;
      const double lo[]{stock.lower_x() + col * stock.pitch(),
                        stock.lower_y() + row * stock.pitch(), low};
      const double hi[]{lo[0] + stock.pitch(), lo[1] + stock.pitch(), high};
      const double ro[]{origin.x, origin.y, origin.z},
          rd[]{direction.x, direction.y, direction.z};
      double a = -std::numeric_limits<double>::infinity(), b = hit;
      bool okay = true;
      for (int k = 0; k < 3; ++k) {
        if (std::abs(rd[k]) < 1e-12) {
          if (ro[k] < lo[k] || ro[k] > hi[k]) okay = false;
        } else {
          double p = (lo[k] - ro[k]) / rd[k], q = (hi[k] - ro[k]) / rd[k];
          if (p > q) std::swap(p, q);
          a = std::max(a, p);
          b = std::min(b, q);
        }
      }
      if (okay && a <= b && b >= 0) hit = std::min(hit, a >= 0 ? a : b);
    }
  return hit;
}
void compare(Renderer& renderer, const Heightmap& stock, const Camera& c,
             int w = 43, int h = 37) {
  renderer.render(stock, c, w, h);
  const auto depths = renderer.read_depth();
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const double px = (x + .5 - c.pan_x) / c.scale,
                   py = (h - y - .5 - c.pan_y) / c.scale;
      const Vec3d ro{
          c.centre.x + c.right.x * px - c.up.x * py + c.toward.x * c.reach,
          c.centre.y + c.right.y * px - c.up.y * py + c.toward.y * c.reach,
          c.centre.z + c.right.z * px - c.up.z * py + c.toward.z * c.reach};
      const auto t = reference(
          stock, ro, {-c.toward.x, -c.toward.y, -c.toward.z}, 2 * c.reach);
      near(depths[std::size_t(y) * w + x], t / (2 * c.reach), 3e-5,
           "GPU stock intersection matches independent column reference");
    }
}
void benchmark(int width = 640, int height = 480) {
  using Clock = std::chrono::steady_clock;
  for (double pitch : {.1, .025, .01}) {
    auto stock = Heightmap::box({{0, 0, 0}, {40, 40, 8}, true}, pitch);
    for (double x : {10., 20., 30.})
      stock.cut({x, -3, 2}, {x, 43, 2}, {CutterShape::Ball, 2, 20});
    stock.compact_history();
    Renderer renderer;
    const float s = 1 / std::sqrt(3.f), r = 1 / std::sqrt(2.f),
                u = 1 / std::sqrt(6.f);
    Camera camera{{20, 20, 4}, {r, -r, 0}, {-u, -u, 2 * u},
                  {s, s, s},   {0, 0, 1},  7.f * height / 480,
                  200, float(width) / 2, float(height) / 2};
    const auto start = Clock::now();
    renderer.render(stock, camera, width, height);
    glFinish();
    const double first =
        std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    const auto warm = Clock::now();
    for (int i = 0; i < 8; ++i) {
      camera.pan_x = float(width) / 2 + i * .1f;
      renderer.render(stock, camera, width, height);
      glFinish();
    }
    const double frame =
        std::chrono::duration<double, std::milli>(Clock::now() - warm).count() /
        8;
    std::cout << "render_benchmark pitch=" << pitch
              << " cells=" << stock.columns() * std::size_t(stock.rows())
              << " viewport=" << width << "x" << height << " first_ms=" << first << " frame_ms=" << frame
              << " resident_bytes=" << renderer.stats().resident_bytes
              << " unchanged_upload_bytes=" << renderer.stats().uploaded_bytes
              << '\n';
  }
}
int main(int argc, char** argv) {
  SDL_Window* window = nullptr;
  SDL_GLContext context = nullptr;
  int result = 0;
  try {
    if (!SDL_getenv("SDL_VIDEODRIVER"))
      SDL_setenv("SDL_VIDEODRIVER", "offscreen", 1);
    check(SDL_Init(SDL_INIT_VIDEO) == 0, SDL_GetError());
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,
                        SDL_GL_CONTEXT_PROFILE_CORE);
    window = SDL_CreateWindow("heightmap renderer tests", 0, 0, 64, 64,
                              SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
    check(window, SDL_GetError());
    context = SDL_GL_CreateContext(window);
    check(context, SDL_GetError());
    std::cout << "OpenGL renderer: " << glGetString(GL_RENDERER) << "; "
              << glGetString(GL_VERSION) << '\n';
    {
      Renderer renderer;
      auto stock = Heightmap::box({{-2, -3, 0}, {4, 4, 6}, true}, .5);
      stock.cut({-2, 0, 2}, {4, 0, 2}, {CutterShape::Ball, 1.5, 20});
      stock.set(3, 3, 0, 0);
      stock.set(4, 3, 0, 0);
      stock.set(0, 0, 1, 5);
      Camera c{{1, .5f, 3}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 1},
               4,           40,        21.3f,     18.1f};
      compare(renderer, stock, c);
      c.right = {0, 1, 0};
      c.up = {0, 0, 1};
      c.toward = {1, 0, 0};
      compare(renderer, stock, c);
      c.right = {1, 0, 0};
      c.up = {0, 0, 1};
      c.toward = {0, -1, 0};
      compare(renderer, stock, c);
      c.right = {1, 0, 0};
      c.up = {0, -1, 0};
      c.toward = {0, 0, -1};
      compare(renderer, stock, c);
      const float s = 1 / std::sqrt(3.f), r = 1 / std::sqrt(2.f),
                  u = 1 / std::sqrt(6.f);
      c.right = {r, -r, 0};
      c.up = {u, u, -2 * u};
      c.toward = {s, s, s};
      compare(renderer, stock, c);
      c.toward = {-s, -s, s};
      c.right = {-r, r, 0};
      c.up = {-u, -u, -2 * u};
      compare(renderer, stock, c);
      c.right = {1, 0, -1e-8f};
      c.up = {0, 1, 0};
      c.toward = {1e-8f, 0, 1};
      compare(renderer, stock, c);
      // Camera movement and unchanged snapshots must not resend the grid.
      renderer.render(stock, c, 43, 37);
      check(renderer.stats().changed_tiles == 0 &&
                renderer.stats().uploaded_bytes == 0,
            "Unchanged stock uploads no tile data");
      auto snapshot = Heightmap::box({{0, 0, 0}, {8, 8, 8}, true}, .125);
      renderer.render(snapshot, c, 43, 37);
      check(renderer.stats().changed_tiles == 4,
            "Initial grid uploads its four tiles");
      auto edited = snapshot;
      edited.set(40, 40, 0, 1);
      renderer.render(edited, c, 43, 37);
      check(renderer.stats().changed_tiles == 1,
            "One edited cell uploads one changed tile");
      renderer.render(snapshot, c, 43, 37);
      check(renderer.stats().changed_tiles == 1,
            "Seeking backward restores only the changed tile");
      // Repeated-row compression is read directly and preserves the surface.
      auto patterned = Heightmap::box({{0, 0, 0}, {8, 8, 8}, true}, .125);
      patterned.cut({4, -2, 2}, {4, 10, 2}, {CutterShape::Ball, 2, 20});
      patterned.compact_history();
      compare(renderer, patterned, c, 31, 29);
      auto horizontal = Heightmap::box({{0, 0, 0}, {8, 8, 8}, true}, .125);
      horizontal.cut({-2, 4, 2}, {10, 4, 2}, {CutterShape::Ball, 2, 20});
      horizontal.compact_history();
      compare(renderer, horizontal, c, 31, 29);
      auto partial =
          Heightmap::box({{-2, -1, 0}, {7.35f, 8.3f, 6}, true}, .125);
      partial.cut({-3, -2, 1}, {9, 10, 4}, {CutterShape::Ball, 1, 20});
      c.right = {r, -r, 0};
      c.up = {u, u, -2 * u};
      c.toward = {s, s, s};
      compare(renderer, partial, c, 31, 29);
      // The stock's depth attachment must also occlude ordinary GPU meshes.
      auto block = Heightmap::box({{-2, -2, 0}, {2, 2, 4}, true}, .25);
      c = {{0, 0, 2}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 1},
           6,         40,        21.3f,     18.1f};
      renderer.render(block, c, 43, 37);
      const auto plain = renderer.read_colour();
      Mesh triangle;
      triangle.triangles.push_back({{-1, -1, 2}, {1, -1, 2}, {0, 1, 2}});
      renderer.render(block, c, 43, 37, {{&triangle, {}, {1, 0, 0, 1}, false}});
      check(renderer.read_colour() == plain,
            "A mesh buried behind stock cannot draw through it");
      for (auto& t : triangle.triangles) {
        t.a.z = t.b.z = t.c.z = 5;
      }
      renderer.render(block, c, 43, 37, {{&triangle, {}, {1, 0, 0, 1}, false}});
      check(renderer.read_colour() != plain,
            "A mesh in front of stock is visible");
      near(renderer.read_depth()[18 * 43 + 21], (40. - 3.) / 80, 1e-6,
           "Stock and mesh share the same depth convention");
      renderer.render(block, c, 43, 37, {}, {{0, 0, 1}, .8f, 1, true});
      check(renderer.read_colour() == plain,
            "A buried ball cutter is occluded by stock");
      renderer.render(block, c, 43, 37, {}, {{0, 0, 4}, .8f, 4, true});
      check(renderer.read_colour() != plain,
            "An exposed ball cutter is visible");
      // Uniform stock stays compact even at 100 million simulation cells.
      auto huge = Heightmap::box({{0, 0, 0}, {100, 100, 10}, true}, .01);
      c = {{50, 50, 5}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 1},
           .3f,         200,       16,        16};
      renderer.render(huge, c, 32, 32);
      check(renderer.stats().resident_bytes < 64 * 1024 * 1024,
            "Fine uniform stock does not expand into a dense GPU height "
            "texture or mesh");
      near(renderer.read_depth()[16 * 32 + 16], 195. / 400, 1e-6,
           "Large compact grid retains its actual surface");
      std::cout << "100-million-cell GPU payload: "
                << renderer.stats().resident_bytes << " bytes\n";
      // State restoration matters because ImGui draws into a different target.
      glViewport(3, 4, 23, 27);
      glEnable(GL_SCISSOR_TEST);
      glScissor(0, 0, 1, 1);
      renderer.render(block, c, 43, 37);
      GLint viewport[4];
      glGetIntegerv(GL_VIEWPORT, viewport);
      check(viewport[0] == 3 && viewport[1] == 4 && viewport[2] == 23 &&
                viewport[3] == 27 && glIsEnabled(GL_SCISSOR_TEST),
            "Renderer restores caller viewport and clipping state");
      Heightmap empty;
      renderer.render(empty, c, 43, 37);
      for (float d : renderer.read_depth())
        near(d, 1, 1e-7, "Empty grids leave the depth buffer clear");
      check(glGetError() == GL_NO_ERROR, "Renderer leaves no OpenGL errors");
    }
    std::cout << "Native heightmap raycasting, compression, tile updates and "
                 "depth compositing passed.\n";
    if (argc > 1 && std::string(argv[1]) == "--benchmark") {
      int width=argc>2?std::stoi(argv[2]):640,height=argc>3?std::stoi(argv[3]):480;
      check(width>0 && height>0 && width<=16384 && height<=16384,"Invalid benchmark viewport");
      benchmark(width,height);
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    result = 1;
  }
  if (context) SDL_GL_DeleteContext(context);
  if (window) SDL_DestroyWindow(window);
  SDL_Quit();
  return result;
}
