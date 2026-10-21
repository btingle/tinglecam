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
// Independent reference for horizontal cap levels: exhaustively intersect
// every sample-centred XY patch and every height band. In a band, the four
// samples define a bilinear material fraction; its 50% isoline is the wall.
// No renderer hierarchy, compressed data, DDA or shader code participates.
double reference(const Heightmap& stock, Vec3d origin, Vec3d direction,
                 double limit) {
  double hit=limit;
  const double pitch=stock.pitch();
  for(int row=-1;row<stock.rows();++row)
    for(int col=-1;col<stock.columns();++col) {
      double low[4],high[4];std::vector<double> levels;
      for(int i=0;i<4;++i) {
        const int x=std::clamp(col+i%2,0,stock.columns()-1),
                  y=std::clamp(row+i/2,0,stock.rows()-1);
        low[i]=stock.bottom(x,y);high[i]=stock.top(x,y);
        if(std::isfinite(low[i]) && std::isfinite(high[i]) && high[i]>low[i]) {
          levels.push_back(low[i]);levels.push_back(high[i]);
        }
      }
      std::sort(levels.begin(),levels.end());
      levels.erase(std::unique(levels.begin(),levels.end()),levels.end());
      for(std::size_t band=1;band<levels.size();++band) {
        const double z=(levels[band-1]+levels[band])*.5;
        double material[4];
        for(int i=0;i<4;++i) material[i]=z>low[i] && z<high[i]?1:0;
        const double lo[]{stock.lower_x()+std::max(0.,col+.5)*pitch,
                          stock.lower_y()+std::max(0.,row+.5)*pitch,levels[band-1]},
                     hi[]{stock.lower_x()+std::min(double(stock.columns()),col+1.5)*pitch,
                          stock.lower_y()+std::min(double(stock.rows()),row+1.5)*pitch,levels[band]};
        const double ro[]{origin.x,origin.y,origin.z},rd[]{direction.x,direction.y,direction.z};
        double enter=0,leave=hit;
        for(int k=0;k<3;++k) {
          if(std::abs(rd[k])<1e-12) {
            if(ro[k]<lo[k] || ro[k]>hi[k]) leave=-1;
          } else {
            double a=(lo[k]-ro[k])/rd[k],b=(hi[k]-ro[k])/rd[k];
            if(a>b) std::swap(a,b);
            enter=std::max(enter,a);leave=std::min(leave,b);
          }
        }
        if(enter>leave || enter>=hit) continue;
        const double u=(origin.x-stock.lower_x())/pitch-col-.5,
                     v=(origin.y-stock.lower_y())/pitch-row-.5,
                     dx=direction.x/pitch,dy=direction.y/pitch;
        const double sx=material[1]-material[0],sy=material[2]-material[0],
                     cross=material[3]-material[2]-material[1]+material[0];
        const double a=cross*dx*dy,b=sx*dx+sy*dy+cross*(u*dy+v*dx),
                     c=material[0]+sx*u+sy*v+cross*u*v-.5;
        if((a*enter+b)*enter+c>=-1e-10) {hit=enter;continue;}
        std::vector<double> roots;
        if(std::abs(a)<1e-12) {if(std::abs(b)>1e-12) roots.push_back(-c/b);}
        else if(b*b-4*a*c>=0) {
          roots.push_back((-b-std::sqrt(b*b-4*a*c))/(2*a));
          roots.push_back((-b+std::sqrt(b*b-4*a*c))/(2*a));
        }
        for(double t:roots) if(t>=enter-1e-9 && t<=leave+1e-9)
          hit=std::min(hit,std::clamp(t,enter,leave));
      }
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
           "GPU surface matches independent material-boundary reference");
    }
}
// An analytic bilinear sheet is reproduced exactly between sample centres.
// Check ray depth against its world-space equation, independent of the shader's
// tiling, traversal, patch construction and root solver.
void compare_sheet(Renderer& renderer, const Heightmap& stock, Camera c,
                   double offset, double sx, double sy, double cross,
                   bool bottom = false) {
  constexpr int w = 43, h = 37;
  renderer.render(stock, c, w, h);
  const auto depths = renderer.read_depth();
  int checked = 0;
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const double px = (x + .5 - c.pan_x) / c.scale,
                   py = (h - y - .5 - c.pan_y) / c.scale;
      Vec3d ro{c.centre.x + c.right.x * px - c.up.x * py + c.toward.x * c.reach,
               c.centre.y + c.right.y * px - c.up.y * py + c.toward.y * c.reach,
               c.centre.z + c.right.z * px - c.up.z * py + c.toward.z * c.reach};
      const Vec3d rd{-c.toward.x, -c.toward.y, -c.toward.z};
      const double a = -cross * rd.x * rd.y,
                   b = rd.z - sx * rd.x - sy * rd.y - cross * (ro.x * rd.y + ro.y * rd.x),
                   d = ro.z - offset - sx * ro.x - sy * ro.y - cross * ro.x * ro.y;
      std::vector<double> roots;
      if (std::abs(a) < 1e-12) roots.push_back(-d/b);
      else if (b*b - 4*a*d >= 0) {
        roots.push_back((-b-std::sqrt(b*b-4*a*d))/(2*a));
        roots.push_back((-b+std::sqrt(b*b-4*a*d))/(2*a));
      }
      std::sort(roots.begin(),roots.end());
      for (double t : roots) {
        const auto p = interpolate(ro,{ro.x+rd.x,ro.y+rd.y,ro.z+rd.z},t);
        if (t <= 0 || t >= 2*c.reach ||
            p.x < stock.x(0)+stock.pitch() || p.x > stock.x(stock.columns()-1)-stock.pitch() ||
            p.y < stock.y(0)+stock.pitch() || p.y > stock.y(stock.rows()-1)-stock.pitch()) continue;
        // The selected cap faces the ray; skip possible far-side roots.
        const double facing = rd.z - (sx+cross*p.y)*rd.x - (sy+cross*p.x)*rd.y;
        if (bottom ? facing <= 0 : facing >= 0) continue;
        near(depths[std::size_t(y)*w+x],t/(2*c.reach),3e-5,
             "Reconstructed surface matches analytic ray intersection");
        ++checked;
        break;
      }
    }
  check(checked > 50,"Analytic comparison must cover the reconstructed surface");
}
void wall_boundaries(Renderer& renderer) {
  auto stock=Heightmap::box({{0,0,0},{12,12,8},true},1);
  // A patch with one high corner has boundary (1-u)(1-v)=1/2.
  // At v=1/3 the exact hit is u=1/4, between the old square cell edges.
  for(int variant=0;variant<3;++variant) {
    for(int y=0;y<stock.rows();++y) for(int x=0;x<stock.columns();++x) {
      const bool high=stock.x(x)+stock.y(y)<12;
      stock.set(x,y,variant==2 && !high?6:0,high || variant==2?8:variant==0?2:0);
    }
    Camera c{{5.75f,5.5f+1.f/3,5},{0,1,0},{0,0,1},{1,0,0},{1,1,0},20,40,.5f,.5f};
    renderer.render(stock,c,1,1);
    near(renderer.read_depth().front(),.5,2e-6,
         "Diagonal wall hits its continuous boundary for steps, holes and undersides");
    c.toward={1/std::sqrt(1.09f),0,(variant==2?-.3f:.3f)/std::sqrt(1.09f)};
    c.up={-c.toward.z,0,c.toward.x};
    c.centre.z=variant==2?5.9f:7.9f;
    renderer.render(stock,c,1,1);
    near(renderer.read_depth().front(),.5,2e-6,"Wall remains joined to its cap under oblique rays");
  }
  // Three solid corners: uv=1/2. This boundary extends into a completely empty
  // tile, and must invalidate neighbouring uniform-box shortcuts as well.
  auto seam=Heightmap::box({{0,0,0},{64,64,8},true},1);
  for(int y=0;y<seam.rows();++y) for(int x=0;x<seam.columns();++x)
    if(seam.x(x)+seam.y(y)>64) seam.set(x,y,0,0);
  seam.compact_history();
  Camera c{{32.25f,31.5f+2.f/3,5},{0,1,0},{0,0,1},{1,0,0},{1,1,0},20,40,.5f,.5f};
  renderer.render(seam,c,1,1);
  near(renderer.read_depth().front(),.5,2e-6,"Reconstructed walls cross full and empty tile seams");
  auto edited=seam;
  edited.set(31,31,0,0);
  renderer.render(edited,c,1,1);
  check(renderer.read_depth().front()>.501f,
        "Editing a border sample invalidates its empty neighbour's wall");
  renderer.render(seam,c,1,1);
  near(renderer.read_depth().front(),.5,2e-6,
       "Seeking backward restores reconstructed walls in empty neighbouring tiles");
  for(bool bottom:{false,true}) {
    c.centre={32.1f,32.1f,bottom?0.f:8.f};c.right={1,0,0};
    c.up={0,bottom?-1.f:1.f,0};c.toward={0,0,bottom?-1.f:1.f};
    renderer.render(seam,c,1,1);
    near(renderer.read_depth().front(),.5,2e-6,"Cap reaches its wall inside an empty neighbouring tile");
  }
  auto straight=Heightmap::box({{0,0,0},{8,8,8},true},.5);
  for(int y=0;y<straight.rows();++y) for(int x=8;x<straight.columns();++x)
    straight.set(x,y,0,0);
  c={{4,3.5f,8},{1,0,0},{0,1,0},{0,0,1},{0,0,1},20,40,.5f,.5f};
  renderer.render(straight,c,1,1);
  near(renderer.read_depth().front(),.5,2e-6,"Cap and wall agree on the exact boundary");
  // A sloping cap extends continuously to an axis-aligned height step.
  auto slope=Heightmap::box({{0,0,0},{8,8,10},true},.5);
  for(int y=0;y<slope.rows();++y) for(int x=0;x<slope.columns();++x)
    slope.set(x,y,0,slope.x(x)<4?6+.2*slope.y(y):2);
  c={{3.99f,5.75f,7.15f},{1,0,0},{0,1,0},{0,0,1},{0,0,1},20,40,.5f,.5f};
  renderer.render(slope,c,1,1);
  near(renderer.read_depth().front(),.5,2e-6,"Sloping cap meets the reconstructed wall without a gap");
}
void profile_sample(Renderer& renderer, const Heightmap& stock, bool along_x) {
  // Centre-to-centre interpolation of the sampled ball groove, observed from
  // above. The sample is halfway between two stored heights.
  const int a = 22, b = 23, fixed = 30;
  const double z = along_x ? (stock.top(a,fixed)+stock.top(b,fixed))*.5
                          : (stock.top(fixed,a)+stock.top(fixed,b))*.5;
  const double x = along_x ? (stock.x(a)+stock.x(b))*.5 : stock.x(fixed),
               y = along_x ? stock.y(fixed) : (stock.y(a)+stock.y(b))*.5;
  Camera c{{float(x),float(y),float(z)},{1,0,0},{0,1,0},{0,0,1},{0,0,1},
           1,40,.5f,.5f};
  renderer.render(stock,c,1,1);
  near(renderer.read_depth().front(),.5,2e-6,"Compressed groove interpolates physical heights");
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
      camera.pan_x = float(width) / 2 + (i + 1) * .1f;
      renderer.render(stock, camera, width, height);
      glFinish();
      check(!renderer.stats().reused_frame,"Moving-camera benchmark must redraw every frame");
    }
    const double frame =
        std::chrono::duration<double, std::milli>(Clock::now() - warm).count() /
        8;
    const auto idle_start = Clock::now();
    for (int i = 0; i < 64; ++i) renderer.render(stock,camera,width,height);
    const double idle = std::chrono::duration<double,std::milli>(Clock::now()-idle_start).count()/64;
    check(renderer.stats().reused_frame,"Idle benchmark reuses the completed frame");
    std::cout << "render_benchmark pitch=" << pitch
              << " cells=" << stock.columns() * std::size_t(stock.rows())
              << " viewport=" << width << "x" << height << " first_ms=" << first << " frame_ms=" << frame
              << " idle_ms=" << idle
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
      stock.cut({-2, 0, 2}, {4, 0, 2}, {CutterShape::Flat, 1.5, 20});
      stock.set(3, 3, 0, 0);
      stock.set(4, 3, 0, 0);
      stock.set(0, 0, 2, 3);
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
      // Slopes and a saddle exercise non-axis-aligned normals and the two
      // roots of a bilinear patch, including across full and partial tiles.
      auto sheet = Heightmap::box({{-2,-1,0},{27.35f,12.3f,20},true},.5);
      for (auto params : {std::array<double,4>{4,.2,.1,0},
                          std::array<double,4>{4,.05,.05,.015}}) {
        for (int y=0;y<sheet.rows();++y)
          for (int x=0;x<sheet.columns();++x)
            sheet.set(x,y,0,params[0]+params[1]*sheet.x(x)+params[2]*sheet.y(y)+
                              params[3]*sheet.x(x)*sheet.y(y));
        sheet.compact_history();
        c={{14,5,6},{1,0,0},{0,1,0},{0,0,1},{0,0,1},4,40,21.3f,18.1f};
        compare_sheet(renderer,sheet,c,params[0],params[1],params[2],params[3]);
        c.right={r,-r,0};c.up={u,u,-2*u};c.toward={s,s,s};
        compare_sheet(renderer,sheet,c,params[0],params[1],params[2],params[3]);
        c.right={-r,r,0};c.up={-u,-u,-2*u};c.toward={-s,-s,s};
        compare_sheet(renderer,sheet,c,params[0],params[1],params[2],params[3]);
      }
      for (int y=0;y<sheet.rows();++y)
        for (int x=0;x<sheet.columns();++x)
          sheet.set(x,y,1+.03*sheet.x(x)+.05*sheet.y(y),12);
      c={{14,5,3},{1,0,0},{0,-1,0},{0,0,-1},{0,0,-1},4,40,21.3f,18.1f};
      compare_sheet(renderer,sheet,c,1,.03,.05,0,true);
      wall_boundaries(renderer);
      // Camera movement and unchanged snapshots must not resend the grid.
      renderer.render(stock, c, 43, 37);
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
      profile_sample(renderer, patterned, true);
      auto horizontal = Heightmap::box({{0, 0, 0}, {8, 8, 8}, true}, .125);
      horizontal.cut({-2, 4, 2}, {10, 4, 2}, {CutterShape::Ball, 2, 20});
      horizontal.compact_history();
      profile_sample(renderer, horizontal, false);
      auto partial =
          Heightmap::box({{-2, -1, 0}, {7.35f, 8.3f, 6}, true}, .125);
      partial.cut({-3, -2, 1}, {9, 10, 1}, {CutterShape::Flat, 1, 20});
      c.right = {r, -r, 0};
      c.up = {u, u, -2 * u};
      c.toward = {s, s, s};
      compare(renderer, partial, c, 31, 29);
      // A neighbouring edit must invalidate the uniform-tile shortcut and its
      // culling bounds even when that tile's own samples have not changed.
      auto seam = Heightmap::box({{0,0,0},{32,16,8},true},.5);
      c={{16,8.25f,8.2f},{1,0,0},{0,1,0},{0,0,1},{0,0,1},1,40,.5f,.5f};
      renderer.render(seam,c,1,1);
      auto seam_edit=seam;
      seam_edit.set(32,16,0,8.4);
      renderer.render(seam_edit,c,1,1);
      near(renderer.read_depth().front(),.5,2e-6,"Neighbouring tiles share a continuous cap");
      check(renderer.stats().changed_tiles==1,"Interpolation uploads only the changed tile's samples");
      renderer.render(seam,c,1,1);
      near(renderer.read_depth().front(),40.2/80,2e-6,"Seeking restores adjacent tile reconstruction");
      // The stock's depth attachment must also occlude ordinary GPU meshes.
      auto block = Heightmap::box({{-2, -2, 0}, {2, 2, 4}, true}, .25);
      c = {{0, 0, 2}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 1},
           6,         40,        21.3f,     18.1f};
      renderer.render(block, c, 43, 37);
      const auto plain = renderer.read_colour();
      const auto plain_depth = renderer.read_depth();
      renderer.render(block,c,43,37);
      check(renderer.stats().reused_frame && renderer.read_colour()==plain &&
                renderer.read_depth()==plain_depth,
            "Unchanged frames reuse the rendered image and depth");
      auto relit=c;
      relit.light={1,0,0};
      renderer.render(block,relit,43,37);
      check(!renderer.stats().reused_frame && renderer.read_colour()!=plain &&
                renderer.read_depth()==plain_depth,
            "Lighting changes invalidate the cached image without changing geometry");
      renderer.render(block,c,43,37);
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
      check(!renderer.stats().reused_frame,"Moving the cutter invalidates the rendered frame");
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
