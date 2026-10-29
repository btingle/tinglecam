#include "wave/mesh_slicer.h"
#include "rendering/prism_renderer.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main(int argc,char** argv) {
  try {
    wave::Mesh mesh;
    auto geometry=std::make_shared<wave::prism::Geometry>();
    if(argc>1) mesh=wave::load_triangle_mesh(argv[1]);
    else {
      for(int y=0;y<160;++y) for(int x=0;x<160;++x) {
        float a=x,b=y;
        mesh.triangles.push_back({{a,b,10},{a+1,b,10},{a+1,b+1,10}});
        mesh.triangles.push_back({{a,b,10},{a+1,b+1,10},{a,b+1,10}});
      }
      mesh.bounds={{0,0,10},{160,160,10},true};
    }
    wave::MeshSlicer slicer;slicer.set_mesh(std::move(mesh));
    const auto& bounds=slicer.mesh().bounds;
    float width=bounds.upper.x-bounds.lower.x,height=bounds.upper.y-bounds.lower.y;
    std::vector<wave::Vec3f> rays;
    for(int y=0;y<256;++y) for(int x=0;x<256;++x)
      rays.push_back({bounds.lower.x+width*(x+.5f)/256,bounds.lower.y+height*(y+.5f)/256,bounds.upper.z+1});
    double ray_ms;auto hits=slicer.vertical_raycast(rays,bounds.lower.z-1,ray_ms);
    auto slice=slicer.slice((bounds.lower.z+bounds.upper.z)*.5f);
    if(slice.polygons.empty()) throw std::runtime_error("benchmark slice unexpectedly empty");
    for(const auto& t:slicer.mesh().triangles) {
      auto tri=t;
      double cross=(double(t.b.x)-t.a.x)*(double(t.c.y)-t.a.y)-(double(t.b.y)-t.a.y)*(double(t.c.x)-t.a.x);
      if(cross<0) std::swap(tri.b,tri.c);
      geometry->triangles.push_back(tri);
    }
    auto* r=wave::prism::create();
    wave::prism::update(r,{{geometry,bounds.lower.z-1,bounds.upper.z}});
    wave::prism::Camera camera{{(bounds.lower.x+bounds.upper.x)*.5f,(bounds.lower.y+bounds.upper.y)*.5f,0},
      {1,0,0},{0,1,0},{0,0,1},{0,0,1},float(480/std::max(width,height)),bounds.upper.z+100,256,256};
    auto start=std::chrono::steady_clock::now();
    auto pixels=wave::prism::render_host(r,camera,512,512);
    auto render_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    wave::prism::destroy(r);
    std::cout<<"triangles="<<slicer.stats().input_triangles<<" build_ms="<<slicer.stats().build_milliseconds
      <<" rays_65536_ms="<<ray_ms<<" slice_query_ms="<<slice.query_milliseconds
      <<" polygon_union_ms="<<slice.polygon_milliseconds<<" render_512x512_ms="<<render_ms<<'\n';
    if(argc==1) {
      for(float hit:hits) if(!std::isfinite(hit) || std::abs(hit-10)>1e-5f)
        throw std::runtime_error("benchmark ray hit mismatch");
      // Identical XY bounds with crossing height planes defeat spatial
      // rejection and exercise the containment preprocessing work limit.
      wave::Mesh overlapping;
      for(int i=0;i<16000;++i) {
        float z=float(i)/16000;
        overlapping.triangles.push_back({{0,0,z},{1,0,1-z},{0,1,z}});
      }
      overlapping.bounds={{0,0,0},{1,1,1},true};
      slicer.set_mesh(std::move(overlapping));
      std::cout<<"overlapping_triangles=16000 build_ms="<<slicer.stats().build_milliseconds<<'\n';
    }
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
