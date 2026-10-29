#include "wave/mesh_slicer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool condition, const char* text) {
  if (!condition) throw std::runtime_error(text);
}
double xy_area(const wave::Triangle3f& t) {
  return ((double(t.b.x)-t.a.x)*(double(t.c.y)-t.a.y)-
          (double(t.b.y)-t.a.y)*(double(t.c.x)-t.a.x))/2;
}
void write(const std::filesystem::path& p, const std::string& text) {
  std::ofstream out(p); out << text;
}
}
int main() {
  const auto dir = std::filesystem::temp_directory_path()/"tinglecam-mesh-loading-tests";
  try {
    std::filesystem::create_directories(dir);
    const auto obj = dir/"part.obj";
    // A concave U in three coordinate planes, both windings, and OBJ's
    // negative/slashed indices, repeated closing index, and inline comments.
    for (int axis = 0; axis < 3; ++axis) for (bool reverse : {false,true}) {
      const float xy[][2]{{0,0},{4,0},{4,4},{3,4},{3,1},{1,1},{1,4},{0,4}};
      std::ofstream out(obj);
      for (const auto& p : xy) {
        if (axis == 0) out << "v " << p[0] << ' ' << p[1] << " 2\n";
        if (axis == 1) out << "v 2 " << p[0] << ' ' << p[1] << '\n';
        if (axis == 2) out << "v " << p[1] << " 2 " << p[0] << '\n';
      }
      out << "f";
      for (int i=0;i<8;++i) out << ' ' << (reverse ? -1-i : -8+i) << "/1/1";
      out << ' ' << (reverse ? -1 : -8) << "/1/1 # concave cap\n";
      out.close();
      auto mesh = wave::load_triangle_mesh(obj);
      check(mesh.triangles.size()==6,"Concave polygon must triangulate into n-2 triangles");
      double area=0;
      for (auto t : mesh.triangles) {
        if (axis==1) {t.a={t.a.y,t.a.z,t.a.x};t.b={t.b.y,t.b.z,t.b.x};t.c={t.c.y,t.c.z,t.c.x};}
        if (axis==2) {t.a={t.a.z,t.a.x,t.a.y};t.b={t.b.z,t.b.x,t.b.y};t.c={t.c.z,t.c.x,t.c.y};}
        const double a=xy_area(t);
        check(reverse ? a<0 : a>0,"Triangulation must preserve face winding");
        area+=a;
        const float x=(t.a.x+t.b.x+t.c.x)/3, y=(t.a.y+t.b.y+t.c.y)/3;
        check(!(x>1 && x<3 && y>1),"Concave cutout must remain empty");
      }
      check(std::abs(std::abs(area)-10)<1e-6,"Concave cap area must match polygon area");
    }
    write(obj,"v 0 0 -8\nv 0 1 -8\nv 0 0 3\nf 1 2 3\n");
    auto vertical=wave::load_triangle_mesh(obj);
    check(vertical.triangles.size()==1 && vertical.bounds.lower.z==-8 && vertical.bounds.upper.z==3,
          "Vertical OBJ faces and their full bounds must survive loading");
    wave::MeshSlicer slicer(wave::MeshSlicerBackend::CPU);
    slicer.set_mesh(vertical);
    check(slicer.slice(0).polygons.empty(),"Vertical display faces must not add heightfield material");
    double ms=0;
    check(!std::isfinite(slicer.vertical_raycast({{0,.2f,5}},-10,ms).front()),
          "Vertical faces must not produce NaN-derived height hits");
    if (wave::cuda_slicer_available()) {
      wave::MeshSlicer gpu(wave::MeshSlicerBackend::CUDA);
      gpu.set_mesh(vertical);
      check(gpu.slice(0).polygons.empty(),"CUDA must ignore vertical display faces");
      check(!std::isfinite(gpu.vertical_raycast({{0,.2f,5}},-10,ms).front()),
            "CUDA vertical-only mesh rays must miss");
    }
    const auto stl=dir/"wall.stl";
    write(stl,"solid wall\nfacet normal 1 0 0\nouter loop\nvertex 0 0 -8\nvertex 0 1 -8\nvertex 0 0 3\nendloop\nendfacet\nendsolid\n");
    check(wave::load_triangle_mesh(stl).triangles.size()==1,"Vertical ASCII STL faces must survive loading");
    const auto binary=dir/"binary-wall.stl";
    {
      std::ofstream out(binary,std::ios::binary);
      const char header[80]{};
      out.write(header,sizeof header);
      const std::uint32_t count=1;
      out.write(reinterpret_cast<const char*>(&count),sizeof count);
      const float data[]{1,0,0, 0,0,-8, 0,1,-8, 0,0,3};
      out.write(reinterpret_cast<const char*>(data),sizeof data);
      const std::uint16_t attribute=0;
      out.write(reinterpret_cast<const char*>(&attribute),sizeof attribute);
    }
    const auto binary_mesh=wave::load_triangle_mesh(binary);
    check(binary_mesh.triangles.size()==1 && binary_mesh.bounds.lower.z==-8,
          "Vertical binary STL faces and bounds must survive loading");
    write(obj,"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 0 2 3\n");
    bool rejected=false;
    try { (void)wave::load_triangle_mesh(obj); } catch (const std::exception&) {rejected=true;}
    check(rejected,"OBJ index zero must be rejected");
    std::filesystem::remove_all(dir);
    std::cout << "Mesh loading preserves vertical walls, concavity, winding and bounds.\n";
  } catch (const std::exception& e) {
    std::filesystem::remove_all(dir);
    std::cerr << e.what() << '\n'; return 1;
  }
}
