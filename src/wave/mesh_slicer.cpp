#include "mesh_slicer.h"

#include "mesh_slicer_cuda.h"
#include "mesh_slicer_cpu.h"
#include "cpu_bvh.h"

#include <clipper2/clipper.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace wave {
namespace {

void extend(Bounds3f& bounds, Vec3f p) {
  if (!bounds.valid) {
    bounds.lower = bounds.upper = p;
    bounds.valid = true;
    return;
  }
  bounds.lower.x = std::min(bounds.lower.x, p.x);
  bounds.lower.y = std::min(bounds.lower.y, p.y);
  bounds.lower.z = std::min(bounds.lower.z, p.z);
  bounds.upper.x = std::max(bounds.upper.x, p.x);
  bounds.upper.y = std::max(bounds.upper.y, p.y);
  bounds.upper.z = std::max(bounds.upper.z, p.z);
}

void append_triangle(Mesh& mesh, Triangle3f triangle) {
  for (const auto p : {triangle.a, triangle.b, triangle.c})
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      throw std::runtime_error("Mesh contains a non-finite vertex");
  const double ux = double(triangle.b.x) - triangle.a.x,
               uy = double(triangle.b.y) - triangle.a.y,
               uz = double(triangle.b.z) - triangle.a.z;
  const double vx = double(triangle.c.x) - triangle.a.x,
               vy = double(triangle.c.y) - triangle.a.y,
               vz = double(triangle.c.z) - triangle.a.z;
  const double nx = uy*vz - uz*vy, ny = uz*vx - ux*vz, nz = ux*vy - uy*vx;
  if (nx == 0 && ny == 0 && nz == 0) return;
  mesh.triangles.push_back(triangle);
  extend(mesh.bounds, triangle.a);
  extend(mesh.bounds, triangle.b);
  extend(mesh.bounds, triangle.c);
}

std::int32_t obj_index(std::string_view token, std::size_t vertex_count) {
  const auto slash = token.find('/');
  if (slash != std::string_view::npos) token = token.substr(0, slash);
  if (token.empty()) throw std::runtime_error("OBJ face has an empty vertex index");
  std::size_t used = 0;
  const int parsed = std::stoi(std::string(token), &used);
  if (used != token.size() || parsed == 0)
    throw std::runtime_error("Invalid OBJ vertex index");
  const std::int64_t index = parsed > 0 ? parsed - 1
                                       : static_cast<std::int64_t>(vertex_count) + parsed;
  if (index < 0 || index >= static_cast<std::int64_t>(vertex_count))
    throw std::runtime_error("OBJ face vertex index is out of range");
  return static_cast<std::int32_t>(index);
}

// Project onto the face's dominant plane. Convex faces use a linear fan;
// concave faces use ears, preserving the original 3D coordinates and winding.
void append_face(Mesh& mesh, const std::vector<Vec3f>& vertices,
                 std::vector<std::int32_t> face) {
  if (face.size() < 3) throw std::runtime_error("OBJ face has fewer than three vertices");
  const auto same_position = [&](std::int32_t a, std::int32_t b) {
    const auto p = vertices[a], q = vertices[b];
    return p.x == q.x && p.y == q.y && p.z == q.z;
  };
  // Exporters can close a surface with collapsed quads, using different OBJ
  // indices for identical positions. Like degenerate triangles, these carry
  // no surface area and must not prevent the rest of the model loading.
  face.erase(std::unique(face.begin(), face.end(), same_position), face.end());
  if (face.size() > 1 && same_position(face.front(), face.back())) face.pop_back();
  if (face.size() < 3) return;
  if (face.size() == 3) {
    append_triangle(mesh, {vertices[face[0]], vertices[face[1]], vertices[face[2]]});
    return;
  }
  double normal[3]{};
  bool has_area = false;
  const auto origin = vertices[face[0]];
  for (std::size_t i = 1; i + 1 < face.size(); ++i) {
    const auto b = vertices[face[i]], c = vertices[face[i+1]];
    const double u[]{double(b.x)-origin.x, double(b.y)-origin.y, double(b.z)-origin.z};
    const double v[]{double(c.x)-origin.x, double(c.y)-origin.y, double(c.z)-origin.z};
    for (int a = 0; a < 3; ++a) {
      const double component = u[(a+1)%3]*v[(a+2)%3] - u[(a+2)%3]*v[(a+1)%3];
      has_area |= component != 0;
      normal[a] += component;
    }
  }
  const int axis = int(std::max_element(normal, normal+3,
      [](double a, double b) { return std::abs(a) < std::abs(b); }) - normal);
  if (normal[axis] == 0) {
    if (!has_area) return; // Entirely collinear, including distinct positions.
    // A bow-tie can have cancelling signed area without being degenerate.
    throw std::runtime_error("OBJ face has cancelling area (check for self intersections)");
  }
  const double winding = normal[axis] > 0 ? 1 : -1;
  const auto xy = [&](std::int32_t id) {
    const auto p = vertices[id];
    if (axis == 0) return std::array<double,2>{p.y, p.z};
    if (axis == 1) return std::array<double,2>{p.z, p.x};
    return std::array<double,2>{p.x, p.y};
  };
  const auto turn = [&](std::int32_t a, std::int32_t b, std::int32_t c) {
    const auto p = xy(a), q = xy(b), r = xy(c);
    return winding*((q[0]-p[0])*(r[1]-p[1]) - (q[1]-p[1])*(r[0]-p[0]));
  };
  bool convex = true;
  for (std::size_t i = 0; i < face.size(); ++i)
    if (turn(face[i], face[(i+1)%face.size()], face[(i+2)%face.size()]) < 0)
      convex = false;
  if (convex) {
    for (std::size_t i = 1; i+1 < face.size(); ++i)
      append_triangle(mesh, {vertices[face[0]], vertices[face[i]], vertices[face[i+1]]});
    return;
  }
  while (face.size() > 3) {
    bool clipped = false;
    for (std::size_t i = 0; i < face.size(); ++i) {
      const auto a = face[(i+face.size()-1)%face.size()], b = face[i],
                 c = face[(i+1)%face.size()];
      const double area = turn(a,b,c);
      if (area < 0) continue;
      bool occupied = false;
      if (area > 0)
        for (auto p : face) {
          if (p == a || p == b || p == c) continue;
          if (turn(a,b,p) >= 0 && turn(b,c,p) >= 0 && turn(c,a,p) >= 0) {
            occupied = true;
            break;
          }
        }
      if (occupied) continue;
      if (area > 0) append_triangle(mesh, {vertices[a], vertices[b], vertices[c]});
      face.erase(face.begin() + i);
      clipped = true;
      break;
    }
    if (!clipped) throw std::runtime_error("Cannot triangulate OBJ face (check for self intersections)");
  }
  append_triangle(mesh, {vertices[face[0]], vertices[face[1]], vertices[face[2]]});
}

Mesh load_obj(const std::filesystem::path& filename) {
  std::ifstream stream(filename);
  if (!stream) throw std::runtime_error("Unable to open mesh: " + filename.string());
  std::vector<Vec3f> vertices;
  Mesh result;
  result.source = filename;
  std::string line;
  while (std::getline(stream, line)) {
    std::istringstream fields(line);
    std::string kind;
    fields >> kind;
    if (kind == "v") {
      Vec3f vertex;
      if (!(fields >> vertex.x >> vertex.y >> vertex.z))
        throw std::runtime_error("Malformed OBJ vertex in " + filename.string());
      vertices.push_back(vertex);
    } else if (kind == "f") {
      std::vector<std::int32_t> face;
      std::string token;
      while (fields >> token) {
        if (token.front() == '#') break;
        face.push_back(obj_index(token, vertices.size()));
      }
      append_face(result, vertices, std::move(face));
    }
  }
  return result;
}

template <typename T>
T read_binary(std::istream& stream) {
  T value{};
  stream.read(reinterpret_cast<char*>(&value), sizeof(value));
  if (!stream) throw std::runtime_error("Unexpected end of binary STL");
  return value;
}

Mesh load_binary_stl(const std::filesystem::path& filename) {
  std::ifstream stream(filename, std::ios::binary);
  if (!stream) throw std::runtime_error("Unable to open mesh: " + filename.string());
  stream.seekg(80);
  const std::uint32_t triangle_count = read_binary<std::uint32_t>(stream);
  Mesh result;
  result.source = filename;
  result.triangles.reserve(triangle_count);
  for (std::uint32_t i = 0; i < triangle_count; ++i) {
    for (int n = 0; n < 3; ++n) (void)read_binary<float>(stream);
    Triangle3f triangle;
    auto read_vertex = [&] {
      return Vec3f{read_binary<float>(stream), read_binary<float>(stream),
                   read_binary<float>(stream)};
    };
    triangle.a = read_vertex();
    triangle.b = read_vertex();
    triangle.c = read_vertex();
    (void)read_binary<std::uint16_t>(stream);
    append_triangle(result, triangle);
  }
  return result;
}

Mesh load_ascii_stl(const std::filesystem::path& filename) {
  std::ifstream stream(filename);
  if (!stream) throw std::runtime_error("Unable to open mesh: " + filename.string());
  Mesh result;
  result.source = filename;
  std::string word;
  std::array<Vec3f, 3> vertices{};
  int vertex_count = 0;
  while (stream >> word) {
    if (word != "vertex") continue;
    Vec3f vertex;
    if (!(stream >> vertex.x >> vertex.y >> vertex.z))
      throw std::runtime_error("Malformed ASCII STL vertex");
    vertices[vertex_count++] = vertex;
    if (vertex_count == 3) {
      append_triangle(result, {vertices[0], vertices[1], vertices[2]});
      vertex_count = 0;
    }
  }
  return result;
}

bool is_binary_stl(const std::filesystem::path& filename) {
  std::ifstream stream(filename, std::ios::binary | std::ios::ate);
  if (!stream) throw std::runtime_error("Unable to open mesh: " + filename.string());
  const auto size = stream.tellg();
  if (size < static_cast<std::streamoff>(84)) return false;
  stream.seekg(80);
  const std::uint32_t count = read_binary<std::uint32_t>(stream);
  return size == static_cast<std::streamoff>(84ull + 50ull * count);
}

struct SliceVertex {
  double x;
  double y;
  double z;
};

std::vector<SliceVertex> clip_triangle(const Triangle3f& triangle, double z) {
  std::vector<SliceVertex> input{
      {triangle.a.x, triangle.a.y, triangle.a.z},
      {triangle.b.x, triangle.b.y, triangle.b.z},
      {triangle.c.x, triangle.c.y, triangle.c.z},
  };
  std::vector<SliceVertex> output;
  output.reserve(4);
  for (std::size_t i = 0; i < input.size(); ++i) {
    const SliceVertex a = input[i];
    const SliceVertex b = input[(i + 1) % input.size()];
    const bool a_inside = a.z >= z;
    const bool b_inside = b.z >= z;
    if (a_inside) output.push_back(a);
    if (a_inside == b_inside) continue;
    const double t = (z - a.z) / (b.z - a.z);
    output.push_back({a.x + (b.x - a.x) * t,
                      a.y + (b.y - a.y) * t, z});
  }
  return output;
}

Paths64 polygons_from_candidates(const Mesh& mesh,
                                 const std::vector<std::uint32_t>& candidates,
                                 float z, std::size_t& contributing) {
  Paths64 pieces;
  pieces.reserve(candidates.size());
  for (const auto id : candidates) {
    if (id >= mesh.triangles.size()) continue;
    const auto clipped = clip_triangle(mesh.triangles[id], z);
    if (clipped.size() < 3) continue;
    Path64 path;
    path.reserve(clipped.size());
    for (const auto vertex : clipped) {
      Point64 point(to_integer(vertex.x), to_integer(vertex.y));
      if (path.empty() || path.back() != point) path.push_back(point);
    }
    const double area = path.size() >= 3 ? Clipper2Lib::Area(path) : 0.0;
    if (std::abs(area) > 0.0) {
      // Mesh winding is not meaningful for a union of downward prisms. Give
      // every contribution the same winding so opposite mesh faces cannot
      // cancel one another under NonZero filling.
      if (area < 0.0) std::reverse(path.begin(), path.end());
      pieces.push_back(std::move(path));
      ++contributing;
    }
  }
  if (pieces.empty()) return {};
  return Clipper2Lib::Union(pieces, Clipper2Lib::FillRule::NonZero);
}

}  // namespace

Mesh load_triangle_mesh(const std::filesystem::path& filename) {
  std::string extension = filename.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  Mesh result;
  if (extension == ".obj") result = load_obj(filename);
  else if (extension == ".stl")
    result = is_binary_stl(filename) ? load_binary_stl(filename) : load_ascii_stl(filename);
  else
    throw std::runtime_error("Unsupported mesh format '" + extension + "' (use OBJ or STL)");
  if (result.triangles.empty())
    throw std::runtime_error("Mesh has no non-degenerate triangles");
  return result;
}

struct MeshSlicer::Impl {
  Mesh mesh;
  MeshBuildStats stats;
  std::shared_ptr<Impl> source, query;
  ModelTransform placement;
  mutable std::mutex cache_mutex;
  mutable std::map<std::array<int, 3>, std::shared_ptr<Impl>> rotations;
  mutable std::unique_ptr<Mesh> display;
  mutable std::once_flag sweep_ready;
  mutable cpu::Bvh sweep_bvh;
  cuda_slicer::Handle* gpu = nullptr;
  cpu_slicer::Handle* cpu = nullptr;
  MeshSlicerBackend backend;

  explicit Impl(MeshSlicerBackend selected) : backend(selected) {
    if (backend != MeshSlicerBackend::Default) return;
    const char* value = std::getenv("TINGLECAM_SLICER");
    if (!value || std::string_view(value) == "auto") return;
    if (std::string_view(value) == "cuda") backend = MeshSlicerBackend::CUDA;
    else if (std::string_view(value) == "cpu") backend = MeshSlicerBackend::CPU;
    else throw std::invalid_argument("TINGLECAM_SLICER must be auto, cpu or cuda");
  }
  ~Impl() {
    cuda_slicer::destroy(gpu);
    cpu_slicer::destroy(cpu);
  }
};

MeshSlicer::MeshSlicer(MeshSlicerBackend backend) : impl_(std::make_shared<Impl>(backend)) {}
MeshSlicer::~MeshSlicer() = default;
MeshSlicer::MeshSlicer(MeshSlicer&&) noexcept = default;
MeshSlicer& MeshSlicer::operator=(MeshSlicer&&) noexcept = default;

bool cuda_slicer_available() { return cuda_slicer::available(); }

void MeshSlicer::set_mesh(Mesh mesh) {
  if (mesh.triangles.empty()) throw std::invalid_argument("Mesh is empty");
  impl_ = std::make_shared<Impl>(impl_->backend);
  impl_->mesh = std::move(mesh);
  impl_->stats = {};
  impl_->stats.input_triangles = impl_->mesh.triangles.size();
  if (impl_->backend == MeshSlicerBackend::CPU ||
      (impl_->backend == MeshSlicerBackend::Default && !cuda_slicer::available())) {
    impl_->cpu = cpu_slicer::create(impl_->mesh.triangles, impl_->mesh.bounds, impl_->stats);
  } else if (impl_->backend == MeshSlicerBackend::CUDA) {
    impl_->gpu = cuda_slicer::create(impl_->mesh.triangles, impl_->mesh.bounds, impl_->stats);
  } else {
    try {
      impl_->gpu = cuda_slicer::create(impl_->mesh.triangles, impl_->mesh.bounds, impl_->stats);
    } catch (const std::runtime_error& error) {
      std::fprintf(stderr, "CUDA mesh slicer unavailable (%s); using cuBQL CPU.\n", error.what());
      impl_->cpu = cpu_slicer::create(impl_->mesh.triangles, impl_->mesh.bounds, impl_->stats);
    }
  }
}

MeshSlice MeshSlicer::slice(float z, int grid_resolution) const {
  if (empty()) throw std::runtime_error("No mesh has been loaded");
  if (impl_->source) {
    MeshSlicer query;
    query.impl_ = impl_->query;
    auto result = query.slice(float(double(z) - impl_->placement.offset[2]), grid_resolution);
    const auto dx = to_integer(impl_->placement.offset[0]);
    const auto dy = to_integer(impl_->placement.offset[1]);
    for (auto& ring : result.polygons) for (auto& p : ring) { p.x += dx; p.y += dy; }
    return result;
  }
  MeshSlice result;
  auto candidates = impl_->cpu
      ? cpu_slicer::query(impl_->cpu, z, result.query_milliseconds)
      : cuda_slicer::query(impl_->gpu, z, grid_resolution, result.query_milliseconds);
  result.candidate_triangles = candidates.size();
  const auto start = std::chrono::steady_clock::now();
  result.polygons = polygons_from_candidates(impl_->mesh, candidates, z,
                                             result.contributing_triangles);
  result.polygon_milliseconds = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  return result;
}

std::vector<float> MeshSlicer::vertical_raycast(
    const std::vector<Vec3f>& origins, float lower_z,
    double& milliseconds) const {
  if (empty()) throw std::runtime_error("No mesh has been loaded");
  if (impl_->source) {
    auto local = origins;
    const auto& o = impl_->placement.offset;
    for (auto& p : local) { p.x -= o[0]; p.y -= o[1]; p.z -= o[2]; }
    MeshSlicer query;
    query.impl_ = impl_->query;
    auto result = query.vertical_raycast(local, float(double(lower_z) - o[2]), milliseconds);
    for (auto& z : result) if (std::isfinite(z)) z += o[2];
    return result;
  }
  return impl_->cpu ? cpu_slicer::vertical_raycast(impl_->cpu, origins, lower_z, milliseconds)
                    : cuda_slicer::vertical_raycast(impl_->gpu, origins, lower_z, milliseconds);
}

namespace {
struct SweepPoint { double x, y; };
SweepPoint subtract(SweepPoint a, SweepPoint b) { return {a.x-b.x,a.y-b.y}; }
double cross(SweepPoint a, SweepPoint b) { return a.x*b.y-a.y*b.x; }
double point_segment_distance_squared(SweepPoint p,SweepPoint a,SweepPoint b) {
  const auto d=subtract(b,a),v=subtract(p,a);
  const double length=d.x*d.x+d.y*d.y;
  const double t=length>0?std::clamp((v.x*d.x+v.y*d.y)/length,0.0,1.0):0;
  const double x=v.x-t*d.x,y=v.y-t*d.y;
  return x*x+y*y;
}
double segment_distance_squared(SweepPoint a,SweepPoint b,SweepPoint c,SweepPoint d) {
  const auto u=subtract(b,a),v=subtract(d,c),w=subtract(c,a);
  const double determinant=cross(u,v);
  if(determinant!=0) {
    const double t=cross(w,v)/determinant,s=cross(w,u)/determinant;
    if(t>=0 && t<=1 && s>=0 && s<=1) return 0;
  }
  return std::min({point_segment_distance_squared(a,c,d),point_segment_distance_squared(b,c,d),
                   point_segment_distance_squared(c,a,b),point_segment_distance_squared(d,a,b)});
}
// Clip one triangle against Z, then test its convex XY projection against a
// capsule. Degenerate projections still protect thin/vertical fixture walls.
bool triangle_touches_sweep(const Triangle3f& triangle,double z,SweepPoint from,SweepPoint to,
                            double radius_squared) {
  const std::array<SliceVertex,3> vertices{{{triangle.a.x,triangle.a.y,triangle.a.z},
      {triangle.b.x,triangle.b.y,triangle.b.z},{triangle.c.x,triangle.c.y,triangle.c.z}}};
  std::array<SweepPoint,4> polygon;
  std::size_t count=0;
  for(std::size_t i=0;i<vertices.size();++i) {
    const auto a=vertices[i],b=vertices[(i+1)%vertices.size()];
    const bool inside=a.z>=z,next_inside=b.z>=z;
    if(inside) polygon[count++]={a.x,a.y};
    if(inside!=next_inside) {
      const double t=(z-a.z)/(b.z-a.z);
      polygon[count++]={a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t};
    }
  }
  if(count==0) return false;
  double area=0;
  bool from_positive=false,from_negative=false,to_positive=false,to_negative=false;
  for(std::size_t i=0;i<count;++i) {
    const auto a=polygon[i],b=polygon[(i+1)%count];
    if(segment_distance_squared(from,to,a,b)<=radius_squared) return true;
    area+=cross(subtract(a,polygon[0]),subtract(b,polygon[0]));
    const double f=cross(subtract(b,a),subtract(from,a)),t=cross(subtract(b,a),subtract(to,a));
    from_positive |= f>0;from_negative |= f<0;to_positive |= t>0;to_negative |= t<0;
  }
  return area!=0 && (!(from_positive && from_negative) || !(to_positive && to_negative));
}
}  // namespace

bool MeshSlicer::projected_sweep_clear(Vec3d from,Vec3d to,double minimum_z,double radius) const {
  for(double value:{from.x,from.y,from.z,to.x,to.y,to.z,minimum_z,radius})
    if(!std::isfinite(value)) throw std::invalid_argument("Fixture sweep coordinates and radius must be finite");
  if(radius<0) throw std::invalid_argument("Fixture sweep radius must be nonnegative");
  if(empty()) return true;
  if(impl_->source) {
    const auto& offset=impl_->placement.offset;
    from.x-=offset[0];from.y-=offset[1];from.z-=offset[2];
    to.x-=offset[0];to.y-=offset[1];to.z-=offset[2];
    MeshSlicer query;query.impl_=impl_->query;
    return query.projected_sweep_clear(from,to,minimum_z-offset[2],radius);
  }
  const double left=std::min(from.x,to.x)-radius,right=std::max(from.x,to.x)+radius;
  const double bottom=std::min(from.y,to.y)-radius,top=std::max(from.y,to.y)+radius;
  const auto overlaps=[&](const cpu::Box& box) {
    return box.hi.z>=minimum_z && box.hi.x>=left && box.lo.x<=right &&
           box.hi.y>=bottom && box.lo.y<=top;
  };
  const auto& bounds=impl_->mesh.bounds;
  if(!overlaps(cpu::Box{bounds.lower,bounds.upper})) return true;
  std::call_once(impl_->sweep_ready,[&] {
    std::vector<cpu::Box> boxes;boxes.reserve(impl_->mesh.triangles.size());
    for(const auto& triangle:impl_->mesh.triangles) boxes.push_back(cpu::bounds(triangle));
    impl_->sweep_bvh.build(boxes);
  });
  bool clear=true;
  impl_->sweep_bvh.query(overlaps,[&](std::uint32_t index) {
    if(triangle_touches_sweep(impl_->mesh.triangles[index],minimum_z,
                              {from.x,from.y},{to.x,to.y},radius*radius)) {
      clear=false;return false;
    }
    return true;
  });
  return clear;
}

MeshSlicer MeshSlicer::placed(const ModelTransform& transform) const {
  transform.validate();
  if (empty()) return MeshSlicer(impl_->backend);
  const auto source = impl_->source ? impl_->source : impl_;
  auto query = source;
  if (transform.orientation != ModelOrientation{}) {
    std::lock_guard lock(source->cache_mutex);
    auto& cached = source->rotations[transform.orientation.axes];
    if (!cached) {
      auto mesh = source->mesh;
      for (auto& t : mesh.triangles) {
        t.a = transform.orientation.apply(t.a);
        t.b = transform.orientation.apply(t.b);
        t.c = transform.orientation.apply(t.c);
      }
      const auto a = transform.orientation.apply(mesh.bounds.lower);
      const auto b = transform.orientation.apply(mesh.bounds.upper);
      mesh.bounds.lower = {std::min(a.x,b.x),std::min(a.y,b.y),std::min(a.z,b.z)};
      mesh.bounds.upper = {std::max(a.x,b.x),std::max(a.y,b.y),std::max(a.z,b.z)};
      MeshSlicer rotated(source->backend);
      rotated.set_mesh(std::move(mesh));
      cached = rotated.impl_;
    }
    query = cached;
  }
  const double limit = double(Clipper2Lib::MAX_COORD) / kIntegerScale;
  for (const auto p : {query->mesh.bounds.lower, query->mesh.bounds.upper}) {
    const double coordinates[]{double(p.x)+transform.offset[0], double(p.y)+transform.offset[1],
                               double(p.z)+transform.offset[2]};
    for (double value : coordinates)
      if (!std::isfinite(value) || std::abs(value) > limit)
        throw std::invalid_argument("Model position exceeds the geometry coordinate range");
  }
  MeshSlicer result(source->backend);
  result.impl_->source = source;
  result.impl_->query = query;
  result.impl_->placement = transform;
  return result;
}
const Mesh& MeshSlicer::source_mesh() const {
  return impl_->source ? impl_->source->mesh : impl_->mesh;
}
const void* MeshSlicer::source_identity() const {
  return impl_->source ? impl_->source.get() : impl_.get();
}
const Mesh& MeshSlicer::mesh() const {
  if (!impl_->source) return impl_->mesh;
  const auto& o = impl_->placement.offset;
  if (o == std::array<double,3>{}) return impl_->query->mesh;
  std::lock_guard lock(impl_->cache_mutex);
  if (!impl_->display) {
    auto display = std::make_unique<Mesh>(impl_->query->mesh);
    const auto shift = [&](Vec3f p) { return Vec3f{float(p.x+o[0]),float(p.y+o[1]),float(p.z+o[2])}; };
    for (auto& t : display->triangles) { t.a=shift(t.a); t.b=shift(t.b); t.c=shift(t.c); }
    display->bounds.lower=shift(display->bounds.lower);
    display->bounds.upper=shift(display->bounds.upper);
    impl_->display=std::move(display);
  }
  return *impl_->display;
}
const MeshBuildStats& MeshSlicer::stats() const { return impl_->source ? impl_->query->stats : impl_->stats; }
std::shared_ptr<const Mesh> MeshSlicer::shared_mesh() const {
  return std::shared_ptr<const Mesh>(impl_, &mesh());
}
void MeshSlicer::clear() { impl_ = std::make_shared<Impl>(impl_->backend); }
bool MeshSlicer::empty() const { return impl_->source ? impl_->query->mesh.triangles.empty() : impl_->mesh.triangles.empty(); }

}  // namespace wave
