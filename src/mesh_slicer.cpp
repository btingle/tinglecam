#include "mesh_slicer.h"

#include "mesh_slicer_cuda.h"

#include <clipper2/clipper.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
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
  const double area_xy =
      static_cast<double>(triangle.b.x - triangle.a.x) * (triangle.c.y - triangle.a.y) -
      static_cast<double>(triangle.b.y - triangle.a.y) * (triangle.c.x - triangle.a.x);
  if (std::abs(area_xy) <= 1e-12) return;
  mesh.triangles.push_back(triangle);
  extend(mesh.bounds, triangle.a);
  extend(mesh.bounds, triangle.b);
  extend(mesh.bounds, triangle.c);
}

std::int32_t obj_index(std::string_view token, std::size_t vertex_count) {
  const auto slash = token.find('/');
  if (slash != std::string_view::npos) token = token.substr(0, slash);
  if (token.empty()) throw std::runtime_error("OBJ face has an empty vertex index");
  const int parsed = std::stoi(std::string(token));
  const std::int64_t index = parsed > 0 ? parsed - 1
                                       : static_cast<std::int64_t>(vertex_count) + parsed;
  if (index < 0 || index >= static_cast<std::int64_t>(vertex_count))
    throw std::runtime_error("OBJ face vertex index is out of range");
  return static_cast<std::int32_t>(index);
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
      while (fields >> token) face.push_back(obj_index(token, vertices.size()));
      if (face.size() < 3) throw std::runtime_error("OBJ face has fewer than three vertices");
      for (std::size_t i = 1; i + 1 < face.size(); ++i)
        append_triangle(result, {vertices[face[0]], vertices[face[i]], vertices[face[i + 1]]});
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
    throw std::runtime_error("Mesh has no triangles with non-degenerate XY projections");
  return result;
}

struct MeshSlicer::Impl {
  Mesh mesh;
  MeshBuildStats stats;
  cuda_slicer::Handle* gpu = nullptr;

  ~Impl() { cuda_slicer::destroy(gpu); }
};

MeshSlicer::MeshSlicer() : impl_(std::make_unique<Impl>()) {}
MeshSlicer::~MeshSlicer() = default;
MeshSlicer::MeshSlicer(MeshSlicer&&) noexcept = default;
MeshSlicer& MeshSlicer::operator=(MeshSlicer&&) noexcept = default;

void MeshSlicer::set_mesh(Mesh mesh) {
  if (mesh.triangles.empty()) throw std::invalid_argument("Mesh is empty");
  cuda_slicer::destroy(impl_->gpu);
  impl_->gpu = nullptr;
  impl_->mesh = std::move(mesh);
  impl_->stats = {};
  impl_->stats.input_triangles = impl_->mesh.triangles.size();
  impl_->gpu = cuda_slicer::create(impl_->mesh.triangles, impl_->mesh.bounds, impl_->stats);
}

MeshSlice MeshSlicer::slice(float z, int grid_resolution) const {
  if (empty()) throw std::runtime_error("No mesh has been loaded");
  MeshSlice result;
  auto candidates = cuda_slicer::query(impl_->gpu, z, grid_resolution,
                                        result.query_milliseconds);
  result.candidate_triangles = candidates.size();
  const auto start = std::chrono::steady_clock::now();
  result.polygons = polygons_from_candidates(impl_->mesh, candidates, z,
                                             result.contributing_triangles);
  result.polygon_milliseconds = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  return result;
}

const Mesh& MeshSlicer::mesh() const { return impl_->mesh; }
const MeshBuildStats& MeshSlicer::stats() const { return impl_->stats; }
bool MeshSlicer::empty() const { return impl_->mesh.triangles.empty(); }

}  // namespace wave
