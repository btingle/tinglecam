#pragma once

#include <array>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "simulation/heightmap.h"

namespace cam {
// Counters for the calling thread; snapshots can be differenced per planning
// task without global atomics or cross-worker attribution.
struct CadQueryStatistics {
  std::size_t containment_tests = 0, outside_reuses = 0,
              native_distance_tests = 0;
};
CadQueryStatistics cad_query_statistics();
struct SurfacePoint {
  wave::Vec3d point, normal;
  bool inside = false;
};
struct CadFace {
  std::size_t id = 0;
  std::string type;
  double area = 0;
};
// Immutable native geometry plus a face-tagged display/roughing triangulation.
// All coordinates are converted by the STEP reader to millimetres.
class CadModel {
 public:
  static std::shared_ptr<CadModel> load(const std::filesystem::path& path,
                                        double tolerance = 0.01);
  ~CadModel();
  const wave::Mesh& mesh() const;
  const std::vector<std::size_t>& triangle_faces() const;
  const std::vector<CadFace>& faces() const;
  const std::string& fingerprint() const;
  SurfacePoint evaluate(std::size_t face, double u, double v) const;
  // Tests the complete native model, including unselected faces. A positive
  // ball contact is permitted within tolerance; buried centres are rejected.
  // Inputs must be finite, with radius > tolerance >= 0 and flute >= radius.
  bool cutter_clear(wave::Vec3d tip, double radius, double flute_length,
                    double tolerance) const;
  bool cutter_sweep_clear(wave::Vec3d from, wave::Vec3d to, double radius,
                          double flute_length, double tolerance) const;
  // True only when the imported faces share a native topological edge.
  bool adjacent(std::size_t a, std::size_t b) const;
  // Normalized UV coordinates on both faces at their shared edge, nearest to
  // the requested contact point. Used to follow a surface across face seams.
  std::optional<std::array<std::array<double, 2>, 2>> shared_boundary(
      std::size_t a, std::size_t b, wave::Vec3d near) const;
  bool solid() const;

 private:
  CadModel();
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
bool step_filename(const std::filesystem::path& path);
}  // namespace cam
