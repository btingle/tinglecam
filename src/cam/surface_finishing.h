#pragma once
#include "cam/job.h"
namespace cam {
// A span belongs to one native face. Contact points are uncompensated and
// remain separate across seams, so the preview never draws an offset bridge.
struct SurfaceSpan {
  std::size_t face = 0, lane = 0;
  double across = 0, along_min = 0, along_max = 1;
  std::vector<wave::Vec3d> contacts;
};
struct SurfaceContour {
  std::vector<wave::Vec3d> points;
  wave::Vec3d first_contact, last_contact;
  std::array<double, 2> first_uv{}, last_uv{};
  std::size_t first_face = 0, last_face = 0;
  bool first_boundary = false, last_boundary = false;
  std::vector<SurfaceSpan> spans;
};
struct SurfaceContourPreview {
  std::vector<SurfaceContour> contours;
  std::vector<std::vector<std::size_t>> neighbours;
  std::string signature, error;
};
struct SurfaceContourOrder {
  static constexpr std::size_t none = std::size_t(-1);
  std::vector<std::size_t> sequence, depth, parent, root, branch;
  std::size_t branches = 0, unreachable = 0;
  std::string error;
};
std::string surface_contour_signature(const CadModel&, const Operation&, const Tool&);
SurfaceContourPreview prepare_surface_contours(const CadModel&, const Operation&,
                                              const Tool&, const ProgressCallback& = {});
SurfaceContourOrder propagate_surface_order(const SurfaceContourPreview&,
                                           const std::vector<std::size_t>& roots);
OperationPlan plan_surface_finishing(const JobGeometry&, const Operation&,
                                     const Tool&, const Machine&,
                                     const CuttingFeeds&, double rapid_plane,
                                     const ProgressCallback& = {});
wave::Vec3d ball_tip(const SurfacePoint&, double radius, double allowance = 0);
}  // namespace cam
