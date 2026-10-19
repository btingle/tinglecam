#pragma once

#include <array>

#include "cam/job.h"

namespace cam::detail {
struct SurfaceRun {
  std::vector<wave::Vec3d> points;
  wave::Vec3d first_contact, last_contact;
  std::array<double, 2> first_uv{}, last_uv{};
  std::size_t first_face = 0, last_face = 0;
  bool first_boundary = false, last_boundary = false;
};
void link_surface_runs(const CadModel&, const Operation&, const Tool&,
                       const Machine&, double rapid_plane,
                       std::vector<SurfaceRun>&, OperationPlan&,
                       const ProgressCallback&);
}  // namespace cam::detail
