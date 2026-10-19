#pragma once
#include "cam/job.h"
namespace cam {
OperationPlan plan_surface_finishing(const JobGeometry&, const Operation&,
                                     const Tool&, const Machine&,
                                     double rapid_plane,
                                     const ProgressCallback& = {});
wave::Vec3d ball_tip(const SurfacePoint&, double radius, double allowance = 0);
}  // namespace cam
