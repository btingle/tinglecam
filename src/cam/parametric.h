#pragma once

#include "cam/job.h"

namespace cam {

// Dimension-driven operations do not need a mesh. Coordinates use the same
// job frame as mesh paths; the post applies the job origin exactly once.
OperationPlan plan_parametric_operation(
    const Operation& operation, const Tool& tool, const Machine& machine,
    double rapid_plane, const PriorOperations& prior_operations,
    const ProgressCallback& progress = {}, double progress_from = 0.0,
    double progress_to = 1.0, const JobGeometry* geometry = nullptr);

// The outline the cutter finishes to: the requested pocket taken in by the
// radial finishing margin, or out by a negative one.
wave::Paths64 pocket_boundary(const PocketParameters& pocket,
                              double radial_margin = 0.0);
wave::Paths64 pocket_margin_boundary(const wave::Paths64& boundary,
                                     double radial_margin);

}  // namespace cam
