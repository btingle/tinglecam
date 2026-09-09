#pragma once

#include "cam/job.h"

namespace cam {

// Dimension-driven operations do not need a mesh. Coordinates use the same
// job frame as mesh paths; the post applies the job origin exactly once.
OperationPlan plan_parametric_operation(
    const Operation& operation, const Tool& tool, const Machine& machine,
    double rapid_plane, const std::vector<OperationPlan>& prior_operations,
    const ProgressCallback& progress = {}, double progress_from = 0.0,
    double progress_to = 1.0);

wave::Paths64 pocket_boundary(const PocketParameters& pocket);

}  // namespace cam
