#pragma once

#include "cam/job.h"

#include <vector>

namespace cam {

// Finds simple, convex holes in the downward-envelope representation of the
// part. The returned geometry is derived planning data; it is deliberately
// not part of the saved job.
std::vector<OperationPlan::IdentifiedPocket> identify_pockets(
    const JobGeometry& geometry, const Operation& operation);

}  // namespace cam
