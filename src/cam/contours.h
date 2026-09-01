#pragma once

#include "geometry.h"
#include "traversal.h"

#include <cstddef>
#include <vector>

namespace cam {

enum class ContourDirectionPolicy {
  Either,
  Climb,
  Conventional,
};

struct DerivedContour {
  wave::Path64 path;
  bool closed = false;
};

// Geometry only: offset the existing slice boundaries, then optionally clip
// their centre-lines to the supplied stock region. No ordering or linking is
// performed here.
std::vector<DerivedContour> derive_offset_contours(
    const wave::Paths64& sliced_boundaries,
    double outward_offset,
    const wave::Paths64* stock_clip = nullptr);

// Converts geometry into generic traversal tasks. Closed paths offer several
// starting positions; open paths offer the permitted orientations.
std::vector<wave::TraversalTask> contour_traversal_tasks(
    const std::vector<DerivedContour>& contours,
    ContourDirectionPolicy direction,
    std::size_t maximum_closed_starts = 16);

// Gives every traversal variant a cutter-clear landing outside the stock and
// a straight approach that does not enter the protected cutter-centre region.
// The contour path itself is left unchanged: traversal uses this approach only
// for an actual safe-plane entry. Variants without a legal approach are
// removed; a task losing every variant is an error.
void add_safe_stock_approaches(
    std::vector<wave::TraversalTask>& tasks,
    const wave::Paths64& stock,
    const wave::Paths64& protected_centres,
    double cutter_radius,
    double extra_clearance = 0.05);

const char* contour_direction_name(ContourDirectionPolicy value);

}  // namespace cam
