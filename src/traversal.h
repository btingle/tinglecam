#pragma once

#include "geometry.h"
#include "toolpath.h"

#include <cstddef>
#include <optional>
#include <vector>

namespace wave {

// One way to execute a task. The path is already oriented and, for a closed
// contour, repeats its starting point at the end. `safe_approach`, when set,
// runs from a cutter-clear landing to path.front(); it is used only when the
// planner must enter from the safe plane, never as a mandatory waypoint for a
// legal stay-down connection.
struct TraversalVariant {
  Path64 path;
  std::optional<Path64> safe_approach;
};

struct TraversalTask {
  std::size_t source = 0;
  std::vector<TraversalVariant> variants;
};

struct TraversalSettings {
  // Cutter-centre region in which a straight at-depth connector is legal.
  // Empty, or allow_stay_down=false, makes the safe retract the only connector.
  Paths64 stay_down_region;
  bool allow_stay_down = true;
  double cutting_feed = 1000.0;
  double rapid_feed = 2000.0;
  double floor_z = 0.0;
  double rapid_clearance_height = 10.0;
  // Lets a caller hand an already-cut node or roughing path directly to this
  // planner. When absent, traversal begins from the safe plane.
  std::optional<Point64> initial_position;
  // Useful when the caller knows the region is already cleared: do not choose
  // a faster retract merely because the legal at-depth feed is slower.
  bool prefer_stay_down_when_legal = false;
};

struct TraversalChoice {
  std::size_t task = 0;
  std::size_t variant = 0;
};

struct TraversalResult {
  Toolpath toolpath;
  std::vector<TraversalChoice> order;
  // Includes vertical retract/descent time even though the 2D Toolpath stores
  // only the XY endpoints posted at the safe plane.
  double estimated_minutes = 0.0;
};

// Orders arbitrary path-like tasks and chooses an orientation/start variant.
// Small task/variant sets use an exact dynamic program; larger collections use
// a deterministic nearest-incremental traversal so many closed-contour start
// choices cannot explode memory or planning time. Every disconnected option
// is costed at Z0 + rapid_clearance_height.
TraversalResult plan_traversal(const std::vector<TraversalTask>& tasks,
                               const TraversalSettings& settings);

}  // namespace wave
