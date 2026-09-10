#pragma once

#include "wave/geometry.h"
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
  // Cutter-centre region an at-depth connector may travel through: everywhere
  // the tool may be at this depth, which is the space beside the stock plus
  // whatever has been cleared on this layer already. Empty, or
  // allow_stay_down=false, makes the safe retract the only connector.
  Paths64 stay_down_region;
  bool allow_stay_down = true;
  // How far a connector may stray outside that region, in integer units. Given
  // one, a connector whose straight line is blocked is routed around what
  // blocks it rather than abandoned to a lift. Zero leaves the straight line as
  // the only at-depth option, which is what this planner offered before.
  double stay_down_slack = 0.0;
  double cutting_feed = 1000.0;
  // Lateral and vertical rapid rates. A lift is costed as the two separately,
  // never as one distance through both, because that is how it is commanded:
  // straight up, across the plane, straight down. Zero uses the lateral rate.
  double rapid_feed = 2000.0;
  double rapid_feed_z = 0.0;
  double floor_z = 0.0;
  // Absolute Z of the one plane a lift traverses at, not a height above it.
  double rapid_plane = 10.0;
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
// is costed at the full trip to `rapid_plane` and back.
TraversalResult plan_traversal(const std::vector<TraversalTask>& tasks,
                               const TraversalSettings& settings);

}  // namespace wave
