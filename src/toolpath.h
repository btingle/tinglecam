#pragma once

#include "geometry.h"
#include "topology.h"

#include <cstddef>
#include <vector>

namespace wave {

// Identifies one leading front within a FrontLineage.
struct FrontId {
  std::size_t generation = 0;
  std::size_t index = 0;
};

enum class MoveKind {
  // Following a leading front at cutting feed.
  Cut,
  // Crossing into the band the next front bounds. The tool is engaged for the
  // width of one stepover, which is the same bite the front itself takes, so
  // this is a cutting move rather than free travel.
  Step,
  // Travel at depth through material that has already been cleared.
  Link,
  // Lift to clearance, rapid, and plunge back down. The plunge lands in cleared
  // material, so the descent itself cuts nothing; the path holds only the
  // rapid's endpoints.
  Retract,
  // Entering material that has not been cleared, which no straight descent can
  // do. The tool has to ramp or helix in, and the first move of any toolpath is
  // one of these.
  Entry,
};

struct ToolpathMove {
  MoveKind kind = MoveKind::Cut;
  Path64 path;
  // Meaningful for Cut moves only.
  FrontId front;
  double length = 0.0;
};

struct ToolpathStats {
  double cut_length = 0.0;
  double step_length = 0.0;
  double link_length = 0.0;
  double retract_length = 0.0;
  double entry_length = 0.0;
  std::size_t cut_count = 0;
  std::size_t step_count = 0;
  std::size_t link_count = 0;
  std::size_t retract_count = 0;
  std::size_t entry_count = 0;
  // Retracts routed around an obstruction at depth instead of lifting.
  std::size_t detour_count = 0;
  // Lifts whose rapid had to pass over the subject because the free space
  // around it is disconnected. Each one assumes the tool clears the part.
  std::size_t lifts_over_subject = 0;
  // Fronts the sequencer could not reach because their parents never were.
  std::size_t unreachable_fronts = 0;
};

struct Toolpath {
  std::vector<ToolpathMove> moves;
  ToolpathStats stats;
};

// Sequences every leading front into a single ordered tool motion. Fronts are
// cut in an order that respects lineage precedence, so a front is never entered
// before the parents that opened the way to it. Consecutive cuts are joined by
// a straight link when the link stays inside the material cleared so far, and
// by a retract otherwise. A link that only crosses the band its target front
// bounds is reported as a step: engaged, but no deeper than one stepover.
//
// When the straight segment is blocked, both a detour through the travelable
// region and a retract/rapid/approach are planned. Their complete represented
// XY distances are compared and the shorter move is emitted. A lift sets down
// in cleared material; only where no cleared landing exists is an entry
// emitted, and that one has to be ramped. The rapid itself is kept out of the
// subject as well, since a single slice says nothing about how far the part
// rises above it.
//
// `tool_radius` sizes the clearance a descent needs: the tool cuts everything
// within a radius of its centre, so that is how far inside cleared material a
// plunge has to land. Zero or negative falls back to half the stepover, which
// is what the planner assumed before the radius was available to it.
Toolpath build_toolpath(const WaveResult& wave, const FrontLineage& lineage,
                        double tool_radius = 0.0);

// Resamples every uninterrupted at-depth cutting path at a uniform travelled
// distance. Original micro-vertices are not retained; motion-type changes are
// assigned by the midpoint of each resulting interval. Retracts and entries
// remain separate and discontinuities restart the pitch.
Toolpath resample_toolpath(const Toolpath& toolpath, double pitch);

// Where the tool is after travelling a given distance along the whole path.
struct ToolpathCursor {
  std::size_t move = 0;
  Point64 position;
  MoveKind kind = MoveKind::Cut;
  // How far into `move` the cursor sits, so a partially finished move can be
  // drawn up to the tool and no further.
  double move_distance = 0.0;
};

double toolpath_length(const Toolpath& toolpath);

// Distance from the start of the path to where `move` begins.
double toolpath_move_start(const Toolpath& toolpath, std::size_t move);

// Clamps to the path: a distance past the end returns the final point.
ToolpathCursor toolpath_at(const Toolpath& toolpath, double distance);

const char* move_kind_name(MoveKind kind);

}  // namespace wave
