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
  // Lift to clearance, rapid, plunge again. The path holds only the rapid's
  // endpoints; a zero-length retract is the initial plunge.
  Retract,
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
  std::size_t cut_count = 0;
  std::size_t step_count = 0;
  std::size_t link_count = 0;
  std::size_t retract_count = 0;
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
Toolpath build_toolpath(const WaveResult& wave, const FrontLineage& lineage);

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
