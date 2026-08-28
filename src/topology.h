#pragma once

#include "geometry.h"

#include <cstddef>
#include <vector>

namespace wave {

struct LeadingFront {
  Path64 path;
  bool closed = false;
  // Index into FrontGeneration::bands of the connected piece of wave band this
  // front was carved out of. A piece can host several fronts, so material is
  // only credited as cleared once every front sharing it has been cut.
  std::size_t band = 0;
  std::vector<std::size_t> parents;
  std::vector<std::size_t> children;
};

struct FrontGeneration {
  std::vector<LeadingFront> fronts;
  // The generation's wave band split into connected pieces, each an outer
  // contour followed by its holes. Grouping matters: a bare hole ring taken on
  // its own describes the material it encloses rather than the band.
  std::vector<Paths64> bands;
};

enum class LineageEventKind {
  Split,
  Merge,
  Termination,
  Birth,
};

struct LineageEvent {
  LineageEventKind kind = LineageEventKind::Split;
  std::size_t generation = 0;
  std::vector<std::size_t> previous_fronts;
  std::vector<std::size_t> current_fronts;
  Point64 position;
};

struct FrontLineage {
  std::vector<FrontGeneration> generations;
  std::vector<LineageEvent> events;
};

// Extracts the reached/unreached interface from each vector wave band, then
// associates consecutive front components using the same stepover offset as
// the wave propagation.
FrontLineage build_front_lineage(const WaveResult& wave);

Point64 representative_point(const LeadingFront& front);
double front_length(const LeadingFront& front);
const char* lineage_event_kind_name(LineageEventKind kind);

}  // namespace wave
