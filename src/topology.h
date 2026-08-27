#pragma once

#include "geometry.h"

#include <cstddef>
#include <vector>

namespace wave {

struct LeadingFront {
  Path64 path;
  bool closed = false;
  std::vector<std::size_t> parents;
  std::vector<std::size_t> children;
};

struct FrontGeneration {
  std::vector<LeadingFront> fronts;
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
// associates consecutive front components using the same engagement offset as
// the wave propagation.
FrontLineage build_front_lineage(const WaveResult& wave, double engagement);

Point64 representative_point(const LeadingFront& front);
double front_length(const LeadingFront& front);
const char* lineage_event_kind_name(LineageEventKind kind);

}  // namespace wave
