#include "topology.h"

namespace wave {

Point64 representative_point(const LeadingFront& front) {
  if (front.path.empty()) return Point64();
  if (front.path.size() == 1) return front.path.front();
  long double total_length = 0.0L;
  const std::size_t segment_count = front.closed ? front.path.size() : front.path.size() - 1;
  for (std::size_t i = 0; i < segment_count; ++i) {
    const auto& a = front.path[i];
    const auto& b = front.path[(i + 1) % front.path.size()];
    total_length += std::hypotl(static_cast<long double>(b.x - a.x),
                               static_cast<long double>(b.y - a.y));
  }
  long double target = total_length * 0.5L;
  for (std::size_t i = 0; i < segment_count; ++i) {
    const auto& a = front.path[i];
    const auto& b = front.path[(i + 1) % front.path.size()];
    const long double length = std::hypotl(static_cast<long double>(b.x - a.x),
                                          static_cast<long double>(b.y - a.y));
    if (target <= length && length > 0.0L) {
      const long double t = target / length;
      return Point64(static_cast<std::int64_t>(std::llround(a.x + (b.x - a.x) * t)),
                     static_cast<std::int64_t>(std::llround(a.y + (b.y - a.y) * t)));
    }
    target -= length;
  }
  return front.path.back();
}

double front_length(const LeadingFront& front) {
  double length = 0.0;
  if (front.path.size() < 2) return length;
  const std::size_t segment_count = front.closed ? front.path.size() : front.path.size() - 1;
  for (std::size_t i = 0; i < segment_count; ++i) {
    const auto& a = front.path[i];
    const auto& b = front.path[(i + 1) % front.path.size()];
    length += std::hypot(to_world(b.x - a.x), to_world(b.y - a.y));
  }
  return length;
}

const char* lineage_event_kind_name(LineageEventKind kind) {
  switch (kind) {
    case LineageEventKind::Split: return "split";
    case LineageEventKind::Merge: return "merge";
    case LineageEventKind::MergeTermination: return "merge termination";
    case LineageEventKind::Termination: return "termination";
    case LineageEventKind::Birth: return "birth (unexpected)";
  }
  return "unknown";
}
}  // namespace wave
