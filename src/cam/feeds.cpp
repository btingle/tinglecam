#include "cam/feeds.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cam {
namespace {

double distance_mm(const wave::Point64& a, const wave::Point64& b) {
  return std::hypot(wave::to_world(b.x - a.x), wave::to_world(b.y - a.y));
}

bool engaged_kind(wave::MoveKind kind) {
  return kind == wave::MoveKind::Cut || kind == wave::MoveKind::Step ||
         kind == wave::MoveKind::Link;
}

bool same_line(const FeedSegment& a, const FeedSegment& b) {
  if (a.to != b.from) return false;
  const long double ax = static_cast<long double>(a.to.x - a.from.x);
  const long double ay = static_cast<long double>(a.to.y - a.from.y);
  const long double bx = static_cast<long double>(b.to.x - b.from.x);
  const long double by = static_cast<long double>(b.to.y - b.from.y);
  const long double cross = ax * by - ay * bx;
  const long double scale =
      std::max(std::hypot(ax, ay) * std::hypot(bx, by), 1.0L);
  return std::abs(cross) <= scale * 1e-10L && ax * bx + ay * by > 0.0L;
}

bool compatible(const FeedSegment& a, const FeedSegment& b) {
  return a.kind == b.kind && a.rapid == b.rapid &&
         std::llround(a.feed) == std::llround(b.feed) && same_line(a, b);
}

void merge_compatible_segments(std::vector<FeedSegment>& segments) {
  std::vector<FeedSegment> merged;
  merged.reserve(segments.size());
  for (const FeedSegment& segment : segments) {
    if (!merged.empty() && compatible(merged.back(), segment)) {
      FeedSegment& run = merged.back();
      run.to = segment.to;
      run.length += segment.length;
      run.engagement = std::max(run.engagement, segment.engagement);
      run.feed = std::min(run.feed, segment.feed);
      run.rate = std::min(run.rate, segment.rate);
      continue;
    }
    merged.push_back(segment);
  }
  segments = std::move(merged);
}

double phase_seconds(const MotionPhase& phase) {
  return phase.length / (std::max(phase.rate, 1.0) / 60.0);
}

wave::Point64 interpolate(const wave::Point64& from,
                          const wave::Point64& to, double t) {
  return {
      static_cast<std::int64_t>(std::llround(
          static_cast<double>(from.x) +
          static_cast<double>(to.x - from.x) * t)),
      static_cast<std::int64_t>(std::llround(
          static_cast<double>(from.y) +
          static_cast<double>(to.y - from.y) * t)),
  };
}

wave::Point64 interpolate_arc(const MotionPhase& phase, double t) {
  if (!phase.arc) return interpolate(phase.from, phase.to, t);
  if (t >= 1.0) return phase.to;
  const double cx = wave::to_world(phase.arc_center.x);
  const double cy = wave::to_world(phase.arc_center.y);
  const double x = wave::to_world(phase.from.x);
  const double y = wave::to_world(phase.from.y);
  const double radius = std::hypot(x - cx, y - cy);
  const double start = std::atan2(y - cy, x - cx);
  const double angle = start + (phase.clockwise ? -1.0 : 1.0) *
                                   2.0 * std::acos(-1.0) * t;
  return {wave::to_integer(cx + radius * std::cos(angle)),
          wave::to_integer(cy + radius * std::sin(angle))};
}

}  // namespace

const char* feed_limit_name(FeedLimit value) {
  switch (value) {
    case FeedLimit::Engagement: return "engagement";
    case FeedLimit::Rapid: return "rapid";
    case FeedLimit::Plunge: return "plunge";
  }
  return "unknown";
}

bool feed_limit_is_binding(FeedLimit) { return false; }

FeedSchedule plan_feeds(const wave::Toolpath& toolpath,
                        const wave::SimulationResult& simulation,
                        const Machine& machine,
                        double full_engagement_feed,
                        double no_engagement_feed,
                        double spindle_rpm,
                        double plunge_feed,
                        double floor_z,
                        double rapid_plane,
                        PassLink link,
                        bool allow_plunge) {
  FeedSchedule schedule;
  if (toolpath.moves.empty()) return schedule;

  const double slow = std::max(full_engagement_feed, 1.0);
  const double fast = std::max(no_engagement_feed, slow);
  const double plunge = std::max(plunge_feed, 1.0);
  const double rapid = std::max(rapid_feed(machine), 1.0);
  const double rapid_z = std::max(rapid_feed_z(machine), 1.0);
  schedule.stats.spindle_rpm = commandable_rpm(machine, spindle_rpm);

  std::vector<double> move_start(toolpath.moves.size(), 0.0);
  for (std::size_t index = 1; index < toolpath.moves.size(); ++index)
    move_start[index] = move_start[index - 1] + toolpath.moves[index - 1].length;

  for (std::size_t index = 0; index < toolpath.moves.size(); ++index) {
    const wave::ToolpathMove& move = toolpath.moves[index];
    double along = 0.0;

    if (move.kind == wave::MoveKind::Retract ||
        move.kind == wave::MoveKind::Entry) {
      bool emitted = false;
      const bool safe_retract =
          move.kind == wave::MoveKind::Retract &&
          index < simulation.moves.size() &&
          !(simulation.moves[index].plunge_collision_area > 0.0);
      if (!allow_plunge && move.kind == wave::MoveKind::Entry)
        throw std::runtime_error(
            "roughing pass contained an entry move; the move was omitted");
      if (!allow_plunge && move.kind == wave::MoveKind::Retract &&
          !safe_retract)
        throw std::runtime_error(
            "roughing pass has an unsafe retract landing; the level was omitted");
      for (std::size_t i = 1; i < move.path.size(); ++i) {
        FeedSegment segment;
        segment.from = move.path[i - 1];
        segment.to = move.path[i];
        segment.move_index = index;
        segment.kind = move.kind;
        segment.rapid = move.kind == wave::MoveKind::Retract;
        segment.safe_retract = safe_retract;
        segment.limit = segment.rapid ? FeedLimit::Rapid : FeedLimit::Plunge;
        segment.length = distance_mm(segment.from, segment.to);
        segment.start_distance = move_start[index] + along;
        segment.feed = segment.rapid ? 0.0 : plunge;
        segment.rate = segment.rapid ? rapid : plunge;
        along += segment.length;
        if (segment.length > 0.0) {
          schedule.segments.push_back(segment);
          emitted = true;
        }
      }
      // Keep a marker for an opening retract whose XY endpoints coincide.
      // Although it has no planar length, it is the proof that the following
      // descent lands in cleared air and is therefore eligible for G0.
      if (!emitted && move.kind == wave::MoveKind::Retract &&
          !move.path.empty()) {
        FeedSegment marker;
        marker.from = move.path.front();
        marker.to = move.path.back();
        marker.move_index = index;
        marker.kind = wave::MoveKind::Retract;
        marker.rapid = true;
        marker.safe_retract = safe_retract;
        marker.limit = FeedLimit::Rapid;
        marker.start_distance = move_start[index];
        marker.rate = rapid;
        schedule.segments.push_back(marker);
      }
      continue;
    }

    if (!engaged_kind(move.kind) || index >= simulation.moves.size()) continue;
    for (const wave::MaterialStep& step : simulation.moves[index].steps) {
      FeedSegment segment;
      segment.from = step.from;
      segment.to = step.to;
      segment.move_index = index;
      segment.kind = move.kind;
      segment.length = distance_mm(step.from, step.to);
      segment.start_distance = move_start[index] + step.move_distance_begin;
      if (!(segment.length > 0.0)) continue;
      segment.engagement = std::clamp(step.load, 0.0, 1.0);
      segment.feed =
          slow + (fast - slow) * (1.0 - segment.engagement);
      segment.rate = segment.feed;
      segment.limit = FeedLimit::Engagement;
      schedule.segments.push_back(segment);
    }
  }

  // Preserve the unmerged segments while constructing the commanded machine
  // timeline.  The post lifts before every retract traversal, plunges before
  // the next at-depth move, and lifts once more at the end of the pass.
  const std::vector<FeedSegment> unmerged_segments = schedule.segments;
  auto append_motion = [&](const wave::Point64& from,
                           const wave::Point64& to, double from_z,
                           double to_z, double path_begin, double path_end,
                           double rate, wave::MoveKind kind, bool is_rapid) {
    MotionPhase phase;
    phase.from = from;
    phase.to = to;
    phase.from_z = from_z;
    phase.to_z = to_z;
    phase.path_distance_begin = path_begin;
    phase.path_distance_end = path_end;
    phase.length = std::hypot(distance_mm(from, to), to_z - from_z);
    // A rapid is commanded as separate moves -- straight up, across the plane,
    // straight down -- so a phase is one or the other and takes the rate of the
    // axis that actually moves. A cutting move keeps the rate it was given.
    phase.rate = is_rapid && to_z != from_z ? rapid_z : rate;
    phase.kind = kind;
    phase.rapid = is_rapid;
    if (phase.length > 0.0) schedule.motion.push_back(phase);
  };

  bool at_depth = false;
  bool rapid_descent = false;
  // Only the first descent starts from where the previous pass left the cutter.
  // A retract inside this pass goes to the plane and comes back from it, the
  // same as it always did.
  bool first_descent = true;
  schedule.begins_at_depth = link.from.has_value() && link.from_z != rapid_plane;
  wave::Point64 position{};
  bool have_position = false;
  std::size_t segment_index = 0;
  for (std::size_t index = 0; index < toolpath.moves.size(); ++index) {
    const wave::ToolpathMove& move = toolpath.moves[index];
    const double path_begin = move_start[index];
    if (move.kind == wave::MoveKind::Retract) {
      if (move.path.empty()) continue;
      while (segment_index < unmerged_segments.size() &&
             unmerged_segments[segment_index].move_index < index)
        ++segment_index;
      const std::size_t first_segment = segment_index;
      while (segment_index < unmerged_segments.size() &&
             unmerged_segments[segment_index].move_index == index)
        ++segment_index;
      const wave::Point64 from = move.path.front();
      if (at_depth) {
        append_motion(from, from, floor_z, rapid_plane, path_begin,
                      path_begin, rapid, wave::MoveKind::Retract, true);
      }
      for (std::size_t i = first_segment; i < segment_index; ++i) {
        const FeedSegment& segment = unmerged_segments[i];
        append_motion(segment.from, segment.to, rapid_plane, rapid_plane,
                      segment.start_distance,
                      segment.start_distance + segment.length, rapid,
                      wave::MoveKind::Retract, true);
      }
      rapid_descent = first_segment < segment_index &&
                      unmerged_segments[first_segment].safe_retract;
      position = move.path.back();
      have_position = true;
      at_depth = false;
      continue;
    }

    while (segment_index < unmerged_segments.size() &&
           unmerged_segments[segment_index].move_index < index)
      ++segment_index;
    const std::size_t first_segment = segment_index;
    while (segment_index < unmerged_segments.size() &&
           unmerged_segments[segment_index].move_index == index)
      ++segment_index;
    if (first_segment == segment_index) continue;

    const FeedSegment& first = unmerged_segments[first_segment];
    if (!at_depth) {
      const bool use_rapid = rapid_descent && move.kind != wave::MoveKind::Entry;
      if (!allow_plunge && !use_rapid)
        throw std::runtime_error(
            "roughing pass requires a plunge descent; the level was omitted");
      const double from_z =
          first_descent && link.from ? link.from_z : rapid_plane;
      append_motion(first.from, first.from, from_z, floor_z,
                    first.start_distance, first.start_distance,
                    use_rapid ? rapid : plunge,
                    use_rapid ? wave::MoveKind::Retract
                              : wave::MoveKind::Entry,
                    use_rapid);
      at_depth = true;
      rapid_descent = false;
      first_descent = false;
    }
    for (std::size_t i = first_segment; i < segment_index; ++i) {
      const FeedSegment& segment = unmerged_segments[i];
      append_motion(segment.from, segment.to, floor_z, floor_z,
                    segment.start_distance,
                    segment.start_distance + segment.length, segment.rate,
                    segment.kind, segment.rapid);
      position = segment.to;
      have_position = true;
    }
  }
  if (at_depth && have_position && !link.stay_down_after) {
    const double path_end = wave::toolpath_length(toolpath);
    append_motion(position, position, floor_z, rapid_plane, path_end,
                  path_end, rapid, wave::MoveKind::Retract, true);
  }
  schedule.ends_at_depth = at_depth && have_position && link.stay_down_after;

  // Whatever ran before this pass left the cutter standing somewhere, and the
  // descent above starts from wherever this pass happens to begin. The move
  // between the two is a real commanded one, so the schedule carries it rather
  // than leaving the tool to appear at the new pass out of nowhere. It covers
  // no path distance, exactly as the descent that follows it does.
  //
  // A pass reached without lifting has nothing left to cross: the pass before
  // it already travelled to this one's first point, at depth, as part of its
  // own geometry. The traverse is then zero length and drops out below.
  if (link.from && !schedule.motion.empty()) {
    const MotionPhase& opening = schedule.motion.front();
    MotionPhase traverse;
    traverse.from = *link.from;
    traverse.to = opening.from;
    traverse.from_z = link.from_z;
    traverse.to_z = link.from_z;
    traverse.path_distance_begin = opening.path_distance_begin;
    traverse.path_distance_end = opening.path_distance_begin;
    traverse.length = distance_mm(traverse.from, traverse.to);
    traverse.rate = rapid;
    traverse.kind = wave::MoveKind::Retract;
    traverse.rapid = true;
    traverse.arrival = true;
    if (traverse.length > 0.0) {
      schedule.motion.insert(schedule.motion.begin(), traverse);
      // The travel totals below count safe-plane traverses through the
      // segments that produced them and vertical moves through the phases.
      // This one has no segment and is not vertical, so it is added here
      // rather than by loosening either of those and double counting.
      schedule.stats.rapid_distance += traverse.length;
      schedule.stats.rapid_minutes += traverse.length / traverse.rate;
    }
  }

  // Geometry reduction belongs here: feeds now exist, so only pieces that the
  // post would command at the same rounded rate are eligible to become one.
  merge_compatible_segments(schedule.segments);

  for (const FeedSegment& segment : schedule.segments) {
    if (segment.rapid) {
      schedule.stats.rapid_distance += segment.length;
      schedule.stats.rapid_minutes += segment.length / segment.rate;
      continue;
    }
    schedule.stats.cutting_distance += segment.length;
    schedule.stats.cutting_minutes += segment.length / segment.rate;
    schedule.stats.minimum_feed =
        schedule.stats.minimum_feed > 0.0
            ? std::min(schedule.stats.minimum_feed, segment.feed)
            : segment.feed;
    schedule.stats.maximum_feed =
        std::max(schedule.stats.maximum_feed, segment.feed);
    schedule.stats.maximum_engagement =
        std::max(schedule.stats.maximum_engagement, segment.engagement);
  }
  // Vertical positioning is non-cutting travel even when the descent uses the
  // tool's plunge feed.  Keep it in the travel totals so the job estimate and
  // the commanded-motion playback account for the same moves.
  for (const MotionPhase& phase : schedule.motion) {
    if (phase.from_z == phase.to_z) continue;
    schedule.stats.rapid_distance += phase.length;
    schedule.stats.rapid_minutes += phase.length / std::max(phase.rate, 1.0);
  }
  return schedule;
}

double feed_schedule_seconds(const FeedSchedule& schedule) {
  double total = 0.0;
  if (!schedule.motion.empty()) {
    for (const MotionPhase& phase : schedule.motion)
      total += phase_seconds(phase);
    return total;
  }
  for (const FeedSegment& segment : schedule.segments)
    total += segment.length / (std::max(segment.rate, 1.0) / 60.0);
  return total;
}

double feed_motion_length(const FeedSchedule& schedule) {
  double total = 0.0;
  for (const MotionPhase& phase : schedule.motion) total += phase.length;
  if (!schedule.motion.empty()) return total;
  for (const FeedSegment& segment : schedule.segments) total += segment.length;
  return total;
}

double feed_seconds_to(const FeedSchedule& schedule, double distance) {
  if (!(distance > 0.0)) return 0.0;
  double total = 0.0;
  if (!schedule.motion.empty()) {
    for (const MotionPhase& phase : schedule.motion) {
      if (phase.path_distance_begin == phase.path_distance_end) {
        if (phase.path_distance_end <= distance) total += phase_seconds(phase);
        continue;
      }
      if (phase.path_distance_begin >= distance) break;
      const double fraction = std::clamp(
          (distance - phase.path_distance_begin) /
              (phase.path_distance_end - phase.path_distance_begin),
          0.0, 1.0);
      total += phase_seconds(phase) * fraction;
      if (fraction < 1.0) break;
    }
    return total;
  }
  for (const FeedSegment& segment : schedule.segments) {
    if (segment.start_distance >= distance) break;
    const double covered =
        std::min(segment.length, distance - segment.start_distance);
    total += covered / (std::max(segment.rate, 1.0) / 60.0);
  }
  return total;
}

MotionCursor feed_motion_at_seconds(const FeedSchedule& schedule,
                                    double seconds) {
  MotionCursor cursor;
  if (schedule.motion.empty()) return cursor;
  if (!(seconds > 0.0)) {
    const MotionPhase& phase = schedule.motion.front();
    cursor.position = phase.from;
    cursor.z = phase.from_z;
    cursor.path_distance = phase.path_distance_begin;
    cursor.kind = phase.kind;
    cursor.rapid = phase.rapid;
    cursor.rate = phase.rate;
    cursor.phase_length = phase.length;
    cursor.from_z = phase.from_z;
    cursor.to_z = phase.to_z;
    return cursor;
  }

  double elapsed = 0.0;
  double travelled = 0.0;
  for (const MotionPhase& phase : schedule.motion) {
    const double duration = phase_seconds(phase);
    if (seconds <= elapsed + duration) {
      const double t = duration > 0.0
                           ? std::clamp((seconds - elapsed) / duration, 0.0,
                                        1.0)
                           : 1.0;
      cursor.position = interpolate_arc(phase, t);
      cursor.z = phase.from_z + (phase.to_z - phase.from_z) * t;
      cursor.path_distance =
          phase.path_distance_begin +
          (phase.path_distance_end - phase.path_distance_begin) * t;
      cursor.kind = phase.kind;
      cursor.rapid = phase.rapid;
      cursor.rate = phase.rate;
      cursor.phase_distance = phase.length * t;
      cursor.phase_length = phase.length;
      cursor.motion_distance = travelled + cursor.phase_distance;
      cursor.from_z = phase.from_z;
      cursor.to_z = phase.to_z;
      return cursor;
    }
    elapsed += duration;
    travelled += phase.length;
  }

  const MotionPhase& phase = schedule.motion.back();
  cursor.position = phase.to;
  cursor.z = phase.to_z;
  cursor.path_distance = phase.path_distance_end;
  cursor.kind = phase.kind;
  cursor.rapid = phase.rapid;
  cursor.rate = phase.rate;
  cursor.phase_distance = phase.length;
  cursor.phase_length = phase.length;
  cursor.motion_distance = travelled;
  cursor.from_z = phase.from_z;
  cursor.to_z = phase.to_z;
  return cursor;
}

double feed_distance_at_seconds(const FeedSchedule& schedule, double seconds) {
  if (!schedule.motion.empty())
    return feed_motion_at_seconds(schedule, seconds).path_distance;
  return feed_distance_after(schedule, 0.0, seconds);
}

double feed_distance_after(const FeedSchedule& schedule, double distance,
                           double seconds) {
  if (schedule.segments.empty() || !(seconds > 0.0)) return distance;
  if (!schedule.motion.empty()) {
    return feed_distance_at_seconds(
        schedule, feed_seconds_to(schedule, distance) + seconds);
  }
  auto current = std::upper_bound(
      schedule.segments.begin(), schedule.segments.end(), distance,
      [](double value, const FeedSegment& segment) {
        return value < segment.start_distance;
      });
  if (current != schedule.segments.begin()) --current;

  double where = distance;
  double left = seconds;
  for (; current != schedule.segments.end(); ++current) {
    const double end = current->start_distance + current->length;
    if (end <= where) continue;
    where = std::max(where, current->start_distance);
    const double mm_per_second = std::max(current->rate, 1.0) / 60.0;
    const double needed = (end - where) / mm_per_second;
    if (needed >= left) return where + left * mm_per_second;
    left -= needed;
    where = end;
  }
  return where;
}

}  // namespace cam
