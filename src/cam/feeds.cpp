#include "cam/feeds.h"

#include <algorithm>
#include <cmath>

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
                        double plunge_feed) {
  FeedSchedule schedule;
  if (toolpath.moves.empty()) return schedule;

  const double slow = std::max(full_engagement_feed, 1.0);
  const double fast = std::max(no_engagement_feed, slow);
  const double plunge = std::max(plunge_feed, 1.0);
  const double rapid = std::max(rapid_feed(machine), 1.0);
  schedule.stats.spindle_rpm = commandable_rpm(machine, spindle_rpm);

  std::vector<double> move_start(toolpath.moves.size(), 0.0);
  for (std::size_t index = 1; index < toolpath.moves.size(); ++index)
    move_start[index] = move_start[index - 1] + toolpath.moves[index - 1].length;

  for (std::size_t index = 0; index < toolpath.moves.size(); ++index) {
    const wave::ToolpathMove& move = toolpath.moves[index];
    double along = 0.0;

    if (move.kind == wave::MoveKind::Retract ||
        move.kind == wave::MoveKind::Entry) {
      for (std::size_t i = 1; i < move.path.size(); ++i) {
        FeedSegment segment;
        segment.from = move.path[i - 1];
        segment.to = move.path[i];
        segment.move_index = index;
        segment.kind = move.kind;
        segment.rapid = move.kind == wave::MoveKind::Retract;
        segment.limit = segment.rapid ? FeedLimit::Rapid : FeedLimit::Plunge;
        segment.length = distance_mm(segment.from, segment.to);
        segment.start_distance = move_start[index] + along;
        segment.feed = segment.rapid ? 0.0 : plunge;
        segment.rate = segment.rapid ? rapid : plunge;
        along += segment.length;
        if (segment.length > 0.0) schedule.segments.push_back(segment);
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
  return schedule;
}

double feed_schedule_seconds(const FeedSchedule& schedule) {
  double total = 0.0;
  for (const FeedSegment& segment : schedule.segments)
    total += segment.length / (std::max(segment.rate, 1.0) / 60.0);
  return total;
}

double feed_seconds_to(const FeedSchedule& schedule, double distance) {
  if (!(distance > 0.0)) return 0.0;
  double total = 0.0;
  for (const FeedSegment& segment : schedule.segments) {
    if (segment.start_distance >= distance) break;
    const double covered =
        std::min(segment.length, distance - segment.start_distance);
    total += covered / (std::max(segment.rate, 1.0) / 60.0);
  }
  return total;
}

double feed_distance_after(const FeedSchedule& schedule, double distance,
                           double seconds) {
  if (schedule.segments.empty() || !(seconds > 0.0)) return distance;
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
