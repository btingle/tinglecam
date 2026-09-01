#pragma once

#include "cam/machine.h"
#include "simulation.h"
#include "toolpath.h"

#include <cstddef>
#include <optional>
#include <vector>

namespace cam {

enum class FeedLimit {
  Engagement,
  Rapid,
  Plunge,
};

const char* feed_limit_name(FeedLimit value);
bool feed_limit_is_binding(FeedLimit value);

struct FeedSegment {
  wave::Point64 from;
  wave::Point64 to;
  std::size_t move_index = 0;
  wave::MoveKind kind = wave::MoveKind::Cut;
  bool rapid = false;
  // A Retract lands in already-cleared space when its simulation check found
  // no stock under the cutter. The post may then return to depth with G0.
  bool safe_retract = false;
  double length = 0.0;
  double feed = 0.0;
  double start_distance = 0.0;
  double rate = 0.0;
  double engagement = 0.0;
  FeedLimit limit = FeedLimit::Engagement;
};

struct FeedStats {
  double spindle_rpm = 0.0;
  double cutting_distance = 0.0;
  double rapid_distance = 0.0;
  double cutting_minutes = 0.0;
  double rapid_minutes = 0.0;
  double minimum_feed = 0.0;
  double maximum_feed = 0.0;
  double maximum_engagement = 0.0;
};

// A commanded machine motion.  Unlike FeedSegment, this also represents the
// vertical motions which surround retract traversals.  path_distance tracks
// the corresponding distance through the planar toolpath and deliberately
// remains constant while the cutter is moving vertically.
struct MotionPhase {
  wave::Point64 from;
  wave::Point64 to;
  double from_z = 0.0;
  double to_z = 0.0;
  double path_distance_begin = 0.0;
  double path_distance_end = 0.0;
  double length = 0.0;
  double rate = 0.0;
  wave::MoveKind kind = wave::MoveKind::Cut;
  bool rapid = false;
  // The opening rapid that carries the cutter from where the previous pass
  // left it to where this one starts. It belongs to no pass's geometry, so it
  // is flagged rather than inferred from being horizontal and distance-free.
  bool arrival = false;
};

struct MotionCursor {
  wave::Point64 position;
  double z = 0.0;
  double path_distance = 0.0;
  wave::MoveKind kind = wave::MoveKind::Cut;
  bool rapid = false;
  double rate = 0.0;
  double phase_distance = 0.0;
  double phase_length = 0.0;
  double motion_distance = 0.0;
  double from_z = 0.0;
  double to_z = 0.0;
};

struct FeedSchedule {
  std::vector<FeedSegment> segments;
  std::vector<MotionPhase> motion;
  FeedStats stats;
};

// Maps simulation engagement directly onto the configured feed range. At full
// engagement the commanded feed is `full_engagement_feed`; at zero engagement
// it is `no_engagement_feed`. Collinear pieces are merged only after feeds
// have been assigned, and only when those feeds are compatible.
//
// `arrive_from` is where the cutter was left standing, at the rapid plane, by
// whatever ran before this pass. Given one, the schedule opens with the rapid
// that carries the tool across to where this pass starts. Without it the pass
// is assumed to begin wherever it likes, which is true only of the first one.
FeedSchedule plan_feeds(const wave::Toolpath& toolpath,
                        const wave::SimulationResult& simulation,
                        const Machine& machine,
                        double full_engagement_feed,
                        double no_engagement_feed,
                        double spindle_rpm,
                        double plunge_feed,
                        double floor_z = 0.0,
                        double rapid_plane = 0.0,
                        std::optional<wave::Point64> arrive_from = std::nullopt);

double feed_schedule_seconds(const FeedSchedule& schedule);
double feed_motion_length(const FeedSchedule& schedule);
double feed_seconds_to(const FeedSchedule& schedule, double distance);
double feed_distance_at_seconds(const FeedSchedule& schedule, double seconds);
MotionCursor feed_motion_at_seconds(const FeedSchedule& schedule,
                                    double seconds);
double feed_distance_after(const FeedSchedule& schedule, double distance,
                           double seconds);

}  // namespace cam
