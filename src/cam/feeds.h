#pragma once

#include "cam/machine.h"
#include "simulation.h"
#include "toolpath.h"

#include <cstddef>
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

struct FeedSchedule {
  std::vector<FeedSegment> segments;
  FeedStats stats;
};

// Maps simulation engagement directly onto the configured feed range. At full
// engagement the commanded feed is `full_engagement_feed`; at zero engagement
// it is `no_engagement_feed`. Collinear pieces are merged only after feeds
// have been assigned, and only when those feeds are compatible.
FeedSchedule plan_feeds(const wave::Toolpath& toolpath,
                        const wave::SimulationResult& simulation,
                        const Machine& machine,
                        double full_engagement_feed,
                        double no_engagement_feed,
                        double spindle_rpm,
                        double plunge_feed);

double feed_schedule_seconds(const FeedSchedule& schedule);
double feed_seconds_to(const FeedSchedule& schedule, double distance);
double feed_distance_after(const FeedSchedule& schedule, double distance,
                           double seconds);

}  // namespace cam
