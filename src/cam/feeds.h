#pragma once

#include "cam/machine.h"
#include "cam/tooling.h"
#include "planning/toolpath.h"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace cam {

enum class FeedLimit {
  Engagement,
  Rapid,
  Plunge,
  // Held at the machine's maximum feed.
  Machine,
};

// Per-operation replacements for the tool's cutting data. Empty means "use the
// tool's figure for the job material".
struct CuttingOverrides {
  std::optional<double> spindle_rpm;
  std::optional<double> chip_load;
  std::optional<double> plunge_feed;
};

// The feeds one operation runs at, resolved from the tool, the job material,
// the operation's overrides and the machine's limits.
struct CuttingFeeds {
  double spindle_rpm = 0.0;  // as the machine can command it
  double chip_load = 0.0;
  int flutes = 1;
  double plunge_feed = 0.0;
  double maximum_feed = 0.0;
  // mm/min at full radial engagement.
  double full_feed() const;
  // Radial chip thinning: at lighter engagement the same chip needs a faster
  // feed. Engagement is removed volume over a full slot, so 0.5 and above is
  // treated as full.
  double feed_at(double engagement, FeedLimit* limit = nullptr) const;
  // Feed through cleared material.
  double travel_feed() const { return feed_at(0.0); }
};

// Throws with a message naming what is missing when the tool has no cutting
// data for the material and the overrides do not cover it.
CuttingFeeds resolve_cutting_feeds(const Tool& tool, const Machine& machine,
                                   const std::string& material_id,
                                   const CuttingOverrides& overrides);

const char* feed_limit_name(FeedLimit value);

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
  double mean_engagement = 0.0;
};

// A commanded machine motion.  Unlike FeedSegment, this also represents the
// vertical motions which surround retract traversals.  path_distance tracks
// the corresponding distance through the planar toolpath and deliberately
// remains constant while the cutter is moving vertically.
struct MotionPhase {
  double removed_volume = 0.0;
  double engagement = 0.0;
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
  // A circular XY move. The endpoint and centre are in the same local frame
  // as from/to; the post emits I/J relative to the move start.
  bool arc = false;
  bool clockwise = false;
  wave::Point64 arc_center{};
  // The opening rapid that carries the cutter from where the previous pass
  // left it to where this one starts. It belongs to no pass's geometry, so it
  // is flagged rather than inferred from being horizontal and distance-free.
  bool arrival = false;
  // A stock-validated positioning move between roughing layers.
  bool inter_layer = false;
};

struct MotionCursor {
  wave::Point64 position;
  double z = 0.0;
  double path_distance = 0.0;
  wave::MoveKind kind = wave::MoveKind::Cut;
  bool rapid = false;
  double rate = 0.0;
  // Of the phase the cursor is in, as the stock simulation measured it.
  double engagement = 0.0;
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
  // Prefix indexes rebuilt after commanded motion is finalized. Unindexed
  // schedules remain supported for callers constructing motion directly.
  std::vector<double> motion_seconds, motion_lengths;
  // Inter-layer XYZ links cannot be reconstructed from planar feed segments.
  bool authoritative_motion = false;
  // This pass finishes with the cutter still at its own floor. The pass after
  // it comes down from there rather than across the rapid plane.
  bool ends_at_depth = false;
  // This pass was reached without visiting clearance. Authoritative motion
  // may begin with a validated XY/Z link from the preceding pass's endpoint.
  bool begins_at_depth = false;
};

// How a pass joins the ones on either side of it. A pass on its own knows only
// its own depth, so where the cutter comes from and whether it stays down
// afterwards are both told to it.
struct PassLink {
  // Where whatever ran before this pass left the cutter standing. Absent for
  // the first pass that cuts, which begins wherever it likes.
  std::optional<wave::Point64> from;
  // The height it was left at: the rapid plane normally, and the previous
  // pass's own floor when that pass ended without lifting. This is the whole of
  // what a stay-down transition between depths costs -- one stepdown of descent
  // instead of the full trip to the plane and back.
  double from_z = 0.0;
  // Leave the cutter at this pass's floor when it finishes, because the pass
  // after it has already been given a way across at this depth.
  bool stay_down_after = false;
};

// Builds the commanded motion for one pass at its floor. Rates here are
// provisional: the XYZ stock simulation measures engagement and assigns the
// final feeds. Every retract is offered as a rapid descent; the simulation
// turns one that lands on stock into a plunge.
FeedSchedule plan_feeds(const wave::Toolpath& toolpath,
                        const Machine& machine,
                        const CuttingFeeds& feeds,
                        double floor_z = 0.0,
                        double rapid_plane = 0.0,
                        PassLink link = {},
                        bool allow_plunge = true);

double feed_schedule_seconds(const FeedSchedule& schedule);
double feed_motion_length(const FeedSchedule& schedule);
double feed_seconds_to(const FeedSchedule& schedule, double distance);
double feed_distance_at_seconds(const FeedSchedule& schedule, double seconds);
void index_feed_motion(FeedSchedule& schedule);

MotionCursor feed_motion_at_seconds(const FeedSchedule& schedule,
                                    double seconds);
double feed_distance_after(const FeedSchedule& schedule, double distance,
                           double seconds);

}  // namespace cam
