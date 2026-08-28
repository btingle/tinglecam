#pragma once

#include "cam/machine.h"
#include "cam/material.h"
#include "cam/tooling.h"
#include "simulation.h"
#include "toolpath.h"

#include <cstddef>
#include <string>
#include <vector>

namespace cam {

// Which stage of the pipeline decided a segment's feed. Recorded rather than
// derived afterwards, because "why is it crawling here" is the first question
// anyone asks of a plan and reconstructing the answer later is guesswork.
enum class FeedLimit {
  ChipLoad,        // nothing clamped it; chip thinning set the feed
  CornerGeometry,  // tight turn, so the edge outruns the centre
  SpindlePower,
  CuttingForce,
  ToolDeflection,
  MachineFeed,
  Acceleration,    // cannot be reached within the run before the next reversal
  BlockRate,       // the controller cannot consume lines fast enough
  PolicyMaximum,
  PolicyMinimum,
  MinimumChip,     // raised to keep the edge cutting rather than rubbing
  Lookahead,       // held down by heavier engagement just ahead
  SlewLimit,       // held down so the feed does not step between blocks
  Rapid,           // nothing engaged, so it travels at the machine's rate
  Plunge,
};

const char* feed_limit_name(FeedLimit value);
// True for the limits that mean something is being given up, so the UI can
// colour them apart from the ones that are simply the plan working.
bool feed_limit_is_binding(FeedLimit value);

struct FeedPolicy {
  bool chip_thinning = true;
  double load_ceiling = 1.0;
  double max_thinning_factor = 4.0;
  bool corner_compensation = true;
  double min_corner_radius = 0.05;  // mm, below which a turn is treated as a corner
  // How far the posted path may stray from the sampled one when consecutive
  // pieces are merged into one block. Offset contours arrive carrying vertices
  // every few hundredths of a millimetre, and a controller metered in blocks
  // per second cannot run those at any useful feed.
  double path_tolerance = 0.01;  // mm
  double lookahead_distance = 3.0;  // mm
  double feed_slew_limit = 2000.0;  // mm/min of change per mm travelled
  double min_feed = 30.0;           // mm/min
  double max_feed = 0.0;            // mm/min, zero defers to the machine
  bool rapid_disengaged = true;
  double power_margin = 0.8;
  bool enable_power = true;
  bool enable_force = true;
  bool enable_deflection = true;
  bool enable_acceleration = true;
  double max_deflection = 0.05;  // mm at the tip
  // The bending load is larger than the tangential force alone, since the
  // radial component acts at right angles to it. This is their resultant as a
  // multiple of the tangential force.
  double lateral_force_ratio = 1.2;
};

// One straight piece of motion, ready to be posted or drawn. Distances are in
// millimetres in the job's coordinates; the integer points are the engine's.
struct FeedSegment {
  wave::Point64 from;
  wave::Point64 to;
  std::size_t move_index = 0;
  wave::MoveKind kind = wave::MoveKind::Cut;
  bool rapid = false;
  double length = 0.0;
  double feed = 0.0;  // mm/min, meaningless when rapid

  // Diagnostics, all at the commanded feed.
  double engagement = 0.0;        // a_e / D
  double chip_thickness = 0.0;    // mm per tooth
  double removal_rate = 0.0;      // mm^3/min
  double power_kw = 0.0;
  double tangential_force = 0.0;  // N
  double deflection = 0.0;        // mm
  FeedLimit limit = FeedLimit::ChipLoad;
  // The chip this segment actually leaves is below what the material needs to
  // be cut rather than rubbed. A limit above has held the feed down and the
  // remedy is fewer revolutions or less depth, not more feed, so this is
  // reported rather than corrected.
  bool below_minimum_chip = false;
};

struct FeedStats {
  double spindle_rpm = 0.0;
  double surface_speed = 0.0;   // m/min actually achieved after clamping
  double cutting_distance = 0.0;
  double rapid_distance = 0.0;
  double cutting_minutes = 0.0;
  double rapid_minutes = 0.0;
  double minimum_feed = 0.0;
  double maximum_feed = 0.0;
  double maximum_power_kw = 0.0;
  double maximum_force = 0.0;
  double maximum_deflection = 0.0;
  double maximum_engagement = 0.0;
  // Distance travelled under each limit, so the histogram is weighted by how
  // much of the job each one actually costs rather than by segment count.
  std::vector<std::pair<FeedLimit, double>> limit_distance;
};

struct FeedSchedule {
  std::vector<FeedSegment> segments;
  FeedStats stats;
  // Conditions the operator should know about but which do not stop a post.
  std::vector<std::string> warnings;
};

// Everything the engine needs that is not in a library: the depth of this pass
// and which way round the cutter is going.
struct CutContext {
  double axial_depth = 1.0;  // a_p for this level, mm
  CuttingDirection direction = CuttingDirection::Climb;
};

// The spindle speed for a tool in a material at a depth, before and after the
// machine's own limits. Exposed separately because the Tool Builder shows it
// live while a tool is being entered.
double requested_spindle_rpm(const Tool& tool, const CuttingData& data,
                             const Material& material, double axial_depth);

// Radial chip thinning. Returns the factor by which the feed per tooth must be
// raised to keep the actual chip at the book thickness, given engagement as a
// fraction of diameter. One at half engagement and above.
double chip_thinning_factor(double engagement);

// The pure transform. No window, no files, no globals: given a planned path and
// what it was simulated to remove, produce a feed for every piece of it.
FeedSchedule plan_feeds(const wave::Toolpath& toolpath,
                        const wave::SimulationResult& simulation,
                        const Tool& tool,
                        const Machine& machine,
                        const Material& material,
                        const CuttingData& data,
                        const CutContext& context,
                        const FeedPolicy& policy);

}  // namespace cam
