#include "cam/feeds.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

namespace cam {
namespace {

// Millimetres between two engine points.
double distance_mm(const wave::Point64& a, const wave::Point64& b) {
  return std::hypot(wave::to_world(b.x - a.x), wave::to_world(b.y - a.y));
}

bool engaged_kind(wave::MoveKind kind) {
  return kind == wave::MoveKind::Cut || kind == wave::MoveKind::Step ||
         kind == wave::MoveKind::Link;
}

// The angle between two consecutive segments, in radians. Zero when either has
// no length, since a degenerate segment turns through nothing.
double turn_angle(const wave::Point64& a, const wave::Point64& b,
                  const wave::Point64& c) {
  const double ux = wave::to_world(b.x - a.x);
  const double uy = wave::to_world(b.y - a.y);
  const double vx = wave::to_world(c.x - b.x);
  const double vy = wave::to_world(c.y - b.y);
  const double lu = std::hypot(ux, uy);
  const double lv = std::hypot(vx, vy);
  if (!(lu > 0.0) || !(lv > 0.0)) return 0.0;
  const double cosine = std::clamp((ux * vx + uy * vy) / (lu * lv), -1.0, 1.0);
  return std::acos(cosine);
}

// Perpendicular distance from `point` to the segment `a`-`b`, in millimetres.
double distance_to_chord(const wave::Point64& a, const wave::Point64& b,
                         const wave::Point64& point) {
  const double ax = wave::to_world(a.x);
  const double ay = wave::to_world(a.y);
  const double bx = wave::to_world(b.x);
  const double by = wave::to_world(b.y);
  const double px = wave::to_world(point.x);
  const double py = wave::to_world(point.y);
  const double dx = bx - ax;
  const double dy = by - ay;
  const double square = dx * dx + dy * dy;
  if (!(square > 0.0)) return std::hypot(px - ax, py - ay);
  const double t = std::clamp(((px - ax) * dx + (py - ay) * dy) / square, 0.0, 1.0);
  return std::hypot(px - (ax + dx * t), py - (ay + dy * t));
}

void record(FeedSegment& segment, double candidate, FeedLimit limit) {
  if (candidate < segment.feed) {
    segment.feed = candidate;
    segment.limit = limit;
  }
}

}  // namespace

const char* feed_limit_name(FeedLimit value) {
  switch (value) {
    case FeedLimit::ChipLoad: return "chip load";
    case FeedLimit::CornerGeometry: return "corner geometry";
    case FeedLimit::SpindlePower: return "spindle power";
    case FeedLimit::CuttingForce: return "cutting force";
    case FeedLimit::ToolDeflection: return "tool deflection";
    case FeedLimit::MachineFeed: return "machine feed limit";
    case FeedLimit::Acceleration: return "acceleration";
    case FeedLimit::BlockRate: return "block rate";
    case FeedLimit::PolicyMaximum: return "policy maximum";
    case FeedLimit::PolicyMinimum: return "policy minimum";
    case FeedLimit::MinimumChip: return "minimum chip";
    case FeedLimit::Lookahead: return "load ahead";
    case FeedLimit::SlewLimit: return "feed smoothing";
    case FeedLimit::Rapid: return "rapid";
    case FeedLimit::Plunge: return "plunge";
  }
  return "unknown";
}

bool feed_limit_is_binding(FeedLimit value) {
  switch (value) {
    case FeedLimit::SpindlePower:
    case FeedLimit::CuttingForce:
    case FeedLimit::ToolDeflection:
    case FeedLimit::Acceleration:
    case FeedLimit::BlockRate:
    case FeedLimit::MachineFeed:
      return true;
    default:
      return false;
  }
}

double requested_spindle_rpm(const Tool& tool, const CuttingData& data,
                             const Material& material, double axial_depth) {
  // The tool's book surface speed already accounts for its own coating and
  // substrate, so nothing here scales for those again. The material multiplier
  // is a user derate, not a physical property, and defaults to one.
  const double surface_speed = data.surface_speed * material.vc_multiplier;
  const double diameter = std::max(effective_diameter(tool, axial_depth), 1e-6);
  return 1000.0 * surface_speed / (std::numbers::pi * diameter);
}

double chip_thinning_factor(double engagement) {
  const double r = std::clamp(engagement, 0.0, 1.0);
  if (r >= 0.5) return 1.0;
  const double sine = 2.0 * std::sqrt(r * (1.0 - r));
  if (!(sine > 1e-9)) return std::numeric_limits<double>::infinity();
  return 1.0 / sine;
}

FeedSchedule plan_feeds(const wave::Toolpath& toolpath,
                        const wave::SimulationResult& simulation,
                        const Tool& tool,
                        const Machine& machine,
                        const Material& material,
                        const CuttingData& data,
                        const CutContext& context,
                        const FeedPolicy& policy) {
  FeedSchedule schedule;
  if (toolpath.moves.empty()) return schedule;

  const double depth = std::max(context.axial_depth, 1e-6);
  const double diameter = std::max(tool.diameter, 1e-6);
  const double teeth = static_cast<double>(std::max(tool.flutes, 1));

  // ---- Speed ------------------------------------------------------------
  const double wanted_rpm = requested_spindle_rpm(tool, data, material, depth);
  const double rpm = commandable_rpm(machine, wanted_rpm);
  schedule.stats.spindle_rpm = rpm;
  schedule.stats.surface_speed =
      std::numbers::pi * effective_diameter(tool, depth) * rpm / 1000.0;
  if (machine.max_rpm > 0.0 && wanted_rpm > machine.max_rpm * 1.001) {
    schedule.warnings.push_back(
        "The tool wants " + std::to_string(static_cast<int>(wanted_rpm)) +
        " rpm but the spindle stops at " +
        std::to_string(static_cast<int>(machine.max_rpm)) +
        ", so it is cutting below its rated surface speed");
  }
  if (machine.min_rpm > 0.0 && wanted_rpm < machine.min_rpm * 0.999) {
    schedule.warnings.push_back(
        "The tool wants " + std::to_string(static_cast<int>(wanted_rpm)) +
        " rpm but the spindle starts at " +
        std::to_string(static_cast<int>(machine.min_rpm)) +
        ", so it is cutting above its rated surface speed");
  }

  // ---- Ceilings that do not vary along the path -------------------------
  double machine_feed = machine.max_cutting_feed;
  if (policy.max_feed > 0.0)
    machine_feed = machine_feed > 0.0 ? std::min(machine_feed, policy.max_feed)
                                      : policy.max_feed;
  if (!(machine_feed > 0.0)) machine_feed = 100000.0;
  const bool policy_is_tighter =
      policy.max_feed > 0.0 &&
      (machine.max_cutting_feed <= 0.0 || policy.max_feed < machine.max_cutting_feed);

  const double book_chip = data.feed_per_tooth * material.fz_multiplier;
  const double floor_chip = data.min_chip_thickness * material.fz_multiplier;
  const double acceleration = limiting_acceleration(machine);
  const double available_kw = spindle_power(machine, rpm) * machine.drivetrain_efficiency;
  const double force_ceiling = machine.max_cutting_force > 0.0
                                   ? machine.max_cutting_force
                                   : default_cutting_force(machine.rigidity);
  const double mrr_ceiling =
      machine.max_mrr > 0.0 ? machine.max_mrr : default_mrr(machine.rigidity);

  const double tool_radius = diameter * 0.5;
  const double plunge_feed = book_chip * teeth * rpm * data.plunge_feed_factor;

  // ---- Build one segment per sampled step -------------------------------
  for (std::size_t index = 0; index < toolpath.moves.size(); ++index) {
    const wave::ToolpathMove& move = toolpath.moves[index];

    if (move.kind == wave::MoveKind::Retract) {
      for (std::size_t i = 1; i < move.path.size(); ++i) {
        FeedSegment segment;
        segment.from = move.path[i - 1];
        segment.to = move.path[i];
        segment.move_index = index;
        segment.kind = move.kind;
        segment.rapid = true;
        segment.limit = FeedLimit::Rapid;
        segment.length = distance_mm(segment.from, segment.to);
        if (segment.length > 0.0) schedule.segments.push_back(segment);
      }
      continue;
    }

    if (move.kind == wave::MoveKind::Entry) {
      for (std::size_t i = 1; i < move.path.size(); ++i) {
        FeedSegment segment;
        segment.from = move.path[i - 1];
        segment.to = move.path[i];
        segment.move_index = index;
        segment.kind = move.kind;
        segment.feed = plunge_feed;
        segment.limit = FeedLimit::Plunge;
        segment.length = distance_mm(segment.from, segment.to);
        if (segment.length > 0.0) schedule.segments.push_back(segment);
      }
      continue;
    }

    if (!engaged_kind(move.kind)) continue;
    if (index >= simulation.moves.size()) continue;

    for (const wave::MaterialStep& step : simulation.moves[index].steps) {
      FeedSegment segment;
      segment.from = step.from;
      segment.to = step.to;
      segment.move_index = index;
      segment.kind = move.kind;
      segment.length = distance_mm(step.from, step.to);
      if (!(segment.length > 0.0)) continue;

      // 01 -- engagement, straight from what the simulation actually removed.
      const double engagement =
          std::clamp(step.load, 0.0, std::max(policy.load_ceiling, 1e-6));
      segment.engagement = engagement;

      // A step that removed nothing is travel, whatever the planner called it.
      if (step.removed_area <= 0.0 && policy.rapid_disengaged) {
        segment.rapid = true;
        segment.limit = FeedLimit::Rapid;
        schedule.segments.push_back(segment);
        continue;
      }

      // 02, 03 -- book chip load, thinned for the engagement actually seen.
      double factor = 1.0;
      if (policy.chip_thinning) {
        factor = chip_thinning_factor(engagement);
        factor = std::min(factor, std::max(policy.max_thinning_factor, 1.0));
      }
      const double chip = book_chip * factor;

      // 04, 05 -- the feed that chip load asks for.
      segment.feed = chip * teeth * rpm;
      segment.limit = FeedLimit::ChipLoad;

      schedule.segments.push_back(segment);
    }
  }

  if (schedule.segments.empty()) return schedule;

  // ---- Consolidation ----------------------------------------------------
  // Offset output carries vertices every few hundredths of a millimetre, and
  // the simulation samples finely on top of that. A controller that consumes a
  // hundred blocks a second cannot run a 0.03 mm block above 180 mm/min, so
  // left alone the path would be throttled by its own resolution rather than by
  // anything about the cut. Consecutive pieces going the same way are merged
  // while the straight line between the ends stays within tolerance of the
  // sampled path. Engagement takes the worst of the group and the feed the
  // slowest, so merging can only ever be conservative.
  if (policy.path_tolerance > 0.0 || machine.min_segment_length > 0.0) {
    const double tolerance = std::max(policy.path_tolerance, 1e-6);
    const double longest = std::max(policy.lookahead_distance, 1.0);
    std::vector<FeedSegment> merged;
    merged.reserve(schedule.segments.size());
    std::size_t start = 0;
    while (start < schedule.segments.size()) {
      FeedSegment run = schedule.segments[start];
      std::size_t stop = start + 1;
      while (stop < schedule.segments.size()) {
        const FeedSegment& next = schedule.segments[stop];
        if (next.kind != run.kind || next.rapid != run.rapid) break;
        if (run.length + next.length > longest) break;
        // Every vertex the run passes through has to stay near the chord that
        // would replace it, or the merge would cut a corner off the part.
        bool within = true;
        for (std::size_t i = start; i < stop && within; ++i)
          within = distance_to_chord(run.from, next.to, schedule.segments[i].to) <=
                   tolerance;
        if (!within) break;
        run.to = next.to;
        run.length += next.length;
        run.engagement = std::max(run.engagement, next.engagement);
        run.feed = std::min(run.feed, next.feed);
        ++stop;
      }
      merged.push_back(run);
      start = stop;
    }
    schedule.segments = std::move(merged);
  }

  // ---- 06: corner compensation ------------------------------------------
  // The cutting edge sweeps a wider arc than the tool centre, so on a tight
  // turn the edge outruns the centre and the chip grows. Only the slowing case
  // is applied: which side the material lies on is not known here, and the safe
  // error is to be slow.
  if (policy.corner_compensation) {
    for (std::size_t i = 1; i + 1 < schedule.segments.size(); ++i) {
      FeedSegment& segment = schedule.segments[i];
      if (segment.rapid) continue;
      const double angle = turn_angle(schedule.segments[i - 1].from,
                                      schedule.segments[i].from,
                                      schedule.segments[i].to);
      if (!(angle > 1e-6)) continue;
      const double span = schedule.segments[i - 1].length + segment.length;
      const double radius = span / (2.0 * angle);
      if (radius < policy.min_corner_radius) continue;
      const double scale = radius / (radius + tool_radius);
      record(segment, segment.feed * scale, FeedLimit::CornerGeometry);
    }
  }

  // ---- 07 to 09: what the cut costs -------------------------------------
  // Each of these limits depends on the chip thickness, which depends on the
  // feed they are working out. With a chip-thickness-dependent cutting energy
  // that is genuinely circular: a lower feed leaves a thinner chip, which costs
  // more per unit volume, which lowers the feed again. So the block is run
  // several times over, each pass recomputing the energy at the feed the last
  // one arrived at. The limits only ever lower a feed, so this settles quickly
  // and from above; with a constant energy the second pass changes nothing.
  for (int pass = 0; pass < 4; ++pass) {
  for (FeedSegment& segment : schedule.segments) {
    if (segment.rapid) continue;

    const double radial = segment.engagement * diameter;  // a_e in mm
    if (!(radial > 0.0)) continue;

    // The mean chip over the engagement arc, which is what sets the force. The
    // arc angle follows from the engagement: cos(theta) = 1 - 2*a_e/D.
    const double arc = std::acos(std::clamp(1.0 - 2.0 * segment.engagement, -1.0, 1.0));
    const double per_tooth = segment.feed / (teeth * std::max(rpm, 1e-9));
    const double mean_chip =
        arc > 1e-9 ? per_tooth * 2.0 * segment.engagement / arc : per_tooth;
    // J/mm^3 as stored; the force form of the same quantity is a thousand times
    // it in newtons per square millimetre.
    const double energy = specific_energy(material, mean_chip);
    const double force_coefficient = energy * 1000.0;

    // 07 -- spindle power. Removal rate times specific energy is watts.
    if (policy.enable_power && available_kw > 0.0) {
      const double budget_watts = available_kw * 1000.0 * policy.power_margin;
      // P = a_p * a_e * F * energy / 60, with F in mm/min.
      const double per_unit_feed = depth * radial * energy / 60.0;
      if (per_unit_feed > 0.0)
        record(segment, budget_watts / per_unit_feed, FeedLimit::SpindlePower);
    }

    // Removal rate is also capped outright on machines with no honest power
    // figure, which is most hobby machines.
    if (policy.enable_power && mrr_ceiling > 0.0) {
      const double cubic_mm_per_minute = mrr_ceiling * 1000.0;
      const double per_unit_feed = depth * radial;
      if (per_unit_feed > 0.0)
        record(segment, cubic_mm_per_minute / per_unit_feed, FeedLimit::SpindlePower);
    }

    // 08 -- cutting force. Averaged over the teeth in cut, the tangential force
    // works out proportional to the feed per tooth and the engagement:
    //   Ft = kc * a_p * f_z * z * (a_e/D) / pi
    if (policy.enable_force && force_ceiling > 0.0) {
      const double per_unit_chip =
          force_coefficient * depth * teeth * segment.engagement / std::numbers::pi;
      if (per_unit_chip > 0.0) {
        const double allowed_chip = force_ceiling / per_unit_chip;
        record(segment, allowed_chip * teeth * rpm, FeedLimit::CuttingForce);
      }
    }

    // 09 -- deflection at the tip under the resultant lateral load.
    if (policy.enable_deflection && policy.max_deflection > 0.0) {
      const double per_unit_chip =
          force_coefficient * depth * teeth * segment.engagement / std::numbers::pi;
      const double lateral_per_chip = per_unit_chip * policy.lateral_force_ratio;
      if (lateral_per_chip > 0.0) {
        const double unit_deflection = tip_deflection(tool, lateral_per_chip);
        if (unit_deflection > 0.0) {
          const double allowed_chip = policy.max_deflection / unit_deflection;
          record(segment, allowed_chip * teeth * rpm, FeedLimit::ToolDeflection);
        }
      }
    }
  }
  }

  // ---- 10: what the machine can actually reach --------------------------
  // Consecutive segments that hold roughly the same heading are one run: the
  // controller does not stop between them, so the whole run is the distance
  // available to get up to speed. A real direction change ends a run.
  if (policy.enable_acceleration && acceleration > 0.0) {
    constexpr double kReversalAngle = 0.5;  // radians, about 29 degrees
    std::size_t start = 0;
    while (start < schedule.segments.size()) {
      std::size_t stop = start + 1;
      double run_length = schedule.segments[start].length;
      while (stop < schedule.segments.size()) {
        const double angle = turn_angle(schedule.segments[stop - 1].from,
                                        schedule.segments[stop].from,
                                        schedule.segments[stop].to);
        if (angle > kReversalAngle) break;
        run_length += schedule.segments[stop].length;
        ++stop;
      }
      // Accelerate over half the run and decelerate over the other half, which
      // is the fastest a move that starts and ends at rest can be taken.
      const double reachable = std::sqrt(acceleration * run_length) * 60.0;
      for (std::size_t i = start; i < stop; ++i)
        if (!schedule.segments[i].rapid)
          record(schedule.segments[i], reachable, FeedLimit::Acceleration);
      start = stop;
    }
  }

  // Block rate: a line the controller cannot consume in time throttles the
  // machine whatever the axes could do.
  if (machine.block_rate > 0.0) {
    for (FeedSegment& segment : schedule.segments) {
      if (segment.rapid) continue;
      record(segment, segment.length * machine.block_rate * 60.0, FeedLimit::BlockRate);
    }
  }

  // ---- Fixed ceilings ---------------------------------------------------
  // Everything up to here is physical: what the spindle, the structure, the
  // cutter and the axes will stand. Nothing below may raise a feed past it, so
  // the ceiling is kept and the floors are held to it.
  std::vector<double> ceiling(schedule.segments.size(), 0.0);
  for (std::size_t i = 0; i < schedule.segments.size(); ++i) {
    FeedSegment& segment = schedule.segments[i];
    if (segment.rapid) continue;
    record(segment, machine_feed,
           policy_is_tighter ? FeedLimit::PolicyMaximum : FeedLimit::MachineFeed);
    ceiling[i] = segment.feed;
  }

  // The chip floor keeps the edge cutting rather than rubbing, and it is the
  // one figure here that wants the feed higher. Where a physical limit has
  // already put the feed below it there is nothing safe to do about it from
  // here: the fix is fewer revolutions or less depth. So the segments are
  // marked, the operator is told what speed would resolve it, and the feed is
  // left where the machine can actually sustain it.
  double slowest_capped = 0.0;
  if (floor_chip > 0.0) {
    const double floor_feed = floor_chip * teeth * rpm;
    for (std::size_t i = 0; i < schedule.segments.size(); ++i) {
      FeedSegment& segment = schedule.segments[i];
      if (segment.rapid) continue;
      if (segment.feed >= floor_feed) continue;
      if (floor_feed <= ceiling[i]) {
        segment.feed = floor_feed;
        segment.limit = FeedLimit::MinimumChip;
        continue;
      }
      segment.below_minimum_chip = true;
      slowest_capped = slowest_capped > 0.0 ? std::min(slowest_capped, ceiling[i])
                                            : ceiling[i];
    }
  }
  if (slowest_capped > 0.0) {
    const double workable_rpm = slowest_capped / (teeth * floor_chip);
    schedule.warnings.push_back(
        "The chip falls below the " + std::to_string(floor_chip) +
        " mm minimum where a limit holds the feed down. Reduce the depth of cut, "
        "or run nearer " + std::to_string(static_cast<int>(workable_rpm)) +
        " rpm so the same feed leaves a thicker chip");
  }

  for (std::size_t i = 0; i < schedule.segments.size(); ++i) {
    FeedSegment& segment = schedule.segments[i];
    if (segment.rapid) continue;
    if (policy.min_feed > 0.0 && segment.feed < policy.min_feed) {
      segment.feed = std::min(policy.min_feed, ceiling[i]);
      segment.limit = FeedLimit::PolicyMinimum;
    }
  }

  // ---- 12a: lookahead ---------------------------------------------------
  // Take the lowest feed within the window ahead, so the machine is already
  // slow when the heavy engagement arrives rather than slowing inside it.
  //
  // The reason travels with the feed. Reporting these as "held down by what is
  // ahead" would be true and useless: on a path of short blocks most of the
  // distance ends up carrying that label, and it hides the corner or the
  // spindle that actually caused it. Naming the original cause is what makes
  // the histogram answer the question anyone is asking of it.
  if (policy.lookahead_distance > 0.0) {
    const std::size_t count = schedule.segments.size();
    std::vector<double> held(count);
    std::vector<FeedLimit> reason(count);
    for (std::size_t i = 0; i < count; ++i) {
      held[i] = schedule.segments[i].feed;
      reason[i] = schedule.segments[i].limit;
    }
    for (std::size_t i = 0; i < count; ++i) {
      if (schedule.segments[i].rapid) continue;
      double travelled = 0.0;
      for (std::size_t j = i + 1; j < count && travelled < policy.lookahead_distance;
           ++j) {
        if (schedule.segments[j].rapid) break;
        if (schedule.segments[j].feed < held[i]) {
          held[i] = schedule.segments[j].feed;
          reason[i] = schedule.segments[j].limit;
        }
        travelled += schedule.segments[j].length;
      }
    }
    for (std::size_t i = 0; i < count; ++i) {
      if (schedule.segments[i].rapid) continue;
      schedule.segments[i].feed = held[i];
      schedule.segments[i].limit = reason[i];
    }
  }

  // ---- 12b: slew limit --------------------------------------------------
  // Two passes, so a step down is anticipated and a step up is eased into.
  if (policy.feed_slew_limit > 0.0) {
    const std::size_t count = schedule.segments.size();
    for (std::size_t i = 1; i < count; ++i) {
      FeedSegment& segment = schedule.segments[i];
      const FeedSegment& previous = schedule.segments[i - 1];
      if (segment.rapid || previous.rapid) continue;
      const double allowed =
          previous.feed + policy.feed_slew_limit * previous.length;
      if (segment.feed > allowed) {
        segment.feed = allowed;
        segment.limit = FeedLimit::SlewLimit;
      }
    }
    for (std::size_t i = count; i-- > 1;) {
      FeedSegment& previous = schedule.segments[i - 1];
      const FeedSegment& segment = schedule.segments[i];
      if (segment.rapid || previous.rapid) continue;
      const double allowed = segment.feed + policy.feed_slew_limit * previous.length;
      if (previous.feed > allowed) {
        previous.feed = allowed;
        previous.limit = FeedLimit::SlewLimit;
      }
    }
  }

  // ---- Diagnostics at the feeds finally arrived at ----------------------
  const double machine_rapid = rapid_feed(machine) > 0.0 ? rapid_feed(machine) : 3000.0;
  for (FeedSegment& segment : schedule.segments) {
    if (segment.rapid) {
      schedule.stats.rapid_distance += segment.length;
      schedule.stats.rapid_minutes += segment.length / machine_rapid;
      continue;
    }
    const double radial = segment.engagement * diameter;
    segment.chip_thickness = segment.feed / (teeth * std::max(rpm, 1e-9));
    segment.removal_rate = depth * radial * segment.feed;
    const double arc = std::acos(std::clamp(1.0 - 2.0 * segment.engagement, -1.0, 1.0));
    const double mean_chip = arc > 1e-9
                                 ? segment.chip_thickness * 2.0 * segment.engagement / arc
                                 : segment.chip_thickness;
    const double energy = specific_energy(material, mean_chip);
    segment.power_kw = segment.removal_rate * energy / 60.0 / 1000.0;
    segment.tangential_force = energy * 1000.0 * depth * segment.chip_thickness *
                               teeth * segment.engagement / std::numbers::pi;
    segment.deflection =
        tip_deflection(tool, segment.tangential_force * policy.lateral_force_ratio);

    schedule.stats.cutting_distance += segment.length;
    if (segment.feed > 0.0) schedule.stats.cutting_minutes += segment.length / segment.feed;
    schedule.stats.maximum_feed = std::max(schedule.stats.maximum_feed, segment.feed);
    schedule.stats.minimum_feed = schedule.stats.minimum_feed > 0.0
                                      ? std::min(schedule.stats.minimum_feed, segment.feed)
                                      : segment.feed;
    schedule.stats.maximum_power_kw =
        std::max(schedule.stats.maximum_power_kw, segment.power_kw);
    schedule.stats.maximum_force =
        std::max(schedule.stats.maximum_force, segment.tangential_force);
    schedule.stats.maximum_deflection =
        std::max(schedule.stats.maximum_deflection, segment.deflection);
    schedule.stats.maximum_engagement =
        std::max(schedule.stats.maximum_engagement, segment.engagement);
  }

  for (const FeedSegment& segment : schedule.segments) {
    auto it = std::find_if(schedule.stats.limit_distance.begin(),
                           schedule.stats.limit_distance.end(),
                           [&](const auto& entry) { return entry.first == segment.limit; });
    if (it == schedule.stats.limit_distance.end())
      schedule.stats.limit_distance.emplace_back(segment.limit, segment.length);
    else
      it->second += segment.length;
  }
  std::sort(schedule.stats.limit_distance.begin(), schedule.stats.limit_distance.end(),
            [](const auto& a, const auto& b) { return a.second > b.second; });

  if (schedule.stats.maximum_deflection > policy.max_deflection * 1.001)
    schedule.warnings.push_back(
        "Tip deflection reaches " +
        std::to_string(schedule.stats.maximum_deflection) +
        " mm, past the limit the feed was planned to hold");

  return schedule;
}

}  // namespace cam
