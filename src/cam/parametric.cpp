#include "cam/parametric.h"
#include "cam/auto_pockets.h"
#include "cam/holes.h"
#include "job_internal.h"

#include <clipper2/clipper.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <limits>
#include <stdexcept>

namespace cam {
namespace {
using namespace Clipper2Lib;
constexpr double pi = std::numbers::pi;
constexpr std::size_t max_phases = 500000;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
bool positive(double value) { return std::isfinite(value) && value > 0; }
Point64 point(double x, double y) {
  require(std::isfinite(x) && std::isfinite(y) &&
              std::abs(x) < 1e8 && std::abs(y) < 1e8,
          "Coordinates must be finite and within the geometry range");
  return {wave::to_integer(x), wave::to_integer(y)};
}
double distance_to_path(const Point64& point, const Path64& path) {
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const Point64 a = path[i];
    const Point64 b = path[(i + 1) % path.size()];
    const double dx = static_cast<double>(b.x - a.x);
    const double dy = static_cast<double>(b.y - a.y);
    const double length_squared = dx * dx + dy * dy;
    const double t = length_squared > 0.0
        ? std::clamp((static_cast<double>(point.x - a.x) * dx +
                      static_cast<double>(point.y - a.y) * dy) /
                         length_squared, 0.0, 1.0)
        : 0.0;
    best = std::min(best, std::hypot(static_cast<double>(point.x - a.x) - dx * t,
                                     static_cast<double>(point.y - a.y) - dy * t));
  }
  return wave::to_world(best);
}
int circle_segments(double radius) {
  // At most 0.002 mm chord error, with a modest minimum for small helices.
  const double angle = 2 * std::acos(std::clamp(1 - 0.002 / radius, -1.0, 1.0));
  return std::clamp(static_cast<int>(std::ceil(2 * pi / angle)), 64, 8192);
}
Path64 rounded_rectangle(double x, double y, double width, double height,
                         double radius) {
  if (radius <= 0) return {point(x-width/2, y-height/2),
      point(x+width/2, y-height/2), point(x+width/2, y+height/2),
      point(x-width/2, y+height/2)};
  Path64 ring;
  const int steps = circle_segments(radius) / 4;
  for (int corner = 0; corner < 4; ++corner) {
    const double cx = x + (corner == 0 || corner == 3 ? 1 : -1) * (width/2-radius);
    const double cy = y + (corner < 2 ? 1 : -1) * (height/2-radius);
    for (int i = 0; i <= steps; ++i) {
      const double angle = (corner + static_cast<double>(i)/steps) * pi/2;
      ring.push_back(point(cx + radius*std::cos(angle), cy + radius*std::sin(angle)));
    }
  }
  return ring;
}

// Keep the planar projection and the actual XYZ timeline in lockstep. Vertical
// phases hold their planar distance constant, so playback follows the helix.
struct PathBuilder {
  LevelPlan& level;
  const Operation& operation;
  const Machine& machine;
  const CuttingFeeds& feeds;
  Point64 position;
  double z;
  double distance = 0;

  void segment(Point64 to, double to_z, wave::MoveKind kind, bool rapid) {
    require(level.feeds.motion.size() < max_phases,
            "Operation has too many motion segments; increase step size or pitch");
    const double xy = std::hypot(wave::to_world(to.x-position.x),
                                 wave::to_world(to.y-position.y));
    MotionPhase phase;
    phase.from = position; phase.to = to;
    phase.from_z = z; phase.to_z = to_z;
    phase.path_distance_begin = distance;
    phase.path_distance_end = distance + xy;
    phase.length = std::hypot(xy, to_z-z);
    phase.kind = kind; phase.rapid = rapid;
    phase.rate = rapid ? (to_z != z ? rapid_feed_z(machine) : rapid_feed(machine))
                       : feeds.plunge_feed;
    if (phase.length > 0 || level.feeds.motion.empty())
      level.feeds.motion.push_back(phase);
    if (xy > 0) {
      auto& moves = level.toolpath.moves;
      if (moves.empty() || moves.back().kind != kind ||
          moves.back().path.back() != position) {
        wave::ToolpathMove move;
        move.kind = kind; move.path.push_back(position);
        moves.push_back(std::move(move));
      }
      moves.back().path.push_back(to);
      moves.back().length += xy;
    }
    position = to; z = to_z; distance += xy;
  }
  void line(Point64 to, double to_z, wave::MoveKind kind, bool rapid = false) {
    const Point64 from = position;
    const double from_z = z;
    const double length = std::hypot(wave::to_world(to.x-from.x),
                                     wave::to_world(to.y-from.y));
    const double count = rapid ? 1 : std::max(1.0, std::ceil(length / operation.path_pitch));
    require(count <= max_phases, "Path pitch creates too many segments");
    const int pieces = static_cast<int>(count);
    for (int i = 1; i <= pieces; ++i) {
      const double t = static_cast<double>(i)/pieces;
      segment({from.x + static_cast<int64_t>(std::llround((to.x-from.x)*t)),
               from.y + static_cast<int64_t>(std::llround((to.y-from.y)*t))},
              from_z + (to_z-from_z)*t, kind, rapid);
    }
  }
  void helix_turn(Point64 center, double radius, double to_z,
                  wave::MoveKind kind) {
    require(level.feeds.motion.size() < max_phases,
            "Operation has too many motion segments; increase step size or pitch");
    const Point64 from = position;
    const int samples = circle_segments(radius);
    double planar = 0.0;
    Point64 previous = from;
    auto& moves = level.toolpath.moves;
    if (moves.empty() || moves.back().kind != kind ||
        moves.back().path.back() != from) {
      wave::ToolpathMove move;
      move.kind = kind;
      move.path.push_back(from);
      moves.push_back(std::move(move));
    }
    for (int i = 1; i <= samples; ++i) {
      const Point64 next = i == samples
                               ? from
                               : point(wave::to_world(center.x) +
                                           radius * std::cos(2 * pi * i / samples),
                                       wave::to_world(center.y) +
                                           radius * std::sin(2 * pi * i / samples));
      const double segment_length =
          std::hypot(wave::to_world(next.x - previous.x),
                     wave::to_world(next.y - previous.y));
      planar += segment_length;
      moves.back().path.push_back(next);
      moves.back().length += segment_length;
      previous = next;
    }

    MotionPhase phase;
    phase.from = from;
    phase.to = from;
    phase.from_z = z;
    phase.to_z = to_z;
    phase.path_distance_begin = distance;
    phase.path_distance_end = distance + planar;
    phase.length = std::hypot(2 * pi * radius, to_z - z);
    phase.kind = kind;
    phase.rate = feeds.plunge_feed;
    phase.arc = true;
    phase.arc_center = center;
    level.feeds.motion.push_back(phase);
    position = from;
    z = to_z;
    distance += planar;
  }
  void ring(const Path64& path) {
    if (path.empty()) return;
    line(path.front(), z, wave::MoveKind::Cut);
    for (std::size_t i = 1; i < path.size(); ++i)
      line(path[i], z, wave::MoveKind::Cut);
    line(path.front(), z, wave::MoveKind::Cut);
  }
};

// Provisional rates and totals; the stock simulation measures engagement and
// assigns the final feeds.
void finish_level(LevelPlan& level, const CuttingFeeds& feeds, double radius) {
  level.part_slice = level.stock_slice;
  level.swept = detail::swept_footprint(level, radius);
  auto& stats = level.feeds.stats;
  stats = {};
  stats.spindle_rpm = feeds.spindle_rpm;
  for (auto& phase : level.feeds.motion) {
    if (!phase.rapid && phase.kind != wave::MoveKind::Entry)
      phase.rate = feeds.full_feed();
    if (phase.rapid) {
      stats.rapid_distance += phase.length;
      stats.rapid_minutes += phase.length/phase.rate;
    } else {
      stats.cutting_distance += phase.length;
      stats.cutting_minutes += phase.length/phase.rate;
      if (stats.minimum_feed == 0 || phase.rate < stats.minimum_feed)
        stats.minimum_feed = phase.rate;
      stats.maximum_feed = std::max(stats.maximum_feed, phase.rate);
    }
  }
}
}  // namespace

bool is_parametric(OperationMode mode) {
  return mode == OperationMode::Pocket || mode == OperationMode::PlaneClearing || mode == OperationMode::Hole;
}
const char* pocket_shape_name(PocketShape shape) {
  switch (shape) {
    case PocketShape::RoundedRectangle: return "rounded_rectangle";
  }
  return "unknown";
}
std::optional<PocketShape> pocket_shape_from_name(const std::string& name) {
  if (name == "rounded_rectangle") return PocketShape::RoundedRectangle;
  return std::nullopt;
}

wave::Paths64 pocket_boundary(const PocketParameters& p, double radial_margin) {
  require(pocket_shape_from_name(pocket_shape_name(p.shape)).has_value(), "Unknown pocket shape");
  require(positive(p.width) && positive(p.height) && p.width < 1e7 && p.height < 1e7,
          "Pocket width and height must be positive and finite");
  require(std::isfinite(p.radius) && p.radius >= 0 &&
              p.radius <= std::min(p.width,p.height)/2,
          "Corner radius must be between zero and half the smaller pocket dimension");
  require(std::isfinite(radial_margin) && std::abs(radial_margin) < 1e6,
          "Radial finishing margin must be finite");
  // A margin moves both walls of each axis, so it costs twice its size. Taking
  // the corner in with them keeps the wall a constant distance from the
  // requested one all the way around; a square corner offset outwards rounds
  // to the margin, which is what a cutter travelling that offset would leave.
  const double width = p.width - 2*radial_margin;
  const double height = p.height - 2*radial_margin;
  require(positive(width) && positive(height) && width < 1e7 && height < 1e7,
          "The radial finishing margin leaves no pocket to cut");
  return {rounded_rectangle(p.center_x, p.center_y, width, height,
                            std::max(0.0, p.radius - radial_margin))};
}

wave::Paths64 pocket_margin_boundary(const wave::Paths64& boundary,
                                     double radial_margin) {
  require(std::isfinite(radial_margin) && std::abs(radial_margin) < 1e6,
          "Radial finishing margin must be finite");
  if (radial_margin == 0.0) return boundary;
  auto offset = InflatePaths(boundary, -wave::to_integer(radial_margin),
                             JoinType::Round, EndType::Polygon);
  require(offset.size() == 1,
          "The radial finishing margin leaves no single pocket to cut");
  return offset;
}

OperationPlan plan_parametric_operation(
    const Operation& operation, const Tool& tool, const Machine& machine,
    const CuttingFeeds& feeds, double rapid_plane,
    const PriorOperations& prior_operations,
    const ProgressCallback& progress, double progress_from, double progress_to,
    const JobGeometry* geometry) {
  if(operation.mode==OperationMode::Hole)
    return plan_hole_operation(operation,tool,machine,feeds,rapid_plane,prior_operations,
                               progress,progress_from,progress_to,geometry);
  const auto started = std::chrono::steady_clock::now();
  OperationPlan result;
  result.operation_name = operation.name; result.tool_id = tool.id;
  try {
    require(is_parametric(operation.mode), "Expected pocket or plane clearing operation");
    const auto problems = validate(tool);
    if (!problems.empty()) throw std::runtime_error(problems.front());
    require(tool.type == ToolType::FlatEndMill,
            "Pocket and plane clearing require a flat end mill for a flat floor");
    require(positive(operation.stepdown) && positive(operation.stepover_ratio) &&
                operation.stepover_ratio <= 1, "Stepdown must be positive; stepover must be in (0, 1]");
    require(positive(operation.path_pitch) && operation.path_pitch >= 0.001,
            "Path pitch must be finite and at least 0.001 mm");
    const bool pocket = operation.mode == OperationMode::Pocket;
    const double radius = tool.diameter/2;
    // Pocket walls only. Plane clearing faces a surface; there is no wall for
    // an allowance to sit on.
    const double margin = pocket ? operation.pocket_radial_margin : 0.0;
    require(std::isfinite(margin) && std::abs(margin) < 1e6,
            "Radial finishing margin must be finite");
    const double step = operation.stepover_ratio * tool.diameter;
    require(step >= 0.001, "Stepover is below the 0.001 mm geometry resolution");
    struct Region {
      const PocketParameters* pocket = nullptr;
      const OperationPlan::IdentifiedPocket* identified = nullptr;
      const ClearingArea* area = nullptr;
      Paths64 boundary;
      Paths64 centres;
      double center_x = 0.0;
      double center_y = 0.0;
      double helix_angle_degrees = 3.0;
      double top = 0.0;
      double depth = 0.0;
      double inradius = 0.0;
      double helix_radius = 0.0;
      int layers = 0;
    };
    std::vector<Region> regions;
    if (pocket) {
      if (operation.auto_identify_pockets) {
        require(geometry != nullptr,
                "Auto pocket identification requires loaded geometry");
        require(positive(operation.pocket_auto.helix_angle_degrees) &&
                    operation.pocket_auto.helix_angle_degrees <= 10,
                "Helix angle must be in (0, 10] degrees");
        result.identified_pockets = identify_pockets(*geometry, operation);
        require(!result.identified_pockets.empty(),
                "No simple convex pockets were identified");
      } else {
        require(!operation.pockets.empty(), "Add at least one pocket");
      }
      const std::size_t pocket_count = operation.auto_identify_pockets
          ? result.identified_pockets.size() : operation.pockets.size();
      regions.reserve(pocket_count);
      if (operation.auto_identify_pockets) {
        for (const auto& identified : result.identified_pockets) {
          Region region;
          region.identified = &identified;
          region.top = identified.top_z;
          region.depth = identified.top_z - identified.bottom_z;
          region.center_x = identified.center_x;
          region.center_y = identified.center_y;
          region.boundary = pocket_margin_boundary(identified.boundary, margin);
          region.inradius = distance_to_path(
                                point(region.center_x, region.center_y),
                                region.boundary.front()) - radius;
          require(region.inradius >= 0.01,
                  "An identified pocket is too small for this tool and margin");
          region.centres = InflatePaths(region.boundary, -wave::to_integer(radius),
                                        JoinType::Round, EndType::Polygon);
          require(region.centres.size() == 1,
                  "An identified pocket has no continuous cutter-centre region");
          region.helix_radius = std::min(radius * 0.5, region.inradius * 0.75);
          require(region.helix_radius >= 0.02,
                  "Pocket/tool combination is too small for a helix at 0.001 mm resolution");
          require(positive(region.depth) && region.depth < 1e7,
                  "Identified pocket depth must be positive and finite");
          require(region.depth <= tool.flute_length,
                  "Identified pocket depth exceeds flute length");
          region.helix_angle_degrees =
              operation.pocket_auto.helix_angle_degrees;
          regions.push_back(std::move(region));
        }
      } else for (const auto& p : operation.pockets) {
        Region region;
        region.pocket = &p;
        region.center_x = p.center_x;
        region.center_y = p.center_y;
        region.helix_angle_degrees = p.helix_angle_degrees;
        region.top = p.top_z;
        region.depth = p.depth;
        region.boundary = pocket_boundary(p, margin);
        // The margin takes the corner in with the walls, so what the cutter
        // has to fit is the finished corner, not the requested one.
        const double corner = p.radius == 0.0 ? 0.0
                                              : std::max(0.0, p.radius - margin);
        require(corner == 0.0 || corner >= radius,
                "A rounded pocket corner, after the radial finishing margin, "
                "must be at least the cutter radius");
        require(positive(p.helix_angle_degrees) && p.helix_angle_degrees <= 10,
                "Helix angle must be in (0, 10] degrees");
        region.inradius = std::min(p.width, p.height) / 2 - margin - radius;
        require(region.inradius >= 0.01,
                "Pocket is too small for this tool to enter helically");
        region.centres = InflatePaths(region.boundary, -wave::to_integer(radius),
                                      JoinType::Round, EndType::Polygon);
        require(region.centres.size() == 1,
                "Pocket has no continuous cutter-centre region");
        region.helix_radius = std::min(radius * 0.5, region.inradius * 0.75);
        require(region.helix_radius >= 0.02,
                "Pocket/tool combination is too small for a helix at 0.001 mm resolution");
        require(std::isfinite(region.top) && positive(region.depth) &&
                    region.depth < 1e7,
                "Top Z must be finite and depth must be positive and finite");
        require(region.depth <= tool.flute_length,
                "Operation depth exceeds flute length; choose a tool with sufficient reach");
        regions.push_back(std::move(region));
      }
    } else {
      require(!operation.plane.areas.empty(), "Add at least one plane clearing area");
      regions.reserve(operation.plane.areas.size());
      for (const auto& area : operation.plane.areas) {
        Region region;
        region.area = &area;
        region.top = area.top_z;
        region.depth = area.depth;
        require(positive(area.width) && positive(area.height) &&
                    area.width >= tool.diameter && area.height >= tool.diameter &&
                    std::max(area.width, area.height) > tool.diameter + 0.001,
                "Each clearing area must be at least as wide and tall as the tool diameter");
        point(area.center_x-area.width/2, area.center_y-area.height/2);
        point(area.center_x+area.width/2, area.center_y+area.height/2);
        require(std::isfinite(region.top) && positive(region.depth) &&
                    region.depth < 1e7,
                "Top Z must be finite and depth must be positive and finite");
        require(region.depth <= tool.flute_length,
                "Operation depth exceeds flute length; choose a tool with sufficient reach");
        region.boundary = {rounded_rectangle(area.center_x, area.center_y,
                                              area.width, area.height, 0)};
        regions.push_back(std::move(region));
      }
    }
    double highest_top = 0.0;
    bool first_region = true;
    std::size_t total_levels = 0;
    for (Region& region : regions) {
      require(std::isfinite(region.top) && region.top > -1e7,
              "Top Z must be finite and within the geometry range");
      if (first_region) {
        highest_top = region.top;
        first_region = false;
      } else {
        highest_top = std::max(highest_top, region.top);
      }
      region.layers = static_cast<int>(std::ceil(region.depth / operation.stepdown));
      require(region.layers > 0 && total_levels + static_cast<std::size_t>(region.layers) <= 20000,
              "Too many area/depth passes; increase stepdown");
      total_levels += static_cast<std::size_t>(region.layers);
    }
    struct Task {
      std::size_t region_index;
      int layer;
    };
    std::vector<Task> tasks;
    tasks.reserve(total_levels);
    if (pocket) {
      // Keep one pocket's layers together. This makes the safe stay-down
      // connection between successive helical layers unambiguous.
      for (std::size_t region_index = 0; region_index < regions.size();
           ++region_index)
        for (int layer = 0; layer < regions[region_index].layers; ++layer)
          tasks.push_back({region_index, layer});
    } else {
      // Plane clearing has historically visited every area at a given depth
      // before moving to the next depth. Preserve that playback order while
      // allowing each area to have its own Z range.
      int deepest_layer = 0;
      for (const Region& region : regions)
        deepest_layer = std::max(deepest_layer, region.layers);
      for (int layer = 0; layer < deepest_layer; ++layer)
        for (std::size_t region_index = 0; region_index < regions.size();
             ++region_index)
          if (layer < regions[region_index].layers)
            tasks.push_back({region_index, layer});
    }
    require(tasks.size() == total_levels, "Internal parametric level count mismatch");
    require(std::isfinite(rapid_plane) && rapid_plane > highest_top,
            "Rapid clearance must be above every parametric top surface");
    const bool modelled_stock = geometry && geometry->has_stock();
    std::optional<Point64> previous;
    std::optional<double> previous_z;
    std::optional<std::size_t> previous_region;
    std::optional<int> previous_layer;
    for (auto op = prior_operations.rbegin(); op != prior_operations.rend() && !previous; ++op)
      if (*op && (*op)->ok())
        for (auto level = (*op)->levels.rbegin(); level != (*op)->levels.rend(); ++level)
          if (!level->feeds.motion.empty()) {
            previous = level->feeds.motion.back().to;
            previous_z = level->feeds.motion.back().to_z;
            break;
          }
    std::size_t completed_levels = 0;
    for (std::size_t task_index = 0; task_index < tasks.size(); ++task_index) {
      const std::size_t region_index = tasks[task_index].region_index;
      const int layer = tasks[task_index].layer;
      Region& region = regions[region_index];
      const double upper = region.top -
                           std::min(region.depth, layer * operation.stepdown);
      const double floor = region.top -
                           std::min(region.depth, (layer + 1) * operation.stepdown);
      if (progress && !progress(
              progress_from + (progress_to - progress_from) *
                  static_cast<double>(completed_levels) / total_levels,
              operation.name + ": area " + std::to_string(region_index + 1) +
                  ", depth " + std::to_string(layer + 1) + "/" +
                  std::to_string(region.layers)))
        throw std::runtime_error("Cancelled");
      LevelPlan level;
      level.explicit_motion = true;
      level.floor_z = floor;
      level.axial_depth = upper - floor;
      level.subdivision_level = result.levels.size();
      const bool next_is_same_pocket_layer =
          (region.pocket != nullptr || region.identified != nullptr) &&
          task_index + 1 < tasks.size() &&
          tasks[task_index + 1].region_index == region_index &&
          tasks[task_index + 1].layer == layer + 1;
      level.stay_down_preferred = next_is_same_pocket_layer;
      level.stock_slice = region.boundary;
      // What earlier operations and earlier areas of this one left. Without a
      // modelled blank the area itself is the material to remove.
      const double sample_z = (upper+floor)/2;
      level.incoming_stock = stock_after_operations_at_z(
          modelled_stock ? Intersect(region.boundary,
                                     geometry->stock_outline(sample_z, operation.slice_grid),
                                     FillRule::NonZero)
                         : region.boundary,
          sample_z, prior_operations);
      level.incoming_stock =
          detail::stock_after_operation_at_z(level.incoming_stock, sample_z, result);
      if (level.incoming_stock.empty()) {
        level.nothing_to_cut = true;
        ++result.stats.levels;
        result.levels.push_back(std::move(level));
        ++completed_levels;
        continue;
      }
      // Facing descends beside the area, not into it. The area is the
      // operator's statement that the surface inside it is theirs to remove;
      // it says nothing about the air beside it, and nothing here checks.
      const Point64 entry = region.pocket || region.identified
          ? point(region.center_x + region.helix_radius, region.center_y)
          : point(region.area->center_x - region.area->width / 2 - radius,
                  region.area->center_y - region.area->height / 2 + radius);
      const bool continuing_pocket =
          previous && previous_z && previous_region && previous_layer &&
          *previous_region == region_index && *previous_layer + 1 == layer &&
          *previous_z != rapid_plane;
      PathBuilder path{level, operation, machine, feeds, previous.value_or(entry),
                       previous_z.value_or(rapid_plane)};
      if (!continuing_pocket && path.z != rapid_plane) {
        path.line(path.position, rapid_plane, wave::MoveKind::Retract, true);
      }
      if (!continuing_pocket) {
        path.line(entry, rapid_plane, wave::MoveKind::Retract, true);
        path.line(entry, region.top, wave::MoveKind::Entry, true);
        path.line(entry, upper, wave::MoveKind::Entry);
      } else {
        // The preceding pocket layer ended at this same XY opening and floor.
        // Move to the opening at depth, then descend helically again; no
        // clearance-plane retract is needed between layers.
        path.line(entry, upper, wave::MoveKind::Entry);
      }
      if (region.pocket || region.identified) {
        const double pitch = 2*pi*region.helix_radius*
                             std::tan(region.helix_angle_degrees*pi/180);
        const double turns = std::ceil((upper-floor)/pitch);
        const int samples = circle_segments(region.helix_radius);
        require(turns*samples <= max_phases, "Helix requires too many segments; increase its angle or pocket size");
        const Point64 helix_center = point(region.center_x, region.center_y);
        for (int turn = 1; turn <= turns; ++turn)
          path.helix_turn(
              helix_center, region.helix_radius,
              upper + (floor - upper) * turn / turns,
              wave::MoveKind::Entry);
        // One complete turn at the floor removes the sloped helix bottom.
        for (int i = 1; i <= samples; ++i) {
          const double angle = 2*pi*i/samples;
          path.line(point(region.center_x+region.helix_radius*std::cos(angle),
                          region.center_y+region.helix_radius*std::sin(angle)), floor,
                    wave::MoveKind::Cut);
        }
        const double inner = std::max(0.0, region.inradius-0.005);
        const double rings_needed = std::ceil(inner/step);
        require(rings_needed <= 20000, "Too many pocket rings; increase stepover");
        const int rings = static_cast<int>(rings_needed);
        for (int ring = rings; ring >= 0; --ring) {
          const double inset = rings ? inner*ring/rings : 0;
          const Paths64 contours = ring == 0 ? region.centres : InflatePaths(
              region.centres, -wave::to_integer(inset), JoinType::Round,
              EndType::Polygon);
          for (const auto& contour : contours) path.ring(contour);
        }
      } else {
        const ClearingArea& area = *region.area;
        path.line(entry, floor, wave::MoveKind::Entry);
        // Facing, not pocketing. Every row runs a full radius clear of both
        // ends of the area, so the swept band reaches past each edge and the
        // rows together take the whole of the top surface, corners included.
        // Held inside the rectangle instead, the cutter centre could only
        // reach a radius short of each corner and left the four of them
        // standing -- which is what the perimeter ring here used to be
        // chasing, and it could not reach them either.
        const double left = area.center_x-area.width/2-radius;
        const double right = area.center_x+area.width/2+radius;
        const double bottom = area.center_y-area.height/2+radius;
        const double span = std::max(0.0, area.height-tool.diameter);
        const double rows_needed = std::ceil(span/step);
        require(rows_needed <= 20000, "Too many clearing rows; increase stepover");
        const int rows = static_cast<int>(rows_needed);
        for (int row = 0; row <= rows; ++row) {
          const double y = bottom + (rows ? span*row/rows : 0);
          path.line(point(row%2 ? right : left,y), floor, wave::MoveKind::Cut);
          path.line(point(row%2 ? left : right,y), floor, wave::MoveKind::Cut);
        }
      }
      if (!next_is_same_pocket_layer)
        path.line(path.position, rapid_plane, wave::MoveKind::Retract, true);
      level.stays_down_after = next_is_same_pocket_layer;
      previous = path.position;
      previous_z = level.stays_down_after ? floor : rapid_plane;
      previous_region = region_index;
      previous_layer = layer;
      finish_level(level, feeds, radius);
      level.feeds.ends_at_depth = level.stays_down_after;
      const auto& stats = level.feeds.stats;
      ++result.stats.levels; ++result.stats.cut_levels;
      result.stats.cutting_distance += stats.cutting_distance;
      result.stats.rapid_distance += stats.rapid_distance;
      result.stats.cutting_minutes += stats.cutting_minutes;
      result.stats.rapid_minutes += stats.rapid_minutes;
      result.metrics.toolpath_moves += level.toolpath.moves.size();
      result.levels.push_back(std::move(level));
      ++completed_levels;
    }
    result.spindle_rpm = feeds.spindle_rpm;
  } catch (const std::exception& error) {
    result.error = error.what();
  }
  result.metrics.total_milliseconds = std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-started).count();
  return result;
}

OperationPlan plan_hole_operation(const Operation& operation,const Tool& tool,
    const Machine& machine,const CuttingFeeds& feeds,double rapid_plane,
    const PriorOperations& prior,const ProgressCallback& progress,double from,double to,
    const JobGeometry* geometry,const ToolLibrary* tools,const std::string& material_id) {
  OperationPlan result;result.operation_name=operation.name;result.tool_id=tool.id;
  const auto started=std::chrono::steady_clock::now();
  try {
    const auto& p=operation.holes;
    if(p.automatic) {
      require(geometry && geometry->has_part(),"Load a part to use identified holes");
      require(!p.detection_signature.empty() &&
          p.detection_signature==hole_detection_signature(*geometry,p),
          "Identify holes again: the source geometry or inspection settings changed");
    }
    require(positive(operation.stepdown),"Hole stepdown must be positive");
    require(positive(operation.path_pitch) && operation.path_pitch>=.001,"Path pitch must be at least 0.001 mm");
    require(positive(p.helix_angle_degrees) && p.helix_angle_degrees<=10,"Helix angle must be in (0, 10] degrees");
    require(std::isfinite(p.adaptive_radius) && p.adaptive_radius>0,"Adaptive cutoff radius must be positive");
    require(std::isfinite(p.radial_margin),"Hole radial margin must be finite");
    require(std::isfinite(p.drilling.peck_depth) && p.drilling.peck_depth>=0 &&
        std::isfinite(p.drilling.peck_clearance) && p.drilling.peck_clearance>=0 &&
        std::isfinite(p.drilling.breakthrough) && p.drilling.breakthrough>=0 &&
        std::isfinite(p.drilling.depth_adjustment),
        "Drilling depths and clearances must be finite and nonnegative");
    std::optional<Point64> previous;double previous_z=rapid_plane;
    for(auto op=prior.rbegin();op!=prior.rend() && !previous;++op)
      if(*op && (*op)->ok()) for(auto l=(*op)->levels.rbegin();l!=(*op)->levels.rend();++l)
        if(!l->feeds.motion.empty()) {previous=l->feeds.motion.back().to;previous_z=l->feeds.motion.back().to_z;break;}
    struct Task {
      const HoleSystem* system;
      const HoleLevel* level;
      std::size_t previous;
      bool complete=false;
    };
    std::vector<Task> tasks;
    const auto none=std::numeric_limits<std::size_t>::max();
    for(const auto& h:p.systems) if(h.selected) {
      auto predecessor=none;
      for(const auto& l:h.levels) if(l.selected) {
        tasks.push_back({&h,&l,predecessor});predecessor=tasks.size()-1;
      }
    }
    const auto selected=tasks.size();std::size_t done=0;
    require(selected>0,"Select at least one hole level to cut");
    require(selected<=20000,"Too many selected hole levels");
    std::string current_tool=tool.id;
    while(done<selected) {
      std::size_t next=none;
      // Prefer ready levels using the same tool. Each system's shallow levels
      // remain predecessors of its deeper levels; across systems this batches
      // drilling/milling instead of changing tools once per hole.
      for(int pass=0;pass<2 && next==none;++pass)
        for(std::size_t i=0;i<tasks.size();++i) {
          const auto& task=tasks[i];
          if(task.complete || (task.previous!=none && !tasks[task.previous].complete)) continue;
          const auto& id=task.level->tool_id.empty()?tool.id:task.level->tool_id;
          if(pass==0 && id!=current_tool) continue;
          next=i;break;
        }
      require(next!=none,"Hole level ordering contains a dependency cycle");
      auto& task=tasks[next];const auto& h=*task.system;const auto& l=*task.level;
      const auto center=point(h.center_x,h.center_y);
      if(progress && !progress(from+(to-from)*double(done)/selected,"Planning hole/counterbore levels"))
        throw std::runtime_error("Cancelled");
      ++done;
      require(positive(l.radius) && std::isfinite(l.top_z) && std::isfinite(l.bottom_z) && l.top_z>l.bottom_z,
              "Hole levels need a positive radius and top Z above bottom Z");
      require(rapid_plane>l.top_z,"Rapid plane must be above every hole");
      const Tool* chosen=&tool;
      if(!l.tool_id.empty() && l.tool_id!=tool.id) {
        chosen=tools?tools->find(l.tool_id):nullptr;
        require(chosen,"Choose a valid tool for this hole level");
      }
      const auto errors=validate(*chosen);require(errors.empty(),errors.empty()?"":errors.front().c_str());
      const auto rates=chosen==&tool?feeds:resolve_cutting_feeds(*chosen,machine,material_id,operation.cutting);
      const auto strategy=effective_hole_strategy(p,l);
      if(strategy==HoleStrategy::Pocket) {
        Operation pocket=operation;pocket.mode=OperationMode::Pocket;pocket.auto_identify_pockets=false;
        pocket.pocket_radial_margin=p.radial_margin;pocket.pockets.clear();
        PocketParameters area;area.center_x=h.center_x;area.center_y=h.center_y;
        area.radius=l.radius;area.width=area.height=2*l.radius;
        area.top_z=l.top_z;area.depth=l.top_z-l.bottom_z;area.helix_angle_degrees=p.helix_angle_degrees;
        pocket.pockets.push_back(area);pocket.tool_id=chosen->id;
        auto earlier=prior;earlier.push_back(&result);
        auto planned=plan_parametric_operation(pocket,*chosen,machine,rates,rapid_plane,earlier,
            progress,from+(to-from)*double(done-1)/selected,
            from+(to-from)*double(done)/selected,geometry);
        require(planned.ok(),planned.error.c_str());
        require(result.levels.size()+planned.levels.size()<=20000,"Too many hole/depth passes");
        for(auto& level:planned.levels) {level.tool_id=chosen->id;result.levels.push_back(std::move(level));}
      } else {
        require(result.levels.size()<20000,"Too many hole/depth passes");
        LevelPlan level;level.tool_id=chosen->id;level.explicit_motion=true;
        level.stock_slice={Clipper2Lib::Ellipse(center,wave::to_integer(l.radius),wave::to_integer(l.radius),96)};
        level.incoming_stock=level.stock_slice;
        level.floor_z=l.bottom_z;level.axial_depth=l.top_z-l.bottom_z;
        const Point64 entry=strategy==HoleStrategy::Drill?center:
            point(h.center_x+l.radius-p.radial_margin-chosen->diameter/2,h.center_y);
        PathBuilder path{level,operation,machine,rates,previous.value_or(entry),previous_z};
        if(path.z!=rapid_plane) path.line(path.position,rapid_plane,wave::MoveKind::Retract,true);
        path.line(entry,rapid_plane,wave::MoveKind::Retract,true);
        path.line(entry,l.top_z,wave::MoveKind::Entry,true);
        if(strategy==HoleStrategy::Drill) {
          require(chosen->type==ToolType::Drill,"A drilling level requires a drill tool");
          if(!p.source_id.empty()) require(chosen->diameter/2<=l.radius+std::max(.005,p.circle_tolerance),
              "The pilot drill must fit inside the selected bore");
          if(p.source_id.empty()) require(std::abs(chosen->diameter/2-l.radius)<=std::max(.005,p.circle_tolerance),
              "Choose a drill matching the hole radius, or insert a pre-drilling operation");
          const double tip=drill_tip_height(*chosen);
          const double bottom=l.bottom_z-p.drilling.depth_adjustment-(l.through?p.drilling.breakthrough:0)-
              (p.drilling.depth_reference==DrillDepth::FullDiameter?tip:0);
          require(bottom<l.top_z,"Adjusted drilling depth must be below the hole top");
          require(l.top_z-bottom<=chosen->flute_length,"Drilling depth exceeds the tool's flute length");
          level.floor_z=bottom;level.axial_depth=l.top_z-bottom;
          const double peck=p.drilling.peck_depth>0?p.drilling.peck_depth:l.top_z-bottom;
          const double n=std::ceil((l.top_z-bottom)/peck);
          require(n<=20000,"Too many drilling pecks");
          for(int i=1;i<=int(n);++i) {
            const double floor=std::max(bottom,l.top_z-peck*i);
            path.line(center,floor,wave::MoveKind::Entry);
            if(i<int(n)) {
              path.line(center,std::min(rapid_plane,l.top_z+p.drilling.peck_clearance),wave::MoveKind::Retract,true);
              path.line(center,std::min(l.top_z,floor+.1),wave::MoveKind::Entry,true);
            }
          }
        } else {
          require(chosen->type==ToolType::FlatEndMill || chosen->type==ToolType::BallEndMill,
                  "Helical finishing requires an end mill");
          const double orbit=l.radius-p.radial_margin-chosen->diameter/2;
          require(orbit>=.002,"The cutter must fit inside the requested hole");
          require(l.top_z-l.bottom_z<=chosen->flute_length,"Hole depth exceeds flute length");
          const double pitch=std::min(operation.stepdown,2*pi*orbit*std::tan(p.helix_angle_degrees*pi/180));
          const double turns=std::ceil((l.top_z-l.bottom_z)/pitch);
          require((turns+1)*circle_segments(orbit)<=max_phases,
                  "Helix requires too many samples; increase angle or stepdown");
          // Intentionally no central-core/stock-clearing restriction: the
          // operator chooses whether this wall-finishing path is appropriate.
          for(int i=1;i<=int(turns);++i)
            path.helix_turn(center,orbit,l.top_z+(l.bottom_z-l.top_z)*i/turns,wave::MoveKind::Cut);
          path.helix_turn(center,orbit,l.bottom_z,wave::MoveKind::Cut);
        }
        path.line(path.position,rapid_plane,wave::MoveKind::Retract,true);
        finish_level(level,rates,chosen->diameter/2);
        result.levels.push_back(std::move(level));
      }
      if(!result.levels.empty() && !result.levels.back().feeds.motion.empty()) {
        previous=result.levels.back().feeds.motion.back().to;
        previous_z=result.levels.back().feeds.motion.back().to_z;
      }
      task.complete=true;current_tool=chosen->id;
    }
    for(std::size_t i=0;i<result.levels.size();++i) result.levels[i].subdivision_level=i;
    result.spindle_rpm=feeds.spindle_rpm;
  } catch(const std::exception& e) {result.error=e.what();}
  result.metrics.total_milliseconds=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-started).count();
  return result;
}
}  // namespace cam
