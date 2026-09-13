#include "cam/stock.h"
#include "cam/parametric.h"
#include "cam/auto_pockets.h"

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
                       : operation.plunge_feed;
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
    phase.rate = operation.plunge_feed;
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

void finish_level(LevelPlan& level, const Operation& operation, const Tool& tool,
                  const Machine& machine, double slow, double fast,
                  double rapid_plane) {
  // The protected frame describes everything outside the requested opening.
  // It also supplies the nonempty final subject required by the 2D simulator.
  const auto protected_frame = Difference(InflatePaths(level.stock_slice,
      wave::to_integer(tool.diameter), JoinType::Round, EndType::Polygon),
      level.stock_slice, FillRule::NonZero);
  level.part_slice = level.stock_slice;
  level.simulation = wave::simulate_material(level.incoming_stock, protected_frame,
                                             level.toolpath, tool.diameter/2);
  // The 2D simulator cannot evaluate changing Z. The explicit entry is already
  // resolved here; material at the floor is subtracted by its final flat turn.
  level.simulation.stats.unresolved_entries = 0;
  level.simulation.stats.unsafe_plunges = 0;
  for (auto& move : level.simulation.moves) {
    move.entry_unresolved = false;
    move.plunge_collision.clear();
    move.plunge_collision_area = 0;
  }
  level.toolpath.cleared = level.simulation.removed_material;
  auto motion = std::move(level.feeds.motion);
  level.feeds = plan_feeds(level.toolpath, level.simulation, machine, slow, fast,
                           operation.spindle_rpm, operation.plunge_feed,
                           level.floor_z, rapid_plane);
  level.feeds.motion = std::move(motion);
  level.feeds.stats = {};
  level.feeds.stats.spindle_rpm = commandable_rpm(machine, operation.spindle_rpm);
  std::size_t segment = 0;
  for (auto& phase : level.feeds.motion) {
    if (!phase.rapid && phase.kind != wave::MoveKind::Entry) {
      const double middle = (phase.path_distance_begin + phase.path_distance_end)/2;
      while (segment + 1 < level.feeds.segments.size() &&
             level.feeds.segments[segment].start_distance +
                 level.feeds.segments[segment].length < middle - 1e-8) ++segment;
      phase.rate = segment < level.feeds.segments.size()
                       ? level.feeds.segments[segment].rate : slow;
    }
    auto& stats = level.feeds.stats;
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
  level.feeds.stats.maximum_engagement = level.simulation.stats.maximum_load;
}
}  // namespace

bool is_parametric(OperationMode mode) {
  return mode == OperationMode::Pocket || mode == OperationMode::PlaneClearing;
}
const char* pocket_shape_name(PocketShape shape) {
  switch (shape) {
    case PocketShape::RoundedRectangle: return "rounded_rectangle";
  }
  return "unknown";
}
std::optional<PocketShape> pocket_shape_from_name(const std::string& name) {
  // The old names remain readable so existing job files still load. They are
  // normalized to the one shape now exposed by the UI.
  if (name == "circle" || name == "rectangle" ||
      name == "rounded_rectangle")
    return PocketShape::RoundedRectangle;
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
    double rapid_plane, const PriorOperations& prior_operations,
    const ProgressCallback& progress, double progress_from, double progress_to,
    const JobGeometry* geometry) {
  const auto started = std::chrono::steady_clock::now();
  OperationPlan result;
  result.operation_name = operation.name; result.tool_id = tool.id;
  result.machine_id = machine.id; result.material_id = operation.material_id;
  try {
    require(is_parametric(operation.mode), "Expected pocket or plane clearing operation");
    const auto problems = validate(tool);
    if (!problems.empty()) throw std::runtime_error(problems.front());
    require(tool.type == ToolType::FlatEndMill,
            "Pocket and plane clearing require a flat end mill for a flat floor");
    require(positive(operation.stepdown) && positive(operation.stepover_ratio) &&
                operation.stepover_ratio <= 1, "Stepdown must be positive; stepover must be in (0, 1]");
    require(positive(operation.spindle_rpm) && positive(operation.plunge_feed),
            "Spindle speed and plunge feed must be positive and finite");
    require(positive(operation.path_pitch) && operation.path_pitch >= 0.001,
            "Path pitch must be finite and at least 0.001 mm");
    require(operation.tool_number >= 0 && operation.length_offset >= 0,
            "Tool number and length offset cannot be negative");
    const auto* feed_range = machine.feeds_for(operation.material_id);
    require(operation.manual_feed_range || feed_range,
            "Select a machine/material feed range or enter operation feeds manually");
    const double slow = operation.manual_feed_range ? operation.full_engagement_feed : feed_range->full_engagement;
    const double fast = operation.manual_feed_range ? operation.no_engagement_feed : feed_range->no_engagement;
    require(positive(slow) && positive(fast) && fast >= slow,
            "Feed range must be positive, with maximum feed at least the full-engagement feed");
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
    std::vector<double> boundaries;
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
      for (int layer = 0; layer <= region.layers; ++layer)
        boundaries.push_back(region.top -
                             std::min(region.depth, layer * operation.stepdown));
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
    result.composed_stock = geometry && geometry->has_stock();
    auto incoming_stack = result.composed_stock
        ? active_stock(*geometry, prior_operations, operation.slice_grid, boundaries)
        : std::vector<StockSlab>{};
    result.preview_context = incoming_stack;
    std::vector<StockSlab> preview_areas;
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
      if (result.composed_stock) {
        level.local_stock = stock_inside(incoming_stack, region.boundary, floor, upper);
        // Partition the initial blank once. Earlier areas own overlaps; each
        // disjoint pocket keeps separate geometry so its live cut does not
        // force retriangulation of every other pocket at the same height.
        auto initial = stock_inside(result.preview_context, region.boundary, floor, upper);
        preview_areas.insert(preview_areas.end(), initial.begin(), initial.end());
        result.preview_context = stock_outside(result.preview_context,
                                               region.boundary, floor, upper);
        // Local stock measures this cut's engagement and volume. Preview uses
        // the complete initial stack and applies cuts in execution order.
        for (const auto& slab : level.local_stock)
          level.incoming_stock.insert(level.incoming_stock.end(), slab.region->begin(), slab.region->end());
        level.incoming_stock = Union(level.incoming_stock, FillRule::NonZero);
      } else {
        const double sample_z = (upper+floor)/2;
        level.incoming_stock = stock_after_operations_at_z(region.boundary, sample_z, prior_operations);
        level.incoming_stock = stock_after_planned_operation_at_z(level.incoming_stock, sample_z, result);
      }
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
      PathBuilder path{level, operation, machine, previous.value_or(entry),
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
      finish_level(level, operation, tool, machine, slow, fast, rapid_plane);
      level.feeds.ends_at_depth = level.stays_down_after;
      const auto& stats = level.feeds.stats;
      ++result.stats.levels; ++result.stats.cut_levels;
      result.stats.cutting_distance += stats.cutting_distance;
      result.stats.rapid_distance += stats.rapid_distance;
      result.stats.cutting_minutes += stats.cutting_minutes;
      result.stats.rapid_minutes += stats.rapid_minutes;
      if (result.composed_stock) {
        for (const auto& slab : level.local_stock) {
          const auto removed = Intersect(*slab.region, level.simulation.removed_material, FillRule::NonZero);
          result.stats.removed_volume += std::abs(Area(removed)) /
              (double(wave::to_integer(1))*wave::to_integer(1)) * (slab.top-slab.bottom);
        }
        incoming_stack = stock_outside(incoming_stack, level.simulation.removed_material, floor, upper);
      } else result.stats.removed_volume += level.simulation.stats.removed_area*level.axial_depth;
      result.stats.maximum_load = std::max(result.stats.maximum_load, level.simulation.stats.maximum_load);
      result.stats.excess_area += level.simulation.stats.excess_area;
      result.metrics.toolpath_moves += level.toolpath.moves.size();
      result.metrics.simulation_intervals += level.simulation.stats.cutting_steps;
      result.levels.push_back(std::move(level));
      ++completed_levels;
    }
    result.preview_context.insert(result.preview_context.end(),
                                   preview_areas.begin(), preview_areas.end());
    result.spindle_rpm = commandable_rpm(machine, operation.spindle_rpm);
  } catch (const std::exception& error) {
    result.error = error.what();
  }
  result.metrics.total_milliseconds = std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-started).count();
  return result;
}
}  // namespace cam
