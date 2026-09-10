#include "cam/stock.h"
#include "cam/parametric.h"

#include <clipper2/clipper.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
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
    case PocketShape::Circle: return "circle";
    case PocketShape::Rectangle: return "rectangle";
    case PocketShape::RoundedRectangle: return "rounded_rectangle";
  }
  return "unknown";
}
std::optional<PocketShape> pocket_shape_from_name(const std::string& name) {
  if (name == "circle") return PocketShape::Circle;
  if (name == "rectangle") return PocketShape::Rectangle;
  if (name == "rounded_rectangle") return PocketShape::RoundedRectangle;
  return std::nullopt;
}

wave::Paths64 pocket_boundary(const PocketParameters& p) {
  require(pocket_shape_from_name(pocket_shape_name(p.shape)).has_value(), "Unknown pocket shape");
  if (p.shape == PocketShape::Circle) {
    require(positive(p.radius) && p.radius < 1e7, "Circle radius must be positive and finite");
    return {Ellipse(point(p.center_x, p.center_y), wave::to_integer(p.radius),
                    wave::to_integer(p.radius), circle_segments(p.radius))};
  }
  require(positive(p.width) && positive(p.height) && p.width < 1e7 && p.height < 1e7,
          "Pocket width and height must be positive and finite");
  const double radius = p.shape == PocketShape::Rectangle ? 0 : p.radius;
  require(std::isfinite(radius) && radius >= 0 && radius <= std::min(p.width,p.height)/2,
          "Corner radius must be between zero and half the smaller pocket dimension");
  return {rounded_rectangle(p.center_x, p.center_y, p.width, p.height, radius)};
}

OperationPlan plan_parametric_operation(
    const Operation& operation, const Tool& tool, const Machine& machine,
    double rapid_plane, const std::vector<OperationPlan>& prior_operations,
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
    const auto& p = operation.pocket;
    const double top = pocket ? p.top_z : operation.plane.top_z;
    const double depth = pocket ? p.depth : operation.plane.depth;
    require(std::isfinite(top) && positive(depth) && depth < 1e7,
            "Top Z must be finite and depth must be positive and finite");
    require(std::isfinite(rapid_plane) && rapid_plane > top,
            "Rapid clearance must be above the operation's top surface");
    require(depth <= tool.flute_length,
            "Operation depth exceeds flute length; choose a tool with sufficient reach");
    const double radius = tool.diameter/2;
    const double step = operation.stepover_ratio * tool.diameter;
    require(step >= 0.001, "Stepover is below the 0.001 mm geometry resolution");
    const double layer_count = std::ceil(depth/operation.stepdown);
    require(layer_count <= 20000, "Too many depth levels; increase stepdown");
    const int layers = static_cast<int>(layer_count);
    const std::size_t areas = pocket ? 1 : operation.plane.areas.size();
    require(areas > 0, "Add at least one plane clearing area");
    require(areas * layer_count <= 20000, "Too many area/depth passes");
    Paths64 boundary;
    Paths64 centres;
    double inradius = 0;
    double helix_radius = 0;
    if (pocket) {
      boundary = pocket_boundary(p);
      require(positive(p.helix_angle_degrees) && p.helix_angle_degrees <= 10,
              "Helix angle must be in (0, 10] degrees");
      const double small = p.shape == PocketShape::Circle ? 2*p.radius : std::min(p.width,p.height);
      inradius = small/2 - radius;
      require(inradius >= 0.01, "Pocket is too small for this tool to enter helically");
      require(p.shape != PocketShape::RoundedRectangle || p.radius >= radius,
              "Pocket corner radius is smaller than the tool radius");
      centres = InflatePaths(boundary, -wave::to_integer(radius), JoinType::Round, EndType::Polygon);
      require(centres.size() == 1, "Pocket has no continuous cutter-centre region");
      helix_radius = std::min(radius*0.5, inradius*0.75);
      require(helix_radius >= 0.02,
              "Pocket/tool combination is too small for a helix at 0.001 mm resolution");
      if (p.shape == PocketShape::Rectangle)
        result.warnings.push_back("Square pocket corners retain the tool radius; a round cutter cannot make sharp internal corners.");
    } else {
      for (const auto& area : operation.plane.areas) {
        require(positive(area.width) && positive(area.height) &&
                    area.width >= tool.diameter && area.height >= tool.diameter &&
                    std::max(area.width, area.height) > tool.diameter + 0.001,
                "Each clearing area must be at least as wide and tall as the tool diameter");
        point(area.center_x-area.width/2, area.center_y-area.height/2);
        point(area.center_x+area.width/2, area.center_y+area.height/2);
      }
    }
    result.composed_stock = geometry && geometry->has_stock();
    std::vector<double> boundaries;
    for (int layer = 0; layer <= layers; ++layer)
      boundaries.push_back(top-std::min(depth, layer*operation.stepdown));
    auto incoming_stack = result.composed_stock
        ? active_stock(*geometry, prior_operations, operation.slice_grid, boundaries)
        : std::vector<StockSlab>{};
    result.preview_context = incoming_stack;
    std::optional<Point64> previous;
    for (auto op = prior_operations.rbegin(); op != prior_operations.rend() && !previous; ++op)
      if (op->ok()) for (auto level = op->levels.rbegin(); level != op->levels.rend(); ++level)
        if (!level->feeds.motion.empty()) { previous = level->feeds.motion.back().to; break; }
    for (int layer = 0; layer < layers; ++layer) {
      const double upper = top-std::min(depth, layer*operation.stepdown);
      const double floor = top-std::min(depth, (layer+1)*operation.stepdown);
      for (std::size_t area_index = 0; area_index < areas; ++area_index) {
        if (progress && !progress(progress_from + (progress_to-progress_from)*
                (layer*areas+area_index)/(layers*areas),
                operation.name + ": depth " + std::to_string(layer+1) +
                "/" + std::to_string(layers) + ", area " + std::to_string(area_index+1)))
          throw std::runtime_error("Cancelled");
        LevelPlan level;
        level.explicit_motion = true; level.floor_z = floor; level.axial_depth = upper-floor;
        level.subdivision_level = result.levels.size();
        ClearingArea area;
        if (!pocket) {
          area = operation.plane.areas[area_index];
          boundary = {rounded_rectangle(area.center_x, area.center_y, area.width, area.height, 0)};
        }
        level.stock_slice = boundary;
        if (result.composed_stock) {
          level.local_stock = stock_inside(incoming_stack, boundary, floor, upper);
          result.preview_context = stock_outside(result.preview_context, boundary, floor, upper);
          // Overlapping areas transfer ownership to the later area. Its local
          // stock includes earlier cuts, while earlier playback still owns the
          // overlap until the later area starts (handled by the preview).
          for (const auto& slab : level.local_stock)
            level.incoming_stock.insert(level.incoming_stock.end(), slab.region->begin(), slab.region->end());
          level.incoming_stock = Union(level.incoming_stock, FillRule::NonZero);
        } else {
          const double sample_z = (upper+floor)/2;
          level.incoming_stock = stock_after_operations_at_z(boundary, sample_z, prior_operations);
          level.incoming_stock = stock_after_planned_operation_at_z(level.incoming_stock, sample_z, result);
        }
        if (level.incoming_stock.empty()) {
          level.nothing_to_cut = true;
          ++result.stats.levels;
          result.levels.push_back(std::move(level));
          continue;
        }
        const Point64 entry = pocket ? point(p.center_x+helix_radius,p.center_y) :
            point(area.center_x-area.width/2+radius, area.center_y-area.height/2+radius);
        PathBuilder path{level, operation, machine, previous.value_or(entry), rapid_plane};
        path.line(entry, rapid_plane, wave::MoveKind::Retract, true);
        path.line(entry, top, wave::MoveKind::Entry, true);
        path.line(entry, upper, wave::MoveKind::Entry);
        if (pocket) {
          const double pitch = 2*pi*helix_radius*std::tan(p.helix_angle_degrees*pi/180);
          const double turns = std::ceil((upper-floor)/pitch);
          const int samples = circle_segments(helix_radius);
          require(turns*samples <= max_phases, "Helix requires too many segments; increase its angle or pocket size");
          const int segments = static_cast<int>(turns)*samples;
          for (int i = 1; i <= segments; ++i) {
            const double angle = 2*pi*i/samples;
            path.line(point(p.center_x+helix_radius*std::cos(angle),
                            p.center_y+helix_radius*std::sin(angle)),
                      upper+(floor-upper)*i/segments, wave::MoveKind::Entry);
          }
          // One complete turn at the floor removes the sloped helix bottom.
          for (int i = 1; i <= samples; ++i) {
            const double angle = 2*pi*i/samples;
            path.line(point(p.center_x+helix_radius*std::cos(angle),
                            p.center_y+helix_radius*std::sin(angle)), floor, wave::MoveKind::Cut);
          }
          const double inner = std::max(0.0, inradius-0.005);
          const double rings_needed = std::ceil(inner/step);
          require(rings_needed <= 20000, "Too many pocket rings; increase stepover");
          const int rings = static_cast<int>(rings_needed);
          for (int ring = rings; ring >= 0; --ring) {
            const double inset = rings ? inner*ring/rings : 0;
            const Paths64 contours = ring == 0 ? centres : InflatePaths(
                centres, -wave::to_integer(inset), JoinType::Round, EndType::Polygon);
            for (const auto& contour : contours) path.ring(contour);
          }
        } else {
          path.line(entry, floor, wave::MoveKind::Entry);
          const double left = area.center_x-area.width/2+radius;
          const double right = area.center_x+area.width/2-radius;
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
          // Close both side edges; alternating row connectors alone leave
          // scallops on the opposite edge of every second row.
          path.line(point(wave::to_world(path.position.x), bottom), floor, wave::MoveKind::Cut);
          path.ring(rounded_rectangle(area.center_x, area.center_y,
                                      area.width-tool.diameter,
                                      area.height-tool.diameter, 0));
        }
        path.line(path.position, rapid_plane, wave::MoveKind::Retract, true);
        previous = path.position;
        finish_level(level, operation, tool, machine, slow, fast, rapid_plane);
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
      }
    }
    result.spindle_rpm = commandable_rpm(machine, operation.spindle_rpm);
  } catch (const std::exception& error) {
    result.error = error.what();
  }
  result.metrics.total_milliseconds = std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-started).count();
  return result;
}
}  // namespace cam
