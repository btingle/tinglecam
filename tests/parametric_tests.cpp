#include "cam/config.h"
#include "cam/parametric.h"
#include "cam/post.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace {
void check(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
void near(double a, double b, const char* message, double tolerance = 1e-8) {
  check(std::abs(a-b) <= tolerance, message);
}
cam::Tool tool() {
  cam::Tool value;
  value.id = "cutter"; value.name = "3.175 mm cutter";
  value.diameter = 3.175; value.flute_length = 25; value.flutes = 3;
  return value;
}
cam::Machine machine() {
  cam::Machine value;
  value.id = "machine"; value.name = "Test machine";
  value.rapid_feed = 3000; value.rapid_feed_z = 500;
  value.min_rpm = 1000; value.max_rpm = 24000;
  value.feed_ranges.push_back({"material", 500, 1500});
  return value;
}
cam::Operation operation(cam::OperationMode mode) {
  cam::Operation value;
  value.mode = mode; value.tool_id = "cutter"; value.machine_id = "machine";
  value.material_id = "material"; value.stepover_ratio = 0.8; value.stepdown = 1;
  value.spindle_rpm = 17000; value.plunge_feed = 135;
  value.pocket.center_x = 12; value.pocket.center_y = -8;
  value.pocket.top_z = 10; value.pocket.depth = 2.5;
  value.pocket.width = 24; value.pocket.height = 14; value.pocket.radius = 5;
  value.plane.top_z = 10; value.plane.depth = 2.5;
  value.plane.areas = {{-25, 4, 20, 14}, {25, -4, 18, 12}};
  return value;
}
cam::JobPlan plan(const cam::Operation& op) {
  cam::Job job; job.operations = {op};
  return cam::plan_job({}, job, {{tool()}}, {{machine()}},
                       {{{"material", "Test material"}}}, {});
}
void check_timeline(const cam::JobPlan& result) {
  check(result.ok(), result.error);
  check(!result.operations.empty() && result.operations.front().ok(), "Operation must plan");
  double seconds = 0;
  for (const auto& op : result.operations) {
    for (const auto& level : op.levels) {
      check(level.explicit_motion, "Parametric paths must carry actual XYZ motion");
      check(!level.feeds.motion.empty(), "A pass must have machine motion");
      double distance = 0;
      for (std::size_t i = 0; i < level.feeds.motion.size(); ++i) {
        const auto& phase = level.feeds.motion[i];
        check(phase.rate > 0 && std::isfinite(phase.rate), "Finite positive feed");
        check(std::isfinite(phase.length), "Finite motion length");
        near(phase.path_distance_begin, distance, "Planar timeline continuity");
        distance = phase.path_distance_end;
        seconds += phase.length/phase.rate*60;
        if (i > 0) {
          const auto& previous = level.feeds.motion[i-1];
          check(phase.from == previous.to, "XYZ motion must not teleport between phases");
          near(phase.from_z, previous.to_z, "Z timeline continuity");
        }
        if (phase.rapid && phase.from != phase.to) {
          near(phase.from_z, result.rapid_plane, "XY rapids start at clearance");
          near(phase.to_z, result.rapid_plane, "XY rapids remain at clearance");
        }
      }
      near(distance, wave::toolpath_length(level.toolpath), "Projection matches machine timeline");
      near(level.feeds.motion.back().to_z, result.rapid_plane, "Every area ends retracted");
    }
  }
  near(seconds/60, result.stats.cutting_minutes + result.stats.rapid_minutes,
       "Timing uses the actual 3D motions", 1e-6);
}

void pockets() {
  for (auto shape : {cam::PocketShape::Circle, cam::PocketShape::Rectangle,
                     cam::PocketShape::RoundedRectangle}) {
    auto op = operation(cam::OperationMode::Pocket);
    op.pocket.shape = shape;
    const auto result = plan(op);
    check_timeline(result);
    const auto& levels = result.operations.front().levels;
    check(levels.size() == 3, "Pocket depth is divided by requested stepdown");
    near(levels.back().floor_z, 7.5, "Pocket finishes at top minus depth");
    for (const auto& level : levels) {
      const double upper = level.floor_z+level.axial_depth;
      bool helix = false;
      double angle_sum = 0;
      for (const auto& phase : level.feeds.motion) {
        if (phase.rapid) continue;
        for (const auto& at : {phase.from, phase.to}) {
          const double x = std::abs(wave::to_world(at.x)-op.pocket.center_x);
          const double y = std::abs(wave::to_world(at.y)-op.pocket.center_y);
          const double cutter_radius = tool().diameter/2;
          if (shape == cam::PocketShape::Circle)
            check(std::hypot(x,y)+cutter_radius <= op.pocket.radius+0.004,
                  "Circular pocket motion keeps the cutter within the opening");
          else {
            check(x+cutter_radius <= op.pocket.width/2+0.004 &&
                      y+cutter_radius <= op.pocket.height/2+0.004,
                  "Rectangular pocket motion respects the finished dimensions");
            if (shape == cam::PocketShape::RoundedRectangle)
              check(std::hypot(std::max(0.0,x-(op.pocket.width/2-op.pocket.radius)),
                               std::max(0.0,y-(op.pocket.height/2-op.pocket.radius))) <=
                        op.pocket.radius-cutter_radius+0.004,
                    "The cutter remains inside rounded pocket corners");
          }
        }
        if (phase.to_z < upper-1e-8 && phase.to_z < phase.from_z-1e-8) {
          check(phase.kind == wave::MoveKind::Entry && phase.from != phase.to,
                "Every descent into a new pocket layer must be helical, not vertical");
          const double xy = std::hypot(wave::to_world(phase.to.x-phase.from.x),
                                       wave::to_world(phase.to.y-phase.from.y));
          check((phase.from_z-phase.to_z)/xy < 0.055,
                "Helical plunge must remain gradual (approximately three degrees)");
          near(phase.rate, op.plunge_feed, "Helix uses operation ramp feed");
          helix = true;
          angle_sum += xy;
        }
        check(phase.to_z >= level.floor_z-1e-9, "No move goes below its pocket floor");
      }
      check(helix && angle_sum > 1, "Every depth level has a helical entry");
      check(level.simulation.stats.unresolved_entries == 0,
            "A real helix must not be reported as an unresolved plunge");
      const double target = wave::material_area(cam::pocket_boundary(op.pocket));
      check(level.simulation.stats.removed_area > target*0.985,
            "Pocket passes clear the interior, including the central core");
    }
    cam::Job job; job.operations = {op}; job.origin_x = 2; job.origin_y = -3; job.origin_z = 7;
    const auto post = cam::post_job(result, job, {{tool()}}, {{machine()}});
    check(post.ok(), post.error);
    check(post.text.find("M3 S17000") != std::string::npos, "Operation RPM reaches G-code");
    check(post.text.find("F135") != std::string::npos, "Ramp feed reaches G-code");
    std::istringstream lines(post.text);
    std::string line;
    int mode = -1, simultaneous = 0;
    double x = 0, y = 0, z = 0;
    bool have_position = false;
    while (std::getline(lines,line)) {
      if (line.empty() || line.front() == '(') continue;
      std::istringstream words(line); std::string word;
      double nx = x, ny = y, nz = z;
      bool coordinate = false;
      while (words >> word) {
        if (word == "G0") mode = 0;
        if (word == "G1") mode = 1;
        if (word.size() < 2) continue;
        if (word[0] == 'X') { nx = std::stod(word.substr(1)); coordinate = true; }
        if (word[0] == 'Y') { ny = std::stod(word.substr(1)); coordinate = true; }
        if (word[0] == 'Z') { nz = std::stod(word.substr(1)); coordinate = true; }
      }
      if (coordinate && mode == 1 && have_position && nz < z && (nx != x || ny != y)) ++simultaneous;
      if (coordinate) { x = nx; y = ny; z = nz; have_position = true; }
    }
    check(simultaneous > 60, "G-code retains simultaneous XY/Z helix motion");
    check(post.text.find("Z14.5") != std::string::npos, "Post applies the origin to final pocket depth");
  }
}

void planes() {
  const auto op = operation(cam::OperationMode::PlaneClearing);
  const auto result = plan(op);
  check_timeline(result);
  const auto& levels = result.operations.front().levels;
  check(levels.size() == 6, "Both clearing areas are visited at each depth");
  for (std::size_t index = 0; index < levels.size(); ++index) {
    const auto& level = levels[index];
    const auto& area = op.plane.areas[index%2];
    bool left = false, right = false;
    for (const auto& phase : level.feeds.motion) {
      if (phase.rapid || phase.kind == wave::MoveKind::Entry) continue;
      near(phase.to_z, level.floor_z, "Raster is at the requested layer depth");
      check(phase.from.x == phase.to.x || phase.from.y == phase.to.y,
            "Raster uses straight rows and end connections");
      left |= phase.to.x < phase.from.x;
      right |= phase.to.x > phase.from.x;
      for (const auto& point : {phase.from,phase.to}) {
        check(std::abs(wave::to_world(point.x)-area.center_x)+tool().diameter/2 <= area.width/2+0.001,
              "Cutter remains within the selected area's X bounds");
        check(std::abs(wave::to_world(point.y)-area.center_y)+tool().diameter/2 <= area.height/2+0.001,
              "Cutter remains within the selected area's Y bounds");
      }
    }
    check(left && right, "Plane clearing alternates row direction");
    check(level.subdivision_level == index, "Area playback retains execution order");
    check(level.simulation.stats.removed_area > area.width*area.height*0.98,
          "Raster clears the rectangle except its rounded corners");
  }
}

void validation_and_storage() {
  auto op = operation(cam::OperationMode::Pocket);
  auto expect_error = [&](const cam::Operation& invalid) { check(!plan(invalid).ok(), "Invalid input must fail planning"); };
  auto bad = op; bad.pocket.depth = 26; expect_error(bad);
  bad = op; bad.pocket.width = 2; expect_error(bad);
  bad = op; bad.pocket.helix_angle_degrees = 0; expect_error(bad);
  bad = op; bad.pocket.shape = cam::PocketShape::RoundedRectangle; bad.pocket.radius = 1; expect_error(bad);
  bad = op; bad.stepover_ratio = 0; expect_error(bad);
  bad = op; bad.stepdown = std::numeric_limits<double>::infinity(); expect_error(bad);
  bad = operation(cam::OperationMode::PlaneClearing); bad.plane.areas.clear(); expect_error(bad);
  bad = operation(cam::OperationMode::PlaneClearing); bad.plane.areas.front().width = 1; expect_error(bad);
  const auto cancelled = cam::plan_parametric_operation(op, tool(), machine(), 20, {},
      [](double,const std::string&) { return false; });
  check(cancelled.error == "Cancelled", "Cancellation must stop parametric planning");

  auto overlap = operation(cam::OperationMode::PlaneClearing);
  overlap.plane.areas = {{0,0,20,20}, {0,0,20,20}};
  const auto overlapping = plan(overlap);
  check_timeline(overlapping);
  const auto& overlap_levels = overlapping.operations.front().levels;
  check(overlap_levels[1].simulation.stats.removed_area < 0.01,
        "Overlapping areas inherit material removed earlier in the pass");

  cam::Job job; job.operations = {op, operation(cam::OperationMode::PlaneClearing)};
  job.operations[0].tool_number = 8; job.operations[0].length_offset = 4;
  job.operations[0].coolant = true;
  const auto text = cam::to_json_text(job);
  const auto loaded = cam::job_from_json_text(text);
  check(cam::to_json_text(loaded) == text, "Shapes, areas, feeds and tool assignments round-trip");
  const auto legacy = cam::tool_from_json_text(R"({"id":"legacy","name":"Old cutter",
    "diameter":3.175,"flute_length":25,"flutes":4,"type":"flat",
    "cutting_data":[{"max_ae_over_d":0.001,"max_ap_over_d":0.001}],"notes":"old"})");
  check(legacy.flutes == 4 && legacy.type == cam::ToolType::FlatEndMill,
        "Legacy tools retain physical attributes");
  const auto simplified = cam::to_json_text(legacy);
  check(simplified.find("cutting_data") == std::string::npos &&
            simplified.find("notes") == std::string::npos &&
            simplified.find("tool_number") == std::string::npos,
        "Tool files contain only identity and physical attributes");
}
}  // namespace

int main() {
  try { pockets(); planes(); validation_and_storage(); }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
  std::cout << "Parametric pocket, plane, motion and persistence checks passed.\n";
}
