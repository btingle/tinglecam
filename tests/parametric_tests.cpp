#include "cam/config.h"
#include "cam/auto_pockets.h"
#include "cam/parametric.h"
#include "cam/post.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
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
  auto& pocket = value.pockets.front();
  pocket.center_x = 12; pocket.center_y = -8;
  pocket.top_z = 10; pocket.depth = 2.5;
  pocket.width = 24; pocket.height = 14; pocket.radius = 5;
  value.plane.areas = {{-25, 4, 20, 14, 10, 2.5},
                       {25, -4, 18, 12, 9, 1.5}};
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
      // A level whose material an earlier area already took is kept as a pass
      // with nothing in it rather than dropped, so playback and the operation
      // list still account for every area at every depth.
      if (level.nothing_to_cut) {
        check(level.feeds.motion.empty(), "A pass with nothing to cut carries no motion");
        continue;
      }
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
      if (level.stays_down_after)
        near(level.feeds.motion.back().to_z, level.floor_z,
             "A stay-down pass ends at its floor");
      else
        near(level.feeds.motion.back().to_z, result.rapid_plane,
             "Every independent pass ends retracted");
    }
  }
  near(seconds/60, result.stats.cutting_minutes + result.stats.rapid_minutes,
       "Timing uses the actual 3D motions", 1e-6);
}

void pockets() {
  for (const double radius : {0.0, 5.0, 7.0}) {
    auto op = operation(cam::OperationMode::Pocket);
    auto& pocket = op.pockets.front();
    pocket.radius = radius;
    const auto result = plan(op);
    check_timeline(result);
    const auto& levels = result.operations.front().levels;
    check(levels.size() == 3, "Pocket depth is divided by requested stepdown");
    check(levels[0].stays_down_after && levels[1].stays_down_after &&
              !levels[2].stays_down_after,
          "Pocket layers prefer staying down until the final layer");
    near(levels[0].feeds.motion.back().to_z, levels[0].floor_z,
         "First pocket layer stays at depth");
    near(levels[1].feeds.motion.front().from_z, levels[0].floor_z,
         "Next pocket layer starts where the previous one ended");
    near(levels.back().floor_z, 7.5, "Pocket finishes at top minus depth");
    for (const auto& level : levels) {
      const double upper = level.floor_z+level.axial_depth;
      bool helix = false;
      double angle_sum = 0;
      for (const auto& phase : level.feeds.motion) {
        if (phase.rapid) continue;
        for (const auto& at : {phase.from, phase.to}) {
          const double x = std::abs(wave::to_world(at.x)-pocket.center_x);
          const double y = std::abs(wave::to_world(at.y)-pocket.center_y);
          const double cutter_radius = tool().diameter/2;
          check(x+cutter_radius <= pocket.width/2+0.004 &&
                    y+cutter_radius <= pocket.height/2+0.004,
                "Rounded-rectangle motion respects the finished dimensions");
          if (radius > 0)
            check(std::hypot(std::max(0.0,x-(pocket.width/2-pocket.radius)),
                             std::max(0.0,y-(pocket.height/2-pocket.radius))) <=
                      pocket.radius-cutter_radius+0.004,
                  "The cutter remains inside rounded pocket corners");
        }
        if (phase.to_z < upper-1e-8 && phase.to_z < phase.from_z-1e-8) {
          check(phase.kind == wave::MoveKind::Entry && phase.arc,
                "Every descent into a new pocket layer must be a helical arc");
          const double xy = phase.path_distance_end - phase.path_distance_begin;
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
      const double target = wave::material_area(cam::pocket_boundary(pocket));
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
    int arcs = 0;
    double x = 0, y = 0, z = 0;
    while (std::getline(lines,line)) {
      if (line.empty() || line.front() == '(') continue;
      std::istringstream words(line); std::string word;
      double nx = x, ny = y, nz = z;
      bool coordinate = false;
      while (words >> word) {
        if (word == "G2" || word == "G3") ++arcs;
        if (word.size() < 2) continue;
        if (word[0] == 'X') { nx = std::stod(word.substr(1)); coordinate = true; }
        if (word[0] == 'Y') { ny = std::stod(word.substr(1)); coordinate = true; }
        if (word[0] == 'Z') { nz = std::stod(word.substr(1)); coordinate = true; }
      }
      if (coordinate) { x = nx; y = ny; z = nz; }
    }
    check(arcs > 0, "G-code emits helical entries as circular moves");
    check(post.text.find("G3") != std::string::npos &&
              post.text.find(" I") != std::string::npos &&
              post.text.find(" J") != std::string::npos,
          "Helical circular moves include the controller arc centre offsets");
    check(post.text.find("Z14.5") != std::string::npos, "Post applies the origin to final pocket depth");
  }
}

// The radial margin moves every wall of the pocket by it, inwards or, when it
// is negative, outwards. What gets cut must follow the margined outline rather
// than the entered one.
void pocket_margin() {
  const double cutter_radius = tool().diameter/2;
  for (const double margin : {0.4, -0.4}) {
    auto op = operation(cam::OperationMode::Pocket);
    op.pocket_radial_margin = margin;
    const auto& pocket = op.pockets.front();
    const auto result = plan(op);
    check_timeline(result);
    double reach_x = 0, reach_y = 0;
    for (const auto& level : result.operations.front().levels) {
      const double target =
          wave::material_area(cam::pocket_boundary(pocket, margin));
      check(level.simulation.stats.removed_area > target*0.985,
            "A margined pocket is still cleared to its own wall");
      for (const auto& phase : level.feeds.motion) {
        if (phase.rapid) continue;
        for (const auto& at : {phase.from, phase.to}) {
          reach_x = std::max(reach_x, std::abs(wave::to_world(at.x)-pocket.center_x)+cutter_radius);
          reach_y = std::max(reach_y, std::abs(wave::to_world(at.y)-pocket.center_y)+cutter_radius);
        }
      }
    }
    check(reach_x <= pocket.width/2-margin+0.004 &&
              reach_y <= pocket.height/2-margin+0.004,
          "The cut stops at the wall the margin asks for");
    check(reach_x > pocket.width/2-margin-0.05 &&
              reach_y > pocket.height/2-margin-0.05,
          "The cut reaches the wall the margin asks for");
  }
  auto bad = operation(cam::OperationMode::Pocket);
  bad.pocket_radial_margin = 7;
  check(!plan(bad).ok(), "A margin that closes the pocket must fail planning");
}

void multiple_pockets() {
  auto op = operation(cam::OperationMode::Pocket);
  auto second = op.pockets.front();
  second.center_x = -22;
  second.center_y = 10;
  second.top_z = 8;
  second.depth = 1;
  op.pockets.push_back(second);
  const auto result = plan(op);
  check_timeline(result);
  check(result.operations.front().levels.size() == 4,
        "One pocket operation can contain several pocket regions");
  check(result.operations.front().levels[2].floor_z == 7.5,
        "The first pocket finishes before the next pocket begins");
  check(!result.operations.front().levels[2].stays_down_after,
        "Different pockets remain separated by a retract");
}

void planes() {
  const auto op = operation(cam::OperationMode::PlaneClearing);
  const auto result = plan(op);
  check_timeline(result);
  const auto& levels = result.operations.front().levels;
  check(levels.size() == 5, "Both clearing areas are visited at each depth");
  for (std::size_t index = 0; index < levels.size(); ++index) {
    const auto& level = levels[index];
    const auto& area = op.plane.areas[index%2];
    const double expected_floor = index == 0 ? 9.0 : index == 1 ? 8.0 :
        index == 2 ? 8.0 : 7.5;
    near(level.floor_z, expected_floor,
         "Each plane area uses its own top and depth");
    const double radius = tool().diameter/2;
    bool left = false, right = false, overran = false;
    for (const auto& phase : level.feeds.motion) {
      if (phase.rapid || phase.kind == wave::MoveKind::Entry) continue;
      near(phase.to_z, level.floor_z, "Raster is at the requested layer depth");
      check(phase.from.x == phase.to.x || phase.from.y == phase.to.y,
            "Raster uses straight rows and end connections");
      left |= phase.to.x < phase.from.x;
      right |= phase.to.x > phase.from.x;
      for (const auto& point : {phase.from,phase.to}) {
        const double x = std::abs(wave::to_world(point.x)-area.center_x);
        check(x <= area.width/2+radius+0.001,
              "Facing rows overrun the area's X edges by exactly a radius");
        overran |= x >= area.width/2+radius-0.001;
        check(std::abs(wave::to_world(point.y)-area.center_y)+radius <= area.height/2+0.001,
              "Rows stay within the area's Y bounds, which the overrun covers");
      }
    }
    check(left && right, "Plane clearing alternates row direction");
    check(overran, "Every row runs clear off the edge it ends on");
    // Descending inside the area would cut into the surface being faced with
    // the whole of the cutter. The area is a statement about the surface, not
    // about the air beside it, so the plunge belongs outside it.
    bool descended = false;
    for (const auto& phase : level.feeds.motion) {
      if (phase.rapid || phase.to_z >= phase.from_z-1e-9) continue;
      check(std::abs(wave::to_world(phase.to.x)-area.center_x) >=
                area.width/2+radius-0.001,
            "Facing descends clear of the area, not into it");
      descended = true;
    }
    check(descended, "Every facing pass descends to its floor");
    check(level.subdivision_level == index, "Area playback retains execution order");
    check(level.simulation.stats.removed_area > area.width*area.height*0.999,
          "Facing clears the complete surface, corners included");
  }
}

void automatic_pockets() {
  const std::filesystem::path fixture =
      std::filesystem::path(__FILE__).parent_path() /
      "fixtures/meshes/auto_pocket.obj";
  cam::JobGeometry geometry;
  geometry.load_part(fixture);
  geometry.set_box_stock(true, 2.0);
  auto op = operation(cam::OperationMode::Pocket);
  op.auto_identify_pockets = true;
  op.pocket_auto.check_depth = 1.0;
  op.pocket_auto.helix_angle_degrees = 6.0;
  op.pockets.clear();
  const auto identified = cam::identify_pockets(geometry, op);
  check(identified.size() == 1,
        "Automatic pocket detection finds the blind hole: " +
            std::to_string(identified.size()));
  near(identified.front().center_x, 10.0, "Identified pocket center X");
  near(identified.front().center_y, 10.0, "Identified pocket center Y");
  near(identified.front().width, 4.0, "Identified pocket width");
  near(identified.front().height, 4.0, "Identified pocket height");
  near(identified.front().bottom_z, 3.0, "GPU ray finds pocket floor", 0.002);
  const auto planned = cam::plan_parametric_operation(
      op, tool(), machine(), 20.0, {}, {}, 0.0, 1.0, &geometry);
  check(planned.ok(), planned.error);
  check(planned.identified_pockets.size() == 1,
        "Automatic pocket geometry is retained in the completed plan");
  check(!planned.levels.empty(), "Automatic pocket produces cutting levels");
  bool auto_arc = false;
  for (const auto& level : planned.levels)
    for (const auto& phase : level.feeds.motion)
      if (phase.arc) {
        auto_arc = true;
        check((phase.from_z - phase.to_z) /
                  (phase.path_distance_end - phase.path_distance_begin) > 0.08,
              "Automatic pockets use the configured helix angle");
      }
  check(auto_arc, "Automatic pockets retain circular helical entries");

  cam::JobGeometry through_geometry;
  through_geometry.load_part(std::filesystem::path(__FILE__).parent_path() /
                             "fixtures/meshes/auto_through_pocket.obj");
  through_geometry.set_box_stock(true, 2.0);
  const auto through = cam::identify_pockets(through_geometry, op);
  check(through.size() == 1, "Automatic detection keeps through holes");
  near(through.front().bottom_z, 0.0, "A missed ray uses the stock bottom", 0.002);

  op.pocket_auto.exclusions.front().enabled = true;
  op.pocket_auto.exclusions.front().center_x = 10.0;
  op.pocket_auto.exclusions.front().center_y = 10.0;
  op.pocket_auto.exclusions.front().width = 5.0;
  op.pocket_auto.exclusions.front().height = 5.0;
  op.pocket_auto.exclusions.push_back({true, -10.0, -10.0, 2.0, 2.0});
  check(cam::identify_pockets(geometry, op).empty(),
        "The exclusion area removes overlapping detected pockets");
  const auto automatic_round_trip = cam::job_from_json_text(
      cam::to_json_text(cam::Job{.operations = {op}}));
  check(automatic_round_trip.operations.front().auto_identify_pockets &&
            automatic_round_trip.operations.front().pocket_auto.exclusions.size() == 2 &&
            automatic_round_trip.operations.front().pocket_auto.exclusions.front().enabled &&
            automatic_round_trip.operations.front().pocket_auto.helix_angle_degrees == 6.0,
        "Automatic pocket settings round-trip without saving derived geometry");

}

void validation_and_storage() {
  auto op = operation(cam::OperationMode::Pocket);
  auto expect_error = [&](const cam::Operation& invalid) { check(!plan(invalid).ok(), "Invalid input must fail planning"); };
  auto bad = op; bad.pockets.front().depth = 26; expect_error(bad);
  bad = op; bad.pockets.front().width = 2; expect_error(bad);
  bad = op; bad.pockets.front().helix_angle_degrees = 0; expect_error(bad);
  bad = op; bad.pockets.front().width = 4; bad.pockets.front().height = 4;
  bad.pockets.front().radius = 1; expect_error(bad);
  bad = op; bad.stepover_ratio = 0; expect_error(bad);
  bad = op; bad.stepdown = std::numeric_limits<double>::infinity(); expect_error(bad);
  bad = operation(cam::OperationMode::PlaneClearing); bad.plane.areas.clear(); expect_error(bad);
  bad = operation(cam::OperationMode::PlaneClearing); bad.plane.areas.front().width = 1; expect_error(bad);
  const auto cancelled = cam::plan_parametric_operation(op, tool(), machine(), 20, {},
      [](double,const std::string&) { return false; });
  check(cancelled.error == "Cancelled", "Cancellation must stop parametric planning");

  auto overlap = operation(cam::OperationMode::PlaneClearing);
  overlap.plane.areas = {{0,0,20,20,10,2.5}, {0,0,20,20,10,2.5}};
  const auto overlapping = plan(overlap);
  check_timeline(overlapping);
  const auto& overlap_levels = overlapping.operations.front().levels;
  check(overlap_levels[1].nothing_to_cut,
        "A coincident later area finds the surface already faced off");

  cam::Job job; job.operations = {op, operation(cam::OperationMode::PlaneClearing)};
  job.operations[0].pocket_radial_margin = -0.25;
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
  try { pockets(); pocket_margin(); multiple_pockets(); planes(); automatic_pockets();
        validation_and_storage(); }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
  std::cout << "Parametric pocket, plane, motion and persistence checks passed.\n";
}
