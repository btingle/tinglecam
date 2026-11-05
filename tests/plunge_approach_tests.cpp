#include "cam/config.h"
#include "cam/heightmap_simulation.h"
#include "cam/post.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

using namespace cam;
namespace {
void check(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
void near(double a, double b, const char* message) {
  check(std::abs(a-b) < 1e-7, std::string(message) + ": " + std::to_string(a) + " vs " + std::to_string(b));
}
MotionPhase motion(wave::Vec3d a, wave::Vec3d b, wave::MoveKind kind, bool rapid = false) {
  MotionPhase p;
  p.from = {wave::to_integer(a.x),wave::to_integer(a.y)};
  p.to = {wave::to_integer(b.x),wave::to_integer(b.y)};
  p.from_z = a.z; p.to_z = b.z;
  p.length = wave::distance(a,b); p.kind = kind; p.rapid = rapid;
  p.rate = rapid ? 1500 : 120;
  return p;
}
}
int main() {
  try {
    Tool tool;
    tool.id = "flat"; tool.name = "Test flat";
    tool.diameter = 2; tool.flute_length = 25;
    tool.cutting = {{"test",12000,.02,120}};
    Machine machine;
    machine.id = "test"; machine.name = "Test"; machine.max_rpm = 24000;
    machine.rapid_feed = 3000; machine.rapid_feed_z = 1500;
    const auto feeds = resolve_cutting_feeds(tool,machine,"test",{});
    Job job;
    job.machine_id = "test"; job.material_id = "test"; job.planning.simulation_pitch = .1;
    Operation op;
    op.mode = OperationMode::SurfaceFinishing; op.tool_id = tool.id;
    job.operations = {op};
    const auto stock = wave::Heightmap::box({{-5,-5,0},{5,5,10},true},.1);
    const auto entry = motion({0,0,20},{0,0,5},wave::MoveKind::Entry);
    const auto run = [&](const wave::Heightmap& initial, std::vector<MotionPhase> phases,
                         const Tool& cutter, const Job& settings, const Operation& operation) {
      OperationPlan before;
      before.stock_after = std::make_shared<const wave::Heightmap>(initial);
      OperationPlan plan;
      plan.planned = true; plan.tool_id = cutter.id;
      plan.levels.emplace_back();
      plan.levels[0].floor_z = phases.back().to_z;
      plan.levels[0].axial_depth = 1;
      plan.levels[0].feeds.motion = std::move(phases);
      simulate_operation_xyz(plan,{},settings,operation,cutter,machine,feeds,{&before});
      check(plan.ok(),plan.error);
      return plan;
    };
    const auto result = run(stock,{entry},tool,job,op);
    const auto& level = result.levels[0];
    const auto& phases = level.feeds.motion;
    check(phases.size() == 2 && phases[0].rapid && !phases[1].rapid,
          "A vertical entry rapids through air and feeds near the stock");
    near(phases[0].from_z,20,"Rapid approach starts at the original clearance");
    near(phases[0].to_z,11,"Rapid stops one millimetre above current stock");
    near(phases[1].from_z,11,"Plunge starts at the rapid endpoint");
    near(phases[1].to_z,5,"Plunge retains its final depth");
    near(phases[0].rate,1500,"Approach uses the machine's Z rapid");
    near(phases[1].rate,120,"Material entry uses plunge feed");
    near(result.stats.rapid_minutes,9./1500,"Timing includes the rapid approach");
    near(result.stats.cutting_minutes,6./120,"Only the near-stock tail uses plunge feed");
    check(result.stats.rapid_minutes + result.stats.cutting_minutes < 15./120,
          "Splitting entries reduces predicted cycle time");
    auto direct = stock;
    direct.cut({0,0,20},{0,0,5},milling_cutter(tool));
    near(result.stock_after->volume(),direct.volume(),"Splitting preserves removed material");
    near(stock_at_motion(level,milling_cutter(tool),9).volume(),stock.volume(),
         "Playback removes no stock during the rapid approach");
    near(stock_at_motion(level,milling_cutter(tool),15).volume(),direct.volume(),
         "Playback agrees with the completed split plunge");
    auto replay = stock;
    double previous = 0;
    for (double distance : {4.,9.,10.,12.,15.}) {
      advance_stock_motion(replay,level,milling_cutter(tool),previous,distance);
      near(replay.volume(),stock_at_motion(level,milling_cutter(tool),distance).volume(),
           "Forward and checkpoint playback agree across the speed transition");
      previous = distance;
    }
    JobPlan posted_plan;
    posted_plan.rapid_plane = 20;
    posted_plan.operations = {result};
    const auto post = post_job(posted_plan,job,{{tool}},{{machine}});
    check(post.ok(),post.error);
    std::istringstream lines(post.text);
    std::string line, mode;
    bool rapid_seen = false, plunge_seen = false;
    double feed = 0;
    while (std::getline(lines,line)) {
      if (line.empty() || line.front() == ';' || line.front() == '(') continue;
      std::istringstream words(line);
      std::string word;
      double z = std::numeric_limits<double>::quiet_NaN();
      while (words >> word) {
        if (word == "G0" || word == "G1") mode = word;
        if (word.front() == 'Z') z = std::stod(word.substr(1));
        if (word.front() == 'F') feed = std::stod(word.substr(1));
      }
      if (mode == "G0" && z == 11) rapid_seen = true;
      if (mode == "G1" && z == 5 && feed == 120 && rapid_seen) plunge_seen = true;
    }
    check(rapid_seen && plunge_seen,
          "G-code commands the same rapid-to-plunge transition as simulation");

    auto cleared = stock;
    const wave::MillingCutter wide{wave::CutterShape::Flat,2,25};
    cleared.cut({0,0,20},{0,0,2},wide);
    auto deeper = entry;
    deeper.to_z = 1; deeper.length = 19;
    const auto inherited = run(cleared,{deeper},tool,job,op);
    near(inherited.levels[0].feeds.motion[0].to_z,3,
         "Prior stock removal lowers the approach below the original stock top");
    cleared.cut({0,0,20},{0,0,0},wide);
    const auto empty = run(cleared,{entry},tool,job,op);
    check(empty.levels[0].feeds.motion.size() == 1 &&
          empty.levels[0].feeds.motion[0].rapid,
          "An entry through a completely cleared opening remains rapid throughout");

    auto near_stock = motion({0,0,10.5},{0,0,5},wave::MoveKind::Entry);
    const auto already_near = run(stock,{near_stock},tool,job,op);
    check(already_near.levels[0].feeds.motion.size() == 1 &&
          !already_near.levels[0].feeds.motion[0].rapid,
          "An entry that already starts near stock stays at plunge feed");
    auto offered = entry;
    offered.rapid = true; offered.kind = wave::MoveKind::Retract;
    auto rough = op; rough.mode = OperationMode::Roughing;
    const auto guarded = run(stock,{offered},tool,job,rough);
    check(guarded.stats.unsafe_plunges == 1 &&
          guarded.levels[0].feeds.motion.size() == 2 &&
          !guarded.levels[0].feeds.motion.back().rapid,
          "An obstructed offered rapid keeps its diagnostic and slows before stock");

    // Clear a wider strip using this operation's own cuts, then plunge again.
    std::vector<MotionPhase> repeated{
        motion({-3,-.2,20},{-3,-.2,5},wave::MoveKind::Entry),
        motion({-3,-.2,5},{3,-.2,5},wave::MoveKind::Cut),
        motion({3,-.2,5},{3,.2,5},wave::MoveKind::Cut),
        motion({3,.2,5},{-3,.2,5},wave::MoveKind::Cut),
        motion({-3,.2,5},{-3,.2,20},wave::MoveKind::Retract,true),
        motion({-3,.2,20},{0,0,20},wave::MoveKind::Retract,true),
        motion({0,0,20},{0,0,4},wave::MoveKind::Entry)};
    const auto recut = run(stock,repeated,tool,job,op);
    const auto& recut_phases = recut.levels[0].feeds.motion;
    near(recut_phases.back().from_z,6,
         "Each approach uses stock after earlier cuts in the same pass");
    check(recut.levels[0].stock_checkpoints.size() <= 10,
          "Splitting entries keeps playback history bounded");

    // Material near the edge of the cutter must be seen even if its centre is
    // over an empty column. Check ball geometry against an independent formula.
    wave::Heightmap edge_stock(-2,-2,4,4,.1);
    edge_stock.set(29,20,0,8);
    for (auto shape : {ToolType::FlatEndMill,ToolType::BallEndMill}) {
      auto cutter = tool; cutter.type = shape;
      const auto sampled = milling_cutter(cutter);
      const double diagonal = edge_stock.pitch()*std::sqrt(.5);
      const double rho2 = std::pow(edge_stock.x(29),2)+std::pow(edge_stock.y(20),2);
      const double contact = shape == ToolType::FlatEndMill ? 8
          : 8 - sampled.radius + std::sqrt(std::pow(sampled.radius+diagonal,2)-rho2);
      near(edge_stock.plunge_contact_z(0,0,sampled),contact,
           "First contact accounts for the entire flat or ball cutter footprint");
      const auto edge_plan = run(edge_stock,{entry},cutter,job,op);
      near(edge_plan.levels[0].feeds.motion[0].to_z,contact+1,
           "A side contact sets the switch height even with no stock under the centre");
      check(edge_stock.clear({0,0,20},{0,0,contact+.001},sampled,0),
            "The rapid approach stays above conservative first contact");
      check(!edge_stock.clear({0,0,20},{0,0,contact-.001},sampled,0),
            "Descending past first contact is not clear");
    }
    const auto volume = edge_stock.volume();
    check(!std::isfinite(edge_stock.plunge_contact_z(20,20,milling_cutter(tool))),
          "No stock under the footprint has no contact height");
    near(edge_stock.volume(),volume,"First-contact queries do not modify stock");

    const auto ramp = motion({-1,0,20},{1,0,5},wave::MoveKind::Entry);
    const auto ramped = run(stock,{ramp},tool,job,op);
    check(ramped.levels[0].feeds.motion.size() == 1 &&
          !ramped.levels[0].feeds.motion[0].rapid,
          "Ramped entries retain their programmed path and plunge feed");
    auto helix = motion({1,0,20},{1,0,5},wave::MoveKind::Entry);
    helix.arc = true; helix.arc_center = {0,0};
    helix.length = std::hypot(2*std::acos(-1.),15);
    const auto helical = run(stock,{helix},tool,job,op);
    check(helical.levels[0].feeds.motion.size() == 1 &&
          helical.levels[0].feeds.motion[0].arc && !helical.levels[0].feeds.motion[0].rapid,
          "A full-circle helical entry is not mistaken for a vertical plunge");

    auto vertical_cut = entry;
    vertical_cut.kind = wave::MoveKind::Cut;
    const auto wall_cut = run(stock,{vertical_cut},tool,job,op);
    check(wall_cut.levels[0].feeds.motion.size() == 1 &&
          !wall_cut.levels[0].feeds.motion[0].rapid &&
          wall_cut.levels[0].feeds.motion[0].kind == wave::MoveKind::Cut,
          "Vertical surface cuts retain their cutting motion rather than becoming entries");

    auto custom = job; custom.plunge_clearance_height = 2.5;
    const auto higher = run(stock,{entry},tool,custom,op);
    near(higher.levels[0].feeds.motion[0].to_z,12.5,"The approach clearance is configurable");
    custom.plunge_clearance_height = 0;
    const auto zero = run(stock,{entry},tool,custom,op);
    check(zero.levels[0].feeds.motion[0].to_z > 10,
          "Zero clearance still leaves a coordinate-rounding margin before contact");
    custom.plunge_clearance_height = 2.5;
    near(job_from_json_text(to_json_text(custom)).plunge_clearance_height,2.5,
         "Plunge clearance survives saving and loading");
    near(job_from_json_text("{}").plunge_clearance_height,1,
         "Older jobs default to a one-millimetre plunge clearance");
    for (double invalid : {-1.,std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
      auto bad_job = job; bad_job.plunge_clearance_height = invalid;
      OperationPlan bad;
      simulate_operation_xyz(bad,{},bad_job,op,tool,machine,feeds,{});
      check(!bad.ok() && bad.error.find("Plunge clearance") != std::string::npos,
            "Invalid plunge clearances fail planning with a specific error");
    }
    std::cout << "Stock-aware plunge approach, timing, playback and posting passed.\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
