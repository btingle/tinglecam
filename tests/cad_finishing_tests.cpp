#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_NurbsConvert.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBndLib.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRep_Builder.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <algorithm>
#include <cmath>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "cam/cad.h"
#include "cam/config.h"
#include "cam/heightmap_simulation.h"
#include "cam/post.h"
#include "cam/surface_finishing.h"
#include "cam/surface_linking.h"

using namespace cam;
void check(bool value, const std::string& message) {
  if (!value) throw std::runtime_error(message);
}
void near(double a, double b, double e, const char* message) {
  check(std::abs(a - b) <= e, message);
}
void write(const TopoDS_Shape& shape, const std::filesystem::path& path) {
  STEPControl_Writer writer;
  check(writer.Transfer(shape, STEPControl_AsIs) == IFSelect_RetDone,
        "STEP transfer");
  check(writer.Write(path.c_str()) == IFSelect_RetDone, "STEP write");
}
// Independent full-model OCCT predicate, without the Bezier certificates or
// per-face broad phase used by CadModel. Exercise accelerated clearance against
// the original exact calculation, including the shaft and transformed NURBS.
bool reference_clear(const TopoDS_Shape& shape, wave::Vec3d a, wave::Vec3d b,
                     double r, double flute, double tolerance) {
  const gp_Pnt p(a.x, a.y, a.z + r), q(b.x, b.y, b.z + r);
  for (TopExp_Explorer e(shape, TopAbs_SOLID); e.More(); e.Next()) {
    BRepClass3d_SolidClassifier classifier(e.Current(), p, tolerance);
    if (classifier.State() == TopAbs_IN) return false;
  }
  const TopoDS_Shape ball =
      wave::distance(a, b) < 1e-8
          ? TopoDS_Shape(BRepBuilderAPI_MakeVertex(p).Vertex())
          : TopoDS_Shape(BRepBuilderAPI_MakeEdge(p, q).Edge());
  BRepExtrema_DistShapeShape ball_distance(ball, shape);
  check(ball_distance.IsDone(), "Reference ball distance computed");
  if (ball_distance.Value() < r - tolerance) return false;
  if (flute <= r + 1e-6) return true;
  TopoDS_Shape shaft;
  if (std::hypot(a.x - b.x, a.y - b.y) < 1e-8)
    shaft =
        BRepBuilderAPI_MakeEdge(gp_Pnt(a.x, a.y, std::min(a.z, b.z) + r),
                                gp_Pnt(a.x, a.y, std::max(a.z, b.z) + flute))
            .Edge();
  else {
    BRepBuilderAPI_MakePolygon wire;
    wire.Add(p);
    wire.Add(q);
    wire.Add(gp_Pnt(b.x, b.y, b.z + flute));
    wire.Add(gp_Pnt(a.x, a.y, a.z + flute));
    wire.Close();
    shaft = BRepBuilderAPI_MakeFace(wire.Wire()).Face();
  }
  BRepExtrema_DistShapeShape shaft_distance(shaft, shape);
  check(shaft_distance.IsDone(), "Reference shaft distance computed");
  return shaft_distance.Value() >= r - tolerance;
}
void continuous(const OperationPlan& plan) {
  std::optional<wave::Vec3d> previous;
  for (const auto& level : plan.levels)
    for (const auto& phase : level.feeds.motion) {
      if (previous)
        near(wave::distance(*previous, motion_point(phase, 0)), 0, 1e-8,
             "Finishing motion remains continuous between passes");
      previous = motion_point(phase, 1);
    }
}
int main(int argc, char** argv) {
  const auto directory =
      std::filesystem::temp_directory_path() / "tinglecam-cad-finishing-tests";
  try {
    std::filesystem::create_directories(directory);
    const auto file = directory / "block.step";
    write(BRepPrimAPI_MakeBox(8, 6, 4).Shape(), file);
    JobGeometry geometry;
    double load_progress = -1;
    int load_reports = 0;
    geometry.load_part(file, [&](double fraction, const std::string& stage) {
      check(fraction >= load_progress && fraction <= 1 && !stage.empty(),
            "Model loading progress is monotonic with named stages");
      load_progress = fraction;
      ++load_reports;
      return true;
    });
    check(load_progress == 1 && load_reports > 6,
          "STEP loading reports intermediate progress through completion");
    const auto original_model = geometry.part_cad();
    bool load_cancelled = false;
    try {
      geometry.load_part(file, [](double fraction, const std::string&) {
        return fraction < .5;
      });
    } catch (const std::runtime_error& error) {
      load_cancelled = std::string(error.what()) == "Cancelled";
    }
    check(load_cancelled && geometry.part_cad() == original_model,
          "Cancelling native face preparation preserves existing geometry");
    BoxStockMargins margins{};
    margins.minus_x = margins.plus_x = margins.minus_y = margins.plus_y = 0;
    margins.plus_z = 0.5;
    geometry.set_box_stock(true, margins);
    auto model = geometry.part_cad();
    check(model && model->solid(), "STEP retains native solid geometry");
    check(model->faces().size() == 6, "Box face identities retained");
    check(model->triangle_faces().size() == model->mesh().triangles.size(),
          "Every display triangle has a CAD face");
    Operation operation;
    operation.name = "Finish";
    operation.mode = OperationMode::SurfaceFinishing;
    operation.tool_id = "ball";
    operation.surface.model_fingerprint = model->fingerprint();
    operation.surface.stepover = 1;
    operation.surface.chord_tolerance = .01;
    for (const auto& face : model->faces()) {
      const auto p = model->evaluate(face.id, .5, .5);
      if (p.normal.z > .9) operation.surface.faces = {face.id};
    }
    check(operation.surface.faces.size() == 1, "Top face can be selected");
    const auto p = model->evaluate(operation.surface.faces.front(), .5, .5);
    auto tip = ball_tip(p, 1);
    near(tip.z, p.point.z, 1e-8,
         "Ball tip stays directly above a flat contact");
    SurfacePoint wall{{3, 4, 5}, {1, 0, 0}, true};
    tip = ball_tip(wall, 2);
    near(tip.x, 5, 1e-9, "Vertical contact offsets ball beside wall");
    near(tip.z, 3, 1e-9, "Vertical contact lowers tool tip by radius");
    SurfacePoint slope{{0, 0, 0}, {.6, 0, .8}, true};
    tip = ball_tip(slope, 2);
    near(tip.x, 1.2, 1e-9, "Inclined contact uses normal X");
    near(tip.z, -.4, 1e-9, "Inclined contact uses normal Z");
    Tool tool;
    tool.id = "ball";
    tool.name = "Ball";
    tool.type = ToolType::BallEndMill;
    tool.diameter = 2;
    tool.flute_length = 10;
    Machine machine;
    machine.id = "machine";
    machine.name = "Machine";
    machine.max_rpm = 24000;
    tool.cutting = {{"test", 12000, 400.0 / 24000, 100}};
    // 400 mm/min at full engagement, as the tool's data for "test" says.
    const CuttingFeeds feeds = resolve_cutting_feeds(tool, machine, "test", {});
    Job job;
    job.machine_id = "machine";
    job.material_id = "test";
    job.simulation_pitch = .1;
    job.operations = {operation};
    ToolLibrary tools{{tool}};
    MachineLibrary machines{{machine}};
    MaterialLibrary materials{{{"test", "Test material"}}};
    const auto coordinator = std::this_thread::get_id();
    std::size_t progress_calls = 0;
    auto planned = plan_job(
        geometry, job, tools, machines, materials,
        [&](PlanningProgress, const std::string&) {
          check(std::this_thread::get_id() == coordinator,
                "Parallel finishing progress stays on the coordinator thread");
          ++progress_calls;
          return true;
        });
    check(progress_calls > 0, "Finishing publishes planning progress");
    check(planned.ok(), planned.error);
    check(planned.operations.size() == 1, "One operation planned");
    const auto& finished = planned.operations.front();
    check(finished.ok(), finished.error);
    check(!finished.levels.empty() && finished.stock_after,
          "Surface finishing has XYZ paths and stock");
    check(finished.stats.removed_volume > 0,
          "Finishing removes the facing allowance");
    near(finished.stock_before->volume() - finished.stock_after->volume(),
         finished.stats.removed_volume, 1e-3,
         "XYZ simulation conserves stock volume");
    for (const auto& level : finished.levels) {
      check(level.explicit_motion && level.feeds.authoritative_motion,
            "Finishing carries posted XYZ motion");
      for (const auto& phase : level.feeds.motion)
        if (!phase.rapid && phase.kind == wave::MoveKind::Cut) {
          near(phase.to_z, 4, 1e-8, "Flat finishing preserves contact plane");
          check(model->cutter_sweep_clear(motion_point(phase, 0),
                                          motion_point(phase, 1), 1, 10, .005),
                "Every cutting sweep clears complete CAD part");
        }
      const wave::MillingCutter cutter{wave::CutterShape::Ball, 1, 10};
      const auto replay =
          stock_at_motion(level, cutter, feed_motion_length(level.feeds) + 1);
      near(replay.volume(), level.stock_checkpoints.back().stock->volume(),
           1e-5, "XYZ playback matches simulation");
    }
    for (const auto& level : finished.levels) {
      auto stock = *level.stock_checkpoints.front().stock;
      const wave::MillingCutter cutter{wave::CutterShape::Ball, 1, 10};
      const double length = feed_motion_length(level.feeds);
      double previous = 0;
      for (double f : {.1, .3, .7, 1.0}) {
        advance_stock_motion(stock, level, cutter, previous, length * f);
        near(stock.volume(),
             stock_at_motion(level, cutter, length * f).volume(), 1e-5,
             "Forward playback agrees with independent checkpoint seeks");
        previous = length * f;
      }
      check(level.stock_checkpoints.size() <= 10,
            "Playback history is bounded per finishing contour");
    }
    check(finished.metrics.surface_stay_down_links ==
                  finished.levels.size() - 1 &&
              finished.metrics.surface_retracts == 1,
          "Flat finishing uses short feed links and one final retract");
    continuous(finished);
    // Two sewn rectangular faces share a native edge, like consecutive spans
    // of the track. Their U/V directions agree at the common V boundary.
    BRepBuilderAPI_Sewing sewing;
    const gp_Pln plane(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    sewing.Add(BRepBuilderAPI_MakeFace(plane, 0, 4, 0, 4).Face());
    sewing.Add(BRepBuilderAPI_MakeFace(plane, 0, 4, 4, 8).Face());
    sewing.Perform();
    const auto seams_file = directory / "seams.step";
    write(sewing.SewedShape(), seams_file);
    JobGeometry seams;
    seams.load_part(seams_file);
    auto seam_op = operation;
    seam_op.surface.faces = {0, 1};
    seam_op.surface.model_fingerprint = seams.part_cad()->fingerprint();
    check(seams.part_cad()->adjacent(0, 1), "Sewn faces share a native edge");
    const auto joined =
        plan_surface_finishing(seams, seam_op, tool, machine, feeds, 15);
    check(joined.ok(), joined.error);
    check(joined.metrics.surface_seam_joins == 5 && joined.levels.size() == 5,
          "Matching face contours become five continuous lanes");
    check(joined.metrics.surface_retracts == 1 &&
              joined.metrics.surface_stay_down_links == 4,
          "Joined faces alternate cutting direction without full retracts");
    continuous(joined);
    for (const auto& level : joined.levels) {
      double low = 1e10, high = -1e10;
      for (const auto& phase : level.feeds.motion)
        if (phase.kind == wave::MoveKind::Cut) {
          low = std::min(
              {low, motion_point(phase, 0).y, motion_point(phase, 1).y});
          high = std::max(
              {high, motion_point(phase, 0).y, motion_point(phase, 1).y});
        }
      near(low, 0, 1e-8, "Joined lane starts at first face boundary");
      near(high, 8, 1e-8, "Joined lane continues to second face boundary");
    }
    auto allowance_op = seam_op;
    allowance_op.surface.allowance = .2;
    const auto with_allowance =
        plan_surface_finishing(seams, allowance_op, tool, machine, feeds, 15);
    check(with_allowance.ok(), with_allowance.error);
    check(with_allowance.metrics.surface_retracts == 1,
          "Allowance-preserving connectors still join flat passes");
    for (const auto& level : with_allowance.levels)
      for (const auto& phase : level.feeds.motion)
        check(phase.from_z >= .2 - 1e-8 && phase.to_z >= .2 - 1e-8,
              "Joined passes and feed links preserve the finishing allowance");
    seam_op.surface.bidirectional = false;
    const auto one_way =
        plan_surface_finishing(seams, seam_op, tool, machine, feeds, 15);
    check(one_way.ok(), one_way.error);
    check(one_way.metrics.surface_seam_joins == 5 &&
              one_way.metrics.surface_retracts == 5,
          "One-way finishing joins faces without reversing cutting direction");
    for (const auto& level : one_way.levels)
      for (const auto& phase : level.feeds.motion)
        if (phase.kind == wave::MoveKind::Cut)
          check(phase.to.y >= phase.from.y,
                "One-way contours retain their direction");
    continuous(one_way);
    auto cancelled = plan_surface_finishing(seams, seam_op, tool, machine, feeds, 15,
        [](PlanningProgress, const std::string& stage) {
          return stage != "Joining CAD finishing contours";
        });
    check(!cancelled.ok() && cancelled.error == "Cancelled",
          "Contour stitching remains cancellable");
    auto loaded = job_from_json_text(to_json_text(job));
    check(loaded.operations.front().surface.faces == operation.surface.faces &&
              loaded.operations.front().surface.model_fingerprint ==
                  model->fingerprint(),
          "Face selection and source identity round-trip");
    check(to_json_text(loaded) == to_json_text(job),
          "All finishing settings round-trip");
    const auto legacy_checked = job_from_json_text(
        R"({"operations":[{"mode":"surface_finishing","surface":{"ignore_collisions":false}}]})");
    const auto legacy_unchecked = job_from_json_text(
        R"({"operations":[{"mode":"surface_finishing","surface":{"ignore_collisions":true}}]})");
    check(to_json_text(legacy_checked) == to_json_text(legacy_unchecked) &&
              to_json_text(legacy_checked).find("ignore_collisions") ==
                  std::string::npos,
          "Legacy collision toggles do not affect geometric finishing paths");
    auto bad = operation;
    bad.surface.model_fingerprint = "other";
    check(!plan_surface_finishing(geometry, bad, tool, machine, feeds, 15).ok(),
          "Changed STEP requires explicit reselection");
    bad = operation;
    bad.surface.faces = {999};
    check(!plan_surface_finishing(geometry, bad, tool, machine, feeds, 15).ok(),
          "Invalid face rejected");
    auto flat = tool;
    flat.type = ToolType::FlatEndMill;
    check(!plan_surface_finishing(geometry, operation, flat, machine, feeds, 15).ok(),
          "CAD finishing rejects unsupported cutter");
    check(!model->cutter_sweep_clear({-2, 3, 1}, {10, 3, 1}, 1, 10, .001),
          "Continuous sweep detects crossing a solid between clear endpoints");
    check(!model->cutter_clear({4, 3, 1}, 1, 10, .001),
          "Buried ball centre is rejected");
    // A successful continuous sweep certifies its endpoint as outside, but a
    // failed sweep must never certify a buried endpoint. Changing cutter size
    // still requires a fresh ball/shaft boundary test at a cached centre.
    const auto cache_before = cad_query_statistics();
    const wave::Vec3d ca{-4, 2.25, 1.25}, cb{-3.5, 2.25, 1.25},
        cc{-3, 2.25, 1.25};
    check(model->cutter_sweep_clear(ca, cb, 1, 10, .001) &&
              model->cutter_sweep_clear(cb, cc, 1, 10, .001),
          "Connected clear sweeps preserve their outside certificate");
    const auto cache_after = cad_query_statistics();
    check(cache_after.containment_tests - cache_before.containment_tests == 1 &&
              cache_after.outside_reuses - cache_before.outside_reuses == 1,
          "Connected sweeps classify the starting centre only once");
    const wave::Vec3d buried{4, 2.25, 1.25};
    check(!model->cutter_sweep_clear(cc, buried, 1, 10, .001),
          "Cached outside starts still reject boundary crossings");
    const auto failed_before = cad_query_statistics();
    check(!model->cutter_clear(buried, 1, 10, .001),
          "Failed sweeps never certify their buried endpoint");
    check(cad_query_statistics().containment_tests ==
              failed_before.containment_tests + 1,
          "A disconnected buried query still performs containment");
    check(!model->cutter_clear({cc.x, cc.y, cc.z - 3}, 4, 10, .001),
          "A larger cutter cannot reuse a smaller cutter's boundary clearance");
    const auto shifted_file = directory / "shifted-block.step";
    write(BRepPrimAPI_MakeBox(gp_Pnt(-6, 0, 0), 8, 6, 4).Shape(), shifted_file);
    const auto shifted = CadModel::load(shifted_file);
    check(!shifted->cutter_clear(cc, 1, 10, .001) &&
              model->cutter_clear(cc, 1, 10, .001),
          "Outside certificates are scoped to the immutable model");
    check(!model->cutter_sweep_clear(ca, buried, 1, 10, 1) &&
              !model->cutter_clear(ca, 1, .5, .001),
          "Invalid cutter dimensions cannot bypass positive clearance checks");
    const auto reference_block = BRepPrimAPI_MakeBox(8, 6, 4).Shape();
    for (const double x : {-1.01, -.98}) {
      const wave::Vec3d a{x, 3, 1}, b{x + 1e-9, 3 + 1e-9, 3};
      check(model->cutter_sweep_clear(a, b, 1, 10, .005) ==
                reference_clear(reference_block, a, b, 1, 10, .005),
            "Near-vertical sweeps retain ball and shaft clearance checks");
    }
    // Tilted native plane verifies compensation from evaluated derivatives.
    const auto tilted = directory / "slope.step";
    write(BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0, 0, 5), gp_Dir(.6, 0, .8)),
                                  -2, 2, -2, 2)
              .Face(),
          tilted);
    auto sloped = CadModel::load(tilted);
    const auto sp = sloped->evaluate(0, .3, .7);
    const auto st = ball_tip(sp, 1);
    near(st.x - sp.point.x, .6, 1e-6,
         "STEP derivative normal gives sloped X compensation");
    near(st.z - sp.point.z, -.2, 1e-6,
         "STEP derivative normal gives sloped Z compensation");
    // Curved face paths must remain compensated in XYZ between native samples.
    const auto cylinder = directory / "cylinder.step";
    write(BRepPrimAPI_MakeCylinder(3, 4).Shape(), cylinder);
    JobGeometry curved;
    curved.load_part(cylinder);
    {
      // Exact surface bounds, so a box blank wraps a curved STEP part as
      // tightly as it wraps the same part as a mesh.
      const auto& b = curved.part().mesh().bounds;
      near(b.lower.x, -3, 1e-5, "Curved STEP bounds are exact, not padded");
      near(b.upper.y, 3, 1e-5, "Curved STEP bounds are exact, not padded");
      near(b.lower.z, 0, 1e-5, "Curved STEP bounds are exact, not padded");
      near(b.upper.z, 4, 1e-5, "Curved STEP bounds are exact, not padded");
    }
    curved.set_box_stock(true, BoxStockMargins{0, 0, 0, 0, 0, 0});
    auto op = operation;
    op.surface.model_fingerprint = curved.part_cad()->fingerprint();
    for (const auto& face : curved.part_cad()->faces())
      if (face.type == "Cylinder") op.surface.faces = {face.id};
    const auto flow = plan_surface_finishing(curved, op, tool, machine, feeds, 15);
    check(flow.ok(), flow.error);
    check(flow.levels.size() > 8,
          "Cylinder physical stepover creates multiple finishing contours");
    for (const auto& level : flow.levels)
      for (const auto& phase : level.feeds.motion)
        if (!phase.rapid && phase.kind == wave::MoveKind::Cut)
          near(std::hypot(wave::to_world(phase.to.x),
                          wave::to_world(phase.to.y)),
               4, .002, "Ball centres offset cylindrical wall by tool radius");
    // A 3 mm ball on a 1 mm convex fillet magnifies contact spacing fourfold.
    // Neighbouring contacts fit the stepover but their tool tips exceed the old
    // two-stepover link limit, and the straight tool-centre chord gouges the
    // face.
    const auto fillet_shape =
        BRepBuilderAPI_MakeFace(
            gp_Cylinder(
                gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, -1, 0), gp_Dir(1, 0, 0)), 1),
            0, std::acos(-1.0) / 2, 0, 8)
            .Face();
    const auto fillet_file = directory / "convex-fillet.step";
    write(fillet_shape, fillet_file);
    JobGeometry fillet_geometry;
    fillet_geometry.load_part(fillet_file);
    auto fillet_op = operation;
    fillet_op.surface.faces = {0};
    fillet_op.surface.stepover = .3;
    fillet_op.surface.model_fingerprint =
        fillet_geometry.part_cad()->fingerprint();
    auto large_ball = tool;
    large_ball.diameter = 6;
    large_ball.flute_length = 16;
    auto fillet = plan_surface_finishing(fillet_geometry, fillet_op, large_ball, machine, feeds, 15);
    check(fillet.ok(), fillet.error);
    check(fillet.levels.size() == 7 && fillet.metrics.surface_retracts == 1 &&
              fillet.metrics.surface_stay_down_links == 6,
          "Compensated convex passes use curved stay-down turnarounds");
    continuous(fillet);
    std::size_t turn_segments = 0;
    for (const auto& level : fillet.levels)
      for (const auto& phase : level.feeds.motion)
        if (phase.kind == wave::MoveKind::Step) {
          ++turn_segments;
          const auto p = motion_point(phase, .5);
          near(std::hypot(p.x, p.z + 3), 4, .011,
               "Turnarounds follow the compensated contour within chord "
               "tolerance");
        }
    check(turn_segments > 6,
          "Convex turnarounds follow the curve rather than straight chords");
    fillet_op.surface.allowance = .2;
    fillet = plan_surface_finishing(fillet_geometry, fillet_op, large_ball, machine, feeds, 15);
    check(fillet.ok(), fillet.error);
    check(fillet.metrics.surface_retracts == 1,
          "Convex turnarounds preserve the normal stock allowance");
    for (const auto& level : fillet.levels)
      for (const auto& phase : level.feeds.motion)
        if (phase.kind == wave::MoveKind::Step) {
          const auto p = motion_point(phase, .5);
          near(std::hypot(p.x, p.z + 3), 4.2, .011,
               "Turnarounds follow the requested normal stock allowance");
        }
    fillet_op.surface.allowance = 0;
    fillet_op.surface.bidirectional = false;
    const auto one_way_fillet = plan_surface_finishing(fillet_geometry, fillet_op, large_ball, machine, feeds, 15);
    check(one_way_fillet.ok(), one_way_fillet.error);
    check(one_way_fillet.metrics.surface_retracts == 7 &&
              one_way_fillet.metrics.surface_stay_down_links == 0,
          "Contact spacing does not reverse one-way fillet passes");
    // Cross the shared edge between two curved faces with endpoints set back
    // from that edge. Their compensated tips are too far apart for a straight
    // neighbour test, but the surface contacts are within two stepovers.
    const gp_Cylinder fillet_surface(
        gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, -1, 0), gp_Dir(1, 0, 0)), 1);
    BRepBuilderAPI_Sewing fillet_sewing;
    const double quarter = std::acos(-1.0) / 4;
    fillet_sewing.Add(
        BRepBuilderAPI_MakeFace(fillet_surface, 0, quarter, 0, 8).Face());
    fillet_sewing.Add(
        BRepBuilderAPI_MakeFace(fillet_surface, quarter, 2 * quarter, 0, 8)
            .Face());
    fillet_sewing.Perform();
    const auto split_fillet_file = directory / "split-fillet.step";
    write(fillet_sewing.SewedShape(), split_fillet_file);
    const auto split_fillet = CadModel::load(split_fillet_file);
    check(split_fillet->faces().size() == 2 && split_fillet->adjacent(0, 1),
          "Curved transition fixture retains a shared native edge");
    std::size_t left_face = 0, right_face = 1;
    if (split_fillet->evaluate(0, .5, .5).point.x <
        split_fillet->evaluate(1, .5, .5).point.x)
      std::swap(left_face, right_face);
    std::vector<detail::SurfaceRun> split_runs;
    auto split_run = [&](std::size_t face, double u, bool reverse) {
      detail::SurfaceRun run;
      run.first_face = run.last_face = face;
      run.first_uv = {u, reverse ? 1.0 : 0.0};
      run.last_uv = {u, reverse ? 0.0 : 1.0};
      const auto a = split_fillet->evaluate(face, run.first_uv[0],
                                            run.first_uv[1]),
                 b = split_fillet->evaluate(face, run.last_uv[0],
                                            run.last_uv[1]);
      run.first_contact = a.point;
      run.last_contact = b.point;
      for (auto p : {ball_tip(a, 3), ball_tip(b, 3)}) {
        p.x = wave::to_world(wave::to_integer(p.x));
        p.y = wave::to_world(wave::to_integer(p.y));
        run.points.push_back(p);
      }
      split_runs.push_back(std::move(run));
    };
    split_run(left_face, .75, false);
    split_run(right_face, .25, true);
    auto split_op = fillet_op;
    split_op.surface.bidirectional = true;
    OperationPlan split_path;
    detail::link_surface_runs(*split_fillet, split_op, large_ball, machine, feeds, 15,
                              split_runs, split_path, {});
    check(split_path.metrics.surface_retracts == 1 &&
              split_path.metrics.surface_stay_down_links == 1,
          "Neighbouring passes connect across the native shared edge");
    continuous(split_path);
    std::size_t split_turn_segments = 0;
    for (const auto& level : split_path.levels)
      for (const auto& phase : level.feeds.motion)
        if (phase.kind == wave::MoveKind::Step) {
          ++split_turn_segments;
          const auto p = motion_point(phase, .5);
          near(std::hypot(p.x, p.z + 3), 4, .011,
               "Shared-edge transitions follow both curved faces");
          near(p.y, -8, 1e-8,
               "Shared-edge transitions stay at the contour end");
        }
    check(split_turn_segments > 2,
          "Shared-edge transition is sampled as a curve");
    // Rational B-spline patches exercise convex-hull bounds, finite shaft
    // sweeps, point queries, vertical moves and imported face locations.
    const auto nurbs_file = directory / "nurbs-cylinder.step";
    gp_Trsf shift;
    shift.SetTranslation(gp_Vec(6, -2, 1));
    const auto nurbs_shape =
        BRepBuilderAPI_NurbsConvert(BRepPrimAPI_MakeCylinder(3, 4).Shape())
            .Shape()
            .Moved(TopLoc_Location(shift));
    write(nurbs_shape, nurbs_file);
    const auto nurbs = CadModel::load(nurbs_file);
    check(std::any_of(nurbs->faces().begin(), nurbs->faces().end(),
                      [](const CadFace& f) { return f.type == "B-spline"; }),
          "Collision reference fixture contains native rational B-splines");
    STEPControl_Reader reference;
    check(reference.ReadFile(nurbs_file.c_str()) == IFSelect_RetDone &&
              reference.TransferRoots() > 0,
          "Independent native reference imported");
    for (int i = 0; i < 72; ++i) {
      const double angle = i * 1.2345, radius = 2.5 + (i % 7) * .5;
      const wave::Vec3d a{6 + radius * std::cos(angle),
                          -2 + radius * std::sin(angle), -1. + (i % 6) * 1.3};
      const wave::Vec3d b =
          i % 3 == 0 ? a
                     : (i % 3 == 1 ? wave::Vec3d{a.x + .35, a.y - .7, a.z + .2}
                                   : wave::Vec3d{a.x, a.y, a.z + 2});
      const bool expected =
          reference_clear(reference.OneShape(), a, b, 1, 6, .005);
      check(nurbs->cutter_sweep_clear(a, b, 1, 6, .005) == expected,
            "Accelerated NURBS sweep agrees with full-model OCCT distance");
    }
    // A trimmed face must split cutting contours across its inner opening.
    BRepBuilderAPI_MakePolygon outer, inner;
    for (auto p : {gp_Pnt(-4, -4, 0), gp_Pnt(4, -4, 0), gp_Pnt(4, 4, 0),
                   gp_Pnt(-4, 4, 0)})
      outer.Add(p);
    for (auto p : {gp_Pnt(-1, -1, 0), gp_Pnt(-1, 1, 0), gp_Pnt(1, 1, 0),
                   gp_Pnt(1, -1, 0)})
      inner.Add(p);
    outer.Close();
    inner.Close();
    BRepBuilderAPI_MakeFace trimmed(gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)),
                                    outer.Wire());
    trimmed.Add(inner.Wire());
    const auto trimfile = directory / "trimmed.step";
    write(trimmed.Face(), trimfile);
    JobGeometry trimmed_geometry;
    trimmed_geometry.load_part(trimfile);
    op = operation;
    op.surface.faces = {0};
    op.surface.model_fingerprint = trimmed_geometry.part_cad()->fingerprint();
    const auto trimmed_plan =
        plan_surface_finishing(trimmed_geometry, op, tool, machine, feeds, 15);
    check(trimmed_plan.ok(), trimmed_plan.error);
    bool near_hole = false;
    for (const auto& level : trimmed_plan.levels)
      for (const auto& phase : level.feeds.motion)
        if (!phase.rapid && phase.kind == wave::MoveKind::Cut) {
          const auto p = motion_point(phase, .5);
          check(!(std::abs(p.x) < .999 && std::abs(p.y) < .999),
                "Trimmed opening splits the selected-face path");
          near_hole |= std::abs(p.x) < 1.01 && std::abs(p.y) < 1.01;
        }
    check(near_hole, "Trim-boundary samples are retained");
    // A separate unselected solid is still an obstruction to a selected face.
    TopoDS_Compound assembly;
    BRep_Builder builder;
    builder.MakeCompound(assembly);
    builder.Add(assembly, trimmed.Face());
    builder.Add(assembly,
                BRepPrimAPI_MakeBox(gp_Pnt(-.25, -3, 0), .5, 1, 3).Shape());
    const auto assembly_file = directory / "assembly.step";
    write(assembly, assembly_file);
    JobGeometry assembly_geometry;
    assembly_geometry.load_part(assembly_file);
    op.surface.model_fingerprint = assembly_geometry.part_cad()->fingerprint();
    const auto assembly_plan =
        plan_surface_finishing(assembly_geometry, op, tool, machine, feeds, 15);
    check(assembly_plan.ok(), assembly_plan.error);
    continuous(assembly_plan);
    auto cut_count = [](const OperationPlan& plan) {
      std::size_t count = 0;
      for (const auto& level : plan.levels)
        for (const auto& phase : level.feeds.motion)
          count += phase.kind == wave::MoveKind::Cut;
      return count;
    };
    check(cut_count(assembly_plan) == cut_count(trimmed_plan) &&
              assembly_plan.levels.size() == trimmed_plan.levels.size(),
          "Unselected obstacles do not split or remove finishing contours");
    builder.Add(
        assembly,
        BRepBuilderAPI_MakeEdge(gp_Pnt(-2, -2, 3), gp_Pnt(-2, 2, 3)).Edge());
    builder.Add(assembly,
                BRepBuilderAPI_MakeVertex(gp_Pnt(10, 10, 2)).Vertex());
    const auto loose_file = directory / "loose-obstacles.step";
    write(assembly, loose_file);
    const auto loose = CadModel::load(loose_file);
    Bnd_Box reference_bounds;
    BRepBndLib::AddOptimal(assembly, reference_bounds, false, false);
    double x0, y0, z0, x1, y1, z1;
    reference_bounds.Get(x0, y0, z0, x1, y1, z1);
    const auto& loose_bounds = loose->mesh().bounds;
    near(loose_bounds.lower.x, x0, 1e-5, "Reused bounds include loose edges");
    near(loose_bounds.lower.y, y0, 1e-5, "Reused bounds preserve model Y minimum");
    near(loose_bounds.lower.z, z0, 1e-5, "Reused bounds preserve model Z minimum");
    near(loose_bounds.upper.x, x1, 1e-5, "Reused bounds include loose vertices");
    near(loose_bounds.upper.y, y1, 1e-5, "Reused bounds preserve model Y maximum");
    near(loose_bounds.upper.z, z1, 1e-5, "Reused bounds preserve model Z maximum");
    check(!loose->cutter_clear({-2, 0, 0}, 1, 6, .005),
          "Free native edges still obstruct the cutter shaft");
    check(!loose->cutter_clear({10, 10, 0}, 1, 6, .005),
          "Free native vertices still obstruct the cutter shaft");
    // An obstacle over the end of a selected face must not turn a geometric
    // finishing path into collision-driven entries, splits or retraces.
    TopoDS_Compound overhead;
    builder.MakeCompound(overhead);
    builder.Add(overhead,
                BRepBuilderAPI_MakeFace(
                    gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 0, 4, 0, 8)
                    .Face());
    builder.Add(overhead,
                BRepPrimAPI_MakeBox(gp_Pnt(-1, 5, 3.5), 6, 4, 1).Shape());
    const auto overhead_file = directory / "overhead-obstruction.step";
    write(overhead, overhead_file);
    JobGeometry overhead_geometry;
    overhead_geometry.load_part(overhead_file);
    auto overhead_op = operation;
    overhead_op.surface.faces = {0};
    overhead_op.surface.model_fingerprint =
        overhead_geometry.part_cad()->fingerprint();
    const auto queries_before_overhead = cad_query_statistics();
    const auto under = plan_surface_finishing(overhead_geometry, overhead_op, tool, machine, feeds, 15);
    check(under.ok(), under.error);
    check(under.levels.size() == 5 && under.metrics.surface_retracts == 1 &&
              under.metrics.surface_stay_down_links == 4,
          "Obstacles do not interrupt the selected surface path");
    check(cad_query_statistics().containment_tests ==
                  queries_before_overhead.containment_tests &&
              cad_query_statistics().native_distance_tests ==
                  queries_before_overhead.native_distance_tests,
          "Surface links perform no CAD collision queries");
    continuous(under);
    overhead_op.surface.bidirectional = false;
    const auto one_way_under = plan_surface_finishing(overhead_geometry, overhead_op, tool, machine, feeds, 15);
    check(one_way_under.ok(), one_way_under.error);
    for (const auto& level : one_way_under.levels)
      for (const auto& phase : level.feeds.motion)
        if (phase.kind == wave::MoveKind::Cut)
          check(phase.to.y >= phase.from.y,
                "Geometric finishing retains one-way cutting direction");
    const auto sphere_file = directory / "sphere.step";
    write(BRepPrimAPI_MakeSphere(3).Shape(), sphere_file);
    JobGeometry sphere_geometry;
    sphere_geometry.load_part(sphere_file);
    op = operation;
    op.surface.model_fingerprint = sphere_geometry.part_cad()->fingerprint();
    for (const auto& face : sphere_geometry.part_cad()->faces())
      if (face.type == "Sphere") op.surface.faces = {face.id};
    const auto sphere_plan =
        plan_surface_finishing(sphere_geometry, op, tool, machine, feeds, 15);
    check(sphere_plan.ok(), sphere_plan.error);
    check(!sphere_plan.levels.empty(),
          "A sphere remains finishable at its parameter singularities");
    check(argc == 3, "Pass the vertical wall and selected track-face fixtures");
    JobGeometry track;
    track.load_part(argv[2]);
    auto track_op = operation;
    track_op.surface.faces.clear();
    track_op.surface.stepover = .3;
    track_op.surface.model_fingerprint = track.part_cad()->fingerprint();
    for (const auto& face : track.part_cad()->faces())
      track_op.surface.faces.push_back(face.id);
    const auto track_plan =
        plan_surface_finishing(track, track_op, large_ball, machine, feeds, 25);
    check(track_plan.ok(), track_plan.error);
    check(track.part_cad()->faces().size() == 14 &&
              track_plan.levels.size() > 100 &&
              track_plan.metrics.surface_retracts == 1 &&
              track_plan.metrics.surface_stay_down_links + 1 ==
                  track_plan.levels.size(),
          "The step_test surfaces form one continuous finishing path");
    continuous(track_plan);
    std::size_t track_entries = 0, track_retracts = 0;
    for (const auto& level : track_plan.levels)
      for (const auto& phase : level.feeds.motion) {
        track_entries += phase.kind == wave::MoveKind::Entry;
        track_retracts += phase.kind == wave::MoveKind::Retract;
      }
    check(track_entries == 1 && track_retracts == 1 &&
              track_plan.levels.back().feeds.motion.back().kind ==
                  wave::MoveKind::Retract,
          "The track has one initial entry, no intermediate lifts, and one "
          "final exit");
    // This extracted track wall stalled in native extrema searches when a
    // tall shaft quad ran tangent to the almost vertical B-spline surface.
    // Keep the original knots/poles: an ideal analytic cylinder misses the bug.
    check(argc == 3, "Pass the vertical wall and selected track-face fixtures");
    JobGeometry vertical_track;
    vertical_track.load_part(argv[1]);
    auto vertical_op = operation;
    vertical_op.surface.faces = {0};
    vertical_op.surface.stepover = .3;
    vertical_op.surface.model_fingerprint =
        vertical_track.part_cad()->fingerprint();
    auto vertical_tool = tool;
    vertical_tool.diameter = 6;
    vertical_tool.flute_length = 16;
    check(vertical_track.part_cad()->faces().size() == 1 &&
              vertical_track.part_cad()->faces()[0].type == "B-spline",
          "Vertical regression retains the original native B-spline wall");
    const auto vertical = plan_surface_finishing(vertical_track, vertical_op, vertical_tool, machine, feeds, 25);
    check(vertical.ok(), vertical.error);
    const auto& wall_model = *vertical_track.part_cad();
    const auto wall_intervals = std::size_t(std::ceil(
        wave::distance(wall_model.evaluate(0,0,.5).point, wall_model.evaluate(0,1,.5).point) /
            vertical_op.surface.stepover - 1e-6));
    check(vertical.levels.size() == wall_intervals + 1 && vertical.warnings.empty(),
          "All vertical-wall contours remain accessible without skipped cuts");
    check(vertical.metrics.surface_stay_down_links == wall_intervals &&
              vertical.metrics.surface_retracts == 1,
          "Vertical-wall finishing retains short feed links");
    continuous(vertical);
    for (double u : {.1, .5, .9}) {
      const auto contact = vertical_track.part_cad()->evaluate(0, u, .5);
      near(contact.normal.z, 0, 1e-9,
           "Regression wall has a horizontal normal");
      const auto touching = ball_tip(contact, 3);
      const wave::Vec3d gouge{touching.x - .25 * contact.normal.x,
                              touching.y - .25 * contact.normal.y, touching.z};
      check(
          !vertical_track.part_cad()->cutter_clear(gouge, 3, 16, .005),
          "Faster tangent clearance still rejects a ball penetrating the wall");
    }
    auto posted = post_job(planned, job, tools, machines);
    check(posted.ok() && posted.cutting_moves > 0,
          "Native finishing posts the validated XYZ timeline");
    check(posted.text.find("G1") != std::string::npos,
          "Finishing emits linear XYZ cuts");
    std::filesystem::remove_all(directory);
    std::cout << "Native STEP, face selection, normal compensation, finishing "
                 "and XYZ stock checks passed.\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
