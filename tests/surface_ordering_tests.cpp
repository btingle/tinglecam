#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRep_Builder.hxx>
#include <STEPControl_Writer.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Pln.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <Geom_BezierSurface.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <set>
#include <stdexcept>

#include "gui/gui_internal.h"
#include "cam/surface_linking.h"

using namespace cam;
void require(bool value, const std::string& message) {
  if (!value) throw std::runtime_error(message);
}
void write_shape(const TopoDS_Shape& shape, const std::filesystem::path& file) {
  STEPControl_Writer writer;
  require(writer.Transfer(shape, STEPControl_AsIs) == IFSelect_RetDone, "STEP transfer");
  require(writer.Write(file.c_str()) == IFSelect_RetDone, "STEP write");
}
wave::Vec3d centre(const SurfaceContour& contour) {
  const auto& points = contour.spans.front().contacts;
  return wave::interpolate(points.front(), points.back(), .5);
}
void matches_sequence(const OperationPlan& plan, const SurfaceContourPreview& preview,
                       const SurfaceContourOrder& order) {
  require(plan.ok(), plan.error);
  require(plan.levels.size() == order.sequence.size(), "Each contour executes exactly once");
  std::optional<wave::Vec3d> previous;
  for (std::size_t i = 0; i < plan.levels.size(); ++i) {
    const auto& contour = preview.contours[order.sequence[i]];
    std::vector<const MotionPhase*> cuts;
    for (const auto& phase : plan.levels[i].feeds.motion) {
      if (previous) require(wave::distance(*previous, motion_point(phase,0)) < 1e-8,
                            "Manual finishing has continuous entries, links and retracts");
      previous = motion_point(phase,1);
      if (phase.kind == wave::MoveKind::Cut) cuts.push_back(&phase);
    }
    require(!cuts.empty(), "Every selected contour has cutting motion");
    const auto a = motion_point(*cuts.front(),0), b = motion_point(*cuts.back(),1);
    const bool forward = wave::distance(a,contour.points.front()) < 1e-8 &&
                         wave::distance(b,contour.points.back()) < 1e-8;
    const bool reverse = wave::distance(b,contour.points.front()) < 1e-8 &&
                         wave::distance(a,contour.points.back()) < 1e-8;
    require(forward || reverse, "Stay-down linking must preserve the displayed execution sequence");
  }
}
int main(int argc, char** argv) {
  const auto directory = std::filesystem::temp_directory_path()/"tinglecam-surface-ordering-tests";
  try {
    std::filesystem::create_directories(directory);
    // The user's cross-section, extruded along Y. U follows the cross-section
    // and V is the finishing direction on all five sewn patches.
    BRepBuilderAPI_Sewing sewing;
    auto patch = [&](gp_Pnt start, gp_Dir across, gp_Dir normal, double width) {
      sewing.Add(BRepBuilderAPI_MakeFace(gp_Pln(gp_Ax3(start,normal,across)),
                                         0,width,0,6).Face());
    };
    patch({-4,0,4},{1,0,0},{0,0,1},4);
    patch({0,0,4},{0,0,-1},{1,0,0},4);
    patch({0,0,0},{1,0,0},{0,0,1},8);
    patch({8,0,0},{0,0,1},{-1,0,0},4);
    patch({8,0,4},{1,0,0},{0,0,1},4);
    sewing.Perform();
    const auto file = directory/"u-channel.step";
    write_shape(sewing.SewedShape(),file);
    JobGeometry geometry;
    geometry.load_part(file);
    Tool tool;
    tool.id = "ball"; tool.name = "Ball"; tool.type = ToolType::BallEndMill;
    tool.diameter = 2; tool.flute_length = 10;
    Machine machine;
    machine.id = "test"; machine.name = "Test"; machine.max_rpm = 24000;
    tool.cutting = {{"test",12000,400.0/24000,100}};
    auto feeds = resolve_cutting_feeds(tool,machine,"test",{});
    Operation op;
    op.name = "Manual finish"; op.mode = OperationMode::SurfaceFinishing;
    op.tool_id = tool.id; op.surface.stepover = 1;
    op.surface.manual_order = true;
    op.surface.model_fingerprint = geometry.part_cad()->fingerprint();
    for (const auto& face : geometry.part_cad()->faces()) op.surface.faces.push_back(face.id);
    auto preview = prepare_surface_contours(*geometry.part_cad(),op,tool);
    require(preview.error.empty(),preview.error);
    require(preview.contours.size() == 29, "Five U-channel patches retain every finishing contour");
    std::vector<std::size_t> roots;
    std::size_t ends = 0;
    for (std::size_t i = 0; i < preview.contours.size(); ++i) {
      const auto p = centre(preview.contours[i]);
      if (std::abs(p.z-4) < 1e-8 && (std::abs(p.x+1) < 1e-8 || std::abs(p.x-9) < 1e-8))
        roots.push_back(i);
      ends += preview.neighbours[i].size() == 1;
      require(preview.neighbours[i].size() <= 2, "U-channel adjacency follows the profile without skipping lanes");
      for (const auto& span : preview.contours[i].spans)
        for (const auto& contact : span.contacts)
          require(std::abs(contact.y) <= 6+1e-8 && contact.z >= -1e-8,
                  "Preview contacts stay on the uncompensated surface");
    }
    require(roots.size() == 2 && ends == 2,"Two upper roots and two outer terminations");
    auto order = propagate_surface_order(preview,roots);
    require(order.error.empty(),order.error);
    require(order.branches == 4,"Two interior roots launch exactly four branches");
    require(order.sequence.size() == preview.contours.size() &&
            std::set<std::size_t>(order.sequence.begin(),order.sequence.end()).size() == preview.contours.size(),
            "Colliding propagation fronts never cut a contour twice");
    std::vector<std::size_t> position(order.sequence.size());
    for (std::size_t i = 0; i < order.sequence.size(); ++i) position[order.sequence[i]] = i;
    std::size_t region_begin = 0;
    for (auto root : std::set<std::size_t>(roots.begin(),roots.end())) {
      const auto region_size = std::count(order.root.begin(),order.root.end(),root);
      require(order.sequence.at(region_begin) == root,
              "Each root executes immediately before its own branches");
      for (std::size_t i = region_begin; i < region_begin + region_size; ++i)
        require(order.root[order.sequence[i]] == root,
                "Finish a root's whole region before travelling to another root");
      region_begin += region_size;
    }
    std::size_t meetings = 0;
    for (std::size_t i = 0; i < order.depth.size(); ++i) {
      if (order.parent[i] != SurfaceContourOrder::none)
        require(position[order.parent[i]] < position[i],"Every branch executes outward from its root");
      for (auto j : preview.neighbours[i])
        if (j > i && order.root[i] != order.root[j]) {
          ++meetings;
          require(std::abs(centre(preview.contours[i]).z) < 1e-8 &&
                  std::abs(centre(preview.contours[j]).z) < 1e-8,
                  "The two downward fronts meet on the horizontal floor");
        }
    }
    require(meetings == 1,"Two inward fronts have one meeting boundary");
    auto reversed_roots = roots;
    std::reverse(reversed_roots.begin(),reversed_roots.end());
    require(propagate_surface_order(preview,reversed_roots).sequence == order.sequence,
            "Root click order cannot change ownership or execution");
    op.surface.order_roots = roots;
    op.surface.order_signature = preview.signature;
    matches_sequence(plan_surface_finishing(geometry,op,tool,machine,feeds,15),preview,order);
    auto changed = op;
    changed.surface.stepover = .5;
    auto stale = plan_surface_finishing(geometry,changed,tool,machine,feeds,15);
    require(!stale.ok() && stale.error.find("changed") != std::string::npos,
            "Changed contours reject stale saved root indices");
    Job job;
    job.operations = {op};
    const auto loaded = job_from_json_text(to_json_text(job));
    require(loaded.operations[0].surface.order_roots == roots &&
            loaded.operations[0].surface.order_signature == preview.signature &&
            loaded.operations[0].surface.manual_order,
            "Manual ordering and exact root identity survive saving and loading");
    require(!job_from_json_text("{\"operations\":[{\"mode\":\"surface_finishing\"}]}")
                 .operations[0].surface.manual_order,"Older jobs keep automatic ordering");
    changed = op;
    changed.surface.bidirectional = false;
    auto one_way_preview = prepare_surface_contours(*geometry.part_cad(),changed,tool);
    require(one_way_preview.error.empty(),one_way_preview.error);
    changed.surface.order_signature = one_way_preview.signature;
    auto one_way_order = propagate_surface_order(one_way_preview,roots);
    const auto one_way = plan_surface_finishing(geometry,changed,tool,machine,feeds,15);
    matches_sequence(one_way,one_way_preview,one_way_order);
    for (const auto& level : one_way.levels)
      for (const auto& phase : level.feeds.motion)
        if (phase.kind == wave::MoveKind::Cut)
          require(phase.to.y >= phase.from.y,"Manual order respects one-way cutting direction");
    auto allowance = op;
    allowance.surface.allowance = .4;
    auto allowance_preview = prepare_surface_contours(*geometry.part_cad(),allowance,tool);
    require(allowance_preview.error.empty(),allowance_preview.error);
    for (std::size_t i = 0; i < preview.contours.size(); ++i)
      require(wave::distance(centre(preview.contours[i]),centre(allowance_preview.contours[i])) < 1e-8,
              "Preview contacts exclude cutter and stock-allowance offsets");
    const auto cancelled = prepare_surface_contours(*geometry.part_cad(),op,tool,
        [](PlanningProgress,const std::string& stage) { return stage != "Finding finishing contour neighbours"; });
    require(cancelled.error == "Cancelled","Adjacency preparation can be cancelled");

    // End-to-end face pieces are one selectable contour, including in manual
    // mode; ordering must not split or rejoin them after root propagation.
    BRepBuilderAPI_Sewing seam_sewing;
    const gp_Pln seam_plane(gp_Pnt(0,0,0),gp_Dir(0,0,1));
    for (double y : {0.,4.})
      seam_sewing.Add(BRepBuilderAPI_MakeFace(seam_plane,0,4,y,y+4).Face());
    seam_sewing.Perform();
    const auto seam_file = directory/"joined-contours.step";
    write_shape(seam_sewing.SewedShape(),seam_file);
    JobGeometry seam_geometry;
    seam_geometry.load_part(seam_file);
    auto seam_op = op;
    seam_op.surface.faces = {0,1};
    seam_op.surface.model_fingerprint = seam_geometry.part_cad()->fingerprint();
    auto seam_preview = prepare_surface_contours(*seam_geometry.part_cad(),seam_op,tool);
    require(seam_preview.error.empty(),seam_preview.error);
    require(seam_preview.contours.size() == 5 &&
            std::all_of(seam_preview.contours.begin(),seam_preview.contours.end(),
                        [](const auto& contour) { return contour.spans.size() == 2; }),
            "Cross-face joined lanes are single selectable preview contours");
    seam_op.surface.order_signature = seam_preview.signature;
    seam_op.surface.order_roots = {2};
    const auto seam_order = propagate_surface_order(seam_preview,{2});
    require(seam_order.error.empty() && seam_order.branches == 2,
            "A root on a joined lane propagates to both neighbouring lanes");
    matches_sequence(plan_surface_finishing(seam_geometry,seam_op,tool,machine,feeds,15),
                       seam_preview,seam_order);

    // A taper needs twice as many physical passes as its cylindrical neighbour
    // even though their shared edge has exactly the same width. Every fine
    // pass must continue onto the formerly coarser patch.
    BRepBuilderAPI_Sewing taper_sewing;
    const gp_Ax3 taper_axes(gp_Pnt(0,0,0),gp_Dir(0,1,0),gp_Dir(1,0,0));
    taper_sewing.Add(BRepBuilderAPI_MakeFace(gp_Cone(taper_axes,-std::atan(.1),4),
        0,std::acos(-1.0)/2,0,std::sqrt(404.)).Face());
    taper_sewing.Add(BRepBuilderAPI_MakeFace(gp_Cylinder(
        gp_Ax3(gp_Pnt(0,20,0),gp_Dir(0,1,0),gp_Dir(1,0,0)),2),
        0,std::acos(-1.0)/2,0,4).Face());
    taper_sewing.Perform();
    const auto taper_file = directory/"different-pass-counts.step";
    write_shape(taper_sewing.SewedShape(),taper_file);
    JobGeometry taper_geometry;
    taper_geometry.load_part(taper_file);
    auto taper_op = seam_op;
    taper_op.surface.stepover = .3;
    taper_op.surface.model_fingerprint = taper_geometry.part_cad()->fingerprint();
    auto taper_preview = prepare_surface_contours(*taper_geometry.part_cad(),taper_op,tool);
    require(taper_preview.error.empty(),taper_preview.error);
    require(taper_preview.contours.size() == 22 &&
            std::all_of(taper_preview.contours.begin(),taper_preview.contours.end(),
                        [](const auto& contour) { return contour.spans.size() == 2; }),
            "Unequal native pass counts synchronize into complete joined lanes");
    taper_op.surface.order_roots = {0};
    taper_op.surface.order_signature = taper_preview.signature;
    auto taper_order = propagate_surface_order(taper_preview,{0});
    auto taper_plan = plan_surface_finishing(taper_geometry,taper_op,tool,machine,feeds,30);
    matches_sequence(taper_plan,taper_preview,taper_order);
    require(taper_plan.metrics.surface_retracts == 1,
            "Manual finishing no longer retracts at unequal-grid seams");
    taper_op.surface.bidirectional = false;
    const auto one_way_taper = prepare_surface_contours(*taper_geometry.part_cad(),taper_op,tool);
    std::array<std::size_t,2> one_way_counts{};
    for (const auto& contour : one_way_taper.contours)
      for (const auto& span : contour.spans) ++one_way_counts.at(span.face);
    // One-way passes may not join when native V directions oppose each other.
    // Matching grids must retain their travel direction.
    require(one_way_taper.error.empty() && one_way_counts == std::array<std::size_t,2>{22,22},
            "One-way finishing retains synchronized grids without reversing passes");
    require(taper_geometry.part_cad()->matching_contour_parameters(0,1,false,{0,.5,1},{0,.5,1}).empty(),
            "Seams parallel to the cuts do not synchronize independent pass grids");
    const auto cancelled_seam = prepare_surface_contours(*taper_geometry.part_cad(),taper_op,tool,
        [](PlanningProgress,const std::string& stage) {
          return stage != "Matching CAD finishing contours across seams";
        });
    require(cancelled_seam.error == "Cancelled","Seam grid synchronization remains cancellable");

    // Reverse U and stretch it quadratically on a flat neighbour. Matching
    // normalized U values would put the two endpoints at different X values.
    TColgp_Array2OfPnt poles(1,3,1,2);
    for (int u = 1; u <= 3; ++u)
      for (int v = 1; v <= 2; ++v)
        poles.SetValue(u,v,gp_Pnt(u == 1 ? 4 : u == 2 ? .2 : 0,4+4*(v-1),0));
    Handle(Geom_BezierSurface) stretched = new Geom_BezierSurface(poles);
    BRepBuilderAPI_Sewing remap_sewing;
    remap_sewing.Add(BRepBuilderAPI_MakeFace(seam_plane,0,4,0,4).Face());
    remap_sewing.Add(BRepBuilderAPI_MakeFace(stretched,1e-8).Face().Reversed());
    remap_sewing.Perform();
    const auto remap_file = directory/"reversed-nonlinear-seam.step";
    write_shape(remap_sewing.SewedShape(),remap_file);
    JobGeometry remap_geometry;
    remap_geometry.load_part(remap_file);
    auto remap_op = seam_op;
    remap_op.surface.model_fingerprint = remap_geometry.part_cad()->fingerprint();
    auto remap_preview = prepare_surface_contours(*remap_geometry.part_cad(),remap_op,tool);
    require(remap_preview.error.empty(),remap_preview.error);
    require(remap_preview.contours.size() == 5 &&
            std::all_of(remap_preview.contours.begin(),remap_preview.contours.end(),
                        [](const auto& contour) { return contour.spans.size() == 2; }),
            "A shared physical grid avoids extra passes on reversed nonlinear patches");
    for (const auto& contour : remap_preview.contours) {
      const auto& a = contour.spans[0].contacts;
      const auto& b = contour.spans[1].contacts;
      double distance = 1e10;
      for (auto p : {a.front(),a.back()})
        for (auto q : {b.front(),b.back()}) distance = std::min(distance,wave::distance(p,q));
      require(distance < 1e-6,"Remapped seam contacts coincide in physical space");
    }
    remap_op.surface.order_roots = {0};
    remap_op.surface.order_signature = remap_preview.signature;
    auto remap_plan = plan_surface_finishing(remap_geometry,remap_op,tool,machine,feeds,15);
    require(remap_plan.ok() && remap_plan.metrics.surface_retracts == 1,
            "Reparameterized shared faces keep a continuous manual finishing path");

    // Physical spacing must be even on a stretched parameterization by itself
    // too; merely merging independently generated face grids overcuts the
    // seam and merely interpolating UV gives unequal physical intervals.
    auto check_spacing = [&](const CadModel& model, Operation spacing_op,
                             std::size_t face, bool along_v, double expected) {
      spacing_op.surface.faces = {face};
      spacing_op.surface.along_v = along_v;
      PlanningMetrics metrics;
      const auto lanes = detail::surface_lanes(model, spacing_op, metrics, {}).at(face);
      for (std::size_t i = 1; i < lanes.size(); ++i) {
        const auto a = model.evaluate(face, along_v ? lanes[i-1] : .5,
                                      along_v ? .5 : lanes[i-1]);
        const auto b = model.evaluate(face, along_v ? lanes[i] : .5,
                                      along_v ? .5 : lanes[i]);
        require(std::abs(wave::distance(a.point,b.point) - expected) < 1e-5,
                "Contour intervals follow physical distance in either U/V direction");
      }
      return lanes.size();
    };
    require(check_spacing(*remap_geometry.part_cad(),remap_op,1,true,1) == 5,
            "Nonlinear standalone patches need only the physical pass count");
    require(check_spacing(*remap_geometry.part_cad(),remap_op,1,false,1) == 5,
            "Swapping flow direction retains exact physical spacing");

    const auto fractional_file = directory/"fractional-spacing.step";
    write_shape(BRepBuilderAPI_MakeFace(seam_plane,0,4.1,0,4).Face(),fractional_file);
    const auto fractional_model = CadModel::load(fractional_file);
    auto fractional_op = seam_op;
    fractional_op.surface.stepover = .3;
    require(check_spacing(*fractional_model,fractional_op,0,true,4.1/14) == 15,
            "A non-divisible face distributes the boundary remainder evenly");

    // A three-patch chain must use one grid all the way through, rather than
    // acquiring an extra interleaved grid at each successive seam.
    BRepBuilderAPI_Sewing chain_sewing;
    chain_sewing.Add(BRepBuilderAPI_MakeFace(seam_plane,0,4,0,4).Face());
    chain_sewing.Add(BRepBuilderAPI_MakeFace(stretched,1e-8).Face().Reversed());
    chain_sewing.Add(BRepBuilderAPI_MakeFace(seam_plane,0,4,8,12).Face());
    chain_sewing.Perform();
    const auto chain_file = directory/"shared-grid-chain.step";
    write_shape(chain_sewing.SewedShape(),chain_file);
    const auto chain_model = CadModel::load(chain_file);
    auto chain_op = remap_op;
    chain_op.surface.faces = {0,1,2};
    chain_op.surface.model_fingerprint = chain_model->fingerprint();
    const auto chain_preview = prepare_surface_contours(*chain_model,chain_op,tool);
    require(chain_preview.error.empty(),chain_preview.error);
    require(chain_preview.contours.size() == 5 &&
            std::all_of(chain_preview.contours.begin(),chain_preview.contours.end(),
                        [](const auto& contour) { return contour.spans.size() == 3; }),
            "A nonlinear three-patch chain retains five complete continuous contours");

    // Nearby but unsewn surfaces must remain separate propagation groups.
    TopoDS_Compound separate;
    BRep_Builder builder;
    builder.MakeCompound(separate);
    for (double z : {0.,.1})
      builder.Add(separate,BRepBuilderAPI_MakeFace(
          gp_Pln(gp_Pnt(0,0,z),gp_Dir(0,0,1)),0,4,0,6).Face());
    const auto separate_file = directory/"nearby-disconnected.step";
    write_shape(separate,separate_file);
    const auto disconnected = CadModel::load(separate_file);
    auto disconnected_op = op;
    disconnected_op.surface.faces = {0,1};
    disconnected_op.surface.model_fingerprint = disconnected->fingerprint();
    const auto disconnected_preview = prepare_surface_contours(*disconnected,disconnected_op,tool);
    require(disconnected_preview.error.empty(),disconnected_preview.error);
    auto incomplete = propagate_surface_order(disconnected_preview,{0});
    require(incomplete.unreachable == 5 && !incomplete.error.empty(),
            "Close disconnected faces require independent roots");
    require(propagate_surface_order(disconnected_preview,{0,5}).error.empty(),
            "A root in every group reaches every contour");
    require(!propagate_surface_order(preview,{}).error.empty() &&
            !propagate_surface_order(preview,{preview.contours.size()}).error.empty(),
            "Missing and invalid roots are rejected");

    // Exercise real ImGui mouse events against contact lines without a GPU.
    using namespace tinglecam_gui;
    ImGui::CreateContext();
    auto& io = ImGui::GetIO();
    io.DisplaySize = {1000,800}; io.DeltaTime = 1.f/60;
    io.IniFilename = nullptr;
    unsigned char* pixels; int width,height;
    io.Fonts->GetTexDataAsRGBA32(&pixels,&width,&height);
    {
      App app;
      app.geometry.load_part(file);
      // Compare the indexed picker against an independent projection oracle,
      // including rotated cameras, misses and overlapping native faces.
      const auto& pick_model=*app.geometry.part_cad();
      for (float yaw : {0.f,.4f,1.5f,3.f}) {
        const auto basis=orbit_basis(yaw,.8f);
        Projection projection{{500,400},{4,3,0},basis.right,basis.up,basis.toward,30};
        for (int y=0;y<10;++y) for(int x=0;x<10;++x) {
          ImVec2 mouse{float(350+x*31.17),float(250+y*29.83)};
          std::optional<std::size_t> expected;
          float best=-std::numeric_limits<float>::infinity();
          for (std::size_t i=0;i<pick_model.mesh().triangles.size();++i) {
            const auto& t=pick_model.mesh().triangles[i];
            const Vec3 a{t.a.x,t.a.y,t.a.z},b{t.b.x,t.b.y,t.b.z},c{t.c.x,t.c.y,t.c.z};
            const auto p=projection.point(a),q=projection.point(b),r=projection.point(c);
            const float det=(q.y-r.y)*(p.x-r.x)+(r.x-q.x)*(p.y-r.y);
            if(std::abs(det)<1e-8f) continue;
            const float u=((q.y-r.y)*(mouse.x-r.x)+(r.x-q.x)*(mouse.y-r.y))/det;
            const float v=((r.y-p.y)*(mouse.x-r.x)+(p.x-r.x)*(mouse.y-r.y))/det;
            if(u<0 || v<0 || u+v>1) continue;
            const float depth=u*projection.depth(a)+v*projection.depth(b)+(1-u-v)*projection.depth(c);
            if(depth>best) {best=depth;expected=pick_model.triangle_faces()[i];}
          }
          require(pick_cad_face(app,projection,mouse)==expected,
                  "Indexed face picking agrees with the projected native surface");
        }
      }
      app.tools.tools = {tool};
      app.job.operations = {op};
      app.job.operations[0].surface.order_roots.clear();
      app.surface_preview = preview;
      app.surface_preview_key = preview.signature;
      refresh_surface_contours(app,app.job.operations[0]);
      const auto p = centre(preview.contours[roots.front()]);
      const ImVec2 mouse{float(500+(p.x-4)*30),float(400-(p.y-3)*30)};
      auto frame = [&] {
        ImGui::NewFrame();
        ImGui::SetNextWindowPos({0,0}); ImGui::SetNextWindowSize({1000,800});
        ImGui::Begin("Surface preview test");
        OrbitScene scene;
        scene.top_left = {0,0}; scene.size = {1000,800}; scene.hovered = true;
        scene.list = ImGui::GetWindowDrawList();
        scene.projection.centre = {500,400}; scene.projection.model_centre = {4,3,0};
        scene.projection.right = {1,0,0}; scene.projection.up = {0,1,0};
        scene.projection.toward = {0,0,1}; scene.projection.scale = 30;
        draw_surface_contours(app,scene,app.job.operations[0]);
        ImGui::End(); ImGui::Render();
      };
      auto click = [&] {
        io.AddMousePosEvent(mouse.x,mouse.y); frame();
        io.AddMouseButtonEvent(0,true); frame();
        io.AddMouseButtonEvent(0,false); frame();
      };
      frame(); click();
      require(app.job.operations[0].surface.order_roots == std::vector<std::size_t>{roots.front()},
              "Clicking a visible preview contour selects its propagation root");
      click();
      require(app.job.operations[0].surface.order_roots.empty(),"Clicking a selected root toggles it off");
    }
    ImGui::DestroyContext();
    // Optional real-model smoke check, kept out of the fast CTest fixture.
    if (argc > 1) {
      JobGeometry actual_geometry;
      actual_geometry.load_part(argv[1]);
      const auto actual = actual_geometry.part_cad();
      auto actual_tool = tool;
      actual_tool.diameter = 6;
      actual_tool.flute_length = 16;
      auto actual_op = op;
      actual_op.surface.model_fingerprint = actual->fingerprint();
      actual_op.surface.faces.clear();
      // Inner channel, upper flats and their connecting fillets on jig_step/Ls.step.
      for (std::size_t face : {4,6,8,10,12,14,16,18,20})
        if (face < actual->faces().size()) actual_op.surface.faces.push_back(face);
      actual_op.surface.stepover = .5;
      auto actual_preview = prepare_surface_contours(*actual,actual_op,actual_tool);
      require(actual_preview.error.empty(),actual_preview.error);
      std::cout << "Real STEP: " << actual_preview.contours.size() << " contours\n";
      std::vector<std::size_t> actual_roots;
      for (std::size_t id = 0; id < actual_preview.contours.size(); ++id) {
        const auto& contour = actual_preview.contours[id];
        for (const auto& span : contour.spans)
          if ((span.face == 4 || span.face == 20) && std::abs(span.across-.5)<1e-8) {
            actual_roots.push_back(id); break;
          }
      }
      auto actual_order = propagate_surface_order(actual_preview,actual_roots);
      std::cout << "Real STEP: " << actual_roots.size() << " roots, " << actual_order.branches
                << " branches, " << actual_order.unreachable << " unseeded contours\n";
      require(actual_order.error.empty(),actual_order.error);
      require(actual_roots.size() == 2 && actual_order.branches == 4,
              "Real Ls STEP upper roots create four outward branches");
      actual_op.surface.order_roots = actual_roots;
      actual_op.surface.order_signature = actual_preview.signature;
      matches_sequence(plan_surface_finishing(actual_geometry,actual_op,actual_tool,
                                               machine,feeds,30),actual_preview,actual_order);
      // Optional review artifact for the actual application screenshot.
      if (argc > 2) {
        Job review;
        review.name = "STEP contour propagation";
        review.part_mesh = std::filesystem::absolute(argv[1]);
        review.box_stock = true;
        review.planning.simulation_pitch = .3;
        review.machine_id = "grbl-router";
        review.material_id = "hdpe";
        actual_op.tool_id = "6mm-ball-alu";
        review.operations = {actual_op};
        save_job(argv[2],review);
      }
    }
    std::filesystem::remove_all(directory);
    std::cout << "Surface propagation, execution, persistence and root picking passed.\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
