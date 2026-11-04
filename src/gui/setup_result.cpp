#include "gui_internal.h"
#include "cam/heightmap_simulation.h"
#include "wave/cpu_bvh.h"
namespace tinglecam_gui {
// ---------------------------------------------------------------------------
// What a dimension-driven operation will take out of the work: the prism
// between its top and its floor, over the shape it was given. It is the whole
// of what such an operation is -- there is no mesh to read it off -- so the
// plan view draws it beside the bodies rather than leaving the operator to
// hold four numbers in their head and imagine where they land.
struct CutVolume {
  Paths64 rings;
  double bottom = 0.0;
  double top = 0.0;
  bool selected = false;
  bool exclusion = false;
};

Paths64 area_outline(const cam::ClearingArea& area) {
  const std::int64_t left = wave::to_integer(area.center_x - area.width * 0.5);
  const std::int64_t right = wave::to_integer(area.center_x + area.width * 0.5);
  const std::int64_t low = wave::to_integer(area.center_y - area.height * 0.5);
  const std::int64_t high = wave::to_integer(area.center_y + area.height * 0.5);
  return {{{left, low}, {right, low}, {right, high}, {left, high}}};
}

Paths64 exclusion_outline(const cam::PocketExclusionArea& area) {
  const std::int64_t left = wave::to_integer(area.center_x - area.width * 0.5);
  const std::int64_t right = wave::to_integer(area.center_x + area.width * 0.5);
  const std::int64_t low = wave::to_integer(area.center_y - area.height * 0.5);
  const std::int64_t high = wave::to_integer(area.center_y + area.height * 0.5);
  return {{{left, low}, {right, low}, {right, high}, {left, high}}};
}

std::vector<CutVolume> cut_volumes(const App& app) {
  std::vector<CutVolume> volumes;
  for (std::size_t i = 0; i < app.job.operations.size(); ++i) {
    const cam::Operation& operation = app.job.operations[i];
    if(!app.job.setups.empty() && operation.setup_id!=app.job.setups.at(app.selected_setup).id) continue;
    if (!cam::is_parametric(operation.mode)) continue;
    const bool selected = static_cast<int>(i) == app.selected_operation;
    // Half-typed dimensions are normal while a field is being edited, and the
    // shape builders reject them. The settings panel and planning both say so
    // properly; a redraw has nothing useful to add and simply draws nothing.
    try {
      if (operation.mode == cam::OperationMode::Pocket) {
        if (operation.auto_identify_pockets) {
          double low = 0.0;
          double high = 0.0;
          if (app.geometry.work_range(low, high)) {
            for (const auto& exclusion : operation.pocket_auto.exclusions) {
              if (!exclusion.enabled || !(exclusion.width > 0.0) ||
                  !(exclusion.height > 0.0))
                continue;
              volumes.push_back({exclusion_outline(exclusion), low, high,
                                 selected, true});
            }
          }
        }
        if (operation.auto_identify_pockets &&
            i < app.plan.operations.size() &&
            app.plan.operations[i].planned && app.plan.operations[i].ok()) {
          for (const auto& pocket : app.plan.operations[i].identified_pockets) {
            volumes.push_back({cam::pocket_margin_boundary(
                                   pocket.boundary,
                                   operation.pocket_radial_margin),
                               pocket.bottom_z, pocket.top_z, selected});
          }
        } else if (!operation.auto_identify_pockets) {
          for (const cam::PocketParameters& pocket : operation.pockets) {
            if (!(pocket.depth > 0.0)) continue;
            volumes.push_back({cam::pocket_boundary(
                                   pocket, operation.pocket_radial_margin),
                               pocket.top_z - pocket.depth, pocket.top_z,
                               selected});
          }
        }
      } else if(operation.mode==cam::OperationMode::Hole) {
        const auto resolved=cam::resolve_hole_source(app.job,i);
        if(resolved.holes.automatic && resolved.holes.detection_signature.empty()) continue;
        for(const auto& h:resolved.holes.systems) if(h.selected)
          for(const auto& l:h.levels) if(l.selected && l.radius>0 && l.top_z>l.bottom_z) {
            const auto* tool=app.tools.find(l.tool_id.empty()?resolved.tool_id:l.tool_id);
            const auto strategy=cam::effective_hole_strategy(resolved.holes,l);
            const double radius=strategy==cam::HoleStrategy::Drill
                ? (tool?tool->diameter/2:l.radius)
                : l.radius-resolved.holes.radial_margin;
            if(!(radius>0) || !std::isfinite(radius)) continue;
            const auto circle=Clipper2Lib::Ellipse(wave::Point64(wave::to_integer(h.center_x),wave::to_integer(h.center_y)),
                wave::to_integer(radius),wave::to_integer(radius),96);
            volumes.push_back({{circle},l.bottom_z,l.top_z,selected});
          }
      } else {
        for (const cam::ClearingArea& area : operation.plane.areas) {
          if (!(area.depth > 0.0)) continue;
          if (!(area.width > 0.0) || !(area.height > 0.0)) continue;
          volumes.push_back({area_outline(area), area.top_z - area.depth,
                             area.top_z, selected});
        }
      }
    } catch (const std::exception&) {
    }
  }
  return volumes;
}

// Translucent and red, because they are the one thing in this view that is not
// material: everything else drawn here is a body, and these are the absence
// the job is going to leave in one.
void append_cut_volumes(const std::vector<CutVolume>& volumes,
                        std::vector<Facet>& facets, std::vector<ImVec2>& pool,
                        const Projection& projection, const Vec3& light,
                        double spacing, bool wireframe) {
  for (const CutVolume& volume : volumes) {
    if (volume.top <= volume.bottom) continue;
    const Paths64 rings = drawable_region(volume.rings, spacing);
    if (rings.empty()) continue;
    const float wall[3] = {volume.exclusion ? 0.95f : 0.86f,
                           volume.exclusion ? 0.62f : 0.26f,
                           volume.exclusion ? 0.12f : 0.24f};
    const float face[3] = {volume.exclusion ? 1.0f : 0.95f,
                           volume.exclusion ? 0.78f : 0.42f,
                           volume.exclusion ? 0.22f : 0.36f};
    const int alpha = volume.selected ? 110 : 60;
    const ImU32 edge = volume.exclusion
        ? IM_COL32(255, 205, 80, volume.selected ? 230 : 130)
        : IM_COL32(255, 140, 130, volume.selected ? 220 : 120);
    append_slab(facets, pool, rings, fill_mesh(rings, nullptr), rings,
                volume.bottom, volume.top, projection, light, wall, face,
                edge, alpha,
                wireframe);
  }
}

// ---------------------------------------------------------------------------
// Stage 1: the models, before there is a plan to look at.

// The Z range the operation will actually cut, so the depth range being set can
// be seen against the models rather than read off two numbers.
// Display geometry and native face ranges are immutable for a placed CAD model.
// Selection changes only per-draw colours, never vertex buffers.
struct SetupCadDisplay {
  struct FaceRange { std::size_t face, first, count; };
  std::shared_ptr<const cam::CadModel> model;
  std::shared_ptr<const wave::Mesh> mesh;
  std::vector<FaceRange> ranges;
  wave::cpu::Bvh picking;
  bool indexed = false;
};
SetupCadDisplay& cad_display(App& app) {
  const auto& cad=app.geometry.part_cad();
  if (!app.setup_cad_display || app.setup_cad_display->model != cad) {
    auto display=std::make_shared<SetupCadDisplay>();
    display->model=cad;
    display->mesh=std::shared_ptr<const wave::Mesh>(cad,&cad->mesh());
    const auto& faces=cad->triangle_faces();
    for (std::size_t i=0;i<faces.size();) {
      const auto first=i, face=faces[i];
      while (i<faces.size() && faces[i]==face) ++i;
      display->ranges.push_back({face,first,i-first});
    }
    app.setup_cad_display=std::move(display);
  }
  return *app.setup_cad_display;
}

// Use a spatial index for face picking, rather than projecting every triangle
// again for each mouse event. The precise barycentric test stays in screen space.
std::optional<std::size_t> pick_cad_face(App& app, const Projection& projection, ImVec2 mouse) {
  auto& display=cad_display(app);
  const auto& mesh=*display.mesh;
  if (!display.indexed) {
    std::vector<wave::cpu::Box> boxes;
    boxes.reserve(mesh.triangles.size());
    for (const auto& triangle : mesh.triangles) boxes.push_back(wave::cpu::bounds(triangle));
    display.picking.build(boxes); display.indexed=true;
  }
  std::optional<std::size_t> face;
  float best=-std::numeric_limits<float>::infinity();
  const auto& bounds=mesh.bounds;
  const float dx=(bounds.lower.x+bounds.upper.x)*.5f-projection.model_centre.x,
              dy=(bounds.lower.y+bounds.upper.y)*.5f-projection.model_centre.y,
              dz=(bounds.lower.z+bounds.upper.z)*.5f-projection.model_centre.z;
  const float reach=std::sqrt(dx*dx+dy*dy+dz*dz)+
      std::max({bounds.upper.x-bounds.lower.x,bounds.upper.y-bounds.lower.y,bounds.upper.z-bounds.lower.z,1.f})*4;
  const float x=(mouse.x-projection.centre.x)/projection.scale,
              y=-(mouse.y-projection.centre.y)/projection.scale;
  wave::cpu::Ray ray;
  ray.origin={projection.model_centre.x+projection.right.x*x+projection.up.x*y+projection.toward.x*reach,
              projection.model_centre.y+projection.right.y*x+projection.up.y*y+projection.toward.y*reach,
              projection.model_centre.z+projection.right.z*x+projection.up.z*y+projection.toward.z*reach};
  ray.direction={-projection.toward.x,-projection.toward.y,-projection.toward.z}; ray.max=reach*2;
  display.picking.query([&](const auto& box) {return wave::cpu::intersects(box,ray);}, [&](std::size_t i) {
    const auto& t=mesh.triangles[i];
    const Vec3 a{t.a.x,t.a.y,t.a.z},b{t.b.x,t.b.y,t.b.z},c{t.c.x,t.c.y,t.c.z};
    const auto p=projection.point(a),q=projection.point(b),r=projection.point(c);
    const float det=(q.y-r.y)*(p.x-r.x)+(r.x-q.x)*(p.y-r.y);
    if (std::abs(det)<1e-8f) return true;
    const float u=((q.y-r.y)*(mouse.x-r.x)+(r.x-q.x)*(mouse.y-r.y))/det;
    const float v=((r.y-p.y)*(mouse.x-r.x)+(p.x-r.x)*(mouse.y-r.y))/det;
    if (u<0 || v<0 || u+v>1) return true;
    const float depth=u*projection.depth(a)+v*projection.depth(b)+(1-u-v)*projection.depth(c);
    if (depth>best) {best=depth;face=display.model->triangle_faces()[i];}
    return true;
  });
  return face;
}

std::shared_ptr<const wave::Mesh> cached_box_mesh(std::shared_ptr<const wave::Mesh>& cached,
                                                 wave::Bounds3f bounds) {
  const auto same=[](auto a,auto b) {return a.x==b.x && a.y==b.y && a.z==b.z;};
  if (cached && same(cached->bounds.lower,bounds.lower) && same(cached->bounds.upper,bounds.upper)) return cached;
  auto mesh=std::make_shared<wave::Mesh>(); mesh->bounds=bounds;
  const auto a=bounds.lower,b=bounds.upper;
  const wave::Vec3f p[]{{a.x,a.y,a.z},{b.x,a.y,a.z},{b.x,b.y,a.z},{a.x,b.y,a.z},
                        {a.x,a.y,b.z},{b.x,a.y,b.z},{b.x,b.y,b.z},{a.x,b.y,b.z}};
  const int faces[][3]{{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},
                       {1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
  for (const auto& f:faces) mesh->triangles.push_back({p[f[0]],p[f[1]],p[f[2]]});
  return cached=std::move(mesh);
}
wave::heightfield::MeshDraw model_draw(std::shared_ptr<const wave::Mesh> source,
                                     std::array<float,4> colour, bool wireframe) {
  return {nullptr,{},colour,wireframe,std::move(source)};
}

void draw_setup(App& app) {
  OrbitScene scene;
  begin_scene(app, scene, "setup");
  // Dimension-driven operations need no mesh, so this view cannot insist on
  // one. Where there is nothing loaded it shows the volumes on their own.
  const std::vector<CutVolume> volumes = cut_volumes(app);
  const bool models_loaded = app.geometry.has_part();
  double low = 0.0;
  double high = 0.0;
  const bool has_range = models_loaded && app.geometry.work_range(low, high);
  if (!has_range && volumes.empty()) {
    scene_message(app, scene, models_loaded ? "The models have no depth to cut"
                                            : "Load a part model to begin (File > Part model)");
    view_overlay(app, ViewKind::Setup, scene.top_left, scene.size);
    end_scene(scene);
    return;
  }
  for (const CutVolume& volume : volumes) {
    if (volume.top <= volume.bottom) continue;
    if (!has_range && low == high) {
      low = volume.bottom;
      high = volume.top;
      continue;
    }
    low = std::min(low, volume.bottom);
    high = std::max(high, volume.top);
  }

  cam::Operation* operation =
      app.job.operations.empty() || app.selected_operation < 0
          ? nullptr
          : &app.job.operations[std::clamp(app.selected_operation, 0,
                                           static_cast<int>(app.job.operations.size()) - 1)];

  const bool draw_stock = app.geometry.has_stock() && (app.setup_models == ModelShow::Stock ||
                                            app.setup_models == ModelShow::Both);
  const bool draw_part = models_loaded && (app.setup_models == ModelShow::Part ||
                                           app.setup_models == ModelShow::Both);
  // A rectangular blank has no mesh behind it; it is drawn as its prism.
  const bool boxed = app.geometry.box_stock();
  const auto inherited=app.geometry.inherited_stock();
  if(inherited!=app.setup_stock_display_source) {
    app.setup_stock_display=inherited?std::make_shared<const wave::Mesh>(inherited->display_mesh()):nullptr;
    app.setup_stock_display_source=inherited;
  }
  const wave::Paths64 box =
      boxed ? app.geometry.stock_outline(low, operation ? operation->slice_grid : 32)
            : wave::Paths64{};

  Clipper2Lib::Rect64 extent;
  bool any = false;
  const auto cover = [&](std::int64_t left, std::int64_t top, std::int64_t right,
                         std::int64_t bottom) {
    if (!any) {
      extent = Clipper2Lib::Rect64(left, top, right, bottom);
      any = true;
      return;
    }
    extent.left = std::min(extent.left, left);
    extent.right = std::max(extent.right, right);
    extent.top = std::min(extent.top, top);
    extent.bottom = std::max(extent.bottom, bottom);
  };
  const auto cover_mesh = [&](const wave::Mesh& mesh) {
    if (!mesh.bounds.valid) return;
    cover(wave::to_integer(mesh.bounds.lower.x), wave::to_integer(mesh.bounds.lower.y),
          wave::to_integer(mesh.bounds.upper.x), wave::to_integer(mesh.bounds.upper.y));
  };
  if (models_loaded) {
    cover_mesh(app.geometry.part().mesh());
    if (boxed) {
      if (!box.empty()) {
        const auto bounds = Clipper2Lib::GetBounds(box);
        cover(bounds.left, bounds.top, bounds.right, bounds.bottom);
      }
    } else {
      cover_mesh(inherited?*app.setup_stock_display:app.geometry.stock().mesh());
    }
  }
  const double work_low=low, work_high=high;
  for(const auto& f:app.geometry.fixtures()) {
    cover_mesh(f.geometry.mesh());
    low=std::min(low,double(f.geometry.mesh().bounds.lower.z));
    high=std::max(high,double(f.geometry.mesh().bounds.upper.z));
  }
  for (const CutVolume& volume : volumes) {
    if (volume.rings.empty()) continue;
    const auto bounds = Clipper2Lib::GetBounds(volume.rings);
    cover(bounds.left, bounds.top, bounds.right, bounds.bottom);
  }
  if (!any) {
    scene_message(app, scene, "Nothing to show");
    view_overlay(app, ViewKind::Setup, scene.top_left, scene.size);
    end_scene(scene);
    return;
  }
  aim_scene(app, scene, extent, low, high);

  const auto& cad = app.geometry.part_cad();
  const bool selecting =
      operation && operation->mode == cam::OperationMode::SurfaceFinishing && cad;
  const auto hovered_face = selecting && scene.hovered && !app.orbit.cube_pressed
      ? pick_cad_face(app, scene.projection, ImGui::GetIO().MousePos)
      : std::optional<std::size_t>{};
  const auto drag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
  if (hovered_face && !operation->surface.manual_order && !app.planner.running() &&
      ImGui::IsMouseReleased(ImGuiMouseButton_Left) && drag.x * drag.x + drag.y * drag.y < 9) {
    auto& selection = app.job.operations[app.selected_operation].surface;
    if (selection.model_fingerprint != cad->fingerprint()) selection.faces.clear();
    selection.model_fingerprint = cad->fingerprint();
    const auto found = std::find(selection.faces.begin(), selection.faces.end(), *hovered_face);
    if (found == selection.faces.end()) selection.faces.push_back(*hovered_face);
    else selection.faces.erase(found);
  }
  std::vector<wave::heightfield::MeshDraw> models;
  if (draw_part) {
    if (selecting) {
      const auto& display=cad_display(app);
      const auto& selection=operation->surface;
      for (const auto& range:display.ranges) {
        auto colour=std::array<float,4>{.42f,.60f,.80f,1};
        if (hovered_face && range.face==*hovered_face) colour={.95f,.7f,.25f,1};
        else if (selection.model_fingerprint==cad->fingerprint() &&
                 std::find(selection.faces.begin(),selection.faces.end(),range.face)!=selection.faces.end())
          colour={.2f,.85f,.5f,1};
        auto draw=model_draw(display.mesh,colour,app.wireframe);
        draw.first_triangle=range.first; draw.triangle_count=range.count;
        if (!models.empty() && models.back().source==draw.source && models.back().colour==draw.colour &&
            models.back().first_triangle+models.back().triangle_count==draw.first_triangle)
          models.back().triangle_count+=draw.triangle_count;
        else models.push_back(std::move(draw));
      }
    } else models.push_back(model_draw(cad?cad_display(app).mesh:app.geometry.part().shared_mesh(),
                                       {.42f,.60f,.80f,app.wireframe?.75f:1.f},app.wireframe));
  }
  if (draw_stock) {
    std::shared_ptr<const wave::Mesh> stock;
    if (boxed && !box.empty()) {
      const auto b=Clipper2Lib::GetBounds(box);
      stock=cached_box_mesh(app.setup_box_display,
          {{float(wave::to_world(b.left)),float(wave::to_world(b.top)),float(work_low)},
           {float(wave::to_world(b.right)),float(wave::to_world(b.bottom)),float(work_high)},true});
    } else stock=inherited?app.setup_stock_display:app.geometry.stock().shared_mesh();
    if (stock) models.push_back(model_draw(stock,{.34f,.38f,.45f,app.wireframe?.59f:draw_part?.35f:1.f},app.wireframe));
  }
  for (const auto& fixture:app.geometry.fixtures()) {
    const bool clamp=fixture.model.role==cam::SetupContext::ProtectedModel::Role::Fixture;
    models.push_back(model_draw(fixture.geometry.shared_mesh(),
        {clamp?.75f:.9f,clamp?.35f:.65f,.18f,.75f},app.wireframe));
  }
  try {
    if (!setup_view) setup_view=std::make_unique<wave::heightfield::Renderer>();
    const auto vec=[](Vec3 v) {return wave::Vec3f{v.x,v.y,v.z};};
    const auto dpi=ImGui::GetIO().DisplayFramebufferScale;
    const float pixels=float(app.settings.surface_render_scale)*std::max({1.f,dpi.x,dpi.y});
    const float span=std::max({float(wave::to_world(extent.Width())),float(wave::to_world(extent.Height())),float(high-low),.001f});
    const auto& p=scene.projection;
    wave::heightfield::Camera camera{vec(p.model_centre),vec(p.right),vec(p.up),vec(p.toward),vec(scene.light),
        p.scale*pixels,span*4,(p.centre.x-scene.top_left.x)*pixels,(p.centre.y-scene.top_left.y)*pixels,
        canvas_colours(app).background_rgba};
    static const wave::Heightmap empty;
    const auto texture=setup_view->render(empty,camera,int(std::ceil(scene.size.x*pixels)),
                                          int(std::ceil(scene.size.y*pixels)),models);
    scene.list->AddImage(static_cast<ImTextureID>(texture),scene.top_left,
        {scene.top_left.x+scene.size.x,scene.top_left.y+scene.size.y},{0,1},{1,0});
  } catch (const std::exception& e) {
    setup_view.reset();
    scene.list->AddText(scene.top_left,ImGui::GetColorU32(severity_colour(app,LogSeverity::Error)),e.what());
  }
  if (selecting && operation->surface.manual_order) {
    refresh_surface_contours(app, *operation);
    draw_surface_contours(app, scene, *operation);
  }
  // The cut volumes go over the bodies, sorted on their own: they are a
  // statement about the bodies rather than another body among them.
  if (!volumes.empty()) {
    std::vector<Facet> cut_facets;
    std::vector<ImVec2> cut_pool;
    append_cut_volumes(volumes, cut_facets, cut_pool, scene.projection, scene.light,
                       scene.spacing, app.wireframe);
    paint_facets(scene.list, cut_facets, cut_pool);
  }
  if(operation && operation->mode==cam::OperationMode::Hole)
    draw_hole_contours(app,scene,*operation);
  draw_origin_gizmo(app, scene);
  view_cube(app, scene.list, scene.top_left, scene.size, scene.hovered);

  const CanvasColours colours = canvas_colours(app);
  char caption[260];
  const double oz = app.effective_context().origin_z;
  std::snprintf(caption, sizeof caption,
                "Work Z %.2f to %.2f mm   Drag: orbit   Right-drag: pan   Wheel: zoom",
                work_low + oz, work_high + oz);
  view_caption(app, scene.list, scene.top_left, scene.size, 0, colours.caption, caption);
  if (operation && operation->mode == cam::OperationMode::SurfaceFinishing) {
    view_caption(app, scene.list, scene.top_left, scene.size, 1,
                 ImGui::GetColorU32(success_colour(app)), operation->surface.manual_order
                     ? "Click a contour to toggle a propagation root"
                     : "Click a face to select it");
  } else if(operation && operation->mode==cam::OperationMode::Hole) {
    view_caption(app,scene.list,scene.top_left,scene.size,1,
        ImGui::GetColorU32(success_colour(app)),"Click a hole rim to toggle that level");
  } else if (operation && (operation->mode == cam::OperationMode::Roughing ||
                           operation->mode == cam::OperationMode::ContourFinishing)) {
    std::snprintf(caption, sizeof caption, "%s cuts Z %.2f down to %.2f mm",
                  operation->name.c_str(), cam::z_at_depth(work_high, operation->depth_from) + oz,
                  cam::z_at_depth(work_high, operation->depth_to) + oz);
    view_caption(app, scene.list, scene.top_left, scene.size, 1,
                 ImGui::GetColorU32(severity_colour(app, LogSeverity::Info)), caption);
  }
  view_overlay(app, ViewKind::Setup, scene.top_left, scene.size);
  end_scene(scene);
}

namespace {

ImU32 phase_colour(const App& app, const cam::MotionPhase& phase, double lowest,
                   double highest, double maximum_feed) {
  if (phase.rapid) return move_colour(wave::MoveKind::Retract);
  switch (app.colouring) {
    case Colouring::Feed: return feed_colour(phase.rate, lowest, highest);
    case Colouring::Load: return engagement_colour(phase.engagement);
    case Colouring::Limit:
      if (phase.kind == wave::MoveKind::Entry) return limit_colour(cam::FeedLimit::Plunge);
      return limit_colour(phase.rate >= maximum_feed - 1e-6 ? cam::FeedLimit::Machine
                                                             : cam::FeedLimit::Engagement);
    case Colouring::Move: return move_colour(phase.kind);
  }
  return IM_COL32(150, 150, 150, 255);
}

void prepare_result_preview(App& app, const cam::OperationPlan& operation,
                            const cam::Tool* tool, double maximum_feed) {
  auto& cached = app.result_preview;
  const int level = app.show_all_passes ? -1 :
      std::clamp(app.selected_level, 0, int(operation.levels.size())-1);
  if (cached.lines && cached.serial == app.plan_serial &&
      cached.operation == app.selected_plan_operation && cached.level == level &&
      cached.colouring == app.colouring && cached.cutting == app.show_cutting_paths &&
      cached.rapids == app.show_rapid_travel && cached.maximum_feed == maximum_feed)
    return;
  cached = {};
  cached.serial = app.plan_serial;
  cached.operation = app.selected_plan_operation;
  cached.level = level;
  cached.colouring = app.colouring;
  cached.cutting = app.show_cutting_paths;
  cached.rapids = app.show_rapid_travel;
  cached.maximum_feed = maximum_feed;
  double lowest = 0, highest = 0;
  feed_range(app, lowest, highest);
  auto lines = std::make_shared<wave::heightfield::Lines>();
  const auto include = [&](wave::Vec3d p, double radius, double flute) {
    auto& b = cached.motion_bounds;
    if (!b.valid) {
      b.lower = {float(p.x-radius),float(p.y-radius),float(p.z)};
      b.upper = {float(p.x+radius),float(p.y+radius),float(p.z+flute)};
      b.valid = true;
    } else {
      b.lower = {std::min(b.lower.x,float(p.x-radius)),std::min(b.lower.y,float(p.y-radius)),std::min(b.lower.z,float(p.z))};
      b.upper = {std::max(b.upper.x,float(p.x+radius)),std::max(b.upper.y,float(p.y+radius)),std::max(b.upper.z,float(p.z+flute))};
    }
  };
  for (std::size_t i = 0; i < operation.levels.size(); ++i) {
    const auto& pass = operation.levels[i];
    const auto* cutter = pass.tool_id.empty() ? tool : app.tools.find(pass.tool_id);
    for (const auto& phase : pass.feeds.motion) {
      const double radius = cutter ? cutter->diameter/2 : 0;
      const double flute = cutter ? cutter->flute_length : 0;
      // Include arc samples in the frame too: endpoints alone miss a full circle.
      const int pieces = phase.arc ? 64 : 1;
      const bool show = (level < 0 || int(i) == level) &&
          (phase.rapid ? app.show_rapid_travel : app.show_cutting_paths);
      const auto colour = ImGui::ColorConvertU32ToFloat4(
          phase_colour(app, phase, lowest, highest, maximum_feed));
      auto previous = cam::motion_point(phase, 0);
      include(previous, radius, flute);
      for (int j = 1; j <= pieces; ++j) {
        const auto next = cam::motion_point(phase, double(j)/pieces);
        include(next, radius, flute);
        if (show) {
          for (auto p : {previous, next})
            lines->vertices.push_back({{float(p.x),float(p.y),float(p.z)},
                                       {colour.x,colour.y,colour.z,colour.w}});
        }
        previous = next;
      }
    }
  }
  cached.lines = std::move(lines);
}

}  // namespace

void draw_result(App& app) {
  ImGui::BeginChild("result", ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  const ImVec2 top_left = ImGui::GetCursorScreenPos();
  ImVec2 size = ImGui::GetContentRegionAvail();
  size.x = std::max(size.x, 50.0f);
  size.y = std::max(size.y, 50.0f);
  ImGui::SetNextItemAllowOverlap();
  ImGui::InvisibleButton("stack", size,
                         ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
  const bool hovered = ImGui::IsItemHovered();
  ImDrawList* list = ImGui::GetWindowDrawList();
  const CanvasColours colours = canvas_colours(app);
  list->AddRectFilled(top_left, ImVec2(top_left.x + size.x, top_left.y + size.y),
                      colours.background);
  list->PushClipRect(top_left, ImVec2(top_left.x + size.x, top_left.y + size.y), true);
  orbit_controls(app, hovered && !app.orbit.cube_pressed);

  const auto finish = [&](const char* message) {
    const ImVec2 extent = ImGui::CalcTextSize(message);
    list->AddText({top_left.x + (size.x - extent.x) * 0.5f,
                   top_left.y + (size.y - extent.y) * 0.5f},
                  colours.message, message);
    list->PopClipRect();
    view_overlay(app, ViewKind::Result, top_left, size);
    ImGui::EndChild();
  };

  if (!app.has_plan || app.plan.operations.empty()) {
    finish("Plan the job to see the result");
    return;
  }
  const auto& planned_operation = app.plan.operations[app.selected_plan_operation];
  const auto& levels = planned_operation.levels;
  if (!planned_operation.planned || !planned_operation.ok() || !planned_operation.stock_after) {
    finish(planned_operation.planned ? "This operation could not be planned"
                                     : "This operation has not been planned yet");
    return;
  }
  if (levels.empty()) {
    finish("This operation has no passes");
    return;
  }

  const int active_level = std::clamp(app.selected_level, 0, static_cast<int>(levels.size()) - 1);
  const cam::LevelPlan& active = levels[active_level];
  const cam::Tool* active_tool = app.tools.find(active.tool_id.empty()?planned_operation.tool_id:active.tool_id);
  std::optional<cam::MotionCursor> cutter_cursor;
  if (active_tool && !active.feeds.motion.empty())
    cutter_cursor = cam::feed_motion_at_seconds(active.feeds, app.playback.motion_seconds);

  // Framed on everything that does not move: the stock and every commanded
  // position, with room for the flute.
  const auto& map = *planned_operation.stock_before;
  Clipper2Lib::Rect64 extent_2d{
      wave::to_integer(map.lower_x()), wave::to_integer(map.lower_y()),
      wave::to_integer(map.lower_x() + map.columns() * map.pitch()),
      wave::to_integer(map.lower_y() + map.rows() * map.pitch())};
  double lowest = map.lowest(), highest = map.highest();
  const double material_lowest = lowest, material_highest = highest;
  const cam::Machine* machine = app.machines.find(app.effective_context().machine_id);
  const double maximum_feed = machine ? cam::rapid_feed(*machine) : 1e12;
  prepare_result_preview(app, planned_operation, app.tools.find(planned_operation.tool_id), maximum_feed);
  const auto& motion_bounds = app.result_preview.motion_bounds;
  if (motion_bounds.valid) {
    extent_2d.left = std::min(extent_2d.left, wave::to_integer(motion_bounds.lower.x));
    extent_2d.right = std::max(extent_2d.right, wave::to_integer(motion_bounds.upper.x));
    extent_2d.top = std::min(extent_2d.top, wave::to_integer(motion_bounds.lower.y));
    extent_2d.bottom = std::max(extent_2d.bottom, wave::to_integer(motion_bounds.upper.y));
    lowest = std::min(lowest, double(motion_bounds.lower.z));
    highest = std::max(highest, double(motion_bounds.upper.z));
  }
  const double width = wave::to_world(extent_2d.right - extent_2d.left);
  const double height = wave::to_world(extent_2d.bottom - extent_2d.top);
  const double span = std::max({width, height, highest - lowest, 1e-3});
  const Vec3 centre{static_cast<float>(wave::to_world(extent_2d.left + extent_2d.right) * 0.5),
                    static_cast<float>(wave::to_world(extent_2d.top + extent_2d.bottom) * 0.5),
                    static_cast<float>((lowest + highest) * 0.5)};
  const Basis basis = orbit_basis(app.orbit.yaw, app.orbit.pitch);
  const Projection projection{
      {top_left.x + size.x * 0.5f + app.orbit.pan.x,
       top_left.y + size.y * 0.5f + app.orbit.pan.y},
      centre, basis.right, basis.up, basis.toward,
      static_cast<float>(std::min(size.x, size.y) * 0.72 / span) * app.orbit.zoom};
  // A single light fixed to the eye, so the model stays lit however it turns.
  const Vec3 raw{projection.toward.x * 0.85f + projection.up.x * 0.35f + projection.right.x * 0.35f,
                 projection.toward.y * 0.85f + projection.up.y * 0.35f + projection.right.y * 0.35f,
                 projection.toward.z * 0.85f + projection.up.z * 0.35f + projection.right.z * 0.35f};
  const float raw_length = std::max(1e-6f, std::sqrt(raw.x * raw.x + raw.y * raw.y + raw.z * raw.z));
  const Vec3 light{raw.x / raw_length, raw.y / raw_length, raw.z / raw_length};

  auto shown = planned_operation.stock_after;
  if (app.stack_follows_playback) {
    if (app.material_heightmap) shown = app.material_heightmap;
    else if (!active.stock_checkpoints.empty()) shown = active.stock_checkpoints.front().stock;
  }
  try {
    if (!heightmap_view) heightmap_view = std::make_unique<wave::heightfield::Renderer>();
    const auto vec = [](Vec3 v) { return wave::Vec3f{v.x, v.y, v.z}; };
    const auto framebuffer_scale = ImGui::GetIO().DisplayFramebufferScale;
    // Start at half Setup's pixel density. Bound ray work on large/high-DPI
    // canvases as well: doubling the window size must not quadruple ray cost.
    const bool automatic=app.settings.stock_render_scale==0;
    const float density=automatic ? .5f : float(app.settings.stock_render_scale);
    float pixel_scale = density * std::max({1.f, framebuffer_scale.x, framebuffer_scale.y});
    constexpr double max_result_pixels = 1024. * 1024.;
    const double pixels = double(size.x) * size.y * pixel_scale * pixel_scale;
    if (automatic && pixels > max_result_pixels)
      pixel_scale *= float(std::sqrt(max_result_pixels / pixels));
    wave::heightfield::Camera camera{vec(projection.model_centre), vec(projection.right),
                                     vec(projection.up), vec(projection.toward), vec(light),
                                     projection.scale * pixel_scale, float(span * 4),
                                     (projection.centre.x - top_left.x) * pixel_scale,
                                     (projection.centre.y - top_left.y) * pixel_scale,
                                     colours.background_rgba};
    std::vector<wave::heightfield::MeshDraw> models;
    const bool want_part = app.result_models == ModelShow::Part ||
                           app.result_models == ModelShow::Both;
    const bool want_stock = app.result_models == ModelShow::Stock ||
                            app.result_models == ModelShow::Both;
    if (want_part && app.geometry.has_part())
      models.push_back(model_draw(app.geometry.part().shared_mesh(), {.82f, .62f, .34f, .37f}, app.wireframe));
    if (want_stock) {
      if (!app.geometry.box_stock() && app.geometry.has_stock() && !app.geometry.inherited_stock()) {
        models.push_back(model_draw(app.geometry.stock().shared_mesh(), {.34f,.38f,.45f,.22f},app.wireframe));
      } else {
        models.push_back(model_draw(cached_box_mesh(app.result_box_display,
            {{float(map.lower_x()),float(map.lower_y()),float(map.lowest())},
             {float(map.lower_x()+map.columns()*map.pitch()),float(map.lower_y()+map.rows()*map.pitch()),float(map.highest())},true}),
            {.34f,.38f,.45f,.22f},app.wireframe));
      }
    }
    for(const auto& f:app.geometry.fixtures())
      models.push_back(model_draw(f.geometry.shared_mesh(),{.8f,.35f,.15f,.65f},app.wireframe));
    wave::heightfield::Cutter cutter;
    if (cutter_cursor && active_tool)
      cutter = {{float(wave::to_world(cutter_cursor->position.x)),
                 float(wave::to_world(cutter_cursor->position.y)), float(cutter_cursor->z)},
                float(active_tool->diameter / 2), float(active_tool->flute_length),
                active_tool->type == cam::ToolType::BallEndMill,
                active_tool->type==cam::ToolType::Drill?float(cam::drill_tip_height(*active_tool)):0,
                active_tool->type==cam::ToolType::Drill?float(active_tool->tip_flat_diameter/2):0,
                cutter_cursor->fixture_contact};
    const auto texture = heightmap_view->render(*shown, camera,
                                                int(std::ceil(size.x * pixel_scale)),
                                                int(std::ceil(size.y * pixel_scale)), models,
                                                cutter, app.result_preview.lines);
    list->AddImage(static_cast<ImTextureID>(texture), top_left,
                   ImVec2(top_left.x + size.x, top_left.y + size.y), ImVec2(0, 1),
                   ImVec2(1, 0));
  } catch (const std::exception& error) {
    heightmap_view.reset();
    list->AddText(top_left, ImGui::GetColorU32(severity_colour(app, LogSeverity::Error)),
                  error.what());
  }

  OrbitScene overlay;
  overlay.list = list;
  overlay.projection = projection;
  overlay.top_left = top_left;
  overlay.size = size;
  draw_origin_gizmo(app, overlay);
  view_cube(app, list, top_left, size, hovered);

  char caption[200];
  const double oz = app.effective_context().origin_z;
  if (planned_operation.subdivisions.size() > 1)
    std::snprintf(caption, sizeof caption,
                  "%zu layers (%zu coarse + upward cleanup), Z %.2f to %.2f mm",
                  levels.size(), planned_operation.subdivisions.front().level_count,
                  material_lowest + oz, material_highest + oz);
  else
    std::snprintf(caption, sizeof caption, "%s: %zu passes, Z %.2f to %.2f mm",
                  planned_operation.operation_name.c_str(), levels.size(),
                  material_lowest + oz, material_highest + oz);
  view_caption(app, list, top_left, size, 0, colours.caption, caption);
  list->PopClipRect();
  view_overlay(app, ViewKind::Result, top_left, size);
  ImGui::EndChild();
}

}  // namespace tinglecam_gui
