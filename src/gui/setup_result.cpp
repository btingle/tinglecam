#include "gui_internal.h"
#include "cam/heightmap_simulation.h"
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
            const double radius=!resolved.holes.source_id.empty() && tool?tool->diameter/2:l.radius;
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
// Orthographic ray picking shares the exact projection used for display.
std::optional<std::size_t> pick_cad_face(const cam::CadModel& cad,
    const Projection& projection, ImVec2 mouse) {
  std::optional<std::size_t> face;
  float best = -std::numeric_limits<float>::infinity();
  const auto& mesh = cad.mesh();
  for (std::size_t i=0;i<mesh.triangles.size();++i) {
    const auto& t=mesh.triangles[i];
    const Vec3 a{t.a.x,t.a.y,t.a.z},b{t.b.x,t.b.y,t.b.z},c{t.c.x,t.c.y,t.c.z};
    const auto p=projection.point(a),q=projection.point(b),r=projection.point(c);
    const float det=(q.y-r.y)*(p.x-r.x)+(r.x-q.x)*(p.y-r.y);
    if(std::abs(det)<1e-8f) continue;
    const float u=((q.y-r.y)*(mouse.x-r.x)+(r.x-q.x)*(mouse.y-r.y))/det;
    const float v=((r.y-p.y)*(mouse.x-r.x)+(p.x-r.x)*(mouse.y-r.y))/det;
    if(u<0||v<0||u+v>1) continue;
    const float depth=u*projection.depth(a)+v*projection.depth(b)+(1-u-v)*projection.depth(c);
    if(depth>best) {best=depth;face=cad.triangle_faces()[i];}
  }
  return face;
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
      cover_mesh(app.geometry.stock().mesh());
    }
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
      ? pick_cad_face(*cad, scene.projection, ImGui::GetIO().MousePos)
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
  // Both bodies whole, the stock see-through when both are on.
  if (draw_part) {
    const float colour[3]{0.42f, 0.60f, 0.80f};
    if (selecting) {
      wave::Mesh normal, selected, hover;
      const auto& selection = operation->surface;
      for (std::size_t i = 0; i < cad->mesh().triangles.size(); ++i) {
        const auto id = cad->triangle_faces()[i];
        auto* target = hovered_face && id == *hovered_face ? &hover
            : (selection.model_fingerprint == cad->fingerprint() &&
                       std::find(selection.faces.begin(), selection.faces.end(), id) !=
                           selection.faces.end()
                   ? &selected : &normal);
        target->triangles.push_back(cad->mesh().triangles[i]);
      }
      const float selected_colour[3]{0.2f, 0.85f, 0.5f}, hover_colour[3]{0.95f, 0.7f, 0.25f};
      append_mesh(scene.facets, scene.pool, normal, scene.projection, scene.light, colour, 255, app.wireframe);
      append_mesh(scene.facets, scene.pool, selected, scene.projection, scene.light, selected_colour, 255, app.wireframe);
      append_mesh(scene.facets, scene.pool, hover, scene.projection, scene.light, hover_colour, 255, app.wireframe);
    } else {
      append_mesh(scene.facets, scene.pool, app.geometry.part().mesh(), scene.projection,
                  scene.light, colour, app.wireframe ? 190 : 255, app.wireframe);
    }
  }
  if (draw_stock) {
    const float colour[3]{0.34f, 0.38f, 0.45f};
    const int alpha = app.wireframe ? 150 : (draw_part ? 90 : 255);
    if (boxed) {
      const wave::Paths64 rings = drawable_region(box, scene.spacing);
      if (!rings.empty()) {
        const float face[3]{0.40f, 0.44f, 0.52f};
        append_slab(scene.facets, scene.pool, rings, fill_mesh(rings, nullptr), rings, low,
                    high, scene.projection, scene.light, colour, face,
                    IM_COL32(160, 180, 205, 70), alpha, app.wireframe);
      }
    } else {
      append_mesh(scene.facets, scene.pool, app.geometry.stock().mesh(), scene.projection,
                  scene.light, colour, alpha, app.wireframe);
    }
  }
  paint_scene(scene);
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
  const double oz = app.job.origin_z;
  std::snprintf(caption, sizeof caption,
                "Work Z %.2f to %.2f mm   Drag: orbit   Right-drag: pan   Wheel: zoom",
                low + oz, high + oz);
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
                  operation->name.c_str(), cam::z_at_depth(high, operation->depth_from) + oz,
                  cam::z_at_depth(high, operation->depth_to) + oz);
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

void draw_motion(const App& app, ImDrawList* list, const Projection& projection,
                 const cam::LevelPlan& level, double lowest, double highest,
                 double maximum_feed) {
  for (const cam::MotionPhase& phase : level.feeds.motion) {
    if (phase.rapid ? !app.show_rapid_travel : !app.show_cutting_paths) continue;
    const ImU32 colour = phase_colour(app, phase, lowest, highest, maximum_feed);
    const int pieces = phase.arc ? 64 : 1;
    auto previous = cam::motion_point(phase, 0);
    const auto screen = [&](wave::Vec3d p) {
      return projection.point({float(p.x), float(p.y), float(p.z)});
    };
    for (int j = 1; j <= pieces; ++j) {
      const auto next = cam::motion_point(phase, double(j) / pieces);
      list->AddLine(screen(previous), screen(next), colour,
                    phase.kind == wave::MoveKind::Entry ? 1.8f : 1.0f);
      previous = next;
    }
  }
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
  if (active_tool) {
    const std::int64_t radius = wave::to_integer(active_tool->diameter * 0.5);
    for (const auto& level : levels)
      for (const cam::MotionPhase& phase : level.feeds.motion) {
        for (const Point64& point : {phase.from, phase.to}) {
          extent_2d.left = std::min(extent_2d.left, point.x - radius);
          extent_2d.right = std::max(extent_2d.right, point.x + radius);
          extent_2d.top = std::min(extent_2d.top, point.y - radius);
          extent_2d.bottom = std::max(extent_2d.bottom, point.y + radius);
        }
        lowest = std::min({lowest, phase.from_z, phase.to_z});
        highest = std::max({highest, std::max(phase.from_z, phase.to_z) +
                                         active_tool->flute_length});
      }
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
    const float pixel_scale = std::max({1.f, framebuffer_scale.x, framebuffer_scale.y});
    wave::heightfield::Camera camera{vec(projection.model_centre), vec(projection.right),
                                     vec(projection.up), vec(projection.toward), vec(light),
                                     projection.scale * pixel_scale, float(span * 4),
                                     (projection.centre.x - top_left.x) * pixel_scale,
                                     (projection.centre.y - top_left.y) * pixel_scale,
                                     colours.background_rgba};
    std::vector<wave::heightfield::MeshDraw> models;
    wave::Mesh box;
    const bool want_part = app.result_models == ModelShow::Part ||
                           app.result_models == ModelShow::Both;
    const bool want_stock = app.result_models == ModelShow::Stock ||
                            app.result_models == ModelShow::Both;
    if (want_part && app.geometry.has_part())
      models.push_back({&app.geometry.part().mesh(), {}, {.82f, .62f, .34f, .37f}, app.wireframe});
    if (want_stock) {
      if (!app.geometry.box_stock() && app.geometry.has_stock()) {
        models.push_back({&app.geometry.stock().mesh(), {}, {.34f, .38f, .45f, .22f},
                          app.wireframe});
      } else {
        const float x0 = map.lower_x(), y0 = map.lower_y();
        const float x1 = x0 + map.columns() * map.pitch();
        const float y1 = y0 + map.rows() * map.pitch();
        const float z0 = map.lowest(), z1 = map.highest();
        const wave::Vec3f points[]{{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0},
                                   {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}};
        const int faces[][3]{{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
                             {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
        for (const auto& f : faces)
          box.triangles.push_back({points[f[0]], points[f[1]], points[f[2]]});
        models.push_back({&box, {}, {.34f, .38f, .45f, .22f}, app.wireframe});
      }
    }
    wave::heightfield::Cutter cutter;
    if (cutter_cursor && active_tool)
      cutter = {{float(wave::to_world(cutter_cursor->position.x)),
                 float(wave::to_world(cutter_cursor->position.y)), float(cutter_cursor->z)},
                float(active_tool->diameter / 2), float(active_tool->flute_length),
                active_tool->type == cam::ToolType::BallEndMill,
                active_tool->type==cam::ToolType::Drill?float(cam::drill_tip_height(*active_tool)):0,
                active_tool->type==cam::ToolType::Drill?float(active_tool->tip_flat_diameter/2):0};
    const auto texture = heightmap_view->render(*shown, camera,
                                                int(std::ceil(size.x * pixel_scale)),
                                                int(std::ceil(size.y * pixel_scale)), models,
                                                cutter);
    list->AddImage(static_cast<ImTextureID>(texture), top_left,
                   ImVec2(top_left.x + size.x, top_left.y + size.y), ImVec2(0, 1),
                   ImVec2(1, 0));
  } catch (const std::exception& error) {
    heightmap_view.reset();
    list->AddText(top_left, ImGui::GetColorU32(severity_colour(app, LogSeverity::Error)),
                  error.what());
  }

  double feed_low = 0.0, feed_high = 0.0;
  feed_range(app, feed_low, feed_high);
  const cam::Machine* machine = app.machines.find(app.job.machine_id);
  const double maximum_feed = machine ? cam::rapid_feed(*machine) : 1e12;
  if (app.show_all_passes) {
    for (const auto& level : levels)
      draw_motion(app, list, projection, level, feed_low, feed_high, maximum_feed);
  } else {
    draw_motion(app, list, projection, active, feed_low, feed_high, maximum_feed);
  }

  OrbitScene overlay;
  overlay.list = list;
  overlay.projection = projection;
  overlay.top_left = top_left;
  overlay.size = size;
  draw_origin_gizmo(app, overlay);
  view_cube(app, list, top_left, size, hovered);

  char caption[200];
  const double oz = app.job.origin_z;
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
