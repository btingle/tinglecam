#include "gui_internal.h"
namespace tinglecam_gui {
#include "prism_view.h"
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
};

Paths64 area_outline(const cam::ClearingArea& area) {
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
        const cam::PocketParameters& pocket = operation.pocket;
        if (!(pocket.depth > 0.0)) continue;
        volumes.push_back({cam::pocket_boundary(pocket), pocket.top_z - pocket.depth,
                           pocket.top_z, selected});
      } else {
        const cam::PlaneParameters& plane = operation.plane;
        if (!(plane.depth > 0.0)) continue;
        for (const cam::ClearingArea& area : plane.areas) {
          if (!(area.width > 0.0) || !(area.height > 0.0)) continue;
          volumes.push_back({area_outline(area), plane.top_z - plane.depth,
                             plane.top_z, selected});
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
    const float wall[3]{0.86f, 0.26f, 0.24f};
    const float face[3]{0.95f, 0.42f, 0.36f};
    const int alpha = volume.selected ? 110 : 60;
    append_slab(facets, pool, rings, fill_mesh(rings, nullptr), rings,
                volume.bottom, volume.top, projection, light, wall, face,
                IM_COL32(255, 140, 130, volume.selected ? 220 : 120), alpha,
                wireframe);
  }
}

// ---------------------------------------------------------------------------
// Stage 1: the models, before there is a plan to look at.

// The Z range the operation will actually cut, so the depth range being set can
// be seen against the models rather than read off two numbers.
void draw_setup(App& app) {
  OrbitScene scene;
  begin_scene(app, scene, "setup");
  // Dimension-driven operations need no mesh, so this view cannot insist on
  // one. Where there is nothing loaded it shows the volumes on their own.
  const std::vector<CutVolume> volumes = cut_volumes(app);
  const bool models_loaded =
      app.geometry.has_part() && app.geometry.has_stock();
  double low = 0.0;
  double high = 0.0;
  const bool has_range = models_loaded && app.geometry.work_range(low, high);
  if (!has_range && volumes.empty()) {
    scene_message(scene, models_loaded
                             ? "The models have no depth to cut"
                             : "Load a part and a stock model to inspect them");
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

  const cam::Operation* operation =
      app.job.operations.empty() ? nullptr
                                 : &app.job.operations[std::clamp(
                                       app.selected_operation, 0,
                                       static_cast<int>(app.job.operations.size()) - 1)];

  const bool draw_stock = models_loaded && app.models != ModelShow::Part;
  const bool draw_part = models_loaded && app.models != ModelShow::Stock;
  // A rectangular blank has no mesh behind it, so it is drawn as the single
  // prism it is defined to be rather than pretending to a model it never had.
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
  if (draw_part) cover_mesh(app.geometry.part().mesh());
  if (draw_stock) {
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
    scene_message(scene, "Neither model has anything to show");
    end_scene(scene);
    return;
  }
  aim_scene(app, scene, extent, low, high);

  // Both bodies whole, and the stock made see-through when both are on. Cutting
  // a part-shaped hole in the stock instead gave every wall of the part an
  // exact coincident twin on the hole's inside for the depth sort to pick
  // between; letting the two solids simply overlap has no coincidence in it.
  if (draw_part) {
    const float colour[3]{0.42f, 0.60f, 0.80f};
    append_mesh(scene.facets, scene.pool, app.geometry.part().mesh(),
                scene.projection, scene.light, colour,
                app.wireframe ? 190 : 255, app.wireframe);
  }
  if (draw_stock) {
    const float colour[3]{0.34f, 0.38f, 0.45f};
    const int alpha = app.wireframe ? 150 : (draw_part ? 90 : 255);
    if (boxed) {
      const wave::Paths64 rings = drawable_region(box, scene.spacing);
      if (!rings.empty()) {
        const float face[3]{0.40f, 0.44f, 0.52f};
        append_slab(scene.facets, scene.pool, rings, fill_mesh(rings, nullptr),
                    rings, low, high, scene.projection, scene.light, colour,
                    face, IM_COL32(160, 180, 205, 70), alpha, app.wireframe);
      }
    } else {
      append_mesh(scene.facets, scene.pool, app.geometry.stock().mesh(),
                  scene.projection, scene.light, colour, alpha, app.wireframe);
    }
  }
  paint_scene(scene);
  // After the bodies and sorted on their own, for the same reason the stacked
  // view puts its models over the passes: a cut volume's wall and the blank
  // wall it is flush with have no honest answer between them in a per-facet
  // depth sort, and the two speckle as the view turns. The volume is a
  // statement about the bodies rather than another body among them.
  if (!volumes.empty()) {
    std::vector<Facet> cut_facets;
    std::vector<ImVec2> cut_pool;
    append_cut_volumes(volumes, cut_facets, cut_pool, scene.projection,
                       scene.light, scene.spacing, app.wireframe);
    paint_facets(scene.list, cut_facets, cut_pool);
  }
  draw_origin_gizmo(app, scene);

  // One button cycling the three, rather than two checkboxes that can both be
  // off and leave the view empty.
  if (scene_button(scene, model_show_name(app.models), 10.0f))
    app.models = app.models == ModelShow::Both      ? ModelShow::Stock
                 : app.models == ModelShow::Stock   ? ModelShow::Part
                                                    : ModelShow::Both;
  // Under the first rather than beside it. The caption runs along the top of
  // every canvas and a second button on that line reaches back into it.
  if (scene_button(scene, app.wireframe ? "Wireframe" : "Solid", 10.0f,
                   8.0f + ImGui::GetFrameHeight() +
                       ImGui::GetStyle().ItemSpacing.y))
    app.wireframe = !app.wireframe;
  view_cube(app, scene.list, scene.top_left, scene.size, scene.hovered);

  char caption[260];
  std::snprintf(caption, sizeof caption,
                "%s, Z %.2f to %.2f mm   Drag: orbit   Right-drag: pan   Wheel: "
                "zoom",
                models_loaded ? "Part and stock" : "Cutting volumes", low, high);
  scene.list->AddText({scene.top_left.x + 12.0f, scene.top_left.y + 10.0f},
                      IM_COL32(130, 142, 158, 255), caption);
  if (operation) {
    std::snprintf(caption, sizeof caption,
                  "This operation cuts depth %.2f to %.2f mm (Z %.2f to %.2f)",
                  operation->depth_from, operation->depth_to,
                  cam::z_at_depth(high, operation->depth_to),
                  cam::z_at_depth(high, operation->depth_from));
    scene.list->AddText({scene.top_left.x + 12.0f, scene.top_left.y + 28.0f},
                        IM_COL32(120, 165, 210, 255), caption);
  }
  end_scene(scene);
}

// ---------------------------------------------------------------------------
// The loaded models, as a translucent sheet over a stack that has already been
// painted. Into a facet list of their own, depth sorted on their own, and drawn
// after: they are an overlay, not another body in the scene.
//
// Sorting them together with the passes was the obvious thing and is wrong. A
// simulated pass wall and the part surface it was cut to sit in the same place
// to within a rounding, so the sort has no real answer for which is in front
// and picks per facet on centroid depth -- the two then interleave, and the
// overlay comes out speckled and crawls as the view turns. Nothing in the
// question the overlay answers needs the model to be occluded by the stack, so
// the fix is to stop asking: it goes on top, and being see-through is what lets
// the stack be read through it.
//
// The part is the more opaque of the two: it is the shape being cut towards,
// and the blank is mostly a boundary.
//
// Warm, where every pass in the stack is cool. The plan view's blue would be
// the consistent choice and is the wrong one here: it is within a shade of the
// colour the passes are already drawn in, and the whole question this overlay
// answers is which of the two a given surface belongs to.
void append_models(const App& app, StackModels which, std::vector<Facet>& facets,
                   std::vector<ImVec2>& pool, const Projection& projection,
                   const Vec3& light, double spacing, int slice_grid) {
  if (which == StackModels::None) return;
  const bool want_part = which != StackModels::Stock;
  const bool want_stock = which != StackModels::Part;

  if (want_part && app.geometry.has_part()) {
    const float colour[3]{0.82f, 0.62f, 0.34f};
    append_mesh(facets, pool, app.geometry.part().mesh(), projection, light,
                colour, 95);
  }
  if (!want_stock || !app.geometry.has_stock()) return;
  const float colour[3]{0.34f, 0.38f, 0.45f};
  if (!app.geometry.box_stock()) {
    append_mesh(facets, pool, app.geometry.stock().mesh(), projection, light,
                colour, 55);
    return;
  }
  // A rectangular blank has no mesh behind it, so it is the prism it is defined
  // to be, exactly as the plan view draws it.
  double low = 0.0;
  double high = 0.0;
  if (!app.geometry.work_range(low, high)) return;
  const Paths64 rings =
      drawable_region(app.geometry.stock_outline(low, slice_grid), spacing);
  if (rings.empty()) return;
  const float face[3]{0.40f, 0.44f, 0.52f};
  append_slab(facets, pool, rings, fill_mesh(rings, nullptr), rings, low, high,
              projection, light, colour, face, IM_COL32(160, 180, 205, 60), 55);
}

void draw_result(App& app) {
  ImGui::BeginChild("result", ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  const ImVec2 top_left = ImGui::GetCursorScreenPos();
  ImVec2 size = ImGui::GetContentRegionAvail();
  size.x = std::max(size.x, 50.0f);
  size.y = std::max(size.y, 50.0f);
  // The overlay control sits over this canvas, so the canvas has to let a later
  // item take the mouse where the two meet, or orbiting starts under it.
  ImGui::SetNextItemAllowOverlap();
  ImGui::InvisibleButton("stack", size,
                         ImGuiButtonFlags_MouseButtonLeft |
                             ImGuiButtonFlags_MouseButtonRight);
  const bool hovered = ImGui::IsItemHovered();
  ImDrawList* list = ImGui::GetWindowDrawList();
  list->AddRectFilled(top_left, ImVec2(top_left.x + size.x, top_left.y + size.y),
                      IM_COL32(14, 17, 22, 255));
  list->PushClipRect(top_left, ImVec2(top_left.x + size.x, top_left.y + size.y), true);

  orbit_controls(app, hovered && !app.orbit.cube_pressed);

  const auto finish = [&](const char* message) {
    const ImVec2 extent = ImGui::CalcTextSize(message);
    list->AddText({top_left.x + (size.x - extent.x) * 0.5f,
                   top_left.y + (size.y - extent.y) * 0.5f},
                  IM_COL32(120, 130, 145, 255), message);
    list->PopClipRect();
    ImGui::EndChild();
  };

  if (!app.has_plan || app.plan.operations.empty()) {
    finish("Plan the job to see the result");
    return;
  }
  const auto& planned_operation =
      app.plan.operations[app.selected_plan_operation];
  const auto& levels = planned_operation.levels;
  if (levels.empty()) {
    finish(planned_operation.planned ? "This operation has no passes"
                                     : "This operation has not been planned yet");
    return;
  }

  const int active_level = std::clamp(
      app.selected_level, 0, static_cast<int>(levels.size()) - 1);
  const cam::LevelPlan& active = levels[active_level];
  const cam::Tool* active_tool = app.tools.find(planned_operation.tool_id);
  std::optional<cam::MotionCursor> cutter_cursor;
  if (active_tool != nullptr && !active.toolpath.moves.empty()) {
    if (!active.feeds.motion.empty()) {
      cutter_cursor = cam::feed_motion_at_seconds(
          active.feeds, app.playback.motion_seconds);
    } else {
      const wave::ToolpathCursor planar =
          wave::toolpath_at(active.toolpath, app.playback.distance);
      cutter_cursor = cam::MotionCursor{planar.position, active.floor_z,
                                        app.playback.distance, planar.kind,
                                        planar.kind == wave::MoveKind::Retract};
    }
  }

  // Bounds over the whole operation: every pass's stock and every pass's
  // toolpath, plus room above for the flute. Deliberately nothing that moves.
  // Framing the live cutter kept the tool centred and swam the model around it
  // on every frame, which made the stack impossible to read.
  Clipper2Lib::Rect64 extent_2d;
  bool any = false;
  double lowest = 0.0;
  double highest = 0.0;
  for (const auto& level : levels) {
    if (level.stock_slice.empty()) continue;
    const auto bounds = Clipper2Lib::GetBounds(level.stock_slice);
    if (!any) {
      extent_2d = bounds;
      lowest = level.floor_z;
      highest = level.floor_z + level.axial_depth;
      any = true;
    } else {
      extent_2d.left = std::min(extent_2d.left, bounds.left);
      extent_2d.right = std::max(extent_2d.right, bounds.right);
      extent_2d.top = std::min(extent_2d.top, bounds.top);
      extent_2d.bottom = std::max(extent_2d.bottom, bounds.bottom);
      lowest = std::min(lowest, level.floor_z);
      highest = std::max(highest, level.floor_z + level.axial_depth);
    }
  }
  if (planned_operation.composed_stock) {
    const auto include = [&](const cam::StockSlab& slab) {
      if (slab.region->empty()) return;
      const auto b = Clipper2Lib::GetBounds(*slab.region);
      if (!any) { extent_2d = b; lowest=slab.bottom; highest=slab.top; any=true; }
      else {
        extent_2d.left=std::min(extent_2d.left,b.left); extent_2d.right=std::max(extent_2d.right,b.right);
        extent_2d.top=std::min(extent_2d.top,b.top); extent_2d.bottom=std::max(extent_2d.bottom,b.bottom);
        lowest=std::min(lowest,slab.bottom); highest=std::max(highest,slab.top);
      }
    };
    for (const auto& slab : planned_operation.preview_context) include(slab);
    for (const auto& level : levels) for (const auto& slab : level.local_stock) include(slab);
  }
  const double material_lowest = lowest;
  double material_highest = highest;
  if (!planned_operation.subdivisions.empty() &&
      planned_operation.subdivisions.front().level_count > 0) {
    const std::size_t first =
        planned_operation.subdivisions.front().first_level;
    if (first < levels.size())
      material_highest = levels[first].floor_z + levels[first].axial_depth;
  }
  // Everywhere the cutter is ever commanded to be, with room for its flute,
  // taken once from the plan rather than from where it happens to stand this
  // frame. A lead-in and a retract both reach outside the stock, so the stack
  // alone is not enough to keep the tool in shot.
  if (any && active_tool != nullptr) {
    const std::int64_t radius = wave::to_integer(active_tool->diameter * 0.5);
    for (const auto& level : levels) {
      for (const cam::MotionPhase& phase : level.feeds.motion) {
        for (const Point64& point : {phase.from, phase.to}) {
          extent_2d.left = std::min(extent_2d.left, point.x - radius);
          extent_2d.right = std::max(extent_2d.right, point.x + radius);
          extent_2d.top = std::min(extent_2d.top, point.y - radius);
          extent_2d.bottom = std::max(extent_2d.bottom, point.y + radius);
        }
        lowest = std::min({lowest, phase.from_z, phase.to_z});
        highest = std::max({highest,
                            std::max(phase.from_z, phase.to_z) +
                                active_tool->flute_length});
      }
    }
  }
  if (!any) {
    finish("Nothing was cut in this operation");
    return;
  }

  const double width = wave::to_world(extent_2d.right - extent_2d.left);
  const double height = wave::to_world(extent_2d.bottom - extent_2d.top);
  const double span = std::max({width, height, highest - lowest, 1e-3});
  const Vec3 centre{
      static_cast<float>(wave::to_world(extent_2d.left + extent_2d.right) * 0.5),
      static_cast<float>(wave::to_world(extent_2d.top + extent_2d.bottom) * 0.5),
      static_cast<float>((lowest + highest) * 0.5)};

  const Basis basis = orbit_basis(app.orbit.yaw, app.orbit.pitch);
  const Projection projection{
      {top_left.x + size.x * 0.5f + app.orbit.pan.x,
       top_left.y + size.y * 0.5f + app.orbit.pan.y},
      centre,
      basis.right,
      basis.up,
      basis.toward,
      static_cast<float>(std::min(size.x, size.y) * 0.72 / span) * app.orbit.zoom};

  // One vertex every thousandth of the part is plenty at any sane zoom, and it
  // keeps the number of quads in the hundreds rather than the tens of thousands.
  const double spacing = std::max(span / 400.0, 0.05);

  // A single light fixed to the eye, so the model stays lit however it is
  // orbited. Every surface here is horizontal or vertical, which is exactly the
  // case a flat colour cannot describe on its own.
  const Vec3 raw{projection.toward.x * 0.85f + projection.up.x * 0.35f +
                     projection.right.x * 0.35f,
                 projection.toward.y * 0.85f + projection.up.y * 0.35f +
                     projection.right.y * 0.35f,
                 projection.toward.z * 0.85f + projection.up.z * 0.35f +
                     projection.right.z * 0.35f};
  const float raw_length =
      std::max(1e-6f, std::sqrt(raw.x * raw.x + raw.y * raw.y + raw.z * raw.z));
  const Vec3 light{raw.x / raw_length, raw.y / raw_length, raw.z / raw_length};

  // Posting and playback keep subdivision groups in execution order. The
  // stack is a physical section, so interleave those groups by their actual Z
  // floors before building its slabs.
  std::vector<std::size_t> display_levels(levels.size());
  for (std::size_t i = 0; i < levels.size(); ++i) display_levels[i] = i;
  std::stable_sort(display_levels.begin(), display_levels.end(),
                   [&](std::size_t a, std::size_t b) {
                     return levels[a].floor_z > levels[b].floor_z;
                   });

  // Future subdivision layers deliberately alias the latest subdivision being
  // evaluated. Once subdivision 2 begins, subdivision 3's virtual layers stop
  // aliasing subdivision 1 and alias subdivision 2 instead. Only the selected
  // source layer ever needs a live, partially simulated stock object.
  const auto stock_for = [&](std::size_t target_index) -> const Paths64* {
    const cam::LevelPlan& target = levels[target_index];
    std::size_t source_index = target_index;
    if (app.stack_follows_playback)
      source_index = cam::subdivision_preview_source(
          planned_operation, active_level, target_index);
    const cam::LevelPlan& source = levels[source_index];
    const Paths64* remaining = &source.simulation.remaining_stock;
    if (source.nothing_to_cut) {
      remaining = &source.incoming_stock;
    } else if (app.stack_follows_playback &&
               source.subdivision == active.subdivision) {
      if (source.subdivision_level > active.subdivision_level) {
        remaining = &source.simulation.initial_stock;
      } else if (source.subdivision_level == active.subdivision_level) {
        remaining = &app.material;
      }
    }
    return remaining->empty() ? nullptr : remaining;
  };

  std::vector<const Paths64*> leftover(display_levels.size(), nullptr);
  for (std::size_t display = 0; display < display_levels.size(); ++display) {
    const std::size_t target = display_levels[display];
    if (!levels[target].stock_slice.empty()) leftover[display] = stock_for(target);
  }

  std::vector<Facet> facets;
  std::vector<ImVec2> pool;
  std::size_t untriangulated = 0;
  std::vector<cam::StockSlab> shader_stock;
  if (!planned_operation.composed_stock) {
    for (std::size_t i=0;i<display_levels.size();++i) {
      if (!leftover[i]) continue;
      const auto& level=levels[display_levels[i]];
      const double top=level.explicit_motion ? level.floor_z+level.axial_depth
          : (i==0 ? material_highest : levels[display_levels[i-1]].floor_z);
      if (top>level.floor_z) shader_stock.push_back({level.floor_z,top,
          std::make_shared<const Paths64>(*leftover[i])});
    }
  }
  if (planned_operation.composed_stock) {
    const int before = app.stack_follows_playback ? active_level : static_cast<int>(levels.size());
    if (app.stock_cache_operation != app.selected_plan_operation || app.stock_cache_level != before) {
      app.stock_cache = cam::preview_before_level(planned_operation, before);
      app.stock_cache_operation = app.selected_plan_operation;
      app.stock_cache_level = before;
    }
    auto stock = app.stock_cache;
    if (app.stack_follows_playback && !active.nothing_to_cut) {
      const auto removed = Clipper2Lib::Difference(active.incoming_stock, app.material, Clipper2Lib::FillRule::NonZero);
      stock = cam::stock_outside(stock, removed, active.floor_z, active.floor_z+active.axial_depth);
    }
    shader_stock=std::move(stock);
  }
  try {
    const GLuint texture=prism_view.render(shader_stock,projection,light,top_left,size,float(span*4));
    list->AddImage(static_cast<ImTextureID>(texture),top_left,
        ImVec2(top_left.x+size.x,top_left.y+size.y),ImVec2(0,1),ImVec2(1,0));
    if (prism_view.failed_layers())
      list->AddText(top_left,IM_COL32(255,150,80,255),"Some stock layers could not be triangulated; details in terminal.");
  } catch (const std::exception& error) {
    list->AddText(top_left,IM_COL32(255,90,90,255),error.what());
    std::fprintf(stderr,"Prism renderer: %s\n",error.what());
  }
  if (cutter_cursor && active_tool != nullptr) {
    append_cutter(
        facets, pool,
        Vec3{static_cast<float>(wave::to_world(cutter_cursor->position.x)),
             static_cast<float>(wave::to_world(cutter_cursor->position.y)),
             static_cast<float>(cutter_cursor->z)},
        static_cast<float>(active_tool->diameter * 0.5),
        static_cast<float>(cutter_cursor->z),
        static_cast<float>(cutter_cursor->z + active_tool->flute_length),
        projection, light);
  }
  paint_facets(list, facets, pool);

  // And the models over the finished stack, sorted among themselves alone.
  if (app.stack_models != StackModels::None) {
    std::vector<Facet> model_facets;
    std::vector<ImVec2> model_pool;
    append_models(app, app.stack_models, model_facets, model_pool, projection,
                  light, spacing,
                  app.job.operations.empty()
                      ? 32
                      : app.job.operations[std::clamp(
                                app.selected_operation, 0,
                                static_cast<int>(app.job.operations.size()) - 1)]
                            .slice_grid);
    paint_facets(list, model_facets, model_pool);
  }
  {
    const auto project = [&](const Point64& xy, double z) {
      return projection.point({static_cast<float>(wave::to_world(xy.x)),
                               static_cast<float>(wave::to_world(xy.y)),
                               static_cast<float>(z)});
    };
    // Dimension-driven jobs may have no mesh. Keep their requested boundaries
    // visible even after the last material has been removed from an area.
    if (active.explicit_motion)
      for (const auto& level : levels) {
        if (level.floor_z != active.floor_z) continue;
        for (const auto& ring : level.stock_slice)
          for (std::size_t i = 0; i < ring.size(); ++i)
            list->AddLine(project(ring[i], level.floor_z),
                          project(ring[(i+1)%ring.size()], level.floor_z),
                          IM_COL32(200, 175, 90, 180), 1.2f);
      }
    // Every operation carries its commanded XYZ timeline, parametric or not, so
    // the overlay is drawn from that rather than from the planar toolpath a
    // roughing pass also has. Read off the planar path the retracts and the
    // traversals at the safe plane are invisible, which is most of what there
    // is to see in three dimensions.
    for (const cam::MotionPhase& phase : active.feeds.motion) {
      if (phase.rapid ? !app.show_rapid_travel : !app.show_cutting_paths)
        continue;
      list->AddLine(project(phase.from, phase.from_z), project(phase.to, phase.to_z),
                    move_colour(phase.kind), phase.kind == wave::MoveKind::Entry ? 1.8f : 1.0f);
    }
  }
  // This view builds its own projection and canvas rather than going through
  // begin_scene, so the gizmo and the corner control are handed one made up on
  // the spot. Nothing else is ever read out of it.
  {
    OrbitScene overlay;
    overlay.list = list;
    overlay.projection = projection;
    overlay.top_left = top_left;
    overlay.size = size;
    draw_origin_gizmo(app, overlay);
    if (scene_button(overlay, stack_models_name(app.stack_models), 10.0f))
      app.stack_models = app.stack_models == StackModels::None ? StackModels::Part
                         : app.stack_models == StackModels::Part
                             ? StackModels::Stock
                         : app.stack_models == StackModels::Stock
                             ? StackModels::Both
                             : StackModels::None;
    view_cube(app, list, top_left, size, hovered);
  }

  char caption[200];
  if (planned_operation.subdivisions.size() > 1) {
    std::snprintf(
        caption, sizeof caption,
        "%zu layers from %zu subdivisions, Z %.2f to %.2f mm   Drag: orbit   "
        "Right-drag: pan   Wheel: zoom",
        levels.size(), planned_operation.subdivisions.size(), material_lowest,
        material_highest);
  } else {
    std::snprintf(
        caption, sizeof caption,
        "%zu passes stacked, Z %.2f to %.2f mm   Drag: orbit   Right-drag: pan   "
        "Wheel: zoom",
        levels.size(), material_lowest, material_highest);
  }
  list->AddText({top_left.x + 12.0f, top_left.y + 10.0f}, IM_COL32(130, 142, 158, 255),
                caption);
  if (untriangulated > 0) {
    std::snprintf(caption, sizeof caption,
                  "%zu layer%s could not be triangulated and is not drawn",
                  untriangulated, untriangulated == 1 ? "" : "s");
    list->AddText({top_left.x + 12.0f, top_left.y + 28.0f}, IM_COL32(232, 156, 92, 255),
                  caption);
  }

  list->PopClipRect();
  ImGui::EndChild();
}

// ---------------------------------------------------------------------------
}  // namespace tinglecam_gui
