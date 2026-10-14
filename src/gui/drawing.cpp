#include "gui_internal.h"
namespace tinglecam_gui {
// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

// Vertices far enough apart to be worth keeping. A ring straight out of an
// offset carries a vertex every few hundredths of a millimetre, and nothing
// downstream -- extrusion, triangulation, the fill itself -- gets anything for
// them but work.
Clipper2Lib::Path64 thinned(const Clipper2Lib::Path64& ring, double spacing) {
  Clipper2Lib::Path64 points;
  points.reserve(ring.size());
  for (const auto& vertex : ring) {
    if (!points.empty()) {
      const double dx = wave::to_world(vertex.x - points.back().x);
      const double dy = wave::to_world(vertex.y - points.back().y);
      if (dx * dx + dy * dy < spacing * spacing) continue;
    }
    points.push_back(vertex);
  }
  return points;
}

Paths64 thinned(const Paths64& paths, double spacing) {
  Paths64 out;
  out.reserve(paths.size());
  for (const auto& ring : paths) {
    Clipper2Lib::Path64 kept = thinned(ring, spacing);
    if (kept.size() >= 3) out.push_back(std::move(kept));
  }
  return out;
}

// A region ready to be drawn: decimated to what the view can resolve, then put
// back through Clipper. That second step is not optional. Decimation can leave
// a ring crossing itself on a tight concave feature, and only Clipper turns
// whatever comes out back into simple, correctly oriented, properly nested
// rings, which is what both the triangulator and the difference below need.
Paths64 drawable_region(const Paths64& region, double spacing) {
  return Clipper2Lib::Union(thinned(region, spacing), Clipper2Lib::FillRule::NonZero);
}

// A difference between two independently sampled versions of a nominally
// coincident curve can still contain a few integer-grid crumbs. They are far
// below the detail this view retains, but a mixture of tiny outer and hole
// rings can make the triangulator reject the whole otherwise-empty surface.
Paths64 drawable_surface(const Paths64& region, double spacing) {
  const double integer_scale2 = wave::kIntegerScale * wave::kIntegerScale;
  const double minimum_area = spacing * spacing * 0.25 * integer_scale2;
  Paths64 visible;
  visible.reserve(region.size());
  for (const auto& ring : region)
    if (std::abs(Clipper2Lib::Area(ring)) >= minimum_area)
      visible.push_back(ring);
  return drawable_region(visible, spacing);
}

// The same region cut into triangles by Clipper2's constrained Delaunay.
//
// Every filled thing in this program goes through here, for two reasons. Holes
// come out as holes, so a pocket needs no painting-over with the background
// colour -- a trick that only ever worked against a flat ground. And a region
// that cannot be triangulated is counted and reported, where an ear clipper
// handed a ring it cannot cut returns arbitrary triangles and says nothing.
Paths64 fill_mesh(const Paths64& region, std::size_t* failures) {
  Paths64 mesh;
  if (region.empty()) return mesh;
  const Clipper2Lib::TriangulateResult result = Clipper2Lib::Triangulate(region, mesh);
  if (failures != nullptr && result != Clipper2Lib::TriangulateResult::success &&
      result != Clipper2Lib::TriangulateResult::no_polygons)
    ++*failures;
  return mesh;
}

void draw_paths(ImDrawList* list, const Paths64& paths, const Transform& to_screen,
                ImU32 colour, float width) {
  for (const auto& ring : paths) {
    if (ring.size() < 2) continue;
    for (std::size_t i = 0; i < ring.size(); ++i)
      list->AddLine(to_screen(ring[i]), to_screen(ring[(i + 1) % ring.size()]), colour,
                    width);
  }
}

// Solid material, holes and all, as triangles. A void is absent from the mesh
// rather than painted over in the background colour, so material drawn on top
// of anything else still reads correctly.
void fill_paths(ImDrawList* list, const Paths64& paths, const Transform& to_screen,
                ImU32 solid, std::size_t* failures) {
  // One vertex per pixel and a half is all this view can show.
  const double spacing = 1.5 / std::max(to_screen.scale, 1e-6f);
  const Paths64 mesh = fill_mesh(drawable_region(paths, spacing), failures);
  for (const auto& triangle : mesh) {
    if (triangle.size() != 3) continue;
    list->AddTriangleFilled(to_screen(triangle[0]), to_screen(triangle[1]),
                            to_screen(triangle[2]), solid);
  }
}

ImU32 blend(ImU32 low, ImU32 high, float t) {
  const ImVec4 a = ImGui::ColorConvertU32ToFloat4(low);
  const ImVec4 b = ImGui::ColorConvertU32ToFloat4(high);
  t = std::clamp(t, 0.0f, 1.0f);
  return ImGui::ColorConvertFloat4ToU32(
      ImVec4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, 1.0f));
}

ImU32 engagement_colour(const wave::MaterialStep& step, int display) {
  double value = step.load;
  if (display == 1) value = step.area_load;
  if (display == 2) value = step.contact_load;
  if (display == 3)
    value = 10*std::abs(std::clamp(step.contact_load,0.0,1.0)-
                       std::clamp(step.area_load,0.0,1.0));
  return blend(IM_COL32(70,150,255,255),IM_COL32(255,80,70,255),static_cast<float>(value));
}

ImU32 limit_colour(cam::FeedLimit limit) {
  switch (limit) {
    case cam::FeedLimit::Engagement: return IM_COL32(90, 200, 140, 255);
    case cam::FeedLimit::Rapid: return IM_COL32(80, 95, 115, 255);
    case cam::FeedLimit::Plunge: return IM_COL32(255, 170, 60, 255);
  }
  return IM_COL32(200, 200, 200, 255);
}

ImU32 move_colour(wave::MoveKind kind) {
  switch (kind) {
    case wave::MoveKind::Cut: return IM_COL32(120, 220, 160, 255);
    case wave::MoveKind::Step: return IM_COL32(255, 190, 90, 255);
    case wave::MoveKind::Link: return IM_COL32(110, 180, 255, 255);
    case wave::MoveKind::Retract: return IM_COL32(90, 105, 130, 255);
    case wave::MoveKind::Entry: return IM_COL32(255, 100, 100, 255);
  }
  return IM_COL32(200, 200, 200, 255);
}

ImU32 chain_colour(std::size_t chain, std::size_t layer,
                   std::size_t layer_count, int alpha) {
  const float hue = std::fmod(static_cast<float>(chain) * 0.61803398875f, 1.0f);
  const float depth = layer_count <= 1
                          ? 0.0f
                          : static_cast<float>(layer) /
                                static_cast<float>(layer_count - 1);
  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;
  ImGui::ColorConvertHSVtoRGB(hue, 0.72f, 1.0f - depth * 0.28f, r, g, b);
  return IM_COL32(static_cast<int>(r * 255.0f),
                  static_cast<int>(g * 255.0f),
                  static_cast<int>(b * 255.0f), alpha);
}

const cam::LevelPlan* current_level(const App& app) {
  if (!app.has_plan) return nullptr;
  if (app.selected_plan_operation < 0 ||
      app.selected_plan_operation >= static_cast<int>(app.plan.operations.size()))
    return nullptr;
  const auto& planned_operation =
      app.plan.operations[app.selected_plan_operation];
  const auto& levels = planned_operation.levels;
  if (app.selected_level < 0 || app.selected_level >= static_cast<int>(levels.size()))
    return nullptr;
  return &levels[app.selected_level];
}

double level_length(const cam::LevelPlan* level) {
  return level ? wave::toolpath_length(level->toolpath) : 0.0;
}

void sync_playback_time(App& app) {
  const cam::LevelPlan* level = current_level(app);
  app.playback.motion_seconds =
      level ? cam::feed_seconds_to(level->feeds, app.playback.distance) : 0.0;
}

// The remaining stock for the pass being watched. Recomputed only when the
// playback has actually moved, since each call redoes the current move's
// booleans and there is no sense doing that for a still frame.
void refresh_material(App& app) {
  const cam::LevelPlan* level = current_level(app);
  if (!level || level->nothing_to_cut) {
    app.material.clear();
    app.material_distance = -1.0;
    return;
  }
  if (app.material_operation == app.selected_plan_operation &&
      app.material_level == app.selected_level &&
      std::abs(app.material_distance - app.playback.distance) < 1e-9)
    return;
  try {
    const bool advancing =
        app.material_operation == app.selected_plan_operation &&
        app.material_level == app.selected_level &&
        app.material_distance >= 0.0 &&
        app.playback.distance > app.material_distance;
    app.material = advancing
        ? wave::advance_simulation_stock(level->simulation, level->toolpath,
                                         app.material, app.material_distance,
                                         app.playback.distance)
        : wave::simulation_stock_at(level->simulation, level->toolpath,
                                    app.playback.distance);
  } catch (const std::exception&) {
    app.material = level->simulation.remaining_stock;
  }
  app.material_distance = app.playback.distance;
  app.material_level = app.selected_level;
  app.material_operation = app.selected_plan_operation;
}

// The segment covering a distance along the pass, or the end of the schedule.
// Segments are laid down in path order, so the search is a bisection on where
// each one starts.
std::size_t segment_at(const std::vector<cam::FeedSegment>& segments, double distance) {
  auto it = std::upper_bound(segments.begin(), segments.end(), distance,
                             [](double value, const cam::FeedSegment& segment) {
                               return value < segment.start_distance;
                             });
  if (it == segments.begin()) return 0;
  return static_cast<std::size_t>(std::distance(segments.begin(), it) - 1);
}

// m:ss for anything over a minute, seconds with a decimal below it. A pass is
// usually tens of seconds and a job is minutes, and one format reads badly for
// both.
std::string clock_text(double seconds) {
  char buffer[32];
  if (seconds < 60.0) std::snprintf(buffer, sizeof(buffer), "%.1fs", seconds);
  else std::snprintf(buffer, sizeof(buffer), "%d:%04.1f", static_cast<int>(seconds / 60.0),
                     std::fmod(seconds, 60.0));
  return buffer;
}

const char* motion_phase_name(const cam::MotionCursor& cursor) {
  if (cursor.to_z > cursor.from_z) return "Rapid retract";
  if (cursor.to_z < cursor.from_z)
    return cursor.rapid ? "Rapid descent" : "Plunge";
  if (cursor.rapid) return "Rapid traverse";
  return wave::move_kind_name(cursor.kind);
}

// Playback runs along one pass and then falls into the next, so the whole job
// can be watched without driving the slider by hand.
void advance_playback(App& app, double seconds) {
  if (!app.playback.playing || !app.has_plan) return;
  // Preserve the budget's unit across pass boundaries. Machine-time playback
  // carries seconds into the next pass; treating them as millimetres makes the
  // cutter jump whenever a frame crosses a boundary.
  double budget = app.playback.real_time
                      ? seconds * app.playback.time_scale
                      : seconds * app.playback.speed;
  while (budget > 0.0) {
    const cam::LevelPlan* level = current_level(app);
    if (!level) {
      app.playback.playing = false;
      return;
    }
    const double total = level_length(level);
    double left_in_pass = 0.0;
    if (app.playback.real_time) {
      left_in_pass = std::max(
          cam::feed_schedule_seconds(level->feeds) -
              app.playback.motion_seconds,
          0.0);
      if (budget < left_in_pass) {
        app.playback.motion_seconds += budget;
        app.playback.distance = cam::feed_distance_at_seconds(
            level->feeds, app.playback.motion_seconds);
        return;
      }
    } else {
      left_in_pass = std::max(total - app.playback.distance, 0.0);
      if (budget < left_in_pass) {
        app.playback.distance += budget;
        sync_playback_time(app);
        return;
      }
    }
    budget -= left_in_pass;
    app.playback.distance = total;
    app.playback.motion_seconds = cam::feed_schedule_seconds(level->feeds);

    if (!app.playback.follow) {
      app.playback.distance = 0.0;
      app.playback.motion_seconds = 0.0;
      return;
    }
    const auto& levels = app.plan.operations[app.selected_plan_operation].levels;
    int next = app.selected_level + 1;
    while (next < static_cast<int>(levels.size()) && levels[next].nothing_to_cut)
      ++next;
    if (next < static_cast<int>(levels.size())) {
      app.selected_level = next;
      app.playback.distance = 0.0;
      app.playback.motion_seconds = 0.0;
      refresh_material(app);
      continue;
    }
    int operation = app.selected_plan_operation + 1;
    while (operation < static_cast<int>(app.plan.operations.size())) {
      const auto& next_levels = app.plan.operations[operation].levels;
      int first_cut = 0;
      while (first_cut < static_cast<int>(next_levels.size()) &&
             next_levels[first_cut].nothing_to_cut)
        ++first_cut;
      if (first_cut < static_cast<int>(next_levels.size())) {
        app.selected_plan_operation = operation;
        app.selected_operation = operation;
        app.selected_level = first_cut;
        app.playback.distance = 0.0;
        app.playback.motion_seconds = 0.0;
        refresh_material(app);
        break;
      }
      ++operation;
    }
    if (operation >= static_cast<int>(app.plan.operations.size())) {
      app.playback.playing = false;
      return;
    }
  }
}

void fit_view(App& app, ImVec2 canvas) {
  const cam::LevelPlan* level = current_level(app);
  if (!level) return;
  // Everything the pass involves, not just the stock: a lead-in stands outside
  // the stock and a part can reach past it, and framing on the stock alone
  // crops both off the edge of the view.
  Paths64 reference = level->stock_slice;
  reference.insert(reference.end(), level->part_slice.begin(),
                   level->part_slice.end());
  for (const auto& segment : level->feeds.segments)
    reference.push_back(Clipper2Lib::Path64{segment.from, segment.to});
  // The pass opens where the previous one finished, which can be well outside
  // anything this one touches.
  for (const auto& phase : level->feeds.motion)
    if (phase.arrival)
      reference.push_back(Clipper2Lib::Path64{phase.from, phase.to});
  if (reference.empty()) return;
  const auto bounds = Clipper2Lib::GetBounds(reference);
  const float width = static_cast<float>(wave::to_world(bounds.right - bounds.left));
  const float height = static_cast<float>(wave::to_world(bounds.bottom - bounds.top));
  if (!(width > 0.0f) || !(height > 0.0f)) return;
  app.view.zoom = 0.85f * std::min(canvas.x / width, canvas.y / height);
  const float centre_x =
      static_cast<float>(wave::to_world(bounds.left + bounds.right)) * 0.5f;
  const float centre_y =
      static_cast<float>(wave::to_world(bounds.top + bounds.bottom)) * 0.5f;
  app.view.pan = {-centre_x * app.view.zoom, centre_y * app.view.zoom};
  app.view.needs_fit = false;
}

// The same marker for the flat pass view, which is one depth seen from above:
// two axes rather than three, since the third points at the reader.
void draw_origin_marker(const App& app, ImDrawList* list,
                        const Transform& to_screen) {
  const ImVec2 centre = to_screen(Point64(
      wave::to_integer(-app.job.origin_x), wave::to_integer(-app.job.origin_y)));
  static const ImVec2 kSteps[2] = {{46.0f, 0.0f}, {0.0f, -46.0f}};
  static const ImU32 kColours[2] = {IM_COL32(232, 96, 92, 255),
                                    IM_COL32(126, 205, 122, 255)};
  static const char* const kNames[2] = {"X", "Y"};
  for (int axis = 0; axis < 2; ++axis) {
    const ImVec2 end{centre.x + kSteps[axis].x, centre.y + kSteps[axis].y};
    list->AddLine(centre, end, IM_COL32(12, 14, 18, 200), 3.5f);
    list->AddLine(centre, end, kColours[axis], 1.7f);
    list->AddText({end.x + 3.0f, end.y - 6.0f}, kColours[axis], kNames[axis]);
  }
  list->AddCircleFilled(centre, 3.5f, IM_COL32(12, 14, 18, 220));
  list->AddCircleFilled(centre, 2.0f, IM_COL32(238, 240, 245, 255));
}

void draw_canvas(App& app) {
  ImGui::BeginChild("canvas", ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  const ImVec2 top_left = ImGui::GetCursorScreenPos();
  ImVec2 size = ImGui::GetContentRegionAvail();
  size.x = std::max(size.x, 50.0f);
  size.y = std::max(size.y, 50.0f);
  ImGui::InvisibleButton("plot", size,
                         ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
  const bool hovered = ImGui::IsItemHovered();
  ImDrawList* list = ImGui::GetWindowDrawList();
  const ImVec2 bottom_right{top_left.x + size.x, top_left.y + size.y};
  list->AddRectFilled(top_left, bottom_right, IM_COL32(16, 19, 25, 255));
  list->PushClipRect(top_left, bottom_right, true);

  if (app.view.needs_fit) fit_view(app, size);

  if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
    app.view.pan.x += ImGui::GetIO().MouseDelta.x;
    app.view.pan.y += ImGui::GetIO().MouseDelta.y;
  }
  if (hovered && ImGui::GetIO().MouseWheel != 0.0f) {
    const float factor = std::pow(1.12f, ImGui::GetIO().MouseWheel);
    app.view.zoom = std::clamp(app.view.zoom * factor, 0.02f, 4000.0f);
    app.view.pan.x *= factor;
    app.view.pan.y *= factor;
  }

  const Transform to_screen{
      {top_left.x + size.x * 0.5f + app.view.pan.x,
       top_left.y + size.y * 0.5f + app.view.pan.y},
      app.view.zoom};

  app.fill_failures = 0;

  const cam::LevelPlan* level = current_level(app);
  if (level) {
    // What is left of the stock, drawn as the solid it is rather than as an
    // outline. This is the whole point of simulating: the outline of a pocket
    // and the outline of an island look the same until one of them is filled.
    if (app.show_material && !app.material.empty())
      fill_paths(list, app.material, to_screen, IM_COL32(52, 78, 104, 255),
                 &app.fill_failures);
    if (app.show_stock)
      draw_paths(list, level->stock_slice, to_screen, IM_COL32(70, 130, 165, 200), 1.6f);
    if (app.show_part)
      draw_paths(list, level->part_slice, to_screen, IM_COL32(235, 200, 90, 220), 2.0f);

    const auto& operation = app.plan.operations[app.selected_plan_operation];
    const std::size_t layer_index = static_cast<std::size_t>(app.selected_level);
    if (layer_index < operation.inter_layer.layers.size()) {
      const wave::LayerTopology& topology =
          operation.inter_layer.layers[layer_index];
      if (app.node_debug_colouring != NodeDebugColouring::None &&
          layer_index < operation.inter_layer.node_chain_ids.size()) {
        const auto& ids = operation.inter_layer.node_chain_ids[layer_index];
        for (std::size_t node = 0; node < topology.nodes.size() && node < ids.size();
             ++node) {
          const bool joined = ids[node] != wave::kNoTopologyId;
          const ImU32 edge = app.node_debug_colouring ==
                                     NodeDebugColouring::Node
                                 ? chain_colour(node, 0, 1, 245)
                                 : (joined
                                        ? chain_colour(ids[node], layer_index,
                                                       operation.levels.size(), 245)
                                        : IM_COL32(145, 150, 165, 150));
          // Use the same fill path as simulated stock. Pre-blending into an
          // opaque dark colour avoids alpha accumulating on shared triangle
          // edges while leaving the toolpath legible above the node area.
          const ImU32 fill = blend(IM_COL32(16, 19, 25, 255), edge, 0.18f);
          fill_paths(list, topology.nodes[node].footprint, to_screen, fill,
                     &app.fill_failures);
          draw_paths(list, topology.nodes[node].footprint, to_screen, edge, 2.2f);
        }
      }
      if (app.show_entry_front_chains &&
          layer_index < operation.inter_layer.entry_front_chain_ids.size()) {
        const auto& ids = operation.inter_layer.entry_front_chain_ids[layer_index];
        for (std::size_t entry = 0;
             entry < topology.entry_fronts.size() && entry < ids.size(); ++entry) {
          const std::size_t id = topology.entry_fronts[entry];
          if (id >= level->lineage.entry_fronts.size()) continue;
          const auto& path = level->lineage.entry_fronts[id].path;
          std::vector<ImVec2> points;
          points.reserve(path.size());
          for (const Point64& point : path) points.push_back(to_screen(point));
          if (points.size() >= 2)
            list->AddPolyline(points.data(), static_cast<int>(points.size()),
                              ids[entry] == wave::kNoTopologyId
                                  ? IM_COL32(145, 150, 165, 150)
                                  : chain_colour(ids[entry], layer_index,
                                                 operation.levels.size(), 255),
                              ImDrawFlags_None, 4.0f);
        }
      }
      if (app.show_boundary_contacts) {
        for (const auto& generation : level->lineage.generations)
          for (const auto& front : generation.fronts)
            if (!front.closed)
              for (const auto& contact : front.boundary_contacts)
                if (contact.valid)
                  list->AddCircleFilled(to_screen(contact.point), 3.5f,
                                        IM_COL32(255, 110, 210, 245));
      }
    }

    double lowest = 0.0;
    double highest = 0.0;
    for (const auto& segment : level->feeds.segments) {
      if (segment.rapid) continue;
      highest = std::max(highest, segment.feed);
      lowest = lowest > 0.0 ? std::min(lowest, segment.feed) : segment.feed;
    }
    const double span = std::max(highest - lowest, 1e-6);

    // Where the cutter has got to, in the same units the segments carry.
    const double travelled = app.playback.distance;

    // The rapid in from the previous pass. It covers no path distance, so it
    // cannot be placed by `travelled` like the rest; whether it has happened is
    // a question about elapsed time.
    if (app.show_rapid_travel)
      for (const auto& phase : level->feeds.motion) {
        if (!phase.arrival) continue;
        const double duration = phase.length / (std::max(phase.rate, 1.0) / 60.0);
        const ImU32 colour = app.playback.motion_seconds >= duration
                                 ? IM_COL32(70, 82, 100, 255)
                                 : IM_COL32(70, 82, 100, 56);
        list->AddLine(to_screen(phase.from), to_screen(phase.to), colour, 1.0f);
      }

    double walked = 0.0;
    for (const auto& segment : level->feeds.segments) {
      const double segment_start = walked;
      walked += segment.length;
      if (segment.rapid ? !app.show_rapid_travel : !app.show_cutting_paths)
        continue;
      if (app.colouring == Colouring::Load && !level->explicit_motion &&
          !segment.rapid && segment.kind != wave::MoveKind::Entry) continue;
      // Anything still ahead of the cutter is drawn faintly, so the part of the
      // pass that has happened reads apart from the part that has not.
      const bool done = segment_start < travelled;
      ImU32 colour = IM_COL32(150, 150, 150, 255);
      float width = 1.7f;
      switch (app.colouring) {
        case Colouring::Feed:
          colour = segment.rapid
                       ? IM_COL32(70, 82, 100, 255)
                       : blend(IM_COL32(60, 110, 230, 255), IM_COL32(120, 235, 140, 255),
                               static_cast<float>((segment.feed - lowest) / span));
          break;
        case Colouring::Limit:
          colour = limit_colour(segment.limit);
          break;
        case Colouring::Load:
          colour = segment.rapid
                       ? IM_COL32(70, 82, 100, 255)
                       : blend(IM_COL32(70, 150, 255, 255), IM_COL32(255, 80, 70, 255),
                               static_cast<float>(segment.engagement));
          break;
        case Colouring::Move:
          colour = move_colour(segment.kind);
          break;
      }
      if (segment.rapid) width = 1.0f;
      if (!done) {
        ImVec4 faded = ImGui::ColorConvertU32ToFloat4(colour);
        faded.w = 0.22f;
        colour = ImGui::ColorConvertFloat4ToU32(faded);
      }
      list->AddLine(to_screen(segment.from), to_screen(segment.to), colour, width);
    }

    if (app.colouring == Colouring::Load && !level->explicit_motion && app.show_cutting_paths) {
      const bool comparison = level->simulation.method == wave::SimulationMethod::Compare ||
                              level->simulation.method == wave::SimulationMethod::Contact;
      double move_start = 0;
      for (std::size_t i=0; i<level->simulation.moves.size(); ++i) {
        for (const auto& step : level->simulation.moves[i].steps) {
          ImU32 colour = engagement_colour(step, comparison ? app.engagement_display : 0);
          if (move_start+step.move_distance_begin >= travelled) {
            auto faded = ImGui::ColorConvertU32ToFloat4(colour); faded.w = 0.22f;
            colour = ImGui::ColorConvertFloat4ToU32(faded);
          }
          list->AddLine(to_screen(step.from),to_screen(step.to),colour,1.7f);
        }
        move_start += level->toolpath.moves[i].length;
      }
    }

    // The cutter itself, at its true diameter, so an engagement that looks
    // reasonable as a line can be seen for what it is.
    const cam::Tool* tool = nullptr;
    for (const auto& operation : app.job.operations)
      if (operation.name ==
          app.plan.operations[app.selected_plan_operation].operation_name)
        tool = app.tools.find(operation.tool_id);
    // The cutter is the playback cursor rather than part of either path, so it
    // survives hiding one of them and goes only when both are off.
    if (tool && (app.show_cutting_paths || app.show_rapid_travel)) {
      const wave::Point64 position = level->feeds.motion.empty()
                                         ? wave::toolpath_at(
                                               level->toolpath,
                                               app.playback.distance)
                                               .position
                                         : cam::feed_motion_at_seconds(
                                               level->feeds,
                                               app.playback.motion_seconds)
                                               .position;
      const ImVec2 centre = to_screen(position);
      const float radius =
          static_cast<float>(tool->diameter * 0.5) * app.view.zoom;
      list->AddCircleFilled(centre, std::max(radius, 2.0f),
                            IM_COL32(255, 255, 255, 40));
      list->AddCircle(centre, std::max(radius, 2.0f), IM_COL32(255, 255, 255, 210), 0,
                      1.6f);
      list->AddCircleFilled(centre, 2.5f, IM_COL32(255, 255, 255, 235));
    }
  } else {
    const char* text = app.has_plan ? "This level has nothing to cut"
                                    : "Load a part and a stock model, then plan";
    const ImVec2 extent = ImGui::CalcTextSize(text);
    list->AddText({top_left.x + (size.x - extent.x) * 0.5f,
                   top_left.y + (size.y - extent.y) * 0.5f},
                  IM_COL32(120, 130, 145, 255), text);
  }

  draw_origin_marker(app, list, to_screen);

  if (app.fill_failures > 0) {
    char note[128];
    std::snprintf(note, sizeof note, "%zu region%s could not be triangulated",
                  app.fill_failures, app.fill_failures == 1 ? "" : "s");
    list->AddText({top_left.x + 12.0f, top_left.y + 10.0f}, IM_COL32(232, 156, 92, 255),
                  note);
  }

  list->PopClipRect();
  ImGui::EndChild();
}
}  // namespace tinglecam_gui
