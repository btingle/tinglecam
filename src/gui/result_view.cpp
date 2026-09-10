#include "gui_internal.h"
namespace tinglecam_gui {
// ---------------------------------------------------------------------------
// The stacked result
// ---------------------------------------------------------------------------

// The eye direction a pair of angles looks from, and the angles that look from
// a direction. Straight up and straight down leave the azimuth undetermined, so
// the caller's own is kept: asked for the top view, what an operator wants is
// this view from above, not this view from above and turned round as well.
void angles_from_direction(const Vec3& direction, float current_yaw,
                           float& yaw, float& pitch) {
  const float length = std::sqrt(direction.x * direction.x +
                                 direction.y * direction.y +
                                 direction.z * direction.z);
  if (!(length > 0.0f)) return;
  const Vec3 unit{direction.x / length, direction.y / length,
                  direction.z / length};
  pitch = std::asin(std::clamp(unit.z, -1.0f, 1.0f));
  yaw = std::hypot(unit.x, unit.y) < 1e-5f ? current_yaw
                                           : std::atan2(unit.y, unit.x);
}

#include "prism_view.h"
PrismView prism_view;

// A convex piece to paint, as a run of points in a shared pool. A frame builds
// tens of thousands of these, so they carry an offset rather than a vector of
// their own.
// A flat colour lit by how far a surface turns away from the light. Every face
// of the stack is either horizontal or vertical, so nothing else separates a
// wall from the lid above it, and without this the whole model reads as one
// silhouette.
ImU32 shaded(const float base[3], float lambert) {
  const float k = 0.42f + 0.58f * std::clamp(lambert, 0.0f, 1.0f);
  return IM_COL32(static_cast<int>(base[0] * k * 255.0f + 0.5f),
                  static_cast<int>(base[1] * k * 255.0f + 0.5f),
                  static_cast<int>(base[2] * k * 255.0f + 0.5f), 255);
}

ImU32 shaded_alpha(const float base[3], float lambert, int alpha) {
  const float k = 0.42f + 0.58f * std::clamp(lambert, 0.0f, 1.0f);
  return IM_COL32(static_cast<int>(base[0] * k * 255.0f + 0.5f),
                  static_cast<int>(base[1] * k * 255.0f + 0.5f),
                  static_cast<int>(base[2] * k * 255.0f + 0.5f), alpha);
}

// One pass's remaining stock, as the slab of material it stands for. Every pass
// is drawn as its own prism and the prisms stack, which is as close to the
// finished part as a set of two-and-a-half-dimensional passes can describe.
//
// `remaining` is what this pass leaves behind and gives the prism its walls;
// `lid_mesh` is the triangulated top surface it exposes, and `lid_rings` the
// same surface as outlines. Everything painted is convex: a wall quad is the
// projection of a vertical rectangle, and the lid arrives already cut into
// triangles, so no ear clipping happens at draw time.
void append_slab(std::vector<Facet>& facets, std::vector<ImVec2>& pool,
                 const Paths64& remaining, const Paths64& lid_mesh,
                 const Paths64& lid_rings, double bottom, double top,
                 const Projection& projection, const Vec3& light,
                 const float wall_colour[3], const float face_colour[3], ImU32 edge,
                 int alpha, bool wireframe) {
  const auto emit = [&](std::initializer_list<Vec3> corners, float depth_bias,
                        ImU32 fill, ImU32 outline) {
    Facet facet;
    facet.first = static_cast<std::uint32_t>(pool.size());
    facet.count = static_cast<std::uint32_t>(corners.size());
    float depth = 0.0f;
    for (const Vec3& corner : corners) {
      pool.push_back(projection.point(corner));
      depth += projection.depth(corner);
    }
    facet.depth = depth / static_cast<float>(corners.size()) + depth_bias;
    facet.fill = fill;
    facet.edge = outline;
    facets.push_back(facet);
  };

  for (const auto& ring : remaining) {
    if (ring.size() < 3) continue;
    for (std::size_t i = 0; i < ring.size(); ++i) {
      const Point64& a = ring[i];
      const Point64& b = ring[(i + 1) % ring.size()];
      // A wall is vertical, so its normal lies in the plane and follows the
      // edge. Taken this way round it points into open space for an outer ring
      // and into the pocket for a hole, which is the face we can see in both.
      const float dx = static_cast<float>(wave::to_world(b.x - a.x));
      const float dy = static_cast<float>(wave::to_world(b.y - a.y));
      const float length = std::sqrt(dx * dx + dy * dy);
      if (length < 1e-6f) continue;
      const Vec3 normal{dy / length, -dx / length, 0.0f};
      // A wireframe is a thing to see through, so the far wall stays.
      if (!wireframe &&
          normal.x * projection.toward.x + normal.y * projection.toward.y <= 0.0f)
        continue;  // turned away from the eye

      const Vec3 a_top{static_cast<float>(wave::to_world(a.x)),
                       static_cast<float>(wave::to_world(a.y)),
                       static_cast<float>(top)};
      const Vec3 b_top{static_cast<float>(wave::to_world(b.x)),
                       static_cast<float>(wave::to_world(b.y)),
                       static_cast<float>(top)};
      const Vec3 a_bottom{a_top.x, a_top.y, static_cast<float>(bottom)};
      const Vec3 b_bottom{b_top.x, b_top.y, static_cast<float>(bottom)};
      const ImU32 wall =
          shaded_alpha(wall_colour,
                       normal.x * light.x + normal.y * light.y +
                           normal.z * light.z,
                       alpha);
      emit({a_top, b_top, b_bottom, a_bottom}, 0.0f, wireframe ? 0 : wall,
           wireframe ? wall : 0);
    }
  }

  const ImU32 face = shaded_alpha(face_colour, light.z, alpha);
  // A wireframe keeps the rings below and skips the lid: a filled top surface
  // is the one thing that would stop the frame being see-through.
  if (!wireframe)
    for (const auto& triangle : lid_mesh) {
      if (triangle.size() != 3) continue;
      emit({Vec3{static_cast<float>(wave::to_world(triangle[0].x)),
                 static_cast<float>(wave::to_world(triangle[0].y)),
                 static_cast<float>(top)},
            Vec3{static_cast<float>(wave::to_world(triangle[1].x)),
                 static_cast<float>(wave::to_world(triangle[1].y)),
                 static_cast<float>(top)},
            Vec3{static_cast<float>(wave::to_world(triangle[2].x)),
                 static_cast<float>(wave::to_world(triangle[2].y)),
                 static_cast<float>(top)}},
           0.002f, face, 0);
    }

  // Outlined from the region rather than its triangles, so the interior edges
  // of the mesh do not show. Where a step faces away from the eye its wall is
  // culled and the outline is the only thing separating the levels either side.
  if (edge == 0) return;
  for (const auto& ring : lid_rings) {
    if (ring.size() < 3) continue;
    Facet outline;
    outline.first = static_cast<std::uint32_t>(pool.size());
    outline.count = static_cast<std::uint32_t>(ring.size());
    float depth = 0.0f;
    for (const Point64& vertex : ring) {
      const Vec3 at{static_cast<float>(wave::to_world(vertex.x)),
                    static_cast<float>(wave::to_world(vertex.y)),
                    static_cast<float>(top)};
      pool.push_back(projection.point(at));
      depth += projection.depth(at);
    }
    outline.depth = depth / static_cast<float>(ring.size()) + 0.004f;
    outline.fill = 0;
    outline.edge = edge;
    facets.push_back(outline);
  }
}

// A loaded model as the triangles it was given, rather than as the stack of
// prisms a slicer sees it as. Sampling the mesh at a fixed set of depths put a
// staircase on every curved face and littered the flat ones with the hairline
// slivers two independently decimated slices differ by -- neither of which is
// in the model. Nothing in this view is a plan, so nothing here has any reason
// to look at the models through a slicer.
//
// Faces turned away from the eye are dropped. Every model loaded here is a
// closed solid wound outward, so those are exactly the faces the rest of the
// solid stands in front of.
void append_mesh(std::vector<Facet>& facets, std::vector<ImVec2>& pool,
                 const wave::Mesh& mesh, const Projection& projection,
                 const Vec3& light, const float colour[3], int alpha,
                 bool wireframe) {
  facets.reserve(facets.size() + mesh.triangles.size() / 2);
  pool.reserve(pool.size() + mesh.triangles.size() * 3 / 2);
  for (const wave::Triangle3f& triangle : mesh.triangles) {
    const Vec3 corners[3]{{triangle.a.x, triangle.a.y, triangle.a.z},
                          {triangle.b.x, triangle.b.y, triangle.b.z},
                          {triangle.c.x, triangle.c.y, triangle.c.z}};
    const Vec3 u{corners[1].x - corners[0].x, corners[1].y - corners[0].y,
                 corners[1].z - corners[0].z};
    const Vec3 v{corners[2].x - corners[0].x, corners[2].y - corners[0].y,
                 corners[2].z - corners[0].z};
    Vec3 normal{u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z,
                u.x * v.y - u.y * v.x};
    const float length = std::sqrt(normal.x * normal.x + normal.y * normal.y +
                                   normal.z * normal.z);
    if (length < 1e-12f) continue;  // degenerate, and it has no normal to shade
    normal = {normal.x / length, normal.y / length, normal.z / length};
    // Back faces are dropped from a solid because the rest of it stands in
    // front of them. A wireframe has no front to stand in the way, and the far
    // side of the model is exactly what it is being asked for.
    if (!wireframe &&
        normal.x * projection.toward.x + normal.y * projection.toward.y +
            normal.z * projection.toward.z <= 0.0f)
      continue;

    Facet facet;
    facet.first = static_cast<std::uint32_t>(pool.size());
    facet.count = 3;
    float depth = 0.0f;
    for (const Vec3& corner : corners) {
      pool.push_back(projection.point(corner));
      depth += projection.depth(corner);
    }
    facet.depth = depth / 3.0f;
    const ImU32 shade = shaded_alpha(colour,
                                     normal.x * light.x + normal.y * light.y +
                                         normal.z * light.z,
                                     alpha);
    facet.fill = wireframe ? 0 : shade;
    facet.edge = wireframe ? shade : 0;
    facets.push_back(facet);
  }
}

// One entry front, as a vertical ribbon along the curve it actually occupies.
// A front is a stretch of wall the wave starts from, not a volume: giving it
// thickness put faces inside the stock for the depth sort to fight over, and
// said something about the material that is not what a front is.
//
// A ribbon has no inside, so both faces are lit and neither is culled. Where a
// lead-in stands the front off the stock, the ribbon stands off with it, which
// is where the pass view draws the same curve.
void append_ribbon(std::vector<Facet>& facets, std::vector<ImVec2>& pool,
                   const wave::Path64& path, bool closed, double bottom, double top,
                   const Projection& projection, const Vec3& light,
                   const float colour[3], int alpha) {
  if (path.size() < 2) return;
  const std::size_t segments = closed ? path.size() : path.size() - 1;
  for (std::size_t i = 0; i < segments; ++i) {
    const Point64& a = path[i];
    const Point64& b = path[(i + 1) % path.size()];
    const float dx = static_cast<float>(wave::to_world(b.x - a.x));
    const float dy = static_cast<float>(wave::to_world(b.y - a.y));
    const float length = std::sqrt(dx * dx + dy * dy);
    if (length < 1e-6f) continue;
    const float nx = dy / length;
    const float ny = -dx / length;
    const float lambert = std::abs(nx * light.x + ny * light.y);

    const Vec3 a_top{static_cast<float>(wave::to_world(a.x)),
                     static_cast<float>(wave::to_world(a.y)),
                     static_cast<float>(top)};
    const Vec3 b_top{static_cast<float>(wave::to_world(b.x)),
                     static_cast<float>(wave::to_world(b.y)),
                     static_cast<float>(top)};
    const Vec3 a_bottom{a_top.x, a_top.y, static_cast<float>(bottom)};
    const Vec3 b_bottom{b_top.x, b_top.y, static_cast<float>(bottom)};

    Facet facet;
    facet.first = static_cast<std::uint32_t>(pool.size());
    facet.count = 4;
    float depth = 0.0f;
    for (const Vec3& corner : {a_top, b_top, b_bottom, a_bottom}) {
      pool.push_back(projection.point(corner));
      depth += projection.depth(corner);
    }
    // Biased towards the eye so a front lying flat on the stock wall, which is
    // what one with no lead-in does, is drawn over it rather than into it.
    facet.depth = depth * 0.25f + 0.05f;
    facet.fill = shaded_alpha(colour, lambert, alpha);
    facet.edge = 0;
    facets.push_back(facet);
  }
}

// The active cutter in the stacked result. It is intentionally translucent:
// the point of this view is to see both where the tool is and the stock it is
// passing through. Side quads, a top cap, and the two silhouette rails make it
// read as a cylinder under the same orthographic projection as the slabs.
void append_cutter(std::vector<Facet>& facets, std::vector<ImVec2>& pool,
                   const Vec3& centre, float radius, float bottom, float top,
                   const Projection& projection, const Vec3& light) {
  constexpr std::size_t kSides = 40;
  constexpr float kPi = 3.14159265358979323846f;
  const float colour[3] = {0.48f, 0.86f, 1.0f};

  const auto point = [&](std::size_t side, float z) {
    const float angle = 2.0f * kPi * static_cast<float>(side) /
                        static_cast<float>(kSides);
    return Vec3{centre.x + radius * std::cos(angle),
                centre.y + radius * std::sin(angle), z};
  };
  const auto emit = [&](std::initializer_list<Vec3> corners, float depth_bias,
                        ImU32 fill, ImU32 edge) {
    Facet facet;
    facet.first = static_cast<std::uint32_t>(pool.size());
    facet.count = static_cast<std::uint32_t>(corners.size());
    float depth = 0.0f;
    for (const Vec3& corner : corners) {
      pool.push_back(projection.point(corner));
      depth += projection.depth(corner);
    }
    facet.depth = depth / static_cast<float>(corners.size()) + depth_bias;
    facet.fill = fill;
    facet.edge = edge;
    facets.push_back(facet);
  };

  for (std::size_t side = 0; side < kSides; ++side) {
    const std::size_t next = (side + 1) % kSides;
    const float angle = 2.0f * kPi *
                        (static_cast<float>(side) + 0.5f) /
                        static_cast<float>(kSides);
    const Vec3 normal{std::cos(angle), std::sin(angle), 0.0f};
    const float facing = normal.x * projection.toward.x +
                         normal.y * projection.toward.y;
    const int alpha = facing > 0.0f ? 54 : 18;
    emit({point(side, top), point(next, top), point(next, bottom),
          point(side, bottom)},
         0.0f,
         shaded_alpha(colour,
                      normal.x * light.x + normal.y * light.y,
                      alpha),
         0);

    // The visible top is a triangle fan. Separate triangles do not overlap, so
    // their alpha remains uniform while still fitting the convex-facet renderer.
    emit({Vec3{centre.x, centre.y, top}, point(side, top), point(next, top)},
         0.003f, shaded_alpha(colour, light.z, 42), 0);
  }

  const ImU32 outline = IM_COL32(165, 226, 250, 175);
  for (const float z : {bottom, top}) {
    Facet ring;
    ring.first = static_cast<std::uint32_t>(pool.size());
    ring.count = kSides;
    float depth = 0.0f;
    for (std::size_t side = 0; side < kSides; ++side) {
      const Vec3 at = point(side, z);
      pool.push_back(projection.point(at));
      depth += projection.depth(at);
    }
    ring.depth = depth / static_cast<float>(kSides) + 0.006f;
    ring.edge = outline;
    facets.push_back(ring);
  }

  const float view_angle =
      std::atan2(projection.toward.y, projection.toward.x);
  for (const float angle : {view_angle - kPi * 0.5f,
                            view_angle + kPi * 0.5f}) {
    const Vec3 lower{centre.x + radius * std::cos(angle),
                     centre.y + radius * std::sin(angle), bottom};
    const Vec3 upper{lower.x, lower.y, top};
    emit({lower, upper}, 0.007f, 0, outline);
  }
}

// The parts of an orbited stack view that say nothing about what is being
// drawn: the canvas, the mouse, the projection, the light, and the painter's
// sort. Three views share them and differ only in the slabs they contribute.
// Left-drag orbits, right-drag pans, wheel zooms. One reader for every 3D view,
// so the gesture means the same thing in all of them and a view added later
// cannot quietly disagree.
//
// The pan is in pixels and is not divided by the zoom: the drag moves what is
// under the pointer by the distance the pointer moved, which is what makes it
// feel like dragging the model rather than steering a camera at it.
void orbit_controls(App& app, bool hovered) {
  if (!hovered) return;
  const ImGuiIO& io = ImGui::GetIO();
  if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
    // Left and right turn the azimuth, which has no limit; up and down raise
    // and lower the elevation, which stops at the poles because past them the
    // model would come back upside down and the azimuth would have reversed.
    app.orbit.turning = false;  // a drag is the user taking the camera back
    app.orbit.yaw += io.MouseDelta.x * 0.009f;
    app.orbit.pitch = std::clamp(app.orbit.pitch - io.MouseDelta.y * 0.009f,
                                 -kQuarterTurn, kQuarterTurn);
  }
  if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
    app.orbit.pan.x += io.MouseDelta.x;
    app.orbit.pan.y += io.MouseDelta.y;
  }
  if (io.MouseWheel != 0.0f) {
    const float before = app.orbit.zoom;
    app.orbit.zoom =
        std::clamp(before * std::pow(1.12f, io.MouseWheel), 0.1f, 40.0f);
    // Scaled by however much the zoom actually moved, clamp included, which
    // makes the wheel scale the picture about the middle of the canvas. Left
    // alone, the view expands about a centre that panning has pushed off the
    // edge, and zooming in after panning throws the model out of shot. The 2D
    // pass view has always done this; these now agree with it.
    const float factor = before > 0.0f ? app.orbit.zoom / before : 1.0f;
    app.orbit.pan.x *= factor;
    app.orbit.pan.y *= factor;
  }
}

// Opens the child window and reads the orbit controls. Returns false when the
// view has nothing to show, having already drawn `empty_message` and closed up.
bool begin_scene(App& app, OrbitScene& scene, const char* id) {
  ImGui::BeginChild(id, ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  scene.top_left = ImGui::GetCursorScreenPos();
  scene.size = ImGui::GetContentRegionAvail();
  scene.size.x = std::max(scene.size.x, 50.0f);
  scene.size.y = std::max(scene.size.y, 50.0f);
  // A view may put a control over its own canvas, so the canvas has to let a
  // later item take the mouse where the two overlap.
  ImGui::SetNextItemAllowOverlap();
  ImGui::InvisibleButton(id, scene.size,
                         ImGuiButtonFlags_MouseButtonLeft |
                             ImGuiButtonFlags_MouseButtonRight);
  scene.hovered = ImGui::IsItemHovered();
  scene.list = ImGui::GetWindowDrawList();
  const ImVec2 bottom_right{scene.top_left.x + scene.size.x,
                            scene.top_left.y + scene.size.y};
  scene.list->AddRectFilled(scene.top_left, bottom_right, IM_COL32(14, 17, 22, 255));
  scene.list->PushClipRect(scene.top_left, bottom_right, true);
  orbit_controls(app, scene.hovered && !app.orbit.cube_pressed);
  return true;
}

void scene_message(OrbitScene& scene, const char* message) {
  const ImVec2 extent = ImGui::CalcTextSize(message);
  scene.list->AddText({scene.top_left.x + (scene.size.x - extent.x) * 0.5f,
                       scene.top_left.y + (scene.size.y - extent.y) * 0.5f},
                      IM_COL32(120, 130, 145, 255), message);
}

void end_scene(OrbitScene& scene) {
  scene.list->PopClipRect();
  ImGui::EndChild();
}

// A control in the corner of a view, declared after the canvas so it takes the
// mouse where the two overlap and orbiting does not start under the pointer.
// Returns true when it was pressed.
bool scene_button(OrbitScene& scene, const char* label, float from_right,
                  float from_top) {
  const ImVec2 saved = ImGui::GetCursorScreenPos();
  const float width = ImGui::CalcTextSize(label).x +
                      ImGui::GetStyle().FramePadding.x * 2.0f;
  ImGui::SetCursorScreenPos({scene.top_left.x + scene.size.x - width - from_right,
                             scene.top_left.y + from_top});
  const bool pressed = ImGui::Button(label);
  if (ImGui::IsItemHovered()) scene.hovered = false;
  // Back inside the canvas rather than to where the canvas left it: that was
  // one item's spacing past the bottom, and putting the cursor there grows the
  // parent, which ImGui rightly complains about.
  (void)saved;
  ImGui::SetCursorScreenPos(scene.top_left);
  return pressed;
}

// Frames the view on a fixed world box. Nothing that moves may contribute to
// it, or the model swims about under the thing being watched.
void aim_scene(App& app, OrbitScene& scene, const Clipper2Lib::Rect64& extent,
               double lowest, double highest) {
  const double width = wave::to_world(extent.right - extent.left);
  const double height = wave::to_world(extent.bottom - extent.top);
  const double span = std::max({width, height, highest - lowest, 1e-3});
  const Vec3 centre{
      static_cast<float>(wave::to_world(extent.left + extent.right) * 0.5),
      static_cast<float>(wave::to_world(extent.top + extent.bottom) * 0.5),
      static_cast<float>((lowest + highest) * 0.5)};
  const Basis basis = orbit_basis(app.orbit.yaw, app.orbit.pitch);
  scene.projection = Projection{
      {scene.top_left.x + scene.size.x * 0.5f + app.orbit.pan.x,
       scene.top_left.y + scene.size.y * 0.5f + app.orbit.pan.y},
      centre,
      basis.right,
      basis.up,
      basis.toward,
      static_cast<float>(std::min(scene.size.x, scene.size.y) * 0.72 / span) *
          app.orbit.zoom};

  // One vertex every thousandth of the part is plenty at any sane zoom, and it
  // keeps the number of quads in the hundreds rather than the tens of thousands.
  scene.spacing = std::max(span / 400.0, 0.05);

  // A single light fixed to the eye, so the model stays lit however it is
  // orbited. Every surface here is horizontal or vertical, which is exactly the
  // case a flat colour cannot describe on its own.
  const Vec3 raw{scene.projection.toward.x * 0.85f + scene.projection.up.x * 0.35f +
                     scene.projection.right.x * 0.35f,
                 scene.projection.toward.y * 0.85f + scene.projection.up.y * 0.35f +
                     scene.projection.right.y * 0.35f,
                 scene.projection.toward.z * 0.85f + scene.projection.up.z * 0.35f +
                     scene.projection.right.z * 0.35f};
  const float raw_length =
      std::max(1e-6f, std::sqrt(raw.x * raw.x + raw.y * raw.y + raw.z * raw.z));
  scene.light = Vec3{raw.x / raw_length, raw.y / raw_length, raw.z / raw_length};
}

// Painter's algorithm, then straight into the draw list.
// Depth sorts one body of facets and paints it. A body at a time rather than
// everything at once, so a caller with two of them can decide that one stands
// in front of the other outright instead of interleaving the two sorts.
void paint_facets(ImDrawList* list, std::vector<Facet>& facets,
                  const std::vector<ImVec2>& pool) {
  std::sort(facets.begin(), facets.end(),
            [](const Facet& a, const Facet& b) { return a.depth < b.depth; });
  // A wall is one quad per edge of the ring, and an antialiased fill insets
  // every one of them by half a pixel, so the wall would come out ruled with
  // hairlines where the quads meet.
  const ImDrawListFlags saved_flags = list->Flags;
  list->Flags &= ~ImDrawListFlags_AntiAliasedFill;
  for (const Facet& facet : facets) {
    if (facet.count < 2) continue;
    const ImVec2* points = pool.data() + facet.first;
    if (facet.fill != 0) {
      // Only ever a triangle from the mesh or a quad from a wall, both convex.
      if (facet.count == 3)
        list->AddTriangleFilled(points[0], points[1], points[2], facet.fill);
      else
        list->AddQuadFilled(points[0], points[1], points[2], points[3], facet.fill);
    }
    if (facet.edge != 0)
      list->AddPolyline(points, static_cast<int>(facet.count), facet.edge,
                        facet.count > 2 ? ImDrawFlags_Closed : ImDrawFlags_None,
                        1.0f);
  }
  list->Flags = saved_flags;
}

void paint_scene(OrbitScene& scene) {
  paint_facets(scene.list, scene.facets, scene.pool);
}

// A triad at the program origin, drawn over the scene rather than into it.
// It is a statement about the coordinates, not a thing standing in the work, so
// it should not be able to go behind the work and disappear -- the one moment
// it is wanted is the moment the origin has been put somewhere unexpected,
// which is very often inside the blank.
//
// Sized in pixels, not millimetres. A fixed world length would vanish on a
// large part and swallow a small one.
void draw_origin_gizmo(const App& app, const OrbitScene& scene) {
  const Projection& projection = scene.projection;
  if (!(projection.scale > 0.0f)) return;
  // Where the program's zero sits, in the model's own coordinates. The job
  // carries the offset added to every posted coordinate, which is the negative
  // of the point it is measured from.
  const Vec3 at{static_cast<float>(-app.job.origin_x),
                static_cast<float>(-app.job.origin_y),
                static_cast<float>(-app.job.origin_z)};
  const float reach = 46.0f / projection.scale;
  const ImVec2 centre = projection.point(at);

  static const Vec3 kDirections[3] = {
      {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
  static const ImU32 kColours[3] = {IM_COL32(232, 96, 92, 255),
                                    IM_COL32(126, 205, 122, 255),
                                    IM_COL32(104, 158, 246, 255)};
  static const char* const kNames[3] = {"X", "Y", "Z"};
  for (int axis = 0; axis < 3; ++axis) {
    const Vec3& step = kDirections[axis];
    const Vec3 tip{at.x + step.x * reach, at.y + step.y * reach,
                   at.z + step.z * reach};
    const ImVec2 end = projection.point(tip);
    // Outlined, because an axis crossing the part it is drawn over is otherwise
    // the same brightness as what it crosses.
    scene.list->AddLine(centre, end, IM_COL32(12, 14, 18, 200), 3.5f);
    scene.list->AddLine(centre, end, kColours[axis], 1.7f);
    scene.list->AddText({end.x + 3.0f, end.y - 6.0f}, kColours[axis],
                        kNames[axis]);
  }
  scene.list->AddCircleFilled(centre, 3.5f, IM_COL32(12, 14, 18, 220));
  scene.list->AddCircleFilled(centre, 2.0f, IM_COL32(238, 240, 245, 255));
}

// ---------------------------------------------------------------------------
// The view cube: the small oriented cube in the corner of a 3D view that turns
// the camera to look straight at whatever part of it is clicked. A face gives
// an axis view, an edge the view halfway between its two faces, and a corner
// the isometric between its three -- which is the whole of the 26 directions,
// obtained by cutting each face into a three-by-three grid and reading a cell's
// direction off its position in it.
//
// Drawn from the orbit angles rather than from the scene's projection, so it is
// the same cube in the same place in every view, at the same size, whatever the
// view behind it is framed on.

ImVec2 cube_centre(ImVec2 top_left, ImVec2 size) {
  return {top_left.x + size.x - kCubeMargin - kCubeBox * 0.5f,
          top_left.y + size.y - kCubeMargin - kCubeBox * 0.5f};
}

bool over_cube(ImVec2 top_left, ImVec2 size) {
  const ImVec2 centre = cube_centre(top_left, size);
  const ImVec2 mouse = ImGui::GetIO().MousePos;
  return std::abs(mouse.x - centre.x) <= kCubeBox * 0.5f &&
         std::abs(mouse.y - centre.y) <= kCubeBox * 0.5f;
}

// Turn the camera to look from `direction`, over the next few frames.
void aim_camera(App& app, const Vec3& direction) {
  float yaw = app.orbit.yaw;
  float pitch = app.orbit.pitch;
  angles_from_direction(direction, app.orbit.yaw, yaw, pitch);
  // The short way round. Azimuth roams, so the two angles naming one direction
  // can be any number of turns apart, and interpolating between them as given
  // would spin the model several times to arrive where it already was.
  float turn = yaw - app.orbit.yaw;
  while (turn > kQuarterTurn * 2.0f) turn -= kQuarterTurn * 4.0f;
  while (turn < -kQuarterTurn * 2.0f) turn += kQuarterTurn * 4.0f;
  app.orbit.from_yaw = app.orbit.yaw;
  app.orbit.from_pitch = app.orbit.pitch;
  app.orbit.to_yaw = app.orbit.yaw + turn;
  app.orbit.to_pitch = std::clamp(pitch, -kQuarterTurn, kQuarterTurn);
  app.orbit.turn = 0.0f;
  app.orbit.turning = true;
}

void advance_camera(App& app, double seconds) {
  if (!app.orbit.turning) return;
  app.orbit.turn += static_cast<float>(seconds) / 0.26f;
  if (app.orbit.turn >= 1.0f) {
    app.orbit.turn = 1.0f;
    app.orbit.turning = false;
  }
  // Eased at both ends, so the movement reads as the camera being carried round
  // rather than as two jumps with a slide between them.
  const float t = app.orbit.turn;
  const float eased = t * t * (3.0f - 2.0f * t);
  app.orbit.yaw = app.orbit.from_yaw +
                  (app.orbit.to_yaw - app.orbit.from_yaw) * eased;
  app.orbit.pitch = app.orbit.from_pitch +
                    (app.orbit.to_pitch - app.orbit.from_pitch) * eased;
}

// One face of the cube: its outward normal and the two axes its grid runs
// along. The tangents are swapped on a negative face so that every face winds
// the same way on screen once projected, which is what keeps the antialiased
// fills consistent between the near faces and the far ones.
CubeFace cube_face(int axis, int sign) {
  float normal[3] = {0.0f, 0.0f, 0.0f};
  float across[3] = {0.0f, 0.0f, 0.0f};
  float down[3] = {0.0f, 0.0f, 0.0f};
  normal[axis] = static_cast<float>(sign);
  across[(axis + (sign > 0 ? 1 : 2)) % 3] = 1.0f;
  down[(axis + (sign > 0 ? 2 : 1)) % 3] = 1.0f;
  return {{normal[0], normal[1], normal[2]},
          {across[0], across[1], across[2]},
          {down[0], down[1], down[2]}};
}

// Which way a cell of a face looks from: the face's own direction, plus a step
// along each tangent it sits off centre. The centre cell is the axis view, an
// edge cell the view halfway between two faces, a corner cell the isometric.
Vec3 cube_direction(const CubeFace& face, int across, int down) {
  const float u = static_cast<float>(across);
  const float v = static_cast<float>(down);
  return {face.normal.x + face.across.x * u + face.down.x * v,
          face.normal.y + face.across.y * u + face.down.y * v,
          face.normal.z + face.across.z * u + face.down.z * v};
}

// The app's own vocabulary for the axes, taken from the origin picker: +X is
// right, +Y is front, +Z is top. A view cube that disagreed with the panel the
// operator sets the origin in would be worse than no labels at all.
const char* cube_face_name(int axis, int sign) {
  static const char* const kNames[3][2] = {
      {"Left", "Right"}, {"Back", "Front"}, {"Bottom", "Top"}};
  return kNames[axis][sign > 0 ? 1 : 0];
}

bool inside_quad(const ImVec2 corners[4], ImVec2 point) {
  // The quad is a projected rectangle and so convex; the point is inside when
  // it stays on the same side of all four edges.
  int positive = 0;
  int negative = 0;
  for (int i = 0; i < 4; ++i) {
    const ImVec2& a = corners[i];
    const ImVec2& b = corners[(i + 1) % 4];
    const float side = (b.x - a.x) * (point.y - a.y) -
                       (b.y - a.y) * (point.x - a.x);
    if (side > 0.0f) ++positive;
    if (side < 0.0f) ++negative;
  }
  return positive == 0 || negative == 0;
}

void view_cube(App& app, ImDrawList* list, ImVec2 top_left, ImVec2 size,
               bool hovered) {
  const ImVec2 centre = cube_centre(top_left, size);
  const Basis basis = orbit_basis(app.orbit.yaw, app.orbit.pitch);
  const float scale = kCubeBox * 0.5f / kCubeFit;
  const auto project = [&](const Vec3& at) {
    return ImVec2{centre.x + (at.x * basis.right.x + at.y * basis.right.y +
                              at.z * basis.right.z) * scale,
                  centre.y - (at.x * basis.up.x + at.y * basis.up.y +
                              at.z * basis.up.z) * scale};
  };
  const ImVec2 mouse = ImGui::GetIO().MousePos;
  const bool pointing = hovered && over_cube(top_left, size);

  bool hot = false;
  Vec3 hot_direction{};

  // The nine cells of a face are one surface, not nine tiles. An antialiased
  // fill insets every one of them by half a pixel, which rules the face with
  // hairlines and turns a view cube into a Rubik's cube. The grid is a hit
  // test; what should be visible of it is the cell under the pointer.
  const ImDrawListFlags saved_flags = list->Flags;
  list->Flags &= ~ImDrawListFlags_AntiAliasedFill;

  for (int axis = 0; axis < 3; ++axis) {
    for (int sign = -1; sign <= 1; sign += 2) {
      const CubeFace face = cube_face(axis, sign);
      const float facing = face.normal.x * basis.toward.x +
                           face.normal.y * basis.toward.y +
                           face.normal.z * basis.toward.z;
      if (facing <= 0.0f) continue;  // the far side of the cube

      const auto at = [&](float u, float v) {
        return project({face.normal.x + face.across.x * u + face.down.x * v,
                        face.normal.y + face.across.y * u + face.down.y * v,
                        face.normal.z + face.across.z * u + face.down.z * v});
      };
      // Lighter as it turns to face the eye, so which face is being looked
      // straight at is legible without reading the labels.
      const int shade = static_cast<int>(70.0f + 90.0f * facing);
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          const auto corner = [&](int r, int c) {
            return at(-1.0f + 2.0f * static_cast<float>(c) / 3.0f,
                      -1.0f + 2.0f * static_cast<float>(r) / 3.0f);
          };
          const ImVec2 cell[4] = {corner(row, column), corner(row, column + 1),
                                  corner(row + 1, column + 1),
                                  corner(row + 1, column)};
          const bool under = pointing && inside_quad(cell, mouse);
          if (under) {
            hot = true;
            hot_direction = cube_direction(face, column - 1, row - 1);
          }
          list->AddConvexPolyFilled(
              cell, 4,
              under ? IM_COL32(96, 148, 214, 235)
                    : IM_COL32(shade, shade + 6, shade + 14, 205));
        }
      }
      // Outlined as a face rather than as nine cells: the grid is a hit test,
      // not a picture of one.
      const ImVec2 outline[4] = {at(-1, -1), at(1, -1), at(1, 1), at(-1, 1)};
      list->AddPolyline(outline, 4, IM_COL32(196, 208, 224, 220),
                        ImDrawFlags_Closed, 1.2f);
      // Named only where the name fits. A face turned nearly edge-on is a
      // sliver a few pixels wide, and a label centred on it runs out over its
      // neighbours -- which are the faces whose names one actually wants.
      ImVec2 low = outline[0];
      ImVec2 high = outline[0];
      for (const ImVec2& corner : outline) {
        low.x = std::min(low.x, corner.x);
        low.y = std::min(low.y, corner.y);
        high.x = std::max(high.x, corner.x);
        high.y = std::max(high.y, corner.y);
      }
      const char* name = cube_face_name(axis, sign);
      const ImVec2 extent = ImGui::CalcTextSize(name);
      if (high.x - low.x >= extent.x + 6.0f &&
          high.y - low.y >= extent.y + 4.0f) {
        const ImVec2 middle = project(face.normal);
        list->AddText({middle.x - extent.x * 0.5f, middle.y - extent.y * 0.5f},
                      IM_COL32(18, 22, 28, 255), name);
      }
    }
  }

  list->Flags = saved_flags;

  if (pointing && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    app.orbit.cube_pressed = true;
  if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
    // On release and only when the pointer stayed put, so that pressing the
    // cube and dragging away is an abandoned click rather than a snap.
    const ImVec2 moved = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
    if (app.orbit.cube_pressed && hot && std::hypot(moved.x, moved.y) < 5.0f)
      aim_camera(app, hot_direction);
    app.orbit.cube_pressed = false;
  }
}
}  // namespace tinglecam_gui
