#include "gui_internal.h"

namespace tinglecam_gui {
void refresh_surface_contours(App& app, const cam::Operation& operation) {
  if (app.model_loading) return;
  const auto model = app.geometry.part_cad();
  const auto* tool = app.tools.find(operation.tool_id);
  if (!operation.surface.manual_order) return;
  if (!model || !tool) {
    app.surface_previewer.stop();
    app.surface_preview = {};
    app.surface_preview.error = "Choose a STEP part and a ball end mill to preview contours.";
    app.surface_preview_key.clear();
    return;
  }
  const auto key = cam::surface_contour_signature(*model, operation, *tool);
  if (key != app.surface_preview_key) {
    app.surface_preview_key = key;
    app.surface_preview = {};
    app.surface_order_key.clear();
    // Capture immutable inputs, so loading another job/model cannot invalidate
    // a preview that is still being sampled on the worker.
    app.surface_previewer.start([model, operation, tool = *tool](
        const cam::ProgressCallback& progress) {
      return cam::prepare_surface_contours(*model, operation, tool, progress);
    });
  }
  cam::SurfaceContourPreview prepared;
  if (app.surface_previewer.take(prepared) && prepared.signature == key) {
    app.surface_preview = std::move(prepared);
    app.surface_order_key.clear();
  }
  if (app.surface_preview.signature != key) return;
  const auto& s = operation.surface;
  const auto roots = s.order_signature == key ? s.order_roots
                                             : std::vector<std::size_t>{};
  if (app.surface_order_key != key || app.surface_order_roots != roots) {
    app.surface_order_key = key;
    app.surface_order_roots = roots;
    app.surface_order = cam::propagate_surface_order(app.surface_preview, roots);
  }
}

namespace {
template <class Point> Vec3 view_point(Point p) { return {float(p.x), float(p.y), float(p.z)}; }
auto projection_values(const Projection& p) {
  return std::array<float, 15>{p.centre.x, p.centre.y,
      p.model_centre.x, p.model_centre.y, p.model_centre.z,
      p.right.x, p.right.y, p.right.z, p.up.x, p.up.y, p.up.z,
      p.toward.x, p.toward.y, p.toward.z, p.scale};
}
// A cached, two-pixel depth grid keeps contact lines and hit testing on the
// visible part surface. It does not move the geometry towards the camera.
void contact_depth(App& app, const OrbitScene& scene) {
  const auto model = app.geometry.part_cad();
  if (app.surface_depth_model == model &&
      projection_values(app.surface_depth_projection) == projection_values(scene.projection) &&
      app.surface_depth_origin.x == scene.top_left.x &&
      app.surface_depth_origin.y == scene.top_left.y &&
      app.surface_depth_size.x == scene.size.x && app.surface_depth_size.y == scene.size.y)
    return;
  app.surface_depth_model = model;
  app.surface_depth_projection = scene.projection;
  app.surface_depth_origin = scene.top_left;
  app.surface_depth_size = scene.size;
  const int width = app.surface_depth_width = std::max(1, int(std::ceil(scene.size.x / 2)));
  const int height = app.surface_depth_height = std::max(1, int(std::ceil(scene.size.y / 2)));
  app.surface_depth.assign(std::size_t(width) * height, -std::numeric_limits<float>::infinity());
  for (const auto& triangle : model->mesh().triangles) {
    const auto a = view_point(triangle.a), b = view_point(triangle.b), c = view_point(triangle.c);
    auto p = scene.projection.point(a), q = scene.projection.point(b), r = scene.projection.point(c);
    for (auto* v : {&p, &q, &r}) {
      v->x = (v->x - scene.top_left.x) / 2;
      v->y = (v->y - scene.top_left.y) / 2;
    }
    const float det = (q.y-r.y)*(p.x-r.x)+(r.x-q.x)*(p.y-r.y);
    if (std::abs(det) < 1e-8f) continue;
    const int left = std::max(0, int(std::floor(std::min({p.x,q.x,r.x}))));
    const int right = std::min(width-1, int(std::ceil(std::max({p.x,q.x,r.x}))));
    const int top = std::max(0, int(std::floor(std::min({p.y,q.y,r.y}))));
    const int bottom = std::min(height-1, int(std::ceil(std::max({p.y,q.y,r.y}))));
    const float da = scene.projection.depth(a), db = scene.projection.depth(b), dc = scene.projection.depth(c);
    for (int y = top; y <= bottom; ++y)
      for (int x = left; x <= right; ++x) {
        const float u = ((q.y-r.y)*(x+.5f-r.x)+(r.x-q.x)*(y+.5f-r.y))/det;
        const float v = ((r.y-p.y)*(x+.5f-r.x)+(p.x-r.x)*(y+.5f-r.y))/det;
        if (u < 0 || v < 0 || u+v > 1) continue;
        auto& depth = app.surface_depth[std::size_t(y)*width+x];
        depth = std::max(depth, u*da+v*db+(1-u-v)*dc);
      }
  }
}
bool contact_visible(const App& app, const Projection& projection, wave::Vec3d point) {
  const auto p = projection.point(view_point(point));
  const int x = int(std::floor((p.x-app.surface_depth_origin.x)/2));
  const int y = int(std::floor((p.y-app.surface_depth_origin.y)/2));
  if (x < 0 || y < 0 || x >= app.surface_depth_width || y >= app.surface_depth_height)
    return false;
  const auto depth = app.surface_depth[std::size_t(y)*app.surface_depth_width+x];
  return projection.depth(view_point(point)) >= depth - .05f - 2.0f/projection.scale;
}
wave::Vec3d contour_middle(const cam::SurfaceContour& contour) {
  const auto& span = contour.spans[contour.spans.size()/2];
  double length = 0;
  for (std::size_t i = 1; i < span.contacts.size(); ++i)
    length += wave::distance(span.contacts[i-1], span.contacts[i]);
  double remaining = length/2;
  for (std::size_t i = 1; i < span.contacts.size(); ++i) {
    const double segment = wave::distance(span.contacts[i-1], span.contacts[i]);
    if (segment > 0 && remaining <= segment)
      return wave::interpolate(span.contacts[i-1], span.contacts[i], remaining/segment);
    remaining -= segment;
  }
  return span.contacts.front();
}
ImU32 branch_colour(std::size_t branch) {
  constexpr ImU32 colours[]{IM_COL32(0, 190, 240, 255), IM_COL32(230, 115, 230, 255),
      IM_COL32(245, 155, 45, 255), IM_COL32(100, 220, 130, 255),
      IM_COL32(100, 150, 255, 255), IM_COL32(240, 130, 160, 255)};
  return colours[branch % std::size(colours)];
}
}

void draw_surface_contours(App& app, OrbitScene& scene, cam::Operation& operation) {
  const auto& preview = app.surface_preview;
  const auto& order = app.surface_order;
  const auto none = cam::SurfaceContourOrder::none;
  if (preview.signature != app.surface_preview_key || !preview.error.empty() ||
      preview.contours.empty() || order.depth.size() != preview.contours.size()) return;
  contact_depth(app, scene);
  const auto screen = [&](wave::Vec3d p) { return scene.projection.point(view_point(p)); };
  const auto visible = [&](wave::Vec3d p) { return contact_visible(app, scene.projection, p); };
  const auto mouse = ImGui::GetIO().MousePos;
  const bool picking = scene.hovered && !app.orbit.cube_pressed && !app.planner.running();
  const bool current = operation.surface.order_signature == preview.signature;
  const auto is_root = [&](std::size_t id) {
    return current && std::find(operation.surface.order_roots.begin(),
        operation.surface.order_roots.end(), id) != operation.surface.order_roots.end();
  };
  std::size_t hovered = none;
  float best = ui_size(app, 7.0f), best_depth = -std::numeric_limits<float>::infinity();
  for (std::size_t id = 0; id < preview.contours.size(); ++id)
    for (const auto& span : preview.contours[id].spans)
      for (std::size_t i = 1; i < span.contacts.size(); ++i) {
        const auto a = span.contacts[i-1], b = span.contacts[i];
        if (!picking) continue;
        const auto p = screen(a), q = screen(b);
        const float dx = q.x-p.x, dy = q.y-p.y;
        const float length2 = dx*dx+dy*dy;
        const float t = length2 > 0 ? std::clamp(((mouse.x-p.x)*dx+(mouse.y-p.y)*dy)/length2, 0.f, 1.f) : 0.f;
        const float distance = std::hypot(mouse.x-p.x-t*dx, mouse.y-p.y-t*dy);
        const auto hit = wave::interpolate(a, b, t);
        const float depth = scene.projection.depth(view_point(hit));
        if ((distance < best-.1f || (std::abs(distance-best) <= .1f && depth > best_depth)) && visible(hit)) {
          best = distance;
          best_depth = depth;
          hovered = id;
        }
      }
  std::vector<wave::Vec3d> centres;
  for (std::size_t id = 0; id < preview.contours.size(); ++id) {
    const auto& contour = preview.contours[id];
    centres.push_back(contour_middle(contour));
    const ImU32 colour = hovered == id ? IM_COL32(255,255,255,255)
        : is_root(id) ? IM_COL32(255,205,45,255)
        : order.branch[id] == none ? IM_COL32(155,165,175,220)
                                   : branch_colour(order.branch[id]);
    const float thickness = ui_size(app, is_root(id) || hovered == id ? 3.f : 1.6f);
    for (const auto& span : contour.spans)
      for (std::size_t i = 1; i < span.contacts.size(); ++i) {
        const auto a = span.contacts[i-1], b = span.contacts[i];
        if (visible(wave::interpolate(a,b,.5)))
          scene.list->AddLine(screen(a), screen(b), colour, thickness);
      }
  }
  // Draw propagation arrows between contacts. These describe precedence across
  // passes; the preview contains neither cutter offsets nor machine moves.
  for (std::size_t id = 0; id < centres.size(); ++id) {
    const auto p = screen(centres[id]);
    if (is_root(id) && visible(centres[id]))
      scene.list->AddCircleFilled(p, ui_size(app,4.f), IM_COL32(255,205,45,255));
    const auto parent = order.parent[id];
    if (parent == none) continue;
    if (visible(centres[id]) && visible(centres[parent])) {
      const auto a = screen(centres[parent]);
      const float dx = p.x-a.x, dy = p.y-a.y, length = std::hypot(dx,dy);
      if (length >= ui_size(app,5.f)) {
        const ImVec2 middle{(a.x+p.x)/2,(a.y+p.y)/2};
        const float size = std::min(ui_size(app,4.f),length*.3f), ux=dx/length, uy=dy/length;
        const ImU32 colour = branch_colour(order.branch[id]);
        scene.list->AddLine(a,p,colour,ui_size(app,1.f));
        scene.list->AddTriangleFilled({middle.x+ux*size,middle.y+uy*size},
            {middle.x-ux*size-uy*size*.65f,middle.y-uy*size+ux*size*.65f},
            {middle.x-ux*size+uy*size*.65f,middle.y-uy*size-ux*size*.65f},colour);
      }
    }
    if (preview.neighbours[id].size() == 1 && visible(centres[id]))
      scene.list->AddCircleFilled(p,ui_size(app,3.f),IM_COL32(235,75,75,255));
    for (auto other : preview.neighbours[id]) {
      if (other <= id || order.depth[other] == none || is_root(other) || is_root(id) ||
          order.branch[id] == order.branch[other] || parent == other || order.parent[other] == id)
        continue;
      const auto meeting = wave::interpolate(centres[id],centres[other],.5);
      if (visible(meeting))
        scene.list->AddCircleFilled(screen(meeting),ui_size(app,4.f),IM_COL32(235,75,75,255));
    }
  }
  if (hovered != none) {
    ImGui::BeginTooltip();
    ImGui::Text("Contour %zu%s", hovered+1, is_root(hovered) ? " (root)" : "");
    if (order.depth[hovered] != none) {
      const auto it = std::find(order.sequence.begin(),order.sequence.end(),hovered);
      ImGui::Text("Pass %zu   Propagation depth %zu",
          std::size_t(it-order.sequence.begin())+1,order.depth[hovered]);
    } else ImGui::TextUnformatted("No propagation root reaches this contour");
    ImGui::TextUnformatted("Click to toggle root");
    ImGui::EndTooltip();
    const auto drag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && drag.x*drag.x+drag.y*drag.y < 9) {
      auto& s = operation.surface;
      if (!current) s.order_roots.clear();
      s.order_signature = preview.signature;
      const auto found = std::find(s.order_roots.begin(),s.order_roots.end(),hovered);
      if (found == s.order_roots.end()) s.order_roots.push_back(hovered);
      else s.order_roots.erase(found);
      refresh_surface_contours(app,operation);
    }
  }
}
}  // namespace tinglecam_gui
