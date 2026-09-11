#include "gui_internal.h"

namespace tinglecam_gui {

bool layout_splitter(App& app, const char* id, bool vertical, float extent,
                     float& delta) {
  delta = 0.0f;
  const float thickness = ui_size(app, 6.0f);
  const ImVec2 size = vertical ? ImVec2(thickness, extent)
                               : ImVec2(extent, thickness);
  if (extent <= 0.0f) return false;

  ImGui::InvisibleButton(id, size);
  const bool hovered = ImGui::IsItemHovered();
  const bool active = ImGui::IsItemActive();
  if (hovered || active)
    ImGui::SetMouseCursor(vertical ? ImGuiMouseCursor_ResizeEW
                                   : ImGuiMouseCursor_ResizeNS);
  if (active) {
    delta = vertical ? ImGui::GetIO().MouseDelta.x
                     : ImGui::GetIO().MouseDelta.y;
    app.panel_layout_dragging = true;
  }

  const ImGuiCol colour_id =
      active ? ImGuiCol_SeparatorActive
             : hovered ? ImGuiCol_SeparatorHovered : ImGuiCol_Separator;
  const ImU32 colour = ImGui::GetColorU32(colour_id);
  const ImVec2 item_min = ImGui::GetItemRectMin();
  const ImVec2 item_max = ImGui::GetItemRectMax();
  const float visible_thickness = ui_size(app, active || hovered ? 2.0f : 1.0f);
  if (vertical) {
    const float centre = (item_min.x + item_max.x) * 0.5f;
    ImGui::GetWindowDrawList()->AddRectFilled(
        ImVec2(centre - visible_thickness * 0.5f, item_min.y),
        ImVec2(centre + visible_thickness * 0.5f, item_max.y), colour);
  } else {
    const float centre = (item_min.y + item_max.y) * 0.5f;
    ImGui::GetWindowDrawList()->AddRectFilled(
        ImVec2(item_min.x, centre - visible_thickness * 0.5f),
        ImVec2(item_max.x, centre + visible_thickness * 0.5f), colour);
  }
  return active;
}

}  // namespace tinglecam_gui
