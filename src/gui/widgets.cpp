#include "gui_internal.h"

#include <cstdarg>

namespace tinglecam_gui {

bool begin_properties(const char* id) {
  if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingStretchProp)) return false;
  ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthStretch, 0.45f);
  ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch, 0.55f);
  return true;
}

void end_properties() { ImGui::EndTable(); }

void help_marker(const char* text) {
  ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
  ImGui::TextDisabled("(?)");
  if (ImGui::BeginItemTooltip()) {
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}

void property_label(const char* label, const char* help) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(label);
  if (help) help_marker(help);
  ImGui::TableSetColumnIndex(1);
}

bool prop_double(const char* label, double& value, float speed,
                 const char* format, const char* help, double minimum,
                 double maximum) {
  property_label(label, help);
  ImGui::PushID(label);
  ImGui::SetNextItemWidth(-FLT_MIN);
  const bool bounded = std::isfinite(minimum) || std::isfinite(maximum);
  const bool changed = ImGui::DragScalar(
      "##value", ImGuiDataType_Double, &value, speed,
      std::isfinite(minimum) ? &minimum : nullptr,
      std::isfinite(maximum) ? &maximum : nullptr, format,
      // What is typed is kept exactly; the format only says how to show it.
      ImGuiSliderFlags_NoRoundToFormat | (bounded ? ImGuiSliderFlags_AlwaysClamp : 0));
  ImGui::PopID();
  return changed;
}

bool prop_int(const char* label, int& value, float speed, int minimum,
              int maximum, const char* help) {
  property_label(label, help);
  ImGui::PushID(label);
  ImGui::SetNextItemWidth(-FLT_MIN);
  const bool changed = ImGui::DragInt("##value", &value, speed, minimum, maximum,
                                      "%d", ImGuiSliderFlags_AlwaysClamp);
  ImGui::PopID();
  return changed;
}

bool prop_check(const char* label, bool& value, const char* help) {
  property_label(label, help);
  ImGui::PushID(label);
  const bool changed = ImGui::Checkbox("##value", &value);
  ImGui::PopID();
  return changed;
}

bool prop_text(const char* label, std::string& value, const char* help) {
  property_label(label, help);
  ImGui::PushID(label);
  std::vector<char> buffer(std::max<std::size_t>(256, value.size() + 64), '\0');
  std::snprintf(buffer.data(), buffer.size(), "%s", value.c_str());
  ImGui::SetNextItemWidth(-FLT_MIN);
  const bool changed = ImGui::InputText("##value", buffer.data(), buffer.size());
  if (changed) value = buffer.data();
  ImGui::PopID();
  return changed;
}

bool prop_coordinate(const char* label, double& stored, double offset,
                     const char* help) {
  double shown = stored + offset;
  if (!prop_double(label, shown, 0.1f, "%.3f mm", help)) return false;
  stored = shown - offset;
  return true;
}

bool prop_override(const char* label, std::optional<double>& value,
                   std::optional<double> inherited, float speed,
                   const char* format, const char* help) {
  property_label(label, help);
  ImGui::PushID(label);
  bool changed = false;
  const float reset = ImGui::GetFrameHeight() * 2.4f;
  ImGui::SetNextItemWidth(-(reset + ImGui::GetStyle().ItemSpacing.x));
  double shown = value ? *value : inherited.value_or(0.0);
  if (!value) ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
  if (ImGui::DragScalar("##value", ImGuiDataType_Double, &shown, speed, nullptr,
                        nullptr, format, ImGuiSliderFlags_NoRoundToFormat)) {
    value = shown;
    changed = true;
  }
  if (!value) ImGui::PopStyleColor();
  ImGui::SetItemTooltip(value ? "Set on this operation"
                              : (inherited ? "From the tool's cutting data"
                                           : "No cutting data; drag to set it here"));
  ImGui::SameLine();
  ImGui::BeginDisabled(!value);
  if (ImGui::Button("Tool", ImVec2(reset, 0))) {
    value.reset();
    changed = true;
  }
  ImGui::SetItemTooltip("Use the tool's figure again");
  ImGui::EndDisabled();
  ImGui::PopID();
  return changed;
}

void prop_value(const char* label, const char* format, ...) {
  property_label(label);
  va_list args;
  va_start(args, format);
  ImGui::TextV(format, args);
  va_end(args);
}

}  // namespace tinglecam_gui
