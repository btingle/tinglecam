#include "gui_internal.h"

namespace tinglecam_gui {
namespace {

const char* severity_name(LogSeverity severity) {
  switch (severity) {
    case LogSeverity::Info: return "INFO";
    case LogSeverity::Warning: return "WARN";
    case LogSeverity::Error: return "ERROR";
    case LogSeverity::Debug: return "DEBUG";
  }
  return "";
}

bool visible(LogSeverity severity, int filter) {
  return (filter == 0 && severity != LogSeverity::Debug) ||
         static_cast<int>(severity) == filter - 1;
}

}  // namespace

void messages_drawer(App& app) {
  const char* filters[] = {"All", "Info", "Warnings", "Errors", "Debug"};
  for (int filter = 0; filter < 5; ++filter) {
    if (filter != 0) ImGui::SameLine();
    if (ImGui::RadioButton(filters[filter], app.message_filter == filter))
      app.message_filter = filter;
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Copy")) {
    std::string copied;
    for (const LogMessage& message : app.messages) {
      if (!visible(message.severity, app.message_filter)) continue;
      copied += std::string("[") + severity_name(message.severity) + "] " +
                message.source + ": " + message.text + '\n';
    }
    ImGui::SetClipboardText(copied.c_str());
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Clear")) app.messages.clear();
  ImGui::SameLine();
  if (ImGui::SmallButton("Close")) app.show_messages = false;

  ImGui::BeginChild("message-list", ImVec2(0, 0));
  bool any_visible = false;
  for (const LogMessage& message : app.messages) {
    if (!visible(message.severity, app.message_filter)) continue;
    any_visible = true;
    ImGui::PushStyleColor(ImGuiCol_Text, severity_colour(app, message.severity));
    ImGui::TextUnformatted(message.source.c_str());
    ImGui::PopStyleColor();
    ImGui::SameLine();
    ImGui::TextWrapped("%s", message.text.c_str());
  }
  if (!any_visible) ImGui::TextDisabled("No messages.");
  if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f) ImGui::SetScrollHereY(1.0f);
  ImGui::EndChild();
}

void status_bar(App& app) {
  std::size_t warnings = 0, errors = 0;
  for (const LogMessage& message : app.messages) {
    if (message.severity == LogSeverity::Warning) ++warnings;
    if (message.severity == LogSeverity::Error) ++errors;
  }
  char right[160];
  int length = 0;
  if (app.has_plan)
    length += std::snprintf(right, sizeof right, "Job %.1f min    ",
                            app.plan.stats.cutting_minutes + app.plan.stats.rapid_minutes +
                                app.plan.stats.tool_change_minutes);
  std::snprintf(right + length, sizeof right - length, "%zu warning%s  %zu error%s",
                warnings, warnings == 1 ? "" : "s", errors, errors == 1 ? "" : "s");
  const float right_width = ImGui::CalcTextSize(right).x;

  const ImVec2 start = ImGui::GetCursorScreenPos();
  const float width = ImGui::GetContentRegionAvail().x;
  const ImVec2 size(width, ImGui::GetFrameHeight());
  if (ImGui::InvisibleButton("status", size)) app.show_messages = !app.show_messages;
  ImGui::SetItemTooltip(app.show_messages ? "Hide messages" : "Show messages");
  ImDrawList* list = ImGui::GetWindowDrawList();
  list->AddRectFilled(start, ImVec2(start.x + size.x, start.y + size.y),
                      ImGui::GetColorU32(ImGuiCol_MenuBarBg));
  const float text_y = start.y + (size.y - ImGui::GetFontSize()) * 0.5f;
  const float padding = ImGui::GetStyle().FramePadding.x;
  const auto last_visible = std::find_if(app.messages.rbegin(), app.messages.rend(),
      [](const auto& message) { return message.severity != LogSeverity::Debug; });
  if (last_visible != app.messages.rend()) {
    const LogMessage& last = *last_visible;
    const std::string text = last.source + ": " + last.text;
    const std::string line = text.substr(0, text.find('\n'));
    list->PushClipRect(start, ImVec2(start.x + width - right_width - padding * 3.0f,
                                     start.y + size.y), true);
    list->AddText(ImVec2(start.x + padding, text_y),
                  ImGui::GetColorU32(severity_colour(app, last.severity)), line.c_str());
    list->PopClipRect();
  }
  const ImU32 summary = ImGui::GetColorU32(
      errors ? severity_colour(app, LogSeverity::Error)
             : warnings ? severity_colour(app, LogSeverity::Warning)
                        : ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
  list->AddText(ImVec2(start.x + width - right_width - padding, text_y), summary, right);
}

}  // namespace tinglecam_gui
