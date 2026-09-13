#include "gui_internal.h"

namespace tinglecam_gui {
namespace {

const char* severity_name(LogSeverity severity) {
  switch (severity) {
    case LogSeverity::Info: return "INFO";
    case LogSeverity::Warning: return "WARN";
    case LogSeverity::Error: return "ERROR";
  }
  return "";
}

ImVec4 severity_colour(LogSeverity severity) {
  switch (severity) {
    case LogSeverity::Info: return ImVec4(0.65f, 0.78f, 0.9f, 1.0f);
    case LogSeverity::Warning: return ImVec4(1.0f, 0.72f, 0.35f, 1.0f);
    case LogSeverity::Error: return ImVec4(1.0f, 0.4f, 0.4f, 1.0f);
  }
  return ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
}

bool visible(LogSeverity severity, int filter) {
  return filter == 0 || static_cast<int>(severity) == filter - 1;
}

}  // namespace

void messages_panel(App& app) {
  std::size_t infos = 0;
  std::size_t warnings = 0;
  std::size_t errors = 0;
  for (const LogMessage& message : app.messages) {
    switch (message.severity) {
      case LogSeverity::Info: ++infos; break;
      case LogSeverity::Warning: ++warnings; break;
      case LogSeverity::Error: ++errors; break;
    }
  }

  ImGui::SeparatorText("Build messages");
  ImGui::Text("%zu total", app.messages.size());
  ImGui::SameLine();
  ImGui::TextColored(severity_colour(LogSeverity::Info), "%zu info", infos);
  ImGui::SameLine();
  ImGui::TextColored(severity_colour(LogSeverity::Warning), "%zu warnings", warnings);
  ImGui::SameLine();
  ImGui::TextColored(severity_colour(LogSeverity::Error), "%zu errors", errors);

  const char* filters[] = {"All", "Info", "Warnings", "Errors"};
  for (int filter = 0; filter < 4; ++filter) {
    if (filter != 0) ImGui::SameLine();
    if (ImGui::RadioButton(filters[filter], app.message_filter == filter))
      app.message_filter = filter;
  }
  if (ImGui::SmallButton("Copy visible messages")) {
    std::string copied;
    for (const LogMessage& message : app.messages) {
      if (!visible(message.severity, app.message_filter)) continue;
      copied += '[';
      copied += severity_name(message.severity);
      copied += "] ";
      copied += message.source;
      copied += ": ";
      copied += message.text;
      copied += '\n';
    }
    ImGui::SetClipboardText(copied.c_str());
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Clear")) app.messages.clear();

  ImGui::BeginChild("message-list", ImVec2(0, 0), ImGuiChildFlags_Borders);
  for (const LogMessage& message : app.messages) {
    if (!visible(message.severity, app.message_filter)) continue;
    ImGui::PushStyleColor(ImGuiCol_Text, severity_colour(message.severity));
    ImGui::Text("[%s] %s", severity_name(message.severity),
                message.source.c_str());
    ImGui::PopStyleColor();
    ImGui::SameLine();
    ImGui::TextWrapped("%s", message.text.c_str());
  }
  if (app.messages.empty()) ImGui::TextDisabled("No messages.");
  ImGui::EndChild();
}

}  // namespace tinglecam_gui
