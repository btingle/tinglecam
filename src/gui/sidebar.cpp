#include "gui_internal.h"
namespace tinglecam_gui {
void left_panel(App& app) {
  ensure_initial_operation(app);
  const float available = ImGui::GetContentRegionAvail().y;
  const float operations_height = std::clamp(available * 0.46f, 330.0f, 420.0f);
  const float program_height = std::max(available - operations_height - 6.0f,
                                        140.0f);

  ImGui::BeginChild("program-controls", ImVec2(0, program_height),
                    ImGuiChildFlags_Borders);
  if (!app.banner.empty()) {
    const ImVec4 colour = app.banner_is_error ? ImVec4(1.0f, 0.45f, 0.4f, 1.0f)
                                              : ImVec4(0.55f, 0.85f, 0.65f, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Text, colour);
    ImGui::TextWrapped("%s", app.banner.c_str());
    ImGui::PopStyleColor();
    ImGui::Separator();
  }
  if (ImGui::BeginTabBar("program-tabs")) {
    if (ImGui::BeginTabItem("Job / Origin")) {
      if (ImGui::BeginChild("job-origin-scroll", ImVec2(0, 0),
                            ImGuiChildFlags_None,
                            ImGuiWindowFlags_AlwaysVerticalScrollbar)) {
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.48f);
        job_panel(app);
        plan_panel(app);
        output_panel(app);
        ImGui::PopItemWidth();
      }
      ImGui::EndChild();
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Libraries")) {
      if (ImGui::BeginChild("libraries-scroll", ImVec2(0, 0),
                            ImGuiChildFlags_None,
                            ImGuiWindowFlags_AlwaysVerticalScrollbar))
        library_panel(app);
      ImGui::EndChild();
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }
  ImGui::EndChild();

  ImGui::BeginChild("operation-list", ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_AlwaysVerticalScrollbar);
  operations_list_panel(app);
  ImGui::EndChild();
}

void right_panel(App& app) {
  ensure_initial_operation(app);
  const float available = ImGui::GetContentRegionAvail().y;
  const float settings_height = std::max(available * 0.58f, 220.0f);

  ImGui::BeginChild("operation-settings", ImVec2(0, settings_height),
                    ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_AlwaysVerticalScrollbar);
  ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.43f);
  // Nothing to do afterwards: an edit is noticed by comparing the job against
  // the one the plan came from, and the plan itself is left alone so the
  // preview being edited against is still there to edit against.
  operation_settings_panel(app);
  ImGui::PopItemWidth();
  ImGui::EndChild();

  ImGui::BeginChild("playback-display", ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_AlwaysVerticalScrollbar);
  view_panel(app);
  ImGui::EndChild();
}
}  // namespace tinglecam_gui
