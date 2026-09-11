#include "gui_internal.h"
namespace tinglecam_gui {
void left_panel(App& app) {
  const float available = ImGui::GetContentRegionAvail().y;
  const float splitter_height = ui_size(app, 6.0f);
  const float minimum_program = ui_size(app, 140.0f);
  const float minimum_operations = ui_size(app, 220.0f);
  const float maximum_program =
      std::max(minimum_program, available - splitter_height - minimum_operations);
  const float program_height = std::clamp(
      available * static_cast<float>(app.settings.panel_layout.left_program_fraction),
      minimum_program, maximum_program);

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
    if (ImGui::BeginTabItem("Settings")) {
      if (ImGui::BeginChild("settings-scroll", ImVec2(0, 0),
                            ImGuiChildFlags_None,
                            ImGuiWindowFlags_AlwaysVerticalScrollbar))
        settings_panel(app);
      ImGui::EndChild();
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }
  ImGui::EndChild();

  float delta = 0.0f;
  layout_splitter(app, "left-program-splitter", false,
                  ImGui::GetContentRegionAvail().x, delta);
  if (delta != 0.0f && available > 0.0f) {
    app.settings.panel_layout.left_program_fraction = std::clamp(
        (program_height + delta) / available, 0.2f, 0.8f);
  }

  ImGui::BeginChild("operation-list", ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_AlwaysVerticalScrollbar);
  operations_list_panel(app);
  ImGui::EndChild();
}

void right_panel(App& app) {
  const float available = ImGui::GetContentRegionAvail().y;
  const float splitter_height = ui_size(app, 6.0f);
  const float minimum_settings = ui_size(app, 220.0f);
  const float minimum_playback = ui_size(app, 180.0f);
  const float maximum_settings =
      std::max(minimum_settings, available - splitter_height - minimum_playback);
  const float settings_height = std::clamp(
      available * static_cast<float>(app.settings.panel_layout.right_settings_fraction),
      minimum_settings, maximum_settings);

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

  float delta = 0.0f;
  layout_splitter(app, "right-settings-splitter", false,
                  ImGui::GetContentRegionAvail().x, delta);
  if (delta != 0.0f && available > 0.0f) {
    app.settings.panel_layout.right_settings_fraction = std::clamp(
        (settings_height + delta) / available, 0.2f, 0.8f);
  }

  ImGui::BeginChild("playback-display", ImVec2(0, 0), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_AlwaysVerticalScrollbar);
  view_panel(app);
  ImGui::EndChild();
}
}  // namespace tinglecam_gui
