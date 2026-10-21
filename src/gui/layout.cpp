#include "gui_internal.h"

namespace tinglecam_gui {

void add_operation_menu(App& app);

bool layout_splitter(App& app, const char* id, bool vertical, float extent,
                     float& delta) {
  delta = 0.0f;
  const float thickness = ui_size(app, 6.0f);
  const ImVec2 size = vertical ? ImVec2(thickness, extent) : ImVec2(extent, thickness);
  if (extent <= 0.0f) return false;

  ImGui::InvisibleButton(id, size);
  const bool hovered = ImGui::IsItemHovered();
  const bool active = ImGui::IsItemActive();
  if (hovered || active)
    ImGui::SetMouseCursor(vertical ? ImGuiMouseCursor_ResizeEW : ImGuiMouseCursor_ResizeNS);
  if (active) {
    delta = vertical ? ImGui::GetIO().MouseDelta.x : ImGui::GetIO().MouseDelta.y;
    app.panel_layout_dragging = true;
  }
  const ImGuiCol colour_id = active ? ImGuiCol_SeparatorActive
                             : hovered ? ImGuiCol_SeparatorHovered : ImGuiCol_Separator;
  const ImU32 colour = ImGui::GetColorU32(colour_id);
  const ImVec2 item_min = ImGui::GetItemRectMin();
  const ImVec2 item_max = ImGui::GetItemRectMax();
  const float visible = ui_size(app, active || hovered ? 2.0f : 1.0f);
  if (vertical) {
    const float centre = (item_min.x + item_max.x) * 0.5f;
    ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(centre - visible * 0.5f, item_min.y),
                                              ImVec2(centre + visible * 0.5f, item_max.y),
                                              colour);
  } else {
    const float centre = (item_min.y + item_max.y) * 0.5f;
    ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(item_min.x, centre - visible * 0.5f),
                                              ImVec2(item_max.x, centre + visible * 0.5f),
                                              colour);
  }
  return active;
}

namespace {

void menu_bar(App& app) {
  if (!ImGui::BeginMenuBar()) return;
  if (ImGui::BeginMenu("File")) {
    if (ImGui::MenuItem("New job", "Ctrl+N")) new_job(app);
    if (ImGui::MenuItem("Open job...", "Ctrl+O")) open_job(app);
    if (ImGui::BeginMenu("Open recent", !app.settings.recent_jobs.empty())) {
      for (const auto& recent : app.settings.recent_jobs)
        if (ImGui::MenuItem(recent.c_str())) open_recent_job(app, recent);
      ImGui::EndMenu();
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Save job", "Ctrl+S")) save_job(app, false);
    if (ImGui::MenuItem("Save job as...", "Ctrl+Shift+S")) save_job(app, true);
    ImGui::Separator();
    if (ImGui::MenuItem("Part model...", nullptr, false, !app.planner.running()))
      choose_part_model(app);
    if (ImGui::MenuItem("Stock model...", nullptr, false, !app.planner.running()))
      choose_stock_model(app);
    ImGui::Separator();
    if (ImGui::MenuItem("Export G-code...", "Ctrl+E", false, app.has_plan))
      export_gcode(app, false);
    if (ImGui::MenuItem("Export checked operations...", nullptr, false, app.has_plan))
      export_gcode(app, true);
    ImGui::Separator();
    if (ImGui::MenuItem("Quit", "Ctrl+Q")) request_quit(app);
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Operations")) {
    ImGui::BeginDisabled(app.planner.running());
    if (ImGui::BeginMenu("Add")) {
      add_operation_menu(app);
      ImGui::EndMenu();
    }
    if (ImGui::MenuItem("Plan out-of-date operations", "F5")) plan_out_of_date(app);
    ImGui::EndDisabled();
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("View")) {
    if (ImGui::MenuItem("Fit to view", "F")) {
      app.view.needs_fit = true;
      app.orbit.pan = ImVec2(0.0f, 0.0f);
      app.orbit.zoom = 1.0f;
    }
    ImGui::MenuItem("Messages", nullptr, &app.show_messages);
    ImGui::MenuItem("G-code preview", nullptr, &app.show_gcode_preview);
    if (ImGui::MenuItem("Debug information", nullptr, &app.settings.show_debug))
      save_settings(app);
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Tools")) {
    ImGui::MenuItem("Tool library...", nullptr, &app.show_tool_library);
    ImGui::MenuItem("Material library...", nullptr, &app.show_material_library);
    ImGui::MenuItem("Settings...", nullptr, &app.show_settings);
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Help")) {
    ImGui::TextDisabled("Keyboard");
    ImGui::Text("Ctrl+N/O/S   New, open, save job");
    ImGui::Text("Ctrl+E       Export G-code");
    ImGui::Text("F5           Plan out-of-date operations");
    ImGui::Text("Space        Play or pause");
    ImGui::Text("Home         Back to the start of the pass");
    ImGui::Text("F            Fit the view");
    ImGui::Text("Delete       Remove the selected operation");
    ImGui::Separator();
    ImGui::TextDisabled("Views");
    ImGui::Text("Drag to orbit, right-drag to pan, wheel to zoom.");
    ImGui::EndMenu();
  }
  ImGui::EndMenuBar();
}

void toolbar(App& app) {
  if (ImGui::Button("New")) new_job(app);
  ImGui::SameLine();
  if (ImGui::Button("Open")) open_job(app);
  ImGui::SameLine();
  if (ImGui::Button("Save")) save_job(app, false);
  ImGui::SameLine();
  ImGui::TextDisabled("|");
  ImGui::SameLine();
  if (app.planner.running()) {
    const auto progress = app.planner.progress();
    char label[96];
    std::snprintf(label, sizeof label, "Planning %.0f%%", 100.0 * progress.fraction);
    ImGui::ProgressBar(static_cast<float>(progress.fraction),
                       ImVec2(ui_size(app, 200.0f), 0), label);
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) app.planner.cancel();
  } else {
    const bool stale = first_out_of_date(app, static_cast<int>(app.job.operations.size()) - 1) >= 0;
    ImGui::BeginDisabled(!stale);
    if (stale)
      ImGui::PushStyleColor(ImGuiCol_Button,
                            ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    if (ImGui::Button("Plan")) plan_out_of_date(app);
    if (stale) ImGui::PopStyleColor();
    ImGui::EndDisabled();
    ImGui::SetItemTooltip(stale ? "Plan every out-of-date operation (F5)"
                                : "Every operation matches the plan");
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(!app.has_plan);
  if (ImGui::Button("Export G-code")) export_gcode(app, false);
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::TextDisabled("|");
  ImGui::SameLine();
  if (ImGui::Button("Tool library")) app.show_tool_library = !app.show_tool_library;
  ImGui::SameLine();
  if (ImGui::Button("Materials")) app.show_material_library = !app.show_material_library;

  const std::string name = (app.job.name.empty() ? "Untitled" : app.job.name) +
                           (job_dirty(app) ? " *" : "");
  const float width = ImGui::CalcTextSize(name.c_str()).x;
  ImGui::SameLine(std::max(ImGui::GetCursorPosX(), ImGui::GetContentRegionMax().x - width));
  ImGui::TextDisabled("%s", name.c_str());
  if (!app.job_file.empty()) ImGui::SetItemTooltip("%s", app.job_file.string().c_str());
}

void left_column(App& app, float height) {
  const float splitter = ui_size(app, 6.0f);
  const float minimum = ui_size(app, 120.0f);
  const float setup_height = std::clamp(
      height * static_cast<float>(app.settings.panel_layout.left_setup_fraction),
      minimum, std::max(minimum, height - splitter - minimum));
  ImGui::BeginChild("setup", ImVec2(0, setup_height), ImGuiChildFlags_Borders);
  setup_panel(app);
  ImGui::EndChild();
  float delta = 0.0f;
  layout_splitter(app, "setup-splitter", false, ImGui::GetContentRegionAvail().x, delta);
  if (delta != 0.0f && height > 0.0f)
    app.settings.panel_layout.left_setup_fraction =
        std::clamp((setup_height + delta) / height, 0.15f, 0.85f);
  ImGui::BeginChild("operations", ImVec2(0, 0), ImGuiChildFlags_Borders);
  operations_panel(app);
  ImGui::EndChild();
}

void centre_column(App& app, float height) {
  const float splitter = ui_size(app, 6.0f);
  const float transport = ImGui::GetFrameHeightWithSpacing() +
                          ImGui::GetTextLineHeightWithSpacing() * 2.0f +
                          ImGui::GetStyle().WindowPadding.y * 2.0f;
  float drawer = 0.0f;
  if (app.show_messages)
    drawer = std::clamp(static_cast<float>(app.settings.panel_layout.messages_height),
                        ui_size(app, 60.0f), std::max(ui_size(app, 60.0f), height * 0.6f));
  const float view_height =
      std::max(ui_size(app, 120.0f),
               height - transport - (app.show_messages ? drawer + splitter : 0.0f) -
                   ImGui::GetStyle().ItemSpacing.y);
  ImGui::BeginChild("views", ImVec2(0, view_height));
  if (ImGui::BeginTabBar("view-tabs")) {
    const auto flag = [&](int index) -> ImGuiTabItemFlags {
      return app.force_tab == index ? ImGuiTabItemFlags_SetSelected : 0;
    };
    if (ImGui::BeginTabItem("Setup", nullptr, flag(0))) {
      draw_setup(app);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Pass", nullptr, flag(1))) {
      draw_canvas(app);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Result", nullptr, flag(2))) {
      draw_result(app);
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }
  ImGui::EndChild();
  ImGui::BeginChild("transport", ImVec2(0, transport), ImGuiChildFlags_Borders,
                    ImGuiWindowFlags_NoScrollbar);
  transport_bar(app);
  ImGui::EndChild();
  if (app.show_messages) {
    float delta = 0.0f;
    layout_splitter(app, "messages-splitter", false, ImGui::GetContentRegionAvail().x, delta);
    if (delta != 0.0f)
      app.settings.panel_layout.messages_height = std::clamp(
          app.settings.panel_layout.messages_height - delta, 60.0, 2000.0);
    ImGui::BeginChild("messages", ImVec2(0, 0), ImGuiChildFlags_Borders);
    messages_drawer(app);
    ImGui::EndChild();
  }
}

}  // namespace

void draw_main_window(App& app) {
  poll_hole_detection(app);
  handle_shortcuts(app);
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->WorkPos);
  ImGui::SetNextWindowSize(viewport->WorkSize);
  ImGui::Begin("TingleCAM", nullptr,
               ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoSavedSettings |
                   ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_MenuBar);
  ImGui::BeginDisabled(app.model_loading);
  menu_bar(app);
  toolbar(app);

  const float status = ImGui::GetFrameHeight();
  const float height = std::max(1.0f, ImGui::GetContentRegionAvail().y - status -
                                          ImGui::GetStyle().ItemSpacing.y);
  const float width = ImGui::GetContentRegionAvail().x;
  const float splitter = ui_size(app, 6.0f);
  const float spacing = ImGui::GetStyle().ItemSpacing.x;
  const float usable = std::max(1.0f, width - 2.0f * splitter - 4.0f * spacing);
  const float minimum_left = ui_size(app, 240.0f);
  const float minimum_centre = ui_size(app, 260.0f);
  const float minimum_right = ui_size(app, 260.0f);
  auto& layout = app.settings.panel_layout;
  const float left = std::clamp(static_cast<float>(layout.left_width), minimum_left,
                                std::max(minimum_left, usable - minimum_centre - minimum_right));
  const float right = std::clamp(static_cast<float>(layout.right_width), minimum_right,
                                 std::max(minimum_right, usable - minimum_centre - left));
  const float centre = std::max(1.0f, usable - left - right);

  ImGui::BeginChild("left-column", ImVec2(left, height));
  left_column(app, ImGui::GetContentRegionAvail().y);
  ImGui::EndChild();
  ImGui::SameLine();
  float delta = 0.0f;
  layout_splitter(app, "left-splitter", true, height, delta);
  if (delta != 0.0f) layout.left_width = std::clamp(layout.left_width + delta, 200.0, 2000.0);
  ImGui::SameLine();
  ImGui::BeginChild("centre-column", ImVec2(centre, height));
  centre_column(app, ImGui::GetContentRegionAvail().y);
  ImGui::EndChild();
  ImGui::SameLine();
  delta = 0.0f;
  layout_splitter(app, "right-splitter", true, height, delta);
  if (delta != 0.0f) layout.right_width = std::clamp(layout.right_width - delta, 220.0, 2000.0);
  ImGui::SameLine();
  ImGui::BeginChild("right-column", ImVec2(right, height), ImGuiChildFlags_Borders);
  operation_panel(app);
  ImGui::EndChild();

  status_bar(app);
  ImGui::EndDisabled();
  ImGui::End();

  if (!app.screenshot_mode && app.panel_layout_dragging &&
      !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    save_settings(app);
    app.panel_layout_dragging = false;
  }
  if (!app.force_tab_hold) app.force_tab = -1;
  if (!app.model_loading) {
    tool_library_window(app);
    material_library_window(app);
    settings_window(app);
    gcode_preview_window(app);
    draw_dialogs(app);
  }
  draw_model_loading(app);
}

}  // namespace tinglecam_gui
