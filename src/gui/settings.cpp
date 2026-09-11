#include "gui_internal.h"

namespace tinglecam_gui {
namespace {

template <typename Entries, typename Name>
bool library_choice(const char* label, std::string& id, const Entries& entries,
                    Name name) {
  int selected = -1;
  std::vector<const char*> labels;
  labels.reserve(entries.size());
  for (std::size_t i = 0; i < entries.size(); ++i) {
    labels.push_back(name(entries[i]));
    if (entries[i].id == id) selected = static_cast<int>(i);
  }
  if (labels.empty()) {
    ImGui::TextDisabled("%s: nothing available", label);
    return false;
  }
  if (selected < 0) selected = 0;
  bool changed = false;
  if (ImGui::Combo(label, &selected, labels.data(),
                   static_cast<int>(labels.size()))) {
    id = entries[static_cast<std::size_t>(selected)].id;
    changed = true;
  }
  return changed;
}

bool save_settings_impl(App& app) {
  try {
    cam::save_user_settings(app.settings_file, app.settings);
    return true;
  } catch (const std::exception& error) {
    say(app, error.what(), true);
    return false;
  }
}

}  // namespace

bool save_settings(App& app) { return save_settings_impl(app); }

void settings_panel(App& app) {
  ImGui::SeparatorText("Application");
  ImGui::TextWrapped("Preferences are saved automatically to %s",
                     app.settings_file.string().c_str());

  static const double kMinimumScale = 0.5;
  static const double kMaximumScale = 3.0;
  double scale = app.settings.ui_scale;
  if (ImGui::SliderScalar("UI scale", ImGuiDataType_Double, &scale,
                          &kMinimumScale, &kMaximumScale,
                          "%.2fx")) {
    app.settings.ui_scale = std::clamp(scale, 0.5, 3.0);
    apply_ui_scale(app);
    save_settings(app);
  }
  ImGui::SetItemTooltip(
      "Scales fonts, spacing, controls, and the main application panels."
      " Changes take effect immediately.");

  ImGui::SeparatorText("Panel layout");
  ImGui::TextWrapped(
      "Drag the separators between panels to resize them. Layout dimensions "
      "are saved automatically.");
  if (ImGui::Button("Reset panel layout")) {
    app.settings.panel_layout = cam::default_panel_layout();
    save_settings(app);
  }

  ImGui::SeparatorText("New operation defaults");
  ImGui::TextWrapped(
      "These values are used when adding a new operation. Existing operations "
      "are not changed. Pocket and plane positions follow the selected job "
      "origin automatically.");
  enum_combo("Operation type", app.settings_mode, operation_mode_label,
             kOperationModes, std::size(kOperationModes));

  cam::Operation& operation =
      cam::operation_default(app.settings, app.settings_mode);
  bool changed = false;
  changed |= library_choice("Tool", operation.tool_id, app.tools.tools,
                            [](const cam::Tool& value) { return value.name.c_str(); });
  changed |= library_choice(
      "Machine", operation.machine_id, app.machines.machines,
      [](const cam::Machine& value) { return value.name.c_str(); });
  changed |= library_choice(
      "Material", operation.material_id, app.materials.materials,
      [](const cam::Material& value) { return value.name.c_str(); });

  changed |= ImGui::InputDouble("Stepover (of diameter)",
                                &operation.stepover_ratio, 0, 0, "%.6g");
  changed |= ImGui::InputDouble("Stepdown (mm)", &operation.stepdown, 0, 0,
                                "%.6g");
  changed |= ImGui::InputDouble("Spindle (rpm)", &operation.spindle_rpm, 0, 0,
                                "%.6g");
  changed |= ImGui::InputDouble("Plunge / ramp feed (mm/min)",
                                &operation.plunge_feed, 0, 0, "%.6g");
  changed |= ImGui::Checkbox("Coolant", &operation.coolant);
  if (ImGui::CollapsingHeader("Tool assignment")) {
    changed |= ImGui::InputInt("Tool number (0 = auto)",
                               &operation.tool_number);
    changed |= ImGui::InputInt("H offset (0 = tool)",
                               &operation.length_offset);
  }

  if (cam::is_parametric(app.settings_mode)) {
    if (app.settings_mode == cam::OperationMode::Pocket) {
      if (operation.pockets.empty()) {
        operation.pockets.push_back(cam::PocketParameters{});
        changed = true;
      }
      ImGui::TextDisabled("Rounded rectangle; radius 0 is square and radius at half the smaller size is circular.");
      int remove = -1;
      for (std::size_t i = 0; i < operation.pockets.size(); ++i) {
        ImGui::PushID(static_cast<int>(i));
        ImGui::SeparatorText(("Pocket " + std::to_string(i + 1)).c_str());
        auto& pocket = operation.pockets[i];
        changed |= ImGui::InputDouble("Width X (mm)", &pocket.width, 0, 0,
                                      "%.6g");
        changed |= ImGui::InputDouble("Height Y (mm)", &pocket.height, 0, 0,
                                      "%.6g");
        changed |= ImGui::InputDouble("Corner radius (mm)", &pocket.radius,
                                      0, 0, "%.6g");
        changed |= ImGui::InputDouble("Pocket depth (mm)", &pocket.depth, 0,
                                      0, "%.6g");
        changed |= ImGui::InputDouble("Helix angle (degrees)",
                                      &pocket.helix_angle_degrees, 0, 0, "%.6g");
        if (ImGui::Button("Remove pocket")) remove = static_cast<int>(i);
        ImGui::PopID();
      }
      if (remove >= 0) {
        operation.pockets.erase(operation.pockets.begin() + remove);
        changed = true;
      }
      if (ImGui::Button("Add pocket")) {
        operation.pockets.push_back(cam::PocketParameters{});
        changed = true;
      }
    } else {
      auto& plane = operation.plane;
      if (plane.areas.empty()) {
        plane.areas.push_back(cam::ClearingArea{});
        changed = true;
      }
      for (std::size_t i = 0; i < plane.areas.size(); ++i) {
        ImGui::PushID(static_cast<int>(i));
        ImGui::SeparatorText(("Area " + std::to_string(i + 1)).c_str());
        auto& area = plane.areas[i];
        changed |= ImGui::InputDouble("Width X (mm)", &area.width, 0, 0,
                                      "%.6g");
        changed |= ImGui::InputDouble("Height Y (mm)", &area.height, 0, 0,
                                      "%.6g");
        changed |= ImGui::InputDouble("Top Z (mm)", &area.top_z, 0, 0,
                                      "%.6g");
        changed |= ImGui::InputDouble("Clearing depth (mm)", &area.depth, 0,
                                      0, "%.6g");
        ImGui::PopID();
      }
      if (ImGui::Button("Add area")) {
        plane.areas.push_back(cam::ClearingArea{});
        changed = true;
      }
    }
  } else {
    changed |= ImGui::InputInt("Finishing subdivisions",
                               &operation.finishing_subdivisions);
    changed |= ImGui::InputDouble("Radial stock to leave (mm)",
                                  &operation.radial_stock_to_leave, 0, 0,
                                  "%.6g");
    changed |= ImGui::InputDouble("Axial stock to leave (mm)",
                                  &operation.axial_stock_to_leave, 0, 0,
                                  "%.6g");
    if (app.settings_mode == cam::OperationMode::Roughing) {
      changed |= ImGui::Checkbox("Lead in from the air", &operation.lead_in);
      changed |= ImGui::InputDouble("Stock oversize (mm)",
                                    &operation.stock_deviation, 0, 0, "%.6g");
      changed |= ImGui::Checkbox("Boundary cleanup contour",
                                 &operation.rough_boundary_cleanup);
      changed |= ImGui::InputDouble("Cleanup tolerance (mm)",
                                    &operation.cleanup_tolerance, 0, 0,
                                    "%.6g");
      changed |= ImGui::InputDouble("Entry-front Z match (mm)",
                                    &operation.entry_front_match_tolerance, 0,
                                    0, "%.6g");
      changed |= ImGui::InputDouble("Tiny split area (stepover^2)",
                                    &operation.tiny_split_area_factor, 0, 0,
                                    "%.6g");
    } else {
      changed |= ImGui::Checkbox("Clip contours to nominal stock",
                                 &operation.clip_contours_to_stock);
      static constexpr cam::ContourDirectionPolicy directions[]{
          cam::ContourDirectionPolicy::Either,
          cam::ContourDirectionPolicy::Climb,
          cam::ContourDirectionPolicy::Conventional};
      changed |= enum_combo("Contour direction", operation.contour_direction,
                            cam::contour_direction_name, directions,
                            std::size(directions));
    }
  }

  changed |= ImGui::InputDouble("Simulation/feed pitch (mm)",
                                &operation.path_pitch, 0, 0, "%.6g");
  changed |= ImGui::Checkbox("Enter manual feed range",
                             &operation.manual_feed_range);
  if (operation.manual_feed_range) {
    changed |= ImGui::InputDouble("Full engagement (mm/min)",
                                  &operation.full_engagement_feed, 0, 0,
                                  "%.6g");
    changed |= ImGui::InputDouble("No engagement (mm/min)",
                                  &operation.no_engagement_feed, 0, 0,
                                  "%.6g");
  }
  changed |= ImGui::InputInt("Slice grid", &operation.slice_grid);
  changed |= ImGui::InputDouble("Simulation stock inflation (mm)",
                                &operation.simulation_stock_inflation, 0, 0,
                                "%.6g");

  if (changed) save_settings(app);
}

}  // namespace tinglecam_gui
