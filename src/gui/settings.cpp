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

constexpr cam::UiTheme kUiThemes[]{
    cam::UiTheme::Default,
    cam::UiTheme::Slate,
    cam::UiTheme::Ember,
    cam::UiTheme::Paper};

constexpr cam::UiFont kUiFonts[]{
    cam::UiFont::Default,
    cam::UiFont::Sans,
    cam::UiFont::Mono,
    cam::UiFont::Serif};

cam::CustomUiTheme* active_custom_theme(App& app) {
  if (app.settings.custom_theme.empty()) return nullptr;
  for (auto& theme : app.settings.custom_themes)
    if (theme.name == app.settings.custom_theme) return &theme;
  return nullptr;
}

const cam::CustomUiTheme* active_custom_theme(const App& app) {
  if (app.settings.custom_theme.empty()) return nullptr;
  for (const auto& theme : app.settings.custom_themes)
    if (theme.name == app.settings.custom_theme) return &theme;
  return nullptr;
}

bool theme_selector(App& app) {
  const cam::CustomUiTheme* custom = active_custom_theme(app);
  const char* preview = custom ? custom->name.c_str()
                               : cam::ui_theme_name(app.settings.ui_theme);
  bool changed = false;
  if (ImGui::BeginCombo("Theme", preview)) {
    for (const cam::UiTheme theme : kUiThemes) {
      const bool selected = !custom && app.settings.ui_theme == theme;
      if (ImGui::Selectable(cam::ui_theme_name(theme), selected)) {
        app.settings.ui_theme = theme;
        app.settings.custom_theme.clear();
        changed = true;
      }
      if (selected) ImGui::SetItemDefaultFocus();
    }
    for (const auto& theme : app.settings.custom_themes) {
      const bool selected = custom && custom->name == theme.name;
      if (ImGui::Selectable(theme.name.c_str(), selected)) {
        app.settings.custom_theme = theme.name;
        changed = true;
      }
      if (selected) ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
  }
  if (changed) apply_ui_theme(app);
  return changed;
}

void additional_style_controls(ImGuiStyle& style) {
  ImGui::SeparatorText("Additional style controls");
  ImGui::TextDisabled(
      "These less commonly used fields are kept out of Dear ImGui's standard "
      "editor, but are still part of the saved theme.");

  ImGui::DragFloat2("Window minimum size", &style.WindowMinSize.x, 1.0f,
                    0.0f, 4096.0f, "%.0f");
  ImGui::DragFloat("Columns minimum spacing", &style.ColumnsMinSpacing, 1.0f,
                   0.0f, 100.0f, "%.0f");

  ImGui::SeparatorText("Interaction geometry");
  ImGui::DragFloat("Selectable rounding", &style.SelectableRounding, 0.25f,
                   0.0f, 12.0f, "%.2f");
  ImGui::SetItemTooltip(
      "Radius of selectable items. Contiguous selections may look uneven when "
      "this is non-zero.");
  ImGui::DragFloat("Drag/drop target rounding", &style.DragDropTargetRounding,
                   0.25f, -1.0f, 12.0f, "%.2f");
  ImGui::SetItemTooltip("Negative means use Frame rounding.");
  ImGui::DragFloat("Drag/drop target border", &style.DragDropTargetBorderSize,
                   0.25f, 0.0f, 8.0f, "%.2f");
  ImGui::DragFloat("Drag/drop target padding", &style.DragDropTargetPadding,
                   0.5f, 0.0f, 32.0f, "%.1f");
  ImGui::DragFloat("Input text cursor thickness", &style.InputTextCursorSize,
                   0.25f, 0.0f, 16.0f, "%.2f");
  ImGui::DragFloat("Software cursor scale", &style.MouseCursorScale, 0.05f,
                   0.25f, 8.0f, "%.2f");

  ImGui::SeparatorText("Tooltip timing");
  ImGui::DragFloat("Stationary delay", &style.HoverStationaryDelay, 0.01f,
                   0.0f, 5.0f, "%.2f s");
  ImGui::DragFloat("Short tooltip delay", &style.HoverDelayShort, 0.01f,
                   0.0f, 5.0f, "%.2f s");
  ImGui::DragFloat("Normal tooltip delay", &style.HoverDelayNormal, 0.01f,
                   0.0f, 5.0f, "%.2f s");
}

}  // namespace

bool save_settings(App& app) { return save_settings_impl(app); }

void settings_panel(App& app) {
  ImGui::SeparatorText("Application");
  ImGui::TextWrapped("Preferences are saved automatically to %s",
                     app.settings_file.string().c_str());

  bool changed = theme_selector(app);
  ImGui::SetItemTooltip(
      "Default keeps the original dark style. Slate is cooler, Ember is warmer, "
      "and Paper is a light theme.");

  static std::array<char, 128> theme_name{};
  ImGui::InputText("Theme name", theme_name.data(), theme_name.size());
  ImGui::SameLine();
  if (ImGui::Button("Create / save theme")) {
    const std::string name(theme_name.data());
    if (name.empty()) {
      say(app, "Theme name cannot be empty", true);
    } else if (std::any_of(kUiThemes, kUiThemes + std::size(kUiThemes),
                           [&](cam::UiTheme theme) {
                             return name == cam::ui_theme_name(theme);
                           })) {
      say(app, "That name is reserved for a built-in theme", true);
    } else {
      cam::CustomUiTheme* target = nullptr;
      for (auto& theme : app.settings.custom_themes)
        if (theme.name == name) target = &theme;
      if (!target) {
        app.settings.custom_themes.push_back({name, cam::UiFont::Default, {}});
        target = &app.settings.custom_themes.back();
      }
      target->style_blob = ui_style_blob(app.unscaled_style);
      app.settings.custom_theme = name;
      changed = true;
      apply_ui_theme(app);
    }
  }
  ImGui::SetItemTooltip(
      "Creates a new custom theme from the currently displayed style, or saves "
      "over an existing custom theme with the same name.");

  if (cam::CustomUiTheme* custom = active_custom_theme(app)) {
    cam::UiFont font = custom->font;
    if (enum_combo("Theme font", font, cam::ui_font_name, kUiFonts,
                   std::size(kUiFonts))) {
      custom->font = font;
      apply_ui_theme(app);
      changed = true;
    }
    if (ImGui::Button("Save current style")) {
      custom->style_blob = ui_style_blob(app.unscaled_style);
      changed = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Delete theme")) {
      const std::string name = custom->name;
      app.settings.custom_themes.erase(
          std::remove_if(app.settings.custom_themes.begin(),
                         app.settings.custom_themes.end(),
                         [&](const cam::CustomUiTheme& theme) {
                           return theme.name == name;
                         }),
          app.settings.custom_themes.end());
      app.settings.custom_theme.clear();
      app.settings.ui_theme = cam::UiTheme::Default;
      apply_ui_theme(app);
      changed = true;
    }
  } else {
    ImGui::TextDisabled(
        "Built-in themes are read-only; enter a name to create an editable copy.");
  }

  if (ImGui::CollapsingHeader("Full style editor")) {
    const ImGuiStyle before = app.unscaled_style;
    ImGui::GetStyle() = app.unscaled_style;
    ImGuiStyle reference = app.unscaled_style;
    ImGui::ShowStyleEditor(&reference);
    additional_style_controls(ImGui::GetStyle());
    const ImGuiStyle edited = ImGui::GetStyle();
    if (std::memcmp(&before, &edited, sizeof(ImGuiStyle)) != 0)
      app.unscaled_style = edited;
    app.unscaled_style_captured = true;
    apply_ui_scale(app);
    if (active_custom_theme(app) &&
        std::memcmp(&before, &edited, sizeof(ImGuiStyle)) != 0)
      ImGui::TextDisabled("Style changes are unsaved until you press Save current style.");
  }

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
  changed |= library_choice("Tool", operation.tool_id, app.tools.tools,
                            [](const cam::Tool& value) { return value.name.c_str(); });
  changed |= library_choice(
      "Machine", operation.machine_id, app.machines.machines,
      [](const cam::Machine& value) { return value.name.c_str(); });
  changed |= library_choice(
      "Material", operation.material_id, app.materials.materials,
      [](const cam::Material& value) { return value.name.c_str(); });

  changed |= ImGui::DragScalar("Stepover (of diameter)", ImGuiDataType_Double,
                               &operation.stepover_ratio, 0.01f);
  changed |= ImGui::DragScalar("Stepdown (mm)", ImGuiDataType_Double,
                               &operation.stepdown, 0.1f);
  changed |= ImGui::DragScalar("Spindle (rpm)", ImGuiDataType_Double,
                               &operation.spindle_rpm, 10.0f);
  changed |= ImGui::DragScalar("Plunge / ramp feed (mm/min)",
                               ImGuiDataType_Double, &operation.plunge_feed,
                               10.0f);
  changed |= ImGui::Checkbox("Coolant", &operation.coolant);
  if (ImGui::CollapsingHeader("Tool assignment")) {
    changed |= ImGui::DragInt("Tool number (0 = auto)", &operation.tool_number,
                              1.0f, 0);
    changed |= ImGui::DragInt("H offset (0 = tool)", &operation.length_offset,
                              1.0f, 0);
  }

  if (cam::is_parametric(app.settings_mode)) {
    if (app.settings_mode == cam::OperationMode::Pocket) {
      changed |= ImGui::DragScalar("Radial finishing margin (mm)",
                                   ImGuiDataType_Double,
                                   &operation.pocket_radial_margin, 0.02f);
      ImGui::SetItemTooltip(
          "Stock left on pocket walls. Negative cuts past them instead.");
      changed |= ImGui::Checkbox("Auto-identify pockets",
                                 &operation.auto_identify_pockets);
      if (operation.auto_identify_pockets) {
        changed |= ImGui::DragScalar("Inspection depth below part top (mm)",
                                     ImGuiDataType_Double,
                                     &operation.pocket_auto.check_depth, 0.1f);
        changed |= ImGui::DragScalar("Helix angle (degrees)",
                                     ImGuiDataType_Double,
                                     &operation.pocket_auto.helix_angle_degrees,
                                     0.1f);
        if (operation.pocket_auto.exclusions.empty()) {
          operation.pocket_auto.exclusions.push_back(cam::PocketExclusionArea{});
          changed = true;
        }
        int remove_exclusion = -1;
        for (std::size_t i = 0; i < operation.pocket_auto.exclusions.size(); ++i) {
          auto& exclusion = operation.pocket_auto.exclusions[i];
          ImGui::PushID(static_cast<int>(i));
          ImGui::SeparatorText(("Exclusion area " + std::to_string(i + 1)).c_str());
          changed |= ImGui::Checkbox("Enabled", &exclusion.enabled);
          ImGui::BeginDisabled(!exclusion.enabled);
          changed |= ImGui::DragScalar("Center X (mm)", ImGuiDataType_Double,
                                       &exclusion.center_x, 0.1f);
          changed |= ImGui::DragScalar("Center Y (mm)", ImGuiDataType_Double,
                                       &exclusion.center_y, 0.1f);
          changed |= ImGui::DragScalar("Width X (mm)", ImGuiDataType_Double,
                                       &exclusion.width, 0.1f);
          changed |= ImGui::DragScalar("Height Y (mm)", ImGuiDataType_Double,
                                       &exclusion.height, 0.1f);
          ImGui::EndDisabled();
          if (ImGui::Button("Remove exclusion")) remove_exclusion = static_cast<int>(i);
          ImGui::PopID();
        }
        if (remove_exclusion >= 0) {
          operation.pocket_auto.exclusions.erase(
              operation.pocket_auto.exclusions.begin() + remove_exclusion);
          changed = true;
        }
        if (ImGui::Button("Add exclusion area")) {
          operation.pocket_auto.exclusions.push_back(cam::PocketExclusionArea{});
          changed = true;
        }
      } else {
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
          changed |= ImGui::DragScalar("Width X (mm)", ImGuiDataType_Double,
                                       &pocket.width, 0.1f);
          changed |= ImGui::DragScalar("Height Y (mm)", ImGuiDataType_Double,
                                       &pocket.height, 0.1f);
          changed |= ImGui::DragScalar("Corner radius (mm)", ImGuiDataType_Double,
                                       &pocket.radius, 0.1f);
          changed |= ImGui::DragScalar("Pocket depth (mm)", ImGuiDataType_Double,
                                       &pocket.depth, 0.1f);
          changed |= ImGui::DragScalar("Helix angle (degrees)",
                                       ImGuiDataType_Double,
                                       &pocket.helix_angle_degrees, 0.1f);
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
        changed |= ImGui::DragScalar("Width X (mm)", ImGuiDataType_Double,
                                     &area.width, 0.1f);
        changed |= ImGui::DragScalar("Height Y (mm)", ImGuiDataType_Double,
                                     &area.height, 0.1f);
        changed |= ImGui::DragScalar("Top Z (mm)", ImGuiDataType_Double,
                                     &area.top_z, 0.1f);
        changed |= ImGui::DragScalar("Clearing depth (mm)", ImGuiDataType_Double,
                                     &area.depth, 0.1f);
        ImGui::PopID();
      }
      if (ImGui::Button("Add area")) {
        plane.areas.push_back(cam::ClearingArea{});
        changed = true;
      }
    }
  } else {
    if (app.settings_mode == cam::OperationMode::Roughing)
      changed |= roughing_subdivision_settings(operation);
    changed |= ImGui::DragScalar("Radial stock to leave (mm)",
                                 ImGuiDataType_Double,
                                 &operation.radial_stock_to_leave, 0.02f);
    changed |= ImGui::DragScalar("Axial stock to leave (mm)",
                                 ImGuiDataType_Double,
                                 &operation.axial_stock_to_leave, 0.02f);
    if (app.settings_mode == cam::OperationMode::Roughing) {
      changed |= ImGui::Checkbox("Lead in from the air", &operation.lead_in);
      changed |= ImGui::DragScalar("Stock oversize (mm)", ImGuiDataType_Double,
                                   &operation.stock_deviation, 0.01f);
      changed |= ImGui::Checkbox("Boundary cleanup contour",
                                 &operation.rough_boundary_cleanup);
      changed |= ImGui::DragScalar("Cleanup tolerance (mm)",
                                   ImGuiDataType_Double,
                                   &operation.cleanup_tolerance, 0.01f);
      changed |= ImGui::DragScalar("Entry-front Z match (mm)",
                                   ImGuiDataType_Double,
                                   &operation.entry_front_match_tolerance, 0.01f);
      changed |= ImGui::DragScalar("Tiny split area (stepover^2)",
                                   ImGuiDataType_Double,
                                   &operation.tiny_split_area_factor, 0.01f);
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

  changed |= ImGui::DragScalar("Simulation/feed pitch (mm)",
                               ImGuiDataType_Double, &operation.path_pitch,
                               0.01f);
  if (!cam::is_parametric(operation.mode)) {
    int method = static_cast<int>(operation.simulation_method);
    const char* methods[]{"Area / local stock", "Contact boundary (experimental)",
                          "Compare both (area feeds)", "Area / original reference"};
    if (ImGui::Combo("Engagement method", &method, methods, 4)) {
      operation.simulation_method = static_cast<wave::SimulationMethod>(method);
      changed = true;
    }
  }
  changed |= ImGui::Checkbox("Enter manual feed range",
                             &operation.manual_feed_range);
  if (operation.manual_feed_range) {
    changed |= ImGui::DragScalar("Full engagement (mm/min)",
                                 ImGuiDataType_Double,
                                 &operation.full_engagement_feed, 10.0f);
    changed |= ImGui::DragScalar("No engagement (mm/min)",
                                 ImGuiDataType_Double,
                                 &operation.no_engagement_feed, 10.0f);
  }
  changed |= ImGui::DragInt("Slice grid", &operation.slice_grid, 1.0f, 1, 256);
  changed |= ImGui::DragScalar("Simulation stock inflation (mm)",
                               ImGuiDataType_Double,
                               &operation.simulation_stock_inflation, 0.1f);

  if (changed) save_settings(app);
}

}  // namespace tinglecam_gui
