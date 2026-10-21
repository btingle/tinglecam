#include "gui_internal.h"

namespace tinglecam_gui {
namespace {

bool save_settings_impl(App& app) {
  if (app.screenshot_mode) return true;
  try {
    cam::save_user_settings(app.settings_file, app.settings);
    return true;
  } catch (const std::exception& error) {
    say(app, error.what(), true);
    return false;
  }
}

constexpr cam::UiTheme kUiThemes[]{cam::UiTheme::Default, cam::UiTheme::Slate,
                                   cam::UiTheme::Ember, cam::UiTheme::Paper};

constexpr cam::UiFont kUiFonts[]{cam::UiFont::Default, cam::UiFont::Sans,
                                 cam::UiFont::Mono, cam::UiFont::Serif};

cam::CustomUiTheme* active_custom_theme(App& app) {
  if (app.settings.custom_theme.empty()) return nullptr;
  for (auto& theme : app.settings.custom_themes)
    if (theme.name == app.settings.custom_theme) return &theme;
  return nullptr;
}

bool theme_selector(App& app) {
  const cam::CustomUiTheme* custom = active_custom_theme(app);
  const char* preview = custom ? custom->name.c_str()
                               : cam::ui_theme_name(app.settings.ui_theme);
  bool changed = false;
  property_label("Theme");
  ImGui::SetNextItemWidth(-FLT_MIN);
  if (ImGui::BeginCombo("##theme", preview)) {
    for (const cam::UiTheme theme : kUiThemes) {
      const bool selected = !custom && app.settings.ui_theme == theme;
      if (ImGui::Selectable(cam::ui_theme_name(theme), selected)) {
        app.settings.ui_theme = theme;
        app.settings.custom_theme.clear();
        changed = true;
      }
    }
    for (const auto& theme : app.settings.custom_themes) {
      const bool selected = custom && custom->name == theme.name;
      if (ImGui::Selectable(theme.name.c_str(), selected)) {
        app.settings.custom_theme = theme.name;
        changed = true;
      }
    }
    ImGui::EndCombo();
  }
  if (changed) apply_ui_theme(app);
  return changed;
}

void custom_themes(App& app) {
  static std::array<char, 128> theme_name{};
  ImGui::SetNextItemWidth(ui_size(app, 200.0f));
  ImGui::InputTextWithHint("##theme-name", "New theme name", theme_name.data(),
                           theme_name.size());
  ImGui::SameLine();
  if (ImGui::Button("Save as theme")) {
    const std::string name(theme_name.data());
    if (name.empty()) {
      say(app, "Theme name cannot be empty", true);
    } else if (std::any_of(std::begin(kUiThemes), std::end(kUiThemes),
                           [&](cam::UiTheme theme) {
                             return name == cam::ui_theme_name(theme);
                           })) {
      say(app, "That name is a built-in theme", true);
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
      apply_ui_theme(app);
      save_settings_impl(app);
    }
  }
  if (cam::CustomUiTheme* custom = active_custom_theme(app)) {
    if (begin_properties("custom-theme")) {
      cam::UiFont font = custom->font;
      if (prop_enum("Theme font", font, cam::ui_font_name, kUiFonts, std::size(kUiFonts))) {
        custom->font = font;
        apply_ui_theme(app);
        save_settings_impl(app);
      }
      end_properties();
    }
    if (ImGui::Button("Save current style")) {
      custom->style_blob = ui_style_blob(app.unscaled_style);
      save_settings_impl(app);
    }
    ImGui::SameLine();
    if (ImGui::Button("Delete theme")) {
      const std::string name = custom->name;
      std::erase_if(app.settings.custom_themes,
                    [&](const cam::CustomUiTheme& theme) { return theme.name == name; });
      app.settings.custom_theme.clear();
      app.settings.ui_theme = cam::UiTheme::Default;
      apply_ui_theme(app);
      save_settings_impl(app);
    }
  }
  if (ImGui::TreeNode("Style editor")) {
    ImGui::GetStyle() = app.unscaled_style;
    ImGuiStyle reference = app.unscaled_style;
    ImGui::ShowStyleEditor(&reference);
    app.unscaled_style = ImGui::GetStyle();
    apply_ui_scale(app);
    if (active_custom_theme(app))
      ImGui::TextDisabled("Press Save current style to keep changes.");
    ImGui::TreePop();
  }
}

}  // namespace

bool save_settings(App& app) { return save_settings_impl(app); }

void settings_window(App& app) {
  if (!app.show_settings) return;
  ImGui::SetNextWindowSize(ImVec2(ui_size(app, 520.0f), ui_size(app, 420.0f)),
                           ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Settings", &app.show_settings)) {
    ImGui::End();
    return;
  }
  bool changed = false;
  if (begin_properties("appearance")) {
    changed |= theme_selector(app);
    double scale = app.settings.ui_scale;
    const double minimum = 0.5, maximum = 3.0;
    property_label("UI scale", "Scales text and controls. Panel widths stay as "
                               "you set them.");
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::SliderScalar("##scale", ImGuiDataType_Double, &scale, &minimum, &maximum,
                            "%.2fx")) {
      app.settings.ui_scale = std::clamp(scale, 0.5, 3.0);
      apply_ui_scale(app);
      changed = true;
    }
    changed |= prop_check("Debug information", app.settings.show_debug,
                          "Planner overlays and detailed readouts in the views.");
    end_properties();
  }
  if (ImGui::Button("Reset panel layout")) {
    app.settings.panel_layout = cam::default_panel_layout();
    changed = true;
  }
  if (app.settings.show_debug && ImGui::CollapsingHeader("Custom themes"))
    custom_themes(app);
  ImGui::SeparatorText("About");
  if (!app.graphics_renderer.empty())
    ImGui::TextWrapped("Renderer: %s", app.graphics_renderer.c_str());
  if (app.software_graphics)
    ImGui::TextColored(severity_colour(app, LogSeverity::Warning),
                       "Software OpenGL is active; 3D playback may be slow.");
  ImGui::TextDisabled("Settings: %s", app.settings_file.string().c_str());
  ImGui::TextDisabled("Tools: %s", app.tools_file.string().c_str());
  if (changed) save_settings_impl(app);
  ImGui::End();
}

}  // namespace tinglecam_gui
