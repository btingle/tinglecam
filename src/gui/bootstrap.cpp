#include "gui_internal.h"
namespace tinglecam_gui {
// ---------------------------------------------------------------------------

void bootstrap(App& app) {
  const std::filesystem::path data = cam::data_directory();
  const std::filesystem::path config = cam::config_directory();
  app.tools_file = config / "tools.json";
  app.settings_file = config / "settings.json";
  app.tools_import_file = (data / "starter-tools.json").string();

  try {
    if (std::filesystem::exists(app.settings_file))
      app.settings = cam::load_user_settings(app.settings_file);
    else
      cam::save_user_settings(app.settings_file, app.settings);
  } catch (const std::exception& error) {
    say(app, error.what(), true);
  }

  try {
    app.machines = cam::load_machines(data / "machines");
    app.materials = cam::load_materials(data / "materials");
  } catch (const std::exception& error) {
    say(app, error.what(), true);
  }
  if (app.machines.machines.empty())
    say(app, "No machine profiles found under " + (data / "machines").string(), true);

  try {
    if (std::filesystem::exists(app.tools_file)) {
      app.tools = cam::load_tools(app.tools_file);
    } else {
      // First run: start from the shipped tools rather than an empty crib, and
      // write them into the user's own file so editing them is normal.
      app.tools = cam::load_tools(data / "starter-tools.json");
      if (!app.tools.tools.empty()) cam::save_tools(app.tools_file, app.tools);
    }
  } catch (const std::exception& error) {
    say(app, error.what(), true);
  }
  std::snprintf(app.save_path.data(), app.save_path.size(), "%s",
                (std::filesystem::current_path() / "tinglecam.nc").string().c_str());
}

void apply_ui_scale(App& app) {
  if (!ImGui::GetCurrentContext()) return;
  if (!app.unscaled_style_captured) {
    app.unscaled_style = ImGui::GetStyle();
    app.unscaled_style_captured = true;
  }
  const float scale = std::clamp(static_cast<float>(app.settings.ui_scale),
                                 0.5f, 3.0f);
  ImGui::GetStyle() = app.unscaled_style;
  ImGui::GetStyle().ScaleAllSizes(scale);
  ImGui::GetIO().FontGlobalScale = scale;
}
}  // namespace tinglecam_gui
