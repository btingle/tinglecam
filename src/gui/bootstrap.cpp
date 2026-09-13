#include "gui_internal.h"
namespace tinglecam_gui {
// ---------------------------------------------------------------------------

namespace {

ImFont* load_font(const std::vector<std::filesystem::path>& candidates) {
  ImGuiIO& io = ImGui::GetIO();
  for (const auto& path : candidates) {
    std::error_code error;
    if (!std::filesystem::is_regular_file(path, error)) continue;
    if (ImFont* font = io.Fonts->AddFontFromFileTTF(path.string().c_str(), 16.0f))
      return font;
  }
  return nullptr;
}

std::vector<std::filesystem::path> font_candidates(
    const char* bundled_name, std::initializer_list<const char*> system_paths) {
  std::vector<std::filesystem::path> candidates;
  candidates.push_back(cam::data_directory() / "fonts" / bundled_name);
  for (const char* path : system_paths) candidates.emplace_back(path);
  return candidates;
}

}  // namespace

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

void load_ui_fonts(App& app) {
  if (app.theme_fonts_loaded || !ImGui::GetCurrentContext()) return;
  app.theme_sans_font = load_font(font_candidates(
      "ui-sans.ttf", {
          "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed.ttf",
          "/usr/share/fonts/truetype/ubuntu/UbuntuSans[wdth,wght].ttf",
          "/Library/Fonts/Arial.ttf",
          "C:/Windows/Fonts/segoeui.ttf"}));
  app.theme_mono_font = load_font(font_candidates(
      "ui-mono.ttf", {
          "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
          "/usr/share/fonts/truetype/ubuntu/UbuntuMono[wght].ttf",
          "/Library/Fonts/Menlo.ttc",
          "C:/Windows/Fonts/consola.ttf"}));
  app.theme_serif_font = load_font(font_candidates(
      "ui-serif.ttf", {
          "/usr/share/fonts/truetype/freefont/FreeSerif.ttf",
          "/usr/share/fonts/truetype/dejavu/DejaVuSerif.ttf",
          "/Library/Fonts/Georgia.ttf",
          "C:/Windows/Fonts/georgia.ttf"}));
  app.theme_fonts_loaded = true;
}

void apply_ui_theme(App& app) {
  if (!ImGui::GetCurrentContext()) return;

  load_ui_fonts(app);
  ImGuiStyle& style = ImGui::GetStyle();
  ImGuiStyle baseline;
  if (app.settings.ui_theme == cam::UiTheme::Paper)
    ImGui::StyleColorsLight(&baseline);
  else
    ImGui::StyleColorsDark(&baseline);
  baseline.FrameRounding = 3.0f;
  style = baseline;

  const auto custom = std::find_if(
      app.settings.custom_themes.begin(), app.settings.custom_themes.end(),
      [&](const cam::CustomUiTheme& theme) {
        return !app.settings.custom_theme.empty() &&
               theme.name == app.settings.custom_theme;
      });
  const bool has_custom =
      custom != app.settings.custom_themes.end() &&
      ui_style_from_blob(custom->style_blob, style);
  const auto set_font = [&](cam::UiFont font) {
    switch (font) {
      case cam::UiFont::Default: ImGui::GetIO().FontDefault = nullptr; break;
      case cam::UiFont::Sans: ImGui::GetIO().FontDefault = app.theme_sans_font; break;
      case cam::UiFont::Mono: ImGui::GetIO().FontDefault = app.theme_mono_font; break;
      case cam::UiFont::Serif: ImGui::GetIO().FontDefault = app.theme_serif_font; break;
    }
  };

  const auto colour = [](float r, float g, float b, float a = 1.0f) {
    return ImVec4(r, g, b, a);
  };
  if (!has_custom) switch (app.settings.ui_theme) {
    case cam::UiTheme::Default:
      ImGui::GetIO().FontDefault = nullptr;
      break;
    case cam::UiTheme::Slate:
      ImGui::GetIO().FontDefault = app.theme_mono_font
          ? app.theme_mono_font : app.theme_sans_font;
      style.Colors[ImGuiCol_WindowBg] = colour(0.075f, 0.090f, 0.120f);
      style.Colors[ImGuiCol_ChildBg] = colour(0.095f, 0.115f, 0.150f);
      style.Colors[ImGuiCol_PopupBg] = colour(0.105f, 0.125f, 0.165f);
      style.Colors[ImGuiCol_Border] = colour(0.240f, 0.300f, 0.380f);
      style.Colors[ImGuiCol_FrameBg] = colour(0.130f, 0.160f, 0.205f);
      style.Colors[ImGuiCol_FrameBgHovered] = colour(0.180f, 0.225f, 0.285f);
      style.Colors[ImGuiCol_FrameBgActive] = colour(0.220f, 0.285f, 0.360f);
      style.Colors[ImGuiCol_Header] = colour(0.145f, 0.205f, 0.285f);
      style.Colors[ImGuiCol_HeaderHovered] = colour(0.205f, 0.300f, 0.400f);
      style.Colors[ImGuiCol_HeaderActive] = colour(0.255f, 0.365f, 0.475f);
      style.Colors[ImGuiCol_Button] = colour(0.145f, 0.205f, 0.285f);
      style.Colors[ImGuiCol_ButtonHovered] = colour(0.205f, 0.300f, 0.400f);
      style.Colors[ImGuiCol_ButtonActive] = colour(0.255f, 0.365f, 0.475f);
      style.Colors[ImGuiCol_CheckMark] = colour(0.430f, 0.750f, 0.950f);
      style.Colors[ImGuiCol_SliderGrab] = colour(0.330f, 0.610f, 0.820f);
      style.Colors[ImGuiCol_SliderGrabActive] = colour(0.500f, 0.800f, 0.980f);
      style.Colors[ImGuiCol_Tab] = colour(0.105f, 0.145f, 0.195f);
      style.Colors[ImGuiCol_TabHovered] = colour(0.205f, 0.300f, 0.400f);
      style.Colors[ImGuiCol_TabActive] = colour(0.160f, 0.235f, 0.320f);
      style.Colors[ImGuiCol_Text] = colour(0.870f, 0.910f, 0.960f);
      style.Colors[ImGuiCol_TextDisabled] = colour(0.500f, 0.570f, 0.650f);
      break;
    case cam::UiTheme::Ember:
      ImGui::GetIO().FontDefault = app.theme_serif_font
          ? app.theme_serif_font : app.theme_sans_font;
      style.Colors[ImGuiCol_WindowBg] = colour(0.105f, 0.080f, 0.075f);
      style.Colors[ImGuiCol_ChildBg] = colour(0.135f, 0.100f, 0.090f);
      style.Colors[ImGuiCol_PopupBg] = colour(0.155f, 0.115f, 0.100f);
      style.Colors[ImGuiCol_Border] = colour(0.350f, 0.225f, 0.160f);
      style.Colors[ImGuiCol_FrameBg] = colour(0.190f, 0.135f, 0.110f);
      style.Colors[ImGuiCol_FrameBgHovered] = colour(0.285f, 0.190f, 0.135f);
      style.Colors[ImGuiCol_FrameBgActive] = colour(0.360f, 0.235f, 0.155f);
      style.Colors[ImGuiCol_Header] = colour(0.300f, 0.175f, 0.115f);
      style.Colors[ImGuiCol_HeaderHovered] = colour(0.450f, 0.255f, 0.145f);
      style.Colors[ImGuiCol_HeaderActive] = colour(0.560f, 0.305f, 0.165f);
      style.Colors[ImGuiCol_Button] = colour(0.300f, 0.175f, 0.115f);
      style.Colors[ImGuiCol_ButtonHovered] = colour(0.450f, 0.255f, 0.145f);
      style.Colors[ImGuiCol_ButtonActive] = colour(0.560f, 0.305f, 0.165f);
      style.Colors[ImGuiCol_CheckMark] = colour(1.000f, 0.620f, 0.270f);
      style.Colors[ImGuiCol_SliderGrab] = colour(0.900f, 0.420f, 0.180f);
      style.Colors[ImGuiCol_SliderGrabActive] = colour(1.000f, 0.650f, 0.300f);
      style.Colors[ImGuiCol_Tab] = colour(0.175f, 0.115f, 0.095f);
      style.Colors[ImGuiCol_TabHovered] = colour(0.450f, 0.255f, 0.145f);
      style.Colors[ImGuiCol_TabActive] = colour(0.330f, 0.190f, 0.120f);
      style.Colors[ImGuiCol_Text] = colour(0.960f, 0.900f, 0.840f);
      style.Colors[ImGuiCol_TextDisabled] = colour(0.620f, 0.500f, 0.420f);
      break;
    case cam::UiTheme::Paper:
      ImGui::GetIO().FontDefault = app.theme_sans_font;
      style.FrameRounding = 4.0f;
      style.WindowRounding = 4.0f;
      style.ChildRounding = 3.0f;
      style.PopupRounding = 4.0f;
      style.Colors[ImGuiCol_WindowBg] = colour(0.930f, 0.925f, 0.900f);
      style.Colors[ImGuiCol_ChildBg] = colour(0.965f, 0.960f, 0.940f);
      style.Colors[ImGuiCol_PopupBg] = colour(0.985f, 0.980f, 0.965f);
      style.Colors[ImGuiCol_Border] = colour(0.650f, 0.640f, 0.610f);
      style.Colors[ImGuiCol_FrameBg] = colour(0.850f, 0.845f, 0.820f);
      style.Colors[ImGuiCol_FrameBgHovered] = colour(0.770f, 0.800f, 0.850f);
      style.Colors[ImGuiCol_FrameBgActive] = colour(0.690f, 0.745f, 0.830f);
      style.Colors[ImGuiCol_Header] = colour(0.760f, 0.790f, 0.860f);
      style.Colors[ImGuiCol_HeaderHovered] = colour(0.650f, 0.710f, 0.820f);
      style.Colors[ImGuiCol_HeaderActive] = colour(0.550f, 0.640f, 0.780f);
      style.Colors[ImGuiCol_Button] = colour(0.760f, 0.790f, 0.860f);
      style.Colors[ImGuiCol_ButtonHovered] = colour(0.650f, 0.710f, 0.820f);
      style.Colors[ImGuiCol_ButtonActive] = colour(0.550f, 0.640f, 0.780f);
      style.Colors[ImGuiCol_CheckMark] = colour(0.100f, 0.300f, 0.700f);
      style.Colors[ImGuiCol_SliderGrab] = colour(0.220f, 0.430f, 0.760f);
      style.Colors[ImGuiCol_SliderGrabActive] = colour(0.100f, 0.300f, 0.700f);
      style.Colors[ImGuiCol_Tab] = colour(0.830f, 0.840f, 0.890f);
      style.Colors[ImGuiCol_TabHovered] = colour(0.650f, 0.710f, 0.820f);
      style.Colors[ImGuiCol_TabActive] = colour(0.730f, 0.780f, 0.880f);
      style.Colors[ImGuiCol_Text] = colour(0.110f, 0.110f, 0.120f);
      style.Colors[ImGuiCol_TextDisabled] = colour(0.420f, 0.420f, 0.430f);
      break;
  } else set_font(custom->font);

  app.unscaled_style = style;
  app.unscaled_style_captured = true;
  apply_ui_scale(app);
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
