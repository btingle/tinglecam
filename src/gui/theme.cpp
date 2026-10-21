#include "gui_internal.h"

#include <cstring>
#include <type_traits>

namespace tinglecam_gui {
namespace {

constexpr char kHex[] = "0123456789abcdef";

int hex_value(char value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  return -1;
}

}  // namespace

std::string ui_style_blob(const ImGuiStyle& style) {
  static_assert(std::is_trivially_copyable<ImGuiStyle>::value,
                "ImGuiStyle must remain a value type for theme storage");
  const auto* bytes = reinterpret_cast<const unsigned char*>(&style);
  std::string result;
  result.reserve(sizeof(style) * 2);
  for (std::size_t i = 0; i < sizeof(style); ++i) {
    result.push_back(kHex[bytes[i] >> 4]);
    result.push_back(kHex[bytes[i] & 0x0f]);
  }
  return result;
}

bool ui_style_from_blob(const std::string& blob, ImGuiStyle& style) {
  static_assert(std::is_trivially_copyable<ImGuiStyle>::value,
                "ImGuiStyle must remain a value type for theme storage");
  if (blob.size() != sizeof(style) * 2) return false;
  ImGuiStyle decoded;
  auto* bytes = reinterpret_cast<unsigned char*>(&decoded);
  for (std::size_t i = 0; i < sizeof(decoded); ++i) {
    const int high = hex_value(blob[i * 2]);
    const int low = hex_value(blob[i * 2 + 1]);
    if (high < 0 || low < 0) return false;
    bytes[i] = static_cast<unsigned char>((high << 4) | low);
  }
  std::memcpy(&style, &decoded, sizeof(style));
  return true;
}

bool light_theme(const App& app) {
  const ImVec4 background = app.unscaled_style.Colors[ImGuiCol_WindowBg];
  return background.x + background.y + background.z > 1.5f;
}

CanvasColours canvas_colours(const App& app) {
  CanvasColours colours;
  if (light_theme(app)) {
    colours.background_rgba = {0.86f, 0.87f, 0.88f, 1.0f};
    colours.caption = IM_COL32(70, 76, 86, 255);
    colours.message = IM_COL32(90, 96, 106, 255);
    colours.outline = IM_COL32(250, 250, 252, 220);
  } else {
    colours.background_rgba = {14.f / 255, 17.f / 255, 22.f / 255, 1.0f};
    colours.caption = IM_COL32(130, 142, 158, 255);
    colours.message = IM_COL32(120, 130, 145, 255);
    colours.outline = IM_COL32(12, 14, 18, 200);
  }
  const auto& c = colours.background_rgba;
  colours.background = ImGui::ColorConvertFloat4ToU32(ImVec4(c[0], c[1], c[2], c[3]));
  return colours;
}

ImVec4 severity_colour(const App& app, LogSeverity severity) {
  const bool light = light_theme(app);
  switch (severity) {
    case LogSeverity::Info:
      return light ? ImVec4(0.18f, 0.32f, 0.52f, 1.0f) : ImVec4(0.65f, 0.78f, 0.9f, 1.0f);
    case LogSeverity::Warning:
      return light ? ImVec4(0.62f, 0.36f, 0.0f, 1.0f) : ImVec4(1.0f, 0.72f, 0.35f, 1.0f);
    case LogSeverity::Error:
      return light ? ImVec4(0.72f, 0.12f, 0.1f, 1.0f) : ImVec4(1.0f, 0.45f, 0.4f, 1.0f);
  }
  return ImGui::GetStyleColorVec4(ImGuiCol_Text);
}

ImVec4 success_colour(const App& app) {
  return light_theme(app) ? ImVec4(0.1f, 0.45f, 0.22f, 1.0f)
                          : ImVec4(0.55f, 0.85f, 0.65f, 1.0f);
}

ImVec4 muted_colour(const App& app) {
  return light_theme(app) ? ImVec4(0.35f, 0.4f, 0.5f, 1.0f)
                          : ImVec4(0.62f, 0.68f, 0.78f, 1.0f);
}

}  // namespace tinglecam_gui
