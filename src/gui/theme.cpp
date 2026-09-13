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

}  // namespace tinglecam_gui
