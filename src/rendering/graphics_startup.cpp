#include "graphics_startup.h"

#include <SDL.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace wave::graphics {
namespace {
bool enabled(const char* value) {
  if (!value || !*value) return false;
  std::string s(value);
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return char(std::tolower(c)); });
  return s != "0" && s != "false" && s != "no";
}
void unset(const char* name) {
#ifdef _WIN32
  _putenv_s(name, "");
#else
  unsetenv(name);
#endif
}
}  // namespace
bool running_under_wsl() {
#ifdef __linux__
  std::error_code error;
  if (!std::filesystem::exists("/dev/dxg", error)) return false;
  std::ifstream release("/proc/sys/kernel/osrelease");
  std::string line;
  std::getline(release, line);
  std::transform(line.begin(), line.end(), line.begin(),
                 [](unsigned char c) { return char(std::tolower(c)); });
  return line.find("microsoft") != std::string::npos;
#else
  return false;
#endif
}
bool software_renderer(std::string name) {
  std::transform(name.begin(), name.end(), name.begin(),
                 [](unsigned char c) { return char(std::tolower(c)); });
  for (const char* marker : {"llvmpipe", "softpipe", "software",
                             "microsoft basic render", "gdi generic"})
    if (name.find(marker) != std::string::npos) return true;
  return false;
}
Defaults apply_wsl_defaults(
    bool wsl, const std::function<std::string()>& preferred_adapter) {
  Defaults result;
  if (!wsl || enabled(SDL_getenv("LIBGL_ALWAYS_SOFTWARE")) ||
      SDL_getenv("GALLIUM_DRIVER") || SDL_getenv("MESA_LOADER_DRIVER_OVERRIDE"))
    return result;
  // In WSL, Mesa can otherwise choose llvmpipe despite usable D3D12 hardware.
  if (SDL_setenv("GALLIUM_DRIVER", "d3d12", 0) != 0)
    throw std::runtime_error("Could not select the WSL OpenGL driver");
  result.driver = true;
  if (!SDL_getenv("MESA_D3D12_DEFAULT_ADAPTER_NAME") && preferred_adapter) {
    // Device discovery is best effort: Mesa's own hardware adapter choice is
    // still useful if the application has no preferred device available.
    std::string name;
    try {
      name = preferred_adapter();
    } catch (...) {
    }
    if (!name.empty() &&
        SDL_setenv("MESA_D3D12_DEFAULT_ADAPTER_NAME", name.c_str(), 0) == 0)
      result.adapter = true;
  }
  return result;
}
void undo_defaults(Defaults defaults) {
  if (defaults.driver) unset("GALLIUM_DRIVER");
  if (defaults.adapter) unset("MESA_D3D12_DEFAULT_ADAPTER_NAME");
}
}  // namespace wave::graphics
