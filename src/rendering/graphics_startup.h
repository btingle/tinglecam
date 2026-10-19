#pragma once

#include <functional>
#include <string>

namespace wave::graphics {
struct Defaults {
  bool driver = false;
  bool adapter = false;
  bool applied() const { return driver || adapter; }
};
bool running_under_wsl();
bool software_renderer(std::string name);
// Apply only missing defaults, before SDL/GL loads a driver. The adapter hint
// comes from the application; this helper and the GL renderer do not use CUDA.
Defaults apply_wsl_defaults(
    bool wsl, const std::function<std::string()>& preferred_adapter = {});
void undo_defaults(Defaults);
}  // namespace wave::graphics
