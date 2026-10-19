#include <SDL.h>

#include <cstdlib>
#include <iostream>
#include <stdexcept>

#include "rendering/graphics_startup.h"

namespace {
void check(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
void unset(const char* name) {
#ifdef _WIN32
  _putenv_s(name, "");
#else
  unsetenv(name);
#endif
}
void clear() {
  for (const char* name :
       {"GALLIUM_DRIVER", "MESA_D3D12_DEFAULT_ADAPTER_NAME",
        "MESA_LOADER_DRIVER_OVERRIDE", "LIBGL_ALWAYS_SOFTWARE"})
    unset(name);
}
bool value(const char* name, const char* expected) {
  const char* actual = SDL_getenv(name);
  return actual && std::string(actual) == expected;
}
}  // namespace
int main() {
  using namespace wave::graphics;
  try {
    check(software_renderer("llvmpipe (LLVM)") &&
              software_renderer("D3D12 (Microsoft Basic Render Driver)") &&
              !software_renderer("D3D12 (NVIDIA GeForce RTX 5060)") &&
              !software_renderer("D3D12 (AMD Radeon Graphics)"),
          "The renderer readout distinguishes software from hardware");
    // This isolated test process modifies only its own environment.
    clear();
    int calls = 0;
    const auto hint = [&] {
      ++calls;
      return "NVIDIA test adapter";
    };
    check(!apply_wsl_defaults(false, hint).applied() && calls == 0,
          "Native Linux/Windows startup must remain unchanged");
    const auto automatic = apply_wsl_defaults(true, hint);
    check(automatic.driver && automatic.adapter && calls == 1 &&
              value("GALLIUM_DRIVER", "d3d12") &&
              value("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA test adapter"),
          "WSL startup selects hardware OpenGL on the preferred device");
    undo_defaults(automatic);
    check(!SDL_getenv("GALLIUM_DRIVER") &&
              !SDL_getenv("MESA_D3D12_DEFAULT_ADAPTER_NAME"),
          "A failed automatic startup can restore the original environment");
    for (const char* forced : {"1", "true", "TRUE"}) {
      clear();
      SDL_setenv("LIBGL_ALWAYS_SOFTWARE", forced, 1);
      check(!apply_wsl_defaults(true, hint).applied(),
            "Explicit software rendering must be respected");
    }
    clear();
    SDL_setenv("GALLIUM_DRIVER", "llvmpipe", 1);
    check(!apply_wsl_defaults(true, hint).applied() &&
              value("GALLIUM_DRIVER", "llvmpipe"),
          "An explicitly selected Gallium driver must be respected");
    clear();
    SDL_setenv("MESA_LOADER_DRIVER_OVERRIDE", "zink", 1);
    check(!apply_wsl_defaults(true, hint).applied(),
          "An explicit Mesa loader choice must be respected");
    clear();
    SDL_setenv("MESA_D3D12_DEFAULT_ADAPTER_NAME", "AMD", 1);
    const auto explicit_adapter = apply_wsl_defaults(true, hint);
    check(explicit_adapter.driver && !explicit_adapter.adapter &&
              value("MESA_D3D12_DEFAULT_ADAPTER_NAME", "AMD") && calls == 1,
          "An explicit graphics adapter must not be replaced or queried");
    undo_defaults(explicit_adapter);
    check(value("MESA_D3D12_DEFAULT_ADAPTER_NAME", "AMD"),
          "Fallback preserves an explicit adapter override");
    clear();
    const auto no_device =
        apply_wsl_defaults(true, [] { return std::string(); });
    check(no_device.driver && !no_device.adapter,
          "Hardware OpenGL still works when no preferred CUDA device is "
          "available");
    undo_defaults(no_device);
    clear();
    const auto unavailable = apply_wsl_defaults(
        true, []() -> std::string { throw std::runtime_error("No device"); });
    check(unavailable.driver && !unavailable.adapter,
          "Device-hint discovery failure must not prevent graphics startup");
    std::cout
        << "WSL graphics selection, overrides and fallback checks passed.\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
