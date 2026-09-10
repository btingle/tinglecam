#include "wave_demo_state.h"
#include <SDL_opengl.h>

#include <fstream>
namespace wave_demo {

StartupInputs resolve_inputs(int argc, char** argv) {
  StartupInputs result;
  for (int i = 1; i < argc; ++i) {
    const std::string_view argument = argv[i];
    auto value_after = [&](std::string_view option) -> std::filesystem::path {
      if (i + 1 >= argc) throw std::runtime_error(std::string(option) + " requires a path");
      return argv[++i];
    };
    auto float_after = [&](std::string_view option) -> float {
      const std::string text = value_after(option).string();
      std::size_t consumed = 0;
      float value = 0.0f;
      try {
        value = std::stof(text, &consumed);
      } catch (const std::exception&) {
        throw std::runtime_error(std::string(option) + " requires a number");
      }
      if (consumed != text.size() || !std::isfinite(value))
        throw std::runtime_error(std::string(option) + " requires a finite number");
      return value;
    };
    auto int_after = [&](std::string_view option) -> int {
      const std::string text = value_after(option).string();
      std::size_t consumed = 0;
      int value = 0;
      try {
        value = std::stoi(text, &consumed);
      } catch (const std::exception&) {
        throw std::runtime_error(std::string(option) + " requires an integer");
      }
      if (consumed != text.size())
        throw std::runtime_error(std::string(option) + " requires an integer");
      return value;
    };
    if (argument == "--help" || argument == "-h") result.help = true;
    else if (argument == "--dump-analysis") result.dump_analysis = true;
    else if (argument == "--svg") result.svg = value_after(argument);
    else if (argument == "--subject-mesh") result.subject_mesh = value_after(argument);
    else if (argument == "--stock-mesh") result.stock_mesh = value_after(argument);
    else if (argument == "--screenshot") result.screenshot = value_after(argument);
    else if (argument == "--subject-inflate") result.subject_inflation = float_after(argument);
    else if (argument == "--stock-inflate") result.stock_inflation = float_after(argument);
    else if (argument == "--lead-in-distance")
      result.lead_in_distance = float_after(argument);
    else if (argument == "--lead-in") result.lead_in = true;
    else if (argument == "--no-lead-in") result.lead_in = false;
    else if (argument == "--allowance") result.allowance = float_after(argument);
    else if (argument == "--stepover") result.stepover = float_after(argument);
    else if (argument == "--round-waves") result.rounded_waves = true;
    else if (argument == "--mitered-waves") result.rounded_waves = false;
    else if (argument == "--slice-z") result.slice_z = float_after(argument);
    else if (argument == "--query-grid") result.query_grid = int_after(argument);
    else if (argument == "--analysis-step")
      result.analysis_step = int_after(argument);
    else if (argument == "--analysis-layers")
      result.analysis_layers = int_after(argument);
    else if (argument == "--tiny-split-area-factor")
      result.tiny_split_area_factor = float_after(argument);
    else if (!argument.starts_with("--") && result.svg.empty()) result.svg = argument;
    else throw std::runtime_error("Unknown command-line argument: " + std::string(argument));
  }
  if (!result.svg.empty()) return result;
  const std::filesystem::path local = "wavetest.svg";
  if (std::filesystem::exists(local)) {
    result.svg = local;
    return result;
  }
  const auto beside_executable = std::filesystem::path(argv[0]).parent_path() / "wavetest.svg";
  result.svg = beside_executable;
  return result;
}

void print_help(const char* executable) {
  std::cout
      << "Usage: " << executable << " [SVG] [options]\n\n"
      << "Mesh/SVG slicing and Clipper2 wavefront topology analysis.\n\n"
      << "Input options:\n"
      << "  --svg PATH              2D fallback subject (default: wavetest.svg)\n"
      << "  --subject-mesh PATH     Load an OBJ or STL subject mesh\n"
      << "  --stock-mesh PATH       Load an OBJ or STL stock mesh\n\n"
      << "Geometry options:\n"
      << "  --slice-z VALUE         Initial mesh slice height\n"
      << "  --query-grid N          GPU query grid resolution, 1..256 (default: 32)\n"
      << "  --subject-inflate VALUE Signed subject boundary offset (default: 0)\n"
      << "  --stock-inflate VALUE   Signed stock boundary offset (default: 0)\n"
      << "  --lead-in-distance VALUE\n"
      << "                           Distance to seed in front of the stock wall\n"
      << "  --lead-in, --no-lead-in Enable or disable that exterior seed distance\n"
      << "                           so the first passes engage gradually (default: on)\n"
      << "  --allowance VALUE       Fallback stock allowance when no stock mesh is loaded\n"
      << "  --stepover VALUE        Wave propagation distance per generation\n"
      << "  --round-waves           Use round offset joins (default)\n"
      << "  --mitered-waves         Use sharp miter offset joins\n\n"
      << "Other options:\n"
      << "  --dump-analysis        Print the recorded decision process and exit\n"
      << "  --analysis-step N      Initially display recorded decision frame N\n"
      << "  --analysis-layers N    Maximum recursive decision layers\n"
      << "  --tiny-split-area-factor VALUE\n"
      << "                         Ignore terminal split branches below VALUE *"
         " stepover^2 (default: 0.5)\n"
      << "  --screenshot PATH       Render one frame to a binary PPM file and exit\n"
      << "  -h, --help              Show this help and exit\n\n"
      << "Positive inflation expands a boundary; negative inflation contracts it.\n";
}

void save_screenshot(const std::filesystem::path& filename, int width, int height) {
  std::vector<unsigned char> pixels(static_cast<std::size_t>(width) * height * 3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
  std::ofstream stream(filename, std::ios::binary);
  if (!stream) throw std::runtime_error("Unable to write screenshot: " + filename.string());
  stream << "P6\n" << width << ' ' << height << "\n255\n";
  const std::size_t row_bytes = static_cast<std::size_t>(width) * 3;
  for (int y = height - 1; y >= 0; --y)
    stream.write(reinterpret_cast<const char*>(pixels.data() + y * row_bytes), row_bytes);
}
}  // namespace wave_demo
