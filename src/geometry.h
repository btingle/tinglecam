#pragma once

#include <clipper2/clipper.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace wave {

using Clipper2Lib::Path64;
using Clipper2Lib::Paths64;
using Clipper2Lib::Point64;
using Clipper2Lib::Rect64;

constexpr double kIntegerScale = 1000.0;

struct SvgGeometry {
  Paths64 paths;
  Rect64 bounds;
  std::size_t source_path_count = 0;
};

enum class StopReason {
  Empty,
  Unchanged,
  IterationLimit,
};

struct WaveResult {
  Paths64 subject;
  Paths64 stock;
  // First outer stock path, retained for source compatibility with the
  // original rectangular-stock demo.
  Path64 stock_boundary;
  std::vector<Paths64> generations;
  std::vector<Paths64> reached_generations;
  StopReason stop_reason = StopReason::Empty;
};

// Loads closed M/L/H/V/Z SVG paths and applies scale/translate/matrix/rotate
// transforms found on their ancestor groups and on the path itself.
SvgGeometry load_svg_closed_paths(const std::filesystem::path& filename);

WaveResult build_wavefront(const Paths64& raw_subject,
                           double allowance,
                           double engagement,
                           std::size_t max_iterations = 2000);

WaveResult build_wavefront(const Paths64& raw_subject,
                           const Paths64& raw_stock,
                           double engagement,
                           std::size_t max_iterations = 2000);

bool paths_equal_canonical(const Paths64& lhs, const Paths64& rhs);
double to_world(std::int64_t value);
std::int64_t to_integer(double value);
const char* stop_reason_name(StopReason reason);

}  // namespace wave
