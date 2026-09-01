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

// Entry points the wave must not propagate from. The wave is seeded on the
// boundary of its domain, and the connected pieces of that seed are the entry
// fronts: dropping one before propagation begins is what it means to decline
// an entry. A piece is dropped when it passes within `tolerance` of any anchor.
// Material only that piece could have reached is then left standing, which the
// plan reports rather than hides.
struct EntryMute {
  std::vector<Point64> anchors;
  double tolerance = 0.0;

  bool empty() const { return anchors.empty(); }
};

struct WaveResult {
  Paths64 subject;
  Paths64 stock;
  double stepover = 0.0;
  double softening = 0.0;
  // First outer stock path, retained for source compatibility with the
  // original rectangular-stock demo.
  Path64 stock_boundary;
  // How far outside the stock the propagation was seeded, and how many fronts
  // stand outside the stock wall as a result. Fronts below
  // `lead_in_generations` are in front of the material, so a cutter following
  // them takes up its bite over the run-in instead of at first contact. The
  // distance is used as given rather than rounded to whole stepovers: where the
  // first bite falls is the caller's decision, not the wave's. Zero leaves the
  // wave seeded on the stock wall.
  double lead_in = 0.0;
  std::size_t lead_in_generations = 0;
  // Everything the wave propagates through: the stock, plus the air in front of
  // it a lead-in opens up. With no lead-in this is the stock exactly. Downstream
  // stages bound the wave with this rather than with the stock, or the run-in
  // has nothing ahead of it to lead into and no outer wall to start from.
  Paths64 domain;
  std::vector<Paths64> generations;
  std::vector<Paths64> reached_generations;
  // Seed pieces an EntryMute removed before propagation.
  std::size_t muted_entries = 0;
  StopReason stop_reason = StopReason::Empty;
};

// Loads closed M/L/H/V/Z SVG paths and applies scale/translate/matrix/rotate
// transforms found on their ancestor groups and on the path itself.
SvgGeometry load_svg_closed_paths(const std::filesystem::path& filename);

// `lead_in` seeds the wave that far in front of the stock, at exactly the
// distance given. It is grown from the stock wall the wave actually starts at
// rather than from the stock solid, so no lead-in is manufactured across
// boundary the subject occludes.
WaveResult build_wavefront(const Paths64& raw_subject,
                           double allowance,
                           double stepover,
                           double softening = 0.0,
                           double lead_in = 0.0,
                           std::size_t max_iterations = 2000,
                           const EntryMute& muted = {});

WaveResult build_wavefront(const Paths64& raw_subject,
                           const Paths64& raw_stock,
                           double stepover,
                           double softening = 0.0,
                           double lead_in = 0.0,
                           std::size_t max_iterations = 2000,
                           const EntryMute& muted = {});

bool paths_equal_canonical(const Paths64& lhs, const Paths64& rhs);
double to_world(std::int64_t value);
std::int64_t to_integer(double value);
const char* stop_reason_name(StopReason reason);

}  // namespace wave
