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
  // Stopped on purpose, before the first propagation step.
  Seeded,
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

// An explicit propagation source used when analysis resumes at a node layer.
// The source curve is the independently selectable identity. `initial_region`
// records the swept state behind it, which supplies direction without turning
// every other boundary of a shared connected band back into an active source.
struct WaveSeed {
  Path64 path;
  bool closed = false;
  Paths64 initial_region;
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
  // Optional physical boundary provenance for a wave resumed inside a smaller
  // remaining domain. When empty, topology derives it from domain - subject.
  // A cleared-prefix cut is a seed interface, not a new static wall.
  Paths64 static_boundaries;
  // When present, these are the only leading fronts allowed in generation
  // zero. This lets a resumed analysis retain its complete cached reached area
  // while activating only a subset of the entry curves on its boundary.
  std::vector<WaveSeed> explicit_entry_fronts;
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

// The wave before it has propagated at all: the domain, and generation zero
// seeded on its boundary. The entry fronts are exactly the connected pieces of
// that seed, so this answers what a wave could start from without paying for
// where it would then go. `build_wavefront` begins with precisely this.
WaveResult seed_wavefront(const Paths64& raw_subject,
                          const Paths64& raw_stock,
                          double stepover,
                          double softening = 0.0,
                          double lead_in = 0.0,
                          const EntryMute& muted = {});

// Propagates through an already isolated remaining domain from explicit node
// entry curves. With `cached_reached`, all entry bands remain in the copied
// state but only the supplied curves may produce the next wave; this is the
// later-generation counterpart of dropping an unselected generation-zero seed
// component. Without cached state, each curve is expanded into an ordinary
// standalone seed region.
WaveResult build_wavefront_from_seeds(
    const Paths64& raw_domain,
    const std::vector<WaveSeed>& seeds,
    double stepover,
    double softening = 0.0,
    std::size_t max_iterations = 2000,
    const Paths64& static_boundaries = {},
    const Paths64& cached_reached = {});

bool paths_equal_canonical(const Paths64& lhs, const Paths64& rhs);
double to_world(std::int64_t value);
std::int64_t to_integer(double value);
const char* stop_reason_name(StopReason reason);

}  // namespace wave
