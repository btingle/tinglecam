#include "topology_detail.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wave {
using namespace detail;
using namespace Clipper2Lib;

FrontLineage build_front_lineage(const WaveResult& wave,
                                 double tiny_split_area_factor) {
  if (!(wave.stepover > 0.0)) throw std::invalid_argument("Stepover must be positive");
  if (wave.softening < 0.0 || wave.softening > wave.stepover)
    throw std::invalid_argument("Softening must be between zero and stepover");
  if (wave.generations.size() != wave.reached_generations.size())
    throw std::invalid_argument("Wave bands and cumulative reached sets do not match");
  if (!std::isfinite(tiny_split_area_factor) || tiny_split_area_factor < 0.0)
    throw std::invalid_argument("Tiny split area factor must be nonnegative");
  FrontLineage result;
  result.stepover = wave.stepover;
  if (wave.generations.empty()) return result;

  const Paths64 stock = wave.stock.empty() ? Paths64{wave.stock_boundary} : wave.stock;
  // The wave runs through the domain, which a lead-in extends past the stock
  // wall into open air. Bounding fronts by the stock instead would leave every
  // run-in band with nothing unreached ahead of it and no front to hand on, and
  // would call the stock wall a static boundary when the wave is meant to cross
  // it. Without a lead-in the two are the same region.
  const Paths64& region = wave.domain.empty() ? stock : wave.domain;
  const Paths64 free_space = Difference(region, wave.subject, FillRule::NonZero);
  // Contact identity belongs to the final machinable region, not to whichever
  // input polygon happened to contribute an edge. Using concatenated subject
  // and domain rings gives one physical boundary two IDs where inputs touch.
  const Paths64 static_boundaries = wave.static_boundaries.empty()
                                         ? free_space
                                         : wave.static_boundaries;
  result.boundary_components = static_boundaries;
  result.generations.reserve(wave.generations.size());

  for (std::size_t generation = 0; generation < wave.generations.size(); ++generation) {
    Paths64 unreached = Difference(free_space, wave.reached_generations[generation],
                                   FillRule::NonZero);
    unreached = Union(unreached, FillRule::NonZero);
    std::vector<Paths64> bands = band_components(wave.generations[generation]);
    const double contact_tolerance = boundary_slack(wave.stepover);
    std::vector<LeadingFront> fronts;
    if (generation == 0 && !wave.explicit_entry_fronts.empty()) {
      std::vector<Paths64> band_guards;
      band_guards.reserve(bands.size());
      for (const Paths64& band : bands)
        band_guards.push_back(InflatePaths(
            band, contact_tolerance, JoinType::Miter, EndType::Polygon));
      for (const WaveSeed& seed : wave.explicit_entry_fronts) {
        if (seed.path.size() < 2 || bands.empty()) continue;
        LeadingFront front;
        front.path = seed.path;
        front.closed = seed.closed;
        double best_overlap = -1.0;
        for (std::size_t band = 0; band < band_guards.size(); ++band) {
          const double overlap = open_intersection_length(front, band_guards[band]);
          if (overlap > best_overlap) {
            best_overlap = overlap;
            front.band = band;
          }
        }
        fronts.push_back(std::move(front));
      }
    } else {
      fronts = extract_leading_fronts(bands, unreached, static_boundaries,
                                      front_probe_pitch(wave.stepover));
    }
    classify_front_contacts(fronts, static_boundaries, contact_tolerance);
    result.generations.push_back({std::move(fronts), std::move(bands)});
  }

  // Entry identity belongs to the wave source itself. Deriving it from the
  // stock wall splits one source whenever the protected subject happens to
  // occlude that wall in several places. Generation zero is the connected
  // source boundary from which propagation actually starts (at the outside of
  // the run-in when one exists), so its fronts are the stable selectable entry
  // fronts.
  if (!result.generations.empty())
    result.entry_fronts = result.generations.front().fronts;

  const double stepover_integer = static_cast<double>(
      std::max<std::int64_t>(1, to_integer(wave.stepover)));
  const double softening_integer = static_cast<double>(
      std::clamp<std::int64_t>(to_integer(wave.softening), 0,
                               static_cast<std::int64_t>(stepover_integer)));
  const double tolerance = std::clamp(stepover_integer * 0.001, 4.0, 100.0);

  for (std::size_t generation = 1; generation < result.generations.size(); ++generation) {
    auto& previous = result.generations[generation - 1].fronts;
    auto& current = result.generations[generation].fronts;
    Paths64 band_guard = InflatePaths(wave.generations[generation], tolerance,
                                      JoinType::Miter, EndType::Polygon);
    std::vector<Paths64> terminal_reaches(previous.size());
    for (std::size_t parent = 0; parent < previous.size(); ++parent) {
      Paths64 candidate = front_reach(previous[parent], stepover_integer + tolerance,
                                      softening_integer);
      candidate = Intersect(candidate, band_guard, FillRule::NonZero);
      if (candidate.empty()) continue;
      // Retain only the material actually infiltrated during this transition
      // for terminal-contact classification. The enlarged band_guard is useful
      // when matching a child curve, but would make unrelated terminal reaches
      // look as though they overlap.
      terminal_reaches[parent] = Intersect(
          candidate, wave.generations[generation], FillRule::NonZero);
      for (std::size_t child = 0; child < current.size(); ++child) {
        if (open_intersection_length(current[child], candidate) <= tolerance) continue;
        previous[parent].children.push_back(child);
        current[child].parents.push_back(parent);
      }
    }

    // A merge normally leaves a current front carrying all of its parents.
    // When the parents consume the last material between them there is no
    // current front, so classify the contact from their swept regions and emit
    // a merge into an empty terminal node instead of unrelated terminations.
    DisjointSet terminal_sets(previous.size());
    for (std::size_t a = 0; a < previous.size(); ++a) {
      if (!previous[a].children.empty() || terminal_reaches[a].empty()) continue;
      for (std::size_t b = a + 1; b < previous.size(); ++b) {
        if (!previous[b].children.empty() || terminal_reaches[b].empty()) continue;
        if (terminal_reaches_touch(terminal_reaches[a], terminal_reaches[b],
                                   tolerance))
          terminal_sets.join(a, b);
      }
    }
    std::vector<std::vector<std::size_t>> terminal_groups(previous.size());
    for (std::size_t parent = 0; parent < previous.size(); ++parent) {
      if (!previous[parent].children.empty()) continue;
      terminal_groups[terminal_sets.find(parent)].push_back(parent);
    }
    std::vector<bool> merged_terminal(previous.size(), false);
    for (const auto& group : terminal_groups) {
      if (group.size() < 2) continue;
      std::vector<Point64> points;
      points.reserve(group.size());
      for (const std::size_t parent : group) {
        merged_terminal[parent] = true;
        points.push_back(representative_point(previous[parent]));
      }
      result.events.push_back({LineageEventKind::MergeTermination, generation,
                               group, {}, average_points(points)});
    }

    for (std::size_t parent = 0; parent < previous.size(); ++parent) {
      const auto& children = previous[parent].children;
      if (children.size() > 1) {
        std::vector<Point64> points;
        points.push_back(representative_point(previous[parent]));
        for (const auto child : children)
          points.push_back(representative_point(current[child]));
        result.events.push_back({LineageEventKind::Split, generation,
                                 {parent}, children, average_points(points)});
      } else if (children.empty() && !merged_terminal[parent]) {
        result.events.push_back({LineageEventKind::Termination, generation,
                                 {parent}, {}, representative_point(previous[parent])});
      }
    }
    for (std::size_t child = 0; child < current.size(); ++child) {
      const auto& parents = current[child].parents;
      if (parents.size() > 1) {
        std::vector<Point64> points;
        points.push_back(representative_point(current[child]));
        for (const auto parent : parents)
          points.push_back(representative_point(previous[parent]));
        result.events.push_back({LineageEventKind::Merge, generation,
                                 parents, {child}, average_points(points)});
      } else if (parents.empty()) {
        result.events.push_back({LineageEventKind::Birth, generation,
                                 {}, {child}, representative_point(current[child])});
      }
    }
  }
  reject_tiny_splits(result, tiny_split_area_factor);
  return result;
}

}  // namespace wave
