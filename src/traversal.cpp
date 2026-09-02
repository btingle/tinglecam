#include "traversal.h"

#include "routing.h"

#include <clipper2/clipper.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <array>
#include <map>
#include <stdexcept>
#include <utility>

namespace wave {
namespace {

using namespace Clipper2Lib;

double distance_mm(const Point64& a, const Point64& b) {
  return std::hypot(to_world(b.x - a.x), to_world(b.y - a.y));
}

double path_length_mm(const Path64& path) {
  double result = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i)
    result += distance_mm(path[i - 1], path[i]);
  return result;
}

double open_length_integer(const Paths64& paths) {
  double result = 0.0;
  for (const Path64& path : paths)
    for (std::size_t i = 1; i < path.size(); ++i)
      result += std::hypot(static_cast<double>(path[i].x - path[i - 1].x),
                           static_cast<double>(path[i].y - path[i - 1].y));
  return result;
}

bool legal_segment(const Point64& from, const Point64& to,
                   const Paths64& region) {
  if (from == to) return true;
  if (region.empty()) return false;
  Clipper64 clipper;
  clipper.AddOpenSubject(Paths64{Path64{from, to}});
  clipper.AddClip(region);
  Paths64 closed;
  Paths64 open;
  if (!clipper.Execute(ClipType::Intersection, FillRule::NonZero, closed, open))
    return false;
  const double wanted = std::hypot(static_cast<double>(to.x - from.x),
                                   static_cast<double>(to.y - from.y));
  return open_length_integer(open) + 1.0 >= wanted;
}

struct Connector {
  bool stay_down = false;
  double minutes = 0.0;
  // Where the connector goes, and how. A retract is the two endpoints at the
  // safe plane; a stay-down is the way across at depth, which is a straight
  // line where one is legal and a routed one where it is not.
  Path64 path;
};

// The ways across at depth, remembered. The ordering search asks about the same
// pairs of endpoints over and over -- every variant against every other -- and
// routing one is a good deal more work than measuring it.
class Crossings {
 public:
  // The region is pulled in by what the router may spend before it is asked
  // for anything. That slack is there to forgive two roundings of one wall
  // disagreeing, which is what lets a connector run along that wall at all;
  // spent instead against the part, or against the stock allowance standing in
  // front of it, it is a cut into the finished surface. Taken off first, a
  // route that uses all of it still only reaches the edge of where the tool was
  // allowed. The straight-line test above is untouched by this: it demands the
  // whole segment inside the region as given, and needs no such margin.
  Crossings(const Paths64& region, double slack)
      : router_(slack > 0.0 ? InflatePaths(region, -2.0 * slack,
                                           JoinType::Miter, EndType::Polygon)
                            : Paths64{},
                slack) {}

  const std::optional<Path64>& between(const Point64& from,
                                       const Point64& to) const {
    const Key key{from.x, from.y, to.x, to.y};
    const auto found = cache_.find(key);
    if (found != cache_.end()) return found->second;
    return cache_.emplace(key, router_.route(from, to)).first->second;
  }

 private:
  using Key = std::array<std::int64_t, 4>;
  Router router_;
  mutable std::map<Key, std::optional<Path64>> cache_;
};

Point64 safe_landing(const TraversalVariant& variant) {
  if (variant.safe_approach && !variant.safe_approach->empty())
    return variant.safe_approach->front();
  return variant.path.front();
}

double approach_length(const TraversalVariant& variant) {
  return variant.safe_approach ? path_length_mm(*variant.safe_approach) : 0.0;
}

double vertical_rapid(const TraversalSettings& settings) {
  return settings.rapid_feed_z > 0.0 ? settings.rapid_feed_z
                                     : settings.rapid_feed;
}

Connector connector(const Point64& from, const TraversalVariant& variant,
                    const TraversalSettings& settings,
                    const Crossings& crossings) {
  const Point64& target = variant.path.front();
  const Point64 landing = safe_landing(variant);
  const double vertical =
      2.0 * std::abs(settings.rapid_plane - settings.floor_z);
  const double retract =
      distance_mm(from, landing) / settings.rapid_feed +
      vertical / vertical_rapid(settings) +
      approach_length(variant) / settings.cutting_feed;
  if (settings.allow_stay_down) {
    // The straight line first, because it is both the cheapest to test and the
    // best answer whenever it is available.
    Path64 across;
    if (legal_segment(from, target, settings.stay_down_region)) {
      across = Path64{from, target};
    } else if (settings.stay_down_slack > 0.0) {
      // Blocked, which used to end the matter. Two contours clipped to the
      // same stock can finish and start a few millimetres apart along its wall
      // with the wall itself between them, and lifting the whole height of the
      // job to cross that is the wrong answer by an order of magnitude.
      if (const auto& routed = crossings.between(from, target))
        across = *routed;
    }
    if (across.size() >= 2) {
      const double stay_down = path_length_mm(across) / settings.cutting_feed;
      if (settings.prefer_stay_down_when_legal || stay_down <= retract)
        return {true, stay_down, std::move(across)};
    }
  }
  return {false, retract, Path64{from, landing}};
}

double initial_entry_minutes(const TraversalVariant& variant,
                             const TraversalSettings& settings) {
  return std::abs(settings.rapid_plane - settings.floor_z) /
             vertical_rapid(settings) +
         approach_length(variant) / settings.cutting_feed;
}

struct State {
  double cost = std::numeric_limits<double>::infinity();
  int previous_task = -1;
  int previous_variant = -1;
};

std::vector<TraversalChoice> exact_order(
    const std::vector<TraversalTask>& tasks,
    const TraversalSettings& settings, const Crossings& crossings) {
  const std::size_t count = tasks.size();
  const std::size_t masks = std::size_t{1} << count;
  std::vector<std::vector<std::vector<State>>> states(masks);
  for (std::size_t mask = 0; mask < masks; ++mask) {
    states[mask].resize(count);
    for (std::size_t task = 0; task < count; ++task)
      states[mask][task].resize(tasks[task].variants.size());
  }

  for (std::size_t task = 0; task < count; ++task)
    for (std::size_t variant = 0; variant < tasks[task].variants.size(); ++variant) {
      const TraversalVariant& candidate = tasks[task].variants[variant];
      const Path64& path = candidate.path;
      const double initial = settings.initial_position
                                 ? connector(*settings.initial_position, candidate,
                                             settings, crossings)
                                       .minutes
                                 : initial_entry_minutes(candidate, settings);
      states[std::size_t{1} << task][task][variant].cost =
          initial + path_length_mm(path) / settings.cutting_feed;
    }

  for (std::size_t mask = 1; mask < masks; ++mask) {
    for (std::size_t last = 0; last < count; ++last) {
      if ((mask & (std::size_t{1} << last)) == 0) continue;
      for (std::size_t last_variant = 0;
           last_variant < tasks[last].variants.size(); ++last_variant) {
        const State current = states[mask][last][last_variant];
        if (!std::isfinite(current.cost)) continue;
        const Point64& from = tasks[last].variants[last_variant].path.back();
        for (std::size_t next = 0; next < count; ++next) {
          if ((mask & (std::size_t{1} << next)) != 0) continue;
          const std::size_t next_mask = mask | (std::size_t{1} << next);
          for (std::size_t next_variant = 0;
               next_variant < tasks[next].variants.size(); ++next_variant) {
            const TraversalVariant& next_choice =
                tasks[next].variants[next_variant];
            const Path64& path = next_choice.path;
            const double candidate =
                current.cost +
                connector(from, next_choice, settings, crossings).minutes +
                path_length_mm(path) / settings.cutting_feed;
            State& destination = states[next_mask][next][next_variant];
            if (candidate >= destination.cost) continue;
            destination.cost = candidate;
            destination.previous_task = static_cast<int>(last);
            destination.previous_variant = static_cast<int>(last_variant);
          }
        }
      }
    }
  }

  const std::size_t full = masks - 1;
  std::size_t last = 0;
  std::size_t variant = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t task = 0; task < count; ++task)
    for (std::size_t candidate = 0; candidate < tasks[task].variants.size(); ++candidate)
      if (states[full][task][candidate].cost < best) {
        best = states[full][task][candidate].cost;
        last = task;
        variant = candidate;
      }

  std::vector<TraversalChoice> reversed;
  std::size_t mask = full;
  while (true) {
    reversed.push_back({last, variant});
    const State state = states[mask][last][variant];
    if (state.previous_task < 0) break;
    mask &= ~(std::size_t{1} << last);
    last = static_cast<std::size_t>(state.previous_task);
    variant = static_cast<std::size_t>(state.previous_variant);
  }
  std::reverse(reversed.begin(), reversed.end());
  return reversed;
}

std::vector<TraversalChoice> greedy_order(
    const std::vector<TraversalTask>& tasks,
    const TraversalSettings& settings, const Crossings& crossings) {
  std::vector<bool> used(tasks.size(), false);
  std::vector<TraversalChoice> order;
  order.reserve(tasks.size());
  // With no preferred XY origin at the safe plane, begin with the shortest
  // task. An at-depth caller instead chooses the cheapest legal connection.
  TraversalChoice first;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t task = 0; task < tasks.size(); ++task)
    for (std::size_t variant = 0; variant < tasks[task].variants.size(); ++variant) {
      const TraversalVariant& candidate = tasks[task].variants[variant];
      const Path64& path = candidate.path;
      const double cost = path_length_mm(path) / settings.cutting_feed +
                          (settings.initial_position
                               ? connector(*settings.initial_position, candidate,
                                           settings, crossings)
                                     .minutes
                               : initial_entry_minutes(candidate, settings));
      if (cost < best) {
        best = cost;
        first = {task, variant};
      }
    }
  order.push_back(first);
  used[first.task] = true;
  while (order.size() < tasks.size()) {
    const Path64& previous =
        tasks[order.back().task].variants[order.back().variant].path;
    TraversalChoice choice;
    best = std::numeric_limits<double>::infinity();
    for (std::size_t task = 0; task < tasks.size(); ++task) {
      if (used[task]) continue;
      for (std::size_t variant = 0; variant < tasks[task].variants.size(); ++variant) {
        const TraversalVariant& candidate = tasks[task].variants[variant];
        const Path64& path = candidate.path;
        const double cost =
            connector(previous.back(), candidate, settings, crossings).minutes +
                            path_length_mm(path) / settings.cutting_feed;
        if (cost < best) {
          best = cost;
          choice = {task, variant};
        }
      }
    }
    used[choice.task] = true;
    order.push_back(choice);
  }
  return order;
}

void add_move(Toolpath& toolpath, MoveKind kind, Path64 path) {
  if (path.empty()) return;
  const double length = path_length_mm(path);
  toolpath.moves.push_back({kind, std::move(path), {}, length});
  switch (kind) {
    case MoveKind::Cut:
      ++toolpath.stats.cut_count;
      toolpath.stats.cut_length += length;
      break;
    case MoveKind::Link:
      ++toolpath.stats.link_count;
      toolpath.stats.link_length += length;
      break;
    case MoveKind::Retract:
      ++toolpath.stats.retract_count;
      toolpath.stats.retract_length += length;
      break;
    default: break;
  }
}

}  // namespace

TraversalResult plan_traversal(const std::vector<TraversalTask>& tasks,
                               const TraversalSettings& settings) {
  if (!(settings.cutting_feed > 0.0) || !(settings.rapid_feed > 0.0) ||
      settings.rapid_feed_z < 0.0 ||
      !std::isfinite(settings.cutting_feed) ||
      !std::isfinite(settings.rapid_feed) ||
      !std::isfinite(settings.rapid_feed_z) ||
      !std::isfinite(settings.floor_z) ||
      !std::isfinite(settings.rapid_plane))
    throw std::invalid_argument("Traversal feeds and heights must be finite and positive");
  std::vector<TraversalTask> usable;
  usable.reserve(tasks.size());
  for (const TraversalTask& task : tasks) {
    TraversalTask copy;
    copy.source = task.source;
    for (const TraversalVariant& variant : task.variants) {
      if (variant.path.size() < 2) continue;
      if (variant.safe_approach &&
          (variant.safe_approach->empty() ||
           variant.safe_approach->back() != variant.path.front()))
        throw std::invalid_argument(
            "Traversal safe approach must end at the task path");
      copy.variants.push_back(variant);
    }
    if (!copy.variants.empty()) usable.push_back(std::move(copy));
  }
  TraversalResult result;
  if (usable.empty()) return result;
  std::size_t variant_count = 0;
  for (const TraversalTask& task : usable) variant_count += task.variants.size();
  const Crossings crossings(settings.stay_down_region,
                            settings.stay_down_slack);
  result.order = usable.size() <= 10 && variant_count <= 64
                     ? exact_order(usable, settings, crossings)
                     : greedy_order(usable, settings, crossings);

  bool have_position = settings.initial_position.has_value();
  Point64 position = settings.initial_position.value_or(Point64(0, 0));
  for (const TraversalChoice choice : result.order) {
    const TraversalVariant& variant =
        usable[choice.task].variants[choice.variant];
    const Path64& path = variant.path;
    if (!have_position) {
      const Point64 landing = safe_landing(variant);
      add_move(result.toolpath, MoveKind::Retract, Path64{landing, landing});
      if (variant.safe_approach && variant.safe_approach->size() >= 2)
        add_move(result.toolpath, MoveKind::Cut, *variant.safe_approach);
      result.estimated_minutes += initial_entry_minutes(variant, settings);
      have_position = true;
    } else if (position != path.front()) {
      const Connector link = connector(position, variant, settings, crossings);
      add_move(result.toolpath, link.stay_down ? MoveKind::Link : MoveKind::Retract,
               link.path);
      if (!link.stay_down && variant.safe_approach &&
          variant.safe_approach->size() >= 2)
        add_move(result.toolpath, MoveKind::Cut, *variant.safe_approach);
      result.estimated_minutes += link.minutes;
    }
    add_move(result.toolpath, MoveKind::Cut, path);
    result.estimated_minutes += path_length_mm(path) / settings.cutting_feed;
    position = path.back();
  }
  return result;
}

}  // namespace wave
