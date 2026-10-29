#include "cam/surface_linking.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace cam {
std::string surface_contour_signature(const CadModel& model,
                                      const Operation& operation,
                                      const Tool& tool) {
  const auto& s = operation.surface;
  std::ostringstream key;
  key.imbue(std::locale::classic());
  key << "surface-contours-v3:" << model.fingerprint() << ':'
      << s.model_fingerprint << ':' << std::setprecision(17)
      << s.stepover << ':' << s.chord_tolerance << ':' << s.allowance << ':'
      << s.along_v << ':' << s.bidirectional << ':' << s.flip_normals << ':'
      << operation.path_pitch << ':' << tool.diameter << ':' << int(tool.type);
  for (auto face : std::set<std::size_t>(s.faces.begin(), s.faces.end()))
    key << ':' << face;
  if (model.orientation() != ModelOrientation{})
    for (int axis : model.orientation().axes) key << ':' << axis;
  return key.str();
}

namespace {
using Cell = std::array<long long, 3>;
Cell cell(wave::Vec3d p, double pitch) {
  return {static_cast<long long>(std::floor(p.x / pitch)),
          static_cast<long long>(std::floor(p.y / pitch)),
          static_cast<long long>(std::floor(p.z / pitch))};
}
struct SpanRef {
  std::size_t contour;
  const SurfaceSpan* span;
};
}

// Adjacency belongs to contact geometry, independently of cutter offsets or
// whether a stay-down motion can connect a particular pair of endpoints.
void detail::build_surface_adjacency(const CadModel& model, const Operation& operation,
                             SurfaceContourPreview& preview,
                             const ProgressCallback& progress) {
  auto report = [&] {
    if (progress && !progress(1.0, "Finding finishing contour neighbours"))
      throw std::runtime_error("Cancelled");
  };
  const auto& settings = operation.surface;
  const double reach = std::max(2 * settings.stepover,
                                2 * settings.chord_tolerance);
  std::vector<SpanRef> spans;
  std::map<std::size_t, std::map<std::size_t, std::vector<std::size_t>>> faces;
  std::vector<std::set<std::size_t>> graph(preview.contours.size());
  auto edge = [&](std::size_t a, std::size_t b) {
    if (a == b) return;
    graph[a].insert(b);
    graph[b].insert(a);
  };
  for (std::size_t id = 0; id < preview.contours.size(); ++id)
    for (const auto& span : preview.contours[id].spans) {
      faces[span.face][span.lane].push_back(spans.size());
      spans.push_back({id, &span});
    }
  auto evaluate = [&](const SurfaceSpan& span, double along, double across) {
    return model.evaluate(span.face, settings.along_v ? across : along,
                          settings.along_v ? along : across);
  };
  // On one face, only consecutive physical lanes can be neighbours. Keep
  // separate trim islands separate unless a sampled strip connects them.
  for (const auto& [face, lanes] : faces) {
    (void)face;
    for (auto it = lanes.begin(); it != lanes.end(); ++it) {
      report();
      auto next = std::next(it);
      if (next == lanes.end() || next->first != it->first + 1) continue;
      for (auto ai : it->second)
        for (auto bi : next->second) {
          const auto& a = *spans[ai].span;
          const auto& b = *spans[bi].span;
          const double low = std::max(a.along_min, b.along_min),
                       high = std::min(a.along_max, b.along_max);
          if (low > high) continue;
          bool connected = false;
          for (double fraction : {.5, .25, .75, 0., 1.}) {
            const double t = low + fraction * (high - low);
            bool inside = true;
            double length = 0;
            auto previous = evaluate(a, t, a.across);
            inside = previous.inside;
            for (int k = 1; k <= 8 && inside; ++k) {
              const auto p = evaluate(a, t, a.across +
                                      (b.across - a.across) * k / 8.0);
              inside = p.inside;
              length += wave::distance(previous.point, p.point);
              previous = p;
            }
            if (inside && length <= reach * (1 + 1e-6)) {
              connected = true;
              break;
            }
          }
          if (connected) edge(spans[ai].contour, spans[bi].contour);
        }
    }
  }
  // Across faces, spatial bins propose candidates, but only a native shared
  // edge may establish adjacency. Distance through that edge, rather than a
  // chord across the part, bounds the connection.
  struct Sample { std::size_t span; wave::Vec3d point; };
  std::map<Cell, std::vector<Sample>> bins;
  struct Candidate { double distance; wave::Vec3d a, b; };
  std::map<std::pair<std::size_t, std::size_t>, Candidate> candidates;
  std::map<std::pair<std::size_t, std::size_t>, bool> adjacent;
  for (std::size_t id = 0; id < spans.size(); ++id) {
    report();
    const auto& span = *spans[id].span;
    std::vector<wave::Vec3d> samples;
    for (std::size_t j = 0; j < span.contacts.size(); ++j) {
      const auto p = span.contacts[j];
      if (samples.empty()) samples.push_back(p);
      else {
        const auto a = samples.back();
        const double d = wave::distance(a, p);
        if (d < reach / 4 && j + 1 < span.contacts.size()) continue;
        const int count = std::max(1, int(std::ceil(d / (reach / 2))));
        for (int k = 1; k <= count; ++k)
          samples.push_back(wave::interpolate(a, p, double(k) / count));
      }
    }
    for (const auto p : samples) {
      const auto c = cell(p, reach);
      for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
          for (int z = -1; z <= 1; ++z) {
            const auto found = bins.find({c[0] + x, c[1] + y, c[2] + z});
            if (found == bins.end()) continue;
            for (const auto& q : found->second) {
              const auto other = spans[q.span].span->face;
              if (span.face == other) continue;
              const double d = wave::distance(p, q.point);
              if (d > reach) continue;
              const auto faces_key = std::minmax(span.face, other);
              auto [it, inserted] = adjacent.try_emplace(faces_key, false);
              if (inserted) it->second = model.adjacent(span.face, other);
              if (!it->second) continue;
              const auto key = std::make_pair(q.span, id);
              auto candidate = candidates.find(key);
              if (candidate == candidates.end() || d < candidate->second.distance)
                candidates[key] = {d, q.point, p};
            }
          }
    }
    for (const auto p : samples) bins[cell(p, reach)].push_back({id, p});
  }
  struct Connection { std::size_t a, b; double length; };
  std::vector<Connection> connections;
  std::map<std::pair<std::size_t, std::size_t>, double> nearest;
  for (const auto& [ids, candidate] : candidates) {
    report();
    const auto& a = *spans[ids.first].span;
    const auto& b = *spans[ids.second].span;
    const auto portal = model.shared_boundary(
        a.face, b.face, wave::interpolate(candidate.a, candidate.b, .5));
    if (!portal) continue;
    const auto pa = model.evaluate(a.face, (*portal)[0][0], (*portal)[0][1]);
    const auto pb = model.evaluate(b.face, (*portal)[1][0], (*portal)[1][1]);
    if (!pa.inside || !pb.inside) continue;
    const double length = wave::distance(candidate.a, pa.point) +
                          wave::distance(pa.point, pb.point) +
                          wave::distance(pb.point, candidate.b);
    if (length > reach * (1 + 1e-6)) continue;
    connections.push_back({ids.first, ids.second, length});
    for (auto key : {std::make_pair(ids.first, b.face),
                     std::make_pair(ids.second, a.face)}) {
      auto [it, inserted] = nearest.try_emplace(key, length);
      if (!inserted) it->second = std::min(it->second, length);
    }
  }
  for (const auto& connection : connections) {
    const auto a = connection.a, b = connection.b;
    // Mutual nearest spans avoid skipping boundary lanes on either face.
    const double slack = settings.chord_tolerance * 2;
    if (connection.length <= nearest.at({a, spans[b].span->face}) + slack &&
        connection.length <= nearest.at({b, spans[a].span->face}) + slack)
      edge(spans[a].contour, spans[b].contour);
  }
  preview.neighbours.resize(graph.size());
  for (std::size_t i = 0; i < graph.size(); ++i)
    preview.neighbours[i].assign(graph[i].begin(), graph[i].end());
}

SurfaceContourOrder propagate_surface_order(
    const SurfaceContourPreview& preview, const std::vector<std::size_t>& selected) {
  SurfaceContourOrder result;
  const auto count = preview.contours.size();
  const auto none = SurfaceContourOrder::none;
  result.depth.assign(count, none);
  result.parent.assign(count, none);
  result.root.assign(count, none);
  result.branch.assign(count, none);
  if (!preview.error.empty()) { result.error = preview.error; return result; }
  if (preview.neighbours.size() != count) {
    result.error = "Finishing contour adjacency is unavailable";
    return result;
  }
  if (selected.empty()) {
    result.unreachable = count;
    result.error = "Select at least one finishing contour as a propagation root";
    return result;
  }
  // Sort roots and neighbours so click order and worker scheduling cannot
  // change where two fronts meet. Every contour is claimed exactly once.
  std::set<std::size_t> roots(selected.begin(), selected.end());
  std::queue<std::size_t> pending;
  for (auto root : roots) {
    if (root >= count) {
      result.error = "A finishing root no longer exists; select roots again";
      return result;
    }
    result.depth[root] = 0;
    result.root[root] = root;
    pending.push(root);
  }
  std::vector<std::vector<std::size_t>> children(count);
  while (!pending.empty()) {
    const auto id = pending.front();
    pending.pop();
    auto neighbours = preview.neighbours[id];
    std::sort(neighbours.begin(), neighbours.end());
    for (auto next : neighbours) {
      if (next >= count) {
        result.error = "Invalid finishing contour adjacency";
        return result;
      }
      if (result.depth[next] != none) continue;
      result.depth[next] = result.depth[id] + 1;
      result.parent[next] = id;
      result.root[next] = result.root[id];
      result.branch[next] = result.depth[id] == 0 ? result.branches++
                                                  : result.branch[id];
      children[id].push_back(next);
      pending.push(next);
    }
  }
  result.unreachable = std::count(result.depth.begin(), result.depth.end(), none);
  if (result.unreachable)
    result.error = "Some finishing contours have no root; select a root in each disconnected group";
  // Complete each root and its branches before moving to another root. The
  // multi-source fill determines ownership before scheduling, so fronts still
  // meet in the same place even though their roots execute at different times.
  for (auto root : roots) {
    std::vector<std::size_t> stack{root};
    while (!stack.empty()) {
      const auto id = stack.back();
      stack.pop_back();
      result.sequence.push_back(id);
      stack.insert(stack.end(), children[id].rbegin(), children[id].rend());
    }
  }
  return result;
}
}  // namespace cam
