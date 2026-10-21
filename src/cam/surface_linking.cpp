#include "cam/surface_linking.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <numeric>
#include <stdexcept>
#include <tuple>

#include "cam/job_internal.h"
#include "cam/surface_finishing.h"

namespace cam::detail {
namespace {
using Cell = std::array<long long, 3>;
Cell cell(wave::Vec3d p, double pitch) {
  return {static_cast<long long>(std::floor(p.x / pitch)),
          static_cast<long long>(std::floor(p.y / pitch)),
          static_cast<long long>(std::floor(p.z / pitch))};
}
// Endpoint bins keep linking local even with thousands of contours.
using Bins = std::map<Cell, std::vector<std::size_t>>;
template <class F>
void nearby(const Bins& bins, wave::Vec3d p, double pitch, F visit) {
  const auto c = cell(p, pitch);
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        const auto it = bins.find({c[0] + x, c[1] + y, c[2] + z});
        if (it != bins.end())
          for (auto id : it->second) visit(id);
      }
}
wave::Vec3d endpoint(const SurfaceRun& run, bool end) {
  return end ? run.points.back() : run.points.front();
}
wave::Vec3d contact(const SurfaceRun& run, bool end) {
  return end ? run.last_contact : run.first_contact;
}
struct SurfaceEnd {
  std::size_t face;
  std::array<double, 2> uv;
  wave::Vec3d contact, tip;
};
SurfaceEnd surface_end(const SurfaceRun& run, bool end) {
  return {end ? run.last_face : run.first_face,
          end ? run.last_uv : run.first_uv, contact(run, end),
          endpoint(run, end)};
}
void reverse_run(SurfaceRun& run) {
  std::reverse(run.points.begin(), run.points.end());
  std::swap(run.first_face, run.last_face);
  std::swap(run.first_contact, run.last_contact);
  std::swap(run.first_uv, run.last_uv);
  std::swap(run.first_boundary, run.last_boundary);
}
}  // namespace

void link_surface_runs(const CadModel& model, const Operation& operation,
                       const Tool& tool, const Machine& machine,
                       const CuttingFeeds& feeds,
                       double rapid_plane, std::vector<SurfaceRun>& runs,
                       OperationPlan& result,
                       const ProgressCallback& progress,
                       std::vector<SurfaceRun>* joined, bool fixed_order) {
  const auto started = PlanningClock::now();
  const auto& settings = operation.surface;
  auto report = [&] {
    if (progress && !progress(1.0, "Joining CAD finishing contours"))
      throw std::runtime_error("Cancelled");
  };
  auto quantize = [](wave::Vec3d p) {
    p.x = wave::to_world(wave::to_integer(p.x));
    p.y = wave::to_world(wave::to_integer(p.y));
    return p;
  };
  auto evaluate = [&](std::size_t face,
                      std::array<double, 2> uv) -> std::optional<SurfacePoint> {
    ++result.metrics.surface_samples;
    SurfacePoint p;
    try {
      p = model.evaluate(face, uv[0], uv[1]);
    } catch (const std::runtime_error&) {
      return std::nullopt;
    }
    if (settings.flip_normals) {
      p.normal.x = -p.normal.x;
      p.normal.y = -p.normal.y;
      p.normal.z = -p.normal.z;
    }
    if (!p.inside) return std::nullopt;
    return p;
  };
  using Path = std::vector<wave::Vec3d>;
  using Sampler = std::function<std::optional<wave::Vec3d>(double)>;
  auto curve = [&](const Sampler& sample, wave::Vec3d a,
                   wave::Vec3d b) -> std::optional<Path> {
    Path path{a};
    const double tolerance =
        settings.chord_tolerance - std::sqrt(.5) / wave::kIntegerScale;
    std::function<bool(double, wave::Vec3d, double, wave::Vec3d, int)> refine =
        [&](double ta, wave::Vec3d p, double tb, wave::Vec3d q, int depth) {
          report();
          const auto m = sample((ta + tb) / 2), q1 = sample((3 * ta + tb) / 4),
                     q3 = sample((ta + 3 * tb) / 4);
          if (!m || !q1 || !q3 || path.size() > 4096) return false;
          const double error =
              std::max({wave::distance(*m, wave::interpolate(p, q, .5)),
                        wave::distance(*q1, wave::interpolate(p, q, .25)),
                        wave::distance(*q3, wave::interpolate(p, q, .75))});
          if (error > tolerance ||
              wave::distance(p, q) > std::min(operation.path_pitch, 1.0)) {
            if (depth >= 16) return false;
            return refine(ta, p, (ta + tb) / 2, *m, depth + 1) &&
                   refine((ta + tb) / 2, *m, tb, q, depth + 1);
          }
          path.push_back(q);
          return true;
        };
    if (!refine(0, a, 1, b, 0)) return std::nullopt;
    return path;
  };
  auto face_curve = [&](const SurfaceEnd& a, const SurfaceEnd& b) {
    return curve(
        [&](double t) -> std::optional<wave::Vec3d> {
          const auto p = evaluate(a.face, {a.uv[0] + t * (b.uv[0] - a.uv[0]),
                                           a.uv[1] + t * (b.uv[1] - a.uv[1])});
          if (!p) return std::nullopt;
          return quantize(ball_tip(*p, tool.diameter / 2, settings.allowance));
        },
        a.tip, b.tip);
  };
  auto surface_connection = [&](const SurfaceEnd& a,
                                const SurfaceEnd& b) -> std::optional<Path> {
    if (a.face == b.face) return face_curve(a, b);
    // Coincident seam samples already meet to the requested contour tolerance.
    if (wave::distance(a.contact, b.contact) < 1e-7 &&
        wave::distance(a.tip, b.tip) < settings.chord_tolerance)
      return Path{a.tip, b.tip};
    const auto portal = model.shared_boundary(
        a.face, b.face, wave::interpolate(a.contact, b.contact, .5));
    if (!portal) return std::nullopt;
    const auto pa = evaluate(a.face, (*portal)[0]),
               pb = evaluate(b.face, (*portal)[1]);
    if (!pa || !pb) return std::nullopt;
    const SurfaceEnd ea{
        a.face, (*portal)[0], pa->point,
        quantize(ball_tip(*pa, tool.diameter / 2, settings.allowance))};
    const SurfaceEnd eb{
        b.face, (*portal)[1], pb->point,
        quantize(ball_tip(*pb, tool.diameter / 2, settings.allowance))};
    auto left = face_curve(a, ea), right = face_curve(eb, b);
    if (!left || !right) return std::nullopt;
    // Across a sharp shared edge, interpolate the ball's contact normal around
    // that edge rather than cutting a chord through the offset contour.
    const auto corner = curve(
        [&](double t) -> std::optional<wave::Vec3d> {
          auto normal = wave::interpolate(pa->normal, pb->normal, t);
          const double length = std::sqrt(
              normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
          if (length < 1e-8) return std::nullopt;
          normal.x /= length;
          normal.y /= length;
          normal.z /= length;
          const SurfacePoint p{wave::interpolate(pa->point, pb->point, t),
                               normal, true};
          return quantize(ball_tip(p, tool.diameter / 2, settings.allowance));
        },
        ea.tip, eb.tip);
    if (!corner) return std::nullopt;
    left->insert(left->end(), corner->begin() + 1, corner->end());
    left->insert(left->end(), right->begin() + 1, right->end());
    return left;
  };
  auto connect = [&](const SurfaceEnd& a, const SurfaceEnd& b) {
    const auto began = PlanningClock::now();
    auto path = surface_connection(a, b);
    result.metrics.surface_sampling_milliseconds += elapsed_milliseconds(began);
    return path;
  };
  std::vector<SurfaceRun> chains;
  if (!fixed_order) {
    // Match compatible boundary endpoints before ordering passes. This
    // interleaves faces into continuous lanes rather than finishing an entire
    // face first.
    const double seam_reach =
        std::max(2 * settings.chord_tolerance, .75 * settings.stepover);
    Bins seams;
    for (std::size_t i = 0; i < runs.size(); ++i)
      for (int end = 0; end < 2; ++end)
        if (end ? runs[i].last_boundary : runs[i].first_boundary)
          seams[cell(endpoint(runs[i], end), seam_reach)].push_back(2 * i + end);
    struct Match {
      double distance;
      std::size_t a, b;
    };
    std::vector<Match> matches;
    for (const auto& [key, ports] : seams) {
      (void)key;
      for (auto a : ports) {
        const auto& ra = runs[a / 2];
        const auto p = endpoint(ra, a % 2);
        nearby(seams, p, seam_reach, [&](std::size_t b) {
          if (b <= a || b / 2 == a / 2) return;
          if (!settings.bidirectional && a % 2 == b % 2) return;
          const auto& rb = runs[b / 2];
          if (!model.adjacent(ra.first_face, rb.first_face)) return;
          const double d = wave::distance(p, endpoint(rb, b % 2));
          if (d > seam_reach) return;
          // Outward endpoint tangents must face one another. Avoid joining two
          // parallel lanes sideways merely because their faces share some edge.
          const auto inner = [](const SurfaceRun& r, bool end) {
            return end ? r.points[r.points.size() - 2] : r.points[1];
          };
          const auto ia = inner(ra, a % 2), ib = inner(rb, b % 2);
          const auto q = endpoint(rb, b % 2);
          const double dot = (p.x - ia.x) * (q.x - ib.x) +
                             (p.y - ia.y) * (q.y - ib.y) +
                             (p.z - ia.z) * (q.z - ib.z);
          if (dot > -.5 * wave::distance(p, ia) * wave::distance(q, ib)) return;
          matches.push_back({d, a, b});
        });
      }
    }
    std::sort(matches.begin(), matches.end(), [](const Match& a, const Match& b) {
      return std::tie(a.distance, a.a, a.b) < std::tie(b.distance, b.a, b.b);
    });
    const std::size_t none = std::size_t(-1);
    std::vector<std::size_t> partner(runs.size() * 2, none);
    std::vector<Path> bridges(runs.size() * 2);
    std::vector<std::size_t> parent(runs.size());
    std::iota(parent.begin(), parent.end(), 0);
    auto root = [&](std::size_t n) {
      while (parent[n] != n) {
        parent[n] = parent[parent[n]];
        n = parent[n];
      }
      return n;
    };
    for (const auto& match : matches) {
      report();
      if (partner[match.a] != none || partner[match.b] != none ||
          root(match.a / 2) == root(match.b / 2))
        continue;
      auto bridge = connect(surface_end(runs[match.a / 2], match.a % 2),
                            surface_end(runs[match.b / 2], match.b % 2));
      if (!bridge) continue;
      bridges[match.a] = *bridge;
      std::reverse(bridge->begin(), bridge->end());
      bridges[match.b] = std::move(*bridge);
      partner[match.a] = match.b;
      partner[match.b] = match.a;
      parent[root(match.a / 2)] = root(match.b / 2);
      ++result.metrics.surface_seam_joins;
    }
    std::vector<bool> used(runs.size(), false);
    for (std::size_t i = 0; i < runs.size(); ++i) {
      if (used[i]) continue;
      // Find the free start of this connected component, retaining the chosen
      // cutting direction when bidirectional machining is disabled.
      std::size_t start = 2 * i;
      while (partner[start] != none) start = partner[start] ^ 1;
      SurfaceRun chain;
      auto port = start;
      while (true) {
        auto& run = runs[port / 2];
        used[port / 2] = true;
        if (port % 2) reverse_run(run);
        if (chain.points.empty()) {
          chain.first_face = run.first_face;
          chain.first_contact = run.first_contact;
          chain.first_uv = run.first_uv;
          chain.first_boundary = run.first_boundary;
        }
        if (chain.points.empty())
          chain.points = std::move(run.points);
        else {
          chain.points.insert(chain.points.end(), run.points.begin(),
                              run.points.end());
          std::vector<wave::Vec3d>().swap(run.points);
        }
        for (auto& span : run.spans) chain.spans.push_back(std::move(span));
        chain.last_face = run.last_face;
        chain.last_contact = run.last_contact;
        chain.last_uv = run.last_uv;
        chain.last_boundary = run.last_boundary;
        const auto next = partner[port ^ 1];
        if (next == none) break;
        const auto& bridge = bridges[port ^ 1];
        chain.points.insert(chain.points.end(), bridge.begin() + 1, bridge.end());
        port = next;
      }
      chains.push_back(std::move(chain));
    }
  } else {
    chains = std::move(runs);
  }
  if (joined) {
    *joined = std::move(chains);
    result.metrics.surface_linking_milliseconds += elapsed_milliseconds(started);
    return;
  }
  // Short stay-down moves are feeds: remaining stock can still be present.
  // Connections follow native surfaces. Disconnected selections retain the
  // clearance-plane route.
  const double link_reach =
      std::max(2 * settings.stepover, 2 * settings.chord_tolerance);
  // Ball compensation can magnify adjacent surface spacing on a convex face.
  // Search both tool-tip and contact space, retaining existing short links.
  Bins ends, contacts;
  for (std::size_t i = 0; i < chains.size(); ++i) {
    ends[cell(chains[i].points.front(), link_reach)].push_back(2 * i);
    contacts[cell(contact(chains[i], false), link_reach)].push_back(2 * i);
    if (settings.bidirectional) {
      ends[cell(chains[i].points.back(), link_reach)].push_back(2 * i + 1);
      contacts[cell(contact(chains[i], true), link_reach)].push_back(2 * i + 1);
    }
  }
  std::vector<bool> used(chains.size(), false);
  std::optional<wave::Vec3d> previous;
  std::vector<wave::Vec3d> feed_link;
  bool stay_down = false;
  auto orient = [&](std::size_t index, int end) {
    if (end) reverse_run(chains[index]);
  };
  std::size_t current = 0, completed = 0;
  while (completed < chains.size()) {
    report();
    if (!stay_down) {
      current = 0;
      while (used[current]) ++current;
    }
    ++completed;
    auto& chain = chains[current];
    used[current] = true;
    LevelPlan level;
    level.explicit_motion = true;
    level.feeds.authoritative_motion = true;
    level.floor_z = chain.points.front().z;
    level.axial_depth = std::max(operation.stepdown, .001);
    level.radial_width = settings.stepover;
    double path_distance = 0;
    auto append = [&](wave::Vec3d a, wave::Vec3d b, bool rapid,
                      wave::MoveKind kind) {
      if (wave::distance(a, b) < 1e-8) return;
      MotionPhase phase;
      phase.from = {wave::to_integer(a.x), wave::to_integer(a.y)};
      phase.to = {wave::to_integer(b.x), wave::to_integer(b.y)};
      phase.from_z = a.z;
      phase.to_z = b.z;
      phase.rapid = rapid;
      phase.kind = kind;
      phase.length = wave::distance(a, b);
      phase.rate =
          rapid ? (a.z == b.z ? rapid_feed(machine) : rapid_feed_z(machine))
                : (kind == wave::MoveKind::Entry ? feeds.plunge_feed
                                                 : feeds.full_feed());
      phase.path_distance_begin = path_distance;
      const double xy = std::hypot(b.x - a.x, b.y - a.y);
      path_distance += xy;
      phase.path_distance_end = path_distance;
      level.feeds.motion.push_back(phase);
      if (xy > 0) {
        wave::ToolpathMove move;
        move.kind = kind;
        move.path = {phase.from, phase.to};
        move.length = xy;
        level.toolpath.moves.push_back(std::move(move));
      }
    };
    const auto first = chain.points.front(), last = chain.points.back();
    const wave::Vec3d above{first.x, first.y, rapid_plane};
    if (stay_down) {
      for (std::size_t i = 1; i < feed_link.size(); ++i)
        append(feed_link[i - 1], feed_link[i], false, wave::MoveKind::Step);
      ++result.metrics.surface_stay_down_links;
    } else {
      if (previous) append(*previous, above, true, wave::MoveKind::Retract);
      append(above, first, false, wave::MoveKind::Entry);
    }
    for (std::size_t j = 1; j < chain.points.size(); ++j)
      append(chain.points[j - 1], chain.points[j], false, wave::MoveKind::Cut);
    std::vector<std::pair<double, std::size_t>> options;
    nearby(ends, last, link_reach, [&](std::size_t port) {
      if (used[port / 2]) return;
      const double d =
          wave::distance(last, endpoint(chains[port / 2], port % 2));
      if (d <= link_reach) options.push_back({d, port});
    });
    const auto last_contact = chain.last_contact;
    nearby(contacts, last_contact, link_reach, [&](std::size_t port) {
      if (used[port / 2]) return;
      const auto& next = chains[port / 2];
      const auto next_face = port % 2 ? next.last_face : next.first_face;
      if (chain.last_face != next_face &&
          !model.adjacent(chain.last_face, next_face))
        return;
      if (wave::distance(last_contact, contact(next, port % 2)) <= link_reach)
        options.push_back(
            {wave::distance(last, endpoint(next, port % 2)), port});
    });
    std::sort(options.begin(), options.end());
    options.erase(std::unique(options.begin(), options.end()), options.end());
    stay_down = false;
    for (auto [d, port] : options) {
      (void)d;
      // A manual sequence is authoritative. A short link may orient the next
      // contour, but must never skip ahead to a closer, later contour.
      if (fixed_order && port / 2 != completed) continue;
      auto path = connect(surface_end(chain, true),
                          surface_end(chains[port / 2], port % 2));
      if (!path) continue;
      feed_link = std::move(*path);
      current = port / 2;
      orient(current, port % 2);
      stay_down = true;
      break;
    }
    previous = last;
    if (!stay_down) {
      const auto departure = last;
      const wave::Vec3d end{departure.x, departure.y, rapid_plane};
      append(departure, end, true, wave::MoveKind::Retract);
      previous = end;
      ++result.metrics.surface_retracts;
    }
    result.levels.push_back(std::move(level));
    std::vector<wave::Vec3d>().swap(chain.points);
  }
  result.metrics.surface_linking_milliseconds += elapsed_milliseconds(started);
}
}  // namespace cam::detail
