#include "geometry.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <numbers>
#include <regex>
#include <stdexcept>
#include <string_view>
#include <tuple>

namespace wave {
namespace {

using namespace Clipper2Lib;

struct PointDLocal {
  double x = 0.0;
  double y = 0.0;
};

struct Affine {
  // SVG matrix: [a c e; b d f; 0 0 1]
  double a = 1.0;
  double b = 0.0;
  double c = 0.0;
  double d = 1.0;
  double e = 0.0;
  double f = 0.0;

  PointDLocal apply(PointDLocal p) const {
    return {a * p.x + c * p.y + e, b * p.x + d * p.y + f};
  }
};

Affine compose(const Affine& outer, const Affine& inner) {
  return {
      outer.a * inner.a + outer.c * inner.b,
      outer.b * inner.a + outer.d * inner.b,
      outer.a * inner.c + outer.c * inner.d,
      outer.b * inner.c + outer.d * inner.d,
      outer.a * inner.e + outer.c * inner.f + outer.e,
      outer.b * inner.e + outer.d * inner.f + outer.f,
  };
}

std::vector<double> parse_numbers(std::string_view text) {
  std::vector<double> values;
  const char* cursor = text.data();
  const char* end = cursor + text.size();
  while (cursor < end) {
    while (cursor < end &&
           (std::isspace(static_cast<unsigned char>(*cursor)) || *cursor == ',')) {
      ++cursor;
    }
    if (cursor == end) break;
    char* parsed_end = nullptr;
    const double value = std::strtod(cursor, &parsed_end);
    if (parsed_end == cursor) {
      ++cursor;
      continue;
    }
    values.push_back(value);
    cursor = parsed_end;
  }
  return values;
}

Affine parse_transform(const std::string& text) {
  Affine result;
  static const std::regex operation(
      R"(([A-Za-z]+)\s*\(([^)]*)\))",
      std::regex::ECMAScript);

  for (auto it = std::sregex_iterator(text.begin(), text.end(), operation);
       it != std::sregex_iterator(); ++it) {
    const std::string name = (*it)[1].str();
    const auto args = parse_numbers((*it)[2].str());
    Affine next;
    if (name == "matrix" && args.size() == 6) {
      next = {args[0], args[1], args[2], args[3], args[4], args[5]};
    } else if (name == "translate" && !args.empty()) {
      next.e = args[0];
      next.f = args.size() > 1 ? args[1] : 0.0;
    } else if (name == "scale" && !args.empty()) {
      next.a = args[0];
      next.d = args.size() > 1 ? args[1] : args[0];
    } else if (name == "rotate" && !args.empty()) {
      const double radians = args[0] * std::numbers::pi / 180.0;
      const double cosine = std::cos(radians);
      const double sine = std::sin(radians);
      next = {cosine, sine, -sine, cosine, 0.0, 0.0};
      if (args.size() >= 3) {
        Affine to_origin;
        to_origin.e = -args[1];
        to_origin.f = -args[2];
        Affine back;
        back.e = args[1];
        back.f = args[2];
        next = compose(back, compose(next, to_origin));
      }
    } else {
      throw std::runtime_error("Unsupported or malformed SVG transform: " + name);
    }
    // Post-multiplication matches SVG's transform-list matrix convention.
    result = compose(result, next);
  }
  return result;
}

std::string attribute(const std::string& tag, const std::string& name) {
  const std::regex expression("(?:^|\\s)" + name +
                              R"(\s*=\s*(["'])(.*?)\1)",
                              std::regex::icase);
  std::smatch match;
  return std::regex_search(tag, match, expression) ? match[2].str() : std::string{};
}

void skip_path_separators(const char*& cursor) {
  while (*cursor &&
         (std::isspace(static_cast<unsigned char>(*cursor)) || *cursor == ',')) {
    ++cursor;
  }
}

bool read_number(const char*& cursor, double& value) {
  skip_path_separators(cursor);
  if (!*cursor || std::isalpha(static_cast<unsigned char>(*cursor))) return false;
  char* end = nullptr;
  value = std::strtod(cursor, &end);
  if (end == cursor) return false;
  cursor = end;
  return true;
}

std::vector<std::vector<PointDLocal>> parse_path_data(const std::string& data) {
  std::vector<std::vector<PointDLocal>> closed_paths;
  std::vector<PointDLocal> current_path;
  PointDLocal current;
  PointDLocal start;
  char command = 0;
  const char* cursor = data.c_str();

  auto close_path = [&] {
    if (current_path.size() > 1 &&
        current_path.front().x == current_path.back().x &&
        current_path.front().y == current_path.back().y) {
      current_path.pop_back();
    }
    if (current_path.size() >= 3) closed_paths.push_back(current_path);
    current_path.clear();
  };

  while (true) {
    skip_path_separators(cursor);
    if (!*cursor) break;
    if (std::isalpha(static_cast<unsigned char>(*cursor))) command = *cursor++;
    if (!command) throw std::runtime_error("SVG path data begins without a command");

    const bool relative = std::islower(static_cast<unsigned char>(command));
    const char upper = static_cast<char>(std::toupper(static_cast<unsigned char>(command)));
    if (upper == 'Z') {
      close_path();
      current = start;
      command = 0;
      continue;
    }

    if (upper != 'M' && upper != 'L' && upper != 'H' && upper != 'V') {
      throw std::runtime_error(std::string("Unsupported SVG path command '") + command +
                               "' (this demo accepts closed M/L/H/V/Z paths)");
    }

    bool consumed = false;
    while (true) {
      const char* before = cursor;
      double x = 0.0;
      double y = 0.0;
      if (upper == 'H') {
        if (!read_number(cursor, x)) break;
        if (relative) x += current.x;
        current.x = x;
      } else if (upper == 'V') {
        if (!read_number(cursor, y)) break;
        if (relative) y += current.y;
        current.y = y;
      } else {
        if (!read_number(cursor, x) || !read_number(cursor, y)) {
          cursor = before;
          break;
        }
        if (relative) {
          x += current.x;
          y += current.y;
        }
        current = {x, y};
      }

      if (upper == 'M' && !consumed) {
        if (!current_path.empty()) current_path.clear();  // unclosed subpath is ignored
        start = current;
      }
      current_path.push_back(current);
      consumed = true;
      skip_path_separators(cursor);
      if (!*cursor || std::isalpha(static_cast<unsigned char>(*cursor))) break;
    }
    if (!consumed) {
      throw std::runtime_error(std::string("Missing coordinate for SVG command '") + command + "'");
    }
    if (upper == 'M') command = relative ? 'l' : 'L';
  }
  // Deliberately ignore a final unclosed subpath: the subject is closed curves only.
  return closed_paths;
}

Path64 canonical_path(const Path64& input) {
  if (input.empty()) return {};
  Path64 path = input;
  if (path.size() > 1 && path.front() == path.back()) path.pop_back();
  if (path.empty()) return {};

  auto rotate_minimum = [](const Path64& source) {
    const auto minimum = std::min_element(source.begin(), source.end(),
        [](const Point64& a, const Point64& b) {
          return std::tie(a.x, a.y) < std::tie(b.x, b.y);
        });
    Path64 result;
    result.reserve(source.size());
    result.insert(result.end(), minimum, source.end());
    result.insert(result.end(), source.begin(), minimum);
    return result;
  };

  Path64 forward = rotate_minimum(path);
  std::reverse(path.begin(), path.end());
  Path64 reverse = rotate_minimum(path);
  return std::lexicographical_compare(
             reverse.begin(), reverse.end(), forward.begin(), forward.end(),
             [](const Point64& a, const Point64& b) {
               return std::tie(a.x, a.y) < std::tie(b.x, b.y);
             })
             ? reverse
             : forward;
}

auto canonical_paths(const Paths64& paths) {
  std::vector<Path64> result;
  result.reserve(paths.size());
  for (const auto& path : paths) {
    if (!path.empty()) result.push_back(canonical_path(path));
  }
  const auto less = [](const Path64& a, const Path64& b) {
    return std::lexicographical_compare(
        a.begin(), a.end(), b.begin(), b.end(),
        [](const Point64& lhs, const Point64& rhs) {
          return std::tie(lhs.x, lhs.y) < std::tie(rhs.x, rhs.y);
        });
  };
  std::sort(result.begin(), result.end(), less);
  return result;
}

Paths64 inflate_with_softening(const Paths64& paths, double distance,
                               double softening) {
  if (softening <= 0.0)
    return InflatePaths(paths, distance, JoinType::Miter, EndType::Polygon);
  if (softening >= distance)
    return InflatePaths(paths, distance, JoinType::Round, EndType::Polygon);
  Paths64 expanded = InflatePaths(paths, distance - softening,
                                  JoinType::Miter, EndType::Polygon);
  return InflatePaths(expanded, softening, JoinType::Round, EndType::Polygon);
}

// Shortest distance from a point to a closed ring, in integer units.
double distance_to_ring(const Point64& point, const Path64& ring) {
  if (ring.empty()) return std::numeric_limits<double>::max();
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < ring.size(); ++i) {
    const Point64& a = ring[i];
    const Point64& b = ring[(i + 1) % ring.size()];
    const double dx = static_cast<double>(b.x - a.x);
    const double dy = static_cast<double>(b.y - a.y);
    const double length_squared = dx * dx + dy * dy;
    double t = 0.0;
    if (length_squared > 0.0)
      t = std::clamp((static_cast<double>(point.x - a.x) * dx +
                      static_cast<double>(point.y - a.y) * dy) /
                         length_squared,
                     0.0, 1.0);
    best = std::min(best, std::hypot(static_cast<double>(point.x - a.x) - dx * t,
                                     static_cast<double>(point.y - a.y) - dy * t));
  }
  return best;
}

// The seed's connected pieces are the entry fronts, one apiece: they are the
// stretches of domain wall the protected subject leaves separate. Declining an
// entry therefore removes a whole piece, rather than cutting a notch out of one
// -- a notch would be closed again by the first inflation, since the wave steps
// a full stepover while the seed ring is one unit thick.
Paths64 drop_muted_seeds(const Paths64& seed, const EntryMute& muted,
                         WaveResult& result) {
  Clipper64 clipper;
  clipper.AddSubject(seed);
  PolyTree64 tree;
  Paths64 open_solution;
  if (!clipper.Execute(ClipType::Union, FillRule::NonZero, tree, open_solution))
    return seed;
  const double tolerance = std::max(muted.tolerance, 1.0);
  Paths64 kept;
  for (std::size_t i = 0; i < tree.Count(); ++i) {
    const PolyPath64* piece = tree.Child(i);
    bool drop = false;
    for (const Point64& anchor : muted.anchors)
      if (distance_to_ring(anchor, piece->Polygon()) <= tolerance) drop = true;
    if (drop) {
      ++result.muted_entries;
      continue;
    }
    kept.push_back(piece->Polygon());
    for (std::size_t j = 0; j < piece->Count(); ++j)
      kept.push_back(piece->Child(j)->Polygon());
  }
  return Union(kept, FillRule::NonZero);
}

}  // namespace

double to_world(std::int64_t value) { return static_cast<double>(value) / kIntegerScale; }

std::int64_t to_integer(double value) {
  return static_cast<std::int64_t>(std::llround(value * kIntegerScale));
}

SvgGeometry load_svg_closed_paths(const std::filesystem::path& filename) {
  std::ifstream stream(filename);
  if (!stream) throw std::runtime_error("Unable to open SVG: " + filename.string());
  const std::string xml((std::istreambuf_iterator<char>(stream)),
                        std::istreambuf_iterator<char>());

  static const std::regex tag_expression(R"(<[^>]+>)");
  std::vector<Affine> transforms{Affine{}};
  SvgGeometry result;

  for (auto it = std::sregex_iterator(xml.begin(), xml.end(), tag_expression);
       it != std::sregex_iterator(); ++it) {
    const std::string tag = it->str();
    if (tag.starts_with("</")) {
      if ((tag.starts_with("</g") || tag.starts_with("</svg")) &&
          transforms.size() > 1) {
        transforms.pop_back();
      }
      continue;
    }

    const bool group = tag.starts_with("<g") || tag.starts_with("<svg");
    const std::string transform_text = attribute(tag, "transform");
    const Affine local = transform_text.empty() ? Affine{} : parse_transform(transform_text);
    if (group) {
      transforms.push_back(compose(transforms.back(), local));
      continue;
    }
    if (!tag.starts_with("<path")) continue;

    const std::string data = attribute(tag, "d");
    if (data.empty()) continue;
    const Affine transform = compose(transforms.back(), local);
    for (const auto& float_path : parse_path_data(data)) {
      Path64 path;
      path.reserve(float_path.size());
      for (const auto point : float_path) {
        const auto transformed = transform.apply(point);
        const Point64 integer_point(to_integer(transformed.x), to_integer(transformed.y));
        if (path.empty() || path.back() != integer_point) path.push_back(integer_point);
      }
      path = TrimCollinear(path, false);
      if (path.size() >= 3 && std::abs(Area(path)) > 0.0) {
        result.paths.push_back(std::move(path));
        ++result.source_path_count;
      }
    }
  }

  if (result.paths.empty()) {
    throw std::runtime_error("The SVG contains no supported closed paths");
  }
  result.bounds = GetBounds(result.paths);
  return result;
}

bool paths_equal_canonical(const Paths64& lhs, const Paths64& rhs) {
  return canonical_paths(lhs) == canonical_paths(rhs);
}

WaveResult build_wavefront(const Paths64& raw_subject,
                           double allowance,
                           double stepover,
                           double softening,
                           double lead_in,
                           std::size_t max_iterations,
                           const EntryMute& muted) {
  if (raw_subject.empty()) throw std::invalid_argument("Subject is empty");
  if (!(allowance > 0.0)) throw std::invalid_argument("Allowance must be positive");
  const Paths64 subject = Union(raw_subject, FillRule::NonZero);
  const Rect64 bounds = GetBounds(subject);
  const std::int64_t margin = std::max<std::int64_t>(1, to_integer(allowance));
  const Path64 stock_boundary = {
      Point64(bounds.left - margin, bounds.top - margin),
      Point64(bounds.right + margin, bounds.top - margin),
      Point64(bounds.right + margin, bounds.bottom + margin),
      Point64(bounds.left - margin, bounds.bottom + margin),
  };
  return build_wavefront(subject, Paths64{stock_boundary}, stepover,
                         softening, lead_in, max_iterations, muted);
}

WaveResult seed_wavefront(const Paths64& raw_subject,
                          const Paths64& raw_stock,
                          double stepover,
                          double softening,
                          double lead_in,
                          const EntryMute& muted) {
  if (raw_subject.empty()) throw std::invalid_argument("Subject is empty");
  if (raw_stock.empty()) throw std::invalid_argument("Stock is empty");
  if (!(stepover > 0.0)) throw std::invalid_argument("Stepover must be positive");
  if (softening < 0.0 || softening > stepover)
    throw std::invalid_argument("Softening must be between zero and stepover");
  if (lead_in < 0.0) throw std::invalid_argument("Lead-in must not be negative");

  WaveResult result;
  result.subject = Union(raw_subject, FillRule::NonZero);
  result.stock = Union(raw_stock, FillRule::NonZero);
  result.stepover = stepover;
  result.softening = softening;
  if (result.stock.empty()) throw std::invalid_argument("Stock union is empty");
  result.stock_boundary = result.stock.front();

  // The stock wall the wave starts from: a one-integer-unit ring at the stock
  // boundary. The subject is deliberately not part of it, so its boundaries can
  // never become independent wave sources.
  const Paths64 stock_inner = InflatePaths(result.stock, -1.0, JoinType::Miter,
                                           EndType::Polygon);
  Paths64 wall = Difference(result.stock, stock_inner, FillRule::NonZero);
  wall = Difference(wall, result.subject, FillRule::NonZero);
  wall = Union(wall, FillRule::NonZero);

  // The lead-in is used exactly as asked for, not rounded up to a whole number
  // of stepovers. Rounding put a front precisely on the stock wall, which left
  // the front before it -- the first one that touches the material at all --
  // taking whatever fraction of a stepover happened to be left over. The caller
  // sizes the run-in to place that first bite where it wants it, so the wave
  // has to honour the distance it was given.
  result.lead_in = lead_in > 0.0 ? lead_in : 0.0;
  // How many fronts stand outside the stock wall: the ones still a positive
  // distance in front of it.
  result.lead_in_generations =
      lead_in > 0.0 ? static_cast<std::size_t>(std::ceil(lead_in / stepover)) : 0;

  // Propagation runs through the stock and, where there is a lead-in, through
  // the air in front of it as well. That approach region is grown from the wall
  // rather than from the stock solid: only boundary the wave actually starts
  // from gets a run-in, so stock the subject occludes contributes none. Growing
  // the solid instead would have manufactured a lead-in running along the part.
  Paths64 domain = result.stock;
  if (result.lead_in_generations > 0 && !wall.empty()) {
    Paths64 approach = InflatePaths(
        wall, static_cast<double>(std::max<std::int64_t>(1, to_integer(result.lead_in))),
        JoinType::Round, EndType::Polygon);
    approach = Difference(approach, result.subject, FillRule::NonZero);
    domain = Union(domain, approach, FillRule::NonZero);
  }
  result.domain = domain;

  // Seed on the boundary of whatever the domain turned out to be, which is the
  // outer edge of the approach where there is one and the stock wall elsewhere.
  //
  // Where the domain runs along the subject the boundary has to be dropped, or
  // the part becomes a wave source in its own right and the material is cleared
  // outward from it as well as inward from the wall. Inside the stock the
  // subject covers that boundary and removing it is enough, but an approach
  // region is carved out of open air and leaves the face of the part exposed
  // alongside the outer edge that is meant to be the only source. So the seed
  // is cut back by a hair more than the ring is thick.
  const Paths64 domain_inner = InflatePaths(domain, -1.0, JoinType::Miter,
                                            EndType::Polygon);
  const Paths64 subject_face = InflatePaths(result.subject, 2.0, JoinType::Miter,
                                            EndType::Polygon);
  Paths64 reached = Difference(domain, domain_inner, FillRule::NonZero);
  reached = Difference(reached, subject_face, FillRule::NonZero);
  reached = Union(reached, FillRule::NonZero);
  if (!muted.empty()) reached = drop_muted_seeds(reached, muted, result);
  if (reached.empty()) {
    result.stop_reason = StopReason::Empty;
    return result;
  }
  result.generations.push_back(reached);
  result.reached_generations.push_back(reached);
  result.stop_reason = StopReason::Seeded;
  return result;
}

WaveResult build_wavefront(const Paths64& raw_subject,
                           const Paths64& raw_stock,
                           double stepover,
                           double softening,
                           double lead_in,
                           std::size_t max_iterations,
                           const EntryMute& muted) {
  if (max_iterations == 0) throw std::invalid_argument("Iteration limit must be positive");
  WaveResult result = seed_wavefront(raw_subject, raw_stock, stepover, softening,
                                     lead_in, muted);
  if (result.generations.empty()) return result;
  Paths64 reached = result.reached_generations.back();
  const double delta = static_cast<double>(
      std::max<std::int64_t>(1, to_integer(stepover)));
  const double softening_delta = static_cast<double>(
      std::clamp<std::int64_t>(to_integer(softening), 0,
                               static_cast<std::int64_t>(delta)));

  for (std::size_t iteration = 0; iteration < max_iterations; ++iteration) {
    // Expand only geometry already reached from the box. Clipping to the domain
    // keeps the expansion inward; subtracting the subject blocks colliding
    // wavefronts.
    Paths64 next = inflate_with_softening(reached, delta, softening_delta);
    next = Intersect(next, result.domain, FillRule::NonZero);
    next = Difference(next, result.subject, FillRule::NonZero);

    if (next.empty()) {
      result.stop_reason = StopReason::Empty;
      return result;
    }
    if (paths_equal_canonical(next, reached)) {
      result.stop_reason = StopReason::Unchanged;
      return result;
    }

    // Retain just the newly infiltrated area as this generation's visible wave.
    Paths64 wave = Difference(next, reached, FillRule::NonZero);
    if (wave.empty()) {
      result.stop_reason = StopReason::Unchanged;
      return result;
    }
    result.generations.push_back(std::move(wave));
    reached = std::move(next);
    result.reached_generations.push_back(reached);
  }

  result.stop_reason = StopReason::IterationLimit;
  return result;
}

const char* stop_reason_name(StopReason reason) {
  switch (reason) {
    case StopReason::Empty: return "empty output";
    case StopReason::Unchanged: return "unchanged output";
    case StopReason::IterationLimit: return "iteration safety limit";
    case StopReason::Seeded: return "seeded but not propagated";
  }
  return "unknown";
}

}  // namespace wave
