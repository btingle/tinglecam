#include "cam/surface_linking.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <set>
#include <stdexcept>

#include "cam/job_internal.h"

namespace cam::detail {
std::map<std::size_t, std::vector<double>> surface_lanes(
    const CadModel& model, const Operation& operation, PlanningMetrics& metrics,
    const ProgressCallback& progress) {
  const auto started = PlanningClock::now();
  const auto& settings = operation.surface;
  std::map<std::size_t, std::vector<double>> grids;
  auto report = [&](const char* stage) {
    if (progress && !progress(0.0, stage)) throw std::runtime_error("Cancelled");
  };
  for (auto face : std::set<std::size_t>(settings.faces.begin(), settings.faces.end())) {
    if (face >= model.faces().size())
      throw std::runtime_error("Selected CAD face no longer exists");
    report("Spacing CAD finishing contours");
    // Subdivide in physical space, reusing sections in both child intervals.
    // This retains every face's original physical stepover bound.
    using Section = std::array<wave::Vec3d, 33>;
    auto section = [&](double across) {
      Section points;
      for (std::size_t i = 0; i < points.size(); ++i) {
        const double along = double(i) / 32;
        ++metrics.surface_samples;
        points[i] = model.evaluate(face, settings.along_v ? across : along,
                                   settings.along_v ? along : across).point;
      }
      return points;
    };
    auto& lanes = grids[face];
    lanes.push_back(0);
    std::size_t intervals = 0;
    std::function<void(double, double, const Section&, const Section&, int)> split =
        [&](double a, double b, const Section& p, const Section& q, int depth) {
          if (++intervals % 128 == 0) report("Spacing CAD finishing contours");
          const auto middle = section((a + b) / 2);
          double width = 0;
          for (std::size_t i = 0; i < p.size(); ++i)
            width = std::max(width, wave::distance(p[i], middle[i]) +
                                       wave::distance(middle[i], q[i]));
          if (width > settings.stepover * (1 + 1e-6)) {
            if (depth >= 20 || lanes.size() > 20000)
              throw std::runtime_error("Surface stepover requires too many passes");
            split(a, (a + b) / 2, p, middle, depth + 1);
            split((a + b) / 2, b, middle, q, depth + 1);
          } else lanes.push_back(b);
        };
    const auto first = section(0), last = section(1);
    split(0, 1, first, last, 0);
  }
  std::vector<std::pair<std::size_t, std::size_t>> seams;
  for (auto a = grids.begin(); a != grids.end(); ++a)
    for (auto b = std::next(a); b != grids.end(); ++b)
      if (model.adjacent(a->first, b->first)) seams.emplace_back(a->first, b->first);
  auto merge = [](std::vector<double>& lanes, std::vector<double> additions) {
    const auto original = lanes.size();
    // Preserve existing coordinates exactly; pcurve inversions can return a
    // few ulps either side of an existing lane, including at 0 and 1.
    std::sort(additions.begin(), additions.end());
    std::vector<double> fresh;
    for (double value : additions) {
      auto next = std::lower_bound(lanes.begin(), lanes.end(), value);
      if ((next != lanes.end() && std::abs(*next - value) < 1e-10) ||
          (next != lanes.begin() && std::abs(*std::prev(next) - value) < 1e-10) ||
          (!fresh.empty() && value - fresh.back() < 1e-10)) continue;
      fresh.push_back(value);
    }
    lanes.insert(lanes.end(), fresh.begin(), fresh.end());
    std::inplace_merge(lanes.begin(), lanes.begin() + original, lanes.end());
    if (lanes.size() > 20000)
      throw std::runtime_error("Synchronized surface contours require too many passes");
    return !fresh.empty();
  };
  // Propagate additions across a whole chain of patches. Only isoparametric
  // contour-end seams qualify; edges parallel to cuts keep independent grids.
  auto synchronize = [&] {
    bool changed = false;
    std::size_t iteration = 0;
    do {
      changed = false;
      for (const auto& [a, b] : seams) {
        report("Matching CAD finishing contours across seams");
        const auto matches = model.matching_contour_parameters(
            a, b, settings.along_v, grids.at(a), grids.at(b));
        std::vector<double> left, right;
        for (const auto& match : matches) {
          left.push_back(match[0]);
          right.push_back(match[1]);
        }
        const bool added_left = merge(grids.at(a), std::move(left));
        const bool added_right = merge(grids.at(b), std::move(right));
        changed |= added_left || added_right;
      }
      if (changed && ++iteration > 2 * grids.size() + 1)
        throw std::runtime_error("Finishing contour grids could not be synchronized across face seams");
    } while (changed);
  };
  synchronize();

  // The dyadic grids above are a measurement mesh, not the cutting lanes.
  // Build a common physical distance coordinate for each chain of complete
  // contour-end seams. Its density follows the widest part of every patch,
  // including patches with reversed or nonlinear native parameterizations.
  std::map<std::size_t, std::vector<std::size_t>> connected;
  for (const auto& [a, b] : seams) {
    const auto ends = model.matching_contour_parameters(a, b, settings.along_v,
                                                       {0, 1}, {});
    if (ends.size() == 2 &&
        std::abs(ends[0][0]) < 1e-9 && std::abs(ends[1][0] - 1) < 1e-9 &&
        ((std::abs(ends[0][1]) < 1e-9 && std::abs(ends[1][1] - 1) < 1e-9) ||
         (std::abs(ends[0][1] - 1) < 1e-9 && std::abs(ends[1][1]) < 1e-9))) {
      connected[a].push_back(b);
      connected[b].push_back(a);
    }
  }
  std::set<std::size_t> visited;
  for (auto& [root, root_grid] : grids) {
    if (!visited.insert(root).second) continue;
    struct Patch { std::size_t face, parent; };
    std::vector<Patch> patches{{root, 0}};
    for (std::size_t i = 0; i < patches.size(); ++i)
      for (auto face : connected[patches[i].face])
        if (visited.insert(face).second) patches.push_back({face, i});

    using Section = std::array<wave::Vec3d, 33>;
    struct Sample {
      std::vector<double> parameters;
      std::vector<Section> sections;
    };
    std::map<double, Sample> samples;
    auto sample = [&](double across) -> const Sample& {
      auto [it, fresh] = samples.try_emplace(across);
      if (fresh) {
        if (samples.size() % 32 == 1) report("Spacing CAD finishing contours");
        auto& value = it->second;
        for (std::size_t i = 0; i < patches.size(); ++i) {
          double parameter = across;
          if (i != 0) {
            const auto parent = patches[i].parent;
            const auto mapped = model.matching_contour_parameters(
                patches[parent].face, patches[i].face, settings.along_v,
                {value.parameters[parent]}, {});
            if (mapped.size() != 1)
              throw std::runtime_error("Ambiguous finishing contour seam mapping");
            parameter = mapped.front()[1];
          }
          value.parameters.push_back(parameter);
          auto& section = value.sections.emplace_back();
          for (std::size_t j = 0; j < section.size(); ++j) {
            const double along = double(j) / 32;
            ++metrics.surface_samples;
            section[j] = model.evaluate(patches[i].face,
                settings.along_v ? parameter : along,
                settings.along_v ? along : parameter).point;
          }
        }
      }
      return it->second;
    };
    auto width = [&](double a, double b) {
      const auto& p = sample(a).sections;
      const auto& m = sample((a + b) / 2).sections;
      const auto& q = sample(b).sections;
      double result = 0;
      for (std::size_t face = 0; face < patches.size(); ++face)
        for (std::size_t j = 0; j < p[face].size(); ++j)
          result = std::max(result, wave::distance(p[face][j], m[face][j]) +
                                    wave::distance(m[face][j], q[face][j]));
      return result;
    };
    auto measurement = root_grid;
    for (int i = 0; i <= 32; ++i) measurement.push_back(i / 32.0);
    std::sort(measurement.begin(), measurement.end());
    measurement.erase(std::unique(measurement.begin(), measurement.end()), measurement.end());
    std::vector<double> distance(measurement.size());
    for (std::size_t i = 1; i < measurement.size(); ++i)
      distance[i] = distance[i - 1] + width(measurement[i - 1], measurement[i]);
    const double length = distance.back();
    const double required = std::max(1.0, std::ceil(length / settings.stepover - 1e-9));
    if (!std::isfinite(required) || required > 20000)
      throw std::runtime_error("Surface stepover requires too many passes");
    std::size_t count = std::size_t(required);
    std::vector<double> lanes;
    for (int attempt = 0; ; ++attempt) {
      if (count > 20000 || attempt == 8)
        throw std::runtime_error("Could not satisfy physical surface stepover");
      lanes = {0};
      for (std::size_t i = 1; i < count; ++i) {
        const double target = length * double(i) / count;
        const auto upper = std::upper_bound(distance.begin(), distance.end(), target);
        const auto segment = std::size_t(upper - distance.begin()) - 1;
        const double a = measurement[segment], b = measurement[segment + 1];
        const double desired = target - distance[segment];
        if (desired < settings.stepover * 1e-7) { lanes.push_back(a); continue; }
        if (distance[segment + 1] - target < settings.stepover * 1e-7) {
          lanes.push_back(b);
          continue;
        }
        double low = a, high = b, low_length = 0,
               high_length = distance[segment + 1] - distance[segment], across = a;
        // Invert physical distance, rather than interpolating UV. This also
        // spaces contours evenly on highly stretched spline parameters.
        for (int iteration = 0; iteration < 32; ++iteration) {
          across = low + (high - low) *
              std::clamp((desired - low_length) / (high_length - low_length), .001, .999);
          const double current = width(a, across);
          if (std::abs(current - desired) < settings.stepover * 1e-7) break;
          if (current < desired) { low = across; low_length = current; }
          else { high = across; high_length = current; }
        }
        lanes.push_back(across);
      }
      lanes.push_back(1);
      double widest = 0;
      for (std::size_t i = 1; i < lanes.size(); ++i)
        widest = std::max(widest, width(lanes[i - 1], lanes[i]));
      if (widest <= settings.stepover * (1 + 1e-6)) break;
      count = std::max(count + 1, std::size_t(std::ceil(count * widest / settings.stepover)));
    }
    for (std::size_t i = 0; i < patches.size(); ++i) {
      auto& grid = grids.at(patches[i].face);
      grid.clear();
      for (double across : lanes) grid.push_back(sample(across).parameters[i]);
      std::sort(grid.begin(), grid.end());
      // Native pcurve inversion can leave endpoint-sized floating-point noise.
      grid.front() = 0;
      grid.back() = 1;
    }
  }
  // Partial shared edges keep their boundary anchors too. Full chains already
  // agree, so this adds lanes only where partial/cyclic seams require them.
  synchronize();
  metrics.surface_sampling_milliseconds += elapsed_milliseconds(started);
  return grids;
}
}  // namespace cam::detail
