#include "cam/surface_finishing.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <set>
#include <stdexcept>

#include "cam/job_internal.h"
#include "cam/surface_linking.h"

namespace cam {
wave::Vec3d ball_tip(const SurfacePoint& p, double radius, double allowance) {
  const double offset = radius + allowance;
  return {p.point.x + offset * p.normal.x, p.point.y + offset * p.normal.y,
          p.point.z + offset * p.normal.z - radius};
}
namespace {
std::vector<detail::SurfaceRun> sample_surface_runs(
    const CadModel& native_model, const Operation& operation, const Tool& tool,
    OperationPlan& result, std::size_t& trimmed, bool retain_contacts,
    const ProgressCallback& progress) {
  const auto* model = &native_model;
  const auto& settings = operation.surface;
  if (tool.type != ToolType::BallEndMill)
    throw std::runtime_error("Surface finishing requires a ball end mill");
  if (!validate(tool).empty() || tool.flute_length < tool.diameter / 2)
    throw std::runtime_error("Invalid finishing tool dimensions");
  if (settings.faces.empty())
    throw std::runtime_error("Select at least one CAD face to finish");
  if (settings.model_fingerprint != model->fingerprint())
    throw std::runtime_error(
        "The STEP model changed; select its finishing faces again");
  const double radius = tool.diameter / 2;
  // Negative allowance cuts below the surface; the ball centre must stay on
  // the outward side of it.
  if (!std::isfinite(settings.stepover) || settings.stepover < 0.001 ||
      !std::isfinite(settings.chord_tolerance) ||
      settings.chord_tolerance < 0.002 ||
      !std::isfinite(settings.allowance) || settings.allowance <= -radius ||
      !std::isfinite(operation.path_pitch) || operation.path_pitch <= 0)
    throw std::runtime_error(
        "Finishing requires positive stepover/path pitch, chord tolerance >= "
        "0.002 mm, and allowance above minus the tool radius");
  const double tolerance = settings.chord_tolerance;
  const double sampling_tolerance =
      tolerance - std::sqrt(0.5) / wave::kIntegerScale;
  const double maximum_length = std::min(operation.path_pitch, 1.0);
  std::size_t face_index = 0;
  bool reverse = false;
  std::vector<detail::SurfaceRun> runs;
  std::set<std::size_t> selection(settings.faces.begin(),
                                  settings.faces.end());
  const auto grids = detail::surface_lanes(*model, operation, result.metrics, progress);
  for (auto face : selection) {
    if (face >= model->faces().size())
      throw std::runtime_error("Selected CAD face no longer exists");
    auto evaluate = [&](double across, double along) {
      auto p = model->evaluate(face, settings.along_v ? across : along,
                               settings.along_v ? along : across);
      if (settings.flip_normals) {
        p.normal.x = -p.normal.x;
        p.normal.y = -p.normal.y;
        p.normal.z = -p.normal.z;
      }
      return p;
    };
    const auto& lanes = grids.at(face);
    struct Lane {
      std::vector<detail::SurfaceRun> runs;
      PlanningMetrics metrics;
      std::size_t trimmed = 0;
    };
    std::vector<Lane> lane_results(lanes.size());
    std::vector<std::size_t> tasks(lanes.size());
    for (std::size_t i = 0; i < tasks.size(); ++i) tasks[i] = i;
    const bool initial_reverse = reverse;
    const bool completed = detail::parallel_for_levels(
        tasks,
        [&](std::size_t lane_index, const detail::LevelProgress& report) {
          const ProgressCallback progress = [&](PlanningProgress p,
                                                const std::string&) {
            return report(p.fraction);
          };
          auto& lane = lane_results[lane_index];
          auto& trimmed = lane.trimmed;
          auto sample = [&](double across, double along) {
            const auto start = std::chrono::steady_clock::now();
            auto p = evaluate(across, along);
            lane.metrics.surface_sampling_milliseconds +=
                detail::elapsed_milliseconds(start);
            ++lane.metrics.surface_samples;
            return p;
          };
          const bool reverse = initial_reverse != (lane_index % 2 != 0);
          const double across = lanes[lane_index], fraction = 0;
          struct Node {
            double t;
            SurfacePoint surface;
            wave::Vec3d tip;
            bool previous_inside = true;
          };
          auto node = [&](double t) {
            auto p = sample(across, t);
            return Node{t, p, ball_tip(p, radius, settings.allowance)};
          };
          std::vector<Node> nodes{node(0)};
          std::function<void(Node, Node, int)> refine = [&](Node a, Node b,
                                                            int depth) {
            const Node m = node((a.t + b.t) / 2),
                       q1 = node((3 * a.t + b.t) / 4),
                       q3 = node((a.t + 3 * b.t) / 4);
            double error = 0;
            for (const auto& q : {q1, m, q3}) {
              const double f = (q.t - a.t) / (b.t - a.t);
              error = std::max(
                  {error,
                   wave::distance(q.tip, wave::interpolate(a.tip, b.tip, f)),
                   wave::distance(q.surface.point,
                                  wave::interpolate(a.surface.point,
                                                    b.surface.point, f))});
            }
            const bool trim_change = a.surface.inside != m.surface.inside ||
                                     m.surface.inside != b.surface.inside ||
                                     q1.surface.inside != a.surface.inside ||
                                     q3.surface.inside != b.surface.inside;
            if ((error > sampling_tolerance ||
                 wave::distance(a.tip, b.tip) > maximum_length ||
                 (trim_change && wave::distance(a.tip, b.tip) > tolerance)) &&
                depth < 22) {
              refine(a, m, depth + 1);
              refine(m, b, depth + 1);
            } else {
              if (error > sampling_tolerance * 1.01)
                throw std::runtime_error(
                    "Could not satisfy surface chord tolerance near a "
                    "singularity");
              auto end = b;
              end.previous_inside = m.surface.inside;
              nodes.push_back(end);
            }
            if (nodes.size() % 256 == 0 && progress &&
                !progress(fraction, "Sampling CAD finishing contours"))
              throw std::runtime_error("Cancelled");
            if (nodes.size() > 1000000)
              throw std::runtime_error(
                  "Surface path exceeds one million samples; increase "
                  "tolerance");
          };
          // A minimum UV scan detects trim islands and periodic curves whose
          // endpoints and midpoint happen to coincide.
          for (int i = 0; i < 32; ++i)
            refine(node(i / 32.0), node((i + 1) / 32.0), 0);
          if (settings.bidirectional && reverse)
            std::reverse(nodes.begin(), nodes.end());
          struct Tip {
            double t;
            wave::Vec3d tip, contact;
          };
          std::vector<Tip> run;
          auto emit_run = [&] {
            if (run.size() >= 2) {
              detail::SurfaceRun path;
              path.first_face = path.last_face = face;
              path.first_contact = run.front().contact;
              path.last_contact = run.back().contact;
              path.first_uv = settings.along_v
                                  ? std::array{across, run.front().t}
                                  : std::array{run.front().t, across};
              path.last_uv = settings.along_v
                                 ? std::array{across, run.back().t}
                                 : std::array{run.back().t, across};
              path.first_boundary =
                  run.front().t < 1e-9 || run.front().t > 1 - 1e-9;
              path.last_boundary =
                  run.back().t < 1e-9 || run.back().t > 1 - 1e-9;
              for (const auto& p : run) path.points.push_back(p.tip);
              if (retain_contacts) {
                SurfaceSpan span;
                span.face = face;
                span.lane = lane_index;
                span.across = across;
                span.along_min = std::min(run.front().t, run.back().t);
                span.along_max = std::max(run.front().t, run.back().t);
                for (const auto& p : run) span.contacts.push_back(p.contact);
                path.spans.push_back(std::move(span));
              }
              lane.runs.push_back(std::move(path));
            }
            run.clear();
          };
          std::vector<Tip> candidates;
          auto emit_candidates = [&] {
            run = std::move(candidates);
            emit_run();
            candidates.clear();
          };
          for (std::size_t i = 0; i < nodes.size(); ++i) {
            auto p = nodes[i];
            p.tip.x = wave::to_world(wave::to_integer(p.tip.x));
            p.tip.y = wave::to_world(wave::to_integer(p.tip.y));
            if (!p.surface.inside) {
              ++trimmed;
              emit_candidates();
              continue;
            }
            if (!candidates.empty()) {
              const bool inside = settings.bidirectional && reverse
                                      ? nodes[i - 1].previous_inside
                                      : p.previous_inside;
              if (!inside) emit_candidates();
            }
            candidates.push_back({p.t, p.tip, p.surface.point});
          }
          emit_candidates();
        },
        progress, double(face_index) / selection.size(),
        double(face_index + 1) / selection.size(),
        "Finishing CAD face " + std::to_string(face + 1));
    if (!completed) throw std::runtime_error("Cancelled");
    reverse = initial_reverse != (lanes.size() % 2 != 0);
    for (auto& lane : lane_results) {
      trimmed += lane.trimmed;
      detail::accumulate(result.metrics, lane.metrics);
      for (auto& run : lane.runs) runs.push_back(std::move(run));
    }
    ++face_index;
  }
  return runs;
}
}  // namespace
SurfaceContourPreview prepare_surface_contours(
    const CadModel& model, const Operation& operation, const Tool& tool,
    const ProgressCallback& progress) {
  SurfaceContourPreview preview;
  preview.signature = surface_contour_signature(model, operation, tool);
  try {
    OperationPlan sampled;
    std::size_t trimmed = 0;
    auto runs = sample_surface_runs(model, operation, tool, sampled, trimmed, true, progress);
    detail::link_surface_runs(model, operation, tool, {}, {}, 0, runs, sampled,
                              progress, &preview.contours);
    if (preview.contours.empty())
      throw std::runtime_error("No finishing contours on the selected faces");
    detail::build_surface_adjacency(model, operation, preview, progress);
  } catch (const std::exception& error) {
    preview.error = error.what();
  }
  return preview;
}
OperationPlan plan_surface_finishing(const JobGeometry& geometry,
                                     const Operation& operation,
                                     const Tool& tool, const Machine& machine,
                                     const CuttingFeeds& feeds,
                                     double rapid_plane,
                                     const ProgressCallback& progress) {
  const auto started = std::chrono::steady_clock::now();
  OperationPlan result;
  result.operation_name = operation.name;
  result.tool_id = tool.id;
  result.spindle_rpm = feeds.spindle_rpm;
  try {
    const auto& model = geometry.part_cad();
    if (!model)
      throw std::runtime_error("Surface finishing requires a STEP part");
    if (!validate(machine).empty())
      throw std::runtime_error("Invalid finishing machine profile");
    if (!std::isfinite(operation.stepdown) || operation.stepdown <= 0)
      throw std::runtime_error("Finishing reference axial depth must be positive and finite");
    if (operation.surface.manual_order) {
      if (operation.surface.order_roots.empty())
        throw std::runtime_error("Select at least one finishing contour as a propagation root");
      if (operation.surface.order_signature !=
          surface_contour_signature(*model, operation, tool))
        throw std::runtime_error("Finishing contours changed; preview and select propagation roots again");
    }
    std::size_t trimmed = 0;
    auto runs = sample_surface_runs(*model, operation, tool, result, trimmed,
                                    operation.surface.manual_order, progress);
    if (operation.surface.manual_order) {
      SurfaceContourPreview preview;
      preview.signature = operation.surface.order_signature;
      detail::link_surface_runs(*model, operation, tool, machine, feeds, rapid_plane,
                                runs, result, progress, &preview.contours);
      detail::build_surface_adjacency(*model, operation, preview, progress);
      const auto order = propagate_surface_order(preview, operation.surface.order_roots);
      if (!order.error.empty()) throw std::runtime_error(order.error);
      runs.clear();
      for (auto id : order.sequence)
        runs.push_back(std::move(preview.contours[id]));
      detail::link_surface_runs(*model, operation, tool, machine, feeds, rapid_plane,
                                runs, result, progress, nullptr, true);
    } else {
      detail::link_surface_runs(*model, operation, tool, machine, feeds, rapid_plane,
                                runs, result, progress);
    }
    if (result.levels.empty())
      throw std::runtime_error(
          "No finishing contours on the selected faces with this "
          "cutter and direction");
    if (trimmed)
      result.warnings.push_back(
          "Contours split at face trims and singularities");
  } catch (const std::exception& error) {
    result.error = error.what();
  }
  result.metrics.toolpath_milliseconds =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started)
          .count();
  return result;
}
}  // namespace cam
